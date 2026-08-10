#!/usr/bin/env python3
"""Gentle waypoint-driven explorer for clean SLAM mapping.
Compared to earlier versions: lower speeds, larger safety margin, no
reverse-while-spinning escape (a major source of wheel-slip/odometry drift
that caused map ghosting last time), and rate-limited angular commands for
smooth turns instead of sharp direction reversals.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

WAYPOINTS = [
    (3.0, 0.0), (2.3, 1.8), (2.3, -1.8),
    (1.3, 2.3), (0.0, 2.3), (-1.3, 2.3),
    (1.3, -2.3), (0.0, -2.3), (-1.3, -2.3),
    (-1.5, 1.5), (-1.5, 0.0), (-1.5, -1.5),
    (-0.55, -0.55), (-0.55, 0.55), (0.55, -0.55), (0.55, 0.55),
    (0.0, 0.0),
]


def yaw_from_quat(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class GentleExplorer(Node):
    def __init__(self, run_seconds):
        super().__init__('gentle_explorer')
        self.run_seconds = run_seconds
        self.start_time = time.time()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False

        self.min_front = float('inf')
        self.min_left = float('inf')
        self.min_right = float('inf')
        self.have_scan = False

        self.wp_idx = 0
        self.wp_deadline = time.time() + 35.0
        self.visited = set()

        self.pos_history = []
        self.stuck_pause_until = 0.0

        # rate-limited angular command to avoid jerky direction reversals
        self.cur_ang = 0.0
        self.cur_lin = 0.0

        self.map_known = 0
        self.map_total = 0
        self.last_report = 0.0

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', QoSProfile(depth=10))

        scan_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, '/scan', self.on_scan, scan_qos)

        odom_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, '/odom', self.on_odom, odom_qos)

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)

        self.timer = self.create_timer(0.1, self.on_tick)

    def on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.have_odom = True

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        def sector_min(center_deg, width_deg=25):
            c = int((center_deg / 360.0) * n) % n
            w = int((width_deg / 360.0) * n)
            vals = [msg.ranges[i % n] for i in range(c - w, c + w)]
            vals = [r for r in vals if not math.isinf(r) and not math.isnan(r) and r > 0.05]
            return min(vals) if vals else float('inf')

        self.min_front = min(sector_min(0), sector_min(360))
        self.min_left = sector_min(90)
        self.min_right = sector_min(270)
        self.have_scan = True

    def on_map(self, msg: OccupancyGrid):
        self.map_total = len(msg.data)
        self.map_known = sum(1 for v in msg.data if v != -1)

    def stuck_check(self, now):
        self.pos_history.append((now, self.x, self.y))
        self.pos_history = [p for p in self.pos_history if now - p[0] < 9.0]
        t0, x0, y0 = self.pos_history[0]
        # Require the window to actually span most of the 9s -- otherwise a
        # freshly-cleared history (e.g. right after a recovery) looks
        # "stuck" just because it hasn't had time to accumulate distance yet.
        if now - t0 < 7.0:
            return False
        dist = math.hypot(self.x - x0, self.y - y0)
        return dist < 0.15

    def next_waypoint(self, now):
        self.visited.add(self.wp_idx)
        if len(self.visited) >= len(WAYPOINTS):
            self.visited = set()
        remaining = [i for i in range(len(WAYPOINTS)) if i not in self.visited]

        def d(i):
            gx, gy = WAYPOINTS[i]
            return math.hypot(gx - self.x, gy - self.y)

        self.wp_idx = min(remaining, key=d)
        self.wp_deadline = now + 35.0

    def publish(self, target_lin, target_ang):
        # rate-limit to avoid abrupt jerks that cause wheel slip / odom drift
        max_dlin = 0.03
        max_dang = 0.06
        dl = max(-max_dlin, min(max_dlin, target_lin - self.cur_lin))
        da = max(-max_dang, min(max_dang, target_ang - self.cur_ang))
        self.cur_lin += dl
        self.cur_ang += da

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = self.cur_lin
        twist.twist.angular.z = self.cur_ang
        self.pub.publish(twist)

    def on_tick(self):
        now = time.time()
        elapsed = now - self.start_time
        if elapsed > self.run_seconds:
            self.stop_and_shutdown('시간 종료')
            return
        if not self.have_odom or not self.have_scan:
            return

        # Stuck recovery: gentle stop-and-slow-turn-in-place, NOT reverse+spin.
        if now < self.stuck_pause_until:
            self.publish(0.0, 0.35)
            self._report(elapsed)
            return

        if self.stuck_check(now):
            self.get_logger().warn('Stuck -> gentle in-place turn, skip waypoint')
            self.stuck_pause_until = now + 3.0
            self.pos_history = []  # don't let the zero-motion recovery window
            self.next_waypoint(now)  # immediately re-trigger stuck detection
            return

        gx, gy = WAYPOINTS[self.wp_idx]
        dist_to_goal = math.hypot(gx - self.x, gy - self.y)
        if dist_to_goal < 0.35 or now > self.wp_deadline:
            self.next_waypoint(now)
            gx, gy = WAYPOINTS[self.wp_idx]

        front, left, right = self.min_front, self.min_left, self.min_right
        SAFE = 0.6  # larger margin than before -> fewer close encounters

        if front < SAFE:
            turn_dir = 1.0 if left > right else -1.0
            # slow down instead of hard-stop+spin; keep some forward motion
            slow_lin = 0.05 if front > 0.35 else 0.0
            self.publish(slow_lin, 0.45 * turn_dir)
        else:
            desired_yaw = math.atan2(gy - self.y, gx - self.x)
            err = ang_diff(desired_yaw, self.yaw)
            # proportional turn, capped lower than before for smoothness
            ang = max(-0.5, min(0.5, 1.0 * err))
            # slow down more when turn is sharp, so linear+angular stay balanced
            lin = 0.15 if abs(err) < 0.5 else 0.06
            if left < 0.55:
                ang -= 0.2
            if right < 0.55:
                ang += 0.2
            self.publish(lin, ang)

        self._report(elapsed)

    def _report(self, elapsed):
        if elapsed - self.last_report > 10.0:
            self.last_report = elapsed
            pct = (self.map_known / self.map_total * 100.0) if self.map_total else 0.0
            gx, gy = WAYPOINTS[self.wp_idx]
            print(f'PROGRESS {elapsed:.0f} {self.map_known} {self.map_total} {pct:.2f} '
                  f'pos=({self.x:.2f},{self.y:.2f}) wp#{self.wp_idx}=({gx},{gy}) '
                  f'visited={len(self.visited)}/{len(WAYPOINTS)}', flush=True)

    def stop_and_shutdown(self, reason):
        for _ in range(10):
            self.publish(0.0, 0.0)
            time.sleep(0.05)
        pct = (self.map_known / self.map_total * 100.0) if self.map_total else 0.0
        print(f'DONE {reason} {self.map_known} {self.map_total} {pct:.2f}', flush=True)
        raise SystemExit(0)


def main():
    run_seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 320.0
    rclpy.init()
    node = GentleExplorer(run_seconds)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
