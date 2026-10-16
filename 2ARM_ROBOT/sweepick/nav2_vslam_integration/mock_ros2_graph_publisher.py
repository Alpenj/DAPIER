#!/usr/bin/env python3
"""C-2 P0-4: labelled MOCK ROS 2 graph for the read-only dry-run.

Publishes the SAME topic names, message types and QoS that the existing Navigation stack uses
(see ros2_readonly_adapter.py header) from a node called `sweepick_c2_mock_graph`, so that the
adapter's output can be checked without a robot. Every message is MOCK data: the node name and
`/docking/target_object_provenance` payload say so, and the dry-run report labels the run
`MOCK_ROS_GRAPH`. This script is never run on the robot network.

Timeline (seconds after start):
  0-4   NAVIGATING: Nav2 goal EXECUTING, /odom vx=0.12, /cmd_vel non-zero, docking NAV2_READY
  4     Nav2 goal SUCCEEDED, /cmd_vel zero once, task VERIFYING, odom zero
  5     docking DOCKED (then NAV2_READY after 2 s as the real controller does)
  6+    detector-like pose stream (stamp = publish time, EMA/latched copy of one sensor frame)
  7+    provenance sidecar for each pose (what the C-2 minimal change would add)
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

_SWEEPICK_ROOT = Path(__file__).resolve().parent.parent
if str(_SWEEPICK_ROOT) not in sys.path:
    sys.path.insert(0, str(_SWEEPICK_ROOT))

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from unique_identifier_msgs.msg import UUID

from nav2_vslam_integration.tf_provenance import rpy_to_quaternion

ACTION_STATUS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
URDF_CHAIN = [  # Waffle Pi URDF + vslam_navigation.launch.py static TF, MOCK copies
    ("base_footprint", "base_link", (0.0, 0.0, 0.010), (0.0, 0.0, 0.0)),
    ("base_link", "camera_link", (0.073, -0.011, 0.084), (0.0, 0.0, 0.0)),
    ("camera_link", "camera_color_optical_frame", (0.0, 0.0, 0.0), (-1.5707963, 0.0, -1.5707963)),
]


class MockGraph(Node):
    def __init__(self, *, republish_same_frame: bool) -> None:
        super().__init__("sweepick_c2_mock_graph")
        self.republish_same_frame = republish_same_frame
        self.t0 = time.monotonic()
        self.status_pub = self.create_publisher(GoalStatusArray, "/navigate_to_pose/_action/status", ACTION_STATUS_QOS)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.dock_pub = self.create_publisher(String, "/docking/status", 10)
        self.task_pub = self.create_publisher(String, "/task/status", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/docking/target_object_pose", 10)
        self.det_pub = self.create_publisher(String, "/docking/object_detection_status", 10)
        self.prov_pub = self.create_publisher(String, "/docking/target_object_provenance", 10)
        self.cam_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", qos_profile_sensor_data)
        self.tf_static = StaticTransformBroadcaster(self)
        self.tf_dyn = TransformBroadcaster(self)
        self.goal_uuid = UUID(uuid=[int(x) for x in bytes.fromhex("c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2c2")])
        self.goal_stamp = self.get_clock().now().to_msg()
        self.x = 0.0
        self.last_sensor_stamp = None
        self.sensor_count = 0
        self.pose_count = 0
        self.docked_at = None
        self.publish_static_tf()
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz like the detector

    def publish_static_tf(self) -> None:
        msgs = []
        for parent, child, xyz, rpy in URDF_CHAIN:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            x, y, z, w = rpy_to_quaternion(*rpy)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = x, y, z, w
            msgs.append(t)
        self.tf_static.sendTransform(msgs)

    def elapsed(self) -> float:
        return time.monotonic() - self.t0

    def tick(self) -> None:
        e = self.elapsed()
        now = self.get_clock().now().to_msg()
        moving = e < 4.0
        vx = 0.12 if moving else 0.0
        self.x += vx * 0.05

        # /tf odom->base_footprint and map->odom (MOCK)
        for parent, child, x in (("odom", "base_footprint", self.x), ("map", "odom", 0.0)):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now, parent, child
            t.transform.translation.x = x
            t.transform.rotation.w = 1.0
            self.tf_dyn.sendTransform(t)

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now, "odom", "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = vx
        self.odom_pub.publish(odom)

        if moving or (4.0 <= e < 4.1):
            cmd = TwistStamped()
            cmd.header.stamp, cmd.header.frame_id = now, "base_footprint"
            cmd.twist.linear.x = vx
            self.cmd_pub.publish(cmd)

        st = GoalStatusArray()
        gs = GoalStatus()
        gs.goal_info.goal_id = self.goal_uuid
        gs.goal_info.stamp = self.goal_stamp
        gs.status = GoalStatus.STATUS_EXECUTING if moving else GoalStatus.STATUS_SUCCEEDED
        st.status_list.append(gs)
        self.status_pub.publish(st)

        if e < 5.0:
            dock = "NAV2_READY"
        elif e < 7.0:
            dock = "DOCKED"
            self.docked_at = self.docked_at or e
        else:
            dock = "NAV2_READY"
        self.dock_pub.publish(String(data=f"STATE={dock}, ey=+0.0cm, yaw_err=+0.0deg, dist=0.04m"))
        task = "NAVIGATING" if moving else "VERIFYING" if e < 4.6 else "DOCKING" if e < 5.0 else "SUCCESS"
        self.task_pub.publish(String(data=f"STATE={task}, task=DOCK, target=desk, bumper=False"))

        cam = CameraInfo()
        cam.header.stamp, cam.header.frame_id = now, "camera_color_optical_frame"
        self.cam_pub.publish(cam)
        self.sensor_count += 1
        if e >= 6.0 and (self.last_sensor_stamp is None or not self.republish_same_frame):
            self.last_sensor_stamp = now  # the one sensor frame the EMA/latched pose keeps re-publishing

        if e >= 6.0 and self.last_sensor_stamp is not None:
            pose = PoseStamped()
            pose.header.stamp = now  # exactly what object_detector_3d.py does: publish time, not sensor time
            pose.header.frame_id = "camera_color_optical_frame"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 0.01, 0.03, 0.28
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)
            self.pose_count += 1
            self.det_pub.publish(String(data="LOCKED: x=0.010, y=0.030, z=0.280, yaw_deg=0.0, inliers=90, fps=20.0"))
            if e >= 7.0:
                sensor_s = self.last_sensor_stamp.sec + self.last_sensor_stamp.nanosec * 1e-9
                self.prov_pub.publish(String(data=json.dumps({
                    "schema": "dapier.c2.detector-provenance.v1-PROPOSAL", "origin": "MOCK_ROS_GRAPH",
                    "pose_stamp_s": now.sec + now.nanosec * 1e-9, "sensor_stamp_s": sensor_s,
                    # re-publish mode mirrors the real detector (EMA after the first sample); fresh-frame mode
                    # stands for the proposed raw per-frame provenance (is_ema_filtered=False)
                    "anchor_source": "hsv", "is_ema_filtered": self.republish_same_frame and self.pose_count > 1,
                    "is_persistence_latched": False,
                    "inlier_count": 90, "rms_m": 0.006, "detector_mode": "hsv", "qwen_bbox_stamp_s": None,
                })))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--fresh-frames", action="store_true", help="give every pose a new sensor frame (default: re-publish one frame)")
    args = ap.parse_args()
    rclpy.init()
    node = MockGraph(republish_same_frame=not args.fresh_frames)
    end = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
