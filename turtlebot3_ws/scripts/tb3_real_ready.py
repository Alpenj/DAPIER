#!/usr/bin/env python3
"""Fast, typed readiness gate for routine physical TurtleBot3 starts."""

from __future__ import annotations

import argparse
import math
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from turtlebot3_msgs.msg import SensorState

from tb3_opencr_probe import OpenCrAudit
from tb3_scan_probe import ScanAudit
from tb3_tf_probe import TfAudit


class OdomAudit:
    def __init__(self) -> None:
        self.count = 0
        self.seen: set[int] = set()
        self.last_stamp_ns: int | None = None
        self.duplicates = 0
        self.regressions = 0
        self.invalid = 0

    def receive(self, message: Odometry) -> None:
        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        self.count += 1
        if stamp_ns in self.seen:
            self.duplicates += 1
        self.seen.add(stamp_ns)
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.regressions += 1
        self.last_stamp_ns = stamp_ns
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        quaternion_norm = math.sqrt(
            orientation.x**2
            + orientation.y**2
            + orientation.z**2
            + orientation.w**2
        )
        if (
            stamp_ns <= 0
            or not all(math.isfinite(value) for value in values)
            or not 0.99 <= quaternion_norm <= 1.01
        ):
            self.invalid += 1

    def errors(self, min_samples: int) -> list[str]:
        result = []
        if self.count < min_samples:
            result.append(f"only {self.count} odometry samples arrived; need {min_samples}")
        if self.duplicates:
            result.append(f"odometry has {self.duplicates} duplicate timestamps")
        if self.regressions:
            result.append(f"odometry has {self.regressions} timestamp regressions")
        if self.invalid:
            result.append(f"odometry has {self.invalid} invalid samples")
        return result


class ReadyGate(Node):
    def __init__(self, *, require_scan: bool) -> None:
        super().__init__("tb3_real_ready")
        self.require_scan = require_scan
        self.odom = OdomAudit()
        self.opencr = OpenCrAudit()
        self.scan = ScanAudit()
        self.tf = TfAudit()
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(Odometry, "/odom", self.odom.receive, 20)
        self.create_subscription(
            BatteryState,
            "/battery_state",
            lambda message: self.opencr.receive_battery(message.voltage),
            10,
        )
        self.create_subscription(
            SensorState,
            "/sensor_state",
            lambda message: self.opencr.receive_torque(message.torque),
            10,
        )
        if require_scan:
            self.create_subscription(
                LaserScan, "/scan", self._receive_scan, qos_profile_sensor_data
            )

    def _receive_scan(self, message: LaserScan) -> None:
        self.scan.receive_values(
            stamp_ns=(
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            ),
            frame_id=message.header.frame_id,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
            ranges=message.ranges,
        )

    def update_tf(self) -> None:
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )
        except TransformException:
            return
        stamp = transform.header.stamp
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.tf.receive_values(
            stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
            translation=(translation.x, translation.y, translation.z),
            rotation=(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def graph_errors(self) -> list[str]:
        cmd_subscriptions = self.get_subscriptions_info_by_topic("/cmd_vel")
        odom_publishers = self.get_publishers_info_by_topic("/odom")
        result = []
        if len(cmd_subscriptions) != 1:
            result.append(
                f"/cmd_vel needs exactly one robot subscriber; found {len(cmd_subscriptions)}"
            )
        elif cmd_subscriptions[0].topic_type != "geometry_msgs/msg/Twist":
            result.append(
                f"/cmd_vel subscriber type is {cmd_subscriptions[0].topic_type}, not Twist"
            )
        if len(odom_publishers) != 1:
            result.append(
                f"/odom needs exactly one publisher; found {len(odom_publishers)}"
            )
        elif odom_publishers[0].topic_type != "nav_msgs/msg/Odometry":
            result.append(
                f"/odom publisher type is {odom_publishers[0].topic_type}, not Odometry"
            )
        if self.require_scan:
            scan_publishers = self.get_publishers_info_by_topic("/scan")
            if len(scan_publishers) != 1:
                result.append(
                    f"/scan needs exactly one publisher; found {len(scan_publishers)}"
                )
            elif scan_publishers[0].topic_type != "sensor_msgs/msg/LaserScan":
                result.append(
                    f"/scan publisher type is {scan_publishers[0].topic_type}, not LaserScan"
                )
        return result

    def data_errors(self) -> list[str]:
        result = self.odom.errors(min_samples=5)
        result.extend(self.tf.errors(min_updates=2))
        result.extend(self.opencr.errors(min_samples=2, min_voltage=11.1))
        if self.require_scan:
            result.extend(self.scan.errors(min_samples=2))
        return result

    def observed_danger(self) -> list[str]:
        """Return safety faults that should fail without waiting for timeout."""
        result = []
        if self.odom.duplicates:
            result.append("duplicate odometry timestamp observed")
        if self.odom.regressions:
            result.append("odometry timestamp regression observed")
        if self.odom.invalid:
            result.append("invalid odometry observed")
        if self.opencr.voltages and (
            not math.isfinite(self.opencr.voltages[-1])
            or self.opencr.voltages[-1] < 11.1
        ):
            result.append("unsafe OpenCR battery voltage observed")
        if self.opencr.torque_states and not self.opencr.torque_states[-1]:
            result.append("Dynamixel torque-off observed")
        if self.require_scan and (
            self.scan.duplicates or self.scan.regressions or self.scan.invalid_samples
        ):
            result.append("invalid LiDAR scan observed")
        return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("drive", "slam", "nav"), default="drive")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("timeout must be positive")

    rclpy.init(args=["--ros-args", "--log-level", "error"])
    gate = ReadyGate(require_scan=args.mode in {"slam", "nav"})
    deadline = time.monotonic() + args.timeout
    passed = False
    danger: list[str] = []
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(gate, timeout_sec=0.1)
        gate.update_tf()
        danger = gate.observed_danger()
        if danger:
            break
        if not gate.graph_errors() and not gate.data_errors():
            passed = True
            break

    graph_errors = gate.graph_errors()
    data_errors = gate.data_errors()
    min_voltage = min(gate.opencr.voltages) if gate.opencr.voltages else math.nan
    print(
        f"mode={args.mode} odom={gate.odom.count} tf={gate.tf.updates} "
        f"battery={min_voltage:.3f}V torque="
        f"{'true' if gate.opencr.torque_states and all(gate.opencr.torque_states) else 'false'} "
        f"scans={gate.scan.count if gate.require_scan else 'n/a'}"
    )
    errors = danger or graph_errors + data_errors
    if not passed:
        for error in errors:
            print(f"ERROR: {error}")
    del gate.tf_listener
    gate.destroy_node()
    rclpy.shutdown()
    if not passed:
        print("ERROR: robot is not ready; run tb3-check for detailed diagnosis")
        return 1
    print("OK: fast robot readiness gate passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
