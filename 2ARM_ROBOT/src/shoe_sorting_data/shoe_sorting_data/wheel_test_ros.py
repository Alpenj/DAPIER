"""Guarded off-ground TurtleBot wheel characterization CLI."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Sequence

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState

from shoe_sorting_data.wheel_characterization import analyze_characterization, summarize_stage


LINEAR_COMMANDS = (0.005, 0.01, 0.02, 0.04, 0.08, 0.13, 0.20, 0.26, -0.02, -0.08, -0.26)
ANGULAR_COMMANDS = (
    0.005,
    0.01,
    0.02,
    0.05,
    0.10,
    0.20,
    0.40,
    0.80,
    1.20,
    1.82,
    -0.20,
    -0.80,
    -1.82,
)


class WheelTestNode(Node):
    def __init__(self) -> None:
        super().__init__("shoe_off_ground_wheel_test")
        self.publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 30)
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 30)
        self.create_subscription(BatteryState, "/battery_state", self._battery_callback, 10)
        self.latest_odom: Odometry | None = None
        self.latest_joint: JointState | None = None
        self.latest_battery: BatteryState | None = None

    def _odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def _joint_callback(self, message: JointState) -> None:
        self.latest_joint = message

    def _battery_callback(self, message: BatteryState) -> None:
        self.latest_battery = message

    def ready(self) -> bool:
        return self.latest_odom is not None and self.latest_joint is not None and self.latest_battery is not None

    def publish_command(self, *, linear: float = 0.0, angular: float = 0.0) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self.publisher.publish(message)

    def sample(self) -> dict[str, object] | None:
        if not self.ready():
            return None
        names = list(self.latest_joint.name)
        velocities = list(self.latest_joint.velocity)
        by_name = dict(zip(names, velocities))
        battery_current = float(self.latest_battery.current)
        return {
            "reception_monotonic_ns": time.monotonic_ns(),
            "odom_linear_x_mps": float(self.latest_odom.twist.twist.linear.x),
            "odom_angular_z_radps": float(self.latest_odom.twist.twist.angular.z),
            # TurtleBot3 Jazzy converts DYNAMIXEL RPM to wheel linear m/s in JointState.
            "left_wheel_mps": float(by_name.get("wheel_left_joint", math.nan)),
            "right_wheel_mps": float(by_name.get("wheel_right_joint", math.nan)),
            "battery_voltage": float(self.latest_battery.voltage),
            "battery_current": battery_current,
        }


def _spin_for(executor: SingleThreadedExecutor, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))


def _stop(node: WheelTestNode, executor: SingleThreadedExecutor, seconds: float = 0.6) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        node.publish_command()
        executor.spin_once(timeout_sec=0.05)


def _run_stage(
    node: WheelTestNode,
    executor: SingleThreadedExecutor,
    *,
    kind: str,
    command: float,
    duration: float,
) -> dict[str, object]:
    samples: list[dict[str, object]] = []
    start = time.monotonic()
    while time.monotonic() - start < duration:
        if kind == "linear":
            node.publish_command(linear=command)
        else:
            node.publish_command(angular=command)
        executor.spin_once(timeout_sec=0.05)
        if time.monotonic() - start >= 0.35:
            sample = node.sample()
            if sample is not None:
                samples.append(sample)
    _stop(node, executor)
    return {"kind": kind, "command": command, "samples": samples, "summary": summarize_stage(samples)}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run a guarded off-ground TurtleBot wheel test.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--wheels-off-ground-confirmed", action="store_true")
    parser.add_argument("--stage-seconds", type=float, default=1.2)
    args, ros_args = parser.parse_known_args(argv)
    if not args.wheels_off_ground_confirmed:
        parser.error("--wheels-off-ground-confirmed is required")
    if args.stage_seconds < 0.8:
        parser.error("--stage-seconds must be at least 0.8")
    if args.output.exists():
        print(json.dumps({"error": f"output already exists: {args.output}"}, indent=2))
        return 2

    rclpy.init(args=ros_args)
    node = WheelTestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stages: list[dict[str, object]] = []
    try:
        wait_deadline = time.monotonic() + 12.0
        while not node.ready() and time.monotonic() < wait_deadline:
            executor.spin_once(timeout_sec=0.1)
        if not node.ready():
            raise ValueError("required /odom, /joint_states, and /battery_state messages were not received")
        _stop(node, executor, seconds=1.0)
        for command in LINEAR_COMMANDS:
            stages.append(_run_stage(node, executor, kind="linear", command=command, duration=args.stage_seconds))
        for command in ANGULAR_COMMANDS:
            stages.append(_run_stage(node, executor, kind="angular", command=command, duration=args.stage_seconds))
        result = {
            "schema_version": "dapier.turtlebot-wheel-characterization.v0.2",
            "created_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "test_condition": "wheels_off_ground_user_confirmed",
            "motion_commands_sent": True,
            "maximum_commanded_linear_mps": 0.26,
            "maximum_commanded_angular_radps": 1.82,
            "stages": stages,
            "derived": analyze_characterization(stages),
            "measurement_notes": {
                "joint_state_velocity_unit": "meter_per_second",
                "joint_state_unit_basis": "ROBOTIS Jazzy joint_state.cpp applies RPM_TO_MS",
                "wheel_separation_meters": 0.287,
                "off_ground_angular_response_basis": "right_minus_left_wheel_mps_divided_by_separation",
                "odom_angular_z_not_used_for_deadband": "IMU reports stationary body while wheels are lifted",
                "all_zero_battery_current_means_uninformative": True,
            },
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"output": str(args.output), "derived": result["derived"]}, indent=2))
        return 0
    except (OSError, ValueError) as error:
        print(json.dumps({"error": str(error)}, indent=2))
        return 1
    finally:
        _stop(node, executor, seconds=1.0)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
