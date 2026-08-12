#!/usr/bin/env python3
"""Safely verify both physical TurtleBot3 Burger wheels while lifted."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import statistics
import time

from geometry_msgs.msg import Twist
import rclpy
from sensor_msgs.msg import JointState


LEFT_JOINT = "wheel_left_joint"
RIGHT_JOINT = "wheel_right_joint"
# turtlebot3_node Humble writes RPM_TO_MS into JointState.velocity, so these
# two wheel velocity values are linear m/s despite JointState's usual rad/s
# convention. See turtlebot3_node/src/sensors/joint_state.cpp.
MIN_ABS_VELOCITY = 0.015
MIN_POSITION_DELTA = 0.10


@dataclass(frozen=True)
class Phase:
    name: str
    linear: float
    angular: float
    relation: str


PHASES = (
    Phase("forward", 0.05, 0.0, "both_positive_balanced"),
    Phase("reverse", -0.05, 0.0, "both_negative_balanced"),
    Phase("rotate_left", 0.0, 0.50, "left_negative_right_positive"),
    Phase("rotate_right", 0.0, -0.50, "left_positive_right_negative"),
    Phase("curve_left", 0.05, 0.30, "both_positive_right_faster"),
    Phase("curve_right", 0.05, -0.30, "both_positive_left_faster"),
)


def validate_phase(
    relation: str, samples: list[tuple[float, float]]
) -> tuple[float, float, list[str]]:
    if len(samples) < 5:
        return 0.0, 0.0, [f"only {len(samples)} joint velocity samples; need 5"]
    left = statistics.median(value[0] for value in samples)
    right = statistics.median(value[1] for value in samples)
    errors: list[str] = []

    def positive(value: float) -> bool:
        return value >= MIN_ABS_VELOCITY

    def negative(value: float) -> bool:
        return value <= -MIN_ABS_VELOCITY

    if relation.startswith("both_positive") and not (positive(left) and positive(right)):
        errors.append("both wheels must rotate forward")
    if relation.startswith("both_negative") and not (negative(left) and negative(right)):
        errors.append("both wheels must rotate backward")
    if relation == "left_negative_right_positive" and not (
        negative(left) and positive(right)
    ):
        errors.append("left/right signs do not produce a left in-place rotation")
    if relation == "left_positive_right_negative" and not (
        positive(left) and negative(right)
    ):
        errors.append("left/right signs do not produce a right in-place rotation")
    if relation.endswith("balanced"):
        larger = max(abs(left), abs(right))
        smaller = min(abs(left), abs(right))
        if larger == 0 or smaller / larger < 0.70:
            errors.append("straight-motion wheel speed ratio is below 70%")
    if relation == "both_positive_right_faster" and right - left < 0.025:
        errors.append("right wheel is not faster for a left curve")
    if relation == "both_positive_left_faster" and left - right < 0.025:
        errors.append("left wheel is not faster for a right curve")
    return left, right, errors


def validate_position_delta(
    relation: str, start: tuple[float, float], end: tuple[float, float]
) -> tuple[float, float, list[str]]:
    left = end[0] - start[0]
    right = end[1] - start[1]
    errors: list[str] = []

    def positive(value: float) -> bool:
        return value >= MIN_POSITION_DELTA

    def negative(value: float) -> bool:
        return value <= -MIN_POSITION_DELTA

    if relation.startswith("both_positive") and not (positive(left) and positive(right)):
        errors.append("both wheel encoders must increase")
    if relation.startswith("both_negative") and not (negative(left) and negative(right)):
        errors.append("both wheel encoders must decrease")
    if relation == "left_negative_right_positive" and not (
        negative(left) and positive(right)
    ):
        errors.append("encoder directions do not produce a left rotation")
    if relation == "left_positive_right_negative" and not (
        positive(left) and negative(right)
    ):
        errors.append("encoder directions do not produce a right rotation")
    return left, right, errors


def zero_twist() -> Twist:
    return Twist()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--wheels-lifted",
        action="store_true",
        help="required acknowledgement that both drive wheels are off the floor",
    )
    parser.add_argument("--phase-seconds", type=float, default=1.5)
    args = parser.parse_args()
    if not args.wheels_lifted:
        parser.error("refusing to move: pass --wheels-lifted only after lifting both wheels")
    if args.phase_seconds < 1.0:
        parser.error("phase-seconds must be at least 1.0")

    rclpy.init()
    node = rclpy.create_node("tb3_wheel_test")
    if node.count_publishers("/cmd_vel") != 0:
        print("ERROR: another /cmd_vel publisher is active; stop it before wheel test")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    received: list[tuple[float, float, float, float, float]] = []

    def receive(message: JointState) -> None:
        try:
            left_index = message.name.index(LEFT_JOINT)
            right_index = message.name.index(RIGHT_JOINT)
            left_velocity = message.velocity[left_index]
            right_velocity = message.velocity[right_index]
            left_position = message.position[left_index]
            right_position = message.position[right_index]
        except (ValueError, IndexError):
            return
        received.append(
            (
                time.monotonic(),
                float(left_velocity),
                float(right_velocity),
                float(left_position),
                float(right_position),
            )
        )

    subscription = node.create_subscription(JointState, "/joint_states", receive, 10)
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)

    wait_deadline = time.monotonic() + 10.0
    while time.monotonic() < wait_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 1 and received:
            break
    if publisher.get_subscription_count() != 1 or not received:
        print(
            "ERROR: wheel test needs exactly one /cmd_vel subscriber and live /joint_states"
        )
        node.destroy_publisher(publisher)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    failures = 0
    try:
        for phase in PHASES:
            command = Twist()
            command.linear.x = phase.linear
            command.angular.z = phase.angular
            phase_start = time.monotonic()
            sample_start = phase_start + 0.45
            deadline = phase_start + args.phase_seconds
            while time.monotonic() < deadline:
                publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.08)
            phase_messages = [
                message
                for message in received
                if phase_start <= message[0] <= deadline + 0.15
            ]
            samples = [
                (left_velocity, right_velocity)
                for received_at, left_velocity, right_velocity, _, _ in phase_messages
                if sample_start <= received_at <= deadline + 0.15
            ]
            left, right, errors = validate_phase(phase.relation, samples)
            if len(phase_messages) >= 2:
                start_position = (phase_messages[0][3], phase_messages[0][4])
                end_position = (phase_messages[-1][3], phase_messages[-1][4])
                left_delta, right_delta, position_errors = validate_position_delta(
                    phase.relation, start_position, end_position
                )
                errors.extend(position_errors)
            else:
                left_delta = 0.0
                right_delta = 0.0
                errors.append("not enough encoder position samples")
            result = "PASS" if not errors else "FAIL"
            print(
                f"{result} {phase.name}: velocity L={left:.3f} R={right:.3f}m/s "
                f"encoder_delta L={left_delta:.3f} R={right_delta:.3f}rad "
                f"samples={len(samples)}"
            )
            for error in errors:
                print(f"  ERROR: {error}")
            failures += bool(errors)
            for _ in range(8):
                publisher.publish(zero_twist())
                rclpy.spin_once(node, timeout_sec=0.06)
            if errors:
                break
    finally:
        for _ in range(10):
            publisher.publish(zero_twist())
            rclpy.spin_once(node, timeout_sec=0.05)

    print("OK: both wheels passed all six lifted-wheel phases." if not failures else "ERROR: wheel test stopped and sent zero velocity.")
    node.destroy_publisher(publisher)
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
