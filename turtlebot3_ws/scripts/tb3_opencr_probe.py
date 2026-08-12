#!/usr/bin/env python3
"""Verify live OpenCR battery and Dynamixel torque through typed topics."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from sensor_msgs.msg import BatteryState
from turtlebot3_msgs.msg import SensorState


class OpenCrAudit:
    def __init__(self) -> None:
        self.voltages: list[float] = []
        self.torque_states: list[bool] = []

    def receive_battery(self, voltage: float) -> None:
        self.voltages.append(float(voltage))

    def receive_torque(self, torque: bool) -> None:
        self.torque_states.append(bool(torque))

    def errors(self, *, min_samples: int, min_voltage: float) -> list[str]:
        result = []
        if len(self.voltages) < min_samples:
            result.append(
                f"only {len(self.voltages)} battery samples arrived; need {min_samples}"
            )
        elif not all(math.isfinite(value) for value in self.voltages):
            result.append("battery voltage contains a non-finite value")
        elif min(self.voltages) < min_voltage:
            result.append(
                f"minimum battery voltage {min(self.voltages):.3f}V is below "
                f"{min_voltage:.3f}V"
            )
        if len(self.torque_states) < min_samples:
            result.append(
                f"only {len(self.torque_states)} torque samples arrived; need {min_samples}"
            )
        elif not all(self.torque_states):
            result.append("Dynamixel torque is not continuously enabled")
        return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument("--min-voltage", type=float, default=11.1)
    args = parser.parse_args()
    if args.seconds <= 0 or args.min_samples <= 0 or args.min_voltage <= 0:
        parser.error("all limits must be positive")

    rclpy.init()
    node = rclpy.create_node("tb3_opencr_probe")
    audit = OpenCrAudit()
    battery_subscription = node.create_subscription(
        BatteryState,
        "/battery_state",
        lambda message: audit.receive_battery(message.voltage),
        10,
    )
    sensor_subscription = node.create_subscription(
        SensorState,
        "/sensor_state",
        lambda message: audit.receive_torque(message.torque),
        10,
    )
    deadline = time.monotonic() + args.seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not audit.errors(
            min_samples=args.min_samples, min_voltage=args.min_voltage
        ):
            break

    errors = audit.errors(min_samples=args.min_samples, min_voltage=args.min_voltage)
    min_voltage = min(audit.voltages) if audit.voltages else math.nan
    torque_text = "true" if audit.torque_states and all(audit.torque_states) else "false"
    print(
        f"battery_samples={len(audit.voltages)} "
        f"torque_samples={len(audit.torque_states)} "
        f"min_voltage={min_voltage:.3f} torque={torque_text}"
    )
    for error in errors:
        print(f"ERROR: {error}")
    node.destroy_subscription(battery_subscription)
    node.destroy_subscription(sensor_subscription)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
