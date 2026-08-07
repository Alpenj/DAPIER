#!/usr/bin/env python3
"""Run SO-101 follower calibration with an explicit torque-off interlock."""

from __future__ import annotations

import argparse

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safely recalibrate an SO-101 follower; torque is disabled before calibration starts."
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Explicit follower serial path, preferably /dev/serial/by-id/...",
    )
    parser.add_argument("--id", default="so101_follower_main")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    device = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.id,
            cameras={},
            disable_torque_on_disconnect=True,
        )
    )

    try:
        print("Connecting to the motor bus...")
        device.bus.connect()

        before = device.bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
        print("Torque state before safety step:", before)
        print("Disabling torque on all six motors...")
        device.bus.disable_torque(num_retry=3)
        after = device.bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
        print("Torque state after safety step:", after)

        enabled = [name for name in MOTOR_NAMES if int(after[name]) != 0]
        if enabled:
            raise RuntimeError(
                "Torque-off interlock failed on: " + ", ".join(enabled) + ". "
                "Keep servo power off and do not move the arm."
            )

        print("All six motors report torque OFF.")
        print(
            "Starting a fresh calibration; the existing JSON will be replaced after completion."
        )
        print(
            "Keep the arm supported and do not press Enter until the displayed pose is safe."
        )

        # Suppress the normal 'use existing calibration?' prompt. The user asked
        # for a fresh calibration, and the explicit torque interlock above has
        # already run before entering the calibration routine.
        device.calibration = {}
        device.calibrate()
        print("Calibration routine completed.")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted. Torque will be disabled before disconnecting.")
        return 130
    finally:
        if device.bus.is_connected:
            try:
                device.bus.disable_torque(num_retry=5)
            finally:
                # Torque was explicitly disabled above; avoid another write
                # during close while still guaranteeing a closed serial port.
                device.bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    raise SystemExit(main())
