#!/usr/bin/env python3
"""Show live SO-101 follower calibration values without commanding the arm."""

from __future__ import annotations

import argparse
import sys
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


MOTOR_IDS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}
ENCODER_MAX = 4095


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only live table of raw and calibrated follower positions."
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Explicit follower serial path, preferably /dev/serial/by-id/...",
    )
    parser.add_argument("--id", default="so101_follower_main")
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument(
        "--samples",
        type=int,
        default=0,
        help="Number of updates; 0 means run until Ctrl+C.",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Do not redraw the terminal (useful for logs/tests).",
    )
    return parser.parse_args()


def calibrated_value(
    name: str, raw: int, range_min: int, range_max: int
) -> tuple[float, str]:
    if name == "gripper":
        bounded = min(range_max, max(range_min, raw))
        return (bounded - range_min) * 100.0 / (range_max - range_min), "%"

    midpoint = (range_min + range_max) / 2.0
    return (raw - midpoint) * 360.0 / ENCODER_MAX, "deg"


def range_flag(raw: int, range_min: int, range_max: int) -> str:
    if raw < range_min:
        return f"LOW {-1 * (raw - range_min):d}"
    if raw > range_max:
        return f"HIGH {raw - range_max:d}"
    return "OK"


def main() -> int:
    args = parse_args()
    if args.hz <= 0:
        raise SystemExit("--hz must be positive")
    if args.samples < 0:
        raise SystemExit("--samples must be non-negative")

    device = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id, cameras={}))
    actual_ids = {name: motor.id for name, motor in device.bus.motors.items()}
    if actual_ids != MOTOR_IDS:
        raise SystemExit(f"Unexpected motor mapping: {actual_ids}")
    if set(device.calibration) != set(MOTOR_IDS):
        raise SystemExit(
            f"Calibration is missing or incomplete: {device.calibration_fpath}"
        )

    observed_min: dict[str, int] = {}
    observed_max: dict[str, int] = {}
    update = 0

    try:
        # Connect only to the bus. Do not call robot.connect(), configure(), or
        # any method that writes torque, limits, offsets, or goal positions.
        device.bus.connect()
        torque = device.bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
        torque_on = [name for name, value in torque.items() if value != 0]
        if torque_on:
            raise SystemExit(
                "Refusing manual position monitoring while torque is enabled on: "
                + ", ".join(torque_on)
            )

        while args.samples == 0 or update < args.samples:
            raw_values = device.bus.sync_read(
                "Present_Position", normalize=False, num_retry=2
            )
            update += 1
            for name, raw in raw_values.items():
                observed_min[name] = min(raw, observed_min.get(name, raw))
                observed_max[name] = max(raw, observed_max.get(name, raw))

            redraw = sys.stdout.isatty() and not args.no_clear
            if redraw:
                print("\033[2J\033[H", end="")

            print("SO-101 FOLLOWER — READ ONLY (torque OFF, Ctrl+C to stop)")
            print(f"port: {args.port}")
            print(f"calibration: {device.calibration_fpath}")
            print()
            print(
                f"{'JOINT':<15} {'ID':>2} {'RAW':>5} {'VALUE':>10} "
                f"{'SAVED RANGE':>13} {'SEEN RANGE':>13} {'STATE':>8}"
            )
            print("-" * 75)
            for name in MOTOR_IDS:
                raw = int(raw_values[name])
                cal = device.calibration[name]
                value, unit = calibrated_value(name, raw, cal.range_min, cal.range_max)
                saved = f"{cal.range_min}..{cal.range_max}"
                seen = f"{observed_min[name]}..{observed_max[name]}"
                state = range_flag(raw, cal.range_min, cal.range_max)
                print(
                    f"{name:<15} {cal.id:>2} {raw:>5} {value:>7.1f} {unit:<3} "
                    f"{saved:>13} {seen:>13} {state:>8}"
                )
            sys.stdout.flush()

            if args.samples == 0 or update < args.samples:
                time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        print("\nStopped. No motor values were changed.")
    finally:
        if device.bus.is_connected:
            device.bus.disconnect(disable_torque=False)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
