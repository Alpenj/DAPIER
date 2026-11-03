#!/usr/bin/env python3
"""Read all six SO-101 motor positions without commanding any motion."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read raw ticks and calibrated positions from one SO-101 arm."
    )
    parser.add_argument("--role", choices=("follower", "leader"), required=True)
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--id", required=True, help="Stable LeRobot calibration ID")
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--output", type=Path, help="Optional JSONL output path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples < 1:
        raise SystemExit("--samples must be at least 1")
    if args.interval < 0:
        raise SystemExit("--interval must be non-negative")

    if args.role == "follower":
        device = SO101Follower(
            SO101FollowerConfig(port=args.port, id=args.id, cameras={})
        )
    else:
        device = SO101Leader(SO101LeaderConfig(port=args.port, id=args.id))

    expected_ids = {
        "shoulder_pan": 1,
        "shoulder_lift": 2,
        "elbow_flex": 3,
        "wrist_flex": 4,
        "wrist_roll": 5,
        "gripper": 6,
    }
    actual_ids = {name: motor.id for name, motor in device.bus.motors.items()}
    if actual_ids != expected_ids:
        raise SystemExit(f"Unexpected motor mapping: {actual_ids}")

    output_handle = None
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        output_handle = args.output.open("a", encoding="utf-8")

    try:
        # Intentionally connect only the motor bus. This avoids configure(), torque
        # changes, camera startup, calibration prompts, and all Goal_Position writes.
        device.bus.connect()
        has_calibration = set(device.calibration) == set(expected_ids)
        print(f"role={args.role} port={args.port} id={args.id}")
        print(f"motor_ids={actual_ids}")
        print(f"calibration_file={device.calibration_fpath}")
        print(f"calibration_loaded={has_calibration}")

        for sample_index in range(args.samples):
            raw = device.bus.sync_read("Present_Position", normalize=False, num_retry=2)
            normalized = (
                device.bus.sync_read("Present_Position", normalize=True, num_retry=2)
                if has_calibration
                else None
            )
            record = {
                "timestamp": datetime.now().astimezone().isoformat(),
                "sample": sample_index + 1,
                "role": args.role,
                "port": args.port,
                "device_id": args.id,
                "raw_ticks": raw,
                "calibrated_position": normalized,
            }
            line = json.dumps(record, ensure_ascii=False)
            print(line)
            if output_handle:
                output_handle.write(line + "\n")
                output_handle.flush()
            if sample_index + 1 < args.samples:
                time.sleep(args.interval)
    finally:
        if output_handle:
            output_handle.close()
        if device.bus.is_connected:
            # No torque state was changed, so close the serial port without a write.
            device.bus.disconnect(disable_torque=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
