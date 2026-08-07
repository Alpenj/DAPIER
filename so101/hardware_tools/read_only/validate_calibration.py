#!/usr/bin/env python3
"""Validate SO-101 LeRobot calibration JSON files before teleoperation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

MOTOR_IDS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}
REQUIRED_FIELDS = {"id", "drive_mode", "homing_offset", "range_min", "range_max"}


def parse_args() -> argparse.Namespace:
    default_root = Path.home() / ".cache/huggingface/lerobot/calibration"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--follower",
        type=Path,
        default=default_root / "robots/so_follower/so101_follower_main.json",
    )
    parser.add_argument(
        "--leader",
        type=Path,
        default=default_root / "teleoperators/so_leader/so101_leader_main.json",
    )
    return parser.parse_args()


def validate(path: Path, label: str) -> list[str]:
    errors: list[str] = []
    if not path.is_file():
        return [f"{label}: missing file: {path}"]
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return [f"{label}: unreadable JSON: {exc}"]
    if not isinstance(data, dict):
        return [f"{label}: top-level JSON must be an object"]

    names = set(data)
    if names != set(MOTOR_IDS):
        missing = sorted(set(MOTOR_IDS) - names)
        extra = sorted(names - set(MOTOR_IDS))
        errors.append(f"{label}: motor keys mismatch; missing={missing}, extra={extra}")

    seen_ids: set[int] = set()
    for name, expected_id in MOTOR_IDS.items():
        record = data.get(name)
        if not isinstance(record, dict):
            errors.append(f"{label}/{name}: record missing or not an object")
            continue
        missing_fields = REQUIRED_FIELDS - set(record)
        if missing_fields:
            errors.append(f"{label}/{name}: missing fields {sorted(missing_fields)}")
            continue
        for field in REQUIRED_FIELDS:
            if not isinstance(record[field], int):
                errors.append(f"{label}/{name}: {field} must be an integer")
        if any(not isinstance(record[field], int) for field in REQUIRED_FIELDS):
            continue
        if record["id"] != expected_id:
            errors.append(
                f"{label}/{name}: expected motor ID {expected_id}, got {record['id']}"
            )
        if record["id"] in seen_ids:
            errors.append(f"{label}/{name}: duplicate motor ID {record['id']}")
        seen_ids.add(record["id"])
        if record["drive_mode"] not in (0, 1):
            errors.append(f"{label}/{name}: drive_mode must be 0 or 1")
        if not record["range_min"] < record["range_max"]:
            errors.append(f"{label}/{name}: range_min must be less than range_max")
        if name == "wrist_roll" and (
            record["range_min"] != 0 or record["range_max"] != 4095
        ):
            errors.append(f"{label}/{name}: LeRobot v0.6.0 expects range 0..4095")
        if name != "wrist_roll" and record["range_max"] - record["range_min"] < 100:
            errors.append(f"{label}/{name}: suspiciously small recorded range")
        if abs(record["homing_offset"]) > 4095:
            errors.append(f"{label}/{name}: homing_offset outside plausible tick range")
    return errors


def main() -> int:
    args = parse_args()
    all_errors = validate(args.follower, "follower") + validate(args.leader, "leader")
    if all_errors:
        print("CALIBRATION INVALID")
        for error in all_errors:
            print(f"- {error}")
        return 1
    print("CALIBRATION VALID")
    print(f"- follower: {args.follower}")
    print(f"- leader: {args.leader}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
