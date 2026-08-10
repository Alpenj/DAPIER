#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Validate SO-101 calibration JSON files against the simulation joint contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from lerobot.envs.so101_mujoco import JOINT_NAMES

REQUIRED_FIELDS = ("id", "drive_mode", "homing_offset", "range_min", "range_max")


def validate_calibration(path: Path) -> list[str]:
    errors: list[str] = []
    try:
        calibration: dict[str, Any] = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        return [f"{path}: cannot read calibration JSON: {exc}"]

    if tuple(calibration) != JOINT_NAMES:
        errors.append(f"{path}: joint order must be {list(JOINT_NAMES)}, got {list(calibration)}")

    for expected_id, joint_name in enumerate(JOINT_NAMES, start=1):
        entry = calibration.get(joint_name)
        if not isinstance(entry, dict):
            errors.append(f"{path}: missing object for joint {joint_name!r}")
            continue
        missing = [field for field in REQUIRED_FIELDS if field not in entry]
        if missing:
            errors.append(f"{path}: {joint_name} is missing {missing}")
            continue
        if any(not isinstance(entry[field], int) for field in REQUIRED_FIELDS):
            errors.append(f"{path}: {joint_name} calibration fields must all be integers")
            continue
        if entry["id"] != expected_id:
            errors.append(f"{path}: {joint_name} motor id must be {expected_id}, got {entry['id']}")
        if not 0 <= entry["range_min"] < entry["range_max"] <= 4095:
            errors.append(
                f"{path}: {joint_name} range must satisfy 0 <= min < max <= 4095, "
                f"got {entry['range_min']}..{entry['range_max']}"
            )
        elif entry["range_max"] - entry["range_min"] < 500:
            errors.append(f"{path}: {joint_name} calibrated span is suspiciously small (<500 ticks)")

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibrations", type=Path, nargs="+", help="One or more calibration JSON paths")
    args = parser.parse_args()

    errors = [error for path in args.calibrations for error in validate_calibration(path)]
    if errors:
        print("SO-101 SIM/REAL CONTRACT: FAIL")
        for error in errors:
            print(f"- {error}")
        return 1

    print("SO-101 SIM/REAL CONTRACT: PASS")
    print(f"joint_order={','.join(JOINT_NAMES)}")
    print("action_units=degree,degree,degree,degree,degree,gripper_percent_0_to_100")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
