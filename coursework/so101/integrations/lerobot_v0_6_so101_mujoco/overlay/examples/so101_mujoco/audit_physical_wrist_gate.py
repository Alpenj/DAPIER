#!/usr/bin/env python3
"""Inventory the physical wrist-camera gate without opening devices or moving motors."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from lerobot.envs.so101_mujoco import (
    WRIST_CAMERA_PROFILE_ID,
    build_physical_wrist_gate_receipt,
    discover_hardware_inventory,
    load_camera_profile,
    write_physical_wrist_gate_receipt,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--expected-camera-name",
        action="append",
        required=True,
        help="Expected substring in the operator-confirmed wrist camera name; repeatable.",
    )
    parser.add_argument("--output", type=Path, required=True, help="JSON receipt path")
    parser.add_argument("--camera-profile-id", default=WRIST_CAMERA_PROFILE_ID)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    profile = load_camera_profile(args.camera_profile_id)
    receipt = build_physical_wrist_gate_receipt(
        discover_hardware_inventory(),
        expected_camera_name_substrings=tuple(args.expected_camera_name),
        camera_profile_id=profile.profile_id,
        physical_alignment_verified=profile.physical_alignment_verified,
    )
    write_physical_wrist_gate_receipt(args.output, receipt)
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0 if receipt["status"] == "ready_for_operator_validation" else 2


if __name__ == "__main__":
    raise SystemExit(main())
