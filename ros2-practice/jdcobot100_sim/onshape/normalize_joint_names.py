#!/usr/bin/env python3
"""Normalize the four jdcobot100 joints after onshape-to-robot export."""

from __future__ import annotations

import argparse
from pathlib import Path


JOINT_NAME_MAP = {
    "base_shoulder": "dof_base",
    "shoulder_arm1": "dof_shoulder",
    "arm1_arm2": "dof_elbow",
    "arn2_end_arm": "dof_wrist_pitch",
}


def normalize(path: Path) -> None:
    """Replace quoted joint identifiers without touching link or mesh names."""
    contents = path.read_text(encoding="utf-8")

    for old_name, new_name in JOINT_NAME_MAP.items():
        old_token = f'"{old_name}"'
        new_token = f'"{new_name}"'
        if old_token in contents:
            contents = contents.replace(old_token, new_token)

    missing = [
        new_name
        for new_name in JOINT_NAME_MAP.values()
        if f'"{new_name}"' not in contents
    ]
    if missing:
        raise ValueError(f"{path}: normalized joints not found: {', '.join(missing)}")

    path.write_text(contents, encoding="utf-8")
    print(f"normalized {path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", type=Path, help="URDF or MJCF files")
    args = parser.parse_args()

    for path in args.paths:
        normalize(path)


if __name__ == "__main__":
    main()
