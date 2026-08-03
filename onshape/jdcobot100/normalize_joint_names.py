#!/usr/bin/env python3
"""Normalize the four jdcobot100 joints after onshape-to-robot export."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


JOINT_NAME_MAP = {
    # Current DAPIER assembly. onshape-to-robot removes the dof_ prefix.
    "base": "dof_base",
    "shoulder": "dof_shoulder",
    "elbow": "dof_elbow",
    "wrist_pitch": "dof_wrist_pitch",
    # Original JD-edu reference assembly.
    "base_shoulder": "dof_base",
    "shoulder_arm1": "dof_shoulder",
    "arm1_arm2": "dof_elbow",
    "arn2_end_arm": "dof_wrist_pitch",
}


def normalize(path: Path) -> None:
    """Normalize joint attributes without changing links, meshes, or materials."""
    tree = ET.parse(path)
    root = tree.getroot()

    for element in root.iter():
        if element.tag == "joint":
            name = element.attrib.get("name")
            if name in JOINT_NAME_MAP:
                element.attrib["name"] = JOINT_NAME_MAP[name]

        for attribute in ("joint", "joint1", "joint2"):
            name = element.attrib.get(attribute)
            if name in JOINT_NAME_MAP:
                element.attrib[attribute] = JOINT_NAME_MAP[name]

    joint_names = {
        element.attrib["name"]
        for element in root.iter("joint")
        if "name" in element.attrib
    }
    missing = sorted(set(JOINT_NAME_MAP.values()) - joint_names)
    if missing:
        raise ValueError(f"{path}: normalized joints not found: {', '.join(missing)}")

    tree.write(path, encoding="unicode", xml_declaration=True)
    print(f"normalized {path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", type=Path, help="URDF or MJCF files")
    args = parser.parse_args()

    for path in args.paths:
        normalize(path)


if __name__ == "__main__":
    main()
