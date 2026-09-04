#!/usr/bin/env python3
"""Small hardware-independent contract check for the RoboTwin adapter."""

import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

from build_dual_so101 import ARM_JOINTS, build


def _source(root: Path) -> tuple[Path, Path]:
    meshes = root / "meshes"
    meshes.mkdir()
    (meshes / "link.stl").write_bytes(b"solid link\nendsolid link\n")
    urdf = root / "so101.urdf"
    links = ['<link name="base_link"/>']
    joints = []
    parent = "base_link"
    for index, name in enumerate((*ARM_JOINTS, "gripper"), start=1):
        child = f"link_{index}"
        links.append(
            f'<link name="{child}"><visual><geometry><mesh filename="link.stl"/></geometry></visual></link>'
        )
        joints.append(
            f'<joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{child}"/>'
            '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>'
        )
        parent = child
    links.append('<link name="gripper_frame_link"/>')
    links.append(
        f'<joint name="gripper_frame_fixed" type="fixed"><parent link="{parent}"/>'
        '<child link="gripper_frame_link"/></joint>'
    )
    urdf.write_text(
        f'<robot name="so101">{"".join(links + joints)}</robot>', encoding="utf-8"
    )
    return urdf, meshes


def main() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        source, meshes = _source(root)
        output = build(source, meshes, root / "out")
        robot = ET.parse(output).getroot()
        active = {
            joint.get("name")
            for joint in robot.findall("joint")
            if joint.get("type") != "fixed"
        }
        expected = {
            f"{side}_{name}"
            for side in ("left", "right")
            for name in (*ARM_JOINTS, "gripper")
        }
        assert active == expected
        mounts = {
            joint.get("name"): joint.find("origin").get("xyz")
            for joint in robot.findall("joint")
            if joint.get("name", "").endswith("_mount_fixed")
        }
        assert mounts == {
            "left_mount_fixed": "-0.064 0.08 0.1095",
            "right_mount_fixed": "-0.064 -0.08 0.1095",
        }
        assert (root / "out/meshes/link.stl").is_file()
    print("PASS: DAPIER RoboTwin 12-axis and mount contract")


if __name__ == "__main__":
    main()
