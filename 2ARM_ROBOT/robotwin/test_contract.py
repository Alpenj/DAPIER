#!/usr/bin/env python3
"""Small hardware-independent contract check for the RoboTwin adapter."""

import json
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
        fixed = {joint.get("name") for joint in robot.findall("joint")}
        assert {"left_ee_joint", "right_ee_joint"} <= fixed
        tcp_origins = {
            joint.get("name"): joint.find("origin").get("xyz")
            for joint in robot.findall("joint")
            if joint.get("name", "").endswith("_ee_joint")
        }
        assert tcp_origins == {
            "left_ee_joint": "0 0 0",
            "right_ee_joint": "0 0 0",
        }
        grasp_joints = {
            joint.get("name"): (
                joint.find("parent").get("link"),
                joint.find("origin").get("xyz"),
            )
            for joint in robot.findall("joint")
            if joint.get("name", "").endswith("_grasp_joint")
        }
        assert grasp_joints == {
            "left_grasp_joint": (
                "left_gripper_frame_link",
                "-0.010238 0.000273 -0.060284",
            ),
            "right_grasp_joint": (
                "right_gripper_frame_link",
                "-0.010238 0.000273 -0.060284",
            ),
        }
        cameras = {
            joint.get("name"): (
                joint.find("parent").get("link"),
                joint.find("origin").get("xyz"),
            )
            for joint in robot.findall("joint")
            if joint.get("name", "").endswith("_camera_fixed")
        }
        assert cameras == {
            "left_camera_fixed": (
                "left_gripper_link",
                "0.0025 -0.072057361 0.004150235",
            ),
            "right_camera_fixed": (
                "right_gripper_link",
                "0.0025 -0.072057361 0.004150235",
            ),
        }
        links = {link.get("name") for link in robot.findall("link")}
        assert {
            "left_ee_link",
            "right_ee_link",
            "left_grasp_link",
            "right_grasp_link",
        } <= links
        assert (root / "out/meshes/link.stl").is_file()
    srdf = ET.parse(
        Path(__file__).parent
        / "overlay/assets/embodiments/dapier-dual-so101/urdf/dapier_dual_so101.srdf"
    ).getroot()
    disabled = {
        frozenset((item.get("link1"), item.get("link2")))
        for item in srdf.findall("disable_collisions")
    }
    assert len(disabled) == 21
    assert frozenset(("dapier_base_link", "left_shoulder_link")) in disabled
    assert frozenset(("dapier_base_link", "right_shoulder_link")) in disabled
    instruction = json.loads(
        (
            Path(__file__).parent
            / "overlay/description/task_instruction/dapier_handover_block.json"
        ).read_text(encoding="utf-8")
    )
    assert instruction["seen"] and instruction["unseen"]
    print("PASS: DAPIER RoboTwin 12-axis and mount contract")


if __name__ == "__main__":
    main()
