#!/usr/bin/env python3
"""Build RoboTwin's runtime-only Dual SO-101 asset from the official arm URDF."""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
MOUNT_X_M = -0.064
MOUNT_Y_M = 0.080
MOUNT_Z_M = 0.1095


def _prefixed_arm(
    source: ET.Element, side: str, mesh_source: Path, mesh_target: Path
) -> list[ET.Element]:
    elements: list[ET.Element] = []
    for original in source:
        if original.tag not in {"link", "joint"}:
            continue
        item = deepcopy(original)
        item.set("name", f"{side}_{item.get('name')}")
        for ref in item.findall(".//parent") + item.findall(".//child"):
            ref.set("link", f"{side}_{ref.get('link')}")
        for mimic in item.findall(".//mimic"):
            mimic.set("joint", f"{side}_{mimic.get('joint')}")
        for mesh in item.findall(".//mesh"):
            name = Path(mesh.get("filename", "")).name
            source_mesh = mesh_source / name
            if not source_mesh.is_file():
                raise FileNotFoundError(f"missing SO-101 mesh: {source_mesh}")
            shutil.copy2(source_mesh, mesh_target / name)
            mesh.set("filename", f"../meshes/{name}")
        elements.append(item)
    return elements


def _box_link(name: str, size: str, xyz: str, rgba: str) -> ET.Element:
    return ET.fromstring(
        f"""<link name="{name}">
  <visual><origin xyz="{xyz}"/><geometry><box size="{size}"/></geometry>
    <material name="{name}_color"><color rgba="{rgba}"/></material></visual>
  <collision><origin xyz="{xyz}"/><geometry><box size="{size}"/></geometry></collision>
  <inertial><origin xyz="{xyz}"/><mass value="0.001"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>
</link>"""
    )


def _fixed_joint(
    name: str, parent: str, child: str, xyz: str, rpy: str = "0 0 0"
) -> ET.Element:
    return ET.fromstring(
        f"""<joint name="{name}" type="fixed"><origin xyz="{xyz}" rpy="{rpy}"/>
  <parent link="{parent}"/><child link="{child}"/></joint>"""
    )


def build(source_urdf: Path, mesh_source: Path, output: Path) -> Path:
    source = ET.parse(source_urdf).getroot()
    output.mkdir(parents=True, exist_ok=True)
    mesh_target = output / "meshes"
    urdf_target = output / "urdf"
    mesh_target.mkdir(exist_ok=True)
    urdf_target.mkdir(exist_ok=True)

    robot = ET.Element("robot", {"name": "dapier_dual_so101"})
    robot.append(
        _box_link(
            "dapier_base_link",
            "0.281 0.306 0.141",
            "0 0 0.0705",
            "0.25 0.25 0.25 1",
        )
    )
    robot.append(
        _box_link(
            "h201_mast_link",
            "0.024 0.030 0.310",
            "0 0 0.155",
            "0.15 0.15 0.15 1",
        )
    )
    robot.append(
        _fixed_joint(
            "h201_mast_fixed",
            "dapier_base_link",
            "h201_mast_link",
            "-0.064 0 0.110",
        )
    )
    robot.append(
        _box_link(
            "h201_link", "0.0255 0.090 0.025", "0 0 0", "0.05 0.05 0.05 1"
        )
    )
    robot.append(
        _fixed_joint(
            "h201_fixed",
            "dapier_base_link",
            "h201_link",
            "-0.064 0 0.420",
            "0 0.750492 0",
        )
    )

    for side, y in (("left", MOUNT_Y_M), ("right", -MOUNT_Y_M)):
        for element in _prefixed_arm(source, side, mesh_source, mesh_target):
            robot.append(element)
        robot.append(
            _fixed_joint(
                f"{side}_mount_fixed",
                "dapier_base_link",
                f"{side}_base_link",
                f"{MOUNT_X_M} {y} {MOUNT_Z_M}",
            )
        )
        robot.append(
            _box_link(
                f"{side}_camera",
                "0.035 0.020 0.035",
                "0 0 0",
                "0.1 0.1 0.1 1",
            )
        )
        robot.append(
            _fixed_joint(
                f"{side}_camera_fixed",
                f"{side}_gripper_frame_link",
                f"{side}_camera",
                "0.0025 -0.072057361 0.004150235",
                "0 1.134464014 1.570796327",
            )
        )

    ET.indent(robot, space="  ")
    destination = urdf_target / "dapier_dual_so101.urdf"
    ET.ElementTree(robot).write(destination, encoding="utf-8", xml_declaration=True)
    return destination


def validate_with_sapien(urdf: Path) -> None:
    import sapien

    scene = sapien.Scene()
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load(str(urdf))
    if robot is None:
        raise RuntimeError("SAPIEN did not load the generated Dual SO-101 URDF")
    active = [joint.name for joint in robot.get_active_joints()]
    expected = [
        f"{side}_{joint}"
        for side in ("left", "right")
        for joint in (*ARM_JOINTS, "gripper")
    ]
    if len(active) != len(expected) or set(active) != set(expected):
        raise RuntimeError(f"unexpected active joints: {active!r}")
    links = {link.name for link in robot.get_links()}
    required = {"left_camera", "right_camera", "h201_link", "dapier_base_link"}
    if not required <= links:
        raise RuntimeError(f"missing links: {sorted(required - links)}")
    print(f"PASS: {len(active)} joints, {len(links)} links, {urdf}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-urdf", type=Path, required=True)
    parser.add_argument("--mesh-source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    urdf = build(
        args.source_urdf.resolve(), args.mesh_source.resolve(), args.output.resolve()
    )
    validate_with_sapien(urdf)


if __name__ == "__main__":
    main()
