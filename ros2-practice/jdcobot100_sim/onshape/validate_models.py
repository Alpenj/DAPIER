#!/usr/bin/env python3
"""Headless structural and simulation checks for the jdcobot100 models."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


EXPECTED_JOINTS = [
    "dof_base",
    "dof_shoulder",
    "dof_elbow",
    "dof_wrist_pitch",
]


def require_expected(actual: list[str], source: str) -> None:
    if actual != EXPECTED_JOINTS:
        raise AssertionError(
            f"{source}: expected {EXPECTED_JOINTS}, got {actual}"
        )


def validate_urdf_structure(path: Path) -> None:
    root = ET.parse(path).getroot()
    names = {
        joint.attrib["name"]
        for joint in root.findall("joint")
        if joint.attrib.get("type") != "fixed"
    }
    if names != set(EXPECTED_JOINTS):
        raise AssertionError(f"URDF joints: expected {EXPECTED_JOINTS}, got {sorted(names)}")


def validate_pybullet(path: Path, steps: int) -> None:
    import pybullet as bullet

    client = bullet.connect(bullet.DIRECT)
    try:
        body = bullet.loadURDF(str(path.resolve()), useFixedBase=True)
        joint_indices = [
            index
            for index in range(bullet.getNumJoints(body))
            if bullet.getJointInfo(body, index)[2] != bullet.JOINT_FIXED
        ]
        joint_names = [
            bullet.getJointInfo(body, index)[1].decode("utf-8")
            for index in joint_indices
        ]
        require_expected(joint_names, "PyBullet")

        targets = [0.08, -0.12, 0.1, -0.06]
        bullet.setJointMotorControlArray(
            body,
            joint_indices,
            bullet.POSITION_CONTROL,
            targetPositions=targets,
        )
        for _ in range(steps):
            bullet.stepSimulation()

        positions = [bullet.getJointState(body, index)[0] for index in joint_indices]
        if not all(math.isfinite(value) for value in positions):
            raise AssertionError(f"PyBullet produced non-finite positions: {positions}")
        print(f"PASS PyBullet: joints={joint_names}, steps={steps}")
    finally:
        bullet.disconnect(client)


def validate_mujoco(path: Path, steps: int) -> None:
    import mujoco
    import numpy as np

    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    data = mujoco.MjData(model)
    hinge_type = int(mujoco.mjtJoint.mjJNT_HINGE)
    hinge_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
        if int(model.jnt_type[index]) == hinge_type
    ]
    require_expected(hinge_names, "MuJoCo")
    if model.nu != len(EXPECTED_JOINTS):
        raise AssertionError(f"MuJoCo: expected nu=4, got nu={model.nu}")

    data.ctrl[:] = np.array([0.08, -0.12, 0.1, -0.06])
    for _ in range(steps):
        mujoco.mj_step(model, data)
    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
        raise AssertionError("MuJoCo produced non-finite state")
    print(
        "PASS MuJoCo: "
        f"hinges={hinge_names}, free_base={model.njnt - len(hinge_names)}, "
        f"actuators={model.nu}, steps={steps}"
    )


def main() -> None:
    directory = Path(__file__).resolve().parent / "reference"
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=directory / "jdcobot100.urdf")
    parser.add_argument("--mjcf", type=Path, default=directory / "jdcobot100.xml")
    parser.add_argument("--steps", type=int, default=240)
    args = parser.parse_args()

    validate_urdf_structure(args.urdf)
    validate_pybullet(args.urdf, args.steps)
    validate_mujoco(args.mjcf, args.steps)


if __name__ == "__main__":
    main()
