#!/usr/bin/env python3
"""Build and validate the DAPIER dual-JDcobot200 MuJoCo model.

This module only creates an in-process simulator model. It has no ROS 2,
serial, USB, Dynamixel, or other hardware command path.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Sequence

import mujoco


PROJECT_DIR = Path(__file__).resolve().parent
UPSTREAM_MODEL = PROJECT_DIR / "upstream" / "jdcobot200.xml"

ARM_CONTROL_NAMES = (
    "base",
    "shoulder",
    "elbow",
    "wrist_pitch",
    "wrist_roll",
    "gripper_motor",
)
ACTION_NAMES = tuple(
    f"{side}_{name}" for side in ("left", "right") for name in ARM_CONTROL_NAMES
)
GRIPPER_CONTROL_RANGE_RAD = (-0.57, 0.57)

_PROVISIONAL_COLLISIONS = (
    (
        "arm_link_assembly",
        "upper_arm",
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        {
            "fromto": [0.0, 0.0, 0.0, 0.135, 0.0, 0.0],
            "size": [0.022, 0.0, 0.0],
        },
    ),
    (
        "arm_link_assembly_2",
        "forearm",
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        {
            "fromto": [0.0, 0.0, 0.0, 0.135, 0.0, 0.0],
            "size": [0.022, 0.0, 0.0],
        },
    ),
    (
        "wrist_assembly",
        "wrist",
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        {
            "fromto": [0.0, 0.0, 0.0, 0.058, 0.0, 0.0],
            "size": [0.018, 0.0, 0.0],
        },
    ),
    (
        "gripper_assembly",
        "gripper_base",
        mujoco.mjtGeom.mjGEOM_SPHERE,
        {"pos": [0.0, 0.0, -0.03], "size": [0.035, 0.0, 0.0]},
    ),
    (
        "gripper_left_assembly",
        "left_finger",
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        {
            "fromto": [0.0, 0.0, 0.0, 0.0, 0.0, 0.075],
            "size": [0.008, 0.0, 0.0],
        },
    ),
    (
        "gripper_right_assembly",
        "right_finger",
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        {
            "fromto": [0.0, 0.0, 0.0, 0.0, 0.0, 0.075],
            "size": [0.008, 0.0, 0.0],
        },
    ),
)

_SCENE_XML = """
<mujoco model="dapier_jdcobot200_dual">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="135" elevation="-22"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
             width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
             width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true"
              texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" pos="0 0 -0.001" type="plane"
          material="groundplane"/>
  </worldbody>
</mujoco>
"""


def _yaw_quaternion(yaw_radians: float) -> list[float]:
    half = yaw_radians / 2.0
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def _add_provisional_collisions(arm_spec: mujoco.MjSpec) -> None:
    """Add coarse contact geometry without modifying the vendored MJCF."""

    for body_name, geom_name, geom_type, geometry in _PROVISIONAL_COLLISIONS:
        body = arm_spec.body(body_name)
        body.add_geom(
            name=f"dapier_collision_{geom_name}",
            type=geom_type,
            contype=2,
            conaffinity=2,
            group=3,
            rgba=[1.0, 0.2, 0.2, 0.25],
            **geometry,
        )


def attach_dual_arms(
    spec: mujoco.MjSpec,
    parent_body: object,
    mount_separation_m: float,
    mount_height_m: float,
    mount_x_m: float = 0.0,
    left_yaw_rad: float = 0.0,
    right_yaw_rad: float = 0.0,
    primitive_collisions: bool = True,
) -> None:
    """Attach namespaced left/right arms to an existing MuJoCo body."""

    if mount_separation_m <= 0:
        raise ValueError("mount_separation_m must be positive")
    if not UPSTREAM_MODEL.is_file():
        raise FileNotFoundError(f"missing upstream model: {UPSTREAM_MODEL}")

    half_separation = mount_separation_m / 2.0
    mounts = (
        (
            "left",
            [mount_x_m, half_separation, mount_height_m],
            _yaw_quaternion(left_yaw_rad),
        ),
        (
            "right",
            [mount_x_m, -half_separation, mount_height_m],
            _yaw_quaternion(right_yaw_rad),
        ),
    )
    for side, position, quaternion in mounts:
        frame = parent_body.add_frame(
            name=f"{side}_arm_mount", pos=position, quat=quaternion
        )
        arm_spec = mujoco.MjSpec.from_file(str(UPSTREAM_MODEL))
        if primitive_collisions:
            _add_provisional_collisions(arm_spec)
        spec.attach(arm_spec, prefix=f"{side}_", frame=frame)


def _apply_gripper_limits(model: mujoco.MjModel) -> None:
    """Constrain the source model's over-wide gripper range in DAPIER only."""

    for side in ("left", "right"):
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{side}_gripper_motor"
        )
        model.actuator_ctrllimited[actuator_id] = 1
        model.actuator_ctrlrange[actuator_id, :] = GRIPPER_CONTROL_RANGE_RAD
        for finger in ("left", "right"):
            joint_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                f"{side}_gripper_{finger}",
            )
            model.jnt_limited[joint_id] = 1
            model.jnt_range[joint_id, :] = GRIPPER_CONTROL_RANGE_RAD


def build_model(
    mount_separation_m: float = 0.36,
    mount_height_m: float = 0.0,
    left_yaw_rad: float = 0.0,
    right_yaw_rad: float = 0.0,
    primitive_collisions: bool = True,
) -> mujoco.MjModel:
    """Return a namespaced two-arm model compiled entirely in memory.

    Defaults are provisional visualization values, not measured TurtleBot3
    mounting transforms.
    """

    spec = mujoco.MjSpec.from_string(_SCENE_XML)
    attach_dual_arms(
        spec,
        spec.worldbody,
        mount_separation_m=mount_separation_m,
        mount_height_m=mount_height_m,
        left_yaw_rad=left_yaw_rad,
        right_yaw_rad=right_yaw_rad,
        primitive_collisions=primitive_collisions,
    )

    model = spec.compile()
    _apply_gripper_limits(model)
    return model


def model_names(
    model: mujoco.MjModel, object_type: mujoco.mjtObj, count: int
) -> tuple[str, ...]:
    return tuple(
        mujoco.mj_id2name(model, object_type, index) or "" for index in range(count)
    )


def actuator_targets_from_qpos(
    model: mujoco.MjModel, qpos: Sequence[float]
) -> tuple[float, ...]:
    """Map each joint position actuator to its current joint position."""

    if len(qpos) != model.nq:
        raise ValueError(f"expected qpos length {model.nq}, got {len(qpos)}")
    if model.nu != len(ACTION_NAMES):
        raise RuntimeError(f"expected {len(ACTION_NAMES)} actuators, got {model.nu}")

    targets = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            raise RuntimeError(f"actuator {actuator_id} is not attached to a joint")
        qpos_address = int(model.jnt_qposadr[joint_id])
        targets.append(float(qpos[qpos_address]))
    return tuple(targets)


def validate_model(model: mujoco.MjModel, smoke_steps: int = 1000) -> dict[str, object]:
    """Check the structural 12D contract and an initial-pose hold rollout."""

    if smoke_steps < 0:
        raise ValueError("smoke_steps must be non-negative")

    joints = model_names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    actuators = model_names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
    if (model.nq, model.nv, model.nu, model.njnt, model.neq) != (14, 14, 12, 14, 2):
        raise RuntimeError(
            "unexpected model dimensions: "
            f"nq={model.nq}, nv={model.nv}, nu={model.nu}, "
            f"njnt={model.njnt}, neq={model.neq}"
        )
    if actuators != ACTION_NAMES:
        raise RuntimeError(f"unexpected actuator order: {actuators!r}")
    if not all(bool(value) for value in model.actuator_ctrllimited):
        raise RuntimeError("every actuator must have a finite control range")

    data = mujoco.MjData(model)
    data.ctrl[:] = actuator_targets_from_qpos(model, data.qpos)
    mujoco.mj_forward(model, data)
    for _ in range(smoke_steps):
        mujoco.mj_step(model, data)

    finite_state = all(math.isfinite(float(value)) for value in data.qpos) and all(
        math.isfinite(float(value)) for value in data.qvel
    )
    if not finite_state:
        raise RuntimeError("simulation produced a non-finite state")

    disabled_collision_geoms = sum(
        int(model.geom_contype[index] == 0 and model.geom_conaffinity[index] == 0)
        for index in range(model.ngeom)
    )
    contacts_after_smoke = data.ncon
    return {
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "njnt": model.njnt,
        "neq": model.neq,
        "ngeom": model.ngeom,
        "joints": joints,
        "actuators": actuators,
        "action_names": ACTION_NAMES,
        "smoke_steps": smoke_steps,
        "finite_state": finite_state,
        "max_abs_qvel": max((abs(float(value)) for value in data.qvel), default=0.0),
        "disabled_collision_geoms": disabled_collision_geoms,
        "contacts_after_smoke": contacts_after_smoke,
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build and headlessly validate the DAPIER dual-JDcobot200 MuJoCo model."
    )
    parser.add_argument("--mount-separation-m", type=float, default=0.36)
    parser.add_argument("--mount-height-m", type=float, default=0.0)
    parser.add_argument("--left-yaw-rad", type=float, default=0.0)
    parser.add_argument("--right-yaw-rad", type=float, default=0.0)
    parser.add_argument("--smoke-steps", type=int, default=1000)
    parser.add_argument(
        "--no-primitive-collisions",
        action="store_true",
        help="disable the provisional DAPIER primitive collision overlay",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="open a MuJoCo viewer after validation; simulation only",
    )
    return parser


def _run_viewer(model: mujoco.MjModel) -> None:
    import mujoco.viewer

    data = mujoco.MjData(model)
    data.ctrl[:] = actuator_targets_from_qpos(model, data.qpos)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    model = build_model(
        mount_separation_m=args.mount_separation_m,
        mount_height_m=args.mount_height_m,
        left_yaw_rad=args.left_yaw_rad,
        right_yaw_rad=args.right_yaw_rad,
        primitive_collisions=not args.no_primitive_collisions,
    )
    report = validate_model(model, smoke_steps=args.smoke_steps)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.viewer:
        _run_viewer(model)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
