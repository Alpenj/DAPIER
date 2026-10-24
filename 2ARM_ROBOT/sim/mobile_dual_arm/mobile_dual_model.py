#!/usr/bin/env python3
"""Compose Waffle Pi and two JDcobot200 arms in a stationary MuJoCo scene."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Sequence

import mujoco


PROJECT_DIR = Path(__file__).resolve().parent
SIM_DIR = PROJECT_DIR.parent
sys.path.insert(0, str(SIM_DIR / "jdcobot200_dual"))
sys.path.insert(0, str(SIM_DIR / "turtlebot3_waffle_pi"))

from dual_model import (
    ACTION_NAMES,
    actuator_targets_from_qpos,
    attach_dual_arms,
    model_names,
)
from waffle_pi_model import build_spec as build_waffle_pi_spec


DEFAULT_ARM_MOUNT_X_M = 0.02
DEFAULT_ARM_MOUNT_SEPARATION_M = 0.18
DEFAULT_ARM_MOUNT_HEIGHT_M = 0.20


def build_model(
    arm_mount_x_m: float = DEFAULT_ARM_MOUNT_X_M,
    arm_mount_separation_m: float = DEFAULT_ARM_MOUNT_SEPARATION_M,
    arm_mount_height_m: float = DEFAULT_ARM_MOUNT_HEIGHT_M,
    left_yaw_rad: float = 0.0,
    right_yaw_rad: float = 0.0,
    primitive_collisions: bool = True,
) -> mujoco.MjModel:
    """Build the fixed-base manipulation model.

    Mount defaults are provisional visualization values, not physical
    measurements from the DAPIER TurtleBot3.
    """

    spec = build_waffle_pi_spec()
    base_link = spec.body("tb3_base_link")
    attach_dual_arms(
        spec,
        base_link,
        mount_separation_m=arm_mount_separation_m,
        mount_height_m=arm_mount_height_m,
        mount_x_m=arm_mount_x_m,
        left_yaw_rad=left_yaw_rad,
        right_yaw_rad=right_yaw_rad,
        primitive_collisions=primitive_collisions,
    )
    return spec.compile()


def validate_model(model: mujoco.MjModel, smoke_steps: int = 1000) -> dict[str, object]:
    if smoke_steps < 0:
        raise ValueError("smoke_steps must be non-negative")
    if (model.nq, model.nv, model.nu, model.njnt, model.neq) != (16, 16, 12, 16, 2):
        raise RuntimeError(
            "unexpected mobile dual-arm dimensions: "
            f"nq={model.nq}, nv={model.nv}, nu={model.nu}, "
            f"njnt={model.njnt}, neq={model.neq}"
        )

    actuators = model_names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
    if actuators != ACTION_NAMES:
        raise RuntimeError(f"unexpected arm actuator order: {actuators!r}")
    wheel_actuators = tuple(name for name in actuators if "wheel" in name)
    if wheel_actuators:
        raise RuntimeError(f"wheel actuators are not authorized: {wheel_actuators!r}")

    base_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link"
    )
    if base_id < 0:
        raise RuntimeError("tb3_base_link is missing")
    for arm_body in ("left_base_assembly", "right_base_assembly"):
        arm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, arm_body)
        if arm_id < 0:
            raise RuntimeError(f"missing arm body: {arm_body}")
        if int(model.body_parentid[arm_id]) != base_id:
            raise RuntimeError(f"{arm_body} is not mounted to tb3_base_link")

    data = mujoco.MjData(model)
    data.ctrl[:] = actuator_targets_from_qpos(model, data.qpos)
    mujoco.mj_forward(model, data)
    for _ in range(smoke_steps):
        mujoco.mj_step(model, data)

    finite_state = all(math.isfinite(float(value)) for value in data.qpos) and all(
        math.isfinite(float(value)) for value in data.qvel
    )
    if not finite_state:
        raise RuntimeError("mobile dual-arm simulation produced a non-finite state")
    return {
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "nbody": model.nbody,
        "njnt": model.njnt,
        "neq": model.neq,
        "ngeom": model.ngeom,
        "actuators": actuators,
        "finite_state": finite_state,
        "smoke_steps": smoke_steps,
        "contacts_after_smoke": data.ncon,
        "max_abs_qvel": max((abs(float(value)) for value in data.qvel), default=0.0),
        "base_fixed": True,
        "wheel_actuators_present": False,
        "hardware_execution": False,
    }


def _run_viewer(model: mujoco.MjModel) -> None:
    import mujoco.viewer

    data = mujoco.MjData(model)
    data.ctrl[:] = actuator_targets_from_qpos(model, data.qpos)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate the stationary Waffle Pi plus dual-JDcobot200 scene."
    )
    parser.add_argument("--arm-mount-x-m", type=float, default=DEFAULT_ARM_MOUNT_X_M)
    parser.add_argument(
        "--arm-mount-separation-m",
        type=float,
        default=DEFAULT_ARM_MOUNT_SEPARATION_M,
    )
    parser.add_argument(
        "--arm-mount-height-m",
        type=float,
        default=DEFAULT_ARM_MOUNT_HEIGHT_M,
    )
    parser.add_argument("--left-yaw-rad", type=float, default=0.0)
    parser.add_argument("--right-yaw-rad", type=float, default=0.0)
    parser.add_argument("--smoke-steps", type=int, default=1000)
    parser.add_argument("--no-primitive-collisions", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args(argv)

    model = build_model(
        arm_mount_x_m=args.arm_mount_x_m,
        arm_mount_separation_m=args.arm_mount_separation_m,
        arm_mount_height_m=args.arm_mount_height_m,
        left_yaw_rad=args.left_yaw_rad,
        right_yaw_rad=args.right_yaw_rad,
        primitive_collisions=not args.no_primitive_collisions,
    )
    print(
        json.dumps(
            validate_model(model, smoke_steps=args.smoke_steps),
            ensure_ascii=False,
            indent=2,
        )
    )
    if args.viewer:
        _run_viewer(model)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
