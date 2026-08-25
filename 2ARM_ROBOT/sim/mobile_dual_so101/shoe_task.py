#!/usr/bin/env python3
"""State-based shoe task for stationary Waffle Pi plus dual SO-101.

This module is simulation-only. It has no ROS 2, serial, USB, or hardware
command path. MuJoCo world coordinates are treated as the provisional map
frame until the mobile-base simulation is introduced.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from typing import Sequence

import mujoco

from mobile_dual_so101 import (
    ACTION_NAMES,
    ARM_CONTROL_NAMES,
    actuator_targets_from_qpos,
    build_spec as build_mobile_spec,
)


SCHEMA_VERSION = "dapier.so101-dual-shoe-task.v0.1"
SHOE_BODY_NAME = "shoe"
SHOE_FREE_JOINT_NAME = "shoe_free"
SHOE_GEOM_NAMES = ("shoe_sole", "shoe_upper")
DEFAULT_SHOE_POSITION_M = (0.26, 0.0, 0.015)
DEFAULT_SHOE_YAW_RAD = 0.0
DEFAULT_ARM_MOUNT_HEIGHT_M = 0.30
DEFAULT_ARM_MOUNT_SEPARATION_M = 0.18
DEFAULT_FRAME_SKIP = 10
SUCCESS_HEIGHT_M = 0.09
SUCCESS_GRIPPER_DISTANCE_M = 0.14
ARM_JOINT_NAMES = ARM_CONTROL_NAMES[:-1]
OBSERVATION_NAMES = (
    "shoe_x_map_m",
    "shoe_y_map_m",
    "shoe_z_map_m",
    "shoe_yaw_map_rad",
    "base_x_map_m",
    "base_y_map_m",
    "base_yaw_map_rad",
    *(f"left_{name}_rad" for name in ARM_JOINT_NAMES),
    "left_gripper_normalized",
    *(f"right_{name}_rad" for name in ARM_JOINT_NAMES),
    "right_gripper_normalized",
    "base_linear_velocity_mps",
    "base_angular_velocity_radps",
)


@dataclass(frozen=True)
class ShoeTaskConfig:
    shoe_position_m: tuple[float, float, float] = DEFAULT_SHOE_POSITION_M
    shoe_yaw_rad: float = DEFAULT_SHOE_YAW_RAD
    arm_mount_height_m: float = DEFAULT_ARM_MOUNT_HEIGHT_M
    arm_mount_separation_m: float = DEFAULT_ARM_MOUNT_SEPARATION_M
    frame_skip: int = DEFAULT_FRAME_SKIP
    success_height_m: float = SUCCESS_HEIGHT_M
    success_gripper_distance_m: float = SUCCESS_GRIPPER_DISTANCE_M

    def validate(self) -> None:
        if len(self.shoe_position_m) != 3 or not all(
            math.isfinite(float(value)) for value in self.shoe_position_m
        ):
            raise ValueError("shoe_position_m must contain three finite values")
        if not math.isfinite(self.shoe_yaw_rad):
            raise ValueError("shoe_yaw_rad must be finite")
        if self.arm_mount_height_m <= 0:
            raise ValueError("arm_mount_height_m must be positive")
        if self.arm_mount_separation_m <= 0:
            raise ValueError("arm_mount_separation_m must be positive")
        if self.frame_skip <= 0:
            raise ValueError("frame_skip must be positive")
        if self.success_height_m <= 0:
            raise ValueError("success_height_m must be positive")
        if self.success_gripper_distance_m <= 0:
            raise ValueError("success_gripper_distance_m must be positive")


def _yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    half = yaw_rad / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _quaternion_yaw(quaternion: Sequence[float]) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _add_primitive_shoe(
    spec: mujoco.MjSpec,
    *,
    position_m: Sequence[float],
    yaw_rad: float,
) -> None:
    shoe = spec.worldbody.add_body(
        name=SHOE_BODY_NAME,
        pos=list(position_m),
        quat=list(_yaw_quaternion(yaw_rad)),
    )
    shoe.add_freejoint(name=SHOE_FREE_JOINT_NAME)
    shoe.add_geom(
        name="shoe_sole",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.13, 0.05, 0.012],
        density=350.0,
        friction=[0.9, 0.02, 0.002],
        condim=4,
        contype=3,
        conaffinity=3,
        rgba=[0.08, 0.08, 0.10, 1.0],
    )
    shoe.add_geom(
        name="shoe_upper",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[-0.015, 0.0, 0.037],
        size=[0.095, 0.045, 0.025],
        density=90.0,
        friction=[0.9, 0.02, 0.002],
        condim=4,
        contype=3,
        conaffinity=3,
        rgba=[0.18, 0.32, 0.75, 1.0],
    )


def build_shoe_task_model(config: ShoeTaskConfig | None = None) -> mujoco.MjModel:
    resolved = config or ShoeTaskConfig()
    resolved.validate()
    spec, _ = build_mobile_spec(
        arm_mount_height_m=resolved.arm_mount_height_m,
        arm_mount_separation_m=resolved.arm_mount_separation_m,
    )
    _add_primitive_shoe(
        spec,
        position_m=resolved.shoe_position_m,
        yaw_rad=resolved.shoe_yaw_rad,
    )
    return spec.compile()


def _object_id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise RuntimeError(f"MuJoCo object is missing: {name}")
    return object_id


def _actuator_joint_position(
    model: mujoco.MjModel, data: mujoco.MjData, actuator_name: str
) -> float:
    actuator_id = _object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
    joint_id = int(model.actuator_trnid[actuator_id, 0])
    return float(data.qpos[int(model.jnt_qposadr[joint_id])])


def _normalize_gripper(
    model: mujoco.MjModel, side: str, position_rad: float
) -> float:
    actuator_id = _object_id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{side}_gripper"
    )
    minimum, maximum = (
        float(value) for value in model.actuator_ctrlrange[actuator_id]
    )
    return min(1.0, max(0.0, (position_rad - minimum) / (maximum - minimum)))


def ground_truth_observation(
    model: mujoco.MjModel, data: mujoco.MjData
) -> dict[str, object]:
    """Return simulator truth without performing perception or estimation."""

    shoe_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, SHOE_BODY_NAME)
    base_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link")
    shoe_position = tuple(float(value) for value in data.xpos[shoe_id])
    shoe_yaw = _quaternion_yaw(data.xquat[shoe_id])
    base_position = tuple(float(value) for value in data.xpos[base_id])
    base_yaw = _quaternion_yaw(data.xquat[base_id])
    joint_positions = {
        side: tuple(
            _actuator_joint_position(model, data, f"{side}_{name}")
            for name in ARM_JOINT_NAMES
        )
        for side in ("left", "right")
    }
    grippers = {
        side: _normalize_gripper(
            model,
            side,
            _actuator_joint_position(model, data, f"{side}_gripper"),
        )
        for side in ("left", "right")
    }
    vector = (
        *shoe_position,
        shoe_yaw,
        base_position[0],
        base_position[1],
        base_yaw,
        *joint_positions["left"],
        grippers["left"],
        *joint_positions["right"],
        grippers["right"],
        0.0,
        0.0,
    )
    if len(vector) != len(OBSERVATION_NAMES):
        raise RuntimeError("observation vector and name contract differ")
    return {
        "schema_version": SCHEMA_VERSION,
        "frame": "map_sim_world",
        "ground_truth": True,
        "shoe": {
            "position_map_m": shoe_position,
            "yaw_map_rad": shoe_yaw,
        },
        "robot": {
            "base_pose_map": (base_position[0], base_position[1], base_yaw),
            "left_arm_rad": joint_positions["left"],
            "left_gripper_normalized": grippers["left"],
            "right_arm_rad": joint_positions["right"],
            "right_gripper_normalized": grippers["right"],
            "base_velocity": (0.0, 0.0),
        },
        "vector_names": OBSERVATION_NAMES,
        "vector": vector,
    }


def _body_is_descendant(model: mujoco.MjModel, body_id: int, ancestor_id: int) -> bool:
    current = body_id
    while current > 0:
        if current == ancestor_id:
            return True
        current = int(model.body_parentid[current])
    return current == ancestor_id


def _shoe_gripper_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, int]:
    shoe_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, SHOE_BODY_NAME)
    gripper_ids = {
        side: _object_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_gripper")
        for side in ("left", "right")
    }
    counts = {"left": 0, "right": 0}
    for index in range(data.ncon):
        contact = data.contact[index]
        body1 = int(model.geom_bodyid[int(contact.geom1)])
        body2 = int(model.geom_bodyid[int(contact.geom2)])
        if body1 == shoe_id:
            other = body2
        elif body2 == shoe_id:
            other = body1
        else:
            continue
        for side, gripper_id in gripper_ids.items():
            if _body_is_descendant(model, other, gripper_id):
                counts[side] += 1
    return counts


def task_metrics(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    success_height_m: float = SUCCESS_HEIGHT_M,
    success_gripper_distance_m: float = SUCCESS_GRIPPER_DISTANCE_M,
) -> dict[str, object]:
    shoe_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, SHOE_BODY_NAME)
    shoe_position = data.xpos[shoe_id]
    distances = {}
    for side in ("left", "right"):
        site_id = _object_id(
            model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_gripperframe"
        )
        distances[side] = math.dist(data.site_xpos[site_id], shoe_position)
    nearest_distance = min(distances.values())
    lifted = float(shoe_position[2]) >= success_height_m
    near_gripper = nearest_distance <= success_gripper_distance_m
    contact_counts = _shoe_gripper_contacts(model, data)
    success = lifted and near_gripper
    lift_progress = min(1.0, max(0.0, float(shoe_position[2]) / success_height_m))
    approach_reward = -min(nearest_distance, 1.0)
    contact_bonus = 0.25 * min(2, max(contact_counts.values()))
    reward = approach_reward + lift_progress + contact_bonus + (10.0 if success else 0.0)
    return {
        "success": success,
        "lifted": lifted,
        "near_gripper": near_gripper,
        "shoe_height_m": float(shoe_position[2]),
        "gripper_distance_m": distances,
        "nearest_gripper_distance_m": nearest_distance,
        "gripper_contact_count": contact_counts,
        "reward": reward,
    }


class ShoeTaskEnv:
    """Small Gym-style API without adding a Gym dependency."""

    def __init__(self, config: ShoeTaskConfig | None = None) -> None:
        self.config = config or ShoeTaskConfig()
        self.config.validate()
        self.model = build_shoe_task_model(self.config)
        self.data = mujoco.MjData(self.model)

    def reset(self, *, seed: int | None = None) -> tuple[dict[str, object], dict[str, object]]:
        del seed  # Deterministic in phase 1; randomization is a later pipeline stage.
        mujoco.mj_resetData(self.model, self.data)
        joint_id = _object_id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, SHOE_FREE_JOINT_NAME
        )
        qpos_address = int(self.model.jnt_qposadr[joint_id])
        self.data.qpos[qpos_address : qpos_address + 3] = self.config.shoe_position_m
        self.data.qpos[qpos_address + 3 : qpos_address + 7] = _yaw_quaternion(
            self.config.shoe_yaw_rad
        )
        self.data.ctrl[:] = actuator_targets_from_qpos(self.model, self.data.qpos)
        mujoco.mj_forward(self.model, self.data)
        return ground_truth_observation(self.model, self.data), {
            "hardware_execution": False,
            "randomized": False,
        }

    def step(
        self, action: Sequence[float]
    ) -> tuple[dict[str, object], float, bool, bool, dict[str, object]]:
        if len(action) != len(ACTION_NAMES):
            raise ValueError(
                f"expected {len(ACTION_NAMES)} actions, received {len(action)}"
            )
        clipped = []
        for actuator_id, raw_value in enumerate(action):
            value = float(raw_value)
            if not math.isfinite(value):
                raise ValueError("actions must be finite")
            lower, upper = self.model.actuator_ctrlrange[actuator_id]
            clipped.append(min(float(upper), max(float(lower), value)))
        self.data.ctrl[:] = clipped
        for _ in range(self.config.frame_skip):
            mujoco.mj_step(self.model, self.data)
        metrics = task_metrics(
            self.model,
            self.data,
            success_height_m=self.config.success_height_m,
            success_gripper_distance_m=self.config.success_gripper_distance_m,
        )
        observation = ground_truth_observation(self.model, self.data)
        return observation, float(metrics["reward"]), bool(metrics["success"]), False, {
            **metrics,
            "action_clipped": tuple(clipped),
            "hardware_execution": False,
        }


def validate_shoe_task(smoke_steps: int = 200) -> dict[str, object]:
    if smoke_steps < 0:
        raise ValueError("smoke_steps must be non-negative")
    env = ShoeTaskEnv()
    observation, reset_info = env.reset(seed=0)
    hold = actuator_targets_from_qpos(env.model, env.data.qpos)
    terminated = False
    metrics = task_metrics(env.model, env.data)
    for _ in range(smoke_steps):
        observation, _, terminated, _, metrics = env.step(hold)
        if terminated:
            break
    finite = all(math.isfinite(float(value)) for value in observation["vector"])
    if not finite:
        raise RuntimeError("shoe task produced a non-finite observation")
    return {
        "schema_version": SCHEMA_VERSION,
        "nq": env.model.nq,
        "nv": env.model.nv,
        "nu": env.model.nu,
        "njnt": env.model.njnt,
        "shoe_body_present": True,
        "observation_dimension": len(observation["vector"]),
        "ground_truth": observation["ground_truth"],
        "finite_observation": finite,
        "terminated": terminated,
        "smoke_steps": smoke_steps,
        "hardware_execution": reset_info["hardware_execution"],
        "last_metrics": metrics,
    }


def _run_viewer(env: ShoeTaskEnv) -> None:
    import mujoco.viewer

    hold = actuator_targets_from_qpos(env.model, env.data.qpos)
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        while viewer.is_running():
            env.step(hold)
            viewer.sync()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the simulation-only DAPIER state-based shoe task."
    )
    parser.add_argument("--smoke-steps", type=int, default=200)
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args(argv)
    report = validate_shoe_task(smoke_steps=args.smoke_steps)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.viewer:
        env = ShoeTaskEnv()
        env.reset(seed=0)
        _run_viewer(env)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
