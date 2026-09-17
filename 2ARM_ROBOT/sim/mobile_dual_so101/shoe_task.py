#!/usr/bin/env python3
"""State-based shoe task for stationary Waffle Pi plus dual SO-101.

This module is simulation-only. It has no ROS 2, serial, USB, or hardware
command path. MuJoCo world coordinates are treated as the provisional map
frame until the mobile-base simulation is introduced.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import json
import math
from typing import Sequence

import mujoco
import numpy as np

from collision_guard import (
    DEFAULT_CLEARANCE_M,
    CollisionAssessment,
    check_bimanual_path,
    minimum_protected_clearance,
    structural_near_support_status,
    general_support_clearance,
)
from mobile_dual_so101 import (
    ACTION_NAMES,
    ARM_CONTROL_NAMES,
    HUMANOID_HOME_ACTION,
    apply_control_as_pose,
    MOUNT_LAYOUTS,
    TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
    TOWER_RECOMMENDED_ARM_MOUNT_SEPARATION_M,
    actuator_targets_from_qpos,
    build_spec as build_mobile_spec,
)


SCHEMA_VERSION = "dapier.so101-dual-shoe-task.v0.1"
SHOE_BODY_NAME = "shoe"
SHOE_FREE_JOINT_NAME = "shoe_free"
SHOE_GEOM_NAMES = ("shoe_sole", "shoe_upper")
DEFAULT_SHOE_POSITION_M = (0.26, 0.0, 0.015)
DEFAULT_SHOE_YAW_RAD = 0.0
DEFAULT_ARM_MOUNT_HEIGHT_M = TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M
DEFAULT_ARM_MOUNT_SEPARATION_M = TOWER_RECOMMENDED_ARM_MOUNT_SEPARATION_M
DEFAULT_MOUNT_LAYOUT = "tower"
DEFAULT_FRAME_SKIP = 10
SUCCESS_HEIGHT_M = 0.09
SUCCESS_GRIPPER_DISTANCE_M = 0.14
SO101_REACH_ENVELOPE_M = 0.40
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
    # Legacy API names are retained; object_kind identifies the actual target.
    scene_id: str = "legacy_tower"
    grippers: str = "stock"
    object_kind: str = "legacy_shoe"
    shoe_position_m: tuple[float, float, float] = DEFAULT_SHOE_POSITION_M
    shoe_yaw_rad: float = DEFAULT_SHOE_YAW_RAD
    arm_mount_height_m: float = DEFAULT_ARM_MOUNT_HEIGHT_M
    arm_mount_separation_m: float = DEFAULT_ARM_MOUNT_SEPARATION_M
    mount_layout: str = DEFAULT_MOUNT_LAYOUT
    frame_skip: int = DEFAULT_FRAME_SKIP
    success_height_m: float = SUCCESS_HEIGHT_M
    success_gripper_distance_m: float = SUCCESS_GRIPPER_DISTANCE_M
    required_clearance_m: float = DEFAULT_CLEARANCE_M
    shoe_xy_range_m: float = 0.0
    shoe_yaw_range_rad: float = 0.0
    initial_home_pose: bool = False

    def validate(self) -> None:
        if self.grippers not in ("stock", "both"):
            raise ValueError("unknown gripper variant")
        if self.scene_id not in ("legacy_tower", "integration_desk"):
            raise ValueError("unknown task scene_id")
        if self.object_kind not in ("legacy_shoe", "block"):
            raise ValueError("object_kind must be legacy_shoe or block")
        for name in ("shoe_xy_range_m", "shoe_yaw_range_rad"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
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
        if self.mount_layout not in MOUNT_LAYOUTS:
            raise ValueError(
                f"mount_layout must be one of {MOUNT_LAYOUTS}, got {self.mount_layout!r}"
            )
        if self.frame_skip <= 0:
            raise ValueError("frame_skip must be positive")
        if self.success_height_m <= 0:
            raise ValueError("success_height_m must be positive")
        if self.success_gripper_distance_m <= 0:
            raise ValueError("success_gripper_distance_m must be positive")
        if not math.isfinite(self.required_clearance_m) or self.required_clearance_m <= 0:
            raise ValueError("required_clearance_m must be finite and positive")


# Current manipulation profile. Legacy defaults remain available for regressions.
MOBILE_BLOCK_CONFIG = ShoeTaskConfig(
    # One millimetre above the floor avoids a roundoff-sensitive exact contact.
    object_kind="block", shoe_position_m=(-.220, .220, .021),
    initial_home_pose=True,
    shoe_xy_range_m=.002, shoe_yaw_range_rad=math.radians(.5),
)


class UnsafeActionError(RuntimeError):
    """Raised before physics advances when a protected path is unsafe."""

    def __init__(self, assessment: CollisionAssessment) -> None:
        super().__init__(assessment.reason)
        self.assessment = assessment

    def __reduce__(self):
        return (type(self), (self.assessment,))


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


def _add_block(spec: mujoco.MjSpec, config: ShoeTaskConfig) -> None:
    """Reuse only measured block geometry/material; preserve the mobile scene."""
    profile = json.loads(Path(__file__).with_name("tabletop_replay.json").read_text())
    block = spec.worldbody.add_body(
        name=SHOE_BODY_NAME, pos=list(config.shoe_position_m),
        quat=list(_yaw_quaternion(config.shoe_yaw_rad)),
    )
    # Compatibility body/joint names; the physical object is a single block.
    block.add_freejoint(name=SHOE_FREE_JOINT_NAME)
    block.add_geom(
        name="block_geom", type=mujoco.mjtGeom.mjGEOM_BOX,
        size=np.asarray(profile["reference_block_size_m"]) / 2,
        mass=profile["block_mass_kg"], friction=profile["block_friction"],
        contype=1, conaffinity=1, condim=4,
        solref=profile["contact_solref"], solimp=[.95, .99, .001, .5, 2.],
        rgba=[.65, .03, .025, 1.],
    )


def build_shoe_task_model(config: ShoeTaskConfig | None = None) -> mujoco.MjModel:
    resolved = config or ShoeTaskConfig()
    resolved.validate()
    if resolved.scene_id == "integration_desk":
        from integration_scenes import build_scene
        return build_scene("desk", grippers=resolved.grippers).compile()
    spec, _ = build_mobile_spec(
        arm_mount_height_m=resolved.arm_mount_height_m,
        arm_mount_separation_m=resolved.arm_mount_separation_m,
        mount_layout=resolved.mount_layout, grippers=resolved.grippers,
    )
    if resolved.object_kind == "block":
        _add_block(spec, resolved)
    else:
        _add_primitive_shoe(
            spec, position_m=resolved.shoe_position_m, yaw_rad=resolved.shoe_yaw_rad,
        )
    return spec.compile()


def target_body_name(model):
    return "red_block" if model.names.startswith(b"desk_learning_OS30A_UNVERIFIED\x00") else SHOE_BODY_NAME


def target_joint_name(model):
    return "red_block_free" if target_body_name(model) == "red_block" else SHOE_FREE_JOINT_NAME


def target_geom_name(model):
    return "red_block_geom" if target_body_name(model) == "red_block" else "block_geom"


def block_finger_geoms(model, *, required=True):
    """Only active physical contact geoms; visual meshes cannot confirm grasp."""
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_pgripper_gear") >= 0:
        result = {f"jaw_{i}": model.geom(f"left_pgripper_pad_{i}").id for i in (1,2)}
    elif target_body_name(model) == "red_block":
        result = {side: model.geom("left_dapier_" + side + "_finger_pad").id
                  for side in ("fixed", "moving")}
    else:
        result = {}
        for g in range(model.ngeom):
            if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            for side, mesh in (("fixed", "left_wrist_roll_follower_so101_v1"),
                               ("moving", "left_moving_jaw_so101_v1")):
                if model.mesh(int(model.geom_dataid[g])).name == mesh and model.geom_contype[g]:
                    result[side] = g
    result = {name: g for name, g in result.items()
              if model.geom_contype[g] and model.geom_conaffinity[g]}
    if required and len(result) != 2:
        raise ValueError("missing active left finger collision geometry")
    return result


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

    shoe_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, target_body_name(model))
    base_id = (0 if target_body_name(model) == "red_block" else
               _object_id(model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link"))
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
        **({"target_object": "block", "block": {
                "position_map_m": shoe_position, "yaw_map_rad": shoe_yaw},
            "legacy_target_alias": "shoe"}
           if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, target_geom_name(model)) >= 0 else {}),
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
    shoe_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, target_body_name(model))
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


def _shoe_gripper_attachment_active(
    model: mujoco.MjModel, data: mujoco.MjData, *, gripper_only: bool = True
) -> bool:
    shoe_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, target_body_name(model))
    gripper_ids = tuple(
        _object_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_gripper")
        for side in ("left", "right")
    )
    attachment_types = {
        mujoco.mjtEq.mjEQ_CONNECT,
        mujoco.mjtEq.mjEQ_WELD,
    }
    for equality_id in range(model.neq):
        if (
            not data.eq_active[equality_id]
            or model.eq_type[equality_id] not in attachment_types
        ):
            continue
        first = int(model.eq_obj1id[equality_id])
        second = int(model.eq_obj2id[equality_id])
        object_type = model.eq_objtype[equality_id]
        if object_type == mujoco.mjtObj.mjOBJ_SITE:
            if not (0 <= first < model.nsite and 0 <= second < model.nsite):
                continue
            first = int(model.site_bodyid[first])
            second = int(model.site_bodyid[second])
        elif object_type != mujoco.mjtObj.mjOBJ_BODY:
            continue
        if first == shoe_id:
            other = second
        elif second == shoe_id:
            other = first
        else:
            continue
        if not gripper_only or any(
            _body_is_descendant(model, other, gripper_id)
            for gripper_id in gripper_ids
        ):
            return True
    return False


def task_metrics(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    success_height_m: float = SUCCESS_HEIGHT_M,
    success_gripper_distance_m: float = SUCCESS_GRIPPER_DISTANCE_M,
) -> dict[str, object]:
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, target_geom_name(model)) >= 0:
        return block_metrics(model, data)
    shoe_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, target_body_name(model))
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
    bilateral_contact = all(count > 0 for count in contact_counts.values())
    attachment_active = _shoe_gripper_attachment_active(model, data)
    carry_supported = bilateral_contact or attachment_active
    success = lifted and near_gripper and carry_supported
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
        "bilateral_gripper_contact": bilateral_contact,
        "shoe_gripper_attachment": attachment_active,
        "carry_supported": carry_supported,
        "reward": reward,
    }


def block_metrics(model: mujoco.MjModel, data: mujoco.MjData, *, reference_bottom_m: float = 0.0) -> dict[str, object]:
    """Block-only snapshot; final success requires continuous physics evidence."""
    block = model.geom(target_geom_name(model)).id
    block_body = int(model.geom_bodyid[block])
    fingers = set()
    finger_geoms = block_finger_geoms(model, required=False)
    forbidden_contact = False
    deepest = 0.0
    for i, contact in enumerate(data.contact):
        pair = (int(contact.geom1), int(contact.geom2))
        if block not in pair:
            continue
        other = pair[1] if pair[0] == block else pair[0]
        deepest = max(deepest, -float(contact.dist))
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        if force[0] <= 0:
            continue
        label = next((name for name, g in finger_geoms.items() if other == g), None)
        if label is not None:
            fingers.add(label)
        else:
            forbidden_contact = True
    rotation = data.geom_xmat[block].reshape(3, 3)
    bottom = float(data.geom_xpos[block, 2] - np.abs(rotation[2]) @ model.geom_size[block])
    attached = _shoe_gripper_attachment_active(model, data, gripper_only=False)
    supported = (len(finger_geoms) == 2 and fingers == set(finger_geoms) and not forbidden_contact
                 and not attached and int(model.body_mocapid[block_body]) < 0
                 and deepest <= .001)
    lifted = bottom >= max(0.0, reference_bottom_m) + .030
    return {"target_object": "block", "block_position_m": data.xpos[block_body].tolist(),
            "block_bottom_height_m": bottom, "lifted": lifted,
            "left_finger_contacts": sorted(fingers), "grasp_supported": supported,
            "forbidden_block_contact": forbidden_contact, "attachment_active": attached,
            "maximum_block_penetration_m": deepest, "lift_supported": lifted and supported,
            "success": False, "success_requires_continuous_hold_s": 3.0,
            "reward": max(0.0, bottom) + float(lifted and supported)}


def task_reachability(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    reach_envelope_m: float = SO101_REACH_ENVELOPE_M,
) -> dict[str, object]:
    if reach_envelope_m <= 0 or not math.isfinite(reach_envelope_m):
        raise ValueError("reach_envelope_m must be finite and positive")
    shoe_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, target_body_name(model))
    distances = {}
    for side in ("left", "right"):
        shoulder_id = _object_id(
            model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_shoulder"
        )
        distances[side] = math.dist(
            data.xpos[shoe_id], data.xpos[shoulder_id]
        )
    nearest = min(distances.values())
    return {
        "shoulder_distance_m": distances,
        "nearest_shoulder_distance_m": nearest,
        "reach_envelope_m": reach_envelope_m,
        "inside_distance_envelope": nearest < reach_envelope_m,
    }


# SIM integration grippers only: 2 ms HOME hold / SETTLE max 8.9633e-9 rad.
# This is measured-state numerical allowance, never a command or model limit.
INTEGRATION_GRIPPER_BOUNDARY_TOLERANCE_RAD = 1e-8


def measured_joint_state(model, data, *, integration_desk=False):
    """Validate physical joint state separately from position commands."""
    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
        raise ValueError("measured joint state must be finite")
    current = actuator_targets_from_qpos(model, data.qpos)
    evidence = []
    for a, value in enumerate(current):
        j = int(model.actuator_trnid[a, 0])
        if (model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT or j < 0
                or model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE
                or not np.array_equal(model.actuator_gear[a], [1., 0., 0., 0., 0., 0.])):
            raise ValueError("unsupported measured joint transmission")
        low, high = model.jnt_range[j]
        excess = max(0., float(low-value), float(value-high)) if model.jnt_limited[j] else 0.
        name = model.actuator(a).name
        tolerance = (INTEGRATION_GRIPPER_BOUNDARY_TOLERANCE_RAD
                     if integration_desk and name in ("left_gripper", "right_gripper") else 0.)
        if excess > tolerance:
            raise ValueError(f"measured joint {model.joint(j).name} exceeds joint range: {excess:.12g} > {tolerance:.12g}")
        if name in ("left_gripper", "right_gripper"):
            evidence.append(dict(time_s=float(data.time), actuator=name,
                raw_joint_qpos=float(value), mapped_measured_position=float(value),
                commanded_ctrl=float(data.ctrl[a]), ctrlrange=model.actuator_ctrlrange[a].tolist(),
                joint_range=model.jnt_range[j].tolist(),
                qvel=float(data.qvel[int(model.jnt_dofadr[j])]),
                tracking_error=float(value-data.ctrl[a]), boundary_excess_rad=excess,
                numerical_tolerance_rad=tolerance,
                status="boundary numerical overshoot" if excess else "within joint range",
                limit_constraints=[dict(position=float(data.efc_pos[k]), force=float(data.efc_force[k]))
                    for k in range(data.nefc)
                    if data.efc_type[k] == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT
                    and data.efc_id[k] == j]))
    return current, evidence


class ShoeTaskEnv:
    """Small Gym-style API without adding a Gym dependency."""

    def __init__(
        self,
        config: ShoeTaskConfig | None = None,
        *,
        model: mujoco.MjModel | None = None,
    ) -> None:
        self.config = config or ShoeTaskConfig()
        self.config.validate()
        self.model = model or build_shoe_task_model(self.config)
        if (self.config.scene_id == "integration_desk") != (target_body_name(self.model) == "red_block"):
            raise ValueError("scene_id does not match compiled scene")
        has_block = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, target_geom_name(self.model)) >= 0
        if has_block != (self.config.object_kind == "block"):
            raise ValueError("object_kind does not match the compiled target geometry")
        self.support_geom = self.model.geom(
            "table" if self.config.scene_id == "integration_desk" else "floor").id
        self.data = mujoco.MjData(self.model)
        self.collision_phase = None  # Explicit SIM teacher phase; no implicit exemption.
        self.physics_observer = None  # Optional read-only viewer hook; no alternate controller.
        self._block_hold_s = 0.0
        self.settle_info = None
        self._reset_valid = self.config.object_kind != "block"

    def reset(self, *, seed: int | None = None) -> tuple[dict[str, object], dict[str, object]]:
        self._reset_valid = False
        seed = 0 if seed is None else seed
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ValueError("seed must be a nonnegative integer")
        randomized = self.config.shoe_xy_range_m > 0 or self.config.shoe_yaw_range_rad > 0
        position = list(self.config.shoe_position_m)
        yaw = self.config.shoe_yaw_rad
        if randomized:
            rng = np.random.default_rng(seed)
            position[:2] = np.asarray(position[:2]) + rng.uniform(
                -self.config.shoe_xy_range_m, self.config.shoe_xy_range_m, size=2
            )
            yaw += float(rng.uniform(-self.config.shoe_yaw_range_rad, self.config.shoe_yaw_range_rad))
        mujoco.mj_resetData(self.model, self.data)
        self.settle_info = None
        if self.config.initial_home_pose:
            # SIM actuator joint angles (radians), mapped through transmission IDs.
            from pgripper import home_action
            apply_control_as_pose(self.model, self.data, home_action(self.model, HUMANOID_HOME_ACTION))
        self._block_hold_s = 0.0
        joint_id = _object_id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, target_joint_name(self.model)
        )
        qpos_address = int(self.model.jnt_qposadr[joint_id])
        self.data.qpos[qpos_address : qpos_address + 3] = position
        self.data.qpos[qpos_address + 3 : qpos_address + 7] = _yaw_quaternion(
            yaw
        )
        self.data.ctrl[:] = actuator_targets_from_qpos(self.model, self.data.qpos)
        mujoco.mj_forward(self.model, self.data)
        validation = {}
        if randomized or self.config.object_kind == "block":
            for joint in range(self.model.njnt):
                if self.model.jnt_limited[joint]:
                    value = self.data.qpos[self.model.jnt_qposadr[joint]]
                    lower, upper = self.model.jnt_range[joint]
                    if not lower <= value <= upper:
                        raise ValueError("randomized reset violates joint limits")
            action = actuator_targets_from_qpos(self.model, self.data.qpos)
            assessment = check_bimanual_path(
                self.model, action, action, required_clearance_m=self.config.required_clearance_m
            )
            if not assessment.safe:
                raise UnsafeActionError(assessment)
            reach = task_reachability(self.model, self.data)
            if reach["shoulder_distance_m"]["left"] >= reach["reach_envelope_m"]:
                raise ValueError("randomized reset outside left-arm reach envelope")
            shoe_geoms = tuple(g for g in range(self.model.ngeom)
                               if self.model.geom_bodyid[g] == self.model.body(target_body_name(self.model)).id)
            others = [g for g in range(self.model.ngeom)
                      if g not in shoe_geoms and self.model.geom_contype[g]
                      and self.model.geom_conaffinity[g]]
            pairs = [(shoe, other) for shoe in shoe_geoms for other in others
                     if self.model.geom_type[other] != mujoco.mjtGeom.mjGEOM_PLANE
                     and other != self.support_geom]
            clearance, _, _ = minimum_protected_clearance(self.model, self.data, pairs)
            floor_pairs = [(shoe, other) for shoe in shoe_geoms for other in others
                           if (self.model.geom_type[other] == mujoco.mjtGeom.mjGEOM_PLANE
                            or other == self.support_geom)]
            # An infinite plane has no enclosing sphere. Query its signed
            # native distance directly; the finite-geom sphere bound is invalid.
            floor_clearance = min(
                (float(mujoco.mj_geomDistance(self.model, self.data, a, b, 2., None))
                 for a, b in floor_pairs), default=math.inf,
            )
            if clearance < self.config.required_clearance_m or floor_clearance < 0:
                raise ValueError(
                    f"randomized reset violates target collision clearance: seed={seed}, "
                    f"target={clearance:.17g} m, floor={floor_clearance:.17g} m"
                )
            validation = {"collision_guard": assessment.as_report(), "reachability": reach,
                          "target_clearance_m": clearance,
                          "floor_nonpenetrating": floor_clearance >= 0, "joint_limits": True}
            if self.config.object_kind == "block":
                arm_floor = [
                    (float(mujoco.mj_geomDistance(self.model, self.data, arm, floor, 2., None)), arm, floor)
                    for arm in others
                    if self.model.body(int(self.model.geom_bodyid[arm])).name.startswith(("left_", "right_"))
                    for floor in others
                    if self.model.geom_type[floor] == mujoco.mjtGeom.mjGEOM_PLANE
                ]
                if arm_floor:
                    distance, arm, floor = min(arm_floor)
                    if distance < self.config.required_clearance_m:
                        raise ValueError(
                            f"mobile block reset violates arm-floor clearance: seed={seed}, "
                            f"pair=({arm},{floor}), distance={distance:.17g} m, "
                            f"required={self.config.required_clearance_m} m; initial robot pose unchanged"
                        )
                    validation["arm_floor_clearance_m"] = distance
        self._reset_valid = True
        return ground_truth_observation(self.model, self.data), {
            "hardware_execution": False,
            "scene_id": self.config.scene_id,
            "randomized": randomized,
            "seed": seed,
            "target_object": self.config.object_kind,
            "initial_joint_pose": "HUMANOID_HOME_ACTION" if self.config.initial_home_pose else "model_default",
            "initial_conditions": {"target_position_m": [float(v) for v in position],
                                   "target_yaw_rad": yaw},
            "randomization": {"distribution": "uniform", "generator": "numpy.PCG64",
                              "target_xy_range_m": self.config.shoe_xy_range_m,
                              "target_yaw_range_rad": self.config.shoe_yaw_range_rad,
                              "center_position_m": self.config.shoe_position_m,
                              "center_yaw_rad": self.config.shoe_yaw_rad},
            "validation": validation,
        }

    def step(
        self, action: Sequence[float]
    ) -> tuple[dict[str, object], float, bool, bool, dict[str, object]]:
        clipped, assessment = self.apply_action(
            action,
            physics_steps=self.config.frame_skip,
        )
        metrics = self.metrics()
        observation = ground_truth_observation(self.model, self.data)
        return observation, float(metrics["reward"]), bool(metrics["success"]), False, {
            **metrics,
            "action_clipped": clipped,
            "collision_guard": assessment.as_report(),
            "hardware_execution": False,
        }

    def metrics(self) -> dict[str, object]:
        metrics = task_metrics(
            self.model,
            self.data,
            success_height_m=self.config.success_height_m,
            success_gripper_distance_m=self.config.success_gripper_distance_m,
        )
        if self.config.object_kind == "block":
            metrics = block_metrics(self.model, self.data, reference_bottom_m=(
                self.settle_info["block_bottom_height_m"] if self.settle_info else 0.0))
            metrics["continuous_hold_s"] = self._block_hold_s
            metrics["success"] = bool(metrics["lift_supported"] and self._block_hold_s >= 3.0)
        return metrics

    def settle(self) -> dict[str, object]:
        """Explicit SIM RESET -> SETTLE -> PREGRASP gate; reset itself stays at t=0."""
        if self.config.object_kind != "block" or not self._reset_valid:
            raise ValueError("settle requires a validated block reset")
        hold = tuple(self.data.ctrl)
        block = self.model.geom(target_geom_name(self.model)).id
        floor = self.support_geom
        dof = int(self.model.joint(target_joint_name(self.model)).dofadr[0])
        arms = [g for g in range(self.model.ngeom)
                if self.model.geom_contype[g] and self.model.geom_conaffinity[g]
                and self.model.body(int(self.model.geom_bodyid[g])).name.startswith(("left_", "right_"))]
        stable_steps = 0
        self.settle_info = None
        for _ in range(100):
            self.apply_action(hold, physics_steps=1)
            mujoco.mj_forward(self.model, self.data)
            floor_gap = general_support_clearance(self.model, self.data, arms, floor)
            block_gap = minimum_protected_clearance(
                self.model, self.data, [(arm, block) for arm in arms])[0]
            if min(floor_gap, block_gap) < self.config.required_clearance_m:
                self._reset_valid = False
                raise ValueError("settling violates arm-floor/block clearance")
            contact = any({int(c.geom1), int(c.geom2)} == {block, floor}
                          for c in self.data.contact)
            slow = np.max(np.abs(self.data.qvel[dof:dof + 6])) < 1e-4
            stable_steps = stable_steps + 1 if contact and slow else 0
        if stable_steps < 10:
            self._reset_valid = False
            raise ValueError("block did not settle: require 10 consecutive contact/low-velocity steps")
        self.settle_info = {
            "phase": "PREGRASP", "physics_steps": 100, "time_s": float(self.data.time),
            "block_bottom_height_m": block_metrics(self.model, self.data)["block_bottom_height_m"],
            "stable_steps": stable_steps, "velocity_threshold": 1e-4,
            "arm_floor_clearance_m": floor_gap, "arm_block_clearance_m": block_gap,
            "hardware_execution": False,
        }
        return dict(self.settle_info)

    def apply_action(
        self,
        action: Sequence[float],
        *,
        physics_steps: int,
    ) -> tuple[tuple[float, ...], CollisionAssessment]:
        """Guard and execute one simulator action for an explicit step count."""

        if self.config.object_kind == "block" and not self._reset_valid:
            raise ValueError("block physics requires a successfully validated reset")
        if len(action) != len(ACTION_NAMES):
            raise ValueError(
                f"expected {len(ACTION_NAMES)} actions, received {len(action)}"
            )
        if physics_steps <= 0:
            raise ValueError("physics_steps must be positive")
        clipped = []
        for actuator_id, raw_value in enumerate(action):
            value = float(raw_value)
            if not math.isfinite(value):
                raise ValueError("actions must be finite")
            lower, upper = self.model.actuator_ctrlrange[actuator_id]
            if not float(lower) <= value <= float(upper):
                raise ValueError(
                    f"action {ACTION_NAMES[actuator_id]} is outside actuator range"
                )
            clipped.append(value)
        bounded = tuple(clipped)
        if not np.all(np.isfinite(self.data.ctrl)) or np.any(
            (self.data.ctrl < self.model.actuator_ctrlrange[:, 0]) |
            (self.data.ctrl > self.model.actuator_ctrlrange[:, 1])):
            raise ValueError("existing ctrl is outside actuator range")
        current, evidence = measured_joint_state(
            self.model, self.data, integration_desk=self.config.scene_id == "integration_desk")
        assessment = check_bimanual_path(
            self.model,
            current,
            bounded,
            required_clearance_m=self.config.required_clearance_m,
            task_phase=self.collision_phase, reference_data=self.data,
        )
        if not assessment.safe:
            raise UnsafeActionError(assessment)
        self.data.ctrl[:] = bounded
        for _ in range(physics_steps):
            mujoco.mj_step(self.model, self.data)
            if self.config.scene_id == "integration_desk":
                # Inspect the integrated qpos, not mj_step's pre-integration FK/contact cache.
                mujoco.mj_forward(self.model, self.data)
            _, evidence = measured_joint_state(
                self.model, self.data, integration_desk=self.config.scene_id == "integration_desk")
            if hasattr(self, "measured_state_trace"):
                self.measured_state_trace.extend(evidence)
            if any(not row["safe"] for row in structural_near_support_status(self.model, self.data)):
                self._reset_valid = False
                raise RuntimeError("measured structural near-support invariant violated")
            if self.config.object_kind == "block":
                evidence = block_metrics(self.model, self.data, reference_bottom_m=(
                    self.settle_info["block_bottom_height_m"] if self.settle_info else 0.0))
                self._block_hold_s = (self._block_hold_s + float(self.model.opt.timestep)
                                      if evidence["lift_supported"] else 0.0)
            if self.collision_phase is not None:
                from collision_guard import task_clearance_status
                if not task_clearance_status(self.model,self.data,self.collision_phase)["safe"]:
                    self._reset_valid = False
                    raise ValueError("measured task-specific/general clearance invariant violated")
            if self.physics_observer is not None:
                self.physics_observer()
        return bounded, assessment


def validate_shoe_task(smoke_steps: int = 200, *, config: ShoeTaskConfig | None = None) -> dict[str, object]:
    if smoke_steps < 0:
        raise ValueError("smoke_steps must be non-negative")
    env = ShoeTaskEnv(config)
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
    reachability = task_reachability(env.model, env.data)
    return {
        "schema_version": SCHEMA_VERSION,
        "nq": env.model.nq,
        "nv": env.model.nv,
        "nu": env.model.nu,
        "njnt": env.model.njnt,
        "shoe_body_present": True,
        "mount_layout": env.config.mount_layout,
        "observation_dimension": len(observation["vector"]),
        "ground_truth": observation["ground_truth"],
        "finite_observation": finite,
        "state_imitation_contract_ready": finite,
        "default_floor_shoe_reachable": reachability[
            "inside_distance_envelope"
        ],
        "reachability": reachability,
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
