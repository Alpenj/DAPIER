#!/usr/bin/env python3
"""Fail-closed simulator guard for dual SO-101 target motions.

This module has no serial, ROS publisher, or hardware-dispatch path. A real
controller may consume the assessment only after measured collision geometry,
joint calibration, state freshness, braking distance, and E-stop behavior are
validated separately.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import json
from itertools import combinations
import math
from pathlib import Path
import sys
from typing import Sequence

import mujoco
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR))

from mobile_dual_so101 import (  # noqa: E402
    ACTION_NAMES,
    HUMANOID_HOME_ACTION,
    apply_control_as_pose,
    build_model,
)


DEFAULT_CLEARANCE_M = 0.030
DEFAULT_MAX_JOINT_STEP_RAD = math.radians(2.0)
REQUIRED_ARM_COLLISION_GEOM_COUNTS = {
    "shoulder": 3,
    "upper_arm": 2,
    "lower_arm": 3,
    "wrist": 2,
    "gripper": 2,
    "moving_jaw_so101_v1": 1,
}
REQUIRED_BASE_COLLISION_GEOM_NAMES = (
    "tb3_base_link_collision",
    "tb3_wheel_left_link_collision",
    "tb3_wheel_right_link_collision",
    "tb3_caster_back_right_link_collision",
    "tb3_caster_back_left_link_collision",
)
REQUIRED_RGBD_COLLISION_GEOM_NAMES = (
    "workspace_depth_camera_collision",
    "front_slam_depth_camera_collision",
)
REQUIRED_FLOOR_COLLISION_GEOM_NAMES = ("floor",)
REQUIRED_TOWER_SUPPORT_COLLISION_GEOM_NAMES = (
    "semi_support_base_collision",
    "semi_support_column_collision",
    "tower_camera_mast_collision",
    "tower_camera_interface_plate_collision",
)
REQUIRED_PRINTED_MOUNT_COLLISION_GEOM_NAMES = (
    "printed_mount_deck_collision",
    "printed_torso_collision",
    "printed_camera_mount_collision",
)
REQUIRED_COMPACT_SUPPORT_COLLISION_GEOM_NAMES = (
    "compact_mount_plate",
    "compact_camera_mast",
)


@dataclass(frozen=True)
class CollisionAssessment:
    safe: bool
    reason: str
    minimum_clearance_m: float
    required_clearance_m: float
    path_fraction: float
    checked_samples: int
    first_body: str
    second_body: str
    first_geom_id: int
    second_geom_id: int
    published: bool = False
    control_authorized: bool = False
    hardware_dispatch_authorized: bool = False
    executed_action: bool = False
    hardware_execution: bool = False

    def as_report(self) -> dict[str, object]:
        return asdict(self)


def _body_name_for_geom(model: mujoco.MjModel, geom_id: int) -> str:
    body_id = int(model.geom_bodyid[geom_id])
    return (
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        or f"body_{body_id}"
    )


def _collision_geoms_for_arm(
    model: mujoco.MjModel, side: str
) -> tuple[int, ...]:
    prefix = f"{side}_"
    return tuple(
        geom_id
        for geom_id in range(model.ngeom)
        if int(model.geom_contype[geom_id]) != 0
        and int(model.geom_conaffinity[geom_id]) != 0
        and _body_name_for_geom(model, geom_id).startswith(prefix)
    )


def _collision_geoms_for_body_prefixes(
    model: mujoco.MjModel, prefixes: Sequence[str]
) -> tuple[int, ...]:
    return tuple(
        geom_id
        for geom_id in range(model.ngeom)
        if int(model.geom_contype[geom_id]) != 0
        and _body_name_for_geom(model, geom_id).startswith(tuple(prefixes))
    )


def _geom_id_if_present(model: mujoco.MjModel, name: str) -> int | None:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    return int(geom_id) if geom_id >= 0 else None


def _required_active_geom_id(model: mujoco.MjModel, name: str) -> int:
    geom_id = _geom_id_if_present(model, name)
    if (
        geom_id is None
        or int(model.geom_contype[geom_id]) == 0
        or int(model.geom_conaffinity[geom_id]) == 0
    ):
        raise RuntimeError(f"collision geometry is missing or inactive: {name}")
    return geom_id


def _required_collision_geoms_for_arm(
    model: mujoco.MjModel, side: str
) -> tuple[int, ...]:
    geoms = _collision_geoms_for_arm(model, side)
    observed = Counter(_body_name_for_geom(model, geom_id) for geom_id in geoms)
    required = REQUIRED_ARM_COLLISION_GEOM_COUNTS
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_pgripper_gear") >= 0:
        required = {**required, "gripper": 1, "pgripper_jaw_1": 1, "pgripper_jaw_2": 1}
        del required["moving_jaw_so101_v1"]
        for suffix in ("pgripper_housing", "pgripper_pad_1", "pgripper_pad_2"):
            _required_active_geom_id(model, f"{side}_{suffix}")
    for suffix, expected_count in required.items():
        body_name = f"{side}_{suffix}"
        if observed[body_name] < expected_count:
            raise RuntimeError(
                "collision geometry is missing or inactive: "
                f"{body_name} expected {expected_count}, found {observed[body_name]}"
            )
    return geoms


def bimanual_geom_pairs(model: mujoco.MjModel) -> tuple[tuple[int, int], ...]:
    """Return every left collision geom against every right collision geom."""

    left = _required_collision_geoms_for_arm(model, "left")
    right = _required_collision_geoms_for_arm(model, "right")
    return tuple((left_id, right_id) for left_id in left for right_id in right)


def _same_arm_geom_pairs(
    model: mujoco.MjModel, geoms: Sequence[int]
) -> tuple[tuple[int, int], ...]:
    def body_distance(first: int, second: int) -> int:
        first_ancestors = {}
        distance = 0
        while first > 0:
            first_ancestors[first] = distance
            first = int(model.body_parentid[first])
            distance += 1
        distance = 0
        while second not in first_ancestors:
            second = int(model.body_parentid[second])
            distance += 1
        return first_ancestors[second] + distance

    pairs = []
    for first, second in combinations(geoms, 2):
        first_body = int(model.geom_bodyid[first])
        second_body = int(model.geom_bodyid[second])
        if body_distance(first_body, second_body) <= 1:
            continue
        pairs.append((first, second))
    return tuple(pairs)


def protected_geom_pairs(model: mujoco.MjModel) -> tuple[tuple[int, int], ...]:
    """Return fail-closed pairs for arms, camera, base, and tower structure."""

    left = _required_collision_geoms_for_arm(model, "left")
    right = _required_collision_geoms_for_arm(model, "right")
    all_arm_geoms = (*left, *right)
    pairs = list(bimanual_geom_pairs(model))
    pairs.extend(_same_arm_geom_pairs(model, left))
    pairs.extend(_same_arm_geom_pairs(model, right))

    camera_ids = tuple(
        _required_active_geom_id(model, name)
        for name in REQUIRED_RGBD_COLLISION_GEOM_NAMES
    )
    pairs.extend(
        (arm_id, camera_id)
        for arm_id in all_arm_geoms
        for camera_id in camera_ids
    )

    base_geoms = tuple(
        _required_active_geom_id(model, name)
        for name in REQUIRED_BASE_COLLISION_GEOM_NAMES
    )
    pairs.extend(
        (arm_id, base_id)
        for arm_id in all_arm_geoms
        for base_id in base_geoms
    )

    floor_geoms = tuple(
        _required_active_geom_id(model, name)
        for name in REQUIRED_FLOOR_COLLISION_GEOM_NAMES
    )
    pairs.extend(
        (arm_id, floor_id)
        for arm_id in all_arm_geoms
        for floor_id in floor_geoms
    )

    tower_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "tower_mount_structure"
    )
    printed_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "printed_mount_structure"
    )
    if tower_body_id >= 0:
        support_names = REQUIRED_TOWER_SUPPORT_COLLISION_GEOM_NAMES
    elif printed_body_id >= 0:
        support_names = REQUIRED_PRINTED_MOUNT_COLLISION_GEOM_NAMES
    elif all(
        _geom_id_if_present(model, name) is not None
        for name in REQUIRED_COMPACT_SUPPORT_COLLISION_GEOM_NAMES
    ):
        support_names = REQUIRED_COMPACT_SUPPORT_COLLISION_GEOM_NAMES
    else:
        raise RuntimeError("mount collision structure is missing")
    support_geoms = tuple(
        _required_active_geom_id(model, name) for name in support_names
    )
    for arm_id in all_arm_geoms:
        body_name = _body_name_for_geom(model, arm_id)
        for support_id in support_geoms:
            support_name = (
                mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_GEOM, support_id
                )
                or ""
            )
            # Both shoulders bolt to side interfaces on the central STEP
            # support. Their overlap with the conservative solid column proxy
            # is intentional; every downstream moving link stays protected.
            own_support_interface = (
                body_name in {"left_shoulder", "right_shoulder"}
                and support_name
                in {
                    "semi_support_column_collision",
                    "printed_torso_collision",
                    "compact_mount_plate",
                }
            )
            if not own_support_interface:
                pairs.append((arm_id, support_id))

    return tuple(dict.fromkeys(pairs))


def minimum_protected_clearance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pairs: Sequence[tuple[int, int]] | None = None,
    *,
    distance_cap_m: float = 2.0,
) -> tuple[float, int, int]:
    """Measure the closest protected geom pair at the current pose."""

    resolved_pairs = protected_geom_pairs(model) if pairs is None else pairs
    if not resolved_pairs:
        raise ValueError("at least one protected geom pair is required")
    if not math.isfinite(distance_cap_m) or distance_cap_m <= 0:
        raise ValueError("distance_cap_m must be finite and positive")
    from_to = np.zeros(6, dtype=float)
    best_distance = math.inf
    best_pair = (-1, -1)
    for first, second in resolved_pairs:
        narrowphase_distance = float(
            mujoco.mj_geomDistance(
                model,
                data,
                int(first),
                int(second),
                distance_cap_m,
                from_to,
            )
        )
        # MuJoCo's mesh GJK can conservatively return exactly zero at isolated
        # far-separated poses. A bounding-sphere lower bound is rigorous:
        # when the spheres do not overlap, the enclosed meshes cannot touch.
        center_distance = float(
            np.linalg.norm(data.geom_xpos[int(first)] - data.geom_xpos[int(second)])
        )
        sphere_lower_bound = center_distance - float(
            model.geom_rbound[int(first)] + model.geom_rbound[int(second)]
        )
        distance = max(narrowphase_distance, sphere_lower_bound)
        if distance < best_distance:
            best_distance = distance
            best_pair = (int(first), int(second))
    return best_distance, best_pair[0], best_pair[1]


def check_bimanual_path(
    model: mujoco.MjModel,
    current_action: Sequence[float],
    target_action: Sequence[float],
    *,
    required_clearance_m: float = DEFAULT_CLEARANCE_M,
    max_joint_step_rad: float = DEFAULT_MAX_JOINT_STEP_RAD,
    obstacle_geom_names: Sequence[str] = (),
) -> CollisionAssessment:
    """Reject a target if its interpolated path violates protected clearance."""

    if len(current_action) != len(ACTION_NAMES):
        raise ValueError(f"expected {len(ACTION_NAMES)} current targets")
    if len(target_action) != len(ACTION_NAMES):
        raise ValueError(f"expected {len(ACTION_NAMES)} target targets")
    if not math.isfinite(required_clearance_m) or required_clearance_m <= 0:
        raise ValueError("required_clearance_m must be finite and positive")
    if not math.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0:
        raise ValueError("max_joint_step_rad must be finite and positive")

    current = np.asarray(current_action, dtype=float)
    target = np.asarray(target_action, dtype=float)
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(target)):
        raise ValueError("all action values must be finite")
    max_delta = float(np.max(np.abs(target - current)))
    intervals = max(1, math.ceil(max_delta / max_joint_step_rad))
    pairs = list(protected_geom_pairs(model))
    obstacle_ids = tuple(
        _required_active_geom_id(model, name) for name in obstacle_geom_names
    )
    arm_geoms = (
        *_collision_geoms_for_arm(model, "left"),
        *_collision_geoms_for_arm(model, "right"),
    )
    pairs.extend(
        (arm_id, obstacle_id)
        for arm_id in arm_geoms
        for obstacle_id in obstacle_ids
    )
    same_arm_pairs = (
        *_same_arm_geom_pairs(model, _collision_geoms_for_arm(model, "left")),
        *_same_arm_geom_pairs(model, _collision_geoms_for_arm(model, "right")),
    )
    same_arm_pair_set = set(same_arm_pairs)
    clearance_pairs = tuple(
        dict.fromkeys(pair for pair in pairs if pair not in same_arm_pair_set)
    )
    data = mujoco.MjData(model)
    best = (math.inf, -1, -1, 0.0)

    for sample_index in range(intervals + 1):
        fraction = sample_index / intervals
        action = current + fraction * (target - current)
        apply_control_as_pose(model, data, action)
        self_distance, self_first, self_second = minimum_protected_clearance(
            model,
            data,
            same_arm_pairs,
            distance_cap_m=required_clearance_m,
        )
        if self_distance <= 0.0:
            return CollisionAssessment(
                safe=False,
                reason="same-arm collision detected; target rejected",
                minimum_clearance_m=self_distance,
                required_clearance_m=0.0,
                path_fraction=fraction,
                checked_samples=sample_index + 1,
                first_body=_body_name_for_geom(model, self_first),
                second_body=_body_name_for_geom(model, self_second),
                first_geom_id=self_first,
                second_geom_id=self_second,
            )
        distance, first, second = minimum_protected_clearance(
            model,
            data,
            clearance_pairs,
            distance_cap_m=required_clearance_m,
        )
        if distance < best[0]:
            best = (distance, first, second, fraction)
        if distance < required_clearance_m:
            return CollisionAssessment(
                safe=False,
                reason="protected clearance violated; target rejected",
                minimum_clearance_m=distance,
                required_clearance_m=required_clearance_m,
                path_fraction=fraction,
                checked_samples=sample_index + 1,
                first_body=_body_name_for_geom(model, first),
                second_body=_body_name_for_geom(model, second),
                first_geom_id=first,
                second_geom_id=second,
            )

    distance, first, second, fraction = best
    return CollisionAssessment(
        safe=True,
        reason="interpolated simulator path satisfies protected clearance",
        minimum_clearance_m=distance,
        required_clearance_m=required_clearance_m,
        path_fraction=fraction,
        checked_samples=intervals + 1,
        first_body=_body_name_for_geom(model, first),
        second_body=_body_name_for_geom(model, second),
        first_geom_id=first,
        second_geom_id=second,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Check a simulator-only dual-SO-101 target path."
    )
    parser.add_argument("--arm-mount-height-m", type=float, default=0.30)
    parser.add_argument(
        "--target-action-rad",
        type=float,
        nargs=len(ACTION_NAMES),
        default=HUMANOID_HOME_ACTION,
    )
    parser.add_argument("--clearance-m", type=float, default=DEFAULT_CLEARANCE_M)
    parser.add_argument(
        "--max-joint-step-deg",
        type=float,
        default=math.degrees(DEFAULT_MAX_JOINT_STEP_RAD),
    )
    args = parser.parse_args(argv)
    model, _ = build_model(arm_mount_height_m=args.arm_mount_height_m)
    assessment = check_bimanual_path(
        model,
        HUMANOID_HOME_ACTION,
        args.target_action_rad,
        required_clearance_m=args.clearance_m,
        max_joint_step_rad=math.radians(args.max_joint_step_deg),
    )
    print(json.dumps(assessment.as_report(), indent=2))
    return 0 if assessment.safe else 2


if __name__ == "__main__":
    raise SystemExit(main())
