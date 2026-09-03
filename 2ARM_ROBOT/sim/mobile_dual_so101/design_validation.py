#!/usr/bin/env python3
"""Deterministic first-pass stability and clearance study for the concept."""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
import sys
from typing import Iterable, Sequence

import mujoco
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR))

from collision_guard import (  # noqa: E402
    DEFAULT_CLEARANCE_M,
    minimum_protected_clearance,
    protected_geom_pairs,
)
from mobile_dual_so101 import (  # noqa: E402
    DEFAULT_MOUNT_LAYOUT,
    HUMANOID_HOME_ACTION,
    MOUNT_LAYOUTS,
    PRINTED_MOUNT_ESTIMATED_MASS_KG,
    TOWER_CAMERA_DOWN_TILT_RAD,
    TOWER_MOUNT_ESTIMATED_MASS_KG,
    WORKSPACE_DEPTH_CAMERA_DOWN_TILT_RAD,
    WORKSPACE_DEPTH_CAMERA_HORIZONTAL_FOV_DEG,
    WORKSPACE_DEPTH_CAMERA_MASS_KG,
    WORKSPACE_DEPTH_CAMERA_VERTICAL_FOV_DEG,
    apply_control_as_pose,
    build_model,
)


GRAVITY_M_S2 = 9.81
ORIGINAL_SUPPORT_POLYGON_XY_M = np.asarray(
    (
        (-0.192, -0.064),
        (0.000, -0.153),
        (0.000, 0.153),
        (-0.192, 0.064),
    ),
    dtype=float,
)
NOMINAL_PAYLOAD_KG = 0.5
PROOF_PAYLOAD_KG = 1.0
DYNAMIC_FACTOR = 2.5
RANDOM_SEED = 20260825


def _body_descendants(model: mujoco.MjModel, root_id: int) -> tuple[int, ...]:
    result = []
    for body_id in range(model.nbody):
        current = body_id
        while current != 0 and current != root_id:
            current = int(model.body_parentid[current])
        if current == root_id:
            result.append(body_id)
    return tuple(result)


def _weighted_com(
    positions: np.ndarray,
    masses: np.ndarray,
) -> tuple[float, np.ndarray]:
    total = float(np.sum(masses))
    if total <= 0:
        raise RuntimeError("mass must be positive")
    return total, np.sum(positions * masses[:, None], axis=0) / total


def _combined_com(
    base_mass: float,
    base_com: np.ndarray,
    payloads: Sequence[tuple[float, np.ndarray]],
) -> tuple[float, np.ndarray]:
    total = base_mass + sum(mass for mass, _ in payloads)
    moment = base_mass * base_com
    for mass, position in payloads:
        moment = moment + mass * position
    return total, moment / total


def _signed_polygon_margin(
    point_xy: np.ndarray, polygon_xy: np.ndarray
) -> float:
    """Return minimum inward edge distance for a counter-clockwise polygon."""

    margins = []
    for start, end in zip(polygon_xy, np.roll(polygon_xy, -1, axis=0)):
        edge = end - start
        relative = point_xy - start
        cross = edge[0] * relative[1] - edge[1] * relative[0]
        margins.append(float(cross / np.linalg.norm(edge)))
    return min(margins)


def _stability_metrics(com: np.ndarray) -> tuple[float, float]:
    margin = _signed_polygon_margin(
        np.asarray(com[:2]), ORIGINAL_SUPPORT_POLYGON_XY_M
    )
    critical_acceleration = GRAVITY_M_S2 * margin / max(float(com[2]), 1e-9)
    return margin, critical_acceleration


def _endpoint_actions(
    lower: np.ndarray, upper: np.ndarray
) -> Iterable[np.ndarray]:
    for bits in itertools.product((0, 1), repeat=len(lower)):
        yield np.where(np.asarray(bits, dtype=bool), upper, lower)


def _sample_actions(
    lower: np.ndarray,
    upper: np.ndarray,
    random_samples: int,
) -> Iterable[tuple[str, np.ndarray]]:
    home = np.asarray(HUMANOID_HOME_ACTION, dtype=float)
    yield "home", home
    for action in _endpoint_actions(lower, upper):
        yield "all_4096_endpoint_corners", action

    rng = np.random.default_rng(RANDOM_SEED)
    for _ in range(random_samples):
        left_only = home.copy()
        left_only[:6] = rng.uniform(lower[:6], upper[:6])
        yield "left_only_random", left_only

        right_only = home.copy()
        right_only[6:] = rng.uniform(lower[6:], upper[6:])
        yield "right_only_random", right_only

        yield "both_random", rng.uniform(lower, upper)


def validate_design(
    *,
    arm_mount_height_m: float = 0.30,
    mount_layout: str = DEFAULT_MOUNT_LAYOUT,
    random_samples: int = 10000,
    collision_samples: int = 2000,
) -> dict[str, object]:
    if random_samples < 0 or collision_samples < 0:
        raise ValueError("sample counts must be non-negative")
    model, _ = build_model(
        arm_mount_height_m=arm_mount_height_m,
        mount_layout=mount_layout,
    )
    data = mujoco.MjData(model)
    lower = model.actuator_ctrlrange[:, 0].copy()
    upper = model.actuator_ctrlrange[:, 1].copy()
    body_ids = np.arange(1, model.nbody)
    body_masses = model.body_mass[body_ids].copy()
    left_root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_base")
    right_root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_base")
    arm_bodies = {
        "left": _body_descendants(model, left_root),
        "right": _body_descendants(model, right_root),
    }
    gripper_sites = {
        side: mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_gripperframe"
        )
        for side in ("left", "right")
    }
    mount_positions = {
        "left": model.body_pos[left_root].copy(),
        "right": model.body_pos[right_root].copy(),
    }

    scenarios = {
        "no_payload": (),
        "left_0p5kg": (("left", NOMINAL_PAYLOAD_KG),),
        "right_0p5kg": (("right", NOMINAL_PAYLOAD_KG),),
        "both_0p5kg": (
            ("left", NOMINAL_PAYLOAD_KG),
            ("right", NOMINAL_PAYLOAD_KG),
        ),
        "left_1p0kg_proof": (("left", PROOF_PAYLOAD_KG),),
        "right_1p0kg_proof": (("right", PROOF_PAYLOAD_KG),),
        "both_1p0kg_proof": (
            ("left", PROOF_PAYLOAD_KG),
            ("right", PROOF_PAYLOAD_KG),
        ),
    }
    worst = {
        name: {
            "minimum_margin_m": math.inf,
            "minimum_critical_linear_acceleration_m_s2": math.inf,
            "source": "",
            "action_rad": [],
        }
        for name in scenarios
    }
    maximum_mount_moment = {
        "left_static_with_1kg_Nm": 0.0,
        "right_static_with_1kg_Nm": 0.0,
    }
    source_counts: dict[str, int] = {}
    total_samples = 0

    for source, action in _sample_actions(lower, upper, random_samples):
        apply_control_as_pose(model, data, action)
        total_samples += 1
        source_counts[source] = source_counts.get(source, 0) + 1
        base_mass, base_com = _weighted_com(data.xipos[body_ids], body_masses)
        grippers = {
            side: data.site_xpos[site_id].copy()
            for side, site_id in gripper_sites.items()
        }
        for name, payload_spec in scenarios.items():
            payloads = tuple(
                (mass, grippers[side]) for side, mass in payload_spec
            )
            _, combined = _combined_com(base_mass, base_com, payloads)
            margin, critical = _stability_metrics(combined)
            if margin < worst[name]["minimum_margin_m"]:
                worst[name] = {
                    "minimum_margin_m": margin,
                    "minimum_critical_linear_acceleration_m_s2": critical,
                    "source": source,
                    "action_rad": action.tolist(),
                    "combined_com_m": combined.tolist(),
                }

        for side in ("left", "right"):
            ids = np.asarray(arm_bodies[side], dtype=int)
            arm_mass, arm_com = _weighted_com(data.xipos[ids], model.body_mass[ids])
            gravity_moment = (
                arm_mass
                * GRAVITY_M_S2
                * float(np.linalg.norm((arm_com - mount_positions[side])[:2]))
            )
            payload_moment = (
                PROOF_PAYLOAD_KG
                * GRAVITY_M_S2
                * float(
                    np.linalg.norm(
                        (grippers[side] - mount_positions[side])[:2]
                    )
                )
            )
            key = f"{side}_static_with_1kg_Nm"
            maximum_mount_moment[key] = max(
                maximum_mount_moment[key], gravity_moment + payload_moment
            )

    # Collision distance is much more expensive than COM evaluation. Check a
    # deterministic simultaneous-motion subset and retain the fail count.
    rng = np.random.default_rng(RANDOM_SEED + 1)
    pairs = protected_geom_pairs(model)
    collision_minimum = math.inf
    collision_action: list[float] = []
    below_clearance = 0
    for _ in range(collision_samples):
        action = rng.uniform(lower, upper)
        apply_control_as_pose(model, data, action)
        clearance, _, _ = minimum_protected_clearance(model, data, pairs)
        if clearance < collision_minimum:
            collision_minimum = clearance
            collision_action = action.tolist()
        if clearance < DEFAULT_CLEARANCE_M:
            below_clearance += 1

    camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, "workspace_depth_camera"
    )
    camera_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "workspace_depth_camera_body"
    )
    apply_control_as_pose(model, data, HUMANOID_HOME_ACTION)
    home_mass, home_com = _weighted_com(data.xipos[body_ids], body_masses)
    del home_mass
    original_home_margin = _signed_polygon_margin(
        home_com[:2], ORIGINAL_SUPPORT_POLYGON_XY_M
    )
    view_direction = -data.cam_xmat[camera_id].reshape(3, 3)[:, 2]
    camera_pitch_deg = math.degrees(
        math.atan2(-float(view_direction[2]), float(view_direction[0]))
    )
    mount_dynamic = {
        key.replace("_static_", "_dynamic_"): value * DYNAMIC_FACTOR
        for key, value in maximum_mount_moment.items()
    }

    return {
        "analysis_scope": {
            "continuous_space_exhaustive": False,
            "endpoint_corners_exhaustive": 4096,
            "random_samples_each_of_left_right_both": random_samples,
            "total_stability_samples": total_samples,
            "source_counts": source_counts,
            "collision_samples_both_random": collision_samples,
            "seed": RANDOM_SEED,
            "mount_layout": mount_layout,
        },
        "assumptions": {
            "original_support_polygon_xy_m": ORIGINAL_SUPPORT_POLYGON_XY_M.tolist(),
            "mount_mass_kg": (
                TOWER_MOUNT_ESTIMATED_MASS_KG
                if mount_layout == "tower"
                else PRINTED_MOUNT_ESTIMATED_MASS_KG
            ),
            "workspace_depth_camera_mass_kg": WORKSPACE_DEPTH_CAMERA_MASS_KG,
            "dynamic_factor": DYNAMIC_FACTOR,
            "unmodeled": [
                "actual print anisotropy and creep",
                "bolt preload and hole bearing",
                "joint acceleration, backlash and braking distance",
                "cable forces",
                "wheel slip and floor unevenness",
            ],
        },
        "model_total_mass_kg": float(mujoco.mj_getTotalmass(model)),
        "home_support_comparison": {
            "center_of_mass_m": home_com.tolist(),
            "original_waffle_margin_m": original_home_margin,
        },
        "workspace_depth_camera": {
            "center_m": model.body_pos[camera_body_id].tolist(),
            "configured_down_tilt_deg": math.degrees(
                TOWER_CAMERA_DOWN_TILT_RAD
                if mount_layout == "tower"
                else WORKSPACE_DEPTH_CAMERA_DOWN_TILT_RAD
            ),
            "measured_model_down_tilt_deg": camera_pitch_deg,
            "horizontal_fov_deg": WORKSPACE_DEPTH_CAMERA_HORIZONTAL_FOV_DEG,
            "vertical_fov_deg": WORKSPACE_DEPTH_CAMERA_VERTICAL_FOV_DEG,
        },
        "worst_stability": worst,
        "maximum_mount_moment": {
            **maximum_mount_moment,
            **mount_dynamic,
        },
        "collision_sampling": {
            "required_clearance_m": DEFAULT_CLEARANCE_M,
            "minimum_clearance_m": (
                collision_minimum if collision_samples else None
            ),
            "samples_below_clearance": below_clearance,
            "worst_action_rad": collision_action,
            "full_joint_ranges_are_collision_free": (
                below_clearance == 0 if collision_samples else None
            ),
        },
        "published": False,
        "control_authorized": False,
        "hardware_dispatch_authorized": False,
        "executed_action": False,
        "hardware_execution": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-mount-height-m", type=float, default=0.30)
    parser.add_argument(
        "--mount-layout",
        choices=MOUNT_LAYOUTS,
        default=DEFAULT_MOUNT_LAYOUT,
    )
    parser.add_argument("--random-samples", type=int, default=10000)
    parser.add_argument("--collision-samples", type=int, default=2000)
    args = parser.parse_args(argv)
    print(
        json.dumps(
            validate_design(
                arm_mount_height_m=args.arm_mount_height_m,
                mount_layout=args.mount_layout,
                random_samples=args.random_samples,
                collision_samples=args.collision_samples,
            ),
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
