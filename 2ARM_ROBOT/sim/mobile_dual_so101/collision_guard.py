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

    if model.names.startswith(b"desk_learning_OS30A_UNVERIFIED\x00"):
        # Exact approved desk builder. No tower cameras or support substitutions.
        names = ("table", "stand_cad_bottom_1", "stand_cad_top_1",
                 "stand_cad_top_3", "os30a_enclosure_UNVERIFIED")
        obstacles = tuple(_required_active_geom_id(model, name) for name in names)
        for side in ("left", "right"):
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_pgripper_gear") < 0:
                for finger in ("fixed", "moving"):
                    _required_active_geom_id(model, f"{side}_dapier_{finger}_finger_pad")
        pairs.extend((arm, obstacle) for arm in all_arm_geoms for obstacle in obstacles)
        return tuple(dict.fromkeys(pairs))

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


def certified_separation_lower_bound(
    model: mujoco.MjModel, data: mujoco.MjData, first: int, second: int
) -> float:
    """Conservative hull separation on nine unit axes; not an exact distance."""
    if any(model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH for g in (first, second)):
        return 0.0
    rotations = [data.geom_xmat[g].reshape(3, 3) for g in (first, second)]
    axes = np.vstack((np.eye(3), rotations[0].T, rotations[1].T))
    norms = np.linalg.norm(axes, axis=1)
    if not np.all(np.isfinite(norms)) or np.any(norms == 0):
        return 0.0
    axes = axes / norms[:, None]
    intervals = []
    for g, rotation in zip((first, second), rotations):
        mesh = int(model.geom_dataid[g])
        start = int(model.mesh_vertadr[mesh])
        vertices = model.mesh_vert[start:start + int(model.mesh_vertnum[mesh])]
        graph_start = int(model.mesh_graphadr[mesh])
        # Native CCD uses all vertices for meshes smaller than 10 vertices,
        # even when maxhullvert produced a smaller graph (MuJoCo 3.3.7).
        if graph_start >= 0 and len(vertices) >= 10:
            graph = model.mesh_graph[graph_start:]
            count = int(graph[0])
            vertices = vertices[graph[2 + count:2 + 2 * count]]
        # Compiled vertices already include asset scale/recentering.
        world = vertices @ rotation.T + data.geom_xpos[g]
        projection = world @ axes.T
        if not np.all(np.isfinite(projection)):
            return 0.0
        intervals.append((projection.min(axis=0), projection.max(axis=0)))
    a, b = intervals
    gap = float(np.max(np.maximum(a[0] - b[1], b[0] - a[1])))
    # Subtract roundoff allowance so numerical touching cannot certify separation.
    magnitude = max(1.0, *(float(np.max(np.abs(x))) for bounds in intervals for x in bounds))
    return max(0.0, gap - 64 * np.finfo(float).eps * magnitude)


def certified_mesh_box_separation_lower_bound(
    model: mujoco.MjModel, data: mujoco.MjData, first: int, second: int
) -> float:
    """Positive separation on the exact box face normals; never exact distance.

    Use ALL compiled mesh vertices (a superset of the collision hull) and the
    box's analytic support radius. Incomplete separating axes can miss a gap,
    but cannot certify overlapping/touching convex sets as separated.
    """
    mesh_geom, box = first, second
    if model.geom_type[mesh_geom] == mujoco.mjtGeom.mjGEOM_BOX:
        mesh_geom, box = box, mesh_geom
    if (model.geom_type[mesh_geom] != mujoco.mjtGeom.mjGEOM_MESH
            or model.geom_type[box] != mujoco.mjtGeom.mjGEOM_BOX):
        return 0.0
    mesh = int(model.geom_dataid[mesh_geom])
    start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
    vertices = model.mesh_vert[start:start+count].astype(float)
    if not len(vertices):
        return 0.0
    box_rotation = data.geom_xmat[box].reshape(3, 3)
    axes = box_rotation.T.copy()
    norms = np.linalg.norm(axes, axis=1)
    if not np.all(np.isfinite(norms)) or np.any(norms == 0):
        return 0.0
    axes /= norms[:, None]
    world = (vertices @ data.geom_xmat[mesh_geom].reshape(3, 3).T
             + data.geom_xpos[mesh_geom])
    projection = (world - data.geom_xpos[box]) @ axes.T
    radius = np.abs(axes @ box_rotation) @ model.geom_size[box]
    if not np.all(np.isfinite(projection)) or not np.all(np.isfinite(radius)):
        return 0.0
    gap = float(np.max(np.maximum(projection.min(axis=0) - radius,
                                 -radius - projection.max(axis=0))))
    magnitude = max(1., float(np.max(np.abs(world))),
                    float(np.max(np.abs(data.geom_xpos[box]))),
                    float(np.max(radius)))
    return float(max(0., gap - 64 * np.finfo(float).eps * magnitude))


def certified_box_box_separation_lower_bound(model, data, first, second) -> float:
    """OBB SAT certified lower bound on 15 axes, not Euclidean distance."""
    if any(model.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX for g in (first, second)):
        return 0.0
    rotations = [data.geom_xmat[g].reshape(3, 3) for g in (first, second)]
    centers = np.array([data.geom_xpos[g] for g in (first, second)])
    sizes = np.array([model.geom_size[g] for g in (first, second)])
    if (not all(np.all(np.isfinite(r)) for r in rotations)
            or not np.all(np.isfinite(centers)) or not np.all(np.isfinite(sizes))
            or np.any(sizes <= 0)):
        return 0.0
    axes = np.vstack((rotations[0].T, rotations[1].T,
                      np.cross(rotations[0].T[:, None, :],
                               rotations[1].T[None, :, :]).reshape(9, 3)))
    norms = np.linalg.norm(axes, axis=1)
    axes = axes[norms > 1e-12] / norms[norms > 1e-12, None]
    if not len(axes):
        return 0.0
    radii = [np.abs(axes @ r) @ s for r, s in zip(rotations, sizes)]
    gap = float(np.max(np.abs(axes @ (centers[0] - centers[1])) - radii[0] - radii[1]))
    magnitude = max(1., float(np.max(np.abs(centers))), float(np.max(sizes)))
    return float(max(0., gap - 64 * np.finfo(float).eps * magnitude))


def minimum_protected_clearance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pairs: Sequence[tuple[int, int]] | None = None,
    *,
    distance_cap_m: float = 2.0,
    diagnostics: list[dict[str, object]] | None = None,
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
        # An infinite plane has no finite enclosing sphere (rbound is zero).
        # In particular, lateral displacement must not hide touching/penetration.
        if any(model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE
               for g in (first, second)):
            sphere_lower_bound = -math.inf
        distance = max(narrowphase_distance, sphere_lower_bound)
        certificate = 0.0
        mesh_box_certificate = 0.0
        box_box_certificate = 0.0
        if narrowphase_distance <= 0.0:
            certificate = certified_separation_lower_bound(model, data, int(first), int(second))
            if certificate > 0.0:
                distance = max(distance, certificate)
            mesh_box_certificate = certified_mesh_box_separation_lower_bound(
                model, data, int(first), int(second))
            if mesh_box_certificate > 0.0:
                distance = max(distance, mesh_box_certificate)
            box_box_certificate = certified_box_box_separation_lower_bound(
                model, data, int(first), int(second))
            if box_box_certificate > 0.0:
                distance = max(distance, box_box_certificate)
        if diagnostics is not None:
            diagnostics.append(dict(
                pair=[int(first), int(second)], distance_cap_m=distance_cap_m,
                native_signed_distance_m=narrowphase_distance,
                final_distance_m=distance,
                certified_box_box_separation_lower_bound_m=box_box_certificate,
                box_box_certificate_eligible=narrowphase_distance <= 0 and all(
                    model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX for g in (first, second)),
                bounding_sphere_eligible=math.isfinite(sphere_lower_bound),
                bounding_sphere_lower_bound_m=(sphere_lower_bound
                    if math.isfinite(sphere_lower_bound) else None),
                bounding_sphere_changed_result=sphere_lower_bound > narrowphase_distance,
                mesh_certificate_eligible=narrowphase_distance <= 0 and all(
                    model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH for g in (first, second)),
                certified_separation_lower_bound_m=certificate,
                mesh_certificate_changed_result=certificate > max(narrowphase_distance, sphere_lower_bound)
                    and certificate > 0,
                mesh_box_certificate_eligible=narrowphase_distance <= 0 and
                    {int(model.geom_type[g]) for g in (first, second)} ==
                    {int(mujoco.mjtGeom.mjGEOM_MESH), int(mujoco.mjtGeom.mjGEOM_BOX)},
                certified_mesh_box_separation_lower_bound_m=mesh_box_certificate,
                mesh_box_certificate_changed_result=mesh_box_certificate > max(
                    narrowphase_distance, sphere_lower_bound, certificate) and mesh_box_certificate > 0))
        if distance < best_distance:
            best_distance = distance
            best_pair = (int(first), int(second))
    return best_distance, best_pair[0], best_pair[1]


NEAR_SUPPORT_SCOPE = "SIM_ONLY / INTEGRATION_DESK / HARDWARE_UNVERIFIED"


def _near_support_geometry_hash(model, pair):
    """Pin just these structural geometries/ancestors, never the runtime qpos."""
    import hashlib
    digest = hashlib.sha256()
    for g in pair:
        digest.update((model.geom(g).name or "<unnamed>").encode())
        for field in ("geom_type", "geom_size", "geom_pos", "geom_quat",
                      "geom_contype", "geom_conaffinity", "geom_margin", "geom_gap"):
            digest.update(np.asarray(getattr(model, field)[g]).tobytes())
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = int(model.geom_dataid[g])
            digest.update(model.mesh(mesh).name.encode())
            start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
            digest.update(model.mesh_vert[start:start+count].tobytes())
            graph = int(model.mesh_graphadr[mesh])
            if graph >= 0:
                n = int(model.mesh_graph[graph])
                digest.update(model.mesh_graph[graph:graph+2+2*n].tobytes())
        b = int(model.geom_bodyid[g])
        while b:
            digest.update(model.body(b).name.encode())
            digest.update(model.body_pos[b].tobytes())
            digest.update(model.body_quat[b].tobytes())
            for j in range(int(model.body_jntadr[b]), int(model.body_jntadr[b]+model.body_jntnum[b])):
                digest.update(model.joint(j).name.encode())
                for field in ("jnt_type", "jnt_limited", "jnt_pos", "jnt_axis", "jnt_range"):
                    digest.update(np.asarray(getattr(model, field)[j]).tobytes())
            b = int(model.body_parentid[b])
    return digest.hexdigest()


def structural_near_support_pairs(model):
    """Exact desk pairs remain protected; unknown compiled geometry fails closed."""
    if not model.names.startswith(b"desk_learning_OS30A_UNVERIFIED\x00"):
        return ()
    profile = json.loads(Path(__file__).with_name("integration_desk_near_support.json").read_text())
    pairs = []
    for side in ("left", "right"):
        mesh_name = side + "_rotation_pitch_so101_v1"
        matches = [g for g in range(model.ngeom)
                   if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                   and model.mesh(int(model.geom_dataid[g])).name == mesh_name
                   and _body_name_for_geom(model, g) == side + "_shoulder"
                   and model.geom_contype[g] and model.geom_conaffinity[g]]
        if len(matches) != 1:
            raise RuntimeError("structural near-support geom identity changed")
        pair = (matches[0], _required_active_geom_id(model, "table"))
        if _near_support_geometry_hash(model, pair) != profile["pairs"][side]["compiled_geometry_sha256"]:
            raise RuntimeError("structural near-support compiled geometry changed; re-audit required")
        pairs.append(pair)
    return tuple(pairs)


def structural_near_support_status(model, data):
    pairs = structural_near_support_pairs(model)
    if not pairs:
        return []
    profile = json.loads(Path(__file__).with_name("integration_desk_near_support.json").read_text())
    result = []
    for side, pair in zip(("left", "right"), pairs):
        details = []
        gap, _, _ = minimum_protected_clearance(model, data, [pair], diagnostics=details)
        # Independently positive exact-box projection is required, even when native >0.
        evidence = certified_mesh_box_separation_lower_bound(model, data, *pair)
        contacts = [float(c.dist) for c in data.contact
                    if {int(c.geom1), int(c.geom2)} == set(pair)]
        nominal = profile["pairs"][side]["nominal_clearance_m"]
        numerical_match = abs(evidence - nominal) <= profile["numerical_regression_tolerance_m"]
        safe = (math.isfinite(gap) and gap > 0 and math.isfinite(evidence)
                and evidence > 0 and not contacts and numerical_match
                and math.isfinite(details[0]["native_signed_distance_m"]))
        result.append(dict(side=side, pair=list(pair), classification="STRUCTURAL NEAR-SUPPORT",
            scope=NEAR_SUPPORT_SCOPE, safe=bool(safe), clearance_m=gap,
            positive_separation_evidence_m=evidence, nominal_compiled_clearance_m=nominal,
            contacts=contacts, penetration=any(x < 0 for x in contacts),
            compiled_geometry_verified=True, nominal_geometry_matches=numerical_match,
            general_30mm_applies=False,
            reason="approved exact mechanical relationship; positive separation/contact/geometry invariant"))
    return result


def general_support_clearance(model, data, arms, support):
    """Keep structural checks, then measure all remaining arm/support pairs."""
    status = structural_near_support_status(model, data)
    if any(not row["safe"] for row in status):
        raise RuntimeError("structural near-support invariant violated")
    structural = {tuple(row["pair"]) for row in status}
    pairs = [(g, support) for g in arms if (g, support) not in structural]
    if not pairs:
        raise RuntimeError("no general arm/support pairs remain")
    return minimum_protected_clearance(model, data, pairs)[0]


MANIPULATION_PHASES = frozenset(("PREGRASP_NEAR", "APPROACH_COARSE", "APPROACH_FINE",
    "CLOSE", "GRASP_CONFIRM", "LIFT_5MM", "LIFT_15MM", "LIFT_30MM", "HOLD"))
FINGER_CONTACT_PHASES = frozenset(("CLOSE", "GRASP_CONFIRM", "LIFT_5MM",
    "LIFT_15MM", "LIFT_30MM", "HOLD"))


def manipulation_pair_status(model, data, phase):
    """Exact left-hand center-block relations; never hardware policy."""
    if phase not in MANIPULATION_PHASES:
        return []
    if not bytes(model.names).startswith(b"desk_learning_OS30A_UNVERIFIED\x00"):
        raise ValueError("manipulation policy requires integration_desk")
    profile = json.loads((Path(__file__).with_name("integration_manipulation_pairs.json")).read_text())
    rows = []
    # Near approach permits proven separation, not contact. Only pads may contact
    # the target during closing/carrying; unrelated pairs still use 30 mm.
    for entry in profile["pairs"]:
        a, b = (model.geom(name).id for name in entry["names"])
        if _near_support_geometry_hash(model, (a,b)) != entry["compiled_geometry_sha256"]:
            raise ValueError("manipulation geometry changed; re-audit required")
        details = []
        gap = minimum_protected_clearance(model, data, [(a,b)], diagnostics=details)[0]
        contacts = [float(c.dist) for c in data.contact if {int(c.geom1),int(c.geom2)} == {a,b}]
        margin = float(64*np.finfo(float).eps*max(1.,np.max(np.abs(data.geom_xpos[[a,b]]))))
        intended = entry["class"] == "INTENDED_FINGER_CONTACT" and phase in FINGER_CONTACT_PHASES
        safe = (math.isfinite(gap) and math.isfinite(details[0]["native_signed_distance_m"])
                and all(math.isfinite(x) for x in contacts))
        safe = safe and ((gap >= -.001 and all(x >= -.001 for x in contacts)) if intended
                         else (gap > margin and not contacts))
        rows.append(dict(pair=[a,b],names=entry["names"],classification=(
            entry["class"] if intended or entry["class"] != "INTENDED_FINGER_CONTACT"
            else "TARGET_NEAR_APPROACH"),phase=phase,scope=NEAR_SUPPORT_SCOPE,
            clearance_m=gap,numerical_margin_m=margin,contacts=contacts,
            intended_contact=intended,safe=bool(safe),compiled_geometry_verified=True,
            penetration_limit_m=.001 if intended else 0.))
    return rows


def task_clearance_status(model, data, phase, required=.03):
    """Runtime counterpart of the segment gate, using measured geometry."""
    rows = manipulation_pair_status(model,data,phase)
    specialized = {tuple(r["pair"]) for r in rows}
    structural = set(structural_near_support_pairs(model))
    arms = (*_collision_geoms_for_arm(model,"left"), *_collision_geoms_for_arm(model,"right"))
    target = model.geom("red_block_geom").id
    table = model.geom("table").id
    general = [(g,table) for g in arms if (g,table) not in specialized|structural]
    general += [(g,target) for g in arms if (g,target) not in specialized]
    gap,a,b = minimum_protected_clearance(model,data,general)
    return dict(general_clearance_m=gap,closest_general_pair=[a,b],pairs=rows,
                safe=bool(gap>=required and all(r["safe"] for r in rows)))


def check_bimanual_path(
    model: mujoco.MjModel,
    current_action: Sequence[float],
    target_action: Sequence[float],
    *,
    required_clearance_m: float = DEFAULT_CLEARANCE_M,
    max_joint_step_rad: float = DEFAULT_MAX_JOINT_STEP_RAD,
    obstacle_geom_names: Sequence[str] = (),
    task_phase: str | None = None,
    reference_data: mujoco.MjData | None = None,
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
    structural_pairs = set(structural_near_support_pairs(model))
    initial = mujoco.MjData(model)
    if reference_data is not None:
        initial.qpos[:] = reference_data.qpos
    mujoco.mj_forward(model, initial)
    task_rows = manipulation_pair_status(model, initial, task_phase)
    task_pairs = {tuple(row["pair"]) for row in task_rows}
    if task_phase is not None:
        target_id = model.geom("red_block_geom").id
        pairs.extend((g,target_id) for g in arm_geoms)

    clearance_pairs = tuple(
        dict.fromkeys(pair for pair in pairs
                      if pair not in same_arm_pair_set and pair not in structural_pairs and pair not in task_pairs)
    )
    data = mujoco.MjData(model)
    if reference_data is not None:
        data.qpos[:] = reference_data.qpos
    best = (math.inf, -1, -1, 0.0)

    for sample_index in range(intervals + 1):
        fraction = sample_index / intervals
        action = current + fraction * (target - current)
        # Interpolated measured poses must not be silently clamped to command limits.
        apply_control_as_pose(model, data, action, preserve_raw_pose=True)
        task_rows = manipulation_pair_status(model,data,task_phase)
        failed_task = next((row for row in task_rows if not row["safe"]),None)
        if failed_task:
            a,b = failed_task["pair"]
            return CollisionAssessment(safe=False,reason="task-specific separation/contact invariant violated",
                minimum_clearance_m=failed_task["clearance_m"],required_clearance_m=0.,
                path_fraction=fraction,checked_samples=sample_index+1,
                first_body=_body_name_for_geom(model,a),second_body=_body_name_for_geom(model,b),
                first_geom_id=a,second_geom_id=b)
        structural = structural_near_support_status(model, data)
        failed = next((row for row in structural if not row["safe"]), None)
        if failed:
            first, second = failed["pair"]
            return CollisionAssessment(
                safe=False, reason="SIM structural near-support invariant violated; target rejected",
                minimum_clearance_m=failed["clearance_m"], required_clearance_m=0.,
                path_fraction=fraction, checked_samples=sample_index+1,
                first_body=_body_name_for_geom(model, first),
                second_body=_body_name_for_geom(model, second),
                first_geom_id=first, second_geom_id=second)
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
