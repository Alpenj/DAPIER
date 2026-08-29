#!/usr/bin/env python3
"""Compose a stationary Waffle Pi and two namespaced SO-101 arms.

This module is simulation-only.  It never opens a serial port and has no
motor-command, ROS 2 publisher, or hardware dispatch path.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Sequence

import mujoco


PROJECT_DIR = Path(__file__).resolve().parent
DAPIER_ROOT = PROJECT_DIR.parents[2]
SIM_DIR = PROJECT_DIR.parent
sys.path.insert(0, str(SIM_DIR / "turtlebot3_waffle_pi"))

from waffle_pi_model import build_spec as build_waffle_pi_spec
from waffle_reference import (
    ASSEMBLED_SUPPORT_UNDER_STL_SHA256,
    ASSEMBLED_SUPPORT_UPPER_STL_SHA256,
    ASSEMBLED_SUPPORT_UPPER_Z_OFFSET_M,
    TOWER_CENTER_X_M,
    TOWER_CENTER_Y_ABS_M,
    SEMI_SUPPORT_BASE_SIZE_M,
    SEMI_SUPPORT_BIG_HOLE_CENTERS_LOCAL_M,
    SEMI_SUPPORT_BIG_HOLE_RADIUS_M,
    SEMI_SUPPORT_BOTTOM_HOLES_LOCAL_M,
    SEMI_SUPPORT_COLUMN_SIZE_M,
    SEMI_SUPPORT_COLUMN_TOP_LOCAL_Z_M,
    SEMI_SUPPORT_LOCAL_MAX_Z_M,
    WAFFLE_BASE_COLLISION_PROXY_TOP_Z_M,
    WAFFLE_TOP_MOUNT_PLANE_Z_M,
    WAFFLE_TOP_REFERENCE_ORIGIN_M,
)


ARM_CONTROL_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
ACTION_NAMES = tuple(
    f"{side}_{name}" for side in ("left", "right") for name in ARM_CONTROL_NAMES
)
RECORDED_UPSTREAM_MODEL_SHA256 = (
    "d75253eb568e8a7214db9c631ab7bed4217f608a26f7276ebe9a7636cac82580"
)
MODEL_RANGE_ROUNDING_TOLERANCE_RAD = 1e-5
DEFAULT_ARM_MOUNT_X_M = 0.02
DEFAULT_ARM_MOUNT_SEPARATION_M = 0.20
TOWER_RECOMMENDED_ARM_MOUNT_SEPARATION_M = 2.0 * TOWER_CENTER_Y_ABS_M
DEFAULT_MOUNT_LAYOUT = "printed-torso"
MOUNT_LAYOUTS = (DEFAULT_MOUNT_LAYOUT, "tower")
HUMANOID_HOLDER_PITCH_RAD = math.pi / 2.0
HUMANOID_LEFT_HOLDER_TWIST_RAD = -math.pi / 2.0
HUMANOID_RIGHT_HOLDER_TWIST_RAD = math.pi / 2.0
WAFFLE_TOP_LOCAL_Z_M = WAFFLE_TOP_MOUNT_PLANE_Z_M
DEPTH_CAMERA_SIZE_M = (0.040, 0.165, 0.048)  # depth, width, height
DEPTH_CAMERA_MASS_KG = 0.310
DEPTH_CAMERA_CENTER_M = (0.120, 0.0, 0.200)
DEPTH_CAMERA_DOWN_TILT_RAD = math.radians(10.0)
DEPTH_CAMERA_HORIZONTAL_FOV_DEG = 58.4
DEPTH_CAMERA_VERTICAL_FOV_DEG = 45.5
PRINTED_MOUNT_ESTIMATED_MASS_KG = 0.90
SO101_BASE_LARGE_HOLE_MESH_SHA256 = (
    "bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d"
)
# base_so101_v2.stl has a radius-8.5 mm circular boundary centered at
# (X=0, Z=0) through its Y=0..72 mm depth. The midpoint below is transformed
# through the XML geom pose into the SO-101 base/body frame.
SO101_BASE_LARGE_HOLE_CENTER_STL_M = (0.0, 0.036, 0.0)
SO101_BASE_LARGE_HOLE_CENTER_ARM_FRAME_M = (
    -0.00636471,
    -8.97657e-09,
    0.0336,
)
TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M = (
    WAFFLE_TOP_LOCAL_Z_M + SEMI_SUPPORT_BIG_HOLE_CENTERS_LOCAL_M[0][2]
    + SO101_BASE_LARGE_HOLE_CENTER_ARM_FRAME_M[0]
)
TOWER_RECOMMENDED_ARM_MOUNT_X_M = TOWER_CENTER_X_M
TOWER_CAMERA_DOWN_TILT_RAD = math.radians(27.0)
TOWER_CAMERA_CENTER_X_M = TOWER_CENTER_X_M
TOWER_CAMERA_CENTER_Z_M = 0.550
TOWER_CAMERA_HEIGHT_ABOVE_ARM_M = (
    TOWER_CAMERA_CENTER_Z_M - TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M
)
TOWER_CAMERA_MAST_SIZE_M = (0.024, 0.030)
TOWER_CAMERA_INTERFACE_PLATE_SIZE_M = (0.050, 0.060, 0.006)
TOWER_MOUNT_ESTIMATED_MASS_KG = 1.50
TOWER_REPLACED_SO101_BASE_MESHES = frozenset(
    {
        "base_motor_holder_so101_v1",
        "base_so101_v2",
        "waveshare_mounting_plate_so101_v2",
    }
)
ASSEMBLED_SUPPORT_UNDER_STL = PROJECT_DIR / "assets" / "assem_base_under.stl"
ASSEMBLED_SUPPORT_UPPER_SOURCE_STL = (
    PROJECT_DIR / "assets" / "assem_base_upper.stl"
)
ASSEMBLED_SUPPORT_UPPER_STL = ASSEMBLED_SUPPORT_UPPER_SOURCE_STL

# Screenshot-matched simulator pose recorded on 2026-08-25. The holder pitch,
# not a shoulder-pan offset, turns both arms into the human-like vertical
# layout. These displayed values remain provisional until the pose editor's
# higher-precision JSON is collected.
HUMANOID_HOME_ACTION = (
    0.004603066159105924,
    -1.571825,
    1.5547999999999997,
    0.01658000000000004,
    1.640225,
    -0.1745,
    0.004603066159105924,
    -1.571825,
    1.5547999999999997,
    0.01658000000000004,
    1.640225,
    -0.1745,
)


def _default_model_candidates() -> tuple[Path, ...]:
    configured = os.environ.get("DAPIER_SO101_MJCF")
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.extend(
        (
            DAPIER_ROOT
            / ".local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco"
            / "assets/so101_new_calib.xml",
            DAPIER_ROOT
            / ".local-workspaces/so101/lerobot/build/lib/lerobot/envs/so101_mujoco"
            / "assets/so101_new_calib.xml",
        )
    )
    return tuple(candidates)


def resolve_so101_model(model_path: Path | str | None = None) -> Path:
    """Resolve an external Apache-2.0 SO-101 MJCF without copying its meshes."""

    if model_path is not None:
        resolved = Path(model_path).expanduser().resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"SO-101 MJCF does not exist: {resolved}")
        return resolved
    for candidate in _default_model_candidates():
        if candidate.is_file():
            return candidate.resolve()
    searched = "\n".join(f"  - {path}" for path in _default_model_candidates())
    raise FileNotFoundError(
        "SO-101 MJCF and adjacent meshes are not available. Set "
        "DAPIER_SO101_MJCF to so101_new_calib.xml. Searched:\n" + searched
    )


def model_provenance(source: Path) -> dict[str, object]:
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    return {
        "model_source": str(source),
        "model_sha256": digest,
        "recorded_upstream_sha256": RECORDED_UPSTREAM_MODEL_SHA256,
        "matches_recorded_upstream": digest == RECORDED_UPSTREAM_MODEL_SHA256,
    }


def _verify_asset_sha256(path: Path, expected_sha256: str) -> Path:
    if not path.is_file():
        raise FileNotFoundError(f"required simulation asset is missing: {path}")
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != expected_sha256:
        raise RuntimeError(
            f"simulation asset hash mismatch for {path.name}: {actual}"
        )
    return path.resolve()


def _quaternion_multiply(
    first: Sequence[float], second: Sequence[float]
) -> list[float]:
    """Compose MuJoCo wxyz quaternions as ``first * second``."""

    w1, x1, y1, z1 = first
    w2, x2, y2, z2 = second
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def _holder_quaternion(pitch_rad: float, twist_rad: float) -> list[float]:
    """Pitch the arm downward, then twist its holder about local X."""

    pitch = [math.cos(pitch_rad / 2.0), 0.0, math.sin(pitch_rad / 2.0), 0.0]
    twist = [math.cos(twist_rad / 2.0), math.sin(twist_rad / 2.0), 0.0, 0.0]
    return _quaternion_multiply(pitch, twist)


def _validate_source_contract(arm_spec: mujoco.MjSpec, source: Path) -> None:
    """Fail closed if the external SO-101 model has a different action contract."""

    compiled = arm_spec.compile()
    names = tuple(
        mujoco.mj_id2name(compiled, mujoco.mjtObj.mjOBJ_ACTUATOR, index) or ""
        for index in range(compiled.nu)
    )
    if names != ARM_CONTROL_NAMES:
        raise RuntimeError(
            f"unexpected SO-101 actuator contract in {source}: {names!r}"
        )
    if (compiled.nq, compiled.nv, compiled.nu, compiled.njnt) != (6, 6, 6, 6):
        raise RuntimeError(
            "unexpected SO-101 dimensions in "
            f"{source}: nq={compiled.nq}, nv={compiled.nv}, "
            f"nu={compiled.nu}, njnt={compiled.njnt}"
        )


def _add_depth_camera(
    base_link: mujoco.MjsBody,
    *,
    center_m: Sequence[float] = DEPTH_CAMERA_CENTER_M,
    down_tilt_rad: float = DEPTH_CAMERA_DOWN_TILT_RAD,
) -> None:
    """Add a conservative Astra-series front RGB-D mass and optical camera.

    AADJA1300GX was observed as an Orbbec Astra-family USB device, but its
    exact enclosure and optical origin have not yet been measured. The
    official Astra-series 165 x 48 x 40 mm, 310 g envelope is therefore used
    as a deliberately conservative placeholder.
    """

    if len(center_m) != 3 or not all(
        math.isfinite(float(value)) for value in center_m
    ):
        raise ValueError("depth camera center must contain three finite values")
    if (
        not math.isfinite(down_tilt_rad)
        or not 0.0 <= down_tilt_rad < math.pi / 2.0
    ):
        raise ValueError("depth camera down tilt must be in [0, pi/2)")

    tilt = down_tilt_rad
    camera_body = base_link.add_body(
        name="depth_camera_body",
        pos=list(center_m),
        quat=[math.cos(tilt / 2.0), 0.0, math.sin(tilt / 2.0), 0.0],
    )
    depth, width, height = DEPTH_CAMERA_SIZE_M
    camera_body.add_geom(
        name="depth_camera_collision",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[depth / 2.0, width / 2.0, height / 2.0],
        mass=DEPTH_CAMERA_MASS_KG,
        contype=1,
        conaffinity=1,
        rgba=[0.04, 0.05, 0.06, 1.0],
    )
    # MuJoCo cameras look along local -Z with local +Y as image-up. This
    # quaternion maps that optical convention to the enclosure's local +X.
    camera_body.add_camera(
        name="front_depth_camera",
        pos=[depth / 2.0 + 0.001, 0.0, 0.0],
        quat=[-0.5, -0.5, 0.5, 0.5],
        fovy=DEPTH_CAMERA_VERTICAL_FOV_DEG,
    )
    camera_body.add_site(
        name="front_depth_optical_frame",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        pos=[depth / 2.0 + 0.001, 0.0, 0.0],
        size=[0.004, 0.004, 0.004],
        rgba=[0.1, 0.8, 1.0, 1.0],
    )


def _add_printed_mount_structure(
    base_link: mujoco.MjsBody,
    *,
    arm_mount_height_m: float,
    arm_mount_x_m: float,
    arm_mount_separation_m: float,
) -> None:
    """Add a panel-built printed torso matching the rotated base flanges.

    Rotating each complete SO-101 by 90 degrees also turns its original
    horizontal mounting flange vertical. The arm therefore bolts to the two
    vertical side walls of a closed torsion box; no horizontal shelf pretends
    to support a vertical flange. Flat panels, internal ribs and through-bolts
    are the intended FDM construction.
    """

    deck_thickness = 0.008
    deck_top = WAFFLE_TOP_LOCAL_Z_M + deck_thickness
    torso_top = arm_mount_height_m + 0.040
    torso_height = torso_top - deck_top
    if torso_height <= 0.10:
        raise ValueError(
            "arm_mount_height_m is too low for the printed torso structure"
        )

    mount_body = base_link.add_body(name="printed_mount_structure")
    common = {
        "type": mujoco.mjtGeom.mjGEOM_BOX,
        "contype": 0,
        "conaffinity": 0,
    }
    for side, y_sign in (("left", 1.0), ("right", -1.0)):
        deck_y = y_sign * 0.07
        mount_body.add_geom(
            name=f"mount_deck_{side}",
            pos=[
                0.0,
                deck_y,
                WAFFLE_TOP_LOCAL_Z_M + deck_thickness / 2.0,
            ],
            size=[0.080, 0.070, deck_thickness / 2.0],
            mass=0.10,
            rgba=[0.12, 0.15, 0.18, 1.0],
            **common,
        )
        mount_body.add_geom(
            name=f"{side}_vertical_arm_mount_plate",
            pos=[
                arm_mount_x_m,
                y_sign * (arm_mount_separation_m / 2.0 - 0.005),
                deck_top + torso_height / 2.0,
            ],
            size=[0.065, 0.005, torso_height / 2.0],
            mass=0.14,
            rgba=[0.15, 0.35, 0.65, 1.0],
            **common,
        )

        for rib_name, x_position in (
            ("rear", arm_mount_x_m - 0.050),
            ("front", arm_mount_x_m + 0.050),
        ):
            mount_body.add_geom(
                name=f"{side}_torso_gusset_{rib_name}",
                pos=[
                    x_position,
                    y_sign * 0.055,
                    deck_top + 0.035,
                ],
                size=[0.008, 0.040, 0.035],
                mass=0.025,
                rgba=[0.25, 0.30, 0.36, 1.0],
                **common,
            )

    # Front and rear walls close the section so the mount resists torsion
    # from asymmetric one-arm motion instead of behaving like two cantilevers.
    for face, x_position in (
        ("front", arm_mount_x_m + 0.060),
        ("rear", arm_mount_x_m - 0.060),
    ):
        mount_body.add_geom(
            name=f"torso_{face}_panel",
            pos=[x_position, 0.0, deck_top + torso_height / 2.0],
            size=[
                0.005,
                arm_mount_separation_m / 2.0 - 0.010,
                torso_height / 2.0,
            ],
            mass=0.10,
            rgba=[0.35, 0.38, 0.42, 1.0],
            **common,
        )
    mount_body.add_geom(
        name="torso_top_panel",
        pos=[arm_mount_x_m, 0.0, torso_top - 0.005],
        size=[0.065, arm_mount_separation_m / 2.0, 0.005],
        mass=0.08,
        rgba=[0.25, 0.30, 0.36, 1.0],
        **common,
    )

    # Two short pads attach the RGB-D enclosure to the front torso wall.
    camera_x, _, camera_z = DEPTH_CAMERA_CENTER_M
    bracket_x = (
        arm_mount_x_m
        + 0.060
        + (camera_x - DEPTH_CAMERA_SIZE_M[0] / 2.0 - (arm_mount_x_m + 0.060))
        / 2.0
    )
    bracket_half_x = camera_x - DEPTH_CAMERA_SIZE_M[0] / 2.0 - bracket_x
    for side, y in (("left", 0.060), ("right", -0.060)):
        mount_body.add_geom(
            name=f"depth_camera_mount_pad_{side}",
            pos=[bracket_x, y, camera_z],
            size=[bracket_half_x, 0.012, 0.025],
            mass=0.02,
            rgba=[0.15, 0.35, 0.65, 1.0],
            **common,
        )

    assigned_mass = (
        2 * 0.10
        + 2 * 0.14
        + 4 * 0.025
        + 2 * 0.10
        + 0.08
        + 2 * 0.02
    )
    if not math.isclose(assigned_mass, PRINTED_MOUNT_ESTIMATED_MASS_KG):
        raise RuntimeError("printed mount mass budget is inconsistent")


def _tower_camera_center(arm_mount_height_m: float) -> tuple[float, float, float]:
    if not math.isclose(
        arm_mount_height_m,
        TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
        abs_tol=1e-9,
    ):
        raise ValueError("tower camera requires the fixed SO-101 socket height")
    return (
        TOWER_CAMERA_CENTER_X_M,
        0.0,
        TOWER_CAMERA_CENTER_Z_M,
    )


def _add_waffle_top_reference_axes(base_link: mujoco.MjsBody) -> None:
    """Show a translated copy of the official base_link axes at the top datum."""

    origin = WAFFLE_TOP_REFERENCE_ORIGIN_M
    base_link.add_site(
        name="waffle_top_reference_origin",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        pos=list(origin),
        size=[0.004, 0.004, 0.004],
        rgba=[1.0, 0.55, 0.0, 1.0],
    )
    for name, offset, color in (
        ("x_forward", (0.060, 0.0, 0.0), (0.95, 0.15, 0.15, 1.0)),
        ("y_left", (0.0, 0.060, 0.0), (0.15, 0.85, 0.20, 1.0)),
        ("z_up", (0.0, 0.0, 0.060), (0.15, 0.35, 1.0, 1.0)),
    ):
        endpoint = [origin[index] + offset[index] for index in range(3)]
        base_link.add_site(
            name=f"waffle_top_axis_{name}",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            fromto=[*origin, *endpoint],
            size=[0.0015, 0.0015, 0.0015],
            rgba=list(color),
        )


def _add_tower_mount_structure(
    base_link: mujoco.MjsBody,
    *,
    support_mesh_names: tuple[str, str],
    arm_mount_height_m: float,
    arm_mount_x_m: float,
    arm_mount_separation_m: float,
    camera_center_m: Sequence[float],
) -> None:
    """Place the exact split-print support visuals plus simple collisions.

    Both visuals are the supplied split-print parts. The upper's side sockets
    replace the stock SO-101 printed base pieces and receive each base servo
    plus shoulder chain on the original large-hole axis. A narrow mast and
    tilted interface plate raise the camera above the fixed arm sockets; they
    are not a bulky enclosure around the camera.
    """

    _, camera_y, _ = (float(value) for value in camera_center_m)
    required = (
        (arm_mount_height_m, TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M, "height"),
        (arm_mount_x_m, TOWER_RECOMMENDED_ARM_MOUNT_X_M, "X"),
        (
            arm_mount_separation_m,
            TOWER_RECOMMENDED_ARM_MOUNT_SEPARATION_M,
            "separation",
        ),
    )
    for actual, expected, label in required:
        if not math.isclose(actual, expected, abs_tol=1e-9):
            raise ValueError(
                f"semi_so101 STEP layout requires arm mount {label} "
                f"{expected:.6f} m"
            )
    if not math.isclose(camera_y, 0.0, abs_tol=1e-12):
        raise ValueError("tower depth camera must remain centered")

    mount_body = base_link.add_body(name="tower_mount_structure")
    _add_waffle_top_reference_axes(base_link)
    hidden_collision = {
        "type": mujoco.mjtGeom.mjGEOM_BOX,
        "mass": 0.0,
        "contype": 1,
        "conaffinity": 1,
        "group": 3,
        "rgba": [0.3, 0.8, 1.0, 0.0],
    }

    support_x = TOWER_CENTER_X_M
    base_depth, base_width, base_height = SEMI_SUPPORT_BASE_SIZE_M
    base_bottom = WAFFLE_TOP_LOCAL_Z_M
    base_top = base_bottom + base_height
    under_mesh_name, upper_mesh_name = support_mesh_names
    mount_body.add_geom(
        name="assembled_support_under_visual",
        type=mujoco.mjtGeom.mjGEOM_MESH,
        meshname=under_mesh_name,
        pos=[support_x, 0.0, base_bottom],
        mass=0.865,
        rgba=[0.72, 0.70, 0.64, 1.0],
        contype=0,
        conaffinity=0,
        group=1,
    )
    mount_body.add_geom(
        name="assembled_support_upper_visual",
        type=mujoco.mjtGeom.mjGEOM_MESH,
        meshname=upper_mesh_name,
        pos=[
            support_x,
            0.0,
            base_bottom + ASSEMBLED_SUPPORT_UPPER_Z_OFFSET_M,
        ],
        mass=0.635,
        rgba=[0.62, 0.64, 0.62, 1.0],
        contype=0,
        conaffinity=0,
        group=1,
    )
    mount_body.add_geom(
        name="semi_support_base_collision",
        pos=[support_x, 0.0, base_bottom + base_height / 2.0],
        size=[base_depth / 2.0, base_width / 2.0, base_height / 2.0],
        **hidden_collision,
    )
    for index, (local_x, local_y, _) in enumerate(
        SEMI_SUPPORT_BOTTOM_HOLES_LOCAL_M, start=1
    ):
        base_link.add_site(
            name=f"semi_support_bottom_hole_{index}",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            pos=[support_x + local_x, local_y, base_top + 0.0005],
            size=[0.003, 0.001, 0.001],
            rgba=[0.90, 0.20, 0.15, 0.9],
        )

    column_depth, column_width, column_height = SEMI_SUPPORT_COLUMN_SIZE_M
    column_center_z = base_top + column_height / 2.0
    column_top = base_bottom + SEMI_SUPPORT_COLUMN_TOP_LOCAL_Z_M
    mount_body.add_geom(
        name="semi_support_column_collision",
        pos=[support_x, 0.0, column_center_z],
        size=[column_depth / 2.0, column_width / 2.0, column_height / 2.0],
        **hidden_collision,
    )

    camera_x, _, camera_z = (float(value) for value in camera_center_m)
    camera_tilt_quat = [
        math.cos(TOWER_CAMERA_DOWN_TILT_RAD / 2.0),
        0.0,
        math.sin(TOWER_CAMERA_DOWN_TILT_RAD / 2.0),
        0.0,
    ]
    plate_depth, plate_width, plate_thickness = (
        TOWER_CAMERA_INTERFACE_PLATE_SIZE_M
    )
    local_down_x = -math.sin(TOWER_CAMERA_DOWN_TILT_RAD)
    local_down_z = -math.cos(TOWER_CAMERA_DOWN_TILT_RAD)
    camera_half_height = DEPTH_CAMERA_SIZE_M[2] / 2.0
    plate_offset = camera_half_height + plate_thickness / 2.0
    plate_center = [
        camera_x + local_down_x * plate_offset,
        0.0,
        camera_z + local_down_z * plate_offset,
    ]
    plate_vertical_half_extent = (
        plate_depth / 2.0 * math.sin(TOWER_CAMERA_DOWN_TILT_RAD)
        + plate_thickness / 2.0 * math.cos(TOWER_CAMERA_DOWN_TILT_RAD)
    )
    mast_top = plate_center[2] - plate_vertical_half_extent
    mast_depth, mast_width = TOWER_CAMERA_MAST_SIZE_M
    mast_height = mast_top - column_top
    if mast_height <= 0:
        raise RuntimeError("camera mast must rise above the support column")
    visible_interface = {
        "type": mujoco.mjtGeom.mjGEOM_BOX,
        "mass": 0.0,
        "contype": 0,
        "conaffinity": 0,
        "group": 1,
        "rgba": [0.23, 0.25, 0.27, 1.0],
    }
    mount_body.add_geom(
        name="tower_camera_mast_visual",
        pos=[support_x, 0.0, column_top + mast_height / 2.0],
        size=[mast_depth / 2.0, mast_width / 2.0, mast_height / 2.0],
        **visible_interface,
    )
    mount_body.add_geom(
        name="tower_camera_mast_collision",
        pos=[support_x, 0.0, column_top + mast_height / 2.0],
        size=[mast_depth / 2.0, mast_width / 2.0, mast_height / 2.0],
        **hidden_collision,
    )
    mount_body.add_geom(
        name="tower_camera_interface_plate_visual",
        pos=plate_center,
        quat=camera_tilt_quat,
        size=[plate_depth / 2.0, plate_width / 2.0, plate_thickness / 2.0],
        **visible_interface,
    )
    mount_body.add_geom(
        name="tower_camera_interface_plate_collision",
        pos=plate_center,
        quat=camera_tilt_quat,
        size=[plate_depth / 2.0, plate_width / 2.0, plate_thickness / 2.0],
        **hidden_collision,
    )
    base_link.add_site(
        name="camera_mount_center_measurement_required",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=plate_center,
        quat=camera_tilt_quat,
        size=[0.003175, plate_thickness / 2.0, plate_thickness / 2.0],
        rgba=[0.95, 0.35, 0.10, 0.85],
    )

    for side, (local_x, local_y, local_z) in zip(
        ("left", "right"), SEMI_SUPPORT_BIG_HOLE_CENTERS_LOCAL_M
    ):
        base_link.add_site(
            name=f"semi_support_{side}_shoulder_hole_center",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            pos=[support_x + local_x, local_y, base_bottom + local_z],
            size=[SEMI_SUPPORT_BIG_HOLE_RADIUS_M] * 3,
            rgba=[0.10, 0.80, 1.0, 0.35],
        )

    assigned_mass = 0.865 + 0.635
    if not math.isclose(assigned_mass, TOWER_MOUNT_ESTIMATED_MASS_KG):
        raise RuntimeError("semi_so101 support mass budget is inconsistent")


def _replace_stock_base_with_tower_socket(arm_spec: mujoco.MjSpec) -> None:
    """Use the custom upper socket instead of duplicate stock base prints."""

    base = arm_spec.body("base")
    deleted = set()
    for geom in list(base.geoms):
        if geom.meshname in TOWER_REPLACED_SO101_BASE_MESHES:
            deleted.add(geom.meshname)
            arm_spec.delete(geom)
    if deleted != TOWER_REPLACED_SO101_BASE_MESHES:
        missing = sorted(TOWER_REPLACED_SO101_BASE_MESHES - deleted)
        raise RuntimeError(f"SO-101 tower socket replacement mismatch: {missing}")


def build_spec(
    *,
    arm_mount_height_m: float,
    arm_mount_x_m: float | None = None,
    arm_mount_separation_m: float | None = None,
    mount_layout: str = DEFAULT_MOUNT_LAYOUT,
    include_lidar: bool = False,
    model_path: Path | str | None = None,
) -> tuple[mujoco.MjSpec, Path]:
    """Build a Waffle Pi with two human-shoulder-oriented SO-101 arms.

    Mount dimensions are deliberately explicit because the previous 0.55 m
    visualization height places the short SO-101 outside floor-shoe reach.
    """

    if mount_layout not in MOUNT_LAYOUTS:
        raise ValueError(
            f"mount_layout must be one of {MOUNT_LAYOUTS}, got {mount_layout!r}"
        )
    if arm_mount_x_m is None:
        arm_mount_x_m = (
            TOWER_RECOMMENDED_ARM_MOUNT_X_M
            if mount_layout == "tower"
            else DEFAULT_ARM_MOUNT_X_M
        )
    if arm_mount_separation_m is None:
        arm_mount_separation_m = (
            TOWER_RECOMMENDED_ARM_MOUNT_SEPARATION_M
            if mount_layout == "tower"
            else DEFAULT_ARM_MOUNT_SEPARATION_M
        )
    values = (arm_mount_height_m, arm_mount_x_m, arm_mount_separation_m)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("arm mount dimensions must be finite")
    if arm_mount_height_m <= 0:
        raise ValueError("arm_mount_height_m must be positive")
    if arm_mount_separation_m <= 0:
        raise ValueError("arm_mount_separation_m must be positive")

    source = resolve_so101_model(model_path)
    source_spec = mujoco.MjSpec.from_file(str(source))
    _validate_source_contract(source_spec, source)

    spec = build_waffle_pi_spec()
    # Replace the original tiny RGB-camera chain with the front RGB-D body.
    spec.delete(spec.body("tb3_camera_link"))
    if not include_lidar:
        spec.delete(spec.body("tb3_base_scan"))
    base_link = spec.body("tb3_base_link")
    if mount_layout == "tower":
        under_path = _verify_asset_sha256(
            ASSEMBLED_SUPPORT_UNDER_STL,
            ASSEMBLED_SUPPORT_UNDER_STL_SHA256,
        )
        upper_path = _verify_asset_sha256(
            ASSEMBLED_SUPPORT_UPPER_STL,
            ASSEMBLED_SUPPORT_UPPER_STL_SHA256,
        )
        under_mesh = spec.add_mesh(
            name="assembled_support_under_mesh",
            file=str(under_path),
            scale=[0.001, 0.001, 0.001],
        )
        upper_mesh = spec.add_mesh(
            name="assembled_support_upper_mesh",
            file=str(upper_path),
            scale=[0.001, 0.001, 0.001],
        )
        camera_center_m = _tower_camera_center(arm_mount_height_m)
        _add_tower_mount_structure(
            base_link,
            support_mesh_names=(under_mesh.name, upper_mesh.name),
            arm_mount_height_m=arm_mount_height_m,
            arm_mount_x_m=arm_mount_x_m,
            arm_mount_separation_m=arm_mount_separation_m,
            camera_center_m=camera_center_m,
        )
        _add_depth_camera(
            base_link,
            center_m=camera_center_m,
            down_tilt_rad=TOWER_CAMERA_DOWN_TILT_RAD,
        )
    else:
        _add_printed_mount_structure(
            base_link,
            arm_mount_height_m=arm_mount_height_m,
            arm_mount_x_m=arm_mount_x_m,
            arm_mount_separation_m=arm_mount_separation_m,
        )
        _add_depth_camera(base_link)
    half_separation = arm_mount_separation_m / 2.0
    mounts = (
        (
            "left",
            [arm_mount_x_m, half_separation, arm_mount_height_m],
            HUMANOID_LEFT_HOLDER_TWIST_RAD,
        ),
        (
            "right",
            [arm_mount_x_m, -half_separation, arm_mount_height_m],
            HUMANOID_RIGHT_HOLDER_TWIST_RAD,
        ),
    )
    for side, position, twist_rad in mounts:
        frame = base_link.add_frame(
            name=f"{side}_arm_mount",
            pos=position,
            quat=_holder_quaternion(HUMANOID_HOLDER_PITCH_RAD, twist_rad),
        )
        arm_spec = mujoco.MjSpec.from_file(str(source))
        if mount_layout == "tower":
            _replace_stock_base_with_tower_socket(arm_spec)
        spec.attach(arm_spec, prefix=f"{side}_", frame=frame)
    return spec, source


def build_model(
    *,
    arm_mount_height_m: float,
    arm_mount_x_m: float | None = None,
    arm_mount_separation_m: float | None = None,
    mount_layout: str = DEFAULT_MOUNT_LAYOUT,
    include_lidar: bool = False,
    model_path: Path | str | None = None,
) -> tuple[mujoco.MjModel, Path]:
    spec, source = build_spec(
        arm_mount_height_m=arm_mount_height_m,
        arm_mount_x_m=arm_mount_x_m,
        arm_mount_separation_m=arm_mount_separation_m,
        mount_layout=mount_layout,
        include_lidar=include_lidar,
        model_path=model_path,
    )
    return spec.compile(), source


def model_names(
    model: mujoco.MjModel, object_type: mujoco.mjtObj, count: int
) -> tuple[str, ...]:
    return tuple(
        mujoco.mj_id2name(model, object_type, index) or ""
        for index in range(count)
    )


def actuator_targets_from_qpos(
    model: mujoco.MjModel, qpos: Sequence[float]
) -> tuple[float, ...]:
    if len(qpos) != model.nq:
        raise ValueError(f"expected qpos length {model.nq}, got {len(qpos)}")
    if model.nu != len(ACTION_NAMES):
        raise RuntimeError(f"expected {len(ACTION_NAMES)} actuators, got {model.nu}")
    targets = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            raise RuntimeError(f"actuator {actuator_id} is not attached to a joint")
        targets.append(float(qpos[int(model.jnt_qposadr[joint_id])]))
    return tuple(targets)


def apply_control_as_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    action: Sequence[float] | None = None,
) -> None:
    """Kinematically apply simulator sliders; this never dispatches hardware."""

    interactive_slider = action is None
    requested = data.ctrl if interactive_slider else action
    if len(requested) != len(ACTION_NAMES):
        raise ValueError(f"expected {len(ACTION_NAMES)} targets, got {len(requested)}")
    for actuator_id, raw_target in enumerate(requested):
        target = float(raw_target)
        if not math.isfinite(target):
            raise ValueError("pose targets must be finite")
        lower, upper = model.actuator_ctrlrange[actuator_id]
        if interactive_slider:
            # MuJoCo's UI rounds displayed slider endpoints (for example,
            # -1.91986 becomes -1.92). Clamp that UI value back to the model
            # range instead of terminating the pose editor.
            target = min(float(upper), max(float(lower), target))
        else:
            if (
                target < float(lower) - MODEL_RANGE_ROUNDING_TOLERANCE_RAD
                or target > float(upper) + MODEL_RANGE_ROUNDING_TOLERANCE_RAD
            ):
                raise ValueError(
                    f"{ACTION_NAMES[actuator_id]}={target} is outside "
                    f"{float(lower)}..{float(upper)}"
                )
            target = min(float(upper), max(float(lower), target))
        data.ctrl[actuator_id] = target
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        data.qpos[int(model.jnt_qposadr[joint_id])] = target
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def pose_report(data: mujoco.MjData, source: Path) -> dict[str, object]:
    return {
        "schema_version": "dapier.so101-dual-mujoco-pose.v0.1",
        "robot": "dual_so101",
        **model_provenance(source),
        "action_order": ACTION_NAMES,
        "radians": tuple(float(value) for value in data.ctrl),
        "degrees": tuple(math.degrees(float(value)) for value in data.ctrl),
        "published": False,
        "control_authorized": False,
        "hardware_dispatch_authorized": False,
        "executed_action": False,
        "hardware_execution": False,
    }


def validate_model(
    model: mujoco.MjModel,
    *,
    source: Path,
    smoke_steps: int = 1000,
    initial_action: Sequence[float] = HUMANOID_HOME_ACTION,
) -> dict[str, object]:
    if smoke_steps < 0:
        raise ValueError("smoke_steps must be non-negative")
    if (model.nq, model.nv, model.nu, model.njnt, model.neq) != (14, 14, 12, 14, 0):
        raise RuntimeError(
            "unexpected mobile dual-SO-101 dimensions: "
            f"nq={model.nq}, nv={model.nv}, nu={model.nu}, "
            f"njnt={model.njnt}, neq={model.neq}"
        )
    actuators = model_names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
    if actuators != ACTION_NAMES:
        raise RuntimeError(f"unexpected actuator order: {actuators!r}")
    if any("wheel" in name for name in actuators):
        raise RuntimeError("wheel actuators are not authorized in this model")

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link")
    for side in ("left", "right"):
        arm_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_base"
        )
        if arm_id < 0 or int(model.body_parentid[arm_id]) != base_id:
            raise RuntimeError(f"{side}_base is not mounted to tb3_base_link")

    data = mujoco.MjData(model)
    apply_control_as_pose(model, data, initial_action)
    for _ in range(smoke_steps):
        mujoco.mj_step(model, data)
    finite_state = all(math.isfinite(float(value)) for value in data.qpos) and all(
        math.isfinite(float(value)) for value in data.qvel
    )
    if not finite_state:
        raise RuntimeError("dual-SO-101 simulation produced a non-finite state")
    tower_layout = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "tower_mount_structure"
    ) >= 0
    mount_layout = "tower" if tower_layout else DEFAULT_MOUNT_LAYOUT
    mount_mass = (
        TOWER_MOUNT_ESTIMATED_MASS_KG
        if tower_layout
        else PRINTED_MOUNT_ESTIMATED_MASS_KG
    )
    return {
        **model_provenance(source),
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
        "base_fixed": True,
        "lidar_present": mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_scan"
        ) >= 0,
        "original_rgb_camera_present": mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "tb3_camera_link"
        ) >= 0,
        "front_depth_camera_present": mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, "front_depth_camera"
        ) >= 0,
        "mount_layout": mount_layout,
        "mount_estimated_mass_kg": mount_mass,
        "printed_mount_estimated_mass_kg": (
            None if tower_layout else PRINTED_MOUNT_ESTIMATED_MASS_KG
        ),
        "depth_camera_assumed_mass_kg": DEPTH_CAMERA_MASS_KG,
        "waffle_top_mount_plane_z_m": WAFFLE_TOP_LOCAL_Z_M,
        "waffle_collision_proxy_top_z_m": WAFFLE_BASE_COLLISION_PROXY_TOP_Z_M,
        "wheel_actuators_present": False,
        "published": False,
        "control_authorized": False,
        "hardware_dispatch_authorized": False,
        "executed_action": False,
        "hardware_execution": False,
    }


def _run_viewer(
    model: mujoco.MjModel, initial_action: Sequence[float]
) -> None:
    import mujoco.viewer

    data = mujoco.MjData(model)
    apply_control_as_pose(model, data, initial_action)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


def _run_pose_editor(
    model: mujoco.MjModel,
    source: Path,
    save_pose: Path | None,
    initial_action: Sequence[float],
) -> None:
    if save_pose is not None and save_pose.exists():
        raise FileExistsError(f"refusing to overwrite existing pose: {save_pose}")
    import mujoco.viewer

    data = mujoco.MjData(model)
    apply_control_as_pose(model, data, initial_action)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            viewer.sync()
            with viewer.lock():
                apply_control_as_pose(model, data)
            time.sleep(0.02)
    report = pose_report(data, source)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if save_pose is not None:
        save_pose.parent.mkdir(parents=True, exist_ok=True)
        save_pose.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"Pose saved: {save_pose}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate stationary Waffle Pi plus dual-SO-101 simulation."
    )
    parser.add_argument("--arm-mount-height-m", type=float, required=True)
    parser.add_argument(
        "--arm-mount-x-m",
        type=float,
        help=(
            "mount X; defaults to -0.060 m for tower and +0.020 m for "
            "printed-torso"
        ),
    )
    parser.add_argument(
        "--arm-mount-separation-m",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--mount-layout",
        choices=MOUNT_LAYOUTS,
        default=DEFAULT_MOUNT_LAYOUT,
        help="printed torso (existing default) or dual tower concept",
    )
    parser.add_argument("--model", type=Path)
    parser.add_argument(
        "--include-lidar",
        action="store_true",
        help="include the removed TurtleBot3 LiDAR for comparison only",
    )
    parser.add_argument("--smoke-steps", type=int, default=1000)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--pose-editor", action="store_true")
    parser.add_argument("--save-pose", type=Path)
    parser.add_argument(
        "--initial-action-rad",
        type=float,
        nargs=len(ACTION_NAMES),
        metavar="RAD",
        help="12 simulator-only initial targets in ACTION_NAMES order",
    )
    args = parser.parse_args(argv)
    if args.viewer and args.pose_editor:
        parser.error("--viewer and --pose-editor are mutually exclusive")
    if args.save_pose is not None and not args.pose_editor:
        parser.error("--save-pose requires --pose-editor")

    initial_action = (
        tuple(args.initial_action_rad)
        if args.initial_action_rad is not None
        else HUMANOID_HOME_ACTION
    )
    model, source = build_model(
        arm_mount_height_m=args.arm_mount_height_m,
        arm_mount_x_m=args.arm_mount_x_m,
        arm_mount_separation_m=args.arm_mount_separation_m,
        mount_layout=args.mount_layout,
        include_lidar=args.include_lidar,
        model_path=args.model,
    )
    print(
        json.dumps(
            validate_model(
                model,
                source=source,
                smoke_steps=args.smoke_steps,
                initial_action=initial_action,
            ),
            ensure_ascii=False,
            indent=2,
        )
    )
    if args.viewer:
        _run_viewer(model, initial_action)
    if args.pose_editor:
        _run_pose_editor(model, source, args.save_pose, initial_action)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
