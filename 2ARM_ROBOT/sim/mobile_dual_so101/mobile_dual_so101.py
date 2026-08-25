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
HUMANOID_HOLDER_PITCH_RAD = math.pi / 2.0
HUMANOID_LEFT_HOLDER_TWIST_RAD = -math.pi / 2.0
HUMANOID_RIGHT_HOLDER_TWIST_RAD = math.pi / 2.0
WAFFLE_TOP_LOCAL_Z_M = 0.094
DEPTH_CAMERA_SIZE_M = (0.040, 0.165, 0.048)  # depth, width, height
DEPTH_CAMERA_MASS_KG = 0.310
DEPTH_CAMERA_CENTER_M = (0.120, 0.0, 0.200)
DEPTH_CAMERA_DOWN_TILT_RAD = math.radians(10.0)
DEPTH_CAMERA_HORIZONTAL_FOV_DEG = 58.4
DEPTH_CAMERA_VERTICAL_FOV_DEG = 45.5
PRINTED_MOUNT_ESTIMATED_MASS_KG = 0.90

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


def _add_depth_camera(base_link: mujoco.MjsBody) -> None:
    """Add a conservative Astra-series front RGB-D mass and optical camera.

    AADJA1300GX was observed as an Orbbec Astra-family USB device, but its
    exact enclosure and optical origin have not yet been measured. The
    official Astra-series 165 x 48 x 40 mm, 310 g envelope is therefore used
    as a deliberately conservative placeholder.
    """

    tilt = DEPTH_CAMERA_DOWN_TILT_RAD
    camera_body = base_link.add_body(
        name="depth_camera_body",
        pos=list(DEPTH_CAMERA_CENTER_M),
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


def build_spec(
    *,
    arm_mount_height_m: float,
    arm_mount_x_m: float = DEFAULT_ARM_MOUNT_X_M,
    arm_mount_separation_m: float = DEFAULT_ARM_MOUNT_SEPARATION_M,
    include_lidar: bool = False,
    model_path: Path | str | None = None,
) -> tuple[mujoco.MjSpec, Path]:
    """Build a Waffle Pi with two human-shoulder-oriented SO-101 arms.

    Mount dimensions are deliberately explicit because the previous 0.55 m
    visualization height places the short SO-101 outside floor-shoe reach.
    """

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
        spec.attach(
            mujoco.MjSpec.from_file(str(source)),
            prefix=f"{side}_",
            frame=frame,
        )
    return spec, source


def build_model(
    *,
    arm_mount_height_m: float,
    arm_mount_x_m: float = DEFAULT_ARM_MOUNT_X_M,
    arm_mount_separation_m: float = DEFAULT_ARM_MOUNT_SEPARATION_M,
    include_lidar: bool = False,
    model_path: Path | str | None = None,
) -> tuple[mujoco.MjModel, Path]:
    spec, source = build_spec(
        arm_mount_height_m=arm_mount_height_m,
        arm_mount_x_m=arm_mount_x_m,
        arm_mount_separation_m=arm_mount_separation_m,
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
        "printed_mount_estimated_mass_kg": PRINTED_MOUNT_ESTIMATED_MASS_KG,
        "depth_camera_assumed_mass_kg": DEPTH_CAMERA_MASS_KG,
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
    parser.add_argument("--arm-mount-x-m", type=float, default=DEFAULT_ARM_MOUNT_X_M)
    parser.add_argument(
        "--arm-mount-separation-m",
        type=float,
        default=DEFAULT_ARM_MOUNT_SEPARATION_M,
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
