#!/usr/bin/env python3
"""Simulation-only compact TurtleBot3 mount for central RGB-D and two SO-101s."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import sys

import mujoco
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parent
SIM_DIR = PROJECT_DIR.parent
sys.path.insert(0, str(SIM_DIR / "turtlebot3_waffle_pi"))

from waffle_pi_model import build_spec as build_waffle_pi_spec  # noqa: E402

from box_shoe_scene import (  # noqa: E402
    BoxShoeSceneConfig,
    _add_box,
    _add_cuboid_shoe,
)
from mobile_dual_so101 import (  # noqa: E402
    ACTION_NAMES,
    HUMANOID_HOME_ACTION,
    WAFFLE_TOP_LOCAL_Z_M,
    _add_front_slam_depth_camera,
    _add_workspace_depth_camera,
    _validate_source_contract,
    apply_control_as_pose,
    resolve_so101_model,
)
from pgripper import replace_gripper, selected_sides, home_action
from mujoco_mission_adapters import _add_gripper_cameras  # noqa: E402
from waffle_reference import WAFFLE_TOP_REFERENCE_ORIGIN_M  # noqa: E402


COMPACT_HOME_ACTION = tuple(
    value - math.pi if index in (4, 10) else value
    for index, value in enumerate(HUMANOID_HOME_ACTION)
)


@dataclass(frozen=True)
class CompactMobileConfig:
    """Provisional mount values; final values require physical measurement."""

    arm_base_separation_m: float = 0.16
    arm_inward_yaw_rad: float = 0.0
    mount_x_m: float = WAFFLE_TOP_REFERENCE_ORIGIN_M[0]
    camera_center_z_m: float = 0.410
    camera_down_tilt_rad: float = math.radians(43.0)
    box_center_xy_m: tuple[float, float] = (0.42, 0.0)
    box_yaw_deg: float = -90.0

    def validate(self) -> None:
        values = (
            self.arm_base_separation_m,
            self.arm_inward_yaw_rad,
            self.mount_x_m,
            self.camera_center_z_m,
            self.camera_down_tilt_rad,
            self.box_yaw_deg,
            *self.box_center_xy_m,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("compact mount values must be finite")
        if not 0.08 <= self.arm_base_separation_m <= 0.20:
            raise ValueError("arm base separation must stay inside the compact mount range")
        if not 0.0 <= self.arm_inward_yaw_rad < math.pi / 2.0:
            raise ValueError("arm inward yaw must be in [0, pi/2)")
        if self.camera_center_z_m <= WAFFLE_TOP_LOCAL_Z_M:
            raise ValueError("camera center must be above the TurtleBot3 top plane")
        if not 0.0 <= self.camera_down_tilt_rad < math.pi / 2.0:
            raise ValueError("camera down tilt must be in [0, pi/2)")
        if len(self.box_center_xy_m) != 2:
            raise ValueError("box center must contain XY")


def _yaw_quaternion(yaw_rad: float) -> list[float]:
    return [math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)]


def build_compact_mobile_spec(
    config: CompactMobileConfig | None = None,
    *,
    model_path: Path | str | None = None,
    grippers: str = "stock",
) -> tuple[mujoco.MjSpec, Path]:
    pgripper_sides = selected_sides(grippers)
    resolved = config or CompactMobileConfig()
    resolved.validate()
    source = resolve_so101_model(model_path)
    _validate_source_contract(mujoco.MjSpec.from_file(str(source)), source)

    spec = build_waffle_pi_spec()
    spec.modelname = "dapier_compact_mobile_dual_so101"
    spec.delete(spec.body("tb3_camera_link"))
    spec.delete(spec.body("tb3_base_scan"))
    base_link = spec.body("tb3_base_link")

    base_link.add_geom(
        name="compact_mount_plate",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[resolved.mount_x_m, 0.0, WAFFLE_TOP_LOCAL_Z_M + 0.004],
        size=[0.065, 0.125, 0.004],
        mass=0.55,
        contype=1,
        conaffinity=1,
        rgba=[0.72, 0.72, 0.70, 1.0],
    )
    mast_height = resolved.camera_center_z_m - WAFFLE_TOP_LOCAL_Z_M - 0.025
    base_link.add_geom(
        name="compact_camera_mast",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[
            resolved.mount_x_m,
            0.0,
            WAFFLE_TOP_LOCAL_Z_M + mast_height / 2.0,
        ],
        size=[0.012, 0.016, mast_height / 2.0],
        mass=0.30,
        contype=1,
        conaffinity=1,
        rgba=[0.07, 0.07, 0.07, 1.0],
    )
    _add_workspace_depth_camera(
        base_link,
        center_m=(resolved.mount_x_m, 0.0, resolved.camera_center_z_m),
        down_tilt_rad=resolved.camera_down_tilt_rad,
    )
    _add_front_slam_depth_camera(base_link)

    half_separation = resolved.arm_base_separation_m / 2.0
    mounts = (
        ("left", half_separation, -resolved.arm_inward_yaw_rad),
        ("right", -half_separation, resolved.arm_inward_yaw_rad),
    )
    for side, y_position, yaw_rad in mounts:
        frame = base_link.add_frame(
            name=f"{side}_compact_arm_mount",
            pos=[resolved.mount_x_m, y_position, WAFFLE_TOP_LOCAL_Z_M + 0.008],
            quat=_yaw_quaternion(yaw_rad),
        )
        arm = mujoco.MjSpec.from_file(str(source))
        if side in pgripper_sides:
            replace_gripper(arm)
        spec.attach(
            arm,
            prefix=f"{side}_",
            frame=frame,
        )

    _add_gripper_cameras(spec)
    box_config = BoxShoeSceneConfig(
        box_center_xy_m=resolved.box_center_xy_m,
        box_yaw_deg=resolved.box_yaw_deg,
    )
    _add_box(spec, box_config)
    _add_cuboid_shoe(spec, box_config)
    return spec, source


def build_compact_mobile_model(
    config: CompactMobileConfig | None = None,
    *,
    model_path: Path | str | None = None,
    grippers: str = "stock",
) -> tuple[mujoco.MjModel, Path]:
    resolved = config or CompactMobileConfig()
    spec, source = build_compact_mobile_spec(resolved, model_path=model_path, grippers=grippers)
    model = spec.compile()
    _orient_wrist_cameras_to_box(model, resolved)
    return model, source


def _orient_wrist_cameras_to_box(
    model: mujoco.MjModel,
    config: CompactMobileConfig,
) -> None:
    """Fix each wrist-camera mount so its home view points at the box."""

    data = mujoco.MjData(model)
    apply_control_as_pose(model, data, home_action(model, COMPACT_HOME_ACTION))
    target = np.asarray((*config.box_center_xy_m, 0.08), dtype=np.float64)
    world_up = np.asarray((0.0, 0.0, 1.0), dtype=np.float64)
    for name in ("left_gripper_camera", "right_gripper_camera"):
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        parent_id = int(model.cam_bodyid[camera_id])
        world_from_parent = data.xmat[parent_id].reshape(3, 3)
        parent_position = data.xpos[parent_id]
        approach = target - parent_position
        horizontal_approach = approach.copy()
        horizontal_approach[2] = 0.0
        horizontal_approach /= np.linalg.norm(horizontal_approach)
        camera_position = (
            parent_position + 0.04 * world_up + 0.08 * horizontal_approach
        )
        model.cam_pos[camera_id] = world_from_parent.T @ (
            camera_position - parent_position
        )
        forward = target - camera_position
        forward /= np.linalg.norm(forward)
        image_up = world_up - forward * float(np.dot(world_up, forward))
        image_up /= np.linalg.norm(image_up)
        camera_z = -forward
        camera_x = np.cross(image_up, camera_z)
        camera_x /= np.linalg.norm(camera_x)
        camera_y = np.cross(camera_z, camera_x)
        world_from_camera = np.column_stack((camera_x, camera_y, camera_z))
        parent_from_camera = world_from_parent.T @ world_from_camera
        quaternion = np.empty(4, dtype=np.float64)
        mujoco.mju_mat2Quat(quaternion, parent_from_camera.ravel())
        model.cam_quat[camera_id] = quaternion


def create_compact_mobile_data(model: mujoco.MjModel) -> mujoco.MjData:
    data = mujoco.MjData(model)
    apply_control_as_pose(model, data, home_action(model, COMPACT_HOME_ACTION))
    return data


def compact_mobile_measurements_mm(model: mujoco.MjModel) -> dict[str, object]:
    """Return inspectable simulation layout measurements, never hardware truth."""

    data = create_compact_mobile_data(model)
    positions = {}
    for name in ("left_base", "right_base", "box_fixture"):
        object_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        positions[name] = data.xpos[object_id].copy()
    for name in (
        "front_slam_depth_camera",
        "workspace_depth_camera",
        "left_gripper_camera",
        "right_gripper_camera",
    ):
        object_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        positions[name] = data.cam_xpos[object_id].copy()

    def distance(first: str, second: str) -> float:
        return round(float(np.linalg.norm(positions[first] - positions[second])) * 1000, 1)

    return {
        "position_world_mm": {
            name: tuple(round(float(value) * 1000, 1) for value in position)
            for name, position in positions.items()
        },
        "distance_mm": {
            "arm_base_to_arm_base": distance("left_base", "right_base"),
            "top_rgbd_to_left_arm_base": distance(
                "workspace_depth_camera", "left_base"
            ),
            "top_rgbd_to_right_arm_base": distance(
                "workspace_depth_camera", "right_base"
            ),
            "top_rgbd_to_front_rgbd": distance(
                "workspace_depth_camera", "front_slam_depth_camera"
            ),
            "top_rgbd_to_box_center": distance(
                "workspace_depth_camera", "box_fixture"
            ),
            "front_rgbd_to_box_center": distance(
                "front_slam_depth_camera", "box_fixture"
            ),
            "left_wrist_rgb_to_box_center": distance(
                "left_gripper_camera", "box_fixture"
            ),
            "right_wrist_rgb_to_box_center": distance(
                "right_gripper_camera", "box_fixture"
            ),
            "left_arm_base_to_box_center": distance("left_base", "box_fixture"),
            "right_arm_base_to_box_center": distance("right_base", "box_fixture"),
        },
        "simulation_values_only": True,
    }


def compact_mobile_contract(
    model: mujoco.MjModel,
    config: CompactMobileConfig,
) -> dict[str, object]:
    config.validate()
    camera_names = (
        "front_slam_depth_camera",
        "workspace_depth_camera",
        "left_gripper_camera",
        "right_gripper_camera",
    )
    if any(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) < 0
        for name in camera_names
    ):
        raise RuntimeError("compact mobile camera contract is incomplete")
    actuator_names = tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index) or ""
        for index in range(model.nu)
    )
    if actuator_names != ACTION_NAMES:
        raise RuntimeError(f"unexpected compact mobile action order: {actuator_names}")
    return {
        "schema_version": "dapier.compact-mobile-layout.v0.1",
        "layout": "TURTLEBOT3_CENTERED_RGBD_COMPACT_DUAL_SO101",
        "config": asdict(config),
        "camera_count": len(camera_names),
        "camera_roles": camera_names,
        "action_names": actuator_names,
        "home_action_rad": home_action(model, COMPACT_HOME_ACTION),
        "camera_centerline_aligned": True,
        "arm_mount_measurement_required": True,
        "simulator_truth_for_runtime_forbidden": True,
        "hardware_execution": False,
    }


__all__ = [
    "COMPACT_HOME_ACTION",
    "CompactMobileConfig",
    "build_compact_mobile_model",
    "build_compact_mobile_spec",
    "compact_mobile_measurements_mm",
    "compact_mobile_contract",
    "create_compact_mobile_data",
]
