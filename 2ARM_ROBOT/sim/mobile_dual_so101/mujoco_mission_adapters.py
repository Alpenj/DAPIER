#!/usr/bin/env python3
"""Simulation-only MuJoCo adapters for mobile shoe-mission capabilities.

The mobility backend is deliberately labelled kinematic.  Existing Waffle
collision proxies were authored for a stationary model and are not yet a
validated wheel-contact dynamics model.  Left/right wheel state is still
maintained internally so turns, limits, and later slip injection have an
explicit differential-drive boundary.
"""

from __future__ import annotations

import math
import time
from typing import Callable

import mujoco
import numpy as np

from mission_core import MissionLocation
from mission_modules.camera import (
    CameraFrame,
    CameraModality,
    CameraRigHealth,
    CameraRole,
    CameraStreamHealth,
    MultiCameraFrameSet,
)
from mission_modules.mobility import MobilityStatus, NavigationRequest
from mission_modules.types import Pose2D
from mobile_dual_so101 import build_spec as build_mobile_spec
from shoe_task import ShoeTaskConfig, _add_primitive_shoe


MOBILE_BASE_FREE_JOINT = "mobile_base_free"
LEFT_WHEEL_JOINT = "tb3_wheel_left_joint"
RIGHT_WHEEL_JOINT = "tb3_wheel_right_joint"
FRONT_CAMERA = "front_depth_camera"
LEFT_GRIPPER_CAMERA = "left_gripper_camera"
RIGHT_GRIPPER_CAMERA = "right_gripper_camera"
CAMERA_NAMES = {
    CameraRole.FRONT_RGBD: FRONT_CAMERA,
    CameraRole.LEFT_GRIPPER_RGB: LEFT_GRIPPER_CAMERA,
    CameraRole.RIGHT_GRIPPER_RGB: RIGHT_GRIPPER_CAMERA,
}

WHEEL_RADIUS_M = 0.033
WHEEL_SEPARATION_M = 0.288
MAX_LINEAR_VELOCITY_MPS = 0.18
MAX_ANGULAR_VELOCITY_RADPS = 1.2
MAX_WHEEL_VELOCITY_RADPS = 6.5


def _yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    half = yaw_rad / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _quaternion_yaw(quaternion: np.ndarray) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _object_id(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    name: str,
) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise RuntimeError(f"MuJoCo object is missing: {name}")
    return object_id


def _reuse_calibrated_gripper_cameras(spec: mujoco.MjSpec) -> None:
    """Expose the source MJCF wrist cameras as the two gripper RGB roles.

    The pinned SO-101 MJCF already contains a camera calibrated for the real
    InnoMaker 130-degree wrist camera. Adding another provisional camera per
    arm produced five MuJoCo cameras for a three-camera mission contract.
    """

    for side in ("left", "right"):
        camera = spec.camera(f"{side}_wrist_cam")
        if camera is None:
            raise RuntimeError(f"calibrated {side} wrist camera is missing")
        camera.name = f"{side}_gripper_camera"


def build_mobile_shoe_mission_model(
    config: ShoeTaskConfig | None = None,
) -> mujoco.MjModel:
    """Build a separate mobile model without changing stationary baselines."""

    resolved = config or ShoeTaskConfig()
    resolved.validate()
    spec, _ = build_mobile_spec(
        arm_mount_height_m=resolved.arm_mount_height_m,
        arm_mount_separation_m=resolved.arm_mount_separation_m,
        mount_layout=resolved.mount_layout,
    )
    base_root = spec.body("tb3_base_footprint")
    base_root.add_freejoint(name=MOBILE_BASE_FREE_JOINT)
    _reuse_calibrated_gripper_cameras(spec)
    _add_primitive_shoe(
        spec,
        position_m=resolved.shoe_position_m,
        yaw_rad=resolved.shoe_yaw_rad,
    )
    return spec.compile()


class MuJoCoMobilityAdapter:
    """Bounded differential-drive controller with kinematic MuJoCo state."""

    simulation_only = True
    backend_kind = "SIM_KINEMATIC"

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        self._base_body_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "tb3_base_link",
        )
        free_joint_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            MOBILE_BASE_FREE_JOINT,
        )
        if model.jnt_type[free_joint_id] != mujoco.mjtJoint.mjJNT_FREE:
            raise RuntimeError("mobile base joint must be a MuJoCo free joint")
        self._base_qpos_address = int(model.jnt_qposadr[free_joint_id])
        self._base_dof_address = int(model.jnt_dofadr[free_joint_id])
        self._wheel_joint_ids = (
            _object_id(model, mujoco.mjtObj.mjOBJ_JOINT, LEFT_WHEEL_JOINT),
            _object_id(model, mujoco.mjtObj.mjOBJ_JOINT, RIGHT_WHEEL_JOINT),
        )
        self._wheel_qpos_addresses = tuple(
            int(model.jnt_qposadr[joint_id]) for joint_id in self._wheel_joint_ids
        )
        self._wheel_dof_addresses = tuple(
            int(model.jnt_dofadr[joint_id]) for joint_id in self._wheel_joint_ids
        )
        self._active_request: NavigationRequest | None = None
        self._request_started_s = 0.0
        self._goal_reached = False
        self._watchdog_ok = True
        self._linear_velocity_mps = 0.0
        self._angular_velocity_radps = 0.0
        self._last_wheel_target_radps = (0.0, 0.0)
        self._safe_stop_reason = ""
        mujoco.mj_forward(self.model, self.data)

    def request_navigation(self, request: NavigationRequest) -> None:
        request.validate()
        if not self._watchdog_ok:
            raise RuntimeError("mobility watchdog is latched; adapter reset is required")
        self._active_request = request
        self._request_started_s = float(self.data.time)
        self._goal_reached = False
        self._safe_stop_reason = ""

    def read_status(self) -> MobilityStatus:
        pose = self._pose()
        status = MobilityStatus(
            pose_map=pose,
            linear_velocity_mps=self._linear_velocity_mps,
            angular_velocity_radps=self._angular_velocity_radps,
            active_goal=(
                self._active_request.goal if self._active_request is not None else None
            ),
            goal_reached=self._goal_reached,
            watchdog_ok=self._watchdog_ok,
            observation_age_ms=0.0,
        )
        status.validate()
        return status

    def safe_stop(self, reason: str) -> None:
        if not reason.strip() or len(reason) > 500:
            raise ValueError("safe-stop reason must contain 1 to 500 characters")
        self._linear_velocity_mps = 0.0
        self._angular_velocity_radps = 0.0
        self._last_wheel_target_radps = (0.0, 0.0)
        self._active_request = None
        self._goal_reached = False
        self._watchdog_ok = False
        self._safe_stop_reason = reason
        self._write_velocity_state(0.0, 0.0)
        mujoco.mj_forward(self.model, self.data)

    def advance(self, steps: int = 1) -> None:
        if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
            raise ValueError("steps must be a positive integer")
        dt_s = float(self.model.opt.timestep)
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise RuntimeError("MuJoCo timestep must be finite and positive")
        for _ in range(steps):
            request = self._active_request
            if request is None or self._goal_reached or not self._watchdog_ok:
                linear_velocity = 0.0
                angular_velocity = 0.0
            elif float(self.data.time) - self._request_started_s > request.timeout_s:
                self.safe_stop("navigation request timed out")
                break
            else:
                linear_velocity, angular_velocity = self._control(request)
            self._integrate(linear_velocity, angular_velocity, dt_s)

    def _pose(self) -> Pose2D:
        position = self.data.xpos[self._base_body_id]
        yaw = _quaternion_yaw(self.data.xquat[self._base_body_id])
        pose = Pose2D(float(position[0]), float(position[1]), yaw)
        pose.validate()
        return pose

    def _control(self, request: NavigationRequest) -> tuple[float, float]:
        pose = self._pose()
        dx = request.target_pose.x_m - pose.x_m
        dy = request.target_pose.y_m - pose.y_m
        distance = math.hypot(dx, dy)
        if distance <= request.position_tolerance_m:
            yaw_error = _wrap_angle(request.target_pose.yaw_rad - pose.yaw_rad)
            if abs(yaw_error) <= request.yaw_tolerance_rad:
                self._goal_reached = True
                return 0.0, 0.0
            return 0.0, max(
                -MAX_ANGULAR_VELOCITY_RADPS,
                min(MAX_ANGULAR_VELOCITY_RADPS, 2.5 * yaw_error),
            )
        heading = math.atan2(dy, dx)
        heading_error = _wrap_angle(heading - pose.yaw_rad)
        angular_velocity = max(
            -MAX_ANGULAR_VELOCITY_RADPS,
            min(MAX_ANGULAR_VELOCITY_RADPS, 2.5 * heading_error),
        )
        linear_velocity = 0.0
        if abs(heading_error) < 0.35:
            linear_velocity = min(MAX_LINEAR_VELOCITY_MPS, 1.2 * distance)
        return linear_velocity, angular_velocity

    def _integrate(
        self,
        linear_velocity_mps: float,
        angular_velocity_radps: float,
        dt_s: float,
    ) -> None:
        left_radps = (
            linear_velocity_mps
            - angular_velocity_radps * WHEEL_SEPARATION_M / 2.0
        ) / WHEEL_RADIUS_M
        right_radps = (
            linear_velocity_mps
            + angular_velocity_radps * WHEEL_SEPARATION_M / 2.0
        ) / WHEEL_RADIUS_M
        peak = max(abs(left_radps), abs(right_radps), MAX_WHEEL_VELOCITY_RADPS)
        if peak > MAX_WHEEL_VELOCITY_RADPS:
            scale = MAX_WHEEL_VELOCITY_RADPS / peak
            left_radps *= scale
            right_radps *= scale
            linear_velocity_mps = (
                WHEEL_RADIUS_M * (left_radps + right_radps) / 2.0
            )
            angular_velocity_radps = (
                WHEEL_RADIUS_M
                * (right_radps - left_radps)
                / WHEEL_SEPARATION_M
            )
        pose = self._pose()
        yaw_mid = pose.yaw_rad + angular_velocity_radps * dt_s / 2.0
        x_m = pose.x_m + linear_velocity_mps * math.cos(yaw_mid) * dt_s
        y_m = pose.y_m + linear_velocity_mps * math.sin(yaw_mid) * dt_s
        yaw_rad = _wrap_angle(pose.yaw_rad + angular_velocity_radps * dt_s)
        base_qpos = self.data.qpos[
            self._base_qpos_address : self._base_qpos_address + 7
        ]
        base_qpos[0] = x_m
        base_qpos[1] = y_m
        base_qpos[3:7] = _yaw_quaternion(yaw_rad)
        for address, wheel_velocity in zip(
            self._wheel_qpos_addresses,
            (left_radps, right_radps),
        ):
            self.data.qpos[address] += wheel_velocity * dt_s
        self._write_velocity_state(linear_velocity_mps, angular_velocity_radps)
        for address, wheel_velocity in zip(
            self._wheel_dof_addresses,
            (left_radps, right_radps),
        ):
            self.data.qvel[address] = wheel_velocity
        self._linear_velocity_mps = linear_velocity_mps
        self._angular_velocity_radps = angular_velocity_radps
        self._last_wheel_target_radps = (left_radps, right_radps)
        self.data.time = float(self.data.time) + dt_s
        mujoco.mj_forward(self.model, self.data)

    def _write_velocity_state(
        self,
        linear_velocity_mps: float,
        angular_velocity_radps: float,
    ) -> None:
        pose = self._pose()
        velocity = self.data.qvel[
            self._base_dof_address : self._base_dof_address + 6
        ]
        velocity[:] = 0.0
        velocity[0] = linear_velocity_mps * math.cos(pose.yaw_rad)
        velocity[1] = linear_velocity_mps * math.sin(pose.yaw_rad)
        velocity[5] = angular_velocity_radps


class MuJoCoMultiCameraAdapter:
    """Render synchronized front RGB-D and gripper RGB observations."""

    simulation_only = True

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        width: int = 64,
        height: int = 48,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("camera width and height must be positive")
        self.model = model
        self.data = data
        self.width = width
        self.height = height
        self._clock_ns = clock_ns
        self._sequence = 0
        self._last_received_ns: int | None = None
        for camera_name in CAMERA_NAMES.values():
            _object_id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        self._renderer = mujoco.Renderer(model, height=height, width=width)

    def capture(self) -> MultiCameraFrameSet:
        sim_timestamp_ns = max(0, int(round(float(self.data.time) * 1e9)))
        received_ns = self._clock_ns()
        if isinstance(received_ns, bool) or not isinstance(received_ns, int):
            raise ValueError("camera clock must return an integer nanosecond value")
        frames: list[CameraFrame] = []
        for role in CameraRole:
            camera_name = CAMERA_NAMES[role]
            rgb = self._render_rgb(camera_name)
            if role == CameraRole.FRONT_RGBD:
                depth = self._render_depth(camera_name)
                frame = CameraFrame(
                    role=role,
                    modality=CameraModality.RGBD,
                    frame_id=self._sequence,
                    optical_frame="front_depth_optical_frame",
                    calibration_id="mujoco-front-rgbd-v1",
                    width=self.width,
                    height=self.height,
                    rgb_timestamp_ns=sim_timestamp_ns,
                    depth_timestamp_ns=sim_timestamp_ns,
                    received_monotonic_ns=received_ns,
                    rgb=rgb.tobytes(order="C"),
                    depth_m_le_f32=depth.astype("<f4", copy=False).tobytes(order="C"),
                )
            else:
                side = "left" if role == CameraRole.LEFT_GRIPPER_RGB else "right"
                frame = CameraFrame(
                    role=role,
                    modality=CameraModality.RGB,
                    frame_id=self._sequence,
                    optical_frame=f"{side}_gripper_camera",
                    calibration_id=f"mujoco-{side}-gripper-rgb-v1",
                    width=self.width,
                    height=self.height,
                    rgb_timestamp_ns=sim_timestamp_ns,
                    received_monotonic_ns=received_ns,
                    rgb=rgb.tobytes(order="C"),
                )
            frame.validate()
            frames.append(frame)
        frame_set = MultiCameraFrameSet(
            sequence=self._sequence,
            frames=tuple(frames),
        )
        frame_set.validate()
        self._sequence += 1
        self._last_received_ns = received_ns
        return frame_set

    def read_health(self) -> CameraRigHealth:
        now_ns = self._clock_ns()
        last_age_ms = (
            1_000_000_000.0
            if self._last_received_ns is None
            else max(0.0, (now_ns - self._last_received_ns) / 1_000_000.0)
        )
        streams = tuple(
            CameraStreamHealth(
                role=role,
                enabled=True,
                online=self._last_received_ns is not None,
                calibration_loaded=True,
                dropped_frames=0,
                last_frame_age_ms=last_age_ms,
                time_sync_error_ms=0.0,
                error_code="" if self._last_received_ns is not None else "no_frame_yet",
            )
            for role in CameraRole
        )
        health = CameraRigHealth(streams=streams)
        health.validate()
        return health

    def close(self) -> None:
        self._renderer.close()

    def _render_rgb(self, camera_name: str) -> np.ndarray:
        self._renderer.disable_depth_rendering()
        self._renderer.update_scene(self.data, camera=camera_name)
        return np.asarray(self._renderer.render(), dtype=np.uint8).copy()

    def _render_depth(self, camera_name: str) -> np.ndarray:
        self._renderer.enable_depth_rendering()
        self._renderer.update_scene(self.data, camera=camera_name)
        depth = np.asarray(self._renderer.render(), dtype=np.float32).copy()
        self._renderer.disable_depth_rendering()
        return depth


__all__ = [
    "CAMERA_NAMES",
    "MAX_ANGULAR_VELOCITY_RADPS",
    "MAX_LINEAR_VELOCITY_MPS",
    "MAX_WHEEL_VELOCITY_RADPS",
    "MOBILE_BASE_FREE_JOINT",
    "MuJoCoMobilityAdapter",
    "MuJoCoMultiCameraAdapter",
    "WHEEL_RADIUS_M",
    "WHEEL_SEPARATION_M",
    "build_mobile_shoe_mission_model",
]
