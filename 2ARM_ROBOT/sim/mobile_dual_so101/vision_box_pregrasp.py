#!/usr/bin/env python3
"""Depth-derived box pose and guarded right-arm pre-grasp planning (SIM-only)."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Sequence

import mujoco
import numpy as np

from box_shoe_scene import BOX_GEOM_NAMES
from box_shoe_physics_demo import _matching_contacts
from collision_guard import check_bimanual_path
from compact_mobile_dual_so101 import (
    COMPACT_HOME_ACTION,
    CompactMobileConfig,
    build_compact_mobile_model,
    create_compact_mobile_data,
)
from mobile_dual_so101 import apply_control_as_pose
from physics_ik import solve_bimanual_position_ik


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    base_from_optical: tuple[float, ...]

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.fx <= 0 or self.fy <= 0:
            raise ValueError("camera intrinsics must be positive")
        if not (0 <= self.cx < self.width and 0 <= self.cy < self.height):
            raise ValueError("camera principal point must be inside the image")
        values = (self.fx, self.fy, self.cx, self.cy, *self.base_from_optical)
        if len(self.base_from_optical) != 16 or not all(map(math.isfinite, values)):
            raise ValueError(
                "camera calibration must contain finite intrinsics and a 4x4 transform"
            )
        transform = np.asarray(self.base_from_optical).reshape(4, 4)
        rotation = transform[:3, :3]
        if not np.allclose(transform[3], (0, 0, 0, 1), atol=1e-8):
            raise ValueError("camera transform must be homogeneous")
        orthonormal = np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        right_handed = math.isclose(
            float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6
        )
        if not orthonormal or not right_handed:
            raise ValueError("camera rotation must be right-handed and orthonormal")


@dataclass(frozen=True)
class DepthBoxDetectorConfig:
    workspace_x_m: tuple[float, float] = (0.20, 0.65)
    workspace_y_m: tuple[float, float] = (-0.25, 0.25)
    top_height_m: tuple[float, float] = (0.08, 0.13)
    box_size_m: tuple[float, float, float] = (0.282, 0.210, 0.105)
    lid_side_flap_m: float = 0.040
    lid_front_flap_m: float = 0.055
    dimension_tolerance_m: float = 0.035
    minimum_points: int = 800
    pixel_stride: int = 2

    def validate(self) -> None:
        bounds = (self.workspace_x_m, self.workspace_y_m, self.top_height_m)
        if any(
            len(pair) != 2
            or not all(math.isfinite(value) for value in pair)
            or pair[0] >= pair[1]
            for pair in bounds
        ):
            raise ValueError("detector bounds must be increasing pairs")
        if len(self.box_size_m) != 3:
            raise ValueError("box_size_m must contain three values")
        dimensions = (
            *self.box_size_m,
            self.lid_side_flap_m,
            self.lid_front_flap_m,
            self.dimension_tolerance_m,
        )
        if not all(math.isfinite(value) and value > 0 for value in dimensions):
            raise ValueError("box dimensions and tolerance must be positive")
        if self.pixel_stride <= 0 or self.minimum_points <= 0:
            raise ValueError("detector sampling values must be positive")


@dataclass(frozen=True)
class BoxPoseEstimate:
    top_center_base_m: tuple[float, float, float]
    long_axis_base_xy: tuple[float, float]
    away_axis_base_xy: tuple[float, float]
    yaw_deg: float
    observed_footprint_m: tuple[float, float]
    shape_model: str
    points_used: int
    confidence: float
    source: str = "depth_points_only"
    simulator_truth_used: bool = False
    hardware_execution: bool = False


@dataclass(frozen=True)
class RightPregraspPlan:
    box: BoxPoseEstimate
    right_front_flap_grasp_base_m: tuple[float, float, float]
    targets_base_m: tuple[tuple[float, float, float], ...]
    actions_rad: tuple[tuple[float, ...], ...]
    residuals_m: tuple[float, ...]
    contact_target_base_m: tuple[float, float, float]
    contact_open_action_rad: tuple[float, ...]
    contact_closed_action_rad: tuple[float, ...]
    contact_residual_m: float
    front_flap_contact_count: int
    minimum_clearance_m: float
    accepted: bool
    hardware_execution: bool = False


@dataclass(frozen=True)
class RightWristServoResult:
    desired_edge_row_px: float
    observed_edge_row_px: float
    corrected_edge_row_px: float
    initial_error_px: float
    corrected_error_px: float
    correction_base_x_m: float
    action_rad: tuple[float, ...]
    accepted: bool
    source: str = "right_wrist_rgb_front_flap_edge"
    simulator_truth_used: bool = False
    hardware_execution: bool = False


def calibration_from_mujoco(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_name: str,
    *,
    width: int,
    height: int,
) -> CameraCalibration:
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if camera_id < 0:
        raise ValueError(f"missing camera: {camera_name}")
    focal = 0.5 * height / math.tan(math.radians(float(model.cam_fovy[camera_id])) / 2)
    camera_rotation = data.cam_xmat[camera_id].reshape(3, 3)
    optical_rotation = np.column_stack(
        (camera_rotation[:, 0], -camera_rotation[:, 1], -camera_rotation[:, 2])
    )
    transform = np.eye(4)
    transform[:3, :3] = optical_rotation
    transform[:3, 3] = data.cam_xpos[camera_id]
    result = CameraCalibration(
        width, height, focal, focal, (width - 1) / 2, (height - 1) / 2,
        tuple(float(value) for value in transform.ravel()),
    )
    result.validate()
    return result


def render_metric_depth(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_name: str,
    *,
    width: int,
    height: int,
) -> np.ndarray:
    with mujoco.Renderer(model, width=width, height=height) as renderer:
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=camera_name)
        return np.asarray(renderer.render(), dtype=np.float32).copy()


def render_rgb(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_name: str,
    *,
    width: int = 352,
    height: int = 288,
) -> np.ndarray:
    if width <= 0 or height <= 0:
        raise ValueError("RGB dimensions must be positive")
    with mujoco.Renderer(model, width=width, height=height) as renderer:
        renderer.update_scene(data, camera=camera_name)
        return np.asarray(renderer.render(), dtype=np.uint8).copy()


def detect_front_flap_edge_row(rgb: np.ndarray) -> float:
    """Detect the cardboard-to-floor edge away from the gripper occlusion."""

    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("wrist RGB must be an HxWx3 uint8 image")
    red, green, blue = (image[..., index].astype(np.float32) for index in range(3))
    cardboard = (
        (red > 130.0)
        & (red > 1.25 * green)
        & (green > 1.25 * blue)
    )
    start = max(0, round(image.shape[1] * 0.06))
    stop = max(start + 1, round(image.shape[1] * 0.40))
    boundary_rows = []
    for column in range(start, stop):
        rows = np.flatnonzero(cardboard[:, column])
        if len(rows) >= 20:
            boundary_rows.append(float(np.quantile(rows, 0.99)))
    if len(boundary_rows) < max(12, (stop - start) // 3):
        raise ValueError("front flap edge is not sufficiently visible in wrist RGB")
    return float(np.median(boundary_rows))


def _wrist_edge_row_for_action(
    model: mujoco.MjModel,
    action_rad: Sequence[float],
) -> float:
    data = create_compact_mobile_data(model)
    apply_control_as_pose(model, data, action_rad)
    return detect_front_flap_edge_row(
        render_rgb(model, data, "right_gripper_camera")
    )


def refine_right_pregrasp_with_wrist_rgb(
    model: mujoco.MjModel,
    start_action_rad: Sequence[float],
    target_base_m: Sequence[float],
    desired_edge_row_px: float,
    *,
    probe_base_x_m: float = 0.005,
    maximum_correction_m: float = 0.012,
) -> RightWristServoResult:
    """Apply one bounded SIM-only image-edge correction along base X."""

    target = np.asarray(target_base_m, dtype=np.float64)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("right wrist servo target must contain three finite values")
    if not math.isfinite(desired_edge_row_px):
        raise ValueError("desired edge row must be finite")
    if not 0.0 < probe_base_x_m <= 0.01:
        raise ValueError("wrist servo probe must be inside (0, 0.01] m")
    if not 0.0 < maximum_correction_m <= 0.02:
        raise ValueError("wrist servo correction limit must be inside (0, 0.02] m")

    observed = _wrist_edge_row_for_action(model, start_action_rad)
    probe_target = target.copy()
    probe_target[0] += probe_base_x_m
    probe = solve_bimanual_position_ik(
        model,
        start_action_rad,
        {"right": probe_target},
    )
    if not probe.converged:
        raise ValueError("right wrist servo probe IK did not converge")
    probe_row = _wrist_edge_row_for_action(model, probe.action_rad)
    pixels_per_metre = (probe_row - observed) / probe_base_x_m
    if not math.isfinite(pixels_per_metre) or abs(pixels_per_metre) < 100.0:
        raise ValueError("right wrist edge has insufficient motion observability")

    correction = (desired_edge_row_px - observed) / pixels_per_metre
    correction = max(-maximum_correction_m, min(maximum_correction_m, correction))
    corrected_target = target.copy()
    corrected_target[0] += correction
    corrected = solve_bimanual_position_ik(
        model,
        start_action_rad,
        {"right": corrected_target},
    )
    if not corrected.converged:
        raise ValueError("right wrist corrected IK did not converge")
    obstacles = tuple(
        name
        for name in BOX_GEOM_NAMES
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0
        and int(model.geom_contype[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, name
        )]) != 0
    )
    guard = check_bimanual_path(
        model,
        start_action_rad,
        corrected.action_rad,
        required_clearance_m=0.005,
        obstacle_geom_names=obstacles,
    )
    if not guard.safe:
        raise ValueError(f"right wrist correction rejected: {guard.reason}")
    corrected_row = _wrist_edge_row_for_action(model, corrected.action_rad)
    return RightWristServoResult(
        desired_edge_row_px=float(desired_edge_row_px),
        observed_edge_row_px=observed,
        corrected_edge_row_px=corrected_row,
        initial_error_px=float(desired_edge_row_px - observed),
        corrected_error_px=float(desired_edge_row_px - corrected_row),
        correction_base_x_m=float(correction),
        action_rad=tuple(float(value) for value in corrected.action_rad),
        accepted=True,
    )


def estimate_box_pose_from_depth(
    depth_m: np.ndarray,
    calibration: CameraCalibration,
    config: DepthBoxDetectorConfig = DepthBoxDetectorConfig(),
) -> BoxPoseEstimate:
    """Fit the known box/lid footprint without segmentation IDs or body poses."""

    calibration.validate()
    config.validate()
    depth = np.asarray(depth_m, dtype=np.float64)
    if depth.shape != (calibration.height, calibration.width):
        raise ValueError("depth shape does not match camera calibration")
    stride = config.pixel_stride
    rows, columns = np.mgrid[
        0 : calibration.height : stride, 0 : calibration.width : stride
    ]
    z = depth[::stride, ::stride]
    optical = np.stack(
        (
            (columns - calibration.cx) * z / calibration.fx,
            (rows - calibration.cy) * z / calibration.fy,
            z,
        ),
        axis=-1,
    )
    transform = np.asarray(calibration.base_from_optical).reshape(4, 4)
    points = optical @ transform[:3, :3].T + transform[:3, 3]
    x_bounds, y_bounds, z_bounds = (
        config.workspace_x_m,
        config.workspace_y_m,
        config.top_height_m,
    )
    mask = (
        np.isfinite(z)
        & (z > 0.05)
        & (z < 3.0)
        & (points[..., 0] >= x_bounds[0])
        & (points[..., 0] <= x_bounds[1])
        & (points[..., 1] >= y_bounds[0])
        & (points[..., 1] <= y_bounds[1])
        & (points[..., 2] >= z_bounds[0])
        & (points[..., 2] <= z_bounds[1])
    )
    selected = points[mask]
    if len(selected) < config.minimum_points:
        raise ValueError("not enough box-height depth points")
    xy = selected[:, :2]

    # The lid has flaps, so PCA alone is biased. A small orientation sweep fits
    # the minimum-area rectangle while keeping this dependency-free on OpenCV.
    best: tuple[float, np.ndarray, np.ndarray, np.ndarray] | None = None
    for angle in np.linspace(0.0, math.pi, 181, endpoint=False):
        axes = np.asarray(
            ((math.cos(angle), math.sin(angle)), (-math.sin(angle), math.cos(angle)))
        )
        projected = xy @ axes.T
        lower, upper = np.quantile(projected, (0.005, 0.995), axis=0)
        area = float(np.prod(upper - lower))
        if best is None or area < best[0]:
            best = area, axes, lower, upper
    assert best is not None
    _, axes, lower, upper = best
    spans = upper - lower
    long_index = int(np.argmax(spans))
    short_index = 1 - long_index
    long_axis = axes[long_index]
    away_axis = axes[short_index]
    rough_center = np.median(xy, axis=0)
    if float(np.dot(away_axis, rough_center)) < 0.0:
        away_axis = -away_axis

    long_projection = xy @ long_axis
    short_projection = xy @ away_axis
    long_low, long_high = np.quantile(long_projection, (0.005, 0.995))
    short_low, short_high = np.quantile(short_projection, (0.005, 0.995))
    observed = (float(long_high - long_low), float(short_high - short_low))
    body_long, body_short, _ = config.box_size_m
    templates = {
        "closed_box": (body_long, body_short),
        "lid_with_flaps": (
            body_long + 2 * config.lid_side_flap_m,
            body_short + config.lid_front_flap_m,
        ),
    }
    shape_model, expected = min(
        templates.items(),
        key=lambda item: sum(
            (a - b) ** 2 for a, b in zip(observed, item[1], strict=True)
        ),
    )
    errors = tuple(abs(a - b) for a, b in zip(observed, expected, strict=True))
    if max(errors) > config.dimension_tolerance_m:
        raise ValueError("depth footprint does not match the configured box")

    long_center = (long_low + long_high) / 2
    if shape_model == "lid_with_flaps":
        short_center = short_high - body_short / 2
    else:
        short_center = (short_low + short_high) / 2
    center_xy = long_center * long_axis + short_center * away_axis
    top_z = float(np.median(selected[:, 2]))
    yaw = math.degrees(math.atan2(long_axis[1], long_axis[0]))
    while yaw < -90.0:
        yaw += 180.0
    while yaw >= 90.0:
        yaw -= 180.0
    confidence = max(0.0, 1.0 - max(errors) / config.dimension_tolerance_m)
    return BoxPoseEstimate(
        top_center_base_m=(float(center_xy[0]), float(center_xy[1]), top_z),
        long_axis_base_xy=tuple(float(value) for value in long_axis),
        away_axis_base_xy=tuple(float(value) for value in away_axis),
        yaw_deg=yaw,
        observed_footprint_m=observed,
        shape_model=shape_model,
        points_used=len(selected),
        confidence=confidence,
    )


def right_pregrasp_targets(
    box: BoxPoseEstimate,
    *,
    clearances_m: Sequence[float] = (0.05, 0.04, 0.03, 0.02),
    heights_above_flap_m: Sequence[float] = (0.095, 0.075, 0.055, 0.045),
    box_short_m: float = 0.210,
    front_flap_depth_m: float = 0.055,
    right_offset_m: float = 0.090,
) -> tuple[tuple[float, float, float], ...]:
    if (
        not clearances_m
        or len(clearances_m) != len(heights_above_flap_m)
        or any(
            value <= 0 or not math.isfinite(value)
            for value in (*clearances_m, *heights_above_flap_m)
        )
    ):
        raise ValueError("pre-grasp offsets must be paired, finite, and positive")
    if not all(
        math.isfinite(value) and value > 0
        for value in (
            box_short_m,
            front_flap_depth_m,
            right_offset_m,
        )
    ):
        raise ValueError("pre-grasp offsets and box size must be finite and positive")
    away = np.asarray(box.away_axis_base_xy)
    near = -away
    long_axis = np.asarray(box.long_axis_base_xy)
    right = long_axis if float(np.dot(long_axis, (0.0, -1.0))) >= 0 else -long_axis
    center = np.asarray(box.top_center_base_m[:2])
    flap_center = (
        center
        + near * (box_short_m / 2 + front_flap_depth_m / 2)
        + right * right_offset_m
    )
    targets = []
    for clearance, height in zip(
        clearances_m, heights_above_flap_m, strict=True
    ):
        xy = flap_center + near * clearance
        targets.append(
            (float(xy[0]), float(xy[1]), box.top_center_base_m[2] + height)
        )
    return tuple(targets)


def plan_right_pregrasp_from_depth(
    model: mujoco.MjModel,
    depth_m: np.ndarray,
    calibration: CameraCalibration,
    *,
    start_action_rad: Sequence[float] = COMPACT_HOME_ACTION,
    detector_config: DepthBoxDetectorConfig = DepthBoxDetectorConfig(),
) -> RightPregraspPlan:
    box = estimate_box_pose_from_depth(depth_m, calibration, detector_config)
    targets = right_pregrasp_targets(
        box,
        box_short_m=detector_config.box_size_m[1],
        front_flap_depth_m=detector_config.lid_front_flap_m,
    )
    near = -np.asarray(box.away_axis_base_xy)
    right = np.asarray(box.long_axis_base_xy)
    if float(np.dot(right, (0.0, -1.0))) < 0:
        right = -right
    grasp_xy = (
        np.asarray(box.top_center_base_m[:2])
        + near
        * (detector_config.box_size_m[1] / 2 + detector_config.lid_front_flap_m / 2)
        + right * 0.090
    )
    flap_grasp = (
        float(grasp_xy[0]),
        float(grasp_xy[1]),
        float(box.top_center_base_m[2]),
    )
    action = list(start_action_rad)
    action[11] = 1.2
    action = tuple(action)
    actions, residuals = [], []
    minimum_clearance = math.inf
    obstacles = tuple(
        name
        for name in BOX_GEOM_NAMES
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0
        and int(
            model.geom_contype[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            ]
        )
        != 0
    )
    for target in targets:
        ik = solve_bimanual_position_ik(model, action, {"right": target})
        if not ik.converged:
            raise ValueError(
                f"right pre-grasp IK did not converge: {ik.residual_m_by_side}"
            )
        assessment = check_bimanual_path(
            model,
            action,
            ik.action_rad,
            required_clearance_m=0.005,
            obstacle_geom_names=obstacles,
        )
        if not assessment.safe:
            raise ValueError(f"right pre-grasp path rejected: {assessment.reason}")
        action = ik.action_rad
        actions.append(action)
        residuals.append(ik.residual_m_by_side["right"])
        minimum_clearance = min(minimum_clearance, assessment.minimum_clearance_m)

    contact_target = tuple(
        float(value)
        for value in np.asarray(flap_grasp) + np.asarray((-0.004, 0.0, 0.005))
    )
    contact_ik = solve_bimanual_position_ik(
        model,
        action,
        {"right": contact_target},
        max_iterations=250,
    )
    if not contact_ik.converged:
        raise ValueError(
            f"right front-flap contact IK did not converge: "
            f"{contact_ik.residual_m_by_side}"
        )
    non_target_obstacles = tuple(
        name for name in obstacles if name != "box_lid_front_tuck_flap"
    )
    contact_guard = check_bimanual_path(
        model,
        action,
        contact_ik.action_rad,
        required_clearance_m=0.003,
        obstacle_geom_names=non_target_obstacles,
    )
    if not contact_guard.safe:
        raise ValueError(f"right front-flap contact path rejected: {contact_guard.reason}")
    contact_closed = list(contact_ik.action_rad)
    contact_closed[11] = 0.2
    close_guard = check_bimanual_path(
        model,
        contact_ik.action_rad,
        contact_closed,
        required_clearance_m=0.003,
        obstacle_geom_names=non_target_obstacles,
    )
    if not close_guard.safe:
        raise ValueError(f"right front-flap close path rejected: {close_guard.reason}")
    contact_data = create_compact_mobile_data(model)
    apply_control_as_pose(model, contact_data, contact_closed)
    front_flap_contacts = _matching_contacts(
        model,
        contact_data,
        geom_name="box_lid_front_tuck_flap",
        arm_side="right",
    )
    if not front_flap_contacts:
        raise ValueError("right gripper did not contact the front tuck flap")
    return RightPregraspPlan(
        box=box,
        right_front_flap_grasp_base_m=flap_grasp,
        targets_base_m=targets,
        actions_rad=tuple(actions),
        residuals_m=tuple(residuals),
        contact_target_base_m=contact_target,
        contact_open_action_rad=contact_ik.action_rad,
        contact_closed_action_rad=tuple(contact_closed),
        contact_residual_m=contact_ik.residual_m_by_side["right"],
        front_flap_contact_count=len(front_flap_contacts),
        minimum_clearance_m=min(
            minimum_clearance,
            contact_guard.minimum_clearance_m,
            close_guard.minimum_clearance_m,
        ),
        accepted=True,
    )


def run_simulation(
    model_path: Path, *, width: int = 640, height: int = 460
) -> RightPregraspPlan:
    model, _ = build_compact_mobile_model(model_path=model_path)
    data = create_compact_mobile_data(model)
    calibration = calibration_from_mujoco(
        model, data, "workspace_depth_camera", width=width, height=height
    )
    depth = render_metric_depth(
        model, data, "workspace_depth_camera", width=width, height=height
    )
    return plan_right_pregrasp_from_depth(model, depth, calibration)


def show_plan(model: mujoco.MjModel, plan: RightPregraspPlan) -> None:
    """Display the depth-derived kinematic plan; never opens hardware."""

    import mujoco.viewer

    data = create_compact_mobile_data(model)
    actions = (
        COMPACT_HOME_ACTION,
        *plan.actions_rad,
        plan.contact_open_action_rad,
        plan.contact_closed_action_rad,
        COMPACT_HOME_ACTION,
    )
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = (0.16, 0.0, 0.16)
        viewer.cam.distance = 0.85
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -25.0
        while viewer.is_running():
            time.sleep(2.0)
            for start, goal in zip(actions[:-1], actions[1:], strict=True):
                start_values, goal_values = np.asarray(start), np.asarray(goal)
                for progress in np.linspace(0.0, 1.0, 101):
                    if not viewer.is_running():
                        return
                    weight = (
                        35 * progress**4
                        - 84 * progress**5
                        + 70 * progress**6
                        - 20 * progress**7
                    )
                    apply_control_as_pose(
                        model,
                        data,
                        start_values + weight * (goal_values - start_values),
                    )
                    viewer.sync()
                    time.sleep(0.02)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-path", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--view", action="store_true")
    mode.add_argument("--teleop", action="store_true")
    parser.add_argument("--record-output", type=Path)
    args = parser.parse_args()
    if args.record_output is not None and not args.teleop:
        parser.error("--record-output requires --teleop")
    model, _ = build_compact_mobile_model(model_path=args.model_path)
    data = create_compact_mobile_data(model)
    calibration = calibration_from_mujoco(
        model, data, "workspace_depth_camera", width=640, height=460
    )
    depth = render_metric_depth(
        model, data, "workspace_depth_camera", width=640, height=460
    )
    plan = plan_right_pregrasp_from_depth(model, depth, calibration)
    print(json.dumps(asdict(plan), indent=2))
    if args.view:
        show_plan(model, plan)
    elif args.teleop:
        from sim_teleop import run_sim_teleop

        report = run_sim_teleop(
            model,
            initial_action=COMPACT_HOME_ACTION,
            obstacle_geom_names=BOX_GEOM_NAMES,
            record_fps=20 if args.record_output is not None else None,
        )
        if args.record_output is not None:
            from compact_teleop_episode import record_compact_pregrasp_episode

            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            manifest = record_compact_pregrasp_episode(
                args.record_output,
                episode_id=f"DAPIER-{stamp}-right-front-flap-pregrasp",
                model=model,
                actions_rad=report["recorded_actions_rad"],
                target_base_m=plan.targets_base_m[-1],
            )
            print(f"teleop_episode_manifest={manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
