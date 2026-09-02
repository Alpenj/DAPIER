#!/usr/bin/env python3
"""Vision-first MuJoCo target estimation and reach planning.

The runtime path derives object coordinates from rendered RGB pixels, aligned
metric depth, camera intrinsics, and calibrated frame transforms. MuJoCo body
poses or segmentation IDs are not used to generate the object target. Simulator
state may still be used outside this module for reset, reward, and test-only
error measurement.

This module is simulation-only. It emits a Python research ``ControlIntent``
proposal but never authorizes hardware or opens ROS/serial/device endpoints.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping, Sequence

import mujoco
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parent
RESEARCH_SRC = PROJECT_DIR.parents[1] / "research" / "src"
if str(RESEARCH_SRC) not in sys.path:
    sys.path.insert(0, str(RESEARCH_SRC))

from dapier_research.control_intent import arm_joint_position_intent  # noqa: E402
from dapier_research.vision_target import (  # noqa: E402
    CartesianTargetProposal,
    PixelDetection,
    PinholeIntrinsics,
    VisionTargetError,
    VisionTargetEstimate,
    approach_target_from_estimate,
    estimate_target_from_rgbd,
)
from collision_guard import CollisionAssessment, check_bimanual_path  # noqa: E402
from mobile_dual_so101 import (  # noqa: E402
    ACTION_NAMES,
    actuator_targets_from_qpos,
)
from physics_ik import IKResult, solve_bimanual_position_ik  # noqa: E402


POLICY_OBSERVATION_SCHEMA_VERSION = "dapier.vision-policy-observation.v1"
VISION_REACH_PLAN_SCHEMA_VERSION = "dapier.vision-guided-reach-plan.v1"
MUJOCO_CAMERA_FROM_OPTICAL = np.diag((1.0, -1.0, -1.0))


@dataclass(frozen=True)
class RenderedRgbdFrame:
    rgb: np.ndarray
    depth_m: np.ndarray
    intrinsics: PinholeIntrinsics
    target_from_optical: np.ndarray
    world_from_target: np.ndarray
    timestamp_ns: int
    clock_domain: str
    camera_frame: str
    target_frame: str


@dataclass(frozen=True)
class VisionGuidedReachPlan:
    schema_version: str
    side: str
    estimate: VisionTargetEstimate
    approach: CartesianTargetProposal
    simulator_ik_target_world_m: tuple[float, float, float]
    ik: IKResult
    collision: CollisionAssessment | None
    control_intent: dict[str, Any] | None
    planning_accepted: bool
    reason: str
    ground_truth_used_for_target: bool = False
    hardware_execution: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "side": self.side,
            "estimate": self.estimate.as_dict(),
            "approach": self.approach.as_dict(),
            "simulator_ik_target_world_m": list(self.simulator_ik_target_world_m),
            "ik": asdict(self.ik),
            "collision": (
                None if self.collision is None else self.collision.as_report()
            ),
            "control_intent": self.control_intent,
            "planning_accepted": self.planning_accepted,
            "reason": self.reason,
            "ground_truth_used_for_target": self.ground_truth_used_for_target,
            "hardware_execution": self.hardware_execution,
        }


def _largest_component(mask: np.ndarray, minimum_pixels: int) -> np.ndarray:
    """Return the best lower-image connected component without OpenCV."""

    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    best_coordinates: list[tuple[int, int]] = []
    best_score = -math.inf
    for row, column in zip(*np.nonzero(mask), strict=True):
        row_i = int(row)
        column_i = int(column)
        if visited[row_i, column_i]:
            continue
        stack = [(row_i, column_i)]
        visited[row_i, column_i] = True
        coordinates: list[tuple[int, int]] = []
        while stack:
            current_row, current_column = stack.pop()
            coordinates.append((current_row, current_column))
            for delta_row, delta_column in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                next_row = current_row + delta_row
                next_column = current_column + delta_column
                if (
                    0 <= next_row < height
                    and 0 <= next_column < width
                    and mask[next_row, next_column]
                    and not visited[next_row, next_column]
                ):
                    visited[next_row, next_column] = True
                    stack.append((next_row, next_column))
        if len(coordinates) < minimum_pixels:
            continue
        centroid_row = sum(item[0] for item in coordinates) / len(coordinates)
        lower_image_weight = 1.0 + centroid_row / max(1.0, height - 1.0)
        score = len(coordinates) * lower_image_weight
        if score > best_score:
            best_score = score
            best_coordinates = coordinates

    result = np.zeros_like(mask, dtype=bool)
    for row, column in best_coordinates:
        result[row, column] = True
    return result


@dataclass(frozen=True)
class SimBlueShoeDetector:
    """Deterministic RGB-only fixture for the current blue MuJoCo shoe.

    This detector is intentionally replaceable. It does not use MuJoCo geom/body
    IDs. Physical deployment should provide a learned detector or keypoint model
    with the same ``PixelDetection`` output contract.
    """

    minimum_blue: int = 80
    blue_over_red: int = 35
    blue_over_green: int = 18
    minimum_component_pixels: int = 6
    ignore_top_fraction: float = 0.20

    def detect(self, rgb: np.ndarray) -> PixelDetection:
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise VisionTargetError(
                "invalid_rgb", "SimBlueShoeDetector expects an HxWx3 uint8 RGB image"
            )
        if (
            isinstance(self.minimum_component_pixels, bool)
            or self.minimum_component_pixels <= 0
        ):
            raise VisionTargetError(
                "invalid_detector_config", "minimum_component_pixels must be positive"
            )
        if not 0.0 <= self.ignore_top_fraction < 1.0:
            raise VisionTargetError(
                "invalid_detector_config", "ignore_top_fraction must be inside [0, 1)"
            )
        red = image[:, :, 0].astype(np.int16)
        green = image[:, :, 1].astype(np.int16)
        blue = image[:, :, 2].astype(np.int16)
        mask = (
            (blue >= self.minimum_blue)
            & ((blue - red) >= self.blue_over_red)
            & ((blue - green) >= self.blue_over_green)
        )
        top_rows = int(round(image.shape[0] * self.ignore_top_fraction))
        if top_rows > 0:
            mask[:top_rows, :] = False
        component = _largest_component(mask, self.minimum_component_pixels)
        pixels = int(np.count_nonzero(component))
        if pixels < self.minimum_component_pixels:
            raise VisionTargetError(
                "detection_not_found", "RGB detector did not find a shoe-like component"
            )
        color_margin = blue[component] - np.maximum(red[component], green[component])
        confidence = min(1.0, 0.55 + float(np.median(color_margin)) / 255.0)
        return PixelDetection(
            label="shoe",
            mask=component,
            confidence=confidence,
            detector="sim_rgb_blue_component_v1",
            uses_privileged_labels=False,
        )


def _object_id(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    name: str,
) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise VisionTargetError(
            "missing_frame", f"MuJoCo frame is missing: {name}"
        )
    return int(object_id)


def _homogeneous(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64)
    result[:3, 3] = np.asarray(translation, dtype=np.float64)
    return result


def _invert_rigid(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return _homogeneous(rotation.T, -(rotation.T @ translation))


def frame_from_rendered_rgbd(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    rgb: np.ndarray,
    depth_m: np.ndarray,
    camera_name: str = "front_depth_camera",
    target_body_name: str | None = "tb3_base_link",
) -> RenderedRgbdFrame:
    """Attach portable calibration metadata to an already rendered RGB-D pair."""

    image = np.asarray(rgb)
    depth = np.asarray(depth_m, dtype=np.float32)
    if image.ndim != 3 or image.shape[2] != 3:
        raise VisionTargetError("invalid_rgb", "RGB render must be HxWx3")
    if image.dtype != np.uint8:
        raise VisionTargetError("invalid_rgb", "RGB render must use uint8 pixels")
    if depth.shape != image.shape[:2]:
        raise VisionTargetError(
            "invalid_depth", "depth render must match the RGB image size"
        )
    camera_id = _object_id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    fovy_deg = float(model.cam_fovy[camera_id])
    intrinsics = PinholeIntrinsics.from_vertical_fov(
        width=image.shape[1],
        height=image.shape[0],
        vertical_fov_deg=fovy_deg,
    )

    world_from_mujoco_camera = _homogeneous(
        data.cam_xmat[camera_id].reshape(3, 3),
        data.cam_xpos[camera_id],
    )
    mujoco_camera_from_optical = _homogeneous(
        MUJOCO_CAMERA_FROM_OPTICAL,
        (0.0, 0.0, 0.0),
    )
    world_from_optical = world_from_mujoco_camera @ mujoco_camera_from_optical

    if target_body_name is None:
        target_from_optical = world_from_optical
        world_from_target = np.eye(4, dtype=np.float64)
        target_frame = "map_sim_world"
    else:
        target_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, target_body_name)
        world_from_target = _homogeneous(
            data.xmat[target_id].reshape(3, 3),
            data.xpos[target_id],
        )
        target_from_optical = _invert_rigid(world_from_target) @ world_from_optical
        target_frame = target_body_name

    return RenderedRgbdFrame(
        rgb=image,
        depth_m=depth,
        intrinsics=intrinsics,
        target_from_optical=target_from_optical,
        world_from_target=world_from_target,
        timestamp_ns=round(float(data.time) * 1_000_000_000),
        clock_domain="mujoco_sim_time",
        camera_frame=f"{camera_name}_optical",
        target_frame=target_frame,
    )


def render_rgbd_frame(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    width: int = 160,
    height: int = 120,
    camera_name: str = "front_depth_camera",
    target_body_name: str | None = "tb3_base_link",
) -> RenderedRgbdFrame:
    if width <= 0 or height <= 0:
        raise VisionTargetError(
            "invalid_render_size", "render width and height must be positive"
        )
    with mujoco.Renderer(model, width=width, height=height) as renderer:
        renderer.disable_depth_rendering()
        renderer.update_scene(data, camera=camera_name)
        rgb = np.asarray(renderer.render(), dtype=np.uint8).copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=camera_name)
        depth = np.asarray(renderer.render(), dtype=np.float32).copy()
        renderer.disable_depth_rendering()
    return frame_from_rendered_rgbd(
        model,
        data,
        rgb=rgb,
        depth_m=depth,
        camera_name=camera_name,
        target_body_name=target_body_name,
    )


def estimate_shoe_from_frame(
    frame: RenderedRgbdFrame,
    *,
    detector: SimBlueShoeDetector | None = None,
) -> VisionTargetEstimate:
    resolved_detector = detector or SimBlueShoeDetector()
    detection = resolved_detector.detect(frame.rgb)
    return estimate_target_from_rgbd(
        rgb=frame.rgb,
        depth_m=frame.depth_m,
        detection=detection,
        intrinsics=frame.intrinsics,
        target_from_optical=frame.target_from_optical,
        timestamp_ns=frame.timestamp_ns,
        camera_frame=frame.camera_frame,
        target_frame=frame.target_frame,
    )


def build_vision_policy_observation(
    frame: RenderedRgbdFrame,
    *,
    robot_state: Mapping[str, object],
    detector: SimBlueShoeDetector | None = None,
) -> dict[str, object]:
    """Build policy input without exposing the simulator object pose."""

    result: dict[str, object] = {
        "schema_version": POLICY_OBSERVATION_SCHEMA_VERSION,
        "timestamp_ns": frame.timestamp_ns,
        "clock_domain": frame.clock_domain,
        "ground_truth": False,
        "robot": dict(robot_state),
        "target_available": False,
        "target": None,
        "failure": None,
        "control_authorized": False,
        "hardware_execution": False,
    }
    try:
        estimate = estimate_shoe_from_frame(frame, detector=detector)
    except VisionTargetError as error:
        result["failure"] = {"code": error.code, "message": str(error)}
    else:
        result["target_available"] = True
        result["target"] = estimate.as_dict()
    return result


def _transform_point(transform: np.ndarray, point: Sequence[float]) -> np.ndarray:
    values = np.asarray(tuple(point), dtype=np.float64)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise VisionTargetError(
            "invalid_target", "Cartesian target must contain three finite values"
        )
    return transform[:3, :3] @ values + transform[:3, 3]


def plan_vision_guided_reach_from_frame(
    model: mujoco.MjModel,
    frame: RenderedRgbdFrame,
    start_action_rad: Sequence[float],
    *,
    side: str = "left",
    detector: SimBlueShoeDetector | None = None,
    approach_offset_target_m: Sequence[float] = (0.0, 0.0, 0.08),
    minimum_confidence: float = 0.25,
    required_clearance_m: float = 0.03,
    sequence: int = 1,
    max_joint_velocity_rad_s: float = 0.35,
) -> VisionGuidedReachPlan:
    """Convert a sensor-derived target into a bounded joint intent proposal."""

    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    if not math.isfinite(required_clearance_m) or required_clearance_m <= 0.0:
        raise ValueError("required_clearance_m must be finite and positive")
    if (
        not math.isfinite(max_joint_velocity_rad_s)
        or max_joint_velocity_rad_s <= 0.0
    ):
        raise ValueError("max_joint_velocity_rad_s must be finite and positive")

    estimate = estimate_shoe_from_frame(frame, detector=detector)
    approach = approach_target_from_estimate(
        estimate,
        offset_target_m=approach_offset_target_m,
        minimum_confidence=minimum_confidence,
    )
    world_target = _transform_point(frame.world_from_target, approach.position_m)
    ik = solve_bimanual_position_ik(
        model,
        start_action_rad,
        {side: world_target},
        max_iterations=250,
    )
    if not ik.converged:
        return VisionGuidedReachPlan(
            schema_version=VISION_REACH_PLAN_SCHEMA_VERSION,
            side=side,
            estimate=estimate,
            approach=approach,
            simulator_ik_target_world_m=tuple(float(value) for value in world_target),
            ik=ik,
            collision=None,
            control_intent=None,
            planning_accepted=False,
            reason="vision target was valid but IK did not converge",
        )

    collision = check_bimanual_path(
        model,
        start_action_rad,
        ik.action_rad,
        required_clearance_m=required_clearance_m,
    )
    if not collision.safe:
        return VisionGuidedReachPlan(
            schema_version=VISION_REACH_PLAN_SCHEMA_VERSION,
            side=side,
            estimate=estimate,
            approach=approach,
            simulator_ik_target_world_m=tuple(float(value) for value in world_target),
            ik=ik,
            collision=collision,
            control_intent=None,
            planning_accepted=False,
            reason="IK target was rejected by the collision guard",
        )

    intent = arm_joint_position_intent(
        sequence=sequence,
        source="mujoco_rgbd_vision_planner",
        source_monotonic_ns=time.monotonic_ns(),
        joint_names=ACTION_NAMES,
        joint_position_rad=ik.action_rad,
        joint_max_velocity_rad_s=(max_joint_velocity_rad_s,) * len(ACTION_NAMES),
    )
    return VisionGuidedReachPlan(
        schema_version=VISION_REACH_PLAN_SCHEMA_VERSION,
        side=side,
        estimate=estimate,
        approach=approach,
        simulator_ik_target_world_m=tuple(float(value) for value in world_target),
        ik=ik,
        collision=collision,
        control_intent=intent.as_dict(),
        planning_accepted=True,
        reason="sensor-derived approach target passed IK and collision planning",
    )


def capture_vision_guided_reach_plan(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    side: str = "left",
    width: int = 160,
    height: int = 120,
    detector: SimBlueShoeDetector | None = None,
) -> VisionGuidedReachPlan:
    frame = render_rgbd_frame(model, data, width=width, height=height)
    start = actuator_targets_from_qpos(model, data.qpos)
    return plan_vision_guided_reach_from_frame(
        model,
        frame,
        start,
        side=side,
        detector=detector,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Plan a simulation-only SO-101 approach from rendered RGB-D."
    )
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--width", type=int, default=160)
    parser.add_argument("--height", type=int, default=120)
    args = parser.parse_args(argv)

    from shoe_task import ShoeTaskEnv

    env = ShoeTaskEnv()
    env.reset(seed=0)
    plan = capture_vision_guided_reach_plan(
        env.model,
        env.data,
        side=args.side,
        width=args.width,
        height=args.height,
    )
    print(json.dumps(plan.as_dict(), ensure_ascii=False, indent=2))
    return 0 if plan.planning_accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
