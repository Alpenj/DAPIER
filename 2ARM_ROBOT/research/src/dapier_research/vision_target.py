"""Sensor-derived RGB-D target geometry for the Python research layer.

This module accepts an RGB detector result, associates it with measured depth,
and reconstructs a target pose in a calibrated robot frame. It never reads a
simulator object pose, opens hardware, or authorizes control. MuJoCo adapters
must provide only rendered images and the camera/robot transform that has an
equivalent in the physical system.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np


VISION_TARGET_SCHEMA_VERSION = "dapier.vision-target.v1"


class VisionTargetError(ValueError):
    """Fail-closed RGB-D estimation error with a stable machine-readable code."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class PinholeIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        if (
            isinstance(self.width, bool)
            or isinstance(self.height, bool)
            or not isinstance(self.width, int)
            or not isinstance(self.height, int)
            or self.width <= 0
            or self.height <= 0
        ):
            raise VisionTargetError(
                "invalid_intrinsics", "image width and height must be positive integers"
            )
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(float(value)) for value in values):
            raise VisionTargetError(
                "invalid_intrinsics", "camera intrinsics must be finite"
            )
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise VisionTargetError(
                "invalid_intrinsics", "camera focal lengths must be positive"
            )
        if not (-0.5 <= self.cx <= self.width - 0.5):
            raise VisionTargetError(
                "invalid_intrinsics", "camera principal point cx is outside the image"
            )
        if not (-0.5 <= self.cy <= self.height - 0.5):
            raise VisionTargetError(
                "invalid_intrinsics", "camera principal point cy is outside the image"
            )

    @classmethod
    def from_vertical_fov(
        cls,
        *,
        width: int,
        height: int,
        vertical_fov_deg: float,
    ) -> "PinholeIntrinsics":
        if not math.isfinite(vertical_fov_deg) or not 1.0 <= vertical_fov_deg < 179.0:
            raise VisionTargetError(
                "invalid_intrinsics", "vertical_fov_deg must be inside [1, 179)"
            )
        focal = 0.5 * float(height) / math.tan(math.radians(vertical_fov_deg) / 2.0)
        result = cls(
            width=width,
            height=height,
            fx=focal,
            fy=focal,
            cx=(float(width) - 1.0) / 2.0,
            cy=(float(height) - 1.0) / 2.0,
        )
        result.validate()
        return result

    def as_dict(self) -> dict[str, float | int]:
        self.validate()
        return {
            "width": self.width,
            "height": self.height,
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
        }


@dataclass(frozen=True)
class PixelDetection:
    """Detector output created from image pixels, not simulator object state."""

    label: str
    mask: np.ndarray
    confidence: float
    detector: str
    uses_privileged_labels: bool = False


@dataclass(frozen=True)
class VisionTargetEstimate:
    schema_version: str
    label: str
    detector: str
    source: str
    camera_frame: str
    target_frame: str
    timestamp_ns: int
    position_optical_m: tuple[float, float, float]
    position_target_m: tuple[float, float, float]
    covariance_diagonal_m2: tuple[float, float, float]
    median_depth_m: float
    detector_confidence: float
    depth_confidence: float
    confidence: float
    mask_pixels: int
    valid_depth_pixels: int
    inlier_pixels: int
    ground_truth_used: bool = False
    control_authorized: bool = False
    hardware_execution: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "label": self.label,
            "detector": self.detector,
            "source": self.source,
            "camera_frame": self.camera_frame,
            "target_frame": self.target_frame,
            "timestamp_ns": self.timestamp_ns,
            "position_optical_m": list(self.position_optical_m),
            "position_target_m": list(self.position_target_m),
            "covariance_diagonal_m2": list(self.covariance_diagonal_m2),
            "median_depth_m": self.median_depth_m,
            "detector_confidence": self.detector_confidence,
            "depth_confidence": self.depth_confidence,
            "confidence": self.confidence,
            "mask_pixels": self.mask_pixels,
            "valid_depth_pixels": self.valid_depth_pixels,
            "inlier_pixels": self.inlier_pixels,
            "ground_truth_used": self.ground_truth_used,
            "control_authorized": self.control_authorized,
            "hardware_execution": self.hardware_execution,
        }


@dataclass(frozen=True)
class CartesianTargetProposal:
    schema_version: str
    label: str
    frame: str
    position_m: tuple[float, float, float]
    confidence: float
    source_estimate_schema: str
    source: str = "vision_rgbd_approach"
    control_authorized: bool = False
    hardware_execution: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "label": self.label,
            "frame": self.frame,
            "position_m": list(self.position_m),
            "confidence": self.confidence,
            "source_estimate_schema": self.source_estimate_schema,
            "source": self.source,
            "control_authorized": self.control_authorized,
            "hardware_execution": self.hardware_execution,
        }


def _finite_transform(target_from_optical: np.ndarray) -> np.ndarray:
    transform = np.asarray(target_from_optical, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise VisionTargetError(
            "invalid_transform", "target_from_optical must be a finite 4x4 matrix"
        )
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise VisionTargetError(
            "invalid_transform", "target_from_optical must be homogeneous"
        )
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise VisionTargetError(
            "invalid_transform", "target_from_optical rotation must be orthonormal"
        )
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5):
        raise VisionTargetError(
            "invalid_transform", "target_from_optical rotation must be right-handed"
        )
    return transform


def _validate_detection(
    detection: PixelDetection,
    *,
    image_shape: tuple[int, int],
) -> np.ndarray:
    if not isinstance(detection, PixelDetection):
        raise VisionTargetError(
            "invalid_detection", "detection must be a PixelDetection"
        )
    if not isinstance(detection.label, str) or not detection.label.strip():
        raise VisionTargetError(
            "invalid_detection", "detection label must be non-empty"
        )
    if not isinstance(detection.detector, str) or not detection.detector.strip():
        raise VisionTargetError(
            "invalid_detection", "detector name must be non-empty"
        )
    if not math.isfinite(detection.confidence) or not 0.0 <= detection.confidence <= 1.0:
        raise VisionTargetError(
            "invalid_detection", "detector confidence must be inside [0, 1]"
        )
    if not isinstance(detection.uses_privileged_labels, bool):
        raise VisionTargetError(
            "invalid_detection", "uses_privileged_labels must be a boolean"
        )
    if detection.uses_privileged_labels:
        raise VisionTargetError(
            "privileged_detection",
            "runtime target estimation rejects simulator segmentation/body labels",
        )
    mask = np.asarray(detection.mask)
    if mask.shape != image_shape:
        raise VisionTargetError(
            "invalid_detection", "detection mask shape does not match the RGB-D frame"
        )
    if mask.dtype != np.bool_:
        if not np.issubdtype(mask.dtype, np.integer):
            raise VisionTargetError(
                "invalid_detection", "detection mask must be boolean or integer"
            )
        unique = np.unique(mask)
        if not np.all(np.isin(unique, (0, 1))):
            raise VisionTargetError(
                "invalid_detection", "integer detection mask must contain only 0 and 1"
            )
        mask = mask.astype(bool)
    return mask


def estimate_target_from_rgbd(
    *,
    rgb: np.ndarray,
    depth_m: np.ndarray,
    detection: PixelDetection,
    intrinsics: PinholeIntrinsics,
    target_from_optical: np.ndarray,
    timestamp_ns: int,
    camera_frame: str,
    target_frame: str,
    minimum_depth_m: float = 0.05,
    maximum_depth_m: float = 5.0,
    minimum_valid_pixels: int = 6,
    minimum_valid_fraction: float = 0.20,
    outlier_sigma: float = 3.5,
    minimum_outlier_tolerance_m: float = 0.008,
) -> VisionTargetEstimate:
    """Estimate one target centroid from RGB pixels plus aligned metric depth.

    Camera optical coordinates follow the ROS optical convention: +X right,
    +Y down, +Z forward. ``target_from_optical`` is the calibrated transform
    into the robot/planning frame. The function has no simulator dependency.
    """

    intrinsics.validate()
    image = np.asarray(rgb)
    depth = np.asarray(depth_m, dtype=np.float64)
    expected_shape = (intrinsics.height, intrinsics.width)
    if image.shape != (*expected_shape, 3):
        raise VisionTargetError(
            "invalid_rgb", "RGB shape does not match camera intrinsics"
        )
    if depth.shape != expected_shape:
        raise VisionTargetError(
            "invalid_depth", "depth shape does not match camera intrinsics"
        )
    if image.dtype != np.uint8:
        raise VisionTargetError("invalid_rgb", "RGB image must use uint8 pixels")
    mask = _validate_detection(detection, image_shape=expected_shape)
    transform = _finite_transform(target_from_optical)

    if (
        isinstance(timestamp_ns, bool)
        or not isinstance(timestamp_ns, int)
        or timestamp_ns < 0
    ):
        raise VisionTargetError(
            "invalid_timestamp", "timestamp_ns must be a non-negative integer"
        )
    if not isinstance(camera_frame, str) or not camera_frame.strip():
        raise VisionTargetError("invalid_frame", "camera_frame must be non-empty")
    if not isinstance(target_frame, str) or not target_frame.strip():
        raise VisionTargetError("invalid_frame", "target_frame must be non-empty")
    depth_limits = (minimum_depth_m, maximum_depth_m)
    if not all(math.isfinite(value) and value > 0.0 for value in depth_limits):
        raise VisionTargetError(
            "invalid_depth_limits", "depth limits must be finite and positive"
        )
    if minimum_depth_m >= maximum_depth_m:
        raise VisionTargetError(
            "invalid_depth_limits", "minimum depth must be below maximum depth"
        )
    if (
        isinstance(minimum_valid_pixels, bool)
        or not isinstance(minimum_valid_pixels, int)
        or minimum_valid_pixels <= 0
    ):
        raise VisionTargetError(
            "invalid_estimator_config", "minimum_valid_pixels must be positive"
        )
    if not 0.0 < minimum_valid_fraction <= 1.0:
        raise VisionTargetError(
            "invalid_estimator_config", "minimum_valid_fraction must be inside (0, 1]"
        )
    if not math.isfinite(outlier_sigma) or outlier_sigma <= 0.0:
        raise VisionTargetError(
            "invalid_estimator_config", "outlier_sigma must be finite and positive"
        )
    if (
        not math.isfinite(minimum_outlier_tolerance_m)
        or minimum_outlier_tolerance_m <= 0.0
    ):
        raise VisionTargetError(
            "invalid_estimator_config",
            "minimum_outlier_tolerance_m must be finite and positive",
        )

    mask_pixels = int(np.count_nonzero(mask))
    if mask_pixels == 0:
        raise VisionTargetError(
            "empty_detection", "detector produced an empty target mask"
        )
    valid = (
        mask
        & np.isfinite(depth)
        & (depth >= minimum_depth_m)
        & (depth <= maximum_depth_m)
    )
    valid_depth_pixels = int(np.count_nonzero(valid))
    valid_fraction = valid_depth_pixels / mask_pixels
    if (
        valid_depth_pixels < minimum_valid_pixels
        or valid_fraction < minimum_valid_fraction
    ):
        raise VisionTargetError(
            "insufficient_depth",
            "target mask does not contain enough valid aligned depth pixels",
        )

    valid_depth = depth[valid]
    median_depth = float(np.median(valid_depth))
    median_absolute_deviation = float(np.median(np.abs(valid_depth - median_depth)))
    robust_sigma = 1.4826 * median_absolute_deviation
    tolerance = max(minimum_outlier_tolerance_m, outlier_sigma * robust_sigma)
    inlier_mask = valid & (np.abs(depth - median_depth) <= tolerance)
    inlier_pixels = int(np.count_nonzero(inlier_mask))
    if inlier_pixels < minimum_valid_pixels:
        raise VisionTargetError(
            "insufficient_inliers", "depth outlier rejection removed too many pixels"
        )

    rows, columns = np.nonzero(inlier_mask)
    z = depth[inlier_mask]
    x = (columns.astype(np.float64) - intrinsics.cx) * z / intrinsics.fx
    y = (rows.astype(np.float64) - intrinsics.cy) * z / intrinsics.fy
    optical_points = np.column_stack((x, y, z))
    target_points = optical_points @ transform[:3, :3].T + transform[:3, 3]
    optical_position = np.median(optical_points, axis=0)
    target_position = np.median(target_points, axis=0)
    covariance_diagonal = np.var(
        target_points,
        axis=0,
        ddof=1 if inlier_pixels > 1 else 0,
    )

    inlier_fraction = inlier_pixels / valid_depth_pixels
    relative_spread = median_absolute_deviation / max(median_depth, 1e-9)
    spread_score = math.exp(-20.0 * relative_spread)
    depth_confidence = min(1.0, valid_fraction * inlier_fraction * spread_score)
    confidence = min(1.0, detection.confidence * depth_confidence)

    return VisionTargetEstimate(
        schema_version=VISION_TARGET_SCHEMA_VERSION,
        label=detection.label.strip(),
        detector=detection.detector.strip(),
        source="rgb_detection_plus_aligned_metric_depth",
        camera_frame=camera_frame.strip(),
        target_frame=target_frame.strip(),
        timestamp_ns=timestamp_ns,
        position_optical_m=tuple(float(value) for value in optical_position),
        position_target_m=tuple(float(value) for value in target_position),
        covariance_diagonal_m2=tuple(float(value) for value in covariance_diagonal),
        median_depth_m=median_depth,
        detector_confidence=float(detection.confidence),
        depth_confidence=depth_confidence,
        confidence=confidence,
        mask_pixels=mask_pixels,
        valid_depth_pixels=valid_depth_pixels,
        inlier_pixels=inlier_pixels,
    )


def approach_target_from_estimate(
    estimate: VisionTargetEstimate,
    *,
    offset_target_m: Sequence[float] = (0.0, 0.0, 0.08),
    minimum_confidence: float = 0.25,
) -> CartesianTargetProposal:
    if not isinstance(estimate, VisionTargetEstimate):
        raise VisionTargetError(
            "invalid_estimate", "estimate must be a VisionTargetEstimate"
        )
    if estimate.ground_truth_used:
        raise VisionTargetError(
            "privileged_estimate", "ground-truth-derived targets cannot enter planning"
        )
    if not math.isfinite(minimum_confidence) or not 0.0 <= minimum_confidence <= 1.0:
        raise VisionTargetError(
            "invalid_confidence", "minimum_confidence must be inside [0, 1]"
        )
    if estimate.confidence < minimum_confidence:
        raise VisionTargetError(
            "low_confidence", "vision estimate confidence is below the planning gate"
        )
    offset = np.asarray(tuple(offset_target_m), dtype=np.float64)
    if offset.shape != (3,) or not np.all(np.isfinite(offset)):
        raise VisionTargetError(
            "invalid_offset", "approach offset must contain three finite values"
        )
    position = np.asarray(estimate.position_target_m, dtype=np.float64) + offset
    return CartesianTargetProposal(
        schema_version="dapier.cartesian-target-proposal.v1",
        label=estimate.label,
        frame=estimate.target_frame,
        position_m=tuple(float(value) for value in position),
        confidence=estimate.confidence,
        source_estimate_schema=estimate.schema_version,
    )
