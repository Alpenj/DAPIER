"""Shoe 6D-pose contract using front RGB-D or gripper RGB frames."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol, runtime_checkable

from .camera import CameraRole, MultiCameraFrameSet
from .types import Pose3D
from .visual_slam import SlamEstimate


@dataclass(frozen=True)
class ShoePoseEstimate:
    pose_map: Pose3D
    confidence: float
    source_role: CameraRole
    source_frame_id: int
    observation_age_ms: float
    map_id: str
    estimator_id: str
    ground_truth: bool = False

    def validate(self) -> None:
        if not isinstance(self.pose_map, Pose3D):
            raise ValueError("pose_map must be a Pose3D")
        self.pose_map.validate()
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be inside [0, 1]")
        if not isinstance(self.source_role, CameraRole):
            raise ValueError("source_role must be a CameraRole")
        if (
            isinstance(self.source_frame_id, bool)
            or not isinstance(self.source_frame_id, int)
            or self.source_frame_id < 0
        ):
            raise ValueError("source_frame_id must be a non-negative integer")
        if not math.isfinite(self.observation_age_ms) or self.observation_age_ms < 0:
            raise ValueError("observation_age_ms must be finite and non-negative")
        if not self.map_id.strip() or not self.estimator_id.strip():
            raise ValueError("map_id and estimator_id must not be empty")
        if not isinstance(self.ground_truth, bool):
            raise ValueError("ground_truth must be a boolean")

    def usable(
        self,
        *,
        minimum_confidence: float,
        max_observation_age_ms: float,
        allow_ground_truth: bool = False,
    ) -> bool:
        if not math.isfinite(minimum_confidence) or not 0.0 <= minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be inside [0, 1]")
        if (
            not math.isfinite(max_observation_age_ms)
            or max_observation_age_ms <= 0
        ):
            raise ValueError("max_observation_age_ms must be finite and positive")
        self.validate()
        return (
            self.confidence >= minimum_confidence
            and self.observation_age_ms <= max_observation_age_ms
            and (allow_ground_truth or not self.ground_truth)
        )


@runtime_checkable
class ObjectPosePort(Protocol):
    @property
    def simulation_only(self) -> bool: ...

    def estimate(
        self,
        frames: MultiCameraFrameSet,
        slam: SlamEstimate,
        preferred_role: CameraRole,
    ) -> ShoePoseEstimate: ...


def validate_object_pose_inputs(
    frames: MultiCameraFrameSet,
    slam: SlamEstimate,
    preferred_role: CameraRole,
) -> None:
    frames.validate()
    slam.validate()
    if not isinstance(preferred_role, CameraRole):
        raise ValueError("preferred_role must be a CameraRole")
    if frames.frame(preferred_role) is None:
        raise ValueError("preferred object-pose camera is disabled or missing")


__all__ = [
    "ObjectPosePort",
    "ShoePoseEstimate",
    "validate_object_pose_inputs",
]
