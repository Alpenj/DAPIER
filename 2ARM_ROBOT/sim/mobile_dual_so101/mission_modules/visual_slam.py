"""Visual-SLAM result contract consuming front RGB-D and mobility state."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol, runtime_checkable

from .camera import CameraFrame, CameraModality, CameraRole
from .mobility import MobilityStatus
from .types import Pose2D


class SlamTrackingState(str, Enum):
    INITIALIZING = "initializing"
    TRACKING = "tracking"
    RELOCALIZING = "relocalizing"
    LOST = "lost"


@dataclass(frozen=True)
class SlamEstimate:
    pose_map: Pose2D
    covariance_diagonal: tuple[float, float, float]
    tracking_state: SlamTrackingState
    map_id: str
    source_frame_id: int
    observation_age_ms: float
    loop_closure_count: int = 0

    def validate(self) -> None:
        if not isinstance(self.pose_map, Pose2D):
            raise ValueError("pose_map must be a Pose2D")
        self.pose_map.validate()
        if len(self.covariance_diagonal) != 3 or not all(
            math.isfinite(value) and value >= 0
            for value in self.covariance_diagonal
        ):
            raise ValueError("SLAM covariance diagonal must contain three non-negative values")
        if not isinstance(self.tracking_state, SlamTrackingState):
            raise ValueError("tracking_state must be a SlamTrackingState")
        if not self.map_id.strip() or len(self.map_id) > 100:
            raise ValueError("map_id must contain 1 to 100 characters")
        for label, value in (
            ("source_frame_id", self.source_frame_id),
            ("loop_closure_count", self.loop_closure_count),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{label} must be a non-negative integer")
        if not math.isfinite(self.observation_age_ms) or self.observation_age_ms < 0:
            raise ValueError("observation_age_ms must be finite and non-negative")

    def usable(
        self,
        *,
        max_observation_age_ms: float,
        max_position_std_m: float,
        max_yaw_std_rad: float,
    ) -> bool:
        limits = (
            max_observation_age_ms,
            max_position_std_m,
            max_yaw_std_rad,
        )
        if not all(math.isfinite(value) and value > 0 for value in limits):
            raise ValueError("SLAM usability limits must be finite and positive")
        self.validate()
        x_variance, y_variance, yaw_variance = self.covariance_diagonal
        return (
            self.tracking_state == SlamTrackingState.TRACKING
            and self.observation_age_ms <= max_observation_age_ms
            and math.sqrt(max(x_variance, y_variance)) <= max_position_std_m
            and math.sqrt(yaw_variance) <= max_yaw_std_rad
        )


@runtime_checkable
class VisualSlamPort(Protocol):
    @property
    def simulation_only(self) -> bool: ...

    def estimate(
        self,
        front_rgbd: CameraFrame,
        mobility: MobilityStatus,
    ) -> SlamEstimate: ...


def validate_slam_inputs(
    front_rgbd: CameraFrame,
    mobility: MobilityStatus,
) -> None:
    front_rgbd.validate()
    mobility.validate()
    if front_rgbd.role != CameraRole.FRONT_RGBD:
        raise ValueError("Visual SLAM requires the front_rgbd camera role")
    if front_rgbd.modality != CameraModality.RGBD:
        raise ValueError("Visual SLAM requires RGB-D modality")


__all__ = [
    "SlamEstimate",
    "SlamTrackingState",
    "VisualSlamPort",
    "validate_slam_inputs",
]
