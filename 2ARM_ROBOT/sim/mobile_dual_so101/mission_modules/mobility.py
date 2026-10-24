"""High-level TurtleBot mobility contract without wheel command exposure.

The port is a capability boundary, not hardware authorization. MuJoCo,
ROS 2/Nav2, and a future native runtime may implement it independently, while
left/right wheel targets remain private to each adapter.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol, runtime_checkable

from mission_core import MissionLocation
from .types import Pose2D


DEFAULT_LINEAR_SETTLED_MPS = 0.0025
DEFAULT_ANGULAR_SETTLED_RADPS = 0.0021


@dataclass(frozen=True)
class NavigationRequest:
    goal: MissionLocation
    target_pose: Pose2D
    position_tolerance_m: float = 0.05
    yaw_tolerance_rad: float = 0.10
    timeout_s: float = 30.0

    def validate(self) -> None:
        if not isinstance(self.goal, MissionLocation):
            raise ValueError("goal must be a MissionLocation")
        if not isinstance(self.target_pose, Pose2D):
            raise ValueError("target_pose must be a Pose2D")
        self.target_pose.validate()
        values = (
            self.position_tolerance_m,
            self.yaw_tolerance_rad,
            self.timeout_s,
        )
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("navigation tolerances and timeout must be finite and positive")


@dataclass(frozen=True)
class MobilityStatus:
    pose_map: Pose2D
    linear_velocity_mps: float
    angular_velocity_radps: float
    active_goal: MissionLocation | None
    goal_reached: bool
    watchdog_ok: bool
    observation_age_ms: float

    def validate(self) -> None:
        if not isinstance(self.pose_map, Pose2D):
            raise ValueError("pose_map must be a Pose2D")
        self.pose_map.validate()
        if self.active_goal is not None and not isinstance(
            self.active_goal, MissionLocation
        ):
            raise ValueError("active_goal must be a MissionLocation")
        values = (
            self.linear_velocity_mps,
            self.angular_velocity_radps,
            self.observation_age_ms,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("mobility status values must be finite")
        if self.observation_age_ms < 0:
            raise ValueError("observation_age_ms must be non-negative")
        if not isinstance(self.goal_reached, bool) or not isinstance(
            self.watchdog_ok, bool
        ):
            raise ValueError("goal_reached and watchdog_ok must be booleans")
        if self.goal_reached and self.active_goal is None:
            raise ValueError("goal_reached requires an active_goal")

    def stationary(
        self,
        *,
        max_linear_mps: float = DEFAULT_LINEAR_SETTLED_MPS,
        max_angular_radps: float = DEFAULT_ANGULAR_SETTLED_RADPS,
    ) -> bool:
        limits = (max_linear_mps, max_angular_radps)
        if not all(math.isfinite(value) and value > 0 for value in limits):
            raise ValueError("stationary limits must be finite and positive")
        return (
            abs(self.linear_velocity_mps) <= max_linear_mps
            and abs(self.angular_velocity_radps) <= max_angular_radps
        )

    def ready_at_goal(self, *, max_observation_age_ms: float) -> bool:
        if (
            not math.isfinite(max_observation_age_ms)
            or max_observation_age_ms <= 0
        ):
            raise ValueError("max_observation_age_ms must be finite and positive")
        self.validate()
        return (
            self.goal_reached
            and self.watchdog_ok
            and self.stationary()
            and self.observation_age_ms <= max_observation_age_ms
        )


@runtime_checkable
class MobilityPort(Protocol):
    """Named-goal mobility API; implementations keep wheel targets private."""

    @property
    def simulation_only(self) -> bool: ...

    def request_navigation(self, request: NavigationRequest) -> None: ...

    def read_status(self) -> MobilityStatus: ...

    def safe_stop(self, reason: str) -> None: ...


__all__ = [
    "DEFAULT_ANGULAR_SETTLED_RADPS",
    "DEFAULT_LINEAR_SETTLED_MPS",
    "MobilityPort",
    "MobilityStatus",
    "NavigationRequest",
]
