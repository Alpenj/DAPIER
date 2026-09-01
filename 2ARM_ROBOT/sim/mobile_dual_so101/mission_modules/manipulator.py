"""High-level dual SO-101 manipulation contract.

The mission layer requests semantic actions.  Joint targets, IK, MoveIt 2,
MuJoCo controls, and serial motor writes stay behind an adapter boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol, runtime_checkable

from .types import Pose3D


SO101_ARM_DOF = 5


class ArmSide(str, Enum):
    LEFT = "left"
    RIGHT = "right"


class ManipulationAction(str, Enum):
    PICK = "pick"
    PLACE = "place"
    MOVE_TO_CARRY = "move_to_carry"
    HOLD = "hold"


class PlanningBackend(str, Enum):
    SCRIPTED = "scripted"
    NATIVE_IK = "native_ik"
    MOVEIT2 = "moveit2"
    LEARNED_POLICY = "learned_policy"


@dataclass(frozen=True)
class ManipulationRequest:
    request_id: str
    action: ManipulationAction
    preferred_arm: ArmSide
    target_pose_map: Pose3D | None
    planning_backend: PlanningBackend
    timeout_s: float = 10.0
    max_joint_velocity_radps: float = 0.5
    require_stationary_base: bool = True

    def validate(self) -> None:
        if not self.request_id.strip() or len(self.request_id) > 100:
            raise ValueError("request_id must contain 1 to 100 characters")
        if not isinstance(self.action, ManipulationAction):
            raise ValueError("action must be a ManipulationAction")
        if not isinstance(self.preferred_arm, ArmSide):
            raise ValueError("preferred_arm must be an ArmSide")
        if not isinstance(self.planning_backend, PlanningBackend):
            raise ValueError("planning_backend must be a PlanningBackend")
        if self.action in {ManipulationAction.PICK, ManipulationAction.PLACE}:
            if not isinstance(self.target_pose_map, Pose3D):
                raise ValueError("pick and place require target_pose_map")
        if self.target_pose_map is not None:
            self.target_pose_map.validate()
        limits = (self.timeout_s, self.max_joint_velocity_radps)
        if not all(math.isfinite(value) and value > 0 for value in limits):
            raise ValueError("timeout and joint velocity limit must be positive")
        if not isinstance(self.require_stationary_base, bool):
            raise ValueError("require_stationary_base must be a boolean")


@dataclass(frozen=True)
class ArmTelemetry:
    side: ArmSide
    joint_position_rad: tuple[float, ...]
    joint_velocity_radps: tuple[float, ...]
    motor_temperature_c: tuple[float, ...]
    motor_error: tuple[bool, ...]
    gripper_position_rad: float
    gripper_holding: bool

    def validate(self) -> None:
        if not isinstance(self.side, ArmSide):
            raise ValueError("side must be an ArmSide")
        vectors = (
            self.joint_position_rad,
            self.joint_velocity_radps,
            self.motor_temperature_c,
            self.motor_error,
        )
        if any(len(vector) != SO101_ARM_DOF for vector in vectors):
            raise ValueError(f"SO-101 telemetry requires {SO101_ARM_DOF} arm joints")
        numeric = (
            *self.joint_position_rad,
            *self.joint_velocity_radps,
            *self.motor_temperature_c,
            self.gripper_position_rad,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("arm telemetry values must be finite")
        if any(temperature < -40.0 for temperature in self.motor_temperature_c):
            raise ValueError("motor temperatures are outside the supported range")
        if not all(isinstance(value, bool) for value in self.motor_error):
            raise ValueError("motor_error values must be booleans")
        if not isinstance(self.gripper_holding, bool):
            raise ValueError("gripper_holding must be a boolean")


@dataclass(frozen=True)
class ManipulatorStatus:
    left: ArmTelemetry
    right: ArmTelemetry
    active_request_id: str | None
    trajectory_complete: bool
    collision_free: bool
    watchdog_ok: bool
    object_lifted: bool
    carry_pose_clear: bool
    observation_age_ms: float

    def validate(self) -> None:
        self.left.validate()
        self.right.validate()
        if self.left.side != ArmSide.LEFT or self.right.side != ArmSide.RIGHT:
            raise ValueError("left/right telemetry sides do not match their slots")
        if self.active_request_id is not None and (
            not self.active_request_id.strip() or len(self.active_request_id) > 100
        ):
            raise ValueError("active_request_id must contain 1 to 100 characters")
        flags = (
            self.trajectory_complete,
            self.collision_free,
            self.watchdog_ok,
            self.object_lifted,
            self.carry_pose_clear,
        )
        if not all(isinstance(value, bool) for value in flags):
            raise ValueError("manipulator status flags must be booleans")
        if not math.isfinite(self.observation_age_ms) or self.observation_age_ms < 0:
            raise ValueError("observation_age_ms must be finite and non-negative")

    def safe_for_motion(
        self,
        *,
        max_observation_age_ms: float,
        max_motor_temperature_c: float,
    ) -> bool:
        limits = (max_observation_age_ms, max_motor_temperature_c)
        if not all(math.isfinite(value) and value > 0 for value in limits):
            raise ValueError("manipulator safety limits must be positive")
        self.validate()
        telemetry = (self.left, self.right)
        return (
            self.watchdog_ok
            and self.collision_free
            and self.observation_age_ms <= max_observation_age_ms
            and not any(any(arm.motor_error) for arm in telemetry)
            and max(
                temperature
                for arm in telemetry
                for temperature in arm.motor_temperature_c
            )
            <= max_motor_temperature_c
        )

    def verified_carry(self, *, max_observation_age_ms: float) -> bool:
        self.validate()
        return (
            self.trajectory_complete
            and self.object_lifted
            and (self.left.gripper_holding or self.right.gripper_holding)
            and self.carry_pose_clear
            and self.watchdog_ok
            and self.observation_age_ms <= max_observation_age_ms
        )


@runtime_checkable
class ManipulatorPort(Protocol):
    @property
    def simulation_only(self) -> bool: ...

    def request_action(self, request: ManipulationRequest) -> None: ...

    def read_status(self) -> ManipulatorStatus: ...

    def hold_safe(self, reason: str) -> None: ...


__all__ = [
    "ArmSide",
    "ArmTelemetry",
    "ManipulationAction",
    "ManipulationRequest",
    "ManipulatorPort",
    "ManipulatorStatus",
    "PlanningBackend",
    "SO101_ARM_DOF",
]
