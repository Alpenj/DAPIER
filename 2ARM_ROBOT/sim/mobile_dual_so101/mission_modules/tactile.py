"""Hardware-neutral tactile status contract for both SO-101 grippers."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol, runtime_checkable

from .manipulator import ArmSide


class TactileRole(str, Enum):
    LEFT_GRIPPER = "left_gripper"
    RIGHT_GRIPPER = "right_gripper"


class TactileModality(str, Enum):
    NORMAL_FORCE = "normal_force"
    TAXEL_ARRAY = "taxel_array"
    SIX_AXIS_FORCE_TORQUE = "six_axis_force_torque"


ALL_TACTILE_ROLES = frozenset(TactileRole)


@dataclass(frozen=True)
class TactileLimits:
    minimum_contact_force_n: float = 0.5
    maximum_normal_force_n: float = 20.0
    maximum_slip_probability: float = 0.5
    maximum_observation_age_ms: float = 50.0

    def validate(self) -> None:
        values = (
            self.minimum_contact_force_n,
            self.maximum_normal_force_n,
            self.maximum_observation_age_ms,
        )
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("tactile force and age limits must be positive")
        if self.minimum_contact_force_n >= self.maximum_normal_force_n:
            raise ValueError("minimum contact force must be below maximum force")
        if (
            not math.isfinite(self.maximum_slip_probability)
            or not 0.0 <= self.maximum_slip_probability <= 1.0
        ):
            raise ValueError("maximum slip probability must be inside [0, 1]")


@dataclass(frozen=True)
class TactileChannelStatus:
    role: TactileRole
    enabled: bool
    modality: TactileModality | None
    sensor_id: str
    calibration_id: str
    sequence: int
    normal_force_n: float
    slip_probability: float
    contact: bool
    overpressure: bool
    valid: bool
    observation_age_ms: float

    def validate(self) -> None:
        if not isinstance(self.role, TactileRole):
            raise ValueError("role must be a TactileRole")
        flags = (self.enabled, self.contact, self.overpressure, self.valid)
        if not all(isinstance(value, bool) for value in flags):
            raise ValueError("tactile status flags must be booleans")
        if self.enabled:
            if not isinstance(self.modality, TactileModality):
                raise ValueError("enabled tactile channel requires a modality")
            if not self.sensor_id.strip() or not self.calibration_id.strip():
                raise ValueError("enabled tactile channel requires sensor and calibration IDs")
        elif self.modality is not None:
            raise ValueError("disabled tactile channel must not declare a modality")
        if (
            isinstance(self.sequence, bool)
            or not isinstance(self.sequence, int)
            or self.sequence < 0
        ):
            raise ValueError("tactile sequence must be a non-negative integer")
        if not math.isfinite(self.normal_force_n) or self.normal_force_n < 0:
            raise ValueError("normal_force_n must be finite and non-negative")
        if (
            not math.isfinite(self.slip_probability)
            or not 0.0 <= self.slip_probability <= 1.0
        ):
            raise ValueError("slip_probability must be inside [0, 1]")
        if not math.isfinite(self.observation_age_ms) or self.observation_age_ms < 0:
            raise ValueError("observation_age_ms must be finite and non-negative")
        if not self.enabled and any(
            (
                self.normal_force_n != 0.0,
                self.slip_probability != 0.0,
                self.contact,
                self.overpressure,
                self.valid,
            )
        ):
            raise ValueError("disabled tactile channel must contain no active measurement")

    def grasp_verified(self, limits: TactileLimits) -> bool:
        self.validate()
        limits.validate()
        return (
            self.enabled
            and self.valid
            and self.contact
            and not self.overpressure
            and limits.minimum_contact_force_n
            <= self.normal_force_n
            <= limits.maximum_normal_force_n
            and self.slip_probability <= limits.maximum_slip_probability
            and self.observation_age_ms <= limits.maximum_observation_age_ms
        )

    def alert_codes(self, limits: TactileLimits) -> tuple[str, ...]:
        self.validate()
        limits.validate()
        prefix = self.role.value
        if not self.enabled:
            return (f"{prefix}:disabled",)
        alerts: list[str] = []
        if not self.valid:
            alerts.append(f"{prefix}:invalid")
        if self.observation_age_ms > limits.maximum_observation_age_ms:
            alerts.append(f"{prefix}:stale")
        if self.overpressure or self.normal_force_n > limits.maximum_normal_force_n:
            alerts.append(f"{prefix}:overpressure")
        if self.slip_probability > limits.maximum_slip_probability:
            alerts.append(f"{prefix}:slip")
        if self.valid and not self.contact:
            alerts.append(f"{prefix}:contact_lost")
        return tuple(alerts)


@dataclass(frozen=True)
class TactileRigStatus:
    channels: tuple[TactileChannelStatus, ...]

    def validate(self) -> None:
        roles = [channel.role for channel in self.channels]
        if len(set(roles)) != len(roles) or set(roles) != ALL_TACTILE_ROLES:
            raise ValueError("tactile rig requires each gripper role exactly once")
        for channel in self.channels:
            channel.validate()

    def channel_for_arm(self, side: ArmSide) -> TactileChannelStatus:
        if not isinstance(side, ArmSide):
            raise ValueError("side must be an ArmSide")
        role = (
            TactileRole.LEFT_GRIPPER
            if side == ArmSide.LEFT
            else TactileRole.RIGHT_GRIPPER
        )
        self.validate()
        return next(channel for channel in self.channels if channel.role == role)

    def grasp_verified(self, side: ArmSide, limits: TactileLimits) -> bool:
        return self.channel_for_arm(side).grasp_verified(limits)

    def alert_codes(self, limits: TactileLimits) -> tuple[str, ...]:
        self.validate()
        return tuple(
            code
            for channel in self.channels
            for code in channel.alert_codes(limits)
        )


@runtime_checkable
class TactilePort(Protocol):
    @property
    def simulation_only(self) -> bool: ...

    def read_status(self) -> TactileRigStatus: ...


__all__ = [
    "ALL_TACTILE_ROLES",
    "TactileChannelStatus",
    "TactileLimits",
    "TactileModality",
    "TactilePort",
    "TactileRigStatus",
    "TactileRole",
]
