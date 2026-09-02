"""Deterministic sensor-first mission coordinator for the mobile dual-arm robot.

This state machine produces semantic proposals only. It neither publishes ROS
messages nor opens hardware. Every transition is fail-closed on stale vision,
base motion, timeout, E-stop, or unverified calibration.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any


class MissionPhase(str, Enum):
    DISABLED = "disabled"
    SEARCH = "search"
    DOCKING = "docking"
    SETTLING = "settling"
    REOBSERVE = "reobserve"
    ARM_PLAN = "arm_plan"
    ARM_EXECUTION = "arm_execution"
    GRASP_VERIFY = "grasp_verify"
    CARRY = "carry"
    PLACE = "place"
    COMPLETE = "complete"
    FAULT = "fault"


@dataclass(frozen=True)
class MissionConfig:
    minimum_target_confidence: float = 0.40
    max_vision_age_ns: int = 250_000_000
    docking_standoff_m: float = 0.20
    docking_position_tolerance_m: float = 0.025
    docking_lateral_tolerance_m: float = 0.020
    maximum_linear_speed_mps: float = 0.08
    maximum_angular_speed_rad_s: float = 0.35
    settled_linear_speed_mps: float = 0.01
    settled_angular_speed_rad_s: float = 0.03
    settle_dwell_ns: int = 500_000_000
    phase_timeout_ns: int = 10_000_000_000

    def validate(self) -> None:
        finite = (
            self.minimum_target_confidence,
            self.docking_standoff_m,
            self.docking_position_tolerance_m,
            self.docking_lateral_tolerance_m,
            self.maximum_linear_speed_mps,
            self.maximum_angular_speed_rad_s,
            self.settled_linear_speed_mps,
            self.settled_angular_speed_rad_s,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in finite):
            raise ValueError("mission configuration contains invalid numeric limits")
        if not 0.0 <= self.minimum_target_confidence <= 1.0:
            raise ValueError("minimum_target_confidence must be in [0, 1]")
        for name in ("max_vision_age_ns", "settle_dwell_ns", "phase_timeout_ns"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class MissionInput:
    now_ns: int
    vision_timestamp_ns: int | None = None
    target_position_base_m: tuple[float, float, float] | None = None
    target_confidence: float = 0.0
    calibration_verified: bool = False
    base_linear_speed_mps: float = 0.0
    base_angular_speed_rad_s: float = 0.0
    arm_reachable: bool = False
    arm_plan_ready: bool = False
    arm_execution_complete: bool = False
    grasp_verified: bool = False
    carry_destination_ready: bool = False
    place_complete: bool = False
    operator_enable: bool = False
    e_stop: bool = False
    fault: str | None = None


@dataclass(frozen=True)
class MissionDecision:
    phase: MissionPhase
    proposal: str
    reason: str
    base_linear_x_mps: float = 0.0
    base_angular_z_rad_s: float = 0.0
    control_authorized: bool = False
    hardware_execution: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "phase": self.phase.value,
            "proposal": self.proposal,
            "reason": self.reason,
            "base_linear_x_mps": self.base_linear_x_mps,
            "base_angular_z_rad_s": self.base_angular_z_rad_s,
            "control_authorized": self.control_authorized,
            "hardware_execution": self.hardware_execution,
        }


class MissionCoordinator:
    def __init__(self, config: MissionConfig = MissionConfig()) -> None:
        config.validate()
        self.config = config
        self.phase = MissionPhase.DISABLED
        self.phase_started_ns = 0
        self.settle_started_ns: int | None = None

    def _transition(self, phase: MissionPhase, now_ns: int) -> None:
        self.phase = phase
        self.phase_started_ns = now_ns
        if phase != MissionPhase.SETTLING:
            self.settle_started_ns = None

    def _decision(
        self,
        proposal: str,
        reason: str,
        *,
        linear: float = 0.0,
        angular: float = 0.0,
    ) -> MissionDecision:
        return MissionDecision(
            phase=self.phase,
            proposal=proposal,
            reason=reason,
            base_linear_x_mps=linear,
            base_angular_z_rad_s=angular,
        )

    def _valid_vision(self, observation: MissionInput) -> bool:
        if observation.vision_timestamp_ns is None or observation.target_position_base_m is None:
            return False
        if observation.vision_timestamp_ns > observation.now_ns:
            return False
        if observation.now_ns - observation.vision_timestamp_ns > self.config.max_vision_age_ns:
            return False
        if not math.isfinite(observation.target_confidence):
            return False
        if observation.target_confidence < self.config.minimum_target_confidence:
            return False
        return len(observation.target_position_base_m) == 3 and all(
            math.isfinite(value) for value in observation.target_position_base_m
        )

    def update(self, observation: MissionInput) -> MissionDecision:
        if isinstance(observation.now_ns, bool) or observation.now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        numeric = (
            observation.target_confidence,
            observation.base_linear_speed_mps,
            observation.base_angular_speed_rad_s,
        )
        if not all(math.isfinite(value) for value in numeric):
            self._transition(MissionPhase.FAULT, observation.now_ns)
            return self._decision("hold", "non-finite mission input")

        if observation.e_stop:
            self._transition(MissionPhase.FAULT, observation.now_ns)
            return self._decision("emergency_stop", "E-stop is active")
        if observation.fault:
            self._transition(MissionPhase.FAULT, observation.now_ns)
            return self._decision("hold", f"upstream fault: {observation.fault}")
        if not observation.operator_enable:
            self._transition(MissionPhase.DISABLED, observation.now_ns)
            return self._decision("hold", "operator enable is false")
        if not observation.calibration_verified:
            self._transition(MissionPhase.FAULT, observation.now_ns)
            return self._decision("hold", "camera/robot calibration is not verified")

        if self.phase == MissionPhase.DISABLED:
            self._transition(MissionPhase.SEARCH, observation.now_ns)
        elif (
            self.phase not in (MissionPhase.COMPLETE, MissionPhase.FAULT)
            and observation.now_ns - self.phase_started_ns > self.config.phase_timeout_ns
        ):
            self._transition(MissionPhase.FAULT, observation.now_ns)
            return self._decision("hold", "mission phase timeout")

        if self.phase == MissionPhase.FAULT:
            return self._decision("hold", "fault is latched; reset is required")
        if self.phase == MissionPhase.COMPLETE:
            return self._decision("hold", "mission is complete")

        if self.phase == MissionPhase.SEARCH:
            if not self._valid_vision(observation):
                return self._decision("request_detection", "waiting for fresh sensor target")
            self._transition(
                MissionPhase.SETTLING if observation.arm_reachable else MissionPhase.DOCKING,
                observation.now_ns,
            )

        if self.phase == MissionPhase.DOCKING:
            if not self._valid_vision(observation):
                self._transition(MissionPhase.SEARCH, observation.now_ns)
                return self._decision("hold", "target was lost or became stale")
            assert observation.target_position_base_m is not None
            forward, lateral, _ = observation.target_position_base_m
            longitudinal_error = forward - self.config.docking_standoff_m
            if (
                abs(longitudinal_error) <= self.config.docking_position_tolerance_m
                and abs(lateral) <= self.config.docking_lateral_tolerance_m
            ):
                self._transition(MissionPhase.SETTLING, observation.now_ns)
                return self._decision("hold", "docking pose reached; begin settle dwell")
            linear = max(
                -self.config.maximum_linear_speed_mps,
                min(self.config.maximum_linear_speed_mps, 0.8 * longitudinal_error),
            )
            angular = max(
                -self.config.maximum_angular_speed_rad_s,
                min(self.config.maximum_angular_speed_rad_s, 2.0 * lateral),
            )
            return self._decision(
                "base_twist",
                "sensor-derived docking correction",
                linear=linear,
                angular=angular,
            )

        if self.phase == MissionPhase.SETTLING:
            settled = (
                abs(observation.base_linear_speed_mps)
                <= self.config.settled_linear_speed_mps
                and abs(observation.base_angular_speed_rad_s)
                <= self.config.settled_angular_speed_rad_s
            )
            if not settled:
                self.settle_started_ns = None
                return self._decision("hold", "waiting for base velocity to settle")
            if self.settle_started_ns is None:
                self.settle_started_ns = observation.now_ns
                return self._decision("hold", "settle dwell started")
            if observation.now_ns - self.settle_started_ns < self.config.settle_dwell_ns:
                return self._decision("hold", "settle dwell in progress")
            self._transition(MissionPhase.REOBSERVE, observation.now_ns)

        if self.phase == MissionPhase.REOBSERVE:
            if not self._valid_vision(observation):
                return self._decision("request_detection", "fresh post-docking target required")
            if not observation.arm_reachable:
                self._transition(MissionPhase.DOCKING, observation.now_ns)
                return self._decision("hold", "target remains outside arm workspace")
            self._transition(MissionPhase.ARM_PLAN, observation.now_ns)
            return self._decision("arm_plan", "fresh reachable sensor target accepted")

        if self.phase == MissionPhase.ARM_PLAN:
            if not observation.arm_plan_ready:
                return self._decision("arm_plan", "waiting for collision-checked arm plan")
            self._transition(MissionPhase.ARM_EXECUTION, observation.now_ns)
            return self._decision("arm_execute", "bounded arm intent may be proposed")

        if self.phase == MissionPhase.ARM_EXECUTION:
            if not observation.arm_execution_complete:
                return self._decision("hold", "waiting for measured arm completion")
            self._transition(MissionPhase.GRASP_VERIFY, observation.now_ns)
            return self._decision("verify_grasp", "arm motion complete; verify contact")

        if self.phase == MissionPhase.GRASP_VERIFY:
            if not observation.grasp_verified:
                return self._decision("verify_grasp", "contact/slip evidence not yet valid")
            self._transition(MissionPhase.CARRY, observation.now_ns)
            return self._decision("carry_plan", "grasp verified")

        if self.phase == MissionPhase.CARRY:
            if not observation.carry_destination_ready:
                return self._decision("carry_plan", "waiting for safe destination")
            self._transition(MissionPhase.PLACE, observation.now_ns)
            return self._decision("place", "destination accepted")

        if self.phase == MissionPhase.PLACE:
            if not observation.place_complete:
                return self._decision("place", "waiting for measured placement completion")
            self._transition(MissionPhase.COMPLETE, observation.now_ns)
            return self._decision("hold", "mission completed")

        self._transition(MissionPhase.FAULT, observation.now_ns)
        return self._decision("hold", "unhandled mission state")

    def reset_fault(self, *, now_ns: int, e_stop: bool, operator_enable: bool) -> None:
        if self.phase != MissionPhase.FAULT:
            raise ValueError("reset_fault is valid only from fault state")
        if e_stop or operator_enable:
            raise ValueError("clear E-stop and operator enable before resetting")
        self._transition(MissionPhase.DISABLED, now_ns)
