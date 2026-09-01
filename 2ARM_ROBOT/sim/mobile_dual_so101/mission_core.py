#!/usr/bin/env python3
"""ROS 2-free mission state machine for the MuJoCo shoe transport task.

The controller emits high-level requests only. It imports no ROS, serial, USB,
or hardware API and never sends actuator commands. Adapters execute requests
and return typed observations to this fail-closed state machine.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


SCHEMA_VERSION = "dapier.mujoco-shoe-mission.v0.1"


class MissionPhase(str, Enum):
    READY_AT_A = "ready_at_a"
    NAVIGATING_TO_B = "navigating_to_b"
    LOCALIZING_SHOE = "localizing_shoe"
    PICKING_AT_B = "picking_at_b"
    VERIFYING_GRASP = "verifying_grasp"
    NAVIGATING_TO_A = "navigating_to_a"
    PLACING_AT_A = "placing_at_a"
    VERIFYING_PLACE = "verifying_place"
    COMPLETED = "completed"
    SAFE_STOPPED = "safe_stopped"


class MissionLocation(str, Enum):
    A = "start_a"
    B = "shoe_b"


class MissionEventType(str, Enum):
    START = "start"
    NAVIGATION_RESULT = "navigation_result"
    POSE_RESULT = "pose_result"
    PICK_RESULT = "pick_result"
    GRASP_RESULT = "grasp_result"
    PLACE_RESULT = "place_result"
    RELEASE_RESULT = "release_result"
    FAULT = "fault"


class MissionCommand(str, Enum):
    NAVIGATE_TO_B = "navigate_to_b"
    ESTIMATE_SHOE_POSE = "estimate_shoe_pose"
    PICK_SHOE = "pick_shoe"
    VERIFY_GRASP = "verify_grasp"
    LATCH_TRANSPORT_HOLD = "latch_transport_hold"
    NAVIGATE_TO_A = "navigate_to_a"
    PLACE_SHOE = "place_shoe"
    VERIFY_PLACE = "verify_place"
    SAFE_STOP = "safe_stop"
    MISSION_COMPLETE = "mission_complete"


@dataclass(frozen=True)
class MissionConfig:
    max_observation_age_ms: float = 100.0
    minimum_pose_confidence: float = 0.60
    max_retries_per_stage: int = 2

    def validate(self) -> None:
        if (
            not math.isfinite(self.max_observation_age_ms)
            or self.max_observation_age_ms <= 0
        ):
            raise ValueError("max_observation_age_ms must be finite and positive")
        if (
            not math.isfinite(self.minimum_pose_confidence)
            or not 0.0 <= self.minimum_pose_confidence <= 1.0
        ):
            raise ValueError("minimum_pose_confidence must be inside [0, 1]")
        if (
            isinstance(self.max_retries_per_stage, bool)
            or not isinstance(self.max_retries_per_stage, int)
            or self.max_retries_per_stage < 0
        ):
            raise ValueError("max_retries_per_stage must be non-negative")


@dataclass(frozen=True)
class MissionEvent:
    kind: MissionEventType
    observation_seq: int
    observation_age_ms: float
    success: bool = True
    recoverable: bool = False
    base_stationary: bool = True
    location: MissionLocation | None = None
    pose_confidence: float = 1.0
    object_lifted: bool = False
    gripper_holding: bool = False
    carry_pose_clear: bool = False
    transport_hold_ok: bool = True
    object_released: bool = False
    at_start_zone: bool = False
    failure_code: str = ""
    detail: str = ""

    def validate(self) -> None:
        if not isinstance(self.kind, MissionEventType):
            raise ValueError("kind must be a MissionEventType")
        if self.location is not None and not isinstance(
            self.location, MissionLocation
        ):
            raise ValueError("location must be a MissionLocation")
        if (
            isinstance(self.observation_seq, bool)
            or not isinstance(self.observation_seq, int)
            or self.observation_seq < 0
        ):
            raise ValueError("observation_seq must be a non-negative integer")
        if (
            not math.isfinite(self.observation_age_ms)
            or self.observation_age_ms < 0
        ):
            raise ValueError("observation_age_ms must be finite and non-negative")
        if (
            not math.isfinite(self.pose_confidence)
            or not 0.0 <= self.pose_confidence <= 1.0
        ):
            raise ValueError("pose_confidence must be inside [0, 1]")
        if len(self.failure_code) > 100 or len(self.detail) > 500:
            raise ValueError("failure_code or detail is too long")
        if self.kind == MissionEventType.NAVIGATION_RESULT and self.location is None:
            raise ValueError("navigation_result requires a location")
        if self.kind == MissionEventType.FAULT and not self.failure_code.strip():
            raise ValueError("fault requires failure_code")
        if not self.success and not self.failure_code.strip():
            raise ValueError("unsuccessful event requires failure_code")


@dataclass(frozen=True)
class MissionTransition:
    sequence: int
    previous_phase: MissionPhase
    phase: MissionPhase
    event: MissionEventType
    commands: tuple[MissionCommand, ...]
    reason: str
    retry_count: int = 0
    hardware_dispatch_authorized: bool = False
    hardware_execution: bool = False

    def as_report(self) -> dict[str, object]:
        return {
            "schema_version": SCHEMA_VERSION,
            "sequence": self.sequence,
            "previous_phase": self.previous_phase.value,
            "phase": self.phase.value,
            "event": self.event.value,
            "commands": [command.value for command in self.commands],
            "reason": self.reason,
            "retry_count": self.retry_count,
            "hardware_dispatch_authorized": self.hardware_dispatch_authorized,
            "hardware_execution": self.hardware_execution,
        }


_EXPECTED_EVENTS = {
    MissionPhase.READY_AT_A: MissionEventType.START,
    MissionPhase.NAVIGATING_TO_B: MissionEventType.NAVIGATION_RESULT,
    MissionPhase.LOCALIZING_SHOE: MissionEventType.POSE_RESULT,
    MissionPhase.PICKING_AT_B: MissionEventType.PICK_RESULT,
    MissionPhase.VERIFYING_GRASP: MissionEventType.GRASP_RESULT,
    MissionPhase.NAVIGATING_TO_A: MissionEventType.NAVIGATION_RESULT,
    MissionPhase.PLACING_AT_A: MissionEventType.PLACE_RESULT,
    MissionPhase.VERIFYING_PLACE: MissionEventType.RELEASE_RESULT,
}

_MANIPULATION_EVENTS = frozenset(
    {
        MissionEventType.POSE_RESULT,
        MissionEventType.PICK_RESULT,
        MissionEventType.GRASP_RESULT,
        MissionEventType.PLACE_RESULT,
        MissionEventType.RELEASE_RESULT,
    }
)


class MissionController:
    """Deterministic A-to-B pick, return-to-A, and place supervisor."""

    def __init__(self, config: MissionConfig | None = None) -> None:
        self.config = config or MissionConfig()
        self.config.validate()
        self.phase = MissionPhase.READY_AT_A
        self._sequence = 0
        self._last_observation_seq = -1
        self._retry_counts: dict[str, int] = {}
        self._carrying = False
        self._history: list[MissionTransition] = []

    @property
    def history(self) -> tuple[MissionTransition, ...]:
        return tuple(self._history)

    @property
    def carrying(self) -> bool:
        return self._carrying

    def dispatch(self, event: MissionEvent) -> MissionTransition:
        event.validate()
        previous = self.phase
        if previous in {MissionPhase.COMPLETED, MissionPhase.SAFE_STOPPED}:
            return self._record(
                previous,
                previous,
                event,
                (),
                "terminal mission ignored event",
            )
        if event.kind == MissionEventType.FAULT:
            return self._safe_stop(previous, event, f"fault:{event.failure_code}")
        if event.observation_seq <= self._last_observation_seq:
            return self._safe_stop(previous, event, "observation sequence is stale")
        self._last_observation_seq = event.observation_seq
        if event.observation_age_ms > self.config.max_observation_age_ms:
            return self._safe_stop(previous, event, "observation age exceeded limit")
        expected = _EXPECTED_EVENTS[previous]
        if event.kind != expected:
            return self._safe_stop(
                previous,
                event,
                f"unexpected event {event.kind.value}; expected {expected.value}",
            )
        if event.kind in _MANIPULATION_EVENTS and not event.base_stationary:
            return self._safe_stop(previous, event, "manipulation requires stationary base")

        if previous == MissionPhase.READY_AT_A:
            if not event.base_stationary:
                return self._safe_stop(previous, event, "mission start requires stationary base")
            return self._advance(
                previous,
                MissionPhase.NAVIGATING_TO_B,
                event,
                (MissionCommand.NAVIGATE_TO_B,),
                "mission started at A",
            )
        if previous == MissionPhase.NAVIGATING_TO_B:
            return self._navigation_to_b(event)
        if previous == MissionPhase.LOCALIZING_SHOE:
            return self._pose_result(event)
        if previous == MissionPhase.PICKING_AT_B:
            return self._pick_result(event)
        if previous == MissionPhase.VERIFYING_GRASP:
            return self._grasp_result(event)
        if previous == MissionPhase.NAVIGATING_TO_A:
            return self._navigation_to_a(event)
        if previous == MissionPhase.PLACING_AT_A:
            return self._place_result(event)
        if previous == MissionPhase.VERIFYING_PLACE:
            return self._release_result(event)
        raise RuntimeError(f"unhandled mission phase: {previous.value}")

    def _navigation_to_b(self, event: MissionEvent) -> MissionTransition:
        if event.location != MissionLocation.B:
            return self._safe_stop(self.phase, event, "navigation result location is not B")
        if not event.success:
            if not event.base_stationary:
                return self._safe_stop(
                    self.phase,
                    event,
                    "navigation failure did not settle base at B",
                )
            return self._retry_or_stop(
                "navigate_to_b",
                MissionPhase.NAVIGATING_TO_B,
                (MissionCommand.NAVIGATE_TO_B,),
                event,
            )
        if not event.base_stationary:
            return self._safe_stop(self.phase, event, "base did not settle at B")
        return self._advance(
            self.phase,
            MissionPhase.LOCALIZING_SHOE,
            event,
            (MissionCommand.ESTIMATE_SHOE_POSE,),
            "B reached with settled base",
        )

    def _pose_result(self, event: MissionEvent) -> MissionTransition:
        if not event.success or event.pose_confidence < self.config.minimum_pose_confidence:
            return self._retry_or_stop(
                "localize_shoe",
                MissionPhase.LOCALIZING_SHOE,
                (MissionCommand.ESTIMATE_SHOE_POSE,),
                event,
                reason="shoe pose confidence below gate",
            )
        return self._advance(
            self.phase,
            MissionPhase.PICKING_AT_B,
            event,
            (MissionCommand.PICK_SHOE,),
            "shoe pose accepted",
        )

    def _pick_result(self, event: MissionEvent) -> MissionTransition:
        if not event.success:
            return self._retry_or_stop(
                "pick_at_b",
                MissionPhase.LOCALIZING_SHOE,
                (MissionCommand.ESTIMATE_SHOE_POSE,),
                event,
            )
        return self._advance(
            self.phase,
            MissionPhase.VERIFYING_GRASP,
            event,
            (MissionCommand.VERIFY_GRASP,),
            "pick execution completed; verification required",
        )

    def _grasp_result(self, event: MissionEvent) -> MissionTransition:
        grasp_ok = (
            event.success
            and event.object_lifted
            and event.gripper_holding
            and event.carry_pose_clear
        )
        if not grasp_ok:
            return self._retry_or_stop(
                "pick_at_b",
                MissionPhase.LOCALIZING_SHOE,
                (MissionCommand.ESTIMATE_SHOE_POSE,),
                event,
                reason="grasp verification failed",
            )
        self._carrying = True
        return self._advance(
            self.phase,
            MissionPhase.NAVIGATING_TO_A,
            event,
            (
                MissionCommand.LATCH_TRANSPORT_HOLD,
                MissionCommand.NAVIGATE_TO_A,
            ),
            "grasp and carry pose verified",
        )

    def _navigation_to_a(self, event: MissionEvent) -> MissionTransition:
        if event.location != MissionLocation.A:
            return self._safe_stop(self.phase, event, "navigation result location is not A")
        if not self._carrying or not event.transport_hold_ok:
            return self._safe_stop(self.phase, event, "transport hold was lost")
        if not event.success:
            if not event.base_stationary:
                return self._safe_stop(
                    self.phase,
                    event,
                    "navigation failure did not settle base at A",
                )
            return self._retry_or_stop(
                "navigate_to_a",
                MissionPhase.NAVIGATING_TO_A,
                (MissionCommand.NAVIGATE_TO_A,),
                event,
            )
        if not event.base_stationary:
            return self._safe_stop(self.phase, event, "base did not settle at A")
        return self._advance(
            self.phase,
            MissionPhase.PLACING_AT_A,
            event,
            (MissionCommand.PLACE_SHOE,),
            "A reached with transport hold and settled base",
        )

    def _place_result(self, event: MissionEvent) -> MissionTransition:
        if not event.success:
            return self._retry_or_stop(
                "place_at_a",
                MissionPhase.PLACING_AT_A,
                (MissionCommand.PLACE_SHOE,),
                event,
            )
        return self._advance(
            self.phase,
            MissionPhase.VERIFYING_PLACE,
            event,
            (MissionCommand.VERIFY_PLACE,),
            "place execution completed; release verification required",
        )

    def _release_result(self, event: MissionEvent) -> MissionTransition:
        release_ok = event.success and event.object_released and event.at_start_zone
        if not release_ok:
            return self._retry_or_stop(
                "place_at_a",
                MissionPhase.PLACING_AT_A,
                (MissionCommand.PLACE_SHOE,),
                event,
                reason="release verification failed",
            )
        self._carrying = False
        return self._advance(
            self.phase,
            MissionPhase.COMPLETED,
            event,
            (MissionCommand.MISSION_COMPLETE,),
            "shoe released inside start zone",
        )

    def _retry_or_stop(
        self,
        stage: str,
        retry_phase: MissionPhase,
        commands: tuple[MissionCommand, ...],
        event: MissionEvent,
        *,
        reason: str | None = None,
    ) -> MissionTransition:
        failure = reason or event.failure_code or "stage failed"
        if not event.recoverable:
            return self._safe_stop(self.phase, event, failure)
        retry_count = self._retry_counts.get(stage, 0) + 1
        self._retry_counts[stage] = retry_count
        if retry_count > self.config.max_retries_per_stage:
            return self._safe_stop(
                self.phase,
                event,
                f"{stage} retry budget exhausted: {failure}",
                retry_count=retry_count,
            )
        return self._advance(
            self.phase,
            retry_phase,
            event,
            commands,
            f"retry {retry_count}/{self.config.max_retries_per_stage}: {failure}",
            retry_count=retry_count,
        )

    def _safe_stop(
        self,
        previous: MissionPhase,
        event: MissionEvent,
        reason: str,
        *,
        retry_count: int = 0,
    ) -> MissionTransition:
        return self._advance(
            previous,
            MissionPhase.SAFE_STOPPED,
            event,
            (MissionCommand.SAFE_STOP,),
            reason,
            retry_count=retry_count,
        )

    def _advance(
        self,
        previous: MissionPhase,
        phase: MissionPhase,
        event: MissionEvent,
        commands: tuple[MissionCommand, ...],
        reason: str,
        *,
        retry_count: int = 0,
    ) -> MissionTransition:
        self.phase = phase
        return self._record(
            previous,
            phase,
            event,
            commands,
            reason,
            retry_count=retry_count,
        )

    def _record(
        self,
        previous: MissionPhase,
        phase: MissionPhase,
        event: MissionEvent,
        commands: tuple[MissionCommand, ...],
        reason: str,
        *,
        retry_count: int = 0,
    ) -> MissionTransition:
        transition = MissionTransition(
            sequence=self._sequence,
            previous_phase=previous,
            phase=phase,
            event=event.kind,
            commands=commands,
            reason=reason,
            retry_count=retry_count,
        )
        self._sequence += 1
        self._history.append(transition)
        return transition


__all__ = [
    "MissionCommand",
    "MissionConfig",
    "MissionController",
    "MissionEvent",
    "MissionEventType",
    "MissionLocation",
    "MissionPhase",
    "MissionTransition",
    "SCHEMA_VERSION",
]
