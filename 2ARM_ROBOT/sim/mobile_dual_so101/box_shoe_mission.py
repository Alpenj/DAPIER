"""ROS 2-free coordinator for the box-open-and-extract-shoe task.

The right SO-101 opens and holds the lid while the left SO-101 extracts the
shoe proxy. This module contains no MuJoCo, serial, USB, or hardware dispatch.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import math


SCHEMA_VERSION = "dapier.box-shoe-mission.v0.1"


class BoxShoePhase(str, Enum):
    READY_AT_A = "ready_at_a"
    NAVIGATING_TO_B = "navigating_to_b"
    LOCALIZING_BOX = "localizing_box"
    OPENING_BOX_RIGHT = "opening_box_right"
    VERIFYING_BOX_OPEN = "verifying_box_open"
    LOCALIZING_SHOE = "localizing_shoe"
    EXTRACTING_SHOE_LEFT = "extracting_shoe_left"
    VERIFYING_SHOE_GRASP = "verifying_shoe_grasp"
    CLEARING_BOX = "clearing_box"
    MOVING_BOTH_TO_TRANSPORT = "moving_both_to_transport"
    NAVIGATING_TO_A = "navigating_to_a"
    PLACING_AT_A_LEFT = "placing_at_a_left"
    VERIFYING_PLACE = "verifying_place"
    COMPLETED = "completed"
    SAFE_STOP_REQUESTED = "safe_stop_requested"
    SAFE_STOPPED = "safe_stopped"


class BoxShoeEventType(str, Enum):
    START = "start"
    NAVIGATION_RESULT = "navigation_result"
    BOX_POSE_RESULT = "box_pose_result"
    BOX_OPEN_RESULT = "box_open_result"
    LID_HOLD_RESULT = "lid_hold_result"
    SHOE_POSE_RESULT = "shoe_pose_result"
    EXTRACTION_RESULT = "extraction_result"
    SHOE_GRASP_RESULT = "shoe_grasp_result"
    SHOE_CLEAR_RESULT = "shoe_clear_result"
    TRANSPORT_POSE_RESULT = "transport_pose_result"
    PLACE_RESULT = "place_result"
    RELEASE_RESULT = "release_result"
    SAFE_STOP_RESULT = "safe_stop_result"
    FAULT = "fault"


class BoxShoeCommand(str, Enum):
    NAVIGATE_TO_B = "navigate_to_b"
    LOCALIZE_BOX_LID_EDGE = "localize_box_lid_edge"
    OPEN_BOX_WITH_RIGHT = "open_box_with_right"
    HOLD_LID_WITH_RIGHT = "hold_lid_with_right"
    VERIFY_LID_OPEN = "verify_lid_open"
    LOCALIZE_SHOE = "localize_shoe"
    EXTRACT_SHOE_WITH_LEFT = "extract_shoe_with_left"
    VERIFY_LEFT_SHOE_GRASP = "verify_left_shoe_grasp"
    CLEAR_SHOE_FROM_BOX = "clear_shoe_from_box"
    RELEASE_LID_WITH_RIGHT = "release_lid_with_right"
    MOVE_BOTH_TO_TRANSPORT = "move_both_to_transport"
    NAVIGATE_TO_A = "navigate_to_a"
    PLACE_SHOE_WITH_LEFT = "place_shoe_with_left"
    VERIFY_PLACE = "verify_place"
    MISSION_COMPLETE = "mission_complete"
    SAFE_STOP = "safe_stop"


@dataclass(frozen=True)
class BoxShoeMissionConfig:
    max_observation_age_ms: float = 100.0
    minimum_pose_confidence: float = 0.60
    minimum_lid_open_angle_rad: float = math.radians(55.0)

    def validate(self) -> None:
        values = (
            self.max_observation_age_ms,
            self.minimum_lid_open_angle_rad,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("age and lid angle limits must be finite and positive")
        if self.minimum_lid_open_angle_rad > math.pi:
            raise ValueError("minimum lid angle cannot exceed pi")
        if (
            not math.isfinite(self.minimum_pose_confidence)
            or not 0.0 <= self.minimum_pose_confidence <= 1.0
        ):
            raise ValueError("minimum_pose_confidence must be inside [0, 1]")


@dataclass(frozen=True)
class BoxShoeEvent:
    kind: BoxShoeEventType
    observation_seq: int
    observation_age_ms: float
    success: bool = True
    base_stationary: bool = True
    location: str = ""
    pose_confidence: float = 1.0
    lid_open_angle_rad: float = 0.0
    lid_held_by_right: bool = False
    lid_safe: bool = False
    shoe_lifted: bool = False
    shoe_clear_of_box: bool = False
    shoe_held_by_left: bool = False
    object_released: bool = False
    at_start_zone: bool = False
    left_arm_moved: bool = False
    right_arm_moved: bool = False
    left_tactile_available: bool = False
    right_tactile_available: bool = False
    left_tactile_contact: bool = False
    right_tactile_contact: bool = False
    actuators_stopped: bool = False
    failure_code: str = ""
    detail: str = ""

    def validate(self) -> None:
        if not isinstance(self.kind, BoxShoeEventType):
            raise ValueError("kind must be a BoxShoeEventType")
        if (
            isinstance(self.observation_seq, bool)
            or not isinstance(self.observation_seq, int)
            or self.observation_seq < 0
        ):
            raise ValueError("observation_seq must be a non-negative integer")
        if (
            not math.isfinite(self.observation_age_ms)
            or self.observation_age_ms < 0.0
        ):
            raise ValueError("observation_age_ms must be finite and non-negative")
        if (
            not math.isfinite(self.pose_confidence)
            or not 0.0 <= self.pose_confidence <= 1.0
        ):
            raise ValueError("pose_confidence must be inside [0, 1]")
        if (
            not math.isfinite(self.lid_open_angle_rad)
            or not 0.0 <= self.lid_open_angle_rad <= math.pi
        ):
            raise ValueError("lid_open_angle_rad must be inside [0, pi]")
        flags = (
            self.success,
            self.base_stationary,
            self.lid_held_by_right,
            self.lid_safe,
            self.shoe_lifted,
            self.shoe_clear_of_box,
            self.shoe_held_by_left,
            self.object_released,
            self.at_start_zone,
            self.left_arm_moved,
            self.right_arm_moved,
            self.left_tactile_available,
            self.right_tactile_available,
            self.left_tactile_contact,
            self.right_tactile_contact,
            self.actuators_stopped,
        )
        if not all(isinstance(value, bool) for value in flags):
            raise ValueError("mission status flags must be booleans")
        if self.left_tactile_contact and not self.left_tactile_available:
            raise ValueError("left tactile contact requires an available sensor")
        if self.right_tactile_contact and not self.right_tactile_available:
            raise ValueError("right tactile contact requires an available sensor")
        if not self.success and not self.failure_code.strip():
            raise ValueError("unsuccessful event requires failure_code")
        if self.kind == BoxShoeEventType.FAULT and not self.failure_code.strip():
            raise ValueError("fault requires failure_code")
        if len(self.failure_code) > 100 or len(self.detail) > 500:
            raise ValueError("failure_code or detail is too long")


@dataclass(frozen=True)
class BoxShoeTransition:
    sequence: int
    previous_phase: BoxShoePhase
    phase: BoxShoePhase
    event: BoxShoeEventType
    commands: tuple[BoxShoeCommand, ...]
    reason: str
    left_arm_participated: bool
    right_arm_participated: bool
    hardware_dispatch_authorized: bool = False
    hardware_execution: bool = False

    def as_report(self) -> dict[str, object]:
        report = asdict(self)
        report["schema_version"] = SCHEMA_VERSION
        report["previous_phase"] = self.previous_phase.value
        report["phase"] = self.phase.value
        report["event"] = self.event.value
        report["commands"] = [command.value for command in self.commands]
        return report


_EXPECTED = {
    BoxShoePhase.READY_AT_A: BoxShoeEventType.START,
    BoxShoePhase.NAVIGATING_TO_B: BoxShoeEventType.NAVIGATION_RESULT,
    BoxShoePhase.LOCALIZING_BOX: BoxShoeEventType.BOX_POSE_RESULT,
    BoxShoePhase.OPENING_BOX_RIGHT: BoxShoeEventType.BOX_OPEN_RESULT,
    BoxShoePhase.VERIFYING_BOX_OPEN: BoxShoeEventType.LID_HOLD_RESULT,
    BoxShoePhase.LOCALIZING_SHOE: BoxShoeEventType.SHOE_POSE_RESULT,
    BoxShoePhase.EXTRACTING_SHOE_LEFT: BoxShoeEventType.EXTRACTION_RESULT,
    BoxShoePhase.VERIFYING_SHOE_GRASP: BoxShoeEventType.SHOE_GRASP_RESULT,
    BoxShoePhase.CLEARING_BOX: BoxShoeEventType.SHOE_CLEAR_RESULT,
    BoxShoePhase.MOVING_BOTH_TO_TRANSPORT: BoxShoeEventType.TRANSPORT_POSE_RESULT,
    BoxShoePhase.NAVIGATING_TO_A: BoxShoeEventType.NAVIGATION_RESULT,
    BoxShoePhase.PLACING_AT_A_LEFT: BoxShoeEventType.PLACE_RESULT,
    BoxShoePhase.VERIFYING_PLACE: BoxShoeEventType.RELEASE_RESULT,
    BoxShoePhase.SAFE_STOP_REQUESTED: BoxShoeEventType.SAFE_STOP_RESULT,
}

_MANIPULATION_EVENTS = frozenset(
    event
    for event in BoxShoeEventType
    if event
    not in {
        BoxShoeEventType.START,
        BoxShoeEventType.NAVIGATION_RESULT,
        BoxShoeEventType.FAULT,
    }
)


class BoxShoeMissionController:
    def __init__(self, config: BoxShoeMissionConfig | None = None) -> None:
        self.config = config or BoxShoeMissionConfig()
        self.config.validate()
        self.phase = BoxShoePhase.READY_AT_A
        self._last_observation_seq = -1
        self._sequence = 0
        self._left_participated = False
        self._right_participated = False
        self._shoe_carrying = False
        self._history: list[BoxShoeTransition] = []

    @property
    def history(self) -> tuple[BoxShoeTransition, ...]:
        return tuple(self._history)

    def dispatch(self, event: BoxShoeEvent) -> BoxShoeTransition:
        event.validate()
        previous = self.phase
        if previous in {BoxShoePhase.COMPLETED, BoxShoePhase.SAFE_STOPPED}:
            return self._record(previous, event, (), "terminal mission ignored event")
        if event.kind == BoxShoeEventType.FAULT:
            return self._stop(event, f"fault:{event.failure_code}")
        if event.observation_seq <= self._last_observation_seq:
            return self._stop(event, "observation sequence is stale")
        self._last_observation_seq = event.observation_seq
        if event.observation_age_ms > self.config.max_observation_age_ms:
            return self._stop(event, "observation age exceeded limit")
        expected = _EXPECTED[previous]
        if event.kind != expected:
            return self._stop(
                event,
                f"unexpected event {event.kind.value}; expected {expected.value}",
            )
        if event.kind in _MANIPULATION_EVENTS and not event.base_stationary:
            return self._stop(event, "manipulation requires stationary base")

        if previous == BoxShoePhase.READY_AT_A:
            if not event.base_stationary:
                return self._stop(event, "mission start requires stationary base")
            return self._advance(
                event,
                BoxShoePhase.NAVIGATING_TO_B,
                (BoxShoeCommand.NAVIGATE_TO_B,),
                "box-shoe mission started at A",
            )
        if previous == BoxShoePhase.NAVIGATING_TO_B:
            return self._check(
                event.location == "B" and event.success and event.base_stationary,
                event,
                BoxShoePhase.LOCALIZING_BOX,
                (BoxShoeCommand.LOCALIZE_BOX_LID_EDGE,),
                "B reached with settled base",
                "navigation to B failed",
            )
        if previous == BoxShoePhase.LOCALIZING_BOX:
            return self._check(
                event.success
                and event.pose_confidence >= self.config.minimum_pose_confidence,
                event,
                BoxShoePhase.OPENING_BOX_RIGHT,
                (BoxShoeCommand.OPEN_BOX_WITH_RIGHT,),
                "box and lid edge pose accepted",
                "box pose confidence below gate",
            )
        if previous == BoxShoePhase.OPENING_BOX_RIGHT:
            opened = (
                event.success
                and event.right_arm_moved
                and event.right_tactile_available
                and event.right_tactile_contact
                and event.lid_open_angle_rad
                >= self.config.minimum_lid_open_angle_rad
            )
            if opened:
                self._right_participated = True
            return self._check(
                opened,
                event,
                BoxShoePhase.VERIFYING_BOX_OPEN,
                (
                    BoxShoeCommand.HOLD_LID_WITH_RIGHT,
                    BoxShoeCommand.VERIFY_LID_OPEN,
                ),
                "right arm opened the lid",
                "right-arm lid opening was not verified",
            )
        if previous == BoxShoePhase.VERIFYING_BOX_OPEN:
            return self._check(
                self._right_lid_hold_ok(event),
                event,
                BoxShoePhase.LOCALIZING_SHOE,
                (
                    BoxShoeCommand.HOLD_LID_WITH_RIGHT,
                    BoxShoeCommand.LOCALIZE_SHOE,
                ),
                "right-arm lid hold verified",
                "right-arm lid hold was lost",
            )
        if previous == BoxShoePhase.LOCALIZING_SHOE:
            return self._check(
                event.success
                and event.pose_confidence >= self.config.minimum_pose_confidence
                and self._right_lid_hold_ok(event),
                event,
                BoxShoePhase.EXTRACTING_SHOE_LEFT,
                (
                    BoxShoeCommand.HOLD_LID_WITH_RIGHT,
                    BoxShoeCommand.EXTRACT_SHOE_WITH_LEFT,
                ),
                "shoe pose accepted while right arm holds lid",
                "shoe pose or right-arm lid hold failed",
            )
        if previous == BoxShoePhase.EXTRACTING_SHOE_LEFT:
            extracted = (
                event.success
                and event.left_arm_moved
                and self._right_lid_hold_ok(event)
            )
            if extracted:
                self._left_participated = True
            return self._check(
                extracted,
                event,
                BoxShoePhase.VERIFYING_SHOE_GRASP,
                (BoxShoeCommand.VERIFY_LEFT_SHOE_GRASP,),
                "left arm extraction motion completed",
                "left extraction or right lid hold failed",
            )
        if previous == BoxShoePhase.VERIFYING_SHOE_GRASP:
            grasped = (
                event.success
                and event.shoe_lifted
                and event.shoe_held_by_left
                and event.left_tactile_available
                and event.left_tactile_contact
                and self._right_lid_hold_ok(event)
            )
            if grasped:
                self._shoe_carrying = True
            return self._check(
                grasped,
                event,
                BoxShoePhase.CLEARING_BOX,
                (BoxShoeCommand.CLEAR_SHOE_FROM_BOX,),
                "left shoe grasp and right lid hold verified",
                "shoe grasp or right lid hold failed",
            )
        if previous == BoxShoePhase.CLEARING_BOX:
            return self._check(
                event.success
                and event.shoe_clear_of_box
                and event.shoe_held_by_left
                and self._right_lid_hold_ok(event),
                event,
                BoxShoePhase.MOVING_BOTH_TO_TRANSPORT,
                (
                    BoxShoeCommand.RELEASE_LID_WITH_RIGHT,
                    BoxShoeCommand.MOVE_BOTH_TO_TRANSPORT,
                ),
                "shoe cleared box; both arms may transition",
                "shoe clearance or lid hold failed",
            )
        if previous == BoxShoePhase.MOVING_BOTH_TO_TRANSPORT:
            ready = (
                event.success
                and event.left_arm_moved
                and event.right_arm_moved
                and event.shoe_clear_of_box
                and event.shoe_held_by_left
                and event.left_tactile_available
                and event.left_tactile_contact
                and event.lid_safe
                and self._left_participated
                and self._right_participated
            )
            return self._check(
                ready,
                event,
                BoxShoePhase.NAVIGATING_TO_A,
                (BoxShoeCommand.NAVIGATE_TO_A,),
                "both arms reached transport state",
                "dual-arm transport state was not verified",
            )
        if previous == BoxShoePhase.NAVIGATING_TO_A:
            return self._check(
                event.location == "A"
                and event.success
                and event.base_stationary
                and self._shoe_carrying
                and event.shoe_held_by_left
                and event.left_tactile_available
                and event.left_tactile_contact,
                event,
                BoxShoePhase.PLACING_AT_A_LEFT,
                (BoxShoeCommand.PLACE_SHOE_WITH_LEFT,),
                "A reached with left shoe hold",
                "navigation or left shoe hold failed",
            )
        if previous == BoxShoePhase.PLACING_AT_A_LEFT:
            return self._check(
                event.success and event.left_arm_moved,
                event,
                BoxShoePhase.VERIFYING_PLACE,
                (BoxShoeCommand.VERIFY_PLACE,),
                "left arm place motion completed",
                "left-arm place failed",
            )
        if previous == BoxShoePhase.VERIFYING_PLACE:
            complete = (
                event.success
                and event.object_released
                and event.at_start_zone
                and self._left_participated
                and self._right_participated
            )
            if complete:
                self._shoe_carrying = False
            return self._check(
                complete,
                event,
                BoxShoePhase.COMPLETED,
                (BoxShoeCommand.MISSION_COMPLETE,),
                "box task completed with both arms participating",
                "release or dual-arm participation verification failed",
            )
        if previous == BoxShoePhase.SAFE_STOP_REQUESTED:
            if event.success and event.base_stationary and event.actuators_stopped:
                return self._advance(
                    event,
                    BoxShoePhase.SAFE_STOPPED,
                    (),
                    "safe stop confirmed by base and actuators",
                )
            return self._advance(
                event,
                BoxShoePhase.SAFE_STOP_REQUESTED,
                (BoxShoeCommand.SAFE_STOP,),
                event.failure_code or "safe stop is not yet confirmed",
            )
        raise RuntimeError(f"unhandled phase: {previous.value}")

    def _right_lid_hold_ok(self, event: BoxShoeEvent) -> bool:
        return (
            event.lid_held_by_right
            and event.right_tactile_available
            and event.right_tactile_contact
            and event.lid_open_angle_rad
            >= self.config.minimum_lid_open_angle_rad
        )

    def _check(
        self,
        condition: bool,
        event: BoxShoeEvent,
        next_phase: BoxShoePhase,
        commands: tuple[BoxShoeCommand, ...],
        success_reason: str,
        failure_reason: str,
    ) -> BoxShoeTransition:
        if not condition:
            return self._stop(event, failure_reason)
        return self._advance(event, next_phase, commands, success_reason)

    def _stop(self, event: BoxShoeEvent, reason: str) -> BoxShoeTransition:
        return self._advance(
            event,
            BoxShoePhase.SAFE_STOP_REQUESTED,
            (BoxShoeCommand.SAFE_STOP,),
            reason,
        )

    def _advance(
        self,
        event: BoxShoeEvent,
        phase: BoxShoePhase,
        commands: tuple[BoxShoeCommand, ...],
        reason: str,
    ) -> BoxShoeTransition:
        previous = self.phase
        self.phase = phase
        return self._record(previous, event, commands, reason)

    def _record(
        self,
        previous: BoxShoePhase,
        event: BoxShoeEvent,
        commands: tuple[BoxShoeCommand, ...],
        reason: str,
    ) -> BoxShoeTransition:
        transition = BoxShoeTransition(
            sequence=self._sequence,
            previous_phase=previous,
            phase=self.phase,
            event=event.kind,
            commands=commands,
            reason=reason,
            left_arm_participated=self._left_participated,
            right_arm_participated=self._right_participated,
        )
        self._sequence += 1
        self._history.append(transition)
        return transition


__all__ = [
    "BoxShoeCommand",
    "BoxShoeEvent",
    "BoxShoeEventType",
    "BoxShoeMissionConfig",
    "BoxShoeMissionController",
    "BoxShoePhase",
    "BoxShoeTransition",
    "SCHEMA_VERSION",
]
