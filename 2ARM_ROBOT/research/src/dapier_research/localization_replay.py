"""Deterministic localization replay and fail-closed recovery monitoring.

The replay path consumes recorded-like metadata only. It never opens a camera,
ROS graph, serial port, SSH connection, or actuator endpoint.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import json
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from .localization_runtime import (
    LOCALIZATION_SCHEMA_VERSION,
    LocalizationEstimate,
    MotionGateConfig,
    assess_localization_for_motion,
    estimate_from_mapping,
    validate_localization_estimate,
)


REPLAY_REPORT_SCHEMA_VERSION = "dapier.localization-replay-report.v1"


class ReplayFault(str, Enum):
    NONE = "none"
    LOW_TEXTURE = "low_texture"
    MOTION_BLUR = "motion_blur"
    OCCLUSION = "occlusion"
    FRAME_DROP = "frame_drop"
    DUPLICATE = "duplicate"
    OUT_OF_ORDER = "out_of_order"
    TIMESTAMP_SKEW = "timestamp_skew"
    WRONG_TF = "wrong_tf"
    RECENTLY_LOST = "recently_lost"
    LOST = "lost"
    RELOCALIZED = "relocalized"
    MAP_RESTART = "map_restart"


@dataclass(frozen=True)
class ReplaySample:
    capture_index: int
    receiver_monotonic_ns: int
    rgb_timestamp_ns: int
    depth_timestamp_ns: int
    estimate: Mapping[str, object]

    def as_dict(self) -> dict[str, Any]:
        return {
            "capture_index": self.capture_index,
            "receiver_monotonic_ns": self.receiver_monotonic_ns,
            "rgb_timestamp_ns": self.rgb_timestamp_ns,
            "depth_timestamp_ns": self.depth_timestamp_ns,
            "estimate": dict(self.estimate),
        }


@dataclass(frozen=True)
class ReplayConfig:
    maximum_rgb_depth_skew_ns: int = 20_000_000
    maximum_interarrival_ns: int = 75_000_000
    motion_gate: MotionGateConfig = MotionGateConfig()

    def validate(self) -> None:
        for value, name in (
            (self.maximum_rgb_depth_skew_ns, "maximum_rgb_depth_skew_ns"),
            (self.maximum_interarrival_ns, "maximum_interarrival_ns"),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        self.motion_gate.validate()


@dataclass(frozen=True)
class RecoveryDecision:
    decision: str
    reason: str
    sequence: int | None
    discard_pending_actions: bool
    replan_required: bool
    identity_changed: bool
    hardware_execution: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "decision": self.decision,
            "reason": self.reason,
            "sequence": self.sequence,
            "discard_pending_actions": self.discard_pending_actions,
            "replan_required": self.replan_required,
            "identity_changed": self.identity_changed,
            "hardware_execution": self.hardware_execution,
        }


@dataclass(frozen=True)
class ReplayReport:
    schema_version: str
    fault: str
    decisions: tuple[RecoveryDecision, ...]
    proceed_count: int
    hold_count: int
    reject_count: int
    replan_count: int
    discarded_action_count: int
    simulator_truth_used: bool
    hardware_execution: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "fault": self.fault,
            "decisions": [decision.as_dict() for decision in self.decisions],
            "proceed_count": self.proceed_count,
            "hold_count": self.hold_count,
            "reject_count": self.reject_count,
            "replan_count": self.replan_count,
            "discarded_action_count": self.discarded_action_count,
            "simulator_truth_used": self.simulator_truth_used,
            "hardware_execution": self.hardware_execution,
        }


class LocalizationRecoveryMonitor:
    """Invalidate motion state whenever localization identity or quality changes."""

    def __init__(self, config: ReplayConfig | None = None) -> None:
        self.config = config or ReplayConfig()
        self.config.validate()
        self.last_sequence: int | None = None
        self.last_receiver_monotonic_ns: int | None = None
        self.identity: tuple[str, str, str, str, int] | None = None
        self.awaiting_replan = True
        self.replan_sequence: int | None = None

    @staticmethod
    def _identity(estimate: LocalizationEstimate) -> tuple[str, str, str, str, int]:
        return (
            estimate.parent_frame,
            estimate.child_frame,
            estimate.map_id,
            estimate.session_id,
            estimate.reset_generation,
        )

    def _hold(
        self,
        reason: str,
        sequence: int,
        *,
        identity_changed: bool = False,
    ) -> RecoveryDecision:
        self.awaiting_replan = True
        self.replan_sequence = sequence
        return RecoveryDecision(
            "hold",
            reason,
            sequence,
            True,
            True,
            identity_changed,
        )

    def observe(self, sample: ReplaySample) -> RecoveryDecision:
        if not isinstance(sample.receiver_monotonic_ns, int) or sample.receiver_monotonic_ns < 0:
            return RecoveryDecision(
                "reject",
                "receiver monotonic timestamp is invalid",
                None,
                True,
                False,
                False,
            )
        skew = abs(sample.rgb_timestamp_ns - sample.depth_timestamp_ns)
        if skew > self.config.maximum_rgb_depth_skew_ns:
            return RecoveryDecision(
                "reject",
                f"RGB/depth skew {skew} ns exceeds the replay contract",
                None,
                True,
                False,
                False,
            )
        validation = validate_localization_estimate(
            sample.estimate,
            receiver_monotonic_ns=sample.receiver_monotonic_ns,
            last_accepted_sequence=self.last_sequence,
        )
        if not validation.accepted or validation.estimate is None:
            return RecoveryDecision(
                "reject",
                validation.reason,
                None,
                True,
                False,
                False,
            )
        estimate = validation.estimate
        current_identity = self._identity(estimate)

        if self.last_receiver_monotonic_ns is not None:
            gap = sample.receiver_monotonic_ns - self.last_receiver_monotonic_ns
            if gap <= 0:
                return RecoveryDecision(
                    "reject",
                    "receiver replay timestamps must increase",
                    estimate.sequence,
                    True,
                    False,
                    False,
                )
        else:
            gap = 0

        previous_identity = self.identity
        if previous_identity is not None:
            previous_generation = previous_identity[-1]
            if estimate.reset_generation < previous_generation:
                return RecoveryDecision(
                    "reject",
                    "reset generation moved backwards",
                    estimate.sequence,
                    True,
                    False,
                    False,
                )
            identity_fields_changed = current_identity[:-1] != previous_identity[:-1]
            if identity_fields_changed and estimate.reset_generation <= previous_generation:
                return RecoveryDecision(
                    "reject",
                    "frame/map/session identity changed without a reset generation increment",
                    estimate.sequence,
                    True,
                    False,
                    True,
                )

        self.last_sequence = estimate.sequence
        self.last_receiver_monotonic_ns = sample.receiver_monotonic_ns
        self.identity = current_identity

        if previous_identity is None:
            return self._hold(
                "first localization identity requires a fresh plan",
                estimate.sequence,
                identity_changed=True,
            )
        if current_identity != previous_identity:
            return self._hold(
                "localization identity changed; stale targets were invalidated",
                estimate.sequence,
                identity_changed=True,
            )
        if gap > self.config.maximum_interarrival_ns:
            return self._hold(
                f"localization interarrival gap {gap} ns exceeded the watchdog",
                estimate.sequence,
            )

        assessment = assess_localization_for_motion(
            estimate,
            validation,
            config=self.config.motion_gate,
        )
        if assessment.decision == "reject":
            return RecoveryDecision(
                "reject",
                assessment.reason,
                estimate.sequence,
                True,
                False,
                False,
            )
        if assessment.decision == "hold":
            return self._hold(assessment.reason, estimate.sequence)
        if self.awaiting_replan:
            return RecoveryDecision(
                "hold",
                "localization recovered but a plan acknowledgement is still required",
                estimate.sequence,
                True,
                True,
                False,
            )
        return RecoveryDecision(
            "proceed",
            assessment.reason,
            estimate.sequence,
            False,
            False,
            False,
        )

    def acknowledge_replan(
        self,
        *,
        sequence: int,
        parent_frame: str,
        child_frame: str,
        map_id: str,
        session_id: str,
        reset_generation: int,
    ) -> None:
        expected_identity = (
            parent_frame,
            child_frame,
            map_id,
            session_id,
            reset_generation,
        )
        if self.identity is None or expected_identity != self.identity:
            raise ValueError("replan identity does not match the active localization identity")
        if self.replan_sequence is None or sequence != self.replan_sequence:
            raise ValueError("replan sequence does not match the invalidation boundary")
        self.awaiting_replan = False


def _diagonal_covariance(
    position_variance: float = 0.0025,
    rotation_variance: float = 0.01,
) -> list[float]:
    result = [0.0] * 36
    for index in (0, 7, 14):
        result[index] = position_variance
    for index in (21, 28, 35):
        result[index] = rotation_variance
    return result


def build_nominal_replay(
    *,
    sample_count: int = 6,
    period_ns: int = 50_000_000,
) -> tuple[ReplaySample, ...]:
    if sample_count < 3:
        raise ValueError("sample_count must be at least 3")
    if period_ns <= 0:
        raise ValueError("period_ns must be positive")
    samples: list[ReplaySample] = []
    for index in range(sample_count):
        timestamp = index * period_ns
        estimate = {
            "schema_version": LOCALIZATION_SCHEMA_VERSION,
            "sequence": index + 1,
            "source_timestamp_ns": timestamp,
            "ttl_ns": 200_000_000,
            "parent_frame": "map",
            "child_frame": "base_link",
            "map_id": "dapier-lab-map-v1",
            "session_id": "replay-session-001",
            "reset_generation": 0,
            "tracking_state": "tracking",
            "position_m": [0.05 * index, 0.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "covariance": _diagonal_covariance(),
            "tracked_features": 80,
            "mean_reprojection_error_px": 0.8,
            "confidence": 0.9,
            "simulator_truth_used": False,
            "control_authorized": False,
        }
        samples.append(
            ReplaySample(
                capture_index=index,
                receiver_monotonic_ns=1_000_000_000 + timestamp,
                rgb_timestamp_ns=timestamp,
                depth_timestamp_ns=timestamp,
                estimate=estimate,
            )
        )
    return tuple(samples)


def _replace_estimate(sample: ReplaySample, **updates: object) -> ReplaySample:
    estimate = dict(sample.estimate)
    estimate.update(updates)
    return replace(sample, estimate=estimate)


def inject_fault(
    samples: Sequence[ReplaySample],
    fault: ReplayFault | str,
    *,
    index: int = 2,
) -> tuple[ReplaySample, ...]:
    resolved = ReplayFault(fault)
    result = list(samples)
    if resolved is ReplayFault.NONE:
        return tuple(result)
    if not 0 <= index < len(result):
        raise ValueError("fault index is outside the replay")

    if resolved is ReplayFault.LOW_TEXTURE:
        result[index] = _replace_estimate(
            result[index], tracked_features=4, confidence=0.2
        )
    elif resolved is ReplayFault.MOTION_BLUR:
        result[index] = _replace_estimate(
            result[index], mean_reprojection_error_px=9.0, confidence=0.35
        )
    elif resolved is ReplayFault.OCCLUSION:
        result[index] = _replace_estimate(
            result[index],
            tracking_state="recently_lost",
            tracked_features=0,
            confidence=0.0,
        )
    elif resolved is ReplayFault.FRAME_DROP:
        del result[index]
    elif resolved is ReplayFault.DUPLICATE:
        duplicate = replace(result[index], capture_index=10_000 + index)
        result.insert(index + 1, duplicate)
    elif resolved is ReplayFault.OUT_OF_ORDER:
        if index + 1 >= len(result):
            raise ValueError("out_of_order requires a following sample")
        result[index], result[index + 1] = result[index + 1], result[index]
    elif resolved is ReplayFault.TIMESTAMP_SKEW:
        result[index] = replace(
            result[index],
            depth_timestamp_ns=result[index].rgb_timestamp_ns + 100_000_000,
        )
    elif resolved is ReplayFault.WRONG_TF:
        result[index] = _replace_estimate(result[index], child_frame="map")
    elif resolved is ReplayFault.RECENTLY_LOST:
        result[index] = _replace_estimate(
            result[index], tracking_state="recently_lost"
        )
    elif resolved is ReplayFault.LOST:
        result[index] = _replace_estimate(result[index], tracking_state="lost")
    elif resolved is ReplayFault.RELOCALIZED:
        result[index] = _replace_estimate(
            result[index],
            tracking_state="relocalized",
            position_m=[1.5, -0.4, 0.0],
        )
    elif resolved is ReplayFault.MAP_RESTART:
        for current in range(index, len(result)):
            result[current] = _replace_estimate(
                result[current],
                map_id="dapier-lab-map-v2",
                session_id="replay-session-002",
                reset_generation=1,
                tracking_state=("relocalized" if current == index else "tracking"),
            )
    return tuple(result)


def run_replay(
    samples: Iterable[ReplaySample],
    *,
    fault: ReplayFault | str = ReplayFault.NONE,
    config: ReplayConfig | None = None,
    auto_acknowledge_replan: bool = True,
) -> ReplayReport:
    monitor = LocalizationRecoveryMonitor(config)
    decisions: list[RecoveryDecision] = []
    simulator_truth_used = False
    for sample in samples:
        try:
            parsed = estimate_from_mapping(sample.estimate)
        except ValueError:
            parsed = None
        if parsed is not None:
            simulator_truth_used = simulator_truth_used or parsed.simulator_truth_used
        decision = monitor.observe(sample)
        decisions.append(decision)
        if (
            auto_acknowledge_replan
            and decision.decision == "hold"
            and decision.replan_required
            and decision.sequence is not None
            and monitor.identity is not None
        ):
            parent, child, map_id, session_id, generation = monitor.identity
            monitor.acknowledge_replan(
                sequence=decision.sequence,
                parent_frame=parent,
                child_frame=child,
                map_id=map_id,
                session_id=session_id,
                reset_generation=generation,
            )
    return ReplayReport(
        schema_version=REPLAY_REPORT_SCHEMA_VERSION,
        fault=ReplayFault(fault).value,
        decisions=tuple(decisions),
        proceed_count=sum(item.decision == "proceed" for item in decisions),
        hold_count=sum(item.decision == "hold" for item in decisions),
        reject_count=sum(item.decision == "reject" for item in decisions),
        replan_count=sum(item.replan_required for item in decisions),
        discarded_action_count=sum(item.discard_pending_actions for item in decisions),
        simulator_truth_used=simulator_truth_used,
        hardware_execution=False,
    )


def run_failure_matrix() -> dict[str, Any]:
    base = build_nominal_replay()
    reports = {
        fault.value: run_replay(inject_fault(base, fault), fault=fault).as_dict()
        for fault in ReplayFault
    }
    return {
        "schema_version": "dapier.localization-failure-matrix.v1",
        "reports": reports,
        "hardware_execution": False,
    }


def save_failure_matrix(path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(run_failure_matrix(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return target


__all__ = [
    "REPLAY_REPORT_SCHEMA_VERSION",
    "LocalizationRecoveryMonitor",
    "RecoveryDecision",
    "ReplayConfig",
    "ReplayFault",
    "ReplayReport",
    "ReplaySample",
    "build_nominal_replay",
    "inject_fault",
    "run_failure_matrix",
    "run_replay",
    "save_failure_matrix",
]
