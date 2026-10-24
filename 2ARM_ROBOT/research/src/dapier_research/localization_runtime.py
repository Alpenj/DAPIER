"""Portable localization estimate validation for replay and policy planning.

This module mirrors the language-neutral ``localization_runtime_v1`` contract.
Source timestamps are trace metadata only. Freshness is always calculated from
receiver-local monotonic time by the consumer.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


LOCALIZATION_SCHEMA_VERSION = "dapier.localization-estimate.v1"
MAX_ESTIMATE_TTL_NS = 250_000_000
MAX_IDENTIFIER_LENGTH = 128
TRACKING_STATES = (
    "initializing",
    "tracking",
    "recently_lost",
    "lost",
    "relocalized",
)
MOTION_DECISIONS = ("proceed", "hold", "reject")

_REQUIRED_KEYS = frozenset(
    {
        "schema_version",
        "sequence",
        "source_timestamp_ns",
        "ttl_ns",
        "parent_frame",
        "child_frame",
        "map_id",
        "session_id",
        "reset_generation",
        "tracking_state",
        "position_m",
        "orientation_xyzw",
        "covariance",
        "tracked_features",
        "mean_reprojection_error_px",
        "confidence",
        "simulator_truth_used",
        "control_authorized",
    }
)


class LocalizationContractError(ValueError):
    """Raised when a localization estimate violates the public contract."""


@dataclass(frozen=True)
class LocalizationEstimate:
    schema_version: str
    sequence: int
    source_timestamp_ns: int
    ttl_ns: int
    parent_frame: str
    child_frame: str
    map_id: str
    session_id: str
    reset_generation: int
    tracking_state: str
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    covariance: tuple[float, ...]
    tracked_features: int
    mean_reprojection_error_px: float
    confidence: float
    simulator_truth_used: bool
    control_authorized: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "sequence": self.sequence,
            "source_timestamp_ns": self.source_timestamp_ns,
            "ttl_ns": self.ttl_ns,
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "map_id": self.map_id,
            "session_id": self.session_id,
            "reset_generation": self.reset_generation,
            "tracking_state": self.tracking_state,
            "position_m": list(self.position_m),
            "orientation_xyzw": list(self.orientation_xyzw),
            "covariance": list(self.covariance),
            "tracked_features": self.tracked_features,
            "mean_reprojection_error_px": self.mean_reprojection_error_px,
            "confidence": self.confidence,
            "simulator_truth_used": self.simulator_truth_used,
            "control_authorized": self.control_authorized,
        }


@dataclass(frozen=True)
class LocalizationValidation:
    accepted: bool
    reason: str
    accepted_until_monotonic_ns: int | None
    estimate: LocalizationEstimate | None = None


@dataclass(frozen=True)
class MotionGateConfig:
    minimum_confidence: float = 0.5
    minimum_tracked_features: int = 20
    maximum_mean_reprojection_error_px: float = 3.0
    maximum_position_variance_m2: float = 0.01
    maximum_rotation_variance_rad2: float = 0.04

    def validate(self) -> None:
        numeric = (
            self.minimum_confidence,
            self.maximum_mean_reprojection_error_px,
            self.maximum_position_variance_m2,
            self.maximum_rotation_variance_rad2,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise LocalizationContractError("motion gate values must be finite")
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise LocalizationContractError("minimum_confidence must be in [0, 1]")
        if isinstance(self.minimum_tracked_features, bool) or self.minimum_tracked_features <= 0:
            raise LocalizationContractError("minimum_tracked_features must be positive")
        if self.maximum_mean_reprojection_error_px <= 0.0:
            raise LocalizationContractError(
                "maximum_mean_reprojection_error_px must be positive"
            )
        if self.maximum_position_variance_m2 <= 0.0:
            raise LocalizationContractError(
                "maximum_position_variance_m2 must be positive"
            )
        if self.maximum_rotation_variance_rad2 <= 0.0:
            raise LocalizationContractError(
                "maximum_rotation_variance_rad2 must be positive"
            )


@dataclass(frozen=True)
class MotionAssessment:
    decision: str
    reason: str


def _text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise LocalizationContractError(f"{name} must be non-empty text")
    if len(value) > MAX_IDENTIFIER_LENGTH:
        raise LocalizationContractError(f"{name} exceeds {MAX_IDENTIFIER_LENGTH} characters")
    return value


def _integer(value: object, name: str, *, positive: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise LocalizationContractError(f"{name} must be an integer")
    if value < 0 or (positive and value <= 0):
        qualifier = "positive" if positive else "non-negative"
        raise LocalizationContractError(f"{name} must be {qualifier}")
    return value


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LocalizationContractError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise LocalizationContractError(f"{name} must be finite")
    return result


def _vector(value: object, name: str, length: int) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise LocalizationContractError(f"{name} must be an array")
    if len(value) != length:
        raise LocalizationContractError(f"{name} must contain {length} values")
    return tuple(_number(item, f"{name}[{index}]") for index, item in enumerate(value))


def estimate_from_mapping(value: Mapping[str, object]) -> LocalizationEstimate:
    if not isinstance(value, Mapping):
        raise LocalizationContractError("localization estimate must be an object")
    keys = frozenset(value)
    if keys != _REQUIRED_KEYS:
        raise LocalizationContractError(
            "localization estimate keys differ; "
            f"missing={sorted(_REQUIRED_KEYS - keys)}, extra={sorted(keys - _REQUIRED_KEYS)}"
        )
    estimate = LocalizationEstimate(
        schema_version=_text(value["schema_version"], "schema_version"),
        sequence=_integer(value["sequence"], "sequence"),
        source_timestamp_ns=_integer(
            value["source_timestamp_ns"], "source_timestamp_ns"
        ),
        ttl_ns=_integer(value["ttl_ns"], "ttl_ns", positive=True),
        parent_frame=_text(value["parent_frame"], "parent_frame"),
        child_frame=_text(value["child_frame"], "child_frame"),
        map_id=_text(value["map_id"], "map_id"),
        session_id=_text(value["session_id"], "session_id"),
        reset_generation=_integer(value["reset_generation"], "reset_generation"),
        tracking_state=_text(value["tracking_state"], "tracking_state"),
        position_m=_vector(value["position_m"], "position_m", 3),
        orientation_xyzw=_vector(
            value["orientation_xyzw"], "orientation_xyzw", 4
        ),
        covariance=_vector(value["covariance"], "covariance", 36),
        tracked_features=_integer(value["tracked_features"], "tracked_features"),
        mean_reprojection_error_px=_number(
            value["mean_reprojection_error_px"], "mean_reprojection_error_px"
        ),
        confidence=_number(value["confidence"], "confidence"),
        simulator_truth_used=value["simulator_truth_used"],
        control_authorized=value["control_authorized"],
    )
    if not isinstance(estimate.simulator_truth_used, bool):
        raise LocalizationContractError("simulator_truth_used must be boolean")
    if not isinstance(estimate.control_authorized, bool):
        raise LocalizationContractError("control_authorized must be boolean")
    return estimate


def validate_localization_estimate(
    value: Mapping[str, object] | LocalizationEstimate,
    *,
    receiver_monotonic_ns: int,
    last_accepted_sequence: int | None = None,
    expected_reset_generation: int | None = None,
) -> LocalizationValidation:
    try:
        estimate = (
            value
            if isinstance(value, LocalizationEstimate)
            else estimate_from_mapping(value)
        )
        receiver = _integer(receiver_monotonic_ns, "receiver_monotonic_ns")
        if estimate.schema_version != LOCALIZATION_SCHEMA_VERSION:
            raise LocalizationContractError("unsupported localization schema_version")
        if estimate.ttl_ns > MAX_ESTIMATE_TTL_NS:
            raise LocalizationContractError("ttl exceeds the localization contract")
        if last_accepted_sequence is not None:
            previous = _integer(last_accepted_sequence, "last_accepted_sequence")
            if estimate.sequence <= previous:
                raise LocalizationContractError(
                    "sequence must be greater than the last accepted sequence"
                )
        if expected_reset_generation is not None:
            expected = _integer(
                expected_reset_generation, "expected_reset_generation"
            )
            if estimate.reset_generation != expected:
                raise LocalizationContractError(
                    "reset_generation differs from the active plan generation"
                )
        if estimate.parent_frame == estimate.child_frame:
            raise LocalizationContractError("parent_frame and child_frame must differ")
        if estimate.tracking_state not in TRACKING_STATES:
            raise LocalizationContractError("tracking_state is not recognized")
        quaternion_norm = math.sqrt(
            sum(value * value for value in estimate.orientation_xyzw)
        )
        if abs(quaternion_norm - 1.0) > 1e-3:
            raise LocalizationContractError("orientation quaternion must be normalized")
        for index in (0, 7, 14, 21, 28, 35):
            if estimate.covariance[index] < 0.0:
                raise LocalizationContractError(
                    "covariance diagonal entries must be non-negative"
                )
        for row in range(6):
            for column in range(row + 1, 6):
                forward = estimate.covariance[row * 6 + column]
                reverse = estimate.covariance[column * 6 + row]
                if abs(forward - reverse) > 1e-8:
                    raise LocalizationContractError("covariance must be symmetric")
        if estimate.mean_reprojection_error_px < 0.0:
            raise LocalizationContractError(
                "mean_reprojection_error_px must be non-negative"
            )
        if not 0.0 <= estimate.confidence <= 1.0:
            raise LocalizationContractError("confidence must be in [0, 1]")
        if estimate.simulator_truth_used:
            raise LocalizationContractError(
                "runtime localization must not depend on simulator truth"
            )
        if estimate.control_authorized:
            raise LocalizationContractError(
                "localization cannot authorize hardware control"
            )
        if estimate.ttl_ns > (2**63 - 1) - receiver:
            raise LocalizationContractError("receiver-local expiry would overflow int64")
    except LocalizationContractError as error:
        return LocalizationValidation(False, str(error), None, None)
    return LocalizationValidation(
        True,
        "accepted by the localization boundary; quality gates still apply",
        receiver + estimate.ttl_ns,
        estimate,
    )


def assess_localization_for_motion(
    estimate: LocalizationEstimate,
    validation: LocalizationValidation,
    *,
    config: MotionGateConfig | None = None,
) -> MotionAssessment:
    if not validation.accepted:
        return MotionAssessment("reject", "invalid localization estimate")
    resolved = config or MotionGateConfig()
    try:
        resolved.validate()
    except LocalizationContractError as error:
        return MotionAssessment("reject", str(error))
    if estimate.tracking_state == "relocalized":
        return MotionAssessment(
            "hold", "relocalized pose requires target invalidation and replanning"
        )
    if estimate.tracking_state == "initializing":
        return MotionAssessment("hold", "localization is initializing")
    if estimate.tracking_state == "recently_lost":
        return MotionAssessment("hold", "localization is recently lost")
    if estimate.tracking_state == "lost":
        return MotionAssessment("hold", "localization is lost")
    if estimate.tracking_state != "tracking":
        return MotionAssessment("reject", "tracking state is invalid")
    if estimate.confidence < resolved.minimum_confidence:
        return MotionAssessment("hold", "localization confidence is too low")
    if estimate.tracked_features < resolved.minimum_tracked_features:
        return MotionAssessment("hold", "tracked feature count is too low")
    if (
        estimate.mean_reprojection_error_px
        > resolved.maximum_mean_reprojection_error_px
    ):
        return MotionAssessment("hold", "reprojection error is too high")
    if any(
        estimate.covariance[index] > resolved.maximum_position_variance_m2
        for index in (0, 7, 14)
    ):
        return MotionAssessment("hold", "position covariance is too high")
    if any(
        estimate.covariance[index] > resolved.maximum_rotation_variance_rad2
        for index in (21, 28, 35)
    ):
        return MotionAssessment("hold", "rotation covariance is too high")
    return MotionAssessment("proceed", "localization is suitable for motion")


def load_contract(path: str | Path) -> Mapping[str, Any]:
    contract_path = Path(path)
    try:
        value = json.loads(contract_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise LocalizationContractError(f"contract not found: {contract_path}") from error
    except json.JSONDecodeError as error:
        raise LocalizationContractError(f"invalid contract JSON: {error}") from error
    if not isinstance(value, Mapping):
        raise LocalizationContractError("contract must be a JSON object")
    return value


def verify_python_contract_matches_json(path: str | Path) -> None:
    contract = load_contract(path)
    expected = {
        "schema_version": LOCALIZATION_SCHEMA_VERSION,
        "max_estimate_ttl_ns": MAX_ESTIMATE_TTL_NS,
        "max_identifier_length": MAX_IDENTIFIER_LENGTH,
        "allowed_tracking_states": list(TRACKING_STATES),
        "localization_may_authorize_hardware": False,
        "simulator_truth_allowed_for_runtime": False,
    }
    for key, value in expected.items():
        if contract.get(key) != value:
            raise LocalizationContractError(
                f"Python localization constant differs from contract: {key}"
            )


__all__ = [
    "LOCALIZATION_SCHEMA_VERSION",
    "MAX_ESTIMATE_TTL_NS",
    "MAX_IDENTIFIER_LENGTH",
    "MOTION_DECISIONS",
    "TRACKING_STATES",
    "LocalizationContractError",
    "LocalizationEstimate",
    "LocalizationValidation",
    "MotionAssessment",
    "MotionGateConfig",
    "assess_localization_for_motion",
    "estimate_from_mapping",
    "load_contract",
    "validate_localization_estimate",
    "verify_python_contract_matches_json",
]
