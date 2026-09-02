"""Deployment-safe policy observations for DAPIER.

The contract deliberately excludes privileged simulator object state. Runtime
objects may enter a policy only through sensor-derived estimates and measured
robot state. The module is hardware-agnostic and never authorizes execution.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping, Sequence


CONTRACT_FILE_NAME = "vision_policy_observation_v1.json"
_ALLOWED_TOP_LEVEL_KEYS = frozenset(
    {
        "schema_version",
        "sequence",
        "timestamp_ns",
        "clock_domain",
        "source",
        "target_frame",
        "calibration_id",
        "calibration_verified",
        "target",
        "robot",
        "ground_truth_used",
        "privileged_state",
        "control_authorized",
        "hardware_execution",
    }
)


class PolicyObservationError(ValueError):
    """Fail-closed observation validation error with a stable code."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class PolicyObservationContract:
    schema_version: str
    max_observation_age_ns: int
    max_joint_count: int
    minimum_target_confidence: float
    required_false_fields: tuple[str, ...]
    forbidden_runtime_fields: tuple[str, ...]


@dataclass(frozen=True)
class PolicyObservation:
    schema_version: str
    sequence: int
    timestamp_ns: int
    clock_domain: str
    source: str
    target_frame: str
    calibration_id: str
    calibration_verified: bool
    target: dict[str, Any]
    robot: dict[str, Any]
    ground_truth_used: bool = False
    privileged_state: bool = False
    control_authorized: bool = False
    hardware_execution: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "sequence": self.sequence,
            "timestamp_ns": self.timestamp_ns,
            "clock_domain": self.clock_domain,
            "source": self.source,
            "target_frame": self.target_frame,
            "calibration_id": self.calibration_id,
            "calibration_verified": self.calibration_verified,
            "target": dict(self.target),
            "robot": dict(self.robot),
            "ground_truth_used": self.ground_truth_used,
            "privileged_state": self.privileged_state,
            "control_authorized": self.control_authorized,
            "hardware_execution": self.hardware_execution,
        }


def default_contract_path() -> Path:
    override = os.environ.get("DAPIER_VISION_POLICY_CONTRACT")
    if override:
        return Path(override).expanduser()
    return Path(__file__).resolve().parents[4] / "contracts" / CONTRACT_FILE_NAME


def _require_text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise PolicyObservationError("invalid_text", f"{name} must be non-empty text")
    return value


def _require_uint64(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 2**64:
        raise PolicyObservationError("invalid_integer", f"{name} must be uint64")
    return value


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PolicyObservationError("invalid_number", f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise PolicyObservationError("invalid_number", f"{name} must be finite")
    return result


def _finite_vector(value: object, name: str, size: int | None = None) -> list[float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise PolicyObservationError("invalid_shape", f"{name} must be an array")
    result = [_finite(item, f"{name} entry") for item in value]
    if size is not None and len(result) != size:
        raise PolicyObservationError("invalid_shape", f"{name} must have {size} entries")
    return result


def _text_vector(value: object, name: str) -> list[str]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise PolicyObservationError("invalid_shape", f"{name} must be an array")
    result = [_require_text(item, f"{name} entry") for item in value]
    if len(set(result)) != len(result):
        raise PolicyObservationError("duplicate_joint", f"{name} entries must be unique")
    return result


def _walk_keys(value: object, prefix: str = "") -> list[str]:
    found: list[str] = []
    if isinstance(value, Mapping):
        for raw_key, nested in value.items():
            key = str(raw_key)
            qualified = f"{prefix}.{key}" if prefix else key
            found.append(qualified)
            found.extend(_walk_keys(nested, qualified))
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for index, nested in enumerate(value):
            found.extend(_walk_keys(nested, f"{prefix}[{index}]"))
    return found


def load_policy_observation_contract(
    path: str | Path | None = None,
) -> PolicyObservationContract:
    contract_path = Path(path) if path is not None else default_contract_path()
    try:
        raw = json.loads(contract_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise PolicyObservationError(
            "contract_missing", f"policy observation contract not found: {contract_path}"
        ) from error
    except json.JSONDecodeError as error:
        raise PolicyObservationError("contract_invalid", str(error)) from error
    if not isinstance(raw, Mapping):
        raise PolicyObservationError("contract_invalid", "contract must be an object")
    false_fields = tuple(raw.get("required_false_fields", ()))
    forbidden = tuple(raw.get("forbidden_runtime_fields", ()))
    if not false_fields or not all(isinstance(item, str) and item for item in false_fields):
        raise PolicyObservationError("contract_invalid", "required_false_fields is invalid")
    if not forbidden or not all(isinstance(item, str) and item for item in forbidden):
        raise PolicyObservationError("contract_invalid", "forbidden_runtime_fields is invalid")
    minimum_confidence = _finite(
        raw.get("minimum_target_confidence"), "minimum_target_confidence"
    )
    if not 0.0 <= minimum_confidence <= 1.0:
        raise PolicyObservationError("contract_invalid", "confidence must be in [0, 1]")
    max_age = raw.get("max_observation_age_ns")
    max_joints = raw.get("max_joint_count")
    if isinstance(max_age, bool) or not isinstance(max_age, int) or max_age <= 0:
        raise PolicyObservationError("contract_invalid", "max age must be positive")
    if isinstance(max_joints, bool) or not isinstance(max_joints, int) or max_joints <= 0:
        raise PolicyObservationError("contract_invalid", "max joints must be positive")
    return PolicyObservationContract(
        schema_version=_require_text(raw.get("schema_version"), "schema_version"),
        max_observation_age_ns=max_age,
        max_joint_count=max_joints,
        minimum_target_confidence=minimum_confidence,
        required_false_fields=false_fields,
        forbidden_runtime_fields=forbidden,
    )


def validate_policy_observation(
    value: PolicyObservation | Mapping[str, Any],
    *,
    now_ns: int | None = None,
    require_verified_calibration: bool = False,
    contract: PolicyObservationContract | None = None,
) -> PolicyObservation:
    resolved = contract or load_policy_observation_contract()
    raw = value.as_dict() if isinstance(value, PolicyObservation) else value
    if not isinstance(raw, Mapping):
        raise PolicyObservationError("invalid_observation", "observation must be an object")
    unexpected = set(raw) - _ALLOWED_TOP_LEVEL_KEYS
    if unexpected:
        raise PolicyObservationError(
            "unexpected_field", f"unexpected top-level fields: {sorted(unexpected)}"
        )
    missing = _ALLOWED_TOP_LEVEL_KEYS - set(raw)
    if missing:
        raise PolicyObservationError("missing_field", f"missing fields: {sorted(missing)}")

    all_keys = _walk_keys(raw)
    for forbidden in resolved.forbidden_runtime_fields:
        if any(path.rsplit(".", 1)[-1] == forbidden for path in all_keys):
            raise PolicyObservationError(
                "privileged_field", f"runtime observation contains forbidden field {forbidden}"
            )
    for field in resolved.required_false_fields:
        if raw.get(field) is not False:
            raise PolicyObservationError("privileged_state", f"{field} must remain false")

    if raw.get("schema_version") != resolved.schema_version:
        raise PolicyObservationError("schema_mismatch", "schema version does not match")
    sequence = _require_uint64(raw.get("sequence"), "sequence")
    timestamp_ns = _require_uint64(raw.get("timestamp_ns"), "timestamp_ns")
    calibration_verified = raw.get("calibration_verified")
    if not isinstance(calibration_verified, bool):
        raise PolicyObservationError(
            "invalid_calibration", "calibration_verified must be boolean"
        )
    if require_verified_calibration and not calibration_verified:
        raise PolicyObservationError(
            "calibration_unverified", "deployment requires verified calibration"
        )
    if now_ns is not None:
        current = _require_uint64(now_ns, "now_ns")
        if timestamp_ns > current or current - timestamp_ns > resolved.max_observation_age_ns:
            raise PolicyObservationError("stale_observation", "observation is stale or future-dated")

    target = raw.get("target")
    robot = raw.get("robot")
    if not isinstance(target, Mapping) or not isinstance(robot, Mapping):
        raise PolicyObservationError("invalid_shape", "target and robot must be objects")
    position = _finite_vector(target.get("position_m"), "target.position_m", 3)
    covariance = _finite_vector(
        target.get("covariance_diagonal_m2"), "target.covariance_diagonal_m2", 3
    )
    if any(item < 0.0 for item in covariance):
        raise PolicyObservationError("invalid_covariance", "covariance must be non-negative")
    confidence = _finite(target.get("confidence"), "target.confidence")
    if not resolved.minimum_target_confidence <= confidence <= 1.0:
        raise PolicyObservationError("low_confidence", "target confidence is below the contract")
    if target.get("ground_truth_used") is not False:
        raise PolicyObservationError(
            "privileged_state", "target ground_truth_used must be false"
        )

    names = _text_vector(robot.get("joint_names"), "robot.joint_names")
    positions = _finite_vector(robot.get("joint_position_rad"), "robot.joint_position_rad")
    if not names or len(names) > resolved.max_joint_count or len(names) != len(positions):
        raise PolicyObservationError("invalid_shape", "robot joint arrays are inconsistent")
    base_pose = _finite_vector(robot.get("base_pose"), "robot.base_pose", 3)
    base_velocity = _finite_vector(robot.get("base_velocity"), "robot.base_velocity", 2)

    normalized_target = {
        "label": _require_text(target.get("label"), "target.label"),
        "position_m": position,
        "covariance_diagonal_m2": covariance,
        "confidence": confidence,
        "source": _require_text(target.get("source"), "target.source"),
        "ground_truth_used": False,
    }
    normalized_robot = {
        "joint_names": names,
        "joint_position_rad": positions,
        "base_pose": base_pose,
        "base_velocity": base_velocity,
        "state_source": _require_text(robot.get("state_source"), "robot.state_source"),
    }
    return PolicyObservation(
        schema_version=resolved.schema_version,
        sequence=sequence,
        timestamp_ns=timestamp_ns,
        clock_domain=_require_text(raw.get("clock_domain"), "clock_domain"),
        source=_require_text(raw.get("source"), "source"),
        target_frame=_require_text(raw.get("target_frame"), "target_frame"),
        calibration_id=_require_text(raw.get("calibration_id"), "calibration_id"),
        calibration_verified=calibration_verified,
        target=normalized_target,
        robot=normalized_robot,
    )


def build_policy_observation(
    *,
    sequence: int,
    timestamp_ns: int,
    clock_domain: str,
    calibration_id: str,
    calibration_verified: bool,
    target_estimate: Mapping[str, Any],
    joint_names: Sequence[str],
    joint_position_rad: Sequence[float],
    base_pose: Sequence[float],
    base_velocity: Sequence[float],
    state_source: str,
    source: str = "sensor_fusion",
    contract: PolicyObservationContract | None = None,
) -> PolicyObservation:
    if not isinstance(target_estimate, Mapping):
        raise PolicyObservationError("invalid_target", "target_estimate must be an object")
    target = {
        "label": target_estimate.get("label"),
        "position_m": target_estimate.get("position_target_m"),
        "covariance_diagonal_m2": target_estimate.get("covariance_diagonal_m2"),
        "confidence": target_estimate.get("confidence"),
        "source": target_estimate.get("source"),
        "ground_truth_used": target_estimate.get("ground_truth_used"),
    }
    raw = {
        "schema_version": (contract or load_policy_observation_contract()).schema_version,
        "sequence": sequence,
        "timestamp_ns": timestamp_ns,
        "clock_domain": clock_domain,
        "source": source,
        "target_frame": target_estimate.get("target_frame"),
        "calibration_id": calibration_id,
        "calibration_verified": calibration_verified,
        "target": target,
        "robot": {
            "joint_names": list(joint_names),
            "joint_position_rad": list(joint_position_rad),
            "base_pose": list(base_pose),
            "base_velocity": list(base_velocity),
            "state_source": state_source,
        },
        "ground_truth_used": False,
        "privileged_state": False,
        "control_authorized": False,
        "hardware_execution": False,
    }
    return validate_policy_observation(raw, contract=contract)
