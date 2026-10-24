"""Versioned, hardware-agnostic intents produced by the Python research layer.

The Python layer may propose bounded semantic intents, but it never authorizes or
performs hardware I/O. The C++ real-time ingress remains authoritative for
receiver-local TTL, sequence, robot limits, watchdogs, and dispatch.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Mapping, Sequence


CONTRACT_FILE_NAME = "research_realtime_control_v1.json"
UINT64_MAX = (1 << 64) - 1
INT64_MAX = (1 << 63) - 1

_REQUIRED_INTENT_KEYS = frozenset(
    {
        "schema_version",
        "sequence",
        "kind",
        "source",
        "source_monotonic_ns",
        "ttl_ns",
    }
)
_OPTIONAL_INTENT_KEYS = frozenset(
    {
        "joint_names",
        "joint_position_rad",
        "joint_max_velocity_rad_s",
        "base_linear_x_mps",
        "base_angular_z_rad_s",
    }
)
_ALLOWED_INTENT_KEYS = _REQUIRED_INTENT_KEYS | _OPTIONAL_INTENT_KEYS


@dataclass(frozen=True)
class ControlBoundaryContract:
    schema_version: str
    clock_domain: str
    max_intent_ttl_ns: int
    max_arm_joint_count: int
    allowed_intent_kinds: tuple[str, ...]
    research_may_authorize_hardware: bool


@dataclass(frozen=True)
class ControlIntent:
    """One proposal sent toward the C++ real-time control ingress.

    ``source_monotonic_ns`` is trace metadata from the producer host. It must
    never be compared with another host's monotonic clock. The C++ ingress
    starts ``ttl_ns`` from its own receiver-local monotonic timestamp.
    """

    schema_version: str
    sequence: int
    kind: str
    source: str
    source_monotonic_ns: int
    ttl_ns: int
    joint_names: tuple[str, ...] = ()
    joint_position_rad: tuple[float, ...] = ()
    joint_max_velocity_rad_s: tuple[float, ...] = ()
    base_linear_x_mps: float = 0.0
    base_angular_z_rad_s: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible mapping with explicit empty arrays."""
        return {
            "schema_version": self.schema_version,
            "sequence": self.sequence,
            "kind": self.kind,
            "source": self.source,
            "source_monotonic_ns": self.source_monotonic_ns,
            "ttl_ns": self.ttl_ns,
            "joint_names": list(self.joint_names),
            "joint_position_rad": list(self.joint_position_rad),
            "joint_max_velocity_rad_s": list(self.joint_max_velocity_rad_s),
            "base_linear_x_mps": self.base_linear_x_mps,
            "base_angular_z_rad_s": self.base_angular_z_rad_s,
        }


def default_contract_path() -> Path:
    override = os.environ.get("DAPIER_RESEARCH_REALTIME_CONTRACT")
    if override:
        return Path(override).expanduser()
    return Path(__file__).resolve().parents[4] / "contracts" / CONTRACT_FILE_NAME


def _require_text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _require_positive_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _require_uint64(value: object, name: str) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= UINT64_MAX
    ):
        raise ValueError(f"{name} must be an unsigned 64-bit integer")
    return value


def _require_non_negative_int64(value: object, name: str) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= INT64_MAX
    ):
        raise ValueError(f"{name} must be a non-negative signed 64-bit integer")
    return value


def _require_array(value: object, name: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must be an array")
    return value


def _finite_scalar(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be a finite number")
    return converted


def _finite_tuple(value: object, name: str) -> tuple[float, ...]:
    values = _require_array(value, name)
    return tuple(_finite_scalar(item, f"{name} entry") for item in values)


def _text_tuple(value: object, name: str) -> tuple[str, ...]:
    values = _require_array(value, name)
    return tuple(_require_text(item, f"{name} entry") for item in values)


def load_contract(path: str | Path | None = None) -> ControlBoundaryContract:
    contract_path = Path(path) if path is not None else default_contract_path()
    try:
        raw = json.loads(contract_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"control boundary contract not found: {contract_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON in control boundary contract: {error}") from error
    if not isinstance(raw, Mapping):
        raise ValueError("control boundary contract must be a JSON object")

    kinds = _text_tuple(raw.get("allowed_intent_kinds"), "allowed_intent_kinds")
    if not kinds or len(set(kinds)) != len(kinds):
        raise ValueError("allowed_intent_kinds must be non-empty and unique")
    if raw.get("research_may_authorize_hardware") is not False:
        raise ValueError("research_may_authorize_hardware must remain false")

    return ControlBoundaryContract(
        schema_version=_require_text(raw.get("schema_version"), "schema_version"),
        clock_domain=_require_text(raw.get("clock_domain"), "clock_domain"),
        max_intent_ttl_ns=_require_positive_int(
            raw.get("max_intent_ttl_ns"), "max_intent_ttl_ns"
        ),
        max_arm_joint_count=_require_positive_int(
            raw.get("max_arm_joint_count"), "max_arm_joint_count"
        ),
        allowed_intent_kinds=kinds,
        research_may_authorize_hardware=False,
    )


def validate_intent(
    intent: ControlIntent,
    contract: ControlBoundaryContract | None = None,
) -> None:
    if not isinstance(intent, ControlIntent):
        raise ValueError("intent must be a ControlIntent")
    resolved = contract or load_contract()

    if intent.schema_version != resolved.schema_version:
        raise ValueError("intent schema_version does not match the shared contract")
    _require_uint64(intent.sequence, "sequence")
    _require_text(intent.source, "source")
    _require_non_negative_int64(intent.source_monotonic_ns, "source_monotonic_ns")
    ttl_ns = _require_positive_int(intent.ttl_ns, "ttl_ns")
    if ttl_ns > resolved.max_intent_ttl_ns:
        raise ValueError("ttl_ns exceeds the shared contract maximum")
    if intent.kind not in resolved.allowed_intent_kinds:
        raise ValueError(f"unsupported intent kind: {intent.kind!r}")

    joint_names = _text_tuple(intent.joint_names, "joint_names")
    if len(set(joint_names)) != len(joint_names):
        raise ValueError("joint_names must be unique")
    positions = _finite_tuple(intent.joint_position_rad, "joint_position_rad")
    velocities = _finite_tuple(
        intent.joint_max_velocity_rad_s, "joint_max_velocity_rad_s"
    )
    linear = _finite_scalar(intent.base_linear_x_mps, "base_linear_x_mps")
    angular = _finite_scalar(intent.base_angular_z_rad_s, "base_angular_z_rad_s")

    if intent.kind == "hold":
        if joint_names or positions or velocities:
            raise ValueError("hold intent must not contain joint targets")
        if linear != 0.0 or angular != 0.0:
            raise ValueError("hold intent must request zero base motion")
        return

    if intent.kind == "arm_joint_position":
        if not joint_names or len(joint_names) > resolved.max_arm_joint_count:
            raise ValueError("arm intent joint count is outside the shared contract")
        if len(positions) != len(joint_names) or len(velocities) != len(joint_names):
            raise ValueError("arm intent joint arrays must have identical lengths")
        if any(velocity <= 0.0 for velocity in velocities):
            raise ValueError("joint_max_velocity_rad_s entries must be positive")
        if linear != 0.0 or angular != 0.0:
            raise ValueError("arm intent must not request base motion")
        return

    if joint_names or positions or velocities:
        raise ValueError("base intent must not contain joint targets")


def _build_intent(
    *,
    contract: ControlBoundaryContract,
    sequence: int,
    kind: str,
    source: str,
    ttl_ns: int,
    source_monotonic_ns: int | None,
    joint_names: Sequence[str] = (),
    joint_position_rad: Sequence[float] = (),
    joint_max_velocity_rad_s: Sequence[float] = (),
    base_linear_x_mps: float = 0.0,
    base_angular_z_rad_s: float = 0.0,
) -> ControlIntent:
    intent = ControlIntent(
        schema_version=contract.schema_version,
        sequence=sequence,
        kind=kind,
        source=source,
        source_monotonic_ns=(
            time.monotonic_ns()
            if source_monotonic_ns is None
            else source_monotonic_ns
        ),
        ttl_ns=ttl_ns,
        joint_names=_text_tuple(joint_names, "joint_names"),
        joint_position_rad=_finite_tuple(
            joint_position_rad, "joint_position_rad"
        ),
        joint_max_velocity_rad_s=_finite_tuple(
            joint_max_velocity_rad_s, "joint_max_velocity_rad_s"
        ),
        base_linear_x_mps=_finite_scalar(
            base_linear_x_mps, "base_linear_x_mps"
        ),
        base_angular_z_rad_s=_finite_scalar(
            base_angular_z_rad_s, "base_angular_z_rad_s"
        ),
    )
    validate_intent(intent, contract)
    return intent


def hold_intent(
    *,
    sequence: int,
    source: str,
    ttl_ns: int = 100_000_000,
    source_monotonic_ns: int | None = None,
    contract: ControlBoundaryContract | None = None,
) -> ControlIntent:
    resolved = contract or load_contract()
    return _build_intent(
        contract=resolved,
        sequence=sequence,
        kind="hold",
        source=source,
        ttl_ns=ttl_ns,
        source_monotonic_ns=source_monotonic_ns,
    )


def arm_joint_position_intent(
    *,
    sequence: int,
    source: str,
    joint_names: Sequence[str],
    joint_position_rad: Sequence[float],
    joint_max_velocity_rad_s: Sequence[float],
    ttl_ns: int = 100_000_000,
    source_monotonic_ns: int | None = None,
    contract: ControlBoundaryContract | None = None,
) -> ControlIntent:
    resolved = contract or load_contract()
    return _build_intent(
        contract=resolved,
        sequence=sequence,
        kind="arm_joint_position",
        source=source,
        ttl_ns=ttl_ns,
        source_monotonic_ns=source_monotonic_ns,
        joint_names=joint_names,
        joint_position_rad=joint_position_rad,
        joint_max_velocity_rad_s=joint_max_velocity_rad_s,
    )


def base_twist_intent(
    *,
    sequence: int,
    source: str,
    linear_x_mps: float,
    angular_z_rad_s: float,
    ttl_ns: int = 100_000_000,
    source_monotonic_ns: int | None = None,
    contract: ControlBoundaryContract | None = None,
) -> ControlIntent:
    resolved = contract or load_contract()
    return _build_intent(
        contract=resolved,
        sequence=sequence,
        kind="base_twist",
        source=source,
        ttl_ns=ttl_ns,
        source_monotonic_ns=source_monotonic_ns,
        base_linear_x_mps=linear_x_mps,
        base_angular_z_rad_s=angular_z_rad_s,
    )


def intent_from_mapping(
    value: Mapping[str, Any],
    contract: ControlBoundaryContract | None = None,
) -> ControlIntent:
    if not isinstance(value, Mapping):
        raise ValueError("intent payload must be a JSON object")

    keys = set(value)
    missing = _REQUIRED_INTENT_KEYS - keys
    if missing:
        raise ValueError(f"intent is missing keys: {sorted(missing)}")
    unexpected = keys - _ALLOWED_INTENT_KEYS
    if unexpected:
        rendered = sorted(repr(key) for key in unexpected)
        raise ValueError(f"intent has unexpected keys: {rendered}")

    intent = ControlIntent(
        schema_version=value["schema_version"],
        sequence=value["sequence"],
        kind=value["kind"],
        source=value["source"],
        source_monotonic_ns=value["source_monotonic_ns"],
        ttl_ns=value["ttl_ns"],
        joint_names=_text_tuple(value.get("joint_names", ()), "joint_names"),
        joint_position_rad=_finite_tuple(
            value.get("joint_position_rad", ()), "joint_position_rad"
        ),
        joint_max_velocity_rad_s=_finite_tuple(
            value.get("joint_max_velocity_rad_s", ()),
            "joint_max_velocity_rad_s",
        ),
        base_linear_x_mps=_finite_scalar(
            value.get("base_linear_x_mps", 0.0), "base_linear_x_mps"
        ),
        base_angular_z_rad_s=_finite_scalar(
            value.get("base_angular_z_rad_s", 0.0),
            "base_angular_z_rad_s",
        ),
    )
    validate_intent(intent, contract)
    return intent
