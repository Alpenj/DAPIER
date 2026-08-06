"""Small public protocols and frame validation for the G0 contract.

This module intentionally has no ROS 2, LeRobot, MuJoCo, serial, or hardware
dependency.  Later gates can adapt those systems at a process boundary without
changing the frame meaning validated here.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Any, Mapping, Protocol, runtime_checkable

FRAME_FIELDS = frozenset(
    {
        "embodiment_id",
        "embodiment_revision",
        "channel_names",
        "values",
        "units",
        "calibration_id",
        "monotonic_timestamp_ns",
        "sequence_id",
        "source",
    }
)

VALID_SOURCES = frozenset(
    {
        "scripted",
        "human_virtual_leader",
        "physical_leader",
        "sim_follower_readback",
        "physical_follower_readback",
        "policy",
    }
)


class FrameContractError(ValueError):
    """Raised when a frame cannot cross the DAPIER sim-first boundary."""


@dataclass(frozen=True, slots=True)
class Frame:
    embodiment_id: str
    embodiment_revision: str
    channel_names: tuple[str, ...]
    values: tuple[float, ...]
    units: tuple[str, ...]
    calibration_id: str
    monotonic_timestamp_ns: int
    sequence_id: int
    source: str

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "Frame":
        fields = frozenset(payload)
        if fields != FRAME_FIELDS:
            missing = sorted(FRAME_FIELDS - fields)
            extra = sorted(fields - FRAME_FIELDS)
            raise FrameContractError(
                f"frame fields mismatch: missing={missing}, extra={extra}"
            )
        try:
            return cls(
                embodiment_id=payload["embodiment_id"],
                embodiment_revision=payload["embodiment_revision"],
                channel_names=tuple(payload["channel_names"]),
                values=tuple(payload["values"]),
                units=tuple(payload["units"]),
                calibration_id=payload["calibration_id"],
                monotonic_timestamp_ns=payload["monotonic_timestamp_ns"],
                sequence_id=payload["sequence_id"],
                source=payload["source"],
            )
        except (KeyError, TypeError) as exc:
            raise FrameContractError(
                f"frame value has the wrong container type: {exc}"
            ) from exc

    def to_mapping(self) -> dict[str, Any]:
        return {
            "embodiment_id": self.embodiment_id,
            "embodiment_revision": self.embodiment_revision,
            "channel_names": list(self.channel_names),
            "values": list(self.values),
            "units": list(self.units),
            "calibration_id": self.calibration_id,
            "monotonic_timestamp_ns": self.monotonic_timestamp_ns,
            "sequence_id": self.sequence_id,
            "source": self.source,
        }


@runtime_checkable
class Leader(Protocol):
    """The DAPIER-owned equivalent of the observed minimal leader seam."""

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def get_action(self) -> Frame: ...


class FrameSpec(Protocol):
    embodiment_id: str
    embodiment_revision: str
    channel_names: tuple[str, ...]
    calibration_id: str

    def units_for_source(self, source: str) -> tuple[str, ...]: ...


def _require_plain_int(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise FrameContractError(f"{field} must be an integer")
    if value < 0:
        raise FrameContractError(f"{field} must be nonnegative")
    return value


def validate_frame(
    frame: Frame,
    *,
    spec: FrameSpec,
    now_ns: int,
    control_period_ns: int,
    previous: Frame | None = None,
) -> None:
    """Validate exact identity, order, units, sequence, timestamp, and age.

    A frame whose age is exactly two control periods is accepted.  Only
    ``age > 2T`` is stale, matching the work contract.
    """

    _require_plain_int(now_ns, "now_ns")
    period = _require_plain_int(control_period_ns, "control_period_ns")
    if period == 0:
        raise FrameContractError("control_period_ns must be positive")

    if frame.embodiment_id != spec.embodiment_id:
        raise FrameContractError("embodiment_id mismatch")
    if frame.embodiment_revision != spec.embodiment_revision:
        raise FrameContractError("embodiment_revision mismatch")
    if frame.calibration_id != spec.calibration_id:
        raise FrameContractError("calibration_id mismatch")
    if frame.channel_names != spec.channel_names:
        raise FrameContractError("channel_names order mismatch")
    if frame.source not in VALID_SOURCES:
        raise FrameContractError(f"unsupported source: {frame.source!r}")

    expected_units = spec.units_for_source(frame.source)
    if frame.units != expected_units:
        raise FrameContractError("units mismatch")
    if len(frame.values) != len(spec.channel_names):
        raise FrameContractError("values width mismatch")
    for value in frame.values:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not isfinite(value)
        ):
            raise FrameContractError("values must contain only finite numbers")

    timestamp = _require_plain_int(
        frame.monotonic_timestamp_ns, "monotonic_timestamp_ns"
    )
    sequence = _require_plain_int(frame.sequence_id, "sequence_id")
    if timestamp > now_ns:
        raise FrameContractError("monotonic_timestamp_ns is in the future")
    if now_ns - timestamp > 2 * period:
        raise FrameContractError("stale frame: age > 2T")

    if previous is not None:
        if sequence <= previous.sequence_id:
            raise FrameContractError("sequence_id must be strictly increasing")
        if timestamp < previous.monotonic_timestamp_ns:
            raise FrameContractError("monotonic_timestamp_ns must be nondecreasing")
