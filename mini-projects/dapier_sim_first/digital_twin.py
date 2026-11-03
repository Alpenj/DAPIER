"""Offline, hardware-neutral metrics for a DAPIER digital twin.

The evaluator never opens a serial port or commands a robot.  It compares
already synchronized command, simulation, and physical readback traces.  This
keeps data alignment and acceptance thresholds separate from the process that
owns the real actuator.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
from statistics import median
from typing import Any


TRACE_SOURCES = frozenset({"command", "simulation", "physical"})


class DigitalTwinContractError(ValueError):
    """Raised when traces cannot be compared without changing their meaning."""


def _require_nonnegative_int(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise DigitalTwinContractError(f"{field} must be a nonnegative integer")
    return value


def _require_optional_nonnegative_float(value: float | None, field: str) -> None:
    if value is None:
        return
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DigitalTwinContractError(f"{field} must be a nonnegative finite number")
    if not isfinite(value) or value < 0:
        raise DigitalTwinContractError(f"{field} must be a nonnegative finite number")


@dataclass(frozen=True, slots=True)
class JointTrace:
    """One ordered, radian-valued joint trace from a declared source."""

    source: str
    contract_id: str
    source_revision: str
    joint_names: tuple[str, ...]
    timestamps_ns: tuple[int, ...]
    positions_rad: tuple[tuple[float, ...], ...]

    def __post_init__(self) -> None:
        if self.source not in TRACE_SOURCES:
            raise DigitalTwinContractError(f"unsupported trace source: {self.source!r}")
        if not isinstance(self.contract_id, str) or not self.contract_id:
            raise DigitalTwinContractError("contract_id must be a non-empty string")
        if not isinstance(self.source_revision, str) or not self.source_revision:
            raise DigitalTwinContractError("source_revision must be a non-empty string")
        if not self.joint_names or any(
            not isinstance(name, str) or not name for name in self.joint_names
        ):
            raise DigitalTwinContractError("joint_names must contain non-empty strings")
        if len(self.joint_names) != len(set(self.joint_names)):
            raise DigitalTwinContractError("joint_names must be unique")
        if len(self.timestamps_ns) < 2:
            raise DigitalTwinContractError("a trace needs at least two samples")
        if len(self.positions_rad) != len(self.timestamps_ns):
            raise DigitalTwinContractError(
                "positions_rad and timestamps_ns must contain the same number of samples"
            )

        previous: int | None = None
        for timestamp in self.timestamps_ns:
            value = _require_nonnegative_int(timestamp, "timestamp")
            if previous is not None and value <= previous:
                raise DigitalTwinContractError("timestamps_ns must be strictly increasing")
            previous = value

        width = len(self.joint_names)
        for row in self.positions_rad:
            if len(row) != width:
                raise DigitalTwinContractError(
                    f"every positions_rad row must have width {width}"
                )
            for value in row:
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not isfinite(value)
                ):
                    raise DigitalTwinContractError(
                        "positions_rad must contain only finite numeric values"
                    )


@dataclass(frozen=True, slots=True)
class TwinThresholds:
    """Optional experiment thresholds; omitted values remain measurement-only."""

    max_rmse_rad: float | None = None
    max_endpoint_error_rad: float | None = None
    max_timestamp_skew_ns: int | None = None
    max_delay_gap_steps: int | None = None

    def __post_init__(self) -> None:
        _require_optional_nonnegative_float(self.max_rmse_rad, "max_rmse_rad")
        _require_optional_nonnegative_float(
            self.max_endpoint_error_rad, "max_endpoint_error_rad"
        )
        if self.max_timestamp_skew_ns is not None:
            _require_nonnegative_int(self.max_timestamp_skew_ns, "max_timestamp_skew_ns")
        if self.max_delay_gap_steps is not None:
            _require_nonnegative_int(self.max_delay_gap_steps, "max_delay_gap_steps")

    def to_mapping(self) -> dict[str, float | int | None]:
        return {
            "max_rmse_rad": self.max_rmse_rad,
            "max_endpoint_error_rad": self.max_endpoint_error_rad,
            "max_timestamp_skew_ns": self.max_timestamp_skew_ns,
            "max_delay_gap_steps": self.max_delay_gap_steps,
        }


def _rmse(values: list[float]) -> float:
    return sqrt(sum(value * value for value in values) / len(values))


def _percentile_95(values: list[float]) -> float:
    ordered = sorted(values)
    index = max(0, (95 * len(ordered) + 99) // 100 - 1)
    return ordered[index]


def _best_delay_steps(
    command: tuple[tuple[float, ...], ...],
    response: tuple[tuple[float, ...], ...],
    *,
    joint_index: int,
    max_lag_steps: int,
) -> tuple[int, float]:
    candidates: list[tuple[float, int]] = []
    for lag in range(max_lag_steps + 1):
        errors = [
            command[index][joint_index] - response[index + lag][joint_index]
            for index in range(len(command) - lag)
        ]
        candidates.append((_rmse(errors), lag))
    tracking_rmse, lag = min(candidates)
    return lag, tracking_rmse


def evaluate_digital_twin(
    *,
    command: JointTrace,
    simulation: JointTrace,
    physical: JointTrace,
    max_lag_steps: int = 5,
    thresholds: TwinThresholds | None = None,
) -> dict[str, Any]:
    """Compare synchronized traces and return JSON-serializable evidence.

    The function estimates nonnegative command-to-readback delay by selecting
    the sample lag with the lowest tracking RMSE.  It does not choose acceptance
    thresholds: without ``thresholds`` the result is ``MEASURED`` rather than a
    pass claim.
    """

    expected_sources = ("command", "simulation", "physical")
    for trace, expected in zip(
        (command, simulation, physical), expected_sources, strict=True
    ):
        if trace.source != expected:
            raise DigitalTwinContractError(
                f"expected {expected!r} trace, got {trace.source!r}"
            )

    if simulation.joint_names != command.joint_names:
        raise DigitalTwinContractError("simulation joint order differs from command")
    if physical.joint_names != command.joint_names:
        raise DigitalTwinContractError("physical joint order differs from command")
    if simulation.contract_id != command.contract_id:
        raise DigitalTwinContractError("simulation contract_id differs from command")
    if physical.contract_id != command.contract_id:
        raise DigitalTwinContractError("physical contract_id differs from command")
    if not (
        len(command.timestamps_ns)
        == len(simulation.timestamps_ns)
        == len(physical.timestamps_ns)
    ):
        raise DigitalTwinContractError("all traces must contain the same sample count")

    sample_count = len(command.timestamps_ns)
    lag_limit = _require_nonnegative_int(max_lag_steps, "max_lag_steps")
    if lag_limit >= sample_count:
        raise DigitalTwinContractError("max_lag_steps must be smaller than sample count")

    periods = [
        right - left
        for left, right in zip(
            command.timestamps_ns, command.timestamps_ns[1:]
        )
    ]
    period_ns = int(round(median(periods)))
    timestamp_skews = [
        max(command_time, simulation_time, physical_time)
        - min(command_time, simulation_time, physical_time)
        for command_time, simulation_time, physical_time in zip(
            command.timestamps_ns,
            simulation.timestamps_ns,
            physical.timestamps_ns,
            strict=True,
        )
    ]
    max_timestamp_skew_ns = max(timestamp_skews)

    per_joint: dict[str, dict[str, float | int]] = {}
    for joint_index, joint_name in enumerate(command.joint_names):
        sim_real_errors = [
            simulation.positions_rad[index][joint_index]
            - physical.positions_rad[index][joint_index]
            for index in range(sample_count)
        ]
        absolute_errors = [abs(value) for value in sim_real_errors]
        simulation_delay, simulation_tracking_rmse = _best_delay_steps(
            command.positions_rad,
            simulation.positions_rad,
            joint_index=joint_index,
            max_lag_steps=lag_limit,
        )
        physical_delay, physical_tracking_rmse = _best_delay_steps(
            command.positions_rad,
            physical.positions_rad,
            joint_index=joint_index,
            max_lag_steps=lag_limit,
        )
        delay_gap_steps = physical_delay - simulation_delay
        per_joint[joint_name] = {
            "mae_rad": sum(absolute_errors) / sample_count,
            "rmse_rad": _rmse(sim_real_errors),
            "p95_abs_error_rad": _percentile_95(absolute_errors),
            "endpoint_error_rad": absolute_errors[-1],
            "simulation_delay_steps": simulation_delay,
            "physical_delay_steps": physical_delay,
            "delay_gap_steps": delay_gap_steps,
            "delay_gap_ns": delay_gap_steps * period_ns,
            "simulation_command_tracking_rmse_rad": simulation_tracking_rmse,
            "physical_command_tracking_rmse_rad": physical_tracking_rmse,
        }

    aggregate = {
        "max_joint_rmse_rad": max(
            float(metrics["rmse_rad"]) for metrics in per_joint.values()
        ),
        "max_joint_endpoint_error_rad": max(
            float(metrics["endpoint_error_rad"]) for metrics in per_joint.values()
        ),
        "max_abs_delay_gap_steps": max(
            abs(int(metrics["delay_gap_steps"])) for metrics in per_joint.values()
        ),
        "max_timestamp_skew_ns": max_timestamp_skew_ns,
    }

    violations: list[str] = []
    if thresholds is not None:
        if (
            thresholds.max_rmse_rad is not None
            and aggregate["max_joint_rmse_rad"] > thresholds.max_rmse_rad
        ):
            violations.append("max_joint_rmse_rad")
        if (
            thresholds.max_endpoint_error_rad is not None
            and aggregate["max_joint_endpoint_error_rad"]
            > thresholds.max_endpoint_error_rad
        ):
            violations.append("max_joint_endpoint_error_rad")
        if (
            thresholds.max_timestamp_skew_ns is not None
            and max_timestamp_skew_ns > thresholds.max_timestamp_skew_ns
        ):
            violations.append("max_timestamp_skew_ns")
        if (
            thresholds.max_delay_gap_steps is not None
            and aggregate["max_abs_delay_gap_steps"]
            > thresholds.max_delay_gap_steps
        ):
            violations.append("max_abs_delay_gap_steps")

    thresholds_mapping = thresholds.to_mapping() if thresholds is not None else None
    has_threshold = thresholds_mapping is not None and any(
        value is not None for value in thresholds_mapping.values()
    )
    if not has_threshold:
        status = "MEASURED"
        passed: bool | None = None
    else:
        passed = not violations
        status = "PASS" if passed else "FAIL"

    return {
        "schema_version": "dapier.digital-twin.v1",
        "status": status,
        "passed": passed,
        "sample_count": sample_count,
        "joint_names": list(command.joint_names),
        "units": "radian",
        "contract_id": command.contract_id,
        "source_revisions": {
            "command": command.source_revision,
            "simulation": simulation.source_revision,
            "physical": physical.source_revision,
        },
        "period_ns": period_ns,
        "alignment": {
            "mode": "pre_synchronized_index",
            "max_timestamp_skew_ns": max_timestamp_skew_ns,
        },
        "per_joint": per_joint,
        "aggregate": aggregate,
        "thresholds": thresholds_mapping,
        "violations": violations,
        "evidence_boundary": (
            "offline trace comparison only; no serial access, hardware command, "
            "or sim-to-real safety certification"
        ),
    }
