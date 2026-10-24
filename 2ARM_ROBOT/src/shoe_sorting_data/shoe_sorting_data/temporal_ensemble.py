"""Hardware-free temporal ensembling and command-stream jitter metrics.

This module only combines policy proposals. It has no ROS, serial, motor, or
hardware imports and never authorizes control. Any future live integration
must submit its output to the independent rollout safety supervisor.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Sequence


TEMPORAL_ENSEMBLE_SCHEMA_VERSION = "dapier.temporal-ensemble.v0.1"
JITTER_REPORT_SCHEMA_VERSION = "dapier.command-jitter-report.v0.1"


class TemporalEnsembleError(ValueError):
    """Raised when an action chunk cannot be combined safely."""


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TemporalEnsembleError(f"{field} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise TemporalEnsembleError(f"{field} must be a finite number")
    return result


def _finite_action(value: Any, *, action_dim: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != action_dim:
        raise TemporalEnsembleError(f"{field} must contain exactly {action_dim} values")
    return tuple(_finite_number(item, f"{field}[{index}]") for index, item in enumerate(value))


def _finite_chunk(value: Any, *, chunk_size: int, action_dim: int) -> tuple[tuple[float, ...], ...]:
    if not isinstance(value, (list, tuple)) or len(value) != chunk_size:
        raise TemporalEnsembleError(f"action chunk must contain exactly {chunk_size} steps")
    return tuple(
        _finite_action(item, action_dim=action_dim, field=f"action_chunk[{index}]")
        for index, item in enumerate(value)
    )


@dataclass(frozen=True)
class TemporalEnsembleConfig:
    action_dim: int
    chunk_size: int
    coefficient: float = 0.01
    max_source_age_ms: float | None = None
    max_disagreement: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        if isinstance(self.action_dim, bool) or not isinstance(self.action_dim, int) or self.action_dim <= 0:
            raise TemporalEnsembleError("action_dim must be a positive integer")
        if isinstance(self.chunk_size, bool) or not isinstance(self.chunk_size, int) or self.chunk_size <= 0:
            raise TemporalEnsembleError("chunk_size must be a positive integer")
        coefficient = _finite_number(self.coefficient, "coefficient")
        if coefficient < 0:
            raise TemporalEnsembleError("coefficient must be non-negative; negative values favor newer chunks")
        if self.max_source_age_ms is not None:
            source_age = _finite_number(self.max_source_age_ms, "max_source_age_ms")
            if source_age <= 0:
                raise TemporalEnsembleError("max_source_age_ms must be positive")
        if self.max_disagreement is not None:
            disagreement = _finite_action(
                self.max_disagreement,
                action_dim=self.action_dim,
                field="max_disagreement",
            )
            if any(value <= 0 for value in disagreement):
                raise TemporalEnsembleError("max_disagreement values must be positive")


@dataclass(frozen=True)
class TemporalEnsembleResult:
    action: tuple[float, ...]
    contributor_count: int
    contributor_ages: tuple[int, ...]
    normalized_weights: tuple[float, ...]
    disagreement: tuple[float, ...]
    source_age_ms: tuple[float | None, ...]
    generation: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": TEMPORAL_ENSEMBLE_SCHEMA_VERSION,
            "action": list(self.action),
            "contributor_count": self.contributor_count,
            "contributor_ages": list(self.contributor_ages),
            "normalized_weights": list(self.normalized_weights),
            "disagreement": list(self.disagreement),
            "source_age_ms": list(self.source_age_ms),
            "generation": self.generation,
            "control_authorized": False,
            "hardware_execution": "NOT_ATTEMPTED",
        }


@dataclass(frozen=True)
class _PendingChunk:
    age: int
    actions: tuple[tuple[float, ...], ...]
    source_observation_monotonic_ns: int | None


class TemporalEnsembler:
    """Blend aligned predictions from overlapping action chunks.

    Chunks are kept oldest-first. The oldest prediction receives weight 1,
    and the i-th newer prediction receives exp(-coefficient * i). This
    matches LeRobot ACT's positive-coefficient convention.
    """

    def __init__(self, config: TemporalEnsembleConfig) -> None:
        self.config = config
        self._pending: list[_PendingChunk] = []
        self.generation = 0

    def reset(self) -> None:
        self._pending.clear()
        self.generation += 1

    def __len__(self) -> int:
        return len(self._pending)

    def update(
        self,
        action_chunk: Sequence[Sequence[float]],
        *,
        source_observation_monotonic_ns: int | None = None,
        now_monotonic_ns: int | None = None,
    ) -> TemporalEnsembleResult:
        """Validate one fresh chunk, return the current proposal, then advance.

        Rejected updates are transactional: the pending ensemble is unchanged.
        """

        actions = _finite_chunk(
            action_chunk,
            chunk_size=self.config.chunk_size,
            action_dim=self.config.action_dim,
        )
        if (source_observation_monotonic_ns is None) != (now_monotonic_ns is None):
            raise TemporalEnsembleError("source and evaluation monotonic timestamps must be provided together")
        if self.config.max_source_age_ms is not None and source_observation_monotonic_ns is None:
            raise TemporalEnsembleError("timestamps are required when max_source_age_ms is configured")
        if source_observation_monotonic_ns is not None:
            if (
                isinstance(source_observation_monotonic_ns, bool)
                or not isinstance(source_observation_monotonic_ns, int)
                or source_observation_monotonic_ns < 0
            ):
                raise TemporalEnsembleError("source_observation_monotonic_ns must be a non-negative integer")
            if isinstance(now_monotonic_ns, bool) or not isinstance(now_monotonic_ns, int) or now_monotonic_ns < 0:
                raise TemporalEnsembleError("now_monotonic_ns must be a non-negative integer")

        trial = [
            *self._pending,
            _PendingChunk(
                age=0,
                actions=actions,
                source_observation_monotonic_ns=source_observation_monotonic_ns,
            ),
        ]
        candidates = [entry.actions[entry.age] for entry in trial]
        source_ages: list[float | None] = []
        for entry in trial:
            if entry.source_observation_monotonic_ns is None:
                source_ages.append(None)
                continue
            age_ms = (now_monotonic_ns - entry.source_observation_monotonic_ns) / 1_000_000
            if age_ms < 0:
                raise TemporalEnsembleError("source observation timestamp is in the future")
            if self.config.max_source_age_ms is not None and age_ms > self.config.max_source_age_ms:
                raise TemporalEnsembleError("source observation is stale")
            source_ages.append(age_ms)

        raw_weights = [math.exp(-self.config.coefficient * rank) for rank in range(len(candidates))]
        weight_sum = sum(raw_weights)
        weights = [weight / weight_sum for weight in raw_weights]
        disagreement = tuple(
            max(candidate[joint] for candidate in candidates) - min(candidate[joint] for candidate in candidates)
            for joint in range(self.config.action_dim)
        )
        if self.config.max_disagreement is not None:
            exceeded = [
                index
                for index, (actual, allowed) in enumerate(
                    zip(disagreement, self.config.max_disagreement, strict=True)
                )
                if actual > allowed
            ]
            if exceeded:
                raise TemporalEnsembleError(
                    "overlapping chunks disagree beyond configured limits at action indices: "
                    + ", ".join(str(index) for index in exceeded)
                )
        action = tuple(
            sum(weight * candidate[joint] for weight, candidate in zip(weights, candidates, strict=True))
            for joint in range(self.config.action_dim)
        )
        result = TemporalEnsembleResult(
            action=action,
            contributor_count=len(candidates),
            contributor_ages=tuple(entry.age for entry in trial),
            normalized_weights=tuple(weights),
            disagreement=disagreement,
            source_age_ms=tuple(source_ages),
            generation=self.generation,
        )
        self._pending = [
            _PendingChunk(
                age=entry.age + 1,
                actions=entry.actions,
                source_observation_monotonic_ns=entry.source_observation_monotonic_ns,
            )
            for entry in trial
            if entry.age + 1 < self.config.chunk_size
        ]
        return result


def _differences(rows: list[tuple[float, ...]]) -> list[tuple[float, ...]]:
    return [
        tuple(current[index] - previous[index] for index in range(len(current)))
        for previous, current in zip(rows, rows[1:])
    ]


def _scale(rows: list[tuple[float, ...]], factor: float) -> list[tuple[float, ...]]:
    return [tuple(value * factor for value in row) for row in rows]


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def _summarize(rows: list[tuple[float, ...]], action_dim: int) -> dict[str, Any]:
    per_joint: list[dict[str, float]] = []
    for joint in range(action_dim):
        values = [abs(row[joint]) for row in rows]
        per_joint.append(
            {
                "rms": math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0,
                "p95_abs": _percentile(values, 0.95),
                "max_abs": max(values, default=0.0),
            }
        )
    flat = [abs(value) for row in rows for value in row]
    return {
        "samples": len(rows),
        "rms": math.sqrt(sum(value * value for value in flat) / len(flat)) if flat else 0.0,
        "p95_abs": _percentile(flat, 0.95),
        "max_abs": max(flat, default=0.0),
        "per_joint": per_joint,
    }


def command_stream_metrics(actions: Sequence[Sequence[float]], *, fps: float) -> dict[str, Any]:
    """Measure command deltas, velocity, acceleration, and jerk without hardware."""

    rate = _finite_number(fps, "fps")
    if rate <= 0:
        raise TemporalEnsembleError("fps must be positive")
    if not isinstance(actions, (list, tuple)) or not actions:
        raise TemporalEnsembleError("actions must contain at least one command")
    first = actions[0]
    if not isinstance(first, (list, tuple)) or not first:
        raise TemporalEnsembleError("actions must contain non-empty command vectors")
    action_dim = len(first)
    rows = [_finite_action(row, action_dim=action_dim, field=f"actions[{index}]") for index, row in enumerate(actions)]
    deltas = _differences(rows)
    velocities = _scale(deltas, rate)
    accelerations = _scale(_differences(velocities), rate)
    jerks = _scale(_differences(accelerations), rate)
    return {
        "schema_version": JITTER_REPORT_SCHEMA_VERSION,
        "command_samples": len(rows),
        "action_dim": action_dim,
        "fps": rate,
        "delta": _summarize(deltas, action_dim),
        "velocity": _summarize(velocities, action_dim),
        "acceleration": _summarize(accelerations, action_dim),
        "jerk": _summarize(jerks, action_dim),
    }


def _improvement(baseline: float, candidate: float) -> float | None:
    return None if baseline == 0 else 100.0 * (baseline - candidate) / baseline


def build_jitter_smoke_report(*, fps: float = 30.0) -> dict[str, Any]:
    """Run a deterministic overlapping-chunk A/B fixture."""

    config = TemporalEnsembleConfig(action_dim=2, chunk_size=6, coefficient=0.01)
    ensembler = TemporalEnsembler(config)
    raw_actions: list[list[float]] = []
    ensemble_actions: list[list[float]] = []
    contributor_counts: list[int] = []
    for step in range(36):
        alternating_offset = 0.08 if step % 2 == 0 else -0.08
        chunk = []
        for horizon in range(config.chunk_size):
            progress = 0.005 * (step + horizon)
            chunk.append([progress + alternating_offset, -0.5 * progress - alternating_offset])
        raw_actions.append(list(chunk[0]))
        result = ensembler.update(chunk)
        ensemble_actions.append(list(result.action))
        contributor_counts.append(result.contributor_count)
    raw_metrics = command_stream_metrics(raw_actions, fps=fps)
    ensemble_metrics = command_stream_metrics(ensemble_actions, fps=fps)
    acceleration_improvement = _improvement(
        raw_metrics["acceleration"]["rms"],
        ensemble_metrics["acceleration"]["rms"],
    )
    jerk_improvement = _improvement(raw_metrics["jerk"]["rms"], ensemble_metrics["jerk"]["rms"])
    passed = bool(
        acceleration_improvement
        and acceleration_improvement > 0
        and jerk_improvement
        and jerk_improvement > 0
    )
    return {
        "schema_version": JITTER_REPORT_SCHEMA_VERSION,
        "status": "PASS" if passed else "FAIL",
        "scope": "deterministic synthetic overlapping action chunks; no ROS, serial, or hardware",
        "config": {
            "action_dim": config.action_dim,
            "chunk_size": config.chunk_size,
            "coefficient": config.coefficient,
            "fps": fps,
        },
        "raw_chunk": raw_metrics,
        "temporal_ensemble": ensemble_metrics,
        "comparison": {
            "acceleration_rms_reduction_percent": acceleration_improvement,
            "jerk_rms_reduction_percent": jerk_improvement,
            "maximum_contributors": max(contributor_counts),
        },
        "control_authorized": False,
        "hardware_execution": "NOT_ATTEMPTED",
        "required_next_gate": "recorded-policy replay and simulator A/B before any approved physical rollout",
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Hardware-free temporal ensembling jitter preparation")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise TemporalEnsembleError(f"output already exists: {args.output}")
    report = build_jitter_smoke_report(fps=args.fps)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
