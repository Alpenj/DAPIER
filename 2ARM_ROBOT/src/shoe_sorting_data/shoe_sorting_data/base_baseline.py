"""Persist and summarize stationary TurtleBot odometry without issuing commands."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


BASELINE_SCHEMA_VERSION = "dapier.base-stationary-baseline.v0.1"


def derive_stationary_tolerances(
    summary: Mapping[str, object], *, granularity: float = 0.0001
) -> dict[str, float]:
    """Derive separate, conservative limits from an observed stationary baseline."""
    if granularity <= 0:
        raise ValueError("granularity must be positive")

    def derive(stream_name: str) -> float:
        stats = summary.get(stream_name)
        if not isinstance(stats, Mapping):
            raise ValueError(f"summary.{stream_name} must be an object")
        try:
            observed_max = float(stats["abs_max"])
            observed_p99 = float(stats["abs_p99"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"summary.{stream_name} lacks numeric abs_max/abs_p99") from error
        if observed_max < 0 or observed_p99 < 0:
            raise ValueError(f"summary.{stream_name} statistics must be non-negative")
        conservative = max(observed_max * 2.0, observed_p99 * 3.0)
        return round(math.ceil(conservative / granularity) * granularity, 10)

    return {
        "linear_x_mps": derive("linear_x_mps"),
        "angular_z_radps": derive("angular_z_radps"),
    }


def _percentile(values: Sequence[float], percentile: float) -> float:
    if not values:
        raise ValueError("cannot calculate a percentile from no values")
    if not 0.0 <= percentile <= 100.0:
        raise ValueError("percentile must be from 0 to 100")
    ordered = sorted(float(value) for value in values)
    rank = (len(ordered) - 1) * percentile / 100.0
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def summarize_samples(samples: Sequence[Mapping[str, float | int]]) -> dict[str, object]:
    if len(samples) < 2:
        raise ValueError("at least two odometry samples are required")
    reception_times = [int(sample["reception_monotonic_ns"]) for sample in samples]
    if any(new <= old for old, new in zip(reception_times, reception_times[1:])):
        raise ValueError("reception timestamps must increase strictly")
    duration_ns = reception_times[-1] - reception_times[0]
    if duration_ns <= 0:
        raise ValueError("sample duration must be positive")

    def stats(key: str) -> dict[str, float]:
        absolute = [abs(float(sample[key])) for sample in samples]
        return {
            "abs_median": _percentile(absolute, 50.0),
            "abs_p95": _percentile(absolute, 95.0),
            "abs_p99": _percentile(absolute, 99.0),
            "abs_max": max(absolute),
        }

    return {
        "sample_count": len(samples),
        "duration_seconds": duration_ns / 1_000_000_000,
        "sample_rate_hz": (len(samples) - 1) * 1_000_000_000 / duration_ns,
        "linear_x_mps": stats("linear_x_mps"),
        "angular_z_radps": stats("angular_z_radps"),
    }


def save_baseline(
    output_dir: str | Path,
    samples: Sequence[Mapping[str, float | int]],
    *,
    source_topic: str,
    warmup_seconds: float,
) -> tuple[Path, Path]:
    root = Path(output_dir)
    if root.exists() and any(root.iterdir()):
        raise ValueError(f"baseline output directory is not empty: {root}")
    summary = summarize_samples(samples)
    payload = "".join(
        json.dumps(dict(sample), sort_keys=True, separators=(",", ":")) + "\n"
        for sample in samples
    ).encode("utf-8")
    report = {
        "schema_version": BASELINE_SCHEMA_VERSION,
        "created_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "source_topic": source_topic,
        "warmup_seconds": warmup_seconds,
        "motion_commands_sent": False,
        "samples_sha256": hashlib.sha256(payload).hexdigest(),
        "summary": summary,
    }
    root.mkdir(parents=True, exist_ok=True)
    samples_path = root / "odom_samples.jsonl"
    report_path = root / "baseline_report.json"
    samples_path.write_bytes(payload)
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return samples_path, report_path
