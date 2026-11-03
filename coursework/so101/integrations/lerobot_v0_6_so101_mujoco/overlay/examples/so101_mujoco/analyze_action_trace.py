#!/usr/bin/env python

"""Summarize SO-101 VLA action stability and RCS-ready tracking traces."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np


def _quantiles(values: np.ndarray) -> dict[str, list[float]]:
    return {
        "p50": np.quantile(values, 0.50, axis=0).round(6).tolist(),
        "p95": np.quantile(values, 0.95, axis=0).round(6).tolist(),
        "p99": np.quantile(values, 0.99, axis=0).round(6).tolist(),
        "max": np.max(values, axis=0).round(6).tolist(),
    }


def analyze_trace(path: Path, *, episode_length: int) -> dict[str, Any]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    if not rows:
        raise ValueError(f"Action trace is empty: {path}")
    timestamps = [int(row["timestamp_ns"]) for row in rows]
    if any(right <= left for left, right in zip(timestamps, timestamps[1:], strict=False)):
        raise ValueError("timestamp_ns must be strictly increasing across the trace")

    raw_delta = np.abs(np.asarray([row["raw_delta_lerobot"] for row in rows]))
    applied_delta = np.abs(np.asarray([row["applied_delta_lerobot"] for row in rows]))
    boundaries = np.asarray([row["chunk_boundary"] for row in rows], dtype=bool)
    tracking = np.abs(
        np.asarray([row["command_positions_rad"] for row in rows])
        - np.asarray([row["simulation_positions_rad"] for row in rows])
    )
    episode_frames = Counter(int(row["episode_index"]) for row in rows)
    return {
        "path": str(path),
        "contract_id": rows[0]["contract_id"],
        "joint_names": rows[0]["joint_names"],
        "action_smoothing": rows[0]["action_smoothing"],
        "frames": len(rows),
        "episodes": len(episode_frames),
        "episode_frames": dict(sorted(episode_frames.items())),
        "successes_inferred_from_early_termination": sum(
            count < episode_length for count in episode_frames.values()
        ),
        "raw_abs_delta_lerobot": _quantiles(raw_delta),
        "applied_abs_delta_lerobot": _quantiles(applied_delta),
        "chunk_boundary_raw_abs_delta_lerobot": _quantiles(raw_delta[boundaries]),
        "chunk_boundary_applied_abs_delta_lerobot": _quantiles(applied_delta[boundaries]),
        "command_sim_abs_error_rad": _quantiles(tracking),
        "slew_limited_axis_events": np.sum(
            np.asarray([row["slew_limited_axes"] for row in rows], dtype=bool), axis=0
        ).tolist(),
        "gripper_deadband_frames": sum(row["gripper_deadband_applied"] for row in rows),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", nargs="+", type=Path)
    parser.add_argument("--episode-length", type=int, default=700)
    args = parser.parse_args()
    if args.episode_length <= 0:
        parser.error("--episode-length must be positive")
    print(
        json.dumps(
            [analyze_trace(path, episode_length=args.episode_length) for path in args.trace],
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
