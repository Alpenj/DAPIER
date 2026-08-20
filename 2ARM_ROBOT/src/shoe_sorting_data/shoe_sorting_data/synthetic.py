"""Deterministic golden episodes for development before hardware exists."""

from __future__ import annotations

import hashlib
import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
import random
from typing import Any

from shoe_sorting_data.contract import build_manifest, save_manifest


FAULTS = {
    "none",
    "base_motion",
    "camera_frame_gap",
    "camera_skew",
    "checksum_mismatch",
    "dimension_mismatch",
    "duplicate_timestamp",
    "joint_jump",
    "missing_camera",
    "sample_gap",
}


def _vector(rng: random.Random, dimension: int, center: float) -> list[float]:
    return [round(center + rng.uniform(-0.004, 0.004), 6) for _ in range(dimension)]


def _make_samples(*, sample_count: int, arm_dof: int, gripper_dof: int, seed: int) -> list[dict[str, Any]]:
    rng = random.Random(seed)
    samples: list[dict[str, Any]] = []
    start_ns = 1_000_000_000
    period_ns = 50_000_000
    for index in range(sample_count):
        center = index * 0.005
        timestamp_ns = start_ns + index * period_ns
        state = {
            "left_arm": _vector(rng, arm_dof, center),
            "left_gripper": _vector(rng, gripper_dof, 0.5),
            "right_arm": _vector(rng, arm_dof, -center),
            "right_gripper": _vector(rng, gripper_dof, 0.5),
            "base_velocity": [0.0, 0.0],
        }
        action = {
            "left_arm": _vector(rng, arm_dof, center + 0.003),
            "left_gripper": _vector(rng, gripper_dof, 0.5),
            "right_arm": _vector(rng, arm_dof, -center - 0.003),
            "right_gripper": _vector(rng, gripper_dof, 0.5),
            "base_velocity": [0.0, 0.0],
        }
        samples.append(
            {
                "timestamp_ns": timestamp_ns,
                "state": state,
                "action": action,
                "cameras": {
                    "workspace_rgb": {
                        "timestamp_ns": timestamp_ns - 2_000_000,
                        "frame_id": index,
                        "valid": True,
                    },
                    "workspace_depth": {
                        "timestamp_ns": timestamp_ns + 3_000_000,
                        "frame_id": index,
                        "valid": True,
                    },
                },
            }
        )
    return samples


def _inject_fault(samples: list[dict[str, Any]], fault: str) -> None:
    if fault == "none" or fault == "checksum_mismatch":
        return
    index = len(samples) // 2
    sample = samples[index]
    if fault == "base_motion":
        sample["state"]["base_velocity"] = [0.03, 0.0]
        sample["action"]["base_velocity"] = [0.05, 0.0]
    elif fault == "camera_frame_gap":
        sample["cameras"]["workspace_rgb"]["frame_id"] += 1
    elif fault == "camera_skew":
        sample["cameras"]["workspace_depth"]["timestamp_ns"] += 100_000_000
    elif fault == "dimension_mismatch":
        sample["action"]["left_arm"] = sample["action"]["left_arm"][:-1]
    elif fault == "duplicate_timestamp":
        sample["timestamp_ns"] = samples[index - 1]["timestamp_ns"]
    elif fault == "joint_jump":
        sample["state"]["left_arm"][0] += 1.0
    elif fault == "missing_camera":
        del sample["cameras"]["workspace_depth"]
    elif fault == "sample_gap":
        sample["timestamp_ns"] += 30_000_000
    else:
        raise ValueError(f"unsupported fault: {fault}")


def _write_samples(path: Path, samples: list[dict[str, Any]]) -> str:
    payload = "".join(
        json.dumps(sample, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n"
        for sample in samples
    ).encode("utf-8")
    path.write_bytes(payload)
    return hashlib.sha256(payload).hexdigest()


def generate_episode(
    episode_dir: str | Path,
    *,
    sample_count: int = 40,
    arm_dof: int = 6,
    gripper_dof: int = 1,
    seed: int = 42,
    fault: str = "none",
    source_split: str = "train",
) -> Path:
    """Generate one deterministic episode and return its manifest path."""
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2")
    if arm_dof <= 0 or gripper_dof <= 0:
        raise ValueError("arm_dof and gripper_dof must be positive")
    if fault not in FAULTS:
        raise ValueError(f"fault must be one of {sorted(FAULTS)}")

    root = Path(episode_dir)
    root.mkdir(parents=True, exist_ok=True)
    samples = _make_samples(
        sample_count=sample_count,
        arm_dof=arm_dof,
        gripper_dof=gripper_dof,
        seed=seed,
    )
    _inject_fault(samples, fault)
    digest = _write_samples(root / "samples.jsonl", samples)
    manifest_digest = "0" * 64 if fault == "checksum_mismatch" else digest
    manifest = build_manifest(
        episode_id=root.name,
        sample_count=sample_count,
        samples_sha256=manifest_digest,
        arm_dof=arm_dof,
        gripper_dof=gripper_dof,
        operator_id="synthetic_generator",
        session_id=f"synthetic_seed_{seed}",
        shoe_pair_id=f"synthetic_pair_{seed % 30:02d}",
        source_split=source_split,
        outcome_status="accepted",
        success=True,
        synthetic=True,
        created_at_utc=(datetime(2026, 1, 1, tzinfo=timezone.utc) + timedelta(seconds=seed))
        .isoformat()
        .replace("+00:00", "Z"),
        object_instance_id=f"synthetic_object_{seed:06d}",
        background_id=f"synthetic_background_{seed % 3:02d}",
        fixture_id="synthetic_grid_v1",
        recording_span_id=f"synthetic_span_{seed:06d}",
        attempt_id=f"synthetic_attempt_{seed:06d}",
    )
    manifest_path = root / "episode_manifest.json"
    save_manifest(manifest_path, manifest)
    return manifest_path


def generate_dataset(
    root: str | Path,
    *,
    count: int = 20,
    sample_count: int = 40,
    arm_dof: int = 6,
    gripper_dof: int = 1,
    seed: int = 42,
    fault: str = "none",
) -> list[Path]:
    if count <= 0:
        raise ValueError("count must be positive")
    dataset_root = Path(root)
    paths = []
    for index in range(count):
        split = "validation" if index % 5 == 4 else "train"
        paths.append(
            generate_episode(
                dataset_root / f"episode_{index + 1:06d}",
                sample_count=sample_count,
                arm_dof=arm_dof,
                gripper_dof=gripper_dof,
                seed=seed + index,
                fault=fault,
                source_split=split,
            )
        )
    return paths
