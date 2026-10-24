"""Dependency-free offline evaluator for ACT action chunks.

The evaluator deliberately separates padding-excluded imitation error from
closed-loop task success and safety interventions, which require Stage 5 logs.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping


OFFLINE_EVAL_INPUT_SCHEMA_VERSION = "dapier.act-offline-eval-input.v0.1"
OFFLINE_EVAL_REPORT_SCHEMA_VERSION = "dapier.act-offline-eval-report.v0.1"
ACTION_NAMES = (
    *(f"left_arm_{index}" for index in range(5)),
    "left_gripper",
    *(f"right_arm_{index}" for index in range(5)),
    "right_gripper",
)
ACTION_UNITS = (*("radian" for _ in range(5)), "normalized_position", *("radian" for _ in range(5)), "normalized_position")
GROUP_PREFIXES = {
    "left_arm": ("left_arm_",),
    "left_gripper": ("left_gripper",),
    "right_arm": ("right_arm_",),
    "right_gripper": ("right_gripper",),
    "all_arm": ("left_arm_", "right_arm_"),
    "all_gripper": ("left_gripper", "right_gripper"),
}
SPLIT_GROUP_KEYS = ("object_set_id", "scene_id", "session_id", "calibration_id")


class OfflineEvaluationError(ValueError):
    pass


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, value: Mapping[str, Any], *, overwrite: bool = False) -> None:
    if path.exists() and not overwrite:
        raise OfflineEvaluationError(f"refusing to overwrite existing evaluator artifact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    if path.exists():
        raise OfflineEvaluationError(f"refusing to overwrite existing evaluator artifact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = "".join(
        json.dumps(row, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n"
        for row in rows
    )
    path.write_text(payload, encoding="utf-8")


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise OfflineEvaluationError(f"invalid JSON file {path}: {error}") from error
    if not isinstance(value, dict):
        raise OfflineEvaluationError(f"JSON root must be an object: {path}")
    return value


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
    except OSError as error:
        raise OfflineEvaluationError(f"cannot read prediction records {path}: {error}") from error
    for line_number, line in enumerate(lines, start=1):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as error:
            raise OfflineEvaluationError(f"invalid JSON at {path}:{line_number}: {error}") from error
        if not isinstance(row, dict):
            raise OfflineEvaluationError(f"record at {path}:{line_number} must be an object")
        rows.append(row)
    if not rows:
        raise OfflineEvaluationError("prediction records must not be empty")
    return rows


def _require_sha(value: Any, field: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise OfflineEvaluationError(f"{field} must be a lowercase SHA-256 hex digest")
    return value


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise OfflineEvaluationError(f"{field} must be a finite number")
    return float(value)


def _validate_split(split: Mapping[str, Any]) -> dict[str, list[str]]:
    if split.get("source_split") not in {"validation", "test"}:
        raise OfflineEvaluationError("offline evaluation source_split must be validation or test")
    evaluation_episodes = split.get("evaluation_episode_ids")
    train_episodes = split.get("train_episode_ids")
    if not isinstance(evaluation_episodes, list) or not evaluation_episodes:
        raise OfflineEvaluationError("split.evaluation_episode_ids must be a non-empty list")
    if not isinstance(train_episodes, list):
        raise OfflineEvaluationError("split.train_episode_ids must be a list")
    episode_overlap = sorted(set(evaluation_episodes) & set(train_episodes))
    if episode_overlap:
        raise OfflineEvaluationError(f"train/evaluation episode overlap: {episode_overlap}")
    if split.get("normalization_stats_source_split") != "train":
        raise OfflineEvaluationError("normalization stats must be computed from the train split only")

    overlaps: dict[str, list[str]] = {}
    train_groups = split.get("train_groups")
    evaluation_groups = split.get("evaluation_groups")
    if not isinstance(train_groups, dict) or not isinstance(evaluation_groups, dict):
        raise OfflineEvaluationError("split train_groups/evaluation_groups must be objects")
    for key in SPLIT_GROUP_KEYS:
        train_values = train_groups.get(key)
        evaluation_values = evaluation_groups.get(key)
        if not isinstance(train_values, list) or not isinstance(evaluation_values, list):
            raise OfflineEvaluationError(f"split group {key} must be represented by lists")
        overlaps[key] = sorted(set(train_values) & set(evaluation_values))
    if split.get("generalization_claim") is True and any(overlaps.values()):
        raise OfflineEvaluationError(f"generalization_claim=true with group overlap: {overlaps}")
    return overlaps


def _validate_action_contract(contract: Mapping[str, Any]) -> tuple[int, int, list[float]]:
    names = contract.get("names")
    units = contract.get("units")
    if names != list(ACTION_NAMES):
        raise OfflineEvaluationError("action names/order differ from the JDcobot 12-DoF contract")
    if units != list(ACTION_UNITS):
        raise OfflineEvaluationError("action units differ from the JDcobot radian/gripper contract")
    if contract.get("convention") != "absolute_joint_target":
        raise OfflineEvaluationError("only absolute_joint_target evaluation is supported in the ACT baseline")
    chunk_size = contract.get("chunk_size")
    fps = contract.get("fps")
    if not isinstance(chunk_size, int) or chunk_size <= 0:
        raise OfflineEvaluationError("action_contract.chunk_size must be a positive integer")
    if not isinstance(fps, int) or fps <= 0:
        raise OfflineEvaluationError("action_contract.fps must be a positive integer")
    expected_delta = [index / fps for index in range(chunk_size)]
    actual_delta = contract.get("delta_timestamps_seconds")
    if actual_delta != expected_delta:
        raise OfflineEvaluationError(
            f"delta timestamps differ from chunk_size/FPS: expected={expected_delta}, actual={actual_delta}"
        )
    return chunk_size, fps, expected_delta


def _group_indices(names: list[str]) -> dict[str, list[int]]:
    result: dict[str, list[int]] = {}
    for group, prefixes in GROUP_PREFIXES.items():
        indices = [
            index
            for index, name in enumerate(names)
            if any(name == prefix or name.startswith(prefix) for prefix in prefixes)
        ]
        if not indices:
            raise OfflineEvaluationError(f"action group has no matching features: {group}")
        units = {ACTION_UNITS[index] for index in indices}
        if len(units) != 1:
            raise OfflineEvaluationError(f"action group mixes physical units: {group}={sorted(units)}")
        result[group] = indices
    return result


def _metric(sum_abs: float, sum_sq: float, max_abs: float, count: int) -> dict[str, Any]:
    if count == 0:
        return {"count": 0, "mae": None, "rmse": None, "max_abs": None}
    return {
        "count": count,
        "mae": sum_abs / count,
        "rmse": math.sqrt(sum_sq / count),
        "max_abs": max_abs,
    }


def evaluate_action_chunks(
    evaluation_manifest_path: str | Path,
    output_path: str | Path,
    *,
    inspection_top_k: int = 3,
) -> dict[str, Any]:
    """Evaluate prediction chunks while excluding every padded timestep."""

    manifest_path = Path(evaluation_manifest_path).resolve()
    output = Path(output_path).resolve()
    if not isinstance(inspection_top_k, int) or inspection_top_k <= 0:
        raise OfflineEvaluationError("inspection_top_k must be a positive integer")
    if output.exists():
        raise OfflineEvaluationError(f"refusing to overwrite existing evaluator artifact: {output}")
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != OFFLINE_EVAL_INPUT_SCHEMA_VERSION:
        raise OfflineEvaluationError("unsupported offline evaluation input schema")
    split = manifest.get("split")
    action_contract = manifest.get("action_contract")
    provenance = manifest.get("provenance")
    if not isinstance(split, dict) or not isinstance(action_contract, dict) or not isinstance(provenance, dict):
        raise OfflineEvaluationError("manifest split, action_contract, and provenance must be objects")
    overlaps = _validate_split(split)
    chunk_size, fps, expected_delta = _validate_action_contract(action_contract)
    for field in (
        "policy_checkpoint_sha256",
        "train_manifest_sha256",
        "normalization_stats_sha256",
        "hardware_profile_sha256",
        "code_sha256",
    ):
        _require_sha(provenance.get(field), f"provenance.{field}")

    records_name = manifest.get("records_file")
    if not isinstance(records_name, str) or Path(records_name).is_absolute() or ".." in Path(records_name).parts:
        raise OfflineEvaluationError("records_file must be a safe relative path")
    records_path = (manifest_path.parent / records_name).resolve()
    expected_records_sha = _require_sha(manifest.get("records_sha256"), "records_sha256")
    if _sha256(records_path) != expected_records_sha:
        raise OfflineEvaluationError("prediction records SHA-256 mismatch")
    rows = _load_jsonl(records_path)
    evaluation_episode_ids = set(split["evaluation_episode_ids"])
    names = list(action_contract["names"])
    groups = _group_indices(names)

    joint_acc = [
        [{"sum_abs": 0.0, "sum_sq": 0.0, "max_abs": 0.0, "count": 0} for _ in names]
        for _ in range(chunk_size)
    ]
    group_acc = {
        group: {"sum_abs": 0.0, "sum_sq": 0.0, "max_abs": 0.0, "count": 0}
        for group in groups
    }
    horizon_valid = [0 for _ in range(chunk_size)]
    masked_timestep_count = 0
    seen: set[tuple[str, int]] = set()
    per_group_candidates: dict[str, list[dict[str, Any]]] = {group: [] for group in groups}

    for row_index, row in enumerate(rows):
        episode_id = row.get("episode_id")
        frame_index = row.get("frame_index")
        if episode_id not in evaluation_episode_ids:
            raise OfflineEvaluationError(f"record {row_index} episode is not in the evaluation split: {episode_id}")
        if not isinstance(frame_index, int) or frame_index < 0:
            raise OfflineEvaluationError(f"record {row_index} frame_index must be a non-negative integer")
        identity = (episode_id, frame_index)
        if identity in seen:
            raise OfflineEvaluationError(f"duplicate evaluation record: {identity}")
        seen.add(identity)
        target = row.get("target_action")
        prediction = row.get("predicted_action")
        mask = row.get("action_is_pad")
        target_episode_ids = row.get("target_episode_ids")
        if (
            not isinstance(target, list)
            or not isinstance(prediction, list)
            or not isinstance(mask, list)
            or not isinstance(target_episode_ids, list)
        ):
            raise OfflineEvaluationError(f"record {row_index} action/mask fields must be lists")
        if (
            len(target) != chunk_size
            or len(prediction) != chunk_size
            or len(mask) != chunk_size
            or len(target_episode_ids) != chunk_size
        ):
            raise OfflineEvaluationError(f"record {row_index} chunk/mask length mismatch")
        if any(target_episode_id != episode_id for target_episode_id in target_episode_ids):
            raise OfflineEvaluationError(f"record {row_index} action window crosses an episode boundary")
        if any(not isinstance(value, bool) for value in mask):
            raise OfflineEvaluationError(f"record {row_index} action_is_pad must contain booleans")
        first_padding = next((index for index, value in enumerate(mask) if value), chunk_size)
        if any(not value for value in mask[first_padding:]):
            raise OfflineEvaluationError(f"record {row_index} padding mask must be a contiguous tail")
        if first_padding == 0:
            raise OfflineEvaluationError(f"record {row_index} must contain the current valid action")

        for horizon in range(chunk_size):
            if not isinstance(target[horizon], list) or not isinstance(prediction[horizon], list):
                raise OfflineEvaluationError(f"record {row_index} horizon {horizon} actions must be lists")
            if len(target[horizon]) != len(names) or len(prediction[horizon]) != len(names):
                raise OfflineEvaluationError(f"record {row_index} horizon {horizon} action dimension mismatch")
            target_values = [
                _finite_number(value, f"record {row_index} target[{horizon}][{joint}]")
                for joint, value in enumerate(target[horizon])
            ]
            predicted_values = [
                _finite_number(value, f"record {row_index} prediction[{horizon}][{joint}]")
                for joint, value in enumerate(prediction[horizon])
            ]
            if mask[horizon]:
                masked_timestep_count += 1
                continue
            horizon_valid[horizon] += 1
            absolute_errors = [abs(predicted_values[joint] - target_values[joint]) for joint in range(len(names))]
            for joint, error in enumerate(absolute_errors):
                acc = joint_acc[horizon][joint]
                acc["sum_abs"] += error
                acc["sum_sq"] += error * error
                acc["max_abs"] = max(acc["max_abs"], error)
                acc["count"] += 1
            for group, indices in groups.items():
                errors = [absolute_errors[index] for index in indices]
                acc = group_acc[group]
                acc["sum_abs"] += sum(errors)
                acc["sum_sq"] += sum(error * error for error in errors)
                acc["max_abs"] = max(acc["max_abs"], max(errors))
                acc["count"] += len(errors)
                per_group_candidates[group].append(
                    {
                        "episode_id": episode_id,
                        "frame_index": frame_index,
                        "horizon": horizon,
                        "mean_abs": sum(errors) / len(errors),
                        "max_abs": max(errors),
                        "calibration_id": row.get("calibration_id"),
                        "scene_id": row.get("scene_id"),
                        "object_set_id": row.get("object_set_id"),
                        "rgb_reference": row.get("rgb_reference"),
                        "depth_available": bool(row.get("depth_available", False)),
                    }
                )

    horizons = []
    for horizon in range(chunk_size):
        per_joint = {}
        for joint, name in enumerate(names):
            acc = joint_acc[horizon][joint]
            per_joint[name] = {
                "unit": ACTION_UNITS[joint],
                **_metric(acc["sum_abs"], acc["sum_sq"], acc["max_abs"], acc["count"]),
            }
        horizons.append(
            {
                "horizon": horizon,
                "delta_seconds": expected_delta[horizon],
                "valid_step_count": horizon_valid[horizon],
                "coverage": horizon_valid[horizon] / len(rows),
                "per_joint": per_joint,
            }
        )

    group_metrics = {}
    inspection_packet = {}
    for group, indices in groups.items():
        acc = group_acc[group]
        group_metrics[group] = {
            "unit": ACTION_UNITS[indices[0]],
            **_metric(acc["sum_abs"], acc["sum_sq"], acc["max_abs"], acc["count"]),
        }
        inspection_packet[group] = sorted(
            per_group_candidates[group],
            key=lambda item: (item["mean_abs"], item["max_abs"]),
            reverse=True,
        )[:inspection_top_k]

    report = {
        "schema_version": OFFLINE_EVAL_REPORT_SCHEMA_VERSION,
        "created_at_utc": _utc_now(),
        "status": "PASS",
        "scope": "held-out padding-excluded action imitation diagnostics; not closed-loop task success",
        "input": {
            "manifest_path": str(manifest_path),
            "manifest_sha256": _sha256(manifest_path),
            "records_path": str(records_path),
            "records_sha256": expected_records_sha,
            "record_count": len(rows),
        },
        "split_audit": {
            "source_split": split["source_split"],
            "split_version": split.get("split_version"),
            "split_seed": split.get("split_seed"),
            "generalization_claim": bool(split.get("generalization_claim", False)),
            "group_overlaps": overlaps,
            "train_only_normalization_stats": True,
        },
        "action_contract": {
            "names": names,
            "units": list(action_contract["units"]),
            "convention": action_contract["convention"],
            "chunk_size": chunk_size,
            "fps": fps,
            "delta_timestamps_seconds": expected_delta,
        },
        "mask_accounting": {
            "valid_timestep_count": sum(horizon_valid),
            "masked_timestep_count": masked_timestep_count,
            "masked_scalar_count": masked_timestep_count * len(names),
        },
        "horizon_metrics": horizons,
        "group_metrics": group_metrics,
        "global_mixed_unit_metric": None,
        "global_metric_reason": "arm radians and gripper normalized positions must not be averaged into one score",
        "inspection_packet": inspection_packet,
        "failure_taxonomy": [
            "data_contract",
            "split_or_calibration_leakage",
            "action_tail_padding",
            "single_joint_or_gripper",
            "bimanual_coordination",
            "perception_or_calibration",
            "off_distribution",
            "safety_or_intervention",
        ],
        "closed_loop_metrics": {
            "status": "NOT_MEASURED",
            "required_stage": "Stage 5 independent safety supervisor and real rollout",
            "fields": ["task_success_rate", "intervention_rate", "reject_rate", "staleness", "near_limit_margin"],
        },
        "provenance": provenance,
        "provenance_note": "hash identifiers are recorded; the producing pipeline must verify the referenced artifacts",
    }
    _write_json(output, report)
    return report


def build_offline_evaluator_fixture(
    root: str | Path,
    *,
    padded_prediction: float = 999.0,
    valid_arm_error: float = 0.1,
    valid_gripper_error: float = 0.2,
) -> Path:
    """Create a deterministic 2 episode x 3 frame evaluator fixture."""

    fixture_root = Path(root).resolve()
    if fixture_root.exists():
        raise OfflineEvaluationError(f"fixture root already exists: {fixture_root}")
    fixture_root.mkdir(parents=True)
    rows = []
    masks = ([False, False, False], [False, False, True], [False, True, True])
    episode_ids = ("eval_episode_001", "eval_episode_002")
    for episode_index, episode_id in enumerate(episode_ids):
        for frame_index, mask in enumerate(masks):
            target = []
            prediction = []
            for horizon in range(3):
                target_row = [episode_index + frame_index * 0.01 + horizon * 0.001 for _ in ACTION_NAMES]
                if mask[horizon]:
                    prediction_row = [padded_prediction for _ in ACTION_NAMES]
                else:
                    prediction_row = [
                        value + (valid_gripper_error if "gripper" in ACTION_NAMES[joint] else valid_arm_error)
                        for joint, value in enumerate(target_row)
                    ]
                target.append(target_row)
                prediction.append(prediction_row)
            rows.append(
                {
                    "episode_id": episode_id,
                    "frame_index": frame_index,
                    "target_action": target,
                    "predicted_action": prediction,
                    "action_is_pad": list(mask),
                    "target_episode_ids": [episode_id, episode_id, episode_id],
                    "object_set_id": f"eval_shoe_set_{episode_index}",
                    "scene_id": f"eval_scene_{episode_index}",
                    "session_id": f"eval_session_{episode_index}",
                    "calibration_id": f"eval_calibration_{episode_index}",
                    "rgb_reference": f"raw/{episode_id}/workspace_rgb/{frame_index:06d}.raw",
                    "depth_available": True,
                }
            )
    records_path = fixture_root / "predictions.jsonl"
    _write_jsonl(records_path, rows)
    placeholder_sha = hashlib.sha256(b"dapier-stage4-fixture").hexdigest()
    manifest = {
        "schema_version": OFFLINE_EVAL_INPUT_SCHEMA_VERSION,
        "created_at_utc": _utc_now(),
        "records_file": records_path.name,
        "records_sha256": _sha256(records_path),
        "split": {
            "source_split": "validation",
            "split_version": "fixture.v1",
            "split_seed": 0,
            "evaluation_episode_ids": list(episode_ids),
            "train_episode_ids": ["train_episode_001", "train_episode_002"],
            "normalization_stats_source_split": "train",
            "generalization_claim": False,
            "train_groups": {
                key: [f"train_{key}_0", f"train_{key}_1"] for key in SPLIT_GROUP_KEYS
            },
            "evaluation_groups": {
                key: [f"eval_{key}_0", f"eval_{key}_1"] for key in SPLIT_GROUP_KEYS
            },
        },
        "action_contract": {
            "names": list(ACTION_NAMES),
            "units": list(ACTION_UNITS),
            "convention": "absolute_joint_target",
            "chunk_size": 3,
            "fps": 20,
            "delta_timestamps_seconds": [0.0, 0.05, 0.1],
        },
        "provenance": {
            "policy_checkpoint_sha256": placeholder_sha,
            "train_manifest_sha256": placeholder_sha,
            "normalization_stats_sha256": placeholder_sha,
            "hardware_profile_sha256": placeholder_sha,
            "code_sha256": placeholder_sha,
            "synthetic_fixture_only": True,
        },
    }
    manifest_path = fixture_root / "evaluation_manifest.json"
    _write_json(manifest_path, manifest)
    return manifest_path
