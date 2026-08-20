"""Stable JSON contract for one synchronized dual-arm shoe episode."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

from shoe_sorting_data.camera_payload import CAMERA_PAYLOAD_CONTRACT_VERSION, CAMERA_PAYLOAD_MODES

EPISODE_SCHEMA_VERSION = "dapier.shoe-episode.v0.3"
SUPPORTED_EPISODE_SCHEMA_VERSIONS = {
    "dapier.shoe-episode.v0.1",
    "dapier.shoe-episode.v0.2",
    EPISODE_SCHEMA_VERSION,
}
OUTCOME_STATUSES = {"recorded", "accepted", "rejected", "aborted"}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _stream(dimension: int, unit: str) -> dict[str, Any]:
    return {"dimension": dimension, "unit": unit}


def build_manifest(
    *,
    episode_id: str,
    sample_count: int,
    samples_sha256: str,
    arm_dof: int = 5,
    gripper_dof: int = 1,
    operator_id: str = "operator_unknown",
    session_id: str = "session_unknown",
    shoe_pair_id: str = "pair_unknown",
    source_split: str = "train",
    outcome_status: str = "recorded",
    success: bool | None = None,
    failure_reason: str | None = None,
    synthetic: bool = False,
    created_at_utc: str | None = None,
    object_instance_id: str = "object_unknown",
    background_id: str = "background_unknown",
    fixture_id: str = "fixture_unknown",
    recording_span_id: str = "span_unknown",
    attempt_id: str = "attempt_unknown",
    camera_payload_mode: str | None = None,
) -> dict[str, Any]:
    """Build one manifest; dimensions are explicit until hardware introspection."""
    state_streams = {
        "left_arm": _stream(arm_dof, "radian"),
        "left_gripper": _stream(gripper_dof, "normalized_position"),
        "right_arm": _stream(arm_dof, "radian"),
        "right_gripper": _stream(gripper_dof, "normalized_position"),
        "base_velocity": _stream(2, "meter_per_second,radian_per_second"),
    }
    action_streams = deepcopy(state_streams)
    resolved_camera_payload_mode = camera_payload_mode or ("metadata_only" if synthetic else "required")
    if resolved_camera_payload_mode not in CAMERA_PAYLOAD_MODES:
        raise ValueError(f"camera_payload_mode must be one of {sorted(CAMERA_PAYLOAD_MODES)}")
    manifest = {
        "schema_version": EPISODE_SCHEMA_VERSION,
        "episode_id": episode_id,
        "created_at_utc": created_at_utc or _utc_now(),
        "task": {
            "name": "shoe_sorting",
            "skill": "pair_and_place",
            "shoe_pair_id": shoe_pair_id,
            "language_instruction": "Match the same shoes and place them side by side.",
            "base_motion_allowed": False,
        },
        "robot": {
            "platform": "JDcobot200_dual_arm_on_turtlebot3_waffle_pi",
            "robot_config_version": "pending_hardware_introspection",
            "controller_version": "phase0_no_hardware",
            "calibration_version": "synthetic_v1" if synthetic else "pending_calibration",
        },
        "recording": {
            "sample_file": "samples.jsonl",
            "sample_count": sample_count,
            "clock": "episode_monotonic_ns",
            "expected_period_ns": 50_000_000,
            "camera_streams": ["workspace_rgb", "workspace_depth"],
            "camera_payload": {
                "contract_version": CAMERA_PAYLOAD_CONTRACT_VERSION,
                "mode": resolved_camera_payload_mode,
                "storage": "ros2_raw_rows",
            },
            "state_streams": state_streams,
            "action_streams": action_streams,
        },
        "quality_limits": {
            "max_camera_skew_ns": 50_000_000,
            "max_joint_step_radian": 0.35,
            "base_linear_stationary_tolerance_mps": 0.0025,
            "base_angular_stationary_tolerance_radps": 0.0021,
        },
        "outcome": {
            "status": outcome_status,
            "success": success,
            "failure_reason": failure_reason,
        },
        "provenance": {
            "operator_id": operator_id,
            "session_id": session_id,
            "pipeline_version": "shoe_data_phase0_v0.1",
            "source_split": source_split,
            "data_origin": "synthetic" if synthetic else "robot",
            "object_instance_id": object_instance_id,
            "background_id": background_id,
            "fixture_id": fixture_id,
            "recording_span_id": recording_span_id,
            "attempt_id": attempt_id,
        },
        "checksums": {"samples_sha256": samples_sha256},
        "lifecycle": {
            "state": "finalized",
            "integrity_verified": True,
        },
    }
    validate_manifest(manifest)
    return manifest


def _require_mapping(parent: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = parent.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"{key} must be an object")
    return value


def _require_text(parent: Mapping[str, Any], key: str, *, allow_empty: bool = False) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or (not allow_empty and not value.strip()):
        raise ValueError(f"{key} must be a non-empty string")
    return value


def _validate_stream_specs(specs: Mapping[str, Any], key: str) -> None:
    required = {"left_arm", "left_gripper", "right_arm", "right_gripper", "base_velocity"}
    if set(specs) != required:
        raise ValueError(f"{key} must contain exactly {sorted(required)}")
    for name, raw_spec in specs.items():
        if not isinstance(raw_spec, Mapping):
            raise ValueError(f"{key}.{name} must be an object")
        dimension = raw_spec.get("dimension")
        if isinstance(dimension, bool) or not isinstance(dimension, int) or dimension <= 0:
            raise ValueError(f"{key}.{name}.dimension must be a positive integer")
        _require_text(raw_spec, "unit")


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    """Raise ``ValueError`` when metadata is incomplete or inconsistent."""
    required = {
        "schema_version",
        "episode_id",
        "created_at_utc",
        "task",
        "robot",
        "recording",
        "quality_limits",
        "outcome",
        "provenance",
        "checksums",
    }
    if manifest.get("schema_version") == EPISODE_SCHEMA_VERSION:
        required.add("lifecycle")
    missing = sorted(required - set(manifest))
    if missing:
        raise ValueError(f"manifest is missing keys: {missing}")
    if manifest["schema_version"] not in SUPPORTED_EPISODE_SCHEMA_VERSIONS:
        raise ValueError(f"unsupported schema_version: {manifest['schema_version']!r}")
    _require_text(manifest, "episode_id")
    _require_text(manifest, "created_at_utc")

    task = _require_mapping(manifest, "task")
    for key in ("name", "skill", "shoe_pair_id", "language_instruction"):
        _require_text(task, key)
    if not isinstance(task.get("base_motion_allowed"), bool):
        raise ValueError("task.base_motion_allowed must be boolean")

    robot = _require_mapping(manifest, "robot")
    for key in ("platform", "robot_config_version", "controller_version", "calibration_version"):
        _require_text(robot, key)

    recording = _require_mapping(manifest, "recording")
    _require_text(recording, "sample_file")
    _require_text(recording, "clock")
    sample_count = recording.get("sample_count")
    if isinstance(sample_count, bool) or not isinstance(sample_count, int) or sample_count <= 0:
        raise ValueError("recording.sample_count must be a positive integer")
    expected_period = recording.get("expected_period_ns")
    if isinstance(expected_period, bool) or not isinstance(expected_period, int) or expected_period <= 0:
        raise ValueError("recording.expected_period_ns must be a positive integer")
    cameras = recording.get("camera_streams")
    if not isinstance(cameras, Sequence) or isinstance(cameras, (str, bytes)):
        raise ValueError("recording.camera_streams must be an array")
    if not all(isinstance(name, str) for name in cameras):
        raise ValueError("recording.camera_streams entries must be strings")
    if set(cameras) != {"workspace_rgb", "workspace_depth"}:
        raise ValueError("recording.camera_streams must contain workspace_rgb and workspace_depth")
    if manifest["schema_version"] == EPISODE_SCHEMA_VERSION:
        camera_payload = _require_mapping(recording, "camera_payload")
        if camera_payload.get("contract_version") != CAMERA_PAYLOAD_CONTRACT_VERSION:
            raise ValueError("recording.camera_payload.contract_version is unsupported")
        if camera_payload.get("mode") not in CAMERA_PAYLOAD_MODES:
            raise ValueError(f"recording.camera_payload.mode must be one of {sorted(CAMERA_PAYLOAD_MODES)}")
        if camera_payload.get("storage") != "ros2_raw_rows":
            raise ValueError("recording.camera_payload.storage must be ros2_raw_rows")
    _validate_stream_specs(_require_mapping(recording, "state_streams"), "state_streams")
    _validate_stream_specs(_require_mapping(recording, "action_streams"), "action_streams")

    limits = _require_mapping(manifest, "quality_limits")
    limit_keys = ["max_camera_skew_ns", "max_joint_step_radian"]
    if manifest["schema_version"] == "dapier.shoe-episode.v0.1":
        limit_keys.append("base_stationary_tolerance")
    else:
        limit_keys.extend(
            [
                "base_linear_stationary_tolerance_mps",
                "base_angular_stationary_tolerance_radps",
            ]
        )
    for key in limit_keys:
        value = limits.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
            raise ValueError(f"quality_limits.{key} must be a non-negative number")

    outcome = _require_mapping(manifest, "outcome")
    status = outcome.get("status")
    if status not in OUTCOME_STATUSES:
        raise ValueError(f"outcome.status must be one of {sorted(OUTCOME_STATUSES)}")
    success_value = outcome.get("success")
    if success_value is not None and not isinstance(success_value, bool):
        raise ValueError("outcome.success must be boolean or null")
    reason = outcome.get("failure_reason")
    if reason is not None and (not isinstance(reason, str) or not reason.strip()):
        raise ValueError("outcome.failure_reason must be null or a non-empty string")
    if status in {"rejected", "aborted"} and not reason:
        raise ValueError("rejected or aborted episodes require outcome.failure_reason")
    if status == "accepted" and success_value is not True:
        raise ValueError("accepted episodes require outcome.success=true")

    provenance = _require_mapping(manifest, "provenance")
    for key in ("operator_id", "session_id", "pipeline_version", "source_split", "data_origin"):
        _require_text(provenance, key)
    for key in ("object_instance_id", "background_id", "fixture_id", "recording_span_id", "attempt_id"):
        if key in provenance:
            _require_text(provenance, key)

    checksums = _require_mapping(manifest, "checksums")
    digest = _require_text(checksums, "samples_sha256")
    if len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest.lower()):
        raise ValueError("checksums.samples_sha256 must be a 64-character hexadecimal digest")

    if manifest["schema_version"] == EPISODE_SCHEMA_VERSION:
        lifecycle = _require_mapping(manifest, "lifecycle")
        if lifecycle.get("state") != "finalized":
            raise ValueError("lifecycle.state must be finalized")
        if lifecycle.get("integrity_verified") is not True:
            raise ValueError("lifecycle.integrity_verified must be true")


def load_manifest(path: str | Path) -> dict[str, Any]:
    manifest_path = Path(path)
    try:
        value = json.loads(manifest_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"manifest not found: {manifest_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON in {manifest_path}: {error}") from error
    if not isinstance(value, Mapping):
        raise ValueError("manifest top level must be an object")
    validate_manifest(value)
    return dict(value)


def save_manifest(path: str | Path, manifest: Mapping[str, Any]) -> None:
    validate_manifest(manifest)
    manifest_path = Path(path)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
