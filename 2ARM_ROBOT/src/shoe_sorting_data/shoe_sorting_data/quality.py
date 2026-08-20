"""Deterministic quality gates for shoe-sorting episode ingestion."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

from shoe_sorting_data.camera_payload import CameraPayloadError, verify_camera_payload
from shoe_sorting_data.contract import load_manifest


@dataclass(frozen=True)
class QualityIssue:
    code: str
    severity: str
    message: str
    sample_index: int | None = None


@dataclass
class ValidationReport:
    episode_id: str
    manifest_path: str
    sample_count: int = 0
    duration_ns: int = 0
    issues: list[QualityIssue] = field(default_factory=list)

    @property
    def errors(self) -> list[QualityIssue]:
        return [issue for issue in self.issues if issue.severity == "error"]

    @property
    def warnings(self) -> list[QualityIssue]:
        return [issue for issue in self.issues if issue.severity == "warning"]

    @property
    def usable(self) -> bool:
        return not self.errors

    def add(self, code: str, message: str, sample_index: int | None = None, severity: str = "error") -> None:
        self.issues.append(QualityIssue(code, severity, message, sample_index))

    def to_dict(self) -> dict[str, Any]:
        return {
            "episode_id": self.episode_id,
            "manifest_path": self.manifest_path,
            "sample_count": self.sample_count,
            "duration_ns": self.duration_ns,
            "usable": self.usable,
            "error_count": len(self.errors),
            "warning_count": len(self.warnings),
            "issues": [asdict(issue) for issue in self.issues],
        }


def _is_number(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _check_vector(
    report: ValidationReport,
    values: Any,
    *,
    dimension: int,
    label: str,
    sample_index: int,
) -> list[float] | None:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
        report.add("stream_not_vector", f"{label} must be an array", sample_index)
        return None
    if len(values) != dimension:
        report.add(
            "stream_dimension_mismatch",
            f"{label} expected {dimension} values but found {len(values)}",
            sample_index,
        )
        return None
    if not all(_is_number(value) for value in values):
        report.add("stream_non_finite", f"{label} contains a non-finite number", sample_index)
        return None
    return [float(value) for value in values]


def _load_samples(path: Path, report: ValidationReport) -> list[Mapping[str, Any]]:
    samples: list[Mapping[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    report.add("blank_sample_line", f"blank JSONL line {line_number}", line_number - 1)
                    continue
                try:
                    value = json.loads(line)
                except json.JSONDecodeError as error:
                    report.add("invalid_sample_json", f"line {line_number}: {error}", line_number - 1)
                    continue
                if not isinstance(value, Mapping):
                    report.add("sample_not_object", f"line {line_number} is not an object", line_number - 1)
                    continue
                samples.append(value)
    except FileNotFoundError:
        report.add("sample_file_missing", f"sample file not found: {path}")
    return samples


def validate_episode(manifest_path: str | Path) -> ValidationReport:
    """Run hard quality gates and return a machine-readable report."""
    path = Path(manifest_path)
    try:
        manifest = load_manifest(path)
    except ValueError as error:
        report = ValidationReport(path.parent.name, str(path))
        report.add("manifest_invalid", str(error))
        return report

    report = ValidationReport(manifest["episode_id"], str(path))
    recording = manifest["recording"]
    sample_path = path.parent / recording["sample_file"]
    samples = _load_samples(sample_path, report)
    report.sample_count = len(samples)
    if len(samples) != recording["sample_count"]:
        report.add(
            "sample_count_mismatch",
            f"manifest declares {recording['sample_count']} samples but file contains {len(samples)}",
        )

    if sample_path.exists():
        actual_digest = hashlib.sha256(sample_path.read_bytes()).hexdigest()
        expected_digest = manifest["checksums"]["samples_sha256"].lower()
        if actual_digest != expected_digest:
            report.add("checksum_mismatch", "samples.jsonl SHA-256 does not match manifest")

    previous_timestamp: int | None = None
    previous_camera_frames: dict[str, int] = {}
    previous_camera_timestamps: dict[str, int] = {}
    previous_joint_state: dict[str, list[float]] = {}
    state_specs = recording["state_streams"]
    action_specs = recording["action_streams"]
    camera_names = recording["camera_streams"]
    camera_payload_config = recording.get("camera_payload", {})
    camera_payload_required = (
        isinstance(camera_payload_config, Mapping) and camera_payload_config.get("mode") == "required"
    )
    synchronized_stream_names = {
        "left_joint_state",
        "right_joint_state",
        "left_joint_action",
        "right_joint_action",
        "base_velocity",
        "base_command",
        "workspace_rgb",
        "workspace_depth",
    }
    limits = manifest["quality_limits"]

    if manifest["outcome"]["status"] != "accepted":
        report.add("outcome_not_accepted", "episode must be accepted before training use")
    if manifest["provenance"]["data_origin"] != "synthetic":
        for field_name in ("calibration_version", "robot_config_version"):
            version = manifest["robot"][field_name].strip().lower()
            if version.startswith("pending") or version in {"unknown", "none", "n/a"}:
                report.add(
                    "hardware_version_unresolved",
                    f"robot.{field_name} must be resolved for recorded robot data",
                )

    for index, sample in enumerate(samples):
        timestamp = sample.get("timestamp_ns")
        if isinstance(timestamp, bool) or not isinstance(timestamp, int):
            report.add("timestamp_invalid", "timestamp_ns must be an integer", index)
            timestamp = None
        elif previous_timestamp is not None:
            if timestamp <= previous_timestamp:
                report.add("timestamp_not_monotonic", "timestamp_ns must increase strictly", index)
            elif timestamp - previous_timestamp > recording["expected_period_ns"] * 1.5:
                report.add("sample_gap_exceeded", "sample timestamp gap exceeds 1.5x expected period", index)
        if timestamp is not None:
            previous_timestamp = timestamp

        if camera_payload_required:
            timing = sample.get("timing")
            if not isinstance(timing, Mapping):
                report.add("timing_group_missing", "required payload samples need timing metadata", index)
            else:
                if timing.get("anchor_timestamp_ns") != timestamp:
                    report.add("timing_anchor_mismatch", "timing anchor must equal sample timestamp", index)
                stream_timestamps = timing.get("stream_timestamps_ns")
                received_timestamps = timing.get("stream_received_monotonic_ns")
                if not isinstance(stream_timestamps, Mapping) or set(stream_timestamps) != synchronized_stream_names:
                    report.add("stream_timing_invalid", "stream header timestamp map is incomplete", index)
                elif not all(
                    isinstance(value, int) and not isinstance(value, bool) and value >= 0
                    for value in stream_timestamps.values()
                ):
                    report.add("stream_timing_invalid", "stream header timestamps must be non-negative integers", index)
                else:
                    actual_delta = max(stream_timestamps.values()) - min(stream_timestamps.values())
                    if timing.get("sync_delta_ns") != actual_delta:
                        report.add("sync_delta_mismatch", "sync_delta_ns does not match stream timestamps", index)
                    if actual_delta > limits["max_camera_skew_ns"]:
                        report.add("sync_delta_exceeded", "synchronized stream delta exceeds tolerance", index)
                if not isinstance(received_timestamps, Mapping) or set(received_timestamps) != synchronized_stream_names:
                    report.add("stream_receive_timing_invalid", "stream receive timestamp map is incomplete", index)
                elif not all(
                    isinstance(value, int) and not isinstance(value, bool) and value >= 0
                    for value in received_timestamps.values()
                ):
                    report.add(
                        "stream_receive_timing_invalid",
                        "stream receive timestamps must be non-negative integers",
                        index,
                    )

        for group_name, specs in (("state", state_specs), ("action", action_specs)):
            group = sample.get(group_name)
            if not isinstance(group, Mapping):
                report.add("stream_group_missing", f"{group_name} must be an object", index)
                continue
            unexpected = sorted(set(group) - set(specs))
            if unexpected:
                report.add("unexpected_stream", f"{group_name} has unexpected streams: {unexpected}", index)
            for stream_name, spec in specs.items():
                vector = _check_vector(
                    report,
                    group.get(stream_name),
                    dimension=spec["dimension"],
                    label=f"{group_name}.{stream_name}",
                    sample_index=index,
                )
                if group_name == "state" and stream_name in {"left_arm", "right_arm"} and vector is not None:
                    previous = previous_joint_state.get(stream_name)
                    if previous is not None and any(
                        abs(current - old) > limits["max_joint_step_radian"]
                        for current, old in zip(vector, previous)
                    ):
                        report.add(
                            "joint_step_exceeded",
                            f"state.{stream_name} exceeds max_joint_step_radian",
                            index,
                        )
                    previous_joint_state[stream_name] = vector

        if not manifest["task"]["base_motion_allowed"]:
            if "base_stationary_tolerance" in limits:
                base_tolerances = [
                    limits["base_stationary_tolerance"],
                    limits["base_stationary_tolerance"],
                ]
            else:
                base_tolerances = [
                    limits["base_linear_stationary_tolerance_mps"],
                    limits["base_angular_stationary_tolerance_radps"],
                ]
            for group_name in ("state", "action"):
                group = sample.get(group_name)
                if isinstance(group, Mapping):
                    base = group.get("base_velocity")
                else:
                    base = None
                if isinstance(base, Sequence) and not isinstance(base, (str, bytes)) and all(
                    _is_number(value) for value in base
                ):
                    if len(base) == 2 and any(
                        abs(float(value)) > float(tolerance)
                        for value, tolerance in zip(base, base_tolerances)
                    ):
                        report.add(
                            "base_interlock_violation",
                            f"{group_name}.base_velocity is non-zero during stationary manipulation",
                            index,
                        )

        cameras = sample.get("cameras")
        if not isinstance(cameras, Mapping):
            report.add("camera_group_missing", "cameras must be an object", index)
            continue
        for camera_name in camera_names:
            camera = cameras.get(camera_name)
            if not isinstance(camera, Mapping):
                report.add("camera_missing", f"camera {camera_name} is missing", index)
                continue
            if camera.get("valid") is not True:
                report.add("camera_invalid", f"camera {camera_name} frame is invalid", index)
            if camera_payload_required:
                received_monotonic_ns = camera.get("received_monotonic_ns")
                if (
                    isinstance(received_monotonic_ns, bool)
                    or not isinstance(received_monotonic_ns, int)
                    or received_monotonic_ns < 0
                ):
                    report.add(
                        "camera_receive_timestamp_invalid",
                        f"camera {camera_name} receive timestamp is invalid",
                        index,
                    )
            camera_timestamp = camera.get("timestamp_ns")
            if (
                timestamp is not None
                and not isinstance(camera_timestamp, bool)
                and isinstance(camera_timestamp, int)
                and abs(camera_timestamp - timestamp) > limits["max_camera_skew_ns"]
            ):
                report.add("camera_skew_exceeded", f"camera {camera_name} exceeds max skew", index)
            elif isinstance(camera_timestamp, bool) or not isinstance(camera_timestamp, int):
                report.add("camera_timestamp_invalid", f"camera {camera_name} timestamp is invalid", index)
            else:
                previous_camera_timestamp = previous_camera_timestamps.get(camera_name)
                if previous_camera_timestamp is not None and camera_timestamp <= previous_camera_timestamp:
                    report.add(
                        "camera_timestamp_not_monotonic",
                        f"camera {camera_name} timestamp must increase strictly",
                        index,
                    )
                previous_camera_timestamps[camera_name] = camera_timestamp
            frame_id = camera.get("frame_id")
            if isinstance(frame_id, bool) or not isinstance(frame_id, int):
                report.add("camera_frame_id_invalid", f"camera {camera_name} frame_id is invalid", index)
            else:
                previous_frame = previous_camera_frames.get(camera_name)
                if previous_frame is not None and frame_id <= previous_frame:
                    report.add("camera_frame_not_monotonic", f"camera {camera_name} frame_id must increase", index)
                elif previous_frame is not None and frame_id != previous_frame + 1:
                    report.add("camera_frame_gap", f"camera {camera_name} skipped a frame_id", index)
                previous_camera_frames[camera_name] = frame_id
            payload_metadata = camera.get("payload")
            if payload_metadata is None:
                if camera_payload_required:
                    report.add(
                        "camera_payload_missing",
                        f"camera {camera_name} requires a pixel payload",
                        index,
                    )
            elif not isinstance(payload_metadata, Mapping):
                report.add(
                    "camera_payload_metadata_invalid",
                    f"camera {camera_name} payload must be an object",
                    index,
                )
            else:
                try:
                    verify_camera_payload(path.parent, camera_name, payload_metadata)
                except CameraPayloadError as error:
                    report.add(error.code, f"camera {camera_name}: {error}", index)

    valid_timestamps = [
        sample.get("timestamp_ns")
        for sample in samples
        if isinstance(sample.get("timestamp_ns"), int) and not isinstance(sample.get("timestamp_ns"), bool)
    ]
    if len(valid_timestamps) >= 2:
        report.duration_ns = max(valid_timestamps) - min(valid_timestamps)
    return report
