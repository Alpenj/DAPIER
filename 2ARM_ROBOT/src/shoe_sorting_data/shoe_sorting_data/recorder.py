"""Approximate-time recorder adapter for the Phase 0 episode contract."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Sequence

from shoe_sorting_data.contract import build_manifest, save_manifest
from shoe_sorting_data.quality import ValidationReport, validate_episode


JOINT_STREAMS = {
    "left_joint_state",
    "right_joint_state",
    "left_joint_action",
    "right_joint_action",
}
BASE_STREAMS = {"base_velocity", "base_command"}
CAMERA_STREAMS = {"workspace_rgb", "workspace_depth"}
REQUIRED_STREAMS = JOINT_STREAMS | BASE_STREAMS | CAMERA_STREAMS


@dataclass(frozen=True)
class StampedStream:
    timestamp_ns: int
    values: tuple[float, ...] = ()
    frame_id: int | None = None
    valid: bool = True


def _numeric_vector(values: Sequence[float], *, dimension: int, stream_name: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)) or len(values) != dimension:
        raise ValueError(f"{stream_name} requires {dimension} numeric values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{stream_name} contains a non-finite value")
    return result


class ApproximateEpisodeRecorder:
    """Collect complete fresh topic sets and persist the stable Phase 0 contract."""

    def __init__(
        self,
        episode_dir: str | Path,
        *,
        arm_dof: int = 5,
        gripper_dof: int = 1,
        max_sync_skew_ns: int = 50_000_000,
        source_split: str = "train",
    ) -> None:
        if arm_dof <= 0 or gripper_dof <= 0:
            raise ValueError("arm_dof and gripper_dof must be positive")
        if max_sync_skew_ns < 0:
            raise ValueError("max_sync_skew_ns must be non-negative")
        self.episode_dir = Path(episode_dir)
        self.arm_dof = arm_dof
        self.gripper_dof = gripper_dof
        self.max_sync_skew_ns = max_sync_skew_ns
        self.source_split = source_split
        self._latest: dict[str, StampedStream] = {}
        self._consumed_timestamps: dict[str, int] = {}
        self._samples: list[dict[str, object]] = []
        self._finalized = False
        self._assert_output_available()

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def _assert_output_available(self) -> None:
        if self.episode_dir.exists() and any(self.episode_dir.iterdir()):
            raise ValueError(f"episode output directory is not empty: {self.episode_dir}")

    def update(
        self,
        stream_name: str,
        *,
        timestamp_ns: int,
        values: Sequence[float] = (),
        frame_id: int | None = None,
        valid: bool = True,
    ) -> bool:
        """Update one topic and return true only when a synchronized sample is emitted."""
        if self._finalized:
            raise ValueError("recorder is already finalized")
        if stream_name not in REQUIRED_STREAMS:
            raise ValueError(f"unsupported recorder stream: {stream_name}")
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int) or timestamp_ns < 0:
            raise ValueError("timestamp_ns must be a non-negative integer")

        if stream_name in JOINT_STREAMS:
            packet_values = _numeric_vector(
                values,
                dimension=self.arm_dof + self.gripper_dof,
                stream_name=stream_name,
            )
        elif stream_name in BASE_STREAMS:
            packet_values = _numeric_vector(values, dimension=2, stream_name=stream_name)
        else:
            if frame_id is None or isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id < 0:
                raise ValueError(f"{stream_name} requires a non-negative integer frame_id")
            packet_values = ()

        self._latest[stream_name] = StampedStream(
            timestamp_ns=timestamp_ns,
            values=packet_values,
            frame_id=frame_id,
            valid=valid,
        )
        return self._try_emit()

    def _try_emit(self) -> bool:
        if set(self._latest) != REQUIRED_STREAMS:
            return False
        if any(
            packet.timestamp_ns <= self._consumed_timestamps.get(name, -1)
            for name, packet in self._latest.items()
        ):
            return False

        timestamps = [packet.timestamp_ns for packet in self._latest.values()]
        if max(timestamps) - min(timestamps) > self.max_sync_skew_ns:
            return False

        joint_dimension = self.arm_dof + self.gripper_dof

        def split(name: str) -> tuple[list[float], list[float]]:
            values = self._latest[name].values
            if len(values) != joint_dimension:
                raise ValueError(f"{name} dimension changed while recording")
            return list(values[: self.arm_dof]), list(values[self.arm_dof :])

        left_state, left_gripper_state = split("left_joint_state")
        right_state, right_gripper_state = split("right_joint_state")
        left_action, left_gripper_action = split("left_joint_action")
        right_action, right_gripper_action = split("right_joint_action")
        rgb = self._latest["workspace_rgb"]
        depth = self._latest["workspace_depth"]
        sample = {
            "timestamp_ns": max(timestamps),
            "state": {
                "left_arm": left_state,
                "left_gripper": left_gripper_state,
                "right_arm": right_state,
                "right_gripper": right_gripper_state,
                "base_velocity": list(self._latest["base_velocity"].values),
            },
            "action": {
                "left_arm": left_action,
                "left_gripper": left_gripper_action,
                "right_arm": right_action,
                "right_gripper": right_gripper_action,
                "base_velocity": list(self._latest["base_command"].values),
            },
            "cameras": {
                "workspace_rgb": {
                    "timestamp_ns": rgb.timestamp_ns,
                    "frame_id": rgb.frame_id,
                    "valid": rgb.valid,
                },
                "workspace_depth": {
                    "timestamp_ns": depth.timestamp_ns,
                    "frame_id": depth.frame_id,
                    "valid": depth.valid,
                },
            },
        }
        self._samples.append(sample)
        self._consumed_timestamps = {
            name: packet.timestamp_ns for name, packet in self._latest.items()
        }
        return True

    def finalize(
        self,
        *,
        outcome_status: str,
        failure_reason: str | None = None,
    ) -> tuple[Path, ValidationReport]:
        """Write one episode without replacing existing output, then run quality gates."""
        if self._finalized:
            raise ValueError("recorder is already finalized")
        if self.sample_count < 2:
            raise ValueError("at least two synchronized samples are required")
        self._assert_output_available()
        self.episode_dir.mkdir(parents=True, exist_ok=True)
        payload = "".join(
            json.dumps(sample, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n"
            for sample in self._samples
        ).encode("utf-8")
        samples_path = self.episode_dir / "samples.jsonl"
        samples_path.write_bytes(payload)
        digest = hashlib.sha256(payload).hexdigest()
        manifest = build_manifest(
            episode_id=self.episode_dir.name,
            sample_count=self.sample_count,
            samples_sha256=digest,
            arm_dof=self.arm_dof,
            gripper_dof=self.gripper_dof,
            operator_id="mock_ros_recorder",
            session_id=f"mock_session_{self.episode_dir.name}",
            shoe_pair_id="mock_pair",
            source_split=self.source_split,
            outcome_status=outcome_status,
            success=True if outcome_status == "accepted" else False if outcome_status == "aborted" else None,
            failure_reason=failure_reason,
            synthetic=True,
            object_instance_id=f"mock_object_{self.episode_dir.name}",
            background_id="mock_background_v1",
            fixture_id="mock_ros_topics_v1",
            recording_span_id=f"mock_span_{self.episode_dir.name}",
            attempt_id=f"mock_attempt_{self.episode_dir.name}",
        )
        manifest_path = self.episode_dir / "episode_manifest.json"
        save_manifest(manifest_path, manifest)
        self._finalized = True
        return manifest_path, validate_episode(manifest_path)
