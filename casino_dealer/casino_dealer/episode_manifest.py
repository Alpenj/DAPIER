"""Create and validate sidecar metadata for recorded CardBench episodes.

The ROS 2 recorder or LeRobot owns the actual sensor/action data.  This
module owns the small, human-editable record that says what was recorded,
which calibrated arms produced it, and whether the episode is safe to use for
training.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


EPISODE_MANIFEST_SCHEMA_VERSION = "dapier.episode-manifest.v0"
VALID_SOURCES = ("lerobot", "rosbag2_mcap")
VALID_STATUSES = ("recorded", "accepted", "rejected")


def parse_arm_spec(spec: str) -> dict[str, str]:
    """Parse ``name,follower_id,leader_id`` from the CLI."""
    parts = [part.strip() for part in spec.split(",")]
    if len(parts) != 3 or any(not part for part in parts):
        raise ValueError(
            "arm spec must have the form name,follower_id,leader_id"
        )
    name, follower_id, leader_id = parts
    return {
        "name": name,
        "follower_id": follower_id,
        "leader_id": leader_id,
    }


def build_manifest(
    *,
    episode_id: str,
    task: str,
    skill: str,
    source: str,
    fps: float,
    cameras: Sequence[str],
    arms: Sequence[Mapping[str, str]],
    calibration_refs: Sequence[str] = (),
    data_path: str = "",
    notes: str = "",
) -> dict[str, Any]:
    """Build a new manifest in the stable JSON representation."""
    manifest = {
        "schema_version": EPISODE_MANIFEST_SCHEMA_VERSION,
        "episode_id": episode_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "task": {
            "name": task,
            "skill": skill,
        },
        "robot": {
            "arms": [dict(arm) for arm in arms],
            "calibration_refs": list(calibration_refs),
        },
        "recording": {
            "source": source,
            "fps": fps,
            "cameras": list(cameras),
            "data_path": data_path,
        },
        "outcome": {
            "status": "recorded",
            "success": None,
            "failure_reason": "",
        },
        "notes": notes,
    }
    validate_manifest(manifest)
    return manifest


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    """Raise ``ValueError`` when a manifest is incomplete or inconsistent."""
    required = {
        "schema_version",
        "episode_id",
        "created_at",
        "task",
        "robot",
        "recording",
        "outcome",
        "notes",
    }
    missing = sorted(required - set(manifest))
    if missing:
        raise ValueError(f"manifest is missing keys: {missing}")

    if manifest["schema_version"] != EPISODE_MANIFEST_SCHEMA_VERSION:
        raise ValueError(
            "schema_version must be "
            f"{EPISODE_MANIFEST_SCHEMA_VERSION!r}"
        )
    _require_text(manifest, "episode_id")
    _require_text(manifest, "created_at")
    _require_text(manifest, "notes", allow_empty=True)

    task = _require_mapping(manifest, "task")
    _require_text(task, "name")
    _require_text(task, "skill")

    robot = _require_mapping(manifest, "robot")
    arms = robot.get("arms")
    if not isinstance(arms, list) or not arms:
        raise ValueError("robot.arms must be a non-empty list")
    arm_names: set[str] = set()
    for index, arm in enumerate(arms):
        if not isinstance(arm, Mapping):
            raise ValueError(f"robot.arms[{index}] must be an object")
        for key in ("name", "follower_id", "leader_id"):
            _require_text(arm, key)
        name = str(arm["name"])
        if name in arm_names:
            raise ValueError(f"duplicate arm name: {name}")
        arm_names.add(name)

    calibration_refs = robot.get("calibration_refs", [])
    if not isinstance(calibration_refs, list) or any(
        not isinstance(ref, str) or not ref for ref in calibration_refs
    ):
        raise ValueError("robot.calibration_refs must be a list of strings")

    recording = _require_mapping(manifest, "recording")
    source = recording.get("source")
    if source not in VALID_SOURCES:
        raise ValueError(f"recording.source must be one of {VALID_SOURCES}")
    fps = recording.get("fps")
    if isinstance(fps, bool) or not isinstance(fps, (int, float)) or fps <= 0:
        raise ValueError("recording.fps must be a positive number")
    cameras = recording.get("cameras")
    if not isinstance(cameras, list) or any(
        not isinstance(camera, str) or not camera for camera in cameras
    ):
        raise ValueError("recording.cameras must be a list of strings")
    _require_text(recording, "data_path", allow_empty=True)

    outcome = _require_mapping(manifest, "outcome")
    status = outcome.get("status")
    if status not in VALID_STATUSES:
        raise ValueError(f"outcome.status must be one of {VALID_STATUSES}")
    success = outcome.get("success")
    if success is not None and not isinstance(success, bool):
        raise ValueError("outcome.success must be true, false, or null")
    failure_reason = outcome.get("failure_reason")
    if not isinstance(failure_reason, str):
        raise ValueError("outcome.failure_reason must be a string")
    if status in ("accepted", "rejected") and success is None:
        raise ValueError(
            "outcome.success must be set before an episode is accepted or rejected"
        )
    if success is False and not failure_reason.strip():
        raise ValueError("a failed episode requires outcome.failure_reason")


def load_manifest(path: str | Path) -> dict[str, Any]:
    """Read and validate one manifest file."""
    manifest_path = Path(path)
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"manifest not found: {manifest_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON in {manifest_path}: {error}") from error
    if not isinstance(manifest, Mapping):
        raise ValueError("manifest top level must be an object")
    validate_manifest(manifest)
    return dict(manifest)


def save_manifest(path: str | Path, manifest: Mapping[str, Any]) -> None:
    """Validate and write one manifest with stable formatting."""
    validate_manifest(manifest)
    manifest_path = Path(path)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _require_mapping(parent: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = parent.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"{key} must be an object")
    return value


def _require_text(
    parent: Mapping[str, Any],
    key: str,
    *,
    allow_empty: bool = False,
) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or (not allow_empty and not value.strip()):
        raise ValueError(f"{key} must be a non-empty string")
    return value
