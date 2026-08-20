"""Fail-closed ACT interchange for validated DAPIER shoe episodes.

This module deliberately stops before native LeRobot Dataset v3 encoding.  It
proves the numeric feature, split, provenance, and integrity contracts without
claiming image-conditioned ACT readiness while the recorder has no pixel data.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
from typing import Any, Iterable, Mapping, Sequence
from uuid import uuid4

from shoe_sorting_data.contract import load_manifest
from shoe_sorting_data.quality import validate_episode


ACT_INTERCHANGE_SCHEMA_VERSION = "dapier.act-interchange.v0.1"
DEFAULT_LEROBOT_COMMIT = "d451fe4f1f1b00a812f95aa9534389b5e42ab155"
POLICY_STREAM_ORDER = ("left_arm", "left_gripper", "right_arm", "right_gripper")
LEAKAGE_IDENTITY_KEYS = ("object_instance_id", "session_id", "recording_span_id")
SUPPORTED_SPLITS = {"train", "validation", "test"}


@dataclass(frozen=True)
class ActInterchangeReport:
    output_root: str
    episode_count: int
    frame_count: int
    split_counts: dict[str, int]
    state_dim: int
    action_dim: int
    act_numeric_contract_ready: bool
    native_lerobot_ready: bool
    blockers: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "output_root": self.output_root,
            "episode_count": self.episode_count,
            "frame_count": self.frame_count,
            "split_counts": self.split_counts,
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "act_numeric_contract_ready": self.act_numeric_contract_ready,
            "native_lerobot_ready": self.native_lerobot_ready,
            "blockers": list(self.blockers),
        }


class _VectorStats:
    def __init__(self, dimension: int) -> None:
        self.dimension = dimension
        self.count = 0
        self.mean = [0.0] * dimension
        self.m2 = [0.0] * dimension
        self.minimum = [math.inf] * dimension
        self.maximum = [-math.inf] * dimension

    def update(self, values: Sequence[float]) -> None:
        if len(values) != self.dimension:
            raise ValueError(f"statistics expected dimension {self.dimension}, received {len(values)}")
        self.count += 1
        for index, raw_value in enumerate(values):
            value = float(raw_value)
            delta = value - self.mean[index]
            self.mean[index] += delta / self.count
            self.m2[index] += delta * (value - self.mean[index])
            self.minimum[index] = min(self.minimum[index], value)
            self.maximum[index] = max(self.maximum[index], value)

    def to_dict(self) -> dict[str, Any]:
        if self.count == 0:
            raise ValueError("cannot finalize empty training statistics")
        return {
            "count": self.count,
            "mean": self.mean,
            "std": [math.sqrt(value / self.count) for value in self.m2],
            "min": self.minimum,
            "max": self.maximum,
        }


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n")


def _discover_manifests(root: Path) -> list[Path]:
    if root.is_file():
        manifests = [root] if root.name == "episode_manifest.json" else []
    else:
        manifests = sorted(root.rglob("episode_manifest.json"))
    if not manifests:
        raise ValueError(f"no episode_manifest.json files found below: {root}")
    return [path.resolve() for path in manifests]


def _load_samples(manifest_path: Path, manifest: Mapping[str, Any]) -> tuple[Path, list[dict[str, Any]]]:
    sample_path = (manifest_path.parent / manifest["recording"]["sample_file"]).resolve()
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(sample_path.read_text(encoding="utf-8").splitlines(), start=1):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(f"invalid JSON at {sample_path}:{line_number}: {error}") from error
        if not isinstance(row, dict):
            raise ValueError(f"sample at {sample_path}:{line_number} must be an object")
        rows.append(row)
    return sample_path, rows


def _feature_names(stream_specs: Mapping[str, Mapping[str, Any]], prefix: str) -> list[str]:
    names: list[str] = []
    for stream_name in POLICY_STREAM_ORDER:
        dimension = int(stream_specs[stream_name]["dimension"])
        names.extend(f"{prefix}.{stream_name}.{index}" for index in range(dimension))
    return names


def _feature_units(stream_specs: Mapping[str, Mapping[str, Any]]) -> list[str]:
    units: list[str] = []
    for stream_name in POLICY_STREAM_ORDER:
        units.extend([str(stream_specs[stream_name]["unit"])] * int(stream_specs[stream_name]["dimension"]))
    return units


def _flatten(sample: Mapping[str, Any], group_name: str) -> list[float]:
    group = sample[group_name]
    values: list[float] = []
    for stream_name in POLICY_STREAM_ORDER:
        values.extend(float(value) for value in group[stream_name])
    return values


def _camera_has_pixels(camera: Mapping[str, Any], episode_dir: Path) -> bool:
    payload = camera.get("payload")
    if isinstance(payload, Mapping):
        value = payload.get("path")
        if isinstance(value, str) and value.strip() and (episode_dir / value).is_file():
            return True
    for key in ("path", "image_path", "payload_path"):
        value = camera.get(key)
        if isinstance(value, str) and value.strip() and (episode_dir / value).is_file():
            return True
    return False


def _audit_split_leakage(episodes: Sequence[Mapping[str, Any]]) -> None:
    identities: dict[tuple[str, str], set[str]] = {}
    for episode in episodes:
        split = episode["split"]
        provenance = episode["manifest"]["provenance"]
        for key in LEAKAGE_IDENTITY_KEYS:
            value = str(provenance[key])
            if value.endswith("_unknown"):
                raise ValueError(f"cannot prove split isolation with unknown provenance: {key}={value}")
            identities.setdefault((key, value), set()).add(split)
    violations = [
        f"{key}={value} spans splits {sorted(splits)}"
        for (key, value), splits in sorted(identities.items())
        if len(splits) > 1
    ]
    if violations:
        raise ValueError("split leakage detected: " + "; ".join(violations))


def _relative_output_hashes(root: Path) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): _sha256(path)
        for path in sorted(root.rglob("*"))
        if path.is_file() and path.name != "conversion_receipt.json"
    }


def export_act_interchange(
    source_root: str | Path,
    output_root: str | Path,
    *,
    lerobot_commit: str = DEFAULT_LEROBOT_COMMIT,
) -> ActInterchangeReport:
    """Export immutable, accepted episodes into a verified ACT interchange.

    The output is not a native LeRobot dataset.  It is a deterministic boundary
    artifact that can be converted with LeRobot only after camera pixels exist.
    """

    source = Path(source_root).resolve()
    output = Path(output_root).resolve()
    if output.exists():
        raise ValueError(f"output path already exists; choose a new versioned path: {output}")

    manifests = _discover_manifests(source)
    for manifest_path in manifests:
        try:
            output.relative_to(manifest_path.parent)
        except ValueError:
            continue
        raise ValueError("output path must not be inside a source episode directory")

    episodes: list[dict[str, Any]] = []
    source_hashes_before: dict[str, str] = {}
    episode_ids: set[str] = set()
    expected_period_ns: int | None = None
    state_names: list[str] | None = None
    action_names: list[str] | None = None
    state_units: list[str] | None = None
    action_units: list[str] | None = None

    for manifest_path in manifests:
        report = validate_episode(manifest_path)
        if not report.usable:
            codes = sorted({issue.code for issue in report.errors})
            raise ValueError(f"episode is not quality accepted: {manifest_path} ({', '.join(codes)})")
        manifest = load_manifest(manifest_path)
        if manifest["outcome"]["status"] != "accepted":
            raise ValueError(f"episode outcome is not accepted: {manifest_path}")
        split = manifest["provenance"]["source_split"]
        if split not in SUPPORTED_SPLITS:
            raise ValueError(f"unsupported source split {split!r} in {manifest_path}")
        episode_id = manifest["episode_id"]
        if episode_id in episode_ids:
            raise ValueError(f"duplicate episode_id: {episode_id}")
        episode_ids.add(episode_id)

        recording = manifest["recording"]
        period = int(recording["expected_period_ns"])
        if expected_period_ns is None:
            expected_period_ns = period
        elif period != expected_period_ns:
            raise ValueError("all episodes must use the same expected_period_ns")

        current_state_names = _feature_names(recording["state_streams"], "observation.state")
        current_action_names = _feature_names(recording["action_streams"], "action")
        current_state_units = _feature_units(recording["state_streams"])
        current_action_units = _feature_units(recording["action_streams"])
        if state_names is None:
            state_names = current_state_names
            action_names = current_action_names
            state_units = current_state_units
            action_units = current_action_units
        elif state_names != current_state_names or action_names != current_action_names:
            raise ValueError("policy feature dimensions changed across episodes")
        elif state_units != current_state_units or action_units != current_action_units:
            raise ValueError("policy feature units changed across episodes")

        sample_path, samples = _load_samples(manifest_path, manifest)
        source_hashes_before[str(manifest_path)] = _sha256(manifest_path)
        source_hashes_before[str(sample_path)] = _sha256(sample_path)
        episodes.append(
            {
                "manifest_path": manifest_path,
                "manifest": manifest,
                "sample_path": sample_path,
                "samples": samples,
                "split": split,
            }
        )

    assert (
        expected_period_ns is not None
        and state_names is not None
        and action_names is not None
        and state_units is not None
        and action_units is not None
    )
    _audit_split_leakage(episodes)
    if 1_000_000_000 % expected_period_ns != 0:
        raise ValueError("expected_period_ns must map to an integer LeRobot fps")
    fps = 1_000_000_000 // expected_period_ns

    state_stats = _VectorStats(len(state_names))
    action_stats = _VectorStats(len(action_names))
    split_counts = {split: 0 for split in sorted(SUPPORTED_SPLITS)}
    split_frame_counts = {split: 0 for split in sorted(SUPPORTED_SPLITS)}
    all_cameras_have_pixels = True
    data_origins: set[str] = set()
    converted: list[tuple[dict[str, Any], list[dict[str, Any]]]] = []

    for episode in episodes:
        manifest = episode["manifest"]
        split = episode["split"]
        start_ns = int(episode["samples"][0]["timestamp_ns"])
        rows: list[dict[str, Any]] = []
        for frame_index, sample in enumerate(episode["samples"]):
            state = _flatten(sample, "state")
            action = _flatten(sample, "action")
            if split == "train":
                state_stats.update(state)
                action_stats.update(action)
            cameras = sample["cameras"]
            pixels_ready = all(
                _camera_has_pixels(cameras[key], episode["manifest_path"].parent)
                for key in ("workspace_rgb", "workspace_depth")
            )
            all_cameras_have_pixels = all_cameras_have_pixels and pixels_ready
            rows.append(
                {
                    "frame_index": frame_index,
                    "timestamp": (int(sample["timestamp_ns"]) - start_ns) / 1_000_000_000,
                    "timestamp_ns": int(sample["timestamp_ns"]),
                    "observation.state": state,
                    "action": action,
                    "task": manifest["task"]["language_instruction"],
                    "camera_metadata": cameras,
                }
            )
        split_counts[split] += 1
        split_frame_counts[split] += len(rows)
        data_origins.add(manifest["provenance"]["data_origin"])
        converted.append((episode, rows))

    if state_stats.count == 0:
        raise ValueError("at least one accepted training frame is required for train-only statistics")

    blockers: list[str] = []
    if not all_cameras_have_pixels:
        blockers.append("camera_pixel_payload_missing")
    if "synthetic" in data_origins:
        blockers.append("synthetic_data_smoke_test_only")
    native_conversion_input_ready = all_cameras_have_pixels and "synthetic" not in data_origins
    blockers.append("native_lerobot_dataset_not_encoded")

    temp_output = output.with_name(f".{output.name}.tmp-{uuid4().hex}")
    output.parent.mkdir(parents=True, exist_ok=True)
    temp_output.mkdir()
    try:
        episode_entries: list[dict[str, Any]] = []
        for episode, rows in converted:
            manifest = episode["manifest"]
            relative_path = Path("episodes") / episode["split"] / f"{manifest['episode_id']}.jsonl"
            _write_jsonl(temp_output / relative_path, rows)
            episode_entries.append(
                {
                    "episode_id": manifest["episode_id"],
                    "split": episode["split"],
                    "frame_count": len(rows),
                    "path": relative_path.as_posix(),
                    "object_instance_id": manifest["provenance"]["object_instance_id"],
                    "session_id": manifest["provenance"]["session_id"],
                    "recording_span_id": manifest["provenance"]["recording_span_id"],
                    "source_manifest_sha256": source_hashes_before[str(episode["manifest_path"])],
                    "source_samples_sha256": source_hashes_before[str(episode["sample_path"])],
                }
            )

        _write_json(
            temp_output / "metadata.json",
            {
                "schema_version": ACT_INTERCHANGE_SCHEMA_VERSION,
                "lerobot_upstream_commit": lerobot_commit,
                "fps": fps,
                "policy_scope": "stationary_base_dual_arm_manipulation",
                "excluded_from_policy": ["state.base_velocity", "action.base_velocity"],
                "features": {
                    "observation.state": {
                        "dtype": "float32",
                        "shape": [len(state_names)],
                        "names": state_names,
                        "units": state_units,
                    },
                    "action": {
                        "dtype": "float32",
                        "shape": [len(action_names)],
                        "names": action_names,
                        "units": action_units,
                    },
                    "task": {"dtype": "string", "shape": [1]},
                },
                "camera_contract": {
                    "required": ["workspace_rgb", "workspace_depth"],
                    "pixel_payload_present": all_cameras_have_pixels,
                    "current_interchange_field": "camera_metadata",
                },
            },
        )
        _write_json(
            temp_output / "stats.json",
            {
                "computed_from_split": "train",
                "observation.state": state_stats.to_dict(),
                "action": action_stats.to_dict(),
            },
        )
        _write_json(
            temp_output / "split_manifest.json",
            {
                "leakage_identity_keys": list(LEAKAGE_IDENTITY_KEYS),
                "split_episode_counts": split_counts,
                "split_frame_counts": split_frame_counts,
                "episodes": episode_entries,
            },
        )
        _write_json(
            temp_output / "preflight.json",
            {
                "act_numeric_contract_ready": True,
                "native_conversion_input_ready": native_conversion_input_ready,
                "native_lerobot_ready": False,
                "blockers": blockers,
                "training_claim": "smoke_test_only",
                "checks": {
                    "accepted_quality_gate": True,
                    "consistent_feature_order": True,
                    "integer_fps": True,
                    "source_immutable": True,
                    "split_leakage_free": True,
                    "train_only_stats": True,
                },
            },
        )

        source_hashes_after = {path: _sha256(Path(path)) for path in source_hashes_before}
        if source_hashes_after != source_hashes_before:
            raise RuntimeError("source episode changed during conversion")
        output_hashes = _relative_output_hashes(temp_output)
        _write_json(
            temp_output / "conversion_receipt.json",
            {
                "schema_version": ACT_INTERCHANGE_SCHEMA_VERSION,
                "created_at_utc": _utc_now(),
                "source_root": str(source),
                "source_files_sha256": source_hashes_before,
                "output_files_sha256": output_hashes,
                "episode_count": len(episodes),
                "frame_count": sum(split_frame_counts.values()),
            },
        )
        temp_output.rename(output)
    except Exception:
        if temp_output.exists():
            shutil.rmtree(temp_output)
        raise

    return ActInterchangeReport(
        output_root=str(output),
        episode_count=len(episodes),
        frame_count=sum(split_frame_counts.values()),
        split_counts={key: value for key, value in split_counts.items() if value},
        state_dim=len(state_names),
        action_dim=len(action_names),
        act_numeric_contract_ready=True,
        native_lerobot_ready=False,
        blockers=tuple(blockers),
    )


def verify_act_interchange(root: str | Path) -> dict[str, Any]:
    """Verify every output hash recorded by an ACT interchange receipt."""

    output = Path(root).resolve()
    errors: list[str] = []
    receipt_path = output / "conversion_receipt.json"
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return {"passed": False, "checked_files": 0, "errors": [f"receipt_invalid: {error}"]}

    expected = receipt.get("output_files_sha256")
    if not isinstance(expected, dict):
        return {"passed": False, "checked_files": 0, "errors": ["receipt output hash map is missing"]}
    actual_names = {
        path.relative_to(output).as_posix()
        for path in output.rglob("*")
        if path.is_file() and path.name != "conversion_receipt.json"
    }
    expected_names = set(expected)
    for relative_name in sorted(actual_names - expected_names):
        errors.append(f"unexpected output file: {relative_name}")
    checked = 0
    for relative_name, expected_digest in sorted(expected.items()):
        relative = Path(relative_name)
        if relative.is_absolute() or ".." in relative.parts:
            errors.append(f"unsafe receipt path: {relative_name}")
            continue
        path = output / relative
        if not path.is_file():
            errors.append(f"missing output file: {relative_name}")
            continue
        checked += 1
        actual = _sha256(path)
        if actual != expected_digest:
            errors.append(f"hash mismatch: {relative_name}")
    return {
        "passed": not errors,
        "schema_version": receipt.get("schema_version"),
        "checked_files": checked,
        "errors": errors,
    }
