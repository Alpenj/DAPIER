"""Optional native LeRobot Dataset v3 encoder for finalized DAPIER episodes.

No LeRobot, Torch, Pillow, Datasets, PyArrow, or NumPy import occurs at module
import time.  The ROS recorder and its base tests therefore stay lightweight.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import importlib
import importlib.metadata
import importlib.util
import json
from pathlib import Path
from typing import Any, Mapping, Sequence
from uuid import uuid4

from shoe_sorting_data.camera_payload import CameraFramePayload, read_camera_payload
from shoe_sorting_data.contract import load_manifest
from shoe_sorting_data.quality import validate_episode


NATIVE_ENCODER_SCHEMA_VERSION = "dapier.lerobot-v3-encoder.v0.1"
NATIVE_REQUIRED_MODULES = ("lerobot", "numpy", "PIL", "torch", "datasets", "pyarrow")
POLICY_STREAM_ORDER = ("left_arm", "left_gripper", "right_arm", "right_gripper")


class NativeEncoderDependencyError(RuntimeError):
    pass


@dataclass(frozen=True)
class NativeEncoderPlan:
    source_root: str
    episode_count: int
    frame_count: int
    fps: int
    state_dim: int
    action_dim: int
    rgb_shape_hwc: tuple[int, int, int]
    depth_shape_hwc: tuple[int, int, int]
    depth_unit: str
    source_files_sha256: dict[str, str]
    episodes: tuple[dict[str, Any], ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "source_root": self.source_root,
            "episode_count": self.episode_count,
            "frame_count": self.frame_count,
            "fps": self.fps,
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "rgb_shape_hwc": list(self.rgb_shape_hwc),
            "depth_shape_hwc": list(self.depth_shape_hwc),
            "depth_unit": self.depth_unit,
            "source_files_sha256": self.source_files_sha256,
            "episodes": list(self.episodes),
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


def native_dependency_status() -> dict[str, Any]:
    modules = {name: importlib.util.find_spec(name) is not None for name in NATIVE_REQUIRED_MODULES}
    versions: dict[str, str | None] = {}
    for name, available in modules.items():
        if not available:
            versions[name] = None
            continue
        package_name = "pillow" if name == "PIL" else name
        try:
            versions[name] = importlib.metadata.version(package_name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = "installed-unversioned"
    missing = [name for name, available in modules.items() if not available]
    return {
        "available": not missing,
        "modules": modules,
        "versions": versions,
        "missing": missing,
        "base_recorder_affected": False,
        "install_note": "Install the project-pinned LeRobot dataset environment on Ubuntu; do not add it to the ROS recorder base dependencies.",
    }


def _discover_manifests(root: Path) -> list[Path]:
    if root.is_file() and root.name == "episode_manifest.json":
        paths = [root]
    elif root.is_dir():
        paths = sorted(root.rglob("episode_manifest.json"))
    else:
        paths = []
    if not paths:
        raise ValueError(f"no episode_manifest.json files found below: {root}")
    return [path.resolve() for path in paths]


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


def _flatten(sample: Mapping[str, Any], group_name: str) -> list[float]:
    values: list[float] = []
    group = sample[group_name]
    for stream_name in POLICY_STREAM_ORDER:
        values.extend(float(value) for value in group[stream_name])
    return values


def _payload_shape(payload: CameraFramePayload, *, depth: bool) -> tuple[int, int, int]:
    channels = 1 if depth else 3
    return (payload.height, payload.width, channels)


def build_native_encoder_plan(source_root: str | Path, *, depth_unit: str) -> NativeEncoderPlan:
    """Validate native export inputs without importing optional dependencies."""

    if depth_unit not in {"mm", "m"}:
        raise ValueError("depth_unit must be explicitly set to 'mm' or 'm'")
    source = Path(source_root).resolve()
    manifests = _discover_manifests(source)
    source_hashes: dict[str, str] = {}
    episode_entries: list[dict[str, Any]] = []
    rgb_shape: tuple[int, int, int] | None = None
    depth_shape: tuple[int, int, int] | None = None
    expected_period_ns: int | None = None
    frame_count = 0
    episode_ids: set[str] = set()

    for manifest_path in manifests:
        report = validate_episode(manifest_path)
        if not report.usable:
            codes = sorted({issue.code for issue in report.errors})
            raise ValueError(f"episode is not native-export usable: {manifest_path} ({', '.join(codes)})")
        manifest = load_manifest(manifest_path)
        if manifest.get("lifecycle") != {"state": "finalized", "integrity_verified": True}:
            raise ValueError(f"episode is not finalized and integrity verified: {manifest_path}")
        payload_config = manifest["recording"].get("camera_payload", {})
        if payload_config.get("mode") != "required":
            raise ValueError(f"native export requires RGB-D pixel payloads: {manifest_path}")
        episode_id = manifest["episode_id"]
        if episode_id in episode_ids:
            raise ValueError(f"duplicate episode_id: {episode_id}")
        episode_ids.add(episode_id)
        period = int(manifest["recording"]["expected_period_ns"])
        if expected_period_ns is None:
            expected_period_ns = period
        elif period != expected_period_ns:
            raise ValueError("all native export episodes must use the same expected_period_ns")

        sample_path, samples = _load_samples(manifest_path, manifest)
        source_hashes[str(manifest_path)] = _sha256(manifest_path)
        source_hashes[str(sample_path)] = _sha256(sample_path)
        for sample in samples:
            cameras = sample["cameras"]
            rgb_meta = cameras["workspace_rgb"]["payload"]
            depth_meta = cameras["workspace_depth"]["payload"]
            rgb_payload = read_camera_payload(manifest_path.parent, "workspace_rgb", rgb_meta)
            depth_payload = read_camera_payload(manifest_path.parent, "workspace_depth", depth_meta)
            current_rgb_shape = _payload_shape(rgb_payload, depth=False)
            current_depth_shape = _payload_shape(depth_payload, depth=True)
            if rgb_shape is None:
                rgb_shape = current_rgb_shape
                depth_shape = current_depth_shape
            elif rgb_shape != current_rgb_shape or depth_shape != current_depth_shape:
                raise ValueError("RGB-D payload shapes changed across native export frames")
            for metadata in (rgb_meta, depth_meta):
                raw_path = (manifest_path.parent / metadata["path"]).resolve()
                source_hashes[str(raw_path)] = _sha256(raw_path)
            if len(_flatten(sample, "state")) != 12 or len(_flatten(sample, "action")) != 12:
                raise ValueError("native ACT baseline requires 12-dimensional state and action")
        frame_count += len(samples)
        episode_entries.append(
            {
                "episode_id": episode_id,
                "manifest_path": str(manifest_path),
                "split": manifest["provenance"]["source_split"],
                "frame_count": len(samples),
                "task": manifest["task"]["language_instruction"],
            }
        )

    assert expected_period_ns is not None and rgb_shape is not None and depth_shape is not None
    if 1_000_000_000 % expected_period_ns != 0:
        raise ValueError("expected_period_ns must map to an integer LeRobot fps")
    return NativeEncoderPlan(
        source_root=str(source),
        episode_count=len(episode_entries),
        frame_count=frame_count,
        fps=1_000_000_000 // expected_period_ns,
        state_dim=12,
        action_dim=12,
        rgb_shape_hwc=rgb_shape,
        depth_shape_hwc=depth_shape,
        depth_unit=depth_unit,
        source_files_sha256=source_hashes,
        episodes=tuple(episode_entries),
    )


def _require_native_stack() -> tuple[Any, Any]:
    status = native_dependency_status()
    if not status["available"]:
        raise NativeEncoderDependencyError(
            "native LeRobot Dataset v3 encoder dependencies are missing: " + ", ".join(status["missing"])
        )
    try:
        numpy_module = importlib.import_module("numpy")
        dataset_module = importlib.import_module("lerobot.datasets.lerobot_dataset")
    except Exception as error:
        raise NativeEncoderDependencyError(f"native LeRobot stack import failed: {error}") from error
    return numpy_module, dataset_module.LeRobotDataset


def _decode_rgb(payload: CameraFramePayload, np: Any) -> Any:
    encoding = payload.encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "8uc1": 1}[encoding]
    row_values = payload.width * channels
    rows = np.frombuffer(payload.data, dtype=np.uint8).reshape(payload.height, payload.step)
    image = rows[:, :row_values].reshape(payload.height, payload.width, channels)
    if encoding in {"bgr8", "bgra8"}:
        image = image[..., [2, 1, 0, 3] if channels == 4 else [2, 1, 0]]
    if channels == 4:
        image = image[..., :3]
    elif channels == 1:
        image = np.repeat(image, 3, axis=2)
    return np.ascontiguousarray(image)


def _decode_depth(payload: CameraFramePayload, np: Any) -> Any:
    encoding = payload.encoding.lower()
    dtype_code = {
        "mono16": "u2",
        "16uc1": "u2",
        "16sc1": "i2",
        "32fc1": "f4",
    }[encoding]
    byte_order = ">" if payload.is_bigendian else "<"
    dtype = np.dtype(byte_order + dtype_code)
    bytes_per_pixel = dtype.itemsize
    rows = np.frombuffer(payload.data, dtype=np.uint8).reshape(payload.height, payload.step)
    contiguous = np.ascontiguousarray(rows[:, : payload.width * bytes_per_pixel])
    return contiguous.view(dtype).reshape(payload.height, payload.width, 1)


def _native_features(plan: NativeEncoderPlan) -> dict[str, dict[str, Any]]:
    state_names = [
        *(f"left_arm_{index}" for index in range(5)),
        "left_gripper",
        *(f"right_arm_{index}" for index in range(5)),
        "right_gripper",
    ]
    return {
        "observation.state": {"dtype": "float32", "shape": (12,), "names": state_names},
        "action": {"dtype": "float32", "shape": (12,), "names": state_names},
        "observation.images.workspace_rgb": {
            "dtype": "image",
            "shape": plan.rgb_shape_hwc,
            "names": ["height", "width", "channels"],
        },
        "observation.images.workspace_depth": {
            "dtype": "image",
            "shape": plan.depth_shape_hwc,
            "names": ["height", "width", "channels"],
            "info": {"is_depth_map": True, "depth_unit": plan.depth_unit},
        },
    }


def _output_hashes(root: Path) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): _sha256(path)
        for path in sorted(root.rglob("*"))
        if path.is_file() and path.name != "dapier_encoder_receipt.json"
    }


def encode_native_lerobot_v3(
    source_root: str | Path,
    output_root: str | Path,
    *,
    repo_id: str,
    depth_unit: str,
) -> dict[str, Any]:
    """Create a derived native v3 dataset in an isolated optional environment."""

    plan = build_native_encoder_plan(source_root, depth_unit=depth_unit)
    output = Path(output_root).resolve()
    if output.exists():
        raise ValueError(f"native output path already exists: {output}")
    np, dataset_class = _require_native_stack()
    partial = output.with_name(f"{output.name}.partial-{uuid4().hex}")
    partial.parent.mkdir(parents=True, exist_ok=True)
    features = _native_features(plan)
    dataset = None
    try:
        dataset = dataset_class.create(
            repo_id=repo_id,
            fps=plan.fps,
            root=partial,
            robot_type="JDcobot200_dual_arm_on_turtlebot3_waffle_pi",
            features=features,
            use_videos=False,
        )
        for episode in plan.episodes:
            manifest_path = Path(episode["manifest_path"])
            manifest = load_manifest(manifest_path)
            _sample_path, samples = _load_samples(manifest_path, manifest)
            for sample in samples:
                rgb = read_camera_payload(
                    manifest_path.parent,
                    "workspace_rgb",
                    sample["cameras"]["workspace_rgb"]["payload"],
                )
                depth = read_camera_payload(
                    manifest_path.parent,
                    "workspace_depth",
                    sample["cameras"]["workspace_depth"]["payload"],
                )
                dataset.add_frame(
                    {
                        "observation.state": np.asarray(_flatten(sample, "state"), dtype=np.float32),
                        "action": np.asarray(_flatten(sample, "action"), dtype=np.float32),
                        "observation.images.workspace_rgb": _decode_rgb(rgb, np),
                        "observation.images.workspace_depth": _decode_depth(depth, np),
                        "task": manifest["task"]["language_instruction"],
                    }
                )
            dataset.save_episode(parallel_encoding=False)
        dataset.finalize()
        source_hashes_after = {path: _sha256(Path(path)) for path in plan.source_files_sha256}
        if source_hashes_after != plan.source_files_sha256:
            raise RuntimeError("raw source changed during native encoding")
        output_hashes = _output_hashes(partial)
        partial.rename(output)
        receipt = {
            "schema_version": NATIVE_ENCODER_SCHEMA_VERSION,
            "created_at_utc": _utc_now(),
            "published": True,
            "repo_id": repo_id,
            "plan": plan.to_dict(),
            "dependency_status": native_dependency_status(),
            "features": features,
            "source_files_sha256": plan.source_files_sha256,
            "output_files_sha256": output_hashes,
            "round_trip": {"status": "pending_stage3"},
        }
        _write_json(output / "dapier_encoder_receipt.json", receipt)
        return receipt
    except Exception as error:
        failure_root = partial if partial.exists() else output.parent
        _write_json(
            failure_root / "dapier_encoder_failure.json",
            {
                "schema_version": NATIVE_ENCODER_SCHEMA_VERSION,
                "created_at_utc": _utc_now(),
                "published": False,
                "error_type": type(error).__name__,
                "error": str(error),
                "source_root": plan.source_root,
            },
        )
        raise
