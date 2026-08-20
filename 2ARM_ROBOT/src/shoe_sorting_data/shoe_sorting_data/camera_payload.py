"""Lossless ROS 2 Image payload contract for RGB and depth frames."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Any, Mapping


CAMERA_PAYLOAD_CONTRACT_VERSION = "dapier.ros2-image-payload.v0.1"
CAMERA_PAYLOAD_MODES = {"metadata_only", "required"}

_BYTES_PER_PIXEL = {
    "mono8": 1,
    "8uc1": 1,
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
    "mono16": 2,
    "16uc1": 2,
    "16sc1": 2,
    "32fc1": 4,
}
_STREAM_ENCODINGS = {
    "workspace_rgb": {"rgb8", "bgr8", "rgba8", "bgra8", "mono8", "8uc1"},
    "workspace_depth": {"mono16", "16uc1", "16sc1", "32fc1"},
}


class CameraPayloadError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class CameraFramePayload:
    width: int
    height: int
    encoding: str
    is_bigendian: int
    step: int
    data: bytes

    def __post_init__(self) -> None:
        if isinstance(self.width, bool) or not isinstance(self.width, int) or self.width <= 0:
            raise CameraPayloadError("camera_payload_geometry_invalid", "width must be a positive integer")
        if isinstance(self.height, bool) or not isinstance(self.height, int) or self.height <= 0:
            raise CameraPayloadError("camera_payload_geometry_invalid", "height must be a positive integer")
        if not isinstance(self.encoding, str) or self.encoding.lower() not in _BYTES_PER_PIXEL:
            raise CameraPayloadError(
                "camera_payload_encoding_invalid",
                f"unsupported ROS image encoding: {self.encoding!r}",
            )
        if self.is_bigendian not in (0, 1):
            raise CameraPayloadError("camera_payload_geometry_invalid", "is_bigendian must be 0 or 1")
        bytes_per_pixel = _BYTES_PER_PIXEL[self.encoding.lower()]
        minimum_step = self.width * bytes_per_pixel
        if isinstance(self.step, bool) or not isinstance(self.step, int) or self.step < minimum_step:
            raise CameraPayloadError(
                "camera_payload_geometry_invalid",
                f"step must be at least width*bytes_per_pixel ({minimum_step})",
            )
        if not isinstance(self.data, bytes):
            raise CameraPayloadError("camera_payload_data_invalid", "data must be immutable bytes")
        expected_bytes = self.step * self.height
        if len(self.data) != expected_bytes:
            raise CameraPayloadError(
                "camera_payload_size_mismatch",
                f"payload contains {len(self.data)} bytes but step*height requires {expected_bytes}",
            )


def validate_camera_frame_payload(stream_name: str, payload: CameraFramePayload) -> None:
    allowed = _STREAM_ENCODINGS.get(stream_name)
    if allowed is None:
        raise CameraPayloadError("camera_payload_stream_invalid", f"unsupported camera stream: {stream_name}")
    if payload.encoding.lower() not in allowed:
        raise CameraPayloadError(
            "camera_payload_encoding_invalid",
            f"{stream_name} does not accept encoding {payload.encoding!r}; expected one of {sorted(allowed)}",
        )


def _safe_payload_path(episode_dir: Path, relative_name: str) -> Path:
    relative = Path(relative_name)
    if relative.is_absolute() or ".." in relative.parts:
        raise CameraPayloadError("camera_payload_path_unsafe", f"unsafe camera payload path: {relative_name}")
    root = episode_dir.resolve()
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root)
    except ValueError as error:
        raise CameraPayloadError(
            "camera_payload_path_unsafe",
            f"camera payload escapes episode directory: {relative_name}",
        ) from error
    return candidate


def write_camera_payload(
    episode_dir: str | Path,
    stream_name: str,
    sample_index: int,
    payload: CameraFramePayload,
) -> dict[str, Any]:
    """Write one lossless ROS image frame without overwriting existing bytes."""

    if isinstance(sample_index, bool) or not isinstance(sample_index, int) or sample_index < 0:
        raise CameraPayloadError("camera_payload_index_invalid", "sample_index must be a non-negative integer")
    validate_camera_frame_payload(stream_name, payload)
    relative = Path("raw") / stream_name / f"frame_{sample_index:06d}.raw"
    destination = _safe_payload_path(Path(episode_dir), relative.as_posix())
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        with destination.open("xb") as stream:
            stream.write(payload.data)
    except FileExistsError as error:
        raise CameraPayloadError(
            "camera_payload_overwrite_blocked",
            f"camera payload already exists: {destination}",
        ) from error
    digest = hashlib.sha256(payload.data).hexdigest()
    return {
        "contract_version": CAMERA_PAYLOAD_CONTRACT_VERSION,
        "storage": "ros2_raw_rows",
        "stream": stream_name,
        "path": relative.as_posix(),
        "width": payload.width,
        "height": payload.height,
        "encoding": payload.encoding,
        "is_bigendian": payload.is_bigendian,
        "step": payload.step,
        "byte_count": len(payload.data),
        "sha256": digest,
    }


def read_camera_payload(
    episode_dir: str | Path,
    stream_name: str,
    metadata: Mapping[str, Any],
) -> CameraFramePayload:
    """Read and verify one payload before any image conversion is attempted."""

    if not isinstance(metadata, Mapping):
        raise CameraPayloadError("camera_payload_metadata_invalid", "payload metadata must be an object")
    if metadata.get("contract_version") != CAMERA_PAYLOAD_CONTRACT_VERSION:
        raise CameraPayloadError("camera_payload_contract_invalid", "camera payload contract version is unsupported")
    if metadata.get("storage") != "ros2_raw_rows" or metadata.get("stream") != stream_name:
        raise CameraPayloadError("camera_payload_metadata_invalid", "payload storage or stream metadata is invalid")
    relative_name = metadata.get("path")
    if not isinstance(relative_name, str) or not relative_name:
        raise CameraPayloadError("camera_payload_metadata_invalid", "payload path must be a non-empty string")
    payload_path = _safe_payload_path(Path(episode_dir), relative_name)
    try:
        data = payload_path.read_bytes()
    except FileNotFoundError as error:
        raise CameraPayloadError(
            "camera_payload_missing_file",
            f"camera payload file does not exist: {relative_name}",
        ) from error
    expected_count = metadata.get("byte_count")
    if isinstance(expected_count, bool) or not isinstance(expected_count, int) or expected_count != len(data):
        raise CameraPayloadError("camera_payload_size_mismatch", "payload byte_count does not match file size")
    expected_digest = metadata.get("sha256")
    actual_digest = hashlib.sha256(data).hexdigest()
    if not isinstance(expected_digest, str) or expected_digest.lower() != actual_digest:
        raise CameraPayloadError("camera_payload_checksum_mismatch", "camera payload SHA-256 does not match")
    try:
        payload = CameraFramePayload(
            width=metadata["width"],
            height=metadata["height"],
            encoding=metadata["encoding"],
            is_bigendian=metadata["is_bigendian"],
            step=metadata["step"],
            data=data,
        )
    except KeyError as error:
        raise CameraPayloadError(
            "camera_payload_metadata_invalid",
            f"camera payload metadata is missing {error.args[0]}",
        ) from error
    validate_camera_frame_payload(stream_name, payload)
    return payload


def verify_camera_payload(episode_dir: str | Path, stream_name: str, metadata: Mapping[str, Any]) -> None:
    read_camera_payload(episode_dir, stream_name, metadata)
