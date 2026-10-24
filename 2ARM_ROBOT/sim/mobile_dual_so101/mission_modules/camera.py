"""Acquisition contract for front/workspace RGB-D and gripper RGB cameras."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol, runtime_checkable


class CameraRole(str, Enum):
    FRONT_RGBD = "front_rgbd"
    WORKSPACE_RGBD = "workspace_rgbd"
    LEFT_GRIPPER_RGB = "left_gripper_rgb"
    RIGHT_GRIPPER_RGB = "right_gripper_rgb"


class CameraModality(str, Enum):
    RGB = "rgb"
    RGBD = "rgbd"


ALL_CAMERA_ROLES = frozenset(CameraRole)


def _non_negative_integer(value: int, *, label: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a non-negative integer")


@dataclass(frozen=True)
class CameraFrame:
    role: CameraRole
    modality: CameraModality
    frame_id: int
    optical_frame: str
    calibration_id: str
    width: int
    height: int
    rgb_timestamp_ns: int
    received_monotonic_ns: int
    rgb: bytes
    depth_timestamp_ns: int | None = None
    depth_m_le_f32: bytes | None = None
    valid: bool = True

    def validate(self) -> None:
        if not isinstance(self.role, CameraRole):
            raise ValueError("role must be a CameraRole")
        if not isinstance(self.modality, CameraModality):
            raise ValueError("modality must be a CameraModality")
        for label, value in (
            ("frame_id", self.frame_id),
            ("width", self.width),
            ("height", self.height),
            ("rgb_timestamp_ns", self.rgb_timestamp_ns),
            ("received_monotonic_ns", self.received_monotonic_ns),
        ):
            _non_negative_integer(value, label=label)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera dimensions must be positive")
        if not self.optical_frame.strip() or not self.calibration_id.strip():
            raise ValueError("optical_frame and calibration_id must not be empty")
        if not isinstance(self.rgb, bytes):
            raise ValueError("rgb payload must be bytes")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")
        if self.valid and len(self.rgb) != self.width * self.height * 3:
            raise ValueError("rgb8 payload size does not match dimensions")
        if self.modality == CameraModality.RGB:
            if self.depth_timestamp_ns is not None or self.depth_m_le_f32 is not None:
                raise ValueError("RGB camera must not provide a depth payload")
        else:
            if self.depth_timestamp_ns is None or self.depth_m_le_f32 is None:
                raise ValueError("RGB-D camera requires depth timestamp and payload")
            _non_negative_integer(
                self.depth_timestamp_ns,
                label="depth_timestamp_ns",
            )
            if not isinstance(self.depth_m_le_f32, bytes):
                raise ValueError("depth payload must be bytes")
            if self.valid and len(self.depth_m_le_f32) != self.width * self.height * 4:
                raise ValueError("32FC1 payload size does not match dimensions")

    @property
    def rgb_depth_sync_delta_ns(self) -> int:
        if self.depth_timestamp_ns is None:
            return 0
        return abs(self.rgb_timestamp_ns - self.depth_timestamp_ns)


@dataclass(frozen=True)
class MultiCameraFrameSet:
    sequence: int
    frames: tuple[CameraFrame, ...]
    disabled_roles: tuple[CameraRole, ...] = ()

    def validate(self) -> None:
        _non_negative_integer(self.sequence, label="sequence")
        roles = [frame.role for frame in self.frames]
        disabled = list(self.disabled_roles)
        if any(not isinstance(role, CameraRole) for role in disabled):
            raise ValueError("disabled_roles must contain CameraRole values")
        if len(set(roles)) != len(roles) or len(set(disabled)) != len(disabled):
            raise ValueError("camera roles must not be duplicated")
        if set(roles) & set(disabled):
            raise ValueError("camera role cannot be active and disabled")
        configured = set(roles) | set(disabled)
        if configured != ALL_CAMERA_ROLES:
            missing = sorted(role.value for role in ALL_CAMERA_ROLES - configured)
            raise ValueError(f"camera roles must be active or explicitly disabled: {missing}")
        for frame in self.frames:
            frame.validate()
        for role in (CameraRole.FRONT_RGBD, CameraRole.WORKSPACE_RGBD):
            rgbd = self.frame(role)
            if rgbd is not None and rgbd.modality != CameraModality.RGBD:
                raise ValueError(f"{role.value} role requires RGB-D modality")
        for role in (
            CameraRole.LEFT_GRIPPER_RGB,
            CameraRole.RIGHT_GRIPPER_RGB,
        ):
            gripper = self.frame(role)
            if gripper is not None and gripper.modality != CameraModality.RGB:
                raise ValueError("gripper camera roles require RGB modality")

    def frame(self, role: CameraRole) -> CameraFrame | None:
        return next((frame for frame in self.frames if frame.role == role), None)

    def synchronized(
        self,
        *,
        max_inter_camera_delta_ns: int,
        max_rgb_depth_delta_ns: int,
    ) -> bool:
        _non_negative_integer(
            max_inter_camera_delta_ns,
            label="max_inter_camera_delta_ns",
        )
        _non_negative_integer(
            max_rgb_depth_delta_ns,
            label="max_rgb_depth_delta_ns",
        )
        self.validate()
        if not self.frames or not all(frame.valid for frame in self.frames):
            return False
        timestamps = [frame.rgb_timestamp_ns for frame in self.frames]
        inter_camera_delta = max(timestamps) - min(timestamps)
        return inter_camera_delta <= max_inter_camera_delta_ns and all(
            frame.rgb_depth_sync_delta_ns <= max_rgb_depth_delta_ns
            for frame in self.frames
        )

    def max_age_ms(self, *, now_monotonic_ns: int) -> float:
        _non_negative_integer(now_monotonic_ns, label="now_monotonic_ns")
        self.validate()
        if not self.frames:
            raise ValueError("cannot measure age without active camera frames")
        oldest_receipt = min(frame.received_monotonic_ns for frame in self.frames)
        if now_monotonic_ns < oldest_receipt:
            raise ValueError("now_monotonic_ns precedes camera frame receipt")
        return (now_monotonic_ns - oldest_receipt) / 1_000_000.0


@dataclass(frozen=True)
class CameraStreamHealth:
    role: CameraRole
    enabled: bool
    online: bool
    calibration_loaded: bool
    dropped_frames: int
    last_frame_age_ms: float
    time_sync_error_ms: float
    error_code: str = ""

    def validate(self) -> None:
        if not isinstance(self.role, CameraRole):
            raise ValueError("health role must be a CameraRole")
        if not all(
            isinstance(value, bool)
            for value in (self.enabled, self.online, self.calibration_loaded)
        ):
            raise ValueError("camera health flags must be booleans")
        _non_negative_integer(self.dropped_frames, label="dropped_frames")
        values = (self.last_frame_age_ms, self.time_sync_error_ms)
        if not all(math.isfinite(value) and value >= 0 for value in values):
            raise ValueError("camera age and sync error must be finite and non-negative")
        if len(self.error_code) > 100:
            raise ValueError("camera error_code is too long")

    def healthy(
        self,
        *,
        max_frame_age_ms: float,
        max_time_sync_error_ms: float,
    ) -> bool:
        limits = (max_frame_age_ms, max_time_sync_error_ms)
        if not all(math.isfinite(value) and value > 0 for value in limits):
            raise ValueError("camera health limits must be finite and positive")
        self.validate()
        return (
            self.enabled
            and self.online
            and self.calibration_loaded
            and not self.error_code
            and self.last_frame_age_ms <= max_frame_age_ms
            and self.time_sync_error_ms <= max_time_sync_error_ms
        )


@dataclass(frozen=True)
class CameraRigHealth:
    streams: tuple[CameraStreamHealth, ...]

    def validate(self) -> None:
        roles = [stream.role for stream in self.streams]
        if len(set(roles)) != len(roles) or set(roles) != ALL_CAMERA_ROLES:
            raise ValueError("camera rig health requires each role exactly once")
        for stream in self.streams:
            stream.validate()

    def operational(
        self,
        required_roles: frozenset[CameraRole],
        *,
        max_frame_age_ms: float,
        max_time_sync_error_ms: float,
    ) -> bool:
        self.validate()
        if not required_roles <= ALL_CAMERA_ROLES:
            raise ValueError("required_roles contains an unknown camera role")
        by_role = {stream.role: stream for stream in self.streams}
        return all(
            by_role[role].healthy(
                max_frame_age_ms=max_frame_age_ms,
                max_time_sync_error_ms=max_time_sync_error_ms,
            )
            for role in required_roles
        )


@runtime_checkable
class CameraPort(Protocol):
    """Acquisition-only port; SLAM and object pose remain separate consumers."""

    @property
    def simulation_only(self) -> bool: ...

    def capture(self) -> MultiCameraFrameSet: ...

    def read_health(self) -> CameraRigHealth: ...


__all__ = [
    "ALL_CAMERA_ROLES",
    "CameraFrame",
    "CameraModality",
    "CameraPort",
    "CameraRigHealth",
    "CameraRole",
    "CameraStreamHealth",
    "MultiCameraFrameSet",
]
