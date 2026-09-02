"""Offline validation of the DAPIER physical commissioning profile.

The validator never opens SSH, cameras, serial ports, ROS graphs, or devices.
It reports whether a profile is structurally ready for connection and which
facts still require physical measurement.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = "dapier.hardware-readiness.v1"
_PLACEHOLDER = re.compile(r"(?:REPLACE_|CHANGE_ME|TODO|TBD)", re.IGNORECASE)
_SECRET_KEY = re.compile(r"(?:password|passwd|token|secret|private_key)", re.IGNORECASE)
_EXPECTED_JOINT_SUFFIXES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class HardwareReadinessError(ValueError):
    pass


@dataclass(frozen=True)
class HardwareReadinessReport:
    schema_version: str
    profile_id: str
    structure_valid: bool
    ready_for_connection: bool
    ready_for_motion: bool
    pending_physical_evidence: tuple[str, ...]
    warnings: tuple[str, ...]
    hardware_accessed: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "profile_id": self.profile_id,
            "structure_valid": self.structure_valid,
            "ready_for_connection": self.ready_for_connection,
            "ready_for_motion": self.ready_for_motion,
            "pending_physical_evidence": list(self.pending_physical_evidence),
            "warnings": list(self.warnings),
            "hardware_accessed": self.hardware_accessed,
        }


def _object(value: object, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise HardwareReadinessError(f"{name} must be an object")
    return value


def _text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise HardwareReadinessError(f"{name} must be non-empty text")
    return value


def _number(value: object, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise HardwareReadinessError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        raise HardwareReadinessError(f"{name} has an invalid value")
    return result


def _bool(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise HardwareReadinessError(f"{name} must be boolean")
    return value


def _contains_placeholder(value: object) -> bool:
    if isinstance(value, str):
        return bool(_PLACEHOLDER.search(value))
    if isinstance(value, Mapping):
        return any(_contains_placeholder(item) for item in value.values())
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return any(_contains_placeholder(item) for item in value)
    return False


def _scan_secrets(value: object, path: str = "") -> None:
    if isinstance(value, Mapping):
        for raw_key, nested in value.items():
            key = str(raw_key)
            qualified = f"{path}.{key}" if path else key
            if _SECRET_KEY.search(key):
                raise HardwareReadinessError(
                    f"secret-bearing key is forbidden in profile: {qualified}"
                )
            _scan_secrets(nested, qualified)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for index, nested in enumerate(value):
            _scan_secrets(nested, f"{path}[{index}]")


def _validate_transform(value: object, name: str) -> None:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 4:
        raise HardwareReadinessError(f"{name} must be a 4x4 matrix")
    matrix: list[list[float]] = []
    for row in value:
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)) or len(row) != 4:
            raise HardwareReadinessError(f"{name} must be a 4x4 matrix")
        matrix.append([_number(item, name) for item in row])
    if any(abs(matrix[3][index] - expected) > 1e-9 for index, expected in enumerate((0, 0, 0, 1))):
        raise HardwareReadinessError(f"{name} must be homogeneous")
    rotation = [row[:3] for row in matrix[:3]]
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[k][i] * rotation[k][j] for k in range(3))
            expected = 1.0 if i == j else 0.0
            if abs(dot - expected) > 1e-4:
                raise HardwareReadinessError(f"{name} rotation must be orthonormal")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 1e-4:
        raise HardwareReadinessError(f"{name} rotation must be right-handed")


def validate_hardware_profile(
    value: Mapping[str, Any], *, require_live_values: bool = False
) -> HardwareReadinessReport:
    profile = _object(value, "profile")
    _scan_secrets(profile)
    if profile.get("schema_version") != SCHEMA_VERSION:
        raise HardwareReadinessError("hardware profile schema mismatch")
    profile_id = _text(profile.get("profile_id"), "profile_id")
    robot = _object(profile.get("robot"), "robot")
    if robot.get("platform") != "turtlebot3_waffle_pi_dual_so101":
        raise HardwareReadinessError("unexpected robot platform")

    tb3 = _object(profile.get("turtlebot3"), "turtlebot3")
    if tb3.get("user") != "user":
        raise HardwareReadinessError("TurtleBot3 user must be user")
    if tb3.get("auth_mode") != "ssh-key":
        raise HardwareReadinessError("TurtleBot3 auth_mode must be ssh-key")
    _text(tb3.get("host"), "turtlebot3.host")
    _text(tb3.get("ssh_key_path"), "turtlebot3.ssh_key_path")
    domain = tb3.get("ros_domain_id")
    if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= 232:
        raise HardwareReadinessError("ros_domain_id must be in 0..232")

    arms = _object(profile.get("arms"), "arms")
    if set(arms) != {"left", "right"}:
        raise HardwareReadinessError("arms must contain exactly left and right")
    serial_paths: set[str] = set()
    joint_names: set[str] = set()
    pending: list[str] = []
    for side in ("left", "right"):
        arm = _object(arms[side], f"arms.{side}")
        serial_path = _text(arm.get("serial_path"), f"arms.{side}.serial_path")
        if not serial_path.startswith("/dev/serial/by-id/"):
            raise HardwareReadinessError(f"arms.{side}.serial_path must use /dev/serial/by-id")
        if serial_path in serial_paths:
            raise HardwareReadinessError("left and right arms must use different serial paths")
        serial_paths.add(serial_path)
        names = arm.get("joint_names")
        expected = [f"{side}_{suffix}" for suffix in _EXPECTED_JOINT_SUFFIXES]
        if names != expected:
            raise HardwareReadinessError(f"arms.{side}.joint_names do not match the contract")
        if joint_names.intersection(names):
            raise HardwareReadinessError("joint names must be globally unique")
        joint_names.update(names)
        _text(arm.get("controller_identity_sha256"), f"arms.{side}.controller_identity_sha256")
        _text(arm.get("calibration_id"), f"arms.{side}.calibration_id")
        if not _bool(arm.get("calibration_verified"), f"arms.{side}.calibration_verified"):
            pending.append(f"{side} SO-101 calibration and direction verification")

    cameras = _object(profile.get("cameras"), "cameras")
    camera = _object(cameras.get("workspace_rgbd"), "cameras.workspace_rgbd")
    _text(camera.get("device_path"), "cameras.workspace_rgbd.device_path")
    _text(camera.get("frame_id"), "cameras.workspace_rgbd.frame_id")
    _text(camera.get("calibration_id"), "cameras.workspace_rgbd.calibration_id")
    if not _bool(camera.get("calibration_verified"), "camera.calibration_verified"):
        pending.append("workspace RGB-D intrinsics/extrinsics verification")
    intrinsics = _object(camera.get("intrinsics"), "camera.intrinsics")
    for dimension in ("width", "height"):
        item = intrinsics.get(dimension)
        if isinstance(item, bool) or not isinstance(item, int) or item <= 0:
            raise HardwareReadinessError(f"camera.intrinsics.{dimension} must be positive")
    for name in ("fx", "fy"):
        _number(intrinsics.get(name), f"camera.intrinsics.{name}", positive=require_live_values)
    for name in ("cx", "cy"):
        _number(intrinsics.get(name), f"camera.intrinsics.{name}")
    _validate_transform(camera.get("base_from_optical"), "camera.base_from_optical")

    safety = _object(profile.get("safety"), "safety")
    if not _bool(safety.get("e_stop_required"), "safety.e_stop_required"):
        raise HardwareReadinessError("e_stop_required must remain true")
    watchdog = _number(safety.get("command_watchdog_ms"), "command_watchdog_ms", positive=True)
    state_timeout = _number(
        safety.get("measured_state_timeout_ms"), "measured_state_timeout_ms", positive=True
    )
    if watchdog > 250 or state_timeout > 250:
        raise HardwareReadinessError("watchdog and measured-state timeout must be <=250 ms")
    for name in (
        "max_base_linear_mps",
        "max_base_angular_rad_s",
        "base_settle_linear_mps",
        "base_settle_angular_rad_s",
    ):
        _number(safety.get(name), f"safety.{name}", positive=True)

    evidence = _object(profile.get("evidence"), "evidence")
    evidence_fields = (
        "stationary_base_verified",
        "dual_arm_read_only_verified",
        "camera_stream_verified",
        "ros_graph_verified",
        "estop_verified",
    )
    for name in evidence_fields:
        if not _bool(evidence.get(name), f"evidence.{name}"):
            pending.append(name)

    placeholders = _contains_placeholder(profile)
    if placeholders:
        pending.append("replace all placeholder identifiers with local measured values")
    ready_for_connection = not placeholders
    ready_for_motion = ready_for_connection and not pending
    if require_live_values and not ready_for_motion:
        raise HardwareReadinessError(
            "live profile is incomplete: " + "; ".join(dict.fromkeys(pending))
        )
    warnings = (
        "validation is offline and does not prove that devices are connected",
        "ready_for_motion becomes true only after physical evidence is entered",
    )
    return HardwareReadinessReport(
        schema_version=SCHEMA_VERSION,
        profile_id=profile_id,
        structure_valid=True,
        ready_for_connection=ready_for_connection,
        ready_for_motion=ready_for_motion,
        pending_physical_evidence=tuple(dict.fromkeys(pending)),
        warnings=warnings,
    )


def load_hardware_profile(path: str | Path) -> Mapping[str, Any]:
    profile_path = Path(path)
    try:
        value = json.loads(profile_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise HardwareReadinessError(f"profile not found: {profile_path}") from error
    except json.JSONDecodeError as error:
        raise HardwareReadinessError(f"invalid profile JSON: {error}") from error
    return _object(value, "profile")
