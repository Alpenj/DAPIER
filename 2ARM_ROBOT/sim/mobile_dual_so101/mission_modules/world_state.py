"""Typed workstation world snapshot assembled from edge and perception data."""

from __future__ import annotations

from dataclasses import dataclass
import math

from mission_core import MissionPhase

from .camera import CameraRigHealth
from .health import EdgeResourceLimits, RuntimeHealthSnapshot
from .manipulator import ManipulatorStatus
from .mobility import MobilityStatus
from .object_pose import ShoePoseEstimate
from .tactile import TactileLimits, TactileRigStatus
from .transport import LinkHeartbeat
from .visual_slam import SlamEstimate


@dataclass(frozen=True)
class WorldStateSnapshot:
    sequence: int
    captured_monotonic_ns: int
    mission_phase: MissionPhase
    mobility: MobilityStatus
    camera_health: CameraRigHealth
    slam: SlamEstimate
    manipulator: ManipulatorStatus
    runtime_health: RuntimeHealthSnapshot
    link_heartbeat: LinkHeartbeat
    shoe_pose: ShoePoseEstimate | None = None
    tactile_status: TactileRigStatus | None = None

    def validate(self) -> None:
        for label, value in (
            ("sequence", self.sequence),
            ("captured_monotonic_ns", self.captured_monotonic_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{label} must be a non-negative integer")
        if not isinstance(self.mission_phase, MissionPhase):
            raise ValueError("mission_phase must be a MissionPhase")
        self.mobility.validate()
        self.camera_health.validate()
        self.slam.validate()
        self.manipulator.validate()
        self.runtime_health.validate()
        self.link_heartbeat.validate()
        if self.shoe_pose is not None:
            self.shoe_pose.validate()
            if self.shoe_pose.map_id != self.slam.map_id:
                raise ValueError("shoe pose and SLAM map_id must match")
        if self.tactile_status is not None:
            self.tactile_status.validate()

    def age_ms(self, *, now_monotonic_ns: int) -> float:
        self.validate()
        if (
            isinstance(now_monotonic_ns, bool)
            or not isinstance(now_monotonic_ns, int)
            or now_monotonic_ns < self.captured_monotonic_ns
        ):
            raise ValueError("now_monotonic_ns must not precede snapshot capture")
        return (now_monotonic_ns - self.captured_monotonic_ns) / 1_000_000.0

    def fresh(
        self,
        *,
        now_monotonic_ns: int,
        max_snapshot_age_ms: float,
    ) -> bool:
        if not math.isfinite(max_snapshot_age_ms) or max_snapshot_age_ms <= 0:
            raise ValueError("max_snapshot_age_ms must be finite and positive")
        component_ages_ms = [
            self.mobility.observation_age_ms,
            self.slam.observation_age_ms,
            self.manipulator.observation_age_ms,
            self.runtime_health.edge_compute.observation_age_ms,
            *(
                stream.last_frame_age_ms
                for stream in self.camera_health.streams
                if stream.enabled
            ),
        ]
        if self.shoe_pose is not None:
            component_ages_ms.append(self.shoe_pose.observation_age_ms)
        if self.tactile_status is not None:
            component_ages_ms.extend(
                channel.observation_age_ms
                for channel in self.tactile_status.channels
                if channel.enabled
            )
        snapshot_age_ms = self.age_ms(now_monotonic_ns=now_monotonic_ns)
        return (
            snapshot_age_ms <= max_snapshot_age_ms
            and self.link_heartbeat.alive(now_monotonic_ns=now_monotonic_ns)
            and all(
                snapshot_age_ms + age <= max_snapshot_age_ms
                for age in component_ages_ms
            )
        )

    def as_supervisor_summary(
        self,
        *,
        edge_limits: EdgeResourceLimits,
        tactile_limits: TactileLimits | None = None,
    ) -> dict[str, object]:
        """Return bounded metadata; raw camera payloads are intentionally absent."""

        self.validate()
        resolved_tactile_limits = tactile_limits or TactileLimits()
        pose = self.mobility.pose_map
        shoe_pose: dict[str, object] | None = None
        if self.shoe_pose is not None:
            shoe_pose = {
                "confidence": self.shoe_pose.confidence,
                "source_role": self.shoe_pose.source_role.value,
                "observation_age_ms": self.shoe_pose.observation_age_ms,
                "ground_truth": self.shoe_pose.ground_truth,
                "position_m": {
                    "x": self.shoe_pose.pose_map.x_m,
                    "y": self.shoe_pose.pose_map.y_m,
                    "z": self.shoe_pose.pose_map.z_m,
                },
            }
        return {
            "sequence": self.sequence,
            "mission_phase": self.mission_phase.value,
            "base": {
                "pose_map": {"x_m": pose.x_m, "y_m": pose.y_m, "yaw_rad": pose.yaw_rad},
                "linear_velocity_mps": self.mobility.linear_velocity_mps,
                "angular_velocity_radps": self.mobility.angular_velocity_radps,
                "watchdog_ok": self.mobility.watchdog_ok,
            },
            "slam": {
                "tracking_state": self.slam.tracking_state.value,
                "observation_age_ms": self.slam.observation_age_ms,
                "covariance_diagonal": list(self.slam.covariance_diagonal),
            },
            "shoe_pose": shoe_pose,
            "manipulator": {
                "trajectory_complete": self.manipulator.trajectory_complete,
                "collision_free": self.manipulator.collision_free,
                "watchdog_ok": self.manipulator.watchdog_ok,
                "object_lifted": self.manipulator.object_lifted,
                "gripper_holding": (
                    self.manipulator.left.gripper_holding
                    or self.manipulator.right.gripper_holding
                ),
                "carry_pose_clear": self.manipulator.carry_pose_clear,
            },
            "tactile": self._tactile_summary(resolved_tactile_limits),
            "edge": {
                "motion_allowed": self.runtime_health.motion_allowed(edge_limits),
                "alert_codes": list(self.runtime_health.alert_codes(edge_limits)),
                "load_shedding_required": self.runtime_health.load_shedding_required(
                    edge_limits
                ),
                "fault_codes": list(self.runtime_health.fault_codes),
            },
            "camera_health": [
                {
                    "role": stream.role.value,
                    "enabled": stream.enabled,
                    "online": stream.online,
                    "calibration_loaded": stream.calibration_loaded,
                    "last_frame_age_ms": stream.last_frame_age_ms,
                    "time_sync_error_ms": stream.time_sync_error_ms,
                    "error_code": stream.error_code,
                }
                for stream in self.camera_health.streams
            ],
        }


    def _tactile_summary(self, limits: TactileLimits) -> dict[str, object]:
        if self.tactile_status is None:
            return {"available": False, "alert_codes": [], "channels": []}
        return {
            "available": any(channel.enabled for channel in self.tactile_status.channels),
            "alert_codes": list(self.tactile_status.alert_codes(limits)),
            "channels": [
                {
                    "role": channel.role.value,
                    "enabled": channel.enabled,
                    "modality": (
                        channel.modality.value if channel.modality is not None else None
                    ),
                    "contact": channel.contact,
                    "normal_force_n": channel.normal_force_n,
                    "slip_probability": channel.slip_probability,
                    "overpressure": channel.overpressure,
                    "observation_age_ms": channel.observation_age_ms,
                }
                for channel in self.tactile_status.channels
            ],
        }

__all__ = ["WorldStateSnapshot"]
