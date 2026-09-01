from __future__ import annotations
from dataclasses import replace

import json
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_core import MissionLocation, MissionPhase
from mission_modules.camera import CameraRigHealth, CameraRole, CameraStreamHealth
from mission_modules.health import EdgeComputeHealth, EdgeResourceLimits, RuntimeHealthSnapshot
from mission_modules.manipulator import ArmSide, ArmTelemetry, ManipulatorStatus
from mission_modules.mobility import MobilityStatus
from mission_modules.object_pose import ShoePoseEstimate
from mission_modules.tactile import (
    TactileChannelStatus,
    TactileModality,
    TactileRigStatus,
    TactileRole,
)
from mission_modules.transport import LinkHeartbeat
from mission_modules.types import Pose2D, Pose3D
from mission_modules.visual_slam import SlamEstimate, SlamTrackingState
from mission_modules.world_state import WorldStateSnapshot


def arm(side: ArmSide) -> ArmTelemetry:
    return ArmTelemetry(
        side=side,
        joint_position_rad=(0.0,) * 5,
        joint_velocity_radps=(0.0,) * 5,
        motor_temperature_c=(30.0,) * 5,
        motor_error=(False,) * 5,
        gripper_position_rad=0.0,
        gripper_holding=False,
    )


def snapshot(*, overloaded: bool = False, shoe_map_id: str = "map-v1") -> WorldStateSnapshot:
    camera_health = CameraRigHealth(
        streams=tuple(
            CameraStreamHealth(
                role=role,
                enabled=True,
                online=True,
                calibration_loaded=True,
                dropped_frames=0,
                last_frame_age_ms=10.0,
                time_sync_error_ms=1.0,
            )
            for role in CameraRole
        )
    )
    manipulator = ManipulatorStatus(
        left=arm(ArmSide.LEFT),
        right=arm(ArmSide.RIGHT),
        active_request_id=None,
        trajectory_complete=True,
        collision_free=True,
        watchdog_ok=True,
        object_lifted=False,
        carry_pose_clear=True,
        observation_age_ms=10.0,
    )
    runtime_health = RuntimeHealthSnapshot(
        sequence=4,
        edge_compute=EdgeComputeHealth(
            cpu_utilization=0.95 if overloaded else 0.30,
            memory_utilization=0.40,
            temperature_c=50.0,
            control_loop_lag_ms=2.0,
            camera_queue_depth=1,
            under_voltage=False,
            observation_age_ms=10.0,
        ),
    )
    return WorldStateSnapshot(
        sequence=7,
        captured_monotonic_ns=1_000_000_000,
        mission_phase=MissionPhase.LOCALIZING_SHOE,
        mobility=MobilityStatus(
            pose_map=Pose2D(1.0, 0.0, 0.0),
            linear_velocity_mps=0.0,
            angular_velocity_radps=0.0,
            active_goal=MissionLocation.B,
            goal_reached=True,
            watchdog_ok=True,
            observation_age_ms=10.0,
        ),
        camera_health=camera_health,
        slam=SlamEstimate(
            pose_map=Pose2D(1.0, 0.0, 0.0),
            covariance_diagonal=(0.01, 0.01, 0.01),
            tracking_state=SlamTrackingState.TRACKING,
            map_id="map-v1",
            source_frame_id=3,
            observation_age_ms=10.0,
        ),
        manipulator=manipulator,
        runtime_health=runtime_health,
        link_heartbeat=LinkHeartbeat(
            sequence=5,
            received_monotonic_ns=1_000_000_000,
            timeout_ms=250.0,
        ),
        shoe_pose=ShoePoseEstimate(
            pose_map=Pose3D(1.2, 0.1, 0.02, 1.0, 0.0, 0.0, 0.0),
            confidence=0.8,
            source_role=CameraRole.LEFT_GRIPPER_RGB,
            source_frame_id=3,
            observation_age_ms=10.0,
            map_id=shoe_map_id,
            estimator_id="test-estimator",
        ),
    )


class WorldStateContractTest(unittest.TestCase):
    def test_summary_contains_alert_but_does_not_block_cpu_pressure(self) -> None:
        state = snapshot(overloaded=True)
        summary = state.as_supervisor_summary(edge_limits=EdgeResourceLimits())
        self.assertTrue(summary["edge"]["motion_allowed"])
        self.assertIn("edge_cpu_pressure", summary["edge"]["alert_codes"])

    def test_summary_excludes_raw_camera_payloads(self) -> None:
        summary_json = json.dumps(
            snapshot().as_supervisor_summary(edge_limits=EdgeResourceLimits())
        )
        self.assertNotIn('"rgb":', summary_json)
        self.assertNotIn("depth_m_le_f32", summary_json)

    def test_snapshot_and_heartbeat_freshness_are_both_required(self) -> None:
        state = snapshot()
        self.assertTrue(
            state.fresh(
                now_monotonic_ns=1_100_000_000,
                max_snapshot_age_ms=200.0,
            )
        )
        self.assertFalse(
            state.fresh(
                now_monotonic_ns=1_300_000_000,
                max_snapshot_age_ms=500.0,
            )
        )

    def test_tactile_summary_contains_fsr_alerts_not_raw_samples(self) -> None:
        left = TactileChannelStatus(
            role=TactileRole.LEFT_GRIPPER,
            enabled=True,
            modality=TactileModality.NORMAL_FORCE,
            sensor_id="left-fsr",
            calibration_id="left-fsr-cal-v1",
            sequence=2,
            normal_force_n=2.0,
            slip_probability=0.8,
            contact=True,
            overpressure=False,
            valid=True,
            observation_age_ms=5.0,
        )
        right = TactileChannelStatus(
            role=TactileRole.RIGHT_GRIPPER,
            enabled=False,
            modality=None,
            sensor_id="",
            calibration_id="",
            sequence=0,
            normal_force_n=0.0,
            slip_probability=0.0,
            contact=False,
            overpressure=False,
            valid=False,
            observation_age_ms=0.0,
        )
        state = replace(
            snapshot(),
            tactile_status=TactileRigStatus(channels=(left, right)),
        )
        summary = state.as_supervisor_summary(edge_limits=EdgeResourceLimits())
        self.assertTrue(summary["tactile"]["available"])
        self.assertIn(
            "left_gripper:slip",
            summary["tactile"]["alert_codes"],
        )
        self.assertEqual(
            summary["tactile"]["channels"][0]["modality"],
            "normal_force",
        )

    def test_shoe_pose_and_slam_map_must_match(self) -> None:
        with self.assertRaisesRegex(ValueError, "map_id"):
            snapshot(shoe_map_id="different-map").validate()


if __name__ == "__main__":
    unittest.main()
