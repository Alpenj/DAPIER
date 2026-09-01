from __future__ import annotations

import ast
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_core import MissionLocation
from mission_modules.camera import CameraFrame, CameraModality, CameraRole, MultiCameraFrameSet
from mission_modules.mobility import MobilityStatus
from mission_modules.object_pose import ShoePoseEstimate, validate_object_pose_inputs
from mission_modules.types import Pose2D, Pose3D
from mission_modules.visual_slam import (
    SlamEstimate,
    SlamTrackingState,
    validate_slam_inputs,
)


def front_frame() -> CameraFrame:
    return CameraFrame(
        role=CameraRole.FRONT_RGBD,
        modality=CameraModality.RGBD,
        frame_id=5,
        optical_frame="front_optical",
        calibration_id="front-v1",
        width=1,
        height=1,
        rgb_timestamp_ns=1_000_000,
        depth_timestamp_ns=1_000_000,
        received_monotonic_ns=2_000_000,
        rgb=bytes(3),
        depth_m_le_f32=bytes(4),
    )


def gripper_frame(role: CameraRole) -> CameraFrame:
    return CameraFrame(
        role=role,
        modality=CameraModality.RGB,
        frame_id=5,
        optical_frame=f"{role.value}_optical",
        calibration_id=f"{role.value}-v1",
        width=1,
        height=1,
        rgb_timestamp_ns=1_000_000,
        received_monotonic_ns=2_000_000,
        rgb=bytes(3),
    )


def mobility_status() -> MobilityStatus:
    return MobilityStatus(
        pose_map=Pose2D(0.0, 0.0, 0.0),
        linear_velocity_mps=0.0,
        angular_velocity_radps=0.0,
        active_goal=MissionLocation.B,
        goal_reached=True,
        watchdog_ok=True,
        observation_age_ms=10.0,
    )


def slam_estimate(**overrides) -> SlamEstimate:
    values = {
        "pose_map": Pose2D(1.0, 0.0, 0.0),
        "covariance_diagonal": (0.01**2, 0.01**2, 0.02**2),
        "tracking_state": SlamTrackingState.TRACKING,
        "map_id": "map-v1",
        "source_frame_id": 5,
        "observation_age_ms": 20.0,
    }
    values.update(overrides)
    return SlamEstimate(**values)


class PerceptionContractTest(unittest.TestCase):
    def test_visual_slam_accepts_front_rgbd_and_bounded_uncertainty(self) -> None:
        validate_slam_inputs(front_frame(), mobility_status())
        estimate = slam_estimate()
        self.assertTrue(
            estimate.usable(
                max_observation_age_ms=100.0,
                max_position_std_m=0.05,
                max_yaw_std_rad=0.10,
            )
        )
        lost = slam_estimate(tracking_state=SlamTrackingState.LOST)
        self.assertFalse(
            lost.usable(
                max_observation_age_ms=100.0,
                max_position_std_m=0.05,
                max_yaw_std_rad=0.10,
            )
        )

    def test_visual_slam_rejects_gripper_camera_input(self) -> None:
        with self.assertRaisesRegex(ValueError, "front_rgbd"):
            validate_slam_inputs(
                gripper_frame(CameraRole.LEFT_GRIPPER_RGB),
                mobility_status(),
            )

    def test_object_pose_uses_selected_active_camera(self) -> None:
        frames = MultiCameraFrameSet(
            sequence=0,
            frames=(
                front_frame(),
                gripper_frame(CameraRole.LEFT_GRIPPER_RGB),
                gripper_frame(CameraRole.RIGHT_GRIPPER_RGB),
            ),
        )
        validate_object_pose_inputs(
            frames,
            slam_estimate(),
            CameraRole.LEFT_GRIPPER_RGB,
        )
        disabled = MultiCameraFrameSet(
            sequence=0,
            frames=(front_frame(), gripper_frame(CameraRole.LEFT_GRIPPER_RGB)),
            disabled_roles=(CameraRole.RIGHT_GRIPPER_RGB,),
        )
        with self.assertRaisesRegex(ValueError, "disabled or missing"):
            validate_object_pose_inputs(
                disabled,
                slam_estimate(),
                CameraRole.RIGHT_GRIPPER_RGB,
            )

    def test_ground_truth_pose_is_not_runtime_usable_by_default(self) -> None:
        estimate = ShoePoseEstimate(
            pose_map=Pose3D(0.5, 0.0, 0.02, 1.0, 0.0, 0.0, 0.0),
            confidence=1.0,
            source_role=CameraRole.FRONT_RGBD,
            source_frame_id=5,
            observation_age_ms=10.0,
            map_id="map-v1",
            estimator_id="mujoco-ground-truth",
            ground_truth=True,
        )
        self.assertFalse(
            estimate.usable(
                minimum_confidence=0.6,
                max_observation_age_ms=100.0,
            )
        )
        self.assertTrue(
            estimate.usable(
                minimum_confidence=0.6,
                max_observation_age_ms=100.0,
                allow_ground_truth=True,
            )
        )

    def test_perception_contracts_import_no_runtime_backend(self) -> None:
        imported: set[str] = set()
        for filename in ("visual_slam.py", "object_pose.py"):
            tree = ast.parse(
                (PROJECT_DIR / "mission_modules" / filename).read_text(
                    encoding="utf-8"
                )
            )
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    imported.update(alias.name.split(".")[0] for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.module:
                    imported.add(node.module.split(".")[0])
        self.assertTrue(
            {"mujoco", "rclpy", "rospy", "serial", "cv2"}.isdisjoint(imported)
        )


if __name__ == "__main__":
    unittest.main()
