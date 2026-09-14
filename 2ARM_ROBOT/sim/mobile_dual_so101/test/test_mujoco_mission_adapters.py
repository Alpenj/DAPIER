from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import mujoco


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_core import MissionLocation
from mission_modules.camera import CameraRole
from mission_modules.mobility import NavigationRequest
from mission_modules.types import Pose2D
from mujoco_mission_adapters import (
    CAMERA_NAMES,
    MOBILE_BASE_FREE_JOINT,
    MuJoCoMobilityAdapter,
    MuJoCoMultiCameraAdapter,
    _add_gripper_cameras,
    build_mobile_shoe_mission_model,
)


class MuJoCoMissionAdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = build_mobile_shoe_mission_model()

    def setUp(self) -> None:
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

    def test_mobile_model_is_separate_and_has_four_cameras(self) -> None:
        free_joint_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            MOBILE_BASE_FREE_JOINT,
        )
        self.assertGreaterEqual(free_joint_id, 0)
        self.assertEqual(
            self.model.jnt_type[free_joint_id],
            mujoco.mjtJoint.mjJNT_FREE,
        )
        self.assertEqual(self.model.ncam, 4)
        for camera_name in CAMERA_NAMES.values():
            camera_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_CAMERA,
                camera_name,
            )
            self.assertGreaterEqual(camera_id, 0)
            if camera_name not in {
                CAMERA_NAMES[CameraRole.FRONT_RGBD],
                CAMERA_NAMES[CameraRole.WORKSPACE_RGBD],
            }:
                self.assertGreater(float(self.model.cam_fovy[camera_id]), 0.0)
        for side in ("left", "right"):
            for source_name in ("wrist_cam", "wrist_camera_sensor", "wrist"):
                self.assertLess(
                    mujoco.mj_name2id(
                        self.model,
                        mujoco.mjtObj.mjOBJ_CAMERA,
                        f"{side}_{source_name}",
                    ),
                    0,
                )
        self.assertNotIn(
            "left_gripper_camera_optical_frame",
            {self.model.site(site_id).name for site_id in range(self.model.nsite)},
        )
        self.assertEqual(self.model.nu, 12)

    def test_existing_source_wrist_cameras_are_reused(self) -> None:
        spec = mujoco.MjSpec()
        for side in ("left", "right"):
            body = spec.worldbody.add_body(name=f"{side}_gripper")
            body.add_camera(name=f"{side}_wrist_cam")
        _add_gripper_cameras(spec)
        model = spec.compile()
        self.assertEqual(model.ncam, 2)
        for side in ("left", "right"):
            self.assertGreaterEqual(
                mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_CAMERA,
                    f"{side}_gripper_camera",
                ),
                0,
            )

    def test_differential_drive_reaches_translation_and_yaw_goals(self) -> None:
        adapter = MuJoCoMobilityAdapter(self.model, self.data)
        adapter.request_navigation(
            NavigationRequest(
                goal=MissionLocation.B,
                target_pose=Pose2D(0.20, 0.0, 0.0),
                position_tolerance_m=0.01,
                yaw_tolerance_rad=0.02,
                timeout_s=5.0,
            )
        )
        adapter.advance(steps=1500)
        status = adapter.read_status()
        self.assertTrue(status.ready_at_goal(max_observation_age_ms=10.0))
        self.assertAlmostEqual(status.pose_map.x_m, 0.20, delta=0.015)

        adapter.request_navigation(
            NavigationRequest(
                goal=MissionLocation.A,
                target_pose=Pose2D(status.pose_map.x_m, status.pose_map.y_m, math.pi / 2),
                position_tolerance_m=0.01,
                yaw_tolerance_rad=0.02,
                timeout_s=5.0,
            )
        )
        adapter.advance(steps=1500)
        turned = adapter.read_status()
        self.assertTrue(turned.ready_at_goal(max_observation_age_ms=10.0))
        self.assertAlmostEqual(turned.pose_map.yaw_rad, math.pi / 2, delta=0.03)

    def test_navigation_timeout_latches_watchdog_and_zeroes_motion(self) -> None:
        adapter = MuJoCoMobilityAdapter(self.model, self.data)
        adapter.request_navigation(
            NavigationRequest(
                goal=MissionLocation.B,
                target_pose=Pose2D(2.0, 0.0, 0.0),
                timeout_s=0.005,
            )
        )
        adapter.advance(steps=10)
        status = adapter.read_status()
        self.assertFalse(status.watchdog_ok)
        self.assertTrue(status.stationary())
        self.assertIsNone(status.active_goal)

    def test_four_camera_payloads_are_rendered_and_synchronized(self) -> None:
        adapter = MuJoCoMultiCameraAdapter(
            self.model,
            self.data,
            width=32,
            height=24,
            clock_ns=lambda: 2_000_000_000,
        )
        try:
            initial_health = adapter.read_health()
            self.assertFalse(
                initial_health.operational(
                    frozenset(CameraRole),
                    max_frame_age_ms=1.0,
                    max_time_sync_error_ms=1.0,
                )
            )
            frames = adapter.capture()
            self.assertTrue(
                frames.synchronized(
                    max_inter_camera_delta_ns=0,
                    max_rgb_depth_delta_ns=0,
                )
            )
            self.assertEqual(
                len(frames.frame(CameraRole.FRONT_RGBD).rgb),
                32 * 24 * 3,
            )
            self.assertEqual(
                len(frames.frame(CameraRole.FRONT_RGBD).depth_m_le_f32),
                32 * 24 * 4,
            )
            self.assertEqual(
                len(frames.frame(CameraRole.WORKSPACE_RGBD).depth_m_le_f32),
                32 * 24 * 4,
            )
            self.assertEqual(
                frames.frame(CameraRole.FRONT_RGBD).optical_frame,
                "front_slam_depth_optical_frame",
            )
            self.assertEqual(
                frames.frame(CameraRole.WORKSPACE_RGBD).optical_frame,
                "workspace_depth_optical_frame",
            )
            for role in (
                CameraRole.LEFT_GRIPPER_RGB,
                CameraRole.RIGHT_GRIPPER_RGB,
            ):
                self.assertEqual(
                    frames.frame(role).optical_frame,
                    CAMERA_NAMES[role],
                )
            health = adapter.read_health()
            self.assertTrue(
                health.operational(
                    frozenset(CameraRole),
                    max_frame_age_ms=1.0,
                    max_time_sync_error_ms=1.0,
                )
            )
        finally:
            adapter.close()

    def test_camera_clock_regression_fails_health_closed(self) -> None:
        clock_values = iter((2_000_000_000, 1_999_999_999, 1_999_999_998))
        adapter = MuJoCoMultiCameraAdapter(
            self.model,
            self.data,
            width=32,
            height=24,
            clock_ns=lambda: next(clock_values),
        )
        try:
            adapter.capture()
            health = adapter.read_health()
            self.assertFalse(
                health.operational(
                    frozenset(CameraRole),
                    max_frame_age_ms=1.0,
                    max_time_sync_error_ms=1.0,
                )
            )
            self.assertTrue(
                all(stream.error_code == "clock_regressed" for stream in health.streams)
            )
            with self.assertRaisesRegex(RuntimeError, "clock regressed"):
                adapter.capture()
        finally:
            adapter.close()


if __name__ == "__main__":
    unittest.main()
