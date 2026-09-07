from __future__ import annotations

import ast
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_modules.camera import (
    CameraFrame,
    CameraModality,
    CameraPort,
    CameraRigHealth,
    CameraRole,
    CameraStreamHealth,
    MultiCameraFrameSet,
)


def camera_frame(role: CameraRole, timestamp_ns: int) -> CameraFrame:
    modality = (
        CameraModality.RGBD
        if role in (CameraRole.FRONT_RGBD, CameraRole.WORKSPACE_RGBD)
        else CameraModality.RGB
    )
    return CameraFrame(
        role=role,
        modality=modality,
        frame_id=7,
        optical_frame=f"{role.value}_optical",
        calibration_id=f"{role.value}_calib_v1",
        width=2,
        height=2,
        rgb_timestamp_ns=timestamp_ns,
        received_monotonic_ns=2_000_000,
        rgb=bytes(2 * 2 * 3),
        depth_timestamp_ns=timestamp_ns + 100_000 if modality == CameraModality.RGBD else None,
        depth_m_le_f32=bytes(2 * 2 * 4) if modality == CameraModality.RGBD else None,
    )


def frame_set(*, disable_right: bool = False) -> MultiCameraFrameSet:
    frames = [
        camera_frame(CameraRole.FRONT_RGBD, 1_000_000),
        camera_frame(CameraRole.WORKSPACE_RGBD, 1_100_000),
        camera_frame(CameraRole.LEFT_GRIPPER_RGB, 1_200_000),
    ]
    disabled = ()
    if disable_right:
        disabled = (CameraRole.RIGHT_GRIPPER_RGB,)
    else:
        frames.append(camera_frame(CameraRole.RIGHT_GRIPPER_RGB, 1_300_000))
    return MultiCameraFrameSet(
        sequence=3,
        frames=tuple(frames),
        disabled_roles=disabled,
    )


def stream_health(role: CameraRole, *, enabled: bool = True) -> CameraStreamHealth:
    return CameraStreamHealth(
        role=role,
        enabled=enabled,
        online=enabled,
        calibration_loaded=enabled,
        dropped_frames=0,
        last_frame_age_ms=10.0,
        time_sync_error_ms=1.0,
    )


class FakeCameraRig:
    simulation_only = True

    def capture(self) -> MultiCameraFrameSet:
        return frame_set()

    def read_health(self) -> CameraRigHealth:
        return CameraRigHealth(
            tuple(stream_health(role) for role in CameraRole)
        )


class CameraContractTest(unittest.TestCase):
    def test_two_rgbd_and_two_gripper_rgb_frames_are_synchronized(self) -> None:
        samples = frame_set()
        samples.validate()
        self.assertEqual(
            samples.frame(CameraRole.FRONT_RGBD).modality,
            CameraModality.RGBD,
        )
        self.assertEqual(
            samples.frame(CameraRole.WORKSPACE_RGBD).modality,
            CameraModality.RGBD,
        )
        self.assertEqual(
            samples.frame(CameraRole.LEFT_GRIPPER_RGB).modality,
            CameraModality.RGB,
        )
        self.assertTrue(
            samples.synchronized(
                max_inter_camera_delta_ns=500_000,
                max_rgb_depth_delta_ns=200_000,
            )
        )
        self.assertEqual(samples.max_age_ms(now_monotonic_ns=3_000_000), 1.0)

    def test_missing_camera_must_be_explicitly_disabled(self) -> None:
        missing = MultiCameraFrameSet(
            sequence=0,
            frames=(camera_frame(CameraRole.FRONT_RGBD, 1_000_000),),
        )
        with self.assertRaisesRegex(ValueError, "explicitly disabled"):
            missing.validate()
        disabled = frame_set(disable_right=True)
        disabled.validate()
        self.assertIsNone(disabled.frame(CameraRole.RIGHT_GRIPPER_RGB))

    def test_modality_and_payload_mismatch_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "must not provide"):
            CameraFrame(
                **{
                    **camera_frame(CameraRole.LEFT_GRIPPER_RGB, 1_000_000).__dict__,
                    "depth_timestamp_ns": 1_000_000,
                    "depth_m_le_f32": bytes(16),
                }
            ).validate()
        wrong_front = CameraFrame(
            **{
                **camera_frame(CameraRole.LEFT_GRIPPER_RGB, 1_000_000).__dict__,
                "role": CameraRole.FRONT_RGBD,
            }
        )
        with self.assertRaisesRegex(ValueError, "front_rgbd"):
            MultiCameraFrameSet(
                sequence=0,
                frames=(
                    wrong_front,
                    camera_frame(CameraRole.WORKSPACE_RGBD, 1_000_000),
                    camera_frame(CameraRole.LEFT_GRIPPER_RGB, 1_000_000),
                    camera_frame(CameraRole.RIGHT_GRIPPER_RGB, 1_000_000),
                ),
            ).validate()

    def test_rig_health_checks_required_roles_only(self) -> None:
        health = CameraRigHealth(
            (
                stream_health(CameraRole.FRONT_RGBD),
                stream_health(CameraRole.WORKSPACE_RGBD),
                stream_health(CameraRole.LEFT_GRIPPER_RGB),
                stream_health(CameraRole.RIGHT_GRIPPER_RGB, enabled=False),
            )
        )
        self.assertTrue(
            health.operational(
                frozenset(
                    {
                        CameraRole.FRONT_RGBD,
                        CameraRole.LEFT_GRIPPER_RGB,
                    }
                ),
                max_frame_age_ms=100.0,
                max_time_sync_error_ms=5.0,
            )
        )
        self.assertFalse(
            health.operational(
                frozenset(CameraRole),
                max_frame_age_ms=100.0,
                max_time_sync_error_ms=5.0,
            )
        )

    def test_protocol_fixture_and_backend_independence(self) -> None:
        port = FakeCameraRig()
        self.assertIsInstance(port, CameraPort)
        self.assertTrue(port.simulation_only)
        self.assertEqual(len(port.capture().frames), 4)

        tree = ast.parse(
            (PROJECT_DIR / "mission_modules" / "camera.py").read_text(
                encoding="utf-8"
            )
        )
        imported: set[str] = set()
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
