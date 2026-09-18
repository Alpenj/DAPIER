from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dapier_research.camera_board_transform import camera_from_board_pose, camera_to_arm_chain
from dapier_research.vision_target import VisionTargetError


class CameraBoardTransformTest(unittest.TestCase):
    def test_opencv_board_to_camera_inverse_and_arm_composition(self):
        rv = np.array([.23, -.19, .08])
        tv = np.array([-.07, -.05, .7])
        camera_board = camera_from_board_pose(rv, tv)
        arm_board = np.eye(4)
        arm_board[:3, :3] = cv2.Rodrigues(np.array([-.1, .2, -.3]))[0]
        arm_board[:3, 3] = [.1, -.2, .3]
        result = camera_to_arm_chain(
            camera_board, arm_board, board_to_arm_verified=True,
            board_to_arm_revision="SYNTHETIC", arm_frame="synthetic_left_arm_base")
        arm_camera = np.asarray(result["T_arm_from_camera"])
        np.testing.assert_allclose(
            arm_camera @ camera_board, arm_board, atol=1e-12, rtol=0)
        point_board = np.array([.024, .048, 0., 1.])
        np.testing.assert_allclose(
            arm_camera @ camera_board @ point_board,
            arm_board @ point_board, atol=1e-12, rtol=0)
        np.testing.assert_allclose(
            np.asarray(result["T_board_from_camera"]) @ camera_board,
            np.eye(4), atol=1e-12, rtol=0)
        self.assertTrue(result["candidate_only"])
        for flag in ("independently_validated", "control_authorized", "hardware_execution"):
            self.assertFalse(result[flag])

    def test_missing_measurement_revision_or_frame_fails_closed(self):
        valid = dict(board_to_arm_verified=True, board_to_arm_revision="SYNTHETIC",
                     arm_frame="synthetic_left_arm_base")
        for change in (dict(board_to_arm_verified=False),
                       dict(board_to_arm_revision=""), dict(arm_frame="")):
            with self.assertRaises(VisionTargetError):
                camera_to_arm_chain(np.eye(4), np.eye(4), **dict(valid, **change))

    def test_invalid_pose_and_rigid_transform_rejected(self):
        for rv, tv in (([0., 0.], [0., 0., 1.]),
                       ([np.nan, 0., 0.], [0., 0., 1.]),
                       ([0., 0., 0.], [0., np.inf, 1.])):
            with self.assertRaises(VisionTargetError):
                camera_from_board_pose(rv, tv)
        valid = dict(board_to_arm_verified=True, board_to_arm_revision="SYNTHETIC",
                     arm_frame="synthetic_left_arm_base")
        for transform in (np.ones((3, 3)), np.diag([2., 1., 1., 1.]),
                          np.diag([-1., 1., 1., 1.])):
            with self.assertRaises(VisionTargetError):
                camera_to_arm_chain(transform, np.eye(4), **valid)


if __name__ == "__main__":
    unittest.main()
