from pathlib import Path
import copy
import hashlib
import json
import sys
import tempfile
import unittest

try:
    import cv2
except ModuleNotFoundError:
    cv2 = None
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dapier_research.camera_board_transform import (
    camera_from_board_pose,
    camera_to_arm_chain,
    measured_board_to_left_arm_transform,
    known_cube_from_board,
)
from dapier_research.real_sensor_ik_adapter import load_block_observation
from dapier_research.vision_target import VisionTargetError


class CameraBoardTransformTest(unittest.TestCase):
    @unittest.skipIf(cv2 is None, "OpenCV is supplied by the vision/MuJoCo environment")
    def test_new_frame_geometry_loads_as_distinct_surface_center_and_support(self):
        camera_board = np.eye(4)
        camera_board[2, 3] = .5
        corners = np.array([[.08,.03,-.04],[.12,.03,-.04],[.12,.07,-.04],[.08,.07,-.04]])
        camera = corners + camera_board[:3, 3]
        pixels = camera[:, :2]/camera[:, 2, None]*400 + [320., 230.]
        observation = {"camera_frame":"os30a_rectified_left_optical", "cube_side_m":.04,
            "clock":"unix_ns", "timestamp_ns":1000, "calibration_revision":"SYNTHETIC",
            "intrinsics":{"width":640,"height":460,"fx":400.,"fy":400.,"cx":320.,"cy":230.},
            "top_corners_uv":pixels.tolist(), "T_camera_from_board":camera_board.tolist(),
            "T_datum_from_board":measured_board_to_left_arm_transform([-30.,200.,-35.]).tolist()}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            frame = root / "frame.npy"
            np.save(frame, np.zeros((460,640,3), np.uint8))
            def source(path):
                return {"path":str(path), "sha256":hashlib.sha256(path.read_bytes()).hexdigest()}
            calibration = root / "calibration.json"
            calibration.write_text(json.dumps(observation["intrinsics"]))
            observation.update(frame_source=source(frame), board_pose_frame_sha256=source(frame)["sha256"],
                               top_corners_frame_sha256=source(frame)["sha256"],
                               calibration_sources=[source(calibration)])
            path = root / "observation.json"
            path.write_text(json.dumps({"board_cube_observation":observation}))
            result = load_block_observation(path)
            np.testing.assert_allclose(result["target_arm_xyz_candidate_m"], [.15,-.13,.005], atol=1e-12)
            np.testing.assert_allclose(result["scene_object"]["center_xyz_m"], [.15,-.13,-.015], atol=1e-12)
            self.assertEqual(result["scene_support"]["top_z_m"], -.035)
            self.assertEqual(result["rgb_timestamp_ns"], 1000)
            self.assertLess(result["geometry_evidence"]["maximum_corner_reprojection_px"], 1e-10)
            self.assertFalse(result["accepted_for_execution"])
            self.assertNotIn("metric_target_verified", result["metric_evidence"])
            for field, value in (("top_corners_uv",pixels[[0,2,1,3]].tolist()),
                                 ("cube_side_m",.025), ("timestamp_ns",True),
                                 ("camera_frame","raw_left")):
                with self.subTest(field=field), self.assertRaises(ValueError):
                    known_cube_from_board({**observation,field:value})
            wrong = copy.deepcopy(observation)
            wrong["intrinsics"]["width"] = 1280
            path.write_text(json.dumps({"board_cube_observation":wrong}))
            with self.assertRaisesRegex(ValueError, "resolution"):
                load_block_observation(path)
            wrong = {**observation, "board_pose_frame_sha256":"0"*64}
            path.write_text(json.dumps({"board_cube_observation":wrong}))
            with self.assertRaisesRegex(ValueError, "same-frame"):
                load_block_observation(path)
            path.write_text(json.dumps({"board_cube_observation":observation}))
            calibration.write_text("{}")
            with self.assertRaisesRegex(ValueError, "source changed"):
                load_block_observation(path)

    @unittest.skipIf(cv2 is None, "OpenCV is supplied by the vision/MuJoCo environment")
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

    @unittest.skipIf(cv2 is None, "OpenCV is supplied by the vision/MuJoCo environment")
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

    def test_measured_board_to_left_arm_transform(self):
        delta = [-30.0, 200.0, 35.0]  # Synthetic millimetre datum, not personal calibration.
        T_arm_board = measured_board_to_left_arm_transform(delta)

        self.assertEqual(T_arm_board.shape, (4, 4))
        self.assertAlmostEqual(float(np.linalg.det(T_arm_board[:3, :3])), 1.0, places=9)
        np.testing.assert_allclose(T_arm_board[:3, :3] @ T_arm_board[:3, :3].T, np.eye(3), atol=1e-9)

        # Origin of board maps to expected arm frame position:
        p_board_origin = np.array([0.0, 0.0, 0.0, 1.0])
        p_arm = T_arm_board @ p_board_origin
        np.testing.assert_allclose(p_arm[:3], [0.200, -0.030, 0.035], atol=1e-6)

        # Arm origin in board coordinates maps to [0, 0, 0] in arm frame:
        p_arm_in_board = np.array([-0.030, 0.200, 0.035, 1.0])
        np.testing.assert_allclose(T_arm_board @ p_arm_in_board, [0.0, 0.0, 0.0, 1.0], atol=1e-6)

    def test_measured_board_to_left_arm_transform_rejects_invalid(self):
        for invalid in ([np.nan, 201.0, 40.0], [0.0, 0.0], np.ones((4,))):
            with self.assertRaises(VisionTargetError):
                measured_board_to_left_arm_transform(invalid)


if __name__ == "__main__":
    unittest.main()
