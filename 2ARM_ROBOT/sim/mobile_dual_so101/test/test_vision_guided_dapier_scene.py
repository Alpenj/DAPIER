from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

import mobile_dual_so101  # noqa: E402
from dapier_vision_demo import (  # noqa: E402
    DAPIER_VISION_CAMERA_DOWN_TILT_RAD,
    DAPIER_VISION_CAMERA_PROFILE,
    DAPIER_VISION_DETECTOR,
    build_dapier_vision_env,
    write_vision_artifacts,
)
from shoe_task import SHOE_BODY_NAME  # noqa: E402
from vision_guided_reach import estimate_shoe_from_frame, render_rgbd_frame  # noqa: E402


class VisionGuidedDapierSceneTest(unittest.TestCase):
    def test_front_rgbd_observes_and_reconstructs_default_shoe(self) -> None:
        original_tilt = mobile_dual_so101.TOWER_CAMERA_DOWN_TILT_RAD
        env = build_dapier_vision_env()
        self.assertEqual(
            mobile_dual_so101.TOWER_CAMERA_DOWN_TILT_RAD,
            original_tilt,
            "temporary vision profile must not leak into other model builds",
        )
        env.reset(seed=0)
        frame = render_rgbd_frame(
            env.model,
            env.data,
            width=320,
            height=240,
            camera_name="front_depth_camera",
            target_body_name="tb3_base_link",
        )
        estimate = estimate_shoe_from_frame(
            frame,
            detector=DAPIER_VISION_DETECTOR,
        )

        camera_id = mujoco.mj_name2id(
            env.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            "front_depth_camera",
        )
        rotation = env.data.cam_xmat[camera_id].reshape(3, 3)
        forward = -rotation[:, 2]
        measured_tilt = math.atan2(-float(forward[2]), float(forward[0]))
        self.assertAlmostEqual(measured_tilt, DAPIER_VISION_CAMERA_DOWN_TILT_RAD)

        shoe_id = mujoco.mj_name2id(
            env.model,
            mujoco.mjtObj.mjOBJ_BODY,
            SHOE_BODY_NAME,
        )
        base_id = mujoco.mj_name2id(
            env.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "tb3_base_link",
        )
        world_from_base_rotation = env.data.xmat[base_id].reshape(3, 3)
        base_from_world_rotation = world_from_base_rotation.T
        shoe_in_base = base_from_world_rotation @ (
            env.data.xpos[shoe_id] - env.data.xpos[base_id]
        )
        error_m = math.dist(estimate.position_target_m, shoe_in_base)

        self.assertEqual(
            estimate.source,
            "rgb_detection_plus_aligned_metric_depth",
        )
        self.assertFalse(estimate.ground_truth_used)
        self.assertGreaterEqual(estimate.inlier_pixels, 6)
        self.assertLess(error_m, 0.12)

    def test_review_artifacts_are_sensor_only_and_secret_free(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            report_path = write_vision_artifacts(Path(directory))
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertTrue((Path(directory) / "vision_rgb.png").is_file())
            self.assertTrue((Path(directory) / "vision_mask.png").is_file())
            self.assertEqual(report["camera_profile"], DAPIER_VISION_CAMERA_PROFILE)
            self.assertFalse(report["ground_truth_used_for_target"])
            self.assertFalse(report["control_authorized"])
            self.assertFalse(report["hardware_execution"])
            serialized = json.dumps(report)
            self.assertNotIn("password", serialized.lower())
            self.assertNotIn("private_key", serialized.lower())


if __name__ == "__main__":
    unittest.main()
