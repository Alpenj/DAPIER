from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

from shoe_task import SHOE_BODY_NAME, ShoeTaskEnv  # noqa: E402
from vision_guided_reach import (  # noqa: E402
    SimBlueShoeDetector,
    estimate_shoe_from_frame,
    render_rgbd_frame,
)


class VisionGuidedDapierSceneTest(unittest.TestCase):
    def test_front_rgbd_observes_and_reconstructs_default_shoe(self) -> None:
        env = ShoeTaskEnv()
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
            detector=SimBlueShoeDetector(),
        )

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


if __name__ == "__main__":
    unittest.main()
