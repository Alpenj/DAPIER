from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np


RESEARCH_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = RESEARCH_ROOT.parents[1]
sys.path.insert(0, str(RESEARCH_ROOT / "src"))

from dapier_research.vision_target import (  # noqa: E402
    PixelDetection,
    PinholeIntrinsics,
    VisionTargetError,
    approach_target_from_estimate,
    estimate_target_from_rgbd,
)


class VisionTargetTest(unittest.TestCase):
    def setUp(self) -> None:
        self.intrinsics = PinholeIntrinsics(
            width=7,
            height=5,
            fx=100.0,
            fy=100.0,
            cx=3.0,
            cy=2.0,
        )
        self.rgb = np.zeros((5, 7, 3), dtype=np.uint8)
        self.mask = np.zeros((5, 7), dtype=bool)
        self.mask[1:4, 2:5] = True
        self.depth = np.full((5, 7), np.nan, dtype=np.float32)
        self.depth[self.mask] = 2.0
        self.detection = PixelDetection(
            label="shoe",
            mask=self.mask,
            confidence=0.9,
            detector="unit_test_rgb_detector",
        )

    def estimate(self, **overrides):
        values = {
            "rgb": self.rgb,
            "depth_m": self.depth,
            "detection": self.detection,
            "intrinsics": self.intrinsics,
            "target_from_optical": np.eye(4),
            "timestamp_ns": 123,
            "camera_frame": "camera_optical",
            "target_frame": "base_link",
        }
        values.update(overrides)
        return estimate_target_from_rgbd(**values)

    def test_centered_mask_projects_to_optical_forward_axis(self) -> None:
        estimate = self.estimate()
        self.assertAlmostEqual(estimate.position_optical_m[0], 0.0, places=8)
        self.assertAlmostEqual(estimate.position_optical_m[1], 0.0, places=8)
        self.assertAlmostEqual(estimate.position_optical_m[2], 2.0, places=8)
        self.assertEqual(estimate.position_target_m, estimate.position_optical_m)
        self.assertFalse(estimate.ground_truth_used)
        self.assertFalse(estimate.control_authorized)
        self.assertFalse(estimate.hardware_execution)
        self.assertGreater(estimate.confidence, 0.85)

    def test_calibrated_rotation_and_translation_are_applied(self) -> None:
        transform = np.eye(4)
        transform[:3, :3] = np.array(
            (
                (0.0, 0.0, 1.0),
                (-1.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
            )
        )
        transform[:3, 3] = (0.5, -0.25, 1.0)
        estimate = self.estimate(target_from_optical=transform)
        self.assertAlmostEqual(estimate.position_target_m[0], 2.5, places=8)
        self.assertAlmostEqual(estimate.position_target_m[1], -0.25, places=8)
        self.assertAlmostEqual(estimate.position_target_m[2], 1.0, places=8)

    def test_depth_outlier_is_rejected_before_centroid(self) -> None:
        depth = self.depth.copy()
        depth[1, 2] = 4.5
        estimate = self.estimate(depth_m=depth)
        self.assertEqual(estimate.valid_depth_pixels, 9)
        self.assertEqual(estimate.inlier_pixels, 8)
        self.assertAlmostEqual(estimate.median_depth_m, 2.0, places=8)
        self.assertAlmostEqual(estimate.position_target_m[2], 2.0, places=8)

    def test_missing_depth_and_privileged_detection_fail_closed(self) -> None:
        missing = np.full_like(self.depth, np.nan)
        with self.assertRaisesRegex(VisionTargetError, "enough valid") as raised:
            self.estimate(depth_m=missing)
        self.assertEqual(raised.exception.code, "insufficient_depth")

        privileged = PixelDetection(
            label="shoe",
            mask=self.mask,
            confidence=1.0,
            detector="mujoco_segmentation_id",
            uses_privileged_labels=True,
        )
        with self.assertRaisesRegex(VisionTargetError, "segmentation") as raised:
            self.estimate(detection=privileged)
        self.assertEqual(raised.exception.code, "privileged_detection")

    def test_approach_target_keeps_authorization_false(self) -> None:
        estimate = self.estimate()
        proposal = approach_target_from_estimate(
            estimate,
            offset_target_m=(0.0, 0.0, 0.10),
            minimum_confidence=0.5,
        )
        self.assertAlmostEqual(proposal.position_m[2], 2.10, places=8)
        self.assertFalse(proposal.control_authorized)
        self.assertFalse(proposal.hardware_execution)

    def test_low_confidence_cannot_enter_planning(self) -> None:
        detection = PixelDetection(
            label="shoe",
            mask=self.mask,
            confidence=0.1,
            detector="weak_rgb_detector",
        )
        estimate = self.estimate(detection=detection)
        with self.assertRaisesRegex(VisionTargetError, "confidence") as raised:
            approach_target_from_estimate(estimate, minimum_confidence=0.5)
        self.assertEqual(raised.exception.code, "low_confidence")

    def test_intrinsics_from_fovy_use_square_pixels(self) -> None:
        intrinsics = PinholeIntrinsics.from_vertical_fov(
            width=640,
            height=480,
            vertical_fov_deg=45.5,
        )
        expected = 240.0 / math.tan(math.radians(45.5) / 2.0)
        self.assertAlmostEqual(intrinsics.fx, expected)
        self.assertAlmostEqual(intrinsics.fy, expected)
        self.assertEqual((intrinsics.cx, intrinsics.cy), (319.5, 239.5))

    def test_runtime_planner_source_has_no_object_ground_truth_dependency(self) -> None:
        planner = (
            REPO_ROOT
            / "2ARM_ROBOT"
            / "sim"
            / "mobile_dual_so101"
            / "vision_guided_reach.py"
        )
        text = planner.read_text(encoding="utf-8")
        forbidden = (
            "ground_truth_observation",
            "SHOE_BODY_NAME",
            "mujoco_segmentation",
            "uses_privileged_labels=True",
        )
        for fragment in forbidden:
            with self.subTest(fragment=fragment):
                self.assertNotIn(fragment, text)


if __name__ == "__main__":
    unittest.main()
