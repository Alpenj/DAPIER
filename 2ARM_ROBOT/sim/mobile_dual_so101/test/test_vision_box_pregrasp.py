from __future__ import annotations

from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from compact_mobile_dual_so101 import (  # noqa: E402
    CompactMobileConfig,
    build_compact_mobile_model,
    create_compact_mobile_data,
)
from mobile_dual_so101 import apply_control_as_pose, resolve_so101_model  # noqa: E402
from vision_box_pregrasp import (  # noqa: E402
    CameraCalibration,
    DepthBoxDetectorConfig,
    calibration_from_mujoco,
    estimate_box_pose_from_depth,
    plan_right_pregrasp_from_depth,
    refine_right_pregrasp_with_wrist_rgb,
    render_metric_depth,
    render_rgb,
    detect_front_flap_edge_row,
)


try:
    ASSET = resolve_so101_model()
except FileNotFoundError:
    ASSET = None


@unittest.skipUnless(ASSET is not None, "SO-101 MuJoCo asset is unavailable")
class VisionBoxPregraspTest(unittest.TestCase):
    def render(self, config: CompactMobileConfig):
        assert ASSET is not None
        model, _ = build_compact_mobile_model(config, model_path=ASSET)
        data = create_compact_mobile_data(model)
        calibration = calibration_from_mujoco(
            model, data, "workspace_depth_camera", width=640, height=460
        )
        depth = render_metric_depth(
            model, data, "workspace_depth_camera", width=640, height=460
        )
        return model, data, calibration, depth

    def test_depth_only_pose_tracks_translation_and_rotation(self) -> None:
        for config in (
            CompactMobileConfig(),
            CompactMobileConfig(box_center_xy_m=(0.44, 0.025), box_yaw_deg=-85.0),
            CompactMobileConfig(box_center_xy_m=(0.40, -0.020), box_yaw_deg=-95.0),
        ):
            with self.subTest(config=config):
                model, data, calibration, depth = self.render(config)
                estimate = estimate_box_pose_from_depth(depth, calibration)
                truth_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_BODY, "box_fixture"
                )
                error = np.linalg.norm(
                    np.asarray(estimate.top_center_base_m[:2])
                    - data.xpos[truth_id, :2]
                )
                self.assertLess(error, 0.008)
                self.assertEqual(estimate.source, "depth_points_only")
                self.assertFalse(estimate.simulator_truth_used)
                self.assertGreater(estimate.confidence, 0.5)

    def test_right_pregrasp_is_four_guarded_segments(self) -> None:
        model, data, calibration, depth = self.render(CompactMobileConfig())
        plan = plan_right_pregrasp_from_depth(model, depth, calibration)
        flap_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "box_lid_front_tuck_flap"
        )
        self.assertTrue(plan.accepted)
        self.assertEqual(len(plan.targets_base_m), 4)
        self.assertEqual(len(plan.actions_rad), 4)
        self.assertTrue(all(action[11] == 1.2 for action in plan.actions_rad))
        self.assertLess(
            np.linalg.norm(
                np.asarray(plan.right_front_flap_grasp_base_m)
                - (data.geom_xpos[flap_id] + np.asarray((0.0, -0.09, 0.0)))
            ),
            0.008,
        )
        self.assertLess(plan.targets_base_m[-1][0], data.geom_xpos[flap_id][0])
        self.assertLess(plan.targets_base_m[-1][1], -0.08)
        self.assertTrue(all(residual < 0.0005 for residual in plan.residuals_m))
        self.assertNotEqual(int(model.geom_contype[flap_id]), 0)
        self.assertLess(plan.contact_residual_m, 0.0005)
        self.assertEqual(plan.contact_closed_action_rad[11], 0.2)
        self.assertGreater(plan.front_flap_contact_count, 0)
        self.assertGreaterEqual(plan.minimum_clearance_m, 0.003)
        self.assertFalse(plan.hardware_execution)

    def test_plan_handles_declared_box_pose_randomization(self) -> None:
        for config in (
            CompactMobileConfig(box_center_xy_m=(0.39, -0.03), box_yaw_deg=-95.0),
            CompactMobileConfig(),
            CompactMobileConfig(box_center_xy_m=(0.45, 0.03), box_yaw_deg=-85.0),
        ):
            with self.subTest(config=config):
                model, _, calibration, depth = self.render(config)
                plan = plan_right_pregrasp_from_depth(model, depth, calibration)
                self.assertTrue(plan.accepted)
                self.assertGreater(plan.front_flap_contact_count, 0)

    def test_invalid_or_missing_depth_fails_closed(self) -> None:
        model, _, calibration, depth = self.render(CompactMobileConfig())
        with self.assertRaisesRegex(ValueError, "shape"):
            estimate_box_pose_from_depth(depth[:100], calibration)
        with self.assertRaisesRegex(ValueError, "not enough"):
            estimate_box_pose_from_depth(np.zeros_like(depth), calibration)
        with self.assertRaisesRegex(ValueError, "bounds"):
            estimate_box_pose_from_depth(
                depth,
                calibration,
                DepthBoxDetectorConfig(workspace_x_m=(0.4, 0.3)),
            )
        bad = CameraCalibration(
            calibration.width,
            calibration.height,
            calibration.fx,
            calibration.fy,
            calibration.cx,
            calibration.cy,
            tuple(np.zeros((4, 4)).ravel()),
        )
        with self.assertRaisesRegex(ValueError, "homogeneous"):
            estimate_box_pose_from_depth(depth, bad)

    def test_right_wrist_rgb_reduces_late_box_motion_error(self) -> None:
        model, data, calibration, depth = self.render(CompactMobileConfig())
        plan = plan_right_pregrasp_from_depth(model, depth, calibration)
        data = create_compact_mobile_data(model)
        apply_control_as_pose(model, data, plan.actions_rad[-1])
        desired_row = detect_front_flap_edge_row(
            render_rgb(model, data, "right_gripper_camera")
        )

        moved_model, _, _, _ = self.render(
            CompactMobileConfig(box_center_xy_m=(0.430, 0.0))
        )
        result = refine_right_pregrasp_with_wrist_rgb(
            moved_model,
            plan.actions_rad[-1],
            plan.targets_base_m[-1],
            desired_row,
        )
        self.assertTrue(result.accepted)
        self.assertGreater(abs(result.initial_error_px), 10.0)
        self.assertLess(abs(result.corrected_error_px), 3.0)
        self.assertGreater(result.correction_base_x_m, 0.0)
        self.assertFalse(result.simulator_truth_used)
        self.assertFalse(result.hardware_execution)


if __name__ == "__main__":
    unittest.main()
