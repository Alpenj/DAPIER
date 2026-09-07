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

from box_shoe_scene import box_shoe_scene_contract
from compact_mobile_dual_so101 import (
    COMPACT_HOME_ACTION,
    CompactMobileConfig,
    build_compact_mobile_model,
    compact_mobile_measurements_mm,
    compact_mobile_contract,
    create_compact_mobile_data,
)
from collision_guard import check_bimanual_path
from mobile_dual_so101 import HUMANOID_HOME_ACTION, WAFFLE_TOP_LOCAL_Z_M
from physics_ik import solve_bimanual_position_ik


class CompactMobileDualSO101Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = CompactMobileConfig()
        cls.model, _ = build_compact_mobile_model(cls.config)

    def test_turtlebot_and_twelve_arm_actuators_are_present(self) -> None:
        self.assertEqual(self.model.nu, 12)
        self.assertGreaterEqual(
            mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link"
            ),
            0,
        )

    def test_arm_bases_are_symmetric_on_the_turtlebot_top(self) -> None:
        base_link_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link"
        )
        for side, sign in (("left", 1.0), ("right", -1.0)):
            arm_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_base"
            )
            self.assertEqual(int(self.model.body_parentid[arm_id]), base_link_id)
            self.assertAlmostEqual(
                float(self.model.body_pos[arm_id, 0]), self.config.mount_x_m
            )
            self.assertAlmostEqual(
                float(self.model.body_pos[arm_id, 1]),
                sign * self.config.arm_base_separation_m / 2.0,
            )
            self.assertAlmostEqual(
                float(self.model.body_pos[arm_id, 2]), WAFFLE_TOP_LOCAL_Z_M + 0.008
            )
            np.testing.assert_allclose(
                self.model.body_quat[arm_id],
                np.asarray((1.0, 0.0, 0.0, 0.0)),
                atol=1e-9,
            )

    def test_top_rgbd_is_centered_and_camera_count_is_four(self) -> None:
        camera_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "workspace_depth_camera_body",
        )
        self.assertAlmostEqual(float(self.model.body_pos[camera_body_id, 1]), 0.0)
        data = create_compact_mobile_data(self.model)
        camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "workspace_depth_camera"
        )
        self.assertLessEqual(float(data.cam_xpos[camera_id, 2]), 0.45)
        for name in (
            "front_slam_depth_camera",
            "workspace_depth_camera",
            "left_gripper_camera",
            "right_gripper_camera",
        ):
            self.assertGreaterEqual(
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, name),
                0,
                name,
            )
        self.assertEqual(self.model.ncam, 4)

    def test_all_cameras_face_the_box_from_the_home_pose(self) -> None:
        data = create_compact_mobile_data(self.model)
        target = np.asarray((*self.config.box_center_xy_m, 0.08))
        for name in (
            "front_slam_depth_camera",
            "workspace_depth_camera",
            "left_gripper_camera",
            "right_gripper_camera",
        ):
            camera_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, name
            )
            forward = -data.cam_xmat[camera_id].reshape(3, 3)[:, 2]
            to_target = target - data.cam_xpos[camera_id]
            to_target /= np.linalg.norm(to_target)
            self.assertGreater(float(np.dot(forward, to_target)), 0.97, name)

    def test_wrist_cameras_start_above_the_closed_box(self) -> None:
        data = create_compact_mobile_data(self.model)
        box_top_m = 0.105
        for name in ("left_gripper_camera", "right_gripper_camera"):
            camera_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, name
            )
            self.assertGreater(float(data.cam_xpos[camera_id, 2]), box_top_m, name)
            parent_id = int(self.model.cam_bodyid[camera_id])
            self.assertAlmostEqual(
                float(data.cam_xpos[camera_id, 2] - data.xpos[parent_id, 2]),
                0.040,
            )

    def test_both_wrists_are_reversed_180_degrees_for_top_rgb_mounts(self) -> None:
        for index in (4, 10):
            difference = math.remainder(
                COMPACT_HOME_ACTION[index] - HUMANOID_HOME_ACTION[index],
                2.0 * math.pi,
            )
            self.assertAlmostEqual(abs(difference), math.pi)

    def test_box_and_cuboid_shoe_keep_measured_geometry(self) -> None:
        contract = box_shoe_scene_contract(self.model)
        self.assertEqual(contract["box_outer_size_m"], (0.282, 0.210, 0.105))
        self.assertAlmostEqual(contract["box_yaw_deg"], -90.0)
        self.assertAlmostEqual(contract["cardboard_thickness_m"], 0.0015)
        self.assertFalse(contract["hardware_execution"])

    def test_front_rgbd_faces_the_box_long_side(self) -> None:
        data = create_compact_mobile_data(self.model)
        camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_slam_depth_camera"
        )
        box_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "box_fixture"
        )
        camera_forward = -data.cam_xmat[camera_id].reshape(3, 3)[:, 2]
        box_long_side_normal = data.xmat[box_id].reshape(3, 3)[:, 1]
        self.assertGreater(
            abs(float(np.dot(camera_forward, box_long_side_normal))), 0.999
        )

    def test_contract_records_centerline_and_measurement_boundary(self) -> None:
        contract = compact_mobile_contract(self.model, self.config)
        self.assertEqual(
            contract["layout"], "TURTLEBOT3_CENTERED_RGBD_COMPACT_DUAL_SO101"
        )
        self.assertEqual(contract["camera_count"], 4)
        self.assertTrue(contract["camera_centerline_aligned"])
        self.assertTrue(contract["arm_mount_measurement_required"])
        self.assertTrue(contract["simulator_truth_for_runtime_forbidden"])
        self.assertFalse(contract["hardware_execution"])

    def test_layout_measurements_are_reported_in_millimetres(self) -> None:
        measurements = compact_mobile_measurements_mm(self.model)
        self.assertEqual(measurements["distance_mm"]["arm_base_to_arm_base"], 160.0)
        self.assertEqual(
            measurements["position_world_mm"]["workspace_depth_camera"],
            (-64.0, 0.0, 420.0),
        )
        self.assertTrue(measurements["simulation_values_only"])

    def test_right_arm_pregrasp_ik_passes_compact_collision_guard(self) -> None:
        result = solve_bimanual_position_ik(
            self.model,
            COMPACT_HOME_ACTION,
            {
                "right": (
                    self.config.box_center_xy_m[0] - 0.135,
                    -0.090,
                    0.200,
                )
            },
        )
        self.assertTrue(result.converged)
        assessment = check_bimanual_path(
            self.model,
            COMPACT_HOME_ACTION,
            result.action_rad,
            required_clearance_m=0.005,
        )
        self.assertTrue(assessment.safe, assessment)

    def test_invalid_dimensions_fail_before_model_build(self) -> None:
        invalid = (
            CompactMobileConfig(arm_base_separation_m=0.0),
            CompactMobileConfig(camera_center_z_m=0.0),
            CompactMobileConfig(camera_down_tilt_rad=math.pi / 2.0),
        )
        for config in invalid:
            with self.subTest(config=config), self.assertRaises(ValueError):
                config.validate()


if __name__ == "__main__":
    unittest.main()
