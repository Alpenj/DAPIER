from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

from mobile_dual_so101 import (
    ACTION_NAMES,
    DEFAULT_ARM_MOUNT_X_M,
    HUMANOID_HOME_ACTION,
    PRINTED_MOUNT_ESTIMATED_MASS_KG,
    WAFFLE_TOP_LOCAL_Z_M,
    WORKSPACE_DEPTH_CAMERA_COLLISION_ORIGIN_M,
    WORKSPACE_DEPTH_CAMERA_DOWN_TILT_RAD,
    WORKSPACE_DEPTH_CAMERA_MASS_KG,
    WORKSPACE_DEPTH_CAMERA_SIZE_M,
    _restore_home_pose_after_reset,
    apply_control_as_pose,
    build_model,
    model_provenance,
    pose_report,
    resolve_so101_model,
    validate_model,
)


TEST_MOUNT_HEIGHT_M = 0.42


class MobileDualSO101Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model, cls.source = build_model(
            arm_mount_height_m=TEST_MOUNT_HEIGHT_M
        )

    def test_combined_dimensions_and_action_order(self) -> None:
        self.assertEqual((self.model.nq, self.model.nv), (14, 14))
        self.assertEqual(self.model.nu, 12)
        actual = tuple(
            mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index
            )
            for index in range(self.model.nu)
        )
        self.assertEqual(actual, ACTION_NAMES)

    def test_arms_are_namespaced_children_of_base(self) -> None:
        base_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link"
        )
        for side in ("left", "right"):
            arm_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_base"
            )
            self.assertEqual(int(self.model.body_parentid[arm_id]), base_id)

    def test_printed_mount_is_a_closed_vertical_flange_torsion_box(self) -> None:
        for name in (
            "mount_deck_left",
            "mount_deck_right",
            "left_vertical_arm_mount_plate",
            "right_vertical_arm_mount_plate",
            "left_torso_gusset_rear",
            "left_torso_gusset_front",
            "right_torso_gusset_rear",
            "right_torso_gusset_front",
            "torso_front_panel",
            "torso_rear_panel",
            "torso_top_panel",
            "depth_camera_mount_pad_left",
            "depth_camera_mount_pad_right",
        ):
            geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, name
            )
            self.assertGreaterEqual(geom_id, 0)
            self.assertEqual(int(self.model.geom_contype[geom_id]), 0)
            self.assertEqual(int(self.model.geom_conaffinity[geom_id]), 0)
        mount_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "printed_mount_structure",
        )
        self.assertAlmostEqual(
            float(self.model.body_mass[mount_id]),
            PRINTED_MOUNT_ESTIMATED_MASS_KG,
        )
        left_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "left_vertical_arm_mount_plate",
        )
        right_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "right_vertical_arm_mount_plate",
        )
        self.assertAlmostEqual(
            float(self.model.geom_pos[left_id, 1]),
            -float(self.model.geom_pos[right_id, 1]),
        )
        self.assertAlmostEqual(
            float(self.model.geom_pos[left_id, 1])
            + float(self.model.geom_size[left_id, 1]),
            0.100,
        )
        self.assertAlmostEqual(
            float(self.model.geom_pos[right_id, 1])
            - float(self.model.geom_size[right_id, 1]),
            -0.100,
        )

        for side in ("left", "right"):
            deck_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                f"mount_deck_{side}",
            )
            deck_bottom = float(
                self.model.geom_pos[deck_id, 2]
                - self.model.geom_size[deck_id, 2]
            )
            self.assertAlmostEqual(deck_bottom, WAFFLE_TOP_LOCAL_Z_M)

    def test_printed_mount_uses_hidden_simple_collision_proxies(self) -> None:
        for name in (
            "printed_mount_deck_collision",
            "printed_torso_collision",
            "printed_camera_mount_collision",
        ):
            geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, name
            )
            self.assertGreaterEqual(geom_id, 0, name)
            self.assertEqual(
                int(self.model.geom_type[geom_id]),
                mujoco.mjtGeom.mjGEOM_BOX,
            )
            self.assertEqual(int(self.model.geom_contype[geom_id]), 1)
            self.assertEqual(int(self.model.geom_conaffinity[geom_id]), 1)
            self.assertEqual(int(self.model.geom_group[geom_id]), 3)
            self.assertAlmostEqual(float(self.model.geom_rgba[geom_id, 3]), 0.0)

    def test_original_camera_is_replaced_by_r77_depth_camera(self) -> None:
        self.assertEqual(
            mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, "tb3_camera_link"
            ),
            -1,
        )
        camera_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "workspace_depth_camera_body",
        )
        camera_geom_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "workspace_depth_camera_collision",
        )
        camera_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            "workspace_depth_camera",
        )
        self.assertGreaterEqual(camera_body_id, 0)
        self.assertGreaterEqual(camera_geom_id, 0)
        self.assertGreaterEqual(camera_id, 0)
        self.assertAlmostEqual(
            float(self.model.body_mass[camera_body_id]),
            WORKSPACE_DEPTH_CAMERA_MASS_KG,
        )
        for actual, expected in zip(
            self.model.geom_size[camera_geom_id],
            (value / 2.0 for value in WORKSPACE_DEPTH_CAMERA_SIZE_M),
            strict=True,
        ):
            self.assertAlmostEqual(float(actual), expected)
        for actual, expected in zip(
            self.model.geom_pos[camera_geom_id],
            WORKSPACE_DEPTH_CAMERA_COLLISION_ORIGIN_M,
            strict=True,
        ):
            self.assertAlmostEqual(float(actual), expected)
        for value in self.model.cam_pos[camera_id]:
            self.assertAlmostEqual(float(value), 0.0)
        self.assertEqual(int(self.model.geom_contype[camera_geom_id]), 1)

        data = mujoco.MjData(self.model)
        apply_control_as_pose(self.model, data, HUMANOID_HOME_ACTION)
        view = -data.cam_xmat[camera_id].reshape(3, 3)[:, 2]
        down_tilt = math.atan2(-float(view[2]), float(view[0]))
        self.assertAlmostEqual(
            down_tilt,
            WORKSPACE_DEPTH_CAMERA_DOWN_TILT_RAD,
        )

    def test_lidar_is_removed_by_default_but_available_for_comparison(self) -> None:
        self.assertEqual(
            mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_scan"
            ),
            -1,
        )
        with_lidar, _ = build_model(
            arm_mount_height_m=TEST_MOUNT_HEIGHT_M,
            include_lidar=True,
        )
        self.assertGreaterEqual(
            mujoco.mj_name2id(
                with_lidar, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_scan"
            ),
            0,
        )

    def test_holder_twists_make_mirrored_arms_and_downward_grippers(self) -> None:
        data = mujoco.MjData(self.model)
        apply_control_as_pose(self.model, data, HUMANOID_HOME_ACTION)
        positions = {}
        for side in ("left", "right"):
            shoulder_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_shoulder"
            )
            site_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_gripperframe"
            )
            shoulder = data.xpos[shoulder_id].copy()
            gripper = data.site_xpos[site_id].copy()
            positions[side] = (shoulder, gripper)
            self.assertGreater(float(shoulder[2] - gripper[2]), 0.20)
            self.assertGreater(float(gripper[2]), 0.0)
            approach_axis = data.site_xmat[site_id].reshape(3, 3)[:, 0]
            self.assertLess(float(approach_axis[2]), -0.999)
        left_shoulder, left_gripper = positions["left"]
        right_shoulder, right_gripper = positions["right"]
        self.assertAlmostEqual(
            float(left_shoulder[1]), -float(right_shoulder[1]), places=3
        )
        self.assertAlmostEqual(
            float((left_gripper[0] + right_gripper[0]) / 2.0),
            DEFAULT_ARM_MOUNT_X_M,
            places=3,
        )
        self.assertAlmostEqual(float(left_gripper[1]), -float(right_gripper[1]), places=3)
        self.assertAlmostEqual(float(left_gripper[2]), float(right_gripper[2]), places=3)
        self.assertGreater(abs(float(left_gripper[1])), abs(float(left_shoulder[1])))
        self.assertGreater(abs(float(right_gripper[1])), abs(float(right_shoulder[1])))

    def test_pose_editor_changes_simulator_qpos_only(self) -> None:
        data = mujoco.MjData(self.model)
        target = list(HUMANOID_HOME_ACTION)
        target[1] = 0.25
        target[11] = 0.5
        apply_control_as_pose(self.model, data, target)
        for actuator_id, expected in enumerate(target):
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            qpos_address = int(self.model.jnt_qposadr[joint_id])
            self.assertAlmostEqual(float(data.qpos[qpos_address]), expected)
        report = pose_report(data, self.source)
        for field in (
            "published",
            "control_authorized",
            "hardware_dispatch_authorized",
            "executed_action",
            "hardware_execution",
        ):
            self.assertFalse(report[field])

    def test_mujoco_reset_restores_recorded_home_pose_and_control(self) -> None:
        expected = tuple(float(value) for value in HUMANOID_HOME_ACTION)
        data = mujoco.MjData(self.model)
        data.qpos[:] = 0.0
        data.ctrl[:] = 0.5
        mujoco.mj_resetData(self.model, data)
        self.assertTrue(
            _restore_home_pose_after_reset(
                self.model, data, HUMANOID_HOME_ACTION
            )
        )
        self.assertEqual(tuple(float(value) for value in data.ctrl), expected)
        self.assertEqual(
            tuple(
                float(data.qpos[int(self.model.jnt_qposadr[int(joint_id)])])
                for joint_id in self.model.actuator_trnid[:, 0]
            ),
            expected,
        )

    def test_rounded_slider_endpoint_is_clamped_without_exit(self) -> None:
        data = mujoco.MjData(self.model)
        apply_control_as_pose(self.model, data, HUMANOID_HOME_ACTION)
        data.ctrl[1] = -1.92
        apply_control_as_pose(self.model, data)
        lower = float(self.model.actuator_ctrlrange[1, 0])
        self.assertEqual(float(data.ctrl[1]), lower)
        joint_id = int(self.model.actuator_trnid[1, 0])
        self.assertEqual(
            float(data.qpos[int(self.model.jnt_qposadr[joint_id])]),
            lower,
        )

        explicit = list(HUMANOID_HOME_ACTION)
        explicit[1] = -1.92
        with self.assertRaisesRegex(ValueError, "outside"):
            apply_control_as_pose(self.model, data, explicit)

        canonical_closed = list(HUMANOID_HOME_ACTION)
        canonical_closed[5] = -0.1745329776
        apply_control_as_pose(self.model, data, canonical_closed)
        self.assertEqual(
            float(data.ctrl[5]),
            float(self.model.actuator_ctrlrange[5, 0]),
        )

    def test_model_keeps_so101_joint_ranges(self) -> None:
        expected_gripper = (-0.17453, 1.74533)
        for side in ("left", "right"):
            actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{side}_gripper"
            )
            actual = tuple(float(value) for value in self.model.actuator_ctrlrange[actuator_id])
            self.assertEqual(actual, expected_gripper)

    def test_model_provenance_is_reported_without_mutating_source(self) -> None:
        provenance = model_provenance(self.source)
        self.assertEqual(len(provenance["model_sha256"]), 64)
        self.assertIsInstance(provenance["matches_recorded_upstream"], bool)

    def test_invalid_mount_and_missing_explicit_model_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be positive"):
            build_model(arm_mount_height_m=0.0)
        with self.assertRaises(FileNotFoundError):
            resolve_so101_model(Path("/tmp/dapier-missing-so101.xml"))

    def test_stationary_smoke(self) -> None:
        report = validate_model(
            self.model,
            source=self.source,
            smoke_steps=200,
        )
        self.assertTrue(report["finite_state"])
        self.assertFalse(report["wheel_actuators_present"])
        self.assertFalse(report["lidar_present"])
        self.assertFalse(report["hardware_execution"])
        self.assertIn("model_sha256", report)


if __name__ == "__main__":
    unittest.main()
