from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

from shoe_task import (
    ACTION_NAMES,
    OBSERVATION_NAMES,
    SHOE_BODY_NAME,
    SHOE_FREE_JOINT_NAME,
    ShoeTaskConfig,
    ShoeTaskEnv,
    UnsafeActionError,
    ground_truth_observation,
    task_metrics,
    task_reachability,
    validate_shoe_task,
)
from mobile_dual_so101 import HUMANOID_HOME_ACTION, apply_control_as_pose


UNSAFE_BIMANUAL_TARGET = (
    0.195731791341601,
    1.6758398119082343,
    -0.1950753295290686,
    -0.5266413229575377,
    0.3401861733712628,
    0.4411373258803475,
    -0.485894097852944,
    0.5940617023869152,
    0.4596671975241349,
    0.45149208438578126,
    -1.875765584091561,
    1.0153142196339973,
)

class ShoeTaskTest(unittest.TestCase):
    def setUp(self) -> None:
        self.env = ShoeTaskEnv()
        self.observation, self.reset_info = self.env.reset(seed=7)

    def test_model_contains_one_free_shoe(self) -> None:
        self.assertEqual(self.env.model.nq, 21)
        self.assertEqual(self.env.model.nv, 20)
        self.assertEqual(self.env.model.nu, 12)
        self.assertEqual(self.env.model.njnt, 15)
        self.assertGreaterEqual(
            mujoco.mj_name2id(
                self.env.model, mujoco.mjtObj.mjOBJ_BODY, SHOE_BODY_NAME
            ),
            0,
        )
        joint_id = mujoco.mj_name2id(
            self.env.model, mujoco.mjtObj.mjOBJ_JOINT, SHOE_FREE_JOINT_NAME
        )
        self.assertEqual(
            self.env.model.jnt_type[joint_id], mujoco.mjtJoint.mjJNT_FREE
        )
        self.assertEqual(self.env.config.mount_layout, "tower")
        self.assertGreaterEqual(
            mujoco.mj_name2id(
                self.env.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                "assembled_support_upper_visual",
            ),
            0,
        )

    def test_ground_truth_observation_contract(self) -> None:
        self.assertTrue(self.observation["ground_truth"])
        self.assertEqual(self.observation["frame"], "map_sim_world")
        self.assertEqual(tuple(self.observation["vector_names"]), OBSERVATION_NAMES)
        self.assertEqual(len(self.observation["vector"]), 21)
        self.assertTrue(all(math.isfinite(value) for value in self.observation["vector"]))
        self.assertEqual(
            self.observation,
            ground_truth_observation(self.env.model, self.env.data),
        )
        self.assertFalse(self.reset_info["hardware_execution"])
        self.assertIn("left_shoulder_pan_rad", OBSERVATION_NAMES)
        self.assertNotIn("left_base_rad", OBSERVATION_NAMES)

    def test_default_floor_shoe_is_reported_outside_tower_reach(self) -> None:
        report = task_reachability(self.env.model, self.env.data)
        self.assertFalse(report["inside_distance_envelope"])
        self.assertGreater(report["nearest_shoulder_distance_m"], 0.48)
        self.assertLess(report["nearest_shoulder_distance_m"], 0.50)
        self.assertAlmostEqual(
            report["nearest_shoulder_distance_m"], 0.497476699, places=6
        )

    def test_hold_action_is_simulation_only(self) -> None:
        hold = tuple(float(value) for value in self.env.data.ctrl)
        _, reward, terminated, truncated, info = self.env.step(hold)
        self.assertTrue(math.isfinite(reward))
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertFalse(info["hardware_execution"])

    def test_action_outside_model_range_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "outside actuator range"):
            self.env.step([100.0] * len(ACTION_NAMES))

    def test_collision_target_is_rejected_before_physics_advances(self) -> None:
        before_time = float(self.env.data.time)
        before_ctrl = tuple(float(value) for value in self.env.data.ctrl)
        with self.assertRaises(UnsafeActionError) as raised:
            self.env.step(UNSAFE_BIMANUAL_TARGET)
        self.assertLess(
            raised.exception.assessment.minimum_clearance_m,
            0.03,
        )
        self.assertEqual(float(self.env.data.time), before_time)
        self.assertEqual(tuple(float(value) for value in self.env.data.ctrl), before_ctrl)

    def test_invalid_action_dimension_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "expected 12 actions"):
            self.env.step([0.0] * 11)

    def test_success_requires_lift_near_a_gripper(self) -> None:
        apply_control_as_pose(
            self.env.model,
            self.env.data,
            HUMANOID_HOME_ACTION,
        )
        joint_id = mujoco.mj_name2id(
            self.env.model, mujoco.mjtObj.mjOBJ_JOINT, SHOE_FREE_JOINT_NAME
        )
        qpos_address = int(self.env.model.jnt_qposadr[joint_id])
        gripper_id = mujoco.mj_name2id(
            self.env.model, mujoco.mjtObj.mjOBJ_SITE, "left_gripperframe"
        )
        target = [float(value) for value in self.env.data.site_xpos[gripper_id]]
        target[2] = max(target[2], self.env.config.success_height_m + 0.01)
        self.env.data.qpos[qpos_address : qpos_address + 3] = target
        self.env.data.qvel[:] = 0.0
        mujoco.mj_forward(self.env.model, self.env.data)
        metrics = task_metrics(self.env.model, self.env.data)
        self.assertTrue(metrics["lifted"])
        self.assertTrue(metrics["near_gripper"])
        self.assertTrue(metrics["success"])

    def test_invalid_configuration_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "frame_skip"):
            ShoeTaskEnv(ShoeTaskConfig(frame_skip=0))
        with self.assertRaisesRegex(ValueError, "mount_layout"):
            ShoeTaskEnv(ShoeTaskConfig(mount_layout="unknown"))

    def test_headless_smoke(self) -> None:
        report = validate_shoe_task(smoke_steps=50)
        self.assertTrue(report["finite_observation"])
        self.assertTrue(report["ground_truth"])
        self.assertTrue(report["state_imitation_contract_ready"])
        self.assertFalse(report["default_floor_shoe_reachable"])
        self.assertEqual(report["mount_layout"], "tower")
        self.assertFalse(report["hardware_execution"])


if __name__ == "__main__":
    unittest.main()
