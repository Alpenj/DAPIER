from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

from mobile_dual_model import ACTION_NAMES, build_model, validate_model


class MobileDualModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = build_model()

    def test_combined_dimensions_and_action_order(self) -> None:
        self.assertEqual(self.model.nq, 16)
        self.assertEqual(self.model.nv, 16)
        self.assertEqual(self.model.nu, 12)
        actual = tuple(
            mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index
            )
            for index in range(self.model.nu)
        )
        self.assertEqual(actual, ACTION_NAMES)

    def test_arms_are_children_of_base_link(self) -> None:
        base_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link"
        )
        for body_name in ("left_base_assembly", "right_base_assembly"):
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body_name
            )
            self.assertEqual(self.model.body_parentid[body_id], base_id)

    def test_wheels_have_no_actuator(self) -> None:
        actuator_names = tuple(
            mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index
            )
            for index in range(self.model.nu)
        )
        self.assertFalse(any("wheel" in name for name in actuator_names))

    def test_stationary_smoke(self) -> None:
        report = validate_model(self.model, smoke_steps=1000)
        self.assertTrue(report["finite_state"])
        self.assertTrue(report["base_fixed"])
        self.assertFalse(report["wheel_actuators_present"])
        self.assertFalse(report["hardware_execution"])
        self.assertLess(report["max_abs_qvel"], 1e-9)

    def test_invalid_arm_separation_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be positive"):
            build_model(arm_mount_separation_m=0.0)


if __name__ == "__main__":
    unittest.main()
