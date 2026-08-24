from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:  # pragma: no cover - environment-dependent skip
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

from dual_model import (
    ACTION_NAMES,
    UPSTREAM_MODEL,
    actuator_targets_from_qpos,
    build_model,
    model_names,
    validate_model,
)


class DualModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = build_model()

    def test_upstream_model_is_vendored(self) -> None:
        self.assertTrue(UPSTREAM_MODEL.is_file())
        self.assertEqual(len(list((UPSTREAM_MODEL.parent / "assets").glob("*.stl"))), 14)

    def test_dual_arm_dimensions(self) -> None:
        self.assertEqual(self.model.nq, 14)
        self.assertEqual(self.model.nv, 14)
        self.assertEqual(self.model.nu, 12)
        self.assertEqual(self.model.njnt, 14)
        self.assertEqual(self.model.neq, 2)

    def test_action_order_is_namespaced(self) -> None:
        actual = model_names(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, self.model.nu
        )
        self.assertEqual(actual, ACTION_NAMES)

    def test_initial_targets_hold_open_grippers(self) -> None:
        targets = actuator_targets_from_qpos(self.model, self.model.qpos0)
        self.assertAlmostEqual(targets[5], -0.57)
        self.assertAlmostEqual(targets[11], -0.57)

        data = mujoco.MjData(self.model)
        data.ctrl[:] = targets
        finger_joint_names = (
            "left_gripper_left",
            "left_gripper_right",
            "right_gripper_left",
            "right_gripper_right",
        )
        qpos_addresses = [
            self.model.jnt_qposadr[
                mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                )
            ]
            for joint_name in finger_joint_names
        ]
        for _ in range(1000):
            mujoco.mj_step(self.model, data)
        for qpos_address in qpos_addresses:
            self.assertAlmostEqual(
                data.qpos[qpos_address],
                self.model.qpos0[qpos_address],
                places=10,
            )

    def test_headless_rollout_remains_finite(self) -> None:
        report = validate_model(self.model, smoke_steps=250)
        self.assertTrue(report["finite_state"])
        self.assertEqual(report["smoke_steps"], 250)
        self.assertEqual(report["disabled_collision_geoms"], 46)
        self.assertEqual(report["contacts_after_smoke"], 0)

    def test_primitive_collision_overlay_is_optional(self) -> None:
        mesh_only_model = build_model(primitive_collisions=False)
        self.assertEqual(self.model.ngeom - mesh_only_model.ngeom, 12)

    def test_invalid_mount_separation_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be positive"):
            build_model(mount_separation_m=0.0)


if __name__ == "__main__":
    unittest.main()
