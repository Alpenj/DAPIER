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

from waffle_pi_model import BASE_URDF, WHEEL_JOINT_NAMES, build_model, validate_model


class WafflePiModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = build_model()

    def test_portable_assets_exist(self) -> None:
        self.assertTrue(BASE_URDF.is_file())
        upstream = PROJECT_DIR / "upstream"
        self.assertTrue((upstream / "LICENSE").is_file())
        meshes = upstream / "turtlebot3_description" / "meshes"
        expected = (
            meshes / "bases" / "waffle_pi_base.stl",
            meshes / "wheels" / "left_tire.stl",
            meshes / "wheels" / "right_tire.stl",
            meshes / "sensors" / "lds.stl",
        )
        self.assertTrue(all(path.is_file() for path in expected))

    def test_base_link_and_wheel_joints_survive_conversion(self) -> None:
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link"
        )
        self.assertGreaterEqual(body_id, 0)
        joints = tuple(
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, index)
            for index in range(self.model.njnt)
        )
        self.assertEqual(joints, WHEEL_JOINT_NAMES)

    def test_no_wheel_actuator_is_added(self) -> None:
        self.assertEqual(self.model.nu, 0)

    def test_stationary_smoke_remains_finite(self) -> None:
        report = validate_model(self.model, smoke_steps=1000)
        self.assertTrue(report["finite_state"])
        self.assertFalse(report["wheel_actuators_present"])


if __name__ == "__main__":
    unittest.main()
