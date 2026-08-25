from __future__ import annotations

from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

from collision_guard import (
    bimanual_geom_pairs,
    check_bimanual_path,
    minimum_protected_clearance,
)
from mobile_dual_so101 import (
    HUMANOID_HOME_ACTION,
    apply_control_as_pose,
    build_model,
)


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


class CollisionGuardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model, _ = build_model(arm_mount_height_m=0.30)

    def test_home_path_is_clear_and_never_authorizes_hardware(self) -> None:
        result = check_bimanual_path(
            self.model,
            HUMANOID_HOME_ACTION,
            HUMANOID_HOME_ACTION,
        )
        self.assertTrue(result.safe)
        self.assertGreaterEqual(result.minimum_clearance_m, 0.03)
        self.assertFalse(result.control_authorized)
        self.assertFalse(result.hardware_execution)

    def test_interpolated_arm_arm_collision_is_rejected(self) -> None:
        result = check_bimanual_path(
            self.model,
            HUMANOID_HOME_ACTION,
            UNSAFE_BIMANUAL_TARGET,
        )
        self.assertFalse(result.safe)
        self.assertLess(result.minimum_clearance_m, 0.03)
        self.assertFalse(result.hardware_dispatch_authorized)
        self.assertFalse(result.executed_action)

        data = mujoco.MjData(self.model)
        apply_control_as_pose(self.model, data, UNSAFE_BIMANUAL_TARGET)
        clearance, first, second = minimum_protected_clearance(
            self.model,
            data,
            bimanual_geom_pairs(self.model),
        )
        self.assertLess(clearance, 0.03)
        first_body = self.model.body(
            int(self.model.geom_bodyid[first])
        ).name
        second_body = self.model.body(
            int(self.model.geom_bodyid[second])
        ).name
        self.assertNotEqual(
            first_body.startswith("left_"),
            second_body.startswith("left_"),
        )

    def test_invalid_clearance_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            check_bimanual_path(
                self.model,
                HUMANOID_HOME_ACTION,
                HUMANOID_HOME_ACTION,
                required_clearance_m=0.0,
            )


if __name__ == "__main__":
    unittest.main()
