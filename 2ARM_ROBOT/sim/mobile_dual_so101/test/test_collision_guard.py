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
    protected_geom_pairs,
)
from mobile_dual_so101 import (
    HUMANOID_HOME_ACTION,
    TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
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
RESTORED_BASE_CAMERA_CLEARANCE_M = 0.10


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

    def test_step_support_home_protects_base_column_and_bare_camera(self) -> None:
        tower_model, _ = build_model(
            arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
            mount_layout="tower",
        )
        pairs = protected_geom_pairs(tower_model)
        protected_names = {
            tower_model.geom(geom_id).name
            for pair in pairs
            for geom_id in pair
            if tower_model.geom(geom_id).name
        }
        self.assertTrue(
            {
                "tb3_base_link_collision",
                "depth_camera_collision",
                "semi_support_column_collision",
                "tower_camera_mast_collision",
                "tower_camera_interface_plate_collision",
            }.issubset(protected_names)
        )
        self.assertNotIn("semi_support_camera_post_collision", protected_names)
        self.assertNotIn("tower_camera_boom_collision", protected_names)
        result = check_bimanual_path(
            tower_model,
            HUMANOID_HOME_ACTION,
            HUMANOID_HOME_ACTION,
        )
        self.assertTrue(result.safe)
        self.assertGreaterEqual(result.minimum_clearance_m, 0.03)
        camera_id = mujoco.mj_name2id(
            tower_model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "depth_camera_collision",
        )
        camera_pairs = tuple(pair for pair in pairs if camera_id in pair)
        camera_data = mujoco.MjData(tower_model)
        apply_control_as_pose(
            tower_model, camera_data, HUMANOID_HOME_ACTION
        )
        camera_clearance, _, _ = minimum_protected_clearance(
            tower_model,
            camera_data,
            camera_pairs,
            distance_cap_m=2.0,
        )
        # The restored SO-101 Waveshare mounting plate is the intentional
        # nearest camera pair. Preserve at least a 100 mm nominal envelope;
        # the operational protected-path gate remains 30 mm above.
        self.assertGreaterEqual(
            camera_clearance,
            RESTORED_BASE_CAMERA_CLEARANCE_M,
        )
        self.assertFalse(result.hardware_execution)

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
