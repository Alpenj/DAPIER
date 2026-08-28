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
    raise unittest.SkipTest(
        "MuJoCo is not installed in this Python environment"
    ) from error

from mobile_dual_so101 import (
    DEPTH_CAMERA_SIZE_M,
    HUMANOID_HOME_ACTION,
    TOWER_CAMERA_DOWN_TILT_RAD,
    TOWER_CAMERA_HEIGHT_ABOVE_ARM_M,
    TOWER_MOUNT_ESTIMATED_MASS_KG,
    TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
    TOWER_RECOMMENDED_ARM_MOUNT_X_M,
    WAFFLE_TOP_LOCAL_Z_M,
    apply_control_as_pose,
    build_model,
    model_names,
    validate_model,
)


class TowerLayoutTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model, cls.source = build_model(
            arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
            mount_layout="tower",
        )
        cls.data = mujoco.MjData(cls.model)
        apply_control_as_pose(cls.model, cls.data, HUMANOID_HOME_ACTION)

    def geom_id(self, name: str) -> int:
        result = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, name
        )
        self.assertGreaterEqual(result, 0, name)
        return result

    def test_two_towers_share_a_deck_and_provisional_mass_budget(self) -> None:
        mount_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "tower_mount_structure",
        )
        self.assertGreaterEqual(mount_id, 0)
        self.assertAlmostEqual(
            float(self.model.body_mass[mount_id]),
            TOWER_MOUNT_ESTIMATED_MASS_KG,
        )

        left = self.geom_id("tower_left_mast_visual")
        right = self.geom_id("tower_right_mast_visual")
        self.assertAlmostEqual(
            float(self.model.geom_pos[left, 1]),
            -float(self.model.geom_pos[right, 1]),
        )
        deck = self.geom_id("tower_common_deck_visual")
        deck_bottom = float(
            self.model.geom_pos[deck, 2] - self.model.geom_size[deck, 2]
        )
        self.assertAlmostEqual(deck_bottom, WAFFLE_TOP_LOCAL_Z_M)

    def test_hidden_collision_proxies_are_present_without_visual_clutter(self) -> None:
        for name in (
            "tower_common_deck_collision",
            "tower_left_mast_collision",
            "tower_right_mast_collision",
            "tower_camera_crossbar_collision",
        ):
            geom_id = self.geom_id(name)
            self.assertEqual(int(self.model.geom_contype[geom_id]), 1)
            self.assertEqual(int(self.model.geom_conaffinity[geom_id]), 1)
            self.assertEqual(int(self.model.geom_group[geom_id]), 3)
            self.assertAlmostEqual(float(self.model.geom_rgba[geom_id, 3]), 0.0)

    def test_depth_camera_is_centered_between_towers_and_clears_both_sides(
        self,
    ) -> None:
        camera_body = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "depth_camera_body"
        )
        self.assertGreaterEqual(camera_body, 0)
        camera_position = self.model.body_pos[camera_body]
        self.assertAlmostEqual(float(camera_position[1]), 0.0)
        self.assertAlmostEqual(
            float(camera_position[2]),
            TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M
            + TOWER_CAMERA_HEIGHT_ABOVE_ARM_M,
        )
        for side in ("left", "right"):
            arm_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_base"
            )
            self.assertAlmostEqual(
                float(self.model.body_pos[arm_id, 0]),
                TOWER_RECOMMENDED_ARM_MOUNT_X_M,
            )

        left = self.geom_id("tower_left_mast_visual")
        right = self.geom_id("tower_right_mast_visual")
        left_inner_face = float(
            self.model.geom_pos[left, 1] - self.model.geom_size[left, 1]
        )
        right_inner_face = float(
            self.model.geom_pos[right, 1] + self.model.geom_size[right, 1]
        )
        camera_left = float(camera_position[1]) + DEPTH_CAMERA_SIZE_M[1] / 2.0
        camera_right = float(camera_position[1]) - DEPTH_CAMERA_SIZE_M[1] / 2.0
        self.assertGreaterEqual(left_inner_face - camera_left, 0.004)
        self.assertGreaterEqual(camera_right - right_inner_face, 0.004)

    def test_camera_center_ray_hits_the_floor_in_front_work_region(self) -> None:
        camera_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            "front_depth_camera",
        )
        view = -self.data.cam_xmat[camera_id].reshape(3, 3)[:, 2]
        down_tilt = math.atan2(-float(view[2]), float(view[0]))
        self.assertAlmostEqual(down_tilt, TOWER_CAMERA_DOWN_TILT_RAD)
        distance = -float(self.data.cam_xpos[camera_id, 2]) / float(view[2])
        floor_hit = self.data.cam_xpos[camera_id] + distance * view
        self.assertAlmostEqual(float(floor_hit[1]), 0.0)
        self.assertAlmostEqual(float(floor_hit[2]), 0.0)
        self.assertGreater(float(floor_hit[0]), 0.62)
        self.assertLess(float(floor_hit[0]), 0.75)

    def test_home_center_of_mass_stays_behind_the_wheel_axis(self) -> None:
        body_ids = range(1, self.model.nbody)
        total_mass = sum(float(self.model.body_mass[body_id]) for body_id in body_ids)
        center_x = sum(
            float(self.data.xipos[body_id, 0])
            * float(self.model.body_mass[body_id])
            for body_id in body_ids
        ) / total_mass
        self.assertLess(center_x, -0.025)

    def test_home_pose_has_no_tower_contact(self) -> None:
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            names = {
                mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)
                )
                or ""
                for geom_id in (contact.geom1, contact.geom2)
            }
            self.assertFalse(
                any(name.startswith("tower_") for name in names),
                names,
            )

    def test_official_waffle_meshes_and_stationary_smoke_are_preserved(self) -> None:
        mesh_names = set(
            model_names(self.model, mujoco.mjtObj.mjOBJ_MESH, self.model.nmesh)
        )
        self.assertTrue(
            {
                "tb3_waffle_pi_base",
                "tb3_left_tire",
                "tb3_right_tire",
                "tb3_lds",
            }.issubset(mesh_names)
        )
        report = validate_model(
            self.model,
            source=self.source,
            smoke_steps=1000,
        )
        self.assertTrue(report["finite_state"])
        self.assertEqual(report["mount_layout"], "tower")
        self.assertEqual(
            report["mount_estimated_mass_kg"],
            TOWER_MOUNT_ESTIMATED_MASS_KG,
        )
        self.assertFalse(report["hardware_execution"])

    def test_invalid_tower_geometry_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires"):
            build_model(arm_mount_height_m=0.27, mount_layout="tower")
        with self.assertRaisesRegex(ValueError, "less than 4 mm"):
            build_model(
                arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
                arm_mount_separation_m=0.19,
                mount_layout="tower",
            )
        with self.assertRaisesRegex(ValueError, "mount_layout"):
            build_model(
                arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
                mount_layout="unknown",
            )


if __name__ == "__main__":
    unittest.main()
