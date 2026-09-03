from __future__ import annotations

import hashlib
import itertools
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
    raise unittest.SkipTest(
        "MuJoCo is not installed in this Python environment"
    ) from error

from mobile_dual_so101 import (
    ASSEMBLED_SUPPORT_UNDER_STL,
    ASSEMBLED_SUPPORT_UPPER_SOURCE_STL,
    ASSEMBLED_SUPPORT_UPPER_STL,
    FRONT_SLAM_CAMERA_CENTER_M,
    FRONT_SLAM_CAMERA_MASS_KG,
    FRONT_SLAM_CAMERA_SIZE_M,
    HUMANOID_HOME_ACTION,
    SO101_BASE_LARGE_HOLE_CENTER_ARM_FRAME_M,
    SO101_BASE_LARGE_HOLE_MESH_SHA256,
    TOWER_CAMERA_DOWN_TILT_RAD,
    TOWER_CAMERA_CENTER_X_M,
    TOWER_CAMERA_CENTER_Z_M,
    TOWER_CAMERA_HEIGHT_ABOVE_ARM_M,
    TOWER_CAMERA_INTERFACE_PLATE_SIZE_M,
    TOWER_CAMERA_MAST_SIZE_M,
    TOWER_MOUNT_ESTIMATED_MASS_KG,
    TOWER_REPLACED_SO101_BASE_MESHES,
    TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
    TOWER_RECOMMENDED_ARM_MOUNT_X_M,
    WAFFLE_TOP_LOCAL_Z_M,
    WORKSPACE_DEPTH_CAMERA_VERTICAL_FOV_DEG,
    apply_control_as_pose,
    build_model,
    model_names,
    validate_model,
)
from waffle_reference import (
    ASSEMBLED_SUPPORT_STEP_SHA256,
    ASSEMBLED_SUPPORT_UNDER_STL_SHA256,
    ASSEMBLED_SUPPORT_UPPER_SOURCE_STL_SHA256,
    ASSEMBLED_SUPPORT_UPPER_STL_SHA256,
    ASSEMBLED_SUPPORT_UPPER_Z_OFFSET_M,
    SEMI_SUPPORT_BASE_SIZE_M,
    SEMI_SUPPORT_BIG_HOLE_CENTERS_LOCAL_M,
    SEMI_SUPPORT_BIG_HOLE_RADIUS_M,
    SEMI_SUPPORT_BOTTOM_HOLES_LOCAL_M,
    SEMI_SUPPORT_COLUMN_SIZE_M,
    SEMI_SUPPORT_COLUMN_TOP_LOCAL_Z_M,
    SO101_SOCKET_AXIS_Y_ABS_M,
    TOWER_CENTER_X_M,
    TOWER_CENTER_Y_ABS_M,
    WAFFLE_TOP_REFERENCE_ORIGIN_M,
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

    def geom_world_z_bounds(self, geom_id: int) -> tuple[float, float]:
        local_center = self.model.geom_aabb[geom_id, :3]
        half_size = self.model.geom_aabb[geom_id, 3:]
        rotation = self.data.geom_xmat[geom_id].reshape(3, 3)
        world_z = []
        for signs in itertools.product((-1.0, 1.0), repeat=3):
            local_corner = [
                float(local_center[index] + signs[index] * half_size[index])
                for index in range(3)
            ]
            world_z.append(
                float(self.data.geom_xpos[geom_id, 2])
                + sum(
                    float(rotation[2, index]) * local_corner[index]
                    for index in range(3)
                )
            )
        return min(world_z), max(world_z)

    def mesh_world_vertices(self, geom_id: int) -> np.ndarray:
        mesh_id = int(self.model.geom_dataid[geom_id])
        vertex_start = int(self.model.mesh_vertadr[mesh_id])
        vertex_count = int(self.model.mesh_vertnum[mesh_id])
        vertices = self.model.mesh_vert[
            vertex_start : vertex_start + vertex_count
        ]
        return (
            self.data.geom_xmat[geom_id].reshape(3, 3) @ vertices.T
        ).T + self.data.geom_xpos[geom_id]

    def mesh_world_bounds(self, geom_id: int) -> tuple[np.ndarray, np.ndarray]:
        world = self.mesh_world_vertices(geom_id)
        return world.min(axis=0), world.max(axis=0)

    def test_split_support_meshes_bottom_column_holes_and_arm_frames(self) -> None:
        self.assertEqual(
            ASSEMBLED_SUPPORT_STEP_SHA256,
            "f9f77f71a77f962aac3c7a3898bf5df7232b12982be3f16e3e2fc20d39c1bb3b",
        )
        self.assertEqual(
            ASSEMBLED_SUPPORT_UNDER_STL_SHA256,
            "f91c58b14bd9932757787d9fea1f72104bae537d38b003d2a430576fecc70076",
        )
        self.assertEqual(
            ASSEMBLED_SUPPORT_UPPER_SOURCE_STL_SHA256,
            "ed6218f3ba83459fc7c436416ef708130e2622a27df6772d4d62bf6a9d822bf7",
        )
        self.assertEqual(
            ASSEMBLED_SUPPORT_UPPER_STL_SHA256,
            "ed6218f3ba83459fc7c436416ef708130e2622a27df6772d4d62bf6a9d822bf7",
        )
        self.assertEqual(
            SO101_BASE_LARGE_HOLE_MESH_SHA256,
            "bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d",
        )
        mount_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "tower_mount_structure"
        )
        self.assertGreaterEqual(mount_id, 0)
        self.assertAlmostEqual(
            float(self.model.body_mass[mount_id]),
            TOWER_MOUNT_ESTIMATED_MASS_KG,
        )

        self.assertTrue(ASSEMBLED_SUPPORT_UNDER_STL.is_file())
        self.assertTrue(ASSEMBLED_SUPPORT_UPPER_SOURCE_STL.is_file())
        self.assertTrue(ASSEMBLED_SUPPORT_UPPER_STL.is_file())
        self.assertEqual(
            hashlib.sha256(ASSEMBLED_SUPPORT_UPPER_STL.read_bytes()).hexdigest(),
            ASSEMBLED_SUPPORT_UPPER_STL_SHA256,
        )
        mesh_names = set(
            model_names(self.model, mujoco.mjtObj.mjOBJ_MESH, self.model.nmesh)
        )
        self.assertTrue(
            {
                "assembled_support_under_mesh",
                "assembled_support_upper_mesh",
            }.issubset(mesh_names)
        )
        under_visual = self.geom_id("assembled_support_under_visual")
        upper_visual = self.geom_id("assembled_support_upper_visual")
        for geom_id in (under_visual, upper_visual):
            self.assertEqual(
                self.model.geom_type[geom_id], mujoco.mjtGeom.mjGEOM_MESH
            )
            self.assertEqual(int(self.model.geom_contype[geom_id]), 0)
            self.assertEqual(int(self.model.geom_conaffinity[geom_id]), 0)
        under_min, under_max = self.mesh_world_bounds(under_visual)
        upper_min, upper_max = self.mesh_world_bounds(upper_visual)
        np.testing.assert_allclose(
            under_max - under_min,
            [0.160, 0.180, 0.170],
            atol=1e-7,
        )
        np.testing.assert_allclose(
            upper_max - upper_min,
            [0.11096289, 0.254, 0.156],
            atol=1e-7,
        )
        self.assertAlmostEqual(float(upper_max[1]), SO101_SOCKET_AXIS_Y_ABS_M)
        self.assertAlmostEqual(float(upper_min[1]), -SO101_SOCKET_AXIS_Y_ABS_M)
        base_link_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link"
        )
        expected_under_bottom = (
            float(self.data.xpos[base_link_id, 2]) + WAFFLE_TOP_LOCAL_Z_M
        )
        self.assertAlmostEqual(float(under_min[2]), expected_under_bottom)
        self.assertAlmostEqual(
            float(upper_min[2]),
            expected_under_bottom + ASSEMBLED_SUPPORT_UPPER_Z_OFFSET_M,
        )
        self.assertAlmostEqual(float(under_max[2] - upper_min[2]), 0.010)

        base = self.geom_id("semi_support_base_collision")
        expected_half = [value / 2.0 for value in SEMI_SUPPORT_BASE_SIZE_M]
        for actual, expected in zip(self.model.geom_size[base], expected_half):
            self.assertAlmostEqual(float(actual), expected)
        base_bottom = float(
            self.model.geom_pos[base, 2] - self.model.geom_size[base, 2]
        )
        self.assertAlmostEqual(base_bottom, WAFFLE_TOP_LOCAL_Z_M)
        self.assertAlmostEqual(float(self.model.geom_pos[base, 0]), TOWER_CENTER_X_M)

        column = self.geom_id("semi_support_column_collision")
        expected_column_half = [
            value / 2.0 for value in SEMI_SUPPORT_COLUMN_SIZE_M
        ]
        for actual, expected in zip(
            self.model.geom_size[column], expected_column_half
        ):
            self.assertAlmostEqual(float(actual), expected)
        column_top = float(
            self.model.geom_pos[column, 2] + self.model.geom_size[column, 2]
        )
        self.assertAlmostEqual(
            column_top,
            WAFFLE_TOP_LOCAL_Z_M + SEMI_SUPPORT_COLUMN_TOP_LOCAL_Z_M,
        )

        for index, (local_x, local_y, _) in enumerate(
            SEMI_SUPPORT_BOTTOM_HOLES_LOCAL_M, start=1
        ):
            site_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_SITE,
                f"semi_support_bottom_hole_{index}",
            )
            self.assertGreaterEqual(site_id, 0)
            self.assertAlmostEqual(
                float(self.model.site_pos[site_id, 0]),
                TOWER_CENTER_X_M + local_x,
            )
            self.assertAlmostEqual(float(self.model.site_pos[site_id, 1]), local_y)

        for side, sign in (("left", 1.0), ("right", -1.0)):
            arm_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_base"
            )
            self.assertAlmostEqual(
                float(self.model.body_pos[arm_id, 0]),
                TOWER_RECOMMENDED_ARM_MOUNT_X_M,
            )
            self.assertAlmostEqual(
                float(self.model.body_pos[arm_id, 1]), sign * TOWER_CENTER_Y_ABS_M
            )
            self.assertAlmostEqual(
                float(self.model.body_pos[arm_id, 2]),
                TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
            )
            site_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_SITE,
                f"semi_support_{side}_shoulder_hole_center",
            )
            self.assertGreaterEqual(site_id, 0)
            arm_hole_world = (
                self.data.xpos[arm_id]
                + self.data.xmat[arm_id].reshape(3, 3)
                @ np.asarray(SO101_BASE_LARGE_HOLE_CENTER_ARM_FRAME_M)
            )
            for actual, expected in zip(
                arm_hole_world, self.data.site_xpos[site_id]
            ):
                self.assertAlmostEqual(float(actual), float(expected), places=6)
            expected_local = SEMI_SUPPORT_BIG_HOLE_CENTERS_LOCAL_M[
                0 if side == "left" else 1
            ]
            self.assertAlmostEqual(
                float(self.model.site_size[site_id, 0]),
                SEMI_SUPPORT_BIG_HOLE_RADIUS_M,
            )
            self.assertAlmostEqual(
                float(self.model.site_pos[site_id, 2]),
                WAFFLE_TOP_LOCAL_Z_M + expected_local[2],
            )

            base_geom_ids = np.flatnonzero(self.model.geom_bodyid == arm_id)
            base_mesh_geoms = {
                (
                    mujoco.mj_id2name(
                        self.model,
                        mujoco.mjtObj.mjOBJ_MESH,
                        int(self.model.geom_dataid[geom_id]),
                    )
                    or ""
                ): int(geom_id)
                for geom_id in base_geom_ids
                if self.model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH
            }
            base_mesh_names = set(base_mesh_geoms)
            self.assertIn(f"{side}_sts3215_03a_v1", base_mesh_names)
            self.assertIn(
                f"{side}_base_motor_holder_so101_v1", base_mesh_names
            )
            self.assertIn(
                f"{side}_waveshare_mounting_plate_so101_v2", base_mesh_names
            )
            for replaced in TOWER_REPLACED_SO101_BASE_MESHES:
                self.assertNotIn(f"{side}_{replaced}", base_mesh_names)

        origin_site = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            "waffle_top_reference_origin",
        )
        for actual, expected in zip(
            self.model.site_pos[origin_site], WAFFLE_TOP_REFERENCE_ORIGIN_M
        ):
            self.assertAlmostEqual(float(actual), expected)

    def test_hidden_collision_proxies_are_present_without_visual_clutter(self) -> None:
        for name in (
            "semi_support_base_collision",
            "semi_support_column_collision",
            "tower_camera_mast_collision",
            "tower_camera_interface_plate_collision",
        ):
            geom_id = self.geom_id(name)
            self.assertEqual(int(self.model.geom_contype[geom_id]), 1)
            self.assertEqual(int(self.model.geom_conaffinity[geom_id]), 1)
            self.assertEqual(int(self.model.geom_group[geom_id]), 3)
            self.assertAlmostEqual(float(self.model.geom_rgba[geom_id, 3]), 0.0)

        for present_name in (
            "tower_camera_mast_visual",
            "tower_camera_interface_plate_visual",
        ):
            geom_id = self.geom_id(present_name)
            self.assertEqual(int(self.model.geom_contype[geom_id]), 0)
            self.assertEqual(int(self.model.geom_conaffinity[geom_id]), 0)
            self.assertEqual(int(self.model.geom_group[geom_id]), 1)

        for absent_name in (
            "semi_support_camera_post_visual",
            "semi_support_camera_post_collision",
            "tower_camera_boom_visual",
            "tower_camera_boom_collision",
        ):
            self.assertEqual(
                mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, absent_name
                ),
                -1,
            )

    def test_workspace_camera_is_raised_on_separate_mast_and_interface_plate(self) -> None:
        camera_body = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "workspace_depth_camera_body",
        )
        camera_position = self.model.body_pos[camera_body]
        self.assertAlmostEqual(
            float(camera_position[0]), TOWER_CAMERA_CENTER_X_M
        )
        self.assertAlmostEqual(float(camera_position[1]), 0.0)
        self.assertAlmostEqual(
            float(camera_position[2]),
            TOWER_CAMERA_CENTER_Z_M,
        )
        self.assertAlmostEqual(
            float(camera_position[2] - TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M),
            TOWER_CAMERA_HEIGHT_ABOVE_ARM_M,
        )
        camera_geom = self.geom_id("workspace_depth_camera_collision")
        plate = self.geom_id("tower_camera_interface_plate_collision")
        from_to = np.zeros(6)
        self.assertAlmostEqual(
            float(
                mujoco.mj_geomDistance(
                    self.model, self.data, camera_geom, plate, 1.0, from_to
                )
            ),
            0.0,
            places=6,
        )
        mast = self.geom_id("tower_camera_mast_collision")
        np.testing.assert_allclose(
            self.model.geom_size[mast, :2],
            np.asarray(TOWER_CAMERA_MAST_SIZE_M) / 2.0,
        )
        np.testing.assert_allclose(
            self.model.geom_size[plate],
            np.asarray(TOWER_CAMERA_INTERFACE_PLATE_SIZE_M) / 2.0,
        )

    def test_astra_is_centered_on_turtlebot_front_plate(self) -> None:
        astra_body = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "front_slam_depth_camera_body",
        )
        astra_geom = self.geom_id("front_slam_depth_camera_collision")
        np.testing.assert_allclose(
            self.model.body_pos[astra_body],
            FRONT_SLAM_CAMERA_CENTER_M,
        )
        self.assertAlmostEqual(
            float(
                self.model.body_pos[astra_body, 2]
                - self.model.geom_size[astra_geom, 2]
            ),
            WAFFLE_TOP_LOCAL_Z_M,
        )
        np.testing.assert_allclose(
            self.model.geom_size[astra_geom],
            np.asarray(FRONT_SLAM_CAMERA_SIZE_M) / 2.0,
        )
        self.assertAlmostEqual(
            float(self.model.body_mass[astra_body]),
            FRONT_SLAM_CAMERA_MASS_KG,
        )

    def test_workspace_camera_vertical_fov_covers_floor(self) -> None:
        camera_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            "workspace_depth_camera",
        )
        rotation = self.data.cam_xmat[camera_id].reshape(3, 3)
        forward = -rotation[:, 2]
        image_up = rotation[:, 1]
        down_tilt = math.atan2(-float(forward[2]), float(forward[0]))
        self.assertAlmostEqual(down_tilt, TOWER_CAMERA_DOWN_TILT_RAD)
        tangent = math.tan(
            math.radians(WORKSPACE_DEPTH_CAMERA_VERTICAL_FOV_DEG) / 2.0
        )
        rays = {
            "near": forward - image_up * tangent,
            "center": forward,
            "far": forward + image_up * tangent,
        }
        floor_x = {}
        for name, ray in rays.items():
            ray = ray / np.linalg.norm(ray)
            distance = -float(self.data.cam_xpos[camera_id, 2]) / float(ray[2])
            hit = self.data.cam_xpos[camera_id] + distance * ray
            self.assertAlmostEqual(float(hit[1]), 0.0)
            self.assertAlmostEqual(float(hit[2]), 0.0)
            floor_x[name] = float(hit[0])
        self.assertGreater(floor_x["near"], 0.29)
        self.assertLess(floor_x["near"], 0.50)
        self.assertGreater(floor_x["center"], 0.70)
        self.assertLess(floor_x["center"], 1.10)
        self.assertGreater(floor_x["far"], 4.0)
        self.assertLess(floor_x["far"], 8.0)

    def test_home_center_of_mass_stays_behind_the_wheel_axis(self) -> None:
        body_ids = range(1, self.model.nbody)
        total_mass = sum(float(self.model.body_mass[i]) for i in body_ids)
        center_x = sum(
            float(self.data.xipos[i, 0]) * float(self.model.body_mass[i])
            for i in body_ids
        ) / total_mass
        self.assertLess(center_x, -0.025)

    def test_home_pose_has_no_support_contact(self) -> None:
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
                any(
                    name.startswith("semi_support_")
                    or name.startswith("tower_camera_")
                    for name in names
                ),
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
        report = validate_model(self.model, source=self.source, smoke_steps=1000)
        self.assertTrue(report["finite_state"])
        self.assertEqual(report["mount_layout"], "tower")
        self.assertEqual(
            report["mount_estimated_mass_kg"], TOWER_MOUNT_ESTIMATED_MASS_KG
        )
        self.assertFalse(report["hardware_execution"])

    def test_step_geometry_overrides_fail_closed(self) -> None:
        invalid_cases = (
            {"arm_mount_height_m": 0.27},
            {
                "arm_mount_height_m": TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
                "arm_mount_separation_m": 0.09,
            },
            {
                "arm_mount_height_m": TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
                "arm_mount_x_m": -0.050,
            },
        )
        for keyword in invalid_cases:
            with self.assertRaisesRegex(ValueError, "requires"):
                build_model(mount_layout="tower", **keyword)
        with self.assertRaisesRegex(ValueError, "mount_layout"):
            build_model(
                arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
                mount_layout="unknown",
            )


if __name__ == "__main__":
    unittest.main()
