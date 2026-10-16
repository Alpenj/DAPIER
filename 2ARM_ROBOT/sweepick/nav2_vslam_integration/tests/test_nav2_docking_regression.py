"""P2 Navigation & Docking Regression Test Suite.

Verifies zero regressions in the existing Navigation & Docking implementation
(`Dapier_project_visaul_slam/robot_ws/src/llm_semantic_docker`):
- `command_validator.py` (`CommandValidator`, `ActionEnum`, `RobotIntent`, `ValidationResult`)
- `semantic_waypoints.yaml` & `docking_profiles.yaml` & `docking_stations.yaml`
- `station_manager.py` (`StationManager` 3D transforms & standoff calculation)
- `vslam_navigation.launch.py` static TF (`camera_link -> camera_color_optical_frame`)
- `object_detector_3d.py` Stage-2 3D plane RANSAC+SVD geometry
- `task_manager.py` state contract & docking feasibility definitions
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
SWEEPICK_ROOT = HERE.parent.parent
if str(SWEEPICK_ROOT) not in sys.path:
    sys.path.insert(0, str(SWEEPICK_ROOT))

from nav2_vslam_integration import (
    NavigationManipulationBridge,
    SweepickExecutionAdapter,
    VslamSweepickCoordinator,
    transform_optical_pose_to_base_footprint,
)
from nav2_vslam_integration.navigation_bridge import (
    load_nav_module,
    resolve_llm_semantic_docker_root,
)
from nav2_vslam_integration.tests.test_mock_e2e_integration import (
    MockSweepickConsoleController,
)


class TestNav2DockingRegression(unittest.TestCase):
    """Regression suite for existing Navigation and Docking components."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.pkg_root = resolve_llm_semantic_docker_root()
        cls.config_dir = cls.pkg_root / "config"
        cls.templates_dir = cls.pkg_root / "templates"
        cls.cv_mod = load_nav_module(
            "reg_command_validator",
            cls.pkg_root / "llm_semantic_docker/command_validator.py",
        )
        cls.sm_mod = load_nav_module(
            "reg_station_manager",
            cls.pkg_root / "llm_semantic_docker/station_manager.py",
        )

    def setUp(self) -> None:
        self.validator = self.cv_mod.CommandValidator(
            waypoints_file=str(self.config_dir / "semantic_waypoints.yaml"),
            profiles_file=str(self.config_dir / "docking_profiles.yaml"),
            colors_file=str(self.config_dir / "color_profiles.yaml"),
        )
        self.station_manager = self.sm_mod.StationManager(
            config_path=str(self.config_dir / "docking_stations.yaml"),
            templates_dir=str(self.templates_dir),
        )

    def test_01_command_validator_semantic_waypoints_regression(self) -> None:
        expected_waypoints = (
            "box_staging",
            "desk_staging",
            "charger_staging",
            "entrance",
            "meeting_room",
            "workplace1",
            "workplace2",
        )
        for wp_name in expected_waypoints:
            raw = f'{{"action": "NAVIGATE", "target": {{"semantic_name": "{wp_name}"}}}}'
            res = self.validator.validate(raw)
            self.assertTrue(res.is_valid, f"Expected valid waypoint {wp_name}, got: {res.error_msg}")
            self.assertEqual(res.action, "NAVIGATE")
            self.assertEqual(res.resolved_task["waypoint_name"], wp_name)

        # Unknown waypoint ("화성으로 가") must be rejected 100%
        res_bad = self.validator.validate(
            '{"action": "NAVIGATE", "target": {"semantic_name": "mars_colony"}}'
        )
        self.assertFalse(res_bad.is_valid)
        self.assertEqual(res_bad.error_code, "UNKNOWN_WAYPOINT")

    def test_02_command_validator_docking_profiles_and_stations_regression(self) -> None:
        # Taught 3D stations
        for st_name in ("workplace1", "workplace2"):
            raw = f'{{"action": "DOCK", "target": {{"target_structure": "{st_name}"}}}}'
            res = self.validator.validate(raw)
            self.assertTrue(res.is_valid, f"Expected valid station {st_name}, got: {res.error_msg}")
            self.assertEqual(res.resolved_task["station_name"], st_name)
            self.assertEqual(res.resolved_task["profile_name"], "station_icp")

        # Desk red object & desk direct & charger
        res_desk_red = self.validator.validate(
            '{"action": "DOCK", "target": {"target_structure": "desk", "anchor_cue": "red_object"}}'
        )
        self.assertTrue(res_desk_red.is_valid)
        self.assertEqual(res_desk_red.resolved_task["profile_name"], "desk_red_object")
        self.assertEqual(res_desk_red.resolved_task["anchor_color_preset"], "red")

        res_desk_direct = self.validator.validate(
            '{"action": "DOCK", "target": {"target_structure": "desk"}}'
        )
        self.assertTrue(res_desk_direct.is_valid)
        self.assertEqual(res_desk_direct.resolved_task["profile_name"], "desk_direct")

        res_charger = self.validator.validate(
            '{"action": "DOCK", "target": {"target_structure": "charger"}}'
        )
        self.assertTrue(res_charger.is_valid)
        self.assertEqual(res_charger.resolved_task["profile_name"], "charger_station")

        # STOP command
        res_stop = self.validator.validate('{"action": "STOP"}')
        self.assertTrue(res_stop.is_valid)
        self.assertEqual(res_stop.resolved_task["action"], "STOP")

    def test_03_station_manager_coordinate_transforms_and_standoff_regression(self) -> None:
        stations = self.station_manager.list_stations()
        self.assertTrue({"workplace1", "workplace2"}.issubset(set(stations)))

        for name in ("workplace1", "workplace2"):
            st_data = self.station_manager.get_station(name)
            self.assertIsNotNone(st_data)
            center = np.asarray(st_data["target_face"]["center"], dtype=float)
            size = np.asarray(st_data["target_face"]["size"], dtype=float)
            yaw_rad = math.radians(float(st_data["standoff"]["yaw_deg"]))
            rot = self.station_manager.rotation_matrix_from_yaw(yaw_rad)

            # Verify map -> station_local -> map round-trip invertibility
            test_pts_map = np.array(
                [[1.25, -0.85, 0.32], [0.80, 0.88, 0.04]], dtype=float
            )
            pts_local = self.station_manager.transform_to_station_local(
                test_pts_map, center, rot
            )
            pts_map_recovered = self.station_manager.transform_to_map(
                pts_local, center, rot
            )
            np.testing.assert_allclose(test_pts_map, pts_map_recovered, atol=1e-6)

            # Verify calculate_standoff_pose matches taught standoff geometry
            sx, sy, syaw = self.station_manager.calculate_standoff_pose(
                center, size, yaw_rad, standoff_distance=0.60
            )
            self.assertAlmostEqual(sx, float(st_data["standoff"]["x"]), places=3)
            self.assertAlmostEqual(sy, float(st_data["standoff"]["y"]), places=3)
            self.assertAlmostEqual(math.degrees(syaw), float(st_data["standoff"]["yaw_deg"]), places=2)

    def test_04_vslam_launch_optical_tf_and_bridge_resolution_regression(self) -> None:
        launch_path = (
            self.pkg_root.parent
            / "turtlebot3_visual_slam/launch/vslam_navigation.launch.py"
        )
        launch_text = launch_path.read_text(encoding="utf-8")
        self.assertIn("camera_to_optical_tf", launch_text)
        self.assertIn("'--frame-id', 'camera_link'", launch_text)
        self.assertIn("'--child-frame-id', 'camera_color_optical_frame'", launch_text)
        self.assertIn("'-1.5707963'", launch_text)

        # Pure-math regression of the optical -> camera_link rotation. The origin is supplied
        # explicitly (C-2: no default). Value = Waffle Pi URDF chain composed by hand
        # (base_footprint->base_link z=0.010, base_link->camera_link (0.073,-0.011,0.084)),
        # labelled here as a constant for math checking only, not as real camera evidence.
        urdf_chain_origin = (0.073, -0.011, 0.094)
        p_base, n_base = transform_optical_pose_to_base_footprint(
            (0.0, 0.0, 0.50),
            (0.0, 0.0, -1.0),
            camera_link_in_base_footprint_m=urdf_chain_origin,
        )
        np.testing.assert_allclose(p_base, (0.573, -0.011, 0.094), atol=1e-6)
        np.testing.assert_allclose(n_base, (-1.0, 0.0, 0.0), atol=1e-6)
        with self.assertRaises(TypeError):
            transform_optical_pose_to_base_footprint((0.0, 0.0, 0.50), (0.0, 0.0, -1.0))  # no default origin

        mock_console = MockSweepickConsoleController()
        coordinator = VslamSweepickCoordinator(
            sweepick_adapter=SweepickExecutionAdapter(mock_console)
        )
        bridge = NavigationManipulationBridge(coordinator=coordinator)
        resolved = bridge.resolve_navigation_goal(
            '{"action": "DOCK", "target": {"target_structure": "workplace1"}}'
        )
        self.assertTrue(resolved["valid"])
        self.assertAlmostEqual(resolved["staging_pose_map"][0], 0.8791, places=3)
        self.assertAlmostEqual(resolved["staging_pose_map"][1], 0.1059, places=3)

    def test_05_object_detector_3d_plane_svd_geometry_regression(self) -> None:
        # Verify the SVD plane normal & RMS math used in object_detector_3d.py
        rng = np.random.default_rng(42)
        y = rng.uniform(-0.10, 0.10, 120)
        z = rng.uniform(0.02, 0.20, 120)
        x = 0.40 + rng.normal(0.0, 0.002, 120)
        pts = np.column_stack([x, y, z])

        centroid = np.mean(pts, axis=0)
        centered = pts - centroid
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[2, :]
        if normal[0] > 0:
            normal = -normal
        rms = float(np.sqrt(np.mean((centered @ normal) ** 2)))

        self.assertAlmostEqual(float(centroid[0]), 0.40, delta=0.005)
        np.testing.assert_allclose(normal, [-1.0, 0.0, 0.0], atol=0.05)
        self.assertLess(rms, 0.005)


if __name__ == "__main__":
    unittest.main(verbosity=2)
