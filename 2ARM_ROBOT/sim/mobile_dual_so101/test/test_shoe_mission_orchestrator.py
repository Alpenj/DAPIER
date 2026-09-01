from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from shoe_mission_orchestrator import (
    MANIPULATION_MODE,
    MOBILITY_MODE,
    PERCEPTION_MODE,
    TACTILE_MODE,
    ShoeMissionScenario,
    run_scripted_ik_mission,
)


class ShoeMissionOrchestratorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scenario = ShoeMissionScenario()
        cls.report = run_scripted_ik_mission(cls.scenario)

    def test_docking_pose_places_shoe_at_left_arm_target(self) -> None:
        docking = self.scenario.docking_pose_map()
        local = self.scenario.docking_shoe_position_base_m
        self.assertAlmostEqual(
            self.scenario.shoe_position_map_m[0] - docking.x_m,
            local[0],
            places=12,
        )
        self.assertAlmostEqual(
            self.scenario.shoe_position_map_m[1] - docking.y_m,
            local[1],
            places=12,
        )

    def test_a_b_pick_return_place_sequence_completes(self) -> None:
        report = self.report
        self.assertTrue(report.completed, report)
        self.assertEqual(report.final_phase, "completed")
        phases = [entry["phase"] for entry in report.transition_reports]
        self.assertEqual(
            phases,
            [
                "navigating_to_b",
                "localizing_shoe",
                "picking_at_b",
                "verifying_grasp",
                "navigating_to_a",
                "placing_at_a",
                "verifying_place",
                "completed",
            ],
        )
        x_m, y_m, yaw_rad = report.final_base_pose_map
        self.assertLess(math.hypot(x_m, y_m), 0.025)
        self.assertLess(abs(yaw_rad), 0.04)

    def test_ik_and_collision_plan_are_accepted(self) -> None:
        self.assertTrue(self.report.ik_converged)
        self.assertLessEqual(
            self.report.ik_residual_m,
            self.scenario.ik_tolerance_m,
        )
        self.assertTrue(self.report.collision_path_safe)
        self.assertGreaterEqual(
            self.report.minimum_clearance_m,
            self.scenario.required_clearance_m,
        )
        self.assertGreater(self.report.planned_trajectory_duration_s, 0.0)

    def test_report_does_not_claim_contact_physics_or_hardware(self) -> None:
        self.assertEqual(self.report.perception_mode, PERCEPTION_MODE)
        self.assertEqual(self.report.mobility_mode, MOBILITY_MODE)
        self.assertEqual(self.report.manipulation_mode, MANIPULATION_MODE)
        self.assertEqual(self.report.tactile_mode, TACTILE_MODE)
        self.assertFalse(self.report.contact_physics_grasp_verified)
        self.assertFalse(self.report.hardware_dispatch_authorized)
        self.assertFalse(self.report.hardware_execution)


if __name__ == "__main__":
    unittest.main()
