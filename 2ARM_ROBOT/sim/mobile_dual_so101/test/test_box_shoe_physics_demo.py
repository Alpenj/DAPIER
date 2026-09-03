from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from box_shoe_physics_demo import BoxShoePhysicsDemo, _most_opposing_normal_dot


class BoxShoePhysicsDemoTest(unittest.TestCase):
    def test_invalid_finger_contacts_are_not_a_grasp(self) -> None:
        demo = BoxShoePhysicsDemo()
        report = demo.run()

        self.assertFalse(report.success)
        self.assertIn(
            report.completed_phase,
            ("left_bilateral_contact_failed", "left_opposing_contact_failed"),
        )
        self.assertGreater(report.right_wing_contact_count, 0)
        self.assertGreaterEqual(report.lid_open_angle_deg, 90.0)
        self.assertGreater(report.left_static_finger_contact_count, 0)
        self.assertFalse(report.opposing_finger_contact_verified)
        self.assertFalse(report.shoe_grasp_weld_present)
        self.assertFalse(report.friction_lift_verified)
        self.assertEqual(report.unintended_contact_pairs, ())
        self.assertEqual(report.runtime_arm_qpos_writes, 0)
        self.assertFalse(report.hardware_execution)

    def test_opposing_contact_metric_is_deterministic(self) -> None:
        fixed = [np.asarray((1.0, 0.0, 0.0))]
        self.assertEqual(
            _most_opposing_normal_dot(fixed, [np.asarray((-1.0, 0.0, 0.0))]),
            -1.0,
        )
        self.assertEqual(
            _most_opposing_normal_dot(fixed, [np.asarray((0.0, 1.0, 0.0))]),
            0.0,
        )
        self.assertIsNone(_most_opposing_normal_dot(fixed, []))


if __name__ == "__main__":
    unittest.main()
