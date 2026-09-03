from __future__ import annotations

from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from box_shoe_physics_demo import BoxShoePhysicsDemo


class BoxShoePhysicsDemoTest(unittest.TestCase):
    def test_single_gripper_side_contact_is_not_a_grasp(self) -> None:
        demo = BoxShoePhysicsDemo()
        report = demo.run()

        self.assertFalse(report.success)
        self.assertEqual(report.completed_phase, "left_bilateral_contact_failed")
        self.assertGreater(report.right_wing_contact_count, 0)
        self.assertGreaterEqual(report.lid_open_angle_deg, 90.0)
        self.assertGreater(report.left_static_finger_contact_count, 0)
        self.assertEqual(report.left_moving_finger_contact_count, 0)
        self.assertFalse(report.bilateral_finger_contact_verified)
        self.assertFalse(report.shoe_grasp_weld_present)
        self.assertFalse(report.friction_lift_verified)
        self.assertEqual(report.unintended_contact_pairs, ())
        self.assertEqual(report.runtime_arm_qpos_writes, 0)
        self.assertFalse(report.hardware_execution)


if __name__ == "__main__":
    unittest.main()
