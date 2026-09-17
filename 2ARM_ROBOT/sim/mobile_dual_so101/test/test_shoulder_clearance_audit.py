import unittest
from unittest.mock import patch
import numpy as np
import mujoco

from integration_scenes import task_env
from shoulder_clearance_audit import audit_sweep, shoulder_dependency


class ShoulderSweepTest(unittest.TestCase):
    def test_full_ranges_are_fk_only_and_do_not_change_source(self):
        env = task_env()
        seen = []
        def observe(model, data, dep, row, count):
            self.assertIsNot(data, env.data)
            self.assertEqual(data.time, 0.)
            seen.append((dep["side"], row["angle_rad"]))
        with patch("mujoco.mj_step", side_effect=AssertionError("sweep must never step physics")):
            report = audit_sweep(env, observer=observe)
        self.assertTrue(report["source_simulation_state_unchanged"])
        self.assertEqual(report["physics_steps"], 0)
        self.assertEqual(report["required_clearance_m"], .03)
        for side, expected_geom in (("left",13), ("right",48)):
            result = report["sides"][side]
            dep, summary = result["dependency"], result["summary"]
            self.assertEqual(dep["geom_id"], expected_geom)
            self.assertEqual(dep["joint_name"], side+"_shoulder_pan")
            self.assertEqual(len(dep["influencing_joint_ids"]), 1)
            np.testing.assert_allclose(dep["world_axis"], [0,0,-1], atol=1e-12)
            np.testing.assert_allclose(np.degrees(dep["effective_range_rad"]), [-110,110], atol=1e-8)
            self.assertTrue(all(x["transform_max_abs_change"] <= 1e-12
                                for x in dep["non_dependency_fk_checks"]))
            angles = np.array([r["angle_rad"] for r in result["samples"]])
            self.assertGreaterEqual(len(angles), 441)
            np.testing.assert_allclose(angles[[0,-1]], dep["effective_range_rad"], atol=1e-14)
            self.assertLessEqual(np.max(np.diff(angles)), np.radians(.5)+1e-14)
            self.assertGreater(summary["continuous_separation_lower_bound_m"], .0161999)
            self.assertLess(summary["min_z_gap_m"], .03)
            self.assertEqual(summary["contact_sample_count"], 0)
            self.assertEqual(summary["penetration_sample_count"], 0)
        self.assertEqual(len(seen), sum(r["summary"]["sample_count"] for r in report["sides"].values()))


if __name__ == "__main__":
    unittest.main()
