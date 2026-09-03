from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from dual_so101_sim_real import run_comparison


class DualSO101SimRealTest(unittest.TestCase):
    def test_bounded_smoke_is_compared_without_claiming_strict_acceptance(self):
        evidence = (
            Path(__file__).resolve().parents[3]
            / "docs"
            / "evidence"
            / "dual_so101_symmetric_smoke_20260903.json"
        )
        report = run_comparison(evidence)

        self.assertFalse(report["hardware_execution"])
        self.assertFalse(report["strict_dynamics_gate_passed"])
        self.assertEqual(
            report["known_gate_failure"],
            "20 Hz step command and finite-difference actual jerk",
        )
        self.assertEqual(report["steps_each_way"], 30)
        self.assertAlmostEqual(report["control_update_period_s"], 0.0512005090713501)
        for side in ("left", "right"):
            self.assertLess(
                report["absolute_error_degrees"][side]["excursion_degrees"], 0.5
            )
            self.assertTrue(report["outbound_physics"]["finite_state"])
            self.assertEqual(report["outbound_physics"]["forbidden_contact_count"], 0)


if __name__ == "__main__":
    unittest.main()
