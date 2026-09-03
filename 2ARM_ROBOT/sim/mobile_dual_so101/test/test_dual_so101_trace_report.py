from pathlib import Path
import runpy
import unittest


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "dual_so101_trace_report"
REPORT = runpy.run_path(str(SCRIPT))


class DualSO101TraceReportTest(unittest.TestCase):
    def test_builds_static_report_and_rejects_missing_trace(self):
        trace = []
        for index in range(3):
            trace.append(
                {
                    "monotonic_s": 10.0 + index * 0.05,
                    "goal": {"left": index, "right": -index},
                    "observed": {"left": index * 0.9, "right": -index * 0.9},
                    "present_load_raw": {"left": index, "right": index + 1},
                    "present_current_raw": {"left": index + 2, "right": index + 3},
                    "present_velocity_raw": {"left": index + 4, "right": index + 5},
                }
            )
        html = REPORT["build_report"]({"trace": trace})
        self.assertIn("interval p50=50.00ms", html)
        self.assertIn("left shoulder_pan position", html)
        self.assertIn("commissioning 완료 증거가 아닙니다", html)
        with self.assertRaisesRegex(ValueError, "no motion trace"):
            REPORT["build_report"]({"trace": []})


if __name__ == "__main__":
    unittest.main()
