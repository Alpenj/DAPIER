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
        health = {
            "Torque_Enable": {"shoulder_pan": 0, "gripper": 0},
            "Status": {"shoulder_pan": 0, "gripper": 0},
            "Moving": {"shoulder_pan": 0, "gripper": 0},
            "Present_Temperature": {"shoulder_pan": 30, "gripper": 35},
            "Present_Voltage": {"shoulder_pan": 121, "gripper": 124},
        }
        record = {
            "trace": trace,
            "arms": {
                side: {"health_after_torque_off": health}
                for side in ("left", "right")
            },
        }
        html = REPORT["build_report"](record)
        self.assertIn("interval p50=50.00ms", html)
        self.assertIn("left shoulder_pan position", html)
        self.assertIn("Post-motion torque-off health", html)
        self.assertIn("30–35", html)
        self.assertIn("commissioning 완료 증거가 아닙니다", html)
        self.assertIn("not recorded", REPORT["build_report"]({"trace": trace}))
        record["arms"]["left"]["health_after_torque_off"] = {}
        with self.assertRaisesRegex(ValueError, "health_after_torque_off"):
            REPORT["build_report"](record)
        with self.assertRaisesRegex(ValueError, "no motion trace"):
            REPORT["build_report"]({"trace": []})


if __name__ == "__main__":
    unittest.main()
