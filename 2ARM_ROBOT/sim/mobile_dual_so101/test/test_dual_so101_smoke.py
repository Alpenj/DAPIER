from pathlib import Path
import runpy
from unittest import mock
import unittest


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "dual_so101_smoke"
SMOKE = runpy.run_path(str(SCRIPT))


class FakeBus:
    def sync_write(self, register, values):
        self.goal = values

    def sync_read(self, register, *_args, **_kwargs):
        values = {
            "Present_Position": 1.0,
            "Present_Load": 2,
            "Present_Current": 3,
            "Present_Velocity": 4,
        }
        return {"shoulder_pan": values.get(register, 0)}


class DualSO101SmokeTest(unittest.TestCase):
    def test_trace_records_position_load_and_current(self):
        trace = []
        buses = {"left": FakeBus(), "right": FakeBus()}
        starts = {side: {"shoulder_pan": 0.0} for side in buses}

        with mock.patch.object(SMOKE["time"], "sleep"):
            SMOKE["interpolate"](buses, starts, 1.0, 1, trace)

        self.assertEqual(trace[0]["observed"], {"left": 1.0, "right": 1.0})
        self.assertEqual(trace[0]["present_load_raw"], {"left": 2, "right": 2})
        self.assertEqual(trace[0]["present_current_raw"], {"left": 3, "right": 3})
        self.assertEqual(trace[0]["present_velocity_raw"], {"left": 4, "right": 4})
        self.assertIn("monotonic_s", trace[0])
        self.assertEqual(
            set(SMOKE["health"](FakeBus())),
            {
                "Torque_Enable",
                "Operating_Mode",
                "Acceleration",
                "Goal_Time",
                "Goal_Velocity",
                "Torque_Limit",
                "Present_Load",
                "Present_Current",
                "Present_Temperature",
                "Present_Voltage",
                "Status",
                "Moving",
                "Maximum_Velocity_Limit",
                "Maximum_Acceleration",
            },
        )


if __name__ == "__main__":
    unittest.main()
