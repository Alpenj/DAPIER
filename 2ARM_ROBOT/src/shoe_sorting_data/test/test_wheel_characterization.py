import math
import unittest

from shoe_sorting_data.wheel_characterization import (
    analyze_characterization,
    first_bilateral_command,
    first_responsive_command,
    summarize_stage,
)


class WheelCharacterizationTest(unittest.TestCase):
    def test_summary_does_not_invent_unsupported_current(self):
        samples = [
            {
                "odom_linear_x_mps": 0.01,
                "odom_angular_z_radps": 0.0,
                "left_wheel_mps": 0.01,
                "right_wheel_mps": 0.012,
                "battery_voltage": 12.0,
                "battery_current": math.nan,
            },
            {
                "odom_linear_x_mps": 0.012,
                "odom_angular_z_radps": 0.001,
                "left_wheel_mps": 0.012,
                "right_wheel_mps": 0.014,
                "battery_voltage": 11.9,
                "battery_current": math.nan,
            },
        ]
        summary = summarize_stage(samples)
        self.assertEqual(summary["median_odom_linear_x_mps"], 0.011)
        self.assertEqual(summary["battery_voltage_range"], [11.9, 12.0])
        self.assertFalse(summary["battery_current_informative"])
        self.assertIsNone(summary["battery_current_range"])
        self.assertEqual(summary["median_encoder_linear_x_mps"], 0.012)
        self.assertAlmostEqual(summary["median_encoder_angular_z_radps"], 0.002 / 0.287)

    def test_first_responsive_command_uses_measured_tolerance(self):
        stages = [
            {"kind": "linear", "command": 0.005, "summary": {"median_encoder_linear_x_mps": 0.0}},
            {"kind": "linear", "command": 0.01, "summary": {"median_encoder_linear_x_mps": 0.004}},
        ]
        self.assertEqual(first_responsive_command(stages, kind="linear", tolerance=0.0025), 0.01)

    def test_bilateral_command_rejects_one_wheel_angular_motion(self):
        stages = [
            {
                "kind": "angular",
                "command": 0.02,
                "summary": {
                    "median_left_wheel_mps": -0.0016,
                    "median_right_wheel_mps": 0.0,
                    "median_encoder_angular_z_radps": 0.0056,
                },
            },
            {
                "kind": "angular",
                "command": 0.05,
                "summary": {
                    "median_left_wheel_mps": -0.0063,
                    "median_right_wheel_mps": 0.0063,
                    "median_encoder_angular_z_radps": 0.044,
                },
            },
        ]
        self.assertEqual(first_responsive_command(stages, kind="angular", tolerance=0.0021), 0.02)
        self.assertEqual(first_bilateral_command(stages, kind="angular", tolerance=0.0021), 0.05)

    def test_analysis_distinguishes_accurate_and_tested_ceiling(self):
        def stage(command, response, asymmetry):
            return {
                "kind": "linear",
                "command": command,
                "summary": {
                    "median_left_wheel_mps": response,
                    "median_right_wheel_mps": response,
                    "median_encoder_linear_x_mps": response,
                    "absolute_wheel_speed_asymmetry_fraction": asymmetry,
                    "battery_voltage_range": [11.3, 11.4],
                    "battery_current_informative": False,
                },
            }

        stages = [stage(0.01, 0.008, 0.0), stage(0.20, 0.198, 0.01), stage(0.26, 0.229, 0.06)]
        analysis = analyze_characterization(stages)
        self.assertEqual(
            analysis["linear"]["recommended_tracking_ceiling"]["command"], 0.20
        )
        self.assertEqual(
            analysis["linear"]["tested_positive_command_ceiling"]["command"], 0.26
        )
        self.assertEqual(analysis["battery_voltage_range_v"], [11.3, 11.4])
        self.assertFalse(analysis["battery_current_informative"])


if __name__ == "__main__":
    unittest.main()
