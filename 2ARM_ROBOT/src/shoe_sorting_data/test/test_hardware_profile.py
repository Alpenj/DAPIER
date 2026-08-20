import unittest

from shoe_sorting_data.hardware_profile import ARM_MOTOR_ROLES, build_hardware_profile


class HardwareProfileTest(unittest.TestCase):
    def test_builds_aliases_without_inventing_left_right(self):
        motor = {
            "status": "ok",
            "model_number_raw": 777,
            "position_tick": 2048,
            "torque_enabled": False,
            "voltage_volts": 12.2,
        }
        arm_snapshot = {
            "read_only": True,
            "results": [
                {
                    "port": "/dev/serial/by-id/a",
                    "baudrate": 1_000_000,
                    "motors": {str(index): dict(motor) for index in range(1, 7)},
                },
                {
                    "port": "/dev/serial/by-id/b",
                    "baudrate": 1_000_000,
                    "motors": {str(index): dict(motor) for index in range(1, 7)},
                },
            ],
        }
        base_baseline = {
            "motion_commands_sent": False,
            "summary": {
                "linear_x_mps": {"abs_max": 0.0012, "abs_p99": 0.00065},
                "angular_z_radps": {"abs_max": 0.00083, "abs_p99": 0.00066},
            },
        }

        profile = build_hardware_profile(
            arm_snapshot,
            base_baseline,
            arm_snapshot_sha256="a" * 64,
            base_baseline_sha256="b" * 64,
        )

        self.assertEqual(set(profile["arms"]), {"arm_a", "arm_b"})
        self.assertIsNone(profile["arms"]["arm_a"]["semantic_side"])
        self.assertEqual(profile["arms"]["arm_b"]["motor_roles"], ARM_MOTOR_ROLES)
        self.assertFalse(profile["reference_facts"]["reference_code_copied"])
        self.assertEqual(profile["arms"]["arm_a"]["supply_class"], "12V")
        self.assertEqual(
            profile["base"]["stationary_tolerances"],
            {"linear_x_mps": 0.0024, "angular_z_radps": 0.002},
        )


if __name__ == "__main__":
    unittest.main()
