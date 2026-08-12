#!/usr/bin/env python3
"""Boundary tests for the physical Burger teleop limiter."""

import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).with_name("tb3_real_teleop.py")
SPEC = importlib.util.spec_from_file_location("tb3_real_teleop", MODULE_PATH)
TELEOP = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(TELEOP)


class MotionLimitTest(unittest.TestCase):
    def assert_wheel_limits(self, linear: float, angular: float) -> None:
        left, right = TELEOP.wheel_speeds(linear, angular)
        self.assertLessEqual(
            max(abs(left), abs(right)), TELEOP.MAX_WHEEL_LINEAR + 1e-9
        )
        if abs(linear) > 1e-9 and abs(angular) > 1e-9:
            inner, outer = sorted((abs(left), abs(right)))
            self.assertGreaterEqual(
                inner + 1e-9, TELEOP.MIN_INNER_WHEEL_RATIO * outer
            )

    def test_mapping_and_sport_straight_limits(self) -> None:
        self.assertEqual(TELEOP.limit_motion(1.0, 0.0), (0.18, 0.0))
        self.assertEqual(
            TELEOP.limit_motion(1.0, 0.0, TELEOP.SPORT_MAX_LINEAR),
            (0.22, 0.0),
        )

    def test_sport_mode_keeps_steering_at_top_speed(self) -> None:
        linear, angular = TELEOP.limit_motion(
            0.22, 0.15, TELEOP.SPORT_MAX_LINEAR
        )
        self.assertAlmostEqual(angular, 0.15)
        self.assertAlmostEqual(linear, 0.208)
        self.assert_wheel_limits(linear, angular)

    def test_low_speed_turn_does_not_accelerate_linear_motion(self) -> None:
        linear, angular = TELEOP.limit_motion(0.02, 1.5)
        self.assertAlmostEqual(linear, 0.02)
        self.assertLess(angular, 1.5)
        self.assert_wheel_limits(linear, angular)

    def test_in_place_turn_limit(self) -> None:
        self.assertEqual(TELEOP.limit_motion(0.0, 9.0), (0.0, 1.5))
        self.assert_wheel_limits(0.0, 1.5)

    def test_slew_rate_and_convergence(self) -> None:
        current = 0.0
        for _ in range(10):
            next_value = TELEOP.slew(current, 1.0, 0.5, 0.1)
            self.assertLessEqual(abs(next_value - current), 0.05 + 1e-12)
            current = next_value
        self.assertAlmostEqual(current, 0.5)
        for _ in range(10):
            current = TELEOP.slew(current, 1.0, 0.5, 0.1)
        self.assertAlmostEqual(current, 1.0)

    def test_slew_rejects_invalid_limits(self) -> None:
        with self.assertRaises(ValueError):
            TELEOP.slew(0.0, 1.0, 0.0, 0.1)
        with self.assertRaises(ValueError):
            TELEOP.slew(0.0, 1.0, 1.0, -0.1)

    def test_smoothed_transition_keeps_wheel_limits(self) -> None:
        linear = 0.0
        angular = 0.0
        targets = ((0.22, 0.0), (0.22, 1.0), (-0.22, -1.0), (0.0, 1.5))
        for target_linear, target_angular in targets:
            target_linear, target_angular = TELEOP.limit_motion(
                target_linear, target_angular, TELEOP.SPORT_MAX_LINEAR
            )
            for _ in range(100):
                linear = TELEOP.slew(linear, target_linear, 0.35, 0.05)
                angular = TELEOP.slew(angular, target_angular, 1.20, 0.05)
                linear, angular = TELEOP.limit_motion(
                    linear, angular, TELEOP.SPORT_MAX_LINEAR
                )
                self.assert_wheel_limits(linear, angular)

    def test_exhaustive_input_grid(self) -> None:
        for max_linear in (
            TELEOP.MAPPING_MAX_LINEAR,
            TELEOP.SPORT_MAX_LINEAR,
        ):
            for linear_step in range(-30, 31):
                for angular_step in range(-20, 21):
                    linear, angular = TELEOP.limit_motion(
                        linear_step / 100.0,
                        angular_step / 10.0,
                        max_linear,
                    )
                    self.assertLessEqual(abs(linear), max_linear + 1e-9)
                    self.assertLessEqual(
                        abs(angular), TELEOP.MAX_ANGULAR + 1e-9
                    )
                    self.assert_wheel_limits(linear, angular)


if __name__ == "__main__":
    unittest.main()
