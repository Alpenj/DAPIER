#!/usr/bin/env python3

import unittest

from tb3_wheel_test import validate_phase, validate_position_delta


class WheelPhaseTests(unittest.TestCase):
    def samples(self, left: float, right: float) -> list[tuple[float, float]]:
        return [(left, right)] * 6

    def test_accepts_all_expected_relations(self) -> None:
        cases = (
            ("both_positive_balanced", 0.050, 0.049),
            ("both_negative_balanced", -0.050, -0.049),
            ("left_negative_right_positive", -0.040, 0.040),
            ("left_positive_right_negative", 0.040, -0.040),
            ("both_positive_right_faster", 0.026, 0.074),
            ("both_positive_left_faster", 0.074, 0.026),
        )
        for relation, left, right in cases:
            with self.subTest(relation=relation):
                self.assertEqual(
                    validate_phase(relation, self.samples(left, right))[2], []
                )

    def test_rejects_stopped_wheel(self) -> None:
        errors = validate_phase(
            "both_positive_balanced", self.samples(0.05, 0.0)
        )[2]
        self.assertTrue(errors)

    def test_rejects_wrong_rotation_signs(self) -> None:
        errors = validate_phase(
            "left_negative_right_positive", self.samples(0.04, 0.04)
        )[2]
        self.assertTrue(errors)

    def test_rejects_too_few_samples(self) -> None:
        errors = validate_phase("both_positive_balanced", [(1.0, 1.0)] * 4)[2]
        self.assertIn("only 4", " ".join(errors))

    def test_encoder_delta_matches_each_direction(self) -> None:
        cases = (
            ("both_positive_balanced", (0.0, 0.0), (1.0, 1.0)),
            ("both_negative_balanced", (1.0, 1.0), (0.0, 0.0)),
            ("left_negative_right_positive", (0.0, 0.0), (-1.0, 1.0)),
            ("left_positive_right_negative", (0.0, 0.0), (1.0, -1.0)),
            ("both_positive_right_faster", (0.0, 0.0), (0.4, 1.0)),
            ("both_positive_left_faster", (0.0, 0.0), (1.0, 0.4)),
        )
        for relation, start, end in cases:
            with self.subTest(relation=relation):
                self.assertEqual(validate_position_delta(relation, start, end)[2], [])


if __name__ == "__main__":
    unittest.main()
