from __future__ import annotations

from dataclasses import replace
import unittest

from dapier_sim_first.embodiment import (
    BimanualEmbodimentSpec,
    SO101_CHANNEL_NAMES,
    so101_bimanual_new_calibration_spec,
)
from dapier_sim_first.protocols import Frame, validate_frame


LEFT_CALIBRATION_ID = "sha256:" + "1" * 64
RIGHT_CALIBRATION_ID = "sha256:" + "2" * 64


class BimanualEmbodimentContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.spec = so101_bimanual_new_calibration_spec(
            LEFT_CALIBRATION_ID,
            RIGHT_CALIBRATION_ID,
        )

    def test_exact_left_then_right_channel_order(self) -> None:
        expected = tuple(f"left_{name}" for name in SO101_CHANNEL_NAMES) + tuple(
            f"right_{name}" for name in SO101_CHANNEL_NAMES
        )
        self.assertEqual(self.spec.channel_names, expected)
        self.assertEqual(len(self.spec.channel_names), 12)

    def test_each_arm_keeps_its_calibration_identity(self) -> None:
        self.assertEqual(
            self.spec.calibration_ids,
            (LEFT_CALIBRATION_ID, RIGHT_CALIBRATION_ID),
        )
        reversed_spec = so101_bimanual_new_calibration_spec(
            RIGHT_CALIBRATION_ID,
            LEFT_CALIBRATION_ID,
        )
        self.assertNotEqual(self.spec.calibration_id, reversed_spec.calibration_id)
        self.assertNotEqual(self.spec.bounds_digest(), reversed_spec.bounds_digest())

    def test_twelve_channel_action_round_trips(self) -> None:
        action = (0.0, -10.0, 20.0, -30.0, 40.0, 25.0) + (
            10.0,
            -20.0,
            30.0,
            -40.0,
            50.0,
            75.0,
        )
        round_trip = self.spec.sim_to_action(self.spec.action_to_sim(action))
        for actual, expected in zip(round_trip, action, strict=True):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_incomplete_action_is_rejected_before_split(self) -> None:
        with self.assertRaisesRegex(ValueError, "expected 12 values"):
            self.spec.action_to_sim((0.0,) * 11)

    def test_existing_frame_validator_accepts_bimanual_spec(self) -> None:
        now_ns = 1_000_000_000
        validate_frame(
            Frame(
                embodiment_id=self.spec.embodiment_id,
                embodiment_revision=self.spec.embodiment_revision,
                channel_names=self.spec.channel_names,
                values=(0.0,) * 12,
                units=self.spec.action_units,
                calibration_id=self.spec.calibration_id,
                monotonic_timestamp_ns=now_ns,
                sequence_id=0,
                source="scripted",
            ),
            spec=self.spec,
            now_ns=now_ns,
            control_period_ns=33_333_333,
        )

    def test_noncanonical_child_contract_is_rejected_even_when_both_arms_match(
        self,
    ) -> None:
        replacements = {
            "embodiment_id": "not-so101",
            "embodiment_revision": "not-new-calibration",
            "channel_names": tuple(reversed(SO101_CHANNEL_NAMES)),
            "action_units": ("radian",) * 6,
            "sim_units": ("degree",) * 6,
            "sim_lower": tuple(value - 0.01 for value in self.spec.left.sim_lower),
            "sim_upper": tuple(value + 0.01 for value in self.spec.left.sim_upper),
        }
        for field, value in replacements.items():
            with self.subTest(field=field):
                left = replace(self.spec.left, **{field: value})
                right = replace(self.spec.right, **{field: value})
                with self.assertRaisesRegex(ValueError, "canonical SO-101"):
                    BimanualEmbodimentSpec(left=left, right=right)

    def test_one_noncanonical_child_contract_is_rejected(self) -> None:
        right = replace(self.spec.right, action_units=("radian",) * 6)
        with self.assertRaisesRegex(ValueError, "right.*canonical SO-101"):
            BimanualEmbodimentSpec(left=self.spec.left, right=right)


if __name__ == "__main__":
    unittest.main()
