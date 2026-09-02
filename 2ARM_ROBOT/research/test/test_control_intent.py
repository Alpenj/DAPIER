from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


RESEARCH_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH_ROOT / "src"))

from dapier_research import (  # noqa: E402
    arm_joint_position_intent,
    base_twist_intent,
    hold_intent,
    intent_from_mapping,
    load_contract,
    validate_intent,
)


class ControlIntentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = load_contract()

    def test_contract_never_allows_python_hardware_authorization(self) -> None:
        self.assertFalse(self.contract.research_may_authorize_hardware)
        self.assertEqual(
            self.contract.schema_version,
            "dapier.research-realtime-control.v1",
        )

    def test_arm_intent_round_trip(self) -> None:
        intent = arm_joint_position_intent(
            sequence=7,
            source="act_policy_eval",
            source_monotonic_ns=123,
            joint_names=("left_shoulder_pan", "left_shoulder_lift"),
            joint_position_rad=(0.1, -0.2),
            joint_max_velocity_rad_s=(0.4, 0.4),
            contract=self.contract,
        )
        restored = intent_from_mapping(intent.as_dict(), self.contract)
        self.assertEqual(restored, intent)

    def test_base_and_hold_intents_are_separate(self) -> None:
        base = base_twist_intent(
            sequence=1,
            source="nav_policy",
            source_monotonic_ns=10,
            linear_x_mps=0.05,
            angular_z_rad_s=-0.1,
            contract=self.contract,
        )
        hold = hold_intent(
            sequence=2,
            source="operator_stop",
            source_monotonic_ns=20,
            contract=self.contract,
        )
        self.assertEqual(base.kind, "base_twist")
        self.assertEqual(hold.kind, "hold")
        self.assertEqual(hold.base_linear_x_mps, 0.0)

    def test_ttl_and_shape_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "ttl_ns exceeds"):
            hold_intent(
                sequence=1,
                source="test",
                source_monotonic_ns=1,
                ttl_ns=self.contract.max_intent_ttl_ns + 1,
                contract=self.contract,
            )
        with self.assertRaisesRegex(ValueError, "identical lengths"):
            arm_joint_position_intent(
                sequence=1,
                source="test",
                source_monotonic_ns=1,
                joint_names=("joint_1",),
                joint_position_rad=(0.0, 0.1),
                joint_max_velocity_rad_s=(0.2,),
                contract=self.contract,
            )

    def test_duplicates_non_finite_and_boolean_integers_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "unique"):
            arm_joint_position_intent(
                sequence=1,
                source="test",
                source_monotonic_ns=1,
                joint_names=("joint_1", "joint_1"),
                joint_position_rad=(0.0, 0.1),
                joint_max_velocity_rad_s=(0.2, 0.2),
                contract=self.contract,
            )
        with self.assertRaisesRegex(ValueError, "finite"):
            base_twist_intent(
                sequence=1,
                source="test",
                source_monotonic_ns=1,
                linear_x_mps=math.nan,
                angular_z_rad_s=0.0,
                contract=self.contract,
            )
        intent = hold_intent(
            sequence=1,
            source="test",
            source_monotonic_ns=1,
            contract=self.contract,
        )
        invalid = intent.__class__(**{**intent.as_dict(), "sequence": True})
        with self.assertRaisesRegex(ValueError, "sequence"):
            validate_intent(invalid, self.contract)

    def test_string_is_not_accepted_as_joint_name_array(self) -> None:
        intent = arm_joint_position_intent(
            sequence=1,
            source="test",
            source_monotonic_ns=1,
            joint_names=("joint_1",),
            joint_position_rad=(0.0,),
            joint_max_velocity_rad_s=(0.2,),
            contract=self.contract,
        )
        invalid = intent.__class__(**{**intent.as_dict(), "joint_names": "joint_1"})
        with self.assertRaisesRegex(ValueError, "joint_names must be an array"):
            validate_intent(invalid, self.contract)

    def test_research_intent_has_no_hardware_authorization_field(self) -> None:
        intent = hold_intent(
            sequence=1,
            source="test",
            source_monotonic_ns=1,
            contract=self.contract,
        )
        self.assertNotIn("hardware_authorized", intent.as_dict())
        self.assertNotIn("device", intent.as_dict())


if __name__ == "__main__":
    unittest.main()
