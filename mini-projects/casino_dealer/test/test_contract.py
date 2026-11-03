from copy import deepcopy
import unittest

from casino_dealer import (
    CARD_BENCH_SCHEMA_VERSION,
    load_cardbench_contract,
    validate_cardbench_contract,
)


class CardBenchContractTest(unittest.TestCase):

    def test_packaged_contract_is_valid(self):
        contract = load_cardbench_contract()

        self.assertEqual(
            contract['schema_version'],
            CARD_BENCH_SCHEMA_VERSION,
        )
        self.assertEqual(contract['control_frequency_hz'], 15)
        self.assertTrue(
            contract['observation']['images']['overhead']['required']
        )

    def test_contract_has_ten_action_scalars(self):
        contract = load_cardbench_contract()
        action = contract['action']

        action_width = sum(feature['shape'][0] for feature in action.values())
        self.assertEqual(action_width, 10)
        self.assertEqual(
            action['left_vacuum.command']['range'],
            [0.0, 1.0],
        )

    def test_commanded_joint_state_cannot_masquerade_as_measurement(self):
        contract = load_cardbench_contract()
        invalid = deepcopy(contract)
        invalid['observation']['state'][
            'left_arm.joint_position'
        ]['source'] = 'commanded'

        with self.assertRaisesRegex(ValueError, 'source must be measured'):
            validate_cardbench_contract(invalid)

    def test_joint_shape_drift_is_rejected(self):
        contract = load_cardbench_contract()
        invalid = deepcopy(contract)
        invalid['action']['right_arm.joint_target']['shape'] = [5]

        with self.assertRaisesRegex(ValueError, 'shape must be'):
            validate_cardbench_contract(invalid)


if __name__ == '__main__':
    unittest.main()
