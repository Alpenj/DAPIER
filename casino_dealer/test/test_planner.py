import json
import unittest

from casino_dealer import Arm, Skill, build_blackjack_opening_plan


class BlackjackOpeningPlanTest(unittest.TestCase):

    def test_three_player_plan_has_expected_deal_order(self):
        plan = build_blackjack_opening_plan(3)

        placements = [
            command
            for command in plan.commands
            if command.skill is Skill.PLACE_CARD
        ]
        self.assertEqual(
            [command.target for command in placements],
            [
                'player_1', 'player_2', 'player_3', 'dealer',
                'player_1', 'player_2', 'player_3', 'dealer',
            ],
        )
        self.assertEqual(
            [command.card_index for command in placements],
            list(range(8)),
        )
        self.assertEqual(
            [command.face_up for command in placements],
            [True, True, True, True, True, True, True, False],
        )

    def test_arms_have_non_overlapping_opening_roles(self):
        plan = build_blackjack_opening_plan(2)

        left_commands = [
            command for command in plan.commands if command.arm is Arm.LEFT
        ]
        right_commands = [
            command for command in plan.commands if command.arm is Arm.RIGHT
        ]

        self.assertEqual(
            [command.skill for command in left_commands],
            [Skill.STABILIZE_DECK, Skill.RELEASE_DECK],
        )
        self.assertEqual(len(right_commands), 12)
        self.assertTrue(
            all(
                command.skill in (Skill.PICK_TOP_CARD, Skill.PLACE_CARD)
                for command in right_commands
            )
        )

    def test_steps_are_contiguous_and_plan_is_json_serializable(self):
        plan = build_blackjack_opening_plan(1)

        self.assertEqual(
            [command.step for command in plan.commands],
            list(range(1, len(plan.commands) + 1)),
        )
        encoded = json.dumps(plan.to_dict())
        self.assertIn('blackjack_opening_deal', encoded)
        self.assertIn('face_up', encoded)

    def test_player_count_bounds_are_enforced(self):
        for invalid_count in (0, 8, -1):
            with self.subTest(invalid_count=invalid_count):
                with self.assertRaises(ValueError):
                    build_blackjack_opening_plan(invalid_count)

    def test_all_supported_table_sizes_preserve_card_invariants(self):
        for player_count in range(1, 8):
            with self.subTest(player_count=player_count):
                plan = build_blackjack_opening_plan(player_count)
                placements = [
                    command
                    for command in plan.commands
                    if command.skill is Skill.PLACE_CARD
                ]

                card_count = 2 * (player_count + 1)
                self.assertEqual(len(placements), card_count)
                self.assertEqual(
                    [command.card_index for command in placements],
                    list(range(card_count)),
                )
                self.assertEqual(
                    sum(command.face_up is False for command in placements),
                    1,
                )
                self.assertEqual(len(plan.commands), 2 * card_count + 2)

    def test_player_count_must_be_an_integer(self):
        for invalid_count in (True, 2.0, '2'):
            with self.subTest(invalid_count=invalid_count):
                with self.assertRaises(TypeError):
                    build_blackjack_opening_plan(invalid_count)


if __name__ == '__main__':
    unittest.main()
