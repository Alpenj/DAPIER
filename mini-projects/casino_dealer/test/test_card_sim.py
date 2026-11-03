import json
from pathlib import Path
import tempfile
import unittest

from casino_dealer.card_sim import (
    CardSimAction,
    CardSimConfig,
    CardSimState,
    OneCardKinematicEnv,
    Vec3,
    run_one_card_baseline,
    run_one_card_episode,
)
from casino_dealer.card_sim_cli import main


class OneCardKinematicSimTest(unittest.TestCase):

    def test_scripted_baseline_is_deterministic_and_successful(self):
        first = run_one_card_episode(seed=610)
        second = run_one_card_episode(seed=610)

        self.assertEqual(first, second)
        self.assertTrue(first["success"])
        self.assertIsNotNone(first["attachment_step"])
        self.assertLessEqual(first["final_card_target_error_m"], 1e-12)

    def test_held_out_seed_batch_preserves_action_bound(self):
        receipt = run_one_card_baseline(episodes=25, seed=700)

        self.assertEqual(receipt["summary"]["passed"], 25)
        self.assertEqual(receipt["summary"]["success_rate"], 1.0)
        self.assertLessEqual(
            receipt["summary"]["max_abs_action_delta_m"],
            CardSimConfig().max_delta_m,
        )
        self.assertFalse(receipt["claims"]["physical_card_manipulation"])
        self.assertFalse(receipt["claims"]["learned_policy_evaluated"])

    def test_pick_requires_left_deck_stabilization(self):
        env = OneCardKinematicEnv()
        state = env.reset(seed=0)
        pickup = Vec3(
            state.card.x,
            state.card.y,
            state.card.z + env.config.suction_offset_m,
        )
        env._state = CardSimState(
            **{
                **state.__dict__,
                "right_tool": pickup,
            }
        )

        state = env.step(CardSimAction(right_vacuum=1.0))
        self.assertFalse(state.card_attached)

    def test_action_above_bound_is_rejected(self):
        env = OneCardKinematicEnv()
        env.reset(seed=0)

        with self.assertRaisesRegex(ValueError, "max_delta_m"):
            env.step(
                CardSimAction(
                    right_delta=Vec3(env.config.max_delta_m + 0.001, 0, 0)
                )
            )

    def test_cli_writes_receipt_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "receipt.json"
            self.assertEqual(
                main(
                    [
                        "--episodes",
                        "5",
                        "--seed",
                        "900",
                        "--output",
                        str(output),
                    ]
                ),
                0,
            )
            payload = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(payload["summary"]["passed"], 5)

            with self.assertRaises(FileExistsError):
                main(
                    [
                        "--episodes",
                        "1",
                        "--output",
                        str(output),
                    ]
                )


if __name__ == "__main__":
    unittest.main()
