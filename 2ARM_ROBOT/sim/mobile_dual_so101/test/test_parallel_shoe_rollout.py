from __future__ import annotations

from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco  # noqa: F401
except ModuleNotFoundError as error:
    raise unittest.SkipTest(
        "MuJoCo is not installed in this Python environment"
    ) from error

from parallel_shoe_rollout import ParallelRolloutConfig, run_parallel_rollouts


class ParallelShoeRolloutTest(unittest.TestCase):
    def test_invalid_parallel_configuration_is_rejected(self) -> None:
        for config in (
            ParallelRolloutConfig(workers=0),
            ParallelRolloutConfig(episodes=0),
            ParallelRolloutConfig(steps_per_episode=0),
        ):
            with self.assertRaises(ValueError):
                config.validate()

    def test_two_process_tower_rollout_is_finite_and_simulation_only(self) -> None:
        report = run_parallel_rollouts(
            ParallelRolloutConfig(
                workers=2,
                episodes=2,
                steps_per_episode=5,
            )
        )
        self.assertEqual(report["workers_used"], 2)
        self.assertEqual(len(report["worker_pids"]), 2)
        self.assertTrue(report["independent_processes"])
        self.assertTrue(report["finite_observations"])
        self.assertFalse(report["hardware_execution"])
        self.assertEqual(report["mount_layout"], "tower")
        self.assertEqual(report["observation_dimension"], 21)
        self.assertEqual(report["action_dimension"], 12)
        self.assertEqual(report["transitions"], 10)


if __name__ == "__main__":
    unittest.main()
