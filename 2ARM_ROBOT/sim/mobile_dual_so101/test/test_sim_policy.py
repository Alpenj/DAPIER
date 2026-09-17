from __future__ import annotations

from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest(
        "MuJoCo is not installed in this Python environment"
    ) from error

from mobile_dual_so101 import actuator_targets_from_qpos
from shoe_task import ShoeTaskEnv, ShoeTaskConfig
from sim_policy import (
    ActionChunkExecutor,
    HoldChunkPolicy,
    MobileSkillGate,
    actuator_targets_to_policy_action,
    observation_to_policy_state,
    policy_action_to_actuator_targets,
)


class SimPolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = ShoeTaskEnv().model

    def test_policy_action_round_trip_preserves_actuator_targets(self) -> None:
        data = mujoco.MjData(self.model)
        targets = actuator_targets_from_qpos(self.model, data.qpos)
        policy_action = actuator_targets_to_policy_action(self.model, targets)
        self.assertEqual(len(policy_action), 12)
        self.assertTrue(0.0 <= policy_action[5] <= 1.0)
        self.assertTrue(0.0 <= policy_action[11] <= 1.0)
        restored = policy_action_to_actuator_targets(self.model, policy_action)
        for expected, actual in zip(targets, restored):
            self.assertAlmostEqual(expected, actual, places=12)

    def test_invalid_policy_units_fail_closed(self) -> None:
        data = mujoco.MjData(self.model)
        action = list(
            actuator_targets_to_policy_action(
                self.model,
                actuator_targets_from_qpos(self.model, data.qpos),
            )
        )
        action[5] = 1.01
        with self.assertRaisesRegex(ValueError, "normalized gripper"):
            policy_action_to_actuator_targets(self.model, action)
        action[5] = 0.5
        action[0] = 100.0
        with self.assertRaisesRegex(ValueError, "outside"):
            policy_action_to_actuator_targets(self.model, action)

        targets = list(actuator_targets_from_qpos(self.model, data.qpos))
        targets[0] = 100.0
        with self.assertRaisesRegex(ValueError, "actuator target"):
            actuator_targets_to_policy_action(self.model, targets)

    def test_receding_horizon_and_queue_have_explicit_query_counts(self) -> None:
        env = ShoeTaskEnv()
        observation, _ = env.reset(seed=0)
        state = observation_to_policy_state(observation)
        self.assertEqual(len(state), 12)
        hold = actuator_targets_to_policy_action(
            env.model,
            actuator_targets_from_qpos(env.model, env.data.qpos),
        )

        receding = ActionChunkExecutor(
            HoldChunkPolicy(hold, chunk_size=4),
            mode="receding_horizon",
        )
        for _ in range(5):
            receding.next_action(observation)
        self.assertEqual(receding.policy_queries, 5)

        queued = ActionChunkExecutor(
            HoldChunkPolicy(hold, chunk_size=4),
            mode="action_queue",
            n_action_steps=2,
        )
        for _ in range(5):
            queued.next_action(observation)
        self.assertEqual(queued.policy_queries, 3)
        queued.reset()
        queued.next_action(observation)
        self.assertEqual(queued.policy_queries, 4)

    def test_navigation_resets_stale_arm_chunk_and_requires_settled_base(self) -> None:
        env = ShoeTaskEnv(ShoeTaskConfig(initial_home_pose=True))
        observation, _ = env.reset(seed=0)
        current = actuator_targets_from_qpos(env.model, env.data.qpos)
        hold = actuator_targets_to_policy_action(
            env.model,
            current,
        )
        executor = ActionChunkExecutor(
            HoldChunkPolicy(hold, chunk_size=4),
            mode="action_queue",
            n_action_steps=4,
        )
        executor.next_action(observation)
        self.assertEqual(executor.policy_queries, 1)

        gate = MobileSkillGate()
        with self.assertRaisesRegex(RuntimeError, "verified carry-ready"):
            gate.begin_navigation(
                executor,
                model=env.model,
                current_actuator_targets=current,
                transport_hold_action=hold,
                carry_pose_clear=True,
            )
        gate.verify_grasp(object_lifted=True, gripper_holding=True)
        invalid_hold = list(hold)
        invalid_hold[0] = 100.0
        with self.assertRaisesRegex(ValueError, "outside"):
            gate.begin_navigation(
                executor,
                model=env.model,
                current_actuator_targets=current,
                transport_hold_action=invalid_hold,
                carry_pose_clear=True,
            )
        gate.begin_navigation(
            executor,
            model=env.model,
            current_actuator_targets=current,
            transport_hold_action=hold,
            carry_pose_clear=True,
        )
        self.assertFalse(gate.arm_policy_allowed)
        self.assertEqual(gate.transport_hold_action, hold)
        with self.assertRaisesRegex(RuntimeError, "verified transport hold"):
            gate.begin_place(
                executor,
                base_linear_velocity_mps=0.0,
                base_angular_velocity_radps=0.0,
                observation_age_ms=10.0,
            )

        loss_gate = MobileSkillGate()
        loss_gate.verify_grasp(object_lifted=True, gripper_holding=True)
        loss_gate.begin_navigation(
            executor,
            model=env.model,
            current_actuator_targets=current,
            transport_hold_action=hold,
            carry_pose_clear=True,
        )
        with self.assertRaisesRegex(RuntimeError, "grasp lost"):
            loss_gate.monitor_transport_hold(
                observation,
                object_lifted=False,
                gripper_holding=True,
            )
        self.assertEqual(loss_gate.phase, "safe_stopped")

        self.assertEqual(
            gate.monitor_transport_hold(
                observation,
                object_lifted=True,
                gripper_holding=True,
            ),
            hold,
        )
        self.assertTrue(gate.transport_hold_verified)

        with self.assertRaisesRegex(ValueError, "not settled"):
            gate.begin_place(
                executor,
                base_linear_velocity_mps=0.1,
                base_angular_velocity_radps=0.0,
                observation_age_ms=10.0,
            )
        gate.begin_place(
            executor,
            base_linear_velocity_mps=0.0,
            base_angular_velocity_radps=0.0,
            observation_age_ms=10.0,
        )
        self.assertTrue(gate.arm_policy_allowed)
        executor.next_action(observation)
        self.assertEqual(executor.policy_queries, 2)


if __name__ == "__main__":
    unittest.main()
