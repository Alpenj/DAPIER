from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.rollout_safety import (
    JDcobotRos2DryRunAdapter,
    SafetyContractError,
    SafetySupervisor,
    build_rollout_safety_fixture,
    run_rollout_safety_smoke,
    supervise_action,
    validate_safety_config,
)


class RolloutSafetyTest(unittest.TestCase):
    def setUp(self):
        self.config = build_rollout_safety_fixture()
        self.now_ns = 10_000_000_000
        self.proposal = {
            "proposal_id": "p1",
            "policy_query_id": "q1",
            "chunk_id": "c1",
            "action_index": 0,
            "proposal_sequence": 0,
            "episode_id": "episode_001",
            "human_approval_id": "approval_001",
            "hardware_profile_sha256": self.config["expected_hardware_profile_sha256"],
            "policy_checkpoint_sha256": self.config["approved_policy_checkpoint_sha256"],
            "policy_reset_generation": 0,
            "created_monotonic_ns": self.now_ns - 10_000_000,
            "action": [0.05] * 5 + [0.55] + [-0.05] * 5 + [0.45],
        }
        self.snapshot = {
            "now_monotonic_ns": self.now_ns,
            "observation_monotonic_ns": self.now_ns - 20_000_000,
            "feedback_monotonic_ns": self.now_ns - 15_000_000,
            "measured_action": [0.0] * 5 + [0.5] + [0.0] * 5 + [0.5],
            "base_velocity": [0.0, 0.0],
            "recent_base_command": [0.0, 0.0],
            "e_stop_healthy": True,
            "watchdog_healthy": True,
            "camera_fresh": True,
            "target_valid": True,
            "operator_authorized": True,
        }

    def test_safe_proposal_passes_but_can_never_publish(self):
        snapshot = dict(self.snapshot)
        snapshot["expected_policy_reset_generation"] = 0
        decision = supervise_action(self.config, self.proposal, snapshot)
        result = JDcobotRos2DryRunAdapter(self.config).dispatch(decision)

        self.assertTrue(decision["safety_passed"])
        self.assertFalse(decision["hardware_dispatch_authorized"])
        self.assertTrue(decision["action_unchanged"])
        self.assertEqual(result["status"], "SIMULATED_ONLY")
        self.assertFalse(result["published"])
        self.assertIsNone(result["executed_action"])
        self.assertEqual(result["would_publish"]["left"]["positions"], self.proposal["action"][:6])
        self.assertEqual(result["would_publish"]["right"]["positions"], self.proposal["action"][6:])

    def test_each_runtime_interlock_fails_closed(self):
        mutations = {
            "stale_observation": ("observation_monotonic_ns", self.now_ns - 200_000_000),
            "e_stop_not_healthy": ("e_stop_healthy", False),
            "watchdog_not_healthy": ("watchdog_healthy", False),
            "camera_not_fresh": ("camera_fresh", False),
            "target_not_valid": ("target_valid", False),
            "operator_not_authorized": ("operator_authorized", False),
            "base_linear_motion": ("base_velocity", [0.01, 0.0]),
            "base_angular_motion": ("base_velocity", [0.0, 0.01]),
        }
        for reason, (field, value) in mutations.items():
            with self.subTest(reason=reason):
                snapshot = dict(self.snapshot)
                snapshot[field] = value
                snapshot["expected_policy_reset_generation"] = 0
                decision = supervise_action(self.config, self.proposal, snapshot)
                self.assertFalse(decision["safety_passed"])
                self.assertIn(reason, decision["reason_codes"])
                self.assertIsNone(decision["approved_action"])

    def test_joint_limit_and_rate_are_rejected_not_clipped(self):
        proposal = dict(self.proposal)
        proposal["action"] = list(self.proposal["action"])
        proposal["action"][0] = 2.0
        snapshot = dict(self.snapshot)
        snapshot["expected_policy_reset_generation"] = 0
        decision = supervise_action(self.config, proposal, snapshot)
        self.assertFalse(decision["safety_passed"])
        self.assertIn("joint_limit:left_arm_0", decision["reason_codes"])
        self.assertIn("joint_rate:left_arm_0", decision["reason_codes"])
        self.assertIsNone(decision["approved_action"])

    def test_hardware_enable_requires_all_physical_gates(self):
        config = dict(self.config)
        config["hardware_enabled"] = True
        with self.assertRaisesRegex(SafetyContractError, "verified physical gates"):
            validate_safety_config(config)

    def test_fault_latches_and_requires_rearm_with_new_generation(self):
        supervisor = SafetySupervisor(self.config)
        supervisor.configure(self.config["expected_hardware_profile_sha256"])
        supervisor.arm(episode_id="episode_001", human_approval_id="approval_001")
        supervisor.activate()
        stale = dict(self.snapshot)
        stale["observation_monotonic_ns"] = self.now_ns - 200_000_000
        decision = supervisor.evaluate(self.proposal, stale)
        self.assertFalse(decision["safety_passed"])
        self.assertEqual(supervisor.state, "FAULT_LATCHED")
        self.assertEqual(supervisor.policy_reset_generation, 1)

        replay = supervisor.evaluate(self.proposal, self.snapshot)
        self.assertIn("lifecycle_not_active:FAULT_LATCHED", replay["reason_codes"])
        supervisor.reset_fault()
        self.assertEqual(supervisor.state, "INACTIVE")
        with self.assertRaisesRegex(SafetyContractError, "cannot activate"):
            supervisor.activate()

    def test_smoke_trace_has_no_publish_or_hardware_authorization(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "trace.json"
            report = run_rollout_safety_smoke(output)
            self.assertEqual(report["scenario_count"], 6)
            self.assertEqual(report["safety_pass_count"], 1)
            self.assertEqual(report["reject_count"], 5)
            self.assertEqual(report["published_command_count"], 0)
            self.assertEqual(report["hardware_dispatch_authorized_count"], 0)
            with self.assertRaisesRegex(SafetyContractError, "overwrite"):
                run_rollout_safety_smoke(output)


if __name__ == "__main__":
    unittest.main()
