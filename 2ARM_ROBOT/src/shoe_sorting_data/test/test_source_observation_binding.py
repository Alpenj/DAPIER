import copy
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from shoe_sorting_data.rollout_safety import (
    SafetyContractError, SafetySupervisor, build_rollout_safety_fixture, _base_snapshot,
)
from shoe_sorting_data.native_act_rollout import SO101RightArmAdapter, evaluate_native_act_rollout


class SourceObservationBindingTest(unittest.TestCase):
    def setUp(self):
        self.now = 10_000_000_000
        self.config = build_rollout_safety_fixture()
        self.snapshot = _base_snapshot(self.now)
        # This immutable query source is older than the latest safety observation.
        self.binding = dict(version=1, observation_id="episode:7", frame_index=7,
                            capture_monotonic_ns=self.now - 60_000_000,
                            receive_monotonic_ns=self.now - 55_000_000)
        self.snapshot.update(policy_source_observation=self.binding,
                             observation_monotonic_ns=self.now - 1_000_000)
        self.proposal = dict(
            proposal_sequence=0, episode_id="episode", human_approval_id="offline",
            hardware_profile_sha256=self.config["expected_hardware_profile_sha256"],
            policy_checkpoint_sha256=self.config["approved_policy_checkpoint_sha256"],
            policy_reset_generation=0, created_monotonic_ns=self.now - 10_000_000,
            source_observation_id="episode:7", source_frame_index=7,
            source_observation_monotonic_ns=self.binding["capture_monotonic_ns"],
            action=list(self.snapshot["measured_action"]))

    def evaluate(self, proposal, snapshot):
        supervisor = SafetySupervisor(self.config)
        supervisor.configure(self.config["expected_hardware_profile_sha256"])
        supervisor.arm(episode_id="episode", human_approval_id="offline")
        supervisor.activate()
        decision = supervisor.evaluate(proposal, snapshot)
        result = SO101RightArmAdapter().dispatch(decision, snapshot["measured_action"])
        self.assertFalse(decision["hardware_dispatch_authorized"])
        self.assertFalse(result["published"])
        self.assertIsNone(result["executed_action"])
        return supervisor, decision

    def test_valid_older_source_and_current_safety_snapshot_are_independent(self):
        _, decision = self.evaluate(self.proposal, self.snapshot)
        self.assertTrue(decision["safety_passed"])
        self.assertEqual(decision["approved_action"], self.proposal["action"])
        self.assertNotEqual(self.binding["capture_monotonic_ns"], self.snapshot["observation_monotonic_ns"])

    def test_chunk_keeps_source_while_current_safety_snapshot_advances(self):
        self.proposal.update(action_index=0, n_action_steps=2)
        supervisor, first = self.evaluate(self.proposal, self.snapshot)
        self.assertTrue(first["safety_passed"])
        proposal = dict(self.proposal, proposal_sequence=1, action_index=1)
        snapshot = copy.deepcopy(self.snapshot)
        snapshot.update(now_monotonic_ns=self.now + 40_000_000,
                        observation_monotonic_ns=self.now + 39_000_000,
                        feedback_monotonic_ns=self.now + 39_000_000)
        second = supervisor.evaluate(proposal, snapshot)
        self.assertTrue(second["safety_passed"])  # Source age=100ms, proposal age=50ms: exact TTL.
        self.assertFalse(second["hardware_dispatch_authorized"])
        result = SO101RightArmAdapter().dispatch(second, snapshot["measured_action"])
        self.assertFalse(result["published"])
        self.assertIsNone(result["executed_action"])
        snapshot["now_monotonic_ns"] += 1
        proposal["proposal_sequence"] = 2
        expired = supervisor.evaluate(proposal, snapshot)
        self.assertIn("stale_source_observation", expired["reason_codes"])
        self.assertIn("stale_action_proposal", expired["reason_codes"])
        self.assertFalse(expired["hardware_dispatch_authorized"])
        self.assertIsNone(expired["approved_action"])

    def test_missing_malformed_bool_mismatch_and_id_collision_fail_closed(self):
        cases = []
        for field, value in (
            ("source_observation_id", "other:7"), ("source_frame_index", 8),
            ("source_observation_monotonic_ns", self.now - 59_000_000)):
            cases.append(("proposal mismatch " + field, "proposal", field, value))
        for field in ("source_observation_id", "source_frame_index", "source_observation_monotonic_ns"):
            cases.append(("missing " + field, "proposal", field, None))
        for field in ("source_frame_index", "source_observation_monotonic_ns"):
            for value in (True, False, -1, 1.0, "7"):
                cases.append(("malformed " + field + repr(value), "proposal", field, value))
        for target, field in (("proposal", "source_observation_id"), ("binding", "observation_id")):
            for value in ("", "  ", True, 7):
                cases.append(("malformed ID " + repr(value), target, field, value))
        for field in self.binding:
            cases.append(("missing bound " + field, "binding", field, None))
        for field in ("version", "frame_index", "capture_monotonic_ns", "receive_monotonic_ns"):
            for value in (True, -1, "7"):
                cases.append(("malformed bound " + field + repr(value), "binding", field, value))
        for name, target, field, value in cases:
            with self.subTest(name=name):
                proposal, snapshot = copy.deepcopy(self.proposal), copy.deepcopy(self.snapshot)
                mapping = proposal if target == "proposal" else snapshot["policy_source_observation"]
                if value is None:
                    mapping.pop(field)
                else:
                    mapping[field] = value
                supervisor, decision = self.evaluate(proposal, snapshot)
                self.assertFalse(decision["safety_passed"])
                self.assertEqual(supervisor.state, "FAULT_LATCHED")
                self.assertEqual(supervisor.policy_reset_generation, 1)
                self.assertIsNone(decision["approved_action"])
        for value in (None, [], "not a binding", False):
            with self.subTest(binding=value):
                snapshot = dict(self.snapshot, policy_source_observation=value)
                _, decision = self.evaluate(self.proposal, snapshot)
                self.assertFalse(decision["safety_passed"])
        # A repeated ID cannot represent a different episode/frame, even if both sides agree.
        proposal, snapshot = copy.deepcopy(self.proposal), copy.deepcopy(self.snapshot)
        proposal["source_frame_index"] = 8
        snapshot["policy_source_observation"]["frame_index"] = 8
        _, decision = self.evaluate(proposal, snapshot)
        self.assertFalse(decision["safety_passed"])

    def test_source_clock_order_and_ttl_are_separate_from_current_freshness(self):
        cases = (
            ("stale_source_observation", -200_000_000, -190_000_000),
            ("source_observation_in_future", 1_000_000, 2_000_000),
            ("source_capture_after_receive", -20_000_000, -30_000_000),
            ("source_received_after_proposal", -20_000_000, -5_000_000),
        )
        for reason, capture_offset, receive_offset in cases:
            with self.subTest(reason=reason):
                proposal, snapshot = copy.deepcopy(self.proposal), copy.deepcopy(self.snapshot)
                binding = snapshot["policy_source_observation"]
                binding["capture_monotonic_ns"] = self.now + capture_offset
                binding["receive_monotonic_ns"] = self.now + receive_offset
                proposal["source_observation_monotonic_ns"] = binding["capture_monotonic_ns"]
                _, decision = self.evaluate(proposal, snapshot)
                self.assertIn(reason, decision["reason_codes"])
        # A valid source cannot hide stale latest camera/feedback or a stale proposal.
        for field, reason in (("observation_monotonic_ns", "stale_observation"),
                              ("feedback_monotonic_ns", "stale_feedback")):
            snapshot = dict(self.snapshot)
            snapshot[field] = self.now - 200_000_000
            _, decision = self.evaluate(self.proposal, snapshot)
            self.assertIn(reason, decision["reason_codes"])
        proposal = dict(self.proposal, created_monotonic_ns=self.now - 90_000_000)
        _, decision = self.evaluate(proposal, self.snapshot)
        self.assertIn("stale_action_proposal", decision["reason_codes"])

    def test_live_clock_fields_reject_bool_and_accept_real_integer_zero(self):
        proposal, snapshot = copy.deepcopy(self.proposal), copy.deepcopy(self.snapshot)
        snapshot.update(now_monotonic_ns=1, observation_monotonic_ns=1, feedback_monotonic_ns=1)
        snapshot["policy_source_observation"].update(capture_monotonic_ns=0, receive_monotonic_ns=0)
        proposal.update(created_monotonic_ns=1, source_observation_monotonic_ns=0)
        _, valid = self.evaluate(proposal, snapshot)
        self.assertTrue(valid["safety_passed"])
        for target, field in (
            ("snapshot", "now_monotonic_ns"), ("snapshot", "observation_monotonic_ns"),
            ("snapshot", "feedback_monotonic_ns"), ("proposal", "created_monotonic_ns"),
        ):
            for value in (True, False):
                with self.subTest(field=field, value=value):
                    p, s = copy.deepcopy(proposal), copy.deepcopy(snapshot)
                    (p if target == "proposal" else s)[field] = value
                    with self.assertRaisesRegex(SafetyContractError, field):
                        self.evaluate(p, s)

    def test_native_producer_uses_bound_capture_and_preserves_right_arm_phase(self):
        # Unit connector fixture; real native checkpoint roundtrip remains in existing tests.
        now = time.monotonic_ns()
        self.snapshot.update(now_monotonic_ns=now,
                             observation_monotonic_ns=now - 1_000_000,
                             feedback_monotonic_ns=now - 1_000_000)
        self.binding.update(capture_monotonic_ns=now - 20_000_000,
                            receive_monotonic_ns=now - 15_000_000)
        raw = list(self.snapshot["measured_action"])
        raw[0] = 0.1
        preview = dict(control_authorized=False, action_chunk=[raw], episode_id="episode", frame_index=7)
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "checkpoint"
            checkpoint.write_bytes(b"connector unit fixture")
            from shoe_sorting_data.native_act_rollout import checkpoint_sha256
            self.config["approved_policy_checkpoint_sha256"] = checkpoint_sha256(checkpoint)
            supervisor = SafetySupervisor(self.config)
            supervisor.configure(self.config["expected_hardware_profile_sha256"])
            supervisor.arm(episode_id="episode", human_approval_id="offline")
            supervisor.activate()
            with patch("shoe_sorting_data.native_act_rollout.infer", return_value=preview):
                trace = evaluate_native_act_rollout(tmp, checkpoint, supervisor=supervisor,
                                                   snapshot=self.snapshot, proposal_sequence=0)
            self.assertTrue(trace["decision"]["safety_passed"])
            self.assertEqual(trace["proposal"]["source_observation_monotonic_ns"],
                             self.binding["capture_monotonic_ns"])
            self.assertEqual(trace["proposal"]["raw_policy_action"], raw)
            self.assertEqual(trace["proposal"]["action"][:6], self.snapshot["measured_action"][:6])
            self.assertEqual(trace["proposal"]["phase_mask"], "hold_left_execute_right")
            self.assertFalse(trace["control_authorized"])
            self.assertFalse(trace["adapter_result"]["published"])
            self.assertIsNone(trace["executed_action"])


if __name__ == "__main__":
    unittest.main()
