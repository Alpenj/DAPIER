import ast
import importlib.util
import inspect
from pathlib import Path
import tempfile
import time
import unittest

import shoe_sorting_data.native_act_rollout as rollout
from shoe_sorting_data.dapier_native_act import infer, run_smoke
from shoe_sorting_data.native_act_rollout import (
    SO101RightArmAdapter,
    checkpoint_sha256,
    evaluate_native_act_rollout,
)
from shoe_sorting_data.rollout_safety import SafetySupervisor, build_rollout_safety_fixture


@unittest.skipUnless(
    importlib.util.find_spec("torch") is not None and importlib.util.find_spec("numpy") is not None,
    "optional DAPIER-native ACT ML environment is not installed",
)
class NativeACTRolloutTest(unittest.TestCase):
    def _fixture(self, root: Path, *, approved_hash: str | None = None):
        receipt = run_smoke(root / "act")
        checkpoint = root / "act" / receipt["checkpoint"]["path"]
        raw = root / "act" / "raw"
        preview = infer(raw, checkpoint)
        action = list(preview["action_chunk"][0])
        config = build_rollout_safety_fixture()
        config["approved_policy_checkpoint_sha256"] = approved_hash or checkpoint_sha256(checkpoint)
        config["joint_lower"] = [value - 0.1 for value in action]
        config["joint_upper"] = [value + 0.1 for value in action]
        config["max_delta_per_step"] = [0.01] * 12
        config["max_observation_age_ms"] = 500.0
        config["max_feedback_age_ms"] = 500.0
        supervisor = SafetySupervisor(config)
        supervisor.configure(config["expected_hardware_profile_sha256"])
        supervisor.arm(episode_id=preview["episode_id"], human_approval_id="approval_test")
        supervisor.activate()
        now_ns = time.monotonic_ns()
        snapshot = {
            "now_monotonic_ns": now_ns,
            "observation_monotonic_ns": now_ns - 1_000_000,
            "feedback_monotonic_ns": now_ns - 1_000_000,
            "measured_action": action,
            "base_velocity": [0.0, 0.0],
            "recent_base_command": [0.0, 0.0],
            "e_stop_healthy": True,
            "watchdog_healthy": True,
            "camera_fresh": True,
            "target_valid": True,
            "operator_authorized": True,
        }
        return raw, checkpoint, preview, supervisor, snapshot

    def test_safe_proposal_passes_but_never_publishes(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            raw, checkpoint, preview, supervisor, snapshot = self._fixture(Path(temp_dir))
            trace = evaluate_native_act_rollout(
                raw, checkpoint, supervisor=supervisor, snapshot=snapshot, proposal_sequence=0
            )
            self.assertTrue(trace["decision"]["safety_passed"])
            self.assertFalse(trace["control_authorized"])
            self.assertFalse(trace["decision"]["hardware_dispatch_authorized"])
            self.assertFalse(trace["adapter_result"]["published"])
            self.assertIsNone(trace["executed_action"])
            self.assertEqual(trace["proposal"]["n_action_steps"], 1)
            self.assertEqual(trace["proposal"]["action_index"], 0)
            self.assertEqual(trace["proposal"]["action"][:6], snapshot["measured_action"][:6])
            self.assertEqual(trace["proposal"]["action"][6:], preview["action_chunk"][0][6:])
            self.assertEqual(trace["proposal"]["raw_policy_action"], preview["action_chunk"][0])
            self.assertEqual(trace["proposal"]["phase_mask"], "hold_left_execute_right")
            self.assertEqual(trace["proposal"]["source_observation_id"], f"{preview['episode_id']}:0")
            self.assertTrue(trace["mock_only"])
            self.assertEqual(trace["hardware_execution"], "NOT_ATTEMPTED")

    def test_checkpoint_mismatch_rejects_and_latches_fault(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            raw, checkpoint, _, supervisor, snapshot = self._fixture(Path(temp_dir), approved_hash="0" * 64)
            trace = evaluate_native_act_rollout(
                raw, checkpoint, supervisor=supervisor, snapshot=snapshot, proposal_sequence=0
            )
            self.assertFalse(trace["decision"]["safety_passed"])
            self.assertIn("unapproved_policy", trace["decision"]["reason_codes"])
            self.assertEqual(supervisor.state, "FAULT_LATCHED")
            self.assertFalse(trace["adapter_result"]["published"])

    def test_no_ros_or_hardware_imports(self):
        tree = ast.parse(inspect.getsource(rollout))
        imported_roots = {
            alias.name.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
            for alias in node.names
        }
        self.assertFalse({"rclpy", "rospy", "dynamixel_sdk", "lerobot"} & imported_roots)

    def test_boolean_sequence_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            raw, checkpoint, _, supervisor, snapshot = self._fixture(Path(temp_dir))
            with self.assertRaisesRegex(ValueError, "proposal_sequence"):
                evaluate_native_act_rollout(
                    raw, checkpoint, supervisor=supervisor, snapshot=snapshot, proposal_sequence=True
                )

    def test_checkpoint_to_supervisor_to_mock_right_arm_bus(self):
        class FakeBus:
            def __init__(self):
                self.writes = []

            def sync_write(self, register, values, **kwargs):
                self.writes.append((register, values, kwargs))

        with tempfile.TemporaryDirectory() as temp_dir:
            raw, checkpoint, preview, _, snapshot = self._fixture(Path(temp_dir))
            action = list(preview["action_chunk"][0])
            snapshot["measured_action"] = list(action)
            config = build_rollout_safety_fixture()
            config.update(
                {
                    "hardware_enabled": True,
                    "limit_source": "measured_and_physically_approved",
                    "mechanical_calibration_verified": True,
                    "e_stop_verified": True,
                    "operator_authorization_verified": True,
                    "ros2_controller_contract_verified": True,
                    "approved_policy_checkpoint_sha256": checkpoint_sha256(checkpoint),
                    "joint_lower": [value - 0.1 for value in action],
                    "joint_upper": [value + 0.1 for value in action],
                    "max_delta_per_step": [0.01] * 12,
                    "max_observation_age_ms": 500.0,
                    "max_feedback_age_ms": 500.0,
                }
            )
            supervisor = SafetySupervisor(config)
            supervisor.configure(config["expected_hardware_profile_sha256"])
            supervisor.arm(
                episode_id=preview["episode_id"],
                human_approval_id="approval_mock_bus",
            )
            supervisor.activate()
            trace = evaluate_native_act_rollout(
                raw,
                checkpoint,
                supervisor=supervisor,
                snapshot=snapshot,
                proposal_sequence=0,
            )
            bus = FakeBus()
            result = SO101RightArmAdapter().dispatch(
                trace["decision"], snapshot["measured_action"], bus=bus
            )

            self.assertTrue(result["published"])
            self.assertEqual(len(bus.writes), 1)
            register, values, kwargs = bus.writes[0]
            self.assertEqual(register, "Goal_Position")
            self.assertEqual(
                tuple(values),
                (
                    "shoulder_pan",
                    "shoulder_lift",
                    "elbow_flex",
                    "wrist_flex",
                    "wrist_roll",
                    "gripper",
                ),
            )
            self.assertEqual(kwargs, {"num_retry": 2})
            self.assertAlmostEqual(values["shoulder_pan"], action[6] * 180.0 / 3.141592653589793)
            self.assertAlmostEqual(values["gripper"], action[11] * 100.0)

    def test_right_arm_adapter_rejects_left_arm_motion(self):
        action = [0.0] * 12
        action[6:11] = [0.1] * 5
        action[11] = 0.5
        measured = list(action)
        measured[0] = 0.01
        with self.assertRaisesRegex(ValueError, "left arm"):
            SO101RightArmAdapter().dispatch(
                {
                    "safety_passed": True,
                    "hardware_dispatch_authorized": True,
                    "approved_action": action,
                },
                measured,
                bus=object(),
            )


if __name__ == "__main__":
    unittest.main()
