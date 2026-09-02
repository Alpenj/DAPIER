from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import json
import unittest


RESEARCH_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESEARCH_ROOT / "src"))

from dapier_research.observation_contract import (  # noqa: E402
    OBSERVATION_PROVENANCE_SCHEMA_VERSION,
    attach_sensor_runtime_provenance,
    attach_simulator_privileged_provenance,
    validate_observation_for_use,
)
from dapier_research.sim_to_real_dataset import (  # noqa: E402
    validate_sim_to_real_episode,
    validate_sim_to_real_episode_files,
)
from dapier_research.sim_to_real_policy import SensorPolicyExecutor  # noqa: E402


def sensor_observation(*, target_available: bool = True):
    observation = {
        "schema_version": "dapier.test-observation.v1",
        "ground_truth": False,
        "robot": {
            "left_arm_rad": [0.0] * 5,
            "left_gripper_normalized": 0.5,
            "right_arm_rad": [0.0] * 5,
            "right_gripper_normalized": 0.5,
        },
        "target_available": target_available,
        "target": (
            {
                "position_target_m": [0.3, 0.0, 0.07],
                "ground_truth_used": False,
                "uses_privileged_labels": False,
            }
            if target_available
            else None
        ),
        "failure": None if target_available else {"code": "detection_not_found"},
        "control_authorized": False,
        "hardware_execution": False,
    }
    return attach_sensor_runtime_provenance(
        observation,
        producer="unit_rgbd_adapter",
        sensor_frames=("front_camera_optical", "joint_state"),
    )


class FakeDelegate:
    def __init__(self) -> None:
        self.policy_queries = 0
        self.executed_actions = 0

    def reset(self) -> None:
        return None

    def next_action(self, observation):
        del observation
        self.policy_queries += 1
        self.executed_actions += 1
        return (0.0,) * 12


class ObservationContractTest(unittest.TestCase):
    def test_sensor_runtime_is_accepted_for_policy_and_training(self) -> None:
        observation = sensor_observation()
        policy = validate_observation_for_use(observation, use="policy_runtime")
        training = validate_observation_for_use(observation, use="training_episode")
        self.assertEqual(policy.source_kind, "sensor_runtime")
        self.assertEqual(
            training.sensor_frames,
            ("front_camera_optical", "joint_state"),
        )

    def test_detection_failure_remains_sensor_observation_without_truth_fallback(self) -> None:
        observation = sensor_observation(target_available=False)
        provenance = validate_observation_for_use(observation, use="policy_runtime")
        self.assertFalse(observation["target_available"])
        self.assertIsNone(observation["target"])
        self.assertFalse(provenance.ground_truth_used)

    def test_privileged_observation_is_evaluation_only(self) -> None:
        observation = attach_simulator_privileged_provenance(
            {
                "schema_version": "dapier.legacy-shoe-task.v1",
                "ground_truth": True,
                "shoe": {"position_map_m": [0.2, 0.0, 0.01]},
                "robot": {},
            },
            producer="mujoco_state",
            privileged_fields=("shoe.position_map_m",),
        )
        validate_observation_for_use(observation, use="evaluation_oracle")
        with self.assertRaisesRegex(ValueError, "requires sensor_runtime"):
            validate_observation_for_use(observation, use="policy_runtime")
        with self.assertRaisesRegex(ValueError, "requires sensor_runtime"):
            validate_observation_for_use(observation, use="training_episode")

    def test_missing_or_lying_provenance_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "explicit observation_provenance"):
            validate_observation_for_use(
                {"ground_truth": False, "robot": {}},
                use="policy_runtime",
            )
        observation = sensor_observation()
        observation["target"]["ground_truth_used"] = True
        with self.assertRaisesRegex(ValueError, "truth flag must be false"):
            validate_observation_for_use(observation, use="policy_runtime")

    def test_runtime_forbidden_simulator_keys_are_rejected(self) -> None:
        observation = sensor_observation()
        observation["simulator_object_pose"] = [0.2, 0.0, 0.01]
        with self.assertRaisesRegex(ValueError, "privileged field"):
            validate_observation_for_use(observation, use="control_monitor")

    def test_sensor_policy_wrapper_rejects_before_delegate_query(self) -> None:
        delegate = FakeDelegate()
        executor = SensorPolicyExecutor(delegate)
        with self.assertRaisesRegex(ValueError, "explicit observation_provenance"):
            executor.next_action({"ground_truth": True, "shoe": {}, "robot": {}})
        self.assertEqual(delegate.policy_queries, 0)
        self.assertEqual(len(executor.next_action(sensor_observation())), 12)
        self.assertEqual(delegate.policy_queries, 1)

    def test_dataset_gate_rejects_legacy_and_accepts_sensor_samples(self) -> None:
        base_manifest = {
            "recording": {
                "sample_count": 1,
                "camera_payload": {"mode": "required"},
            },
            "provenance": {},
        }
        legacy_sample = {
            "state": {},
            "simulation": {"hardware_execution": False},
        }
        with self.assertRaisesRegex(ValueError, "not marked"):
            validate_sim_to_real_episode(base_manifest, [legacy_sample])

        manifest = {
            **base_manifest,
            "provenance": {
                "sim_to_real_observation_compatible": True,
                "ground_truth_used_for_policy": False,
                "observation_provenance_schema_version": (
                    OBSERVATION_PROVENANCE_SCHEMA_VERSION
                ),
            },
        }
        sample = {
            "policy_observation": sensor_observation(),
            "simulation": {"hardware_execution": False},
        }
        report = validate_sim_to_real_episode(manifest, [sample])
        self.assertTrue(report.accepted)
        self.assertEqual(report.sample_count, 1)
        self.assertFalse(report.ground_truth_used_for_policy)
        self.assertFalse(report.hardware_execution)

    def test_file_gate_uses_the_same_contract(self) -> None:
        manifest = {
            "recording": {
                "sample_count": 1,
                "camera_payload": {"mode": "required"},
            },
            "provenance": {
                "sim_to_real_observation_compatible": True,
                "ground_truth_used_for_policy": False,
                "observation_provenance_schema_version": (
                    OBSERVATION_PROVENANCE_SCHEMA_VERSION
                ),
            },
        }
        sample = {
            "policy_observation": sensor_observation(),
            "simulation": {"hardware_execution": False},
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            manifest_path = root / "manifest.json"
            samples_path = root / "samples.jsonl"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            samples_path.write_text(json.dumps(sample) + "\n", encoding="utf-8")
            report = validate_sim_to_real_episode_files(manifest_path, samples_path)
        self.assertTrue(report.accepted)


if __name__ == "__main__":
    unittest.main()
