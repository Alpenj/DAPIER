from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco  # noqa: F401
except ModuleNotFoundError as error:
    raise unittest.SkipTest(
        "MuJoCo is not installed in this Python environment"
    ) from error

import sim_episode
from sim_episode import SHOE_DATA_SOURCE, SimEpisodeConfig, record_sim_episode
from shoe_task import UnsafeActionError

sys.path.insert(0, str(SHOE_DATA_SOURCE))
from shoe_sorting_data.camera_payload import read_camera_payload
from shoe_sorting_data.contract import load_manifest
from shoe_sorting_data.quality import validate_episode


UNSAFE_BIMANUAL_TARGET = (
    0.195731791341601,
    1.6758398119082343,
    -0.1950753295290686,
    -0.5266413229575377,
    0.3401861733712628,
    0.4411373258803475,
    -0.485894097852944,
    0.5940617023869152,
    0.4596671975241349,
    0.45149208438578126,
    -1.875765584091561,
    1.0153142196339973,
)


class SimEpisodeTest(unittest.TestCase):
    def test_acceptance_requires_continuous_terminal_carry_evidence(self) -> None:
        self.assertTrue(
            sim_episode._continuous_carry_success([False, True, True], True)
        )
        self.assertFalse(
            sim_episode._continuous_carry_success([False, True, False, True], True)
        )
        self.assertFalse(
            sim_episode._continuous_carry_success([False, True, True], False)
        )
        self.assertFalse(sim_episode._continuous_carry_success([False, False], True))

    def test_records_four_synchronized_cameras_and_12_axis_executed_action(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "episode"
            manifest_path = record_sim_episode(
                output,
                SimEpisodeConfig(
                    episode_id="DAPIER-2026-08-31-mujoco-recorder-smoke",
                    sample_count=3,
                    width=16,
                    height=12,
                    seed=31,
                ),
            )
            manifest = load_manifest(manifest_path)
            self.assertEqual(
                manifest["robot"]["platform"],
                "SO101_dual_arm_on_turtlebot3_waffle_pi",
            )
            self.assertEqual(manifest["recording"]["sample_count"], 3)
            self.assertEqual(manifest["recording"]["expected_period_ns"], 50_000_000)
            self.assertEqual(tuple(manifest["recording"]["camera_streams"]), sim_episode.CAMERA_STREAMS)
            self.assertEqual(manifest["outcome"]["status"], "recorded")
            self.assertFalse(manifest["outcome"]["success"])
            self.assertEqual(manifest["provenance"]["data_origin"], "synthetic")

            samples = [
                json.loads(line)
                for line in (output / "samples.jsonl").read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            self.assertEqual(
                [sample["timestamp_ns"] for sample in samples],
                [0, 50_000_000, 100_000_000],
            )
            for index, sample in enumerate(samples):
                self.assertEqual(len(sample["state"]["left_arm"]), 5)
                self.assertEqual(len(sample["action"]["right_arm"]), 5)
                self.assertEqual(len(sample["action"]["left_gripper"]), 1)
                self.assertFalse(sample["simulation"]["hardware_execution"])
                self.assertFalse(sample["simulation"]["task_success"])
                self.assertFalse(sample["simulation"]["carry_supported"])
                self.assertFalse(sample["simulation"]["bilateral_gripper_contact"])
                self.assertFalse(sample["simulation"]["shoe_gripper_attachment"])
                self.assertEqual(set(sample["cameras"]), set(sim_episode.CAMERA_STREAMS))
                front = read_camera_payload(
                    output,
                    "front_rgb",
                    sample["cameras"]["front_rgb"]["payload"],
                )
                rgb = read_camera_payload(
                    output,
                    "workspace_rgb",
                    sample["cameras"]["workspace_rgb"]["payload"],
                )
                depth = read_camera_payload(
                    output,
                    "workspace_depth",
                    sample["cameras"]["workspace_depth"]["payload"],
                )
                left = read_camera_payload(
                    output,
                    "left_gripper_rgb",
                    sample["cameras"]["left_gripper_rgb"]["payload"],
                )
                self.assertEqual((front.width, front.height, front.encoding), (16, 12, "rgb8"))
                self.assertEqual((rgb.width, rgb.height, rgb.encoding), (16, 12, "rgb8"))
                self.assertEqual((depth.width, depth.height, depth.encoding), (16, 12, "32FC1"))
                self.assertEqual((left.width, left.height, left.encoding), (16, 12, "rgb8"))
                self.assertEqual(sample["cameras"]["workspace_rgb"]["frame_id"], index)
                self.assertTrue(sample["simulation"]["collision_guard"]["safe"])

            report = validate_episode(manifest_path)
            issue_codes = {issue.code for issue in report.issues}
            self.assertEqual(issue_codes, {"outcome_not_accepted"})

    def test_output_overwrite_and_invalid_frequency_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            existing = Path(temp_dir) / "existing"
            existing.mkdir()
            with self.assertRaisesRegex(ValueError, "already exists"):
                record_sim_episode(
                    existing,
                    SimEpisodeConfig(episode_id="DAPIER-2026-08-31-existing"),
                )
            with self.assertRaisesRegex(ValueError, "MuJoCo timestep"):
                record_sim_episode(
                    Path(temp_dir) / "bad-fps",
                    SimEpisodeConfig(
                        episode_id="DAPIER-2026-08-31-bad-fps",
                        fps=333,
                    ),
                )

    def test_unsafe_action_is_rejected_before_episode_finalization(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "unsafe"
            with self.assertRaises(UnsafeActionError) as raised:
                record_sim_episode(
                    output,
                    SimEpisodeConfig(
                        episode_id="DAPIER-2026-09-01-mujoco-unsafe-action",
                        sample_count=2,
                        width=16,
                        height=12,
                    ),
                    action_source=lambda _index, _observation: UNSAFE_BIMANUAL_TARGET,
                )
            self.assertFalse(output.exists())
            self.assertEqual(
                list(Path(temp_dir).glob(".unsafe.staging-*")), []
            )
            self.assertLess(
                raised.exception.assessment.minimum_clearance_m,
                0.03,
            )

    def test_terminal_success_without_prior_carry_is_not_accepted(self) -> None:
        real_task_metrics = sim_episode.task_metrics

        def boundary_metrics(model, data):
            metrics = dict(real_task_metrics(model, data))
            metrics["success"] = float(data.time) >= 0.10
            return metrics

        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "boundary"
            with patch.object(
                sim_episode,
                "task_metrics",
                side_effect=boundary_metrics,
            ):
                manifest_path = record_sim_episode(
                    output,
                    SimEpisodeConfig(
                        episode_id="DAPIER-2026-09-01-mujoco-terminal-boundary",
                        sample_count=3,
                        width=16,
                        height=12,
                    ),
                )
            manifest = load_manifest(manifest_path)
            samples = [
                json.loads(line)
                for line in (output / "samples.jsonl").read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            self.assertTrue(samples[-1]["simulation"]["task_success"])
            self.assertFalse(manifest["outcome"]["success"])

    def test_interrupted_recording_can_retry_same_output(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "retry"
            config = SimEpisodeConfig(
                episode_id="DAPIER-2026-09-01-mujoco-retry",
                sample_count=2,
                width=16,
                height=12,
            )
            with patch.object(
                sim_episode.MuJoCoMultiCameraAdapter,
                "capture",
                side_effect=RuntimeError("render interrupted"),
            ):
                with self.assertRaisesRegex(RuntimeError, "render interrupted"):
                    record_sim_episode(output, config)
            self.assertFalse(output.exists())
            self.assertEqual(list(Path(temp_dir).iterdir()), [])

            manifest_path = record_sim_episode(output, config)
            self.assertEqual(manifest_path, output / "episode_manifest.json")
            self.assertTrue(manifest_path.is_file())


if __name__ == "__main__":
    unittest.main()
