from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco  # noqa: F401
except ModuleNotFoundError as error:
    raise unittest.SkipTest(
        "MuJoCo is not installed in this Python environment"
    ) from error

from sim_episode import SHOE_DATA_SOURCE, SimEpisodeConfig, record_sim_episode

sys.path.insert(0, str(SHOE_DATA_SOURCE))
from shoe_sorting_data.camera_payload import read_camera_payload
from shoe_sorting_data.contract import load_manifest
from shoe_sorting_data.quality import validate_episode


class SimEpisodeTest(unittest.TestCase):
    def test_records_synchronized_rgbd_and_12_axis_executed_action(self) -> None:
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
                self.assertEqual((rgb.width, rgb.height, rgb.encoding), (16, 12, "rgb8"))
                self.assertEqual((depth.width, depth.height, depth.encoding), (16, 12, "32FC1"))
                self.assertEqual(sample["cameras"]["workspace_rgb"]["frame_id"], index)

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


if __name__ == "__main__":
    unittest.main()
