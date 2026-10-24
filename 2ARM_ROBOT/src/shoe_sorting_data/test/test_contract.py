import hashlib
from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.contract import build_manifest, load_manifest, save_manifest, validate_manifest


class EpisodeContractTest(unittest.TestCase):
    def test_default_dimensions_match_observed_jdcobot200(self):
        manifest = build_manifest(
            episode_id="episode_000001",
            sample_count=10,
            samples_sha256=hashlib.sha256(b"samples").hexdigest(),
        )
        self.assertEqual(manifest["recording"]["state_streams"]["left_arm"]["dimension"], 5)
        self.assertEqual(manifest["recording"]["state_streams"]["left_gripper"]["dimension"], 1)
        self.assertEqual(
            manifest["robot"]["platform"],
            "JDcobot200_dual_arm_on_turtlebot3_waffle_pi",
        )
        self.assertEqual(
            manifest["quality_limits"]["base_linear_stationary_tolerance_mps"], 0.0025
        )
        self.assertEqual(
            manifest["quality_limits"]["base_angular_stationary_tolerance_radps"], 0.0021
        )
        self.assertEqual(manifest["recording"]["camera_payload"]["mode"], "required")
        self.assertEqual(manifest["lifecycle"], {"state": "finalized", "integrity_verified": True})

    def test_round_trip_preserves_configurable_dimensions(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = build_manifest(
                episode_id="episode_000001",
                sample_count=10,
                samples_sha256=hashlib.sha256(b"samples").hexdigest(),
                arm_dof=7,
                gripper_dof=2,
            )
            path = Path(temp_dir) / "episode_manifest.json"
            save_manifest(path, manifest)
            loaded = load_manifest(path)
            self.assertEqual(loaded["recording"]["action_streams"]["left_arm"]["dimension"], 7)
            self.assertEqual(loaded["recording"]["state_streams"]["right_gripper"]["dimension"], 2)

    def test_rejected_episode_requires_reason(self):
        manifest = build_manifest(
            episode_id="episode_000001",
            sample_count=10,
            samples_sha256=hashlib.sha256(b"samples").hexdigest(),
        )
        manifest["outcome"] = {"status": "rejected", "success": False, "failure_reason": None}
        with self.assertRaisesRegex(ValueError, "failure_reason"):
            validate_manifest(manifest)

    def test_unfinalized_v03_manifest_is_rejected(self):
        manifest = build_manifest(
            episode_id="episode_000001",
            sample_count=10,
            samples_sha256=hashlib.sha256(b"samples").hexdigest(),
        )
        manifest["lifecycle"]["state"] = "recording"
        with self.assertRaisesRegex(ValueError, "finalized"):
            validate_manifest(manifest)


if __name__ == "__main__":
    unittest.main()
