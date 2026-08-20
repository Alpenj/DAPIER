import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.act_interchange import export_act_interchange, verify_act_interchange
from shoe_sorting_data.contract import load_manifest, save_manifest
from shoe_sorting_data.synthetic import generate_dataset, generate_episode


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ActInterchangeTest(unittest.TestCase):
    def test_export_is_immutable_and_uses_train_only_stats(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            episodes = root / "episodes"
            output = root / "act_interchange"
            manifests = generate_dataset(episodes, count=6, sample_count=4)
            source_hashes = {
                path: (_digest(path), _digest(path.parent / "samples.jsonl"))
                for path in manifests
            }

            report = export_act_interchange(episodes, output)

            self.assertEqual(report.episode_count, 6)
            self.assertEqual(report.frame_count, 24)
            self.assertEqual(report.state_dim, 12)
            self.assertEqual(report.action_dim, 12)
            self.assertEqual(report.split_counts, {"train": 5, "validation": 1})
            self.assertTrue(report.act_numeric_contract_ready)
            self.assertFalse(report.native_lerobot_ready)
            self.assertIn("camera_pixel_payload_missing", report.blockers)
            self.assertIn("native_lerobot_dataset_not_encoded", report.blockers)
            stats = json.loads((output / "stats.json").read_text(encoding="utf-8"))
            self.assertEqual(stats["computed_from_split"], "train")
            self.assertEqual(stats["observation.state"]["count"], 20)
            metadata = json.loads((output / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["features"]["action"]["shape"], [12])
            self.assertEqual(metadata["features"]["action"]["units"][:5], ["radian"] * 5)
            self.assertEqual(metadata["features"]["action"]["units"][5], "normalized_position")
            self.assertEqual(metadata["excluded_from_policy"], ["state.base_velocity", "action.base_velocity"])
            preflight = json.loads((output / "preflight.json").read_text(encoding="utf-8"))
            self.assertFalse(preflight["native_conversion_input_ready"])
            self.assertFalse(preflight["native_lerobot_ready"])
            for manifest_path, (manifest_digest, samples_digest) in source_hashes.items():
                self.assertEqual(_digest(manifest_path), manifest_digest)
                self.assertEqual(_digest(manifest_path.parent / "samples.jsonl"), samples_digest)
            self.assertTrue(verify_act_interchange(output)["passed"])

    def test_split_leakage_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            episodes = root / "episodes"
            manifests = generate_dataset(episodes, count=5, sample_count=3)
            train_manifest = load_manifest(manifests[0])
            validation_manifest = load_manifest(manifests[4])
            validation_manifest["provenance"]["object_instance_id"] = train_manifest["provenance"][
                "object_instance_id"
            ]
            save_manifest(manifests[4], validation_manifest)

            with self.assertRaisesRegex(ValueError, "split leakage detected"):
                export_act_interchange(episodes, root / "output")

    def test_unusable_episode_fails_closed(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            generate_episode(root / "episodes" / "episode_bad", sample_count=4, fault="base_motion")
            with self.assertRaisesRegex(ValueError, "not quality accepted"):
                export_act_interchange(root / "episodes", root / "output")

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            generate_episode(root / "episodes" / "episode_001", sample_count=3)
            output = root / "output"
            output.mkdir()
            (output / "keep.txt").write_text("keep", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "output path already exists"):
                export_act_interchange(root / "episodes", output)
            self.assertEqual((output / "keep.txt").read_text(encoding="utf-8"), "keep")

    def test_unit_drift_across_splits_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            manifests = generate_dataset(root / "episodes", count=5, sample_count=3)
            validation_manifest = load_manifest(manifests[4])
            validation_manifest["recording"]["state_streams"]["left_arm"]["unit"] = "degree"
            save_manifest(manifests[4], validation_manifest)
            with self.assertRaisesRegex(ValueError, "policy feature units changed"):
                export_act_interchange(root / "episodes", root / "output")

    def test_verifier_detects_tampering(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            generate_episode(root / "episodes" / "episode_001", sample_count=3)
            output = root / "output"
            export_act_interchange(root / "episodes", output)
            episode_path = next((output / "episodes").rglob("*.jsonl"))
            episode_path.write_text("tampered\n", encoding="utf-8")
            (output / "unexpected.txt").write_text("unexpected\n", encoding="utf-8")
            verification = verify_act_interchange(output)
            self.assertFalse(verification["passed"])
            self.assertTrue(any("hash mismatch" in error for error in verification["errors"]))
            self.assertTrue(any("unexpected output file" in error for error in verification["errors"]))


if __name__ == "__main__":
    unittest.main()
