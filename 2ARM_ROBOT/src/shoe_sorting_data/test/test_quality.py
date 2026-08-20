import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.quality import validate_episode
from shoe_sorting_data.contract import load_manifest, save_manifest
from shoe_sorting_data.synthetic import generate_episode


class EpisodeQualityTest(unittest.TestCase):
    def test_golden_episode_passes_all_hard_gates(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = generate_episode(Path(temp_dir) / "episode_000001")
            report = validate_episode(manifest)
            self.assertTrue(report.usable, report.to_dict())
            self.assertEqual(report.sample_count, 40)
            self.assertGreater(report.duration_ns, 0)

    def test_same_seed_produces_identical_episode_bytes(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            first = generate_episode(Path(temp_dir) / "first" / "episode_000001", seed=77)
            second = generate_episode(Path(temp_dir) / "second" / "episode_000001", seed=77)
            self.assertEqual(first.read_bytes(), second.read_bytes())
            self.assertEqual(
                (first.parent / "samples.jsonl").read_bytes(),
                (second.parent / "samples.jsonl").read_bytes(),
            )

    def test_known_faults_are_rejected_with_expected_codes(self):
        expectations = {
            "base_motion": "base_interlock_violation",
            "camera_frame_gap": "camera_frame_gap",
            "camera_skew": "camera_skew_exceeded",
            "checksum_mismatch": "checksum_mismatch",
            "dimension_mismatch": "stream_dimension_mismatch",
            "duplicate_timestamp": "timestamp_not_monotonic",
            "joint_jump": "joint_step_exceeded",
            "missing_camera": "camera_missing",
            "sample_gap": "sample_gap_exceeded",
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            for index, (fault, expected_code) in enumerate(expectations.items()):
                with self.subTest(fault=fault):
                    manifest = generate_episode(
                        Path(temp_dir) / f"episode_{index:06d}",
                        fault=fault,
                    )
                    report = validate_episode(manifest)
                    self.assertFalse(report.usable)
                    self.assertIn(expected_code, {issue.code for issue in report.errors})

    def test_manifest_sample_count_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = generate_episode(Path(temp_dir) / "episode_000001", sample_count=4)
            sample_path = manifest.parent / "samples.jsonl"
            lines = sample_path.read_text(encoding="utf-8").splitlines()
            sample_path.write_text("\n".join(lines[:-1]) + "\n", encoding="utf-8")
            report = validate_episode(manifest)
            codes = {issue.code for issue in report.errors}
            self.assertIn("sample_count_mismatch", codes)
            self.assertIn("checksum_mismatch", codes)

    def test_stationary_gate_uses_separate_linear_and_angular_tolerances(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = generate_episode(Path(temp_dir) / "episode_000001", sample_count=4)
            manifest = load_manifest(manifest_path)
            sample_path = manifest_path.parent / "samples.jsonl"
            samples = [json.loads(line) for line in sample_path.read_text(encoding="utf-8").splitlines()]
            samples[1]["state"]["base_velocity"] = [0.0024, 0.0022]
            payload = "".join(
                json.dumps(sample, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n"
                for sample in samples
            ).encode("utf-8")
            sample_path.write_bytes(payload)
            manifest["checksums"]["samples_sha256"] = hashlib.sha256(payload).hexdigest()
            save_manifest(manifest_path, manifest)
            report = validate_episode(manifest_path)
            self.assertIn("base_interlock_violation", {issue.code for issue in report.errors})

    def test_pending_hardware_versions_and_review_are_not_training_usable(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = generate_episode(Path(temp_dir) / "episode_000001")
            manifest = load_manifest(manifest_path)
            manifest["provenance"]["data_origin"] = "robot"
            manifest["robot"]["calibration_version"] = "pending_calibration"
            manifest["robot"]["robot_config_version"] = "pending_hardware_introspection"
            manifest["outcome"] = {"status": "recorded", "success": None, "failure_reason": None}
            save_manifest(manifest_path, manifest)
            report = validate_episode(manifest_path)
            codes = [issue.code for issue in report.errors]
            self.assertIn("outcome_not_accepted", codes)
            self.assertEqual(codes.count("hardware_version_unresolved"), 2)


if __name__ == "__main__":
    unittest.main()
