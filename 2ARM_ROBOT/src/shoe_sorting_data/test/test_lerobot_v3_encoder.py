import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

from shoe_sorting_data.lerobot_v3_encoder import (
    NativeEncoderDependencyError,
    build_native_encoder_plan,
    encode_native_lerobot_v3,
    native_dependency_status,
)
from shoe_sorting_data.synthetic import generate_dataset, generate_episode


class LeRobotV3EncoderTest(unittest.TestCase):
    def test_preflight_is_pure_and_builds_narrow_native_schema(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            episodes = root / "episodes"
            manifests = generate_dataset(
                episodes,
                count=2,
                sample_count=3,
                include_camera_payload=True,
            )
            before = {
                path: hashlib.sha256(path.read_bytes()).hexdigest()
                for manifest in manifests
                for path in (manifest, manifest.parent / "samples.jsonl")
            }
            lerobot_was_loaded = "lerobot" in sys.modules

            plan = build_native_encoder_plan(episodes, depth_unit="mm")

            self.assertEqual(plan.episode_count, 2)
            self.assertEqual(plan.frame_count, 6)
            self.assertEqual(plan.state_dim, 12)
            self.assertEqual(plan.action_dim, 12)
            self.assertEqual(plan.rgb_shape_hwc, (6, 8, 3))
            self.assertEqual(plan.depth_shape_hwc, (6, 8, 1))
            self.assertEqual(plan.fps, 20)
            self.assertEqual("lerobot" in sys.modules, lerobot_was_loaded)
            for path, digest in before.items():
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)

    def test_depth_unit_and_payload_are_explicit_preconditions(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            payload_episode = generate_episode(
                root / "payload",
                sample_count=3,
                include_camera_payload=True,
            )
            with self.assertRaisesRegex(ValueError, "depth_unit"):
                build_native_encoder_plan(payload_episode.parent, depth_unit="unknown")

            metadata_only = generate_episode(root / "metadata_only", sample_count=3)
            with self.assertRaisesRegex(ValueError, "requires RGB-D pixel payloads"):
                build_native_encoder_plan(metadata_only.parent, depth_unit="mm")

    def test_dependency_status_never_changes_base_recorder_contract(self):
        status = native_dependency_status()
        self.assertIn("available", status)
        self.assertEqual(set(status["modules"]), {"lerobot", "numpy", "PIL", "torch", "datasets", "pyarrow"})
        self.assertFalse(status["base_recorder_affected"])

    def test_native_export_reports_missing_optional_stack_cleanly(self):
        status = native_dependency_status()
        if status["available"]:
            self.skipTest("native stack is installed; Stage 3 integration covers the real backend")
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            episodes = root / "episodes"
            generate_episode(episodes / "episode_001", sample_count=3, include_camera_payload=True)
            with self.assertRaisesRegex(NativeEncoderDependencyError, "dependencies are missing"):
                encode_native_lerobot_v3(
                    episodes,
                    root / "native",
                    repo_id="local/dapier-shoe-smoke",
                    depth_unit="mm",
                )
            self.assertFalse((root / "native").exists())


if __name__ == "__main__":
    unittest.main()
