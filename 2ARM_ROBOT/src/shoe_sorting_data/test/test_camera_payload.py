import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.camera_payload import (
    CameraFramePayload,
    CameraPayloadError,
    read_camera_payload,
    write_camera_payload,
)
from shoe_sorting_data.contract import load_manifest, save_manifest
from shoe_sorting_data.quality import validate_episode
from shoe_sorting_data.synthetic import generate_episode


class CameraPayloadTest(unittest.TestCase):
    def test_rgb_and_depth_raw_rows_round_trip(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir) / "episode"
            rgb = CameraFramePayload(2, 2, "rgb8", 0, 6, bytes(range(12)))
            depth = CameraFramePayload(2, 2, "16UC1", 0, 4, b"\x01\x00\x02\x00" * 2)

            rgb_meta = write_camera_payload(root, "workspace_rgb", 0, rgb)
            depth_meta = write_camera_payload(root, "workspace_depth", 0, depth)

            self.assertEqual(read_camera_payload(root, "workspace_rgb", rgb_meta), rgb)
            self.assertEqual(read_camera_payload(root, "workspace_depth", depth_meta), depth)
            self.assertEqual(rgb_meta["storage"], "ros2_raw_rows")
            self.assertEqual(depth_meta["byte_count"], 8)

    def test_geometry_and_stream_encoding_are_fail_closed(self):
        with self.assertRaisesRegex(CameraPayloadError, "step"):
            CameraFramePayload(4, 2, "rgb8", 0, 4, b"12345678")
        with tempfile.TemporaryDirectory() as temp_dir:
            payload = CameraFramePayload(2, 2, "rgb8", 0, 6, bytes(range(12)))
            with self.assertRaisesRegex(CameraPayloadError, "does not accept"):
                write_camera_payload(Path(temp_dir), "workspace_depth", 0, payload)

    def test_unsafe_path_and_overwrite_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            payload = CameraFramePayload(2, 2, "rgb8", 0, 6, bytes(range(12)))
            metadata = write_camera_payload(root, "workspace_rgb", 0, payload)
            with self.assertRaisesRegex(CameraPayloadError, "already exists"):
                write_camera_payload(root, "workspace_rgb", 0, payload)
            metadata["path"] = "../escape.raw"
            with self.assertRaisesRegex(CameraPayloadError, "unsafe"):
                read_camera_payload(root, "workspace_rgb", metadata)

    def test_quality_gate_detects_missing_and_tampered_payloads(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            missing_manifest_path = generate_episode(root / "missing", sample_count=3)
            missing_manifest = load_manifest(missing_manifest_path)
            missing_manifest["recording"]["camera_payload"]["mode"] = "required"
            save_manifest(missing_manifest_path, missing_manifest)
            missing_codes = {issue.code for issue in validate_episode(missing_manifest_path).errors}
            self.assertIn("camera_payload_missing", missing_codes)

            tampered_manifest_path = generate_episode(
                root / "tampered",
                sample_count=3,
                include_camera_payload=True,
            )
            first_sample = json.loads(
                (tampered_manifest_path.parent / "samples.jsonl").read_text(encoding="utf-8").splitlines()[0]
            )
            payload_path = tampered_manifest_path.parent / first_sample["cameras"]["workspace_rgb"]["payload"]["path"]
            payload_path.write_bytes(b"tampered")
            tampered_codes = {issue.code for issue in validate_episode(tampered_manifest_path).errors}
            self.assertIn("camera_payload_size_mismatch", tampered_codes)

    def test_synthetic_payload_episode_is_quality_accepted(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = generate_episode(
                Path(temp_dir) / "episode",
                sample_count=4,
                include_camera_payload=True,
            )
            report = validate_episode(manifest_path)
            self.assertTrue(report.usable, report.to_dict())
            manifest = load_manifest(manifest_path)
            self.assertEqual(manifest["recording"]["camera_payload"]["mode"], "required")

    def test_sync_delta_tampering_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = generate_episode(
                Path(temp_dir) / "episode",
                sample_count=3,
                include_camera_payload=True,
            )
            manifest = load_manifest(manifest_path)
            sample_path = manifest_path.parent / "samples.jsonl"
            samples = [json.loads(line) for line in sample_path.read_text(encoding="utf-8").splitlines()]
            samples[1]["timing"]["sync_delta_ns"] += 1
            payload = "".join(
                json.dumps(sample, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n"
                for sample in samples
            ).encode("utf-8")
            sample_path.write_bytes(payload)
            manifest["checksums"]["samples_sha256"] = hashlib.sha256(payload).hexdigest()
            save_manifest(manifest_path, manifest)
            codes = {issue.code for issue in validate_episode(manifest_path).errors}
            self.assertIn("sync_delta_mismatch", codes)


if __name__ == "__main__":
    unittest.main()
