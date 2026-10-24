import json
from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.contract import load_manifest
from shoe_sorting_data.camera_payload import CameraFramePayload, read_camera_payload
from shoe_sorting_data.recorder import ApproximateEpisodeRecorder, REQUIRED_STREAMS


class ApproximateEpisodeRecorderTest(unittest.TestCase):
    def _feed_sample(self, recorder, index, *, skew_ns=3_000_000, with_payload=False):
        timestamp_ns = 1_000_000_000 + index * 50_000_000
        joints = [index * 0.005] * 5 + [0.5]
        emitted = []
        for name in sorted(REQUIRED_STREAMS):
            if name in {"workspace_rgb", "workspace_depth"}:
                offset = -2_000_000 if name == "workspace_rgb" else skew_ns
                emitted.append(
                    recorder.update(
                        name,
                        timestamp_ns=timestamp_ns + offset,
                        frame_id=index,
                        camera_payload=(
                            CameraFramePayload(
                                width=2,
                                height=2,
                                encoding="rgb8" if name == "workspace_rgb" else "16UC1",
                                is_bigendian=0,
                                step=6 if name == "workspace_rgb" else 4,
                                data=(bytes(range(12)) if name == "workspace_rgb" else b"\x01\x00" * 4),
                            )
                            if with_payload
                            else None
                        ),
                    )
                )
            elif name in {"base_velocity", "base_command"}:
                emitted.append(recorder.update(name, timestamp_ns=timestamp_ns, values=[0.0, 0.0]))
            else:
                emitted.append(recorder.update(name, timestamp_ns=timestamp_ns, values=joints))
        return emitted

    def test_synchronized_topics_persist_a_quality_accepted_episode(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = ApproximateEpisodeRecorder(Path(temp_dir) / "mock_episode")
            self.assertEqual(sum(self._feed_sample(recorder, 0)), 1)
            self.assertEqual(sum(self._feed_sample(recorder, 1)), 1)
            manifest_path, report = recorder.finalize(outcome_status="accepted")

            self.assertTrue(report.usable)
            self.assertEqual(report.sample_count, 2)
            self.assertEqual(load_manifest(manifest_path)["outcome"]["status"], "accepted")

    def test_out_of_slop_topics_are_not_recorded(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = ApproximateEpisodeRecorder(
                Path(temp_dir) / "mock_episode",
                max_sync_skew_ns=10_000_000,
            )
            self.assertEqual(sum(self._feed_sample(recorder, 0, skew_ns=20_000_000)), 0)
            self.assertEqual(recorder.sample_count, 0)

    def test_abort_reason_is_written_and_validator_rejects_training_use(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = ApproximateEpisodeRecorder(Path(temp_dir) / "mock_episode")
            self._feed_sample(recorder, 0)
            self._feed_sample(recorder, 1)
            manifest_path, report = recorder.finalize(
                outcome_status="aborted",
                failure_reason="operator_stop",
            )

            manifest = load_manifest(manifest_path)
            self.assertEqual(manifest["outcome"]["failure_reason"], "operator_stop")
            self.assertFalse(report.usable)
            self.assertIn("outcome_not_accepted", {issue.code for issue in report.errors})

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            episode_dir = Path(temp_dir) / "mock_episode"
            episode_dir.mkdir()
            (episode_dir / "notes.txt").write_text("keep", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "not empty"):
                ApproximateEpisodeRecorder(episode_dir)
            self.assertEqual((episode_dir / "notes.txt").read_text(encoding="utf-8"), "keep")

    def test_required_camera_payload_is_persisted_and_verified(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            episode_dir = Path(temp_dir) / "payload_episode"
            recorder = ApproximateEpisodeRecorder(episode_dir, require_camera_payload=True)
            self._feed_sample(recorder, 0, with_payload=True)
            self._feed_sample(recorder, 1, with_payload=True)
            manifest_path, report = recorder.finalize(outcome_status="accepted")

            self.assertTrue(report.usable, report.to_dict())
            manifest = load_manifest(manifest_path)
            self.assertEqual(manifest["recording"]["camera_payload"]["mode"], "required")
            first_sample = json.loads(
                (episode_dir / "samples.jsonl").read_text(encoding="utf-8").splitlines()[0]
            )
            rgb = read_camera_payload(
                episode_dir,
                "workspace_rgb",
                first_sample["cameras"]["workspace_rgb"]["payload"],
            )
            self.assertEqual(rgb.encoding, "rgb8")

    def test_required_camera_payload_rejects_metadata_only_update(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = ApproximateEpisodeRecorder(
                Path(temp_dir) / "payload_episode",
                require_camera_payload=True,
            )
            with self.assertRaisesRegex(ValueError, "requires a camera pixel payload"):
                recorder.update("workspace_rgb", timestamp_ns=1, frame_id=0)


if __name__ == "__main__":
    unittest.main()
