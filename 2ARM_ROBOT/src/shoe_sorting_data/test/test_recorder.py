from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.contract import load_manifest
from shoe_sorting_data.recorder import ApproximateEpisodeRecorder, REQUIRED_STREAMS


class ApproximateEpisodeRecorderTest(unittest.TestCase):
    def _feed_sample(self, recorder, index, *, skew_ns=3_000_000):
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


if __name__ == "__main__":
    unittest.main()
