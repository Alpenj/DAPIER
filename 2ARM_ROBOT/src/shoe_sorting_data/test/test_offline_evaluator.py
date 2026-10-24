import json
from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.offline_evaluator import (
    OfflineEvaluationError,
    build_offline_evaluator_fixture,
    evaluate_action_chunks,
)


class OfflineEvaluatorTest(unittest.TestCase):
    def _evaluate(self, root: Path, *, padded_prediction: float = 999.0):
        manifest = build_offline_evaluator_fixture(root / "fixture", padded_prediction=padded_prediction)
        return evaluate_action_chunks(manifest, root / "report.json")

    def test_padding_is_excluded_and_units_are_not_mixed(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            report = self._evaluate(Path(temp_dir))

            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["mask_accounting"]["valid_timestep_count"], 12)
            self.assertEqual(report["mask_accounting"]["masked_timestep_count"], 6)
            self.assertEqual(report["mask_accounting"]["masked_scalar_count"], 72)
            self.assertEqual(
                [item["valid_step_count"] for item in report["horizon_metrics"]],
                [6, 4, 2],
            )
            self.assertEqual(
                [item["coverage"] for item in report["horizon_metrics"]],
                [1.0, 2 / 3, 1 / 3],
            )
            self.assertAlmostEqual(report["group_metrics"]["all_arm"]["mae"], 0.1)
            self.assertAlmostEqual(report["group_metrics"]["all_gripper"]["mae"], 0.2)
            self.assertIsNone(report["global_mixed_unit_metric"])
            self.assertEqual(report["closed_loop_metrics"]["status"], "NOT_MEASURED")

    def test_changing_only_padded_predictions_never_changes_metrics(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            first = self._evaluate(root / "first", padded_prediction=0.0)
            second = self._evaluate(root / "second", padded_prediction=1_000_000.0)
            self.assertEqual(first["horizon_metrics"], second["horizon_metrics"])
            self.assertEqual(first["group_metrics"], second["group_metrics"])

    def test_valid_action_change_updates_the_matching_group(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            manifest = build_offline_evaluator_fixture(root / "fixture")
            manifest_data = json.loads(manifest.read_text(encoding="utf-8"))
            records = manifest.parent / manifest_data["records_file"]
            rows = [json.loads(line) for line in records.read_text(encoding="utf-8").splitlines()]
            rows[0]["predicted_action"][0][0] += 1.0
            records.write_text(
                "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in rows),
                encoding="utf-8",
            )
            import hashlib

            manifest_data["records_sha256"] = hashlib.sha256(records.read_bytes()).hexdigest()
            manifest.write_text(json.dumps(manifest_data, indent=2) + "\n", encoding="utf-8")

            report = evaluate_action_chunks(manifest, root / "report.json")
            self.assertGreater(report["group_metrics"]["left_arm"]["mae"], 0.1)
            self.assertAlmostEqual(report["group_metrics"]["right_arm"]["mae"], 0.1)

    def test_split_overlap_and_shape_mismatch_fail_closed(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            manifest = build_offline_evaluator_fixture(root / "overlap")
            data = json.loads(manifest.read_text(encoding="utf-8"))
            data["split"]["train_episode_ids"].append(data["split"]["evaluation_episode_ids"][0])
            manifest.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(OfflineEvaluationError, "episode overlap"):
                evaluate_action_chunks(manifest, root / "overlap-report.json")

            manifest = build_offline_evaluator_fixture(root / "shape")
            data = json.loads(manifest.read_text(encoding="utf-8"))
            records = manifest.parent / data["records_file"]
            rows = [json.loads(line) for line in records.read_text(encoding="utf-8").splitlines()]
            rows[0]["action_is_pad"] = [False, False]
            records.write_text(
                "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in rows),
                encoding="utf-8",
            )
            import hashlib

            data["records_sha256"] = hashlib.sha256(records.read_bytes()).hexdigest()
            manifest.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(OfflineEvaluationError, "chunk/mask length"):
                evaluate_action_chunks(manifest, root / "shape-report.json")

    def test_cross_episode_action_window_fails_closed(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            manifest = build_offline_evaluator_fixture(root / "fixture")
            data = json.loads(manifest.read_text(encoding="utf-8"))
            records = manifest.parent / data["records_file"]
            rows = [json.loads(line) for line in records.read_text(encoding="utf-8").splitlines()]
            rows[2]["target_episode_ids"][1] = "eval_episode_002"
            records.write_text(
                "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in rows),
                encoding="utf-8",
            )
            import hashlib

            data["records_sha256"] = hashlib.sha256(records.read_bytes()).hexdigest()
            manifest.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(OfflineEvaluationError, "crosses an episode boundary"):
                evaluate_action_chunks(manifest, root / "report.json")

    def test_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            manifest = build_offline_evaluator_fixture(root / "fixture")
            output = root / "report.json"
            evaluate_action_chunks(manifest, output)
            with self.assertRaisesRegex(OfflineEvaluationError, "overwrite"):
                evaluate_action_chunks(manifest, output)


if __name__ == "__main__":
    unittest.main()
