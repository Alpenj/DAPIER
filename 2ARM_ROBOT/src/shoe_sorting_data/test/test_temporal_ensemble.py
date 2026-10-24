import ast
import inspect
import math
from pathlib import Path
import tempfile
import unittest

import shoe_sorting_data.temporal_ensemble as temporal
from shoe_sorting_data.temporal_ensemble import (
    TemporalEnsembleConfig,
    TemporalEnsembleError,
    TemporalEnsembler,
    build_jitter_smoke_report,
    command_stream_metrics,
    main,
)


class TemporalEnsembleTest(unittest.TestCase):
    def test_positive_coefficient_favors_oldest_prediction(self):
        ensembler = TemporalEnsembler(TemporalEnsembleConfig(action_dim=1, chunk_size=3, coefficient=1.0))
        first = ensembler.update([[0.0], [10.0], [20.0]])
        second = ensembler.update([[30.0], [40.0], [50.0]])
        self.assertEqual(first.action, (0.0,))
        self.assertEqual(second.contributor_ages, (1, 0))
        self.assertGreater(second.normalized_weights[0], second.normalized_weights[1])
        expected = (10.0 + math.exp(-1.0) * 30.0) / (1.0 + math.exp(-1.0))
        self.assertAlmostEqual(second.action[0], expected)

    def test_uniform_coefficient_averages_overlapping_predictions(self):
        ensembler = TemporalEnsembler(TemporalEnsembleConfig(action_dim=2, chunk_size=3, coefficient=0.0))
        ensembler.update([[0.0, 0.0], [2.0, 4.0], [4.0, 8.0]])
        result = ensembler.update([[6.0, 10.0], [8.0, 12.0], [10.0, 14.0]])
        self.assertEqual(result.action, (4.0, 7.0))
        self.assertEqual(result.disagreement, (4.0, 6.0))
        self.assertAlmostEqual(sum(result.normalized_weights), 1.0)
        self.assertFalse(result.to_dict()["control_authorized"])

    def test_reset_discards_pending_chunks_and_changes_generation(self):
        ensembler = TemporalEnsembler(TemporalEnsembleConfig(action_dim=1, chunk_size=2))
        ensembler.update([[1.0], [2.0]])
        ensembler.reset()
        result = ensembler.update([[9.0], [10.0]])
        self.assertEqual(result.action, (9.0,))
        self.assertEqual(result.contributor_count, 1)
        self.assertEqual(result.generation, 1)

    def test_shape_nonfinite_and_negative_coefficient_are_rejected(self):
        with self.assertRaisesRegex(TemporalEnsembleError, "negative"):
            TemporalEnsembleConfig(action_dim=1, chunk_size=2, coefficient=-0.1)
        ensembler = TemporalEnsembler(TemporalEnsembleConfig(action_dim=2, chunk_size=2))
        with self.assertRaisesRegex(TemporalEnsembleError, "exactly 2 steps"):
            ensembler.update([[1.0, 2.0]])
        with self.assertRaisesRegex(TemporalEnsembleError, "finite"):
            ensembler.update([[1.0, float("nan")], [2.0, 3.0]])

    def test_stale_timestamp_rejection_is_transactional(self):
        ensembler = TemporalEnsembler(
            TemporalEnsembleConfig(action_dim=1, chunk_size=3, max_source_age_ms=5.0)
        )
        ensembler.update(
            [[1.0], [2.0], [3.0]],
            source_observation_monotonic_ns=10_000_000,
            now_monotonic_ns=11_000_000,
        )
        with self.assertRaisesRegex(TemporalEnsembleError, "stale"):
            ensembler.update(
                [[4.0], [5.0], [6.0]],
                source_observation_monotonic_ns=20_000_000,
                now_monotonic_ns=21_000_000,
            )
        self.assertEqual(len(ensembler), 1)

    def test_disagreement_rejection_is_transactional(self):
        ensembler = TemporalEnsembler(
            TemporalEnsembleConfig(action_dim=1, chunk_size=3, max_disagreement=(0.5,))
        )
        ensembler.update([[0.0], [0.0], [0.0]])
        with self.assertRaisesRegex(TemporalEnsembleError, "disagree"):
            ensembler.update([[2.0], [2.0], [2.0]])
        self.assertEqual(len(ensembler), 1)

    def test_metrics_reject_mixed_dimensions_and_measure_derivatives(self):
        with self.assertRaisesRegex(TemporalEnsembleError, "exactly 1 values"):
            command_stream_metrics([[0.0], [1.0, 2.0]], fps=10.0)
        report = command_stream_metrics([[0.0], [1.0], [3.0], [6.0]], fps=2.0)
        self.assertEqual(report["delta"]["samples"], 3)
        self.assertEqual(report["velocity"]["max_abs"], 6.0)
        self.assertEqual(report["acceleration"]["max_abs"], 4.0)
        self.assertEqual(report["jerk"]["max_abs"], 0.0)

    def test_synthetic_ab_reduces_acceleration_and_jerk(self):
        report = build_jitter_smoke_report()
        self.assertEqual(report["status"], "PASS")
        self.assertGreater(report["comparison"]["acceleration_rms_reduction_percent"], 0)
        self.assertGreater(report["comparison"]["jerk_rms_reduction_percent"], 0)
        self.assertFalse(report["control_authorized"])
        self.assertEqual(report["hardware_execution"], "NOT_ATTEMPTED")

    def test_cli_refuses_overwrite_and_writes_hardware_free_report(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "jitter.json"
            self.assertEqual(main(["--output", str(output)]), 0)
            self.assertIn('"hardware_execution": "NOT_ATTEMPTED"', output.read_text(encoding="utf-8"))
            with self.assertRaisesRegex(TemporalEnsembleError, "already exists"):
                main(["--output", str(output)])

    def test_module_has_no_ros_serial_or_hardware_imports(self):
        tree = ast.parse(inspect.getsource(temporal))
        imported_roots = {
            alias.name.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
            for alias in node.names
        }
        self.assertFalse({"rclpy", "rospy", "serial", "dynamixel_sdk", "lerobot", "torch"} & imported_roots)


if __name__ == "__main__":
    unittest.main()
