from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.base_baseline import (
    derive_stationary_tolerances,
    save_baseline,
    summarize_samples,
)


class BaseBaselineTest(unittest.TestCase):
    def setUp(self):
        self.samples = [
            {
                "reception_monotonic_ns": 1_000_000_000 + index * 100_000_000,
                "message_timestamp_ns": 2_000_000_000 + index * 100_000_000,
                "linear_x_mps": value,
                "angular_z_radps": value * 2,
            }
            for index, value in enumerate((0.0, 0.001, -0.002, 0.003))
        ]

    def test_summary_keeps_linear_and_angular_noise_separate(self):
        summary = summarize_samples(self.samples)
        self.assertEqual(summary["sample_count"], 4)
        self.assertAlmostEqual(summary["sample_rate_hz"], 10.0)
        self.assertEqual(summary["linear_x_mps"]["abs_max"], 0.003)
        self.assertEqual(summary["angular_z_radps"]["abs_max"], 0.006)

    def test_save_writes_checksum_report_without_overwriting(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "baseline"
            samples_path, report_path = save_baseline(
                output,
                self.samples,
                source_topic="/odom",
                warmup_seconds=3.0,
            )
            self.assertTrue(samples_path.is_file())
            self.assertTrue(report_path.is_file())
            with self.assertRaisesRegex(ValueError, "not empty"):
                save_baseline(output, self.samples, source_topic="/odom", warmup_seconds=3.0)

    def test_summary_rejects_non_monotonic_reception_time(self):
        invalid = [dict(sample) for sample in self.samples]
        invalid[2]["reception_monotonic_ns"] = invalid[1]["reception_monotonic_ns"]
        with self.assertRaisesRegex(ValueError, "increase strictly"):
            summarize_samples(invalid)

    def test_tolerances_use_separate_conservative_margins(self):
        summary = {
            "linear_x_mps": {"abs_max": 0.00123153, "abs_p99": 0.000656899},
            "angular_z_radps": {"abs_max": 0.000831313, "abs_p99": 0.00066736},
        }
        self.assertEqual(
            derive_stationary_tolerances(summary),
            {"linear_x_mps": 0.0025, "angular_z_radps": 0.0021},
        )


if __name__ == "__main__":
    unittest.main()
