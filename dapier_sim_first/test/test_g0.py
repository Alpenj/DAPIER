from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import stat
import tempfile
import unittest

from dapier_sim_first.embodiment import SO101_CHANNEL_NAMES, so101_new_calibration_spec
from dapier_sim_first.gate import (
    EXPECTED_SOURCE_REVISIONS,
    RECORD_ID,
    _frame_contract_checks,
    initialize_g0_manifest,
    run_g0,
)
from dapier_sim_first.protocols import Frame, FrameContractError, validate_frame


CALIBRATION_ID = "sha256:" + "1" * 64


class EmbodimentContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.spec = so101_new_calibration_spec(CALIBRATION_ID)

    def test_exact_six_channel_order(self) -> None:
        self.assertEqual(self.spec.channel_names, SO101_CHANNEL_NAMES)
        self.assertEqual(len(self.spec.channel_names), 6)

    def test_each_channel_round_trips_at_low_mid_high(self) -> None:
        midpoint = tuple(
            (low + high) / 2
            for low, high in zip(
                self.spec.action_lower, self.spec.action_upper, strict=True
            )
        )
        passed = 0
        for index in range(6):
            for value in (
                self.spec.action_lower[index],
                midpoint[index],
                self.spec.action_upper[index],
            ):
                action = list(midpoint)
                action[index] = value
                round_trip = self.spec.sim_to_action(self.spec.action_to_sim(action))
                self.assertAlmostEqual(round_trip[index], value, places=9)
            passed += 1
        self.assertEqual(passed, 6)

    def test_out_of_range_action_is_rejected_instead_of_clipped(self) -> None:
        values = list(self.spec.action_lower)
        values[0] -= 0.01
        with self.assertRaisesRegex(ValueError, "outside the declared bounds"):
            self.spec.action_to_sim(values)


class FrameContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.spec = so101_new_calibration_spec(CALIBRATION_ID)
        self.now_ns = 1_000_000_000
        self.period_ns = 33_333_333
        self.frame = Frame(
            embodiment_id=self.spec.embodiment_id,
            embodiment_revision=self.spec.embodiment_revision,
            channel_names=self.spec.channel_names,
            values=(0.0, 0.0, 0.0, 0.0, 0.0, 50.0),
            units=self.spec.action_units,
            calibration_id=self.spec.calibration_id,
            monotonic_timestamp_ns=self.now_ns - 2 * self.period_ns,
            sequence_id=1,
            source="scripted",
        )

    def test_age_equal_to_two_periods_is_accepted(self) -> None:
        validate_frame(
            self.frame,
            spec=self.spec,
            now_ns=self.now_ns,
            control_period_ns=self.period_ns,
        )

    def test_age_above_two_periods_is_rejected(self) -> None:
        stale = replace(
            self.frame, monotonic_timestamp_ns=self.frame.monotonic_timestamp_ns - 1
        )
        with self.assertRaisesRegex(FrameContractError, "age > 2T"):
            validate_frame(
                stale,
                spec=self.spec,
                now_ns=self.now_ns,
                control_period_ns=self.period_ns,
            )

    def test_full_g0_rejection_suite_has_no_violation(self) -> None:
        result = _frame_contract_checks(self.spec)
        self.assertEqual(result["violation_count"], 0)
        self.assertEqual(
            result["accepted_checks_passed"], result["accepted_checks_total"]
        )
        self.assertEqual(
            result["rejection_checks_passed"], result["rejection_checks_total"]
        )
        self.assertTrue(result["leader_protocol_passed"])


class ManifestSafetyTest(unittest.TestCase):
    def test_manifest_is_new_read_only_and_exactly_pinned(self) -> None:
        repo_root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            model = root / "model.xml"
            calibration = root / "calibration.xml"
            model.write_text("<mujoco/>", encoding="utf-8")
            calibration.write_text("<mujoco/>", encoding="utf-8")
            run_root = root / "new-run"

            manifest_path = initialize_g0_manifest(
                run_root=run_root,
                repo_root=repo_root,
                model_path=model,
                calibration_path=calibration,
            )
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))

            self.assertEqual(payload["record_id"], RECORD_ID)
            self.assertEqual(payload["gate"], "G0")
            self.assertEqual(payload["seed"], 0)
            self.assertEqual(payload["source_revisions"], EXPECTED_SOURCE_REVISIONS)
            self.assertEqual(len(payload["source_revisions"]), 5)
            self.assertFalse(manifest_path.stat().st_mode & stat.S_IWUSR)

            with self.assertRaisesRegex(ValueError, "must not already exist"):
                initialize_g0_manifest(
                    run_root=run_root,
                    repo_root=repo_root,
                    model_path=model,
                    calibration_path=calibration,
                )

    def test_existing_artifact_refuses_gate_before_model_load(self) -> None:
        repo_root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            model = root / "model.xml"
            calibration = root / "calibration.xml"
            model.write_text("<mujoco/>", encoding="utf-8")
            calibration.write_text("<mujoco/>", encoding="utf-8")
            run_root = root / "new-run"
            manifest_path = initialize_g0_manifest(
                run_root=run_root,
                repo_root=repo_root,
                model_path=model,
                calibration_path=calibration,
            )
            (run_root / "old-artifact.json").write_text("{}\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "existing artifact"):
                run_g0(manifest_path=manifest_path, out_path=run_root / "G0")
            self.assertFalse((run_root / "G0").exists())


if __name__ == "__main__":
    unittest.main()
