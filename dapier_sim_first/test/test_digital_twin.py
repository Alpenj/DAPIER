from __future__ import annotations

from dataclasses import replace
import json
import math
import unittest

from dapier_sim_first.digital_twin import (
    DigitalTwinContractError,
    JointTrace,
    TwinThresholds,
    evaluate_digital_twin,
)
from dapier_sim_first.embodiment import SO101_CHANNEL_NAMES


CONTRACT_ID = "sha256:" + "1" * 64


def _command_rows(samples: int) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(step * 0.01 * (joint + 1) for joint in range(6))
        for step in range(samples)
    )


def _delayed_rows(
    command: tuple[tuple[float, ...], ...], *, lag: int, offset: float = 0.0
) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(value + offset for value in command[max(0, index - lag)])
        for index in range(len(command))
    )


class DigitalTwinTraceContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.timestamps = tuple(index * 100_000_000 for index in range(12))
        self.command_rows = _command_rows(len(self.timestamps))
        self.command = JointTrace(
            source="command",
            contract_id=CONTRACT_ID,
            source_revision="command-plan:test-v1",
            joint_names=SO101_CHANNEL_NAMES,
            timestamps_ns=self.timestamps,
            positions_rad=self.command_rows,
        )
        self.simulation = JointTrace(
            source="simulation",
            contract_id=CONTRACT_ID,
            source_revision="mujoco-model:test-v1",
            joint_names=SO101_CHANNEL_NAMES,
            timestamps_ns=self.timestamps,
            positions_rad=_delayed_rows(self.command_rows, lag=1),
        )
        self.physical = JointTrace(
            source="physical",
            contract_id=CONTRACT_ID,
            source_revision="calibration:test-v1",
            joint_names=SO101_CHANNEL_NAMES,
            timestamps_ns=self.timestamps,
            positions_rad=_delayed_rows(self.command_rows, lag=2, offset=0.002),
        )

    def test_measurement_reports_known_delay_gap_without_claiming_pass(self) -> None:
        report = evaluate_digital_twin(
            command=self.command,
            simulation=self.simulation,
            physical=self.physical,
            max_lag_steps=3,
        )

        self.assertEqual(report["status"], "MEASURED")
        self.assertIsNone(report["passed"])
        self.assertEqual(report["sample_count"], 12)
        self.assertEqual(report["period_ns"], 100_000_000)
        self.assertEqual(report["aggregate"]["max_abs_delay_gap_steps"], 1)
        self.assertTrue(
            all(metrics["delay_gap_steps"] == 1 for metrics in report["per_joint"].values())
        )
        json.dumps(report)

    def test_explicit_thresholds_can_pass_or_fail(self) -> None:
        measurement_only = evaluate_digital_twin(
            command=self.command,
            simulation=self.simulation,
            physical=self.physical,
            max_lag_steps=3,
            thresholds=TwinThresholds(),
        )
        self.assertEqual(measurement_only["status"], "MEASURED")
        self.assertIsNone(measurement_only["passed"])

        passing = evaluate_digital_twin(
            command=self.command,
            simulation=self.simulation,
            physical=self.physical,
            max_lag_steps=3,
            thresholds=TwinThresholds(
                max_rmse_rad=0.2,
                max_endpoint_error_rad=0.2,
                max_timestamp_skew_ns=0,
                max_delay_gap_steps=1,
            ),
        )
        self.assertEqual(passing["status"], "PASS")
        self.assertTrue(passing["passed"])

        failing = evaluate_digital_twin(
            command=self.command,
            simulation=self.simulation,
            physical=self.physical,
            max_lag_steps=3,
            thresholds=TwinThresholds(max_rmse_rad=0.001),
        )
        self.assertEqual(failing["status"], "FAIL")
        self.assertEqual(failing["violations"], ["max_joint_rmse_rad"])

    def test_joint_order_mismatch_is_rejected(self) -> None:
        mismatched = replace(
            self.physical,
            joint_names=tuple(reversed(self.physical.joint_names)),
        )
        with self.assertRaisesRegex(DigitalTwinContractError, "joint order"):
            evaluate_digital_twin(
                command=self.command,
                simulation=self.simulation,
                physical=mismatched,
            )

    def test_contract_identity_mismatch_is_rejected(self) -> None:
        mismatched = replace(self.physical, contract_id="sha256:" + "2" * 64)
        with self.assertRaisesRegex(DigitalTwinContractError, "contract_id"):
            evaluate_digital_twin(
                command=self.command,
                simulation=self.simulation,
                physical=mismatched,
            )

    def test_timestamp_skew_is_measured_and_gated(self) -> None:
        skewed = replace(
            self.physical,
            timestamps_ns=tuple(value + 1_000_000 for value in self.timestamps),
        )
        report = evaluate_digital_twin(
            command=self.command,
            simulation=self.simulation,
            physical=skewed,
            max_lag_steps=3,
            thresholds=TwinThresholds(max_timestamp_skew_ns=999_999),
        )
        self.assertEqual(report["alignment"]["max_timestamp_skew_ns"], 1_000_000)
        self.assertIn("max_timestamp_skew_ns", report["violations"])

    def test_nonfinite_or_nonmonotonic_trace_is_rejected(self) -> None:
        bad_rows = list(self.command_rows)
        bad_rows[3] = tuple([math.nan, *bad_rows[3][1:]])
        with self.assertRaisesRegex(DigitalTwinContractError, "finite"):
            JointTrace(
                source="command",
                contract_id=CONTRACT_ID,
                source_revision="command-plan:test-v1",
                joint_names=SO101_CHANNEL_NAMES,
                timestamps_ns=self.timestamps,
                positions_rad=tuple(bad_rows),
            )
        with self.assertRaisesRegex(DigitalTwinContractError, "strictly increasing"):
            replace(
                self.command,
                timestamps_ns=tuple([0, 0, *self.timestamps[2:]]),
            )


if __name__ == "__main__":
    unittest.main()
