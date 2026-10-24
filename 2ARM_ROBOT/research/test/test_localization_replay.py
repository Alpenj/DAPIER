from __future__ import annotations

from pathlib import Path
import sys
import unittest


RESEARCH_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = RESEARCH_ROOT.parents[1]
sys.path.insert(0, str(RESEARCH_ROOT / "src"))

from dapier_research.localization_replay import (  # noqa: E402
    LocalizationRecoveryMonitor,
    ReplayFault,
    build_nominal_replay,
    inject_fault,
    run_failure_matrix,
    run_replay,
)
from dapier_research.localization_runtime import (  # noqa: E402
    validate_localization_estimate,
    verify_python_contract_matches_json,
)


class LocalizationReplayTest(unittest.TestCase):
    def test_python_constants_match_language_neutral_contract(self) -> None:
        verify_python_contract_matches_json(
            REPO_ROOT / "contracts" / "localization_runtime_v1.json"
        )

    def test_nominal_replay_requires_initial_plan_then_proceeds(self) -> None:
        report = run_replay(build_nominal_replay())
        self.assertEqual(report.decisions[0].decision, "hold")
        self.assertTrue(report.decisions[0].replan_required)
        self.assertGreaterEqual(report.proceed_count, 1)
        self.assertEqual(report.reject_count, 0)
        self.assertFalse(report.simulator_truth_used)
        self.assertFalse(report.hardware_execution)

    def test_low_texture_blur_occlusion_and_lost_all_hold(self) -> None:
        base = build_nominal_replay()
        for fault in (
            ReplayFault.LOW_TEXTURE,
            ReplayFault.MOTION_BLUR,
            ReplayFault.OCCLUSION,
            ReplayFault.RECENTLY_LOST,
            ReplayFault.LOST,
        ):
            with self.subTest(fault=fault.value):
                report = run_replay(inject_fault(base, fault), fault=fault)
                self.assertGreaterEqual(report.hold_count, 2)
                self.assertGreaterEqual(report.discarded_action_count, 1)
                self.assertEqual(report.reject_count, 0)

    def test_transport_faults_fail_closed(self) -> None:
        base = build_nominal_replay()
        for fault in (
            ReplayFault.DUPLICATE,
            ReplayFault.OUT_OF_ORDER,
            ReplayFault.TIMESTAMP_SKEW,
            ReplayFault.WRONG_TF,
        ):
            with self.subTest(fault=fault.value):
                report = run_replay(inject_fault(base, fault), fault=fault)
                self.assertGreaterEqual(report.reject_count, 1)
                self.assertGreaterEqual(report.discarded_action_count, 1)

    def test_frame_drop_triggers_receiver_local_watchdog(self) -> None:
        report = run_replay(
            inject_fault(build_nominal_replay(), ReplayFault.FRAME_DROP),
            fault=ReplayFault.FRAME_DROP,
        )
        self.assertTrue(
            any("interarrival gap" in item.reason for item in report.decisions)
        )
        self.assertGreaterEqual(report.replan_count, 2)

    def test_relocalization_and_map_restart_invalidate_stale_plan(self) -> None:
        base = build_nominal_replay()
        relocalized = run_replay(
            inject_fault(base, ReplayFault.RELOCALIZED),
            fault=ReplayFault.RELOCALIZED,
        )
        self.assertTrue(
            any("relocalized" in item.reason for item in relocalized.decisions)
        )
        restarted = run_replay(
            inject_fault(base, ReplayFault.MAP_RESTART),
            fault=ReplayFault.MAP_RESTART,
        )
        self.assertTrue(any(item.identity_changed for item in restarted.decisions))
        self.assertGreaterEqual(restarted.replan_count, 2)

    def test_replan_acknowledgement_must_match_identity_and_sequence(self) -> None:
        monitor = LocalizationRecoveryMonitor()
        sample = build_nominal_replay()[0]
        decision = monitor.observe(sample)
        self.assertEqual(decision.decision, "hold")
        with self.assertRaisesRegex(ValueError, "identity"):
            monitor.acknowledge_replan(
                sequence=decision.sequence or 0,
                parent_frame="map",
                child_frame="wrong",
                map_id="dapier-lab-map-v1",
                session_id="replay-session-001",
                reset_generation=0,
            )
        with self.assertRaisesRegex(ValueError, "sequence"):
            monitor.acknowledge_replan(
                sequence=999,
                parent_frame="map",
                child_frame="base_link",
                map_id="dapier-lab-map-v1",
                session_id="replay-session-001",
                reset_generation=0,
            )

    def test_privileged_or_authorizing_estimate_is_rejected(self) -> None:
        sample = build_nominal_replay()[0]
        for key in ("simulator_truth_used", "control_authorized"):
            value = dict(sample.estimate)
            value[key] = True
            validation = validate_localization_estimate(
                value,
                receiver_monotonic_ns=sample.receiver_monotonic_ns,
            )
            self.assertFalse(validation.accepted)

    def test_failure_matrix_covers_every_declared_fault(self) -> None:
        matrix = run_failure_matrix()
        self.assertEqual(set(matrix["reports"]), {fault.value for fault in ReplayFault})
        self.assertFalse(matrix["hardware_execution"])


if __name__ == "__main__":
    unittest.main()
