from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from local_llm_supervisor import (
    SCHEMA_VERSION,
    LocalSupervisorInput,
    SupervisorDirective,
    evaluate_directive,
)
from mission_core import MissionPhase


def proposal(
    directive: SupervisorDirective,
    *,
    sequence: int = 7,
    failure_code: str = "",
) -> str:
    return json.dumps(
        {
            "schema_version": SCHEMA_VERSION,
            "directive": directive.value,
            "expected_world_sequence": sequence,
            "reason": "bounded local decision",
            "failure_code": failure_code,
        }
    )


def world(**overrides) -> LocalSupervisorInput:
    values = {
        "world_sequence": 7,
        "mission_phase": MissionPhase.VERIFYING_GRASP,
        "snapshot_fresh": True,
        "base_stationary": True,
        "safety_motion_allowed": True,
        "alert_codes": (),
        "fault_codes": (),
        "recovery_attempts": 0,
    }
    values.update(overrides)
    return LocalSupervisorInput(**values)


class LocalLlmSupervisorTest(unittest.TestCase):
    def test_only_bounded_directives_are_accepted_without_dispatch(self) -> None:
        decision = evaluate_directive(
            proposal(SupervisorDirective.REGRASP, failure_code="grasp_missed"),
            world(),
        )
        self.assertTrue(decision.accepted)
        self.assertFalse(decision.hardware_dispatch_authorized)
        self.assertFalse(decision.hardware_execution)

    def test_raw_motion_or_unknown_directive_is_rejected(self) -> None:
        payload = json.loads(proposal(SupervisorDirective.CONTINUE))
        payload["directive"] = "set_joint_target"
        self.assertFalse(evaluate_directive(json.dumps(payload), world()).accepted)
        payload = json.loads(proposal(SupervisorDirective.CONTINUE))
        payload["joint_target"] = [0.1]
        self.assertFalse(evaluate_directive(json.dumps(payload), world()).accepted)

    def test_pi_load_alert_does_not_reject_continue(self) -> None:
        loaded = world(alert_codes=("edge_cpu_pressure",))
        decision = evaluate_directive(
            proposal(SupervisorDirective.CONTINUE),
            loaded,
        )
        self.assertTrue(decision.accepted)

    def test_stale_sequence_or_snapshot_is_rejected(self) -> None:
        self.assertFalse(
            evaluate_directive(
                proposal(SupervisorDirective.CONTINUE, sequence=6),
                world(),
            ).accepted
        )
        self.assertFalse(
            evaluate_directive(
                proposal(SupervisorDirective.CONTINUE),
                world(snapshot_fresh=False),
            ).accepted
        )

    def test_safe_stop_is_accepted_even_when_state_is_stale(self) -> None:
        stale = world(snapshot_fresh=False, safety_motion_allowed=False)
        decision = evaluate_directive(
            proposal(SupervisorDirective.SAFE_STOP),
            stale,
        )
        self.assertTrue(decision.accepted)

    def test_regrasp_requires_correct_phase_stationary_base_and_budget(self) -> None:
        raw = proposal(SupervisorDirective.REGRASP, failure_code="grasp_missed")
        invalid_worlds = (
            world(mission_phase=MissionPhase.NAVIGATING_TO_A),
            world(base_stationary=False),
            world(recovery_attempts=2),
        )
        for state in invalid_worlds:
            with self.subTest(state=state):
                self.assertFalse(evaluate_directive(raw, state).accepted)


if __name__ == "__main__":
    unittest.main()
