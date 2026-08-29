from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from llm_task_supervisor import (
    SCHEMA_VERSION,
    SupervisorWorldState,
    evaluate_skill_proposal,
)


def proposal(skill: str, arguments: dict[str, object] | None = None) -> str:
    return json.dumps(
        {
            "schema_version": SCHEMA_VERSION,
            "skill": skill,
            "expected_observation_seq": 7,
            "reason": "bounded test proposal",
            "arguments": arguments or {},
        }
    )


class LlmTaskSupervisorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.world = SupervisorWorldState(
            observation_seq=7,
            observation_age_ms=40.0,
            base_stationary=True,
            safety_gate_ready=True,
        )

    def test_valid_skill_is_only_proposed_never_executed(self) -> None:
        result = evaluate_skill_proposal(
            proposal("pick", {"target_id": "shoe-01", "side": "left"}),
            self.world,
        )
        self.assertTrue(result.accepted)
        self.assertEqual(result.skill, "pick")
        self.assertFalse(result.hardware_dispatch_authorized)
        self.assertFalse(result.executed_action)
        self.assertFalse(result.hardware_execution)

    def test_raw_joint_or_hardware_arguments_are_rejected(self) -> None:
        result = evaluate_skill_proposal(
            proposal("pick", {"target_id": "shoe-01", "joint_target": [0.1]}),
            self.world,
        )
        self.assertFalse(result.accepted)

    def test_stale_observation_and_sequence_are_rejected(self) -> None:
        stale = SupervisorWorldState(7, 501.0, True, True)
        self.assertFalse(evaluate_skill_proposal(proposal("observe"), stale).accepted)
        payload = json.loads(proposal("observe"))
        payload["expected_observation_seq"] = 6
        self.assertFalse(
            evaluate_skill_proposal(json.dumps(payload), self.world).accepted
        )

    def test_motion_is_rejected_while_base_moves(self) -> None:
        moving = SupervisorWorldState(7, 40.0, False, True)
        result = evaluate_skill_proposal(
            proposal("approach", {"target_id": "shoe-01", "standoff_m": 0.4}),
            moving,
        )
        self.assertFalse(result.accepted)
        self.assertIn("stationary", result.reason)

    def test_hardware_mode_and_exhausted_recovery_fail_closed(self) -> None:
        hardware = SupervisorWorldState(7, 40.0, True, True, simulation_only=False)
        self.assertFalse(
            evaluate_skill_proposal(proposal("observe"), hardware).accepted
        )
        exhausted = SupervisorWorldState(7, 40.0, True, True, recovery_attempts=2)
        self.assertFalse(
            evaluate_skill_proposal(
                proposal("recover", {"failure_code": "grasp_missed"}),
                exhausted,
            ).accepted
        )

    def test_stop_is_accepted_without_motion_even_if_state_is_stale(self) -> None:
        stale_moving = SupervisorWorldState(7, 999.0, False, False)
        result = evaluate_skill_proposal(proposal("stop"), stale_moving)
        self.assertTrue(result.accepted)
        self.assertFalse(result.executed_action)

    def test_invalid_json_unknown_skill_and_extra_keys_are_rejected(self) -> None:
        self.assertFalse(evaluate_skill_proposal("{", self.world).accepted)
        unknown = json.loads(proposal("observe"))
        unknown["skill"] = "send_torque"
        self.assertFalse(
            evaluate_skill_proposal(json.dumps(unknown), self.world).accepted
        )
        extra = json.loads(proposal("observe"))
        extra["free_form_command"] = "anything"
        self.assertFalse(
            evaluate_skill_proposal(json.dumps(extra), self.world).accepted
        )


if __name__ == "__main__":
    unittest.main()
