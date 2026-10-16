"""C-2 P0-2 / P0-5: result classification and operator-recovery contract.

Mock cases follow the Console `status()` payload shape (`console/control.py`). Real cases read
saved `RUN_RECORD.json` files from `~/sweepick_console_runs` read-only and are skipped when absent.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
SWEEPICK_ROOT = HERE.parent.parent
if str(SWEEPICK_ROOT) not in sys.path:
    sys.path.insert(0, str(SWEEPICK_ROOT))

from nav2_vslam_integration import (  # noqa: E402
    BLOCK_ORIGIN_INTEGRATION_GATE,
    BLOCK_ORIGIN_SWEEPICK_CONSOLE,
    ApprovedSweepickScope,
    ManipulationResultStatus,
    SweepickExecutionAdapter,
    VslamSweepickCoordinator,
    active_from_console_run_record,
    classify_terminal_evidence,
)
from nav2_vslam_integration.tests.test_mock_e2e_integration import (  # noqa: E402
    MockSweepickConsoleController,
    _make_standard_fixtures,
)

CONSOLE_RUNS = Path.home() / "sweepick_console_runs"


def _status(active: dict, state: str) -> dict:
    return {"state": state, "active": dict(active, state=state), "can_prepare": False, "owner_alive": False}


class TestC2FaultClassification(unittest.TestCase):
    def setUp(self) -> None:
        self.adapter = SweepickExecutionAdapter(MockSweepickConsoleController())

    def _eval(self, active: dict, state: str = "FAULT"):
        return self.adapter.evaluate_session_outcome(status_payload=_status(active, state))

    def test_bus_lost_with_owner_stop_is_fault_not_stop(self) -> None:
        active = {
            "id": "r1", "execution_scope": "FULL_CHAIN_RECEIVE_CONTACT",
            "first_error": {"gate": "BUS_LOST", "stop_event": "owner_stop", "detail": "the bus could not be opened: ConnectionError",
                            "source": "episode/events.jsonl"},
            "session_result": "STOPPED_IN_DESCEND_HOLDING_NEEDS_A_PERSON", "normal_return_confirmed": False,
        }
        res = self._eval(active)
        self.assertEqual(res.status, ManipulationResultStatus.FAULT)
        self.assertEqual(res.native_gate, "BUS_LOST")
        self.assertIsNotNone(res.hardware_fault)
        self.assertIsNone(res.operator_stop)
        self.assertTrue(res.subsequent_commands_blocked and res.requires_operator_recovery)

    def test_owner_stop_gate_with_bus_lost_text_is_fault(self) -> None:
        # Real shape from run 20261015_151937_81d00c: gate OWNER_STOP, native gate inside the detail text.
        active = {
            "id": "r2", "first_error": {"gate": "OWNER_STOP", "stop_event": "owner_stop",
                                        "detail": "RECEIVE_CONTACT: RuntimeError: contact_right_return_initial (right): the executor did not plan it: {'gate': 'bus_lost', 'level': 'HARD', 'detail': 'the bus could not be opened: ConnectionError: '}"},
            "normal_return_confirmed": False,
        }
        res = self._eval(active)
        self.assertEqual(res.status, ManipulationResultStatus.FAULT)
        self.assertEqual(res.error_code, "SWEEPICK_FAULT")

    def test_real_operator_stop_is_stop(self) -> None:
        active = {
            "id": "r3", "stop_sent": True,
            "first_error": {"gate": "OWNER_STOP", "stop_event": "owner_stop", "detail": "operator STOP or an earlier executor stop is active"},
            "session_result": "STOPPED_IN_DESCEND_HOLDING_NEEDS_A_PERSON", "normal_return_confirmed": False,
        }
        res = self._eval(active)
        self.assertEqual(res.status, ManipulationResultStatus.STOP)
        self.assertEqual(res.error_code, "SWEEPICK_STOP")
        self.assertIsNone(res.hardware_fault)
        self.assertEqual(res.operator_stop["source"], "active.stop_sent")

    def test_owner_stop_of_unknown_origin_is_fault_not_stop(self) -> None:
        active = {"id": "r4", "first_error": {"gate": "OWNER_STOP", "stop_event": "owner_stop", "detail": "existing owner stopped"},
                  "normal_return_confirmed": False}
        res = self._eval(active)
        self.assertEqual(res.status, ManipulationResultStatus.FAULT)
        self.assertIsNone(res.operator_stop)

    def test_observation_failure_with_normal_return_is_observation_failed(self) -> None:
        active = {"id": "r5", "session_result": "PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME", "normal_return_confirmed": True,
                  "failure_point": {"stage": "look", "gate": "ready_to_close=false"},
                  "stage_results": [{"side": "left", "stage": "home_arm", "result": "RETURNED_RELEASED"}]}
        res = self._eval(active, state="ENDED")
        self.assertEqual(res.status, ManipulationResultStatus.OBSERVATION_FAILED)
        self.assertEqual(res.observation_failure, "PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME")
        self.assertTrue(res.normal_return_off_confirmed)
        self.assertFalse(res.subsequent_commands_blocked)

    def test_observation_failure_plus_console_fault_state_is_fault_with_observation_preserved(self) -> None:
        active = {"id": "r6", "session_result": "PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME", "normal_return_confirmed": False,
                  "first_error": {"gate": "EXECUTOR_ERROR", "detail": "exit 3", "source": "review/lifetime.jsonl"},
                  "failure_point": {"stage": "look", "gate": "ready_to_close=false"}}
        res = self._eval(active, state="FAULT")
        self.assertEqual(res.status, ManipulationResultStatus.FAULT)
        self.assertEqual(res.observation_failure, "PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME")
        self.assertIn("관측 실패 별도 보존", res.reason)
        self.assertTrue(res.subsequent_commands_blocked)

    def test_observation_failure_plus_hardware_fault_is_fault_with_observation_preserved(self) -> None:
        active = {"id": "r7", "session_result": "NOT_ALIGNED_RETURNED_HOME", "normal_return_confirmed": True,
                  "first_error": {"gate": "SERVO_OVERLOAD", "detail": "ID=6 error=0x20 [RxPacketError] Overload"}}
        res = self._eval(active, state="FAULT")
        self.assertEqual(res.status, ManipulationResultStatus.FAULT)
        self.assertEqual(res.native_gate, "SERVO_OVERLOAD")
        self.assertEqual(res.observation_failure, "NOT_ALIGNED_RETURNED_HOME")
        self.assertIsNotNone(res.hardware_fault)

    def test_classification_object_keeps_all_three_classes(self) -> None:
        payload = _status({"id": "r8", "stop_sent": True, "session_result": "NO_CONTACT_RETURNED_HOME",
                           "first_error": {"gate": "BUS_LOST", "stop_event": "owner_stop", "detail": "bus_lost"}}, "FAULT")
        ev = classify_terminal_evidence(payload)
        self.assertIsNotNone(ev.hardware_fault)
        self.assertIsNotNone(ev.operator_stop)
        self.assertEqual(ev.observation_failure, "NO_CONTACT_RETURNED_HOME")
        self.assertTrue(ev.console_fault_state)
        self.assertIn("owner_stop_after_hardware_fault->FAULT_wins", ev.basis)

    # ---------------- real stored runs (read-only) ----------------
    def _real(self, run_id: str) -> dict:
        path = CONSOLE_RUNS / run_id / "RUN_RECORD.json"
        if not path.is_file():
            self.skipTest(f"real run record not on this machine: {path}")
        return json.loads(path.read_text(encoding="utf-8"))

    def test_real_20261016_112020_exit3_pad_placement_is_fault_with_observation_preserved(self) -> None:
        rec = self._real("20261016_112020_4d7c2b")
        active = active_from_console_run_record(rec)
        res = self._eval(active, state=rec["state"])
        self.assertEqual(res.status, ManipulationResultStatus.FAULT)
        self.assertEqual(res.observation_failure, "PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME")
        self.assertEqual(res.native_gate, "EXECUTOR_ERROR")
        self.assertIsNone(res.hardware_fault)  # exit 3 is a native non-success class, not a bus fault

    def test_real_20261015_175106_motor_check_failed_is_hardware_fault(self) -> None:
        rec = self._real("20261015_175106_f5aabc")
        res = self._eval(active_from_console_run_record(rec), state=rec["state"])
        self.assertEqual(res.status, ManipulationResultStatus.FAULT)
        self.assertIsNotNone(res.hardware_fault)
        self.assertEqual(res.hardware_fault["source"], "active.failure_point")
        self.assertIsNone(res.observation_failure)

    def test_real_20261015_151937_owner_stop_after_bus_lost_is_fault(self) -> None:
        rec = self._real("20261015_151937_81d00c")
        res = self._eval(active_from_console_run_record(rec), state=rec["state"])
        self.assertEqual(res.status, ManipulationResultStatus.FAULT)
        self.assertIsNotNone(res.hardware_fault)
        self.assertIsNone(res.operator_stop)
        self.assertEqual(res.native_gate, "OWNER_STOP")  # the Console's gate name is kept as recorded


class TestC2OperatorRecoveryContract(unittest.TestCase):
    def setUp(self) -> None:
        self.console = MockSweepickConsoleController()
        self.adapter = SweepickExecutionAdapter(self.console)
        self.coord = VslamSweepickCoordinator(sweepick_adapter=self.adapter)
        pre, arrival, base_stop, post = _make_standard_fixtures()
        self.fixtures = (pre, arrival, base_stop, post)
        self.coord.execute_pipeline(initial_detection=pre, arrival=arrival, base_stop=base_stop,
                                    post_stop_observation=post, reobserve_checked_at_s=112.90)
        self.console.simulate_operator_start_and_finish(
            final_state="FAULT", session_result="STOPPED_IN_CLOSE_HOLDING_NEEDS_A_PERSON", held_after_lift=False,
            normal_return_confirmed=False, first_error={"gate": "BUS_LOST", "stop_event": "owner_stop", "detail": "bus_lost"},
        )
        self.outcome = self.coord.finalize_after_operator_execution()
        self.run_id = self.outcome.sweepick_run_id

    def _console_status_with_recovery(self, *, job_id=None, power=True, reset=True, source="local_operator_manual_confirmation", can_prepare=True):
        status = self.console.status()
        status["can_prepare"] = can_prepare
        status["active"] = dict(status["active"], operator_recovery={
            "job_id": job_id or self.run_id, "session_out": status["active"]["session_out"], "source": source,
            "power_isolated": power, "workspace_reset": reset, "torque_readback": "UNKNOWN",
            "normal_return_confirmed": False, "motor_writes": 0,
        })
        return status

    def test_block_origin_is_console_and_boolean_alone_is_refused(self) -> None:
        self.assertEqual(self.outcome.result_status, ManipulationResultStatus.FAULT)
        self.assertEqual(self.coord.block_origin, BLOCK_ORIGIN_SWEEPICK_CONSOLE)
        self.assertEqual(self.coord.blocked_run_id, self.run_id)
        self.assertFalse(self.coord.confirm_operator_recovery(operator_verified=True, note="power off, block replaced"))
        self.assertTrue(self.coord.command_latch_blocked)
        self.assertIn("CONSOLE_STATUS_REQUIRED_BOOLEAN_NOT_ACCEPTED", self.coord.last_recovery_decision.checks)

    def test_console_status_without_recovery_record_is_refused(self) -> None:
        status = self.console.status()
        self.assertFalse(self.coord.confirm_operator_recovery(operator_verified=True, note="n", console_status=status))
        self.assertIn("CONSOLE_MANUAL_RECOVERY_RECORD_ABSENT", self.coord.last_recovery_decision.checks)

    def test_console_recovery_record_for_other_run_is_refused(self) -> None:
        status = self._console_status_with_recovery(job_id="other_run")
        self.assertFalse(self.coord.confirm_operator_recovery(operator_verified=True, note="n", console_status=status))
        self.assertTrue(any(c.startswith("CONSOLE_RECOVERY_JOB_MISMATCH") for c in self.coord.last_recovery_decision.checks))

    def test_console_recovery_record_incomplete_is_refused(self) -> None:
        status = self._console_status_with_recovery(power=False)
        self.assertFalse(self.coord.confirm_operator_recovery(operator_verified=True, note="n", console_status=status))
        self.assertIn("CONSOLE_RECOVERY_POWER_NOT_ISOLATED", self.coord.last_recovery_decision.checks)
        status = self._console_status_with_recovery(can_prepare=False)
        self.assertFalse(self.coord.confirm_operator_recovery(operator_verified=True, note="n", console_status=status))
        self.assertIn("CONSOLE_CAN_PREPARE_FALSE", self.coord.last_recovery_decision.checks)

    def test_console_recovery_record_matching_run_clears_latch(self) -> None:
        status = self._console_status_with_recovery()
        self.assertTrue(self.coord.confirm_operator_recovery(operator_verified=True, note="field recovery done", console_status=status))
        self.assertFalse(self.coord.command_latch_blocked)
        self.assertEqual(self.coord.last_recovery_decision.console_recovery_record["job_id"], self.run_id)
        # a new cycle is now accepted (mock console has to be prepared again by the pipeline);
        # it needs a NEW post-stop observation: the one already used is consumed (C-2 re-publish rule).
        from dataclasses import replace
        self.console.state, self.console.can_prepare, self.console.active = "IDLE", True, None
        pre, arrival, base_stop, post = self.fixtures
        reused = self.coord.execute_pipeline(initial_detection=pre, arrival=arrival, base_stop=base_stop,
                                             post_stop_observation=post, reobserve_checked_at_s=130.0)
        self.assertEqual(reused.result_status, ManipulationResultStatus.OBSERVATION_FAILED)
        self.assertIn("REPUBLISHED_OBSERVATION_SAME_SENSOR_FRAME", reused.reason)
        fresh = replace(post, observation_id="obs_post_stop_140", frame_seq=140, captured_at_s=129.9)
        again = self.coord.execute_pipeline(initial_detection=pre, arrival=arrival, base_stop=base_stop,
                                            post_stop_observation=fresh, reobserve_checked_at_s=130.0)
        self.assertEqual(again.result_status, ManipulationResultStatus.WAITING_FOR_OPERATOR_APPROVAL)

    def test_prior_unrecovered_fault_blocks_new_task(self) -> None:
        pre, arrival, base_stop, post = self.fixtures
        blocked = self.coord.execute_pipeline(initial_detection=pre, arrival=arrival, base_stop=base_stop,
                                              post_stop_observation=post, reobserve_checked_at_s=130.0)
        self.assertEqual(blocked.result_status, ManipulationResultStatus.EXECUTION_REJECTED)
        self.assertEqual(blocked.error_report["error_code"], "COMMAND_BLOCKED_BY_PRIOR_FAILURE_OR_HOLD")
        self.assertEqual(self.console.prepare_calls, 1)

    def test_integration_gate_block_accepts_operator_note(self) -> None:
        coord = VslamSweepickCoordinator(sweepick_adapter=SweepickExecutionAdapter(MockSweepickConsoleController()))
        pre, arrival, base_stop, post = _make_standard_fixtures()
        out = coord.execute_pipeline(initial_detection=pre, arrival=arrival, base_stop=base_stop,
                                     post_stop_observation=post, reobserve_checked_at_s=112.9,
                                     handoff_mode="DIRECT_BASE_TO_ARM_METRIC_TF")
        self.assertEqual(out.result_status, ManipulationResultStatus.EXECUTION_REJECTED)
        self.assertEqual(coord.block_origin, BLOCK_ORIGIN_INTEGRATION_GATE)
        self.assertFalse(coord.confirm_operator_recovery(operator_verified=True, note=""))
        self.assertTrue(coord.confirm_operator_recovery(operator_verified=True, note="handoff mode corrected"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
