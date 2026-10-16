"""P0/P1/P2 Pipeline Coordinator: VslamSweepickCoordinator.

Connects Navigation (`DETECT -> NAVIGATE -> ARRIVED -> BASE_STOPPED -> REOBSERVE`)
with Manipulation (`MANIPULATION_READY -> ARM_TASK -> RESULT`) under deterministic
contracts:
- Nav2 arrival (`STATUS_SUCCEEDED` / `DOCKED`) is NEVER treated as manipulation success.
- Pre-navigation object coordinates are invalidated as soon as motion begins and
  can NEVER be reused after arrival.
- A fresh post-stop camera observation (`REOBSERVE` after `BASE_STOPPED`) is mandatory
  before entering `MANIPULATION_READY`.
- Existing TF2 frames are reused; unverified `base_footprint <-> arm_base` transforms
  are strictly rejected.
- Upon `STOP`, `FAULT`, or `EXECUTION_REJECTED`, subsequent commands are blocked and
  a structured `TaskManagerErrorReport` is propagated to the Task Manager.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional, Tuple

from .contracts import (
    ApprovedSweepickScope,
    BaseStopEvidence,
    CameraObservation3D,
    CoordinateFrameContract,
    HardwareReadinessState,
    IntegrationOutcomeRecord,
    IntegrationStage,
    ManipulationResultStatus,
    NavigationArrivalEvidence,
    ReobserveVerdict,
    TaskManagerErrorReport,
    validate_stage_transition,
)
from .reobserve_gate import (
    BaseStopVerifier,
    PostStopReobserveGate,
    ReobserveGateConfig,
)
from .sweepick_adapter import (
    SweepickAdapterResult,
    SweepickExecutionAdapter,
)


BLOCK_ORIGIN_NONE = "NONE"
BLOCK_ORIGIN_SWEEPICK_CONSOLE = "SWEEPICK_CONSOLE"      # STOP / FAULT / LIMITED_HOLDING / rejection with a Console run
BLOCK_ORIGIN_SWEEPICK_NO_RUN = "SWEEPICK_CONSOLE_NO_RUN"  # Console rejected before a run existed (foreign owner, device, scope)
BLOCK_ORIGIN_INTEGRATION_GATE = "INTEGRATION_GATE"      # Nav2 / base-stop / TF gate of this layer, no Console run involved

CONSOLE_RECOVERY_SOURCE = "local_operator_manual_confirmation"


@dataclass(frozen=True)
class OperatorRecoveryDecision:
    """Result of `confirm_operator_recovery` (C-2 P0-2).

    A Console-origin block is cleared only by the Console's own manual-recovery record
    (`review/MANUAL_RECOVERY_CONFIRMATION.json`, exposed as `status()['active']['operator_recovery']`
    by `console/control.py:Controller.recovery_confirmation`). A boolean from the caller is never
    enough for that origin.
    """

    accepted: bool
    block_origin: str
    reason: str
    console_run_id: Optional[str] = None
    console_recovery_record: Optional[Dict[str, Any]] = None
    checks: Tuple[str, ...] = ()

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def console_recovery_evidence(
    console_status: Optional[Dict[str, Any]],
    *,
    run_id: Optional[str],
) -> Tuple[Optional[Dict[str, Any]], Tuple[str, ...]]:
    """Validate the Console's recovery record for `run_id`. Returns `(record, failed_checks)`."""
    failed: List[str] = []
    if not isinstance(console_status, dict):
        return None, ("CONSOLE_STATUS_MISSING",)
    active = console_status.get("active") if isinstance(console_status.get("active"), dict) else {}
    record = active.get("operator_recovery") if isinstance(active.get("operator_recovery"), dict) else None
    if run_id is not None and active.get("id") != run_id:
        failed.append(f"CONSOLE_ACTIVE_RUN_MISMATCH:{active.get('id')}!={run_id}")
    if record is None:
        failed.append("CONSOLE_MANUAL_RECOVERY_RECORD_ABSENT")
    else:
        if record.get("source") != CONSOLE_RECOVERY_SOURCE:
            failed.append(f"CONSOLE_RECOVERY_SOURCE_INVALID:{record.get('source')}")
        if record.get("power_isolated") is not True:
            failed.append("CONSOLE_RECOVERY_POWER_NOT_ISOLATED")
        if record.get("workspace_reset") is not True:
            failed.append("CONSOLE_RECOVERY_WORKSPACE_NOT_RESET")
        if run_id is not None and record.get("job_id") != run_id:
            failed.append(f"CONSOLE_RECOVERY_JOB_MISMATCH:{record.get('job_id')}!={run_id}")
        if active.get("session_out") and record.get("session_out") not in (None, active.get("session_out")):
            failed.append("CONSOLE_RECOVERY_SESSION_MISMATCH")
    if console_status.get("owner_alive") is True:
        failed.append("CONSOLE_OWNER_STILL_ALIVE")
    if console_status.get("can_prepare") is not True:
        failed.append("CONSOLE_CAN_PREPARE_FALSE")
    if console_status.get("state") == "BLOCKED":
        failed.append("CONSOLE_FOREIGN_OWNER_BLOCKED")
    return record, tuple(failed)


class VslamSweepickCoordinator:
    """State-machine coordinator for VSLAM/Nav2 x Sweepick Visual Servo integration."""

    def __init__(
        self,
        *,
        sweepick_adapter: SweepickExecutionAdapter,
        gate_config: Optional[ReobserveGateConfig] = None,
        hardware_readiness: Optional[HardwareReadinessState] = None,
    ) -> None:
        self.sweepick_adapter = sweepick_adapter
        self.hardware_readiness = hardware_readiness or HardwareReadinessState()
        self.frame_contract = CoordinateFrameContract(
            hardware_readiness=self.hardware_readiness
        )
        self.base_stop_verifier = BaseStopVerifier(gate_config)
        self.reobserve_gate = PostStopReobserveGate(gate_config, self.frame_contract)

        self.current_stage: Optional[IntegrationStage] = None
        self.stage_history: List[IntegrationStage] = []
        self.pre_nav_detection: Optional[CameraObservation3D] = None
        self.arrival_evidence: Optional[NavigationArrivalEvidence] = None
        self.base_stop_evidence: Optional[BaseStopEvidence] = None
        self.reobserve_verdict: Optional[ReobserveVerdict] = None
        self.last_adapter_result: Optional[SweepickAdapterResult] = None
        self.task_manager_error_report: Optional[TaskManagerErrorReport] = None
        self.command_latch_blocked: bool = False
        self.command_block_reason: Optional[str] = None
        self.block_origin: str = BLOCK_ORIGIN_NONE
        self.blocked_run_id: Optional[str] = None
        self.last_recovery_decision: Optional[OperatorRecoveryDecision] = None
        self.arm_commands_invoked_count: int = 0

    def _transition_to(self, next_stage: IntegrationStage) -> None:
        if self.current_stage is None:
            if next_stage != IntegrationStage.DETECT:
                raise ValueError(f"Pipeline must start at DETECT, got {next_stage.value}")
        else:
            validate_stage_transition(self.current_stage, next_stage)
        self.current_stage = next_stage
        self.stage_history.append(next_stage)

    def _check_not_blocked(self, requested_action: str) -> Optional[IntegrationOutcomeRecord]:
        if not self.command_latch_blocked:
            return None
        report = TaskManagerErrorReport(
            failed_stage=self.current_stage or IntegrationStage.RESULT,
            result_status=ManipulationResultStatus.EXECUTION_REJECTED,
            error_code="COMMAND_BLOCKED_BY_PRIOR_FAILURE_OR_HOLD",
            detail=(
                f"Action '{requested_action}' rejected because prior terminal state "
                f"blocked subsequent commands: {self.command_block_reason}"
            ),
            subsequent_commands_blocked=True,
            requires_operator_recovery=True,
            run_id=self.last_adapter_result.run_id if self.last_adapter_result else None,
        )
        self.task_manager_error_report = report
        return IntegrationOutcomeRecord(
            current_stage=IntegrationStage.RESULT,
            stage_history=tuple(s.value for s in self.stage_history),
            result_status=ManipulationResultStatus.EXECUTION_REJECTED,
            manipulation_succeeded=False,
            left_pick_lift_confirmed=False,
            right_handover_confirmed=False,
            approved_scope=(
                self.last_adapter_result.approved_scope
                if self.last_adapter_result
                else ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT.value
            ),
            motor_authority_created_by_integration=False,
            pre_nav_coordinate_reused=False,
            post_stop_reobserved=bool(self.reobserve_verdict and self.reobserve_verdict.passed),
            subsequent_commands_blocked=True,
            hardware_readiness=self.hardware_readiness.to_dict(),
            reason=report.detail,
            error_report=report.to_dict(),
            sweepick_run_id=report.run_id,
        )

    def confirm_operator_recovery(
        self,
        *,
        operator_verified: bool,
        note: str,
        console_status: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Clear the command latch after STOP / FAULT / EXECUTION_REJECTED / LIMITED_HOLDING.

        C-2 contract:
        - `BLOCK_ORIGIN_SWEEPICK_CONSOLE`: requires `console_status` (a fresh `Controller.status()`)
          whose `active.operator_recovery` is the Console's own manual-recovery record for the
          blocked run and whose `can_prepare` is True. `operator_verified=True` alone is refused.
        - `BLOCK_ORIGIN_SWEEPICK_CONSOLE_NO_RUN`: requires `console_status` not BLOCKED, no owner alive,
          `can_prepare` True.
        - `BLOCK_ORIGIN_INTEGRATION_GATE`: no Console run was involved; an explicit operator note is
          accepted and recorded.
        The decision is stored in `last_recovery_decision`.
        """
        checks: List[str] = []
        if not operator_verified:
            checks.append("OPERATOR_NOT_VERIFIED")
        if not note or not note.strip():
            checks.append("OPERATOR_NOTE_EMPTY")
        if not self.command_latch_blocked:
            decision = OperatorRecoveryDecision(
                accepted=False, block_origin=self.block_origin, reason="NO_ACTIVE_BLOCK", checks=tuple(checks)
            )
            self.last_recovery_decision = decision
            return False

        origin = self.block_origin
        record: Optional[Dict[str, Any]] = None
        if origin in (BLOCK_ORIGIN_SWEEPICK_CONSOLE, BLOCK_ORIGIN_SWEEPICK_NO_RUN):
            if console_status is None:
                checks.append("CONSOLE_STATUS_REQUIRED_BOOLEAN_NOT_ACCEPTED")
            else:
                record, failed = console_recovery_evidence(
                    console_status,
                    run_id=self.blocked_run_id if origin == BLOCK_ORIGIN_SWEEPICK_CONSOLE else None,
                )
                if origin == BLOCK_ORIGIN_SWEEPICK_NO_RUN:
                    failed = tuple(f for f in failed if not f.startswith("CONSOLE_MANUAL_RECOVERY_RECORD")
                                   and not f.startswith("CONSOLE_RECOVERY_") and not f.startswith("CONSOLE_ACTIVE_RUN"))
                checks.extend(failed)
        elif origin == BLOCK_ORIGIN_INTEGRATION_GATE:
            pass
        else:
            checks.append(f"UNKNOWN_BLOCK_ORIGIN:{origin}")

        accepted = not checks
        decision = OperatorRecoveryDecision(
            accepted=accepted,
            block_origin=origin,
            reason="RECOVERY_ACCEPTED" if accepted else "; ".join(checks),
            console_run_id=self.blocked_run_id,
            console_recovery_record=record if accepted else None,
            checks=tuple(checks),
        )
        self.last_recovery_decision = decision
        if not accepted:
            return False
        self.command_latch_blocked = False
        self.command_block_reason = None
        self.task_manager_error_report = None
        self.block_origin = BLOCK_ORIGIN_NONE
        self.blocked_run_id = None
        return True

    def reset_for_next_cycle(self) -> None:
        """Reset pipeline stage pointers for a new cycle if and only if not blocked."""
        if self.command_latch_blocked:
            raise RuntimeError(
                f"Cannot reset pipeline while command latch is blocked: {self.command_block_reason}"
            )
        self.current_stage = None
        self.stage_history = []
        self.pre_nav_detection = None
        self.arrival_evidence = None
        self.base_stop_evidence = None
        self.reobserve_verdict = None
        self.last_adapter_result = None

    def reject_at_detect(
        self,
        *,
        error_code: str,
        detail: str,
        block_subsequent: bool = False,
    ) -> IntegrationOutcomeRecord:
        """Reject an invalid command at DETECT -> RESULT before any navigation or arm action."""
        blocked = self._check_not_blocked("reject_at_detect")
        if blocked is not None:
            return blocked
        self.reset_for_next_cycle()
        self._transition_to(IntegrationStage.DETECT)
        return self._abort_with_error(
            failed_stage=IntegrationStage.DETECT,
            status=ManipulationResultStatus.EXECUTION_REJECTED,
            error_code=error_code,
            detail=detail,
            block_subsequent=block_subsequent,
            requires_recovery=block_subsequent,
        )

    def step_detect(self, initial_detection: Optional[CameraObservation3D]) -> IntegrationStage:
        """Stage 1: DETECT (initial target or semantic waypoint cue before navigation)."""
        if self.command_latch_blocked:
            raise RuntimeError(f"Command blocked: {self.command_block_reason}")
        self._transition_to(IntegrationStage.DETECT)
        self.pre_nav_detection = initial_detection
        return self.current_stage

    def step_navigate(self) -> IntegrationStage:
        """Stage 2: NAVIGATE. Immediately invalidates pre-navigation object coordinates."""
        if self.command_latch_blocked:
            raise RuntimeError(f"Command blocked: {self.command_block_reason}")
        self._transition_to(IntegrationStage.NAVIGATE)
        if self.pre_nav_detection is not None:
            self.reobserve_gate.invalidate_observation(self.pre_nav_detection)
        return self.current_stage

    def step_arrived(self, arrival: NavigationArrivalEvidence) -> Optional[IntegrationOutcomeRecord]:
        """Stage 3: ARRIVED. Nav2 success only marks base arrival, NEVER manipulation completion."""
        self._transition_to(IntegrationStage.ARRIVED)
        self.arrival_evidence = arrival
        if not arrival.is_arrived():
            return self._abort_with_error(
                failed_stage=IntegrationStage.ARRIVED,
                status=ManipulationResultStatus.EXECUTION_REJECTED,
                error_code=f"NAV2_ARRIVAL_FAILED:{arrival.nav2_status}",
                detail=f"Nav2 failed to reach goal '{arrival.goal_label}' (status={arrival.nav2_status})",
                block_subsequent=True,
                requires_recovery=False,
            )
        return None

    def step_base_stopped(self, stop_evidence: BaseStopEvidence) -> Optional[IntegrationOutcomeRecord]:
        """Stage 4: BASE_STOPPED. Verifies zero velocity settle before camera re-observation."""
        self._transition_to(IntegrationStage.BASE_STOPPED)
        self.base_stop_evidence = stop_evidence
        ok, problems = self.base_stop_verifier.verify(stop_evidence)
        if not ok:
            return self._abort_with_error(
                failed_stage=IntegrationStage.BASE_STOPPED,
                status=ManipulationResultStatus.EXECUTION_REJECTED,
                error_code="BASE_STOP_NOT_CONFIRMED",
                detail="Base stop verification failed: " + "; ".join(problems),
                block_subsequent=True,
                requires_recovery=False,
            )
        return None

    def step_reobserve(
        self,
        post_stop_observation: Optional[CameraObservation3D],
        *,
        checked_at_s: float,
    ) -> Optional[IntegrationOutcomeRecord]:
        """Stage 5: REOBSERVE. Requires a fresh post-stop 3D observation; aborts without moving arm if missing/stale."""
        if self.base_stop_evidence is None:
            raise RuntimeError("step_reobserve requires step_base_stopped first")
        self._transition_to(IntegrationStage.REOBSERVE)
        verdict = self.reobserve_gate.evaluate(
            base_stop=self.base_stop_evidence,
            observation=post_stop_observation,
            checked_at_s=checked_at_s,
            pre_nav_observation=self.pre_nav_detection,
        )
        self.reobserve_verdict = verdict
        if not verdict.passed:
            return self._abort_with_error(
                failed_stage=IntegrationStage.REOBSERVE,
                status=ManipulationResultStatus.OBSERVATION_FAILED,
                error_code="POST_STOP_REOBSERVE_FAILED",
                detail=f"Post-arrival re-observation failed ({verdict.reason}); arm motion forbidden",
                block_subsequent=False,
                requires_recovery=False,
            )
        return None

    def step_manipulation_ready(
        self,
        *,
        handoff_mode: str = "INDEPENDENT_LOCAL_CAMERA_OBSERVATION",
        target_frame: str = "assembled dual model world (left base at its model pose)",
    ) -> Optional[IntegrationOutcomeRecord]:
        """Stage 6: MANIPULATION_READY. Validates TF frame contract before invoking Sweepick."""
        if self.reobserve_verdict is None or not self.reobserve_verdict.passed:
            raise RuntimeError("Cannot enter MANIPULATION_READY without a passed ReobserveVerdict")
        self._transition_to(IntegrationStage.MANIPULATION_READY)
        tf_ok, tf_err = self.frame_contract.validate_handoff_mode(
            handoff_mode=handoff_mode,
            target_frame=target_frame,
        )
        if not tf_ok:
            return self._abort_with_error(
                failed_stage=IntegrationStage.MANIPULATION_READY,
                status=ManipulationResultStatus.EXECUTION_REJECTED,
                error_code="UNVERIFIED_BASE_TO_ARM_TF",
                detail=str(tf_err),
                block_subsequent=True,
                requires_recovery=True,
            )
        return None

    def step_arm_task(
        self,
        *,
        scope: ApprovedSweepickScope = ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT,
        require_full_handover: bool = False,
        right_x_inputs: Optional[Dict[str, Any]] = None,
    ) -> IntegrationOutcomeRecord:
        """Stage 7 & 8: ARM_TASK -> RESULT."""
        if self.current_stage != IntegrationStage.ARM_TASK:
            self._transition_to(IntegrationStage.ARM_TASK)
        self.arm_commands_invoked_count += 1
        adapter_res = self.sweepick_adapter.request_arm_task(
            scope=scope,
            require_full_handover=require_full_handover,
            right_x_inputs=right_x_inputs,
        )
        self.last_adapter_result = adapter_res

        if adapter_res.status == ManipulationResultStatus.WAITING_FOR_OPERATOR_APPROVAL:
            return IntegrationOutcomeRecord(
                current_stage=IntegrationStage.ARM_TASK,
                stage_history=tuple(s.value for s in self.stage_history),
                result_status=ManipulationResultStatus.WAITING_FOR_OPERATOR_APPROVAL,
                manipulation_succeeded=False,
                left_pick_lift_confirmed=False,
                right_handover_confirmed=False,
                approved_scope=adapter_res.approved_scope,
                motor_authority_created_by_integration=False,
                pre_nav_coordinate_reused=False,
                post_stop_reobserved=True,
                subsequent_commands_blocked=False,
                hardware_readiness=self.hardware_readiness.to_dict(),
                reason=adapter_res.reason,
                sweepick_run_id=adapter_res.run_id,
                sweepick_session_result=adapter_res.session_result,
                reobserve_verdict=self.reobserve_verdict.to_dict() if self.reobserve_verdict else None,
            )

        return self._finalize_from_adapter(adapter_res)

    def finalize_after_operator_execution(
        self,
        *,
        scope: ApprovedSweepickScope = ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT,
        require_full_handover: bool = False,
    ) -> IntegrationOutcomeRecord:
        """Evaluate the final result once the operator has approved and the Sweepick owner has finished."""
        if self.current_stage != IntegrationStage.ARM_TASK:
            raise RuntimeError("finalize_after_operator_execution requires current_stage == ARM_TASK")
        status_payload = self.sweepick_adapter.controller.status()
        if status_payload.get("state") in ("PREPARED", "STARTING", "RUNNING") and (
            status_payload.get("state") == "PREPARED"
            or (status_payload.get("active") or {}).get("pending_view")
        ):
            adapter_res = self.sweepick_adapter.request_arm_task(
                scope=scope,
                require_full_handover=require_full_handover,
            )
            self.last_adapter_result = adapter_res
            return IntegrationOutcomeRecord(
                current_stage=IntegrationStage.ARM_TASK,
                stage_history=tuple(s.value for s in self.stage_history),
                result_status=ManipulationResultStatus.WAITING_FOR_OPERATOR_APPROVAL,
                manipulation_succeeded=False,
                left_pick_lift_confirmed=False,
                right_handover_confirmed=False,
                approved_scope=adapter_res.approved_scope,
                motor_authority_created_by_integration=False,
                pre_nav_coordinate_reused=False,
                post_stop_reobserved=True,
                subsequent_commands_blocked=False,
                hardware_readiness=self.hardware_readiness.to_dict(),
                reason=adapter_res.reason,
                sweepick_run_id=adapter_res.run_id,
                sweepick_session_result=adapter_res.session_result,
                reobserve_verdict=self.reobserve_verdict.to_dict() if self.reobserve_verdict else None,
            )

        adapter_res = self.sweepick_adapter.evaluate_session_outcome(
            status_payload=status_payload,
            requested_scope=scope,
            require_full_handover=require_full_handover,
        )
        self.last_adapter_result = adapter_res
        return self._finalize_from_adapter(adapter_res)

    def execute_pipeline(
        self,
        *,
        initial_detection: Optional[CameraObservation3D],
        arrival: NavigationArrivalEvidence,
        base_stop: BaseStopEvidence,
        post_stop_observation: Optional[CameraObservation3D],
        reobserve_checked_at_s: float,
        scope: ApprovedSweepickScope = ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT,
        require_full_handover: bool = False,
        handoff_mode: str = "INDEPENDENT_LOCAL_CAMERA_OBSERVATION",
        target_frame: str = "assembled dual model world (left base at its model pose)",
        right_x_inputs: Optional[Dict[str, Any]] = None,
    ) -> IntegrationOutcomeRecord:
        """Run the full P0/P1 pipeline end-to-end up to ARM_TASK / RESULT."""
        blocked = self._check_not_blocked("execute_pipeline")
        if blocked is not None:
            return blocked

        self.reset_for_next_cycle()
        self.step_detect(initial_detection)
        self.step_navigate()

        arr_abort = self.step_arrived(arrival)
        if arr_abort is not None:
            return arr_abort

        stop_abort = self.step_base_stopped(base_stop)
        if stop_abort is not None:
            return stop_abort

        reobs_abort = self.step_reobserve(
            post_stop_observation,
            checked_at_s=reobserve_checked_at_s,
        )
        if reobs_abort is not None:
            return reobs_abort

        ready_abort = self.step_manipulation_ready(
            handoff_mode=handoff_mode,
            target_frame=target_frame,
        )
        if ready_abort is not None:
            return ready_abort

        return self.step_arm_task(
            scope=scope,
            require_full_handover=require_full_handover,
            right_x_inputs=right_x_inputs,
        )

    def _finalize_from_adapter(
        self,
        adapter_res: SweepickAdapterResult,
    ) -> IntegrationOutcomeRecord:
        self._transition_to(IntegrationStage.RESULT)
        error_report: Optional[TaskManagerErrorReport] = None
        if adapter_res.status in (
            ManipulationResultStatus.STOP,
            ManipulationResultStatus.FAULT,
            ManipulationResultStatus.EXECUTION_REJECTED,
            ManipulationResultStatus.OBSERVATION_FAILED,
        ) or adapter_res.subsequent_commands_blocked:
            if adapter_res.subsequent_commands_blocked:
                self.command_latch_blocked = True
                self.command_block_reason = adapter_res.reason
                self.block_origin = (
                    BLOCK_ORIGIN_SWEEPICK_CONSOLE if adapter_res.run_id else BLOCK_ORIGIN_SWEEPICK_NO_RUN
                )
                self.blocked_run_id = adapter_res.run_id
            if adapter_res.status not in (
                ManipulationResultStatus.COMPLETED,
                ManipulationResultStatus.PARTIAL_COMPLETED,
            ):
                error_report = TaskManagerErrorReport(
                    failed_stage=IntegrationStage.ARM_TASK,
                    result_status=adapter_res.status,
                    error_code=adapter_res.error_code or adapter_res.status.value,
                    detail=adapter_res.reason,
                    subsequent_commands_blocked=adapter_res.subsequent_commands_blocked,
                    requires_operator_recovery=adapter_res.requires_operator_recovery,
                    run_id=adapter_res.run_id,
                    native_gate=adapter_res.native_gate,
                )
                self.task_manager_error_report = error_report

        return IntegrationOutcomeRecord(
            current_stage=IntegrationStage.RESULT,
            stage_history=tuple(s.value for s in self.stage_history),
            result_status=adapter_res.status,
            manipulation_succeeded=(adapter_res.status == ManipulationResultStatus.COMPLETED),
            left_pick_lift_confirmed=adapter_res.left_pick_lift_confirmed,
            right_handover_confirmed=adapter_res.right_handover_confirmed,
            approved_scope=adapter_res.approved_scope,
            motor_authority_created_by_integration=False,
            pre_nav_coordinate_reused=False,
            post_stop_reobserved=bool(self.reobserve_verdict and self.reobserve_verdict.passed),
            subsequent_commands_blocked=self.command_latch_blocked,
            hardware_readiness=self.hardware_readiness.to_dict(),
            reason=adapter_res.reason,
            error_report=error_report.to_dict() if error_report else None,
            sweepick_run_id=adapter_res.run_id,
            sweepick_session_result=adapter_res.session_result,
            reobserve_verdict=self.reobserve_verdict.to_dict() if self.reobserve_verdict else None,
        )

    def _abort_with_error(
        self,
        *,
        failed_stage: IntegrationStage,
        status: ManipulationResultStatus,
        error_code: str,
        detail: str,
        block_subsequent: bool,
        requires_recovery: bool,
    ) -> IntegrationOutcomeRecord:
        self._transition_to(IntegrationStage.RESULT)
        if block_subsequent:
            self.command_latch_blocked = True
            self.command_block_reason = f"{error_code}: {detail}"
            self.block_origin = BLOCK_ORIGIN_INTEGRATION_GATE
            self.blocked_run_id = None
        report = TaskManagerErrorReport(
            failed_stage=failed_stage,
            result_status=status,
            error_code=error_code,
            detail=detail,
            subsequent_commands_blocked=block_subsequent,
            requires_operator_recovery=requires_recovery,
        )
        self.task_manager_error_report = report
        return IntegrationOutcomeRecord(
            current_stage=IntegrationStage.RESULT,
            stage_history=tuple(s.value for s in self.stage_history),
            result_status=status,
            manipulation_succeeded=False,
            left_pick_lift_confirmed=False,
            right_handover_confirmed=False,
            approved_scope=ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT.value,
            motor_authority_created_by_integration=False,
            pre_nav_coordinate_reused=False,
            post_stop_reobserved=bool(self.reobserve_verdict and self.reobserve_verdict.passed),
            subsequent_commands_blocked=self.command_latch_blocked,
            hardware_readiness=self.hardware_readiness.to_dict(),
            reason=detail,
            error_report=report.to_dict(),
            reobserve_verdict=self.reobserve_verdict.to_dict() if self.reobserve_verdict else None,
        )
