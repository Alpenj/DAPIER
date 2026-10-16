"""P1 Sweepick Execution Adapter.

Connects the Navigation coordinator to Sweepick's approved execution scopes
(`FULL_CHAIN_RECEIVE_CONTACT`, `LEFT_START_TO_LIFT`, `RIGHT_X_FINITE_METROLOGY`)
while strictly preserving:
- Single-owner lock and device occupancy checks
- Operator START approval & manifest SHA256 binding
- Zero Motor/Torque write authority inside the integration layer
- Strict outcome classification (`WAITING_FOR_OPERATOR_APPROVAL`, `COMPLETED`,
  `PARTIAL_COMPLETED`, `OBSERVATION_FAILED`, `EXECUTION_REJECTED`, `STOP`, `FAULT`)
- Never marking `COMPLETED` unless actual left-arm PICK/LIFT or right-arm handover
  is confirmed completed.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional, Protocol

from .contracts import (
    ApprovedSweepickScope,
    ManipulationResultStatus,
)


OBSERVATION_FAILURE_RESULTS = frozenset({
    "NOT_ALIGNED_RETURNED_HOME",
    "NOT_READY_ABOVE_THE_BLOCK_RETURNED_HOME",
    "NOT_READY_JUST_ABOVE_THE_BLOCK_RETURNED_HOME",
    "SECOND_STOP_NOT_PLANNABLE_RETURNED_HOME",
    "PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME",
    "NO_CONTACT_RETURNED_HOME",
    "CONTACT_WITHOUT_THE_BLOCK_SEEN_RETURNED_HOME",
    "LIFT_NOT_CONFIRMED_RETURNED_HOME",
})

OBSERVATION_FAILURE_STEPS = frozenset({
    "observe",
    "look",
    "look_failed",
    "visual",
    "align",
})


class SweepickConsoleProtocol(Protocol):
    """Protocol matching `2ARM_ROBOT/sweepick/console/control.py:Controller`."""

    def status(self) -> Dict[str, Any]:
        ...

    def operation_contract(self) -> Dict[str, Any]:
        ...

    def prepare(
        self,
        source_session: Optional[str] = None,
        *,
        execution_scope: str = "FULL_CHAIN_RECEIVE_CONTACT",
        auto_after: Optional[str] = None,
        right_x: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        ...

    def stop(self, job_id: str) -> Dict[str, Any]:
        ...


@dataclass(frozen=True)
class SweepickAdapterResult:
    """Evaluated outcome from the Sweepick execution adapter."""

    status: ManipulationResultStatus
    approved_scope: str
    run_id: Optional[str]
    session_result: Optional[str]
    left_pick_lift_confirmed: bool
    right_handover_confirmed: bool
    normal_return_off_confirmed: bool
    operator_approval_required: bool
    subsequent_commands_blocked: bool
    requires_operator_recovery: bool
    motor_authority_created_by_integration: bool
    reason: str
    error_code: Optional[str] = None
    native_gate: Optional[str] = None

    def __post_init__(self) -> None:
        if self.motor_authority_created_by_integration is not False:
            raise ValueError("SweepickExecutionAdapter must never create motor/torque authority")
        if self.status == ManipulationResultStatus.COMPLETED:
            if not (self.left_pick_lift_confirmed or self.right_handover_confirmed):
                raise ValueError(
                    "SweepickAdapterResult cannot report COMPLETED without confirmed "
                    "left PICK/LIFT or right handover"
                )

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["status"] = self.status.value
        return data


class SweepickExecutionAdapter:
    """Adapter that invokes approved Sweepick Console scopes without creating motor authority."""

    motor_write_authorized: bool = False
    auto_start_permitted: bool = False
    direct_device_open_permitted: bool = False

    def __init__(self, controller: SweepickConsoleProtocol) -> None:
        self.controller = controller

    def request_arm_task(
        self,
        *,
        scope: ApprovedSweepickScope = ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT,
        require_full_handover: bool = False,
        right_x_inputs: Optional[Dict[str, Any]] = None,
    ) -> SweepickAdapterResult:
        """Prepare or inspect an approved Sweepick manipulation session.

        - Never calls `controller.start(...)` automatically (operator must approve START).
        - If a session is IDLE and `can_prepare` is True, prepares the unapproved session and
          returns `WAITING_FOR_OPERATOR_APPROVAL`.
        - If the session is already `PREPARED` or awaiting a mid-session observation move decision,
          returns `WAITING_FOR_OPERATOR_APPROVAL`.
        - Once the operator starts and the native owner completes, classifies the terminal evidence.
        """
        scope_id = scope.value
        try:
            contract = self.controller.operation_contract()
            status = self.controller.status()
        except Exception as exc:
            return self._rejected(
                scope_id=scope_id,
                run_id=None,
                error_code="CONTROLLER_STATUS_ERROR",
                reason=f"{type(exc).__name__}: {exc}",
            )

        scope_entries = {
            row.get("id"): row
            for row in (contract.get("scopes") or [])
            if isinstance(row, dict)
        }
        scope_contract = scope_entries.get(scope_id)
        if not scope_contract or scope_contract.get("supported") is not True:
            backend_reason = (scope_contract or {}).get("reason") or f"scope {scope_id} not supported"
            return self._rejected(
                scope_id=scope_id,
                run_id=(status.get("active") or {}).get("id"),
                error_code="UNSUPPORTED_OR_UNVERIFIED_BACKEND_SCOPE",
                reason=str(backend_reason),
            )

        owner_info = contract.get("owner") or {}
        if owner_info.get("device_query_error"):
            return self._rejected(
                scope_id=scope_id,
                run_id=(status.get("active") or {}).get("id"),
                error_code="DEVICE_ROUTE_OR_WRIST_IDENTITY_UNCONFIRMED",
                reason=str(owner_info["device_query_error"]),
            )

        state = status.get("state")
        active = status.get("active") or {}
        run_id = active.get("id")

        if state == "BLOCKED":
            return self._rejected(
                scope_id=scope_id,
                run_id=run_id,
                error_code="FOREIGN_MANAGED_OWNER_ACTIVE",
                reason=status.get("block_reason") or "기존 managed owner가 존재함",
            )

        if state == "IDLE":
            occupancy = owner_info.get("device_occupancy")
            if isinstance(occupancy, dict) and len(occupancy) > 0:
                return self._rejected(
                    scope_id=scope_id,
                    run_id=run_id,
                    error_code="DEVICE_OCCUPIED_BY_OTHER_PROCESS",
                    reason=f"장치 owner가 이미 존재함: {occupancy}",
                )
            if not status.get("can_prepare"):
                return self._rejected(
                    scope_id=scope_id,
                    run_id=run_id,
                    error_code="CANNOT_PREPARE_SESSION",
                    reason=status.get("block_reason") or "새 세션을 준비할 수 없음",
                )
            try:
                status = self.controller.prepare(
                    None,
                    execution_scope=scope_id,
                    right_x=right_x_inputs if scope == ApprovedSweepickScope.RIGHT_X_FINITE_METROLOGY else None,
                )
                state = status.get("state")
                active = status.get("active") or {}
                run_id = active.get("id")
            except Exception as exc:
                return self._rejected(
                    scope_id=scope_id,
                    run_id=run_id,
                    error_code="PREPARE_REJECTED",
                    reason=f"{type(exc).__name__}: {exc}",
                )

        active_scope = active.get("execution_scope", ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT.value)
        if active and active_scope != scope_id and state in ("PREPARED", "STARTING", "RUNNING"):
            return self._rejected(
                scope_id=scope_id,
                run_id=run_id,
                error_code="ACTIVE_SESSION_SCOPE_MISMATCH",
                reason=f"활성 세션 범위({active_scope})가 요청 범위({scope_id})와 다름",
            )

        if state == "PREPARED":
            return SweepickAdapterResult(
                status=ManipulationResultStatus.WAITING_FOR_OPERATOR_APPROVAL,
                approved_scope=scope_id,
                run_id=run_id,
                session_result=None,
                left_pick_lift_confirmed=False,
                right_handover_confirmed=False,
                normal_return_off_confirmed=False,
                operator_approval_required=True,
                subsequent_commands_blocked=False,
                requires_operator_recovery=False,
                motor_authority_created_by_integration=False,
                reason="세션 준비 완료 (PREPARED) — 운영자 현장 확인 및 콘솔 START 승인 대기",
            )

        if state in ("STARTING", "RUNNING"):
            pending_view = active.get("pending_view")
            if pending_view:
                if pending_view.get("approval_refused"):
                    return self._rejected(
                        scope_id=scope_id,
                        run_id=run_id,
                        error_code="OBSERVATION_MOVE_SCOPE_MISMATCH",
                        reason=str(pending_view["approval_refused"]),
                    )
                return SweepickAdapterResult(
                    status=ManipulationResultStatus.WAITING_FOR_OPERATOR_APPROVAL,
                    approved_scope=scope_id,
                    run_id=run_id,
                    session_result=None,
                    left_pick_lift_confirmed=False,
                    right_handover_confirmed=False,
                    normal_return_off_confirmed=False,
                    operator_approval_required=True,
                    subsequent_commands_blocked=False,
                    requires_operator_recovery=False,
                    motor_authority_created_by_integration=False,
                    reason=(
                        f"실행 중 추가 관측/측정 이동({pending_view.get('scope')}) 요청 발생 — "
                        "운영자 승인 대기 (WAITING_FOR_OPERATOR_APPROVAL)"
                    ),
                )

        return self.evaluate_session_outcome(
            status_payload=status,
            requested_scope=scope,
            require_full_handover=require_full_handover,
        )

    def evaluate_session_outcome(
        self,
        *,
        status_payload: Dict[str, Any],
        requested_scope: ApprovedSweepickScope = ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT,
        require_full_handover: bool = False,
    ) -> SweepickAdapterResult:
        """Classify a finished or stopped Sweepick session into P1 result categories."""
        state = status_payload.get("state")
        active = status_payload.get("active") or {}
        run_id = active.get("id")
        scope_id = active.get("execution_scope") or requested_scope.value
        terminal = active.get("terminal") or {}
        limited_doc = active.get("limited_terminal") or {}
        first_error = active.get("first_error") if isinstance(active.get("first_error"), dict) else {}
        error_gate = first_error.get("gate")
        error_detail = first_error.get("detail") or active.get("error") or status_payload.get("block_reason")
        session_result = active.get("session_result") or (limited_doc.get("session_result") if isinstance(limited_doc, dict) else None)
        stages: List[Dict[str, Any]] = active.get("stage_results") or []

        # 1. Check explicit STOP conditions first
        stop_requested = (
            state == "STOP_REQUESTED"
            or active.get("stop_sent") is True
            or first_error.get("stop_event") == "owner_stop"
            or bool(terminal.get("stopped_by"))
            or (isinstance(limited_doc, dict) and (
                limited_doc.get("terminal") == "OPERATOR_STOPPED"
                or limited_doc.get("stop_latch_present") is True
            ))
            or (isinstance(error_gate, str) and "STOP" in error_gate.upper())
        )
        if stop_requested:
            return SweepickAdapterResult(
                status=ManipulationResultStatus.STOP,
                approved_scope=scope_id,
                run_id=run_id,
                session_result=session_result,
                left_pick_lift_confirmed=False,
                right_handover_confirmed=False,
                normal_return_off_confirmed=False,
                operator_approval_required=False,
                subsequent_commands_blocked=True,
                requires_operator_recovery=True,
                motor_authority_created_by_integration=False,
                reason=str(error_detail or "Sweepick STOP 발생 · 자동 재시작 금지 및 수동 복구 필요"),
                error_code="SWEEPICK_STOP",
                native_gate=error_gate or "OPERATOR_OR_OWNER_STOP",
            )

        # 2. Check explicit refusal / authorization rejection
        if terminal.get("refused") or first_error.get("kind") == "observation_plan_authorization_failed" or (
            isinstance(error_gate, str) and "AUTHORIZATION" in error_gate.upper()
        ):
            return self._rejected(
                scope_id=scope_id,
                run_id=run_id,
                error_code=error_gate or "EXECUTION_AUTHORIZATION_REFUSED",
                reason=str(error_detail or "Sweepick 실행/관측 승인 거부"),
                native_gate=error_gate,
            )

        # 3. Check native observation / alignment failure before grasp/lift
        failure_stage = (active.get("failure_point") or {}).get("stage")
        limited_kind = limited_doc.get("terminal") if isinstance(limited_doc, dict) else None
        if (
            session_result in OBSERVATION_FAILURE_RESULTS
            or (failure_stage in OBSERVATION_FAILURE_STEPS and not self._has_servo_or_bus_fault(first_error, error_detail))
            or limited_kind in ("LEFT_CLOSE_UNAVAILABLE", "ENDED_BEFORE_LIFT")
        ):
            normal_off = bool(active.get("normal_return_confirmed") or (isinstance(limited_doc, dict) and limited_doc.get("torque_off_confirmed")))
            return SweepickAdapterResult(
                status=ManipulationResultStatus.OBSERVATION_FAILED,
                approved_scope=scope_id,
                run_id=run_id,
                session_result=session_result or limited_kind,
                left_pick_lift_confirmed=False,
                right_handover_confirmed=False,
                normal_return_off_confirmed=normal_off,
                operator_approval_required=False,
                subsequent_commands_blocked=not normal_off,
                requires_operator_recovery=not normal_off,
                motor_authority_created_by_integration=False,
                reason=str(error_detail or session_result or limited_kind or "Sweepick 시각 관측/정렬 또는 파지 확인 실패"),
                error_code="SWEEPICK_OBSERVATION_OR_GRASP_GATE_FAILED",
                native_gate=error_gate or str(session_result or limited_kind),
            )

        # 4. Check FAULT conditions
        if state == "FAULT" or first_error or active.get("error") or limited_kind in ("BUS_OR_SERVO_FAULT", "LEFT_LIFT_FAILED"):
            return SweepickAdapterResult(
                status=ManipulationResultStatus.FAULT,
                approved_scope=scope_id,
                run_id=run_id,
                session_result=session_result or limited_kind,
                left_pick_lift_confirmed=False,
                right_handover_confirmed=False,
                normal_return_off_confirmed=False,
                operator_approval_required=False,
                subsequent_commands_blocked=True,
                requires_operator_recovery=True,
                motor_authority_created_by_integration=False,
                reason=str(error_detail or "Sweepick FAULT 발생 · 수동 복구 확인 전 다음 명령 차단"),
                error_code="SWEEPICK_FAULT",
                native_gate=error_gate,
            )

        # 5. Evaluate actual manipulation evidence (LEFT PICK/LIFT & RIGHT HANDOVER)
        stage_map = {
            (s.get("side", "left"), s.get("stage")): s.get("result")
            for s in stages
            if isinstance(s, dict)
        }
        close_ok = stage_map.get(("left", "close")) == "GRIP_CONTACT_HOLDING"
        lift_ok = stage_map.get(("left", "lift")) == "HOLDING"
        held_after_lift = bool(
            active.get("held_after_lift") is True
            or (isinstance(limited_doc, dict) and limited_doc.get("held_after_lift") is True)
        )
        left_pick_lift_confirmed = bool(close_ok and lift_ok and held_after_lift)

        receive_contact = active.get("receive_contact") or {}
        right_handover_confirmed = bool(
            session_result == "FULL_CHAIN_DONE"
            or receive_contact.get("bimanual_handover_success") is True
        )
        off_confirmed = bool(active.get("normal_return_confirmed") is True)

        # Handle LEFT_START_TO_LIFT limited scope
        if scope_id == ApprovedSweepickScope.LEFT_START_TO_LIFT.value or state == "LIMITED_HOLDING":
            if limited_kind == "LEFT_LIFT_HELD_CONFIRMED" and left_pick_lift_confirmed:
                outcome_status = (
                    ManipulationResultStatus.PARTIAL_COMPLETED
                    if require_full_handover
                    else ManipulationResultStatus.COMPLETED
                )
                return SweepickAdapterResult(
                    status=outcome_status,
                    approved_scope=scope_id,
                    run_id=run_id,
                    session_result=session_result or limited_kind,
                    left_pick_lift_confirmed=True,
                    right_handover_confirmed=False,
                    normal_return_off_confirmed=False,
                    operator_approval_required=False,
                    subsequent_commands_blocked=True,  # Left arm holds object with Torque ON; manual recovery required before next run
                    requires_operator_recovery=True,
                    motor_authority_created_by_integration=False,
                    reason=(
                        "LEFT_START_TO_LIFT 완료 (LEFT_LIFT_HELD_CONFIRMED) · 왼팔 파지/들기 확인됨 · "
                        "Torque ON 유지 상태이므로 수동 복구 확인 전 후속 명령 차단"
                    ),
                )
            if limited_kind == "LEFT_LIFT_HOLD_UNKNOWN":
                return SweepickAdapterResult(
                    status=ManipulationResultStatus.OBSERVATION_FAILED,
                    approved_scope=scope_id,
                    run_id=run_id,
                    session_result=limited_kind,
                    left_pick_lift_confirmed=False,
                    right_handover_confirmed=False,
                    normal_return_off_confirmed=False,
                    operator_approval_required=False,
                    subsequent_commands_blocked=True,
                    requires_operator_recovery=True,
                    motor_authority_created_by_integration=False,
                    reason="LEFT_START_TO_LIFT 종료 후 물체 파지 보유(held_after_lift) 미확인 · 성공 처리 금지",
                    error_code="LEFT_LIFT_HOLD_UNKNOWN",
                )

        # Handle RIGHT_X_FINITE_METROLOGY diagnostic scope (never marked COMPLETED)
        if scope_id == ApprovedSweepickScope.RIGHT_X_FINITE_METROLOGY.value:
            if not off_confirmed:
                return SweepickAdapterResult(
                    status=ManipulationResultStatus.FAULT,
                    approved_scope=scope_id,
                    run_id=run_id,
                    session_result=session_result,
                    left_pick_lift_confirmed=left_pick_lift_confirmed,
                    right_handover_confirmed=False,
                    normal_return_off_confirmed=False,
                    operator_approval_required=False,
                    subsequent_commands_blocked=True,
                    requires_operator_recovery=True,
                    motor_authority_created_by_integration=False,
                    reason="RIGHT_X 측정 종료 후 양팔 Torque OFF 미확인",
                    error_code="TORQUE_OFF_UNCONFIRMED",
                )
            return SweepickAdapterResult(
                status=ManipulationResultStatus.PARTIAL_COMPLETED,
                approved_scope=scope_id,
                run_id=run_id,
                session_result=session_result or "RIGHT_X_METROLOGY_DONE",
                left_pick_lift_confirmed=left_pick_lift_confirmed,
                right_handover_confirmed=False,
                normal_return_off_confirmed=True,
                operator_approval_required=False,
                subsequent_commands_blocked=False,
                requires_operator_recovery=False,
                motor_authority_created_by_integration=False,
                reason="RIGHT_X_FINITE_METROLOGY 비접촉 측정 범위 종료 (측정 세션은 전체 조작 완료로 분류하지 않음)",
            )

        # Handle FULL_CHAIN_RECEIVE_CONTACT
        if not off_confirmed:
            return SweepickAdapterResult(
                status=ManipulationResultStatus.FAULT,
                approved_scope=scope_id,
                run_id=run_id,
                session_result=session_result,
                left_pick_lift_confirmed=left_pick_lift_confirmed,
                right_handover_confirmed=right_handover_confirmed,
                normal_return_off_confirmed=False,
                operator_approval_required=False,
                subsequent_commands_blocked=True,
                requires_operator_recovery=True,
                motor_authority_created_by_integration=False,
                reason="세션 종료 후 양팔 RETURNED_RELEASED / Torque OFF 미확인 · FAULT 분류",
                error_code="TORQUE_OFF_UNCONFIRMED",
            )

        if not (left_pick_lift_confirmed or right_handover_confirmed):
            return SweepickAdapterResult(
                status=ManipulationResultStatus.OBSERVATION_FAILED,
                approved_scope=scope_id,
                run_id=run_id,
                session_result=session_result,
                left_pick_lift_confirmed=False,
                right_handover_confirmed=False,
                normal_return_off_confirmed=True,
                operator_approval_required=False,
                subsequent_commands_blocked=False,
                requires_operator_recovery=False,
                motor_authority_created_by_integration=False,
                reason="실제 왼팔 PICK/LIFT(held_after_lift) 또는 오른팔 인계가 확인되지 않아 성공 표시 불가",
                error_code="PICK_LIFT_OR_HANDOVER_NOT_CONFIRMED",
            )

        if right_handover_confirmed:
            return SweepickAdapterResult(
                status=ManipulationResultStatus.COMPLETED,
                approved_scope=scope_id,
                run_id=run_id,
                session_result=session_result,
                left_pick_lift_confirmed=left_pick_lift_confirmed,
                right_handover_confirmed=True,
                normal_return_off_confirmed=True,
                operator_approval_required=False,
                subsequent_commands_blocked=False,
                requires_operator_recovery=False,
                motor_authority_created_by_integration=False,
                reason="오른팔 인계(FULL_CHAIN_DONE) 및 정상 복귀/Torque OFF 확인 완료",
            )

        # Left PICK/LIFT confirmed, followed by RECEIVE_CONTACT non-contact observation and put-back
        receive_done = session_result in (
            "RECEIVE_CONTACT_DONE_PUT_BACK",
            "RECEIVE_APPROACH_DONE_PUT_BACK",
            "PICKED_LIFTED_AND_PUT_BACK",
        )
        if left_pick_lift_confirmed and receive_done:
            outcome_status = (
                ManipulationResultStatus.PARTIAL_COMPLETED
                if require_full_handover
                else ManipulationResultStatus.COMPLETED
            )
            return SweepickAdapterResult(
                status=outcome_status,
                approved_scope=scope_id,
                run_id=run_id,
                session_result=session_result,
                left_pick_lift_confirmed=True,
                right_handover_confirmed=False,
                normal_return_off_confirmed=True,
                operator_approval_required=False,
                subsequent_commands_blocked=False,
                requires_operator_recovery=False,
                motor_authority_created_by_integration=False,
                reason=(
                    "왼팔 PICK/LIFT 확인 및 승인된 RECEIVE_CONTACT 비접촉 관측·정상 put-back/Torque OFF 완료"
                    if not require_full_handover
                    else "왼팔 PICK/LIFT 및 비접촉 관측 완료 (오른팔 실물 인계 미포함 범위이므로 PARTIAL_COMPLETED)"
                ),
            )

        return SweepickAdapterResult(
            status=ManipulationResultStatus.PARTIAL_COMPLETED,
            approved_scope=scope_id,
            run_id=run_id,
            session_result=session_result,
            left_pick_lift_confirmed=left_pick_lift_confirmed,
            right_handover_confirmed=right_handover_confirmed,
            normal_return_off_confirmed=off_confirmed,
            operator_approval_required=False,
            subsequent_commands_blocked=False,
            requires_operator_recovery=False,
            motor_authority_created_by_integration=False,
            reason=f"부분 실행 완료 ({session_result or 'PARTIAL'})",
        )

    @staticmethod
    def _has_servo_or_bus_fault(first_error: Dict[str, Any], detail: Optional[str]) -> bool:
        text = f"{first_error.get('gate') or ''} {detail or ''}".lower()
        return any(
            token in text
            for token in ("motor check failed", "overload", "bus_lost", "connectionerror", "servo", "executor_error")
        )

    @staticmethod
    def _rejected(
        *,
        scope_id: str,
        run_id: Optional[str],
        error_code: str,
        reason: str,
        native_gate: Optional[str] = None,
    ) -> SweepickAdapterResult:
        return SweepickAdapterResult(
            status=ManipulationResultStatus.EXECUTION_REJECTED,
            approved_scope=scope_id,
            run_id=run_id,
            session_result=None,
            left_pick_lift_confirmed=False,
            right_handover_confirmed=False,
            normal_return_off_confirmed=False,
            operator_approval_required=False,
            subsequent_commands_blocked=True,
            requires_operator_recovery=True,
            motor_authority_created_by_integration=False,
            reason=reason,
            error_code=error_code,
            native_gate=native_gate,
        )
