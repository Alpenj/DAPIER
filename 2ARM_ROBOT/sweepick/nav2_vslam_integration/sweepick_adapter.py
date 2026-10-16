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
from typing import Any, Dict, List, Optional, Protocol, Tuple

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

# C-2 P0-2: evidence classes. A Motor/Bus FAULT is recognised from the native executor's own
# gate names and from the recorded error text (`console/replay.py:stop_gate` keeps the native
# gate inside the owner_stop reason, e.g. "{'gate': 'bus_lost', ...}"); an `owner_stop` event
# alone is NOT an operator STOP, because the owner also stops itself after a bus loss.
HARDWARE_FAULT_GATES = frozenset({
    "BUS_LOST",
    "SERVO_FAULT",
    "SERVO_OVERLOAD",
    "MOTOR_FAULT",
    "EXECUTOR_EXCEPTION",
    "EXCEPTION",
    "CHILD_TERMINAL_ERROR",
    "FAULT_HOLDING",
    "FAULT_JOINTS_RELEASED",
    "BUS_OR_SERVO_FAULT",
})

HARDWARE_FAULT_TOKENS = (
    "bus_lost",
    "bus could not be opened",
    "could not connect",
    "connectionerror",
    "serialexception",
    "motor check failed",
    "missing motor",
    "모터 무응답",
    "overload",
    "rxpacketerror",
    "servo_fault",
    "servo fault",
    "motor fault",
    "executor_exception",
    "feetechmotorsbus",
)

# Operator STOP evidence: the Console's own stop() (`stop_sent`, `STOP_REQUESTED`, `.STOP` latch)
# or a native gate that names the operator. "owner_stop"/"OWNER_STOP" by itself names the native
# owner, not a person.
OPERATOR_STOP_GATES = frozenset({"OPERATOR_STOP", "OPERATOR_STOPPED", "STOP_LATCH", "SIGINT"})
OPERATOR_STOP_TOKENS = ("operator stop", "sigint", "stop latch", ".stop", "운영자 stop", "운영자 정지")

CONSOLE_FAULT_STATES = frozenset({"FAULT"})


@dataclass(frozen=True)
class TerminalEvidenceClassification:
    """Separated evidence classes read from one Console status payload (C-2 P0-2).

    All three classes are kept; `evaluate_session_outcome` picks the result status with the
    priority FAULT > STOP > OBSERVATION_FAILED but never drops the other classes.
    """

    hardware_fault: Optional[Dict[str, Any]]
    operator_stop: Optional[Dict[str, Any]]
    observation_failure: Optional[str]
    console_fault_state: bool
    refusal: Optional[Dict[str, Any]]
    basis: Tuple[str, ...]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _lower_text(*parts: Any) -> str:
    return " ".join(str(part) for part in parts if part is not None).lower()


def _hardware_fault_from(gate: Optional[str], detail: Any, source: str) -> Optional[Dict[str, Any]]:
    gate_u = str(gate).upper() if isinstance(gate, str) else None
    text = _lower_text(gate, detail)
    if gate_u in HARDWARE_FAULT_GATES or any(token in text for token in HARDWARE_FAULT_TOKENS):
        return {"gate": gate_u or "HARDWARE_FAULT", "detail": str(detail) if detail is not None else "", "source": source}
    return None


def _operator_stop_from(gate: Optional[str], detail: Any, source: str) -> Optional[Dict[str, Any]]:
    gate_u = str(gate).upper() if isinstance(gate, str) else None
    text = _lower_text(gate, detail)
    if gate_u in OPERATOR_STOP_GATES or any(token in text for token in OPERATOR_STOP_TOKENS):
        return {"gate": gate_u or "OPERATOR_STOP", "detail": str(detail) if detail is not None else "", "source": source}
    return None


def _session_result_from_outcome(outcome: Any) -> Optional[str]:
    """`EPISODE.json task_outcome` is "PARTIAL_OR_DIAGNOSTIC: <RESULT>" or "<RESULT>"."""
    if not isinstance(outcome, str) or not outcome or outcome == "UNKNOWN":
        return None
    return outcome.split(":", 1)[1].strip() if ":" in outcome else outcome.strip()


def classify_terminal_evidence(status_payload: Dict[str, Any]) -> TerminalEvidenceClassification:
    """Read one Console `status()` payload and separate hardware FAULT / operator STOP / observation failure."""
    state = status_payload.get("state")
    active = status_payload.get("active") or {}
    terminal = active.get("terminal") if isinstance(active.get("terminal"), dict) else {}
    limited_doc = active.get("limited_terminal") if isinstance(active.get("limited_terminal"), dict) else {}
    first_error = active.get("first_error") if isinstance(active.get("first_error"), dict) else {}
    failure_point = active.get("failure_point") if isinstance(active.get("failure_point"), dict) else {}
    native_terminal = active.get("native_terminal") if isinstance(active.get("native_terminal"), dict) else {}
    basis: List[str] = []

    # --- hardware / executor fault evidence ---
    hardware: Optional[Dict[str, Any]] = None
    candidates = [
        (first_error.get("gate"), first_error.get("detail"), "active.first_error"),
        ((terminal.get("stopped_by") or {}).get("gate") if isinstance(terminal.get("stopped_by"), dict) else None,
         (terminal.get("stopped_by") or {}).get("detail") if isinstance(terminal.get("stopped_by"), dict) else terminal.get("stopped_by"),
         "active.terminal.stopped_by"),
        ((terminal.get("failed_stage") or {}).get("result") if isinstance(terminal.get("failed_stage"), dict) else None, None,
         "active.terminal.failed_stage"),
        (failure_point.get("gate"), failure_point.get("detail") or failure_point.get("reason"), "active.failure_point"),
        (limited_doc.get("terminal"), limited_doc.get("error"), "active.limited_terminal"),
        (None, active.get("error"), "active.error"),
    ]
    for gate, detail, source in candidates:
        hardware = _hardware_fault_from(gate, detail, source)
        if hardware:
            basis.append(f"hardware_fault<-{source}")
            break

    # --- operator STOP evidence ---
    operator: Optional[Dict[str, Any]] = None
    if active.get("stop_sent") is True or state == "STOP_REQUESTED" or active.get("state") == "STOP_REQUESTED":
        operator = {"gate": "CONSOLE_STOP", "detail": "Console stop(): .STOP latch + SIGINT sent", "source": "active.stop_sent"}
        basis.append("operator_stop<-active.stop_sent")
    elif limited_doc.get("terminal") == "OPERATOR_STOPPED" or limited_doc.get("stop_latch_present") is True:
        operator = {"gate": "OPERATOR_STOPPED", "detail": str(limited_doc.get("terminal")), "source": "active.limited_terminal"}
        basis.append("operator_stop<-active.limited_terminal")
    else:
        for gate, detail, source in (
            (first_error.get("gate"), first_error.get("detail"), "active.first_error"),
            ((terminal.get("stopped_by") or {}).get("gate") if isinstance(terminal.get("stopped_by"), dict) else None,
             (terminal.get("stopped_by") or {}).get("detail") if isinstance(terminal.get("stopped_by"), dict) else terminal.get("stopped_by"),
             "active.terminal.stopped_by"),
        ):
            operator = _operator_stop_from(gate, detail, source)
            if operator:
                basis.append(f"operator_stop<-{source}")
                break
    if operator and hardware:
        basis.append("owner_stop_after_hardware_fault->FAULT_wins")

    # --- observation / alignment / grasp-gate failure evidence ---
    session_result = active.get("session_result") or limited_doc.get("session_result") \
        or _session_result_from_outcome(native_terminal.get("result")) \
        or _session_result_from_outcome(active.get("outcome"))
    observation: Optional[str] = None
    if session_result in OBSERVATION_FAILURE_RESULTS:
        observation = str(session_result)
        basis.append("observation_failure<-session_result")
    elif limited_doc.get("terminal") in ("LEFT_CLOSE_UNAVAILABLE", "ENDED_BEFORE_LIFT"):
        observation = str(limited_doc.get("terminal"))
        basis.append("observation_failure<-limited_terminal")
    elif failure_point.get("stage") in OBSERVATION_FAILURE_STEPS and not hardware:
        observation = f"{failure_point.get('stage')}:{failure_point.get('gate') or failure_point.get('reason') or 'observation step failed'}"
        basis.append("observation_failure<-failure_point.stage")

    # --- refusal / authorization ---
    refusal: Optional[Dict[str, Any]] = None
    gate = first_error.get("gate")
    if terminal.get("refused") or first_error.get("kind") == "observation_plan_authorization_failed" or (
        isinstance(gate, str) and "AUTHORIZATION" in gate.upper()
    ):
        refusal = {"gate": gate or "EXECUTION_AUTHORIZATION_REFUSED", "detail": terminal.get("refused") or first_error.get("detail")}
        basis.append("refusal")

    console_fault = state in CONSOLE_FAULT_STATES or active.get("state") in CONSOLE_FAULT_STATES
    if console_fault:
        basis.append("console_state=FAULT")

    return TerminalEvidenceClassification(
        hardware_fault=hardware,
        operator_stop=operator,
        observation_failure=observation,
        console_fault_state=bool(console_fault),
        refusal=refusal,
        basis=tuple(basis),
    )


def active_from_console_run_record(record: Dict[str, Any]) -> Dict[str, Any]:
    """Map a saved `RUN_RECORD.json` (`console/records.py:run_record`, read-only) to the `active` shape
    that `Controller.status()` exposes, so stored real runs can be classified offline."""
    outcome = record.get("outcome")
    native_terminal = record.get("native_terminal") if isinstance(record.get("native_terminal"), dict) else {}
    return {
        "id": record.get("id"),
        "state": record.get("state"),
        "execution_scope": record.get("execution_scope"),
        "first_error": record.get("first_error"),
        "session_result": _session_result_from_outcome(native_terminal.get("result")) or _session_result_from_outcome(outcome),
        "outcome": outcome,
        "normal_return_confirmed": record.get("normal_return_confirmed"),
        "stage_results": record.get("stage_results") or [],
        "failure_point": record.get("failure_point"),
        "native_terminal": native_terminal,
        "terminal": record.get("terminal") or {},
        "limited_terminal": record.get("limited_terminal"),
        "receive_contact": record.get("receive_contact") or {},
        "operator_recovery": record.get("operator_recovery"),
        "record_source": "RUN_RECORD.json",
    }


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
    # C-2 P0-2: evidence classes are preserved side by side; a FAULT never hides an observation
    # failure and an observation failure never hides an executor FAULT.
    hardware_fault: Optional[Dict[str, Any]] = None
    operator_stop: Optional[Dict[str, Any]] = None
    observation_failure: Optional[str] = None
    classification_basis: Tuple[str, ...] = ()

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

        evidence = classify_terminal_evidence(status_payload)
        session_result = session_result or (evidence.observation_failure if evidence.observation_failure in OBSERVATION_FAILURE_RESULTS else None)
        limited_kind = limited_doc.get("terminal") if isinstance(limited_doc, dict) else None
        preserved = dict(
            hardware_fault=evidence.hardware_fault,
            operator_stop=evidence.operator_stop,
            observation_failure=evidence.observation_failure,
            classification_basis=evidence.basis,
        )

        # 1. Motor/Bus/executor FAULT wins over everything, including an owner_stop that followed it
        #    and an observation failure recorded in the same session (both are kept in the result).
        if evidence.hardware_fault is not None:
            fault = evidence.hardware_fault
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
                reason=str(fault.get("detail") or error_detail or "Sweepick Motor/Bus FAULT · 수동 복구 확인 전 다음 명령 차단")
                + (f" · 관측 실패 별도 보존: {evidence.observation_failure}" if evidence.observation_failure else ""),
                error_code="SWEEPICK_FAULT",
                native_gate=str(fault.get("gate") or error_gate or "HARDWARE_FAULT"),
                **preserved,
            )

        # 2. Operator STOP (Console stop() / .STOP latch / operator-named gate), no hardware fault
        if evidence.operator_stop is not None:
            return SweepickAdapterResult(
                status=ManipulationResultStatus.STOP,
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
                reason=str(evidence.operator_stop.get("detail") or error_detail or "Sweepick STOP 발생 · 자동 재시작 금지 및 수동 복구 필요"),
                error_code="SWEEPICK_STOP",
                native_gate=str(evidence.operator_stop.get("gate") or error_gate or "OPERATOR_STOP"),
                **preserved,
            )

        # 3. Explicit refusal / authorization rejection
        if evidence.refusal is not None:
            return self._rejected(
                scope_id=scope_id,
                run_id=run_id,
                error_code=str(evidence.refusal.get("gate") or "EXECUTION_AUTHORIZATION_REFUSED"),
                reason=str(evidence.refusal.get("detail") or error_detail or "Sweepick 실행/관측 승인 거부"),
                native_gate=error_gate,
                **preserved,
            )

        # 4. The Console's own FAULT verdict (executor error, unconfirmed torque-off, owner stop of
        #    unknown origin) is never downgraded to OBSERVATION_FAILED; the observation failure is kept.
        if (
            evidence.console_fault_state
            or first_error
            or active.get("error")
            or limited_kind in ("BUS_OR_SERVO_FAULT", "LEFT_LIFT_FAILED")
        ):
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
                reason=str(error_detail or "Sweepick FAULT 발생 · 수동 복구 확인 전 다음 명령 차단")
                + (f" · 관측 실패 별도 보존: {evidence.observation_failure}" if evidence.observation_failure else ""),
                error_code="SWEEPICK_FAULT",
                native_gate=error_gate,
                **preserved,
            )

        # 5. Observation / alignment / grasp-gate failure with the executor itself healthy
        if evidence.observation_failure is not None:
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
                **preserved,
            )

        # 6. Evaluate actual manipulation evidence (LEFT PICK/LIFT & RIGHT HANDOVER)
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
    def _rejected(
        *,
        scope_id: str,
        run_id: Optional[str],
        error_code: str,
        reason: str,
        native_gate: Optional[str] = None,
        **preserved: Any,
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
            **preserved,
        )
