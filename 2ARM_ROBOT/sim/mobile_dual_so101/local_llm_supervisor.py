#!/usr/bin/env python3
"""Provider-neutral local-workstation LLM exception supervisor.

Normal mission sequencing remains deterministic.  The LLM may only recommend
one bounded exception directive and never emits a goal pose, joint target,
wheel velocity, motor value, or hardware authorization.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json

from mission_core import MissionPhase
from mission_modules.health import EdgeResourceLimits
from mission_modules.world_state import WorldStateSnapshot


SCHEMA_VERSION = "dapier.local-llm-supervisor.v0.1"


class SupervisorDirective(str, Enum):
    CONTINUE = "continue"
    REOBSERVE = "reobserve"
    REPLAN = "replan"
    REGRASP = "regrasp"
    SAFE_STOP = "safe_stop"


@dataclass(frozen=True)
class LocalSupervisorInput:
    world_sequence: int
    mission_phase: MissionPhase
    snapshot_fresh: bool
    base_stationary: bool
    safety_motion_allowed: bool
    alert_codes: tuple[str, ...]
    fault_codes: tuple[str, ...]
    recovery_attempts: int = 0

    @classmethod
    def from_world_state(
        cls,
        state: WorldStateSnapshot,
        *,
        now_monotonic_ns: int,
        max_snapshot_age_ms: float,
        edge_limits: EdgeResourceLimits,
        recovery_attempts: int = 0,
    ) -> LocalSupervisorInput:
        state.validate()
        return cls(
            world_sequence=state.sequence,
            mission_phase=state.mission_phase,
            snapshot_fresh=state.fresh(
                now_monotonic_ns=now_monotonic_ns,
                max_snapshot_age_ms=max_snapshot_age_ms,
            ),
            base_stationary=state.mobility.stationary(),
            safety_motion_allowed=state.runtime_health.motion_allowed(edge_limits),
            alert_codes=state.runtime_health.alert_codes(edge_limits),
            fault_codes=state.runtime_health.fault_codes,
            recovery_attempts=recovery_attempts,
        )

    def validate(self) -> None:
        if (
            isinstance(self.world_sequence, bool)
            or not isinstance(self.world_sequence, int)
            or self.world_sequence < 0
        ):
            raise ValueError("world_sequence must be a non-negative integer")
        if not isinstance(self.mission_phase, MissionPhase):
            raise ValueError("mission_phase must be a MissionPhase")
        if not all(
            isinstance(value, bool)
            for value in (
                self.snapshot_fresh,
                self.base_stationary,
                self.safety_motion_allowed,
            )
        ):
            raise ValueError("supervisor input flags must be booleans")
        if (
            isinstance(self.recovery_attempts, bool)
            or not isinstance(self.recovery_attempts, int)
            or self.recovery_attempts < 0
        ):
            raise ValueError("recovery_attempts must be a non-negative integer")
        for code in (*self.alert_codes, *self.fault_codes):
            if not isinstance(code, str) or not code.strip() or len(code) > 100:
                raise ValueError("alert and fault codes must contain 1 to 100 characters")


@dataclass(frozen=True)
class DirectiveProposal:
    directive: SupervisorDirective
    expected_world_sequence: int
    reason: str
    failure_code: str = ""


@dataclass(frozen=True)
class DirectiveDecision:
    accepted: bool
    directive: SupervisorDirective | None
    reason: str
    expected_world_sequence: int | None
    hardware_dispatch_authorized: bool = False
    hardware_execution: bool = False


def parse_directive_proposal(raw_json: str) -> DirectiveProposal:
    try:
        payload = json.loads(raw_json)
    except json.JSONDecodeError as error:
        raise ValueError("proposal is not valid JSON") from error
    if not isinstance(payload, dict):
        raise ValueError("proposal must be a JSON object")
    expected_keys = {
        "schema_version",
        "directive",
        "expected_world_sequence",
        "reason",
        "failure_code",
    }
    if set(payload) != expected_keys:
        raise ValueError("proposal keys must exactly match the directive schema")
    if payload["schema_version"] != SCHEMA_VERSION:
        raise ValueError("proposal schema_version is unsupported")
    try:
        directive = SupervisorDirective(payload["directive"])
    except (TypeError, ValueError) as error:
        raise ValueError("proposal directive is not allowed") from error
    sequence = payload["expected_world_sequence"]
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("expected_world_sequence must be a non-negative integer")
    reason = payload["reason"]
    failure_code = payload["failure_code"]
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 500:
        raise ValueError("reason must contain 1 to 500 characters")
    if not isinstance(failure_code, str) or len(failure_code) > 100:
        raise ValueError("failure_code must be a string of at most 100 characters")
    if directive in {
        SupervisorDirective.REOBSERVE,
        SupervisorDirective.REPLAN,
        SupervisorDirective.REGRASP,
    } and not failure_code.strip():
        raise ValueError("recovery directives require a failure_code")
    if directive in {SupervisorDirective.CONTINUE, SupervisorDirective.SAFE_STOP}:
        if failure_code:
            raise ValueError("continue and safe_stop must not include a failure_code")
    return DirectiveProposal(
        directive=directive,
        expected_world_sequence=sequence,
        reason=reason.strip(),
        failure_code=failure_code.strip(),
    )


def _reject(reason: str, proposal: DirectiveProposal | None = None) -> DirectiveDecision:
    return DirectiveDecision(
        accepted=False,
        directive=proposal.directive if proposal else None,
        reason=reason,
        expected_world_sequence=(
            proposal.expected_world_sequence if proposal else None
        ),
    )


def evaluate_directive(
    raw_json: str,
    world: LocalSupervisorInput,
    *,
    max_recovery_attempts: int = 2,
) -> DirectiveDecision:
    """Validate a local LLM recommendation without dispatching any command."""

    if (
        isinstance(max_recovery_attempts, bool)
        or not isinstance(max_recovery_attempts, int)
        or max_recovery_attempts < 0
    ):
        raise ValueError("max_recovery_attempts must be non-negative")
    try:
        world.validate()
        proposal = parse_directive_proposal(raw_json)
    except ValueError as error:
        return _reject(str(error))
    if proposal.directive == SupervisorDirective.SAFE_STOP:
        return DirectiveDecision(
            accepted=True,
            directive=proposal.directive,
            reason="safe-stop recommendation accepted for deterministic controller review",
            expected_world_sequence=proposal.expected_world_sequence,
        )
    if proposal.expected_world_sequence != world.world_sequence:
        return _reject("proposal world sequence is stale", proposal)
    if not world.snapshot_fresh:
        return _reject("world snapshot or edge heartbeat is stale", proposal)
    if not world.safety_motion_allowed and proposal.directive not in {
        SupervisorDirective.REOBSERVE,
    }:
        return _reject("safety fault permits only reobserve or safe_stop", proposal)
    if proposal.directive == SupervisorDirective.REGRASP:
        if world.mission_phase not in {
            MissionPhase.PICKING_AT_B,
            MissionPhase.VERIFYING_GRASP,
        }:
            return _reject("regrasp is not valid in the current mission phase", proposal)
        if not world.base_stationary:
            return _reject("regrasp requires a stationary base", proposal)
    if proposal.directive in {
        SupervisorDirective.REOBSERVE,
        SupervisorDirective.REPLAN,
        SupervisorDirective.REGRASP,
    } and world.recovery_attempts >= max_recovery_attempts:
        return _reject("recovery attempt budget is exhausted", proposal)
    return DirectiveDecision(
        accepted=True,
        directive=proposal.directive,
        reason="bounded directive accepted for deterministic controller review",
        expected_world_sequence=proposal.expected_world_sequence,
    )


__all__ = [
    "DirectiveDecision",
    "DirectiveProposal",
    "LocalSupervisorInput",
    "SCHEMA_VERSION",
    "SupervisorDirective",
    "evaluate_directive",
    "parse_directive_proposal",
]
