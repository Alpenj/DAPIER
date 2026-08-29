#!/usr/bin/env python3
"""Provider-neutral, simulation-only contract for an LLM task supervisor.

The supervisor accepts only high-level skill proposals. It cannot publish ROS,
open a serial device, authorize hardware, or emit joint/torque commands.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from typing import Any, Mapping


SCHEMA_VERSION = "dapier.llm-task-supervisor.v0.1"
MAX_OBSERVATION_AGE_MS = 500.0
MAX_RECOVERY_ATTEMPTS = 2
SKILLS = (
    "observe",
    "approach",
    "pick",
    "handoff",
    "bimanual_align",
    "place",
    "recover",
    "stop",
)
MOTION_SKILLS = frozenset(SKILLS) - {"observe", "stop"}
FORBIDDEN_ARGUMENT_TOKENS = (
    "joint",
    "torque",
    "velocity",
    "effort",
    "motor",
    "serial",
    "device",
    "cmd_vel",
    "hardware",
)
ALLOWED_ARGUMENTS = {
    "observe": frozenset({"view"}),
    "approach": frozenset({"target_id", "standoff_m"}),
    "pick": frozenset({"target_id", "side"}),
    "handoff": frozenset({"target_id", "from_side", "to_side"}),
    "bimanual_align": frozenset({"target_ids", "target_zone"}),
    "place": frozenset({"target_id", "target_zone", "side"}),
    "recover": frozenset({"failure_code"}),
    "stop": frozenset(),
}


@dataclass(frozen=True)
class SupervisorWorldState:
    observation_seq: int
    observation_age_ms: float
    base_stationary: bool
    safety_gate_ready: bool
    simulation_only: bool = True
    recovery_attempts: int = 0

    def validate(self) -> None:
        if self.observation_seq < 0:
            raise ValueError("observation_seq must be non-negative")
        if not math.isfinite(self.observation_age_ms) or self.observation_age_ms < 0:
            raise ValueError("observation_age_ms must be finite and non-negative")
        if self.recovery_attempts < 0:
            raise ValueError("recovery_attempts must be non-negative")


@dataclass(frozen=True)
class SkillProposal:
    schema_version: str
    skill: str
    expected_observation_seq: int
    reason: str
    arguments: Mapping[str, Any]


@dataclass(frozen=True)
class SupervisorDecision:
    accepted: bool
    reason: str
    skill: str
    expected_observation_seq: int | None
    hardware_dispatch_authorized: bool = False
    executed_action: bool = False
    hardware_execution: bool = False

    def as_report(self) -> dict[str, object]:
        return asdict(self)


def _reject(reason: str, proposal: SkillProposal | None = None) -> SupervisorDecision:
    return SupervisorDecision(
        accepted=False,
        reason=reason,
        skill=proposal.skill if proposal else "",
        expected_observation_seq=(
            proposal.expected_observation_seq if proposal else None
        ),
    )


def _contains_forbidden_key(value: Any) -> bool:
    if isinstance(value, Mapping):
        for key, nested in value.items():
            normalized = str(key).lower()
            if any(token in normalized for token in FORBIDDEN_ARGUMENT_TOKENS):
                return True
            if _contains_forbidden_key(nested):
                return True
    elif isinstance(value, list):
        return any(_contains_forbidden_key(item) for item in value)
    return False


def parse_skill_proposal(raw_json: str) -> SkillProposal:
    try:
        payload = json.loads(raw_json)
    except json.JSONDecodeError as error:
        raise ValueError("proposal is not valid JSON") from error
    if not isinstance(payload, dict):
        raise ValueError("proposal must be a JSON object")
    expected_keys = {
        "schema_version",
        "skill",
        "expected_observation_seq",
        "reason",
        "arguments",
    }
    if set(payload) != expected_keys:
        raise ValueError("proposal keys must exactly match the supervisor schema")
    if payload["schema_version"] != SCHEMA_VERSION:
        raise ValueError("proposal schema_version is unsupported")
    skill = payload["skill"]
    if not isinstance(skill, str) or skill not in SKILLS:
        raise ValueError("proposal skill is not allowed")
    sequence = payload["expected_observation_seq"]
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("expected_observation_seq must be a non-negative integer")
    reason = payload["reason"]
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 500:
        raise ValueError("reason must contain 1 to 500 characters")
    arguments = payload["arguments"]
    if not isinstance(arguments, dict):
        raise ValueError("arguments must be a JSON object")
    if set(arguments) - ALLOWED_ARGUMENTS[skill]:
        raise ValueError("proposal contains arguments not allowed for this skill")
    if _contains_forbidden_key(arguments):
        raise ValueError("raw actuator or hardware arguments are forbidden")
    return SkillProposal(
        schema_version=SCHEMA_VERSION,
        skill=skill,
        expected_observation_seq=sequence,
        reason=reason.strip(),
        arguments=arguments,
    )


def evaluate_skill_proposal(
    raw_json: str,
    world: SupervisorWorldState,
) -> SupervisorDecision:
    """Validate an LLM proposal without executing or authorizing any action."""

    try:
        world.validate()
        proposal = parse_skill_proposal(raw_json)
    except ValueError as error:
        return _reject(str(error))

    if proposal.skill == "stop":
        return SupervisorDecision(
            accepted=True,
            reason="stop proposal accepted; no motion executed",
            skill=proposal.skill,
            expected_observation_seq=proposal.expected_observation_seq,
        )
    if not world.simulation_only:
        return _reject("this supervisor revision is simulation-only", proposal)
    if proposal.expected_observation_seq != world.observation_seq:
        return _reject("proposal observation sequence is stale", proposal)
    if world.observation_age_ms > MAX_OBSERVATION_AGE_MS:
        return _reject("world observation is too old", proposal)
    if not world.safety_gate_ready:
        return _reject("independent safety gate is not ready", proposal)
    if proposal.skill in MOTION_SKILLS and not world.base_stationary:
        return _reject("arm/task motion requires a stationary mobile base", proposal)
    if (
        proposal.skill == "recover"
        and world.recovery_attempts >= MAX_RECOVERY_ATTEMPTS
    ):
        return _reject("recovery attempt budget is exhausted", proposal)

    return SupervisorDecision(
        accepted=True,
        reason="high-level skill proposal accepted for simulation executor review",
        skill=proposal.skill,
        expected_observation_seq=proposal.expected_observation_seq,
    )
