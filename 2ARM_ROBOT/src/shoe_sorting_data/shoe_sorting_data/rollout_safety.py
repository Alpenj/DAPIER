"""Policy-independent rollout safety supervisor and dry-run JDcobot ROS 2 adapter.

This module cannot publish to ROS 2 or write to a motor bus. It validates a
policy proposal, produces an auditable decision, and maps approved dry-run
commands into ROS 2 JointTrajectory-shaped envelopes for later integration.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

from shoe_sorting_data.offline_evaluator import ACTION_NAMES


ROLLOUT_SAFETY_SCHEMA_VERSION = "dapier.rollout-safety.v0.1"
ROLLOUT_TRACE_SCHEMA_VERSION = "dapier.rollout-trace.v0.1"


class SafetyContractError(ValueError):
    pass


LIFECYCLE_STATES = {"UNCONFIGURED", "INACTIVE", "ARMED", "ACTIVE", "FAULT_LATCHED", "FINALIZED"}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _require_sha256(value: Any, field: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise SafetyContractError(f"{field} must be a lowercase SHA-256 hex digest")
    return value


def _finite_vector(value: Any, *, dimension: int, field: str) -> list[float]:
    if not isinstance(value, list) or len(value) != dimension:
        raise SafetyContractError(f"{field} must be a {dimension}-element list")
    result = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)):
            raise SafetyContractError(f"{field}[{index}] must be finite")
        result.append(float(item))
    return result


def validate_safety_config(config: Mapping[str, Any]) -> dict[str, Any]:
    if config.get("schema_version") != ROLLOUT_SAFETY_SCHEMA_VERSION:
        raise SafetyContractError("unsupported rollout safety config schema")
    if config.get("action_names") != list(ACTION_NAMES):
        raise SafetyContractError("safety action names/order differ from the 12-DoF policy contract")
    lower = _finite_vector(config.get("joint_lower"), dimension=12, field="joint_lower")
    upper = _finite_vector(config.get("joint_upper"), dimension=12, field="joint_upper")
    max_delta = _finite_vector(config.get("max_delta_per_step"), dimension=12, field="max_delta_per_step")
    if any(low >= high for low, high in zip(lower, upper, strict=True)):
        raise SafetyContractError("every joint lower limit must be below its upper limit")
    if any(value <= 0 for value in max_delta):
        raise SafetyContractError("max_delta_per_step values must be positive")
    for field in (
        "base_linear_tolerance_mps",
        "base_angular_tolerance_radps",
        "max_observation_age_ms",
        "max_feedback_age_ms",
        "max_proposal_age_ms",
    ):
        value = config.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or float(value) <= 0:
            raise SafetyContractError(f"{field} must be positive")
    if config.get("reject_instead_of_clip") is not True:
        raise SafetyContractError("supervisor must reject unsafe actions instead of silently clipping")
    _require_sha256(config.get("expected_hardware_profile_sha256"), "expected_hardware_profile_sha256")
    _require_sha256(config.get("approved_policy_checkpoint_sha256"), "approved_policy_checkpoint_sha256")
    if config.get("hardware_enabled") is True:
        required_live = {
            "limit_source": "measured_and_physically_approved",
            "mechanical_calibration_verified": True,
            "e_stop_verified": True,
            "operator_authorization_verified": True,
            "ros2_controller_contract_verified": True,
        }
        mismatches = [field for field, expected in required_live.items() if config.get(field) != expected]
        if mismatches:
            raise SafetyContractError(
                "hardware_enabled requires verified physical gates; missing/mismatched: " + ", ".join(mismatches)
            )
    return {
        "joint_lower": lower,
        "joint_upper": upper,
        "max_delta_per_step": max_delta,
    }


def supervise_action(
    config: Mapping[str, Any],
    proposal: Mapping[str, Any],
    snapshot: Mapping[str, Any],
) -> dict[str, Any]:
    """Return a deterministic PASS/REJECT decision without modifying the action."""

    normalized = validate_safety_config(config)
    action = _finite_vector(proposal.get("action"), dimension=12, field="proposal.action")
    measured = _finite_vector(snapshot.get("measured_action"), dimension=12, field="snapshot.measured_action")
    base_velocity = _finite_vector(snapshot.get("base_velocity"), dimension=2, field="snapshot.base_velocity")
    now_ns = snapshot.get("now_monotonic_ns")
    observation_ns = snapshot.get("observation_monotonic_ns")
    feedback_ns = snapshot.get("feedback_monotonic_ns")
    proposal_ns = proposal.get("created_monotonic_ns")
    for field, value in (
        ("snapshot.now_monotonic_ns", now_ns),
        ("snapshot.observation_monotonic_ns", observation_ns),
        ("snapshot.feedback_monotonic_ns", feedback_ns),
        ("proposal.created_monotonic_ns", proposal_ns),
    ):
        if not isinstance(value, int) or value < 0:
            raise SafetyContractError(f"{field} must be a non-negative integer")
    observation_age_ms = (now_ns - observation_ns) / 1_000_000
    feedback_age_ms = (now_ns - feedback_ns) / 1_000_000
    proposal_age_ms = (now_ns - proposal_ns) / 1_000_000
    reasons: list[str] = []

    if observation_age_ms < 0:
        reasons.append("observation_timestamp_in_future")
    elif observation_age_ms > float(config["max_observation_age_ms"]):
        reasons.append("stale_observation")
    if proposal_age_ms < 0:
        reasons.append("proposal_timestamp_in_future")
    elif proposal_age_ms > float(config["max_proposal_age_ms"]):
        reasons.append("stale_action_proposal")
    if feedback_age_ms < 0:
        reasons.append("feedback_timestamp_in_future")
    elif feedback_age_ms > float(config["max_feedback_age_ms"]):
        reasons.append("stale_feedback")
    if proposal.get("hardware_profile_sha256") != config.get("expected_hardware_profile_sha256"):
        reasons.append("profile_mismatch")
    if proposal.get("policy_checkpoint_sha256") != config.get("approved_policy_checkpoint_sha256"):
        reasons.append("unapproved_policy")
    if proposal.get("policy_reset_generation") != snapshot.get("expected_policy_reset_generation"):
        reasons.append("stale_chunk_generation")
    if snapshot.get("e_stop_healthy") is not True:
        reasons.append("e_stop_not_healthy")
    if snapshot.get("watchdog_healthy") is not True:
        reasons.append("watchdog_not_healthy")
    if snapshot.get("camera_fresh") is not True:
        reasons.append("camera_not_fresh")
    if snapshot.get("target_valid") is not True:
        reasons.append("target_not_valid")
    if snapshot.get("operator_authorized") is not True:
        reasons.append("operator_not_authorized")
    if abs(base_velocity[0]) > float(config["base_linear_tolerance_mps"]):
        reasons.append("base_linear_motion")
    if abs(base_velocity[1]) > float(config["base_angular_tolerance_radps"]):
        reasons.append("base_angular_motion")
    base_command = _finite_vector(snapshot.get("recent_base_command"), dimension=2, field="snapshot.recent_base_command")
    if abs(base_command[0]) > float(config["base_linear_tolerance_mps"]):
        reasons.append("base_linear_command_nonzero")
    if abs(base_command[1]) > float(config["base_angular_tolerance_radps"]):
        reasons.append("base_angular_command_nonzero")

    limit_margin = []
    for index, (target, current, low, high, allowed_delta) in enumerate(
        zip(
            action,
            measured,
            normalized["joint_lower"],
            normalized["joint_upper"],
            normalized["max_delta_per_step"],
            strict=True,
        )
    ):
        name = ACTION_NAMES[index]
        limit_margin.append(min(target - low, high - target))
        if target < low or target > high:
            reasons.append(f"joint_limit:{name}")
        if abs(target - current) > allowed_delta:
            reasons.append(f"joint_rate:{name}")

    safety_passed = not reasons
    hardware_dispatch_authorized = safety_passed and config.get("hardware_enabled") is True
    return {
        "schema_version": ROLLOUT_SAFETY_SCHEMA_VERSION,
        "decision": "PASS" if safety_passed else "REJECT",
        "safety_passed": safety_passed,
        "hardware_dispatch_authorized": hardware_dispatch_authorized,
        "mock_dispatch_authorized": safety_passed,
        "reason_codes": reasons,
        "proposal_id": proposal.get("proposal_id"),
        "policy_query_id": proposal.get("policy_query_id"),
        "chunk_id": proposal.get("chunk_id"),
        "action_index": proposal.get("action_index"),
        "observation_age_ms": observation_age_ms,
        "proposal_age_ms": proposal_age_ms,
        "feedback_age_ms": feedback_age_ms,
        "minimum_limit_margin": min(limit_margin),
        "action_unchanged": True,
        "approved_action": action if safety_passed else None,
    }


class SafetySupervisor:
    """Application lifecycle and fault latch around the pure safety checks."""

    def __init__(self, config: Mapping[str, Any]):
        validate_safety_config(config)
        self.config = dict(config)
        self.state = "UNCONFIGURED"
        self.policy_reset_generation = 0
        self.active_episode_id: str | None = None
        self.human_approval_id: str | None = None
        self.last_proposal_sequence = -1

    def configure(self, hardware_profile_sha256: str) -> None:
        if self.state not in {"UNCONFIGURED", "INACTIVE"}:
            raise SafetyContractError(f"cannot configure supervisor from {self.state}")
        if hardware_profile_sha256 != self.config["expected_hardware_profile_sha256"]:
            raise SafetyContractError("hardware profile identity mismatch")
        self.state = "INACTIVE"

    def arm(self, *, episode_id: str, human_approval_id: str) -> None:
        if self.state != "INACTIVE":
            raise SafetyContractError(f"cannot arm supervisor from {self.state}")
        if not episode_id or not human_approval_id:
            raise SafetyContractError("episode_id and human_approval_id are required")
        self.active_episode_id = episode_id
        self.human_approval_id = human_approval_id
        self.state = "ARMED"

    def activate(self) -> None:
        if self.state != "ARMED":
            raise SafetyContractError(f"cannot activate supervisor from {self.state}")
        self.state = "ACTIVE"

    def evaluate(self, proposal: Mapping[str, Any], snapshot: Mapping[str, Any]) -> dict[str, Any]:
        if self.state != "ACTIVE":
            return {
                "schema_version": ROLLOUT_SAFETY_SCHEMA_VERSION,
                "decision": "REJECT",
                "safety_passed": False,
                "hardware_dispatch_authorized": False,
                "mock_dispatch_authorized": False,
                "reason_codes": [f"lifecycle_not_active:{self.state}"],
                "approved_action": None,
                "lifecycle_state": self.state,
                "policy_reset_generation": self.policy_reset_generation,
            }
        sequence = proposal.get("proposal_sequence")
        if not isinstance(sequence, int) or sequence <= self.last_proposal_sequence:
            reasons = ["replay_or_out_of_order"]
            decision = None
        elif proposal.get("episode_id") != self.active_episode_id:
            reasons = ["episode_identity_mismatch"]
            decision = None
        elif proposal.get("human_approval_id") != self.human_approval_id:
            reasons = ["human_approval_identity_mismatch"]
            decision = None
        else:
            enriched_snapshot = dict(snapshot)
            enriched_snapshot["expected_policy_reset_generation"] = self.policy_reset_generation
            decision = supervise_action(self.config, proposal, enriched_snapshot)
            reasons = list(decision["reason_codes"])
        if decision is None:
            decision = {
                "schema_version": ROLLOUT_SAFETY_SCHEMA_VERSION,
                "decision": "REJECT",
                "safety_passed": False,
                "hardware_dispatch_authorized": False,
                "mock_dispatch_authorized": False,
                "reason_codes": reasons,
                "approved_action": None,
            }
        if decision["safety_passed"]:
            self.last_proposal_sequence = sequence
        else:
            self.state = "FAULT_LATCHED"
            self.policy_reset_generation += 1
            decision["hardware_dispatch_authorized"] = False
            decision["mock_dispatch_authorized"] = False
            decision["approved_action"] = None
        decision["lifecycle_state"] = self.state
        decision["policy_reset_generation"] = self.policy_reset_generation
        return decision

    def reset_fault(self) -> None:
        if self.state != "FAULT_LATCHED":
            raise SafetyContractError(f"cannot reset fault from {self.state}")
        self.active_episode_id = None
        self.human_approval_id = None
        self.last_proposal_sequence = -1
        self.state = "INACTIVE"

    def finalize(self) -> None:
        self.active_episode_id = None
        self.human_approval_id = None
        self.state = "FINALIZED"


class JDcobotRos2DryRunAdapter:
    """Build ROS 2 command envelopes but never publish or access hardware."""

    def __init__(self, config: Mapping[str, Any]):
        validate_safety_config(config)
        self.config = dict(config)

    def dispatch(self, decision: Mapping[str, Any]) -> dict[str, Any]:
        if decision.get("safety_passed") is not True or decision.get("approved_action") is None:
            return {
                "status": "NOT_DISPATCHED",
                "published": False,
                "reason": "supervisor_rejected",
                "executed_action": None,
            }
        action = list(decision["approved_action"])
        controller = self.config.get("ros2_controller", {})
        left_names = controller.get("left_joint_names")
        right_names = controller.get("right_joint_names")
        if not isinstance(left_names, list) or len(left_names) != 6:
            raise SafetyContractError("ros2_controller.left_joint_names must contain 6 names")
        if not isinstance(right_names, list) or len(right_names) != 6:
            raise SafetyContractError("ros2_controller.right_joint_names must contain 6 names")
        duration = controller.get("trajectory_duration_s")
        if not isinstance(duration, (int, float)) or isinstance(duration, bool) or duration <= 0:
            raise SafetyContractError("ros2 trajectory_duration_s must be positive")
        envelopes = {
            "left": {
                "topic": controller.get("left_command_topic"),
                "message_type": "trajectory_msgs/msg/JointTrajectory",
                "joint_names": left_names,
                "positions": action[:6],
                "time_from_start_s": float(duration),
            },
            "right": {
                "topic": controller.get("right_command_topic"),
                "message_type": "trajectory_msgs/msg/JointTrajectory",
                "joint_names": right_names,
                "positions": action[6:],
                "time_from_start_s": float(duration),
            },
        }
        return {
            "status": "SIMULATED_ONLY",
            "published": False,
            "reason": "dry_run_adapter_has_no_publish_capability",
            "would_publish": envelopes,
            "executed_action": None,
        }


def build_rollout_safety_fixture() -> dict[str, Any]:
    """Return a conservative synthetic-only config; it can never enable hardware."""

    return {
        "schema_version": ROLLOUT_SAFETY_SCHEMA_VERSION,
        "hardware_enabled": False,
        "limit_source": "synthetic_fixture_only",
        "mechanical_calibration_verified": False,
        "e_stop_verified": False,
        "operator_authorization_verified": False,
        "ros2_controller_contract_verified": False,
        "reject_instead_of_clip": True,
        "action_names": list(ACTION_NAMES),
        "joint_lower": [-1.0] * 5 + [0.0] + [-1.0] * 5 + [0.0],
        "joint_upper": [1.0] * 5 + [1.0] + [1.0] * 5 + [1.0],
        "max_delta_per_step": [0.2] * 5 + [0.3] + [0.2] * 5 + [0.3],
        "base_linear_tolerance_mps": 0.0025,
        "base_angular_tolerance_radps": 0.0021,
        "max_observation_age_ms": 100.0,
        "max_feedback_age_ms": 100.0,
        "max_proposal_age_ms": 50.0,
        "expected_hardware_profile_sha256": _sha256_bytes(b"dapier-stage5-synthetic-hardware-profile"),
        "approved_policy_checkpoint_sha256": _sha256_bytes(b"dapier-stage5-synthetic-policy"),
        "ros2_controller": {
            "left_command_topic": None,
            "right_command_topic": None,
            "left_joint_names": [*(f"left_joint_{index}" for index in range(5)), "left_gripper"],
            "right_joint_names": [*(f"right_joint_{index}" for index in range(5)), "right_gripper"],
            "trajectory_duration_s": 0.05,
            "topic_status": "unverified_pending_education_pc_controller_discovery",
        },
    }


def _base_snapshot(now_ns: int) -> dict[str, Any]:
    return {
        "now_monotonic_ns": now_ns,
        "observation_monotonic_ns": now_ns - 20_000_000,
        "feedback_monotonic_ns": now_ns - 15_000_000,
        "measured_action": [0.0] * 5 + [0.5] + [0.0] * 5 + [0.5],
        "base_velocity": [0.0, 0.0],
        "recent_base_command": [0.0, 0.0],
        "e_stop_healthy": True,
        "watchdog_healthy": True,
        "camera_fresh": True,
        "target_valid": True,
        "operator_authorized": True,
    }


def run_rollout_safety_smoke(output_path: str | Path) -> dict[str, Any]:
    """Run deterministic safe/reject mutations and persist an auditable trace."""

    output = Path(output_path).resolve()
    if output.exists():
        raise SafetyContractError(f"refusing to overwrite rollout trace: {output}")
    config = build_rollout_safety_fixture()
    now_ns = 10_000_000_000
    safe_action = [0.05] * 5 + [0.55] + [-0.05] * 5 + [0.45]
    scenarios: list[tuple[str, dict[str, Any], dict[str, Any], SafetySupervisor]] = []
    for name in ("safe", "joint_limit", "stale_observation", "base_moving", "e_stop", "watchdog"):
        supervisor = SafetySupervisor(config)
        supervisor.configure(config["expected_hardware_profile_sha256"])
        supervisor.arm(episode_id="episode_fixture_001", human_approval_id="approval_fixture_001")
        supervisor.activate()
        proposal = {
            "proposal_id": f"proposal_{name}",
            "policy_query_id": "query_001",
            "chunk_id": "chunk_001",
            "action_index": 0,
            "proposal_sequence": 0,
            "episode_id": "episode_fixture_001",
            "human_approval_id": "approval_fixture_001",
            "hardware_profile_sha256": config["expected_hardware_profile_sha256"],
            "policy_checkpoint_sha256": config["approved_policy_checkpoint_sha256"],
            "policy_reset_generation": supervisor.policy_reset_generation,
            "created_monotonic_ns": now_ns - 10_000_000,
            "action": list(safe_action),
        }
        snapshot = _base_snapshot(now_ns)
        if name == "joint_limit":
            proposal["action"][0] = 2.0
        elif name == "stale_observation":
            snapshot["observation_monotonic_ns"] = now_ns - 200_000_000
        elif name == "base_moving":
            snapshot["base_velocity"] = [0.01, 0.0]
        elif name == "e_stop":
            snapshot["e_stop_healthy"] = False
        elif name == "watchdog":
            snapshot["watchdog_healthy"] = False
        scenarios.append((name, proposal, snapshot, supervisor))

    adapter = JDcobotRos2DryRunAdapter(config)
    events = []
    for name, proposal, snapshot, supervisor in scenarios:
        decision = supervisor.evaluate(proposal, snapshot)
        adapter_result = adapter.dispatch(decision)
        events.append(
            {
                "scenario": name,
                "proposal": proposal,
                "snapshot": snapshot,
                "decision": decision,
                "adapter_result": adapter_result,
                "supervisor_state_after": supervisor.state,
            }
        )
    summary = {
        "schema_version": ROLLOUT_TRACE_SCHEMA_VERSION,
        "created_at_utc": _utc_now(),
        "status": "PASS",
        "scope": "dry-run safety contract only; no ROS publish and no motor command",
        "config": config,
        "config_sha256": _sha256_bytes(
            json.dumps(config, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ),
        "scenario_count": len(events),
        "safety_pass_count": sum(event["decision"]["safety_passed"] for event in events),
        "reject_count": sum(not event["decision"]["safety_passed"] for event in events),
        "published_command_count": sum(event["adapter_result"]["published"] for event in events),
        "hardware_dispatch_authorized_count": sum(
            event["decision"]["hardware_dispatch_authorized"] for event in events
        ),
        "events": events,
        "next_live_gates": [
            "physical joint side/order/sign/zero and per-joint limit validation",
            "verified E-stop and watchdog path",
            "education PC ROS 2 controller topic/message discovery",
            "signed operator authorization for low-speed one-joint-at-a-time test",
            "independent collision/workspace validation before learned-policy rollout",
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary
