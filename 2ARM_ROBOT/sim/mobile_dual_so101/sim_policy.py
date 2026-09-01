#!/usr/bin/env python3
"""ACT-compatible action boundary for the dual SO-101 MuJoCo model.

The policy contract is simulation-only and contains 12 values in canonical
left-arm/left-gripper/right-arm/right-gripper order. Arm values are radians;
gripper values are normalized to [0, 1]. No hardware API is imported here.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Mapping, Protocol, Sequence

import mujoco

from collision_guard import check_bimanual_path
from mobile_dual_so101 import ACTION_NAMES


POLICY_ACTION_DIM = len(ACTION_NAMES)
GRIPPER_ACTION_INDICES = (5, 11)
EXECUTION_MODES = ("receding_horizon", "action_queue")
MOBILE_SKILL_PHASES = ("manipulation", "carry_ready", "navigation", "place", "safe_stopped")


def _finite_vector(values: Sequence[float], *, label: str) -> tuple[float, ...]:
    if len(values) != POLICY_ACTION_DIM:
        raise ValueError(
            f"{label} expected {POLICY_ACTION_DIM} values, received {len(values)}"
        )
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


def observation_to_policy_state(observation: Mapping[str, object]) -> tuple[float, ...]:
    """Flatten measured follower state without simulator object ground truth."""

    robot = observation.get("robot")
    if not isinstance(robot, Mapping):
        raise ValueError("observation.robot must be an object")
    try:
        values = (
            *robot["left_arm_rad"],
            float(robot["left_gripper_normalized"]),
            *robot["right_arm_rad"],
            float(robot["right_gripper_normalized"]),
        )
    except (KeyError, TypeError) as error:
        raise ValueError("observation.robot is missing the dual-arm policy state") from error
    return _finite_vector(values, label="policy state")


def actuator_targets_to_policy_action(
    model: mujoco.MjModel,
    actuator_targets: Sequence[float],
) -> tuple[float, ...]:
    """Convert MuJoCo actuator radians to the dataset policy unit contract."""

    targets = list(_finite_vector(actuator_targets, label="actuator targets"))
    for index, target in enumerate(targets):
        lower, upper = (float(value) for value in model.actuator_ctrlrange[index])
        if upper <= lower:
            raise ValueError(f"invalid actuator range at index {index}")
        if not lower <= target <= upper:
            raise ValueError(
                f"actuator target {ACTION_NAMES[index]}={target} is outside [{lower}, {upper}]"
            )
        if index in GRIPPER_ACTION_INDICES:
            targets[index] = (target - lower) / (upper - lower)
    return _finite_vector(targets, label="policy action")


def policy_action_to_actuator_targets(
    model: mujoco.MjModel,
    policy_action: Sequence[float],
) -> tuple[float, ...]:
    """Convert one bounded policy proposal into MuJoCo actuator targets."""

    action = list(_finite_vector(policy_action, label="policy action"))
    for index, value in enumerate(action):
        lower, upper = (float(bound) for bound in model.actuator_ctrlrange[index])
        if index in GRIPPER_ACTION_INDICES:
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"normalized gripper action {index} is outside [0, 1]")
            action[index] = lower + value * (upper - lower)
        elif not lower <= value <= upper:
            raise ValueError(
                f"policy action {ACTION_NAMES[index]}={value} is outside [{lower}, {upper}]"
            )
    return _finite_vector(action, label="actuator targets")


class ChunkPolicy(Protocol):
    def reset(self) -> None: ...

    def predict_chunk(
        self, observation: Mapping[str, object]
    ) -> Sequence[Sequence[float]]: ...


@dataclass
class HoldChunkPolicy:
    """Deterministic contract fixture used until a checkpoint is supplied."""

    action: tuple[float, ...]
    chunk_size: int = 4

    def __post_init__(self) -> None:
        self.action = _finite_vector(self.action, label="hold action")
        if self.chunk_size <= 0:
            raise ValueError("chunk_size must be positive")

    def reset(self) -> None:
        return None

    def predict_chunk(
        self, observation: Mapping[str, object]
    ) -> tuple[tuple[float, ...], ...]:
        observation_to_policy_state(observation)
        return tuple(self.action for _ in range(self.chunk_size))


class ActionChunkExecutor:
    """Execute a bounded chunk prefix and discard it on every reset."""

    def __init__(
        self,
        policy: ChunkPolicy,
        *,
        mode: str = "receding_horizon",
        n_action_steps: int = 1,
    ) -> None:
        if mode not in EXECUTION_MODES:
            raise ValueError(f"mode must be one of {EXECUTION_MODES}")
        if n_action_steps <= 0:
            raise ValueError("n_action_steps must be positive")
        self.policy = policy
        self.mode = mode
        self.n_action_steps = n_action_steps
        self._pending: deque[tuple[float, ...]] = deque()
        self.policy_queries = 0
        self.executed_actions = 0

    def reset(self) -> None:
        self._pending.clear()
        self.policy.reset()

    def _query(self, observation: Mapping[str, object]) -> None:
        chunk = tuple(
            _finite_vector(action, label="predicted action")
            for action in self.policy.predict_chunk(observation)
        )
        if not chunk:
            raise ValueError("policy returned an empty action chunk")
        prefix = 1 if self.mode == "receding_horizon" else self.n_action_steps
        if len(chunk) < prefix:
            raise ValueError("policy chunk is shorter than the executable prefix")
        self._pending.extend(chunk[:prefix])
        self.policy_queries += 1

    def next_action(self, observation: Mapping[str, object]) -> tuple[float, ...]:
        if self.mode == "receding_horizon":
            self._pending.clear()
        if not self._pending:
            self._query(observation)
        self.executed_actions += 1
        return self._pending.popleft()


class MobileSkillGate:
    """Fail-closed arm-policy boundary around stationary-base navigation."""

    def __init__(self) -> None:
        self.phase = "manipulation"
        self.grasp_verified = False
        self.transport_hold_action: tuple[float, ...] | None = None
        self.transport_hold_verified = False
        self.stop_reason: str | None = None
        self._transport_model: mujoco.MjModel | None = None
        self._transport_hold_actuator: tuple[float, ...] | None = None

    @property
    def arm_policy_allowed(self) -> bool:
        return self.phase in {"manipulation", "place"}

    def verify_grasp(self, *, object_lifted: bool, gripper_holding: bool) -> None:
        if self.phase != "manipulation":
            raise RuntimeError("grasp can only be verified during manipulation")
        if not object_lifted or not gripper_holding:
            raise ValueError("grasp verification requires lift and gripper hold")
        self.grasp_verified = True
        self.phase = "carry_ready"

    def begin_navigation(
        self,
        executor: ActionChunkExecutor,
        *,
        model: mujoco.MjModel,
        current_actuator_targets: Sequence[float],
        transport_hold_action: Sequence[float],
        carry_pose_clear: bool,
    ) -> None:
        if self.phase != "carry_ready" or not self.grasp_verified:
            raise RuntimeError("navigation requires a verified carry-ready grasp")
        if not carry_pose_clear:
            raise ValueError("navigation requires both arms inside the carry envelope")
        hold_policy = _finite_vector(
            transport_hold_action,
            label="transport hold action",
        )
        hold_actuator = policy_action_to_actuator_targets(
            model,
            hold_policy,
        )
        current = _finite_vector(
            current_actuator_targets,
            label="current actuator targets",
        )
        actuator_targets_to_policy_action(model, current)
        assessment = check_bimanual_path(
            model,
            current,
            hold_actuator,
        )
        if not assessment.safe:
            raise ValueError(f"unsafe transport hold: {assessment.reason}")
        self.transport_hold_action = hold_policy
        self.transport_hold_verified = False
        self._transport_model = model
        self._transport_hold_actuator = hold_actuator
        executor.reset()
        self.phase = "navigation"

    def monitor_transport_hold(
        self,
        observation: Mapping[str, object],
        *,
        object_lifted: bool,
        gripper_holding: bool,
        max_hold_deviation: float = 0.05,
    ) -> tuple[float, ...]:
        """Return the enforced hold, or latch a safe stop on carry failure."""

        if self.phase != "navigation":
            raise RuntimeError("transport hold monitoring requires navigation")
        if not math.isfinite(max_hold_deviation) or max_hold_deviation <= 0:
            raise ValueError("max_hold_deviation must be finite and positive")
        if not object_lifted or not gripper_holding:
            self._stop_transport("transport grasp lost")
        if (
            self.transport_hold_action is None
            or self._transport_model is None
            or self._transport_hold_actuator is None
        ):
            self._stop_transport("transport hold is not initialized")
        state = observation_to_policy_state(observation)
        deviation = max(
            abs(current - target)
            for current, target in zip(state, self.transport_hold_action, strict=True)
        )
        if deviation > max_hold_deviation:
            self._stop_transport(f"transport hold deviation {deviation:.6f}")
        current_actuator = policy_action_to_actuator_targets(self._transport_model, state)
        assessment = check_bimanual_path(
            self._transport_model,
            current_actuator,
            self._transport_hold_actuator,
        )
        if not assessment.safe:
            self._stop_transport(f"unsafe transport recovery: {assessment.reason}")
        self.transport_hold_verified = True
        return self.transport_hold_action

    def _stop_transport(self, reason: str) -> None:
        self.phase = "safe_stopped"
        self.stop_reason = reason
        self.transport_hold_verified = False
        raise RuntimeError(reason)

    def begin_place(
        self,
        executor: ActionChunkExecutor,
        *,
        base_linear_velocity_mps: float,
        base_angular_velocity_radps: float,
        observation_age_ms: float,
        max_observation_age_ms: float = 100.0,
    ) -> None:
        if self.phase != "navigation":
            raise RuntimeError("place requires the navigation phase")
        if not self.transport_hold_verified:
            raise RuntimeError("place requires a verified transport hold")
        values = (
            base_linear_velocity_mps,
            base_angular_velocity_radps,
            observation_age_ms,
            max_observation_age_ms,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("navigation transition values must be finite")
        if abs(base_linear_velocity_mps) > 0.0025:
            raise ValueError("base linear velocity is not settled")
        if abs(base_angular_velocity_radps) > 0.0021:
            raise ValueError("base angular velocity is not settled")
        if observation_age_ms < 0 or observation_age_ms > max_observation_age_ms:
            raise ValueError("observation is stale at the place transition")
        executor.reset()
        self.phase = "place"


__all__ = [
    "ActionChunkExecutor",
    "ChunkPolicy",
    "EXECUTION_MODES",
    "GRIPPER_ACTION_INDICES",
    "HoldChunkPolicy",
    "MOBILE_SKILL_PHASES",
    "MobileSkillGate",
    "POLICY_ACTION_DIM",
    "actuator_targets_to_policy_action",
    "observation_to_policy_state",
    "policy_action_to_actuator_targets",
]
