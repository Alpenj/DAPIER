"""Deterministic kinematic baseline for one-card bimanual transfer.

This module intentionally stops below a physics-engine or hardware claim. It
models two Cartesian tool points, binary vacuum attachment, bounded motion and
table/target geometry so the CardBench role split can be exercised before
motors, cameras or a vacuum adapter are connected.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import random
from typing import Any


CARD_SIM_RECEIPT_VERSION = "dapier.card-sim-receipt.v1"


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def distance_xy(self, other: "Vec3") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def distance(self, other: "Vec3") -> float:
        return math.sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )


@dataclass(frozen=True)
class CardSimConfig:
    """Geometry and safety bounds for the small kinematic experiment."""

    control_frequency_hz: int = 20
    max_delta_m: float = 0.02
    table_z_m: float = 0.0
    tool_clearance_m: float = 0.006
    card_thickness_m: float = 0.0003
    suction_offset_m: float = 0.012
    pickup_xy_tolerance_m: float = 0.014
    pickup_z_tolerance_m: float = 0.008
    target_radius_m: float = 0.025
    release_height_m: float = 0.020
    randomization_m: float = 0.015
    deck_nominal_x_m: float = -0.12
    deck_nominal_y_m: float = 0.02
    target_nominal_x_m: float = 0.18
    target_nominal_y_m: float = 0.02
    max_steps: int = 240

    def __post_init__(self) -> None:
        if self.control_frequency_hz <= 0:
            raise ValueError("control_frequency_hz must be positive")
        if self.max_delta_m <= 0:
            raise ValueError("max_delta_m must be positive")
        if self.tool_clearance_m <= 0:
            raise ValueError("tool_clearance_m must be positive")
        if self.target_radius_m <= 0:
            raise ValueError("target_radius_m must be positive")
        if self.max_steps <= 0:
            raise ValueError("max_steps must be positive")


@dataclass(frozen=True)
class CardSimAction:
    """Bounded Cartesian increments and vacuum commands for both arms."""

    left_delta: Vec3 = Vec3(0.0, 0.0, 0.0)
    right_delta: Vec3 = Vec3(0.0, 0.0, 0.0)
    left_vacuum: float = 0.0
    right_vacuum: float = 0.0


@dataclass(frozen=True)
class CardSimState:
    step: int
    left_tool: Vec3
    right_tool: Vec3
    card: Vec3
    target: Vec3
    left_vacuum: float
    right_vacuum: float
    deck_stabilized: bool
    card_attached: bool
    card_placed: bool
    done: bool
    failure_reason: str | None


class OneCardKinematicEnv:
    """Small deterministic environment for the first casino manipulation unit."""

    def __init__(self, config: CardSimConfig | None = None) -> None:
        self.config = config or CardSimConfig()
        self._state: CardSimState | None = None

    @property
    def state(self) -> CardSimState:
        if self._state is None:
            raise RuntimeError("reset must be called before reading state")
        return self._state

    def reset(self, seed: int = 0) -> CardSimState:
        rng = random.Random(seed)
        jitter = self.config.randomization_m
        card = Vec3(
            self.config.deck_nominal_x_m + rng.uniform(-jitter, jitter),
            self.config.deck_nominal_y_m + rng.uniform(-jitter, jitter),
            self.config.table_z_m + self.config.card_thickness_m / 2,
        )
        target = Vec3(
            self.config.target_nominal_x_m + rng.uniform(-jitter, jitter),
            self.config.target_nominal_y_m + rng.uniform(-jitter, jitter),
            self.config.table_z_m + self.config.card_thickness_m / 2,
        )
        self._state = CardSimState(
            step=0,
            left_tool=Vec3(card.x, card.y, 0.14),
            right_tool=Vec3(-0.02, -0.16, 0.14),
            card=card,
            target=target,
            left_vacuum=0.0,
            right_vacuum=0.0,
            deck_stabilized=False,
            card_attached=False,
            card_placed=False,
            done=False,
            failure_reason=None,
        )
        return self.state

    def step(self, action: CardSimAction) -> CardSimState:
        state = self.state
        if state.done:
            raise RuntimeError("episode is already done")
        self._validate_action(action)

        left_tool = self._advance(state.left_tool, action.left_delta)
        right_tool = self._advance(state.right_tool, action.right_delta)
        left_vacuum = self._vacuum(action.left_vacuum, "left_vacuum")
        right_vacuum = self._vacuum(action.right_vacuum, "right_vacuum")
        failure_reason: str | None = None

        deck_stabilized = (
            left_vacuum >= 0.5
            and left_tool.distance_xy(state.card) <= self.config.pickup_xy_tolerance_m
            and abs(
                left_tool.z
                - (self.config.table_z_m + self.config.suction_offset_m)
            )
            <= self.config.pickup_z_tolerance_m
        )

        card_attached = state.card_attached
        card = state.card
        if not card_attached and right_vacuum >= 0.5:
            pickup_pose = Vec3(
                card.x,
                card.y,
                card.z + self.config.suction_offset_m,
            )
            if deck_stabilized and right_tool.distance(pickup_pose) <= max(
                self.config.pickup_xy_tolerance_m,
                self.config.pickup_z_tolerance_m,
            ):
                card_attached = True

        if card_attached and right_vacuum >= 0.5:
            card = Vec3(
                right_tool.x,
                right_tool.y,
                max(
                    self.config.table_z_m + self.config.card_thickness_m / 2,
                    right_tool.z - self.config.suction_offset_m,
                ),
            )

        card_placed = state.card_placed
        if card_attached and right_vacuum < 0.5:
            card_attached = False
            target_error = card.distance_xy(state.target)
            if (
                target_error <= self.config.target_radius_m
                and right_tool.z
                <= self.config.release_height_m + self.config.suction_offset_m
            ):
                card = Vec3(
                    state.target.x,
                    state.target.y,
                    self.config.table_z_m + self.config.card_thickness_m / 2,
                )
                card_placed = True
            else:
                failure_reason = (
                    "released_outside_target"
                    if target_error > self.config.target_radius_m
                    else "released_too_high"
                )

        step = state.step + 1
        if left_tool.z < self.config.tool_clearance_m:
            failure_reason = failure_reason or "left_tool_table_contact"
        if right_tool.z < self.config.tool_clearance_m:
            failure_reason = failure_reason or "right_tool_table_contact"
        if step >= self.config.max_steps and not card_placed:
            failure_reason = failure_reason or "timeout"

        done = failure_reason is not None or (card_placed and left_vacuum < 0.5)
        self._state = CardSimState(
            step=step,
            left_tool=left_tool,
            right_tool=right_tool,
            card=card,
            target=state.target,
            left_vacuum=left_vacuum,
            right_vacuum=right_vacuum,
            deck_stabilized=deck_stabilized,
            card_attached=card_attached,
            card_placed=card_placed,
            done=done,
            failure_reason=failure_reason,
        )
        return self.state

    @staticmethod
    def _advance(position: Vec3, delta: Vec3) -> Vec3:
        return Vec3(
            position.x + delta.x,
            position.y + delta.y,
            position.z + delta.z,
        )

    def _validate_action(self, action: CardSimAction) -> None:
        limit = self.config.max_delta_m
        for name, delta in (
            ("left_delta", action.left_delta),
            ("right_delta", action.right_delta),
        ):
            if max(abs(delta.x), abs(delta.y), abs(delta.z)) > limit + 1e-12:
                raise ValueError(f"{name} exceeds max_delta_m={limit}")

    @staticmethod
    def _vacuum(value: float, name: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError(f"{name} must be numeric")
        if not 0.0 <= float(value) <= 1.0:
            raise ValueError(f"{name} must be in [0.0, 1.0]")
        return float(value)


class OneCardScriptedPolicy:
    """State-based baseline that respects the bounded action interface."""

    def __init__(self, config: CardSimConfig | None = None) -> None:
        self.config = config or CardSimConfig()

    def action(self, state: CardSimState) -> CardSimAction:
        card_pick = Vec3(
            state.card.x,
            state.card.y,
            self.config.table_z_m + self.config.suction_offset_m,
        )
        left_hold = card_pick
        transfer_high = Vec3(state.target.x, state.target.y, 0.12)
        release_pose = Vec3(
            state.target.x,
            state.target.y,
            self.config.release_height_m + self.config.suction_offset_m,
        )
        right_high = Vec3(state.card.x, state.card.y, 0.12)

        if not state.deck_stabilized:
            return CardSimAction(
                left_delta=self._toward(state.left_tool, left_hold),
                right_delta=self._toward(state.right_tool, right_high),
                left_vacuum=1.0,
                right_vacuum=0.0,
            )

        if not state.card_attached and not state.card_placed:
            return CardSimAction(
                left_delta=self._toward(state.left_tool, left_hold),
                right_delta=self._toward(state.right_tool, card_pick),
                left_vacuum=1.0,
                right_vacuum=1.0,
            )

        if state.card_attached:
            target = (
                transfer_high
                if state.right_tool.distance_xy(transfer_high)
                > self.config.target_radius_m / 2
                else release_pose
            )
            at_release = state.right_tool.distance(release_pose) <= 1e-9
            return CardSimAction(
                left_delta=self._toward(state.left_tool, left_hold),
                right_delta=self._toward(state.right_tool, target),
                left_vacuum=1.0,
                right_vacuum=0.0 if at_release else 1.0,
            )

        return CardSimAction(
            left_delta=self._toward(
                state.left_tool,
                Vec3(left_hold.x, left_hold.y, 0.12),
            ),
            right_delta=self._toward(
                state.right_tool,
                Vec3(release_pose.x, release_pose.y, 0.12),
            ),
            left_vacuum=0.0,
            right_vacuum=0.0,
        )

    def _toward(self, current: Vec3, target: Vec3) -> Vec3:
        limit = self.config.max_delta_m
        return Vec3(
            _clip(target.x - current.x, -limit, limit),
            _clip(target.y - current.y, -limit, limit),
            _clip(target.z - current.z, -limit, limit),
        )


def run_one_card_episode(
    seed: int,
    config: CardSimConfig | None = None,
) -> dict[str, Any]:
    """Run one baseline episode and return evidence without physical claims."""
    cfg = config or CardSimConfig()
    env = OneCardKinematicEnv(cfg)
    policy = OneCardScriptedPolicy(cfg)
    initial = env.reset(seed)
    maximum_delta = 0.0
    attachment_step: int | None = None

    while not env.state.done:
        action = policy.action(env.state)
        maximum_delta = max(
            maximum_delta,
            abs(action.left_delta.x),
            abs(action.left_delta.y),
            abs(action.left_delta.z),
            abs(action.right_delta.x),
            abs(action.right_delta.y),
            abs(action.right_delta.z),
        )
        previous_attached = env.state.card_attached
        state = env.step(action)
        if state.card_attached and not previous_attached:
            attachment_step = state.step

    final = env.state
    return {
        "seed": seed,
        "success": final.card_placed and final.failure_reason is None,
        "steps": final.step,
        "attachment_step": attachment_step,
        "failure_reason": final.failure_reason,
        "max_abs_action_delta_m": maximum_delta,
        "final_card_target_error_m": final.card.distance_xy(final.target),
        "initial_card": asdict(initial.card),
        "target": asdict(initial.target),
    }


def run_one_card_baseline(
    episodes: int = 20,
    seed: int = 0,
    config: CardSimConfig | None = None,
) -> dict[str, Any]:
    if isinstance(episodes, bool) or not isinstance(episodes, int):
        raise TypeError("episodes must be an integer")
    if episodes <= 0:
        raise ValueError("episodes must be positive")

    cfg = config or CardSimConfig()
    results = [run_one_card_episode(seed + index, cfg) for index in range(episodes)]
    successes = sum(result["success"] for result in results)
    return {
        "schema_version": CARD_SIM_RECEIPT_VERSION,
        "task": "bimanual_one_card_kinematic_transfer",
        "scope": "deterministic kinematic simulation",
        "config": asdict(cfg),
        "episodes": results,
        "summary": {
            "passed": successes,
            "total": episodes,
            "success_rate": successes / episodes,
            "mean_steps": sum(result["steps"] for result in results) / episodes,
            "max_abs_action_delta_m": max(
                result["max_abs_action_delta_m"] for result in results
            ),
        },
        "claims": {
            "task_level_simulation_executed": True,
            "physical_card_manipulation": False,
            "camera_recognition": False,
            "vacuum_hardware_verified": False,
            "learned_policy_evaluated": False,
        },
    }


def _clip(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)
