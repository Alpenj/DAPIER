"""Deterministic selection contract for single-arm and bimanual grasps.

The selector does not run IK or command a robot. Planning adapters provide
measured candidate assessments, and this module rejects unsafe candidates
before applying an explicit, inspectable ranking policy.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence


class GraspMode(str, Enum):
    SINGLE_LEFT = "single_left"
    SINGLE_RIGHT = "single_right"
    BIMANUAL = "bimanual"


@dataclass(frozen=True)
class GraspCandidateAssessment:
    mode: GraspMode
    ik_converged: bool
    ik_residual_m: float
    collision_path_safe: bool
    minimum_clearance_m: float
    required_clearance_m: float
    base_travel_m: float
    tactile_channels: int
    rejection_reason: str = ""

    def validate(self) -> None:
        if not isinstance(self.mode, GraspMode):
            raise ValueError("mode must be a GraspMode")
        finite_nonnegative = (
            self.ik_residual_m,
            self.minimum_clearance_m,
            self.base_travel_m,
        )
        if not all(
            math.isfinite(value) and value >= 0.0
            for value in finite_nonnegative
        ):
            raise ValueError("candidate metrics must be finite and non-negative")
        if (
            not math.isfinite(self.required_clearance_m)
            or self.required_clearance_m <= 0
        ):
            raise ValueError("required_clearance_m must be finite and positive")
        expected_channels = 2 if self.mode == GraspMode.BIMANUAL else 1
        if self.tactile_channels != expected_channels:
            raise ValueError(
                f"{self.mode.value} requires {expected_channels} tactile channel(s)"
            )

    @property
    def feasible(self) -> bool:
        self.validate()
        return (
            self.ik_converged
            and self.collision_path_safe
            and self.minimum_clearance_m >= self.required_clearance_m
            and not self.rejection_reason
        )


@dataclass(frozen=True)
class GraspSelection:
    selected: GraspCandidateAssessment | None
    ranked_feasible_modes: tuple[GraspMode, ...]
    rejected: tuple[tuple[GraspMode, str], ...]


def _rejection_reason(candidate: GraspCandidateAssessment) -> str:
    if candidate.rejection_reason:
        return candidate.rejection_reason
    if not candidate.ik_converged:
        return "ik_not_converged"
    if not candidate.collision_path_safe:
        return "collision_path_unsafe"
    if candidate.minimum_clearance_m < candidate.required_clearance_m:
        return "clearance_below_requirement"
    return "not_feasible"


def select_grasp_candidate(
    candidates: Sequence[GraspCandidateAssessment],
    *,
    prefer_bimanual: bool = True,
    require_bimanual: bool = False,
) -> GraspSelection:
    """Select a safe candidate without allowing preference to bypass safety.

    Bimanual preference is a small tie-break benefit. Clearance margin and
    base travel remain part of the ranking, while every unsafe candidate is
    removed before ranking.
    """

    if not candidates:
        raise ValueError("at least one grasp candidate is required")
    seen: set[GraspMode] = set()
    feasible: list[GraspCandidateAssessment] = []
    rejected: list[tuple[GraspMode, str]] = []
    for candidate in candidates:
        candidate.validate()
        if candidate.mode in seen:
            raise ValueError(f"duplicate grasp mode: {candidate.mode.value}")
        seen.add(candidate.mode)
        if require_bimanual and candidate.mode != GraspMode.BIMANUAL:
            rejected.append((candidate.mode, "task_requires_bimanual"))
        elif candidate.feasible:
            feasible.append(candidate)
        else:
            rejected.append((candidate.mode, _rejection_reason(candidate)))

    def score(
        candidate: GraspCandidateAssessment,
    ) -> tuple[float, float, float, str]:
        clearance_margin = (
            candidate.minimum_clearance_m - candidate.required_clearance_m
        )
        bimanual_bonus = (
            0.025
            if prefer_bimanual and candidate.mode == GraspMode.BIMANUAL
            else 0.0
        )
        return (
            -(clearance_margin + bimanual_bonus),
            candidate.base_travel_m,
            candidate.ik_residual_m,
            candidate.mode.value,
        )

    ranked = sorted(feasible, key=score)
    return GraspSelection(
        selected=ranked[0] if ranked else None,
        ranked_feasible_modes=tuple(candidate.mode for candidate in ranked),
        rejected=tuple(rejected),
    )


__all__ = [
    "GraspCandidateAssessment",
    "GraspMode",
    "GraspSelection",
    "select_grasp_candidate",
]
