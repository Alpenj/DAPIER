from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_modules.grasp_planner import (
    GraspCandidateAssessment,
    GraspMode,
    select_grasp_candidate,
)


def candidate(
    mode: GraspMode,
    *,
    safe: bool = True,
    clearance_m: float = 0.020,
    travel_m: float = 0.40,
) -> GraspCandidateAssessment:
    return GraspCandidateAssessment(
        mode=mode,
        ik_converged=True,
        ik_residual_m=0.0005,
        collision_path_safe=safe,
        minimum_clearance_m=clearance_m,
        required_clearance_m=0.010,
        base_travel_m=travel_m,
        tactile_channels=2 if mode == GraspMode.BIMANUAL else 1,
    )


class GraspPlannerTest(unittest.TestCase):
    def test_bimanual_requirement_rejects_safe_single_arm(self) -> None:
        selection = select_grasp_candidate(
            (
                candidate(GraspMode.SINGLE_LEFT),
                candidate(
                    GraspMode.BIMANUAL,
                    safe=False,
                    clearance_m=0.0079,
                    travel_m=0.315,
                ),
            ),
            require_bimanual=True,
        )
        self.assertIsNone(selection.selected)
        self.assertEqual(
            selection.rejected,
            (
                (GraspMode.SINGLE_LEFT, "task_requires_bimanual"),
                (GraspMode.BIMANUAL, "collision_path_unsafe"),
            ),
        )

    def test_safe_bimanual_is_selected_when_required(self) -> None:
        selection = select_grasp_candidate(
            (
                candidate(GraspMode.SINGLE_LEFT, clearance_m=0.060),
                candidate(GraspMode.BIMANUAL, clearance_m=0.012),
            ),
            require_bimanual=True,
        )
        self.assertEqual(selection.selected.mode, GraspMode.BIMANUAL)

    def test_clearance_margin_selects_safer_mode_when_not_required(self) -> None:
        selection = select_grasp_candidate(
            (
                candidate(GraspMode.SINGLE_RIGHT, clearance_m=0.060),
                candidate(GraspMode.BIMANUAL, clearance_m=0.012),
            )
        )
        self.assertEqual(selection.selected.mode, GraspMode.SINGLE_RIGHT)

    def test_tactile_channel_count_matches_mode(self) -> None:
        invalid = GraspCandidateAssessment(
            mode=GraspMode.BIMANUAL,
            ik_converged=True,
            ik_residual_m=0.0,
            collision_path_safe=True,
            minimum_clearance_m=0.02,
            required_clearance_m=0.01,
            base_travel_m=0.2,
            tactile_channels=1,
        )
        with self.assertRaisesRegex(ValueError, "requires 2"):
            invalid.validate()

    def test_candidate_flags_and_channel_count_require_exact_types(self) -> None:
        valid = candidate(GraspMode.SINGLE_LEFT)
        valid.validate()
        for field, value in (
            ("ik_converged", 1),
            ("collision_path_safe", 1),
            ("tactile_channels", True),
        ):
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    replace(valid, **{field: value}).validate()


if __name__ == "__main__":
    unittest.main()
