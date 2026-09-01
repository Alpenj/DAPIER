from __future__ import annotations

from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_core import (
    MissionController,
    MissionEvent,
    MissionEventType,
    MissionLocation,
    MissionPhase,
)


class TactileMissionIntegrationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.sequence = 0

    def event(self, kind: MissionEventType, **overrides) -> MissionEvent:
        values = {
            "kind": kind,
            "observation_seq": self.sequence,
            "observation_age_ms": 10.0,
            "base_stationary": True,
        }
        values.update(overrides)
        self.sequence += 1
        return MissionEvent(**values)

    def advance_to_grasp(self, controller: MissionController) -> None:
        controller.dispatch(self.event(MissionEventType.START))
        controller.dispatch(
            self.event(
                MissionEventType.NAVIGATION_RESULT,
                location=MissionLocation.B,
            )
        )
        controller.dispatch(
            self.event(MissionEventType.POSE_RESULT, pose_confidence=0.9)
        )
        controller.dispatch(self.event(MissionEventType.PICK_RESULT))

    def test_available_fsr_contact_is_required_for_grasp(self) -> None:
        controller = MissionController()
        self.advance_to_grasp(controller)
        transition = controller.dispatch(
            self.event(
                MissionEventType.GRASP_RESULT,
                object_lifted=True,
                gripper_holding=True,
                carry_pose_clear=True,
                tactile_available=True,
                tactile_contact=False,
                recoverable=True,
                failure_code="tactile_contact_missing",
            )
        )
        self.assertEqual(transition.phase, MissionPhase.LOCALIZING_SHOE)
        self.assertFalse(controller.carrying)

    def test_fsr_contact_complements_vision_and_gripper_hold(self) -> None:
        controller = MissionController()
        self.advance_to_grasp(controller)
        transition = controller.dispatch(
            self.event(
                MissionEventType.GRASP_RESULT,
                object_lifted=True,
                gripper_holding=True,
                carry_pose_clear=True,
                tactile_available=True,
                tactile_contact=True,
            )
        )
        self.assertEqual(transition.phase, MissionPhase.NAVIGATING_TO_A)
        self.assertTrue(controller.carrying)

    def test_inferred_slip_during_return_latches_safe_stop(self) -> None:
        controller = MissionController()
        self.advance_to_grasp(controller)
        controller.dispatch(
            self.event(
                MissionEventType.GRASP_RESULT,
                object_lifted=True,
                gripper_holding=True,
                carry_pose_clear=True,
                tactile_available=True,
                tactile_contact=True,
            )
        )
        transition = controller.dispatch(
            self.event(
                MissionEventType.NAVIGATION_RESULT,
                location=MissionLocation.A,
                tactile_available=True,
                tactile_contact=True,
                tactile_slip=True,
            )
        )
        self.assertEqual(transition.phase, MissionPhase.SAFE_STOPPED)
        self.assertIn("transport hold", transition.reason)

    def test_tactile_measurement_without_available_sensor_is_invalid(self) -> None:
        with self.assertRaisesRegex(ValueError, "tactile_available"):
            self.event(
                MissionEventType.START,
                tactile_available=False,
                tactile_contact=True,
            ).validate()


if __name__ == "__main__":
    unittest.main()
