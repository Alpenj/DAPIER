from __future__ import annotations

from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from box_shoe_mission import (
    BoxShoeCommand,
    BoxShoeEvent,
    BoxShoeEventType,
    BoxShoeMissionController,
    BoxShoePhase,
)


class BoxShoeMissionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.sequence = 0

    def event(self, kind: BoxShoeEventType, **overrides) -> BoxShoeEvent:
        values = {
            "kind": kind,
            "observation_seq": self.sequence,
            "observation_age_ms": 10.0,
            "base_stationary": True,
        }
        values.update(overrides)
        self.sequence += 1
        return BoxShoeEvent(**values)

    def right_hold(self) -> dict[str, object]:
        return {
            "lid_open_angle_rad": 1.2,
            "lid_held_by_right": True,
            "right_tactile_available": True,
            "right_tactile_contact": True,
        }

    def advance_to_extraction(self, controller: BoxShoeMissionController) -> None:
        controller.dispatch(self.event(BoxShoeEventType.START))
        controller.dispatch(
            self.event(
                BoxShoeEventType.NAVIGATION_RESULT,
                location="B",
            )
        )
        controller.dispatch(
            self.event(BoxShoeEventType.BOX_POSE_RESULT, pose_confidence=0.9)
        )
        controller.dispatch(
            self.event(
                BoxShoeEventType.BOX_OPEN_RESULT,
                right_arm_moved=True,
                **self.right_hold(),
            )
        )
        controller.dispatch(
            self.event(BoxShoeEventType.LID_HOLD_RESULT, **self.right_hold())
        )
        controller.dispatch(
            self.event(
                BoxShoeEventType.SHOE_POSE_RESULT,
                pose_confidence=0.9,
                **self.right_hold(),
            )
        )

    def run_happy_path(self) -> BoxShoeMissionController:
        controller = BoxShoeMissionController()
        self.advance_to_extraction(controller)
        controller.dispatch(
            self.event(
                BoxShoeEventType.EXTRACTION_RESULT,
                left_arm_moved=True,
                **self.right_hold(),
            )
        )
        controller.dispatch(
            self.event(
                BoxShoeEventType.SHOE_GRASP_RESULT,
                shoe_lifted=True,
                shoe_held_by_left=True,
                left_tactile_available=True,
                left_tactile_contact=True,
                **self.right_hold(),
            )
        )
        controller.dispatch(
            self.event(
                BoxShoeEventType.SHOE_CLEAR_RESULT,
                shoe_clear_of_box=True,
                shoe_held_by_left=True,
                **self.right_hold(),
            )
        )
        controller.dispatch(
            self.event(
                BoxShoeEventType.TRANSPORT_POSE_RESULT,
                left_arm_moved=True,
                right_arm_moved=True,
                shoe_clear_of_box=True,
                shoe_held_by_left=True,
                left_tactile_available=True,
                left_tactile_contact=True,
                lid_safe=True,
            )
        )
        controller.dispatch(
            self.event(
                BoxShoeEventType.NAVIGATION_RESULT,
                location="A",
                shoe_held_by_left=True,
                left_tactile_available=True,
                left_tactile_contact=True,
            )
        )
        controller.dispatch(
            self.event(BoxShoeEventType.PLACE_RESULT, left_arm_moved=True)
        )
        controller.dispatch(
            self.event(
                BoxShoeEventType.RELEASE_RESULT,
                object_released=True,
                at_start_zone=True,
            )
        )
        return controller

    def test_happy_path_requires_both_arm_roles(self) -> None:
        controller = self.run_happy_path()
        self.assertEqual(controller.phase, BoxShoePhase.COMPLETED)
        final = controller.history[-1]
        self.assertTrue(final.left_arm_participated)
        self.assertTrue(final.right_arm_participated)
        self.assertEqual(final.commands, (BoxShoeCommand.MISSION_COMPLETE,))
        phases = [transition.phase for transition in controller.history]
        self.assertIn(BoxShoePhase.OPENING_BOX_RIGHT, phases)
        self.assertIn(BoxShoePhase.EXTRACTING_SHOE_LEFT, phases)
        self.assertIn(BoxShoePhase.MOVING_BOTH_TO_TRANSPORT, phases)
        self.assertTrue(
            all(not transition.hardware_execution for transition in controller.history)
        )

    def test_right_fsr_contact_is_required_to_open_box(self) -> None:
        controller = BoxShoeMissionController()
        controller.dispatch(self.event(BoxShoeEventType.START))
        controller.dispatch(
            self.event(BoxShoeEventType.NAVIGATION_RESULT, location="B")
        )
        controller.dispatch(
            self.event(BoxShoeEventType.BOX_POSE_RESULT, pose_confidence=0.9)
        )
        transition = controller.dispatch(
            self.event(
                BoxShoeEventType.BOX_OPEN_RESULT,
                right_arm_moved=True,
                lid_open_angle_rad=1.2,
                right_tactile_available=True,
                right_tactile_contact=False,
            )
        )
        self.assertEqual(transition.phase, BoxShoePhase.SAFE_STOPPED)
        self.assertIn("right-arm lid opening", transition.reason)

    def test_lid_hold_loss_blocks_left_extraction(self) -> None:
        controller = BoxShoeMissionController()
        self.advance_to_extraction(controller)
        hold = self.right_hold()
        hold["right_tactile_contact"] = False
        transition = controller.dispatch(
            self.event(
                BoxShoeEventType.EXTRACTION_RESULT,
                left_arm_moved=True,
                **hold,
            )
        )
        self.assertEqual(transition.phase, BoxShoePhase.SAFE_STOPPED)
        self.assertIn("lid hold", transition.reason)

    def test_left_arm_motion_is_required_for_extraction(self) -> None:
        controller = BoxShoeMissionController()
        self.advance_to_extraction(controller)
        transition = controller.dispatch(
            self.event(
                BoxShoeEventType.EXTRACTION_RESULT,
                left_arm_moved=False,
                **self.right_hold(),
            )
        )
        self.assertEqual(transition.phase, BoxShoePhase.SAFE_STOPPED)
        self.assertFalse(transition.left_arm_participated)

    def test_manipulation_rejects_moving_base(self) -> None:
        controller = BoxShoeMissionController()
        controller.dispatch(self.event(BoxShoeEventType.START))
        controller.dispatch(
            self.event(BoxShoeEventType.NAVIGATION_RESULT, location="B")
        )
        transition = controller.dispatch(
            self.event(
                BoxShoeEventType.BOX_POSE_RESULT,
                pose_confidence=0.9,
                base_stationary=False,
            )
        )
        self.assertEqual(transition.phase, BoxShoePhase.SAFE_STOPPED)
        self.assertIn("stationary", transition.reason)


if __name__ == "__main__":
    unittest.main()
