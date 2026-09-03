from __future__ import annotations

from contextlib import redirect_stdout
from io import StringIO
import math
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from box_shoe_mission import (
    BoxShoeCommand,
    BoxShoeEvent,
    BoxShoeEventType,
    BoxShoeMissionController,
    BoxShoePhase,
)
from box_shoe_physics_demo import (
    BoxShoePhysicsDemo,
    _classify_unintended_contact,
    _continuous_lift_evidence_ok,
    _most_opposing_normal_dot,
    _shoe_pose_in_left_gripper,
)
from box_shoe_scene import (
    SHOE_JOINT_NAME,
    BoxShoeSceneConfig,
    box_shoe_scene_contract,
    build_box_shoe_scene_model,
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
        self.assertEqual(transition.phase, BoxShoePhase.SAFE_STOP_REQUESTED)
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
        self.assertEqual(transition.phase, BoxShoePhase.SAFE_STOP_REQUESTED)
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
        self.assertEqual(transition.phase, BoxShoePhase.SAFE_STOP_REQUESTED)
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
        self.assertEqual(transition.phase, BoxShoePhase.SAFE_STOP_REQUESTED)
        self.assertIn("stationary", transition.reason)

    def test_safe_stop_requires_actuator_and_base_confirmation(self) -> None:
        controller = BoxShoeMissionController()
        requested = controller.dispatch(
            self.event(
                BoxShoeEventType.FAULT,
                success=False,
                failure_code="watchdog",
            )
        )
        self.assertEqual(requested.phase, BoxShoePhase.SAFE_STOP_REQUESTED)
        self.assertEqual(requested.commands, (BoxShoeCommand.SAFE_STOP,))

        dropped = controller.dispatch(
            self.event(
                BoxShoeEventType.SAFE_STOP_RESULT,
                success=False,
                failure_code="command_dropped",
            )
        )
        self.assertEqual(dropped.phase, BoxShoePhase.SAFE_STOP_REQUESTED)
        self.assertEqual(dropped.commands, (BoxShoeCommand.SAFE_STOP,))

        confirmed = controller.dispatch(
            self.event(
                BoxShoeEventType.SAFE_STOP_RESULT,
                actuators_stopped=True,
                base_stationary=True,
            )
        )
        self.assertEqual(confirmed.phase, BoxShoePhase.SAFE_STOPPED)
        self.assertEqual(confirmed.commands, ())


class BoxShoePhysicsDemoTest(unittest.TestCase):
    def test_continuous_lift_evidence_rejects_launch_recontact_and_tracking_error(
        self,
    ) -> None:
        demo = BoxShoePhysicsDemo()
        mujoco.mj_resetData(demo.model, demo.data)
        demo.data.ctrl[:] = demo.data.qpos[demo.actuated_qpos_addresses]
        mujoco.mj_forward(demo.model, demo.data)
        demo._start_grasp_tracking()

        shoe_joint = demo.model.joint(SHOE_JOINT_NAME)
        address = int(shoe_joint.qposadr[0])
        original = demo.data.qpos[address : address + 7].copy()
        demo.data.qpos[address] += 0.05
        mujoco.mj_forward(demo.model, demo.data)
        demo._sample_dynamics()
        demo.data.qpos[address : address + 7] = original
        mujoco.mj_forward(demo.model, demo.data)
        demo._sample_dynamics()

        final_position, _ = _shoe_pose_in_left_gripper(demo.model, demo.data)
        self.assertLess(
            float(np.linalg.norm(final_position - demo._grasp_reference[0])),
            1e-9,
        )
        self.assertGreaterEqual(
            demo.maximum_grasp_relative_translation_drift_m,
            0.049,
        )
        self.assertFalse(
            _continuous_lift_evidence_ok(
                demo.maximum_grasp_relative_translation_drift_m,
                demo.maximum_grasp_relative_rotation_drift_rad,
                0.0,
            )
        )

        demo.data.qpos[demo.actuated_qpos_addresses[0]] += 0.2
        mujoco.mj_forward(demo.model, demo.data)
        demo._sample_dynamics()
        self.assertGreaterEqual(demo.maximum_actuator_tracking_error_rad, 0.19)
        self.assertFalse(_continuous_lift_evidence_ok(0.0, 0.0, 0.2))

    def test_collision_evidence_covers_structures_with_one_exact_hold_exception(
        self,
    ) -> None:
        for pair in (
            ("left_gripper", "right_gripper"),
            ("left_gripper", "tb3_base_link"),
            ("left_gripper", "depth_camera_body"),
            ("left_gripper", "tower_mount_structure"),
            ("left_gripper", "world"),
            ("left_gripper", "box_fixture"),
            ("left_upper_arm", "cuboid_shoe_proxy"),
        ):
            with self.subTest(pair=pair):
                self.assertEqual(
                    _classify_unintended_contact(
                        pair,
                        distance_m=-0.002,
                        right_lid_hold_active=True,
                    ),
                    tuple(sorted(pair)),
                )

        expected_hold = ("box_lid", "right_gripper")
        self.assertIsNone(
            _classify_unintended_contact(
                expected_hold,
                distance_m=-0.002,
                right_lid_hold_active=True,
            )
        )
        self.assertEqual(
            _classify_unintended_contact(
                expected_hold,
                distance_m=-0.002,
                right_lid_hold_active=False,
            ),
            expected_hold,
        )
        self.assertEqual(
            _classify_unintended_contact(
                ("box_lid", "right_upper_arm"),
                distance_m=-0.002,
                right_lid_hold_active=True,
            ),
            ("box_lid", "right_upper_arm"),
        )

    def test_shoe_box_penetration_is_accumulated_dynamic_evidence(self) -> None:
        for structure in ("box_fixture", "box_lid"):
            pair = ("cuboid_shoe_proxy", structure)
            with self.subTest(structure=structure):
                self.assertEqual(
                    _classify_unintended_contact(
                        pair,
                        distance_m=-0.010,
                        right_lid_hold_active=True,
                    ),
                    tuple(sorted(pair)),
                )
                self.assertIsNone(
                    _classify_unintended_contact(
                        pair,
                        distance_m=0.0,
                        right_lid_hold_active=True,
                    )
                )
        self.assertIsNone(
            _classify_unintended_contact(
                ("box_lid", "right_gripper"),
                distance_m=-0.010,
                right_lid_hold_active=True,
            )
        )

    def test_invalid_physical_contacts_do_not_verify_a_grasp(self) -> None:
        with redirect_stdout(StringIO()):
            report = BoxShoePhysicsDemo().run()

        self.assertFalse(report.success)
        self.assertEqual(report.completed_phase, "left_physical_grasp_failed")
        self.assertGreater(report.right_wing_contact_count, 0)
        self.assertGreaterEqual(report.lid_open_angle_deg, 90.0)
        self.assertEqual(report.left_moving_finger_contact_count, 0)
        self.assertFalse(report.bilateral_finger_contact_verified)
        self.assertFalse(report.opposing_finger_contact_verified)
        self.assertFalse(report.shoe_grasp_weld_present)
        self.assertFalse(report.friction_lift_verified)
        self.assertNotIn(
            ("box_lid", "right_gripper"), report.unintended_contact_pairs
        )
        self.assertIn(("box_lid", "left_wrist"), report.unintended_contact_pairs)
        self.assertTrue(
            any(
                phase.startswith("left_")
                and tuple(bodies) == ("box_lid", "left_wrist")
                for phase, *bodies in report.unintended_contact_phases
            )
        )
        self.assertEqual(report.runtime_arm_qpos_writes, 0)
        self.assertFalse(report.hardware_execution)

    def test_opposing_contact_metric_is_deterministic(self) -> None:
        fixed = [np.asarray((1.0, 0.0, 0.0))]
        self.assertEqual(
            _most_opposing_normal_dot(fixed, [np.asarray((-1.0, 0.0, 0.0))]),
            -1.0,
        )
        self.assertEqual(
            _most_opposing_normal_dot(fixed, [np.asarray((0.0, 1.0, 0.0))]),
            0.0,
        )
        self.assertIsNone(_most_opposing_normal_dot(fixed, []))

    def test_shoe_clearance_uses_oriented_world_extent(self) -> None:
        demo = BoxShoePhysicsDemo()
        mujoco.mj_resetData(demo.model, demo.data)
        shoe_joint = demo.model.joint(SHOE_JOINT_NAME)
        address = int(shoe_joint.qposadr[0])
        center_z = demo.scene_config.box_outer_size_m[2] + 0.05
        demo.data.qpos[address : address + 3] = (0.2, 0.0, center_z)
        demo.data.qpos[address + 3 : address + 7] = (
            math.sqrt(0.5),
            0.0,
            math.sqrt(0.5),
            0.0,
        )
        mujoco.mj_forward(demo.model, demo.data)

        shoe_body_id = int(demo.model.jnt_bodyid[shoe_joint.id])
        with redirect_stdout(StringIO()):
            report = demo._report(
                "test", 0, 0, demo.data.xpos[shoe_body_id].copy()
            )
        expected = center_z - demo.scene_config.shoe_half_size_m[0]
        self.assertAlmostEqual(
            report.shoe_bottom_clearance_m,
            expected - demo.scene_config.box_outer_size_m[2],
            places=6,
        )
        self.assertFalse(report.shoe_clear_of_box)

    def test_custom_scene_contract_reports_compiled_geometry(self) -> None:
        config = BoxShoeSceneConfig(
            box_outer_size_m=(0.30, 0.22, 0.12),
            cardboard_thickness_m=0.002,
            box_yaw_deg=17.5,
        )
        contract = box_shoe_scene_contract(build_box_shoe_scene_model(config))
        self.assertEqual(contract["box_outer_size_m"], config.box_outer_size_m)
        self.assertAlmostEqual(contract["box_yaw_deg"], config.box_yaw_deg)
        self.assertEqual(
            contract["cardboard_thickness_m"],
            config.cardboard_thickness_m,
        )
        self.assertFalse(contract["shoe_grasp_weld_present"])


if __name__ == "__main__":
    unittest.main()
