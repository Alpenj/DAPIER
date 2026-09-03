from __future__ import annotations

from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_core import (
    MissionCommand,
    MissionConfig,
    MissionController,
    MissionEvent,
    MissionEventType,
    MissionLocation,
    MissionPhase,
)


class MissionCoreTest(unittest.TestCase):
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

    def advance_to_grasp_verification(self, controller: MissionController) -> None:
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

    def test_happy_path_emits_ordered_high_level_commands(self) -> None:
        controller = MissionController()
        transitions = [controller.dispatch(self.event(MissionEventType.START))]
        transitions.append(
            controller.dispatch(
                self.event(
                    MissionEventType.NAVIGATION_RESULT,
                    location=MissionLocation.B,
                )
            )
        )
        transitions.append(
            controller.dispatch(
                self.event(MissionEventType.POSE_RESULT, pose_confidence=0.9)
            )
        )
        transitions.append(controller.dispatch(self.event(MissionEventType.PICK_RESULT)))
        transitions.append(
            controller.dispatch(
                self.event(
                    MissionEventType.GRASP_RESULT,
                    object_lifted=True,
                    gripper_holding=True,
                    carry_pose_clear=True,
                )
            )
        )
        transitions.append(
            controller.dispatch(
                self.event(
                    MissionEventType.TRANSPORT_HOLD_RESULT,
                    transport_hold_ok=True,
                )
            )
        )
        transitions.append(
            controller.dispatch(
                self.event(
                    MissionEventType.NAVIGATION_RESULT,
                    location=MissionLocation.A,
                    transport_hold_ok=True,
                )
            )
        )
        transitions.append(controller.dispatch(self.event(MissionEventType.PLACE_RESULT)))
        transitions.append(
            controller.dispatch(
                self.event(
                    MissionEventType.RELEASE_RESULT,
                    object_released=True,
                    at_start_zone=True,
                )
            )
        )

        self.assertEqual(controller.phase, MissionPhase.COMPLETED)
        self.assertFalse(controller.carrying)
        self.assertEqual(
            [transition.phase for transition in transitions],
            [
                MissionPhase.NAVIGATING_TO_B,
                MissionPhase.LOCALIZING_SHOE,
                MissionPhase.PICKING_AT_B,
                MissionPhase.VERIFYING_GRASP,
                MissionPhase.LATCHING_TRANSPORT_HOLD,
                MissionPhase.NAVIGATING_TO_A,
                MissionPhase.PLACING_AT_A,
                MissionPhase.VERIFYING_PLACE,
                MissionPhase.COMPLETED,
            ],
        )
        self.assertEqual(
            transitions[4].commands,
            (MissionCommand.LATCH_TRANSPORT_HOLD,),
        )
        self.assertEqual(transitions[5].commands, (MissionCommand.NAVIGATE_TO_A,))
        self.assertTrue(
            all(not transition.hardware_execution for transition in transitions)
        )
        self.assertTrue(
            all(
                not transition.hardware_dispatch_authorized
                for transition in transitions
            )
        )

    def test_stale_age_or_sequence_latches_safe_stop(self) -> None:
        stale = MissionController()
        transition = stale.dispatch(
            MissionEvent(
                kind=MissionEventType.START,
                observation_seq=0,
                observation_age_ms=101.0,
            )
        )
        self.assertEqual(transition.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertEqual(transition.commands, (MissionCommand.SAFE_STOP,))

        duplicate = MissionController()
        duplicate.dispatch(self.event(MissionEventType.START))
        transition = duplicate.dispatch(
            MissionEvent(
                kind=MissionEventType.NAVIGATION_RESULT,
                observation_seq=0,
                observation_age_ms=10.0,
                location=MissionLocation.B,
            )
        )
        self.assertEqual(transition.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertIn("sequence", transition.reason)

    def test_safe_stop_requires_actuator_and_base_confirmation(self) -> None:
        controller = MissionController()
        requested = controller.dispatch(
            self.event(
                MissionEventType.FAULT,
                success=False,
                failure_code="watchdog",
            )
        )
        self.assertEqual(requested.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertEqual(requested.commands, (MissionCommand.SAFE_STOP,))

        dropped = controller.dispatch(
            self.event(
                MissionEventType.SAFE_STOP_RESULT,
                success=False,
                failure_code="command_dropped",
            )
        )
        self.assertEqual(dropped.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertEqual(dropped.commands, (MissionCommand.SAFE_STOP,))

        confirmed = controller.dispatch(
            self.event(
                MissionEventType.SAFE_STOP_RESULT,
                actuators_stopped=True,
                base_stationary=True,
            )
        )
        self.assertEqual(confirmed.phase, MissionPhase.SAFE_STOPPED)
        self.assertEqual(confirmed.commands, ())

    def test_base_arm_interlock_and_unexpected_event_fail_closed(self) -> None:
        moving = MissionController()
        moving.dispatch(self.event(MissionEventType.START))
        moving.dispatch(
            self.event(
                MissionEventType.NAVIGATION_RESULT,
                location=MissionLocation.B,
            )
        )
        transition = moving.dispatch(
            self.event(
                MissionEventType.POSE_RESULT,
                pose_confidence=0.9,
                base_stationary=False,
            )
        )
        self.assertEqual(transition.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertIn("stationary", transition.reason)

        unexpected = MissionController()
        transition = unexpected.dispatch(
            self.event(MissionEventType.PICK_RESULT)
        )
        self.assertEqual(transition.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertIn("unexpected", transition.reason)

    def test_transport_hold_loss_during_return_fails_closed(self) -> None:
        controller = MissionController()
        self.advance_to_grasp_verification(controller)
        controller.dispatch(
            self.event(
                MissionEventType.GRASP_RESULT,
                object_lifted=True,
                gripper_holding=True,
                carry_pose_clear=True,
            )
        )
        controller.dispatch(
            self.event(
                MissionEventType.TRANSPORT_HOLD_RESULT,
                transport_hold_ok=True,
            )
        )
        transition = controller.dispatch(
            self.event(
                MissionEventType.NAVIGATION_RESULT,
                location=MissionLocation.A,
                transport_hold_ok=False,
            )
        )
        self.assertEqual(transition.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertIn("transport hold", transition.reason)

    def test_navigation_waits_for_healthy_transport_hold_ack(self) -> None:
        def request_hold(controller: MissionController):
            self.advance_to_grasp_verification(controller)
            return controller.dispatch(
                self.event(
                    MissionEventType.GRASP_RESULT,
                    object_lifted=True,
                    gripper_holding=True,
                    carry_pose_clear=True,
                )
            )

        acknowledged = MissionController()
        requested = request_hold(acknowledged)
        self.assertEqual(requested.phase, MissionPhase.LATCHING_TRANSPORT_HOLD)
        self.assertEqual(
            requested.commands,
            (MissionCommand.LATCH_TRANSPORT_HOLD,),
        )
        navigating = acknowledged.dispatch(
            self.event(
                MissionEventType.TRANSPORT_HOLD_RESULT,
                transport_hold_ok=True,
            )
        )
        self.assertEqual(navigating.phase, MissionPhase.NAVIGATING_TO_A)
        self.assertEqual(navigating.commands, (MissionCommand.NAVIGATE_TO_A,))

        no_ack = MissionController()
        request_hold(no_ack)
        rejected = no_ack.dispatch(
            self.event(
                MissionEventType.NAVIGATION_RESULT,
                location=MissionLocation.A,
            )
        )
        self.assertEqual(rejected.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertNotIn(MissionCommand.NAVIGATE_TO_A, rejected.commands)

        unhealthy = MissionController()
        request_hold(unhealthy)
        rejected = unhealthy.dispatch(
            self.event(
                MissionEventType.TRANSPORT_HOLD_RESULT,
                transport_hold_ok=False,
            )
        )
        self.assertEqual(rejected.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertNotIn(MissionCommand.NAVIGATE_TO_A, rejected.commands)

        commands = [
            command
            for transition in acknowledged.history
            for command in transition.commands
        ]
        self.assertLess(
            commands.index(MissionCommand.LATCH_TRANSPORT_HOLD),
            commands.index(MissionCommand.NAVIGATE_TO_A),
        )

    def test_navigation_retry_requires_settled_base(self) -> None:
        controller = MissionController()
        controller.dispatch(self.event(MissionEventType.START))
        transition = controller.dispatch(
            self.event(
                MissionEventType.NAVIGATION_RESULT,
                success=False,
                recoverable=True,
                base_stationary=False,
                location=MissionLocation.B,
                failure_code="network_timeout",
            )
        )
        self.assertEqual(transition.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertIn("did not settle", transition.reason)

    def test_recovery_budget_is_bounded_and_reported(self) -> None:
        controller = MissionController(MissionConfig(max_retries_per_stage=2))
        self.advance_to_grasp_verification(controller)

        first = controller.dispatch(
            self.event(
                MissionEventType.GRASP_RESULT,
                success=False,
                recoverable=True,
                failure_code="grasp_missed",
            )
        )
        self.assertEqual(first.phase, MissionPhase.LOCALIZING_SHOE)
        self.assertEqual(first.retry_count, 1)

        controller.dispatch(
            self.event(MissionEventType.POSE_RESULT, pose_confidence=0.9)
        )
        controller.dispatch(self.event(MissionEventType.PICK_RESULT))
        second = controller.dispatch(
            self.event(
                MissionEventType.GRASP_RESULT,
                success=False,
                recoverable=True,
                failure_code="grasp_missed",
            )
        )
        self.assertEqual(second.retry_count, 2)

        controller.dispatch(
            self.event(MissionEventType.POSE_RESULT, pose_confidence=0.9)
        )
        controller.dispatch(self.event(MissionEventType.PICK_RESULT))
        exhausted = controller.dispatch(
            self.event(
                MissionEventType.GRASP_RESULT,
                success=False,
                recoverable=True,
                failure_code="grasp_missed",
            )
        )
        self.assertEqual(exhausted.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertEqual(exhausted.retry_count, 3)
        self.assertIn("budget exhausted", exhausted.reason)

    def test_low_pose_confidence_retries_and_fault_stops(self) -> None:
        controller = MissionController()
        controller.dispatch(self.event(MissionEventType.START))
        controller.dispatch(
            self.event(
                MissionEventType.NAVIGATION_RESULT,
                location=MissionLocation.B,
            )
        )
        retry = controller.dispatch(
            self.event(
                MissionEventType.POSE_RESULT,
                pose_confidence=0.2,
                recoverable=True,
            )
        )
        self.assertEqual(retry.phase, MissionPhase.LOCALIZING_SHOE)
        fault = controller.dispatch(
            self.event(
                MissionEventType.FAULT,
                success=False,
                failure_code="network_timeout",
            )
        )
        self.assertEqual(fault.phase, MissionPhase.SAFE_STOP_REQUESTED)
        self.assertIn("network_timeout", fault.reason)

    def test_invalid_event_and_terminal_state_contract(self) -> None:
        with self.assertRaisesRegex(ValueError, "failure_code"):
            self.event(MissionEventType.PICK_RESULT, success=False).validate()
        with self.assertRaisesRegex(ValueError, "location"):
            self.event(MissionEventType.NAVIGATION_RESULT).validate()

        with self.assertRaisesRegex(ValueError, "integer"):
            MissionEvent(
                kind=MissionEventType.START,
                observation_seq=1.5,
                observation_age_ms=10.0,
            ).validate()
        with self.assertRaisesRegex(ValueError, "MissionEventType"):
            MissionEvent(
                kind="start",
                observation_seq=0,
                observation_age_ms=10.0,
            ).validate()
        for field in ("success", "recoverable", "base_stationary", "carry_pose_clear"):
            with self.subTest(field=field), self.assertRaisesRegex(
                ValueError, "flags must be booleans"
            ):
                self.event(MissionEventType.START, **{field: 1}).validate()

        stopped = MissionController()
        stopped.dispatch(
            self.event(
                MissionEventType.FAULT,
                success=False,
                failure_code="watchdog",
            )
        )
        stopped.dispatch(
            self.event(
                MissionEventType.SAFE_STOP_RESULT,
                actuators_stopped=True,
            )
        )
        terminal = stopped.dispatch(self.event(MissionEventType.START))
        self.assertEqual(terminal.phase, MissionPhase.SAFE_STOPPED)
        self.assertEqual(terminal.commands, ())

        report = terminal.as_report()
        self.assertFalse(report["hardware_dispatch_authorized"])
        self.assertFalse(report["hardware_execution"])


if __name__ == "__main__":
    unittest.main()
