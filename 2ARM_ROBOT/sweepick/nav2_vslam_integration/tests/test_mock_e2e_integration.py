"""P2 Independent Mock End-to-End Verification Suite.

Covers:
1. Normal flow: DETECT -> NAVIGATE -> ARRIVED -> BASE_STOPPED -> REOBSERVE -> MANIPULATION_READY -> ARM_TASK -> RESULT
   (including WAITING_FOR_OPERATOR_APPROVAL -> Operator approval -> COMPLETED / PARTIAL_COMPLETED).
2. Observation failure: After Nav2 arrival and base stop, missing or stale/latched observation aborts at REOBSERVE
   without moving the arm (arm_commands_invoked_count == 0).
3. Execution failure: Sweepick STOP / FAULT / EXECUTION_REJECTED blocks subsequent commands and propagates a
   structured TaskManagerErrorReport to the Task Manager.
4. Safety invariants:
   - Nav2 arrival (`STATUS_SUCCEEDED` / `DOCKED`) is never manipulation completion.
   - Pre-navigation coordinates are never reused after motion.
   - Unverified `base_footprint <-> arm_base` metric TF projection is rejected.
   - Zero motor/torque authority is created by the integration layer.
"""

from __future__ import annotations

from pathlib import Path
import sys
from typing import Any, Dict, Optional
import unittest

HERE = Path(__file__).resolve().parent
SWEEPICK_ROOT = HERE.parent.parent
if str(SWEEPICK_ROOT) not in sys.path:
    sys.path.insert(0, str(SWEEPICK_ROOT))

from nav2_vslam_integration import (
    ApprovedSweepickScope,
    BaseStopEvidence,
    CameraObservation3D,
    HardwareReadinessState,
    IntegrationStage,
    ManipulationResultStatus,
    NavigationArrivalEvidence,
    NavigationManipulationBridge,
    SweepickExecutionAdapter,
    VslamSweepickCoordinator,
)


class MockSweepickConsoleController:
    """Mock of `2ARM_ROBOT/sweepick/console/control.py:Controller` for device-free E2E testing."""

    def __init__(self) -> None:
        self.state: str = "IDLE"
        self.owner_alive: bool = False
        self.can_prepare: bool = True
        self.can_start: bool = False
        self.can_confirm_recovery: bool = False
        self.block_reason: Optional[str] = None
        self.active: Optional[Dict[str, Any]] = None
        self.device_occupancy: Dict[str, str] = {}
        self.device_query_error: Optional[str] = None
        self.supported_scopes: Dict[str, bool] = {
            ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT.value: True,
            ApprovedSweepickScope.LEFT_START_TO_LIFT.value: True,
            ApprovedSweepickScope.RIGHT_X_FINITE_METROLOGY.value: True,
        }
        self.prepare_calls: int = 0
        self.stop_calls: int = 0
        self._run_counter: int = 0

    def operation_contract(self) -> Dict[str, Any]:
        return {
            "schema": "sweepick.operation-contract.v1",
            "scopes": [
                {
                    "id": scope_id,
                    "supported": supported,
                    "can_prepare": supported and self.can_prepare and not self.device_occupancy,
                    "reason": "mock contract" if supported else "active native SHA mismatch",
                }
                for scope_id, supported in self.supported_scopes.items()
            ],
            "owner": {
                "state": self.state,
                "owner_alive": self.owner_alive,
                "active_run_id": (self.active or {}).get("id"),
                "device_occupancy": dict(self.device_occupancy),
                "device_query_error": self.device_query_error,
            },
            "device_access": False,
        }

    def status(self) -> Dict[str, Any]:
        return {
            "execution_connected": True,
            "state": self.state,
            "owner_alive": self.owner_alive,
            "active": self.active,
            "can_prepare": self.can_prepare,
            "can_confirm_recovery": self.can_confirm_recovery,
            "can_start": self.can_start,
            "can_stop": self.owner_alive and self.state == "RUNNING",
            "block_reason": self.block_reason,
        }

    def prepare(
        self,
        source_session: Optional[str] = None,
        *,
        execution_scope: str = "FULL_CHAIN_RECEIVE_CONTACT",
        auto_after: Optional[str] = None,
        right_x: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        if not self.can_prepare:
            raise RuntimeError(self.block_reason or "cannot prepare")
        self.prepare_calls += 1
        self._run_counter += 1
        run_id = f"20261016_12300{self._run_counter}_c0ffee"
        self.state = "PREPARED"
        self.can_prepare = False
        self.can_start = True
        self.active = {
            "id": run_id,
            "source_session": source_session,
            "session_out": f"/tmp/mock_sweepick_runs/{run_id}/session",
            "execution_scope": execution_scope,
            "state": "PREPARED",
            "phase": "awaiting_operator_approval",
            "consumed": False,
        }
        if right_x is not None:
            self.active["right_x_metrology"] = right_x
        return self.status()

    def stop(self, job_id: str) -> Dict[str, Any]:
        self.stop_calls += 1
        self.state = "STOP_REQUESTED"
        if self.active:
            self.active["state"] = "STOP_REQUESTED"
            self.active["stop_sent"] = True
        return self.status()

    def simulate_operator_start_and_finish(
        self,
        *,
        final_state: str = "ENDED",
        session_result: str = "RECEIVE_CONTACT_DONE_PUT_BACK",
        close_result: str = "GRIP_CONTACT_HOLDING",
        lift_result: str = "HOLDING",
        held_after_lift: bool = True,
        normal_return_confirmed: bool = True,
        bimanual_handover_success: bool = False,
        limited_terminal: Optional[str] = None,
        first_error: Optional[Dict[str, Any]] = None,
        stop_sent: bool = False,
        failure_stage: Optional[str] = None,
    ) -> None:
        assert self.active is not None, "prepare must be called before operator start"
        self.state = final_state
        self.owner_alive = False
        self.can_start = False
        self.can_prepare = final_state == "ENDED" and normal_return_confirmed and not first_error
        self.can_confirm_recovery = final_state in ("FAULT", "LIMITED_HOLDING")
        self.active.update(
            state=final_state,
            consumed=True,
            session_result=session_result,
            held_after_lift=held_after_lift,
            normal_return_confirmed=normal_return_confirmed,
            stop_sent=stop_sent,
            first_error=first_error,
            stage_results=[
                {"side": "left", "stage": "pregrasp", "result": "HOLDING"},
                {"side": "left", "stage": "descend", "result": "HOLDING"},
                {"side": "left", "stage": "close", "result": close_result},
                {"side": "left", "stage": "lift", "result": lift_result},
            ],
            receive_contact={
                "result": "RECEIVE_CONTACT_COLLECTED",
                "bimanual_handover_success": bimanual_handover_success,
            },
            terminal={},
        )
        if failure_stage:
            self.active["failure_point"] = {"stage": failure_stage}
        if limited_terminal is not None:
            self.active["limited_terminal"] = {
                "terminal": limited_terminal,
                "session_result": session_result,
                "held_after_lift": held_after_lift,
                "torque_off_confirmed": normal_return_confirmed,
                "stop_latch_present": stop_sent,
            }


def _make_standard_fixtures():
    clock = "nav2_host:MONOTONIC"
    pre_nav_obs = CameraObservation3D(
        observation_id="obs_pre_nav_001",
        frame_seq=10,
        captured_at_s=100.0,
        clock_id=clock,
        frame_id="base_footprint",
        position_in_base_footprint_m=(1.25, 0.15, 0.05),
        normal_in_base_footprint=(-1.0, 0.0, 0.0),
        inlier_count=65,
        rms_m=0.011,
    )
    arrival = NavigationArrivalEvidence(
        goal_label="desk_red_object",
        nav2_status="DOCKED",
        arrived_at_s=112.0,
        clock_id=clock,
        final_pose_map=(1.82, -0.45, 0.0),
        last_nav_frame_seq=120,
    )
    base_stop = BaseStopEvidence(
        arrived_at_s=112.0,
        stopped_at_s=112.6,
        clock_id=clock,
        linear_vel_mps=0.0,
        angular_vel_radps=0.0,
        cmd_vel_zero_published=True,
        nav2_controller_idle=True,
        settled_duration_s=0.6,
        last_nav_frame_seq=120,
        pre_arrival_coordinates_invalidated=True,
    )
    post_stop_obs = CameraObservation3D(
        observation_id="obs_post_stop_125",
        frame_seq=125,
        captured_at_s=112.85,
        clock_id=clock,
        frame_id="base_footprint",
        position_in_base_footprint_m=(0.34, -0.02, 0.04),
        normal_in_base_footprint=(-0.998, 0.06, 0.0),
        inlier_count=92,
        rms_m=0.007,
    )
    return pre_nav_obs, arrival, base_stop, post_stop_obs


class TestMockEndToEndIntegration(unittest.TestCase):
    """P2 Mock End-to-End Verification for Scenarios 1, 2, 3 and P0/P1 invariants."""

    def setUp(self) -> None:
        self.mock_console = MockSweepickConsoleController()
        self.adapter = SweepickExecutionAdapter(self.mock_console)
        self.coordinator = VslamSweepickCoordinator(sweepick_adapter=self.adapter)

    # -------------------------------------------------------------------------
    # Scenario 1: Normal Flow (DETECT -> NAVIGATE -> ARRIVED -> BASE_STOPPED ->
    #             REOBSERVE -> MANIPULATION_READY -> ARM_TASK -> RESULT)
    # -------------------------------------------------------------------------
    def test_01_normal_flow_full_chain_waiting_then_completed(self) -> None:
        pre_obs, arrival, base_stop, post_obs = _make_standard_fixtures()

        # First pass prepares the Sweepick session and pauses for operator START approval
        outcome_wait = self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=post_obs,
            reobserve_checked_at_s=112.90,
            scope=ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT,
            require_full_handover=False,
        )
        self.assertEqual(outcome_wait.current_stage, IntegrationStage.ARM_TASK)
        self.assertEqual(
            outcome_wait.result_status,
            ManipulationResultStatus.WAITING_FOR_OPERATOR_APPROVAL,
        )
        self.assertFalse(outcome_wait.manipulation_succeeded)
        self.assertFalse(outcome_wait.motor_authority_created_by_integration)
        self.assertEqual(
            outcome_wait.stage_history,
            (
                "DETECT",
                "NAVIGATE",
                "ARRIVED",
                "BASE_STOPPED",
                "REOBSERVE",
                "MANIPULATION_READY",
                "ARM_TASK",
            ),
        )

        # Operator clicks START in Console and session completes with confirmed LEFT PICK/LIFT & put-back/OFF
        self.mock_console.simulate_operator_start_and_finish(
            final_state="ENDED",
            session_result="RECEIVE_CONTACT_DONE_PUT_BACK",
            close_result="GRIP_CONTACT_HOLDING",
            lift_result="HOLDING",
            held_after_lift=True,
            normal_return_confirmed=True,
        )
        outcome_done = self.coordinator.finalize_after_operator_execution(
            scope=ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT,
            require_full_handover=False,
        )
        self.assertEqual(outcome_done.current_stage, IntegrationStage.RESULT)
        self.assertEqual(outcome_done.result_status, ManipulationResultStatus.COMPLETED)
        self.assertTrue(outcome_done.manipulation_succeeded)
        self.assertTrue(outcome_done.left_pick_lift_confirmed)
        self.assertTrue(outcome_done.post_stop_reobserved)
        self.assertFalse(outcome_done.pre_nav_coordinate_reused)
        self.assertFalse(outcome_done.motor_authority_created_by_integration)
        self.assertEqual(
            outcome_done.stage_history,
            (
                "DETECT",
                "NAVIGATE",
                "ARRIVED",
                "BASE_STOPPED",
                "REOBSERVE",
                "MANIPULATION_READY",
                "ARM_TASK",
                "RESULT",
            ),
        )

    def test_02_normal_flow_left_start_to_lift_completed_and_partial(self) -> None:
        pre_obs, arrival, base_stop, post_obs = _make_standard_fixtures()

        # Run LEFT_START_TO_LIFT when full handover is required -> PARTIAL_COMPLETED
        self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=post_obs,
            reobserve_checked_at_s=112.90,
            scope=ApprovedSweepickScope.LEFT_START_TO_LIFT,
            require_full_handover=True,
        )
        self.mock_console.simulate_operator_start_and_finish(
            final_state="LIMITED_HOLDING",
            session_result="LEFT_LIFT_HELD_CONFIRMED",
            held_after_lift=True,
            normal_return_confirmed=False,
            limited_terminal="LEFT_LIFT_HELD_CONFIRMED",
        )
        outcome_partial = self.coordinator.finalize_after_operator_execution(
            scope=ApprovedSweepickScope.LEFT_START_TO_LIFT,
            require_full_handover=True,
        )
        self.assertEqual(outcome_partial.result_status, ManipulationResultStatus.PARTIAL_COMPLETED)
        self.assertTrue(outcome_partial.left_pick_lift_confirmed)
        self.assertFalse(outcome_partial.right_handover_confirmed)
        # Left arm holds with Torque ON -> subsequent commands blocked until manual recovery
        self.assertTrue(outcome_partial.subsequent_commands_blocked)

    def test_03_nav2_arrival_is_never_manipulation_success(self) -> None:
        pre_obs, arrival, _, _ = _make_standard_fixtures()
        self.coordinator.step_detect(pre_obs)
        self.coordinator.step_navigate()
        abort = self.coordinator.step_arrived(arrival)
        self.assertIsNone(abort)
        self.assertEqual(self.coordinator.current_stage, IntegrationStage.ARRIVED)
        self.assertFalse(arrival.manipulation_succeeded)
        self.assertEqual(self.coordinator.arm_commands_invoked_count, 0)

    # -------------------------------------------------------------------------
    # Scenario 2: Observation Failure (Post-Arrival Missing / Stale / Latched)
    # -------------------------------------------------------------------------
    def test_04_observation_failure_no_object_after_arrival_aborts_without_arm_motion(self) -> None:
        pre_obs, arrival, base_stop, _ = _make_standard_fixtures()

        outcome = self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=None,
            reobserve_checked_at_s=113.0,
        )
        self.assertEqual(outcome.current_stage, IntegrationStage.RESULT)
        self.assertEqual(outcome.result_status, ManipulationResultStatus.OBSERVATION_FAILED)
        self.assertFalse(outcome.manipulation_succeeded)
        self.assertEqual(self.coordinator.arm_commands_invoked_count, 0)
        self.assertEqual(self.mock_console.prepare_calls, 0)
        self.assertEqual(
            outcome.stage_history,
            ("DETECT", "NAVIGATE", "ARRIVED", "BASE_STOPPED", "REOBSERVE", "RESULT"),
        )
        self.assertIn("NO_POST_STOP_OBSERVATION_AVAILABLE", outcome.reason)

    def test_05_stale_pre_navigation_coordinates_rejected_at_reobserve(self) -> None:
        pre_obs, arrival, base_stop, _ = _make_standard_fixtures()

        # Attempt to reuse `pre_obs` after arrival
        outcome = self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=pre_obs,
            reobserve_checked_at_s=113.0,
        )
        self.assertEqual(outcome.result_status, ManipulationResultStatus.OBSERVATION_FAILED)
        self.assertFalse(outcome.manipulation_succeeded)
        self.assertEqual(self.coordinator.arm_commands_invoked_count, 0)
        self.assertEqual(self.mock_console.prepare_calls, 0)
        self.assertIn("PRE_NAVIGATION_OBSERVATION_REUSE_FORBIDDEN", outcome.reason)
        self.assertIn("STALE_PRE_STOP_OBSERVATION", outcome.reason)

    def test_06_latched_qwen_bbox_and_motion_ema_rejected_at_reobserve(self) -> None:
        pre_obs, arrival, base_stop, _ = _make_standard_fixtures()
        latched_obs = CameraObservation3D(
            observation_id="obs_qwen_latched_126",
            frame_seq=126,
            captured_at_s=112.9,
            clock_id=base_stop.clock_id,
            frame_id="base_footprint",
            position_in_base_footprint_m=(0.35, 0.0, 0.04),
            inlier_count=80,
            rms_m=0.009,
            detector_mode="qwen_vlm",
            is_ema_filtered_from_motion=True,
            is_persistence_latched=True,
            qwen_bbox_captured_at_s=95.0,  # Latched from before BASE_STOPPED (112.6s)
        )
        outcome = self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=latched_obs,
            reobserve_checked_at_s=113.0,
        )
        self.assertEqual(outcome.result_status, ManipulationResultStatus.OBSERVATION_FAILED)
        self.assertEqual(self.coordinator.arm_commands_invoked_count, 0)
        self.assertIn("EMA_FILTER_CARRYOVER_FROM_MOTION_FORBIDDEN", outcome.reason)
        self.assertIn("PERSISTENCE_LATCHED_OBSERVATION_FORBIDDEN", outcome.reason)
        self.assertIn("STALE_LATCHED_QWEN_BBOX", outcome.reason)

    def test_07_sweepick_native_look_or_grasp_failure_classified_as_observation_failed(self) -> None:
        pre_obs, arrival, base_stop, post_obs = _make_standard_fixtures()
        self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=post_obs,
            reobserve_checked_at_s=112.90,
        )
        # Native session could not align wrist camera and returned home without grasping
        self.mock_console.simulate_operator_start_and_finish(
            final_state="ENDED",
            session_result="NOT_ALIGNED_RETURNED_HOME",
            close_result="NOT_RUN",
            lift_result="NOT_RUN",
            held_after_lift=False,
            normal_return_confirmed=True,
            failure_stage="align",
        )
        outcome = self.coordinator.finalize_after_operator_execution()
        self.assertEqual(outcome.result_status, ManipulationResultStatus.OBSERVATION_FAILED)
        self.assertFalse(outcome.manipulation_succeeded)
        self.assertFalse(outcome.left_pick_lift_confirmed)

    # -------------------------------------------------------------------------
    # Scenario 3: Execution Failure (STOP / FAULT / EXECUTION_REJECTED) & Blocking
    # -------------------------------------------------------------------------
    def test_08_execution_failure_sweepick_stop_blocks_subsequent_commands(self) -> None:
        pre_obs, arrival, base_stop, post_obs = _make_standard_fixtures()
        self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=post_obs,
            reobserve_checked_at_s=112.90,
        )
        self.mock_console.simulate_operator_start_and_finish(
            final_state="FAULT",
            session_result="STOPPED_IN_DESCEND_HOLDING_NEEDS_A_PERSON",
            held_after_lift=False,
            normal_return_confirmed=False,
            stop_sent=True,
            first_error={
                "gate": "OPERATOR_STOP",
                "stop_event": "owner_stop",
                "detail": "SIGINT requested STOP during session",
            },
        )
        outcome_stop = self.coordinator.finalize_after_operator_execution()
        self.assertEqual(outcome_stop.current_stage, IntegrationStage.RESULT)
        self.assertEqual(outcome_stop.result_status, ManipulationResultStatus.STOP)
        self.assertTrue(outcome_stop.subsequent_commands_blocked)
        self.assertIsNotNone(outcome_stop.error_report)
        self.assertEqual(outcome_stop.error_report["result_status"], "STOP")

        # Next command is strictly blocked until explicit operator recovery confirmation
        outcome_next = self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=post_obs,
            reobserve_checked_at_s=120.0,
        )
        self.assertEqual(outcome_next.result_status, ManipulationResultStatus.EXECUTION_REJECTED)
        self.assertIn("COMMAND_BLOCKED_BY_PRIOR_FAILURE_OR_HOLD", outcome_next.error_report["error_code"])

    def test_09_execution_failure_sweepick_fault_blocks_subsequent_commands(self) -> None:
        pre_obs, arrival, base_stop, post_obs = _make_standard_fixtures()
        self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=post_obs,
            reobserve_checked_at_s=112.90,
        )
        self.mock_console.simulate_operator_start_and_finish(
            final_state="FAULT",
            session_result="STOPPED_IN_CLOSE_HOLDING_NEEDS_A_PERSON",
            held_after_lift=False,
            normal_return_confirmed=False,
            first_error={
                "gate": "SERVO_OVERLOAD",
                "detail": "ID=6 error=0x20 [RxPacketError] Overload",
            },
        )
        outcome_fault = self.coordinator.finalize_after_operator_execution()
        self.assertEqual(outcome_fault.result_status, ManipulationResultStatus.FAULT)
        self.assertTrue(outcome_fault.subsequent_commands_blocked)
        self.assertIsNotNone(self.coordinator.task_manager_error_report)
        self.assertEqual(self.coordinator.task_manager_error_report.error_code, "SWEEPICK_FAULT")

        # Verify subsequent command is blocked
        blocked_try = self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=post_obs,
            reobserve_checked_at_s=125.0,
        )
        self.assertEqual(blocked_try.result_status, ManipulationResultStatus.EXECUTION_REJECTED)

    def test_10_execution_failure_sweepick_rejected_manifest_or_foreign_owner(self) -> None:
        pre_obs, arrival, base_stop, post_obs = _make_standard_fixtures()
        # Simulate foreign managed owner active
        self.mock_console.state = "BLOCKED"
        self.mock_console.block_reason = "기존 managed owner가 존재함"

        outcome = self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=post_obs,
            reobserve_checked_at_s=112.90,
        )
        self.assertEqual(outcome.result_status, ManipulationResultStatus.EXECUTION_REJECTED)
        self.assertTrue(outcome.subsequent_commands_blocked)
        self.assertEqual(outcome.error_report["error_code"], "FOREIGN_MANAGED_OWNER_ACTIVE")

    def test_11_unverified_base_to_arm_tf_direct_projection_rejected(self) -> None:
        pre_obs, arrival, base_stop, post_obs = _make_standard_fixtures()
        # HardwareReadinessState defaults to base_to_arm_tf_calibrated=False
        self.assertFalse(self.coordinator.hardware_readiness.physical_cross_tree_tf_ready())

        outcome = self.coordinator.execute_pipeline(
            initial_detection=pre_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=post_obs,
            reobserve_checked_at_s=112.90,
            handoff_mode="DIRECT_BASE_TO_ARM_METRIC_TF",
        )
        self.assertEqual(outcome.result_status, ManipulationResultStatus.EXECUTION_REJECTED)
        self.assertEqual(outcome.error_report["failed_stage"], "MANIPULATION_READY")
        self.assertEqual(outcome.error_report["error_code"], "UNVERIFIED_BASE_TO_ARM_TF")
        self.assertEqual(self.coordinator.arm_commands_invoked_count, 0)

    def test_12_navigation_manipulation_bridge_full_e2e(self) -> None:
        """C-2: the bridge needs labelled MOCK provenance + a MOCK transform chain (URDF constants)
        and `allow_mock_evidence=True`; in REAL mode the same inputs are refused (test_12b)."""
        from nav2_vslam_integration import DetectionProvenance, compose_static_chain
        from nav2_vslam_integration.tf_provenance import PROVENANCE_SOURCE_MOCK

        bridge = NavigationManipulationBridge(coordinator=self.coordinator)
        _, arrival, base_stop, _ = _make_standard_fixtures()
        clock = base_stop.clock_id
        # Waffle Pi URDF + vslam_navigation.launch.py static TF, composed as a labelled MOCK chain.
        chain = [
            ("base_footprint", "base_link", (0.0, 0.0, 0.010), (0.0, 0.0, 0.0)),
            ("base_link", "camera_link", (0.073, -0.011, 0.084), (0.0, 0.0, 0.0)),
            ("camera_link", "camera_color_optical_frame", (0.0, 0.0, 0.0), (-1.5707963, 0.0, -1.5707963)),
        ]
        tf_pre = compose_static_chain(chain, stamp_s=100.0, clock_id=clock)
        tf_post = compose_static_chain(chain, stamp_s=112.85, clock_id=clock)
        prov_pre = DetectionProvenance(
            source_topic="/docking/target_object_pose", message_stamp_s=100.02, receive_index=10, clock_id=clock,
            provenance_source=PROVENANCE_SOURCE_MOCK, sensor_stamp_s=100.0, anchor_source="hsv",
            is_ema_filtered=False, is_persistence_latched=False, inlier_count=60, rms_m=0.012, detector_mode="hsv",
        )
        prov_post = DetectionProvenance(
            source_topic="/docking/target_object_pose", message_stamp_s=112.87, receive_index=125, clock_id=clock,
            provenance_source=PROVENANCE_SOURCE_MOCK, sensor_stamp_s=112.85, anchor_source="hsv",
            is_ema_filtered=False, is_persistence_latched=False, inlier_count=85, rms_m=0.008, detector_mode="hsv",
        )

        # Valid DOCK command on `desk` with `red_object` anchor cue
        outcome = bridge.execute_integrated_task(
            raw_intent_json='{"action": "DOCK", "target": {"target_structure": "desk", "anchor_cue": "red_object"}}',
            initial_detection_optical=(0.05, 0.02, 1.15),
            initial_provenance=prov_pre,
            initial_transform=tf_pre,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_detection_optical=(0.01, 0.03, 0.28),
            post_stop_provenance=prov_post,
            post_stop_transform=tf_post,
            reobserve_checked_at_s=112.90,
            allow_mock_evidence=True,
        )
        self.assertEqual(
            outcome.result_status,
            ManipulationResultStatus.WAITING_FOR_OPERATOR_APPROVAL,
        )

    def test_12b_bridge_refuses_unverified_or_mock_evidence_in_real_mode(self) -> None:
        from nav2_vslam_integration import DetectionProvenance, compose_static_chain
        from nav2_vslam_integration.tf_provenance import PROVENANCE_SOURCE_POSE_HEADER_ONLY

        bridge = NavigationManipulationBridge(coordinator=self.coordinator)
        _, arrival, base_stop, _ = _make_standard_fixtures()
        clock = base_stop.clock_id
        header_only = DetectionProvenance(
            source_topic="/docking/target_object_pose", message_stamp_s=112.87, receive_index=125, clock_id=clock,
            provenance_source=PROVENANCE_SOURCE_POSE_HEADER_ONLY,
        )
        outcome = bridge.execute_integrated_task(
            raw_intent_json='{"action": "DOCK", "target": {"target_structure": "desk", "anchor_cue": "red_object"}}',
            arrival=arrival,
            base_stop=base_stop,
            post_stop_detection_optical=(0.01, 0.03, 0.28),
            post_stop_provenance=header_only,
            post_stop_transform=None,
            reobserve_checked_at_s=112.90,
        )
        self.assertEqual(outcome.result_status, ManipulationResultStatus.OBSERVATION_FAILED)
        self.assertEqual(outcome.error_report["failed_stage"], "REOBSERVE")
        self.assertEqual(outcome.error_report["error_code"], "POST_STOP_OBSERVATION_UNVERIFIED")
        self.assertIn("TF_UNAVAILABLE_AT_MESSAGE_TIME", outcome.reason)
        self.assertEqual(self.mock_console.prepare_calls, 0)
        self.assertEqual(self.coordinator.arm_commands_invoked_count, 0)

    def test_13_real_console_controller_contract_integration(self) -> None:
        import subprocess
        import tempfile

        console_dir = SWEEPICK_ROOT / "console"
        if str(console_dir) not in sys.path:
            sys.path.insert(0, str(console_dir))
        import control as sweepick_control

        def fake_subprocess_run(argv, **kwargs):
            return subprocess.CompletedProcess(argv, returncode=1, stdout="", stderr="")

        with tempfile.TemporaryDirectory() as tmpdir:
            real_controller = sweepick_control.Controller(
                directory=Path(tmpdir),
                run=fake_subprocess_run,
            )
            real_adapter = SweepickExecutionAdapter(real_controller)
            real_coord = VslamSweepickCoordinator(sweepick_adapter=real_adapter)
            pre_obs, arrival, base_stop, post_obs = _make_standard_fixtures()

            # On a machine without /home/robot/sweepick_261013_receive_alignment_restore,
            # real_controller.operation_contract() reports supported=False, and adapter
            # safely rejects execution without opening any device.
            outcome = real_coord.execute_pipeline(
                initial_detection=pre_obs,
                arrival=arrival,
                base_stop=base_stop,
                post_stop_observation=post_obs,
                reobserve_checked_at_s=112.90,
            )
            self.assertEqual(
                outcome.result_status,
                ManipulationResultStatus.EXECUTION_REJECTED,
            )
            self.assertFalse(outcome.motor_authority_created_by_integration)


if __name__ == "__main__":
    unittest.main(verbosity=2)
