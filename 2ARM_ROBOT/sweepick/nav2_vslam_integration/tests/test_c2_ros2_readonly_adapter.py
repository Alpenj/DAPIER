"""C-2 P0-4 / P0-5: read-only graph state (no rclpy needed)."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
SWEEPICK_ROOT = HERE.parent.parent
if str(SWEEPICK_ROOT) not in sys.path:
    sys.path.insert(0, str(SWEEPICK_ROOT))

from nav2_vslam_integration import (  # noqa: E402
    IntegrationStage,
    ManipulationResultStatus,
    SweepickExecutionAdapter,
    VslamSweepickCoordinator,
    compose_static_chain,
)
from nav2_vslam_integration.ros2_readonly_adapter import (  # noqa: E402
    GOAL_STATUS_EXECUTING,
    GOAL_STATUS_SUCCEEDED,
    ReadOnlyGraphState,
)
from nav2_vslam_integration.tests.test_mock_e2e_integration import MockSweepickConsoleController  # noqa: E402

CLOCK = "ros2:ROS_TIME(use_sim_time=False)"
CHAIN = [
    ("base_footprint", "base_link", (0.0, 0.0, 0.010), (0.0, 0.0, 0.0)),
    ("base_link", "camera_link", (0.073, -0.011, 0.084), (0.0, 0.0, 0.0)),
    ("camera_link", "camera_color_optical_frame", (0.0, 0.0, 0.0), (-1.5707963, 0.0, -1.5707963)),
]


def _state(lookup=None) -> ReadOnlyGraphState:
    return ReadOnlyGraphState(clock_id=CLOCK, transform_lookup=lookup)


def _drive(state: ReadOnlyGraphState, t0: float, t1: float, vx: float, wz: float = 0.0, dt: float = 0.05) -> None:
    t = t0
    while t < t1:
        state.ingest_odom(stamp_s=t, receive_s=t, frame_id="odom", child_frame_id="base_footprint", linear_x=vx, linear_y=0.0,
                          angular_z=wz, pose_xy_yaw=(vx * t, 0.0, 0.0), publisher_node="/turtlebot3_node")
        t += dt


class TestReadOnlyGraphState(unittest.TestCase):
    def test_base_still_moving_forbids_arm_request(self) -> None:
        s = _state()
        s.ingest_nav2_status(receive_s=10.0, statuses=[("g1", GOAL_STATUS_EXECUTING, 10.0)], publisher_node="/bt_navigator")
        _drive(s, 10.0, 12.0, vx=0.12)
        s.ingest_cmd_vel(topic="/cmd_vel", stamp_s=11.95, receive_s=11.95, linear_x=0.12, angular_z=0.0, stamped=True, publisher_node="/velocity_smoother")
        gate = s.arm_task_gate(now_s=12.0)
        self.assertFalse(gate.allowed)
        self.assertIn("BASE_STILL_MOVING_ARM_REQUEST_FORBIDDEN", gate.reasons)
        self.assertIn("NAV2_GOAL_ACTIVE_ARM_REQUEST_FORBIDDEN", gate.reasons)
        self.assertIn("CMD_VEL_NONZERO_ARM_REQUEST_FORBIDDEN", gate.reasons)
        _, reasons = s.base_stop_evidence(now_s=12.0, arrived_at_s=11.0)
        self.assertIn("BASE_STILL_MOVING", reasons)

    def test_nav2_arrival_alone_is_not_manipulation_success(self) -> None:
        s = _state()
        s.ingest_nav2_status(receive_s=20.0, statuses=[("g1", GOAL_STATUS_SUCCEEDED, 20.0)], publisher_node="/bt_navigator")
        _drive(s, 19.0, 21.0, vx=0.0)
        arrival, reasons = s.arrival_evidence(goal_label="desk_staging")
        self.assertEqual(reasons, [])
        self.assertEqual(arrival.nav2_status, "STATUS_SUCCEEDED")
        self.assertFalse(arrival.manipulation_succeeded)
        self.assertEqual(arrival.final_pose_frame, "odom")  # no map TF lookup available -> labelled odom
        coord = VslamSweepickCoordinator(sweepick_adapter=SweepickExecutionAdapter(MockSweepickConsoleController()))
        coord.step_detect(None); coord.step_navigate()
        self.assertIsNone(coord.step_arrived(arrival))
        self.assertEqual(coord.current_stage, IntegrationStage.ARRIVED)
        self.assertEqual(coord.arm_commands_invoked_count, 0)

    def test_base_stop_evidence_from_observed_state_only(self) -> None:
        s = _state()
        s.ingest_nav2_status(receive_s=20.0, statuses=[("g1", GOAL_STATUS_SUCCEEDED, 20.0)], publisher_node="/bt_navigator")
        s.ingest_docking_status(receive_s=20.1, text="STATE=DOCKED, dist=0.040, yaw_err=+0.5deg", publisher_node="/precision_docking_controller_node")
        _drive(s, 18.0, 20.0, vx=0.05)
        _drive(s, 20.0, 21.5, vx=0.0)
        s.ingest_cmd_vel(topic="/cmd_vel", stamp_s=20.05, receive_s=20.05, linear_x=0.0, angular_z=0.0, stamped=True, publisher_node="/task_manager_node")
        for i in range(30):
            s.ingest_sensor_frame(header_stamp_s=18.0 + i * 0.1, receive_s=18.0 + i * 0.1)
        evidence, reasons = s.base_stop_evidence(now_s=21.5, arrived_at_s=20.0)
        self.assertEqual(reasons, [])
        self.assertTrue(evidence.nav2_controller_idle)
        self.assertTrue(evidence.cmd_vel_zero_published)
        self.assertGreaterEqual(evidence.settled_duration_s, 1.4)
        self.assertEqual(evidence.last_nav_frame_seq, 21)  # frames stamped at/before the confirmed stop (20.0 s)
        self.assertEqual(s.sensor_frame_index_at(20.9), 30)
        self.assertTrue(evidence.evidence_source.startswith("observed:"))
        self.assertTrue(s.arm_task_gate(now_s=21.5).allowed)
        arrival, _ = s.arrival_evidence(goal_label="workplace1")
        self.assertEqual(arrival.nav2_status, "DOCKED")

    def test_docking_active_forbids_arm_request(self) -> None:
        s = _state()
        _drive(s, 0.0, 2.0, vx=0.0)
        s.ingest_docking_status(receive_s=2.0, text="STATE=FINAL_APPROACH, dist=0.210, yaw_err=+1.0deg")
        gate = s.arm_task_gate(now_s=2.0)
        self.assertFalse(gate.allowed)
        self.assertTrue(any(r.startswith("DOCKING_ACTIVE_ARM_REQUEST_FORBIDDEN") for r in gate.reasons))

    def test_prior_sweepick_fault_unrecovered_forbids_new_task(self) -> None:
        s = _state()
        _drive(s, 0.0, 2.0, vx=0.0)
        s.ingest_console_status({"state": "FAULT", "owner_alive": False, "can_prepare": False,
                                 "active": {"id": "20261016_112020_4d7c2b", "operator_recovery": None}})
        gate = s.arm_task_gate(now_s=2.0)
        self.assertFalse(gate.allowed)
        self.assertIn("PRIOR_SWEEPICK_FAULT_UNRECOVERED_NEW_TASK_FORBIDDEN:20261016_112020_4d7c2b", gate.reasons)
        s.ingest_console_status({"state": "FAULT", "owner_alive": False, "can_prepare": True,
                                 "active": {"id": "20261016_112020_4d7c2b", "operator_recovery": {"source": "local_operator_manual_confirmation"}}})
        self.assertTrue(s.arm_task_gate(now_s=2.0).allowed)

    def test_pose_without_sidecar_is_unverified_even_with_tf(self) -> None:
        tf = compose_static_chain(CHAIN, stamp_s=30.0, clock_id=CLOCK)
        s = _state(lookup=lambda p, c, t: tf)
        _drive(s, 28.0, 31.0, vx=0.0)
        s.ingest_target_pose(receive_s=30.5, header_stamp_s=30.5, frame_id="camera_color_optical_frame", position=(0.0, 0.0, 0.3),
                             orientation_xyzw=(0.0, 0.0, 0.0, 1.0), publisher_node="/object_detector_3d")
        s.ingest_detection_status(receive_s=30.5, text="LOCKED: x=0.000, y=0.000, z=0.300, yaw_deg=0.0, inliers=88, fps=19.6")
        evidence, _ = s.base_stop_evidence(now_s=31.0, arrived_at_s=29.0)
        cand = s.latest_post_stop_candidate(base_stop=evidence)
        self.assertIsNone(cand["observation"])
        self.assertTrue(any(r.startswith("POST_STOP_OBSERVATION_UNVERIFIED") for r in cand["rejected"]))
        self.assertEqual(cand["provenance"]["provenance_source"], "POSE_HEADER_ONLY")
        self.assertEqual(cand["provenance"]["inlier_count"], 88)  # read from the status string, still not enough

    def test_sidecar_with_mock_tf_is_rejected_in_real_mode(self) -> None:
        tf = compose_static_chain(CHAIN, stamp_s=30.4, clock_id=CLOCK)
        s = _state(lookup=lambda p, c, t: tf)
        _drive(s, 28.0, 31.0, vx=0.0)
        s.ingest_target_pose(receive_s=30.5, header_stamp_s=30.5, frame_id="camera_color_optical_frame", position=(0.0, 0.0, 0.3),
                             orientation_xyzw=(0.0, 0.0, 0.0, 1.0), publisher_node="/sweepick_c2_mock_graph")
        s.ingest_provenance_sidecar(receive_s=30.5, payload={"pose_stamp_s": 30.5, "sensor_stamp_s": 30.4, "anchor_source": "hsv",
                                                             "is_ema_filtered": False, "is_persistence_latched": False,
                                                             "inlier_count": 90, "rms_m": 0.006, "detector_mode": "hsv"})
        evidence, _ = s.base_stop_evidence(now_s=31.0, arrived_at_s=29.0)
        cand = s.latest_post_stop_candidate(base_stop=evidence)
        self.assertIsNone(cand["observation"])
        self.assertTrue(any(r.startswith("TF_SOURCE_NOT_A_LOOKUP") for r in cand["rejected"]))
        self.assertTrue(cand["provenance"]["verified"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
