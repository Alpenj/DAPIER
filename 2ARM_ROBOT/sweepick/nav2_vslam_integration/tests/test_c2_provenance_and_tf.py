"""C-2 P0-3 / P0-5: TF2 provenance, detector provenance and re-publish rejection."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
SWEEPICK_ROOT = HERE.parent.parent
if str(SWEEPICK_ROOT) not in sys.path:
    sys.path.insert(0, str(SWEEPICK_ROOT))

from nav2_vslam_integration import (  # noqa: E402
    DetectionProvenance,
    ManipulationResultStatus,
    PostStopReobserveGate,
    SensorFrameLedger,
    SweepickExecutionAdapter,
    VerifiedTransform,
    VslamSweepickCoordinator,
    build_post_stop_observation,
    compose_static_chain,
)
from nav2_vslam_integration.tf_provenance import (  # noqa: E402
    PROVENANCE_SOURCE_MOCK,
    PROVENANCE_SOURCE_POSE_HEADER_ONLY,
    PROVENANCE_SOURCE_SIDECAR,
    REJECT_MOCK_NOT_ALLOWED,
    REJECT_OBSERVATION_UNVERIFIED,
    REJECT_REPUBLISHED,
    REJECT_TF_NOT_REAL,
    REJECT_TF_STAMP_MISMATCH,
    REJECT_TF_UNAVAILABLE,
    TF_SOURCE_LOOKUP,
    TF_SOURCE_MOCK,
    rpy_to_quaternion,
)
from nav2_vslam_integration.tests.test_mock_e2e_integration import (  # noqa: E402
    MockSweepickConsoleController,
    _make_standard_fixtures,
)

CLOCK = "nav2_host:MONOTONIC"
URDF_CHAIN = [
    ("base_footprint", "base_link", (0.0, 0.0, 0.010), (0.0, 0.0, 0.0)),
    ("base_link", "camera_link", (0.073, -0.011, 0.084), (0.0, 0.0, 0.0)),
    ("camera_link", "camera_color_optical_frame", (0.0, 0.0, 0.0), (-1.5707963, 0.0, -1.5707963)),
]


def _lookup_like(stamp_s: float) -> VerifiedTransform:
    """A transform shaped like a tf2 lookup result (REAL grade) built for the test from the chain."""
    mock = compose_static_chain(URDF_CHAIN, stamp_s=stamp_s, clock_id=CLOCK)
    return VerifiedTransform(parent_frame=mock.parent_frame, child_frame=mock.child_frame, stamp_s=stamp_s, clock_id=CLOCK,
                             translation_m=mock.translation_m, rotation_xyzw=mock.rotation_xyzw, source=TF_SOURCE_LOOKUP,
                             lookup_time_s=stamp_s + 0.01, tf_authority="test double of tf2_ros.Buffer")


def _sidecar(stamp_s: float, **over) -> DetectionProvenance:
    base = dict(source_topic="/docking/target_object_pose", message_stamp_s=stamp_s + 0.02, receive_index=125, clock_id=CLOCK,
                provenance_source=PROVENANCE_SOURCE_SIDECAR, publisher_node="/object_detector_3d", sensor_stamp_s=stamp_s,
                anchor_source="hsv", is_ema_filtered=False, is_persistence_latched=False, inlier_count=90, rms_m=0.006,
                detector_mode="hsv")
    base.update(over)
    return DetectionProvenance(**base)


class TestC2TransformContract(unittest.TestCase):
    def test_constant_transform_cannot_claim_lookup_grade(self) -> None:
        with self.assertRaises(ValueError):
            VerifiedTransform("base_footprint", "camera_color_optical_frame", 1.0, CLOCK, (0.073, 0.0, 0.084), (0, 0, 0, 1), source="URDF_CONSTANT")
        with self.assertRaises(ValueError):
            compose_static_chain(URDF_CHAIN, stamp_s=1.0, clock_id=CLOCK, source=TF_SOURCE_LOOKUP)
        self.assertEqual(compose_static_chain(URDF_CHAIN, stamp_s=1.0, clock_id=CLOCK).evidence_grade, "MOCK")

    def test_urdf_chain_differs_from_argon_hardcoded_origin(self) -> None:
        chain = compose_static_chain(URDF_CHAIN, stamp_s=1.0, clock_id=CLOCK)
        np.testing.assert_allclose(chain.translation_m, (0.073, -0.011, 0.094), atol=1e-9)
        self.assertFalse(np.allclose(chain.translation_m, (0.073, 0.0, 0.084)))
        # optical (x_right, y_down, z_forward) -> base: forward = z, left = -x, up = -y
        p = chain.apply_point((0.10, -0.05, 0.50))
        np.testing.assert_allclose(p, (0.573, -0.111, 0.144), atol=1e-6)
        q = rpy_to_quaternion(-1.5707963, 0.0, -1.5707963)
        self.assertAlmostEqual(sum(v * v for v in q), 1.0, places=9)

    def test_tf_missing_is_rejected_not_defaulted(self) -> None:
        obs, rej = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                               sensor_frame_id="camera_color_optical_frame", provenance=_sidecar(112.85), transform=None)
        self.assertIsNone(obs)
        self.assertIn(REJECT_TF_UNAVAILABLE, rej)

    def test_tf_from_the_past_is_rejected(self) -> None:
        obs, rej = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                               sensor_frame_id="camera_color_optical_frame", provenance=_sidecar(112.85),
                                               transform=_lookup_like(100.0))
        self.assertIsNone(obs)
        self.assertTrue(any(r.startswith(REJECT_TF_STAMP_MISMATCH) for r in rej))

    def test_mock_tf_rejected_in_real_mode_accepted_in_mock_mode(self) -> None:
        mock_tf = compose_static_chain(URDF_CHAIN, stamp_s=112.85, clock_id=CLOCK)
        obs, rej = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                               sensor_frame_id="camera_color_optical_frame", provenance=_sidecar(112.85), transform=mock_tf)
        self.assertIsNone(obs)
        self.assertTrue(any(r.startswith(REJECT_TF_NOT_REAL) for r in rej))
        obs, rej = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                               sensor_frame_id="camera_color_optical_frame",
                                               provenance=_sidecar(112.85, provenance_source=PROVENANCE_SOURCE_MOCK), transform=mock_tf, allow_mock=True)
        self.assertEqual(rej, ())
        self.assertEqual(obs.frame_id, "base_footprint")
        np.testing.assert_allclose(obs.position_in_base_footprint_m, (0.373, -0.011, 0.094), atol=1e-6)

    def test_real_lookup_grade_transform_accepted(self) -> None:
        obs, rej = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                               sensor_frame_id="camera_color_optical_frame", provenance=_sidecar(112.85), transform=_lookup_like(112.85))
        self.assertEqual(rej, ())
        self.assertEqual(obs.captured_at_s, 112.85)
        self.assertEqual(obs.inlier_count, 90)
        self.assertEqual(obs.observation_id, "/docking/target_object_pose@sensor:112850000000")


class TestC2DetectorProvenance(unittest.TestCase):
    def test_pose_header_only_is_unverified(self) -> None:
        prov = DetectionProvenance(source_topic="/docking/target_object_pose", message_stamp_s=112.87, receive_index=3, clock_id=CLOCK,
                                   provenance_source=PROVENANCE_SOURCE_POSE_HEADER_ONLY, inlier_count=85)
        self.assertFalse(prov.is_verified())
        obs, rej = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                               sensor_frame_id="camera_color_optical_frame", provenance=prov, transform=_lookup_like(112.87))
        self.assertIsNone(obs)
        self.assertTrue(any(r.startswith(REJECT_OBSERVATION_UNVERIFIED) for r in rej))
        self.assertIn("sensor_stamp_s", rej[0])

    def test_sidecar_missing_rms_is_unverified(self) -> None:
        prov = _sidecar(112.85, rms_m=None)
        self.assertEqual(prov.missing_fields(), ["rms_m"])
        obs, rej = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                               sensor_frame_id="camera_color_optical_frame", provenance=prov, transform=_lookup_like(112.85))
        self.assertIsNone(obs)

    def test_mock_provenance_rejected_in_real_mode(self) -> None:
        obs, rej = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                               sensor_frame_id="camera_color_optical_frame",
                                               provenance=_sidecar(112.85, provenance_source=PROVENANCE_SOURCE_MOCK), transform=_lookup_like(112.85))
        self.assertIsNone(obs)
        self.assertTrue(any(r.startswith(REJECT_MOCK_NOT_ALLOWED) for r in rej))

    def test_republished_pose_from_same_sensor_frame_is_rejected(self) -> None:
        ledger = SensorFrameLedger()
        first, rej1 = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                                  sensor_frame_id="camera_color_optical_frame", provenance=_sidecar(112.85),
                                                  transform=_lookup_like(112.85), ledger=ledger)
        self.assertEqual(rej1, ())
        # the detector re-publishes 50 ms later with a NEW header stamp but the SAME sensor frame
        again = _sidecar(112.85, message_stamp_s=112.92, receive_index=126)
        second, rej2 = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                                   sensor_frame_id="camera_color_optical_frame", provenance=again,
                                                   transform=_lookup_like(112.85), ledger=ledger)
        self.assertIsNone(second)
        self.assertTrue(any(r.startswith(REJECT_REPUBLISHED) for r in rej2))

    def test_ema_or_latched_sidecar_is_rejected_by_reobserve_gate(self) -> None:
        _, _, base_stop, _ = _make_standard_fixtures()
        gate = PostStopReobserveGate()
        obs, _ = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                             sensor_frame_id="camera_color_optical_frame",
                                             provenance=_sidecar(112.85, is_ema_filtered=True, is_persistence_latched=True),
                                             transform=_lookup_like(112.85))
        verdict = gate.evaluate(base_stop=base_stop, observation=obs, checked_at_s=112.9)
        self.assertFalse(verdict.passed)
        self.assertIn("EMA_FILTER_CARRYOVER_FROM_MOTION_FORBIDDEN", verdict.rejected_reasons)
        self.assertIn("PERSISTENCE_LATCHED_OBSERVATION_FORBIDDEN", verdict.rejected_reasons)

    def test_reobserve_gate_consumes_passed_observation(self) -> None:
        _, _, base_stop, _ = _make_standard_fixtures()
        gate = PostStopReobserveGate()
        obs, _ = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                             sensor_frame_id="camera_color_optical_frame", provenance=_sidecar(112.85),
                                             transform=_lookup_like(112.85))
        self.assertTrue(gate.evaluate(base_stop=base_stop, observation=obs, checked_at_s=112.9).passed)
        second = gate.evaluate(base_stop=base_stop, observation=obs, checked_at_s=112.95)
        self.assertFalse(second.passed)
        self.assertIn("REPUBLISHED_OBSERVATION_SAME_SENSOR_FRAME", second.rejected_reasons)

    def test_pipeline_republished_observation_stops_at_reobserve_without_arm(self) -> None:
        console = MockSweepickConsoleController()
        coord = VslamSweepickCoordinator(sweepick_adapter=SweepickExecutionAdapter(console))
        pre, arrival, base_stop, _ = _make_standard_fixtures()
        obs, _ = build_post_stop_observation(position_in_sensor_frame_m=(0, 0, 0.3), normal_in_sensor_frame=(0, 0, -1),
                                             sensor_frame_id="camera_color_optical_frame", provenance=_sidecar(112.85),
                                             transform=_lookup_like(112.85))
        first = coord.execute_pipeline(initial_detection=pre, arrival=arrival, base_stop=base_stop, post_stop_observation=obs, reobserve_checked_at_s=112.9)
        self.assertEqual(first.result_status, ManipulationResultStatus.WAITING_FOR_OPERATOR_APPROVAL)
        # the Console session ends normally; the next cycle re-uses the same frame -> refused before any arm request
        console.simulate_operator_start_and_finish(final_state="ENDED", session_result="RECEIVE_CONTACT_DONE_PUT_BACK")
        coord.finalize_after_operator_execution()
        console.state, console.can_prepare, console.active = "IDLE", True, None
        second = coord.execute_pipeline(initial_detection=pre, arrival=arrival, base_stop=base_stop, post_stop_observation=obs, reobserve_checked_at_s=112.95)
        self.assertEqual(second.result_status, ManipulationResultStatus.OBSERVATION_FAILED)
        self.assertIn("REPUBLISHED_OBSERVATION_SAME_SENSOR_FRAME", second.reason)
        self.assertEqual(console.prepare_calls, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
