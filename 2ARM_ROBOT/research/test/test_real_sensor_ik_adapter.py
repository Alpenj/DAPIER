import math
import hashlib
import json
import tempfile
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dapier_research.control_intent import ControlIntent, load_contract
from dapier_research.real_sensor_ik_adapter import (
    bounded_pregrasp_plan,
    create_joint_position_intents,
    plan_pregrasp_staging_waypoints,
    transform_optical_point_to_arm,
    validate_sensor_estimate_for_motion,
)
from dapier_research.vision_target import VisionTargetError, VisionTargetEstimate


class RealSensorIkAdapterTest(unittest.TestCase):
    def test_carry_cannot_be_relabelled_pregrasp_even_with_success_flag(self):
        for extra in ({"candidate_mode":"carry_endpoint_ik"}, {"planning_phase":"LIFT"},
                      {"planning_phase":"PLACE"}, {"planning_phase":"CLOSE"}):
            with self.subTest(extra=extra), self.assertRaisesRegex(ValueError, "own checked phase path"):
                bounded_pregrasp_plan({"offline_candidate_accepted":True, **extra}, Path("unused"), now_s=1000.)

    def test_metric_geometry_verdict_reaches_plan_without_depth(self):
        # Synthetic inputs exercise the real boundary, not physical acceptance.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def source(name, value):
                path = root / name
                path.write_text(json.dumps(value))
                return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
            profile = source("profile.json", {
                "arm_signs": [1]*5, "arm_zero_offsets_deg": [0.]*5,
                "gripper_rad_limits": [0., 2.],
                "joints": [{"name": name, "sign": 1, "zero_offset_deg": 0.,
                            "maximum_velocity_rad_s": .3} for name in names]})
            measured = {**profile, "timestamp": "1970-01-01T00:16:40+00:00", "calibration": profile}
            candidate = {"offline_candidate_accepted": True,
                "scene_object": {"bound_to_path_reference": True},
                "seed_posture": {"seed_q_rad": [0.]*12, "left": measured, "right": measured},
                "solved_action_rad": [0.]*12, "mapping": {"profile": profile},
                "model": {**profile, "gripper_ranges_rad": [[0., 2.]]},
                "position_error_m": 0., "tool_axis_error_rad_by_side": {"left": 0.},
                "structured_clearance": {"safe": True, "minimum_clearance_m": .05}}
            cases = [({"metric_evidence": {"method": "known_cube_board_geometry", "metric_target_verified": True}}, True),
                     ({"depth_evidence": {"metric_target_verified": True}}, True),
                     ({"metric_evidence": {"metric_target_verified": False},
                       "depth_evidence": {"metric_target_verified": True}}, False),
                     ({"P2_PASS": True}, False),
                     ({"metric_evidence": {"metric_target_verified": "true"}}, False)]
            for evidence, expected in cases:
                with self.subTest(evidence=evidence):
                    candidate["block_source"] = source("block.json", {"rgb_timestamp_ns": 1_000_000_000_000, **evidence})
                    plan = bounded_pregrasp_plan(candidate, Path(profile["path"]), now_s=1000.)
                    self.assertIs(plan["sensor_target_verified"], expected)
                    self.assertFalse(plan["path_envelope_verified"])
                    self.assertFalse(plan["task_success"])
            self.assertEqual(plan["maximum_duration_s"], 3.5)
            explicit = bounded_pregrasp_plan(candidate, Path(profile["path"]), now_s=1000.,
                                            maximum_duration_s=40.)
            self.assertEqual(explicit["maximum_duration_s"], 40.)
            self.assertFalse(explicit["sensor_target_verified"])
            self.assertFalse(explicit["path_envelope_verified"])
            self.assertEqual(explicit["goal_rad"], plan["goal_rad"])
            for budget in (True, "40", 0., 3.49, 60.01, float("nan"), float("inf")):
                with self.subTest(budget=budget), self.assertRaisesRegex(ValueError, "maximum duration"):
                    bounded_pregrasp_plan(candidate, Path(profile["path"]), now_s=1000.,
                                          maximum_duration_s=budget)
            with self.assertRaisesRegex(ValueError, "stale or future"):
                bounded_pregrasp_plan(candidate, Path(profile["path"]), now_s=1061.,
                                      maximum_duration_s=40.)
            candidate["block_source"] = source("block.json", {"rgb_timestamp_ns": 1_000_000_000_000,
                "metric_evidence": None, "depth_evidence": {"metric_target_verified": True}})
            with self.assertRaisesRegex(ValueError, "metric target evidence"):
                bounded_pregrasp_plan(candidate, Path(profile["path"]), now_s=1000.)

    def test_changed_wrist_frame_is_rejected_before_plan_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            source_path = Path(directory) / "source"
            source_path.write_bytes(b"MOCK source")
            source = {"path":str(source_path), "sha256":hashlib.sha256(source_path.read_bytes()).hexdigest()}
            frame = Path(directory) / "frame"
            frame.write_bytes(b"changed image")
            measured = {**source, "timestamp":"1970-01-01T00:16:40+00:00", "calibration":source}
            candidate = {"offline_candidate_accepted":True, "candidate_mode":"wrist_feedback",
                "scene_object":{"bound_to_path_reference":True},
                "seed_posture":{"seed_q_rad":[0.]*12, "left":measured, "right":measured},
                "solved_action_rad":[0.]*12, "block_source":source, "model":source,
                "mapping":{"profile":source},
                "wrist_source":{**source, "frame_source":{"path":str(frame), "sha256":"0"*64}}}
            with self.assertRaisesRegex(ValueError, "source changed"):
                bounded_pregrasp_plan(candidate, Path("unused"), now_s=1000.)

    def test_nominal_scene_candidate_cannot_reach_executor(self):
        with self.assertRaisesRegex(ValueError, "observed object"):
            bounded_pregrasp_plan({"offline_candidate_accepted": True}, Path("unused"), now_s=1000.)

    def test_transform_optical_point_to_arm(self):
        # Identity transform with translation (0.1, 0.2, 0.3)
        T_arm_camera = np.eye(4)
        T_arm_camera[:3, 3] = [0.1, 0.2, 0.3]

        pt_optical = [0.05, -0.02, 0.50]
        pt_arm = transform_optical_point_to_arm(pt_optical, T_arm_camera)

        expected = np.array([0.15, 0.18, 0.80])
        np.testing.assert_allclose(pt_arm, expected, atol=1e-6)

    def test_transform_invalid_point_or_transform_fails_closed(self):
        T_arm_camera = np.eye(4)
        with self.assertRaises(VisionTargetError):
            transform_optical_point_to_arm([np.nan, 0.0, 0.5], T_arm_camera)
        with self.assertRaises(VisionTargetError):
            transform_optical_point_to_arm([0.0, 0.0], T_arm_camera)
        with self.assertRaises(VisionTargetError):
            transform_optical_point_to_arm([0.0, 0.0, 0.5], np.ones((3, 3)))

    def test_validate_sensor_estimate_for_motion(self):
        now_ns = 1_000_000_000
        valid_estimate = VisionTargetEstimate(
            schema_version="dapier.vision-target.v1",
            label="blue_cube",
            detector="mock_detector",
            source="mock_stream",
            camera_frame="os30a_optical",
            target_frame="left_arm_base",
            timestamp_ns=now_ns - 50_000_000,  # 50ms old
            position_optical_m=(0.02, 0.05, 0.40),
            position_target_m=(0.10, 0.20, 0.30),
            covariance_diagonal_m2=(1e-6, 1e-6, 1e-6),
            median_depth_m=0.40,
            detector_confidence=0.95,
            depth_confidence=0.90,
            confidence=0.92,
            mask_pixels=250,
            valid_depth_pixels=240,
            inlier_pixels=230,
        )
        # Should pass with no exception
        validate_sensor_estimate_for_motion(valid_estimate, now_monotonic_ns=now_ns)
        for bounds in ({"minimum_confidence": float("nan")}, {"max_age_ns": 0},
                       {"now_monotonic_ns": float("nan")}):
            with self.subTest(bounds=bounds), self.assertRaises(VisionTargetError):
                validate_sensor_estimate_for_motion(valid_estimate, **bounds)

        # Stale target
        with self.assertRaises(VisionTargetError) as cm:
            validate_sensor_estimate_for_motion(
                valid_estimate, now_monotonic_ns=now_ns + 5_000_000_000, max_age_ns=1_000_000_000
            )
        self.assertEqual(cm.exception.code, "stale_sensor_target")

        # Low confidence target
        low_conf = VisionTargetEstimate(
            schema_version="dapier.vision-target.v1",
            label="blue_cube",
            detector="mock_detector",
            source="mock_stream",
            camera_frame="os30a_optical",
            target_frame="left_arm_base",
            timestamp_ns=now_ns,
            position_optical_m=(0.02, 0.05, 0.40),
            position_target_m=(0.10, 0.20, 0.30),
            covariance_diagonal_m2=(1e-6, 1e-6, 1e-6),
            median_depth_m=0.40,
            detector_confidence=0.30,
            depth_confidence=0.30,
            confidence=0.30,
            mask_pixels=50,
            valid_depth_pixels=40,
            inlier_pixels=30,
        )
        with self.assertRaises(VisionTargetError) as cm:
            validate_sensor_estimate_for_motion(low_conf, now_monotonic_ns=now_ns, minimum_confidence=0.50)
        self.assertEqual(cm.exception.code, "low_confidence")

    def test_plan_pregrasp_staging_waypoints(self):
        grasp = [0.25, 0.10, 0.02]
        waypoints = plan_pregrasp_staging_waypoints(
            grasp, pregrasp_offset_m=(0.0, 0.0, 0.05), stage_offset_m=(0.0, 0.07, 0.05)
        )
        np.testing.assert_allclose(waypoints["GRASP"], [0.25, 0.10, 0.02], atol=1e-6)
        np.testing.assert_allclose(waypoints["PREGRASP_NEAR"], [0.25, 0.10, 0.07], atol=1e-6)
        np.testing.assert_allclose(waypoints["SAFE_STAGE"], [0.25, 0.17, 0.12], atol=1e-6)
        for offset in ((0., float("inf"), 0.), (0., 0.)):
            with self.assertRaises(VisionTargetError):
                plan_pregrasp_staging_waypoints(grasp, stage_offset_m=offset)

    def test_create_joint_position_intents(self):
        joints = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
        traj = [
            [0.0, -1.57, 1.57, 0.0, 0.0, 0.0],
            [0.05, -1.50, 1.50, -0.05, 0.0, 0.0],
        ]
        intents = create_joint_position_intents(
            traj,
            joints,
            start_sequence=10,
            start_monotonic_ns=1_000_000_000,
            update_period_s=0.05,
        )
        self.assertEqual(len(intents), 2)
        self.assertEqual(intents[0].schema_version, load_contract().schema_version)
        self.assertEqual(intents[0].sequence, 10)
        self.assertEqual(intents[0].kind, "arm_joint_position")
        self.assertEqual(intents[0].joint_position_rad, tuple(traj[0]))
        self.assertEqual(intents[1].sequence, 11)
        self.assertEqual(intents[1].source_monotonic_ns, 1_050_000_000)
        for period in (0., -1., float("nan"), float("inf"), 1e-12):
            with self.subTest(period=period), self.assertRaises(ValueError):
                create_joint_position_intents(traj, joints, update_period_s=period)


if __name__ == "__main__":
    unittest.main()
