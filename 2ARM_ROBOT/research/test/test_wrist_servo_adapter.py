import time
import math
import unittest
import tempfile
import hashlib
import json
from pathlib import Path
import numpy as np
from dataclasses import replace

from dapier_research.wrist_servo_adapter import (
    StaleObservationError,
    TargetLostError,
    WristObservation,
    WristServoConfig,
    WristServoError,
    compute_bounded_wrist_correction,
    wrist_correction_intent,
    block_grasp_observation_from_wrist,
    block_support_observation_from_wrist,
)
from dapier_research.real_sensor_ik_adapter import create_joint_position_intents


class TestWristServoAdapter(unittest.TestCase):
    def test_saved_wrist_producer_preserves_time_and_unknown_contact(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def source(path):
                return {"path":str(path), "sha256":hashlib.sha256(path.read_bytes()).hexdigest()}
            frame, mask, capture = root/"frame.npy", root/"mask.npy", root/"capture.json"
            np.save(frame, np.zeros((12,16,3), dtype=np.uint8))
            region = np.zeros((12,16), dtype=bool); region[3:6,5:9] = True
            np.save(mask, region)
            capture.write_text(json.dumps({"schema_version":"dapier.wrist-frame.v1", "side":"left",
                "clock":"host_monotonic_ns", "frame_acquired":True, "normal_stream_close":True,
                "timestamp_ns":123456, "host_boot_id":"MOCK-boot", "frame_path":str(frame),
                "frame_sha256":source(frame)["sha256"]}))
            document = {"capture_source":source(capture), "detection":{
                "frame_sha256":source(frame)["sha256"], "mask_source":source(mask), "label":"block",
                "confidence":.9, "detector":"MOCK-mask", "uses_privileged_labels":False}}
            kwargs = dict(run_id="MOCK-run", object_id="cube", calibration_revision="MOCK",
                          sequence=1, source_kind="mock")
            observed = block_grasp_observation_from_wrist(document, **kwargs)
            self.assertEqual(observed["captured_monotonic_ns"], 123456)
            self.assertEqual(observed["boot_id"], "MOCK-boot")
            self.assertEqual(observed["frame_sha256"], source(frame)["sha256"])
            self.assertIsNotNone(observed["visible_feature"])
            for key in ("bilateral_grasp_verified", "external_support", "bottom_clearance_lower_bound_m"):
                self.assertIsNone(observed[key])
            self.assertFalse(observed["observer_physically_verified"])
            supported = block_support_observation_from_wrist(document, support_id="MOCK-table", **kwargs)
            self.assertEqual(supported["schema_version"], "dapier.block-support-observation.v1")
            self.assertEqual(supported["captured_monotonic_ns"], 123456)
            self.assertEqual(supported["visible_feature"], observed["visible_feature"])
            self.assertIsNone(supported["approved_support_verified"])
            self.assertIsNone(supported["object_released_verified"])
            with self.assertRaises(WristServoError):
                block_support_observation_from_wrist(document, support_id="", **kwargs)
            document["detection"]["frame_sha256"] = "0"*64
            with self.assertRaises(WristServoError):
                block_grasp_observation_from_wrist(document, **kwargs)
            document.pop("detection")
            self.assertIsNone(block_grasp_observation_from_wrist(document, **kwargs)["visible_feature"])
            np.save(frame, np.ones((12,16,3), dtype=np.uint8))
            with self.assertRaises(WristServoError):
                block_grasp_observation_from_wrist(document, **kwargs)

    def test_ik_intent_through_image_correction_keeps_rad_gripper_and_limits(self):
        names = tuple(self.nominal_cmd)
        q = [math.radians(self.nominal_cmd[name]) if name != "gripper" else 1.0 for name in names]
        nominal = create_joint_position_intents([q], names, start_monotonic_ns=100,
                                                max_velocity_rad_s=.3)[0]
        obs = WristObservation(100, self.nominal_cmd, (.1, -.15), .95)
        limits = dict.fromkeys(names, (-2., 2.))
        intent = wrist_correction_intent(obs, nominal, self.config, 101, sequence=2, joint_limits_rad=limits)
        values = dict(zip(intent.joint_names, intent.joint_position_rad))
        self.assertAlmostEqual(values["wrist_flex"], math.radians(10.3))
        self.assertAlmostEqual(values["wrist_roll"], math.radians(4.8))
        self.assertEqual(values["gripper"], 1.)
        self.assertEqual(intent.joint_max_velocity_rad_s, (.3,) * 6)
        for observation, bounds, now, seq in (
            (replace(obs, feature_center_uv=None), limits, 101, 2),
            (replace(obs, measured_q={}), limits, 101, 2),
            (obs, {**limits, "wrist_flex": (-.1, .1)}, 101, 2),
            (obs, limits, 300_000_101, 2),
            (obs, limits, 101, 1),
        ):
            with self.assertRaises(WristServoError):
                wrist_correction_intent(observation, nominal, self.config, now,
                                        sequence=seq, joint_limits_rad=bounds)

    def test_nonfinite_configuration_and_nominal_state_rejected(self):
        obs = WristObservation(100, {"wrist_roll": 0.0}, (0.0, 0.0), .9)
        for config in (WristServoConfig(max_delta_deg=float("nan")),
                       WristServoConfig(min_confidence=float("nan")),
                       WristServoConfig(kp_roll=float("inf")), WristServoConfig(max_age_ns=0)):
            with self.subTest(config=config), self.assertRaises(WristServoError):
                compute_bounded_wrist_correction(obs, {"wrist_roll": 0.0}, config, 101)
        with self.assertRaises(WristServoError):
            compute_bounded_wrist_correction(obs, {"wrist_roll": float("nan")}, WristServoConfig(), 101)

    def setUp(self):
        self.config = WristServoConfig(
            target_uv=(0.0, 0.0),
            kp_roll=2.0,
            kp_flex=2.0,
            max_delta_deg=0.50,
            max_age_ns=500_000_000,  # 500 ms
            min_confidence=0.70,
        )
        self.nominal_cmd = {
            "shoulder_pan": 0.0,
            "shoulder_lift": -45.0,
            "elbow_flex": 60.0,
            "wrist_flex": 10.0,
            "wrist_roll": 5.0,
            "gripper": 20.0,
        }

    def test_bounded_correction_in_mock_trace(self):
        now_ns = time.monotonic_ns()
        # Simulated observation with target slightly off-center (u=0.10, v=-0.15)
        obs = WristObservation(
            timestamp_ns=now_ns - 50_000_000,  # 50ms old
            measured_q=dict(self.nominal_cmd),
            feature_center_uv=(0.10, -0.15),
            confidence=0.95,
        )

        corrected = compute_bounded_wrist_correction(obs, self.nominal_cmd, self.config, now_ns)

        # err_u = 0.0 - 0.10 = -0.10 -> delta_roll = 2.0 * (-0.10) = -0.20 deg (within 0.50)
        # err_v = 0.0 - (-0.15) = +0.15 -> delta_flex = 2.0 * (+0.15) = +0.30 deg (within 0.50)
        self.assertAlmostEqual(corrected["wrist_roll"], 5.0 - 0.20, places=2)
        self.assertAlmostEqual(corrected["wrist_flex"], 10.0 + 0.30, places=2)
        # Unaffected joints remain unchanged
        self.assertEqual(corrected["shoulder_pan"], 0.0)
        self.assertEqual(corrected["elbow_flex"], 60.0)

    def test_saturation_at_max_delta(self):
        now_ns = time.monotonic_ns()
        # Large error (u=0.8, v=-0.9) that would exceed 0.50 deg
        obs = WristObservation(
            timestamp_ns=now_ns - 10_000_000,
            measured_q=dict(self.nominal_cmd),
            feature_center_uv=(0.8, -0.9),
            confidence=0.90,
        )

        corrected = compute_bounded_wrist_correction(obs, self.nominal_cmd, self.config, now_ns)
        # Clamped to -0.50 and +0.50
        self.assertAlmostEqual(corrected["wrist_roll"], 5.0 - 0.50, places=2)
        self.assertAlmostEqual(corrected["wrist_flex"], 10.0 + 0.50, places=2)

    def test_stale_observation_rejected(self):
        now_ns = time.monotonic_ns()
        # 600ms old (> max_age 500ms)
        obs = WristObservation(
            timestamp_ns=now_ns - 600_000_000,
            measured_q=dict(self.nominal_cmd),
            feature_center_uv=(0.05, 0.05),
            confidence=0.90,
        )
        with self.assertRaises(StaleObservationError):
            compute_bounded_wrist_correction(obs, self.nominal_cmd, self.config, now_ns)

    def test_lost_target_rejected(self):
        now_ns = time.monotonic_ns()
        # Feature is None
        obs_none = WristObservation(
            timestamp_ns=now_ns - 20_000_000,
            measured_q=dict(self.nominal_cmd),
            feature_center_uv=None,
            confidence=0.0,
        )
        with self.assertRaises(TargetLostError):
            compute_bounded_wrist_correction(obs_none, self.nominal_cmd, self.config, now_ns)

        # Low confidence
        obs_low_conf = WristObservation(
            timestamp_ns=now_ns - 20_000_000,
            measured_q=dict(self.nominal_cmd),
            feature_center_uv=(0.0, 0.0),
            confidence=0.40,  # < min_confidence 0.70
        )
        with self.assertRaises(TargetLostError):
            compute_bounded_wrist_correction(obs_low_conf, self.nominal_cmd, self.config, now_ns)


if __name__ == "__main__":
    unittest.main()
