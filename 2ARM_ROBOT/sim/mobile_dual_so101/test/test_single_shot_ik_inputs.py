"""Offline regressions for forged/stale seeds and model-limit clipping."""
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluate_single_shot_ik import (JOINTS, candidate_seed, load_measured_state,
    target_in_model_base, check_native_feedback_endpoint, fingerprint, bind_observed_block,
    load_wrist_correction, evaluate)


class SingleShotInputsTest(unittest.TestCase):
    def test_saved_wrist_features_bind_to_measured_start_and_preserve_gripper(self):
        from integration_scenes import task_env
        env = task_env("desk")
        seed = np.zeros(12)
        seed[5] = .7
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            frame = root / "MOCK-frame.bin"
            frame.write_bytes(b"MOCK image evidence, no device")
            path = root / "wrist.json"
            source = {"sha256":"a" * 64}
            observation = {"schema_version":"dapier.wrist-observation.v1", "side":"left",
                "clock":"host_monotonic_ns", "timestamp_ns":100,
                "measured_state_sha256":source["sha256"], "measured_q_model_rad":seed[:6].tolist(),
                "frame_source":fingerprint(frame), "feature_center_uv":[.15,-.15],
                "target_uv":[0.,0.], "confidence":.95}
            path.write_text(json.dumps(observation))
            q, intent, evidence = load_wrist_correction(path, env.model, seed, source, now_ns=101)
            self.assertAlmostEqual(q[3], np.deg2rad(.3))
            self.assertAlmostEqual(q[4], np.deg2rad(-.3))
            self.assertEqual(q[5], .7)
            np.testing.assert_array_equal(q[6:], seed[6:])
            self.assertEqual(intent.source, "wrist_servo_adapter")
            self.assertEqual(evidence["sha256"], fingerprint(path)["sha256"])
            for change in ({"measured_state_sha256":"b" * 64},
                           {"measured_q_model_rad":[0.] * 6},
                           {"timestamp_ns":102}, {"timestamp_ns":-2_000_000_000},
                           {"feature_center_uv":None}, {"feature_center_uv":[float("nan"),0.]},
                           {"side":"right"}, {"clock":"unix_ns"}):
                path.write_text(json.dumps({**observation, **change}))
                with self.subTest(change=change), self.assertRaises(ValueError):
                    load_wrist_correction(path, env.model, seed, source, now_ns=101)
            path.write_text(json.dumps(observation))
            frame.write_bytes(b"changed")
            with self.assertRaisesRegex(ValueError, "source frame changed"):
                load_wrist_correction(path, env.model, seed, source, now_ns=101)

    def test_wrist_route_requires_real_measured_start_before_any_solver(self):
        with mock.patch("evaluate_single_shot_ik.solve_bimanual_position_ik") as solver:
            with self.assertRaisesRegex(ValueError, "measured path start"):
                evaluate(SimpleNamespace(wrist_observation_json=Path("unused.json"), sim_home_seed=True))
            solver.assert_not_called()

    def test_wrist_evaluation_reaches_fk_path_gate_without_running_ik(self):
        from integration_scenes import task_env
        from mobile_dual_so101 import apply_control_as_pose
        env = task_env("desk")
        seed = np.zeros(12)
        seed[5] = .7
        goal = seed.copy()
        goal[3:5] = np.deg2rad([.3, -.3])
        apply_control_as_pose(env.model, env.data, goal)
        base = env.data.body("left_base")
        target = base.xmat.reshape(3, 3).T @ (env.data.site("left_cube_grasp").xpos - base.xpos)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            frame = root / "MOCK-frame.bin"
            frame.write_bytes(b"MOCK")
            wrist = root / "wrist.json"
            source = {"sha256":"a" * 64, "timestamp":datetime.now(timezone.utc).isoformat()}
            wrist.write_text(json.dumps({"schema_version":"dapier.wrist-observation.v1",
                "side":"left", "clock":"host_monotonic_ns", "timestamp_ns":100,
                "measured_state_sha256":source["sha256"], "measured_q_model_rad":seed[:6].tolist(),
                "frame_source":fingerprint(frame), "feature_center_uv":[.15,-.15],
                "target_uv":[0.,0.], "confidence":.95}))
            block = root / "MOCK-block.json"
            block.write_text(json.dumps({"arm_mapping":{"candidate_frame":"left_motor1_datum"},
                "target_arm_xyz_candidate_m":target.tolist(), "rgb_timestamp_ns":time.time_ns(),
                "scene_object":{"frame":"left_motor1_datum", "position_semantics":"object_center",
                    "center_xyz_m":[.12,-.05,0.], "size_m":[.04]*3, "quaternion_wxyz":[1.,0.,0.,0.]}}))
            args = SimpleNamespace(sim_home_seed=False, wrist_observation_json=wrist,
                measured_state_json=root / "left", right_measured_state_json=root / "right",
                left_calibration=root / "left-cal", right_calibration=root / "right-cal",
                block_json=block, model=Path(os.environ["DAPIER_SO101_MJCF"]),
                motor_datum_in_base_m=[0.,0.,0.], pregrasp_offset_z=0.)
            with (mock.patch("evaluate_single_shot_ik.load_measured_state", return_value=(np.zeros(6),source)),
                  mock.patch("evaluate_single_shot_ik.candidate_seed", return_value=seed),
                  mock.patch("evaluate_single_shot_ik.task_env", return_value=env),
                  mock.patch("evaluate_single_shot_ik.time.monotonic_ns", return_value=101),
                  mock.patch("evaluate_single_shot_ik.solve_bimanual_position_ik") as solver,
                  mock.patch("evaluate_single_shot_ik.check_bimanual_path",
                      return_value=SimpleNamespace(safe=True, reason="MOCK path", minimum_clearance_m=.05)) as path):
                result = evaluate(args)
            solver.assert_not_called()
            np.testing.assert_array_equal(path.call_args.args[1], seed)
            np.testing.assert_allclose(path.call_args.args[2], goal)
            self.assertEqual(result["candidate_mode"], "wrist_feedback")
            self.assertEqual(result["goal_intent"]["source"], "wrist_servo_adapter")
            self.assertLess(result["position_error_m"], 1e-12)
            self.assertGreater(result["tool_axis_error_rad_by_side"]["left"], np.deg2rad(2))
            self.assertFalse(result["offline_candidate_accepted"])

    def test_observed_center_replaces_nominal_collision_block_not_approach_target(self):
        from integration_scenes import task_env
        from mobile_dual_so101 import apply_control_as_pose
        env = task_env("desk")
        model, data = env.model, env.data
        nominal = data.body("red_block").xpos.copy()
        observed = {"frame":"left_motor1_datum", "position_semantics":"object_center",
                    "center_xyz_m":[.12, -.05, -.0054], "size_m":[.04]*3,
                    "quaternion_wxyz":[1.,0.,0.,0.]}
        block = {"scene_object":observed, "target_arm_xyz_candidate_m":[.12,-.05,.0546]}
        result = bind_observed_block(model, data, block, [.0388353,0.,.0254])
        expected = data.body("left_base").xpos + data.body("left_base").xmat.reshape(3,3) @ np.array([.1588353,-.05,.02])
        np.testing.assert_allclose(data.body("red_block").xpos, expected)
        self.assertGreater(np.linalg.norm(expected - nominal), .01)
        apply_control_as_pose(model, data, np.zeros(12))
        np.testing.assert_allclose(data.body("red_block").xpos, expected)
        self.assertFalse(result["uncertainty_covered_by_path_envelope"])
        for field, value in (("position_semantics","surface"), ("frame","camera"),
                             ("size_m",[.025]*3), ("quaternion_wxyz",[2.,0.,0.,0.]),
                             ("center_xyz_m",[0.,0.,float("nan")])):
            with self.subTest(field=field), self.assertRaises(ValueError):
                bind_observed_block(model, data, {"scene_object":{**observed,field:value}}, [.0388353,0.,.0254])

    def test_native_feedback_position_match_does_not_discard_axis(self):
        from integration_scenes import task_env, portable_model_sha256
        from mobile_dual_so101 import apply_control_as_pose
        env = task_env("desk")
        model, data = env.model, env.data
        q = np.zeros(12)
        apply_control_as_pose(model, data, q)
        candidate = {"model":{**fingerprint(Path(os.environ["DAPIER_SO101_MJCF"])),
                              "compiled_sha256":portable_model_sha256(model)},
                     "seed_posture":{"seed_q_rad":q.tolist()},
                     "kinematic_analysis":{"target_world_xyz_m":data.site("left_cube_grasp").xpos.tolist()},
                     "mapping":{"physically_verified":True}}
        result = check_native_feedback_endpoint(candidate,
            {"reached_joint_endpoint":True,"final_measured_rad":q[:6].tolist(),"hardware_execution":False})
        self.assertLess(result["position_error_m"], 1e-12)
        self.assertGreater(result["axis_error_rad"], np.deg2rad(2))
        self.assertFalse(result["kinematic_endpoint_within_tolerance"])
        self.assertFalse(result["cartesian_endpoint_verified"])
        self.assertFalse(result["task_success"])

    def test_actual_readonly_writer_to_loader_without_hardware(self):
        from test_dual_so101_smoke import SMOKE, FakeBus, write_profile
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = write_profile(root)
            path = root / "snapshot.json"
            with (mock.patch.dict(SMOKE["main"].__globals__, {
                    "load_bus": lambda *_: FakeBus(),
                    "controller_serial": lambda port: f"synthetic-controller-{Path(port).name}"}),
                  mock.patch.object(SMOKE["os"], "isatty", return_value=True),
                  mock.patch("builtins.print"),
                  mock.patch("sys.argv", ["dual_so101_smoke", "--profile", str(profile),
                      "--log", str(path), "--operator-present", "--confirm", SMOKE["READONLY_CONFIRMATION"]])):
                self.assertEqual(SMOKE["main"](), 0)
            record = json.loads(path.read_text())
            now = datetime.fromisoformat(record["finished_at"]).timestamp() + 1
            for side in ("left", "right"):
                values, evidence = load_measured_state(path, root / f"{side}.json", side, now_s=now)
                self.assertEqual(values.tolist(), [-180] * 5 + [0])
                self.assertFalse(evidence["connected_endpoint_identity_bound"])
                self.assertEqual(evidence["reader_source_sha256"], record["source_sha256"])
            mutations = [
                lambda r: r.update(schema_version="dapier.dual-so101-smoke.v0.4"),
                lambda r: r.update(error="read failed"),
                lambda r: r["arms"]["left"].update(calibration_sha256="wrong"),
                lambda r: r["arms"]["left"]["position_raw_tick"].update(elbow_flex=4096),
                lambda r: r["arms"]["left"].update(position_read_duration_ns=-1),
                lambda r: r["controller_identity_revalidated"].update(left=False),
            ]
            for mutate in mutations:
                bad = copy.deepcopy(record)
                mutate(bad)
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):
                    load_measured_state(path, root / "left.json", "left", now_s=now)
            path.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, "stale"):
                load_measured_state(path, root / "left.json", "left", now_s=now + 61)

    def test_motor_datum_is_not_silently_used_as_model_base(self):
        block = {"arm_mapping": {"candidate_frame": "left_motor1_datum"},
                 "target_arm_xyz_candidate_m": [.224, -.145, 0.]}
        np.testing.assert_allclose(target_in_model_base(block, [.0388353, 0., .0254]),
                                   [.2628353, -.145, .0254])
        for offset in (None, [0., 0., float("nan")]):
            with self.assertRaises(ValueError):
                target_in_model_base(block, offset)
        with self.assertRaises(ValueError):
            target_in_model_base({**block, "arm_mapping": {}}, [0., 0., 0.])

    def test_complete_sample_and_negative_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.jsonl"
            cal = Path(directory) / "calibration.json"
            cal.write_text(json.dumps({j: {"range_min": 1000, "range_max": 3000} for j in JOINTS}))
            record = {"timestamp": datetime.fromtimestamp(1000, timezone.utc).isoformat(),
                "role": "follower", "device_id": "dapier_dual_follower_left",
                "calibration_sha256": hashlib.sha256(cal.read_bytes()).hexdigest(),
                "raw_ticks": dict.fromkeys(JOINTS, 2000),
                "calibrated_position": {j: 50 if j == "gripper" else 0 for j in JOINTS},
                "units": {j: "range_0_100" if j == "gripper" else "degrees (mid-relative)" for j in JOINTS}}
            path.write_text(json.dumps(record))
            values, evidence = load_measured_state(path, cal, "left", now_s=1001)
            self.assertEqual(values.tolist(), [0, 0, 0, 0, 0, 50])
            self.assertEqual(evidence["age_s"], 1)
            for now in (999, 1061, float("nan")):
                with self.subTest(now=now), self.assertRaises(ValueError):
                    load_measured_state(path, cal, "left", now_s=now)
            bad = []
            for field, key, value in (("raw_ticks", "elbow_flex", 3161),
                                      ("calibrated_position", "gripper", float("nan")),
                                      ("calibrated_position", "elbow_flex", 1),
                                      ("units", "wrist_roll", "rad")):
                item = copy.deepcopy(record)
                item[field][key] = value
                bad.append(item)
            for field, value in (("calibration_sha256", "wrong"), ("role", "leader"),
                                 ("device_id", "dapier_dual_follower_right")):
                item = copy.deepcopy(record)
                item[field] = value
                bad.append(item)
            for item in bad:
                path.write_text(json.dumps(item))
                with self.subTest(record=item), self.assertRaises(ValueError):
                    load_measured_state(path, cal, "left", now_s=1001)

    def test_pgripper_range_and_no_seed_clipping(self):
        ranges = np.tile([-1.69, 1.69], (12, 1))
        ranges[[5, 11]] = [0, 2.2028]
        model = SimpleNamespace(nu=12, actuator_ctrlrange=ranges,
            actuator_trnid=np.column_stack((np.arange(12), np.zeros(12))), jnt_range=ranges,
            actuator=lambda i: SimpleNamespace(name=f"joint_{i}"))
        profile = {"arm_signs": [1] * 10, "arm_zero_offsets_deg": [0] * 10}
        left = np.array([0, 0, 0, 0, 0, 100.])
        right = np.zeros(6)
        seed = candidate_seed(model, left, right, profile)
        self.assertEqual(seed[5], 2.2028)
        self.assertEqual(seed[11], 0)
        left[2] = 98.7252747
        with self.assertRaisesRegex(ValueError, "not clipped"):
            candidate_seed(model, left, right, profile)
        self.assertEqual(left[2], 98.7252747)


if __name__ == "__main__":
    unittest.main()
