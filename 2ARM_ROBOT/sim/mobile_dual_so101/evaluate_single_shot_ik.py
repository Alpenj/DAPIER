#!/usr/bin/env python3
"""Offline IK candidate audit; never authorizes hardware execution.

Both complete readbacks are required. Measured values are never clipped or
replaced by defaults. The desk profile's physical joint/gripper mapping remains
unverified, even if an offline candidate converges.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "research/src"))
from dapier_research.real_sensor_ik_adapter import load_block_observation

from collision_guard import (DEFAULT_CLEARANCE_M, TASK_GENERAL_QUERY_CAP_M, NEAR_SUPPORT_SCOPE,
    check_bimanual_path, _carried_object_attachment, _apply_carried_object)
from integration_scenes import task_env, portable_model_sha256
from mobile_dual_so101 import apply_control_as_pose, HUMANOID_HOME_ACTION
from pgripper import home_action
from physics_ik import solve_bimanual_position_ik
from replay_recorded_episode import finite_vector, mapping

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def fingerprint(path):
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def load_measured_state(path, calibration_path, side, *, now_s, max_age_s=60.0):
    """Read one complete sample, never combine joints from different acquisitions."""
    if not math.isfinite(now_s) or not math.isfinite(max_age_s) or max_age_s <= 0:
        raise ValueError("invalid readback freshness bounds")
    if side not in ("left", "right"):
        raise ValueError("readback side must be left or right")
    text = path.read_text()
    try:
        document = json.loads(text)
        records = [document]
    except json.JSONDecodeError:
        records = [json.loads(line) for line in text.splitlines() if line.strip()]
    if not records:
        raise ValueError(f"{side}: empty readback")
    record = records[-1]
    snapshot = record if "schema_version" in record else None
    if snapshot is not None:
        if (snapshot.get("schema_version") != "dapier.dual-so101-smoke.v0.5"
                or snapshot.get("trusted_profile_verified") is not True
                or snapshot.get("controller_identity_revalidated", {}).get(side) is not True
                or snapshot.get("motion_enabled") is not False
                or "error" in snapshot or snapshot.get("disconnect_errors")):
            raise ValueError(f"{side}: incomplete or unsupported read-only snapshot")
        arm = snapshot["arms"][side]
        duration = arm.get("position_read_duration_ns")
        if type(duration) is not int or not 0 <= duration <= max_age_s * 1e9:
            raise ValueError(f"{side}: invalid read interval")
        ended = datetime.fromisoformat(arm["position_finished_at"])
        begun = datetime.fromisoformat(arm["position_started_at"])
        if (ended.tzinfo is None or begun.tzinfo is None
                or not begun.timestamp() <= ended.timestamp() <= now_s):
            raise ValueError(f"{side}: invalid read interval timestamps")
        record = {**arm, "timestamp": arm["position_started_at"],
                  "raw_ticks": arm["position_raw_tick"]}
    if record.get("role") != "follower" or record.get("device_id") != f"dapier_dual_follower_{side}":
        raise ValueError(f"{side}: readback identity mismatch")
    captured = datetime.fromisoformat(record["timestamp"])
    if captured.tzinfo is None:
        raise ValueError(f"{side}: timezone is missing")
    age_s = now_s - captured.timestamp()
    if not 0 <= age_s <= max_age_s:
        raise ValueError(f"{side}: stale or future readback ({age_s:.3f} s)")
    cal_bytes = calibration_path.read_bytes()
    if record.get("calibration_sha256") != hashlib.sha256(cal_bytes).hexdigest():
        raise ValueError(f"{side}: calibration hash mismatch")
    calibration = json.loads(cal_bytes)
    values = np.empty(6) if snapshot is not None else finite_vector(
        [record["calibrated_position"][j] for j in JOINTS], 6, f"{side} measured state")
    for index, joint in enumerate(JOINTS):
        expected_unit = "range_0_100" if joint == "gripper" else "degrees (mid-relative)"
        if snapshot is None and record["units"].get(joint) != expected_unit:
            raise ValueError(f"{side}/{joint}: unit mismatch")
        raw = record["raw_ticks"][joint]
        lo, hi = calibration[joint]["range_min"], calibration[joint]["range_max"]
        if (type(raw) is not int or not math.isfinite(lo) or not math.isfinite(hi)
                or not lo < hi or not lo <= raw <= hi):
            raise ValueError(f"{side}/{joint}: measured tick {raw} outside calibration [{lo}, {hi}]")
        # Homing offsets are already applied by the servo before this LeRobot conversion.
        expected = ((raw - lo) * 100 / (hi - lo) if joint == "gripper"
                    else (raw - (lo + hi) / 2) * 360 / 4095)
        if snapshot is not None:
            values[index] = expected
        elif not math.isclose(values[index], expected, rel_tol=0, abs_tol=1e-6):
            raise ValueError(f"{side}/{joint}: raw/calibrated readback mismatch")
    return values, {**fingerprint(path), "timestamp": record["timestamp"], "age_s": age_s,
                    "calibration": fingerprint(calibration_path), "raw_ticks": record["raw_ticks"],
                    "calibrated_position": values.tolist(),
                    "connected_endpoint_identity_bound": snapshot.get("connected_endpoint_identity_bound", False) if snapshot else False,
                    "reader_source_sha256": snapshot.get("source_sha256") if snapshot else None}


def validate_model_action(model, action, label):
    values = finite_vector(action, model.nu, label)
    joints = model.actuator_trnid[:, 0].astype(int)
    lo = np.maximum(model.actuator_ctrlrange[:, 0], model.jnt_range[joints, 0])
    hi = np.minimum(model.actuator_ctrlrange[:, 1], model.jnt_range[joints, 1])
    outside = np.flatnonzero((values < lo) | (values > hi))
    if outside.size:
        names = [model.actuator(int(i)).name for i in outside]
        raise ValueError(f"{label} outside model limits: {names}; values were not clipped")


def candidate_seed(model, left, right, profile):
    """Unverified mapping for diagnostics; use this model's actual gripper ranges."""
    values = finite_vector(np.concatenate((left, right)), 12, "bimanual measured state")
    signs, offsets = mapping(profile)
    arm = [0, 1, 2, 3, 4, 6, 7, 8, 9, 10]
    values[arm] = np.deg2rad(values[arm] * signs + offsets)
    for i in (5, 11):
        if not 0 <= values[i] <= 100:
            raise ValueError("gripper percentage outside [0, 100]")
        lo, hi = model.actuator_ctrlrange[i]
        values[i] = lo + values[i] / 100 * (hi - lo)
    validate_model_action(model, values, "measured candidate seed")
    return values


def target_in_model_base(block, motor_datum_in_base_m):
    """Translate the measured datum frame; never silently identify it with base."""
    if block.get("arm_mapping", {}).get("candidate_frame") != "left_motor1_datum":
        raise ValueError("explicit left_motor1_datum target frame required")
    if motor_datum_in_base_m is None:
        raise ValueError("audited motor datum offset in model base is required")
    target = finite_vector(block["target_arm_xyz_candidate_m"], 3, "motor-datum target")
    offset = finite_vector(motor_datum_in_base_m, 3, "motor datum in model base")
    # Candidate axes are parallel to left_base; physical rotation is still unverified.
    return target + offset


def load_staging_reference(path, block, motor_datum_in_base_m):
    """Reuse a relative waypoint and IK seed, never its old absolute target/q path."""
    reference = json.loads(path.read_text())
    if (reference.get("schema_version") != "dapier.sensor-staging-reference.v1"
            or reference.get("frame") != "left_base" or reference.get("phase") != "ALIGN_HIGH"
            or block["scene_object"].get("frame") != "left_motor1_datum"):
        raise ValueError("explicit left-base ALIGN_HIGH reference and observed object frame required")
    offset = finite_vector(reference["center_to_stage_offset_m"], 3, "relative staging offset")
    if np.any(np.abs(offset) > .15) or offset[2] < 0:
        raise ValueError("staging offset outside bounded task region")
    center = finite_vector(block["scene_object"]["center_xyz_m"], 3, "observed object center")
    datum = finite_vector(motor_datum_in_base_m, 3, "motor datum")
    seed = finite_vector(reference["ik_seed_rad"], 12, "staging IK seed")
    return center + datum + offset, seed, fingerprint(path)


def load_carry_reference(path, measured_tcp_world_m, observed_center_world_m):
    """Vertical task displacement preserves the measured TCP/object separation."""
    reference = json.loads(path.read_text())
    phase = reference.get("phase")
    if (reference.get("schema_version") != "dapier.sensor-carry-reference.v1"
            or reference.get("frame") != "model_world" or phase not in ("LIFT", "PLACE")):
        raise ValueError("explicit world-frame LIFT/PLACE carry reference required")
    dz = reference.get("translation_z_m")
    if (type(dz) not in (int, float) or not math.isfinite(dz) or not 0 < abs(dz) <= .10
            or (dz > 0) != (phase == "LIFT")):
        raise ValueError("carry displacement must be signed vertical metres within0.10m")
    tcp = finite_vector(measured_tcp_world_m, 3, "measured FK TCP")
    center = finite_vector(observed_center_world_m, 3, "observed object center")
    shift = np.array([0., 0., dz])
    return tcp + shift, {"phase":phase, "reference":fingerprint(path),
        "translation_world_m":shift.tolist(), "desired_object_goal_center_world_m":(center+shift).tolist(),
        "tcp_minus_object_center_world_m":(tcp-center).tolist(),
        "object_motion_model":"rigid TCP-relative attachment hypothesis; not observed attachment",
        "contact_path_verified":False}


def bind_observed_block(model, data, block, motor_datum_in_base_m):
    """Place the observed object in the collision scene, separately from its TCP goal."""
    observed = block["scene_object"]
    if (observed.get("frame") != "left_motor1_datum"
            or observed.get("position_semantics") != "object_center"):
        raise ValueError("collision object requires explicit motor-datum object center")
    center = finite_vector(observed["center_xyz_m"], 3, "observed object center")
    size = finite_vector(observed["size_m"], 3, "observed object dimensions")
    quat = finite_vector(observed["quaternion_wxyz"], 4, "observed object orientation")
    if not np.allclose(size, [.04] * 3, rtol=0, atol=1e-9):
        raise ValueError("observed object must be the confirmed 4 cm cube")
    if not math.isclose(float(np.linalg.norm(quat)), 1., rel_tol=0, abs_tol=1e-6):
        raise ValueError("observed object quaternion must be unit length")
    geom = model.geom("red_block_geom")
    if (geom.type != mujoco.mjtGeom.mjGEOM_BOX
            or not np.allclose(2 * geom.size, size, rtol=0, atol=1e-9)):
        raise ValueError("collision geometry differs from observed object dimensions")
    datum = finite_vector(motor_datum_in_base_m, 3, "motor datum in model base")
    mujoco.mj_forward(model, data)
    base = data.body("left_base")
    rotation = base.xmat.reshape(3, 3)
    world = base.xpos + rotation @ (center + datum)
    base_quat, world_quat = np.empty(4), np.empty(4)
    mujoco.mju_mat2Quat(base_quat, rotation.ravel())
    mujoco.mju_mulQuat(world_quat, base_quat, quat)
    address = int(model.joint("red_block_free").qposadr[0])
    # Never derive the observed center from the simulator or the offset approach TCP.
    data.qpos[address:address + 7] = np.r_[world, world_quat]
    mujoco.mj_forward(model, data)
    return {"bound_to_path_reference": True, "source": observed,
            "center_world_m": world.tolist(), "quaternion_world_wxyz": world_quat.tolist(),
            "uncertainty_covered_by_path_envelope": False}


def observed_task_env(block, motor_datum_in_base_m):
    """Compile the measured horizontal support in the existing desk scene."""
    support = block["scene_support"]
    if (not isinstance(support, dict) or support.get("frame") != "left_motor1_datum"
            or support.get("normal_xyz") != [0., 0., 1.]
            or not isinstance(support.get("revision"), str) or not support["revision"].strip()):
        raise ValueError("support requires a revision and horizontal motor-datum plane")
    top = float(support["top_z_m"])
    if not math.isfinite(top):
        raise ValueError("support height must be finite")
    datum = finite_vector(motor_datum_in_base_m, 3, "motor datum in model base")
    # Existing desk left_base has world Z=0 and a vertical +Z axis. Check that
    # assumption explicitly; a tilted/new mount needs its measured transform.
    world_z = top + float(datum[2])
    env = task_env("desk", table_top_z_m=world_z)
    base = env.data.body("left_base")
    if (not math.isclose(float(base.xpos[2]), 0., abs_tol=1e-9)
            or not np.allclose(base.xmat.reshape(3, 3)[:, 2], [0., 0., 1.], rtol=0, atol=1e-9)):
        raise ValueError("observed support needs the changed base transform")
    return env, {"bound_to_path_reference": True, "source": support,
                 "top_world_z_m": world_z, "motor_datum_in_model_base_m": datum.tolist(),
                 "near_support_policy": "general_clearance_no_sim_exception",
                 "extent_source": "existing desk profile; unchanged nominal XY extent"}


def check_native_feedback_endpoint(candidate, native_result):
    """Evaluate returned measured joints with the same model/TCP/down axis, no IK.

    This is endpoint evidence only. It does not observe contact, lift, or HOLD.
    The passive right arm retains the plan's measured sample and is labelled as such.
    """
    if native_result.get("reached_joint_endpoint") is not True:
        raise ValueError("native measured endpoint was not reached")
    model_path = Path(candidate["model"]["path"])
    if fingerprint(model_path)["sha256"] != candidate["model"]["sha256"]:
        raise ValueError("endpoint model differs from planned model")
    os.environ["DAPIER_SO101_MJCF"] = str(model_path.resolve(strict=True))
    support = candidate.get("scene_support")
    if support is not None:
        source = candidate["block_source"]
        if fingerprint(Path(source["path"]))["sha256"] != source["sha256"]:
            raise ValueError("endpoint observation differs from planned observation")
        block = load_block_observation(Path(source["path"]))
        env, rebuilt = observed_task_env(block, support["motor_datum_in_model_base_m"])
        if rebuilt != support:
            raise ValueError("endpoint support differs from checked support")
    else:
        env = task_env("desk")
    model, data = env.model, env.data
    if portable_model_sha256(model) != candidate["model"]["compiled_sha256"]:
        raise ValueError("compiled endpoint model differs from planned model")
    q = finite_vector(candidate["seed_posture"]["seed_q_rad"], 12, "passive arm reference").copy()
    q[:6] = finite_vector(native_result["final_measured_rad"], 6, "native measured left arm")
    validate_model_action(model, q, "native endpoint feedback")
    apply_control_as_pose(model, data, q)
    target = finite_vector(candidate["kinematic_analysis"]["target_world_xyz_m"], 3, "checked target")
    site = data.site("left_cube_grasp")
    position_error = float(np.linalg.norm(site.xpos - target))
    # physics_ik uses site local +X as the approach axis, not local +Z.
    axis = site.xmat.reshape(3, 3)[:, 0]
    axis_error = math.acos(float(np.clip(np.dot(axis, [0., 0., -1.]), -1., 1.)))
    within = position_error <= .0005 and axis_error <= math.radians(2.)
    return {"tcp_world_m":site.xpos.tolist(),"target_world_m":target.tolist(),
        "position_error_m":position_error,"axis_error_rad":axis_error,
        "kinematic_endpoint_within_tolerance":within,
        "cartesian_endpoint_verified":within and candidate["mapping"].get("physically_verified") is True
            and native_result.get("hardware_execution") is True,
        "passive_right_state":"plan readback, not a new native right-arm measurement",
        "hardware_execution":native_result.get("hardware_execution") is True,"task_success":False}


def load_wrist_correction(path, model, seed, measured_source, *, now_ns):
    """Bind saved wrist features to the measured seed; propose, never dispatch."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "research/src"))
    from dapier_research.control_intent import arm_joint_position_intent
    from dapier_research.wrist_servo_adapter import (
        WristObservation, WristServoConfig, WristServoError, wrist_correction_intent,
        wrist_observation_from_detection,
        bind_native_wrist_observation,
    )
    raw = path.read_bytes()
    observation = json.loads(raw)
    if isinstance(observation, dict) and observation.get("schema_version") == "dapier.wrist-observation.v2":
        try:
            observation = bind_native_wrist_observation(observation, measured_source, seed[:6], now_ns=now_ns)
        except WristServoError as exc:
            raise ValueError(str(exc)) from exc
    if (not isinstance(observation, dict)
            or observation.get("schema_version") != "dapier.wrist-observation.v1"
            or observation.get("side") != "left"
            or observation.get("clock") != "host_monotonic_ns"
            or observation.get("measured_state_sha256") != measured_source["sha256"]):
        raise ValueError("wrist observation frame/clock/readback binding mismatch")
    measured = finite_vector(observation["measured_q_model_rad"], 6, "wrist measured model radians")
    if not np.allclose(measured, seed[:6], rtol=0, atol=1e-9):
        raise ValueError("wrist exposure state differs from measured path start")
    frame = observation["frame_source"]
    if fingerprint(Path(frame["path"]))["sha256"] != frame["sha256"]:
        raise ValueError("wrist source frame changed")
    nominal = arm_joint_position_intent(
        sequence=1, source="fresh_measured_state", source_monotonic_ns=now_ns,
        ttl_ns=250_000_000, joint_names=JOINTS, joint_position_rad=tuple(measured),
        joint_max_velocity_rad_s=(.3,) * 6,
    )
    joint_ids = model.actuator_trnid[:6, 0].astype(int)
    lo = np.maximum(model.actuator_ctrlrange[:6, 0], model.jnt_range[joint_ids, 0])
    hi = np.minimum(model.actuator_ctrlrange[:6, 1], model.jnt_range[joint_ids, 1])
    # Exposure state is measured input, not the proposed command or copied feedback.
    measured_deg = {name: math.degrees(measured[i]) for i, name in enumerate(JOINTS[:5])}
    mask_source = None
    try:
        if "detection" in observation:
            import cv2
            from dapier_research.vision_target import PixelDetection
            if "feature_center_uv" in observation or "confidence" in observation:
                raise ValueError("supply either a detection mask or explicit wrist features")
            detection = observation["detection"]
            mask_source = detection["mask_source"]
            if fingerprint(Path(mask_source["path"]))["sha256"] != mask_source["sha256"]:
                raise ValueError("wrist detection mask changed")
            frame_path = Path(frame["path"])
            image = (np.load(frame_path, allow_pickle=False) if frame_path.suffix == ".npy"
                     else cv2.imread(str(frame_path), cv2.IMREAD_COLOR))
            if image is None or image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
                raise ValueError("wrist source must be an HxWx3 uint8 image")
            obs = wrist_observation_from_detection(PixelDetection(
                detection["label"], np.load(mask_source["path"], allow_pickle=False),
                detection["confidence"], detection["detector"], detection["uses_privileged_labels"]),
                image_shape=image.shape[:2], timestamp_ns=observation["timestamp_ns"],
                measured_q=measured_deg)
        else:
            obs = WristObservation(observation["timestamp_ns"], measured_deg,
                observation["feature_center_uv"], observation["confidence"])
        intent = wrist_correction_intent(obs, nominal,
            WristServoConfig(target_uv=tuple(observation["target_uv"])), now_ns,
            sequence=2, joint_limits_rad=dict(zip(JOINTS, zip(lo, hi))))
    except WristServoError as exc:
        raise ValueError(str(exc)) from exc
    action = seed.copy()
    action[:6] = intent.joint_position_rad
    return action, intent, {"path":str(path), "sha256":hashlib.sha256(raw).hexdigest(),
                            "frame_source":frame,
                            **({"capture_source":observation["capture_source"],
                                "time_association":observation["time_association"]}
                               if "capture_source" in observation else {}),
                            **({"mask_source":mask_source} if mask_source else {})}


def evaluate(args):
    now_s = time.time()
    wrist_path = getattr(args, "wrist_observation_json", None)
    carry_path = getattr(args, "carry_reference", None)
    if carry_path is not None and (args.sim_home_seed or wrist_path is not None
                                   or getattr(args, "staging_reference", None) is not None):
        raise ValueError("carry planning requires measured start and a separate carry phase")
    if wrist_path is not None and args.sim_home_seed:
        raise ValueError("wrist feedback requires a measured path start")
    if not args.sim_home_seed:
        if args.measured_state_json is None or args.right_measured_state_json is None:
            raise ValueError("both complete readbacks are required unless --sim-home-seed is explicit")
        left, left_source = load_measured_state(args.measured_state_json, args.left_calibration, "left", now_s=now_s)
        right, right_source = load_measured_state(args.right_measured_state_json, args.right_calibration, "right", now_s=now_s)
        if abs(datetime.fromisoformat(left_source["timestamp"]).timestamp()
               - datetime.fromisoformat(right_source["timestamp"]).timestamp()) > 2:
            raise ValueError("left/right readbacks are more than 2 s apart")
    block = load_block_observation(args.block_json)
    target = target_in_model_base(block, args.motor_datum_in_base_m)
    timestamp_ns = block.get("timing", {}).get("rgb_timestamp_ns", block.get("rgb_timestamp_ns"))
    if not args.sim_home_seed and (type(timestamp_ns) is not int or not 0 <= now_s - timestamp_ns / 1e9 <= 60):
        raise ValueError("target timestamp is missing, stale or future")
    if not math.isfinite(args.pregrasp_offset_z) or not 0 <= args.pregrasp_offset_z <= .10:
        raise ValueError("pregrasp offset must be finite and within [0, 0.10] m")
    target[2] += args.pregrasp_offset_z
    staging_path = getattr(args, "staging_reference", None)
    staging_seed, staging_source = None, None
    if staging_path is not None:
        if args.sim_home_seed or wrist_path is not None:
            raise ValueError("sensor staging requires measured start and a separate wrist phase")
        target, staging_seed, staging_source = load_staging_reference(staging_path, block, args.motor_datum_in_base_m)
    os.environ["DAPIER_SO101_MJCF"] = str(args.model.resolve(strict=True))
    if "scene_support" in block:
        env, scene_support = observed_task_env(block, args.motor_datum_in_base_m)
    elif args.sim_home_seed:
        env, scene_support = task_env("desk"), None
    else:
        raise ValueError("observed support plane missing; nominal SIM table cannot certify real path")
    model, data = env.model, env.data
    profile_path = getattr(args, "mapping_profile", None) or Path(__file__).with_name("tabletop_replay.json")
    profile = json.loads(profile_path.read_text())
    if args.sim_home_seed:
        seed = np.asarray(home_action(model, HUMANOID_HOME_ACTION))
        validate_model_action(model, seed, "explicit SIM home")
        seed_source = {"source": "explicit_sim_home_not_measured", "historical_target_allowed": True}
    else:
        seed = candidate_seed(model, left, right, profile)
        seed_source = {"source": "complete measured samples under unverified model mapping",
                       "left": left_source, "right": right_source}
    scene_object = (bind_observed_block(model, data, block, args.motor_datum_in_base_m)
                    if "scene_object" in block else None)
    if scene_object is None and not args.sim_home_seed:
        raise ValueError("observed object center/orientation missing; nominal SIM block cannot certify real path")
    mujoco.mj_forward(model, data)
    base = model.body("left_base").id
    rotation = data.xmat[base].reshape(3, 3).copy()
    origin = data.xpos[base].copy()
    world = origin + rotation @ target
    measured_preview = mujoco.MjData(model)
    apply_control_as_pose(model, measured_preview, seed)
    seed_tcp = {side: measured_preview.site(f"{side}_cube_grasp").xpos.copy().tolist()
                for side in ("left", "right")}
    carry = None
    if carry_path is not None:
        world, carry = load_carry_reference(carry_path, seed_tcp["left"], scene_object["center_world_m"])
        target = rotation.T @ (world-origin)
    goal_intent, wrist_source = None, None
    if wrist_path is not None:
        solved, goal_intent, wrist_source = load_wrist_correction(
            wrist_path, model, seed, left_source, now_ns=time.monotonic_ns())
        result = None
    else:
        solver_seed = seed.copy() if staging_seed is None else staging_seed.copy()
        # A successful historical q is initialization only. Both physical path
        # start and passive right arm remain the new measured sample.
        solver_seed[6:] = seed[6:]
        validate_model_action(model, solver_seed, "IK initialization")
        result = solve_bimanual_position_ik(model, solver_seed, {"left": world},
            site_names={"left": "left_cube_grasp"}, tool_axis_targets={"left": [0, 0, -1]},
            max_iterations=300, tolerance_m=5e-4)
        solved = np.asarray(result.action_rad).copy()
        # Only PREGRASP opens: carrying and wrist feedback keep measured aperture.
        solved[5] = seed[5] if carry is not None else model.actuator_ctrlrange[5, 1]
    validate_model_action(model, solved, "IK solution")
    preview = mujoco.MjData(model)
    apply_control_as_pose(model, preview, solved)
    actual = preview.site("left_cube_grasp").xpos.copy()
    error = float(np.linalg.norm(actual - world))
    axis = preview.site("left_cube_grasp").xmat.reshape(3, 3)[:, 0]
    axis_error = math.acos(float(np.clip(axis @ np.array([0., 0., -1.]), -1., 1.)))
    closing_error = math.acos(float(np.clip(preview.site("left_cube_grasp").xmat.reshape(3,3)[:,2]
                                          @ rotation[:,0], -1., 1.)))
    joint_ids = model.actuator_trnid[:, 0].astype(int)
    margins = np.minimum(solved - model.jnt_range[joint_ids, 0],
                         model.jnt_range[joint_ids, 1] - solved)
    guard = check_bimanual_path(model, seed, solved, required_clearance_m=DEFAULT_CLEARANCE_M,
                               task_phase=carry["phase"] if carry else "pregrasp", reference_data=data,
                               allow_sim_near_support=scene_support is None, carried_object=carry is not None)
    if carry is not None:
        carry.update(model_path_checked=True, model_path_safe=guard.safe, path_policy_scope=NEAR_SUPPORT_SCOPE)
        payload_preview = mujoco.MjData(model)
        payload_preview.qpos[:] = data.qpos
        attachment = _carried_object_attachment(model, payload_preview, seed)
        apply_control_as_pose(model, payload_preview, solved, preserve_raw_pose=True)
        _apply_carried_object(model, payload_preview, attachment)
        predicted = payload_preview.geom("red_block_geom").xpos.copy()
        address = attachment[0]
        carry.update(predicted_endpoint_center_world_m=predicted.tolist(),
            predicted_endpoint_quaternion_wxyz=payload_preview.qpos[address+3:address+7].tolist(),
            object_center_goal_error_m=float(np.linalg.norm(predicted-np.asarray(carry["desired_object_goal_center_world_m"]))),
            endpoint_semantics="kinematic hypothesis only; does not imply path or physical acceptance")
    # A sampled rigid-payload path under SIM contact rules is not a physical
    # contact certificate and cannot enter the PREGRASP/native plan adapter.
    candidate_ok = bool(carry is None and (result is None or result.converged)
                        and error <= 5e-4 and axis_error <= math.radians(2.) and guard.safe
                        and math.isfinite(guard.minimum_clearance_m)
                        and guard.minimum_clearance_m >= DEFAULT_CLEARANCE_M
                        and (staging_source is None or closing_error <= math.radians(15)))
    return {
        "candidate_mode": "carry_endpoint_ik" if carry is not None else "wrist_feedback" if wrist_path is not None else "pregrasp_ik",
        **({"planning_phase":carry["phase"], "carry_planning":carry} if carry is not None else {}),
        **({"goal_intent":goal_intent.as_dict(), "wrist_source":wrist_source} if goal_intent else {}),
        "ik_converged": bool(result and result.converged), "offline_candidate_accepted": candidate_ok,
        "iterations": result.iterations if result else 0, "position_error_m": error,
        **({"staging_reference":staging_source, "planning_phase":"ALIGN_HIGH",
            "closing_error_rad":closing_error, "solver_seed_rad":solver_seed.tolist()}
           if staging_source else {}),
        "tool_axis_error_rad_by_side": {"left":axis_error},
        "joint_margins_rad": {model.actuator(i).name: float(margins[i]) for i in range(model.nu)},
        "kinematic_analysis": {"target_arm_xyz_m": target.tolist(), "target_world_xyz_m": world.tolist(),
            "actual_arm_xyz_m": (rotation.T @ (actual - origin)).tolist(),
            "horizontal_distance_m": float(np.hypot(*target[:2])),
            "target_frame_interpretation": "motor1 datum translated to left_base; parallel axes candidate",
            "motor_datum_in_model_base_m": list(args.motor_datum_in_base_m),
            "source_arm_mapping": block.get("arm_mapping", {}),
            "reachability": "IK convergence is local; no physical maximum reach is asserted"},
        "seed_posture": {**seed_source, "seed_q_rad": seed.tolist(), "clipped": False},
        "seed_fk_world_m": seed_tcp,
        "scene_object": scene_object,
        "scene_support": scene_support,
        "solved_action_rad": solved.tolist(),
        "structured_clearance": {"safe": guard.safe, "reason": guard.reason,
            "minimum_clearance_m": guard.minimum_clearance_m,
            "required_clearance_m": DEFAULT_CLEARANCE_M, "query_cap_m": TASK_GENERAL_QUERY_CAP_M},
        "path_assessment": guard.as_report(),
        "model": {**fingerprint(args.model), "compiled_sha256": portable_model_sha256(model),
            "gripper_ranges_rad": model.actuator_ctrlrange[[5, 11]].tolist()},
        "mapping": {"profile": fingerprint(profile_path), "physically_verified": False,
            "assumption": "profile arm signs/zeros and linear measured percent to model gripper range"},
        "block_source": fingerprint(args.block_json),
        "execution_blockers": ["physical joint zero/sign and PGripper mapping unverified",
            "camera/board/arm target provenance requires independent physical validation",
            "offline IK and kinematic path are not a dynamic hardware execution plan"],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("output", "block-json", "model"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    seeds = parser.add_mutually_exclusive_group(required=True)
    seeds.add_argument("--measured-state-json", type=Path)
    seeds.add_argument("--sim-home-seed", action="store_true",
                       help="SIM diagnostic only: synthetic home and historical candidate target")
    parser.add_argument("--right-measured-state-json", type=Path)
    parser.add_argument("--mapping-profile", type=Path,
                        help="Explicit joint sign/zero candidate; preserved with its SHA, never self-certifies physical mapping")
    parser.add_argument("--staging-reference", type=Path,
                        help="Relative ALIGN_HIGH waypoint and IK seed; target is rebuilt from current observation")
    parser.add_argument("--carry-reference", type=Path,
                        help="Measured-start LIFT/PLACE with sampled rigid-payload path; SIM-only contact rules")
    parser.add_argument("--wrist-observation-json", type=Path,
                        help="Measured-state-bound wrist features; validate correction with FK/path, without IK")
    calibration = Path.home() / ".config/dapier/lerobot-calibration"
    for side in ("left", "right"):
        parser.add_argument(f"--{side}-calibration", type=Path,
                            default=calibration / f"dapier_dual_follower_{side}.json")
    parser.add_argument("--pregrasp-offset-z", type=float, default=.060)
    parser.add_argument("--motor-datum-in-base-m", type=float, nargs=3,
                        help="Audited measured-datum position in model left_base, metres")
    args = parser.parse_args(argv)
    report = {"schema_version": "dapier.offline-ik-candidate.v1", "hardware_execution": False,
              "accepted_for_execution": False, "source": fingerprint(Path(__file__)),
              "mujoco_version": mujoco.__version__, "ik_converged": False,
              "offline_candidate_accepted": False}
    # Reserve a new report before computing; historical results cannot be overwritten.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        try:
            report.update(evaluate(args))
        except (ValueError, KeyError, OSError, TypeError) as exc:
            report["input_rejection"] = str(exc)
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["offline_candidate_accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
