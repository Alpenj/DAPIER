"""Sensor-to-IK staging and ControlIntent translation adapter.

This module accepts a validated VisionTargetEstimate and calibrated extrinsic transform,
transforms the target into the arm base frame, plans staging waypoints, and formats
versioned ControlIntents according to contracts/research_realtime_control_v1.json.
It performs no hardware I/O and authorizes no motor execution.
"""

from __future__ import annotations

import math
import hashlib
import json
from pathlib import Path
import time
from typing import Any, Mapping, Sequence

import numpy as np

from dapier_research.control_intent import ControlIntent, arm_joint_position_intent, load_contract, validate_intent
from dapier_research.vision_target import VisionTargetError, VisionTargetEstimate, _finite_transform


def load_block_observation(path: Path) -> dict:
    """Normalize observed board/cube geometry for the existing IK/executor path."""
    document = json.loads(path.read_text())
    if not isinstance(document, dict):
        raise ValueError("block observation must be an object")
    if "board_cube_observation" not in document:
        return document
    from dapier_research.camera_board_transform import known_cube_from_board
    observation = document["board_cube_observation"]
    frame = observation["frame_source"]
    calibration = observation["calibration_sources"]
    if (not isinstance(calibration, list) or not calibration
            or observation["board_pose_frame_sha256"] != frame["sha256"]
            or observation["top_corners_frame_sha256"] != frame["sha256"]):
        raise ValueError("same-frame board pose and calibration sources required")
    for source in [frame, *calibration]:
        if hashlib.sha256(Path(source["path"]).read_bytes()).hexdigest() != source["sha256"]:
            raise ValueError("block observation source changed")
    image = np.load(frame["path"], allow_pickle=False)
    k = observation["intrinsics"]
    if image.shape != (k["height"], k["width"], 3) or image.dtype != np.uint8:
        raise ValueError("rectified uint8 BGR frame must match the intrinsics resolution")
    result = known_cube_from_board(observation)
    metric = document.get("metric_evidence", {})
    if not isinstance(metric, dict):
        raise ValueError("metric target evidence must be an object")
    # Computed geometry never promotes its own execution verdict. The source
    # review remains explicit and is re-read by the bounded executor adapter.
    result["metric_evidence"] = {**metric, "method":result["geometry_evidence"]["method"],
                                  "direct_depth_used":False}
    result["source_evidence"] = [frame, *calibration]
    return result


def transform_optical_point_to_arm(
    point_optical_m: Sequence[float],
    T_arm_from_camera: np.ndarray,
) -> np.ndarray:
    """Transform a 3D point from camera optical frame (+X right, +Y down, +Z forward) to arm frame."""
    point = np.asarray(point_optical_m, dtype=np.float64)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise VisionTargetError("invalid_point", "point_optical_m must be finite 3D coordinates")

    transform = _finite_transform(T_arm_from_camera)
    homo = np.append(point, 1.0)
    transformed = transform @ homo
    return transformed[:3]


def validate_sensor_estimate_for_motion(
    estimate: VisionTargetEstimate,
    *,
    now_monotonic_ns: int | None = None,
    max_age_ns: int = 2_000_000_000,  # 2.0s
    minimum_confidence: float = 0.50,
) -> None:
    """Fail-closed check for freshness, confidence, and finite values before planning."""
    if not isinstance(estimate, VisionTargetEstimate):
        raise VisionTargetError("invalid_estimate", "estimate must be a VisionTargetEstimate")
    if (not math.isfinite(minimum_confidence) or not 0 <= minimum_confidence <= 1
            or type(max_age_ns) is not int or max_age_ns <= 0):
        raise VisionTargetError("invalid_bounds", "finite confidence and positive freshness bounds required")

    if not math.isfinite(estimate.confidence) or estimate.confidence < minimum_confidence:
        raise VisionTargetError(
            "low_confidence",
            f"target confidence {estimate.confidence:.3f} is below minimum {minimum_confidence:.3f}",
        )

    pos_optical = np.asarray(estimate.position_optical_m, dtype=np.float64)
    if pos_optical.shape != (3,) or not np.all(np.isfinite(pos_optical)):
        raise VisionTargetError("nonfinite_target", "target position must be finite")

    if estimate.median_depth_m <= 0.0 or not math.isfinite(estimate.median_depth_m):
        raise VisionTargetError("invalid_depth", "target depth must be positive and finite")

    if now_monotonic_ns is not None:
        if type(now_monotonic_ns) is not int or type(estimate.timestamp_ns) is not int:
            raise VisionTargetError("invalid_timestamp", "integer nanosecond timestamps required")
        age_ns = now_monotonic_ns - estimate.timestamp_ns
        if age_ns < 0 or age_ns > max_age_ns:
            raise VisionTargetError(
                "stale_sensor_target",
                f"sensor target age {age_ns * 1e-6:.1f}ms exceeds maximum allowance {max_age_ns * 1e-6:.1f}ms",
            )


def plan_pregrasp_staging_waypoints(
    grasp_target_arm_m: Sequence[float],
    *,
    pregrasp_offset_m: Sequence[float] = (0.0, 0.0, 0.050),
    stage_offset_m: Sequence[float] = (0.0, 0.070, 0.050),
) -> dict[str, list[float]]:
    """Derive staging waypoints (SAFE_STAGE, PREGRASP, GRASP) from arm-frame target."""
    grasp = np.asarray(grasp_target_arm_m, dtype=np.float64)
    if grasp.shape != (3,) or not np.all(np.isfinite(grasp)):
        raise VisionTargetError("invalid_target", "grasp_target_arm_m must be finite 3D coordinates")

    pregrasp_off = np.asarray(pregrasp_offset_m, dtype=np.float64)
    stage_off = np.asarray(stage_offset_m, dtype=np.float64)
    if any(offset.shape != (3,) or not np.isfinite(offset).all() for offset in (pregrasp_off, stage_off)):
        raise VisionTargetError("invalid_offset", "staging offsets must be finite 3D vectors")

    pregrasp = grasp + pregrasp_off
    safe_stage = pregrasp + stage_off

    return {
        "SAFE_STAGE": safe_stage.tolist(),
        "PREGRASP_NEAR": pregrasp.tolist(),
        "GRASP": grasp.tolist(),
    }


def create_joint_position_intents(
    joint_trajectory_rad: Sequence[Sequence[float]],
    joint_names: Sequence[str],
    *,
    source: str = "real_sensor_ik_adapter",
    update_period_s: float = 0.050,  # 20 Hz
    max_velocity_rad_s: float = 0.50,
    start_sequence: int = 1,
    start_monotonic_ns: int | None = None,
    ttl_ns: int = 250_000_000,  # 250 ms
) -> list[ControlIntent]:
    """Format a sequenced list of ControlIntents from joint waypoints."""
    if not joint_names or not all(isinstance(name, str) and name.strip() for name in joint_names):
        raise ValueError("joint_names must be a non-empty sequence of strings")

    num_joints = len(joint_names)
    if not math.isfinite(update_period_s) or update_period_s <= 0:
        raise ValueError("update_period_s must be positive and finite")
    intents = []
    base_ns = time.monotonic_ns() if start_monotonic_ns is None else start_monotonic_ns
    dt_ns = int(round(update_period_s * 1e9))
    if dt_ns <= 0:
        raise ValueError("update period is below timestamp resolution")

    contract = load_contract()
    for idx, q in enumerate(joint_trajectory_rad):
        q_arr = np.asarray(q, dtype=np.float64)
        if q_arr.shape != (num_joints,) or not np.all(np.isfinite(q_arr)):
            raise ValueError(f"step {idx}: joint positions must match joint count {num_joints} and be finite")

        step_ns = base_ns + idx * dt_ns
        intent = arm_joint_position_intent(
            sequence=start_sequence + idx,
            source=source,
            joint_names=tuple(joint_names),
            joint_position_rad=tuple(q_arr.tolist()),
            joint_max_velocity_rad_s=tuple([max_velocity_rad_s] * num_joints),
            ttl_ns=ttl_ns,
            source_monotonic_ns=step_ns,
            contract=contract,
        )
        intents.append(intent)

    return intents


def bounded_pregrasp_plan(candidate: Mapping[str, Any], profile_path: Path,
                         *, now_s: float, goal_intent: ControlIntent | None = None) -> dict:
    """Bind a checked current sensor candidate to the existing native executor.

    A wrist proposal uses this same boundary after FK/axis/path revalidation; its
    joint endpoint must exactly match the validated candidate. Missing physical
    evidence remains false and the native hardware gate refuses it.
    """
    if not math.isfinite(now_s):
        raise ValueError("finite audit time required")
    if candidate.get("offline_candidate_accepted") is not True:
        raise ValueError("current sensor candidate has not passed IK/path acceptance")
    if (candidate.get("scene_object") or {}).get("bound_to_path_reference") is not True:
        raise ValueError("observed object was not bound to the checked collision scene")
    start = np.asarray(candidate["seed_posture"]["seed_q_rad"], dtype=float)
    goal = np.asarray(candidate["solved_action_rad"], dtype=float)
    if any(q.shape != (12,) or not np.isfinite(q).all() for q in (start, goal)):
        raise ValueError("complete finite bimanual radian states required")
    if not np.array_equal(start[6:], goal[6:]):
        raise ValueError("left-only executor cannot dispatch right-arm changes")
    sources = [candidate["block_source"], candidate["model"], candidate["mapping"]["profile"]]
    if "staging_reference" in candidate:
        sources.append(candidate["staging_reference"])
    if candidate.get("candidate_mode") == "wrist_feedback":
        wrist = candidate["wrist_source"]
        sources.extend((wrist, wrist["frame_source"]))
        if "capture_source" in wrist:
            sources.append(wrist["capture_source"])
        if "mask_source" in wrist:
            sources.append(wrist["mask_source"])
    timestamps = []
    for side in ("left", "right"):
        from datetime import datetime
        measured = candidate["seed_posture"][side]
        timestamps.append(datetime.fromisoformat(measured["timestamp"]).timestamp())
        sources.extend((measured, measured["calibration"]))
    for source in sources:
        if hashlib.sha256(Path(source["path"]).read_bytes()).hexdigest() != source["sha256"]:
            raise ValueError("candidate source changed since validation")
    block = load_block_observation(Path(candidate["block_source"]["path"]))
    sources.extend(block.get("source_evidence", []))
    timestamp_ns = block.get("timing", {}).get("rgb_timestamp_ns", block.get("rgb_timestamp_ns"))
    if type(timestamp_ns) is not int:
        raise ValueError("target acquisition timestamp missing")
    timestamps.append(timestamp_ns / 1e9)
    if any(not 0 <= now_s - stamp <= 60 for stamp in timestamps):
        raise ValueError("current target/readback is stale or future")
    position_error = float(candidate["position_error_m"])
    axis_error = float(candidate["tool_axis_error_rad_by_side"]["left"])
    clearance = candidate["structured_clearance"]
    distance = float(clearance["minimum_clearance_m"])
    if (not all(math.isfinite(x) for x in (position_error, axis_error, distance))
            or not 0 <= position_error <= .0005 or not 0 <= axis_error <= math.radians(2)
            or clearance["safe"] is not True or distance < .030):
        raise ValueError("position/vertical-axis/path acceptance failed")
    profile_raw = profile_path.read_bytes()
    profile = json.loads(profile_raw)
    mapping_profile = json.loads(Path(candidate["mapping"]["profile"]["path"]).read_text())
    for i, spec in enumerate(profile["joints"][:5]):
        if (spec["sign"] != mapping_profile["arm_signs"][i]
                or spec["zero_offset_deg"] != mapping_profile["arm_zero_offsets_deg"][i]):
            raise ValueError("native joint mapping differs from FK/IK mapping")
    if profile["gripper_rad_limits"] != candidate["model"]["gripper_ranges_rad"][0]:
        raise ValueError("native PGripper range differs from FK/IK model")
    names = tuple(j["name"] for j in profile["joints"])
    velocities = np.asarray([j["maximum_velocity_rad_s"] for j in profile["joints"]], dtype=float)
    if velocities.shape != (6,) or not np.isfinite(velocities).all() or np.any((velocities <= 0) | (velocities > .3)):
        raise ValueError("invalid native velocity bounds")
    if goal_intent is None:
        goal_intent = arm_joint_position_intent(sequence=1, source="real_sensor_ik_adapter",
            joint_names=names, joint_position_rad=tuple(goal[:6]),
            joint_max_velocity_rad_s=tuple(velocities), ttl_ns=250_000_000,
            source_monotonic_ns=time.monotonic_ns())
    validate_intent(goal_intent)
    if (goal_intent.kind != "arm_joint_position" or goal_intent.joint_names != names
            or not np.array_equal(goal_intent.joint_position_rad, goal[:6])):
        raise ValueError("intent endpoint differs from checked FK/IK/path candidate")
    envelope = candidate.get("path_envelope", {})
    # Metric geometry can establish a target without direct depth. An explicit
    # current verdict takes precedence over legacy depth evidence, even if false.
    metric = block.get("metric_evidence", block.get("depth_evidence", {}))
    if not isinstance(metric, Mapping):
        raise ValueError("metric target evidence must be an object")
    duration = max(2.5, float(np.max(1.875*np.abs(goal[:6]-start[:6])/velocities))*1.15)
    if duration+1 > 60:
        raise ValueError("bounded motion exceeds maximum duration")
    return {"schema_version":"dapier.bounded-pregrasp-plan.v1",
        "phase":"WRIST_ALIGN" if goal_intent.source == "wrist_servo_adapter" else "PREGRASP",
        "initial_torque_enabled":goal_intent.source == "wrist_servo_adapter",
        "profile_sha256":hashlib.sha256(profile_raw).hexdigest(),
        "start_rad":start[:6].tolist(), "goal_rad":goal[:6].tolist(),
        "goal_intent":goal_intent.as_dict(), "maximum_duration_s":duration+1,
        "observation_unix_s":timestamps[-1], "measured_state_unix_s":min(timestamps[:2]),
        "position_error_m":position_error,"axis_error_rad":axis_error,
        "offline_candidate_accepted":True,"path_clear":True,"path_clearance_m":distance,
        "sensor_target_verified":metric.get("metric_target_verified") is True,
        "path_tracking_tolerance_rad":float(envelope.get("tracking_tolerance_rad",.01)),
        "path_envelope_verified":envelope.get("verified") is True,
        "path_envelope_clearance_m":envelope.get("minimum_clearance_m",0.),
        "source_evidence":sources,"task_success":False}
