"""PICK02 HANDOFF_SNAPSHOT input, including explicit scene/held-pose bindings."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np


def array(value, name, shape):
    if value is None:
        return None
    if isinstance(value, (str, bool)):
        raise ValueError(f"{name}: numeric array required")
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "fiu":
            raise ValueError("non-numeric elements")
        result = raw.astype(float)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name}: numeric array required") from exc
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name}: expected {shape} finite values")
    return result


def transform(value, name):
    result = array(value, name, (4, 4))
    if result is not None:
        rotation = result[:3, :3]
        if (not np.allclose(result[3], [0, 0, 0, 1])
                or not np.allclose(rotation.T @ rotation, np.eye(3))
                or not np.isclose(np.linalg.det(rotation), 1)):
            raise ValueError(f"{name}: right-handed rigid transform required")
    return result


def q_values(value, name, count):
    result = array(value, name, (count,))
    if result is not None and any(not 0 <= result[i] <= 1 for i in range(5, count, 6)):
        raise ValueError(f"{name}: grippers must be canonical openings in [0, 1]")
    return result


def adapt_snapshot(snapshot, bindings=None):
    """Keep missing facts missing; bindings are existing pose/scene evidence, not defaults."""
    out = dict(status="MISSING", missing=[], invalid=[], real_execution_ready=False,
               motor_commands_sent=0, source_kind="UNKNOWN", snapshot_only=True,
               left_measured_q=None, left_hold_q=None, right_measured_q=None,
               right_last_applied_q=None, object_pose_model=None,
               object_half_extents_m=None, object_pose_uncertainty=None,
               scene={}, grasp_evidence={}, right_jaw_profile=None,
               q_mapping_verified=None, geometry_verified=None)
    if not isinstance(snapshot, dict) or (bindings is not None and not isinstance(bindings, dict)):
        out["status"] = "INVALID"
        out["invalid"] = ["snapshot and bindings must be JSON objects"]
        return out
    try:
        json.dumps((snapshot, bindings), allow_nan=False)
    except (TypeError, ValueError):
        out["status"] = "INVALID"
        out["invalid"] = ["snapshot and bindings must contain finite JSON values"]
        return out
    bindings = copy.deepcopy({} if bindings is None else bindings)
    out["source_kind"] = bindings.get("source_kind", "UNKNOWN")
    out["object_pose_role"] = bindings.get("object_pose_role", "UNKNOWN")
    out["object_pose_components"] = copy.deepcopy(bindings.get("object_pose_components"))
    out["kind"] = snapshot.get("kind")
    out["identifiers"] = copy.deepcopy(snapshot.get("identifiers") or {})
    out["timestamps"] = {k: copy.deepcopy(snapshot.get(k)) for k in
                         ("joint_reads", "top_stamps", "wrist_frames")}
    out["command_owner_at_the_lift"] = snapshot.get("command_owner_at_the_lift")
    for name in ("joint_reads", "top_stamps", "wrist_frames", "command_owner_at_the_lift"):
        if snapshot.get(name) is None:
            out["missing"].append(name)
    if snapshot.get("schema") != "tjj.handoff-snapshot.v1":
        out["invalid"].append("schema: expected tjj.handoff-snapshot.v1")
    if out["source_kind"] not in ("REAL", "SYNTHETIC", "UNKNOWN"):
        out["invalid"].append("source_kind: REAL, SYNTHETIC or UNKNOWN required")

    def read(name, value, validator, *args):
        if value is None:
            out["missing"].append(name)
            return None
        try:
            result = validator(value, name, *args)
            return None if result is None else result.tolist()
        except ValueError as exc:
            out["invalid"].append(str(exc))
            return None

    measured = read("q12_measured", snapshot.get("q12_measured"), q_values, 12)
    if measured is not None:
        out["left_measured_q"], out["right_measured_q"] = measured[:6], measured[6:]
    applied = snapshot.get("last_applied", {})
    if applied is None:
        applied = {}
    if not isinstance(applied, dict):
        out["invalid"].append("last_applied: JSON object required")
        applied = {}
    out["left_hold_q"] = read("last_applied.left_goal_q", applied.get("left_goal_q"), q_values, 6)
    out["left_applied_evidence"] = copy.deepcopy(applied)
    if applied.get("source") is None:
        out["missing"].append("last_applied.source")
    # The uncommanded right side remains None; measured geometry is never an applied command.
    obj = snapshot.get("object", {})
    if obj is None:
        obj = {}
    if not isinstance(obj, dict):
        out["invalid"].append("object: JSON object required")
        obj = {}
    size = read("object.size_m", obj.get("size_m"), array, (3,))
    if size is not None:
        if any(v <= 0 for v in size):
            out["invalid"].append("object.size_m: positive full dimensions required")
        else:
            out["object_half_extents_m"] = [v / 2 for v in size]
    out["object_observed_on_desk"] = copy.deepcopy(obj.get("observed_on_the_desk_world_m"))
    out["hand_target_offset"] = copy.deepcopy(obj.get("hand_target_offset_world_m"))
    out["object_pose_model"] = read("object_pose_model", bindings.get("object_pose_model"), transform)
    out["object_pose_uncertainty"] = copy.deepcopy(bindings.get("object_pose_uncertainty"))
    out["object_pose_source"] = copy.deepcopy(bindings.get("object_pose_source"))
    out["object_pose_frame"] = bindings.get("object_pose_frame")
    for name in ("object_pose_uncertainty", "object_pose_source", "object_pose_frame"):
        if out[name] is None:
            out["missing"].append(name)
    scene = bindings.get("scene", {})
    if scene is None:
        scene = {}
    if not isinstance(scene, dict):
        out["invalid"].append("scene: JSON object required")
        scene = {}
    out["scene"] = copy.deepcopy(scene)
    if out["object_pose_frame"] is not None and scene.get("frame") is not None and out["object_pose_frame"] != scene["frame"]:
        out["invalid"].append("object_pose_frame differs from scene.frame")
    for name in ("T_model_left_base", "T_model_right_base", "T_model_camera", "T_model_table", "T_task_from_model"):
        out["scene"][name] = read("scene." + name, scene.get(name), transform)
    for name in ("frame", "model_sha256", "source_sha256"):
        value = scene.get(name)
        if value is None:
            out["missing"].append("scene." + name)
        elif not isinstance(value, str) or not value:
            out["invalid"].append(f"scene.{name}: nonempty string required")
    height = scene.get("table_top_z_m")
    if height is None:
        out["missing"].append("scene.table_top_z_m")
    elif isinstance(height, bool) or not isinstance(height, (int, float)) or not np.isfinite(height):
        out["invalid"].append("scene.table_top_z_m: finite task-frame height required")
    for name in ("q_mapping_verified", "geometry_verified"):
        out[name] = bindings.get(name)
        if out[name] is not None and type(out[name]) is not bool:
            out["invalid"].append(f"{name}: boolean or null required")
    evidence = snapshot.get("grasp_evidence")
    if evidence is None or evidence == {}:
        out["missing"].append("grasp_evidence")
    elif not isinstance(evidence, dict):
        out["invalid"].append("grasp_evidence: JSON object required")
    else:
        out["grasp_evidence"] = copy.deepcopy(evidence)
    out["right_jaw_profile"] = copy.deepcopy(bindings.get("right_jaw_profile"))
    out["status"] = "INVALID" if out["invalid"] else ("MISSING" if out["missing"] else "INPUT_VALID")
    out["stages"] = dict(planning="NOT_READY" if out["status"] != "INPUT_VALID" else "INPUT_VALID",
                         right_grasp="NOT_READY", left_release="NOT_READY")
    return out


def main():
    parser = argparse.ArgumentParser(description="Inspect a saved PICK02 snapshot; no device access")
    parser.add_argument("snapshot")
    parser.add_argument("--bindings", help="Existing held-pose and scene evidence JSON")
    args = parser.parse_args()
    snapshot = json.loads(Path(args.snapshot).read_text())
    bindings = {} if args.bindings is None else json.loads(Path(args.bindings).read_text())
    result = adapt_snapshot(snapshot, bindings)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 2 if result["status"] == "INVALID" else 0


if __name__ == "__main__":
    raise SystemExit(main())
