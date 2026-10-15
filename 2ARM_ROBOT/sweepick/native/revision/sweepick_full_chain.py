"""The FULL-CHAIN mode of the PICK02 session, bound to the functions that exist.

ONE state machine runs the part after the lift: the implementation's FullEpisodeHandoff -> TaskManager -> ReceiveManipulation ->
ReceiveSession (COORDINATE -> RELEASE -> PLACE -> VERIFY_PLACE -> ARM_STOW -> REOBSERVE_CLEAR -> CLEAN_RESUME_REQUEST).
This module does not run a second one. It is the LOCAL OWNER that state machine is given: the same process that picked
and holds the block, and the only one that reads or writes an arm.

  writer        Owner.apply_dispatch  -> sweepick_move_a.run / grip (the executor of the PICK02 stages: checked trajectory,
                                         motion profile, progress watch, settle, torque state) once per planned waypoint or
                                         per commanded gripper / arm target. No goal stream of its own.
  observation   Owner.observation     -> read-only registers of both arms + the session recorder's camera frames
  plan          the implementation's adapter + receive planner over the SIM teacher (carry, right approach, right insert)
  load transfer LoadTransfer          -> the left Jaw's hold command backed off by the left profile's own squeeze step
  release       Release               -> left jaws to their open tick, then the teacher's own retreat directions / hops
  PLACE / STOW  Downstream            -> teacher.plan_move with the block carried by the right hand; executor 'return'
  perception    Area                  -> the session's look (top depth through the board) on the target / source region

What needs evidence from the real robot is read from three files that do not exist until it has been taken
(FIELD below). Without them `preflight` says NOT READY and names them; nothing is defaulted, and the left gripper's
profile is never used for the right gripper."""
import hashlib
import copy
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PIN_ORIGINAL = HERE / "_codex_receive_pin_2303"      # kept as it was: the implementation's receive modules at 23:03 on 2026-10-08 (the pin of the 2026-10-08 integration)
PIN = HERE / "_codex_receive_pin_final_0107"        # SELECTED 2026-10-09: the implementation's final_connections pin (01:07; entry / session / full_episode changed, six modules identical to the original pin; SHA256SUMS inside). Re-pinned deliberately, never silently
CODEX = Path(os.environ.get("SWEEPICK_RECEIVE_DIR", PIN if PIN.exists() else Path.home() / "sweepick_261008_receive_real"))
RUNTIME = Path.home() / "DAPIER/tjj-runtime/v1"
MJCF = Path.home() / "DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml"
FIELD = dict(
    right_jaw=(HERE / "sweepick_jaw_profile_right.json", "measured device profile of the RIGHT gripper (free-stroke residual, contact onset, rest residual), same schema as the left file plus device_identity and measurement_evidence",
               "the right close (COORDINATE cannot become HANDOFF_CANDIDATE), the right open at the place"),
    right_wrist=(HERE / "sweepick_right_wrist_reference.json", "the right wrist camera with a block between the right jaws, from the right contact observation: block_between_jaws_centre_px, centre_tolerance_px, and for the insert's orientation bound square_slope_deg (far-edge slope of a block square to the right jaws) and slope_resolution_deg",
                 "object_at_tool for the right hand (grasp label, support record, left release) and the orientation bound of the right insert"),
    support=(HERE / "sweepick_support_reference.json", "how the load transfer shows on THIS robot: the small right-hand lift used (right_lift_m), the left-arm joints whose Present_Load falls when the right hand takes the block and by how much (load_joints, drop_raw), read in one supervised transfer",
             "the load transfer and the support record, and therefore the left release"),
    place=(HERE / "sweepick_place_target.json", "the place target (centre and surface height in the model frame) confirmed on the real desk; a candidate file exists as sweepick_place_target.candidate.json",
           "PLACE, VERIFY_PLACE"))
FIELD = {name: (Path.home() / "DAPIER/.local-artifacts/sweepick-field-tools" / path.name, what, blocks)
         for name, (path, what, blocks) in FIELD.items()}                       # one canonical field record, even when the code revision is selected
ALL = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"); ARM = ALL[:5]
STOPPING = ("FAULT_HOLDING", "FAULT_JOINTS_RELEASED", "BUS_LOST", "STOPPED_HOLDING", "NO_PROGRESS_HOLDING", "RELEASE_UNCONFIRMED", "REFUSED")


def observation_plan_missing(plan):
    """The plan of a SUPPORT_OBSERVATION session (the implementation's contract): an explicit motion and object-support plan is the INPUT; the
    support thresholds are what the session is run to find, never an input. `end_state` names the state at the OBSERVATION
    POINT (both hands on the block, nothing released); what the session does afterwards is this owner's planned return."""
    if not isinstance(plan, dict):
        return ["explicit support-observation motion and object-support plan"]
    missing = [name for name in ("source", "object_support_plan") if not isinstance(plan.get(name), str) or not plan[name]]
    lift = plan.get("right_lift_m")
    if isinstance(lift, bool) or not isinstance(lift, (int, float)) or not math.isfinite(lift) or lift <= 0:
        missing.append("explicit finite positive right_lift_m motion target")
    if plan.get("end_state") != "HOLD_BOTH_NO_RELEASE":
        missing.append("end_state HOLD_BOTH_NO_RELEASE (the state at the observation point)")
    return missing


def observation_candidate_errors(candidate):
    """Malformed proposal checks before START; observation is once, after the final target."""
    if not isinstance(candidate, dict) or candidate.get("source_kind") != "SENSOR_SIM_CANDIDATE":
        return ["an explicit sensor-simulation view proposal is required"]
    if any(k in candidate for k in ("q12", "trajectory", "gt_pose", "ready", "held_object")):
        return ["joint trajectories, GT and readiness are not view-candidate inputs"]
    if candidate.get("observation_policy", "FINAL_TARGET_ONLY") != "FINAL_TARGET_ONLY":
        return ["observation_policy must be FINAL_TARGET_ONLY: all targets run sequentially, then one fresh frame is observed"]
    targets = candidate.get("targets")
    components = candidate.get("expected_observed_components")
    if (not isinstance(targets, list) or not targets or not isinstance(components, list)
            or not components or not all(isinstance(c, str) and c for c in components)):
        return ["finite relative-tool targets and expected observable components are required"]
    errors = []
    for target in targets:
        try:
            if not isinstance(target, dict) or set(target) != {"side", "reference_frame", "reference_from_target_tool"}:
                raise ValueError("target must contain side, reference_frame and relative tool transform only")
            if target["side"] not in ("left", "right") or target["reference_frame"] not in ("CURRENT_TOOL", "SEARCH_PRIOR_OBJECT"):
                raise ValueError("unknown A arm or proposal reference frame")
            delta = np.asarray(target["reference_from_target_tool"], float)
            if (delta.shape != (4, 4) or not np.isfinite(delta).all()
                    or not np.allclose(delta[3], [0, 0, 0, 1])
                    or not np.allclose(delta[:3, :3].T @ delta[:3, :3], np.eye(3))
                    or not np.isclose(np.linalg.det(delta[:3, :3]), 1)):
                raise ValueError("malformed rigid relative tool transform")
        except (ValueError, TypeError, KeyError) as exc:
            errors.append(str(exc))
    return errors


def contact_plan_check(plan, mapping, mapping_sha, profile):
    """Validate the explicit RECEIVE_CONTACT input against the saved right free-close and the active native executor inputs."""
    missing, evidence = [], None
    if not isinstance(plan, dict):
        return ["explicit receive-contact plan"], None
    candidates = plan.get("observation_candidates", [])
    if not isinstance(candidates, list):
        missing.append("observation_candidates must be a finite proposal list")
    else:
        for i, candidate in enumerate(candidates):
            missing.extend(f"observation candidate {i}: {error}" for error in observation_candidate_errors(candidate))
    for name in ("source", "baseline_file", "object_support_plan"):
        if not isinstance(plan.get(name), str) or not plan[name]:
            missing.append(f"nonempty {name}")
    for name in ("close_target_tick", "observe_s"):
        value = plan.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            missing.append(f"finite {name}")
    if isinstance(plan.get("observe_s"), (int, float)) and not isinstance(plan.get("observe_s"), bool) and plan["observe_s"] <= 0:
        missing.append("positive observe_s")
    if plan.get("end_state") != "LEFT_HOLD_THEN_PLANNED_RETURN":
        missing.append("end_state LEFT_HOLD_THEN_PLANNED_RETURN")
    if missing:
        return missing, None
    try:
        import sweepick_trajectory as tt
        baseline_path = Path(plan["baseline_file"]).expanduser().resolve()
        baseline = json.loads(baseline_path.read_text()); trajectory_path = baseline_path.with_name("TRAJECTORY.json")
        saved = json.loads(trajectory_path.read_text()); target = float(plan["close_target_tick"])
        traj, saved_traj = baseline.get("trajectory") or {}, saved.get("trajectory") or {}
        active_lo = float(mapping["right"]["gripper"]["lo"])
        checks = [
            (baseline.get("schema") == "tjj.move.v3", "baseline schema tjj.move.v3"),
            (baseline.get("side") == saved.get("side") == "right", "right-side baseline"),
            (baseline.get("result") == "HOLDING" and (baseline.get("outcome") or {}).get("target_reached") is True, "completed free-close baseline"),
            (traj.get("moving") == saved_traj.get("moving") == ["gripper"], "gripper-only baseline"),
            ((traj.get("target") or {}).get("gripper") == (saved_traj.get("target") or {}).get("gripper") == target == active_lo,
             "close target equal to the active calibrated right gripper minimum"),
            ((traj.get("start") or {}).get("gripper", -math.inf) > target, "closing direction toward decreasing ticks"),
            ((baseline.get("profile") or {}).get("sha256") == (saved.get("profile") or {}).get("sha256") == profile.get("sha256"), "active motion profile identity"),
            (saved.get("mapping_sha256") == mapping_sha, "active candidate mapping identity"),
            (saved.get("identity") == tt.identity("right", saved_traj, mapping_sha), "native trajectory identity"),
        ]
        missing += [name for ok, name in checks if not ok]
        rows = [r for r in baseline.get("rows", []) if isinstance(r, dict) and isinstance(r.get("present"), dict) and isinstance(r.get("commanded"), dict)
                and "gripper" in r["present"] and "gripper" in r["commanded"]]
        lead = [float(r["present"]["gripper"]) - float(r["commanded"]["gripper"]) for r in rows]
        if not lead or not np.isfinite(lead).all() or max(lead) < 0:
            missing.append("finite measured closing lead rows in the baseline")
        if missing:
            return missing, None
        evidence = dict(baseline_file=str(baseline_path), baseline_sha256=hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
                        trajectory_file=str(trajectory_path), trajectory_sha256=hashlib.sha256(trajectory_path.read_bytes()).hexdigest(),
                        native_trajectory_identity=saved["identity"], side="right", joint="gripper", direction=-1, target_tick=target,
                        baseline_start_tick=float(traj["start"]["gripper"]), baseline_max_lead_ticks=float(max(lead)), baseline_rows=len(lead),
                        profile_sha256=profile["sha256"], mapping_sha256=mapping_sha,
                        classification="PROVISIONAL measured free-closing lead; not a HARD gate and not support/grasp confirmation")
    except Exception as exc:
        missing.append(f"readable bound baseline_file and TRAJECTORY.json: {type(exc).__name__}: {exc}")
    return missing, evidence


def save_wrist_surface_diagnostics(folder, frames, features):
    """Store the same RGB masks/edges consumed by the owner, with clipped borders separate."""
    import cv2
    masks = {}
    for name, feature in features.items():
        image = (frames.get(name) or {}).get("frame")
        if image is None:
            continue
        overlay = image.copy()
        for key in ("_mask", "_rgb_edge_mask"):
            if isinstance(feature.get(key), np.ndarray):
                masks[name + key] = feature[key]
        for candidate, mask in zip(feature.get("candidates", []), feature.get("_candidate_masks", [])):
            masks[f"{name}_candidate_{candidate['component_label']}"] = mask
        if feature.get("status") == "AMBIGUOUS":
            for candidate in feature.get("candidates", []):
                points = np.asarray(candidate["contour_points_px"], np.int32)
                if len(points) > 1:
                    cv2.putText(overlay, "candidate " + str(candidate["component_label"]), tuple(points[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, .3, (0, 180, 255), 1)
                for key, colour in (("visible_edges", (0, 180, 255)), ("clipped_boundary", (255, 0, 255))):
                    for edge in candidate.get(key, []):
                        points = np.asarray(edge["points_px"], np.int32)
                        if len(points) > 1:
                            cv2.polylines(overlay, [points], False, colour, 1)
        for key, colour in (("visible_edges", (0, 255, 0)), ("interior_edge_candidates", (255, 255, 0)),
                            ("clipped_boundary", (255, 0, 255))):
            for edge in feature.get(key, []):
                points = np.asarray(edge["points_px"], np.int32)
                if len(points) > 1:
                    cv2.polylines(overlay, [points], False, colour, 1)
        cv2.putText(overlay, name + ": " + feature["status"] + " / 2D ONLY", (4, 13),
                    cv2.FONT_HERSHEY_SIMPLEX, .35, (255, 255, 255), 1)
        if not cv2.imwrite(str(Path(folder) / (name + "_SURFACE_OVERLAY.png")), overlay):
            raise OSError(f"wrist surface overlay not saved: {name}")
    if masks:
        np.savez_compressed(Path(folder) / "WRIST_SURFACE_MASKS.npz", **masks)


def camera_valid(frame):
    """Whether this camera record is usable under the reader's own cadence/status result (the implementation's function)."""
    return bool(isinstance(frame, dict)
                and frame.get("reader_valid") is True and frame.get("fresh") is True
                and isinstance(frame.get("n"), int) and not isinstance(frame.get("n"), bool)
                and isinstance(frame.get("recv_monotonic_s"), (int, float))
                and not isinstance(frame.get("recv_monotonic_s"), bool)
                and math.isfinite(frame["recv_monotonic_s"]))


def camera_identity(frame):
    """Copy only the reader-owned identity/validity, never the bus observation identity (the implementation's function)."""
    keys = ("n", "recv_monotonic_s", "frame_number", "sdk_color_stamp_s", "sdk_depth_stamp_s",
            "helper_receipt_monotonic_s", "reader_valid", "reader_reason", "freshness_limit_s")
    out = {k: frame[k] for k in keys if isinstance(frame, dict) and k in frame}
    out["fresh"] = camera_valid(frame)
    return out


def camera_evidence(frame, observed, source):
    """A vision child keeps the camera identity of the pixels it evaluated (the implementation's function)."""
    valid = camera_valid(frame)
    return dict(camera_identity(frame), observed=bool(observed) if valid else False,
                source=source if valid else str((frame or {}).get("reader_reason") or "camera frame is not valid"))


def reader_frame(reader, packet, now, **images):
    """A reader's frame with its own sequence and receipt times, judged at `now` (the clock read right after the frame was
    fetched) by the reader's own state and its own measured cadence (sweepick_local_loop.cadence_limit). the implementation's function,
    with two things kept from the active executor: a recorder's lent reader is looked through to the real reader, and a
    stream whose cadence is not known yet is NOT valid (no frame is called fresh on its own say-so)."""
    if packet is None:
        return None
    from sweepick_local_loop import cadence_limit
    real = getattr(reader, "reader", reader); n, stamp = packet.get("n"), packet.get("recv_monotonic_s"); reasons = []
    for name in ("reader_valid", "fresh", "usable", "integrity_valid"):
        if packet.get(name) is False:
            reasons.append(f"reader's existing {name} result is false")
    if not isinstance(n, int) or isinstance(n, bool):
        reasons.append("reader sequence is malformed")
    if not isinstance(stamp, (int, float)) or isinstance(stamp, bool) or not math.isfinite(stamp):
        reasons.append("reader receipt time is malformed")
    basis = packet.get("helper_receipt_monotonic_s", stamp); limit = packet.get("cadence_limit_s", None) if "cadence_limit_s" in packet else cadence_limit(getattr(real, "intervals", ()))
    if isinstance(basis, (int, float)) and not isinstance(basis, bool) and math.isfinite(basis):
        if basis > now:
            reasons.append("reader receipt time is in the future")
        if limit is None:
            reasons.append("reader cadence validity is UNKNOWN (too few frame intervals seen)")
        elif now - basis > limit:
            reasons.append("frame exceeds the reader's measured-cadence freshness limit")
    else:
        reasons.append("reader freshness time is malformed")
    error = getattr(real, "reader_error", None) or getattr(real, "error", None)
    if error:
        reasons.append(f"reader reported an error: {error}")
    done = getattr(real, "done", None)
    if (done is not None and done.is_set()) or getattr(real, "reader_ended_monotonic_s", None) is not None:
        reasons.append("reader stream has ended")
    keep = ("n", "recv_monotonic_s", "frame_number", "sdk_color_stamp_s", "sdk_depth_stamp_s", "helper_receipt_monotonic_s")
    out = {k: packet[k] for k in keep if k in packet}
    out.update(images, reader_valid=not reasons, reader_reason="; ".join(reasons), freshness_limit_s=limit, fresh=not reasons, age_at_fetch_s=(now - basis) if isinstance(basis, (int, float)) and not isinstance(basis, bool) else None)
    return out


def selection(scope=None):
    """What this process actually loaded: the native files and the receive pin, with their hashes. Written into the session folder."""
    sha = lambda f: hashlib.sha256(Path(f).read_bytes()).hexdigest(); cx = codex(); import sweepick_pick01, sweepick_move_a, sweepick_episode, sweepick_grasp, sweepick_trajectory, sweepick_real_exec, sweepick_progress
    native = {m.__name__: dict(file=m.__file__, sha256=sha(m.__file__)) for m in (sys.modules[__name__], sweepick_pick01, sweepick_move_a, sweepick_episode, sweepick_grasp, sweepick_trajectory, sweepick_real_exec, sweepick_progress)}
    profiles = {f.name: sha(f) for f in sorted(HERE.glob("sweepick_*profile*.json"))}
    teacher_source = RUNTIME / "source/production_source_g000/sim_data_factory/teacher.py"
    return dict(schema="sweepick.active-selection.v1", scope=scope, pin_dir=str(CODEX), pin_selected_by="SWEEPICK_RECEIVE_DIR" if os.environ.get("SWEEPICK_RECEIVE_DIR") else "sweepick_full_chain.PIN (default)", pin_original_kept=str(PIN_ORIGINAL), pin_original_present=PIN_ORIGINAL.exists(),
                teacher_source=dict(file=str(teacher_source), sha256=sha(teacher_source)),
                pin_modules={n: dict(file=m.__file__, sha256=sha(m.__file__)) for n, m in cx.items()}, native=native, profiles=profiles, writer="sweepick_move_a.run / grip (finite checked trajectories); no per-cycle writer is constructed",
                field_records={n: dict(file=str(pth), present=pth.exists()) for n, (pth, _, _) in FIELD.items()})


def codex():
    """the implementation's modules from where they were delivered; imported, not copied or changed."""
    os.environ.setdefault("FACTORY_ROOT", str(RUNTIME)); os.environ.setdefault("DAPIER_SO101_MJCF", str(MJCF))
    if str(CODEX) not in sys.path:
        sys.path.insert(0, str(CODEX))
    import importlib
    return {n: importlib.import_module(f"sweepick_261008_{n}") for n in ("receive_real_adapter", "receive_plan", "receive_entry", "receive_session", "full_episode")}


def tool_pose(model, q12, site="left_cube_grasp"):
    import mujoco
    import sweepick_kin
    d = mujoco.MjData(model); sweepick_kin.set_q12(model, d, np.asarray(q12, float)); mujoco.mj_forward(model, d)
    i = model.site(site).id; T = np.eye(4); T[:3, :3], T[:3, 3] = d.site_xmat[i].reshape(3, 3), d.site_xpos[i]
    return T, d


def held_pose(q12_at_close, q12_now, block_centre_desk, block_yaw, *, model, pad_geoms, source):
    """What this session knows about the block in the left hand, with every part named by what it is:
    OBSERVED (read by a sensor in this state), ESTIMATED (computed from observations of an earlier state), ASSUMED
    (a physical assumption), UNIDENTIFIED. Same vocabulary as the implementation's version of this function: `known`, `missing`,
    `object_pose_model` is None until the pose in the hand has been OBSERVED. The 4x4 put together from the estimated and
    assumed parts is given separately as `coarse_preview_pose`: good for previewing the carry and the right approach,
    not for putting the right jaws around the block."""
    Tc, dc = tool_pose(model, q12_at_close); Tn, _ = tool_pose(model, q12_now)
    pads = [dc.geom_xpos[model.geom(n).id].copy() for n in pad_geoms]; mid = (pads[0] + pads[1]) / 2; jaw = pads[1] - pads[0]; jaw[2] = 0; jaw /= np.linalg.norm(jaw)
    c = np.array(block_centre_desk, float); c[:2] += jaw[:2] * float(np.dot(mid[:2] - c[:2], jaw[:2]))
    yaw = float(np.arctan2(jaw[1], jaw[0]) - np.pi / 2); yaw += round((block_yaw - yaw) / (np.pi / 2)) * np.pi / 2
    Tb = np.eye(4); Tb[:3, :3] = [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]; Tb[:3, 3] = c; rel = np.linalg.inv(Tc) @ Tb
    comp = dict(along_jaw_axis=dict(cls="ASSUMED", basis="the closed pads put the block at their midpoint (pad compliance not known)"),
                across_jaw_axis=dict(cls="ESTIMATED", basis="top depth of the block on the desk before the pick through the candidate chain; 4 .. 14 mm off against the hand in the runs of 2026-10-08", uncertainty_m=0.014),
                height=dict(cls="ESTIMATED", basis="candidate desk height plus half the block at the close, carried rigidly with the tool"),
                yaw=dict(cls="ASSUMED", basis="squared to the jaws by the closed pads", uncertainty_deg=3.0), tilt=dict(cls="ASSUMED", basis="the block stood flat on the desk when it was closed on"), slip_after_close=dict(cls="UNIDENTIFIED"))
    return dict(object_pose_model=None, tool_from_object=None, status="MISSING", object_pose_source=source, kind="partial grasp evidence; candidate mapping and frame chain", components=comp,
                known=dict(desk_centre_before_pick=list(map(float, block_centre_desk)), desk_yaw_before_pick=float(block_yaw), candidate_pad_midpoint_at_close=mid.tolist(), candidate_tool_motion=(Tn @ np.linalg.inv(Tc)).tolist()),
                object_pose_uncertainty=dict(measured=False, height="MISSING", across_pad_width="MISSING", orientation="not identified in the model frame", slip_after_close="not observed"),
                missing=["held object height/tool offset", "held across-pad-width offset", "held orientation in model frame"], pose_role="COARSE_PREVIEW", final_insertion_ready=False,
                observed_components=["measured joints at close and now", "desk observation before close"], estimated_components=["tool motion by candidate FK", "pad midpoint along jaw axis"],
                assumed_components=["upright desk object", "nearest face squared by pads", "no slip after close"], unidentified_components=["current transverse offset", "pad compliance", "slip", "current orientation error"],
                coarse_preview_pose=(Tn @ rel).tolist(), coarse_tool_from_object=rel.tolist(), coarse_use="carry and right-approach PREVIEW only; the right insert needs the block OBSERVED in the hand")


def held_geometry_reference(path, *, model_sha, mapping_sha, planner_options, camera_source_sha, source_kind="REAL"):
    """Consume independent object/right-jaw measurements; a camera plane fit is not such a measurement."""
    if not path:
        raise ValueError("optional measured held/right-jaw error reference is absent")
    p = Path(path).expanduser().resolve(); doc = json.loads(p.read_text())
    if (doc.get("schema") != "sweepick.held-right-geometry.v1" or doc.get("source_kind") != source_kind
            or doc.get("scope") != "RECEIVE_CONTACT"):
        raise ValueError("geometry reference schema/source/scope does not match this contact session")
    binding = doc.get("binding") or {}
    expected = dict(model_sha256=model_sha, mapping_sha256=mapping_sha,
                    planner_candidate=(planner_options or {}).get("candidate"), numerical_seed=(planner_options or {}).get("right_roll_seed"), camera_chain_source_sha256=camera_source_sha)
    if any(binding.get(k) != v for k, v in expected.items()):
        raise ValueError("geometry reference model/mapping/selected candidate binding differs")
    if doc.get("relative_frame") != "physical right inner-pad midpoint and jaw axes, expressed in model right_cube_grasp convention":
        raise ValueError("geometry measurements must use the identified physical right-jaw frame")
    if doc.get("applies_to") != ["updated_pregrasp", "insert_path", "before_close"]:
        raise ValueError("geometry measurement coverage does not include the pregrasp, insertion path and close view")
    evidence = doc.get("measurement_evidence") or []
    if not evidence or not isinstance(doc.get("measurement_method"), str) or not doc["measurement_method"].strip():
        raise ValueError("independent measurement method and raw evidence are required")
    calibration = doc.get("calibration_sources") or []
    K = np.asarray(binding.get("top_intrinsics"), float)
    if not calibration or K.shape != (3, 3) or not np.isfinite(K).all():
        raise ValueError("camera calibration sources and registered top intrinsic revision are required")
    for e in evidence + calibration:
        f = Path(e["file"]); f = f if f.is_absolute() else p.parent / f
        if hashlib.sha256(f.read_bytes()).hexdigest() != e.get("sha256"):
            raise ValueError("geometry measurement evidence SHA differs")
    def pose(value):
        T = np.asarray(value, float)
        if (T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1])
                or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3)) or not np.isclose(np.linalg.det(T[:3, :3]), 1)):
            raise ValueError("geometry reference has an invalid rigid transform")
        return T
    centre, angle, pad_gap = [], [], []
    samples = doc.get("samples") or []
    if not samples or {s.get("stage") for s in samples} != set(doc["applies_to"]):
        raise ValueError("independent paired samples for every covered stage are required")
    for s in samples:
        estimated, measured = pose(s["estimated_right_from_object"]), pose(s["measured_right_from_object"])
        resolution = [s.get("measurement_centre_uncertainty_m"), s.get("measurement_rotation_uncertainty_rad")]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0 for v in resolution):
            raise ValueError("identified nonzero metrology uncertainty in metres/radians is required")
        if not s.get("independent_measurement_source") or not s.get("frame_identity") or not s.get("feedback_identity"):
            raise ValueError("paired sample lacks independent metrology, frame or feedback provenance")
        centre.append(float(np.linalg.norm(estimated[:3, 3] - measured[:3, 3]) + resolution[0]))
        a = math.acos(float(np.clip((np.trace(estimated[:3, :3].T @ measured[:3, :3]) - 1) / 2, -1, 1)))
        angle.append(a + resolution[1])
        gaps = [s.get("estimated_inner_pad_gap_m"), s.get("measured_inner_pad_gap_m"), s.get("gap_measurement_uncertainty_m")]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0 for v in gaps):
            raise ValueError("independently measured open inner-pad gap and its uncertainty are required")
        pad_gap.append((abs(gaps[0] - gaps[1]) + gaps[2]) / 2)
    return dict(measured=True, centre_m=max(centre), rotation_angle_bound_rad=max(angle),
                rotation_bound_source=f"{p} sha256 {hashlib.sha256(p.read_bytes()).hexdigest()}: independent paired physical right-jaw/object measurements plus metrology uncertainty",
                source_kind=source_kind, classification="PROVISIONAL measured relative-error envelope; not a global certified physical bound",
                bound_scope="right-hand/block clearance of the bound RECEIVE_CONTACT candidate only",
                hard_total_bound_claimed=False, inner_pad_half_gap_uncertainty_m=max(pad_gap), binding=binding, measurement_evidence=evidence, calibration_sources=calibration,
                samples=len(samples), applies_to=doc["applies_to"], relative_frame=doc["relative_frame"])


def held_seen(depth_mm, K, T_world_camera, predicted, *, model, q12, half=0.02, color=None, geometry_reference=None, back_project=None, inside=None, diagnostics=None, prepared_geometry=None):
    """Measure a current held face; the predicted pose seeds detection and supplies only hidden-size/symmetry priors."""
    import mujoco
    import sweepick_kin
    import sweepick_pick01 as p1
    d = mujoco.MjData(model); sweepick_kin.set_q12(model, d, np.asarray(q12, float)); mujoco.mj_forward(model, d)
    import tjj_perception as tp
    from sweepick_261009_offsite_relative_features import observed_face_from_seed
    diagnostic = diagnostics if diagnostics is not None else {}
    diagnostic.update(status="MISSING", search_prior_role="SEARCH_PRIOR", minimum_points=50)
    def unavailable(reason):
        diagnostic["missing_reason"] = reason
        return None
    depth_mm = np.asarray(depth_mm, float); depth = depth_mm / 1000.0
    T = np.array(T_world_camera, float)
    image_shape = np.asarray(color).shape[:2] if color is not None else (depth.shape[0] * 2, depth.shape[1] * 2)
    Kd = np.array(K, float).reshape(3, 3).copy()
    Kd[0] *= depth.shape[1] / image_shape[1]; Kd[1] *= depth.shape[0] / image_shape[0]
    pts = (back_project or tp.back_project)(depth, Kd, T)
    valid_depth = np.isfinite(depth) & (depth >= 0.05) & (depth <= 2.5)
    diagnostic.update(raw_valid_points=int(valid_depth.sum()), xyz_points=len(pts), K_depth=Kd.tolist(),
                      rgb_shape=list(image_shape), depth_shape=list(depth.shape), search_prior=np.asarray(predicted).tolist())
    diagnostic["_masks"] = dict(raw_valid=valid_depth)
    if len(pts) != np.count_nonzero(valid_depth):
        return unavailable("DEPTH_XYZ_PIXEL_CORRESPONDENCE_MISMATCH")
    xyz = np.full((*depth.shape, 3), np.nan, float); xyz[valid_depth] = pts
    P = np.array(predicted, float)
    delta = pts - P[:3, 3]
    near_before = (np.hypot(delta[:, 0], delta[:, 1]) < 2 * half) & (np.abs(delta[:, 2]) < 2 * half)
    permitted = np.zeros(depth.shape, bool)
    if inside is None:
        from sweepick_261010_observed_robot_mask import observed_robot_mask
        diagnostic['_robot_exclusion'] = {}
        removed = observed_robot_mask(model, d, pts, near_before, 0.004, diagnostics=diagnostic['_robot_exclusion'], prepared=None if prepared_geometry is None else prepared_geometry['robot_mask'])
    else:
        removed = inside(model, d, pts, "", 0.004)
    permitted[valid_depth] = ~removed
    near = near_before & permitted[valid_depth]
    seed = np.zeros(depth.shape, bool); seed[valid_depth] = near
    before = np.zeros(depth.shape, bool); before[valid_depth] = near_before
    diagnostic.update(search_before_robot_points=int(near_before.sum()), search_after_robot_points=int(near.sum()),
                      robot_removed_search_points=int((near_before & ~permitted[valid_depth]).sum()))
    diagnostic["_masks"].update(search_before_robot=before, search_after_robot=seed, robot_permitted=permitted)
    if np.count_nonzero(seed) < 50:
        return unavailable("ROBOT_MASK_REMOVES_SEARCH_SUPPORT" if near_before.sum() >= 50 else "SEARCH_PRIOR_DEPTH_SUPPORT_INSUFFICIENT")
    image = np.zeros((*depth.shape, 3), np.uint8) if color is None else color
    diagnostic['_face_diagnostic'] = {}
    feature, missing = observed_face_from_seed(
        depth_mm, Kd, image, seed, image_shape=depth_mm.shape, xyz_image=xyz, allowed_mask=permitted, minimum_points=50,
        T_output_camera=T, diagnostics=diagnostic['_face_diagnostic'])
    if feature is None:
        return unavailable("FACE_REACQUISITION: " + str(missing))
    plane_centre = np.asarray(feature["plane_centre"], float)
    diagnostic["_masks"]["plane_inlier"] = feature["_mask"]
    diagnostic["_feature_masks"] = feature.get("_diagnostic_masks", {})
    diagnostic["feature"] = {key: value for key, value in feature.items() if not key.startswith("_")}
    normal = np.asarray(feature["plane_normal"], float)
    if np.dot(normal, T[:3, 3] - plane_centre) < 0:
        normal = -normal
    observed_edges = feature["visible_edges"]
    observed_face_centroid = feature.get("visible_face_centroid")
    centre_identified = False
    unconstrained_centre = []
    if observed_face_centroid is not None:
        face_centroid = np.asarray(observed_face_centroid, float)
        centre_identified = True
        centre_basis = "OBSERVED_FULL_RGB_DEPTH_BOUNDARY"
        face_centroid_class = "OBSERVED"
    elif len(observed_edges) >= 2:
        first, second = observed_edges[:2]
        inward = [np.asarray(first["inward_direction"], float), np.asarray(second["inward_direction"], float)]
        rows = np.vstack([inward[0], inward[1], normal])
        if abs(np.linalg.det(rows)) > np.finfo(float).eps:
            edge_points = [np.asarray(first["points"], float).mean(axis=0),
                           np.asarray(second["points"], float).mean(axis=0)]
            rhs = np.array([np.dot(inward[index], edge_points[index]) + half for index in range(2)]
                           + [np.dot(normal, plane_centre)])
            face_centroid = np.linalg.solve(rows, rhs)
            centre_identified = True
            centre_basis = "MODEL_PRIOR_HALF_EXTENT_FROM_TWO_ACTUAL_EXTERIOR_EDGES"
            face_centroid_class = "MODEL_PRIOR_EDGE_OFFSET"
        else:
            face_centroid = P[:3, 3] + normal * np.dot(normal, plane_centre - P[:3, 3])
            centre_basis = "MEASURED_PLANE_WITH_PREDICTED_TANGENTIAL_POSITION"
            face_centroid_class = "PARTIAL_MODEL_PRIOR"
            unconstrained_centre = ["two tangential centre directions"]
    elif observed_edges and observed_edges[0].get("inward_direction") is not None:
        edge = observed_edges[0]
        inward = np.asarray(edge["inward_direction"], float)
        direction = np.asarray(edge["direction"], float)
        edge_point = np.asarray(edge["points"], float).mean(axis=0)
        rows = np.vstack([inward, normal, direction])
        rhs = np.array([np.dot(inward, edge_point) + half,
                        np.dot(normal, plane_centre), np.dot(direction, P[:3, 3])])
        face_centroid = np.linalg.solve(rows, rhs) if abs(np.linalg.det(rows)) > np.finfo(float).eps else (
            P[:3, 3] + normal * np.dot(normal, plane_centre - P[:3, 3]))
        centre_basis = "MODEL_PRIOR_HALF_EXTENT_FROM_ONE_ACTUAL_EDGE_WITH_PREDICTED_EDGE_TANGENT"
        face_centroid_class = "PARTIAL_MODEL_PRIOR"
        unconstrained_centre = ["translation along the observed face edge"]
    else:
        face_centroid = P[:3, 3] + normal * np.dot(normal, plane_centre - P[:3, 3])
        centre_basis = "MEASURED_PLANE_WITH_PREDICTED_TANGENTIAL_POSITION"
        face_centroid_class = "PARTIAL_MODEL_PRIOR"
        unconstrained_centre = ["two tangential centre directions"]
    centre = face_centroid - half * normal
    rotation = None; edge_direction = None
    full_rotation = bool(color is not None and feature["orientation_identified"])
    if full_rotation:
        observed_axes = [np.asarray(edge["direction"], float) for edge in observed_edges]
        candidates = []
        for axis in observed_axes:
            axis -= normal * np.dot(normal, axis); axis /= np.linalg.norm(axis)
            for signed in (axis, -axis):
                other = np.cross(normal, signed); other /= np.linalg.norm(other)
                candidates.append((float(np.dot(signed, P[:3, 0]) + np.dot(other, P[:3, 1])), signed, other))
        _score, edge_direction, other_edge = max(candidates, key=lambda item: item[0])
        rotation = np.column_stack([edge_direction, other_edge, normal])
    edge_diag = dict(classification="PROVISIONAL" if full_rotation else "MISSING",
                     reason=("actual RGB/depth exterior edge(s) identify the in-plane cube axis" if full_rotation
                             else feature["edge_missing_reason"] or "top RGB was not supplied"),
                     edge_points=feature["observed_exterior_edge_pixels"],
                     visible_edges=feature["visible_edges"], observed_exterior_edges=feature["observed_exterior_edges"],
                     rejected_boundary_edges=feature["rejected_boundary_edges"],
                     visibility=feature["visibility"],
                     complete_face_boundary=feature["complete_face_boundary"],
                     occlusion=feature["occlusion"], observed_polygon_uv=feature["polygon"],
                     symmetry_choice=("MODEL_PRIOR predicted convention selects edge axis/sign among the observed cube 90/180-degree symmetries"
                                      if full_rotation else None))

    out = P.copy(); out[:3, 3] = centre
    if rotation is not None:
        out[:3, :3] = rotation
    components = dict(
        visible_face_centroid=dict(cls=face_centroid_class, basis=centre_basis),
        face_plane_normal=dict(cls="OBSERVED", basis="3-D plane fit; two orientation components"),
        centre_xyz=dict(cls="MODEL_PRIOR_OFFSET" if centre_identified else "PARTIAL_MODEL_PRIOR",
                        basis=centre_basis + "; A MODEL_PRIOR cube half-size offsets the hidden centre along the measured normal"),
        rotation_about_face_normal=dict(cls="PROVISIONAL" if full_rotation else "UNIDENTIFIED",
                                        basis="registered top RGB face edges" if full_rotation else edge_diag["reason"]),
        full_rotation=dict(cls="PROVISIONAL" if full_rotation else "MISSING",
                           basis=("plane normal plus coherent actual RGB-D exterior edges; MODEL_PRIOR selects cube-axis symmetry"
                                  if full_rotation else "face normal alone leaves one axis unidentified")),
        slip_after_close=dict(cls="UNIDENTIFIED"))
    uncertainty = dict(measured=False, hard_total_bound_claimed=False,
                       depth_error_correlation="registered depth errors are correlated",
                       calibration_bound="MISSING", orientation_clearance_bound="MISSING",
                       centre_identified=centre_identified,
                       note="face components are measured, but no measured relative-error reference is connected")
    if full_rotation and geometry_reference is not None:
        uncertainty.update(geometry_reference)
    diagnostic.update(status="OBSERVED" if centre_identified and full_rotation else "PARTIAL_OBSERVATION",
                      missing_reason=None if centre_identified and full_rotation else
                      "; ".join(unconstrained_centre + ([] if full_rotation else [edge_diag["reason"]])),
                      centre_identified=centre_identified, orientation_identified=full_rotation)
    return dict(object_pose_model=out.tolist(), rotation_model=None if rotation is None else rotation.tolist(),
                face_normal_model=normal.tolist(), face_edge_direction_model=None if edge_direction is None else edge_direction.tolist(),
                visible_face_centroid_model=None if observed_face_centroid is None else face_centroid.tolist(),
                face_centre_model=face_centroid.tolist(), centre_identified=centre_identified,
                centre_observation=dict(identified=centre_identified, classification=face_centroid_class,
                                        basis=centre_basis, unconstrained=unconstrained_centre,
                                        hidden_normal_offset=dict(classification="MODEL_PRIOR", half_extent_m=half)),
                centre_model_prior_offset_m=(-half * normal).tolist(),
                points=feature["depth_pixels"], moved_from_estimate_m=[float(v) for v in centre - P[:3, 3]], components=components,
                observed_components=(["visible face centroid from its complete actual RGB-D boundary"] if observed_face_centroid is not None else []) +
                                    ["3-D face-plane normal"] + (["RGB face-edge direction"] if full_rotation else []),
                missing_components=(([] if centre_identified else unconstrained_centre) +
                                    ([] if full_rotation else ["rotation about the observed face normal"])),
                orientation_observation=dict(classification="PROVISIONAL", full_rotation_observed=full_rotation,
                                             symmetry_choice=edge_diag["symmetry_choice"],
                                             calibration_bound=uncertainty.get("rotation_bound_source", "MISSING"), orientation_clearance_bound=uncertainty.get("bound_scope", "MISSING")),
                fit_diagnostics=dict(plane_points=feature["depth_pixels"], **feature["fit_diagnostics"], edge=edge_diag),
                object_geometry=dict(classification="MODEL_PRIOR", half_extents_m=[half] * 3,
                                     source="existing A block model; visible patch is not a measurement of hidden dimensions"),
                object_pose_uncertainty=uncertainty)


def save_vision_diagnostics(folder, rgb, held, relative):
    """Save produced masks and their pixel provenance beside the original consumed FrameSet."""
    import csv
    import cv2
    masks, rows = {}, []
    def extract(value, path):
        if isinstance(value, np.ndarray):
            key = path.replace(".", "__")
            masks[key] = value
            if value.dtype == bool and value.ndim == 2:
                rows.append(dict(source=path.split(".")[0], stage=path, points=int(value.sum()), reason=""))
            return dict(array_key=key, shape=list(value.shape), dtype=str(value.dtype))
        if isinstance(value, dict):
            return {k: extract(v, path + "." + str(k)) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [extract(v, path + "." + str(k)) for k, v in enumerate(value)]
        return _plain(value)
    summary = dict(held=extract(held, "held"), relative=extract(relative, "relative"),
                   pixel_domain="original registered-depth pixels; original RGB/depth/K/q/timestamps are adjacent")
    for name, record in (("held", held), ("relative", relative)):
        rows.append(dict(source=name, stage="result", points="", reason=record.get("missing_reason") or record.get("status", "")))
    if masks:
        np.savez_compressed(folder / "VISION_MASKS.npz", **masks)
    with (folder / "FILTER_STAGE_SUMMARY.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=["source", "stage", "points", "reason"])
        writer.writeheader(); writer.writerows(rows)
    (folder / "OBSERVATION_DIAGNOSTICS.json").write_text(json.dumps(summary, indent=2) + "\n")
    if rgb is not None:
        overlay = np.asarray(rgb).copy()
        for label, colour in (("search_before_robot", (0, 165, 255)), ("search_after_robot", (255, 255, 0)), ("plane_inlier", (0, 255, 0))):
            mask = (held.get("_masks") or {}).get(label)
            if mask is not None:
                resized = cv2.resize(np.asarray(mask, np.uint8), (overlay.shape[1], overlay.shape[0]), interpolation=cv2.INTER_NEAREST) != 0
                overlay[resized] = colour
        feature = held.get("feature") or {}
        for edge in feature.get("visible_edges", []):
            uv = np.asarray(edge["uv"], float)
            depth_shape = held.get("depth_shape")
            if depth_shape is not None:
                uv *= np.array([overlay.shape[1] / depth_shape[1], overlay.shape[0] / depth_shape[0]])
                cv2.polylines(overlay, [np.rint(uv).astype(np.int32)], False, (255, 255, 255), 2)
        cv2.putText(overlay, "SEARCH orange / retained cyan / observed plane green; " + str(held.get("status", "NOT_RUN")),
                    (12, 26), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
        if not cv2.imwrite(str(folder / "PLANE_EDGE_OVERLAY.png"), overlay):
            raise OSError("diagnostic plane/edge overlay not saved")
    return summary


def scene_bindings(model, look_scene, table_z, *, model_sha, source_sha):
    """The frames the implementation's adapter asks for: the table's own centre and orientation with the recorded candidate surface height (the implementation's version of this function)."""
    import mujoco
    d = mujoco.MjData(model); mujoco.mj_forward(model, d)
    def body(n):
        T = np.eye(4); b = d.body(n); T[:3, 3], T[:3, :3] = b.xpos, b.xmat.reshape(3, 3); return T.tolist()
    g = model.geom("table"); table = d.geom("table"); Tt = np.eye(4); Tt[:3, :3] = table.xmat.reshape(3, 3); Tt[:3, 3] = table.xpos + Tt[:3, 2] * g.size[2]; Tt[2, 3] = float(table_z)
    return dict(frame="assembled dual model world (left base at its model pose)", model_sha256=model_sha, source_sha256=source_sha, table_top_z_m=float(table_z), T_model_left_base=body("left_base"), T_model_right_base=body("right_base"),
                T_model_camera=look_scene.get("T_world_camera"), T_model_table=Tt.tolist(), T_task_from_model=np.eye(4).tolist(), provenance="candidate compiled bases/table centre + recorded camera/chain height; compiler checks both bases", geometry_verified=False)


def field_refs():
    """The three records only the real robot can give. Each: the loaded document or None, with what it is and what waits for it."""
    out = {}
    for name, (path, what, blocks) in FIELD.items():
        doc = None
        if path.exists():
            try:
                doc = json.loads(path.read_text())
            except Exception as e:
                doc = None; what = f"{what} (file unreadable: {e})"
        ok = bool(doc) and doc.get("source_kind") == "REAL" and bool(doc.get("measurement_evidence")) and doc.get("field_confirmed", True) is True
        out[name] = dict(file=str(path), present=bool(doc), usable=ok, doc=doc if ok else None, what=what, blocks=blocks)
    return out


class Scoped:
    """the implementation's FullEpisodeHandoff with the scope of this session said out loud. FULL_TASK: unchanged. SUPPORT_OBSERVATION: the
    same state machine, told that scope, runs up to the right contact and the load-transfer lift and ends at the observation
    point by its own contract (no support record is asked for, no release is possible). Then the owner's planned return."""
    def __init__(self, inner, owner, scope):
        self.inner, self.owner, self.scope, self.abandon, self.area = inner, owner, scope, owner.abandon, getattr(inner, "area", None)

    def ready(self, out, state):
        r = self.inner.ready(out, state); r["scope"] = self.scope; return r

    def __call__(self, ctx):
        if self.scope in ("approach", "contact"):
            return self._approach(ctx)
        h = self.inner(ctx); o = self.owner
        if self.scope != "observe":
            return h
        # Two different ends. (1) The OBSERVATION POINT: the state machine itself ends there (pin: OBSERVATION_COMPLETE ->
        # SUPPORT_OBSERVATION_ONLY; both hands on the block, nothing released, not a success of any task). What was observed is
        # written out at once, whatever follows. (2) The SESSION's end: this owner's planned return, run only from that point and
        # only when fresh readings allow it; otherwise the arms stay as they are and the result says so.
        point = h.get("result") == "SUPPORT_OBSERVATION_ONLY"
        o._safe(lambda: o.write_observation(None, "AT_THE_OBSERVATION_POINT" if point else f"ENDED_BEFORE_THE_OBSERVATION_POINT: {h.get('result')}: {h.get('why')}"))
        common = dict(scope="SUPPORT_OBSERVATION", full_task=False, release_allowed=False, support_confirmed=False, left_released=False, state_machine_result=h.get("result"), state_machine_why=h.get("why"), stages=h.get("stages"), recording_errors=h.get("recording_errors"), observation_file="SUPPORT_OBSERVATION.json")
        if not point:
            o.note("support_observation_outcome", result=h.get("result"), normal_end=False, why=h.get("why"), full_task=False); return dict(h, **{k: v for k, v in common.items() if k not in h})
        ep = o.ep
        try:
            if ep is not None:
                with ep.lease("left", "right"):
                    end = o.finish_observation()
            else:
                end = o.finish_observation()
        except Exception as e:
            end = dict(returned=False, why=[f"{type(e).__name__}: {e}"], left_at_lift=False, state="RETURN_NOT_RUN_A_PERSON_DECIDES")
        o._safe(lambda: o.write_observation(end, "OBSERVATION_COMPLETE" + ("_AND_RETURNED" if end.get("returned") else "_NOT_RETURNED")))
        ok = bool(end.get("returned") and end.get("left_at_lift") and end.get("left_still_loaded") and end.get("right_released"))
        o.note("support_observation_outcome", result="OBSERVED" if ok else "SUPPORT_OBSERVATION_ONLY", normal_end=ok, end=end, full_task=False)
        return dict(common, result="OBSERVED" if ok else "SUPPORT_OBSERVATION_ONLY", commands_sent=True, object_owner="LEFT" if ok else "UNKNOWN", left_at_lift=bool(end.get("left_at_lift")), end=end,
                    why=("right contact and load-transfer observation done; the left hand was not released; the right arm is home and released; the left hand holds at the pose it lifted to" if ok
                         else "the observation was made; the planned return was not completed: " + "; ".join(end.get("why") or [str(end.get("state"))])))

    def _approach(self, context):
        """Run the existing RECEIVE prefix and its prechecked normal return; RECEIVE_CONTACT may add one checked finite suffix."""
        inner, o, is_contact = self.inner, self.owner, self.scope == "contact"
        observation_only = is_contact and (o.contact_plan or {}).get("observation_only") is True

        def sent():
            rows = o.notes[note0:]
            return sum(sum(int(v or 0) for v in (row.get("writes") or {}).values()) for row in rows if row.get("kind") in ("segment", "right_contact_close"))

        def answer(result, why, **extra):
            writes = sent()
            external_scope = "RECEIVE_CONTACT" if is_contact else "RECEIVE_APPROACH"
            out = dict(result=result, why=why, scope=external_scope, full_task=False, release_allowed=False,
                       support_confirmed=False, left_released=False, left_released_in_the_hand_over=False,
                       commands_sent=bool(writes), commands_sent_count=writes, object_owner="LEFT" if o.block_owner == "left" and o.stopped is None else "UNKNOWN",
                       system_success=False, clean_resume_request_emitted=False, cleaning_succeeded=False,
                       recording_errors=list(getattr(inner, "recording_errors", [])))
            out.update(extra)
            try:
                inner.recorder.event("receive_contact_outcome" if is_contact else "receive_approach_outcome", result=result, why=why,
                                     commands_sent=out["commands_sent"], normal_end=bool((out.get("end") or {}).get("returned")))
            except Exception as exc:
                out["recording_errors"].append(f"{type(exc).__name__}: {exc}")
            return out

        note0 = len(o.notes)
        if getattr(inner, "_consumed", False):
            return answer("NOT_READY", "this handoff callback was already consumed", missing=["fresh full-episode callback"])
        inner._consumed = True
        missing = []
        try:
            out = Path(context.get("out", "")).resolve() if isinstance(context, dict) else None
        except (TypeError, ValueError) as exc:
            out = None; missing.append(f"output directory: {type(exc).__name__}: {exc}")
        if not isinstance(context, dict):
            missing.append("handoff context")
        else:
            if not inner._readiness or not inner._readiness.get("ready"):
                missing.extend((inner._readiness or {}).get("missing", ["successful ready(out, state) check"]))
            if inner._ready_out != out:
                missing.append("same output directory used by ready(out, state)")
            if context.get("episode") is not inner.recorder:
                missing.append("same Episode recorder object")
            if not isinstance(context.get("scene"), dict):
                missing.append("fresh lift scene")
            if not isinstance(context.get("evidence"), dict):
                missing.append("lift evidence")
        snapshot = None
        if out is not None:
            try:
                snapshot = json.loads((out / "HANDOFF_SNAPSHOT.json").read_text())
            except Exception as exc:
                missing.append(f"newly saved HANDOFF_SNAPSHOT.json: {type(exc).__name__}: {exc}")
        if missing:
            return answer("NOT_READY", ("receive-contact" if is_contact else "receive-approach") + " input is incomplete", missing=list(dict.fromkeys(missing)))

        preparation, path, suffix, stages, approach_observation, contact_log, insertion = None, None, None, [], None, None, None
        metrology = None
        try:
            with inner.recorder.lease("left", "right"):
                preparation = inner.prepare(snapshot, context["scene"], context["evidence"])
                raw0 = o._read()
                left_jaw_goal = raw0["goal"]["left"]["gripper"]
                right_jaw_goal = raw0["goal"]["right"]["gripper"]
                right_jaw_torque = raw0["torque"]["right"]["gripper"]

                def live(raw, phase, right_jaw_state="INITIAL"):
                    reasons = []
                    if o.stopped is not None or Path(str(o.out) + ".STOP").exists():
                        reasons.append("operator STOP or an earlier executor stop is active")
                    faults = {side: {n: v for n, v in raw["status"][side].items() if v} for side in ("left", "right")}
                    if any(faults.values()):
                        reasons.append(f"servo Status register is non-zero: {faults}")
                    if not all(raw["torque"]["left"][n] for n in ALL):
                        reasons.append("the left lift is no longer powered and holding on all six joints")
                    if round(raw["goal"]["left"]["gripper"]) != round(left_jaw_goal):
                        reasons.append("the left Jaw hold Goal_Position changed")
                    if right_jaw_state == "ON":
                        if not raw["torque"]["right"]["gripper"]:
                            reasons.append("the right Jaw is not ON in the contact phase")
                    elif right_jaw_state == "NORMAL_OFF":
                        if raw["torque"]["right"]["gripper"]:
                            reasons.append("the right Jaw is still ON after the planned normal return")
                        if round(raw["goal"]["right"]["gripper"]) != round(path["initial_ticks"]["right"]["gripper"]):
                            reasons.append("the right Jaw Goal_Position is not the planned reopened tick")
                    else:
                        if raw["torque"]["right"]["gripper"] != right_jaw_torque:
                            reasons.append("the right Jaw torque state changed outside the contact phase")
                        if round(raw["goal"]["right"]["gripper"]) != round(right_jaw_goal):
                            reasons.append("the right Jaw Goal_Position changed outside the contact phase")
                    if reasons:
                        raise RuntimeError(f"{phase}: " + "; ".join(reasons))

                live(raw0, "before " + ("RECEIVE_CONTACT" if is_contact else "RECEIVE_APPROACH") + " planning")
                if not isinstance(preparation, dict) or preparation.get("status") != "PLAN_PREVIEW":
                    prep = preparation if isinstance(preparation, dict) else {}
                    return answer("NOT_READY", prep.get("detail", "receive-approach preparation is not ready"),
                                  missing=list(prep.get("missing") or []), preparation=prep)
                path = o.plan_approach_prefix(o.approach_plan, raw0)
                (o.out / "RECEIVE_APPROACH_PLAN.json").write_text(json.dumps(_plain(path), indent=1))
                actual = o._read(); live(actual, "after RECEIVE_APPROACH planning"); o.track_held(actual)
                if not path.get("ready"):
                    return answer("NOT_READY", path.get("why", "the complete approach and return did not pass its checks"),
                                  missing=list(path.get("missing") or []), preparation=preparation, path=path)

                if is_contact:
                    observed, actual, _ = o.contact_observation("AFTER_LIFT_BEFORE_CARRY", after=o.clock(), previous=copy.deepcopy(o.last_images))
                    live(actual, "after fresh lift observation"); o.track_held(actual)
                    stages.append(dict(stage="AFTER_LIFT_BEFORE_CARRY", observation=_plain(observed)))

                for step in path["forward"]:
                    if is_contact and step['label']=='RIGHT_APPROACH':
                        observed, actual, _ = o.contact_observation("BEFORE_RIGHT_PREGRASP", after=o.clock(), previous=copy.deepcopy(o.last_images))
                        live(actual, "before right pregrasp observation"); o.track_held(actual)
                        o.before_right_pregrasp_observation=copy.deepcopy(observed)
                        o.before_right_pregrasp_q12=np.asarray(observed['q12'],float).copy()
                        stages.append(dict(stage="BEFORE_RIGHT_PREGRASP", observation=_plain(observed)))
                    logs = o.go("approach_" + step["label"].lower(), step["target_q12"], arms=(step["side"],))
                    actual = o._read(); live(actual, f"after {step['label']}"); o.track_held(actual); o.done.add(step["label"])
                    stages.append(dict(stage=step["label"], side=step["side"], result=logs[step["side"]]["result"],
                                       measured_ticks={n: actual["ticks"][step["side"]][n] for n in ARM}))
                    if is_contact and step['label'].startswith('CARRY_'):
                        observed, actual, _ = o.contact_observation("AFTER_"+step['label'], after=o.clock(), previous=copy.deepcopy(o.last_images))
                        live(actual, "after carry observation"); o.track_held(actual)
                        stages.append(dict(stage="AFTER_"+step['label'], observation=_plain(observed)))

                if is_contact:
                    observed, _, _ = o.contact_observation("RECEIVE_CONTACT", after=o.clock(), previous=getattr(o, "last_images", {}))
                else:
                    observed = o.observation("RECEIVE_APPROACH", None, o.clock())
                approach_observation = _plain(observed)
                approach_observation.update(right_jaw_commanded_by_prefix=False,
                                            right_jaw_goal_basis="fresh Goal_Position register readback only; the prefix did not apply a right Jaw command")
                observation_file = "RECEIVE_CONTACT_OBSERVATION.json" if is_contact else "RECEIVE_APPROACH_OBSERVATION.json"
                (o.out / observation_file).write_text(json.dumps(approach_observation, indent=1))
                prefix_raw = o.last_raw
                live(prefix_raw, "at the " + ("RECEIVE_CONTACT" if is_contact else "RECEIVE_APPROACH") + " observation")
                o.track_held(prefix_raw)
                right_changed = [n for n in ARM if prefix_raw["ticks"]["right"][n] != raw0["ticks"]["right"][n]]
                left_changed = [n for n in ARM if prefix_raw["ticks"]["left"][n] != raw0["ticks"]["left"][n]]
                if is_contact:
                    insertion_ready = o.insertion_ready(observed)
                    if not insertion_ready:
                        observed, prefix_raw, insertion_ready = o.acquire_insertion_observation(
                            observed, path, live, stages, stage="PLAN")
                    insertion = dict((o.insert or {}).get("record") or {})
                    if not insertion:
                        insertion = next((dict(n) for n in reversed(o.notes) if n.get("kind") == "insertion_check"), dict(ready=False, why="no insertion-ready record"))
                    if observation_only:
                        insertion.update(status="READY_FOR_INSERTION_PLANNING" if insertion_ready else "NOT_READY",
                                         observation_only=True, actual_insert_permitted=False,
                                         actual_contact_permitted=False, live_execution_permitted=False)
                        if isinstance(o.insert, dict):
                            o.insert["record"] = dict(insertion)
                        o.note("observation_only_insertion_decision", record=insertion)
                        if (o.contact_plan or {}).get("right_x_metrology") is not None:
                            metrology = o.run_right_x_metrology(path, live, stages)   # its executed moves are undone by the normal reverse below
                    approach_observation["insertion"] = _plain(insertion)
                    (o.out / observation_file).write_text(json.dumps(approach_observation, indent=1))
                    if insertion_ready and not observation_only:
                        suffix = o.plan_contact_suffix(prefix_raw, path)
                        (o.out / "RECEIVE_CONTACT_PLAN.json").write_text(json.dumps(_plain(suffix), indent=1))
                        if not suffix.get("ready"):
                            insertion_ready = False
                            insertion = dict(insertion, ready=False, why=suffix.get("why"), missing=suffix.get("missing"))
                    if insertion_ready and not observation_only:
                        step = suffix["forward"][0]
                        log = o.segment("contact_" + step["label"].lower(), "right", target=step["target_ticks"], purpose="RIGHT_INSERT_REPLAN")
                        o.contact_pregrasp_history = dict(start_ticks={n: log["actual_start_ticks"][n] for n in ARM},
                                                         end_ticks={n: log["end"]["ticks"][n] for n in ARM}, result=log["result"])
                        reached = o.clock(); before = copy.deepcopy(o.last_images)
                        observed, actual, _ = o.contact_observation("AFTER_PREGRASP", after=reached, previous=before)
                        live(actual, "after updated pregrasp"); o.contact_pregrasp_reached_ticks = {n: actual["ticks"]["right"][n] for n in ARM}
                        stages.append(dict(stage=step["label"], result=log["result"], target_ticks=step["target_ticks"], target_revision=(o.insert or {}).get("for_seen_seq"), observation=_plain(observed)))
                        insertion_ready = o.insertion_ready(observed, stage="AFTER_PREGRASP")
                        if not insertion_ready:
                            observed, actual, insertion_ready = o.acquire_insertion_observation(
                                observed, path, live, stages, stage="AFTER_PREGRASP")
                        insertion = dict((o.insert or {}).get("record") or {})
                        if insertion_ready:
                            insertion_ready, suffix, insertion = o.run_insert_hops(path, live, stages)
                            if insertion_ready:
                                o.near_handover = True
                                contact_log = o.contact_close(); o.contact_close_executed = True
                                actual = o._read(); live(actual, "after RIGHT_CONTACT_CLOSE", "ON"); o.track_held(actual)
                                stages.append(dict(stage="RIGHT_CONTACT_CLOSE", side="right", result=contact_log["result"],
                                                   measured_tick=actual["ticks"]["right"]["gripper"], support_confirmed=False, grasp_confirmed=False))
                live(o._read(), "before the normal return", "ON" if contact_log is not None else "INITIAL")

                reverse = suffix["reverse"] if is_contact and suffix is not None and suffix.get("ready") and contact_log is not None else path["reverse"]
                if is_contact:
                    observed, actual, frames = o.contact_observation("BEFORE_RIGHT_REOPEN" if contact_log else "BEFORE_NORMAL_RETURN", after=o.clock(), previous=copy.deepcopy(o.last_images))
                    ok, why = o.recovery_ready("before RIGHT_CONTACT_OPEN" if contact_log else "before normal contact-scope return", observation=actual, frames=frames)
                    if not ok:
                        o.block_owner = "unknown"
                        raise RuntimeError(f"left support is UNKNOWN; right reopen/return not commanded: {why}")
                    o.block_owner = "left"
                    normal_path = o.plan_contact_suffix(actual, path, normal_only=True)
                    if not normal_path.get("ready"):
                        raise RuntimeError(f"current observed normal return is not ready: {normal_path.get('why')}")
                    reverse = normal_path["reverse"]
                home_log = left_log = None; jaw_expected = "ON" if contact_log is not None else "INITIAL"
                for step in reverse:
                    live(o._read(), f"before {step['label']}", jaw_expected)
                    name = ("contact_" if is_contact else "approach_") + step["label"].lower()
                    if step["label"] == "RIGHT_RETURN_INITIAL":
                        log = o.segment(name, "right", rest=step["target_ticks"]); home_log = log; o.near_handover = False; jaw_expected = "NORMAL_OFF" if contact_log is not None else "INITIAL"
                    else:
                        log = o.segment(name, step["side"], target=step["target_ticks"])
                        if step["label"] == "RIGHT_BACK_TO_UPDATED_PREGRASP":
                            o.near_handover = False
                        if step["label"] == "LEFT_BACK_TO_LIFT":
                            left_log = log
                    actual = o._read(); live(actual, f"after {step['label']}", jaw_expected); o.track_held(actual)
                    stages.append(dict(stage=step["label"], side=step["side"], result=log["result"], measured_ticks={n: actual["ticks"][step["side"]][n] for n in ARM}))
                if home_log is None or left_log is None:
                    raise RuntimeError("the normal reverse did not include the right initial return and left lift return")

                right_released = home_log["result"] == "RETURNED_RELEASED" and not any(actual["torque"]["right"][n] for n in (ALL if is_contact else ARM))
                left_at_lift = left_log["result"] == "HOLDING" and all(actual["torque"]["left"][n] for n in ALL)
                right_jaw_torque_unchanged = actual["torque"]["right"]["gripper"] == right_jaw_torque
                right_jaw_goal_unchanged = round(actual["goal"]["right"]["gripper"]) == round(path["initial_ticks"]["right"]["gripper"] if contact_log is not None else right_jaw_goal)
                left_jaw_hold_kept = round(actual["goal"]["left"]["gripper"]) == round(left_jaw_goal)
                right_motion = dict(status="OBSERVED" if right_changed else "NOT_OBSERVED", changed_joints=right_changed,
                                    start_ticks={n: raw0["ticks"]["right"][n] for n in ARM}, approach_readback_ticks={n: prefix_raw["ticks"]["right"][n] for n in ARM})
                left_motion = dict(status="OBSERVED" if left_changed else "NOT_OBSERVED", changed_joints=left_changed,
                                   start_ticks={n: raw0["ticks"]["left"][n] for n in ARM}, carry_readback_ticks={n: prefix_raw["ticks"]["left"][n] for n in ARM})
                normal = bool(right_released and left_at_lift and right_changed and left_changed and right_jaw_torque_unchanged
                              and right_jaw_goal_unchanged and left_jaw_hold_kept)
                end = dict(returned=normal, right_released=right_released, right_arm5_released=right_released,
                           right_jaw_torque_unchanged=right_jaw_torque_unchanged, right_jaw_goal_unchanged=right_jaw_goal_unchanged,
                           right_jaw_expected_state=("OFF -> ON during contact collection -> OFF after planned return" if contact_log is not None else "OFF throughout; no concrete insert/contact command" if is_contact else "unchanged throughout RECEIVE_APPROACH"),
                           left_at_lift=left_at_lift, left_jaw_hold_kept=left_jaw_hold_kept,
                           left_jaw_goal_register_tick=actual["goal"]["left"]["gripper"], left_jaw_present_tick=actual["ticks"]["left"]["gripper"],
                           measured_ticks={side: actual["ticks"][side] for side in ("left", "right")})
                if is_contact:
                    candidate = bool(contact_log and contact_log.get("result") == "GRIP_CONTACT_HOLDING")
                    result = ("RECEIVE_CONTACT_COLLECTED" if candidate else "RECEIVE_CONTACT_NO_CONTACT") if contact_log is not None else "RECEIVE_CONTACT_NOT_READY_RETURNED"
                    if not normal:
                        result = "RECEIVE_CONTACT_RETURN_NOT_CONFIRMED"
                    why = ("a PROVISIONAL right contact candidate was collected; support/grasp stay false; the right side reopened/returned OFF and the left hand still owns the block" if candidate and normal
                           else "the finite right closing range stayed free; no contact was promoted; the planned return completed" if contact_log is not None and normal
                           else "insertion planning readiness was recorded; observation-only permission withheld insert/contact and the actual-state normal return completed" if observation_only and normal
                           else "the insert ran, but the new placement observation withheld the close; the healthy planned return completed" if normal and "RIGHT_INSERT" in o.done
                           else f"{len(o.contact_insert_history)} insert hops completed; the remaining insert/close was withheld and the actual-state normal return completed" if normal and o.contact_insert_history
                           else "no insert/close was sent; the observation gap was recorded and the healthy planned return completed" if normal
                           else "the contact scope or its normal return lacked an actual motion/readback condition")
                    if metrology is not None:
                        why += f"; RIGHT_X_FINITE_METROLOGY {metrology['status']}: {metrology.get('why')}"
                    return answer(result, why, preparation=preparation, path=path, suffix=suffix, stages=stages, right_x_metrology=metrology,
                                  approach_observation=approach_observation, observation_file=observation_file, insertion=insertion,
                                  contact=None if contact_log is None else dict(result=contact_log.get("result"), outcome=contact_log.get("outcome"), candidate=contact_log.get("contact_candidate"), folder=contact_log.get("folder")),
                                  contact_candidate=candidate, grasp_confirmed=False, real_arm_motion=right_motion,
                                  insert_path_completion=dict(completed_hops=len(o.contact_insert_history), complete="RIGHT_INSERT" in o.done, reached=o.contact_insert_history),
                                  left_carry_motion=left_motion, left_at_lift=left_at_lift, end=end)
                return answer("RECEIVE_APPROACH_OBSERVED" if normal else "RECEIVE_APPROACH_NOT_OBSERVED",
                              "right ARM5 approach was observed and the prechecked normal return completed; the left hand still owns the block" if normal
                              else "the prefix or its normal end lacked an actual motion/readback condition",
                              preparation=preparation, path=path, stages=stages, approach_observation=approach_observation, real_arm_motion=right_motion,
                              left_carry_motion=left_motion, left_at_lift=left_at_lift, end=end)
        except Exception as exc:
            writes = sent()
            if writes:
                o.block_owner = "unknown"
                o.on_stop(type("D", (), dict(reasons=[f"{'RECEIVE_CONTACT' if is_contact else 'RECEIVE_APPROACH'}: {type(exc).__name__}: {exc}"]))(), None)
            record = getattr(o, "right_x_record", None)
            if isinstance(record, dict) and record.get("status") == "STARTED":
                record.update(status="STOPPED_BY_FAULT", why=f"{type(exc).__name__}: {exc}")
                o._safe(lambda: (o.out / "RIGHT_X_METROLOGY.json").write_text(json.dumps(_plain(record), indent=1, default=str)))
            return answer("FAILED", f"{'receive contact' if is_contact else 'receive approach'} stopped: {type(exc).__name__}: {exc}",
                          preparation=preparation, path=path, suffix=suffix, stages=stages, approach_observation=approach_observation, right_x_metrology=record,
                          end=dict(returned=False, right_released=False, left_at_lift=False,
                                   state="AS_LEFT_BY_THE_EXECUTOR_A_PERSON_DECIDES"))


class LoadTransfer:
    """The 'controlled load transfer' reference inside the implementation's contract (the left Jaw's hold command is kept until support is
    confirmed; arm postures may change): the right hand, closed on the block, is raised by the short lift named in the
    support reference, as one checked hop of the teacher's own straight-line planner, run once by the executor. Whether the
    load went over is then read from the left arm's Present_Load against its value at the start (Owner.support). Without the
    support reference this is NOT READY; no lift height or load drop is assumed here."""
    def __init__(self, owner):
        self.o, self.name, self.goal = owner, "right hand raised by the support reference's lift; left Jaw hold kept", None

    def begin(self, phase):
        self.goal = None; raw = getattr(self.o, "last_raw", None); self.o.load_baseline = None if raw is None else {n: float(v) for n, v in raw["load"]["left"].items()}; self.o.note("load_transfer_begin", phase=phase, left_load_baseline=self.o.load_baseline)

    def next_command(self, ctx):
        obs = ctx["observation"]; q = list(ctx["measured_q12"]); left, right = obs.get("left") or {}, obs.get("right") or {}; lift = self.o.lift_m()
        if lift is None:
            return q, dict(status="NOT_READY", missing=["the right-hand lift of the load transfer (approved for an observation session, or from the support reference in the full task)"])
        ref = dict(right_lift_m=lift)
        if left.get("command") is None or right.get("command") is None:
            return q, dict(status="NOT_READY", missing=["both gripper commands"])
        if self.goal is None:
            t = self.o.teacher; base = ctx["data"].qpos.copy(); t.cmd = self.o.gear(q); adr = t.plan.cm.block_qpos
            hops, why = t.cartesian_hops(base, t.cmd, "right", np.array([0.0, 0.0, 1.0]), float(ref["right_lift_m"]), allow=("left", "right"), block_qpos=base[adr:adr + 7].copy())
            if not hops:
                return q, dict(status="NOT_READY", missing=[f"a checked right-hand lift of {ref['right_lift_m']} m from this pose ({why})"])
            self.goal = t.to_canonical(hops[-1]).tolist()
        g = list(q); g[6:11] = self.goal[6:11]; g[5], g[11] = left["command"], right["command"]
        return g, dict(reference=self.name, right_lift_m=float(ref["right_lift_m"]))


class Release:
    """The release / retreat reference: left jaws to their open tick (the session's own open target), then the left hand away
    along the teacher's own retreat directions as short checked hops. Called only after the session has a support record."""
    def __init__(self, owner, distance_m=None):
        self.o, self.name, self.goals = owner, "left open, then teacher retreat hops", None

    def begin(self, phase):
        self.goals = None; self.o.note("release_begin", phase=phase)

    def next_command(self, ctx):
        obs = ctx["observation"]; q = list(ctx["measured_q12"]); right = obs.get("right") or {}; left = obs.get("left") or {}
        if right.get("command") is None or left.get("command") is None:
            return q, dict(status="NOT_READY", missing=["both gripper commands"])
        q[11] = right["command"]; open_q = self.o.left_open_q
        if (left.get("actual") or 0.0) < open_q - self.o.left_profile["floor"]:
            q[5] = open_q; return q, dict(reference=self.name, step="open")
        q[5] = open_q
        if self.goals is None:
            t = self.o.teacher; base = ctx["data"].qpos.copy(); t.cmd = self.o.gear(q); a_r = None
            for direction in t.retreat_directions(base, t.cmd, a_r):
                hops, why = t.cartesian_hops(base, t.cmd, "left", direction, t.cfg.retreat_m if hasattr(t.cfg, "retreat_m") else 0.06, allow=("left", "right"), block_qpos=base[t.plan.cm.block_qpos:t.plan.cm.block_qpos + 7].copy())
                if hops:
                    self.goals = [t.to_canonical(hops[-1]).tolist()]; self.o.note("release_retreat_planned", direction=[float(v) for v in direction], hops=len(hops)); break
            if self.goals is None:
                return q, dict(status="NOT_READY", missing=["a collision-free left retreat from this pose (teacher.cartesian_hops found none)"])
        g = list(self.goals[0]); g[5] = open_q; g[6:] = q[6:]
        return g, dict(reference=self.name, step="away")


class Owner:
    """The one local owner. Everything it sends goes through `run` / `grip` (sweepick_move_a) inside the recorder's lease that
    the hand-over already holds; everything it reports is read back from registers or cameras."""
    def __init__(self, out, episode, refs, *, p1, run=None, grip=None, run_kw=None, read=None, frames=None, look=None, check=None, clock=time.monotonic, sleep=time.sleep, state=None):
        import sweepick_move_a as mv
        self.out, self.ep, self.refs, self.p1 = Path(out), episode, refs, p1
        self.run, self.grip, self.run_kw, self.clock, self.sleep = run or mv.run, grip or mv.grip, dict(run_kw or {}), clock, sleep
        self._read, self._frames, self._look, self._check = read or self.read_registers, frames or self.recorder_frames, look or p1.look, check or self.segment_check
        self._read_other = (lambda side: read()["ticks"][side]) if read is not None else self.read_other_actual
        self.mp = self.run_kw.get("mapping") or p1.mapping(); self.msha = self.run_kw.get("mapping_sha") or hashlib.sha256(p1.CAND.read_bytes()).hexdigest()
        self.seq, self.k, self.pending, self.done, self.stopped, self.session, self.notes = 0, 0, {}, set(), None, None, []
        self.held, self.carry, self.block_owner, self.state_fn = None, None, "left", state
        self.held_half, self.scene_look = 0.02, None
        self.last_held_pose, self.near_handover = None, False
        self.evaluation_mode, self.physical_execution_allowed = "LIVE_REAL", True
        self.model_assisted_live_enabled = False
        self.right_wrist_optical = self.right_wrist_binding = None
        d = json.loads(mv.JAW_PROFILE.read_text()); lo, hi = float(self.mp["left"]["gripper"]["lo"]), float(self.mp["left"]["gripper"]["hi"]); R = hi - lo
        self.left_profile = dict(step=1.0 / R, floor=float(d["residual_floor_ticks"]["value"]) / R, source="sweepick_jaw_profile_left.json (one tick of the left servo; residual floor)")
        self.left_onset_command, self.left_open_q, self.home, self.home_command, self.home_command_basis, self.right_closed = None, None, {}, {}, {}, False
        self.last_frame_n = {}; self.last_images = {}; self.insert = None; self.images_gave_up = False; self.lift_seen_since = None; self.lift_result = None; self.observation_plan = None; self.scope, self.right_lift_m, self.scope_result, self.support_rows = "full", None, None, []
        self.teacher = None; self.approach_plan = None; self.approach_state = None; self.contact_plan = None; self.contact_baseline = None
        self.load_baseline = None; self.transfer_seen = False; self.cycle = None; self.notes_sent = []; self.idle_sig, self.idle_since = None, clock()
        self.idle_limit_s, self.idle_basis = 100 * float(d["confirm_s"]["value"]), "100 times the Jaw profile's confirmation time"
        self.observe_s = 10 * float(d["confirm_s"]["value"])                      # how long the state after the lift is recorded in the OBSERVE scope: ten of the Jaw profile's confirmation times

    # -- records ------------------------------------------------------------------------------------------------------
    def note(self, kind, **k):
        k = _plain(k)                                                          # images stay in the recorder's own frame files; the event keeps their frame number and stamp
        self.notes.append(dict(kind=kind, **k))
        try:
            if self.ep is not None:
                self.ep.event(kind, **k)
        except Exception as e:                                                 # the record's quality, not the motion's outcome
            self.notes.append(dict(kind="recording_error", what=f"{type(e).__name__}: {e}"))

    def gear(self, q12):
        from sweepick_261008_receive_plan import gear
        return gear(self.teacher, q12)

    # -- reading ------------------------------------------------------------------------------------------------------
    def read_registers(self, sides=("left", "right")):
        """Both arms, read-only: Present_Position, Goal_Position, Torque_Enable, Status, Present_Load. Measured and goal are separate registers and stay separate fields."""
        from sweepick_local_loop import open_readonly_bus
        raw = dict(ticks={}, goal={}, torque={}, status={}, load={}, moving={}, current={}, velocity={})
        for side in sides:
            b = open_readonly_bus(side)
            try:
                for key, reg in (("ticks", "Present_Position"), ("goal", "Goal_Position"), ("torque", "Torque_Enable"), ("status", "Status"), ("load", "Present_Load"), ("moving", "Moving"), ("current", "Present_Current"), ("velocity", "Present_Velocity")):
                    raw[key][side] = {n: float(v) for n, v in b.sync_read(reg, normalize=False).items()}
            finally:
                b.disconnect()
        return raw

    def read_other_actual(self, side):
        import sweepick_move_a as mv
        import sweepick_gates as G
        raw = self.read_registers(sides=(side,))
        if any(raw["status"][side].values()):
            raise mv.Stop(G.hard("servo_fault", f"other arm Status register non-zero: {raw['status'][side]}"))
        return raw["ticks"][side]

    def recorder_frames(self):
        """The newest frame of each of the session recorder's readers (no camera is opened here), each with the reader's own
        identity and validity, judged at the clock read right after it was fetched."""
        out = {}
        if self.ep is None:
            return out
        for name, r in getattr(self.ep, "wrists", {}).items():
            try:
                f, _ = r.get(); out[name] = None if f is None else reader_frame(r, f, self.clock(), frame=f["frame"])
            except Exception as exc:
                out[name] = dict(reader_valid=False, fresh=False, reader_reason=f"{name} camera error: {type(exc).__name__}: {exc}")
        try:
            t = getattr(self.ep, "top", None); f = t.get() if t is not None else None
            out["top"] = None if f is None else reader_frame(t, f, self.clock(), frame=f.get("bgr"), depth=f.get("zd"))
        except Exception as exc:
            out["top"] = dict(reader_valid=False, fresh=False, reader_reason=f"top camera error: {type(exc).__name__}: {exc}")
        return out

    def raw(self):
        t0 = self.clock(); r = self._read(); self.seq += 1
        r.update(t_bus_monotonic=self.clock(), bus_seq=self.seq, cycle_start=t0, usable=True, snapshot_only=False); return r

    def observation(self, stage, task, now):
        """One observation packet in the shape the implementation's session takes. Every record that claims something (motion result, insert,
        support, left away, acknowledgement) was made by a function of this owner from this read; it is stamped with this read."""
        tg = codex()["receive_session"].tg
        if self.cycle is None:                                                  # a second observation in the same step of the task manager: a new read (it will be stamped later than that step's 'now' and the session will wait for the next cycle)
            raw_ = self.raw(); self.cycle = (raw_, self._frames())
        (raw, fr), self.cycle = self.cycle, None; stamp, seq = raw["t_bus_monotonic"], raw["bus_seq"]
        tel = {s: dict(goal_ticks=raw["goal"][s], feedback_stamp=stamp, seq=seq, fresh=True, source="independent motor read: Present_Position and Goal_Position registers", provenance=tg.INDEPENDENT,
                       torque=bool(all(raw["torque"][s][n] for n in ALL)), velocity=None, load=raw["load"][s].get("gripper"), current=None, current_units=None, current_baseline=None) for s in ("left", "right")}
        def img(f, src, cam):
            """A camera packet is the reader's frame as it came: its own sequence, receipt time, SDK stamps and the reader's
            validity result. It does NOT get the bus read's stamp or sequence. (A frame handed in without a validity result,
            as an injected source may do, is judged here by the same rule.)"""
            if f is None:
                return None
            if "reader_valid" not in f:
                f = reader_frame(None, f, self.clock(), **{k: f[k] for k in ("frame", "depth") if k in f})
            same = self.last_frame_n.get(cam) == f.get("n"); self.last_frame_n[cam] = f.get("n")
            return dict({k: v for k, v in f.items() if k not in ("frame", "depth")}, rgb=f.get("frame"), source=src, same_frame_as_last_cycle=same, frame_seq=f.get("n"))
        ev = dict(wrist=dict(left=img(fr.get("left_wrist"), "left wrist reader of the session recorder", "left_wrist"), right=img(fr.get("right_wrist"), "right wrist reader of the session recorder", "right_wrist")), os30a=img(fr.get("top"), "SDK top reader of the session recorder", "top"),
                  servo_status={s: raw["status"][s] for s in ("left", "right")})
        if ev["wrist"]["left"] is None or ev["wrist"]["right"] is None:
            ev["wrist"] = None if ev["wrist"]["left"] is None and ev["wrist"]["right"] is None else ev["wrist"]
        obs = codex()["receive_entry"].owner_observation(dict(raw, ticks=raw["ticks"]), self.mp, executor_pid=os.getpid(), telemetry=tel, evidence=ev)
        self.last_raw, self.last_obs = raw, obs; self.last_images = dict(left_wrist=(ev.get("wrist") or {}).get("left") if ev.get("wrist") else None, right_wrist=(ev.get("wrist") or {}).get("right") if ev.get("wrist") else None, top=ev.get("os30a"))
        fr = {k: (dict(f, **{x: self.last_images[k][x] for x in ("reader_valid", "fresh", "reader_reason", "freshness_limit_s") if x in self.last_images[k]}) if camera_valid(self.last_images.get(k)) else None) for k, f in fr.items()}      # everything below (held pose, object at the tool, support, left away) uses a frame only when the reader's own result calls it valid
        recovery = stage in ("BEFORE_RIGHT_REOPEN", "BEFORE_NORMAL_RETURN")
        if not recovery:
            try:
                held_started = time.perf_counter()
                held = self.held_packet(obs, raw, fr, stamp, seq)
                self.held_production_s = time.perf_counter() - held_started
                if held is not None:
                    obs["held_object"] = held
                obs["object_evidence"] = self.object_evidence(obs, fr, stamp, seq)
            except Exception as exc:
                if self.scope != "contact":
                    raise
                obs["held_object"] = dict(object_pose_model=None, final_insertion_ready=False)
                obs["observation_missing"] = dict(visual_processing=f"{type(exc).__name__}: {exc}")
                obs.setdefault("producer_errors", {})["held_object"] = obs["observation_missing"]["visual_processing"]
                self.note("contact_visual_processing_unavailable", stage=stage, detail=obs["observation_missing"])
        if self.scope == "observe" and self.right_closed:
            lifted = "load_transfer" in self.done; self.observe_row(obs, raw, "after the right-hand lift" if lifted else "right closed, before the lift")
            if lifted:
                self.lift_seen_since = self.lift_seen_since or stamp; dwell = stamp - self.lift_seen_since; oe = obs["object_evidence"]
                if self.lift_result is not None and "motion_result" not in self.pending:
                    self.pending["motion_result"] = dict(self.lift_result, completed_before_this_cycle=True)      # the executor's result for the lift, still true in this cycle (stamped with this read below)
                base = self.load_baseline or {}; cur = raw["load"]["left"]
                obs["support_observation"] = dict(observation_valid=bool(dwell >= self.observe_s and oe.get("evidence_valid") is True), confirmed=False, release_allowed=False, label="UNKNOWN", source_kind=self.record_kind,
                                                  source="sweepick_full_chain.Owner.observation: registers and frames of this cycle; raw values, no support judgement", session_id=self.session.session_id, executor_pid=os.getpid(), feedback_stamp=stamp, seq=seq,
                                                  dwell_after_lift_s=dwell, dwell_required_s=self.observe_s, rows=len(self.support_rows), left_load_baseline_raw=base, left_load_now_raw=cur, left_load_drop_raw={n: abs(base[n]) - abs(cur[n]) for n in base} if base else None,
                                                  right_lift_m=self.lift_m(), vision_evidence=oe.get("vision_evidence"), object_at_right_tool=oe.get("object_at_tool"))
        for key, rec in list(self.pending.items()):                             # what the executor did since the last read, stamped with THIS read
            obs[key] = dict(rec, feedback_stamp=stamp, seq=seq); del self.pending[key]
        if self.session is not None and getattr(self.session, "phase", None) in ("LOAD_TRANSFER", "RELEASE"):
            sup = self.support(obs, raw, stamp, seq)
            if sup is not None:
                obs["support_confirmation"] = sup
            away = self.left_away(obs, raw, fr, stamp, seq)
            if away is not None:
                obs["left_open_and_away"] = away
        return obs

    def save_decision_observation(self, stage, observation, raw, frames):
        """Keep exactly the resident FrameSet consumed here, including depth skipped by periodic recording."""
        ep = getattr(self, "ep", None)
        if ep is None:
            return None
        folder = self.out / "decision_observations" / f"{raw['bus_seq']:06d}_{stage}"
        try:
            folder.mkdir(parents=True, exist_ok=False)
            files = {}
            for name, frame in frames.items():
                frame = frame or {}
                if frame.get("frame") is not None:
                    path = folder / f"{name}_rgb.png"
                    if not ep._save(path, frame["frame"]):
                        raise OSError(f"decision RGB not saved: {path}")
                    files[name] = dict(rgb=path.name)
                if name == "top" and frame.get("depth") is not None:
                    path = folder / "top_depth_mm.npy"
                    np.save(path, np.asarray(frame["depth"]), allow_pickle=False)
                    files.setdefault(name, {})["raw_depth_mm"] = path.name
            reader = getattr(ep, "top", None)
            identity = dict(feedback_stamp=raw["t_bus_monotonic"], seq=raw["bus_seq"])
            held_diagnostic = getattr(self, "held_diagnostic", {})
            if held_diagnostic.get("feedback_identity") != identity:
                held_diagnostic = dict(status="NOT_RUN", missing_reason="held face extraction is not run in this recovery observation", feedback_identity=identity)
            relative_diagnostic = getattr(self, "relative_diagnostic", {})
            if relative_diagnostic.get("feedback_identity") != identity:
                relative_diagnostic = dict(status="NOT_RUN", missing_reason="relative face extraction is not run in this recovery observation", feedback_identity=identity)
            save_vision_diagnostics(folder, (frames.get("top") or {}).get("frame"), held_diagnostic, relative_diagnostic)
            if observation.get("wrist_rgb_surfaces"):
                save_wrist_surface_diagnostics(folder, frames, self.wrist_surface_diagnostic)
            packet = dict(stage=stage, source_kind=self.record_kind, files=files,
                          camera_frames={name: camera_identity(frame) for name, frame in frames.items()},
                          top_intrinsics=getattr(reader, "K", None),
                          top_registration=dict(rectification_sha256=getattr(reader, "rect_sha", None),
                                                K_domain="original rectified RGB pixels; SDK NewCamMat1",
                                                depth_domain="same FrameSet registered rectified-left pixels",
                                                rgb_shape=None if (frames.get("top") or {}).get("frame") is None else list(frames["top"]["frame"].shape),
                                                depth_shape=None if (frames.get("top") or {}).get("depth") is None else list(frames["top"]["depth"].shape)),
                          T_world_camera=(getattr(self, "scene_look", None) or {}).get("T_world_camera"),
                          camera_chain_role="existing A candidate chain; not newly measured extrinsics",
                          q12=observation.get("q12"), actual_feedback=raw,
                          observation=observation)
            (folder / "OBSERVATION.json").write_text(json.dumps(_plain(packet), indent=2) + "\n")
            self.note("decision_observation_saved", stage=stage, folder=str(folder),
                      camera_frames=packet["camera_frames"], bus_seq=raw["bus_seq"])
            return str(folder)
        except Exception as exc:
            self.note("recording_error", stage=stage, what=f"decision observation: {type(exc).__name__}: {exc}")
            return None

    def contact_observation(self, stage, *, after, previous=None):
        """Consume new reader frames after a finite move, with their own clocks/sequences and current feedback."""
        started = self.clock(); measured_started = time.perf_counter(); previous = previous or {}
        self.held_production_s = None; self.relative_producer_timings = {}
        required = ("left_wrist",) if stage in ("BEFORE_RIGHT_REOPEN", "BEFORE_NORMAL_RETURN") else ("top", "left_wrist", "right_wrist")
        while True:
            if self.stopped is not None or Path(str(self.out) + ".STOP").exists():
                raise RuntimeError("the owner is stopped; no contact observation advances the session")
            raw = self.raw()
            try:
                frames = self._frames()
            except Exception as exc:
                frames = {name: dict(reader_valid=False, fresh=False, reader_reason=f"camera acquisition error: {type(exc).__name__}: {exc}") for name in ("top", "left_wrist", "right_wrist")}
            now = self.clock(); normalized, waiting = {}, {}
            if any(any(raw["status"][s].values()) for s in ("left", "right")):
                raise RuntimeError(f"{stage}: servo fault while waiting for a new image: {raw['status']}")
            for name in ("top", "left_wrist", "right_wrist"):
                f = frames.get(name)
                if f is not None and "reader_valid" not in f:
                    f = reader_frame(None, f, now, **{k: f[k] for k in ("frame", "depth") if k in f})
                normalized[name] = f
                old = previous.get(name) or {}; stamp = (f or {}).get("helper_receipt_monotonic_s", (f or {}).get("recv_monotonic_s"))
                if name in required and (not camera_valid(f) or stamp < after or (old.get("n") is not None and f["n"] <= old["n"])):
                    waiting[name] = (f or {}).get("reader_reason") or "no new frame after the finite move"
            unavailable = bool(waiting and (any("error" in reason or "ended" in reason or "malformed" in reason for reason in waiting.values()) or now - started > self.idle_limit_s))
            if not waiting or unavailable:
                for name, reason in waiting.items():
                    normalized[name] = dict(normalized.get(name) or {}, reader_valid=False, fresh=False, reader_reason=reason)
                timings = dict(frame_wait_s=time.perf_counter() - measured_started)
                self.last_contact_frames = normalized
                self.cycle = (raw, normalized); self._contact_visual_refresh = True
                try:
                    obs = self.observation(stage, None, now)
                finally:
                    self._contact_visual_refresh = False
                timings["held_production_s"] = getattr(self, "held_production_s", None)
                if stage not in ("BEFORE_RIGHT_REOPEN", "BEFORE_NORMAL_RETURN"):
                    obs["wrist_rgb_surfaces"] = self.observe_wrist_surfaces(obs, normalized)
                    if isinstance(obs.get("held_object"), dict):
                        obs["held_object"]["current_rgb_surfaces"] = obs["wrist_rgb_surfaces"]
                    try:
                        obs["model_assisted_relative_geometry"] = self.observe_model_assisted_geometry(obs, normalized)
                    except Exception as exc:
                        why = f"model-assisted producer: {type(exc).__name__}: {exc}"
                        obs["model_assisted_relative_geometry"] = dict(status="PRODUCER_ERROR", missing_reason=why,
                            model_pose_available=False, live_execution_permitted=False)
                        obs.setdefault("producer_errors", {})["model_assisted_relative_geometry"] = why
                right_frame = normalized["right_wrist"] or {}
                try:
                    right_view = self.p1.wrist_object(right_frame["frame"]) if stage not in ("BEFORE_RIGHT_REOPEN", "BEFORE_NORMAL_RETURN") and camera_valid(right_frame) and right_frame.get("frame") is not None else dict(visible=False)
                except Exception as exc:
                    right_view = dict(visible=False, missing_reason=f"right wrist image: {type(exc).__name__}: {exc}")
                    obs.setdefault("observation_missing", {})["right_wrist_processing"] = right_view["missing_reason"]
                    obs.setdefault("producer_errors", {})["right_wrist"] = right_view["missing_reason"]
                right_surface = (obs.get("wrist_rgb_surfaces") or {}).get("right_wrist")
                if right_surface is not None:
                    right_view["visible"] = right_surface.get("visible") is True
                    right_view["target_identity_status"] = right_surface["status"]
                obs["contact_right_wrist_view"] = dict(right_view, camera=camera_identity(right_frame),
                                                       role="current image observation; not a metric 3-D TF or support confirmation")
                obs.setdefault("observation_missing", {}).update(waiting)
                pad_started = time.perf_counter()
                try:
                    obs["visible_face_relation"] = None if stage in ("BEFORE_RIGHT_REOPEN", "BEFORE_NORMAL_RETURN") else self.partial_observer(stage, obs, normalized)
                except Exception as exc:
                    obs["visible_face_relation"] = dict(status="MISSING", missing_reason=f"current-frame producer: {type(exc).__name__}: {exc}")
                    obs.setdefault("producer_errors", {})["visible_face_relation"] = obs["visible_face_relation"]["missing_reason"]
                timings.update(pad_production_total_s=time.perf_counter() - pad_started,
                               **getattr(self, "relative_producer_timings", {}))
                obs["relative_alignment"] = self.relative_alignment(obs, stage=stage)
                obs["production_timings"] = timings
                record_started = time.perf_counter()
                obs["decision_record"] = self.save_decision_observation(stage, obs, raw, normalized)
                timings.update(record_s=time.perf_counter() - record_started, observation_total_s=time.perf_counter() - measured_started)
                if obs["decision_record"] is not None:
                    self._safe(lambda: (Path(obs["decision_record"]) / "PRODUCTION_TIMINGS.json").write_text(json.dumps(timings, indent=1)))
                self.note("observation_production_timings", stage=stage, **timings)
                self.note("contact_visual_observation", stage=stage, motion_completed_monotonic_s=after,
                          camera_frames={k: camera_identity(f) for k, f in normalized.items()},
                          previous_frames=previous, bus_seq=raw["bus_seq"], right_wrist_view=right_view,
                          held_pose_role=(obs.get("held_object") or {}).get("pose_role"), observation_missing=waiting)
                return obs, raw, normalized
            self.sleep(0.02)                                                   # existing reader wait cadence and idle timeout, no new session cap

    def observe_relative_faces(self, stage, observation, frames):
        """Produce current RGB-D face evidence; nominal projections are search regions, never observed marks."""
        from current_rgbd_faces import current_face_relation
        return current_face_relation(self, stage, observation, frames)

    def observe_wrist_surfaces(self, observation, frames):
        """Current RGB identity/edges; no wrist calibration or metric pose is inferred."""
        output = {}; self.wrist_surface_diagnostic = {}
        feedback = {k: observation.get(k) for k in ("feedback_stamp", "seq")}
        for name in ("left_wrist", "right_wrist"):
            frame = frames.get(name) or {}
            if not camera_valid(frame) or frame.get("frame") is None:
                feature = dict(status="MISSING", visible=False, missing_reason=frame.get("reader_reason") or "no current wrist RGB")
            else:
                try:
                    feature = self.p1.wrist_object_surfaces(frame["frame"])
                except Exception as exc:
                    feature = dict(status="PRODUCER_ERROR", visible=False, missing_reason=f"{type(exc).__name__}: {exc}")
                    observation.setdefault("producer_errors", {})[name + "_surfaces"] = feature["missing_reason"]
            self.wrist_surface_diagnostic[name] = feature
            output[name] = dict({k: v for k, v in feature.items() if not k.startswith("_")},
                                source_kind=self.record_kind, camera=camera_identity(frame), feedback_identity=feedback,
                                q12=list(observation["q12"]), classification="CURRENT_RGB_2D_MEASUREMENT",
                                metric_pose_available=False, inner_pad_identity_verified=False,
                                camera_registration="UNAVAILABLE_WRIST_K_AND_OPTICAL_TF")
        return output

    def relative_alignment(self, observation, *, stage=None):
        """Partial local alignment evidence can withhold a move, never certify insertion or support."""
        relation = observation.get("visible_face_relation")
        wrist = observation.get("contact_right_wrist_view") or {}
        result = dict(status="PARTIAL_UNOBSERVED", request="KEEP_EXISTING_GEOMETRY_DECISION", insertion_authorized=False,
                      right_wrist_2d=dict(visible=wrist.get("visible"), centre_px=wrist.get("centre_px"),
                                          far_edge_slope_deg=wrist.get("far_edge_slope_deg"), camera=wrist.get("camera")),
                      unknown=["opposite inner-pad gap", "hidden housing clearance", "complete object rotation/physical error"],
                      reference=None, ready=False)
        measured = {}
        for name, feature in (observation.get("wrist_rgb_surfaces") or {}).items():
            expected = camera_identity((getattr(self, "last_contact_frames", {}) or {}).get(name))
            bound = (feature.get("source_kind") == self.record_kind and expected.get("fresh") is True
                     and feature.get("camera") == expected
                     and feature.get("feedback_identity") == {k: observation.get(k) for k in ("feedback_stamp", "seq")}
                     and feature.get("q12") == observation.get("q12"))
            measured[name] = dict(status=feature.get("status") if bound else "UNPAIRED_RGB_OBSERVATION",
                                  visible=bool(bound and feature.get("visible")),
                                  exterior_edges=feature.get("visible_edges", []) if bound else [],
                                  interior_edge_candidates=feature.get("interior_edge_candidates", []) if bound else [],
                                  unidentified_rgb_candidates=feature.get("candidates", []) if bound else [],
                                  clipped_boundary=feature.get("clipped_boundary", []) if bound else [],
                                  camera=feature.get("camera"), metric_pose_available=False,
                                  inner_pad_identity_verified=False)
        result["wrist_rgb_evidence"] = measured
        result["block_rgb_identified"] = any(item["visible"] for item in measured.values())
        result["metric_association"] = "UNAVAILABLE_WRIST_K_AND_OPTICAL_TF" if measured else "NOT_RUN"
        candidate = observation.get("model_assisted_relative_geometry")
        if candidate is not None:
            result["model_assisted_candidate"] = candidate
            if candidate.get("model_pose_available") is True:
                result["metric_association"] = "MODEL_ASSISTED_RELATIVE_CANDIDATE; not measured readiness"
        if not isinstance(relation, dict) or relation.get("status") != "OK":
            return result
        expected = camera_identity((getattr(self, "last_images", {}) or {}).get("top"))
        identity = relation.get("camera") or {}
        keys = ("n", "frame_number", "sdk_color_stamp_s", "sdk_depth_stamp_s", "helper_receipt_monotonic_s", "recv_monotonic_s")
        if (relation.get("source_kind") != self.record_kind or not expected.get("fresh") or
                any(k not in expected or identity.get(k) != expected[k] for k in keys) or
                relation.get("feedback_identity") != {k: observation.get(k) for k in ("feedback_stamp", "seq")}):
            result.update(status="UNPAIRED_PARTIAL_OBSERVATION", request="WITHHOLD_INSERT")
            return result
        gap = relation.get("observed_object_face_signed_gap_m")
        overlap = relation.get("tangent_overlap")
        if (not isinstance(gap, (list, tuple)) or len(gap) != 2 or
                any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in gap) or gap[0] > gap[1] or
                not isinstance(overlap, list) or len(overlap) != 2 or
                any(not isinstance(x, dict) or isinstance(x.get("signed_overlap_m"), bool) or not isinstance(x.get("signed_overlap_m"), (int, float)) or not math.isfinite(x["signed_overlap_m"]) for x in overlap)):
            result.update(status="MALFORMED_PARTIAL_OBSERVATION", request="WITHHOLD_INSERT")
            return result
        stage = stage or relation.get("stage")
        identified = relation.get("inner_face_identity_verified") is True
        arrival = stage == "BEFORE_CLOSE"
        overlapping = all(x["signed_overlap_m"] > 0 for x in overlap)
        review = identified and ((overlapping and gap[0] <= 0) or (arrival and not overlapping))
        result.update(status="LOCAL_ALIGNMENT_REVIEW" if review else "OBSERVED_FACE_RELATION_ONLY",
                      request="WITHHOLD_INSERT_REALIGN_FROM_OBSERVED_POSE" if review else "KEEP_EXISTING_GEOMETRY_DECISION",
                      signed_gap_m=list(gap), tangent_overlap=overlap, camera=identity, stage=stage,
                      inner_face_identity_verified=identified, arrival_relation_reviewed=identified and arrival,
                      remaining_plan_input="current observed held pose and actual joints; these face values do not manufacture a 6-D pose")
        return result

    def observe_model_assisted_geometry(self, observation, frames):
        """Keep optical/plane fusion separate from the existing measured-geometry decision."""
        from right_wrist_relative_geometry import produce_relative_geometry, pad_projection, optical_inputs
        calibration = getattr(self, "right_wrist_optical", None)
        raw_frame = frames.get("right_wrist") or {}
        frame = dict(raw_frame, source_kind=self.record_kind, binding=self.current_right_wrist_binding())
        diagnostic = getattr(self, "held_diagnostic", {})
        feedback = {k: observation.get(k) for k in ("feedback_stamp", "seq")}
        feature = diagnostic.get("feature") or {}
        plane = None
        if (calibration is not None and diagnostic.get("camera") == camera_identity(frames.get("top")) and
                diagnostic.get("feedback_identity") == feedback and
                feature.get("plane_normal") is not None and feature.get("plane_centre") is not None):
            plane = dict(status="CURRENT_OBJECT_PLANE", source_kind=self.record_kind,
                         centre_world_m=feature["plane_centre"], normal_world=feature["plane_normal"],
                         observer_world_m=np.asarray(self.scene_look["T_world_camera"])[:3, 3].tolist(),
                         feedback_identity=feedback, camera=camera_identity(frames.get("top")),
                         competing_planes=False, association_role="current top held SEARCH_PRIOR-linked measurement")
        search = ((observation.get("held_object") or {}).get("search_prior") or {}).get("pose_model")
        # Missing optical inputs require neither another view motion nor another model load.
        prepared = getattr(self, "face_geometry_prepared", None)
        model = None if calibration is None else prepared["model"] if prepared else self.p1.model()
        tool, data = (None, None) if model is None else tool_pose(model, observation["q12"], site="right_cube_grasp")
        result = produce_relative_geometry(observation=dict(observation, source_kind=self.record_kind),
            frames=dict(frames, right_wrist=frame), calibration=calibration, plane=plane,
            search_pose=search, T_world_tool=tool, half=getattr(self, "held_half", 0.02))
        if result.get("model_pose_available") is True and prepared is not None:
            K, D, T_tool_camera = optical_inputs(calibration, frame, self.record_kind)
            result["right_pad_model_prior"] = pad_projection(model, data, prepared["contracts"], K, D, tool @ T_tool_camera)
        return result

    def plan_model_assisted_insertion(self, observation, *, stage="AFTER_PREGRASP"):
        from model_assisted_receive_plan import plan_model_assisted_receive
        packet = dict(observation)
        packet.setdefault("source_kind", self.record_kind)
        return plan_model_assisted_receive(self, packet,
                                           observation.get("model_assisted_relative_geometry"), stage=stage)

    def _model_assisted_live_candidate(self, observation, stage):
        """Bind an explicitly enabled REAL candidate to this contact cycle's actual hold."""
        if getattr(self, "model_assisted_live_enabled", False) is not True:
            return None
        from model_assisted_receive_plan import (
            _owner_context, _validate_geometry, apply_model_assisted_scene,
        )
        packet = dict(observation or {})
        packet.setdefault("source_kind", self.record_kind)
        geometry = packet.get("model_assisted_relative_geometry")
        q12, feedback = _validate_geometry(packet, geometry)
        model, data, teacher, hold, source = _owner_context(
            self, packet, geometry, q12, feedback)
        if source != "REAL":
            raise ValueError("LIVE model-assisted insertion requires REAL evidence")

        raw = self.last_raw
        try:
            if any(raw["status"][side][name] for side in ("left", "right") for name in ALL):
                raise ValueError("a current servo Status register is non-zero")
            if not all(raw["torque"]["left"][name] for name in ALL):
                raise ValueError("the current left hold is not powered on all six joints")
            expected = self.p1.rc.q12_to_ticks(self.mp, np.r_[hold, q12[6:]])["left"]["gripper"]
            actual_goal = raw["goal"]["left"]["gripper"]
            if round(actual_goal) != round(expected):
                raise ValueError("the current left Jaw goal differs from the applied hold")
            range_ticks = float(self.mp["left"]["gripper"]["hi"] - self.mp["left"]["gripper"]["lo"])
            residual = abs(raw["ticks"]["left"]["gripper"] - actual_goal)
            residual_floor = float(self.left_profile["floor"] * range_ticks)
            if not residual > residual_floor:
                raise ValueError("the current left Jaw has no loaded-hold evidence")
        except (KeyError, TypeError) as exc:
            raise ValueError(f"current left-hold registers are incomplete: {exc}") from exc
        if not isinstance(self.held, dict):
            raise ValueError("the same-session left held-object record is unavailable")

        prepared = getattr(self, "face_geometry_prepared", None)
        left_tool, _ = tool_pose(prepared["model"] if prepared is not None else self.p1.model(),
                                 q12, site="left_cube_grasp")
        object_pose = np.asarray(geometry["object_pose_model"], dtype=float)
        tool_from_object = np.linalg.inv(left_tool) @ object_pose
        state = dict(self.approach_state, _model=model, _data=data, _teacher=teacher)
        applied = apply_model_assisted_scene(state, q12, geometry, feedback["feedback_stamp"])
        if (applied.get("model_pose_applied") is not True or
                applied.get("planning_pose_role") != "MODEL_ASSISTED_RELATIVE" or
                applied.get("actual_held_pose_available") is not False or
                applied.get("live_pose_source_permitted") is not False):
            raise ValueError("the model-assisted scene did not preserve its estimated-pose role")
        identity = dict(source_kind=source, feedback_identity=dict(feedback),
                        camera=dict(geometry["camera"]), stage=stage)
        self.held["model_assisted_tool_from_object"] = tool_from_object.tolist()
        self.held["model_assisted_identity"] = identity
        self.held["active_model_assisted_identity"] = copy.deepcopy(identity)
        self.last_held_pose = object_pose.tolist()
        return dict(q12=q12, feedback=feedback, geometry=geometry, model=model,
                    data=data, teacher=teacher, left_hold_q=hold, source_kind=source,
                    scene_application=applied, left_support=dict(
                        status_registers_zero=True, all_left_torque_enabled=True,
                        left_jaw_goal_tick=float(actual_goal), left_jaw_hold_goal_tick=float(expected),
                        left_jaw_residual_ticks=float(residual),
                        left_jaw_residual_floor_ticks=residual_floor))

    def replan_model_assisted_after_hop(self, previous_plan, observation, completed_monotonic_s):
        from model_assisted_receive_plan import replan_after_hop
        packet = dict(observation)
        packet.setdefault("source_kind", self.record_kind)
        return replan_after_hop(self, previous_plan, packet,
                               observation.get("model_assisted_relative_geometry"), completed_monotonic_s)

    def capture_right_wrist_calibration(self, observation, raw, frames, out_dir, *, split, board):
        """Borrow this owner's current readers; no second camera/serial owner or actuator write."""
        from right_wrist_optical_calibration import capture_snapshot
        frame = frames.get("right_wrist") or {}
        if not camera_valid(frame):
            raise ValueError("calibration requires this owner's current valid right wrist frame")
        if any(any(raw["moving"][side].values()) for side in ("left", "right")):
            raise ValueError("calibration snapshot requires actual settled joints")
        if any(any(raw["status"][side].values()) for side in ("left", "right")):
            raise ValueError("calibration snapshot has servo fault/protection status")
        feedback = {k: observation.get(k) for k in ("feedback_stamp", "seq")}
        if feedback != dict(feedback_stamp=raw["t_bus_monotonic"], seq=raw["bus_seq"]):
            raise ValueError("calibration frame/actual feedback identity differs")
        mapped = self.p1.rc.ticks_to_q12(self.p1.mapping(), raw["ticks"])
        if not np.array_equal(np.asarray(mapped), np.asarray(observation["q12"])):
            raise ValueError("calibration q12 differs from current mapped actual ticks")
        binding = self.current_right_wrist_binding()
        if binding is None:
            raise ValueError("current physical mount binding unavailable; not copied from calibration")
        mode = binding["mode"]
        packet = dict(camera_key="right_wrist", source_kind=self.record_kind, rgb=frame["frame"],
                      encoding="BGR8", mode=mode, binding=binding,
                      frame_stamps=dict(camera_identity(frame), receipt_clock="host time.monotonic", exposure_clock="unknown"),
                      q=list(observation["q12"]),
                      q_stamps=dict(feedback, clock="host time.monotonic", cycle_start=raw.get("cycle_start")),
                      tool_frame="right_cube_grasp",
                      T_base_tool=tool_pose(self.p1.model(), observation["q12"], site="right_cube_grasp")[0],
                      base_frame="A_MODEL_WORLD", q_mapping=dict(source=str(self.p1.CAND), sha256=self.msha, verified=False),
                      geometry=dict(model_source=str(self.p1.SCENE), tool_frame="right_cube_grasp", verified=False))
        return capture_snapshot(lambda: packet, out_dir, split=split, board=board)

    def current_right_wrist_binding(self):
        binding = getattr(self, "right_wrist_binding", None)
        readers = getattr(getattr(self, "ep", None), "wrists", {})
        reader = readers.get("right") or readers.get("right_wrist")
        if binding is None or reader is None:
            return binding
        info = reader.info
        mode = dict(requested=info.get("requested"), negotiated=dict(fourcc=info.get("fourcc"),
                    width=info.get("width"), height=info.get("height"), fps=info.get("driver_fps")))
        return dict(binding, mode=mode)

    def configure_right_wrist_optical(self, path, current_mount_binding):
        """Pin a supplied optical artifact to this owner's independently confirmed current camera mount."""
        from right_wrist_relative_geometry import optical_inputs
        if getattr(self, "held", None) is not None or getattr(self, "done", set()):
            raise ValueError("optical inputs must be fixed before the physical task starts")
        document = json.loads(Path(path).read_text())
        previous_binding = self.right_wrist_binding
        self.right_wrist_binding = copy.deepcopy(current_mount_binding)
        frame = (getattr(self, "last_contact_frames", {}) or {}).get("right_wrist") or {}
        try:
            optical_inputs(document, dict(frame, source_kind=self.record_kind, binding=self.current_right_wrist_binding()), self.record_kind)
        except Exception:
            self.right_wrist_binding = previous_binding
            raise
        self.right_wrist_optical = document
        self.note("right_wrist_optical_configured", path=str(path), sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                  source_kind=document["source_kind"], optical_execution_permission=document.get("live_execution_permitted"),
                  current_mount_binding=self.current_right_wrist_binding(), geometry_path="MODEL_ASSISTED_RELATIVE preview; existing measured path preserved")

    def held_packet(self, obs, raw, fr, stamp, seq):
        """The held-object record of this cycle. Before the right insert the coarse estimate is not enough: the block has to be
        OBSERVED in the hand (top depth) or the record carries no pose, and the implementation's session then waits (NOT_READY)."""
        if self.held is None:
            return None
        need_seen = getattr(self, "_contact_visual_refresh", False) or ("RIGHT_APPROACH" in self.done and "RIGHT_INSERT" not in self.done)
        prepared = getattr(self, "face_geometry_prepared", None)
        face_model = prepared["model"] if prepared is not None else self.p1.model()
        T_tool, _ = tool_pose(face_model, obs["q12"], site=f"{self.block_owner}_cube_grasp" if self.block_owner in ("left", "right") else "left_cube_grasp")
        rel = self.held.get("seen_tool_from_object") if self.held.get("seen_tool_from_object") is not None else self.held["coarse_tool_from_object"]
        search_prior = dict(role="SEARCH_PRIOR", pose_model=(T_tool @ np.asarray(rel)).tolist(),
                            source="current actual FK + same-session held relationship; not an observed pose or readiness",
                            feedback_identity=dict(feedback_stamp=stamp, seq=seq), q12=list(obs["q12"]))
        if need_seen:
            top = fr.get("top")
            import inspect
            image_kw = dict(color=top.get("frame")) if top is not None and "color" in inspect.signature(self.held_seen_fn).parameters else {}
            self.held_diagnostic = dict(status="MISSING", missing_reason="current registered top RGB-D unavailable",
                                        camera=camera_identity(top), feedback_identity=dict(feedback_stamp=stamp, seq=seq))
            if "diagnostics" in inspect.signature(self.held_seen_fn).parameters:
                image_kw["diagnostics"] = self.held_diagnostic
            if "prepared_geometry" in inspect.signature(self.held_seen_fn).parameters:
                image_kw["prepared_geometry"] = prepared
            if "geometry_reference" in inspect.signature(self.held_seen_fn).parameters:
                image_kw["geometry_reference"] = getattr(self, "geometry_reference", None)
                reference = image_kw["geometry_reference"]
                if reference is not None and not np.array_equal(np.asarray(reference["binding"]["top_intrinsics"]), np.asarray(self.scene_look.get("K")).reshape(3, 3)):
                    self.note("held_geometry_reference_not_applicable", why="registered camera intrinsic revision differs")
                    self.geometry_reference_error = "supplied relative-error reference binding differs from the current registered camera intrinsics"
                    image_kw["geometry_reference"] = None
            seen = None if not camera_valid(top) or top.get("depth") is None or self.scene_look is None else self.held_seen_fn(top["depth"], self.scene_look.get("K"), self.scene_look.get("T_world_camera"), T_tool @ np.array(rel), model=face_model, q12=obs["q12"], **image_kw)
            if seen is None:
                self.note("held_pose_not_observed", why=self.held_diagnostic.get("missing_reason"),
                          diagnostic={k: v for k, v in self.held_diagnostic.items() if not k.startswith("_")})
                return dict(object_pose_model=None, pose_frame=self.scene["frame"], pose_source=None, object_pose_uncertainty=None, feedback_stamp=stamp, seq=seq, components=self.held["components"], missing=self.held["missing"],
                            search_prior=search_prior, observation_diagnostic={k: v for k, v in self.held_diagnostic.items() if not k.startswith("_")})
            seen = dict(seen, vision_evidence=camera_evidence(self.last_images.get("top"), True, "top depth: top face of the block in the hand")); unc = dict(seen.get("object_pose_uncertainty") or {})
            if "rotation_angle_bound_rad" not in unc:                              # retain the wrist-plane diagnostic; the missing full orientation bound stays missing
                unc.update(self.rotation_bound(fr))
            seen["object_pose_uncertainty"] = unc
            self.held["seen_tool_from_object"] = (np.linalg.inv(T_tool) @ np.array(seen["object_pose_model"])).tolist(); self.held["seen"] = seen; self.held["seen_seq"] = seq; rel = self.held["seen_tool_from_object"]; self.note("held_pose_observed", moved_from_estimate_m=seen["moved_from_estimate_m"], points=seen["points"])
        seen = self.held.get("seen"); self.last_held_pose = (T_tool @ np.array(rel)).tolist()      # where the block is NOW (with whoever carries it): the scene of the next segment's collision check
        uncertainty = (seen or {}).get("object_pose_uncertainty") or {}
        rotation_observation = (seen or {}).get("orientation_observation")
        full_rotation = bool(seen and (rotation_observation.get("full_rotation_observed") if rotation_observation is not None else uncertainty.get("rotation_bound_source")))
        centre_identified = bool(seen and seen.get("centre_identified") is True)
        measured_envelope = bool(uncertainty.get("rotation_bound_source")) and all(
            isinstance(uncertainty.get(k), (int, float)) and not isinstance(uncertainty.get(k), bool) and math.isfinite(uncertainty[k]) and uncertainty[k] >= 0
            for k in ("centre_m", "rotation_angle_bound_rad"))
        return dict(object_pose_model=(T_tool @ np.array(rel)).tolist(), pose_frame=self.scene["frame"], feedback_stamp=stamp, seq=seq, transforms=dict(tool_from_object=rel, tool=f"{self.block_owner}_cube_grasp"), search_prior=search_prior,
                    observation_diagnostic={k: v for k, v in getattr(self, "held_diagnostic", {}).items() if not k.startswith("_")},
                    pose_role="CURRENT_OBSERVATION" if seen and self.held.get("seen_seq") == seq else "CURRENT_TRACKED_ESTIMATE", source_kind=self.record_kind, final_insertion_ready=bool(full_rotation and centre_identified and self.held.get("seen_seq") == seq),
                    centre_identified=centre_identified, centre_observation=(seen or {}).get("centre_observation"),
                    object_geometry=dict(classification="MODEL_PRIOR", half_extents_m=[self.held_half] * 3, source="existing A block model"),
                    orientation_observation=rotation_observation, fit_diagnostics=(seen or {}).get("fit_diagnostics"),
                    observed_components=list((seen or {}).get("observed_components", ["block centre and height in the hand (top depth at the hand-over pose)"] if seen else [])) + ["measured joints now"], estimated_components=["centre from measured face and A MODEL_PRIOR half-size"] if rotation_observation is not None else [] if seen else ["block position in the hand from the desk observation before the close", "pad midpoint along the jaw axis"],
                    assumed_components=(["nearest face squared by the pads"] if not full_rotation else []) + ["rigid with the tool since " + ("it was observed" if seen else "the close")], unidentified_components=["slip", "pad compliance"] + list((seen or {}).get("missing_components", [] if seen else ["current transverse offset", "current orientation error"])) + ([] if measured_envelope else ["physical relative-error envelope; nominal candidate-model checks only"]),
                    pose_source=("top depth of the block in the hand, carried with the measured tool" if seen else "COARSE ESTIMATE: grasp geometry + measured joints (estimated and assumed parts named in components)"),
                    object_pose_uncertainty=(seen or {}).get("object_pose_uncertainty") or dict(measured=False, across_jaw_axis_m=0.014, note="coarse preview estimate"), components=(seen or self.held)["components"], grasp_evidence=self.held.get("grasp_evidence"),
                    vision_evidence=(seen or {}).get("vision_evidence"))

    def rotation_bound(self, fr):
        """How far the block's faces may be turned against the RIGHT jaws, from the one observation that exists for it: this
        cycle's valid right wrist frame, read with the detector of the pick (far-edge slope of the block) against the right
        wrist reference's slope for a block square to the right jaws and that reference's own slope resolution. It bounds the
        turn seen in the right wrist image plane; turns about the other two axes remain MISSING. This diagnostic is not
        the full orientation bound required for insertion. No default angle is put in its place."""
        ref = self.refs["right_wrist"]["doc"] or {}; f = fr.get("right_wrist")
        if not all(isinstance(ref.get(k), (int, float)) and not isinstance(ref.get(k), bool) and math.isfinite(ref[k]) for k in ("square_slope_deg", "slope_resolution_deg")) or ref["slope_resolution_deg"] < 0 or not camera_valid(f) or f.get("frame") is None:
            return {}
        o = self.p1.wrist_object(f["frame"]); slope = o.get("far_edge_slope_deg") if o.get("visible") else None
        if isinstance(slope, bool) or not isinstance(slope, (int, float)) or not math.isfinite(slope):
            return {}
        bound = abs(float(slope) - float(ref["square_slope_deg"])) + float(ref["slope_resolution_deg"])
        return dict(image_plane_rotation_bound_rad=float(np.radians(bound)), image_plane_rotation_bound_deg=bound, measured_slope_deg=float(slope), rotation_bound_frame=camera_identity(f),
                    rotation_bound_scope="RIGHT_WRIST_IMAGE_PLANE_ONLY", rotation_missing=["rotation about the other two axes"],
                    image_plane_rotation_bound_source="right wrist image of this cycle: far-edge slope against the right wrist reference's square slope plus its slope resolution; the other two rotation axes are MISSING")

    def object_evidence(self, obs, fr, stamp, seq):
        """Is the block at the RIGHT tool point? Judged on this cycle's valid right wrist frame against the right wrist reference
        (taken in the right contact observation that precedes any hand-over session). Without the reference, or without a
        valid frame: not evaluated (unknown), never 'yes'. The vision child carries the identity of the frame that was judged."""
        ref = self.refs["right_wrist"]["doc"]; f = fr.get("right_wrist"); at = None; valid = False; cam = self.last_images.get("right_wrist")
        if ref is not None and f is not None and f.get("frame") is not None:
            o = self.p1.wrist_object(f["frame"]); valid = True
            at = bool(o.get("visible") and abs(o["centre_px"] - float(ref["block_between_jaws_centre_px"])) <= float(ref["centre_tolerance_px"]))
        src = "right wrist image against the right wrist reference" if valid else ("no right wrist reference: not evaluated" if ref is None else "no valid right wrist frame in this cycle: not evaluated")
        return dict(object_at_tool=at, evidence_valid=valid, fresh=True, source=src, feedback_stamp=stamp, seq=seq, vision_evidence=camera_evidence(cam if valid else None, bool(at), src),
                    object_on_support=False, co_motion_observed=False, hand_raised_since_contact=False, hand_raised=True)

    def support(self, obs, raw, stamp, seq):
        """'The right hand carries the block', from three things read in this cycle: the right jaws are loaded (residual above the
        RIGHT profile's floor), the block is at the right tool in the right wrist image, and the left jaws no longer press
        (left residual at or below the LEFT profile's no-load floor after the preload was backed off). Without the right
        profile or the right wrist reference there is no record; a contact alone is never support."""
        tg = codex()["receive_session"].tg; rj = self.refs["right_jaw"]["doc"]; left, right, oe = obs["left"], obs["right"], obs["object_evidence"]
        if self.scope == "observe":                                             # no support judgement exists yet: this scope measures what one will be made from, and never produces a support record
            return None
        cam = self.last_images.get("right_wrist")
        if rj is None or not oe.get("evidence_valid") or left.get("command") is None or right.get("command") is None:
            return None
        Rr = float(self.mp["right"]["gripper"]["hi"] - self.mp["right"]["gripper"]["lo"]); r_res = abs(right["actual"] - right["command"]); ref = self.refs["support"]["doc"]
        motor = r_res > float(rj["residual_floor_ticks"]["value"]) / Rr; vision = bool(oe.get("object_at_tool")); drop = None
        if ref is not None and self.load_baseline is not None and "load_transfer" in self.done:
            drop = {n: abs(self.load_baseline[n]) - abs(raw["load"]["left"][n]) for n in ref["load_joints"]}
            if all(v >= float(ref["drop_raw"]) for v in drop.values()):
                self.transfer_seen = True                                         # seen once with the left hand still holding; the left opening that follows changes the left load for another reason
        transfer = bool(self.transfer_seen); l_res = drop
        e = lambda ok, src, **k: dict(observed=bool(ok), fresh=True, source=src, feedback_stamp=stamp, seq=seq, **k)
        rec = dict(label=tg.SUPPORT_CONFIRMED if motor and vision and transfer else tg.UNKNOWN, confirmed=bool(motor and vision and transfer), source_kind="REAL", source="sweepick_full_chain.Owner.support (registers and right wrist image of this cycle)", session_id=self.session.session_id,
                   executor_pid=os.getpid(), feedback_stamp=stamp, seq=seq, provenance=tg.INDEPENDENT, motor_evidence=e(motor, "right gripper residual above the right profile floor", residual=r_res),
                   vision_evidence=camera_evidence(cam, vision, "block at the right tool in the right wrist image"), load_transfer_evidence=e(transfer, "left arm Present_Load fell by the support reference's amount after the right-hand lift", left_load_drop=l_res))
        self.note("support_evaluated", confirmed=rec["confirmed"], motor=bool(motor), vision=vision, load_transfer=bool(transfer))
        return rec if rec["confirmed"] else None

    def left_away(self, obs, raw, fr, stamp, seq):
        """The left hand is open and away: its gripper stands at its open tick (register), its retreat ended HOLDING (executor), and the left wrist image no longer shows the block between the left jaws (the detector of the pick)."""
        tg = codex()["receive_session"].tg; left = obs["left"]
        if "left_retreat" not in self.done or left.get("actual") is None:
            return None
        opened = left["actual"] >= self.left_open_q - self.left_profile["floor"]; f = fr.get("left_wrist"); gone = None if f is None or f.get("frame") is None else not self.p1.wrist_object(f["frame"]).get("visible")
        e = lambda ok, src: dict(observed=bool(ok), fresh=True, source=src, feedback_stamp=stamp, seq=seq)
        return dict(open=bool(opened), away=bool(gone), source_kind="REAL", source="left gripper register, executor retreat result, left wrist image", session_id=self.session.session_id, executor_pid=os.getpid(), provenance=tg.INDEPENDENT,
                    feedback_stamp=stamp, seq=seq, motor_evidence=e(opened, "left gripper at its open tick; retreat ended HOLDING"), vision_evidence=camera_evidence(self.last_images.get("left_wrist") if gone is not None else None, bool(gone), "no block between the left jaws in the left wrist image"))

    # -- writing: the existing executor ------------------------------------------------------------------------------------
    def waypoint(self):
        s = self.session; c = getattr(s, "corrector", None)
        if s is None or c is None or getattr(c, "phase", None) not in (None, "COORDINATE"):
            return None
        return c.plan["waypoints"][c.index]

    def segment_check(self, doc, side, raw):
        """The commanded path with the block in the scene, by the teacher's own collision check (as the PICK02 stages do): the block
        carried by whoever holds it; allowed contacts are that hand's pads on the block (both hands' at the hand-over)."""
        import mujoco
        import sweepick_kin
        import sweepick_trajectory as tt
        from sim_data_factory.teacher import HandoverTeacher
        from sim_data_factory.task_success import separation_lower_bound
        p1 = self.p1; T = None if self.last_held_pose is None else np.array(self.last_held_pose); m = p1.model(None if T is None else (T[:3, 3], float(np.arctan2(T[1, 0], T[0, 0])))); t = HandoverTeacher(m); d = mujoco.MjData(m); traj = doc["trajectory"]
        adr = t.cm.block_qpos; block_q = m.qpos0[adr:adr + 7].copy(); other = "right" if side == "left" else "left"; allow = ("left", "right") if self.block_owner in ("both", "right") or self.near_handover or doc.get("purpose") == "RIGHT_CLOSE" else ("left",)
        if T is not None:                                                       # the block as it is held: its whole orientation, not only its yaw (the implementation's repair)
            quat = np.empty(4); mujoco.mju_mat2Quat(quat, np.ascontiguousarray(T[:3, :3]).ravel()); block_q = np.r_[T[:3, 3], quat]
        def ctrl(arm):
            q = p1.rc.ticks_to_q12(self.mp, {side: arm, other: raw["ticks"][other]}); sweepick_kin.set_q12(m, d, q); return d.qpos.copy(), d.qpos[t.plan.adr].copy()
        carrier = self.block_owner if self.block_owner in ("left", "right") else "left"; rel = None
        if T is not None and carrier == side:
            base, c = ctrl({n: raw["ticks"][side][n] for n in ALL}); t.plan.pose(base, c, block_q); ps, rot = t.plan.site_frame(side); rb = T[:3, :3]; rel = np.eye(4); rel[:3, :3] = rot.T @ rb; rel[:3, 3] = rot.T @ (T[:3, 3] - ps)
        first = None; steps = int(traj["steps"]); required = max(1, steps); checked = 0; clearance = None; scan = doc.get("purpose") == "RIGHT_INSERT_REPLAN" and T is not None; scanned = []
        entry = None
        if traj.get("command_start") is not None:
            entry = tt.make(traj["start"], {n: v for n, v in traj["command_start"].items() if v != traj["start"][n]}, doc.get("profile") or tt.load_profile())
            required += entry["steps"] + 1
        samples = ([dict(traj["start"])] + [{n: entry["goals"][n][k] for n in ALL} for k in range(entry["steps"])] if entry else []) + [{n: float(traj["goals"][n][k]) if steps and n in traj["goals"] else raw["ticks"][side][n] for n in ALL} for k in range(max(1, steps))]
        for k, arm in enumerate(samples):                                       # static pose and every actual-entry/command sample are checked.
            base, c = ctrl(arm)
            bq = t.plan.carried_block(base, c, (side, rel)) if rel is not None else block_q
            bad = t.plan.collisions(base, c, bq if T is not None else None, allow_pad_block=allow, clearance=0.0); checked += 1
            if scan:                                                            # how near the MOVING hand comes to the block on this path, by the existing whole-shape separation proof (pads of an allowed side excluded: their contact is intended)
                pd = t.plan.pose(base, c, bq)
                for g in t.cm.gripper_geoms[side]:
                    if g in t.cm.pad_index and t.cm.pad_index[g][0] in allow:
                        continue
                    gap = float(separation_lower_bound(m, pd, t.cm.block_geom, g)); clearance = gap if clearance is None else min(clearance, gap)
                    if k == 0:
                        scanned.append(m.geom(g).name)
            if bad:
                first = dict(step=k, pairs=[list(map(str, b)) for b in bad[:4]]); break
        return dict(schema="tjj.object-collision-result.v1", identity=tt.identity(doc["side"], traj, self.msha), nominal_verdict="CLEAR" if first is None else "CONTACT", first_nominal_contact=first, nominal_steps_checked=checked, samples_required=required, static_pose_checked=steps == 0,
                    range_entry_checked=entry is not None and first is None, range_entry_identity=tt.identity(doc["side"], traj, self.msha) if entry is not None else None, range_entry_trajectory=entry,
                    min_nominal_object_clearance_m=clearance, clearance_geoms=scanned, clearance_basis="sim_data_factory.task_success.separation_lower_bound between the block and the moving hand's housing and non-allowed pads, minimum over the samples; 0.0 means no proof of separation" if scan else None,
                    scene=f"assembled dual model, desk at the real height, block held by {self.block_owner}", allowed=[f"{a} pad - block" for a in allow], check="sim_data_factory.teacher.Planner.collisions (unchanged)", mapping_status="CANDIDATE (verified=false)")

    def segment(self, name, side, *, target=None, rest=None, power=None, purpose=None):
        """One checked executor run of one arm (or its gripper): plan -> collision check with the block -> run. Returns the executor's log. Raises on anything but HOLDING / RETURNED_RELEASED, so that the caller's state machine stops."""
        if (self.scope == "contact" and (self.contact_plan or {}).get("observation_only") is True
                and purpose in ("RIGHT_INSERT_REPLAN", "RIGHT_CLOSE")):
            raise RuntimeError(f"{purpose} is outside this observation-only RECEIVE_CONTACT permission")
        if (getattr(self, "evaluation_mode", "LIVE_REAL") != "LIVE_REAL"
                or getattr(self, "physical_execution_allowed", True) is not True):
            raise RuntimeError("physical execution is not permitted in this evaluation")
        if self.stopped is not None or Path(str(self.out) + ".STOP").exists():
            raise RuntimeError("the owner is stopped; no next segment or normal return is sent")
        raw = self._read(); other = "right" if side == "left" else "left"; d = self.out / "handoff" / f"{self.k:02d}_{name}_{side}"; self.k += 1
        on = [n for n in ALL if raw["torque"][side][n]]; want = target if target is not None else rest
        already = target is not None and all(round(raw["ticks"][side][n]) == round(v) for n, v in target.items()) and set(target) <= set(on)
        if already and purpose is None:
            self.note("segment", name=name, side=side, result="HOLDING", wrote=False, why="already at the target ticks"); return dict(result="HOLDING", end=dict(ticks=raw["ticks"][side]), wrote=False, folder=str(d))
        tj = power or [n for n in ALL if n in on or n in want or (n in ARM and any(j in ARM for j in want))]           # an arm move holds the whole arm; a gripper move adds the gripper to what is on
        mode = "return" if rest is not None else ("next" if on else "out")
        kw = dict(level="model", torque_joints=tj, other_arm_ticks=raw["ticks"][other], **self.run_kw); kw.update(dict(rest=rest) if rest is not None else dict(target=target))
        if purpose == "OBSERVATION_VIEW":
            kw.update(observation_view=True, settle_trim=())
        kw["other_arm_read"] = lambda: self._read_other(other)
        if purpose in ("RIGHT_INSERT_REPLAN", "OBSERVATION_VIEW"):
            self.track_held(raw)
        check_counts = {}

        def chk(doc, tag):
            if purpose is not None:
                doc = dict(doc, purpose=purpose)
            check_raw = raw
            if doc.get("current_feedback") is not None:
                check_raw = copy.deepcopy(raw)
                feedback = doc["current_feedback"]
                for key, field in (("ticks", "present"), ("goal", "goal"), ("torque", "torque"), ("status", "status")):
                    check_raw[key][side] = dict(feedback[field])
                check_raw["ticks"][other] = dict(doc["other_arm_ticks"])
                self.track_held(check_raw)
            checked = complete(self._check(doc, side, check_raw), doc)
            if purpose == "RIGHT_INSERT_REPLAN" and checked.get("nominal_verdict") == "CLEAR":
                clearance = checked.get("min_nominal_object_clearance_m")
                record = (self.insert or {}).get("record") or {}; bound = record.get("pose_bound_m")
                nominal = bound is None and record.get("geometry_mode") in (
                    "CURRENT_OBSERVATION_NOMINAL_MODEL", "MODEL_ASSISTED_REAL_NOMINAL_MODEL")
                bounded = isinstance(bound, (int, float)) and not isinstance(bound, bool) and math.isfinite(bound) and bound >= 0
                if (not isinstance(clearance, (int, float)) or isinstance(clearance, bool) or not math.isfinite(clearance) or clearance <= 0
                        or not (nominal or bounded) or (bounded and clearance <= bound)):
                    checked = dict(checked, nominal_verdict="UNKNOWN", incomplete="actual connection path does not establish the existing relative-pose clearance")
            if tag in ("trim", "start"):
                check_counts[tag] = check_counts.get(tag, 0) + 1
                tag = f"{tag}_{check_counts[tag]:03d}"
            return self._write_check(d, tag, checked)
        if already:
            import sweepick_trajectory as tt
            view_state = None
            if purpose == "OBSERVATION_VIEW":
                import sweepick_move_a as mv
                import sweepick_progress as PG
                pg = PG.load(); samples = [raw]; started = stable_since = self.clock(); best = dict(raw["ticks"][side])
                while True:
                    if self.stopped is not None or Path(str(self.out) + ".STOP").exists():
                        raise RuntimeError("owner STOP during zero-move observation settling")
                    sample = self._read(); samples.append(sample)
                    if any(any(sample["status"][s].values()) for s in ("left", "right")):
                        raise RuntimeError("servo fault during zero-move observation settling")
                    if any(abs(sample["ticks"][side][n] - best[n]) > pg["resolution"] for n in target):
                        stable_since = self.clock(); best = dict(sample["ticks"][side])
                    if len(samples) >= mv.POST_READS and self.clock() - stable_since > pg["start_delay"]:
                        break
                    if self.clock() - started > self.idle_limit_s:
                        raise RuntimeError("existing observation work budget exhausted while view is moving")
                    self.sleep(float((self.run_kw.get("profile") or tt.load_profile())["dt"]))
                raw = samples[-1]; self.track_held(raw)
                (d / "STATIC_VIEW_READS.json").parent.mkdir(parents=True, exist_ok=True)
                (d / "STATIC_VIEW_READS.json").write_text(json.dumps(_plain(samples), indent=1))
                stationary = samples[-mv.POST_READS:]
                settled = all(max(r["ticks"][side][n] for r in stationary) - min(r["ticks"][side][n] for r in stationary) <= pg["resolution"]
                              and not any(r["moving"][side][n] or r["velocity"][side][n] for r in stationary) for n in target)
                healthy = not any(any(r["status"][s].values()) for r in samples for s in ("left", "right"))
                resolved = all(r["goal"][side][n] == round(target[n]) and r["torque"][side][n] and abs(r["goal"][side][n] - r["ticks"][side][n]) <= pg["follow"] for r in samples for n in target)
                view_state = dict(nominal_target_reached=all(PG.reached(pg, raw["ticks"][side][n], target[n]) for n in target),
                    command_completed=True, actual_settled=settled, command_state_resolved=resolved and healthy,
                    current_support=None, fresh_observation_acquired=False, insertion_ready=False,
                    actual_ticks=dict(raw["ticks"][side]), nominal_target=target, settling_evidence="existing POST_READS stationary register window; no command",
                    endpoint_contract_unchanged=True, trim_policy="no_integral_trim")
                if not settled or not resolved or not healthy or Path(str(self.out) + ".STOP").exists():
                    raise RuntimeError(f"{name}: current zero-move view is not settled/resolved/healthy; no continuation")
            static = dict(side=side, trajectory=tt.make(raw["ticks"][side], target, self.run_kw.get("profile") or tt.load_profile()))
            c = chk(static, "static_insert_pose")
            if c.get("nominal_verdict") != "CLEAR":
                raise RuntimeError(f"{name}: the current insertion pose is not established CLEAR: {c.get('incomplete') or c.get('first_nominal_contact')}")
            self.note("segment", name=name, side=side, result="HOLDING", wrote=False, why="already at target; current insertion pose checked")
            result = dict(result="TARGET_NOT_REACHED_HOLDING" if view_state is not None and not view_state["nominal_target_reached"] else "HOLDING", end=dict(ticks=raw["ticks"][side]), wrote=False, folder=str(d), actual_start_ticks=dict(raw["ticks"][side]))
            if view_state is not None:
                result["observation_view"] = view_state
            return result
        code, plan = self.run("plan" if mode != "return" else "return", side, d / "plan", **kw)
        if plan["result"] != "PLANNED":
            raise RuntimeError(f"{name} ({side}): the executor did not plan it: {plan.get('refused') or plan['result']}")
        c = complete(chk(json.loads((d / "plan/TRAJECTORY.json").read_text()), "path"))
        if c.get("nominal_verdict") == "UNKNOWN":
            raise RuntimeError(f"{name} ({side}): the path check did not cover every sample of the commanded path ({c.get('nominal_steps_checked')} of {c.get('samples_required')}): not run")
        if c.get("nominal_verdict") != "CLEAR":
            raise RuntimeError(f"{name} ({side}): the commanded path touches something that is not an intended contact: {c.get('first_nominal_contact')}")
        if self.ep is not None:
            self._safe(lambda: self.ep.mark(f"handoff_{name}_{side}", owner=f"sweepick_move_a.run ({side} arm)"))
        self.append_motion_record(dict(kind="attempt", name=name, side=side, purpose=purpose, folder=str(d), planned=True,
                                       writes=None, actual_motion=None, terminal_result=None))
        try:
            code, log = self.run(mode, side, d / "run", execute=True, cert=c, trim_check=lambda doc: chk(doc, "trim"), start_recheck=lambda doc: chk(doc, "start"), stop_requested=Path(str(self.out) + ".STOP").exists, **kw)
        except BaseException as exc:
            log_path = d / "run/move.json"
            if hasattr(exc, "executor_log"):
                self.record_segment_motion(name, side, purpose, d, exc.executor_log)
            elif log_path.exists():
                self.record_segment_motion(name, side, purpose, d, json.loads(log_path.read_text()))
            else:
                self.append_motion_record(dict(kind="exception", name=name, side=side, purpose=purpose, folder=str(d),
                    writes="UNKNOWN", actual_motion="UNKNOWN", terminal_result=f"{type(exc).__name__}: {exc}"))
            raise
        log["folder"] = str(d)
        log["actual_start_ticks"] = dict(log.get("start", {}).get("ticks") or {})
        self.record_segment_motion(name, side, purpose, d, log)
        if self.ep is not None:
            self._safe(lambda: self.ep.stage(f"handoff_{name}", d, side, log["result"]))
        self.note("segment", name=name, side=side, result=log["result"], writes=log.get("writes"), folder=str(d)); self.notes_sent.append(name)
        view = log.get("observation_view") or {}
        missed_view = (purpose == "OBSERVATION_VIEW" and log["result"] == "TARGET_NOT_REACHED_HOLDING"
                       and view.get("command_completed") is True and view.get("actual_settled") is True
                       and view.get("command_state_resolved") is True and isinstance(log.get("end", {}).get("status"), dict)
                       and not any(log["end"]["status"].values()))
        if log["result"] not in ("HOLDING", "RETURNED_RELEASED") and not missed_view:
            raise RuntimeError(f"{name} ({side}): executor ended {log['result']}")
        if purpose == "OBSERVATION_VIEW":
            # Endpoint failure stays native; only a resolved settled view may acquire a diagnostic frame.
            if not (view.get("actual_settled") is True and view.get("command_state_resolved") is True):
                raise RuntimeError(f"{name}: observation drive is unresolved; no insertion or further view")
            actual = self._read(); self.track_held(actual)
            if any(any(actual["status"][s].values()) for s in ("left", "right")):
                raise RuntimeError(f"{name}: servo fault after observation drive; no continuation")
            if any(actual["moving"][side][n] or actual["velocity"][side][n] for n in want):
                raise RuntimeError(f"{name}: actual view is moving again; no continuation")
            if any(round(actual["goal"][side][n]) != round(want[n]) for n in want):
                raise RuntimeError(f"{name}: applied goal differs from the uncompensated view target; no continuation")
            import sweepick_progress as PG
            pg = PG.load()
            if any(abs(actual["goal"][side][n] - actual["ticks"][side][n]) > pg["follow"] for n in want):
                log["observation_view"].update(command_state_resolved=False, actual_ticks=dict(actual["ticks"][side]))
                self.note("observation_view_actual_unresolved", actual=actual, native_progress_profile=pg)
                raise RuntimeError(f"{name}: latest actual no longer satisfies the native resolved drive condition; no continuation")
            import sweepick_trajectory as tt
            static = dict(side=side, purpose=purpose, trajectory=tt.make(actual["ticks"][side], {}, self.run_kw.get("profile") or tt.load_profile()))
            checked = complete(self._check(static, side, actual), static)
            self._write_check(d, "settled_actual_pose", checked)
            if checked.get("nominal_verdict") != "CLEAR":
                raise RuntimeError(f"{name}: actual settled view is not model CLEAR; no observation continuation")
            log["observation_view"]["actual_ticks"] = dict(actual["ticks"][side])
            self.last_raw = actual
        return log

    def append_motion_record(self, row):
        row = dict(row, sequence=len(getattr(self, "motion_history", [])), recorded_monotonic_s=self.clock())
        if not hasattr(self, "motion_history"):
            self.motion_history = []
        self.motion_history.append(row)
        self.out.mkdir(parents=True, exist_ok=True)
        with (self.out / "MOTION_HISTORY.jsonl").open("a") as stream:
            stream.write(json.dumps(_plain(row), default=str) + "\n")
        return row

    def record_segment_motion(self, name, side, purpose, d, log):
        start = log.get("start", {}).get("ticks")
        end = log.get("end", {}).get("ticks")
        rows = log.get("rows") or []
        last = end or (rows[-1].get("present") if rows else None)
        last_goal = log.get("end", {}).get("goal_register") or (rows[-1].get("goal") if rows else None)
        actual_samples = [row.get("present") for row in rows if row.get("present")]
        if last:
            actual_samples.append(last)
        moved = None if not start or not actual_samples else any(any(sample[n] != start[n] for n in ARM) for sample in actual_samples)
        rec = self.append_motion_record(dict(kind="terminal", name=name, side=side, purpose=purpose, folder=str(d),
             actual_start_ticks=start, last_actual_ticks=last, last_applied_goal=last_goal,
             writes=log.get("writes"), actual_motion=moved, terminal_result=log.get("result"),
             outcome=log.get("outcome"), observation_view=log.get("observation_view"),
             executor_error=log.get("executor_error"), last_goal_write_attempt=log.get("last_goal_write_attempt"),
             last_acknowledged_goal=log.get("last_acknowledged_goal")))
        if purpose == "OBSERVATION_VIEW" and moved is True and any((log.get("writes") or {}).values()):
            history = getattr(self, "contact_observation_history", [])
            history.append(dict(side=side, start_ticks={n: float(start[n]) for n in ARM}, last_actual_ticks=last,
                last_applied_goal=last_goal, motion_sequence=rec["sequence"], terminal_result=log.get("result"),
                insert_count=len(getattr(self, "contact_insert_history", [])),
                after_updated_pregrasp=bool(getattr(self, "contact_pregrasp_history", None))))
            self.contact_observation_history = history
        return rec

    def _write_check(self, d, tag, c):
        d.mkdir(parents=True, exist_ok=True); (d / f"{tag}.CHECK.json").write_text(json.dumps(c, indent=1, default=str)); return c

    def _safe(self, fn):
        try:
            fn()
        except Exception as e:
            self.notes.append(dict(kind="recording_error", what=f"{type(e).__name__}: {e}"))

    def go(self, name, q12, *, arms=("left", "right"), grippers=()):
        """Bring the named arms (5 joints) and grippers to a canonical q12 through the executor, one arm at a time."""
        tk = self.p1.rc.q12_to_ticks(self.mp, np.array(q12, float)); logs = {}
        for side in arms:
            logs[side] = self.segment(name, side, target={n: float(round(tk[side][n])) for n in ARM})
        for side in grippers:
            logs[side + "_gripper"] = self.segment(name + "_gripper", side, target=dict(gripper=float(round(tk[side]["gripper"]))))
        return logs

    def track_held(self, raw):
        """Track the coarse held pose from fresh joint readback; this never upgrades it to an observed insertion pose."""
        if self.held is None:
            return None
        q12 = self.p1.rc.ticks_to_q12(self.mp, {side: raw["ticks"][side] for side in ("left", "right")})
        T, _ = tool_pose(self.p1.model(), q12, site="left_cube_grasp")
        model_identity = self.held.get("model_assisted_identity")
        model_active = (getattr(self, "model_assisted_live_enabled", False) is True and
                        model_identity is not None and
                        self.held.get("active_model_assisted_identity") == model_identity)
        rel = self.held.get("model_assisted_tool_from_object") if model_active else self.held.get("seen_tool_from_object")
        if rel is None:
            rel = self.held.get("model_assisted_tool_from_object")
        if rel is None:
            rel = self.held["coarse_tool_from_object"]
        rel = np.asarray(rel, float)
        self.last_held_pose = (T @ rel).tolist()
        return self.last_held_pose

    def plan_approach_prefix(self, plan, raw):
        """Check the complete ARM-only prefix and its normal reverse before the first prefix write."""
        import sweepick_trajectory as tt
        result = dict(scope="RECEIVE_APPROACH", ready=False, missing=[], why=None, forward=[], reverse=[],
                      right_gripper_commanded=False, right_insert_commanded=False, left_release_commanded=False,
                      profile_source="existing sweepick_motion_profile.json", collision_check="Owner.segment_check / configured Owner._check")
        if not isinstance(plan, dict) or plan.get("status") != "PLAN_PREVIEW":
            result["missing"] = ["existing plan_receive PLAN_PREVIEW"]
            result["why"] = "the existing RECEIVE plan is not available"
            return result
        try:
            if any(raw["status"][side][n] for side in ("left", "right") for n in ALL):
                raise ValueError("a servo Status register is non-zero")
            if not all(raw["torque"]["left"][n] for n in ALL):
                raise ValueError("the fresh left lift is not powered and holding on all six existing joints")
            if raw["torque"]["right"]["gripper"]:
                raise ValueError("the fresh right Jaw torque is on; this ARM5-only scope must not change or release ID6")
            hold = list(plan["left_hold_q"])
            if len(hold) != 6 or not np.isfinite(hold).all():
                raise ValueError("the existing plan has no finite canonical left hold")
            actual_q = self.p1.rc.ticks_to_q12(self.mp, {side: raw["ticks"][side] for side in ("left", "right")})
            hold_ticks = self.p1.rc.q12_to_ticks(self.mp, np.r_[hold, actual_q[6:]])["left"]
            if round(raw["goal"]["left"]["gripper"]) != round(hold_ticks["gripper"]):
                raise ValueError("the fresh left Goal_Position no longer matches the lift snapshot's applied Jaw hold")
            source = []
            for waypoint in plan.get("waypoints", []):
                label = waypoint.get("label")
                if not (isinstance(label, str) and (label.startswith("CARRY_") or label == "RIGHT_APPROACH")):
                    continue
                q12 = list(waypoint.get("q12") or [])
                if len(q12) != 12 or not np.isfinite(q12).all() or not np.isclose(q12[5], hold[5]):
                    raise ValueError(f"{label}: malformed canonical q12 or changed left Jaw hold")
                side = "right" if label == "RIGHT_APPROACH" else label.removeprefix("CARRY_").lower()
                if side not in ("left", "right"):
                    raise ValueError(f"{label}: unknown carry side")
                source.append(dict(label=label, side=side, source_q12=q12))
            labels = [step["label"] for step in source]
            if not source or labels[-1] != "RIGHT_APPROACH" or not any(s["side"] == "left" for s in source):
                raise ValueError("plan_receive did not produce left carry through RIGHT_APPROACH")
            right_carry = next((s for s in reversed(source[:-1]) if s["side"] == "right"), None)
            if right_carry is None:
                raise ValueError("plan_receive produced no checked right carry waypoint for the reverse")

            profile = self.run_kw.get("profile") or tt.load_profile()
            initial = {side: dict(raw["ticks"][side]) for side in ("left", "right")}
            simulated = copy.deepcopy(initial); saved_pose = self.last_held_pose

            def check(label, side, target, mode):
                self.track_held(dict(ticks=simulated))
                trajectory = tt.make(simulated[side], target, profile)
                other = "right" if side == "left" else "left"
                doc = dict(side=side, level="model", mapping_sha256=self.msha, trajectory=trajectory,
                           purpose="RECEIVE_APPROACH_PREFIX" if mode == "forward" else "RECEIVE_APPROACH_NORMAL_REVERSE")
                fake = dict(raw, ticks=copy.deepcopy(simulated))
                cert = complete(self._check(doc, side, fake), doc)
                validation = tt.validate(side, self.mp[side], trajectory, profile, mapping_sha=self.msha,
                                         level="model", cert=cert, other_arm_ticks=simulated[other])
                row = dict(label=label, side=side, mode=mode, target_ticks={n: float(target[n]) for n in ARM},
                           check=dict(verdict=cert.get("nominal_verdict"), samples=cert.get("nominal_steps_checked"),
                                      required=cert.get("samples_required"), first_contact=cert.get("first_nominal_contact"),
                                      incomplete=cert.get("incomplete")),
                           native_validation=dict(executable=validation["executable"], hard=validation["hard"],
                                                  provisional=validation["provisional"]))
                if cert.get("nominal_verdict") != "CLEAR":
                    raise ValueError(f"{label}: path check is {cert.get('nominal_verdict')} ({cert.get('incomplete') or cert.get('first_nominal_contact')})")
                refused = validation["hard"] + validation["provisional"]
                if refused:
                    raise ValueError(f"{label}: existing native trajectory validation refused the path: {refused[0].get('gate')}: {refused[0].get('detail')}")
                simulated[side].update(target)
                self.track_held(dict(ticks=simulated))
                row["target_q12"] = self.p1.rc.ticks_to_q12(self.mp, simulated).tolist()
                return row

            try:
                for step in source:
                    ticks = self.p1.rc.q12_to_ticks(self.mp, np.asarray(step["source_q12"], float))[step["side"]]
                    result["forward"].append(check(step["label"], step["side"], {n: float(round(ticks[n])) for n in ARM}, "forward"))
                ticks = self.p1.rc.q12_to_ticks(self.mp, np.asarray(right_carry["source_q12"], float))["right"]
                result["reverse"].append(check("RIGHT_BACK_TO_CARRY", "right", {n: float(round(ticks[n])) for n in ARM}, "reverse"))
                result["reverse"].append(check("RIGHT_RETURN_INITIAL", "right", {n: float(initial["right"][n]) for n in ARM}, "return"))
                result["reverse"].append(check("LEFT_BACK_TO_LIFT", "left", {n: float(initial["left"][n]) for n in ARM}, "reverse"))
            finally:
                self.last_held_pose = saved_pose
            result.update(ready=True, why="the whole forward prefix and normal reverse are CLEAR under the existing model check",
                          initial_ticks=initial, planned_labels=labels,
                          skipped_waypoints=[w.get("label") for w in plan.get("waypoints", []) if w.get("label") in ("LEFT_HOLD_SEED", "RIGHT_OPEN", "RIGHT_INSERT")],
                          note="RIGHT_OPEN and RIGHT_INSERT are not executed; every target is ARM5 only and both actual gripper ticks stay in each full canonical q12")
            return result
        except Exception as exc:
            self.last_held_pose = locals().get("saved_pose", self.last_held_pose)
            result["missing"] = [f"complete collision-checked RECEIVE_APPROACH prefix and normal reverse: {type(exc).__name__}: {exc}"]
            result["why"] = result["missing"][0]
            return result

    def plan_contact_suffix(self, raw, prefix, *, at_insert=False, normal_only=False, from_actual=False):
        """Precheck the observed-pose insert, full finite jaw range, and normal reverse before the first suffix write."""
        import sweepick_trajectory as tt
        result = dict(scope="RECEIVE_CONTACT", ready=False, missing=[], why=None, forward=[], reverse=[],
                      support_confirmed=False, grasp_confirmed=False, left_release_commanded=False,
                      right_jaw_expected_state=["OFF before the contact suffix", "ON only during close/observation/reopen/reverse", "OFF after the normal right return"])
        if not normal_only and (not (self.insert or {}).get("ready") or not isinstance(self.contact_baseline, dict)):
            result["missing"] = ["insertion_ready result and bound free-close baseline"]
            result["why"] = "there is no concrete checked insert/contact input"
            return result
        profile = self.run_kw.get("profile") or tt.load_profile(); simulated = copy.deepcopy(raw["ticks"]); saved_pose, saved_near = self.last_held_pose, self.near_handover

        def check(label, side, target, mode, *, allow_right_contact=False):
            self.track_held(dict(ticks=simulated)); traj = tt.make(simulated[side], target, profile); other = "right" if side == "left" else "left"
            doc = dict(side=side, level="model", mapping_sha256=self.msha, trajectory=traj,
                       purpose="RIGHT_CLOSE" if label in ("RIGHT_CONTACT_CLOSE", "RIGHT_CONTACT_OPEN") else
                               "RIGHT_INSERT_REPLAN" if label.startswith("RIGHT_INSERT_HOP") else "RECEIVE_CONTACT_SUFFIX")
            fake = dict(raw, ticks=copy.deepcopy(simulated)); self.near_handover = allow_right_contact
            cert = complete(self._check(doc, side, fake), doc)
            validation = (tt.validate(side, self.mp[side], traj, profile, mapping_sha=self.msha, level="model", cert=cert, other_arm_ticks=simulated[other])
                          if traj["steps"] else dict(executable=True, hard=[], provisional=[], no_write="already at target; static pose collision check only"))
            row = dict(label=label, side=side, mode=mode, target_ticks={n: float(v) for n, v in target.items()},
                       check=dict(verdict=cert.get("nominal_verdict"), samples=cert.get("nominal_steps_checked"), required=cert.get("samples_required"),
                                  first_contact=cert.get("first_nominal_contact"), incomplete=cert.get("incomplete")),
                       native_validation=dict(executable=validation["executable"], hard=validation["hard"], provisional=validation["provisional"]))
            row["planned_duration_s"] = traj["duration_s"]
            if cert.get("nominal_verdict") != "CLEAR":
                raise ValueError(f"{label}: path check is {cert.get('nominal_verdict')} ({cert.get('incomplete') or cert.get('first_nominal_contact')})")
            refused = validation["hard"] + validation["provisional"]
            if refused:
                raise ValueError(f"{label}: existing native trajectory validation refused the path: {refused[0].get('gate')}: {refused[0].get('detail')}")
            if label.startswith("RIGHT_INSERT_HOP"):
                clearance, bound = cert.get("min_nominal_object_clearance_m"), self.insert["record"]["pose_bound_m"]
                nominal = bound is None and self.insert["record"].get("geometry_mode") in (
                    "CURRENT_OBSERVATION_NOMINAL_MODEL", "MODEL_ASSISTED_REAL_NOMINAL_MODEL")
                bounded = isinstance(bound, (int, float)) and not isinstance(bound, bool) and math.isfinite(bound) and bound >= 0
                if (not isinstance(clearance, (int, float)) or isinstance(clearance, bool) or not math.isfinite(clearance) or clearance <= 0
                        or not (nominal or bounded) or (bounded and clearance <= bound)):
                    raise ValueError(f"{label}: the finite path does not exceed the existing observed relative-pose bound")
            simulated[side].update(target); self.track_held(dict(ticks=simulated)); row["certificate"] = cert
            row["target_q12"] = self.p1.rc.ticks_to_q12(self.mp, simulated).tolist()
            return row

        try:
            close_tick = float(self.contact_plan["close_target_tick"]); open_tick = float(prefix["initial_ticks"]["right"]["gripper"])
            back = [(row["label"], dict(row["start_ticks"])) for row in getattr(self, "contact_insert_history", [])]
            if not normal_only:
                pre = self.p1.rc.q12_to_ticks(self.mp, np.asarray(self.insert["pre_q12"], float))["right"]
                if not at_insert:
                    if not from_actual:
                        pregrasp_back = {n: float(simulated["right"][n]) for n in ARM}
                        result["forward"].append(check("RIGHT_PREGRASP_UPDATED", "right", {n: float(round(pre[n])) for n in ARM}, "forward"))
                    hops = self.insert.get("hops_q12")
                    if not isinstance(hops, list) or not hops:
                        raise ValueError("the current insertion revision has no complete Cartesian-hop list")
                    for k, q12 in enumerate(hops):
                        label = f"RIGHT_INSERT_HOP_{k + 1:03d}"
                        start = {n: float(simulated["right"][n]) for n in ARM}
                        target = self.p1.rc.q12_to_ticks(self.mp, np.asarray(q12, float))["right"]
                        row = check(label, "right", {n: float(round(target[n])) for n in ARM}, "forward")
                        row.update(target_revision=self.insert["target_revision"], hop_index=k, hop_count=len(hops))
                        result["forward"].append(row); back.append((label, start))
                result["forward"].append(check("RIGHT_CONTACT_CLOSE", "right", {"gripper": close_tick}, "contact collection", allow_right_contact=True))
            if not normal_only or self.contact_close_executed:
                result["reverse"].append(check("RIGHT_CONTACT_OPEN", "right", {"gripper": open_tick}, "normal reverse", allow_right_contact=True))
            def undo_views(insert_count, after_updated_pregrasp=None):
                for j, view in reversed(list(enumerate(getattr(self,'contact_observation_history',[])))):
                    if view['insert_count']==insert_count and (after_updated_pregrasp is None or view['after_updated_pregrasp']==after_updated_pregrasp):
                        result['reverse'].append(check(f'OBSERVATION_BACK_{j:03d}',view['side'],dict(view['start_ticks']),'normal reverse'))
            for k, (label, target) in reversed(list(enumerate(back))):
                undo_views(k+1)
                reverse_label = "RIGHT_BACK_TO_UPDATED_PREGRASP" if k == 0 else f"RIGHT_BACK_FROM_{label}_{k}"
                result["reverse"].append(check(reverse_label, "right", target, "normal reverse", allow_right_contact=not normal_only or self.contact_close_executed))
            pregrasp_back = (getattr(self, "contact_pregrasp_history", None) or {}).get("start_ticks", locals().get("pregrasp_back"))
            undo_views(0,True)
            if pregrasp_back is not None:
                result["reverse"].append(check("RIGHT_BACK_TO_ORIGINAL_PREGRASP", "right", pregrasp_back, "normal reverse", allow_right_contact=not normal_only or self.contact_close_executed))
            undo_views(0,False)
            for step in prefix["reverse"]:
                result["reverse"].append(check(step["label"], step["side"], dict(step["target_ticks"]), "normal reverse"))
            result.update(ready=True, why=("the current observed normal reverse is CLEAR under the existing collision and native trajectory checks" if normal_only else "the updated insert, finite right-jaw range and complete normal reverse are CLEAR under the existing collision and native trajectory checks"),
                          baseline=self.contact_baseline, close_target_tick=close_tick, open_target_tick=open_tick,
                          insertion_target_revision=(self.insert or {}).get("target_revision"),
                          insert_planned_trajectory_s=sum(row["planned_duration_s"] for row in result["forward"] if row["label"].startswith("RIGHT_INSERT_HOP")),
                          note="CONTACT_CANDIDATE is PROVISIONAL; support and grasp remain false; the left Jaw is never released")
        except Exception as exc:
            result["missing"] = [f"complete collision/native-validated RECEIVE_CONTACT suffix and normal reverse: {type(exc).__name__}: {exc}"]
            result["why"] = result["missing"][0]
        finally:
            self.last_held_pose, self.near_handover = saved_pose, saved_near
        return result

    def run_insert_hops(self, prefix, live, stages):
        """Same owner, checked finite runs; fresh images re-plan only the still-unexecuted suffix."""
        started = self.clock(); latest_suffix = None
        while True:
            if self.stopped is not None or Path(str(self.out) + ".STOP").exists():
                raise RuntimeError("the owner is stopped; no remaining insert hop or close is sent")
            actual = self._read(); live(actual, "before the next insert hop"); self.track_held(actual)
            latest_suffix = self.plan_contact_suffix(actual, prefix, from_actual=True)
            if not latest_suffix.get("ready"):
                return False, latest_suffix, dict(ready=False, why=latest_suffix.get("why"))
            hops = [step for step in latest_suffix["forward"] if step["label"].startswith("RIGHT_INSERT_HOP")]
            if not hops:
                raise RuntimeError("no insert hop was passed to the finite executor")
            step = hops[0]; revision = self.insert["target_revision"]
            self.note("insert_hops_planned", target_revision=revision, q12_hops=self.insert["hops_q12"],
                      actual_start_ticks=dict(actual["ticks"]["right"]), seen_seq=self.insert.get("for_seen_seq"),
                      planned_trajectory_s=latest_suffix["insert_planned_trajectory_s"])
            log = self.segment(f"contact_insert_r{revision}_{step['label'].lower()}", "right", target=step["target_ticks"], purpose="RIGHT_INSERT_REPLAN")
            completed = self.clock(); before = copy.deepcopy(self.last_images)
            final = len(hops) == 1
            native_end = log.get("end") or {}; native_ticks = native_end.get("ticks")
            end_available = isinstance(native_ticks, dict) and all(n in native_ticks for n in ARM)
            history = dict(label=step["label"], target_revision=revision, start_ticks={n: log["actual_start_ticks"][n] for n in ARM},
                           target_ticks=step["target_ticks"], executor_end_ticks={n: native_ticks[n] for n in ARM} if end_available else None,
                           executor_end_error=native_end.get("error"),
                           result=log["result"], completed_monotonic_s=completed, observation_status="PENDING",
                           planned_trajectory_s=step["planned_duration_s"])
            stage_row = dict(stage=step["label"], **{k: v for k, v in history.items() if k != "label"})
            self.contact_insert_history.append(history); stages.append(stage_row)
            self.note("insert_hop_motion_completed", **history)
            try:
                if not end_available:
                    raise RuntimeError(f"insert hop executor end feedback unavailable: {native_end.get('error') or 'missing ARM5 ticks'}")
                observed, actual, _ = self.contact_observation("BEFORE_CLOSE" if final else "BETWEEN_HOPS", after=completed, previous=before)
                live(actual, "after insert hop, before any next command"); self.track_held(actual)
            except Exception as exc:
                history.update(observation_status="UNAVAILABLE", observation_error=f"{type(exc).__name__}: {exc}")
                stage_row.update(history)
                self.note("insert_hop_observation_failed", **history)
                raise
            history.update(actual_ticks={n: actual["ticks"]["right"][n] for n in ARM}, observed_monotonic_s=self.clock(),
                           observation_status="OBSERVED", observation=_plain(observed))
            stage_row.update(history)
            self.note("insert_hop_reached", **history)
            if final:
                ready = self.insertion_ready(observed, stage="BEFORE_CLOSE")
                self.done.add("RIGHT_INSERT")
                self.note("insert_path_completed", reached_hops=len(self.contact_insert_history), actual_elapsed_s=self.clock() - started,
                          timing_basis="finite run/settle and intervening observations; not the saved preview's 5.94 seconds")
                record = dict(getattr(self, "close_geometry_check", {}) or {})
                if ready:
                    latest_suffix = self.plan_contact_suffix(actual, prefix, at_insert=True)
                    ready = latest_suffix.get("ready") is True
                return ready, latest_suffix, record
            if not self.insertion_ready(observed, stage="BETWEEN_HOPS"):
                observed, actual, ready = self.acquire_insertion_observation(
                    observed, prefix, live, stages, stage="BETWEEN_HOPS")
                if not ready:
                    return False, latest_suffix, dict((self.insert or {}).get("record") or {})

    def observation_deficits(self, observation):
        """Keep both missing face relation and held components, even when the first gate refuses."""
        held = (observation or {}).get("held_object") or {}
        rotation = held.get("orientation_observation") or {}
        return dict(pad_object_relation=(observation.get("visible_face_relation") or {}).get("status") != "OK",
                    held_centre=held.get("centre_identified") is not True,
                    held_orientation=rotation.get("full_rotation_observed") is not True,
                    unavailable=dict(observation.get("observation_missing") or {}))

    def plan_observation_candidate(self, candidate, raw, prefix):
        """A relative view proposal becomes A IK and checked finite paths, never imported joint commands."""
        result = dict(ready=False, phase="OBSERVATION_CANDIDATE", forward=[], normal_return=None,
                      physical_success=False, pose_role="SEARCH_PRIOR for observation planning only",
                      observation_policy="FINAL_TARGET_ONLY",
                      target_reference_semantics="sequential; CURRENT_TOOL is post-previous-target FK")
        saved_pose, saved_near = self.last_held_pose, self.near_handover
        saved_views=getattr(self,'contact_observation_history',None)
        planned_views=list(saved_views or [])
        try:
            if self.stopped is not None or Path(str(self.out) + ".STOP").exists():
                raise ValueError("operator STOP or owner stopped")
            errors = observation_candidate_errors(candidate)
            if errors:
                raise ValueError("; ".join(errors))
            targets = candidate.get("targets")
            if self.teacher is None or self.held is None:
                raise ValueError("current A teacher and same-session left held SEARCH_PRIOR are unavailable")
            import mujoco
            import sweepick_trajectory as tt
            from sim_data_factory.teacher import HandoverTeacher
            simulated = copy.deepcopy(raw["ticks"])
            q = self.p1.rc.ticks_to_q12(self.mp, simulated)
            prior = np.asarray(self.track_held(raw), float)
            model = self.p1.model((prior[:3, 3], float(np.arctan2(prior[1, 0], prior[0, 0]))))
            teacher = HandoverTeacher(model, self.teacher.cfg, planner_options=self.planner_options)
            teacher.set_planning_scope("RECEIVE_CONTACT")
            data = mujoco.MjData(model)
            self.p1.sweepick_kin.set_q12(model, data, q)
            quat = np.empty(4); mujoco.mju_mat2Quat(quat, np.ascontiguousarray(prior[:3, :3]).ravel())
            data.qpos[teacher.cm.block_qpos:teacher.cm.block_qpos + 7] = np.r_[prior[:3, 3], quat]
            mujoco.mj_forward(model, data)
            command_seed = q.copy()
            command_seed[[5, 11]] = np.clip(command_seed[[5, 11]], 0.0, 1.0)
            teacher.cmd = np.asarray(self.gear(command_seed))
            carried = ("left", teacher.site_block_rel(data, "left"))
            profile = self.run_kw.get("profile") or tt.load_profile()
            for target in targets:
                side, reference = target["side"], target["reference_frame"]
                delta = np.asarray(target["reference_from_target_tool"], float)
                teacher.plan.pose(data.qpos, teacher.cmd)
                position, rotation = teacher.plan.site_frame(side)
                reference_pose = np.eye(4); reference_pose[:3, :3], reference_pose[:3, 3] = rotation, position
                if reference == "SEARCH_PRIOR_OBJECT":
                    pd = teacher.plan.pose(data.qpos, teacher.cmd,
                            teacher.plan.carried_block(data.qpos, teacher.cmd, carried))
                    reference_pose[:3, :3] = pd.xmat[teacher.cm.block_body].reshape(3, 3)
                    reference_pose[:3, 3] = pd.xpos[teacher.cm.block_body]
                desired = reference_pose @ delta
                goals, info = teacher.plan_move(data, side, desired[:3, 3], desired[:3, 0], [desired[:3, 2]],
                                               carry=carried, allow=("left",), label="observation view")
                for goal in goals:
                    if not np.array_equal(goal[[5, 11]], teacher.cmd[[5, 11]]):
                        raise ValueError("the observation IK changed a jaw command")
                    q_goal = teacher.to_canonical(goal)
                    ticks = self.p1.rc.q12_to_ticks(self.mp, q_goal)[side]
                    command = {name: float(round(ticks[name])) for name in ARM}
                    fake = dict(raw, ticks=copy.deepcopy(simulated))
                    self.track_held(fake); self.near_handover = False
                    trajectory = tt.make(simulated[side], command, profile)
                    doc = dict(side=side, purpose="OBSERVATION_VIEW", trajectory=trajectory)
                    certificate = complete(self._check(doc, side, fake), doc)
                    other = "right" if side == "left" else "left"
                    validation = (tt.validate(side, self.mp[side], trajectory, profile, mapping_sha=self.msha,
                                               level="model", cert=certificate, other_arm_ticks=simulated[other])
                                  if trajectory["steps"] else dict(executable=True, hard=[], provisional=[],
                                      no_write="already at target; static pose collision check only"))
                    if certificate.get("nominal_verdict") != "CLEAR" or not validation["executable"]:
                        raise ValueError(f"view finite path refused: {certificate.get('first_nominal_contact') or validation['hard'] or validation['provisional']}")
                    start_ticks={n:float(simulated[side][n]) for n in ARM}
                    planned_views.append(dict(side=side,start_ticks=start_ticks,
                        insert_count=len(getattr(self,'contact_insert_history',[])),
                        after_updated_pregrasp=bool(getattr(self,'contact_pregrasp_history',None))))
                    result["forward"].append(dict(side=side, start_ticks=start_ticks, target_ticks=command, target_q12=q_goal.tolist(),
                            trajectory=trajectory, certificate=certificate, native_validation=validation, ik=info))
                    simulated[side].update(command)
                    teacher.cmd = goal.copy()
                    teacher.plan.pose(data.qpos, teacher.cmd,
                            teacher.plan.carried_block(data.qpos, teacher.cmd, carried))
                    data.qpos[:] = teacher.plan.pd.qpos
            self.track_held(dict(raw, ticks=simulated))
            self.contact_observation_history=planned_views
            result["normal_return"] = self.plan_contact_suffix(dict(raw, ticks=simulated), prefix, normal_only=True)
            if not result["normal_return"].get("ready"):
                raise ValueError(f"view normal reverse refused: {result['normal_return'].get('why')}")
            result.update(ready=True, why=None, candidate_id=candidate.get("id"),
                          expected_observed_components=candidate["expected_observed_components"],
                          actual_start_q12=q.tolist(), command_seed_q12=command_seed.tolist(), source_kind=self.record_kind,
                          measured_components=["actual joints", "applied left hold retained"],
                          estimated_components=["same-session held SEARCH_PRIOR and A model geometry"],
                          unobserved_components=["new view's object centre/axis and pad relation until acquired"])
        except Exception as exc:
            result.update(why=f"{type(exc).__name__}: {exc}", failure="OBSERVATION_PLAN_UNAVAILABLE")
        finally:
            self.last_held_pose, self.near_handover = saved_pose, saved_near
            if saved_views is None:self.__dict__.pop('contact_observation_history',None)
            else:self.contact_observation_history=saved_views
        return result

    def acquire_insertion_observation(self, observed, prefix, live, stages, *, stage):
        """Each supplied view is used once; new sensors, existing readiness, then insert or normal return."""
        if observed.get("producer_errors"):
            self.note("observation_producer_failed_no_view", errors=observed["producer_errors"])
            return observed, self.last_raw, False
        failed_readers = {name: reason for name, reason in (observed.get("observation_missing") or {}).items()
                          if "error" in reason or "ended" in reason or "malformed" in reason}
        if failed_readers:
            self.note("observation_reader_failed_no_view", errors=failed_readers)
            return observed, self.last_raw, False
        observation_only = self.scope == "contact" and (self.contact_plan or {}).get("observation_only") is True
        if observation_only:
            if self.stopped is not None or Path(str(self.out) + ".STOP").exists():
                raise RuntimeError("owner STOP; observation branch must not move or recover")
            metrology = (self.contact_plan or {}).get("right_x_metrology")
            if isinstance(metrology, dict) and metrology.get("t_pose") == "AFTER_RIGHT_PREGRASP":
                self.note("right_x_metrology_skips_adopted_view",
                          why="T is the measured state after RIGHT_APPROACH; the adopted clearance+roll view is not run")
                return observed, self.last_raw, False
            if getattr(self, "observation_only_candidate_consumed", False):
                self.note("observation_candidates_exhausted", missing_components=self.observation_deficits(observed),
                          why="the one observation-only receiver-inner-pad proposal was already consumed")
                return observed, self.last_raw, False
            self.observation_only_candidate_consumed = True
            observation_raw = self._read(); live(observation_raw, "before observation candidate")
            self.last_raw = observation_raw
            ok, why = self.recovery_ready("before observation candidate", observation=observation_raw)
            if not ok:
                raise RuntimeError(f"current left hold/bus is unconfirmed; no view or return: {why}")
            try:
                from current_view_candidate import propose_receiver_inner_pad_view
                candidates = [propose_receiver_inner_pad_view(self, observed, actual=observation_raw)]
            except Exception as exc:
                self.note("current_view_proposal_unavailable", reason=f"{type(exc).__name__}: {exc}",
                          missing_components=self.observation_deficits(observed))
                return observed, self.last_raw, False
        else:
            candidates = (self.contact_plan or {}).get("observation_candidates") or []
        proposals=getattr(self,'current_view_proposals_consumed',set())
        self.current_view_proposals_consumed=proposals
        deficits=self.observation_deficits(observed)
        component='held' if deficits['held_centre'] or deficits['held_orientation'] else 'pad'
        if (not observation_only and (self.contact_plan or {}).get("observation_candidate_policy") == "CURRENT_FRAME_PIXEL_SUPPORT"
                and component not in proposals):
            proposals.add(component)
            try:
                from current_view_candidate import propose_current_view, propose_held_view
                proposal=(propose_held_view(self,observed) if deficits['held_centre'] or deficits['held_orientation']
                          else propose_current_view(self, observed, self.last_contact_frames))
                candidates = list(candidates) + [proposal]
            except Exception as exc:
                self.note("current_view_proposal_unavailable", reason=f"{type(exc).__name__}: {exc}",
                          missing_components=self.observation_deficits(observed))
        if not isinstance(candidates, list):
            raise ValueError("observation_candidates must be a finite proposal list")
        consumed = getattr(self, "observation_candidates_consumed", set())
        self.observation_candidates_consumed = consumed
        for index, candidate in enumerate(candidates):
            candidate_key=candidate.get('id',index) if isinstance(candidate,dict) else index
            if candidate_key in consumed:
                continue
            if self.stopped is not None or Path(str(self.out) + ".STOP").exists():
                raise RuntimeError("owner STOP; observation branch must not move or recover")
            consumed.add(candidate_key)
            if observation_only:
                raw = observation_raw
            else:
                raw = self._read(); live(raw, "before observation candidate")
                self.last_raw = raw
                ok, why = self.recovery_ready("before observation candidate", observation=raw)
                if not ok:
                    raise RuntimeError(f"current left hold/bus is unconfirmed; no view or return: {why}")
            plan = self.plan_observation_candidate(candidate, raw, prefix)
            self.note("observation_candidate_planned", candidate_index=index,
                      observation_policy="FINAL_TARGET_ONLY",
                      target_reference_semantics="sequential; CURRENT_TOOL is post-previous-target FK",
                      missing_components=self.observation_deficits(observed), plan=plan)
            if not plan["ready"]:
                continue
            if observation_only:
                authorize = getattr(self, "authorize_observation_plan", None)
                try:
                    approved = callable(authorize) and authorize(plan, raw, candidate) is True
                except Exception as exc:
                    self.note("observation_plan_authorization_failed", reason=f"{type(exc).__name__}: {exc}")
                    raise
                self.note("observation_plan_authorization", approved=approved, candidate_index=index,
                          candidate_id=candidate.get("id") if isinstance(candidate, dict) else None)
                if not approved:
                    return observed, raw, False
            for k, step in enumerate(plan["forward"]):
                log = self.segment(f"observe_{index}_{k}", step["side"], target=step["target_ticks"], purpose="OBSERVATION_VIEW")
                raw = self._read(); live(raw, "after observation view move"); self.track_held(raw)
                self.last_raw = raw
                supported, support_why = self.recovery_ready("after settled observation view", observation=raw)
                log["observation_view"]["current_support"] = supported
                if not supported:
                    raise RuntimeError(f"left support unconfirmed after view; no further motion: {support_why}")
                self.last_observation_view = log["observation_view"]
                stages.append(dict(stage="OBSERVATION_VIEW", candidate_index=index, side=step["side"], result=log["result"]))
            completed, previous = self.clock(), copy.deepcopy(self.last_images)
            observed, raw, _ = self.contact_observation("OBSERVATION_CANDIDATE", after=completed, previous=previous)
            live(raw, "after new view observation"); self.track_held(raw)
            if observed.get("producer_errors"):
                self.note("observation_producer_failed_no_view", errors=observed["producer_errors"])
                return observed, raw, False
            ready = self.insertion_ready(observed, stage=stage)
            self.last_observation_view.update(fresh_observation_acquired=not bool(observed.get("observation_missing")), insertion_ready=ready,
                observed_actual=copy.deepcopy(raw["ticks"]), frame_identity={name: camera_identity(frame) for name, frame in self.last_images.items()})
            self.note("observation_view_actual_consumed", **self.last_observation_view)
            self.note("observation_candidate_consumed", candidate_index=index, ready=ready,
                      observation_policy="FINAL_TARGET_ONLY", observation_after="all forward targets",
                      missing_components=self.observation_deficits(observed), new_frame_ids={name: camera_identity(frame) for name, frame in self.last_images.items()})
            if ready:
                return observed, raw, True
            deficits=self.observation_deficits(observed)
            if (not observation_only and component=='held' and not deficits['held_centre'] and not deficits['held_orientation']
                    and deficits['pad_object_relation'] and 'pad' not in proposals
                    and (self.contact_plan or {}).get('observation_candidate_policy')=='CURRENT_FRAME_PIXEL_SUPPORT'):
                proposals.add('pad')
                try:
                    from current_view_candidate import propose_current_view
                    candidates.append(propose_current_view(self,observed,self.last_contact_frames))
                except Exception as exc:
                    self.note('current_pad_view_proposal_unavailable',reason=f'{type(exc).__name__}: {exc}')
        self.note("observation_candidates_exhausted", missing_components=self.observation_deficits(observed),
                  why="no unused checked view yielded insertion-ready evidence; existing normal-return conditions still apply")
        return observed, self.last_raw, False

    @staticmethod
    def x_split(T0, T1, basis):
        """T1 relative to T0, in T0's tool frame, split into the approved X basis, the off-axis remainder and rotation."""
        D = np.linalg.inv(np.asarray(T0, float)) @ np.asarray(T1, float); t = D[:3, 3]; b = np.asarray(basis, float)
        along = float(t @ b); off = t - along * b
        angle = float(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
        return dict(translation_tool_m=t.tolist(), along_x_m=along, off_axis_tool_m=off.tolist(), off_axis_norm_m=float(np.linalg.norm(off)),
                    rotation_deg=float(np.degrees(angle)), frame="previous endpoint's right_cube_grasp tool frame (candidate FK)")

    def right_x_endpoint(self, role, observed, raw, frames, cfg, reference, previous, move):
        """One endpoint: both arms' actual, openings, the three cameras' own identities/clocks, candidate FK of the actual."""
        model = self.p1.model()
        sides = ("left", "right")
        q12 = np.asarray(self.p1.rc.ticks_to_q12(self.mp, {s: raw["ticks"][s] for s in sides}), float)
        T, _ = tool_pose(model, q12, site="right_cube_grasp")
        row = dict(role=role, decision_record=observed.get("decision_record"),
                   actual=dict(ticks={s: dict(raw["ticks"][s]) for s in sides}, goal={s: dict(raw["goal"][s]) for s in sides},
                               torque={s: dict(raw["torque"][s]) for s in sides}, status={s: dict(raw["status"][s]) for s in sides},
                               bus_seq=raw.get("bus_seq"), t_bus_monotonic=raw.get("t_bus_monotonic"), cycle_start=raw.get("cycle_start"), q12=q12.tolist()),
                   opening=dict(right_present_tick=raw["ticks"]["right"]["gripper"], right_goal_tick=raw["goal"]["right"]["gripper"],
                                left_present_tick=raw["ticks"]["left"]["gripper"], left_goal_tick=raw["goal"]["left"]["gripper"],
                                basis="raw gripper Present_Position/Goal_Position ticks; jaw gap in mm UNKNOWN"),
                   cameras={name: camera_identity(f) for name, f in (frames or {}).items()},
                   observation_missing=dict(observed.get("observation_missing") or {}),
                   time_basis="each camera keeps its own receipt clock and frame number; the joint read keeps its bus clock; they are not equated",
                   right_tool_pose_fk=T.tolist(), fk_role="candidate mapping FK of the measured ticks (verified=false), not a physical measurement")
        if previous is not None:
            row["actual_fk_from_previous"] = self.x_split(previous["right_tool_pose_fk"], T, cfg["x_basis_tool"])
            row["actual_fk_from_T"] = self.x_split(reference["right_tool_pose_fk"], T, cfg["x_basis_tool"])
        if move is not None:
            start = move["planned_from"]; target = dict(start["ticks"]["right"]); target.update(move["forward"][0]["target_ticks"])
            q0 = self.p1.rc.ticks_to_q12(self.mp, start["ticks"]); q1 = self.p1.rc.ticks_to_q12(self.mp, dict(start["ticks"], right=target))
            row["command"] = dict(commanded_tool_displacement_m=move["candidate"]["commanded_tool_displacement_m"], target_ticks=move["forward"][0]["target_ticks"],
                                  command_fk_from_planning_actual=self.x_split(tool_pose(model, q0, site="right_cube_grasp")[0], tool_pose(model, q1, site="right_cube_grasp")[0], cfg["x_basis_tool"]),
                                  basis="commanded displacement and the FK of the rounded command ticks; kept apart from the actual FK above")
            view = move.get("observation_view") or {}
            row["executor"] = dict(result=move.get("result"), nominal_target_reached=view.get("nominal_target_reached"),
                                   actual_settled=view.get("actual_settled"), command_state_resolved=view.get("command_state_resolved"), writes=move.get("writes"))
        return row

    def run_right_x_metrology(self, prefix, live, stages):
        """RIGHT_X_FINITE_METROLOGY: observe T, then each X move (F, V) planned from the latest actual by the existing view path,
        explicitly approved, run by segment(purpose=OBSERVATION_VIEW) and observed once. A refused plan, decline or missing view
        ends the measurement without further motion; STOP, servo/bus fault or unconfirmed left support raise into the existing
        stop contract. The caller's existing normal reverse undoes every executed move. No correction is generated."""
        import right_x_metrology as rx
        cfg = (self.contact_plan or {}).get("right_x_metrology")
        record = dict(schema="sweepick.right-x-metrology.v1", scope=rx.SCOPE, config=copy.deepcopy(cfg), source_kind=self.record_kind,
                      endpoints=[], moves=[], status="STARTED",
                      jacobian=dict(delta_u_px=None, J_uX_px_per_mm=None, REAL_J_VALIDATED=False, validated=False, Correction=None,
                                    estimated_tool_correction=None, reason="FIELD_VERIFIED_TARGET_IMAGE and its u feature are not defined; endpoints are recorded only"),
                      correction_command_generated=False)
        self.right_x_record = record

        def save():
            (self.out / "RIGHT_X_METROLOGY.json").write_text(json.dumps(_plain(record), indent=1, default=str))

        def end(status, why):
            record.update(status=status, why=why); self.note("right_x_metrology_ended", status=status, why=why); save()
            return record
        errors = rx.config_errors(cfg)
        if errors:
            return end("REFUSED_CONFIG", "; ".join(errors))
        if cfg["t_pose"] == "AFTER_ADOPTED_VIEW" and (getattr(self, "last_observation_view", None) or {}).get("fresh_observation_acquired") is not True:
            return end("REFUSED_T_POSE", "t_pose=AFTER_ADOPTED_VIEW but no adopted view was executed and freshly observed in this session")
        reference = previous = None
        for role in rx.ROLES:
            if self.stopped is not None or Path(str(self.out) + ".STOP").exists():
                raise RuntimeError("owner STOP; right X metrology must not move or recover")
            raw = self._read(); live(raw, f"before right X {role}"); self.last_raw = raw
            ok, why = self.recovery_ready(f"before right X {role}", observation=raw)
            if not ok:
                raise RuntimeError(f"current left hold/bus is unconfirmed before right X {role}; no move: {why}")
            move = None
            if role != "T":
                candidate = rx.move_candidate(role, cfg)
                plan = self.plan_observation_candidate(candidate, raw, prefix)
                move = dict(role=role, candidate=candidate, plan_ready=plan.get("ready"), why=plan.get("why"), failure=plan.get("failure"),
                            planned_from=dict(bus_seq=raw.get("bus_seq"), t_bus_monotonic=raw.get("t_bus_monotonic"),
                                              ticks={s: dict(raw["ticks"][s]) for s in ("left", "right")}),
                            forward=[dict(start_ticks=s["start_ticks"], target_ticks=s["target_ticks"], ik=s.get("ik"),
                                          collision=s["certificate"].get("nominal_verdict"), native_executable=s["native_validation"].get("executable"))
                                     for s in plan.get("forward", [])],
                            normal_return_ready=(plan.get("normal_return") or {}).get("ready"))
                record["moves"].append(move)
                self.note("right_x_metrology_planned", role=role, plan=plan)
                if not plan.get("ready") or len(plan.get("forward", [])) != 1:
                    return end("REFUSED_PLAN", f"{role}: {plan.get('why') or 'not exactly one planned X target'}")
                authorize = getattr(self, "authorize_right_x_move", None)
                try:
                    approved = callable(authorize) and authorize(plan, raw, candidate) is True
                except Exception as exc:
                    self.note("right_x_metrology_authorization_failed", role=role, reason=f"{type(exc).__name__}: {exc}"); save()
                    raise
                move["approved"] = approved
                self.note("right_x_metrology_authorization", role=role, approved=approved)
                if not approved:
                    return end("DECLINED", f"{role}: no explicit operator approval of this actual-bound move")
                step = plan["forward"][0]
                log = self.segment(f"right_x_{role.lower()}", "right", target=step["target_ticks"], purpose="OBSERVATION_VIEW")
                move.update(result=log.get("result"), writes=log.get("writes"), folder=log.get("folder"), observation_view=log.get("observation_view"))
                raw = self._read(); live(raw, f"after right X {role}"); self.track_held(raw); self.last_raw = raw
                supported, why = self.recovery_ready(f"after settled right X {role}", observation=raw)
                if not supported:
                    save()
                    raise RuntimeError(f"left support unconfirmed after right X {role}; no further motion: {why}")
                stages.append(dict(stage=f"RIGHT_X_{role}", side="right", result=log.get("result")))
            completed, before = self.clock(), copy.deepcopy(getattr(self, "last_images", None) or {})
            observed, raw, frames = self.contact_observation(f"RIGHT_X_{role}", after=completed, previous=before)
            live(raw, f"after right X {role} observation"); self.track_held(raw); self.last_raw = raw
            endpoint = self.right_x_endpoint(role, observed, raw, frames, cfg, reference, previous, move)
            record["endpoints"].append(endpoint); save()
            missing = sorted(k for k in (observed.get("observation_missing") or {}) if k in ("top", "left_wrist", "right_wrist"))
            if missing:
                return end("OBSERVATION_MISSING", f"{role}: no new frame for {missing}; no further X move")
            reference = reference or endpoint; previous = endpoint
        return end("ENDPOINTS_RECORDED", "T, F and V endpoints recorded; the existing normal reverse follows; no correction is generated")

    def contact_close(self):
        """Run the finite right-ID6 close with the baseline-bound early observer; this method never opens or returns."""
        if self.scope == "contact" and (self.contact_plan or {}).get("observation_only") is True:
            raise RuntimeError("RIGHT_CLOSE is outside this observation-only RECEIVE_CONTACT permission")
        if (getattr(self, "evaluation_mode", "LIVE_REAL") != "LIVE_REAL"
                or getattr(self, "physical_execution_allowed", True) is not True):
            raise RuntimeError("physical execution is not permitted in this evaluation")
        if self.stopped is not None or Path(str(self.out) + ".STOP").exists():
            raise RuntimeError("the owner is stopped; no right contact close is sent")
        import sweepick_trajectory as tt
        raw = self._read(); d = self.out / "handoff" / f"{self.k:02d}_right_contact_close_right"; self.k += 1
        baseline, target = self.contact_baseline, float(self.contact_plan["close_target_tick"])

        def observer(sample):
            return sample["closing_lead_ticks"] > baseline["baseline_max_lead_ticks"]

        observer.scope = dict(side="right", joint="gripper", direction=-1, target_tick=target, observe_s=float(self.contact_plan["observe_s"]),
                              baseline_file=baseline["baseline_file"], baseline_sha256=baseline["baseline_sha256"],
                              baseline_max_lead_ticks=baseline["baseline_max_lead_ticks"], profile_sha256=baseline["profile_sha256"], mapping_sha256=baseline["mapping_sha256"])
        on = [n for n in ALL if raw["torque"]["right"][n]]; power = [n for n in ALL if n in on or n == "gripper"]
        kw = dict(level="model", torque_joints=power, other_arm_ticks=raw["ticks"]["left"], target=dict(gripper=target), **self.run_kw)
        code, planned = self.run("plan", "right", d / "plan", **kw)
        if planned["result"] != "PLANNED":
            raise RuntimeError(f"right contact close: the executor did not plan it: {planned.get('refused') or planned['result']}")
        chk = lambda doc, tag, at=raw: self._write_check(d, tag, self._check(doc, "right", at))
        cert = complete(chk(json.loads((d / "plan/TRAJECTORY.json").read_text()), "path"), json.loads((d / "plan/TRAJECTORY.json").read_text()))
        if cert.get("nominal_verdict") != "CLEAR":
            raise RuntimeError(f"right contact close: closing sweep check is {cert.get('nominal_verdict')}: {cert.get('incomplete') or cert.get('first_nominal_contact')}")
        if self.ep is not None:
            self._safe(lambda: self.ep.mark("handoff_right_contact_close_right", owner="sweepick_move_a.run (right gripper contact collection)"))
        code, log = self.run("next", "right", d / "run", execute=True, cert=cert, contact_observer=observer,
                             start_recheck=lambda doc: self._write_check(d, "start", self._check(doc, "right", self._read())),
                             stop_requested=Path(str(self.out) + ".STOP").exists, **kw)
        if self.ep is not None:
            self._safe(lambda: self.ep.stage("handoff_right_contact_close", d, "right", log["result"]))
        self.note("right_contact_close", result=log["result"], writes=log.get("writes"), contact_candidate=log.get("contact_candidate"), outcome=log.get("outcome"),
                  support_confirmed=False, grasp_confirmed=False, folder=str(d)); self.notes_sent.append("right_contact_close")
        if log["result"] not in ("GRIP_CONTACT_HOLDING", "HOLDING"):
            raise RuntimeError(f"right contact close: executor ended {log['result']}")
        log["folder"] = str(d); return log

    def result(self, label, q12, source):
        tg = codex()["receive_session"].tg
        return dict(label=label, result="HOLDING", q12=list(q12), independent=True, source_kind="REAL", provenance=tg.INDEPENDENT, source=source, session_id=self.session.session_id, executor_pid=os.getpid())

    def ack(self):
        """After a write to the right arm: its Goal_Position register read back, as the applied command."""
        tg = codex()["receive_session"].tg; raw = self._read(); q = self.p1.rc.ticks_to_q12(self.mp, dict(left=raw["goal"]["left"], right=raw["goal"]["right"]))
        self.pending["owner_ack"] = dict(accepted=True, independent=True, source="Goal_Position registers read back after the executor run", session_id=self.session.session_id, executor_pid=os.getpid(), q12=[float(v) for v in q])
        if getattr(self.session.applier, "relax", None) is not None:
            self._safe(lambda: self.session.applier.relax({s: raw["ticks"][s] for s in ("left", "right")}))      # the proposal generator starts its next proposal from where the executor left the arms

    def apply_dispatch(self, dispatch, observation):
        """The writer the implementation's ReceiveManipulation calls with each SEND proposal. The proposal is recorded by the caller; what is
        SENT is the executor's own checked run to the target the proposal is heading for:
          COORDINATE     the planned waypoint the corrector is on (once), then the right close by the Jaw (once)
          LOAD_TRANSFER  the left gripper command of the proposal
          RELEASE        the left gripper / left arm target of the proposal
        The receipt says what was run; it is not feedback (the next observation is)."""
        s = self.session; phase = getattr(s, "phase", None); prop = s.proposals[-1]["q12"] if s.proposals else None
        if self.stopped is not None:
            raise RuntimeError("the owner is stopped")
        if phase == "COORDINATE":
            wp = self.waypoint()
            if wp["label"] not in self.done:
                if wp["label"] == "RIGHT_INSERT":
                    if not (self.insert or {}).get("ready"):
                        return dict(status="NOTHING_SENT", why="the right insert is not run: no insert re-planned from the observed block has passed its check", not_independent_feedback=True)
                    self.go("right_pregrasp_replanned", self.insert["pre_q12"], arms=("right",))      # the re-planned pre-grasp first (the approach that was run belonged to the estimate)
                self.near_handover = wp["label"] == "RIGHT_INSERT"
                moved = [side for side, sl in (("left", slice(0, 5)), ("right", slice(6, 11))) if not np.allclose(np.array(wp["q12"])[sl], np.array(observation["q12"])[sl], atol=2e-3)]
                if wp["label"] == "RIGHT_INSERT" and "right" not in moved:
                    moved.append("right")                                       # the pre-grasp move changed the arm since this observation: the executor's own fresh read decides whether the insert is a move (the implementation's repair)
                grips = ["right"] if abs(wp["q12"][11] - observation["q12"][11]) > 2e-3 and wp["label"] == "RIGHT_OPEN" else []
                logs = self.go(wp["label"].lower(), wp["q12"], arms=moved, grippers=grips); self.done.add(wp["label"])
                rec = self.result(wp["label"], wp["q12"], "sweepick_move_a.run result HOLDING for this waypoint")
                self.pending["right_insert_confirmed" if wp["label"] == "RIGHT_INSERT" else "motion_result"] = rec
                if "right" in moved or grips:
                    self.ack()
                return dict(status="EXECUTOR_RAN", waypoint=wp["label"], runs={k: v["result"] for k, v in logs.items()}, not_independent_feedback=True)
            if getattr(s, "_right_close_started", False) and not self.right_closed:
                rj = self.refs["right_jaw"]
                if not rj["usable"]:
                    raise RuntimeError("the right gripper profile is not there")
                raw = self._read(); d = self.out / "handoff" / f"{self.k:02d}_right_close"; self.k += 1; cert = self._write_check(d, "sweep", self.close_sweep_check(raw))
                if cert.get("nominal_verdict") != "CLEAR":
                    raise RuntimeError(f"right close: the closing sweep touches something that is not a pad on the block: {cert.get('first_nominal_contact')}" if cert.get("nominal_verdict") == "CONTACT"
                                       else f"right close: the closing sweep check is {cert.get('nominal_verdict')} ({cert.get('incomplete') or cert.get('missing')}); the close is not run")
                code, log = self.grip("right", d, torque_joints=list(ALL), cert=cert, level="model", execute=True, jaw_profile=Path(rj["file"]), jaw_obj=s.right_jaw, stop_requested=Path(str(self.out) + ".STOP").exists, **self.run_kw)
                self.note("right_close", result=log["result"], grip=log.get("grip"))
                if log["result"] != "GRIP_CONTACT_HOLDING":
                    raise RuntimeError(f"right close: executor ended {log['result']}")
                self.right_closed = True; self.block_owner = "both"; self.ack()
                return dict(status="EXECUTOR_RAN", waypoint="RIGHT_CLOSE", result=log["result"], not_independent_feedback=True)
            return dict(status="NOTHING_SENT", why="the waypoint was already run; waiting for the session's next step", not_independent_feedback=True)
        tk = self.p1.rc.q12_to_ticks(self.mp, np.array(prop, float)); raw = self._read(); ran = {}
        if phase == "LOAD_TRANSFER":
            if "load_transfer" not in self.done:                               # always handed to the executor (its fresh read decides whether it is a move); its completion is a motion result of this owner
                self.pre_lift_right = {n: float(raw["ticks"]["right"][n]) for n in ARM}
                ran["right_lift"] = self.segment("load_transfer", "right", target={n: float(round(tk["right"][n])) for n in ARM})["result"]; self.done.add("load_transfer"); self.ack()
                goal = getattr(getattr(s.corrector, "load_transfer_controller", None), "goal", None)
                if goal is not None:
                    self.lift_result = self.result("LOAD_TRANSFER", goal, "sweepick_move_a.run result HOLDING for the load-transfer lift"); self.pending["motion_result"] = dict(self.lift_result)
            return dict(status="EXECUTOR_RAN" if ran else "NOTHING_SENT", phase=phase, runs=ran, not_independent_feedback=True)
        if self.scope == "observe":
            raise RuntimeError("OBSERVE scope: a command of the left release reached the writer; it is not sent")
        lg = float(round(tk["left"]["gripper"]))
        if round(raw["goal"]["left"]["gripper"]) != lg:
            ran["left_gripper"] = self.segment("left_gripper", "left", target=dict(gripper=lg))["result"]
        if phase == "RELEASE" and not np.allclose(np.array(prop)[:5], np.array(observation["q12"])[:5], atol=2e-3):
            ran["left_arm"] = self.segment("left_retreat", "left", target={n: float(round(tk["left"][n])) for n in ARM})["result"]; self.done.add("left_retreat"); self.block_owner = "right"
        return dict(status="EXECUTOR_RAN" if ran else "NOTHING_SENT", phase=phase, runs=ran, not_independent_feedback=True)

    def lift_m(self):
        """The planned right-hand lift of the load transfer. OBSERVE scope: the value approved for this session (it is a motion to be run, not a judgement). FULL scope: the value the support reference was measured with."""
        if self.scope == "observe":
            return self.right_lift_m
        ref = self.refs["support"]["doc"]; return None if ref is None else float(ref["right_lift_m"])

    def observe_row(self, obs, raw, when):
        """OBSERVE scope: what the support reference will be made from. Raw registers of this cycle; no judgement."""
        Rl = float(self.mp["left"]["gripper"]["hi"] - self.mp["left"]["gripper"]["lo"]); Rr = float(self.mp["right"]["gripper"]["hi"] - self.mp["right"]["gripper"]["lo"]); l, r = obs["left"], obs["right"]
        self.support_rows.append(dict(when=when, seq=obs["seq"], t_bus_monotonic=obs["feedback_stamp"], left_load_raw=raw["load"]["left"], right_load_raw=raw["load"]["right"], left_gripper_residual_ticks=None if l.get("command") is None else abs(l["actual"] - l["command"]) * Rl,
                                      right_gripper_residual_ticks=None if r.get("command") is None else abs(r["actual"] - r["command"]) * Rr, left_ticks=raw["ticks"]["left"], right_ticks=raw["ticks"]["right"],
                                      frames={k: None if v is None else dict(n=v.get("n"), fresh=v.get("fresh"), recv_monotonic_s=v.get("recv_monotonic_s")) for k, v in self.last_images.items()}, object_at_right_tool=(obs.get("object_evidence") or {}).get("object_at_tool")))

    def write_observation(self, end, state):
        doc = dict(schema="sweepick.support-observation.v1", session_id=getattr(self.ep, "id", None), scope="SUPPORT_OBSERVATION: right contact and load-transfer observation; the left hand is not released; not a hand-over and not a full task", state=state,
                   plan=self.observation_plan, right_lift_m=self.lift_m(), rows=self.support_rows, end=end,
                   note="raw observations for a person to read. A support reference (which left-arm joints' load fell, by how much) is written from this by a person as sweepick_support_reference.json with source_kind REAL and measurement_evidence naming this file; nothing here is a judgement and nothing is activated by itself")
        (self.out / "SUPPORT_OBSERVATION.json").write_text(json.dumps(_plain(doc), indent=1))

    def recovery_ready(self, step, observation=None, frames=None):
        """May a return or contact reopen continue while the left hand still demonstrably owns the block?"""
        why, raw, f, res, expected_goal = [], observation, None, None, None
        checked_at, register_checks_ok, detection = None, False, None

        def record():
            packet = dict(step=step, ok=not why, why=why, scope=self.scope,
                          checked_monotonic_s=checked_at,
                          register_identity={k: raw[k] for k in ("bus_seq", "t_bus_monotonic", "cycle_start") if isinstance(raw, dict) and k in raw},
                          left_jaw_residual_ticks=None if res is None else float(res),
                          left_jaw_residual_floor_ticks=float(self.left_profile["floor"] * (self.mp["left"]["gripper"]["hi"] - self.mp["left"]["gripper"]["lo"])),
                          register_checks_ok=register_checks_ok,
                          left_jaw_actual_tick=None if not isinstance(raw, dict) else (raw.get("ticks", {}).get("left", {}) or {}).get("gripper"),
                          left_jaw_goal_register_tick=None if not isinstance(raw, dict) else (raw.get("goal", {}).get("left", {}) or {}).get("gripper"),
                          left_jaw_hold_goal_tick=expected_goal,
                          left_wrist_frame=camera_identity(f) if f else None,
                          left_wrist_detection=detection)
            self.last_recovery_check = packet
            self.note("observation_return_check", **packet)
            if self.scope == "contact":
                self.note("left_hold_recovery_ready", **packet)

        if (self.stopped is not None or getattr(self.session, "stopped_dispatch", None) is not None
                or Path(str(self.out) + ".STOP").exists()):
            why.append("operator STOP, the owner, or the state machine is stopped")
        if raw is None:
            try:
                raw = self._read()
            except Exception as e:
                checked_at = self.clock(); why.append(f"registers could not be read: {type(e).__name__}: {e}"); record(); return False, why
        try:
            for side in ("left", "right"):
                if any(raw["status"][side].values()):
                    why.append(f"{side} arm Status register non-zero: {raw['status'][side]}")
            if not all(raw["torque"]["left"][n] for n in ALL):
                why.append(f"left arm torque is not on at every joint: {raw['torque']['left']}")
            right_id6_off = self.scope == "approach" or (self.scope == "contact" and getattr(self, "contact_close_executed", True) is False)
            right_joints = ARM if right_id6_off else ALL
            if not all(raw["torque"]["right"][n] for n in right_joints) or (right_id6_off and raw["torque"]["right"]["gripper"]):
                expected = "ARM5 on and ID6 off" if right_id6_off else "all six joints on"
                why.append(f"right arm torque is not {expected}: {raw['torque']['right']}")
            Rl = float(self.mp["left"]["gripper"]["hi"] - self.mp["left"]["gripper"]["lo"])
            res = abs(raw["ticks"]["left"]["gripper"] - raw["goal"]["left"]["gripper"])
            if not res > self.left_profile["floor"] * Rl:
                why.append(f"the left jaws are not loaded any more (residual {res:.0f} ticks, no-load floor {self.left_profile['floor'] * Rl:.0f})")
            if self.scope in ("approach", "contact"):
                hold = (self.approach_state or {}).get("left_hold_q")
                if not isinstance(hold, (list, tuple)) or len(hold) != 6 or not np.isfinite(hold).all():
                    why.append("the lift snapshot's applied left Jaw hold is unavailable")
                else:
                    actual_q = self.p1.rc.ticks_to_q12(self.mp, raw["ticks"])
                    expected_goal = float(self.p1.rc.q12_to_ticks(self.mp, np.r_[hold, actual_q[6:]])["left"]["gripper"])
                    if round(raw["goal"]["left"]["gripper"]) != round(expected_goal):
                        why.append(f"the left Jaw Goal_Position changed from the lift snapshot's applied hold ({raw['goal']['left']['gripper']:.0f} != {expected_goal:.0f} ticks)")
        except Exception as e:
            why.append(f"register packet or applied left hold is invalid: {type(e).__name__}: {e}")
        register_checks_ok = not why
        try:
            fr = self._frames() if frames is None else frames
            checked_at = self.clock()
            f = fr.get("left_wrist")
            f = f if f is None or "reader_valid" in f else reader_frame(None, f, checked_at, frame=f.get("frame"))
            if camera_valid(f) and f.get("frame") is not None:
                detection = self.p1.wrist_object(f["frame"])
            seen = detection is not None and bool(detection.get("visible"))
        except Exception as e:
            checked_at = self.clock() if checked_at is None else checked_at
            seen = False; why.append(f"left wrist frame could not be consumed: {type(e).__name__}: {e}")
        if not seen:
            why.append("no valid fresh left wrist frame showing the block between the left jaws")
        record(); return not why, why

    def finish_observation(self):
        """SUPPORT_OBSERVATION, after the observation point: the planned return. The right hand goes back down the lift, opens to
        the opening it was given at the start of this hand-over, goes back to its pre-grasp and home (released); the left hand,
        which never let go, carries the block back to the pose it was lifted to; the session's own planned put-back follows.
        Each move is a checked executor run. It is started, and the right jaws are opened, only when recovery_ready says so on
        fresh readings; a stop or fault in any move ends it there. The left jaws are never opened here."""
        ok, why = self.recovery_ready("before the return")
        if not ok:
            self.scope_result = dict(returned=False, why=why, left_at_lift=False, state="AS_AT_THE_OBSERVATION_POINT_A_PERSON_DECIDES"); return self.scope_result
        self.near_handover = True; step = "right lower"
        try:
            self.segment("observe_right_lower", "right", target={n: float(self.pre_lift_right[n]) for n in ARM})
            ok, why = self.recovery_ready("before the right jaws open")
            if not ok:
                self.scope_result = dict(returned=False, why=why, left_at_lift=False, state="RIGHT_LOWERED_BOTH_HANDS_ON_THE_BLOCK_A_PERSON_DECIDES"); return self.scope_result
            step = "right open"; wp = next(w for w in self.session.corrector.plan["waypoints"] if w["label"] == "RIGHT_OPEN"); tk = self.p1.rc.q12_to_ticks(self.mp, np.array(wp["q12"], float))
            self.segment("observe_right_open", "right", target=dict(gripper=float(round(tk["right"]["gripper"])))); self.block_owner = "left"
            step = "right back"; back = (self.insert or {}).get("pre_q12") or next(w["q12"] for w in self.session.corrector.plan["waypoints"] if w["label"] == "RIGHT_APPROACH")
            self.go("observe_right_back", back, arms=("right",)); self.near_handover = False
            step = "right home"; raw = self._read(); self.segment("observe_right_home", "right", rest={n: float(self.home["right"][n]) for n in ALL if raw["torque"]["right"][n]})
            step = "left back to the lift pose"; self.segment("observe_left_back_to_lift", "left", target={n: float(self.left_at_lift[n]) for n in ARM})
        except Exception as e:
            self.on_stop(type("D", (), dict(reasons=[f"observation return, {step}: {type(e).__name__}: {e}"]))(), None)
            self.scope_result = dict(returned=False, why=[f"{step}: {type(e).__name__}: {e}"], left_at_lift=False, state="RETURN_STOPPED_A_PERSON_DECIDES"); return self.scope_result
        raw = self._read(); Rl = float(self.mp["left"]["gripper"]["hi"] - self.mp["left"]["gripper"]["lo"]); res = abs(raw["ticks"]["left"]["gripper"] - raw["goal"]["left"]["gripper"])
        at = all(abs(raw["ticks"]["left"][n] - self.left_at_lift[n]) <= 50 for n in ARM) and all(raw["torque"]["left"][n] for n in ALL)       # 50 = the progress profile's following lead: holding at the lift pose as a loaded arm does
        self.scope_result = dict(returned=True, left_at_lift=bool(at), left_jaw_residual_ticks=float(res), left_still_loaded=bool(res > self.left_profile["floor"] * Rl), right_released=not any(raw["torque"]["right"].values()), state="RIGHT_HOME_RELEASED_LEFT_HOLDS_AT_THE_LIFT_POSE")
        self.note("support_observation_return_done", **self.scope_result); return self.scope_result

    def insertion_ready(self, observation, *, stage="PLAN"):
        """Use the current held-pose input, actual q/hold and existing teacher/complete collision checks.
        An explicitly enabled REAL contact run may use a validated MODEL_ASSISTED_RELATIVE pose while retaining its
        estimated role; otherwise the original current-observation requirements apply unchanged.
        Physical error bounds remain optional, named measurements; absence never becomes a zero bound. In their absence
        positive nominal separation is required and the result remains PROVISIONAL candidate-model planning.
        The final arrival observation checks the actual pose and never commands another insertion."""
        held = (observation or {}).get("held_object") or {}; rec = dict(stage=stage, observed_in_hand=bool(self.held is not None and self.held.get("seen_tool_from_object") is not None), pose_role=held.get("pose_role"), classification="PROVISIONAL candidate-model planning",
                                                                       evaluation_mode=getattr(self, "evaluation_mode", "LIVE_REAL"), model_path_ready=False,
                                                                       live_execution_permitted=False,
                                                                       camera_frames={k: camera_identity(v) for k, v in getattr(self, "last_images", {}).items()})
        computed = False
        model_assisted = getattr(self, "model_assisted_live_enabled", False) is True
        assisted = None
        try:
            if stage not in ("PLAN", "AFTER_PREGRASP", "BETWEEN_HOPS", "BEFORE_CLOSE"):
                raise ValueError(f"unknown insertion observation stage {stage}")
            if getattr(self, "geometry_reference_error", None):
                raise ValueError(self.geometry_reference_error)
            if not model_assisted and observation.get("observation_missing"):
                rec.update(ready=False, missing=dict(observation["observation_missing"]), why="current stage observation unavailable; remaining insert/close withheld")
                if stage != "BEFORE_CLOSE":
                    self.insert = dict(self.insert or {}, ready=False, record=dict(rec))
                return False
            partial = self.relative_alignment(observation, stage=stage)
            rec["relative_alignment"] = partial
            rec["current_wrist_rgb_consumed"] = partial["wrist_rgb_evidence"]
            current_alignment_contradiction = (
                partial.get("status") == "LOCAL_ALIGNMENT_REVIEW" and
                partial.get("request") == "WITHHOLD_INSERT_REALIGN_FROM_OBSERVED_POSE" and
                partial.get("inner_face_identity_verified") is True)
            if current_alignment_contradiction:
                rec.update(ready=False,
                           why="the current paired inner-face observation requires alignment review; no model CLEAR is inferred")
                if stage != "BEFORE_CLOSE":
                    self.insert = dict(self.insert or {}, ready=False, record=dict(rec))
                return False
            if not model_assisted:
                if self.scope == "contact" and (observation.get("visible_face_relation") or {}).get("status") != "OK":
                    rec.update(ready=False, missing=["current visible right-pad/object face relation"],
                               why=(observation.get("visible_face_relation") or {}).get("missing_reason", "current-frame pad/object producer has no observed relation"))
                    if stage != "BEFORE_CLOSE":
                        self.insert = dict(self.insert or {}, ready=False, record=dict(rec))
                    return False
                if partial["request"].startswith("WITHHOLD"):
                    rec.update(ready=False, why="the current partial relative observation requires alignment review; no full pose or CLEAR is inferred")
                    if stage != "BEFORE_CLOSE":
                        self.insert = dict(self.insert or {}, ready=False, record=dict(rec))
                    return False
            if "RIGHT_INSERT" in self.done and stage == "PLAN":               # preserve the full-task controller's completed waypoint
                rec.update(ready=bool((self.insert or {}).get("ready")), why="the insert was already run in this session"); return rec["ready"]
            computed = True
            if model_assisted:
                assisted = self._model_assisted_live_candidate(observation, stage)
                rec.update(
                    geometry_mode="MODEL_ASSISTED_REAL_NOMINAL_MODEL",
                    planning_pose_role="MODEL_ASSISTED_RELATIVE",
                    physical_error_envelope_available=False, pose_bound_m=None,
                    uncertainty_missing=["centre_m", "rotation_angle_bound_rad", "inner_pad_half_gap_uncertainty_m"],
                    uncertainty_role="MISSING physical bound; not replaced by zero, residual, history or SIM",
                    scene_application=assisted["scene_application"],
                    current_left_support=assisted["left_support"],
                    evidence_roles=dict(
                        observed=["current right-wrist pixels", "current actual q12 and feedback identity", "current left-hold registers"],
                        estimated=["MODEL_ASSISTED_RELATIVE object pose"],
                        model_prior=["existing A object geometry and kinematics"],
                    ))
            else:
                if not rec["observed_in_hand"] or held.get("object_pose_model") is None:
                    rec.update(ready=False, why="the block has not been observed in the hand in this cycle's pose"); return False
                if stage == "PLAN" and self.insert is not None and self.insert.get("for_seen_seq") == self.held.get("seen_seq"):
                    rec.update(self.insert["record"]); return bool(self.insert["ready"])
            import mujoco
            import sweepick_trajectory as tt
            from pgripper import jaw_gap_m
            if model_assisted:
                unc = {}; centre_bound = angle_bound = pad_bound = None; measured = False
                s = None; t = assisted["teacher"]
                d, m = assisted["data"], assisted["model"]
                half_source = (self.approach_state or {}).get("object_half_extents_m")
                q = np.asarray(assisted["q12"], dtype=float)
                hold_q = assisted["left_hold_q"]
            else:
                unc = (self.held.get("seen") or {}).get("object_pose_uncertainty") or {}; centre_bound, angle_bound = unc.get("centre_m"), unc.get("rotation_angle_bound_rad")
                pad_bound = unc.get("inner_pad_half_gap_uncertainty_m")
                for key in ("centre_m", "rotation_angle_bound_rad", "inner_pad_half_gap_uncertainty_m"):
                    if key in unc and (isinstance(unc[key], bool) or not isinstance(unc[key], (int, float)) or not math.isfinite(unc[key]) or unc[key] < 0):
                        raise ValueError(f"malformed supplied relative-error measurement: {key}={unc[key]}")
                measured = all(v is not None for v in (centre_bound, angle_bound, pad_bound))
                if any(k in unc for k in ("centre_m", "rotation_angle_bound_rad", "inner_pad_half_gap_uncertainty_m")) and not unc.get("rotation_bound_source"):
                    raise ValueError("supplied relative-error measurements have no identified source")
                rec.update(geometry_mode="MEASURED_RELATIVE_ENVELOPE" if measured else "CURRENT_OBSERVATION_NOMINAL_MODEL",
                           physical_error_envelope_available=measured, pose_bound_m=None,
                           uncertainty_missing=[k for k in ("centre_m", "rotation_angle_bound_rad", "inner_pad_half_gap_uncertainty_m") if k not in unc],
                           uncertainty_role="named measured relative envelope" if measured else "MISSING physical bound; not replaced by zero, residual, history or SIM")
                if not held.get("final_insertion_ready"):
                    rec.update(ready=False, why="the current held-pose observation has not identified the orientation and centre required for insertion"); return False
                if stage in ("AFTER_PREGRASP", "BETWEEN_HOPS", "BEFORE_CLOSE") and not (observation.get("contact_right_wrist_view") or {}).get("visible"):
                    rec.update(ready=False, why="the current right wrist image does not show the block; no assumed 3-D wrist TF fills that observation"); return False
                s = self.session; t = self.teacher
                if s is not None:
                    d, m, half_source = s.state["_data"], s.state["_model"], s.state.get("object_half_extents_m")
                else:
                    if not isinstance(self.approach_plan, dict) or self.approach_plan.get("_data") is None or self.approach_plan.get("_model") is None:
                        raise ValueError("the scoped approach plan has no planning data/model for the insertion re-plan")
                    d, m, half_source = self.approach_plan["_data"], self.approach_plan["_model"], (self.approach_state or {}).get("object_half_extents_m")
                q = np.array(observation["q12"], float)
                if s is None:
                    scoped_state = dict(self.approach_state or {}, _model=m, _data=d, _teacher=t)
                    codex()["receive_entry"].apply_actual_observation(
                        scoped_state, q, observation, evaluation_mode=getattr(self, "evaluation_mode", "LIVE_REAL"))
                    if not scoped_state.get("model_held_pose_available"):
                        raise ValueError("the scoped planning scene was not updated from this cycle's current observed block pose")
                hold_q = (self.approach_state or {}).get("left_hold_q") if s is None else s.state.get("left_hold_q")
            if hold_q is None or len(hold_q) != 6 or not np.isfinite(hold_q).all():
                raise ValueError("the fresh receive state has no applied left Jaw hold command")
            seed = q.copy(); seed[5] = float(hold_q[5])
            t.cmd = self.gear(seed); t.queue = []
            if stage == "BEFORE_CLOSE":
                pre = grasp = t.cmd.copy(); pre_q = grasp_q = seed.copy()
            else:
                t._plan_right_grasp(d); approach, queued = t.segment, list(t.queue); t.queue = []
                if len(queued) != 1 or queued[0][0] != "right":
                    raise ValueError("the right-grasp planner returned no unique insertion")
                pre = np.array(approach["goal"], float); grasp = np.array(queued[0][1], float); pre_q, grasp_q = t.to_canonical(pre), t.to_canonical(grasp)
                if stage in ("AFTER_PREGRASP", "BETWEEN_HOPS"):
                    rec["teacher_pregrasp_q12_not_commanded"] = pre_q.tolist()
                    pre = t.cmd.copy(); pre_q = seed.copy()                    # check the remaining insert from the actual reached pregrasp
                if self.scope == "contact":
                    pre_q[11] = grasp_q[11] = q[11]                            # measured opening, never the teacher's nominal open command
                    pre, grasp = self.gear(pre_q), self.gear(grasp_q)
            hops = [grasp_q.tolist()]
            if self.scope == "contact" and stage != "BEFORE_CLOSE":
                hops = codex()["receive_plan"].plan_insert_hops(t, d, pre_q, grasp_q, hold_q)
                grasp_q = np.asarray(hops[-1], float); grasp = self.gear(grasp_q)
            if not (np.isclose(pre_q[5], t.to_canonical(t.cmd)[5]) and np.isclose(grasp_q[5], t.to_canonical(t.cmd)[5])):
                raise ValueError("the right insertion re-plan changed the left Jaw command")
            base = d.qpos.copy(); rel = t.site_block_rel(d, "left"); bq = t.plan.carried_block(base, grasp, ("left", rel)); pd = t.plan.pose(base, grasp, bq)
            c = pd.xpos[t.cm.block_body].copy(); rb = pd.xmat[t.cm.block_body].reshape(3, 3); half = np.asarray(half_source or [self.held_half] * 3, float)
            axis = pd.body("right_gripper").xmat.reshape(3, 3)[:, 0]; midpoint = sum(pd.site(f"right_pgripper_pad_{i}_inner").xpos for i in (1, 2)) / 2
            gap = abs(float(jaw_gap_m(m, pd, "right"))); extent = float(np.abs(axis @ rb) @ half); off = float(axis @ (c - midpoint)); opening = float(gap / 2 - abs(off) - extent)
            pose_bound = float(centre_bound + 2 * np.linalg.norm(half) * np.sin(min(angle_bound, np.pi) / 2) + pad_bound) if measured else None
            checks, ticks_now, was = [], dict(self.last_raw["ticks"]["right"]), self.near_handover; self.near_handover = False
            try:
                paths = [("current insert pose before close", grasp_q)] if stage == "BEFORE_CLOSE" else (
                    ([] if stage in ("AFTER_PREGRASP", "BETWEEN_HOPS") else [("now -> pre-grasp", pre_q)]) +
                    [(f"insert hop {k + 1}/{len(hops)}", target) for k, target in enumerate(hops)])
                for label, target in paths:
                    tk = self.p1.rc.q12_to_ticks(self.mp, np.array(target, float))["right"]; traj = tt.make(ticks_now, {n: float(round(tk[n])) for n in ARM}, self.run_kw.get("profile") or tt.load_profile())
                    doc = dict(side="right", trajectory=traj, purpose="RIGHT_INSERT_REPLAN"); raw = dict(self.last_raw, ticks=dict(self.last_raw["ticks"], right=dict(ticks_now)))
                    ck = complete(self._check(doc, "right", raw), doc); checks.append(dict(path=label, verdict=ck.get("nominal_verdict"), samples=ck.get("nominal_steps_checked"), required=ck.get("samples_required"), clearance_m=ck.get("min_nominal_object_clearance_m"), first_contact=ck.get("first_nominal_contact"), geoms=ck.get("clearance_geoms")))
                    ticks_now = dict(ticks_now, **{n: float(round(tk[n])) for n in ARM})
            finally:
                self.near_handover = was
            clear = [k_["clearance_m"] for k_ in checks]; paths_ok = all(k_["verdict"] == "CLEAR" for k_ in checks) and all(isinstance(v, (int, float)) and math.isfinite(v) for v in clear)
            smallest = min([opening] + [v for v in clear if isinstance(v, (int, float))]) if paths_ok else None
            ok = bool(paths_ok and math.isfinite(smallest) and smallest > 0 and (pose_bound is None or smallest > pose_bound))
            rec.update(ready=ok, model_path_ready=ok,
                       live_execution_permitted=bool(ok and getattr(self, "evaluation_mode", "LIVE_REAL") == "LIVE_REAL" and getattr(self, "physical_execution_allowed", True) is True),
                       planner=("current actual right pose against the validated model-assisted pose" if model_assisted else "current observed block/actual right pose") if stage == "BEFORE_CLOSE" else ("HandoverTeacher._plan_right_grasp on the validated model-assisted pose" if model_assisted else "HandoverTeacher._plan_right_grasp on the observed block"), right_depth_m=t.info.get("right_depth_m"), right_beta_deg=t.info.get("right_beta_deg"), jaw_gap_inner_m=gap, block_extent_along_jaw_m=2 * extent, block_off_centre_m=off,
                       planner_options=getattr(self, "planner_options", None), left_jaw_measured_q=float(q[5]), left_jaw_applied_hold_q=float(hold_q[5]),
                       opening_clearance_m=opening, path_checks=checks, smallest_clearance_m=smallest, pose_bound_m=pose_bound, centre_m=centre_bound, rotation_angle_bound_rad=angle_bound, inner_pad_half_gap_uncertainty_m=pad_bound, rotation_bound_source=unc.get("rotation_bound_source"),
                       definition="complete nominal CLEAR and positive finite opening/path separation; additionally exceeds measured relative-error envelope when available",
                       why=None if ok else ("a path check is not CLEAR, not complete or gives no clearance" if not paths_ok else "the clearance does not exceed what the observation leaves open"))
            if stage == "BEFORE_CLOSE":
                self.close_geometry_check = dict(rec); return ok              # an observation of arrival does not command/re-plan another insert
            revision = int((self.insert or {}).get("target_revision", 0)) + 1
            self.insert = dict(for_seen_seq=None if model_assisted else self.held.get("seen_seq"),
                               model_assisted_feedback_identity=None if assisted is None else dict(assisted["feedback"]),
                               target_revision=revision, stage=stage,
                               ready=ok, pre_q12=pre_q.tolist(), grasp_q12=grasp_q.tolist(), hops_q12=hops, record=dict(rec))
            if ok:                                                              # the active plan goes on with the re-planned insert
                plan = s.corrector.plan if s is not None else self.approach_plan
                i = next(k for k, w in enumerate(plan["waypoints"]) if w["label"] == "RIGHT_INSERT"); old = plan["waypoints"][i]["q12"]
                new = list(self.insert["grasp_q12"])
                if self.scope != "contact":
                    new[5], new[11] = old[5], old[11]
                plan["waypoints"][i] = dict(plan["waypoints"][i], q12=new, q12_hops=hops, target_revision=revision,
                                          replanned_from="validated MODEL_ASSISTED_RELATIVE pose" if model_assisted else "block observed in the hand",
                                          previous_q12=old); plan["_trajectories"][i] = None
                plan["right_insert_q12"] = new
                plan["right_insert_hops_q12"] = hops
                if s is not None:
                    s.state["right_insert_q12"] = new; s.corrector.trajectory = None
                self.insert["grasp_q12"] = new
            return ok
        except Exception as e:
            computed = True
            source = "the model-assisted candidate" if model_assisted else "the observed block"
            rec.update(ready=False, why=f"the insert could not be re-planned from {source}: {type(e).__name__}: {e}")
            if stage == "BEFORE_CLOSE":
                self.close_geometry_check = dict(rec)
            else:
                self.insert = dict(for_seen_seq=None if model_assisted else self.held.get("seen_seq") if self.held else None,
                                   model_assisted_feedback_identity=None if assisted is None else dict(assisted["feedback"]),
                                   ready=False, record=dict(rec))
            return False
        finally:
            if stage == "BEFORE_CLOSE":
                self.close_geometry_check = dict(rec)
            if computed and not model_assisted and not rec.get("ready") and "RIGHT_INSERT" not in self.done and self.held is not None:
                # not ready on THIS observation: the next cycle observes the block again (a 'no' is not kept for the whole session); the right wrist frame and the top frame of that cycle are used then
                for k_ in ("seen_tool_from_object", "seen", "seen_seq"):
                    self.held.pop(k_, None)
            key = (rec.get("ready"), rec.get("why"), (self.held or {}).get("seen_seq"))
            if key != getattr(self, "_insert_noted", None):                     # recorded when it is computed or its answer changes, not once per cycle
                self._insert_noted = key; self.note("insertion_check", **rec)

    def close_sweep_check(self, raw):
        """The whole possible closing sweep of the RIGHT jaws at the pose they stand in, checked with the block in the scene by the
        same collision check as every segment (teacher's Planner.collisions): the pads of the two hands on the block are the
        intended contacts; any other pair is a contact and the close is not run. Without a held pose the checker is not asked
        and the answer is UNKNOWN; a CLEAR that did not cover every tick of the sweep is UNKNOWN. Nothing here writes CLEAR."""
        import sweepick_trajectory as tt
        if self.last_held_pose is None:
            return dict(nominal_verdict="UNKNOWN", missing=["current held object pose for the closing sweep"])
        rj = self.refs["right_jaw"]["doc"]; lo = float(self.mp["right"]["gripper"]["lo"]); arm = raw["ticks"]["right"]; g0, g1 = float(arm["gripper"]), lo + float(rj["end_stop_ticks_above_closed"]["value"]); n = int(abs(g0 - g1)) + 1
        goals = {j: [float(arm[j])] * n for j in ARM}; goals["gripper"] = [float(v) for v in np.linspace(g0, g1, n)]
        doc = dict(side="right", purpose="RIGHT_CLOSE", trajectory=dict(start=dict(arm), target=dict(gripper=g1), moving=["gripper"], dt=0.02, steps=n, goals=goals)); was = self.near_handover; self.near_handover = True
        try:
            c = complete(self._check(doc, "right", raw), doc)
        finally:
            self.near_handover = was
        return dict(c, arm_ticks={n_: arm[n_] for n_ in ARM}, sweep=[g0, g1], checked="the closing sweep itself, every tick, with the block where this cycle's held pose puts it")

    def on_stop(self, dispatch, latest):
        """A STOP of the proposal generator or a failed executor run: nothing more is sent by this owner. The arms stay as the executor left them (holding); no release, no opening, no way back is run by itself."""
        self.stopped = dict(reasons=list(getattr(dispatch, "reasons", []) or []), t=self.clock()); self.note("owner_stop", **self.stopped)
        return dict(handled=True, sent_after_stop=0, state="holding as the executor left it; a person decides")

    def wait_next_cycle(self):
        """The next feedback cycle: registers and frames are read HERE, before the task manager takes its 'now', so that what it
        acts on was measured before it decided. Cycles in which nothing was sent and nothing changed are counted in time: when
        the Jaw's own confirmation time has passed many times over with no change, waiting longer cannot change anything."""
        self.sleep(0.05); step = (getattr(self.ep, "phase", None) or {}).get("step")
        if step in ("VERIFY_PLACE", "REOBSERVE_CLEAR") and self.area is not None and self.task is not None:
            self.area.take(step, self.task); self.idle_since = self.clock()
        raw_ = self.raw(); fr_ = self._frames(); t0 = self.clock(); waited = 0
        while self.ep is not None and not all(camera_valid(fr_.get(k)) or "reader_valid" not in (fr_.get(k) or {"reader_valid": 0}) for k in ("left_wrist", "right_wrist", "top")):
            # a cycle is complete when the three images that belong to it are valid by their readers' own results. A frame that is late is waited
            # for (a new register read with it), not passed as fresh and not a reason to end the hand-over; a reader that reports an error or its
            # end is not waited for, and the wait is bounded by the same idle time as every other wait here.
            bad = {k: (fr_.get(k) or {}).get("reader_reason", "no frame") for k in ("left_wrist", "right_wrist", "top") if not camera_valid(fr_.get(k))}
            if self.images_gave_up or any("error" in v or "ended" in v or "malformed" in v for v in bad.values()) or self.clock() - t0 > self.idle_limit_s:
                if not self.images_gave_up:
                    self.note("cycle_images_not_valid", reasons=bad, waited_s=self.clock() - t0)
                self.images_gave_up = True; break                              # waited once for the whole idle time: later cycles are passed on as they are (their images invalid) so that the state machine ends on it
            self.sleep(0.02); waited += 1; raw_ = self.raw(); fr_ = self._frames()
        self.cycle = (raw_, fr_)                                                # the registers, then the frames each judged at its own fetch time by its reader's own rule
        sig = (len(self.notes_sent), getattr(self.session, "phase", None), tuple(sorted(self.done)), self.right_closed)
        if sig != self.idle_sig:
            self.idle_sig, self.idle_since = sig, self.clock()
        elif self.clock() - self.idle_since > self.idle_limit_s:
            raise RuntimeError(f"nothing was sent and nothing changed for {self.idle_limit_s:.1f} s ({self.idle_basis}); the hand-over waits for something that is not coming")

    def take_right(self, plan):
        """The right arm joins this owner with the plan's own first right-hand command (RIGHT_OPEN): the executor opens the right
        jaws and powers the whole right arm where it stands. From then on the right Goal_Position registers are commands of this session."""
        raw = self._read(); wp = next(w for w in plan["waypoints"] if w["label"] == "RIGHT_OPEN"); tk = self.p1.rc.q12_to_ticks(self.mp, np.array(wp["q12"], float))
        log = self.segment("right_take_open", "right", target=dict(gripper=float(round(tk["right"]["gripper"]))), power=list(ALL)); self.took_right = True; return log

    def abandon(self):
        """The hand-over was not entered (nothing of it was commanded to the block): if this owner powered the right arm, it is put back to its start pose and released by the executor."""
        if not getattr(self, "took_right", False):
            return None
        raw = self._read(); on = [n for n in ALL if raw["torque"]["right"][n]]
        log = self.segment("right_release", "right", rest={n: float(self.home["right"][n]) for n in on}) if on else dict(result="RETURNED_RELEASED"); self.took_right = False; return log


class Downstream:
    """PLACE and STOW for the implementation's ReceiveManipulation, with the same executor."""
    def __init__(self, owner):
        self.o, self.context = owner, None

    def set_receive_context(self, data):
        self.context = data

    def place(self, task, now):
        tm = codex()["receive_session"].tm; o = self.o; tgt = o.refs["place"]["doc"]
        if tgt is None:
            return tm.CommandResult(tm.Status.RUNNING, tm.Kind.REAL, "place target not confirmed on the real desk", now, dict(status="NOT_READY", missing=["confirmed place target"]))
        try:
            import mujoco
            obs = o.last_obs; t = o.teacher; m = t.m; d = mujoco.MjData(m); o.p1.sweepick_kin.set_q12(m, d, np.array(obs["q12"])); mujoco.mj_forward(m, d)
            T = np.array(self.context["object_pose_model"]); adr = t.plan.cm.block_qpos; quat = np.empty(4); mujoco.mju_mat2Quat(quat, np.ascontiguousarray(T[:3, :3]).ravel()); d.qpos[adr:adr + 7] = np.r_[T[:3, 3], quat]; mujoco.mj_forward(m, d)
            t.cmd = o.gear(obs["q12"]); rel = t.site_block_rel(d, "right"); t.plan.pose(d.qpos.copy(), t.cmd); ps, rot = t.plan.site_frame("right")
            B = np.eye(4); B[:3, :3] = rot.T @ T[:3, :3]; B[:3, 3] = rot.T @ (T[:3, 3] - ps); Binv = np.linalg.inv(B)       # the block in the right tool frame, from this cycle's held pose
            c = np.array(tgt["centre_xy_m"], float); z = float(tgt["surface_z_m"]) + float(o.held_half); up = float(tgt.get("approach_height_m", 0.05)); found = None; tried = []
            ups = [np.eye(3)] + [np.array(r, float) for r in ([[1, 0, 0], [0, 0, -1], [0, 1, 0]], [[1, 0, 0], [0, 0, 1], [0, -1, 0]], [[0, 0, 1], [0, 1, 0], [-1, 0, 0]], [[0, 0, -1], [0, 1, 0], [1, 0, 0]], [[1, 0, 0], [0, -1, 0], [0, 0, -1]])]
            for R0 in ups:                                                        # the block flat on the target with any of its faces down and any yaw: the first the right arm can reach and the teacher's path check passes
                for yaw in np.linspace(-np.pi, np.pi, 24, endpoint=False):
                    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.0]]); Tb = np.eye(4); Tb[:3, :3] = Rz @ R0; Tb[:3, 3] = [c[0], c[1], z]; Ts = Tb @ Binv; Ta = Ts.copy(); Ta[2, 3] += up
                    try:
                        t.cmd = o.gear(obs["q12"]); g1, i1 = t.plan_move(d, "right", Ta[:3, 3], Ta[:3, 0], [Ta[:3, 2]], carry=("right", rel), allow=("right",), label="PLACE above")
                        keep = t.cmd.copy(); t.cmd = g1[-1]; base2 = t.plan.pose(d.qpos.copy(), g1[-1], t.plan.carried_block(d.qpos.copy(), g1[-1], ("right", rel))).qpos.copy(); d2 = mujoco.MjData(m); d2.qpos[:] = base2; mujoco.mj_forward(m, d2)
                        g2, i2 = t.plan_move(d2, "right", Ts[:3, 3], Ts[:3, 0], [Ts[:3, 2]], carry=("right", rel), allow=("right",), label="PLACE down"); found = (g1, g2, i1, i2, float(yaw)); break
                    except Exception as e:
                        tried.append(str(e)[:60])
                if found:
                    break
            if not found:
                raise RuntimeError(f"no reachable, collision-free way to set the block flat on the place target ({len(tried)} candidates; last: {tried[-1] if tried else None})")
            goals, goals2, info, info2, yaw = found
            for g in goals:
                o.go("place_above", t.to_canonical(g).tolist(), arms=("right",))
            for g in goals2:
                o.go("place_down", t.to_canonical(g).tolist(), arms=("right",))
            o.block_owner = "placed"; q_open = float(o.right_open_q); raw = o._read(); tk = o.p1.rc.q12_to_ticks(o.mp, np.r_[obs["q12"][:11], q_open])
            o.segment("place_open", "right", target=dict(gripper=float(round(tk["right"]["gripper"]))))
            for g in reversed(goals):
                o.go("place_retreat", t.to_canonical(g).tolist(), arms=("right",))
            o.ack(); o.note("place_done", target=tgt.get("name"))
            return tm.CommandResult(tm.Status.SUCCEEDED, tm.Kind.REAL, "placed and opened by the executor; whether the block rests in the target is VERIFY_PLACE's question", now, dict(status="COMPLETE", target=tgt, ik=[str(info), str(info2)]))
        except Exception as e:
            o.on_stop(type("D", (), dict(reasons=[f"PLACE: {type(e).__name__}: {e}"]))(), None)
            return tm.CommandResult(tm.Status.FAILED, tm.Kind.REAL, f"PLACE: {type(e).__name__}: {e}", now, dict(status="STOP", requires_explicit_restart=True))

    def stow(self, now):
        tm = codex()["receive_session"].tm; o = self.o
        try:
            for side in ("right", "left"):
                raw = o._read(); on = [n for n in ALL if raw["torque"][side][n]]
                if on:
                    o.segment("stow", side, rest={n: float(o.home_command[side][n]) for n in on})
            o.note("stow_done")
            return tm.CommandResult(tm.Status.SUCCEEDED, tm.Kind.REAL, "both arms returned to their start poses and released by the executor", now, dict(status="COMPLETE"))
        except Exception as e:
            o.on_stop(type("D", (), dict(reasons=[f"STOW: {type(e).__name__}: {e}"]))(), None)
            return tm.CommandResult(tm.Status.FAILED, tm.Kind.REAL, f"STOW: {type(e).__name__}: {e}", now, dict(status="STOP", requires_explicit_restart=True))


class Area:
    """Perception for the TaskManager's VERIFY_PLACE and REOBSERVE_CLEAR: the session's own look (SDK top depth through the board) on a region. This is the ONE final area check and the one place a clean-resume request comes from."""
    def __init__(self, owner, out, look):
        self.o, self.out, self.look, self.kind, self.k = owner, Path(out), look, codex()["receive_session"].tm.Kind.REAL, 0
        self.records = []; self.taken = None

    def region(self, region, tag):
        from sweepick_local_loop import real_area
        self.k += 1; sc = self.look(self.out, f"{tag}_{self.k}")
        if sc.get("support_region_at") is not None:                              # a look that already evaluated this region (tests)
            reg = sc["support_region_at"](region)
        else:
            p1 = self.o.p1; board = (sc.get("board") or {}).get("T_camera_from_board")
            if board is None:
                reg = dict(valid=False, state="UNKNOWN", reason="board not found")
            else:                                                                 # as sweepick_pick01.support_region: the region centre into the printed board's frame
                depth = np.load(Path(sc["_out"]) / "looks" / sc["tag"] / "depth_mm.npy"); m = p1.model(); Tb = np.eye(4); Tb[:3, 3] = p1.DATUM_IN_MODEL_BASE; Tw = np.eye(4); Tw[:3, 3] = m.body("left_base").pos
                pb = np.linalg.inv(Tw @ Tb @ p1.T_DATUM_FROM_BOARD) @ np.r_[region.center_xy[0], region.center_xy[1], p1.BOARD_Z_IN_WORLD, 1.0]
                reg = real_area(depth, sc["K"], board, 0.0, (float(pb[0]), float(-pb[1])), region.radius_m)
        done = self.o.clock(); self.records.append(dict(tag=tag, region=region.region_id, result=reg, look_completed_monotonic_s=done, sensor_stamps=sc.get("stamps")))
        self.o.note("area_look", tag=tag, region=region.region_id, state=reg.get("state"), valid=reg.get("valid")); return reg, sc, done

    def take(self, step, task):
        """Called by the owner BEFORE the task manager's step for the two looking states, so that the look is finished when the manager takes its 'now'. The record's stamp is when the look was completed; the sensor's own stamps are kept beside it."""
        if step == "VERIFY_PLACE":
            self.taken = ("place", self.region(task.target_region, "05_place"))
        elif step == "REOBSERVE_CLEAR":
            self.taken = ("source", self.region(task.source_region, "06_area"))

    def _get(self, kind, region, tag):
        if self.taken is not None and self.taken[0] == kind:
            r, self.taken = self.taken[1], None; return r
        return self.region(region, tag)                                          # not taken beforehand: taken now, and then it is later than the manager's 'now' and the manager will call it not fresh

    def observe_object(self, task, now):
        return None

    def _validity(self, reg, now):
        tm = codex()["receive_session"].tm; import dataclasses
        names = [f.name for f in dataclasses.fields(tm.Validity)]; v = dict(valid=bool(reg.get("valid")), stamp_s=now, reason="" if reg.get("valid") else str(reg.get("reason") or reg.get("state") or "not validly observed"), source="sweepick look: SDK top depth through the printed board", frame_id="model")
        return tm.Validity(**{n: v[n] for n in names if n in v})

    def observe_area(self, region, now):
        tm = codex()["receive_session"].tm; reg, sc, done = self._get("source", region, "06_area")
        return tm.AreaObservation(region_id=region.region_id, clear=bool(reg.get("valid") and reg.get("state") == "CLEAR"), points_above_surface=int(reg.get("points_above_8mm") or 0), validity=self._validity(reg, done))

    def verify_place(self, task, now):
        """In the target region: validly observed and occupied (top depth). Supported: the right jaws were opened by the executor at the place pose and the right hand went back up (its runs ended HOLDING). Separated: the right gripper register stands at its open tick."""
        tm = codex()["receive_session"].tm; reg, sc, done_t = self._get("place", task.target_region, "05_place"); raw = self.o._read(); done = [n["name"] for n in self.o.notes if n.get("kind") == "segment" and n.get("result") == "HOLDING"]
        rlo, rhi = float(self.o.mp["right"]["gripper"]["lo"]), float(self.o.mp["right"]["gripper"]["hi"]); opened = (raw["ticks"]["right"]["gripper"] - rlo) / (rhi - rlo) >= self.o.right_open_q - 2 * self.o.left_profile["floor"]
        return tm.PlaceVerification(object_in_region=bool(reg.get("valid") and reg.get("state") == "OCCUPIED"), object_supported=bool("place_open" in done and "place_retreat" in done), gripper_separated=bool(opened and "place_retreat" in done),
                                    evidence=dict(region=reg, executor_runs=[n for n in done if n.startswith("place")], right_gripper_tick=raw["ticks"]["right"]["gripper"]), validity=self._validity(reg, done_t))


def complete(c, doc=None):
    """A CLEAR counts only when every required sample was really checked; otherwise the verdict is UNKNOWN (the implementation's rule)."""
    need = c.get("samples_required")
    if need is None and doc is not None:
        need = max(1, int(doc["trajectory"]["steps"]))
    if c.get("nominal_verdict") == "CLEAR" and need is not None and c.get("nominal_steps_checked") != need:
        return dict(c, nominal_verdict="UNKNOWN", incomplete=f"{c.get('nominal_steps_checked')} of {need} samples checked")
    return c



def record_receive_planning(out, inputs, plan=None):
    """Preserve exact non-image planner inputs and both accepted/rejected candidate results."""
    encode = lambda value: value.tolist() if isinstance(value, np.ndarray) else _plain(value)
    (Path(out) / "RECEIVE_PLANNER_INPUT.json").write_text(json.dumps(inputs, indent=1, default=encode))
    if plan is not None:
        public = {key: value for key, value in plan.items() if not key.startswith("_")}
        (Path(out) / "RECEIVE_PLANNER_RESULT.json").write_text(json.dumps(public, indent=1, default=encode))


def _plain(o, depth=0):
    """A record without image arrays: an array becomes its shape (the frame itself is in the recorder's files under its number)."""
    if isinstance(o, np.ndarray):
        return o.tolist() if o.size <= 16 else f"<array {o.shape}>"
    if isinstance(o, dict):
        return {str(k): _plain(v, depth + 1) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_plain(v, depth + 1) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return o if isinstance(o, (str, int, float, bool, type(None))) else str(o)[:300]


def _mk(cls, **k):
    """Build one of the TaskManager's records from the fields it has (its field names are its own)."""
    import dataclasses
    names = [f.name for f in dataclasses.fields(cls)]; return cls(**{n: k[n] for n in names if n in k})


NEEDS = dict(full=("right_jaw", "right_wrist", "support", "place"), observe=("right_jaw", "right_wrist"), approach=(), contact=())
# SUPPORT_OBSERVATION asks for neither the support reference (it is what this session is run to find) nor the place target (nothing is placed).
# It does ask for the right gripper's own profile (the close) and the right wrist reference (the judgement 'the block is at the right tool' and
# the orientation bound of the insert are made on the right wrist image): both come from the RIGHT CONTACT OBSERVATION that is run before any
# hand-over session (sweepick_contact01.py --side right), so nothing is asked for at the start that only this session could produce.
SCOPES = dict(full="full", FULL_TASK="full", observe="observe", SUPPORT_OBSERVATION="observe", approach="approach", RECEIVE_APPROACH="approach",
              contact="contact", RECEIVE_CONTACT="contact")


def build(out, episode, *, p1=None, run_kw=None, read=None, frames=None, look=None, check=None, refs=None, run=None, grip=None, clock=time.monotonic, sleep=time.sleep, applier=None, rig=None, scope="full", right_lift_m=None, observation_plan=None, contact_plan=None, record_kind=None, partial_observer=None):
    """The hand-over callback for sweepick_pick01.run_session(full_chain=True): the implementation's FullEpisodeHandoff with this process's owner injected. Returns (handoff, owner)."""
    if p1 is None:
        import sweepick_pick01 as p1
    cx = codex(); entry, sess, tm, tg = cx["receive_entry"], cx["receive_session"], cx["receive_session"].tm, cx["receive_session"].tg
    refs = refs if refs is not None else field_refs(); out = Path(out); codex_sha = {n: hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for n, m in cx.items()}
    o = Owner(out, episode, refs, p1=p1, run=run, grip=grip, run_kw=run_kw, read=read, frames=frames, look=look, check=check, clock=clock, sleep=sleep)
    o.face_geometry_prepared = globals().get("PREPARED_FACE_GEOMETRY")
    if o.face_geometry_prepared is not None:
        o.note("observation_immutable_prepare", identity=o.face_geometry_prepared["identity"],
               timings=globals().get("PREPARED_FACE_GEOMETRY_TIMINGS"), prepared_before_camera=True)
    assert scope in SCOPES, scope; scope = SCOPES[scope]; plan_missing = []
    if scope == "observe":                                                  # the motion and object-support plan is an explicit input (the implementation's contract); a bare number is not a plan
        plan_missing = observation_plan_missing(observation_plan)
        if not plan_missing:
            right_lift_m = float(observation_plan["right_lift_m"])
        elif right_lift_m is not None:
            plan_missing = ["a support-observation plan file (source, right_lift_m, object_support_plan, end_state): a lift given as a bare number is not an approved plan"]; right_lift_m = None
    contact_evidence = None
    if scope == "contact":
        import sweepick_trajectory as tt
        plan_missing, contact_evidence = contact_plan_check(contact_plan, o.mp, o.msha, (run_kw or {}).get("profile") or tt.load_profile())
    o.scope, o.right_lift_m, o.observation_plan = scope, None if right_lift_m is None else float(right_lift_m), observation_plan
    o.contact_plan, o.contact_baseline = contact_plan, contact_evidence
    o.contact_close_executed = False
    episode_kind = (getattr(episode, "kind", None) if scope == "contact" else "REAL" if getattr(episode, "kind", "REAL") == "REAL" else "SYNTHETIC")
    if scope == "contact" and episode_kind not in ("REAL", "SYNTHETIC"):
        plan_missing.append("contact Episode source kind must be explicitly REAL or SYNTHETIC")
    o.record_kind = record_kind or episode_kind
    o.partial_observer = partial_observer or o.observe_relative_faces
    o.contact_insert_history = []
    if scope == "contact" and o.record_kind != episode_kind:
        plan_missing.append("contact evidence source kind must match the actual Episode source kind")
    pin_scope = "SUPPORT_OBSERVATION" if scope == "observe" else "FULL_TASK"; import inspect
    scope_in_pin = "execution_scope" in inspect.signature(entry.make_session).parameters
    motion_plan = contact_plan if scope == "contact" else observation_plan
    o.planner_options = copy.deepcopy((motion_plan or {}).get("planner_options"))
    planning_scope = dict(full="FULL_TASK", observe="SUPPORT_OBSERVATION", approach="RECEIVE_APPROACH", contact="RECEIVE_CONTACT")[scope]
    scoped_planner = "planning_scope" in inspect.signature(cx["receive_plan"].plan_receive).parameters
    planning_kw = dict(planning_scope=planning_scope, planner_options=o.planner_options) if scoped_planner else {}
    if o.planner_options is not None:
        if not scoped_planner:
            plan_missing.append("the selected receive revision has no explicit IK-candidate/scope input")
        else:
            try:
                cx["receive_plan"].validate_planner_options(o.planner_options)
            except (ValueError, TypeError, KeyError) as exc:
                plan_missing.append(f"receive planner options: {exc}")
    if scope == "contact" and (contact_plan or {}).get("arm_settle_trim") is True:
        o.run_kw.setdefault("settle_trim", list(ARM))
    o.geometry_reference = None; o.geometry_reference_error = None
    if scope == "contact" and (contact_plan or {}).get("relative_geometry_reference"):
        try:
            o.geometry_reference = held_geometry_reference((contact_plan or {}).get("relative_geometry_reference"),
                model_sha=hashlib.sha256(p1.SCENE.read_bytes()).hexdigest(), mapping_sha=o.msha,
                planner_options=o.planner_options, camera_source_sha=hashlib.sha256(Path(p1.__file__).read_bytes()).hexdigest(), source_kind=episode_kind)
        except (ValueError, TypeError, KeyError, OSError) as exc:
            o.geometry_reference_error = f"supplied optional held/right-jaw error reference: {exc}"
    if scope == "observe" and not scope_in_pin:
        plan_missing.append("the selected receive pin has no SUPPORT_OBSERVATION scope (select the final pin)")
    o.scene_look = o.last_held_pose = o.scene = None; o.area = o.task = None; o.near_handover = False; o.last_obs = None; o.held_half = 0.02; o.held_seen_fn = held_seen
    down = Downstream(o); area = Area(o, out, look or p1.look); ports = ("/dev/dapier/left_arm", "/dev/dapier/right_arm")
    import sweepick_real_exec as rx
    import sweepick_move_a as mv
    import sweepick_trajectory as tt

    def right_profile():
        rj = refs["right_jaw"]
        if not rj["usable"]:
            return None
        d = rj["doc"]; lo, hi = float(o.mp["right"]["gripper"]["lo"]), float(o.mp["right"]["gripper"]["hi"])
        return dict(verified=True, side="right", source_kind="REAL", source=rj["file"], device_identity=d.get("device_identity"), measurement_evidence=d.get("measurement_evidence"), device=None)

    def session_kwargs(present):
        lo, hi = float(o.mp["left"]["gripper"]["lo"]), float(o.mp["left"]["gripper"]["hi"]); rp = right_profile()
        ap = applier or rx.Applier(o.mp, (run_kw or {}).get("profile") or tt.load_profile(), present, real_jaw_profile=None if rp is None else dict(left=str(mv.JAW_PROFILE), right=refs["right_jaw"]["file"]))                       # the same motion profile the executor uses
        prof = (run_kw or {}).get("profile") or tt.load_profile(); dt = float(prof["dt"])
        if rp is not None:
            rp["device"] = mv.jaw_device(float(o.mp["right"]["gripper"]["lo"]), float(o.mp["right"]["gripper"]["hi"]), prof, Path(refs["right_jaw"]["file"]))
        return dict(ports=ports, owner_pid=os.getpid(), port_owner_pids={p: os.getpid() for p in ports}, rig=rig if rig is not None else o, applier=ap, control_dt=dt, session_id=episode.id,
                    left_jaw=tg.Jaw(mv.jaw_device(lo, hi, prof), dt, provenance=tg.INDEPENDENT), right_jaw_profile=rp, right_jaw=None, support_confirmation=lambda ob: ob.get("support_confirmation"),
                    release_controller=Release(o), load_transfer_controller=LoadTransfer(o), insertion_alignment=o.insertion_ready, **(dict(execution_scope=pin_scope) if scope_in_pin else {}))

    def task_of(state):
        c = (state.get("plan") or {}).get("object_centre_world") or json.loads((out / state["scene"]).read_text())["object"]["centre_world"]; tgt = refs["place"]["doc"] or {}
        return tm.Task(f"sweepick-{episode.id}", tm.Region("source", "model", (float(c[0]), float(c[1])), 0.04, float(p1.BOARD_Z_IN_WORLD)), tm.Region(tgt.get("name", "place"), "model", tuple(tgt.get("centre_xy_m", (0.0, 0.0))), float(tgt.get("radius_m", 0.04)), float(tgt.get("surface_z_m", p1.BOARD_Z_IN_WORLD))), needs_two_arm_coordination=True)

    def preflight(o_, state):
        """Before the first motor command: the field records, then the implementation's own reference check on exactly the objects the hand-over will use."""
        items = {n: dict(present=r["present"], usable=r["usable"], file=r["file"], what=r["what"], blocks=r["blocks"]) for n, r in refs.items()}
        missing = [f"FIELD {n}: {r['what']} -> {r['file']}" for n, r in refs.items() if not r["usable"] and n in NEEDS[scope]]; items["scope"] = dict(scope=scope, needs=NEEDS[scope], left_release="NOT POSSIBLE in this scope" if scope in ("observe", "approach", "contact") else "after a confirmed support record")
        missing += [f"PLAN {m}" for m in plan_missing]; items["observation_plan"] = dict(given=observation_plan is not None, missing=plan_missing) if scope == "observe" else None
        items["contact_plan"] = dict(given=contact_plan is not None, missing=plan_missing, baseline=contact_evidence,
                                     held_relative_input_stage="after actual lift/right pregrasp; never a START input",
                                     optional_error_reference=getattr(o, "geometry_reference_error", None),
                                     execution_range="fresh actual right ID6 -> calibrated minimum from the plan; early CONTACT_CANDIDATE holds actual and records raw rows; normal end reopens and releases right") if scope == "contact" else None
        o.home = dict(left=state["home_ticks"], right=state["right_ticks"])
        for side in ("left", "right"):
            o.home_command[side], o.home_command_basis[side] = p1.home_command_ticks(o.home[side], o.mp, side)
        items["home_command"] = o.home_command_basis
        lo, hi = float(o.mp["left"]["gripper"]["lo"]), float(o.mp["left"]["gripper"]["hi"])
        o.left_open_q = (float(o.home_command["left"]["gripper"]) - lo) / (hi - lo)
        rlo, rhi = float(o.mp["right"]["gripper"]["lo"]), float(o.mp["right"]["gripper"]["hi"])
        o.right_open_q = (float(o.home_command["right"]["gripper"]) - rlo) / (rhi - rlo)
        if scope in ("approach", "contact"):
            code_missing = []
            try:
                if state.get("ready") is not True:
                    code_missing.append("current PICK observation is not ready")
                if any(not isinstance(state.get(k), dict) or not all(n in state[k] for n in ALL) for k in ("home_ticks", "right_ticks")):
                    code_missing.append("current left home/lift seed and right raw pose")
                p1.model()
                for name, fn in (("adapt_snapshot", cx["receive_real_adapter"].adapt_snapshot), ("plan_receive", cx["receive_plan"].plan_receive),
                                 ("Owner.go", o.go), ("Owner.segment", o.segment), ("Owner._check", o._check),
                                 ("Owner.insertion_ready", o.insertion_ready), ("Owner.contact_close", o.contact_close),
                                 ("current-frame producer", o.partial_observer), ("new-frame consumer", o.contact_observation)):
                    if not callable(fn):
                        code_missing.append(name)
                if scope == "contact" and not callable(getattr(cx["receive_plan"], "plan_insert_hops", None)):
                    code_missing.append("selected RECEIVE pin plan_insert_hops")
                if scope == "contact":
                    from sweepick_261009_offsite_relative_features import current_frame_visible_face_relation
                    if not callable(current_frame_visible_face_relation):
                        code_missing.append("current RGB-D face producer")
            except Exception as e:
                code_missing.append(f"scene/model check failed: {type(e).__name__}: {e}")
            items["code_references"] = dict(missing=code_missing, checked_by="RECEIVE_APPROACH existing adapter + plan_receive + Owner")
            items["fresh_left_grasp"] = "checked after this session's lift from HANDOFF_SNAPSHOT plus fresh Goal_Position/Torque/Status; no saved run is live input"
            now_sha = {n: hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for n, m in cx.items()}; items["codex_modules"] = dict(sha256=codex_sha, dir=str(CODEX))
            if now_sha != codex_sha:
                code_missing.append("the implementation's receive modules changed on disk since this process imported them: " + ", ".join(n for n in cx if now_sha[n] != codex_sha[n]) + " (a session runs on the files it started with)")
            return dict(ready=not missing and not code_missing, missing=missing + [f"CODE {m}" for m in code_missing], items=items)
        handoff.task = o.task = task_of(state)
        present = dict(left=state["home_ticks"], right=state["right_ticks"])
        try:
            rr = entry.receive_references_ready(session_kwargs(present), observation_source=o.observation, downstream=down, real_stow_profile=dict(kind="return to the start poses of this session", home=o.home_command), apply_dispatch=o.apply_dispatch, on_stop=o.on_stop, record_event=o.note)
            code_missing = [m for m in rr["missing"] if m not in ("right REAL Jaw profile",)]
        except Exception as e:
            code_missing = [f"reference check failed: {type(e).__name__}: {e}"]
        items["code_references"] = dict(missing=code_missing, checked_by="sweepick_261008_receive_entry.receive_references_ready")
        now_sha = {n: hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for n, m in cx.items()}; items["codex_modules"] = dict(sha256=codex_sha, dir=str(CODEX))
        if now_sha != codex_sha:
            code_missing.append("the implementation's receive modules changed on disk since this process imported them: " + ", ".join(n for n in cx if now_sha[n] != codex_sha[n]) + " (a session runs on the files it started with)")
        return dict(ready=not missing and not code_missing, missing=missing + [f"CODE {m}" for m in code_missing], items=items)

    def prepare(snapshot, scene, evidence):
        prepare_started = time.perf_counter()
        st = json.loads((out / "STATE.json").read_text()); close = (st["stages"].get("close") or {}); end = close.get("end") or {}
        raw = o._read(); o.scene_look = scene
        q_close = p1.rc.ticks_to_q12(o.mp, dict(left={n: float(v) for n, v in end["ticks"].items()}, right=raw["ticks"]["right"])); c, yaw = p1.state_object(out, st)
        h = held_pose(q_close, scene["q12"], c, yaw, model=p1.model(), pad_geoms=p1.PAD_GEOMS, source=f"session {out.name}: close readback + look {scene.get('tag')}"); h["grasp_evidence"] = (snapshot or {}).get("grasp_evidence"); o.held = h
        o.scene = scene_bindings(p1.model(), scene, p1.BOARD_Z_IN_WORLD, model_sha=hashlib.sha256(p1.SCENE.read_bytes()).hexdigest(), source_sha=hashlib.sha256((RUNTIME / "source/production_source_g000/integration_scenes.py").read_bytes()).hexdigest())
        b = dict(source_kind="REAL", object_pose_model=h["coarse_preview_pose"], object_pose_frame=o.scene["frame"], object_pose_source="COARSE PREVIEW ESTIMATE (see components): " + h["object_pose_source"], object_pose_uncertainty=dict(h["object_pose_uncertainty"], components=h["components"]),
                 q_mapping_verified=False, geometry_verified=False, right_jaw_profile=None, scene={k: v for k, v in o.scene.items() if k not in ("provenance", "geometry_verified")})
        (out / "HELD_POSE.json").write_text(json.dumps(h, indent=1, default=str))
        grip_log = (close.get("grip") or {}); onset = (grip_log.get("contact_onset") or {}).get("commanded"); lo, hi = float(o.mp["left"]["gripper"]["lo"]), float(o.mp["left"]["gripper"]["hi"])
        o.left_onset_command = None if onset is None else (float(onset) - lo) / (hi - lo)
        o.last_held_pose = h["coarse_preview_pose"]; o.left_at_lift = {n: float(raw["ticks"]["left"][n]) for n in ALL}
        if scope in ("approach", "contact"):
            state = cx["receive_real_adapter"].adapt_snapshot(snapshot, b)
            planner_input = dict(snapshot=snapshot, held_binding=b, adapted_state=state, measured_feedback=raw,
                held_pose=h, planning_arguments=planning_kw, scene_look={key: scene.get(key) for key in
                    ("tag", "q12", "T_world_camera", "K", "object", "camera", "desk_z", "stamps")},
                source_references={name: dict(file=str(out / name), sha256=hashlib.sha256((out / name).read_bytes()).hexdigest())
                    for name in ("HELD_POSE.json", "HANDOFF_SNAPSHOT.json", "close/run1/move.json", "lift/run1/move.json") if (out / name).exists()},
                source_selection=selection("RECEIVE_CONTACT" if scope == "contact" else "RECEIVE_APPROACH"))
            record_receive_planning(out, planner_input)
            if state.get("status") != "INPUT_VALID":
                return dict(status="NOT_READY", detail="fresh lift snapshot/held scene is not valid for RECEIVE_APPROACH",
                            missing=list(state.get("missing") or []) + list(state.get("invalid") or []), real_execution_ready=False, motor_commands_sent=0)
            planner_started = time.perf_counter()
            try:
                plan = cx["receive_plan"].plan_receive(state, **planning_kw)
            finally:
                timing = dict(stage="fresh_lift_receive_prepare", planner_call_s=time.perf_counter() - planner_started,
                              prepare_to_planner_return_s=time.perf_counter() - prepare_started,
                              basis="actual call durations; separate from camera producer time and event gaps")
                (out / "RECEIVE_PLANNER_TIMINGS.json").write_text(json.dumps(timing, indent=1))
                o.note("receive_planner_timing", **timing)
            record_receive_planning(out, planner_input, plan)
            if plan.get("status") != "PLAN_PREVIEW":
                return dict(status="NOT_READY", detail="the existing teacher did not produce the carry/right-approach plan",
                            missing=list(plan.get("missing") or []) + [str((plan.get("failure") or {}).get("cause"))],
                            planner_info=plan.get("planner_info"), input_record="RECEIVE_PLANNER_INPUT.json", result_record="RECEIVE_PLANNER_RESULT.json",
                            real_execution_ready=False, motor_commands_sent=0)
            o.approach_plan, o.approach_state, o.teacher = plan, state, plan.get("_teacher")
            external_scope = "RECEIVE_CONTACT" if scope == "contact" else "RECEIVE_APPROACH"
            public = dict(status="PLAN_PREVIEW", scope=external_scope, waypoints=plan["waypoints"], planner_info=plan.get("planner_info"),
                          execution_range=("CARRY_* through RIGHT_APPROACH; only with a fresh insertion-ready observation, re-plan/check RIGHT_INSERT then finite right-ID6 contact collection; normal reopen/reverse only; no left release/support/full task"
                                           if scope == "contact" else "CARRY_* through RIGHT_APPROACH only; RIGHT_OPEN, RIGHT_INSERT, right Jaw, release and full task are not executed"),
                          pose_provenance="coarse held estimate from this session's fresh lift; sufficient for collision-checked approach preview, not insertion",
                          real_execution_ready=False, motor_commands_sent=0)
            (out / "RECEIVE_PLAN.json").write_text(json.dumps(_plain(public), indent=1))
            return public
        man = _prepare(entry, snapshot, b, session_kwargs(dict(left=raw["ticks"]["left"], right=raw["ticks"]["right"])), o, down, planning_kw=planning_kw)
        if not isinstance(man, dict):
            try:
                o.take_right(man.session.corrector.plan)                          # after the plan exists and before the first cycle; undone by abandon() if the hand-over is not entered
            except Exception:
                man.session.close(); raise                                        # the session made for this hand-over is ended with it
            o.session = man.session; o.teacher = man.session.state.get("_teacher") or getattr(man.session.corrector, "teacher", None)
            (out / "RECEIVE_PLAN.json").write_text(json.dumps(dict(waypoints=man.session.corrector.plan["waypoints"], planner_info=man.session.corrector.plan.get("planner_info"), pose_provenance="coarse preview estimate; the insert waits for the block observed in the hand"), indent=1, default=str))
        return man

    handoff = cx["full_episode"].FullEpisodeHandoff(prepare=prepare, preflight=preflight, task=None, perception=area, wait_next_cycle=o.wait_next_cycle, recorder=episode, **(dict(execution_scope=pin_scope) if scope_in_pin else {}))
    handoff.task = object()                                                     # replaced by the task of this session in preflight (the source region is where the block was observed)
    handoff.owner, handoff.area, handoff.downstream, handoff.abandon = o, area, down, o.abandon; o.area = area
    return Scoped(handoff, o, scope), o


def _prepare(entry, snapshot, b, kw, o, down, *, planning_kw=None):
    """the implementation's prepare_receive on this owner's references."""
    return entry.prepare_receive(snapshot, b, session_kwargs=kw, observation_source=o.observation, downstream=down, real_stow_profile=dict(kind="return to the start poses of this session", home=o.home_command),
                                 apply_dispatch=o.apply_dispatch, on_stop=o.on_stop, record_event=o.note, **(planning_kw or {}))
