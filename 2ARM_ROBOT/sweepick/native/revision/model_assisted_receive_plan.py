"""Plan one observed right-hand receive hop without changing the live Owner.

The geometry producer is deliberately separate from this consumer.  A candidate
pose is accepted only when its camera and motor identities are the identities of
the observation and of the Owner's current stored read.  This module previews
the first hop and its normal reverse; it never authorises or sends a command.
"""
import copy
from functools import partial

import numpy as np


ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
MODE_SOURCE = {
    "NO_DEVICE_REAL_STORED": "REAL",
    "NO_DEVICE_SENSOR_SIM": "SYNTHETIC",
    "LIVE_REAL": "REAL",
}
REQUIRED_OWNER_ATTRIBUTES = (
    "evaluation_mode", "approach_plan", "approach_state", "teacher", "last_raw",
    "p1", "mp", "msha", "run_kw", "_check", "gear", "block_owner",
    "last_held_pose", "near_handover",
)


class _Rejected(ValueError):
    def __init__(self, stage, cause):
        super().__init__(cause)
        self.stage = stage


def _reject(stage, cause):
    raise _Rejected(stage, cause)


def _finite_vector(value, length, name):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (length,) or not np.isfinite(vector).all():
        _reject("identity", f"{name}: finite length-{length} vector required")
    return vector


def _camera_identity(observation):
    wrist = observation.get("wrist") or {}
    frame = wrist.get("right")
    if frame is None:
        frame = (observation.get("camera_frames") or {}).get("right_wrist")
    if not isinstance(frame, dict):
        _reject("identity", "current right wrist frame is missing")
    if frame.get("reader_valid") is not True or frame.get("fresh") is not True:
        _reject("identity", "current right wrist frame is not valid and fresh")
    identity = {key: frame.get(key) for key in ("n", "recv_monotonic_s")}
    if identity["n"] is None or identity["recv_monotonic_s"] is None:
        _reject("identity", "current right wrist frame has no complete identity")
    return identity


def _validate_geometry(observation, geometry):
    if not isinstance(geometry, dict):
        _reject("geometry", "MODEL_ASSISTED_RELATIVE geometry packet is missing")
    if geometry.get("status") != "CANDIDATE":
        _reject("geometry", geometry.get("missing_reason") or
                f"geometry status is {geometry.get('status')!r}, not CANDIDATE")
    if geometry.get("classification") != "MODEL_ASSISTED_RELATIVE":
        _reject("geometry", "geometry classification is not MODEL_ASSISTED_RELATIVE")
    if geometry.get("model_pose_available") is not True:
        _reject("geometry", "geometry has no model pose")
    if geometry.get("model_path_ready") is not False:
        _reject("geometry", "geometry producer must not inject model path readiness")
    if geometry.get("live_execution_permitted") is not False:
        _reject("geometry", "geometry packet must explicitly withhold live execution")
    q12 = _finite_vector(observation.get("q12"), 12, "observation.q12")
    geometry_q12 = _finite_vector(geometry.get("actual_q12"), 12, "geometry.actual_q12")
    if not np.array_equal(q12, geometry_q12):
        _reject("identity", "geometry actual_q12 is not the exact current observation q12")
    feedback = {key: observation.get(key) for key in ("feedback_stamp", "seq")}
    if any(feedback[key] is None for key in feedback):
        _reject("identity", "current observation has no complete feedback identity")
    if geometry.get("feedback_identity") != feedback:
        _reject("identity", "geometry feedback identity differs from the current observation")
    if geometry.get("camera") != _camera_identity(observation):
        _reject("identity", "geometry right wrist frame differs from the current observation")
    from right_wrist_relative_geometry import transform
    transform(geometry.get("object_pose_model"), "geometry.object_pose_model")
    transform(geometry.get("object_pose_in_receiver_tool"),
              "geometry.object_pose_in_receiver_tool")
    return q12, feedback


def _owner_context(owner, observation, geometry, q12, feedback):
    mode = getattr(owner, "evaluation_mode", None)
    source = MODE_SOURCE.get(mode)
    if source is None:
        _reject("source", f"unsupported evaluation mode {mode!r}")
    if mode == 'LIVE_REAL' and (getattr(owner, 'model_assisted_live_enabled', False) is not True or
                              getattr(owner, 'scope', None) != 'contact' or
                              getattr(owner, 'physical_execution_allowed', False) is not True):
        _reject('owner', 'LIVE model-assisted preview requires the approved contact-only contract')
    if observation.get("source_kind") != source or geometry.get("source_kind") != source:
        _reject("source", f"observation and geometry must both be {source} in {mode}")
    missing = [name for name in REQUIRED_OWNER_ATTRIBUTES if not hasattr(owner, name)]
    if missing:
        _reject("owner", f"owner missing required attributes: {missing}")
    if observation.get("fresh") is not True or observation.get("snapshot_only") is True:
        _reject("identity", "observation is not a fresh current feedback packet")
    if owner.block_owner != "left":
        _reject("owner", "model-assisted receive requires the existing left block owner")

    plan, state = owner.approach_plan, owner.approach_state
    if not isinstance(plan, dict) or plan.get("_model") is None or plan.get("_data") is None:
        _reject("owner", "existing A approach plan has no private model/data")
    if not isinstance(state, dict) or not isinstance(state.get("scene"), dict):
        _reject("owner", "existing A approach state has no scene contract")
    if plan.get("_teacher") is not None and plan["_teacher"] is not owner.teacher:
        _reject("owner", "owner teacher is not the A approach-plan teacher")
    hold = _finite_vector(state.get("left_hold_q"), 6, "approach_state.left_hold_q")

    raw = owner.last_raw
    if not isinstance(raw, dict) or not isinstance(raw.get("ticks"), dict):
        _reject("identity", "owner has no current stored motor ticks")
    if raw.get("t_bus_monotonic") != feedback["feedback_stamp"] or raw.get("bus_seq") != feedback["seq"]:
        _reject("identity", "owner stored bus identity differs from the observation")
    if observation.get("present_ticks") != raw["ticks"]:
        _reject("identity", "observation Present_Position differs from the owner stored read")
    mapped = np.asarray(owner.p1.rc.ticks_to_q12(owner.mp, raw["ticks"]), dtype=float)
    if mapped.shape != (12,) or not np.isfinite(mapped).all() or not np.array_equal(mapped, q12):
        _reject("identity", "observation q12 is not the exact mapping of the owner stored read")
    if any(name not in raw["ticks"].get(side, {}) for side in ("left", "right") for name in ARM):
        _reject("identity", "owner stored read lacks an ARM5 Present_Position")
    if not callable(owner._check) or not callable(owner.gear):
        _reject("owner", "owner checker and gear conversion must be callable")
    return plan["_model"], plan["_data"], owner.teacher, hold, source


def initialize_model_assisted_receive(owner):
    """Return the stateless planner callback expected by the Owner integration."""
    missing = [name for name in REQUIRED_OWNER_ATTRIBUTES if not hasattr(owner, name)]
    if missing:
        raise ValueError(f"owner missing required attributes: {missing}")
    if owner.evaluation_mode not in MODE_SOURCE:
        raise ValueError(f"unsupported evaluation mode {owner.evaluation_mode!r}")
    return partial(plan_model_assisted_receive, owner)


def _a_runtime():
    from sweepick_261008_receive_plan import runtime
    return runtime()


def apply_model_assisted_scene(state, q12, geometry, feedback_stamp):
    """Apply an estimated pose to private numeric planning data without changing evidence roles."""
    q12 = _finite_vector(q12, 12, "model-assisted scene q12")
    if geometry.get("classification") != "MODEL_ASSISTED_RELATIVE":
        _reject("scene", "numeric scene input is not MODEL_ASSISTED_RELATIVE")
    if geometry.get("source_kind") not in MODE_SOURCE.values():
        _reject("source", "model-assisted scene source must be REAL or SYNTHETIC")
    if (geometry.get("feedback_identity") or {}).get("feedback_stamp") != feedback_stamp:
        _reject("identity", "model-assisted scene feedback stamp differs from its geometry")
    from right_wrist_relative_geometry import transform
    model_pose = transform(geometry.get("object_pose_model"), "geometry.object_pose_model")
    task_from_model = transform(state["scene"].get("T_task_from_model"),
                                "scene.T_task_from_model")
    model, data, teacher = state["_model"], state["_data"], state["_teacher"]
    mj, _, _, _, set_q12 = _a_runtime()
    set_q12(model, data, q12)
    task_pose = task_from_model @ model_pose
    quat = np.empty(4)
    mj.mju_mat2Quat(quat, np.ascontiguousarray(task_pose[:3, :3]).ravel())
    address = teacher.plan.cm.block_qpos
    data.qpos[address:address + 7] = np.r_[task_pose[:3, 3], quat]
    data.time = float(feedback_stamp)
    mj.mj_forward(model, data)
    return dict(
        model_pose_applied=True, planning_pose_role="MODEL_ASSISTED_RELATIVE",
        actual_held_pose_available=False, live_pose_source_permitted=False,
        source_kind=geometry.get("source_kind"), feedback_identity=dict(geometry["feedback_identity"]),
    )


def _replan_rejection(stage, cause):
    return dict(
        status="REJECTED", classification="MODEL_ASSISTED_RECEIVE_PLAN",
        stage="BETWEEN_HOPS", model_path_ready=False,
        live_execution_permitted=False, motor_commands_sent=0,
        first_hop=None, remaining_hops=[], forward_checks=[], reverse_checks=[],
        reason=cause, rejection=dict(stage=stage, cause=cause),
    )


def _newer_scalar(value, previous, name):
    if (isinstance(value, bool) or isinstance(previous, bool) or
            not isinstance(value, (int, float)) or not isinstance(previous, (int, float)) or
            not np.isfinite(value) or not np.isfinite(previous) or value <= previous):
        _reject("identity", f"{name} must be finite and newer than the previous plan")


def replan_after_hop(owner, previous_plan, observation, geometry, completed_monotonic_s):
    """Require new camera/motor evidence, then re-plan from its actual q12.

    Reaching the old target is not inferred from a tick-error gate.  The fresh
    q12 is the start of the remaining path and its complete normal reverse.
    """
    try:
        if not isinstance(previous_plan, dict):
            _reject("previous_plan", "previous model-assisted plan is missing")
        first = previous_plan.get("first_hop")
        checks = previous_plan.get("forward_checks")
        if (previous_plan.get("status") != "CANDIDATE" or
                previous_plan.get("model_path_ready") is not True or
                previous_plan.get("live_execution_permitted") is not False or
                not isinstance(first, dict) or first.get("model_check_confirmed") is not True or
                not isinstance(checks, list) or not checks):
            _reject("previous_plan", "previous plan has no confirmed checked first_hop")
        checked = checks[0]
        native = checked.get("native_validation") or {}
        if (checked.get("label") != first.get("label") or
                checked.get("target_q12") != first.get("target_q12") or
                (checked.get("certificate") or {}).get("nominal_verdict") != "CLEAR" or
                native.get("executable") is not True or native.get("hard") or
                native.get("provisional") or checked.get("refusal") is not None):
            _reject("previous_plan", "previous first_hop is not bound to a complete CLEAR/native check")
        if (isinstance(completed_monotonic_s, bool) or
                not isinstance(completed_monotonic_s, (int, float)) or
                not np.isfinite(completed_monotonic_s)):
            _reject("identity", "first_hop completion monotonic time must be finite")

        previous_camera = previous_plan.get("camera_identity") or {}
        current_camera = _camera_identity(observation or {})
        _newer_scalar(current_camera.get("n"), previous_camera.get("n"),
                      "right wrist frame number")
        _newer_scalar(current_camera.get("recv_monotonic_s"), completed_monotonic_s,
                      "right wrist receive time after first_hop completion")
        previous_feedback = previous_plan.get("feedback_identity") or {}
        _newer_scalar((observation or {}).get("seq"), previous_feedback.get("seq"),
                      "motor feedback sequence")
        _newer_scalar((observation or {}).get("feedback_stamp"),
                      previous_feedback.get("feedback_stamp"), "motor feedback stamp")
        _newer_scalar((observation or {}).get("feedback_stamp"), completed_monotonic_s,
                      "motor feedback stamp after first_hop completion")
    except _Rejected as exc:
        return _replan_rejection(exc.stage, str(exc))

    next_plan = plan_model_assisted_receive(
        owner, observation, geometry, stage="BETWEEN_HOPS")
    if not isinstance(next_plan, dict):
        return _replan_rejection("planner", "BETWEEN_HOPS planner returned no result packet")
    if (next_plan.get("live_execution_permitted") is not False or
            next_plan.get("motor_commands_sent") != 0):
        return _replan_rejection(
            "source", "BETWEEN_HOPS planner did not preserve the write-zero evaluation contract")
    result = dict(next_plan)
    result["replanned_after"] = dict(
        previous_first_hop_label=first["label"],
        previous_feedback_identity=dict(previous_feedback),
        previous_camera_identity=dict(previous_camera),
        completed_monotonic_s=float(completed_monotonic_s),
        new_feedback_identity={key: observation.get(key) for key in ("feedback_stamp", "seq")},
        new_camera_identity=current_camera,
        actual_q12_is_new_path_start=True,
        target_tick_match_gate=None,
    )
    return result


def _teacher_snapshot(teacher):
    saved = {}
    for name, value in teacher.__dict__.items():
        if isinstance(value, np.ndarray):
            saved[name] = value.copy()
        elif isinstance(value, (dict, list, set)):
            saved[name] = copy.deepcopy(value)
        else:
            saved[name] = value
    return saved


def _restore_teacher(teacher, saved):
    teacher.__dict__.clear()
    teacher.__dict__.update(saved)


def _planner_snapshot(teacher):
    planner = teacher.plan
    return dict(
        qpos=planner.pd.qpos.copy(), qvel=planner.pd.qvel.copy(),
        time=float(planner.pd.time), roll_sign=copy.deepcopy(planner._roll_sign),
        last_roll_choice=copy.deepcopy(planner._last_roll_choice),
    )


def _restore_planner(model, teacher, saved):
    import mujoco
    planner = teacher.plan
    planner.pd.qpos[:] = saved["qpos"]
    planner.pd.qvel[:] = saved["qvel"]
    planner.pd.time = saved["time"]
    planner._roll_sign = saved["roll_sign"]
    planner._last_roll_choice = saved["last_roll_choice"]
    mujoco.mj_forward(model, planner.pd)


def _path_row(owner, tt, complete, profile, simulated, label, target_q12, mode):
    target_q12 = _finite_vector(target_q12, 12, f"{label}.target_q12")
    hold = np.asarray(owner.approach_state["left_hold_q"], dtype=float)
    current_q12 = np.asarray(owner.p1.rc.ticks_to_q12(owner.mp, simulated), dtype=float)
    if (not np.array_equal(target_q12[:5], current_q12[:5]) or
            not np.isclose(target_q12[5], hold[5], rtol=0, atol=0) or
            not np.isclose(target_q12[11], current_q12[11], rtol=0, atol=0)):
        _reject("planner", f"{label}: target changed a left arm joint or either Jaw command")
    target_all = owner.p1.rc.q12_to_ticks(owner.mp, target_q12)["right"]
    target = {name: float(round(target_all[name])) for name in ARM}
    trajectory = tt.make(simulated["right"], target, profile)
    for values in trajectory["goals"].values():
        if not np.isfinite(np.asarray(values, dtype=float)).all():
            _reject("trajectory", f"{label}: existing trajectory builder returned non-finite samples")
    doc = dict(side="right", level="model", mapping_sha256=owner.msha,
               trajectory=trajectory, purpose="RIGHT_INSERT_REPLAN")
    raw = dict(owner.last_raw, ticks=copy.deepcopy(simulated))
    certificate = complete(owner._check(doc, "right", raw), doc)
    validation = tt.validate(
        "right", owner.mp["right"], trajectory, profile, mapping_sha=owner.msha,
        level="model", cert=certificate, other_arm_ticks=simulated["left"])
    row = dict(
        label=label, mode=mode, side="right", target_q12=target_q12.tolist(),
        target_ticks=target, trajectory=trajectory, certificate=certificate,
        native_validation=validation, refusal=None,
    )
    if certificate.get("nominal_verdict") != "CLEAR":
        detail = certificate.get("incomplete") or certificate.get("first_nominal_contact")
        row["refusal"] = dict(
            stage="collision",
            cause=f"{label}: path check is {certificate.get('nominal_verdict')} ({detail})")
    refused = validation.get("hard", []) + validation.get("provisional", [])
    if row["refusal"] is None and (not validation.get("executable") or refused):
        first = refused[0] if refused else {"gate": "native_validation", "detail": "not executable"}
        row["refusal"] = dict(
            stage="trajectory",
            cause=f"{label}: {first.get('gate')}: {first.get('detail')}")
    clearance = certificate.get("min_nominal_object_clearance_m")
    if (row["refusal"] is None and
            (isinstance(clearance, bool) or not isinstance(clearance, (int, float)) or
             not np.isfinite(clearance) or clearance <= 0)):
        row["refusal"] = dict(
            stage="collision", cause=f"{label}: no positive finite nominal object clearance")
    if row["refusal"] is None:
        simulated["right"].update(target)
    return row


def plan_model_assisted_receive(owner, observation, geometry, stage="AFTER_PREGRASP"):
    """Return an A-planner candidate with a checked first hop and normal reverse.

    ``remaining_hops`` are a preview only.  A fresh current observation must call
    this function again with ``stage='BETWEEN_HOPS'`` after the first hop.
    """
    result = dict(
        status="REJECTED", classification="MODEL_ASSISTED_RECEIVE_PLAN",
        stage=stage, model_path_ready=False, live_execution_permitted=False,
        motor_commands_sent=0, first_hop=None, remaining_hops=[],
        forward_checks=[], reverse_checks=[],
    )
    try:
        if stage not in ("AFTER_PREGRASP", "BETWEEN_HOPS"):
            _reject("stage", f"unsupported model-assisted receive stage {stage!r}")
        q12, feedback = _validate_geometry(observation or {}, geometry)
        model, data, teacher, hold, source = _owner_context(
            owner, observation, geometry, q12, feedback)
    except _Rejected as exc:
        result.update(reason=str(exc), rejection=dict(stage=exc.stage, cause=str(exc)))
        return result
    except (ValueError, KeyError, TypeError) as exc:
        result.update(reason=str(exc), rejection=dict(stage="geometry", cause=str(exc)))
        return result

    saved_teacher = _teacher_snapshot(teacher)
    saved_planner = _planner_snapshot(teacher)
    saved_qpos, saved_time = data.qpos.copy(), float(data.time)
    saved_pose, saved_near = owner.last_held_pose, owner.near_handover
    try:
        from sweepick_261008_receive_plan import plan_insert_hops
        from sweepick_full_chain import complete
        import sweepick_trajectory as tt

        state = dict(owner.approach_state, _model=model, _data=data, _teacher=teacher)
        applied = apply_model_assisted_scene(
            state, q12, geometry, feedback["feedback_stamp"])
        result["scene_application"] = applied
        if (not applied.get("model_pose_applied") or
                applied.get("planning_pose_role") != "MODEL_ASSISTED_RELATIVE" or
                applied.get("actual_held_pose_available") is not False or
                applied.get("live_pose_source_permitted") is not False):
            _reject("scene", "private A scene did not preserve the model-assisted evidence role")

        seed = q12.copy()
        seed[5] = hold[5]
        teacher.cmd = owner.gear(seed)
        teacher.queue = []
        task_from_model = np.asarray(state["scene"]["T_task_from_model"], dtype=float)
        block_qpos = data.qpos[teacher.cm.block_qpos:teacher.cm.block_qpos + 7].copy()
        teacher.plan.pose(data.qpos, teacher.cmd, block_qpos)
        tool_position, tool_rotation = teacher.plan.site_frame("right")
        task_from_tool = np.eye(4)
        task_from_tool[:3, 3], task_from_tool[:3, :3] = tool_position, tool_rotation
        model_from_tool = np.linalg.inv(task_from_model) @ task_from_tool
        expected_relative = np.linalg.inv(model_from_tool) @ np.asarray(
            geometry["object_pose_model"], dtype=float)
        if not np.allclose(expected_relative,
                           np.asarray(geometry["object_pose_in_receiver_tool"], dtype=float)):
            _reject("identity", "geometry receiver-tool relative pose differs from exact actual-q A FK")
        teacher._plan_right_grasp(data)
        approach, queued = teacher.segment, list(teacher.queue)
        if not isinstance(approach, dict) or len(queued) != 1 or queued[0][0] != "right":
            _reject("planner", "existing A right-grasp planner returned no unique right insertion")
        pre_q = np.asarray(teacher.to_canonical(np.asarray(approach["goal"], dtype=float)), dtype=float)
        grasp_q = np.asarray(teacher.to_canonical(np.asarray(queued[0][1], dtype=float)), dtype=float)
        pre_q[5], grasp_q[5] = hold[5], hold[5]
        pre_q[11], grasp_q[11] = q12[11], q12[11]

        start = pre_q if stage == "AFTER_PREGRASP" else q12
        hops = [np.asarray(value, dtype=float) for value in
                plan_insert_hops(teacher, data, start, grasp_q, hold)]
        targets = ([dict(label="RIGHT_PREGRASP_CORRECTION", q12=pre_q)]
                   if stage == "AFTER_PREGRASP" else [])
        targets += [dict(label=f"RIGHT_INSERT_HOP_{index + 1:03d}", q12=target)
                    for index, target in enumerate(hops)]
        if not targets:
            _reject("planner", "existing A planner returned no model-assisted receive hop")
        for target in targets:
            value = _finite_vector(target["q12"], 12, target["label"])
            if (not np.array_equal(value[:5], q12[:5]) or
                    not np.isclose(value[5], hold[5], rtol=0, atol=0) or
                    not np.isclose(value[11], q12[11], rtol=0, atol=0)):
                expected = q12.copy(); expected[5] = hold[5]
                planned_ticks = owner.p1.rc.q12_to_ticks(owner.mp, value)
                expected_ticks = owner.p1.rc.q12_to_ticks(owner.mp, expected)
                result['stationary_component_changes'] = [dict(
                    side='left' if index < 6 else 'right', joint=joint,
                    planned_q=float(value[index]), expected_q=float(expected[index]),
                    planned_tick=float(planned_ticks['left' if index < 6 else 'right'][joint]),
                    expected_tick=float(expected_ticks['left' if index < 6 else 'right'][joint]))
                    for index, joint in [(i, name) for i, name in enumerate((*ARM, 'gripper'))] + [(11, 'gripper')]
                    if value[index] != expected[index]]
                _reject("planner", f"{target['label']}: A planner changed left ARM5 or a Jaw command")

        profile = owner.run_kw.get("profile") or tt.load_profile()
        simulated = copy.deepcopy(owner.last_raw["ticks"])
        reverse_targets = []
        owner.last_held_pose = copy.deepcopy(geometry["object_pose_model"])
        owner.near_handover = False
        for target in targets:
            reverse_targets.append(copy.deepcopy(simulated["right"]))
            row = _path_row(owner, tt, complete, profile, simulated,
                            target["label"], target["q12"], "forward")
            result["forward_checks"].append(row)
            if row["refusal"] is not None:
                _reject(row["refusal"]["stage"], row["refusal"]["cause"])
        for index, (target, ticks) in enumerate(reversed(list(zip(targets, reverse_targets)))):
            reverse_q = np.asarray(owner.p1.rc.ticks_to_q12(
                owner.mp, dict(simulated, right=ticks)), dtype=float)
            reverse_q[5], reverse_q[11] = hold[5], q12[11]
            row = _path_row(owner, tt, complete, profile, simulated,
                            f"NORMAL_REVERSE_{target['label']}", reverse_q, "normal reverse")
            result["reverse_checks"].append(row)
            if row["refusal"] is not None:
                _reject(row["refusal"]["stage"], row["refusal"]["cause"])

        first = targets[0]
        result.update(
            status="CANDIDATE", model_path_ready=True, reason=None,
            source_kind=source, feedback_identity=feedback,
            camera_identity=dict(geometry["camera"]), actual_q12=q12.tolist(),
            correction_q12=pre_q.tolist() if stage == "AFTER_PREGRASP" else None,
            grasp_q12=grasp_q.tolist(), insert_hops_q12=[value.tolist() for value in hops],
            first_hop=dict(label=first["label"], target_q12=np.asarray(first["q12"]).tolist(),
                           target_ticks=result["forward_checks"][0]["target_ticks"],
                           model_check_confirmed=True,
                           authorized_for_live_execution=False),
            remaining_hops=[dict(label=target["label"], target_q12=np.asarray(target["q12"]).tolist(),
                                  authorized_for_live_execution=False) for target in targets[1:]],
            replan_contract=dict(
                after_first_hop="acquire a new current right-wrist frame and exact motor feedback",
                next_stage="BETWEEN_HOPS", remaining_hops_are_preview_only=True),
            planner="existing A HandoverTeacher._plan_right_grasp + plan_insert_hops",
            planner_options=getattr(owner, "planner_options", None),
            scene_application=applied,
        )
    except _Rejected as exc:
        result.update(reason=str(exc), rejection=dict(stage=exc.stage, cause=str(exc)))
    except Exception as exc:
        # TeacherFailure/IK/collision text is part of the result; never replace it with CLEAR.
        result.update(reason=f"{type(exc).__name__}: {exc}",
                      rejection=dict(stage="A_planner_or_checker",
                                     cause=f"{type(exc).__name__}: {exc}"))
    finally:
        owner.last_held_pose, owner.near_handover = saved_pose, saved_near
        data.qpos[:] = saved_qpos
        data.time = saved_time
        try:
            import mujoco
            mujoco.mj_forward(model, data)
        finally:
            try:
                _restore_planner(model, teacher, saved_planner)
            finally:
                _restore_teacher(teacher, saved_teacher)
    return result
