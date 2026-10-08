"""Connect the existing teacher trajectories to one already-owned local execution loop."""
import numpy as np

from sweepick.manipulation.sweepick_right_receive_planner import gear, runtime
from sweepick.integration.sweepick_handoff_snapshot_adapter import q_values, transform


def owner_observation(raw, mapping, *, executor_pid, telemetry, evidence=None):
    """Adapt an already-owned local read; this function performs no I/O or feedback measurement."""
    from sweepick.integration.sweepick_receive_runtime import field_module
    rc = field_module("real_command")
    raw, telemetry = raw or {}, telemetry or {}
    out = dict(evidence or {})
    stamp, seq = raw.get("t_bus_monotonic"), raw.get("bus_seq")
    out.update(executor_pid=executor_pid, feedback_stamp=stamp, seq=seq,
               cycle_start=raw.get("cycle_start"), present_ticks=raw.get("ticks"),
               fresh=raw.get("usable") is True and raw.get("cycle_start") is not None,
               snapshot_only=raw.get("snapshot_only", False), q12=None)
    try:
        measured = rc.ticks_to_q12(mapping, raw["ticks"])
        out["q12"] = q_values(measured, "local measured q12", 12).tolist()
    except (KeyError, TypeError, ValueError):
        out["fresh"] = False
    for i, side in enumerate(("left", "right")):
        packet = dict(telemetry.get(side) or {})
        packet["actual"] = None if out["q12"] is None else out["q12"][i * 6 + 5]
        goal = (packet.get("goal_ticks") or {}).get("gripper")
        packet["command"] = None
        if goal is not None:
            try:
                command = rc.tick_to_model("gripper", mapping[side]["gripper"], goal)
                if np.isfinite(command) and 0 <= command <= 1:
                    packet["command"] = float(command)
            except (KeyError, TypeError, ValueError):
                pass
        # Keep telemetry's own time/provenance; a stale Goal read cannot become current by repackaging it.
        out[side] = packet
    return out


class ReceiveCorrector:
    """Sample existing motion profiles; measured executor results advance each waypoint."""

    def __init__(self, plan, *, release_controller=None, load_transfer_controller=None,
                 insertion_alignment=None):
        if plan.get("status") != "PLAN_PREVIEW":
            raise ValueError("a RECEIVE plan preview is required")
        self.plan, self.teacher = plan, plan["_teacher"]
        self.release_controller = release_controller
        self.load_transfer_controller = load_transfer_controller
        self.insertion_alignment = insertion_alignment
        self.failure = None
        self.phase, self.index, self.elapsed = None, 0, 0.0
        self.trajectory = None
        self.session_id = self.owner_pid = None

    def begin(self, phase):
        self.phase = phase
        if phase == "COORDINATE":
            self.index, self.elapsed, self.trajectory = 0, 0.0, None
        elif phase == "RELEASE" and self.release_controller is not None:
            self.release_controller.begin(phase)
        elif phase == "LOAD_TRANSFER" and self.load_transfer_controller is not None:
            self.load_transfer_controller.begin(phase)

    def _actual_result(self, observation, waypoint):
        result = observation.get("motion_result")
        if not isinstance(result, dict):
            return False
        try:
            target = q_values(result.get("q12"), "motion_result.q12", 12)
        except ValueError:
            return False
        return bool(target is not None and np.array_equal(target, waypoint["q12"])
                    and result.get("label") == waypoint["label"]
                    and result.get("result") == "HOLDING"
                    and result.get("independent") is True
                    and result.get("source_kind") == "REAL"
                    and result.get("provenance") == "independent register / sensor"
                    and result.get("source")
                    and result.get("session_id") == self.session_id
                    and result.get("executor_pid") == self.owner_pid
                    and result.get("feedback_stamp") == observation.get("feedback_stamp")
                    and result.get("seq") == observation.get("seq"))

    def next_command(self, context):
        observation = context["observation"]
        measured = context["measured_q12"]
        if self.phase == "LOAD_TRANSFER":
            return self.load_transfer_controller.next_command(context)
        if self.phase == "RELEASE":
            if self.release_controller is None:
                held = list(measured)
                held[5] = self.plan["left_hold_q"][5]
                return held, dict(stage="RELEASE", status="NOT_READY",
                                  missing=["existing real release/retreat controller"])
            return self.release_controller.next_command(context)
        waypoint = self.plan["waypoints"][self.index]
        if self._actual_result(observation, waypoint) and self.index < len(self.plan["waypoints"]) - 1:
            self.index += 1
            self.elapsed, self.trajectory = 0.0, None
            waypoint = self.plan["waypoints"][self.index]
        if waypoint["label"] == "RIGHT_INSERT" and (
                not callable(self.insertion_alignment) or self.insertion_alignment(observation) is not True):
            return None, dict(status="NOT_READY", waypoint_label="RIGHT_INSERT",
                              missing=["current visual insertion alignment / uncertainty / geometry clearance"])
        if self.trajectory is None:
            self.trajectory = self.plan["_trajectories"][self.index]
            if self.trajectory is None:
                self.teacher.cmd = gear(self.teacher, measured)
                self.teacher.start_segment(gear(self.teacher, waypoint["q12"]), ["left", "right"])
                self.trajectory = self.teacher.segment["traj"]
        self.elapsed += float(context["control_dt"])
        target, *_ = self.trajectory.sample(self.elapsed)
        command = self.teacher.to_canonical(target).tolist()
        command[5], command[11] = waypoint["q12"][5], waypoint["q12"][11]
        complete = self.elapsed >= self.trajectory.duration_s
        return command, dict(waypoint_label=waypoint["label"], trajectory_complete=complete,
                             awaiting_actual_result=True,
                             right_insert_ready=waypoint["label"] == "RIGHT_INSERT" and self._actual_result(observation, waypoint))


def apply_actual_observation(state, q12, observation):
    """Update private planning data from live facts; never replace a missing pose by the planned one."""
    mj, _, _, _, set_q12 = runtime()
    model, data = state["_model"], state["_data"]
    set_q12(model, data, q12)
    held = observation.get("held_object")
    current_pose = (isinstance(held, dict) and held.get("pose_frame") == state["scene"]["frame"]
                    and held.get("pose_role") in ("CURRENT_OBSERVATION", "CURRENT_TRACKED_ESTIMATE")
                    and held.get("source_kind") == "REAL"
                    and held.get("feedback_stamp") == observation["feedback_stamp"]
                    and held.get("seq") == observation["seq"] and bool(held.get("pose_source"))
                    and held.get("object_pose_uncertainty") is not None)
    pose = held.get("object_pose_model") if current_pose else None
    if pose is not None:
        pose = transform(pose, "held_object.object_pose_model")
        task_pose = np.asarray(state["scene"]["T_task_from_model"]) @ pose
        address = state["_teacher"].plan.cm.block_qpos
        quat = np.empty(4)
        mj.mju_mat2Quat(quat, np.ascontiguousarray(task_pose[:3, :3]).ravel())
        data.qpos[address:address + 7] = np.r_[task_pose[:3, 3], quat]
    state["actual_held_pose_available"] = pose is not None
    data.time = float(observation["feedback_stamp"])
    mj.mj_forward(model, data)


def make_session(state, plan, *, ports, owner_pid, rig, applier, control_dt,
                 release_controller=None, primary=None, camera_current=None,
                 right_jaw_profile=None, right_jaw=None, left_jaw=None,
                 support_confirmation=None, port_owner_pids=None,
                 load_transfer_controller=None, session_id=None, insertion_alignment=None):
    runtime()
    from sweepick.integration.sweepick_receive_session import ReceiveSession
    corrector = ReceiveCorrector(plan, release_controller=release_controller,
                                 load_transfer_controller=load_transfer_controller,
                                 insertion_alignment=insertion_alignment)
    context = dict(state, **{k: plan[k] for k in ("_model", "_data", "_teacher")})
    context["right_insert_q12"] = plan["right_insert_q12"]
    session = ReceiveSession(
        context, ports=ports, owner_pid=owner_pid, port_owner_pids=port_owner_pids,
        rig=rig, applier=applier, corrector=corrector, primary=primary,
        control_dt=control_dt, camera_current=camera_current,
        apply_observation=apply_actual_observation, right_jaw_profile=right_jaw_profile,
        right_jaw=right_jaw, left_jaw=left_jaw, support_confirmation=support_confirmation,
        session_id=session_id)
    corrector.session_id, corrector.owner_pid = session.session_id, owner_pid
    return session


def prepare_receive(snapshot, bindings, *, session_kwargs, observation_source, downstream,
                    real_stow_profile=None, apply_dispatch=None, on_stop=None, record_event=None):
    """Called locally after lift; saved snapshot plans geometry, fresh feedback authorizes cycles."""
    from sweepick.integration.sweepick_handoff_snapshot_adapter import adapt_snapshot
    from sweepick.manipulation.sweepick_right_receive_planner import plan_receive
    from sweepick.integration.sweepick_receive_session import ReceiveManipulation
    state = adapt_snapshot(snapshot, bindings)
    readiness = receive_references_ready(session_kwargs, observation_source=observation_source,
                                         downstream=downstream, real_stow_profile=real_stow_profile,
                                         apply_dispatch=apply_dispatch, on_stop=on_stop, record_event=record_event)
    if not readiness["ready"]:
        return dict(status="NOT_READY", missing=state["missing"] + readiness["missing"],
                    invalid=state["invalid"], real_execution_ready=False, motor_commands_sent=0)
    plan = plan_receive(state)
    if plan["status"] != "PLAN_PREVIEW":
        return dict(status="NOT_READY", missing=state["missing"], invalid=state["invalid"],
                    plan=plan, real_execution_ready=False, motor_commands_sent=0)
    session = make_session(state, plan, **session_kwargs)
    return ReceiveManipulation(session, observation_source, downstream,
                               real_stow_profile=real_stow_profile,
                               apply_dispatch=apply_dispatch, on_stop=on_stop,
                               record_event=record_event)


def receive_references_ready(session_kwargs, *, observation_source, downstream, real_stow_profile,
                             apply_dispatch, on_stop, record_event):
    """Reference completeness for the existing pre-lift ready() hook, not physical readiness."""
    import os
    from sweepick.integration.sweepick_receive_session import verified_right_profile, tg
    refs = dict(observation_source=observation_source, apply_dispatch=apply_dispatch,
                on_stop=on_stop, record_event=record_event,
                applier_dispatch=getattr(session_kwargs.get("applier"), "dispatch", None),
                support_confirmation=session_kwargs.get("support_confirmation"),
                PLACE=getattr(downstream, "place", None), STOW=getattr(downstream, "stow", None),
                set_receive_context=getattr(downstream, "set_receive_context", None))
    missing = [name for name, value in refs.items() if not callable(value)]
    for name in ("release_controller", "load_transfer_controller"):
        reference = session_kwargs.get(name)
        for method in ("begin", "next_command"):
            if not callable(getattr(reference, method, None)):
                missing.append(name + "." + method)
    if not verified_right_profile(session_kwargs.get("right_jaw_profile")):
        missing.append("right REAL Jaw profile")
    if not isinstance(session_kwargs.get("left_jaw"), tg.Jaw):
        missing.append("same-owner left Jaw")
    right_jaw = session_kwargs.get("right_jaw")
    profile = session_kwargs.get("right_jaw_profile") or {}
    if right_jaw is not None and (not isinstance(right_jaw, tg.Jaw)
                                  or getattr(right_jaw, "provenance", None) != tg.INDEPENDENT
                                  or getattr(right_jaw, "dev", None) != profile.get("device")):
        missing.append("right Jaw identity/provenance matches measured profile")
    for name in ("rig", "control_dt", "session_id"):
        if session_kwargs.get(name) is None:
            missing.append(name)
    if real_stow_profile is None:
        missing.append("real_stow_profile")
    ports, owners = session_kwargs.get("ports"), session_kwargs.get("port_owner_pids")
    if (not isinstance(ports, (tuple, list)) or len(ports) != 2 or len(set(ports)) != 2
            or not isinstance(owners, dict) or set(owners) != set(ports)
            or set(owners.values()) != {os.getpid()} or session_kwargs.get("owner_pid") != os.getpid()):
        missing.append("existing same-PID dual-port ownership")
    return dict(ready=not missing, missing=missing, structural_only=True,
                real_execution_ready=False, motor_commands_sent=0)
