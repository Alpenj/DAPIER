"""Concrete callbacks for captured executor APIs and the existing per-cycle bridge.

The blocking references and ReceiveSession.cycle are mutually exclusive writers.
No bus is constructed/opened here; the local owner supplies every I/O reference.
"""
from __future__ import annotations

import inspect
import os
from pathlib import Path

import numpy as np

from sweepick.integration.sweepick_handoff_snapshot_adapter import transform, q_values


def not_ready(*missing):
    return dict(ok=False, status="NOT_READY", why="; ".join(missing),
                missing=list(missing), commanded=False, real_execution_ready=False)


def classify_held_estimate(estimate, *, frame, stamp, seq):
    """Preserve the existing estimator's values; classify its assumptions, not new poses."""
    out = dict(estimate)
    pose = transform(out.get("object_pose_model"), "estimator.object_pose_model")
    out.update(object_pose_model=None if pose is None else pose.tolist(),
               pose_role="COARSE_PREVIEW", pose_frame=frame,
               feedback_stamp=stamp, seq=seq, source_kind="ESTIMATED",
               observed_components=["measured joints at close and now", "desk observation before close"],
               estimated_components=["tool motion by candidate FK", "pad midpoint along jaw axis"],
               assumed_components=["upright desk object", "nearest face squared by pads", "no slip after close"],
               unidentified_components=["current transverse offset", "pad compliance", "slip", "current orientation error"],
               final_insertion_ready=False, real_execution_ready=False)
    if pose is None or not out.get("object_pose_source") or out.get("object_pose_uncertainty") is None or not frame:
        out.update(status="MISSING", missing=["labelled estimator pose/source/frame/uncertainty"])
    else:
        out.update(status="CANDIDATE_ESTIMATE", missing=[])
    return out


def insertion_alignment_ready(record, observation, session_id):
    """Use the existing visual alignment verdict, never invent a clearance or threshold."""
    return bool(isinstance(record, dict) and record.get("ready") is True
                and record.get("purpose") == "RIGHT_INSERT"
                and record.get("source_kind") == "REAL" and record.get("independent") is True
                and record.get("source") and record.get("frame")
                and record.get("uncertainty") is not None and record.get("clearance_source")
                and record.get("session_id") == session_id
                and record.get("executor_pid") == os.getpid()
                and record.get("feedback_stamp") == observation.get("feedback_stamp")
                and record.get("seq") == observation.get("seq"))


def _current_registers(observation):
    """Only the owner's independently read register packets, never its write return."""
    if (observation.get("source_kind") != "REAL" or observation.get("fresh") is not True
            or observation.get("snapshot_only") is True or observation.get("executor_pid") != os.getpid()):
        return None
    packets = observation.get("motor_feedback") or {
        side: (observation.get(side) or {}).get("registers") for side in ("left", "right")}
    for side in ("left", "right"):
        p = packets.get(side) or {}
        if (p.get("source_kind") != "REAL" or p.get("provenance") != "independent register / sensor"
                or p.get("fresh") is not True
                or not p.get("source") or p.get("feedback_stamp") != observation.get("feedback_stamp")
                or p.get("seq") != observation.get("seq")):
            return None
    return packets


def observed_owner_ack(write, observation, *, mapping, session_id):
    """Confirm the last SENT ticks from fresh Goal_Position readback; Present is separate."""
    from sweepick.integration.sweepick_receive_runtime import field_module
    rc = field_module("real_command")
    registers = _current_registers(observation)
    if registers is None or not isinstance(write, dict):
        return None
    goals = {}
    for side in ("left", "right"):
        goals[side] = registers[side].get("goal") or {}
        for name in rc.JOINTS:
            actual, sent = goals[side].get(name), (write.get(side) or {}).get(name)
            if actual is None or sent is None or actual != sent:
                return None
    try:
        q = q_values(rc.ticks_to_q12(mapping, goals), "Goal_Position readback q12", 12).tolist()
    except (TypeError, ValueError, KeyError):
        return None
    return dict(accepted=True, independent=True, q12=q, source_kind="REAL",
                source="same-cycle independent Goal_Position readback matched the previous writer send",
                provenance="independent register / sensor", session_id=session_id, executor_pid=os.getpid(),
                feedback_stamp=observation["feedback_stamp"], seq=observation["seq"])


def observed_motion_result(expected, observation, *, mapping, progress, session_id):
    """Use existing PG.reached and actual stopped/held registers; no new completion threshold."""
    from sweepick.integration.sweepick_receive_runtime import field_module
    rc, pg = field_module("real_command"), field_module("progress")
    registers = _current_registers(observation)
    if registers is None or not isinstance(expected, dict):
        return None
    q = q_values(expected.get("q12"), "motion target q12", 12)
    if q is None or not expected.get("label"):
        return None
    targets = rc.q12_to_ticks(mapping, q)
    for side in ("left", "right"):
        packet = registers[side]
        for name in rc.JOINTS[:5]:
            values = {k: (packet.get(k) or {}).get(name) for k in ("goal", "present", "torque", "status", "moving", "velocity")}
            if any(v is None or not np.isfinite(v) for v in values.values()):
                return None
            if (values["goal"] != round(targets[side][name]) or values["torque"] != 1
                    or values["status"] != 0 or values["moving"] != 0 or values["velocity"] != 0
                    or not pg.reached(progress, values["present"], targets[side][name])):
                return None
    return dict(label=expected["label"], q12=q.tolist(), measured_q12=observation.get("q12"),
                result="HOLDING", source_kind="REAL", independent=True,
                provenance="independent register / sensor", source="current Goal/Present/torque/status/moving/velocity + existing progress profile",
                progress_profile_sha256=progress.get("sha256"), session_id=session_id, executor_pid=os.getpid(),
                feedback_stamp=observation["feedback_stamp"], seq=observation["seq"])


class ExistingCertificate:
    """File adapter for the captured native certify(traj_file, out_file, scene_file) API."""

    def __init__(self, native_certify, current_scene):
        self.native_certify, self.current_scene = native_certify, current_scene

    def __call__(self, document, ctx, memo):
        scene = self.current_scene(document, ctx, memo)
        if (not isinstance(scene, dict) or not scene.get("path") or not scene.get("source")
                or (scene.get("carried_preview") or {}).get("identity") != document["identity"]
                or scene["carried_preview"].get("nominal_verdict") != "CLEAR"):
            return None
        directory = Path(ctx["out"]) / "sweepick_261008_certificates"
        directory.mkdir(exist_ok=True)
        trajectory, output = (directory / (document["identity"] + suffix) for suffix in ("_trajectory.json", "_certificate.json"))
        import json
        trajectory.write_text(json.dumps(document, default=lambda a: a.tolist()))
        result = self.native_certify(trajectory, output, Path(scene["path"]))
        return dict(result, source=scene["source"], carried_preview=scene["carried_preview"])


class ExistingPlanReferences:
    """Convert existing PLACE/STOW planner output; real target/pose remain owner inputs."""

    def __init__(self, owner, *, place_planner, stow_planner):
        self.owner, self.place_planner, self.stow_planner = owner, place_planner, stow_planner

    def __call__(self, step, ctx, memo, observation):
        o = self.owner
        if step not in ("place", "right_open", "stow"):
            existing = getattr(o, "release_reference", None)
            return existing(step, ctx, memo, observation) if callable(existing) else not_ready(f"existing {step} reference")
        data = o.downstream_inputs(step, observation)
        if not isinstance(data, dict) or data.get("missing"):
            return not_ready(*(data.get("missing", []) if isinstance(data, dict) else ["current downstream inputs"]))
        required = ("me", "gear", "source") + (("placed_xyz", "target_gear", "target_q12", "stow_reference")
                   if step == "stow" else ("centre", "rotation", "vertical_half_extent", "region", "table_z", "right_open_q"))
        missing = [name for name in required if data.get(name) is None]
        if missing:
            return not_ready(*missing)
        if not callable(getattr(o, "to_canonical", None)):
            return not_ready("existing teacher.to_canonical")
        if step == "stow":
            if "target_gear" not in inspect.signature(self.stow_planner).parameters:
                return not_ready("existing plan_stow explicit actual target function merge")
            segments, info = self.stow_planner(
                data["me"], data["gear"], data["placed_xyz"], target_gear=data["target_gear"],
                target_q12=data["target_q12"], reference=data["stow_reference"])
        else:
            if "table_z" not in inspect.signature(self.place_planner).parameters:
                return not_ready("existing plan_place_held actual table height function merge")
            if "place_segments" not in memo:
                segments, info = self.place_planner(
                    data["me"], data["gear"], data["centre"], data["rotation"], data["vertical_half_extent"],
                    data["region"], "current owner held estimate", data["source"], table_z=data["table_z"])
                memo["place_segments"], memo["place_info"] = segments, info
            segments, info = memo["place_segments"], memo["place_info"]
            if segments is not None:
                segments = [s for s in segments if (s[0] in ("carry", "lower")) == (step == "place")]
        if segments is None:
            return not_ready(f"existing {step} IK/collision plan")
        converted, controller_segments = [], []
        for label, start, goal, duration in segments:
            q = q_values(o.to_canonical(goal), "existing planner canonical goal", 12)
            if step == "right_open":
                q[11] = data["right_open_q"]
                if not callable(getattr(o, "to_gear", None)):
                    return not_ready("existing teacher canonical-to-gear conversion")
                goal = o.to_gear(q)
            side = "left" if label.startswith("left_") else "right"
            controller_segments.append((label, start, goal, duration))
            converted.append(dict(label=label, side=side,
                                  target_ticks=o.executor.rc.q12_to_ticks(o.mapping, q)[side]))
        return dict(status="REFERENCE_READY", segments=converted, controller_segments=controller_segments,
                    source=data["source"], planner_info=info)


class ExistingDownstream:
    """Bind the existing Script actor to the same Arbiter/Applier/writer after RECEIVE."""

    def __init__(self, owner, *, script_type):
        self.owner, self.script_type = owner, script_type
        self.bridge = None
        self.context = None
        self.actors = {}

    def set_receive_context(self, context):
        self.context = context

    def _advance(self, stage, now):
        from sweepick.integration.sweepick_receive_session import tm
        from sweepick.manipulation.sweepick_right_receive_dispatch import ReceiveCorrector
        bridge = self.bridge
        if bridge is None:
            return tm.CommandResult(tm.Status.RUNNING, tm.Kind.REAL, "receive bridge not attached", now,
                                    dict(status="NOT_READY", missing=["same owner ReceiveManipulation"]))
        session, o = bridge.session, self.owner
        observation = session.latest
        if stage not in self.actors:
            missing = [name for name in ("reference", "reference_preview", "to_canonical")
                       if not callable(getattr(o, name, None))]
            if self.script_type is None or missing:
                return bridge._result(dict(status="NOT_READY", stage=stage,
                                           missing=missing + ([] if self.script_type else ["existing Script actor"])), now)
            ctx, memo = dict(out=o.out, continuation=self.context), {}
            if stage == "PLACE":
                first = o.reference("place", ctx, memo, observation)
                second = o.reference("right_open", ctx, memo, observation)
                refs = (first, second)
            else:
                refs = (o.reference("stow", ctx, memo, observation),)
            for ref in refs:
                if not isinstance(ref, dict) or ref.get("status") != "REFERENCE_READY":
                    return bridge._result(dict(status="NOT_READY", stage=stage,
                                               missing=(ref.get("missing") if isinstance(ref, dict) else None) or [f"existing {stage} inputs"]), now)
                preview = o.reference_preview(ref, observation, self.context)
                if not isinstance(preview, dict) or preview.get("nominal_verdict") != "CLEAR":
                    return bridge._result(dict(status="NOT_READY", stage=stage,
                                               missing=[f"existing {stage} carried-object collision preview"], preview=preview), now)
            segments = [s for ref in refs for s in ref["controller_segments"]]
            if not segments:
                return bridge._result(dict(status="NOT_READY", stage=stage, missing=[f"existing {stage} motion segments"]), now)
            script = self.script_type(session.state["_model"], segments, f"existing {stage} Script")
            # Arbiter's actor interface expects begin(); Script's own interpolation stays unchanged.
            actor = type("ScriptActor", (), {"failure": None, "begin": lambda actor, phase: None,
                                             "next_command": lambda actor, context: script.next_command(context)})()
            if session.arbiter is not None:
                session.arbiter.close()
            session.phase = stage
            session.arbiter = session.arbiter_type(session.rig, None, None, actor,
                                                   f"CORR:{stage}_REFERENCE", stage, guard=False)
            self.actors[stage] = (script, segments[-1][2])
        script, final_gear = self.actors[stage]
        target = q_values(o.to_canonical(final_gear), f"{stage} final target", 12).tolist()
        validator = session.corrector
        actual_complete = (script.done and isinstance(validator, ReceiveCorrector)
                           and validator._actual_result(observation, dict(label=f"{stage}_END", q12=target)))
        if script.done:
            command = target
        else:
            command, diag = session.arbiter.next_command(session._context(observation))
            if script.done:
                command = target
        dispatch = session._dispatch(command, observation, stage)
        out = dict(stage=stage, status="COMPLETE" if actual_complete and dispatch.status in ("SEND", "HOLD") else "RUNNING",
                   dispatch=dispatch, proposal_q12=list(command), missing=[],
                   expected_motion_result=dict(label=f"{stage}_END", q12=target),
                   detail=f"existing {stage} Script; completion requires current independent motion result")
        if dispatch.status == "STOP":
            out.update(status="STOP", requires_explicit_restart=True)
        return bridge._result(out, now)

    def place(self, task, now):
        return self._advance("PLACE", now)

    def stow(self, now):
        return self._advance("ARM_STOW", now)


class ConcreteBindings:
    """Bind local integrator's ten step names to existing run/grip and current owner references.

    ``owner`` is a local object, not a serialized profile. It supplies connected
    buses, actual observation/capture, existing cert/reference planners, STOP,
    Episode and the independent writer ledger. Missing data gives NOT_READY.
    These blocking callbacks must not be called inside Applier/apply_dispatch.
    """

    COMMAND_STEPS = ("carry", "right_approach", "right_insert", "right_close",
                     "left_release", "left_away", "place", "right_open", "stow")

    def __init__(self, owner, *, planning):
        self.owner, self.planning = owner, dict(planning)
        self.recording_errors = []

    def reference_missing(self, step):
        o = self.owner
        if o is None:
            return ["current same-process executor owner"]
        missing = []
        required = ("observe", "capture", "stop_requested", "port_owner", "execution_stats")
        required += (("certify",) if step in self.COMMAND_STEPS and step != "right_close" else ())
        required += (("close_certificate",) if step == "right_close" else ())
        required += (("insertion_alignment",) if step == "right_insert" else ())
        required += (("support_record", "reference") if step == "support" else ())
        required += (("reference",) if step in ("left_release", "left_away", "place", "right_open", "stow") else ())
        required += (("support_record",) if step in ("left_release", "left_away", "place") else ())
        missing.extend(name for name in required if not callable(getattr(o, name, None)))
        if "support_record" in required:
            from sweepick.integration.sweepick_receive_session import ReceiveSession
            guard = getattr(o, "support_record", None)
            if (getattr(guard, "__self__", None) is not getattr(o, "session", None)
                    or getattr(guard, "__func__", None) is not ReceiveSession._support_record):
                missing.append("existing ReceiveSession support guard bound to this owner session")
        if getattr(o, "pid", None) != os.getpid():
            missing.append("same executor PID")
        if getattr(o, "execution_mode", None) != "BLOCKING_REFERENCES" or getattr(o, "per_cycle_active", None) is not False:
            missing.append("blocking reference boundary with per-cycle writer inactive")
        for name in ("mapping", "mapping_sha", "profile", "progress", "torque_joints", "buses", "episode"):
            if getattr(o, name, None) is None:
                missing.append(name)
        executor = getattr(o, "executor", None)
        for name, kw in (("run", "borrowed_bus"), ("grip", "jaw_device_override")):
            fn = getattr(executor, name, None)
            if not callable(fn) or kw not in inspect.signature(fn).parameters:
                missing.append(f"current executor.{name} {kw} function merge")
        if step in ("right_close", "right_open"):
            from sweepick.integration.sweepick_receive_session import verified_right_profile
            p = getattr(o, "right_jaw_profile", None)
            if not verified_right_profile(p) or p.get("opening_scale") != "EEPROM_MIN_MAX":
                missing.append("measured right REAL JawDevice on executor EEPROM opening scale")
        return missing

    def callbacks(self):
        callbacks = dict(self.planning)
        for step in (*self.COMMAND_STEPS, "support"):
            def callback(ctx, memo, step=step):
                missing = self.reference_missing(step)
                if missing:
                    return not_ready(*missing)
                if step == "support":
                    return self._support(ctx, memo)
                if step == "right_close":
                    return self._close(ctx, memo)
                if step in ("carry", "right_approach", "right_insert"):
                    return self._receive_motion(step, ctx, memo)
                return self._reference_motion(step, ctx, memo)
            callback.reference_missing = lambda step=step: self.reference_missing(step)
            callback.owner = "existing same-PID executor reference"
            callbacks[step] = callback
        return callbacks

    def _actual(self, ctx, memo):
        now, observation = self.owner.observe(ctx, memo)
        captured = self.owner.capture(now, observation)
        if not isinstance(captured, dict) or captured.get("accepted_feedback") is not True or captured.get("status") == "STOP":
            return None, not_ready("fresh independent dual-arm observation accepted by existing session")
        self._event("actual", observation=observation)
        ack = observation.get("owner_ack")
        if ack is not None:
            self._event("ACK", evidence=ack, accepted_by_existing_ledger=captured.get("right_applied_evidence"))
        return observation, None

    def _event(self, event, **data):
        try:
            self.owner.episode.event(event, session_id=self.owner.episode.id,
                                     executor_pid=self.owner.pid, **data)
        except Exception as exc:
            self.recording_errors.append(f"{type(exc).__name__}: {exc}")

    def _controller_event(self, event, **data):
        # run/grip catch sink failures and retain them as recording errors.
        self._event(event, **data)

    def _motion(self, step, side, target, ctx, memo, *, hold_left=False):
        o = self.owner
        obs, error = self._actual(ctx, memo)
        if error:
            return error
        if o.stop_requested():
            return dict(ok=False, stopped=True, commanded=False, controller_result="STOPPED_HOLDING", why="existing operator STOP")
        if step == "right_insert" and not insertion_alignment_ready(
                o.insertion_alignment(obs), obs, o.episode.id):
            return not_ready("insertion alignment must match the command-start observation")
        if side not in ("left", "right") or not isinstance(target, dict):
            return not_ready("existing reference side/target ticks")
        if hold_left and side == "left":
            hold = (memo["plan"]["plan"].get("left_hold_q") or [None] * 6)[5]
            q = q_values(obs.get("q12"), "actual.q12", 12)
            q[5] = hold
            expected = o.executor.rc.q12_to_ticks(o.mapping, q)["left"]["gripper"]
            if target.get("gripper") != expected:
                return not_ready("load transfer must preserve the existing left Jaw hold command")
        other = "right" if side == "left" else "left"
        traj = o.executor.tt.make(obs["present_ticks"][side], target, o.profile)
        document = dict(side=side, level="model", mapping_sha256=o.mapping_sha,
                        profile=o.profile, identity=o.executor.tt.identity(side, traj, o.mapping_sha),
                        other_arm_ticks=obs["present_ticks"][other], trajectory=traj)
        cert = o.certify(document, ctx, memo)
        if cert is None:
            return not_ready("existing carried-object collision certificate")
        self._event("proposal", step=step, side=side, target_ticks=target,
                    other_arm_ticks=document["other_arm_ticks"], certificate_source=cert.get("source"))
        out = Path(ctx["out"]) / f"sweepick_261008_{step}_{len(memo.setdefault('controller_results', []))}"
        code, log = o.executor.run(
            "next", side, out, target=target, level="model", execute=True,
            other_arm_ticks=document["other_arm_ticks"], cert=cert,
            mapping=o.mapping, mapping_sha=o.mapping_sha, profile=o.profile,
            progress=o.progress, torque_joints=o.torque_joints[side],
            owner=o.port_owner, borrowed_bus=o.buses[side],
            start_recheck=lambda doc: o.certify(doc, ctx, memo),
            stop_requested=o.stop_requested, on_event=self._controller_event,
            **getattr(o, "clock_kwargs", {}))
        memo["controller_results"].append(log)
        return self._controller_result(code, log)

    @staticmethod
    def _controller_result(code, log):
        result = log.get("result")
        return dict(ok=code == 0, commanded=any(log.get("writes", {}).values()),
                    controller_result=result, stopped=result in (
                        "STOPPED_HOLDING", "BUS_LOST", "FAULT_HOLDING", "FAULT_JOINTS_RELEASED",
                        "NO_PROGRESS_HOLDING", "RELEASE_UNCONFIRMED"),
                    why=None if code == 0 else str(log.get("stopped_by") or log.get("refused") or result),
                    owner_unknown=result == "BUS_LOST")

    def _receive_motion(self, step, ctx, memo):
        plan = (memo.get("plan") or {}).get("plan") or {}
        if plan.get("status") != "PLAN_PREVIEW":
            return not_ready("existing RECEIVE plan preview")
        waypoints = [w for w in plan["waypoints"] if
                     (step == "carry" and (w["label"].startswith("CARRY_") or w["label"] in ("LEFT_HOLD_SEED", "RIGHT_OPEN")))
                     or w["label"] == step.upper()]
        if not waypoints:
            return not_ready(f"existing {step} waypoints")
        commanded = False
        for waypoint in waypoints:
            if step == "right_insert":
                obs, error = self._actual(ctx, memo)
                if error:
                    return error
                align = self.owner.insertion_alignment(obs)
                if not insertion_alignment_ready(align, obs, self.owner.episode.id):
                    return not_ready("current visual right insertion alignment / uncertainty / geometry clearance")
            side = "left" if waypoint["label"] in ("LEFT_HOLD_SEED", "CARRY_LEFT") else "right"
            q = q_values(waypoint["q12"], "teacher waypoint", 12)
            target = self.owner.executor.rc.q12_to_ticks(self.owner.mapping, q)[side]
            result = self._motion(step, side, target, ctx, memo)
            commanded |= result.get("commanded", False)
            if not result["ok"]:
                return dict(result, commanded=commanded)
        return dict(ok=True, commanded=commanded, controller_result="HOLDING")

    def _close(self, ctx, memo):
        o = self.owner
        obs, error = self._actual(ctx, memo)
        if error:
            return error
        device = o.right_jaw_profile["device"]
        cert = o.close_certificate(obs, device, ctx, memo)
        if cert is None:
            return not_ready("existing current right closing-sweep collision certificate")
        code, log = o.executor.grip(
            "right", Path(ctx["out"]) / "sweepick_261008_right_close", execute=True,
            torque_joints=o.torque_joints["right"], level="model", cert=cert,
            mapping=o.mapping, mapping_sha=o.mapping_sha, profile=o.profile,
            owner=o.port_owner, borrowed_bus=o.buses["right"], jaw_device_override=device,
            stop_requested=o.stop_requested, on_event=self._controller_event,
            **getattr(o, "clock_kwargs", {}))
        memo.setdefault("controller_results", []).append(log)
        return dict(self._controller_result(code, log), contact_candidate=log.get("outcome", {}).get("contact_candidate"),
                    support_confirmed=False)

    def _support(self, ctx, memo):
        from sweepick.integration.sweepick_receive_session import tg
        obs, error = self._actual(ctx, memo)
        if error:
            return error
        record = self.owner.support_record(obs)
        if record is not None:
            return dict(ok=True, commanded=False, support_confirmation=record)
        session = getattr(self.owner, "session", None)
        if (not getattr(session, "right_profile_verified", False)
                or getattr(getattr(session, "right_jaw", None), "state", None) != tg.CONTACT_CANDIDATE):
            return not_ready("current right contact candidate before controlled load transfer")
        # The existing controlled-transfer reference precedes support assessment.
        transfer = self._reference_motion("load_transfer", ctx, memo)
        if not transfer["ok"]:
            return transfer
        obs, error = self._actual(ctx, memo)
        if error:
            return error
        record = self.owner.support_record(obs)
        if record is None:
            return dict(not_ready("actual right motor/vision/load-transfer support confirmation"),
                        commanded=transfer.get("commanded", False))
        memo["right_support"] = record
        self._event("right_support", evidence=record)
        return dict(ok=True, commanded=transfer.get("commanded", False), support_confirmation=record)

    def _reference_motion(self, step, ctx, memo):
        obs, error = self._actual(ctx, memo)
        if error:
            return error
        if step in ("left_release", "left_away", "place") and self.owner.support_record(obs) is None:
            return not_ready("current actual right support confirmation")
        reference = self.owner.reference(step, ctx, memo, obs)
        if not isinstance(reference, dict) or reference.get("status") != "REFERENCE_READY" or not reference.get("source"):
            return not_ready(f"existing {step} target/reference", *(reference.get("missing", []) if isinstance(reference, dict) else []))
        segments = reference.get("segments")
        if not isinstance(segments, list) or not segments:
            return not_ready(f"existing {step} segments")
        commanded = False
        for segment in segments:
            result = self._motion(step, segment.get("side"), segment.get("target_ticks"), ctx, memo,
                                  hold_left=step == "load_transfer")
            commanded |= result.get("commanded", False)
            if not result["ok"]:
                return dict(result, commanded=commanded)
        return dict(ok=True, commanded=commanded, controller_result="HOLDING")


def bind_full_episode(owner):
    """Capture the local owner's existing per-cycle references without a second writer.

    This is the active FullEpisode route. The blocking callbacks above are
    controller references for local integrator's stage interface, never nested here.
    """
    from sweepick.integration.sweepick_manipulation_handoff import FullEpisodeHandoff
    from sweepick.manipulation.sweepick_right_receive_dispatch import prepare_receive, receive_references_ready

    recording_errors = []
    current_bridge, last_sent = None, None

    def event(name, **data):
        try:
            owner.episode.event(name, **data)
        except Exception as exc:
            recording_errors.append(f"{type(exc).__name__}: {exc}")

    def observation(stage, task, now):
        obs = owner.observation_source(stage, task, now)
        if not isinstance(obs, dict):
            return obs
        obs = dict(obs)
        if current_bridge is not None:
            session = current_bridge.session
            if obs.get("owner_ack") is None:
                ack = observed_owner_ack(last_sent, obs, mapping=session.applier.mp, session_id=session.session_id)
                if ack is not None:
                    obs["owner_ack"] = ack
            expected = None
            downstream = owner.downstream
            if isinstance(downstream, ExistingDownstream) and session.phase in downstream.actors:
                script, final_gear = downstream.actors[session.phase]
                if script.done:
                    expected = dict(label=f"{session.phase}_END", q12=owner.to_canonical(final_gear))
            elif (stage == "COORDINATE" and (session.arbiter is None or session.arbiter.owner == "C")
                  and hasattr(session.corrector, "plan")):
                expected = session.corrector.plan["waypoints"][session.corrector.index]
            if obs.get("motion_result") is None:
                result = observed_motion_result(expected, obs, mapping=session.applier.mp,
                                                progress=owner.progress, session_id=session.session_id)
                if result is not None:
                    obs["motion_result"] = result
        event("actual", stage=stage, observation=obs)
        if obs.get("owner_ack") is not None:
            event("ACK", evidence=obs["owner_ack"], ledger_owner="ReceiveSession._owner_ack")
        if (obs.get("motion_result") or {}).get("label") == "RIGHT_INSERT":
            alignment = owner.insertion_alignment(obs)
            if not insertion_alignment_ready(alignment, obs, owner.episode.id):
                obs = dict(obs, motion_result=None)
        return obs

    def apply(dispatch, latest):
        nonlocal last_sent
        event("proposal", owner=dispatch.owner, target=dispatch.raw_target)
        event("retimed", owner=dispatch.owner, goal=dispatch.applied)
        event("sent", owner=dispatch.owner, write=dispatch.write, status="ATTEMPTED")
        receipt = owner.apply_dispatch(dispatch, latest)
        last_sent = dispatch.write
        event("writer_return", receipt=receipt, independent=False)
        return receipt

    def preflight(out, state):
        result = receive_references_ready(
            owner.session_kwargs, observation_source=observation,
            downstream=owner.downstream, real_stow_profile=owner.real_stow_profile,
            apply_dispatch=apply, on_stop=owner.on_stop, record_event=event)
        for name in ("apply_dispatch", "on_stop", "insertion_alignment", "observation_source", "execution_stats", "pose_bindings"):
            if not callable(getattr(owner, name, None)):
                result["missing"].append(name)
        if getattr(owner, "progress", None) is None:
            result["missing"].append("existing progress profile")
        if owner.execution_mode != "PER_CYCLE" or owner.per_cycle_active is not True:
            result["missing"].append("one active per-cycle owner")
        result["ready"] = not result["missing"]
        return result

    def prepare(snapshot, scene, evidence):
        nonlocal current_bridge
        bindings = owner.pose_bindings(snapshot, scene, evidence)
        session_kwargs = dict(owner.session_kwargs, insertion_alignment=lambda obs:
                              insertion_alignment_ready(owner.insertion_alignment(obs), obs, owner.episode.id))
        result = prepare_receive(
            snapshot, bindings, session_kwargs=session_kwargs,
            observation_source=observation, downstream=owner.downstream,
            real_stow_profile=owner.real_stow_profile, apply_dispatch=apply,
            on_stop=owner.on_stop, record_event=event)
        if isinstance(owner.downstream, ExistingDownstream) and not isinstance(result, dict):
            owner.downstream.bridge = result
        if not isinstance(result, dict):
            current_bridge = result
        return result

    handoff = FullEpisodeHandoff(
        prepare=prepare, preflight=preflight, task=owner.task, perception=owner.perception,
        wait_next_cycle=owner.wait_next_cycle, recorder=owner.episode,
        execution_stats=owner.execution_stats)
    handoff.recording_errors = recording_errors
    return handoff
