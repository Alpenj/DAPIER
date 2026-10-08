"""Feedback-owned RECEIVE bridge; this module never opens a port or writes a motor."""
from __future__ import annotations

import copy
import math
import os
import threading
import uuid
from dataclasses import replace

from sweepick.integration.sweepick_receive_runtime import field_module

tg = field_module("grasp")
tm = field_module("task_manager")
Arbiter = field_module("mvp_r3").Arbiter


class OwnershipError(RuntimeError):
    pass


_REGISTRY = {}
_REGISTRY_LOCK = threading.Lock()


def _q12(value):
    if not isinstance(value, (list, tuple)) or len(value) != 12:
        return None
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value):
        return None
    if not all(0 <= value[i] <= 1 for i in (5, 11)):
        return None
    return list(value)


def _present(value):
    return value if isinstance(value, dict) and all(isinstance(value.get(side), dict) for side in ("left", "right")) else None


def verified_right_profile(profile):
    device = profile.get("device") if isinstance(profile, dict) else None
    return bool(isinstance(profile, dict) and profile.get("verified") is True
                and profile.get("side") == "right" and profile.get("source_kind") == "REAL"
                and profile.get("source") and profile.get("device_identity")
                and profile.get("measurement_evidence") and isinstance(device, tg.JawDevice)
                and device != tg.SIM_PGRIPPER and bool(device.source))


class ReceiveSession:
    """Own one pair of already-open controller ports and turn fresh observations into write proposals."""

    SIDE_FIELDS = ("command", "actual", "feedback_stamp", "seq", "source", "provenance", "torque", "velocity", "load", "current", "current_units", "current_baseline")

    def __init__(self, state, *, ports, owner_pid, rig, applier, primary=None,
                 primary_name="ACT:RECEIVE_3V", corrector=None,
                 corrector_name="CORR:RECEIVE_SENSOR", control_dt=None,
                 port_owner_pids=None, right_jaw_profile=None, right_jaw=None,
                 left_jaw=None, support_confirmation=None, camera_current=None,
                 apply_observation=None, kind=None, arbiter_type=Arbiter,
                 session_id=None):
        self.state = copy.copy(state)
        self.pid = owner_pid
        self.ports = tuple(ports)
        if len(self.ports) != 2 or len(set(self.ports)) != 2:
            raise OwnershipError("RECEIVE requires one distinct left/right port pair")
        if not isinstance(port_owner_pids, dict) or set(port_owner_pids) != set(self.ports):
            raise OwnershipError("port_owner_pids must name exactly both RECEIVE ports")
        supplied = set(port_owner_pids.values())
        if supplied != {owner_pid}:
            raise OwnershipError("both RECEIVE ports must have one PID owner")
        if owner_pid != os.getpid():
            raise OwnershipError("the RECEIVE command owner must be this local PID")
        with _REGISTRY_LOCK:
            overlap = set(self.ports) & set(_REGISTRY)
            if overlap:
                raise OwnershipError(f"RECEIVE port already registered in this PID: {sorted(overlap)}")
            for port in self.ports:
                _REGISTRY[port] = self
        try:
            self.rig, self.applier = rig, applier
            self.session_id = session_id or uuid.uuid4().hex
            self.primary, self.corrector = primary, corrector
            if primary is None and corrector is None:
                raise ValueError("an ACT primary or existing corrector is required")
            self.primary_name, self.corrector_name = primary_name, corrector_name
            self.arbiter_type, self.arbiter, self.phase = arbiter_type, None, None
            self.control_dt = control_dt
            self.support_confirmation = support_confirmation
            self.camera_current, self.apply_observation = camera_current, apply_observation
            self.observation_applied = False
            source = self.state.get("source_kind", "UNKNOWN")
            self.kind = kind or (tm.Kind.REAL if source == "REAL" else tm.Kind.MOCK)
            profile = right_jaw_profile if right_jaw_profile is not None else self.state.get("right_jaw_profile")
            device = profile.get("device") if isinstance(profile, dict) else None
            self.right_profile_verified = verified_right_profile(profile)
            if right_jaw is not None:
                if not isinstance(right_jaw, tg.Jaw):
                    raise TypeError("right_jaw must use tjj_grasp.Jaw")
                if getattr(right_jaw, "provenance", None) != tg.INDEPENDENT:
                    raise ValueError("right_jaw must retain independent feedback provenance")
                self.right_jaw = right_jaw
                self.right_profile_verified = self.right_profile_verified and getattr(right_jaw, "dev", None) == device
            elif self.right_profile_verified and control_dt is not None:
                self.right_jaw = tg.Jaw(device, control_dt, provenance=tg.INDEPENDENT)
            else:
                self.right_jaw = None
                self.right_profile_verified = False
            if left_jaw is not None and not isinstance(left_jaw, tg.Jaw):
                raise TypeError("left_jaw must use tjj_grasp.Jaw")
            self.left_jaw = left_jaw
            self.closed = False
            self.latest = self.last_seen = None
            self.latest_missing = []
            self.last_feedback_stamp = self.last_seq = None
            self.right_last_applied_q = None
            self.right_applied_evidence = None
            self.handoff_candidate = False
            self.right_support = None
            self.release_complete = False
            self.stopped_dispatch = None
            self._right_close_started = False
            self.right_insert_confirmed = None
            self.proposals = []
        except Exception:
            with _REGISTRY_LOCK:
                for port in self.ports:
                    if _REGISTRY.get(port) is self:
                        del _REGISTRY[port]
            raise

    def close(self):
        if self.closed:
            return
        if self.arbiter is not None:
            self.arbiter.close()
        with _REGISTRY_LOCK:
            for port in self.ports:
                if _REGISTRY.get(port) is self:
                    del _REGISTRY[port]
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def _arbiter(self, phase):
        if self.phase == phase:
            return self.arbiter
        if self.arbiter is not None:
            self.arbiter.close()
        elif phase in ("RELEASE", "LOAD_TRANSFER") and hasattr(self.primary, "discard_chunk"):
            self.primary.discard_chunk()
        self.phase, self.arbiter = phase, None
        reference = "load_transfer_controller" if phase == "LOAD_TRANSFER" else "release_controller"
        if phase in ("RELEASE", "LOAD_TRANSFER") and getattr(self.corrector, reference, None) is None:
            return None
        self.arbiter = self.arbiter_type(
            self.rig, None if phase in ("RELEASE", "LOAD_TRANSFER") else self.primary, self.primary_name,
            corrector=self.corrector,
            corrector_name=("CORR:RELEASE_REFERENCE" if phase == "RELEASE" else
                            "CORR:LOAD_TRANSFER_REFERENCE" if phase == "LOAD_TRANSFER" else self.corrector_name),
            begin=phase, guard=False)
        return self.arbiter

    def _missing(self, observation):
        missing = []
        for name in ("q12", "present_ticks", "feedback_stamp", "seq", "executor_pid", "wrist", "os30a"):
            if observation.get(name) is None:
                missing.append(name)
        for side in ("left", "right"):
            packet = observation.get(side)
            for name in self.SIDE_FIELDS:
                if not isinstance(packet, dict) or packet.get(name) is None:
                    missing.append(f"{side}.{name}")
        return missing

    def _accept(self, now, observation):
        if self.closed:
            raise RuntimeError("RECEIVE session is closed")
        if self.stopped_dispatch is not None:
            return False, "existing Applier STOP requires explicit recovery and a new session"
        if not isinstance(observation, dict):
            return False, "observation must be a mapping"
        candidate = copy.deepcopy(observation)
        self.last_seen = candidate
        self.latest_missing = self._missing(observation)
        q = _q12(observation.get("q12"))
        stamp, seq = observation.get("feedback_stamp"), observation.get("seq")
        if observation.get("snapshot_only") is True:
            return False, "a saved snapshot cannot authorize a live cycle"
        if observation.get("executor_pid") != self.pid:
            return False, "feedback is not owned by this local executor"
        if observation.get("fresh") is not True or isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or seq is None:
            return False, "fresh feedback_stamp and seq are required"
        if stamp > now:
            return False, "feedback stamp is in the future"
        if stamp == self.last_feedback_stamp or seq == self.last_seq:
            return False, "feedback did not advance"
        if self.last_feedback_stamp is not None and stamp < self.last_feedback_stamp:
            return False, "feedback moved backward"
        if q is None or _present(observation.get("present_ticks")) is None:
            return False, "finite q12 and both present-tick maps are required"
        self.last_feedback_stamp, self.last_seq = stamp, seq
        self.latest = candidate
        self.observation_applied = self._apply_actual(q, observation)
        self._owner_ack(observation)
        self._jaw_feedback(now, observation)
        return True, ""

    def _apply_actual(self, q, observation):
        if self.apply_observation is not None:
            self.apply_observation(self.state, q, observation)
            applied = True
        else:
            applied = self.state.get("_model") is None and self.state.get("_data") is None
        wrist = observation.get("wrist")
        sensor = getattr(self.primary, "sensor", None)
        current = getattr(sensor, "current", None)
        if isinstance(current, dict):
            for side in ("left", "right"):
                current.pop(side + "_wrist_rgb", None)
                if isinstance(wrist, dict) and self._packet_current(wrist.get(side), observation):
                    current[side + "_wrist_rgb"] = wrist[side]
        if isinstance(self.camera_current, dict):
            self.camera_current.pop("top", None)
            if self._packet_current(observation.get("os30a"), observation):
                self.camera_current["top"] = observation["os30a"]
        return applied

    def _packet_current(self, packet, observation):
        return bool(isinstance(packet, dict)
                    and packet.get("fresh") is True
                    and packet.get("feedback_stamp") == observation.get("feedback_stamp")
                    and packet.get("seq") == observation.get("seq")
                    and isinstance(packet.get("source"), str) and packet["source"])

    def _motor_packet_valid(self, side, observation):
        packet = observation.get(side)
        return bool(self._packet_current(packet, observation)
                    and packet.get("provenance") == tg.INDEPENDENT
                    and all(isinstance(packet.get(name), (int, float))
                            and not isinstance(packet.get(name), bool)
                            and math.isfinite(packet[name]) for name in ("command", "actual")))

    def _images_fresh(self, observation):
        wrist = observation.get("wrist")
        return bool(isinstance(wrist, dict)
                    and self._packet_current(wrist.get("left"), observation)
                    and self._packet_current(wrist.get("right"), observation)
                    and self._packet_current(observation.get("os30a"), observation))

    def _owner_ack(self, observation):
        ack = observation.get("owner_ack")
        if not isinstance(ack, dict):
            return
        q = _q12(ack.get("q12"))
        if (ack.get("accepted") is True and ack.get("independent") is True
                and ack.get("session_id") == self.session_id
                and ack.get("executor_pid") == self.pid
                and ack.get("feedback_stamp") == observation.get("feedback_stamp")
                and ack.get("seq") == observation.get("seq")
                and isinstance(ack.get("source"), str) and ack["source"] and q is not None):
            self.right_last_applied_q = q[6:]
            self.right_applied_evidence = copy.deepcopy(ack)

    def _jaw_feedback(self, now, observation):
        for side, jaw in (("left", self.left_jaw), ("right", self.right_jaw)):
            packet = observation.get(side)
            if jaw is not None and self._motor_packet_valid(side, observation):
                jaw.update(now, packet.get("command"), packet.get("actual"),
                           feedback_stamp=packet.get("feedback_stamp"),
                           torque_enabled=packet.get("torque"), seq=packet.get("seq"))

    def capture(self, now, observation):
        accepted, why = self._accept(now, observation)
        return self._base("OBSERVE", accepted, why)

    def _base(self, stage, accepted, why=""):
        out = dict(stage=stage, accepted_feedback=accepted, status="RUNNING" if accepted else "NOT_READY",
                    detail=why, missing=list(self.latest_missing), source_kind=self.state.get("source_kind", "UNKNOWN"),
                    real_execution_ready=False, motor_commands_sent=0,
                    session_id=self.session_id,
                    right_last_applied_q=copy.deepcopy(self.right_last_applied_q),
                    right_applied_evidence=copy.deepcopy(self.right_applied_evidence),
                    feedback=copy.deepcopy(self.latest if accepted else self.last_seen),
                    handoff_candidate=self.handoff_candidate,
                    right_support_confirmed=False,
                    release_complete=self.release_complete)
        if self.stopped_dispatch is not None:
            out.update(status="STOP", dispatch=self.stopped_dispatch, requires_explicit_restart=True,
                       detail="existing Applier STOP; explicit recovery and a new session required: "
                       + "; ".join(getattr(self.stopped_dispatch, "reasons", [])))
        return out

    def _object_label(self, observation, *, left_away=None):
        evidence = observation.get("object_evidence")
        if (self.right_jaw is None or not self._motor_packet_valid("right", observation)
                or not self._packet_current(evidence, observation) or not self._images_fresh(observation)):
            return tg.UNKNOWN
        return tg.fuse(
            self.right_jaw.state,
            object_at_tool=evidence.get("object_at_tool"),
            evidence_valid=evidence.get("evidence_valid", False),
            object_on_support=evidence.get("object_on_support"),
            co_motion_observed=evidence.get("co_motion_observed", False),
            hand_raised_since_contact=evidence.get("hand_raised_since_contact", False),
            other_hand_unloaded_and_away=left_away,
            was_held=self.handoff_candidate,
            hand_raised=evidence.get("hand_raised", True))

    def _support_record(self, observation):
        record = self.support_confirmation(observation) if self.support_confirmation is not None else observation.get("support_confirmation")
        right = observation.get("right") or {}
        ok = (self.right_profile_verified and isinstance(record, dict)
              and record.get("label") == tg.SUPPORT_CONFIRMED
              and record.get("confirmed") is True
              and record.get("source_kind") == "REAL"
              and isinstance(record.get("source"), str) and bool(record["source"])
              and record.get("session_id") == self.session_id
              and record.get("executor_pid") == self.pid
              and record.get("feedback_stamp") == right.get("feedback_stamp")
              and record.get("seq") == right.get("seq")
              and record.get("provenance") == tg.INDEPENDENT
              and self._motor_packet_valid("right", observation)
              and right.get("torque") is True
              and all(self._packet_current(record.get(name), observation)
                      and record[name].get("observed") is True
                      for name in ("motor_evidence", "vision_evidence", "load_transfer_evidence"))
              and self._object_label(observation) in (tg.GRASP_CANDIDATE, tg.HELD_CONFIRMED))
        return copy.deepcopy(record) if ok else None

    def _left_open_away(self, observation):
        record = observation.get("left_open_and_away")
        left = observation.get("left") or {}
        return bool(isinstance(record, dict) and record.get("open") is True and record.get("away") is True
                    and record.get("source_kind") == "REAL"
                    and isinstance(record.get("source"), str) and record["source"]
                    and record.get("session_id") == self.session_id
                    and record.get("executor_pid") == self.pid
                    and record.get("provenance") == tg.INDEPENDENT
                    and record.get("feedback_stamp") == left.get("feedback_stamp")
                    and record.get("seq") == left.get("seq")
                    and self._motor_packet_valid("left", observation)
                    and all(self._packet_current(record.get(name), observation)
                            and record[name].get("observed") is True
                            for name in ("motor_evidence", "vision_evidence")))

    def _right_insert_record(self, observation):
        record = observation.get("right_insert_confirmed") or observation.get("motion_result")
        target = self.state.get("right_insert_q12")
        recorded = _q12(record.get("q12")) if isinstance(record, dict) else None
        exact = isinstance(target, (list, tuple)) and len(target) == 12 and recorded == list(target)
        right = observation.get("right") or {}
        ok = (isinstance(record, dict) and record.get("label") == "RIGHT_INSERT"
              and record.get("result") == "HOLDING"
              and record.get("source_kind") == "REAL"
              and isinstance(record.get("source"), str) and record["source"]
              and record.get("session_id") == self.session_id
              and record.get("executor_pid") == self.pid
              and record.get("provenance") == tg.INDEPENDENT
              and record.get("feedback_stamp") == right.get("feedback_stamp")
              and record.get("seq") == right.get("seq")
              and self._motor_packet_valid("right", observation)
              and exact)
        return copy.deepcopy(record) if ok else None

    def cycle(self, stage, now, observation):
        stage = stage.upper()
        if stage not in ("COORDINATE", "RELEASE"):
            raise ValueError("ReceiveSession owns only COORDINATE and RELEASE")
        accepted, why = self._accept(now, observation)
        out = self._base(stage, accepted, why)
        if not accepted:
            return out
        if stage == "RELEASE" and getattr(self.corrector, "release_controller", None) is None:
            self._arbiter(stage)
            out.update(status="NOT_READY", detail="existing real release/retreat controller is missing",
                       missing=list(out["missing"]) + ["existing real release/retreat controller"])
            return out
        if self.state.get("actual_held_pose_available") is False:
            out.update(status="NOT_READY", detail="fresh held pose in the scene frame is missing")
            return out
        if stage == "COORDINATE" and hasattr(self.primary, "discard_chunk") and (not self.observation_applied or not self._images_fresh(observation)):
            self.primary.discard_chunk()
            out.update(status="NOT_READY", detail="fresh left/right/top packets and applied actual observation are required for ACT")
            return out
        insert = self._right_insert_record(observation) if stage == "COORDINATE" else None
        if insert is not None:
            self.right_insert_confirmed = insert
        label = self._object_label(observation)
        out["right_grasp_label"] = label
        current_handoff = self.right_insert_confirmed is not None and label in (tg.GRASP_CANDIDATE, tg.HELD_CONFIRMED)
        if current_handoff:
            self.handoff_candidate = True
        out["handoff_candidate"] = self.handoff_candidate
        support = self._support_record(observation) if stage == "RELEASE" else None
        if support is not None:
            self.right_support = support
        out["right_support_confirmed"] = support is not None
        if stage == "COORDINATE" and not self.right_profile_verified:
            out.update(right_close_status="NOT_READY",
                       detail="right REAL Jaw profile missing or unverified; feedback captured, close NOT_READY")
        elif stage == "COORDINATE" and self.right_insert_confirmed is None:
            out.update(right_close_status="NOT_READY",
                       detail="fresh independent RIGHT_INSERT completion is required before right close")
        if stage == "RELEASE" and not self.handoff_candidate:
            out.update(status="NOT_READY", detail="HANDOFF_CANDIDATE is required before RELEASE")
            return out
        dispatch_stage = stage
        if stage == "RELEASE" and support is None:
            if (self.phase == "RELEASE" or not current_handoff or not self.right_profile_verified
                    or getattr(self.corrector, "load_transfer_controller", None) is None):
                out.update(status="NOT_READY", detail="fresh independent right support evidence is required before left release; existing load-transfer reference unavailable",
                           missing=list(out["missing"]) + ["right support or existing controlled load-transfer reference"])
                return out
            dispatch_stage = "LOAD_TRANSFER"
            out["subphase"] = dispatch_stage
        if stage == "RELEASE" and self._left_open_away(observation):
            label = self._object_label(observation, left_away=True)
            out["right_grasp_label"] = label
            if label == tg.SUPPORT_CONFIRMED:
                hold = list(observation["q12"])
                hold[5], hold[11] = observation["left"]["command"], observation["right"]["command"]
                dispatch = self._dispatch(hold, observation, stage)
                out["dispatch"] = dispatch
                if dispatch.status in ("BRAKE", "STOP"):
                    out.update(status="STOP" if dispatch.status == "STOP" else "NOT_READY",
                               requires_explicit_restart=dispatch.status == "STOP",
                               detail=f"RELEASE completion blocked by existing dispatch {dispatch.status}: "
                               + "; ".join(getattr(dispatch, "reasons", [])))
                    return out
                self.release_complete = True
                out.update(status="COMPLETE", release_complete=True,
                           continuation=self.continuation())
                return out
        command, diag = self._arbiter(dispatch_stage).next_command(self._context(observation))
        if isinstance(diag, dict) and diag.get("status") == "NOT_READY":
            out.update(status="NOT_READY", controller_diag=diag,
                       detail="existing controller inputs are incomplete",
                       missing=list(out["missing"]) + list(diag.get("missing", [])))
            return out
        proposal = _q12(command)
        if proposal is None:
            out.update(status="NOT_READY", detail="controller did not return a finite canonical q12")
            return out
        hold = self.state.get("left_hold_q")
        if not isinstance(hold, (list, tuple)) or len(hold) != 6:
            out.update(status="NOT_READY", detail="left hold command is missing")
            return out
        if stage != "RELEASE" or support is None:
            proposal[5] = hold[5]
        right = observation.get("right") or {}
        if not self._motor_packet_valid("right", observation):
            out.update(status="NOT_READY", detail="independent right applied-command/actual feedback is missing")
            return out
        if self.right_profile_verified and self.right_jaw is not None and self.right_insert_confirmed is not None:
            if stage == "COORDINATE" and not self._right_close_started:
                self.right_jaw.start_close()
                self._right_close_started = True
            proposal[11] = self.right_jaw.next_command(right.get("command"))
        elif not self.right_profile_verified:
            proposal[11] = right["command"]
        else:
            proposal[11] = max(proposal[11], right["command"])
        dispatch = self._dispatch(proposal, observation, dispatch_stage)
        row = dict(stage=stage, q12=proposal, owner=self._arbiter(dispatch_stage).name,
                   diag=diag, dispatch=dispatch, applied=False)
        self.proposals.append(row)
        dispatch_status = getattr(dispatch, "status", None)
        completed = stage == "COORDINATE" and current_handoff and dispatch_status not in ("BRAKE", "STOP")
        out.update(status="HANDOFF_CANDIDATE" if completed else ("STOP" if dispatch_status == "STOP" else "NOT_READY" if dispatch_status == "BRAKE" else "RUNNING"),
                   proposal_q12=proposal, dispatch=dispatch, controller_diag=diag,
                   right_last_applied_q=copy.deepcopy(self.right_last_applied_q))
        if dispatch_status in ("BRAKE", "STOP"):
            out.update(requires_explicit_restart=dispatch_status == "STOP",
                       detail=f"dispatch proposal returned {dispatch_status}; stage completion is not reported: "
                       + "; ".join(getattr(dispatch, "reasons", [])))
        return out

    def _dispatch(self, proposal, observation, stage):
        dispatch = self.applier.dispatch(
            self._arbiter(stage).name, proposal, observation["present_ticks"],
            fresh=True, servo_status=observation.get("servo_status"),
            t_read=observation.get("feedback_stamp"), cycle_start=observation.get("cycle_start"),
            closed_loop=self.state.get("source_kind") == "REAL")
        if dispatch.status == "STOP":
            self.stopped_dispatch = dispatch
            self.arbiter.close()
        return dispatch

    def _context(self, observation):
        return dict(model=self.state.get("_model"), data=self.state.get("_data"),
                    control_dt=self.control_dt, observation=observation,
                    measured_q12=list(observation["q12"]),
                    terminal_failure=None)

    def continuation(self):
        observation = self.latest or {}
        right = observation.get("right") or {}
        held = observation.get("held_object") or {}
        out = dict(
            right_actual_hold=right.get("actual"),
            right_command_hold=right.get("command"),
            right_last_applied_q=copy.deepcopy(self.right_last_applied_q),
            measured_q12=copy.deepcopy(observation.get("q12")),
            object_pose_model=copy.deepcopy(held.get("object_pose_model")),
            object_pose_uncertainty=copy.deepcopy(held.get("object_pose_uncertainty")),
            object_pose_frame=held.get("pose_frame"), object_pose_source=held.get("pose_source"),
            object_pose_stamp=held.get("feedback_stamp"), object_pose_seq=held.get("seq"),
            transforms=copy.deepcopy(held.get("transforms")),
            grasp_evidence=copy.deepcopy(held.get("grasp_evidence")),
            right_support=copy.deepcopy(self.right_support),
            feedback_stamp=observation.get("feedback_stamp"), seq=observation.get("seq"))
        out["missing"] = [name for name in ("right_actual_hold", "right_command_hold", "right_last_applied_q",
                                                  "measured_q12", "object_pose_model", "object_pose_uncertainty",
                                                  "transforms", "grasp_evidence", "right_support") if out[name] is None]
        return out


class ReceiveManipulation:
    """Task-manager adapter for RECEIVE/RELEASE and an injected existing PLACE/STOW controller."""

    def __init__(self, session, observation_source, downstream, *, real_stow_profile=None,
                 apply_dispatch=None, on_stop=None, record_event=None):
        self.session, self.observation_source, self.downstream = session, observation_source, downstream
        self.real_stow_profile = real_stow_profile
        self.kind = session.kind
        self.last_downstream_evidence = None
        self.apply_dispatch, self.on_stop, self.record_event = apply_dispatch, on_stop, record_event
        self.last_result = None
        self.application_failure = None
        self.stop_handled = False
        self.recording_errors = []

    def _observation(self, stage, task, now):
        observation = self.observation_source(stage, task, now)
        self._record("receive_observation", stage=stage, session_id=self.session.session_id,
                     observation={k: copy.deepcopy(observation.get(k)) for k in
                                  ("q12", "present_ticks", "feedback_stamp", "seq", "executor_pid",
                                   "left", "right", "held_object", "owner_ack")}
                     if isinstance(observation, dict) else None)
        return observation

    def _record(self, event, **data):
        if self.record_event is not None:
            try:
                self.record_event(event, **data)
            except Exception as exc:
                # Recording quality does not replace the control/feedback outcome.
                self.recording_errors.append(f"{type(exc).__name__}: {exc}")

    def _deliver(self, out):
        dispatch = out.get("dispatch")
        if dispatch is None:
            return out
        self._record("receive_dispatch", stage=out["stage"], owner=dispatch.owner,
                     proposal_q12=out.get("proposal_q12"), raw_target=getattr(dispatch, "raw_target", None),
                     retimed_goal=getattr(dispatch, "applied", None), planned_write=dispatch.write,
                     dispatch_status=dispatch.status, reasons=getattr(dispatch, "reasons", []),
                     actual_application="UNKNOWN", session_id=self.session.session_id)
        if dispatch.status == "STOP":
            if not self.stop_handled:
                self.stop_handled = True
                if self.on_stop is not None:
                    try:
                        out["stop_handler_result"] = self.on_stop(dispatch, self.session.latest)
                    except Exception as exc:
                        out["stop_handler_error"] = f"{type(exc).__name__}: {exc}"
                else:
                    out["stop_handler_status"] = "MISSING"
            return out
        if dispatch.write is not None:
            if self.apply_dispatch is None:
                out.update(status="NOT_READY", detail="existing local writer callback missing; proposal not applied",
                           missing=list(out.get("missing", [])) + ["apply_dispatch"])
            else:
                try:
                    # A writer receipt is not independent feedback and never updates the applied ledger here.
                    out["writer_receipt"] = self.apply_dispatch(dispatch, self.session.latest)
                    self._record("receive_writer_receipt", stage=out["stage"],
                                 session_id=self.session.session_id, receipt=out["writer_receipt"])
                except Exception as exc:
                    self.application_failure = f"existing writer failed: {type(exc).__name__}: {exc}"
                    from types import SimpleNamespace
                    failed = SimpleNamespace(status="STOP", write=None, owner=dispatch.owner,
                                             reasons=[self.application_failure], gates=[])
                    self.session.stopped_dispatch = failed
                    self.session.close()
                    out.update(status="STOP", detail=self.application_failure, requires_explicit_restart=True,
                               attempted_dispatch=dispatch, dispatch=failed)
                    if self.on_stop is not None:
                        try:
                            out["stop_handler_result"] = self.on_stop(failed, self.session.latest)
                        except Exception as stop_exc:
                            out["stop_handler_error"] = f"{type(stop_exc).__name__}: {stop_exc}"
        return out

    def _result(self, out, now):
        out = self._deliver(out)
        status = (tm.Status.FAILED if out["status"] == "STOP" else tm.Status.SUCCEEDED
                  if out["status"] in ("HANDOFF_CANDIDATE", "COMPLETE") else tm.Status.RUNNING)
        self.last_result = tm.CommandResult(status, self.kind, out.get("detail", out["status"]), now, out)
        return self.last_result

    def _blocked(self, stage, now):
        if self.application_failure is not None:
            return self._result(dict(stage=stage, status="STOP", detail=self.application_failure,
                                     requires_explicit_restart=True), now)
        if self.session.stopped_dispatch is not None:
            return self._result(self.session._base(stage, False), now)
        return None

    def reference_missing(self):
        refs = {"observation_source": self.observation_source, "apply_dispatch": self.apply_dispatch,
                "on_stop": self.on_stop, "record_event": self.record_event,
                "PLACE": getattr(self.downstream, "place", None),
                "STOW": getattr(self.downstream, "stow", None),
                "set_receive_context": getattr(self.downstream, "set_receive_context", None)}
        missing = [name for name, value in refs.items() if not callable(value)]
        if self.real_stow_profile is None:
            missing.append("real_stow_profile")
        if not self.session.right_profile_verified:
            missing.append("right REAL Jaw profile")
        if self.session.left_jaw is None:
            missing.append("same-owner left Jaw")
        if not callable(self.session.support_confirmation):
            missing.append("support_confirmation evaluator")
        for name in ("release_controller", "load_transfer_controller"):
            reference = getattr(self.session.corrector, name, None)
            for method in ("begin", "next_command"):
                if not callable(getattr(reference, method, None)):
                    missing.append(name + "." + method)
        return missing

    def coordinate(self, task, now):
        blocked = self._blocked("COORDINATE", now)
        if blocked is not None:
            return blocked
        return self._result(self.session.cycle("COORDINATE", now, self._observation("COORDINATE", task, now)), now)

    def release(self, task, now):
        blocked = self._blocked("RELEASE", now)
        if blocked is not None:
            return blocked
        return self._result(self.session.cycle("RELEASE", now, self._observation("RELEASE", task, now)), now)

    def place(self, task, now):
        blocked = self._blocked("PLACE", now)
        if blocked is not None:
            return blocked
        if not callable(getattr(self.downstream, "place", None)):
            return self._result(self.session._base("PLACE", False, "existing real PLACE reference missing"), now)
        if not self.session.release_complete:
            return self._result(self.session._base("PLACE", False, "observed RELEASE completion is required"), now)
        observed = self.session.capture(now, self._observation("PLACE", task, now))
        continuation = self.session.continuation()
        if (not observed["accepted_feedback"] or continuation["missing"]
                or not self.session._motor_packet_valid("right", self.session.latest or {})
                or (self.session.latest.get("right") or {}).get("torque") is not True
                or self.session.state.get("actual_held_pose_available") is False
                or (self.session.state.get("source_kind") == "REAL"
                    and self.session.state.get("actual_held_pose_available") is not True)):
            observed.update(stage="PLACE", status="NOT_READY", continuation=continuation,
                            detail="current held-object continuation is incomplete")
            return self._result(observed, now)
        self.last_downstream_evidence = continuation
        if hasattr(self.downstream, "set_receive_context"):
            self.downstream.set_receive_context(continuation)
        result = self.downstream.place(task, now)
        self.last_result = replace(result, evidence={**result.evidence, "receive_continuation": continuation})
        self._record("receive_downstream_result", stage="PLACE", status=result.status.value,
                     evidence=self.last_result.evidence, session_id=self.session.session_id)
        return self.last_result

    def stow(self, now):
        blocked = self._blocked("ARM_STOW", now)
        if blocked is not None:
            return blocked
        if not callable(getattr(self.downstream, "stow", None)):
            return self._result(self.session._base("ARM_STOW", False, "existing real STOW reference missing"), now)
        if self.real_stow_profile is None:
            out = self.session._base("ARM_STOW", False, "real stow profile missing; ARM_STOW NOT_READY")
            return self._result(out, now)
        observed = self.session.capture(now, self._observation("ARM_STOW", None, now))
        if (not observed["accepted_feedback"]
                or not self.session._motor_packet_valid("right", self.session.latest or {})):
            observed.update(stage="ARM_STOW", status="NOT_READY",
                            detail="fresh measured joint and right-hold feedback are required for ARM_STOW")
            return self._result(observed, now)
        self.last_downstream_evidence = observed["feedback"]
        if hasattr(self.downstream, "set_receive_context"):
            self.downstream.set_receive_context({"stage": "ARM_STOW", "feedback": observed["feedback"]})
        result = self.downstream.stow(now)
        self.last_result = replace(result, evidence={**result.evidence, "receive_feedback": observed["feedback"]})
        self._record("receive_downstream_result", stage="ARM_STOW", status=result.status.value,
                     evidence=self.last_result.evidence, session_id=self.session.session_id)
        return self.last_result
