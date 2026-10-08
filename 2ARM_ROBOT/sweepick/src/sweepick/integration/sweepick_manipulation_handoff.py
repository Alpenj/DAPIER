"""Thin PICK02 lift-to-TaskManager handoff; device ownership stays with the injected bridge."""
from __future__ import annotations

import json
import os
from dataclasses import asdict
from pathlib import Path

from sweepick.integration.sweepick_receive_session import tm


class FullEpisodeHandoff:
    """The optional ``handoff_fn`` used by the existing PICK02 session."""

    def __init__(self, *, prepare, preflight, task, perception, wait_next_cycle, recorder,
                 execution_stats=None):
        self.prepare = prepare
        self.check_ready = preflight
        self.task = task
        self.perception = perception
        self.wait_next_cycle = wait_next_cycle
        self.recorder = recorder
        self.execution_stats = execution_stats
        self._readiness = None
        self._ready_out = None
        self._consumed = False
        self.recording_errors = []

    def ready(self, out, state):
        """Check structural references before PICK02 sends its first motor command."""
        required = {
            "prepare": self.prepare,
            "preflight": self.check_ready,
            "wait_next_cycle": self.wait_next_cycle,
            "perception.verify_place": getattr(self.perception, "verify_place", None),
            "perception.observe_area": getattr(self.perception, "observe_area", None),
            "recorder.mark": getattr(self.recorder, "mark", None),
            "recorder.event": getattr(self.recorder, "event", None),
            "recorder.lease": getattr(self.recorder, "lease", None),
        }
        missing = [name for name, value in required.items() if not callable(value)]
        if self.task is None:
            missing.append("task")
        if not getattr(self.recorder, "id", None):
            missing.append("recorder.id")

        supplied = {}
        if not missing:
            try:
                supplied = self.check_ready(out, state)
                if not isinstance(supplied, dict):
                    missing.append("preflight result")
                    supplied = {}
                elif not isinstance(supplied.get("missing", []), list):
                    missing.append("preflight.missing")
                else:
                    missing.extend(supplied.get("missing", []))
                    if supplied.get("ready") is not True and not supplied.get("missing"):
                        missing.append("preflight reported not ready")
            except Exception as exc:
                missing.append(f"preflight failed: {type(exc).__name__}: {exc}")

        record = {
            "ready": not missing and supplied.get("ready") is True,
            "missing": list(dict.fromkeys(missing)),
            "items": supplied.get("items", {}),
            "structural_only": True,
        }
        self._readiness = record
        self._ready_out = Path(out).resolve()
        if callable(getattr(self.recorder, "event", None)):
            try:
                self.recorder.event("full_episode_readiness", **record)
            except Exception as exc:
                record["ready"] = False
                record["missing"].append(f"recorder.event failed: {type(exc).__name__}: {exc}")
        return record

    def __call__(self, context):
        """Continue the same recorded episode from an externally observed PICK prefix."""
        if self._consumed:
            return self._result("NOT_READY", "this handoff callback was already consumed",
                                missing=["fresh full-episode callback"], commands_sent=False)
        self._consumed = True
        if not isinstance(context, dict):
            return self._result("NOT_READY", "handoff context is missing",
                                missing=["handoff context"], commands_sent=False)

        missing = []
        try:
            out = Path(context.get("out", "")).resolve()
        except (TypeError, ValueError) as exc:
            out = None
            missing.append(f"output directory: {type(exc).__name__}: {exc}")
        if not self._readiness or not self._readiness.get("ready"):
            missing.extend((self._readiness or {}).get("missing", ["successful ready(out, state) check"]))
        if self._ready_out != out:
            missing.append("same output directory used by ready(out, state)")
        if context.get("episode") is not self.recorder:
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
            return self._result("NOT_READY", "full-episode handoff input is incomplete",
                                missing=missing, commands_sent=False)

        manipulation = None
        result = None
        preparation = None
        try:
            with self.recorder.lease("left", "right"):
                try:
                    prepared = self.prepare(snapshot, context["scene"], context["evidence"])
                    if isinstance(prepared, dict):
                        preparation = dict(prepared)
                        preparation["real_execution_ready"] = False
                        preparation["motor_commands_sent"] = 0
                        if prepared.get("status") == "NOT_READY":
                            result = self._result(
                                "NOT_READY", prepared.get("detail", "receive preparation is not ready"),
                                missing=list(prepared.get("missing") or []), commands_sent=False,
                                preparation=preparation,
                            )
                        else:
                            result = self._result(
                                "FAILED", "prepare returned no ReceiveManipulation",
                                missing=["ReceiveManipulation"], commands_sent=False,
                                preparation=preparation,
                            )
                    else:
                        manipulation = prepared
                        preparation = self._preparation(manipulation)
                        result = self._run(manipulation, context, preparation)
                finally:
                    if manipulation is not None:
                        manipulation.session.close()
        except Exception as exc:
            result = self._result(
                "FAILED", f"full-episode callback failed: {type(exc).__name__}: {exc}",
                commands_sent=False if manipulation is None else None,
                preparation=preparation,
                object_owner="LEFT" if manipulation is None else "UNKNOWN",
            )

        try:
            self.recorder.event("full_episode_outcome", **{
                key: result.get(key) for key in (
                    "result", "why", "object_owner", "clean_resume_request_emitted",
                    "cleaning_succeeded", "source_kind", "system_success"
                )
            })
        except Exception as exc:
            self.recording_errors.append(f"{type(exc).__name__}: {exc}")
        result["recording_errors"] = list(self.recording_errors)
        result["receive_recording_errors"] = list(getattr(manipulation, "recording_errors", []))
        return result

    def _record(self, method, *args, **data):
        try:
            getattr(self.recorder, method)(*args, **data)
        except Exception as exc:
            self.recording_errors.append(f"{type(exc).__name__}: {exc}")

    def _run(self, manipulation, context, preparation):
        session = getattr(manipulation, "session", None)
        missing = []
        if session is None:
            missing.append("ReceiveManipulation.session")
        else:
            if getattr(session, "pid", None) != os.getpid():
                missing.append("ReceiveSession.pid == current PID")
            if getattr(session, "session_id", None) != self.recorder.id:
                missing.append("ReceiveSession.session_id == Episode.id")
        for name in ("coordinate", "release", "place", "stow"):
            if not callable(getattr(manipulation, name, None)):
                missing.append(f"ReceiveManipulation.{name}")
        reference_missing = getattr(manipulation, "reference_missing", None)
        if callable(reference_missing):
            missing.extend(reference_missing())
        if missing:
            return self._result("NOT_READY", "prepared receive session does not belong to this episode",
                                missing=missing, commands_sent=False, preparation=preparation)

        manager = tm.TaskManager(
            self.perception, tm.StationaryBase(), manipulation,
            tm.NoCleaner(), tm.PowerExcluded(),
        )
        prefix = {
            "prefix": "PICK observed externally in this same recorded session; TaskManager did not perform it",
            "episode_id": self.recorder.id,
            "executor_pid": os.getpid(),
            "source_kind": (getattr(session, "state", {}) or {}).get("source_kind"),
            "lift_scene_supplied": isinstance(context.get("scene"), dict),
            "lift_evidence_supplied": isinstance(context.get("evidence"), dict),
            "snapshot_role": "planning input only; not live authorization",
        }
        manager.resume(
            self.task, tm.State.COORDINATE, None,
            prefix["prefix"], manipulation.kind, prefix,
        )
        recorded_events = 0
        stages = []
        object_owner = "LEFT"
        self._record_manager_events(manager, recorded_events)
        recorded_events = len(manager.events)

        terminal = (tm.State.FAILED, tm.State.CANCELLED, tm.State.DONE, tm.State.CHARGING, tm.State.IDLE)
        while manager.state not in terminal and not manager.stopped_at_request:
            stage = manager.state
            owner = self._phase_owner(manipulation, stage)
            self._record("mark", stage.value, skill=self._skill(stage), owner=owner,
                               task_id=self.task.task_id)
            self.wait_next_cycle()
            manager.step()
            self._record_manager_events(manager, recorded_events)
            recorded_events = len(manager.events)

            if stage in (tm.State.COORDINATE, tm.State.RELEASE, tm.State.PLACE, tm.State.ARM_STOW):
                current = getattr(manipulation, "last_result", None)
                if current is None:
                    raise RuntimeError("ReceiveManipulation did not retain its current CommandResult")
                evidence = current.evidence if isinstance(current.evidence, dict) else {}
                actual_owner = self._phase_owner(manipulation, stage)
                stages.append({
                    "stage": stage.value,
                    "status": current.status.value,
                    "evidence_status": evidence.get("status"),
                    "owner": actual_owner,
                    "missing": list(evidence.get("missing") or []),
                })
                self._record("event", "full_episode_stage", **stages[-1])
                if stage == tm.State.RELEASE and evidence.get("right_support_confirmed") is True:
                    object_owner = "BOTH"
                if evidence.get("status") == "NOT_READY":
                    untouched = stage == tm.State.COORDINATE and not getattr(session, "proposals", []) and evidence.get("dispatch") is None
                    return self._result(
                        "NOT_READY", current.detail or "runtime input or reference is not ready",
                        missing=stages[-1]["missing"], commands_sent=False if untouched else None,
                        object_owner="LEFT" if untouched else object_owner,
                        preparation=preparation, stages=stages, task_manager=manager.report(),
                        source_kind=prefix["source_kind"], system_success=manager.system_success,
                    )

            if stage == tm.State.COORDINATE and manager.state == tm.State.RELEASE:
                object_owner = "LEFT"
            elif stage == tm.State.RELEASE and manager.state == tm.State.PLACE:
                object_owner = "RIGHT"
            elif stage == tm.State.VERIFY_PLACE and manager.state == tm.State.ARM_STOW:
                object_owner = "PLACED"

            if manager.state == tm.State.FAILED:
                return self._result(
                    "FAILED", manager.failure or "TaskManager failed", commands_sent=None,
                    object_owner=object_owner, preparation=preparation, stages=stages,
                    task_manager=manager.report(), source_kind=prefix["source_kind"],
                    system_success=manager.system_success,
                )

        if manager.state == tm.State.CLEAN_RESUME_REQUEST and manager.request_emitted and manager.stopped_at_request:
            self._record("event",
                "clean_resume_request", task_id=self.task.task_id,
                state=manager.state.value, cleaning_succeeded=False,
            )
            return self._result(
                "DONE", "CLEAN_RESUME_REQUEST emitted; no cleaning device was run",
                commands_sent=None, object_owner=object_owner, preparation=preparation,
                stages=stages, task_manager=manager.report(), source_kind=prefix["source_kind"],
                system_success=manager.system_success, clean_resume_request_emitted=True,
                cleaning_succeeded=False,
            )
        return self._result(
            "FAILED", f"TaskManager stopped at {manager.state.value} without a clean-resume request",
            commands_sent=None, object_owner=object_owner, preparation=preparation,
            stages=stages, task_manager=manager.report(), source_kind=prefix["source_kind"],
            system_success=manager.system_success,
        )

    def _record_manager_events(self, manager, start):
        for event in manager.events[start:]:
            self._record("event", "task_manager_event", transition=asdict(event))

    def _phase_owner(self, manipulation, state):
        session = manipulation.session
        if state == tm.State.COORDINATE:
            arbiter = getattr(session, "arbiter", None)
            return getattr(arbiter, "name", None) or "ReceiveSession Arbiter"
        if state == tm.State.RELEASE:
            arbiter = getattr(session, "arbiter", None)
            if getattr(session, "phase", None) in ("RELEASE", "LOAD_TRANSFER"):
                return getattr(arbiter, "name", None) or getattr(session, "corrector_name", "release reference")
            return getattr(session, "corrector_name", "release reference")
        if state in (tm.State.PLACE, tm.State.ARM_STOW):
            return type(manipulation.downstream).__name__
        if state in (tm.State.VERIFY_PLACE, tm.State.REOBSERVE_CLEAR):
            return type(self.perception).__name__
        if state == tm.State.CLEAN_RESUME_REQUEST:
            return "NoCleaner"
        return type(manipulation).__name__

    @staticmethod
    def _skill(state):
        return {
            tm.State.COORDINATE: "RECEIVE",
            tm.State.RELEASE: "RELEASE",
            tm.State.PLACE: "PLACE",
            tm.State.VERIFY_PLACE: "PLACE",
            tm.State.ARM_STOW: "STOW",
            tm.State.REOBSERVE_CLEAR: "OBSERVE",
            tm.State.CLEAN_RESUME_REQUEST: "OBSERVE",
        }.get(state, state.value)

    @staticmethod
    def _preparation(manipulation):
        state = getattr(manipulation.session, "state", {}) or {}
        return {
            "status": state.get("status"),
            "missing": list(state.get("missing") or []),
            "real_execution_ready": False,
            "motor_commands_sent": 0,
            "session_id": getattr(manipulation.session, "session_id", None),
        }

    def _result(self, result, why, *, missing=None, commands_sent=None, object_owner="LEFT",
                preparation=None, stages=None, task_manager=None, source_kind=None,
                system_success=False, clean_resume_request_emitted=False,
                cleaning_succeeded=False):
        stats = {}
        if callable(self.execution_stats):
            try:
                stats = self.execution_stats()
                if not isinstance(stats, dict):
                    raise TypeError("execution_stats must return the measured writer ledger")
            except Exception as exc:
                self.recording_errors.append(f"execution ledger: {type(exc).__name__}: {exc}")
                stats = {}
        return {
            "result": result,
            "why": why,
            "missing": list(missing or []),
            "commands_sent": commands_sent,
            "object_owner": object_owner,
            "preparation": preparation,
            "stages": list(stages or []),
            "task_manager": task_manager,
            "source_kind": source_kind,
            "system_success": bool(system_success),
            "clean_resume_request_emitted": bool(clean_resume_request_emitted),
            "cleaning_succeeded": bool(cleaning_succeeded),
            "real_execution_ready": False,
            "motor_commands_sent": stats.get("motor_commands_sent"),
            "independent_ack_count": stats.get("independent_ack_count"),
            "writer_ledger_source": stats.get("source"),
            "writer_ledger_scope": stats.get("scope"),
            "final_verification_owner": "TaskManager",
        }
