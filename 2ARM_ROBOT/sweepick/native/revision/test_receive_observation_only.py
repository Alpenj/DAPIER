import copy
import json
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import pytest

import current_view_candidate
import sweepick_full_chain as fc


def static_view_owner(tmp_path, *, transient=None, persistent=None, verdict="CLEAR"):
    state = {key: {side: {name: 0 for name in fc.ALL} for side in ("left", "right")}
             for key in ("ticks", "goal", "torque", "status", "moving", "velocity")}
    for side in ("left", "right"):
        state["ticks"][side] = dict.fromkeys(fc.ALL, 2000)
        state["goal"][side] = dict.fromkeys(fc.ALL, 2000)
        state["torque"][side] = dict.fromkeys(fc.ALL, 1)
    owner = fc.Owner.__new__(fc.Owner)
    owner.scope, owner.stopped, owner.out, owner.k = "contact", None, tmp_path, 0
    owner.contact_plan, owner.run_kw, owner.notes = {"observation_only": True}, {}, []
    elapsed, reads, checks = [0.0], [0], []
    owner.clock = lambda: elapsed[0]
    owner.sleep = lambda seconds: elapsed.__setitem__(0, elapsed[0] + seconds)
    owner.idle_limit_s = 20.0
    owner.note = lambda kind, **values: owner.notes.append({"kind": kind, **values})
    owner.track_held = lambda raw: None
    owner.run = lambda *args, **kwargs: pytest.fail("static view must not send a motor command")

    def read():
        reads[0] += 1
        row = copy.deepcopy(state)
        if transient == "moving" and reads[0] == 1:
            row["moving"]["right"]["wrist_roll"] = 1
        if transient == "drift" and reads[0] > 1:
            row["ticks"]["right"]["wrist_roll"] += 2
        if persistent and reads[0] > 1:
            row[persistent]["right"]["wrist_roll"] = (
                0 if persistent == "torque" else 2001 if persistent == "goal" else 1)
        return row

    def check(doc, side, raw):
        checks.append(copy.deepcopy(raw))
        return {"nominal_verdict": verdict, "samples_required": max(1, doc["trajectory"]["steps"]),
                "nominal_steps_checked": max(1, doc["trajectory"]["steps"])}

    owner._read, owner._check = read, check
    owner._write_check = lambda folder, tag, checked: checked
    return owner, checks


@pytest.mark.parametrize("transient", ["moving", "drift"])
def test_static_view_uses_recent_stationary_window_after_initial_transient(tmp_path, transient):
    owner, checks = static_view_owner(tmp_path, transient=transient)
    result = owner.segment("view", "right", target={"wrist_roll": 2000}, purpose="OBSERVATION_VIEW")
    assert result["wrote"] is False and result["observation_view"]["actual_settled"] is True
    actual = 2002 if transient == "drift" else 2000
    assert result["end"]["ticks"]["wrist_roll"] == actual
    assert checks[-1]["ticks"]["right"]["wrist_roll"] == actual
    samples = json.loads((Path(result["folder"]) / "STATIC_VIEW_READS.json").read_text())
    assert samples[0]["ticks"]["right"]["wrist_roll"] == 2000
    assert len(samples) > 15


@pytest.mark.parametrize("persistent", ["moving", "velocity", "status", "goal", "torque"])
def test_static_view_does_not_accept_unresolved_or_unhealthy_feedback(tmp_path, persistent):
    owner, checks = static_view_owner(tmp_path, persistent=persistent)
    with pytest.raises(RuntimeError, match="not settled/resolved/healthy|servo fault"):
        owner.segment("view", "right", target={"wrist_roll": 2000}, purpose="OBSERVATION_VIEW")
    assert checks == []


@pytest.mark.parametrize("verdict", ["CONTACT", "UNKNOWN"])
def test_static_view_keeps_current_pose_collision_gate(tmp_path, verdict):
    owner, checks = static_view_owner(tmp_path, verdict=verdict)
    with pytest.raises(RuntimeError, match="not established CLEAR"):
        owner.segment("view", "right", target={"wrist_roll": 2000}, purpose="OBSERVATION_VIEW")
    assert len(checks) == 1


def observation_owner(tmp_path, monkeypatch, *, authorization=True, insertion_ready=True):
    owner = fc.Owner.__new__(fc.Owner)
    owner.scope = "contact"
    owner.contact_plan = {"observation_only": True}
    owner.stopped = None
    owner.out = tmp_path
    owner.last_raw = {"ticks": {"left": {}, "right": {"wrist_roll": 4.0}}}
    owner.last_images = {"right_wrist": {"n": 1, "recv_monotonic_s": 1.0,
                                          "reader_valid": True, "fresh": True}}
    owner.notes = []
    calls = {name: [] for name in ("proposal", "read", "recovery", "plan", "authorize",
                                    "segment", "observation", "ready")}
    candidate = {"id": "receiver-inner-pad", "expected_observed_components": ["receiver inner pad"],
                 "targets": [{"side": "right"}]}
    plan = {"ready": True, "forward": [{"side": "right", "target_ticks": {"wrist_roll": 5.0}}]}
    raw = {"ticks": {"left": {}, "right": {"wrist_roll": 5.0}}}
    fresh = {"fresh": True, "frame": "new", "visible_face_relation": {"status": "OK"}}

    def propose(given_owner, observed, *, actual=None):
        calls["proposal"].append((given_owner, observed, actual))
        return candidate

    monkeypatch.setattr(current_view_candidate, "propose_receiver_inner_pad_view", propose, raising=False)
    owner.note = lambda kind, **values: owner.notes.append({"kind": kind, **values})
    owner.observation_deficits = lambda observation: {
        "pad_object_relation": (observation.get("visible_face_relation") or {}).get("status") != "OK",
        "held_centre": False, "held_orientation": False, "unavailable": {}}
    owner._read = lambda: (calls["read"].append(True) or copy.deepcopy(raw))
    owner.recovery_ready = lambda step, observation=None, frames=None: (
        calls["recovery"].append((step, observation)) or (True, []))
    owner.plan_observation_candidate = lambda given, actual, prefix: (
        calls["plan"].append((given, actual, prefix)) or plan)
    if authorization is not None:
        def authorize(given_plan, actual, given_candidate):
            calls["authorize"].append((given_plan, actual, given_candidate))
            if isinstance(authorization, BaseException):
                raise authorization
            return authorization
        owner.authorize_observation_plan = authorize
    owner.segment = lambda name, side, **kwargs: (
        calls["segment"].append((name, side, kwargs)) or
        {"result": "HOLDING", "observation_view": {"actual_settled": True}})
    owner.track_held = lambda actual: None
    owner.clock = lambda: 2.0

    def contact_observation(stage, **kwargs):
        calls["observation"].append((stage, kwargs))
        owner.last_images = {"right_wrist": {"n": 2, "recv_monotonic_s": 2.0,
                                              "reader_valid": True, "fresh": True}}
        owner.last_raw = copy.deepcopy(raw)
        return fresh, copy.deepcopy(raw), owner.last_images

    owner.contact_observation = contact_observation
    owner.insertion_ready = lambda observed, **kwargs: (
        calls["ready"].append((observed, kwargs)) or insertion_ready)
    return owner, calls, fresh


@pytest.mark.parametrize("authorization", [None, False])
def test_observation_view_requires_explicit_true_authorization(tmp_path, monkeypatch, authorization):
    owner, calls, _ = observation_owner(tmp_path, monkeypatch, authorization=authorization)
    observed = {"visible_face_relation": {"status": "MISSING"}}
    result = owner.acquire_insertion_observation(observed, {}, lambda *_: None, [], stage="PLAN")
    assert result[2] is False
    assert len(calls["proposal"]) == len(calls["read"]) == len(calls["recovery"]) == len(calls["plan"]) == 1
    assert calls["segment"] == calls["observation"] == calls["ready"] == []


def test_authorization_exception_is_not_treated_as_a_safe_decline(tmp_path, monkeypatch):
    owner, calls, _ = observation_owner(
        tmp_path, monkeypatch, authorization=RuntimeError("approval failed"))
    with pytest.raises(RuntimeError, match="approval failed"):
        owner.acquire_insertion_observation(
            {"visible_face_relation": {"status": "MISSING"}}, {}, lambda *_: None, [], stage="PLAN")
    assert calls["segment"] == calls["observation"] == calls["ready"] == []
    assert any(note["kind"] == "observation_plan_authorization_failed" for note in owner.notes)


def test_approved_view_consumes_one_fresh_observation(tmp_path, monkeypatch):
    owner, calls, fresh = observation_owner(tmp_path, monkeypatch)
    stages = []
    observed, raw, ready = owner.acquire_insertion_observation(
        {"visible_face_relation": {"status": "MISSING"}}, {}, lambda *_: None, stages, stage="PLAN")
    assert ready is True and observed is fresh
    assert len(calls["proposal"]) == len(calls["authorize"]) == len(calls["segment"]) == 1
    assert calls["proposal"][0][2] is calls["plan"][0][1]
    assert [call[0] for call in calls["observation"]] == ["OBSERVATION_CANDIDATE"]
    assert calls["ready"] == [(fresh, {"stage": "PLAN"})]
    assert stages == [{"stage": "OBSERVATION_VIEW", "candidate_index": 0,
                       "side": "right", "result": "HOLDING"}]
    consumed = [note for note in owner.notes if note["kind"] == "observation_view_actual_consumed"]
    assert len(consumed) == 1 and consumed[0]["frame_identity"]["right_wrist"]["n"] == 2
    assert raw["ticks"]["right"]["wrist_roll"] == 5.0


def test_insufficient_approved_view_ends_after_one_candidate(tmp_path, monkeypatch):
    owner, calls, _ = observation_owner(tmp_path, monkeypatch, insertion_ready=False)
    observed = {"visible_face_relation": {"status": "MISSING"}}
    first = owner.acquire_insertion_observation(observed, {}, lambda *_: None, [], stage="PLAN")
    second = owner.acquire_insertion_observation(first[0], {}, lambda *_: None, [], stage="PLAN")
    assert first[2] is second[2] is False
    assert len(calls["proposal"]) == len(calls["segment"]) == len(calls["observation"]) == 1
    assert any(note["kind"] == "observation_candidates_exhausted" and "already consumed" in note["why"]
               for note in owner.notes)


def test_stop_prevents_proposal_recovery_and_motion(tmp_path, monkeypatch):
    owner, calls, _ = observation_owner(tmp_path, monkeypatch)
    Path(str(tmp_path) + ".STOP").write_text("stop")
    with pytest.raises(RuntimeError, match="must not move or recover"):
        owner.acquire_insertion_observation(
            {"visible_face_relation": {"status": "MISSING"}}, {}, lambda *_: None, [], stage="PLAN")
    assert calls["proposal"] == calls["read"] == calls["recovery"] == calls["segment"] == []


def test_ended_reader_does_not_request_a_view_move(tmp_path, monkeypatch):
    owner, calls, _ = observation_owner(tmp_path, monkeypatch)
    observed = {"observation_missing": {"right_wrist": "reader reported an error: capture.read returned no frame"}}
    result = owner.acquire_insertion_observation(observed, {}, lambda *_: None, [], stage="PLAN")
    assert result[2] is False
    assert all(not calls[name] for name in calls)
    assert owner.notes[-1]["kind"] == "observation_reader_failed_no_view"


def test_observation_only_permission_rejects_insert_and_close_before_read(tmp_path):
    owner = fc.Owner.__new__(fc.Owner)
    owner.scope = "contact"
    owner.contact_plan = {"observation_only": True}
    owner._read = lambda: pytest.fail("permission rejection must precede a register read")
    with pytest.raises(RuntimeError, match="observation-only"):
        owner.segment("insert", "right", target={"wrist_roll": 1.0}, purpose="RIGHT_INSERT_REPLAN")
    with pytest.raises(RuntimeError, match="observation-only"):
        owner.segment("close", "right", target={"gripper": 1.0}, purpose="RIGHT_CLOSE")
    with pytest.raises(RuntimeError, match="observation-only"):
        owner.contact_close()


@pytest.mark.parametrize("planning_ready, expected_status", [
    (True, "READY_FOR_INSERTION_PLANNING"),
    (False, "NOT_READY"),
])
def test_observation_only_scope_records_decision_and_runs_normal_reverse(
        tmp_path, planning_ready, expected_status):
    (tmp_path / "HANDOFF_SNAPSHOT.json").write_text("{}")
    state = {
        "ticks": {side: {name: 0.0 for name in fc.ALL} for side in ("left", "right")},
        "goal": {side: {name: 0.0 for name in fc.ALL} for side in ("left", "right")},
        "torque": {"left": {name: True for name in fc.ALL},
                   "right": {name: False for name in fc.ALL}},
        "status": {side: {name: 0 for name in fc.ALL} for side in ("left", "right")},
    }
    state["goal"]["left"]["gripper"] = 10.0
    state["goal"]["right"]["gripper"] = 20.0
    recorder_events = []

    class Recorder:
        @contextmanager
        def lease(self, *_):
            yield

        def event(self, kind, **values):
            recorder_events.append((kind, values))

    recorder = Recorder()
    inner = SimpleNamespace(_consumed=False, _readiness={"ready": True}, _ready_out=tmp_path,
                            recorder=recorder, recording_errors=[])
    inner.prepare = lambda snapshot, scene, evidence: {"status": "PLAN_PREVIEW"}
    owner = fc.Owner.__new__(fc.Owner)
    owner.out = tmp_path
    owner.contact_plan = {"observation_only": True}
    owner.notes = []
    owner.block_owner = "left"
    owner.stopped = None
    owner.done = set()
    owner.last_images = {}
    owner.contact_insert_history = []
    owner.contact_close_executed = False
    owner.insert = None
    owner.clock = lambda: 1.0
    owner.note = lambda kind, **values: owner.notes.append({"kind": kind, **values})
    owner._read = lambda: copy.deepcopy(state)
    owner.track_held = lambda raw: None
    reverse = [
        {"label": "RIGHT_RETURN_INITIAL", "side": "right",
         "target_ticks": {name: 0.0 for name in fc.ALL}},
        {"label": "LEFT_BACK_TO_LIFT", "side": "left",
         "target_ticks": {name: 0.0 for name in fc.ARM}},
    ]
    path = {
        "ready": True,
        "initial_ticks": {side: dict(state["ticks"][side]) for side in ("left", "right")},
        "forward": [
            {"label": "CARRY_LEFT", "side": "left", "target_q12": [1.0] * 12},
            {"label": "RIGHT_APPROACH", "side": "right", "target_q12": [2.0] * 12},
        ],
        "reverse": reverse,
    }
    owner.plan_approach_prefix = lambda plan, raw: path
    owner.approach_plan = {"status": "PLAN_PREVIEW"}

    def go(name, q12, *, arms):
        side = arms[0]
        state["ticks"][side]["wrist_roll"] += 5.0
        owner.notes.append({"kind": "segment", "name": name, "side": side,
                            "writes": {"wrist_roll": 1}})
        return {side: {"result": "HOLDING"}}

    owner.go = go
    observed = {"q12": [0.0] * 12, "visible_face_relation": {"status": "OK"}}

    def contact_observation(stage, **kwargs):
        owner.last_raw = copy.deepcopy(state)
        owner.last_images = {"right_wrist": {"n": len(owner.notes) + 1,
                                              "recv_monotonic_s": 1.0,
                                              "reader_valid": True, "fresh": True}}
        return copy.deepcopy(observed), copy.deepcopy(state), owner.last_images

    owner.contact_observation = contact_observation

    def insertion_ready(current, **kwargs):
        owner.insert = {"record": {"ready": planning_ready, "live_execution_permitted": planning_ready}}
        return planning_ready

    owner.insertion_ready = insertion_ready
    acquisition_calls = []

    def acquire(current, *args, **kwargs):
        acquisition_calls.append(current)
        if planning_ready:
            pytest.fail("already-ready must not request a view")
        return current, copy.deepcopy(state), False

    owner.acquire_insertion_observation = acquire
    normal_plans = []

    def plan_contact_suffix(raw, prefix, normal_only=False, **kwargs):
        normal_plans.append(normal_only)
        assert normal_only is True
        return {"ready": True, "reverse": reverse}

    owner.plan_contact_suffix = plan_contact_suffix
    segment_calls = []

    def segment(name, side, *, target=None, rest=None, purpose=None, **kwargs):
        segment_calls.append((name, side, purpose))
        assert purpose not in ("RIGHT_INSERT_REPLAN", "RIGHT_CLOSE")
        if name.endswith("right_return_initial"):
            state["torque"]["right"] = {joint: False for joint in fc.ALL}
            result = "RETURNED_RELEASED"
        else:
            result = "HOLDING"
        owner.notes.append({"kind": "segment", "name": name, "side": side,
                            "writes": {joint: 1 for joint in fc.ARM}})
        return {"result": result, "end": {"ticks": copy.deepcopy(state["ticks"][side])}}

    owner.segment = segment
    owner.recovery_ready = lambda *args, **kwargs: (True, [])
    owner.run_insert_hops = lambda *args, **kwargs: pytest.fail("insert hops are outside the scope")
    owner.contact_close = lambda: pytest.fail("close is outside the scope")
    owner.on_stop = lambda *args: pytest.fail("healthy normal return must not stop")
    scoped = fc.Scoped(inner, owner, "contact")
    result = scoped._approach({"out": str(tmp_path), "episode": recorder, "scene": {}, "evidence": {}})
    assert result["result"] == "RECEIVE_CONTACT_NOT_READY_RETURNED"
    assert result["end"]["returned"] is True and result["left_at_lift"] is True
    assert result["insertion"]["status"] == expected_status
    assert result["insertion"]["observation_only"] is True
    assert result["insertion"]["actual_insert_permitted"] is False
    assert result["insertion"]["actual_contact_permitted"] is False
    assert result["insertion"]["live_execution_permitted"] is False
    assert result["contact"] is None and result["suffix"] is None
    assert normal_plans == [True]
    assert len(acquisition_calls) == (0 if planning_ready else 1)
    assert [name for name, _, _ in segment_calls] == [
        "contact_right_return_initial", "contact_left_back_to_lift"]
    saved = __import__("json").loads((tmp_path / "RECEIVE_CONTACT_OBSERVATION.json").read_text())
    assert saved["insertion"]["status"] == expected_status
