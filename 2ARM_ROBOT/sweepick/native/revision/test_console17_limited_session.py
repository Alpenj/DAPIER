"""Console-17 Fake Native runs of LEFT_START_TO_LIFT through the REAL session(), stage(), planner, executors (run/grip) and
judgements of the PICK02 fixture. Replaced: the device (fake bus), the collision answers (stubs) and the camera (image model).
These are Fake results: they check what is commanded and recorded, not what a real arm does."""
import json
from pathlib import Path
import pytest
import sweepick_pick01 as p1, sweepick_move_a as mv
from test_sweepick_pick02_actual import base, cell, S, trail, all_off, grip_writes   # noqa: F401  (fixture)

L = p1.LEFT_TO_LIFT
AFTER_LIFT = ("lower", "open", "retreat", "home", "retreat_arm", "home_arm")


def counted(stage_fn, fake, log):
    """the same stage function; records how many bus writes existed after each stage"""
    def run(o, name, execute=False):
        r = stage_fn(o, name, execute=execute); log.append((name, len(fake.writes), r.get("controller_result"))); return r
    return run


def limited(out, stage_fn, w_look, **k):
    return p1.session(out, execute=True, stage_fn=stage_fn, look_fn=w_look, observe_fn=lambda o: S(out), limit=L, **k)


def test_held_lift_ends_at_the_boundary_with_no_further_write(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075); log = []
    w = t.World(base, f)
    r = limited(out, counted(stage_fn, f, log), lambda o, tag: w.scene(tag))
    order = S(out)["order"]
    assert r["result"] == "LEFT_LIFT_HELD_CONFIRMED_HOLDING_NEEDS_A_PERSON" and r["limited_terminal"] == "LEFT_LIFT_HELD_CONFIRMED", (r["result"], order)
    assert order[-1] == "lift" and not set(order) & set(AFTER_LIFT) and log[-1][0] == "lift" and log[-1][1] == len(f.writes)   # 0 writes after LIFT
    assert not all_off(f) and f.reg["Torque_Enable"]["gripper"] == 1                    # still powered and holding: nothing released or switched off
    assert r["normal_home"] is False and r["torque_off_confirmed"] is False and r["safe_shutdown"] == "NOT_PERFORMED" and r["operator_required"] is True
    assert r["boundary"] == dict(after="held_after_lift", next_stage_commanded=None, handoff_entered=False, put_back_entered=False, torque_writes_after_lift=0)
    assert r["hold_evidence"]["ok"] is True and r["last_lift"]["controller_result"] == "HOLDING" and r["last_lift"]["end"]["torque"]["gripper"] == 1
    assert [x["step"] for x in trail(out)][-1] == "limited_scope_terminal" and not trail(out, "handoff")


def test_baseline_without_the_limit_is_unchanged(tmp_path, monkeypatch, base):
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075)
    r = run(); assert r["result"] == "PICKED_LIFTED_AND_PUT_BACK" and S(out)["order"][-4:] == ["lower", "open", "retreat", "home"] and all_off(f)


def test_lift_not_confirmed_is_hold_unknown_and_does_not_go_home(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, over={"04": lambda o, tp, d, g: (dict(visible=False, stable=False), tp, dict(t.DESK, centre_xy=base["scene"]["object"]["centre_xy"]), "OCCUPIED")}, block=3075)
    w = t.World(base, f, **{"04": lambda o, tp, d, g: (dict(visible=False, stable=False), tp, dict(t.DESK, centre_xy=base["scene"]["object"]["centre_xy"]), "OCCUPIED")}); log = []
    r = limited(out, counted(stage_fn, f, log), lambda o, tag: w.scene(tag))
    assert r["limited_terminal"] == "LEFT_LIFT_HOLD_UNKNOWN" and r["held_after_lift"] is False and S(out)["order"][-1] == "lift" and log[-1][1] == len(f.writes) and not all_off(f)


def test_no_contact_close_keeps_the_existing_way_back_and_never_lifts(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base); w = t.World(base, f)
    r = limited(out, stage_fn, lambda o, tag: w.scene(tag))
    assert r["result"] == "NO_CONTACT_RETURNED_HOME" and "lift" not in S(out)["stages"] and S(out)["order"][-4:] == ["close", "open", "retreat", "home"]
    assert p1.limited_terminal(r, S(out)) == "ENDED_BEFORE_LIFT"


def test_close_fault_ends_there_as_bus_or_servo_fault(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075, fault_from=30); w = t.World(base, f)
    r = limited(out, stage_fn, lambda o, tag: w.scene(tag))
    assert r["result"] == "STOPPED_IN_CLOSE_HOLDING_NEEDS_A_PERSON" and S(out)["order"][-1] == "close" and p1.limited_terminal(r, S(out)) == "BUS_OR_SERVO_FAULT"
    assert stage_fn(out, "lift", execute=True)["result"] == "NOT_RUN"


def test_grip_unavailable_is_not_contact_and_forbids_lift(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075); w = t.World(base, f)
    real_grip = mv.grip
    def unavailable(side, o, **k):
        code, log = real_grip(side, o, **dict(k, execute=False))                    # plan only: no write; then the executor's own UNAVAILABLE result
        log.update(result="GRIP_UNAVAILABLE_HOLDING"); return mv.EXIT["GRIP_UNAVAILABLE_HOLDING"], log
    sf = lambda o, name, execute=False: (p1.stage(o, name, execute=execute, grip=unavailable, cert=lambda a, b, c: dict(identity=json.loads(Path(a).read_text())["identity"], nominal_verdict="CLEAR"),
                                                   run_kw=stage_fn.__closure__ and _kw(stage_fn), torque_now=lambda: f.reg["Torque_Enable"]) if name == "close" else stage_fn(o, name, execute=execute))
    r = limited(out, sf, lambda o, tag: w.scene(tag))
    assert r["result"] == "STOPPED_IN_CLOSE_HOLDING_NEEDS_A_PERSON" and "lift" not in S(out)["stages"] and p1.limited_terminal(r, S(out)) == "LEFT_CLOSE_UNAVAILABLE"
    assert p1.allowed(S(out), "lift")[0] is False


def _kw(stage_fn):
    """the run_kw bound inside the fixture's stage function"""
    for c in stage_fn.__closure__:
        v = c.cell_contents
        if isinstance(v, dict) and "open_bus" in v:
            return v
    raise AssertionError("fixture run_kw not found")


def test_operator_stop_during_lift_ends_without_another_stage(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075); w = t.World(base, f); seen = []
    write = f.write
    def stop_after_some_lift_goals(reg, n, v, normalize=True):
        write(reg, n, v, normalize)
        if seen and reg == "Goal_Position":
            seen[0] += 1
            if seen[0] == 12: Path(str(out) + ".STOP").touch()                # the operator's STOP arrives while LIFT is moving
    f.write = stop_after_some_lift_goals
    def sf(o, name, execute=False):
        if name == "lift": seen.append(0)
        return stage_fn(o, name, execute=execute)
    r = limited(out, sf, lambda o, tag: w.scene(tag))
    st = S(out); lift = st["stages"]["lift"]
    assert r["result"] == "STOPPED_IN_LIFT_HOLDING_NEEDS_A_PERSON" and st["order"][-1] == "lift" and lift["stopped_by"]["gate"] == "operator_stop", (r["result"], st["order"], lift.get("controller_result"))
    assert p1.limited_terminal(r, st, stop_requested=True) == "OPERATOR_STOPPED" and lift["writes"]["Torque_Enable"] == 0 and not set(st["order"]) & set(AFTER_LIFT)


def test_stop_before_lift_is_refused_without_a_write(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075); w = t.World(base, f)
    def sf(o, name, execute=False):
        if name == "lift": Path(str(out) + ".STOP").touch()
        return stage_fn(o, name, execute=execute)
    n0 = []
    r = limited(out, sf, lambda o, tag: w.scene(tag)); st = S(out)
    assert "lift" not in st["order"] and st["refusals"][-1]["stage"] == "lift" and st["refusals"][-1]["writes"] == dict(Goal_Position=0, Torque_Enable=0)
    assert p1.limited_terminal(r, st, stop_requested=True) == "OPERATOR_STOPPED"


def test_a_limited_session_rejects_a_hand_over_or_full_chain(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075); w = t.World(base, f)
    class H:
        def ready(self, *a): return dict(ready=True)
        def __call__(self, ctx): raise AssertionError("never entered")
    with pytest.raises(ValueError):
        p1.session(out, execute=True, stage_fn=stage_fn, look_fn=lambda o, tag: w.scene(tag), observe_fn=lambda o: S(out), limit=L, handoff_fn=H())
    assert f.writes == []
    for kw in (dict(full_chain=True), dict(handoff=H()), dict(record=False), dict(limit="LEFT_LIFT_AND_CARRY")):
        with pytest.raises(ValueError):
            p1.run_session(tmp_path / f"rs{len(kw)}{list(kw)[0]}", execute=True, **dict(dict(record=True, limit=L), **kw))


class Recorder:
    """a fake recorder with the Episode interface run_session uses; it opens nothing"""
    def __init__(self, folder): self.leases, self.events, self.closed, self.owned, self.dir = [], [], None, [], Path(folder) / "episode"; self.dir.mkdir(parents=True, exist_ok=True)
    def mark(self, *a, **k): self.events.append(("mark", a))
    def event(self, kind, **k): self.events.append((kind, k))
    def stage(self, *a): self.events.append(("stage", a))
    def lease(self, *sides):
        import contextlib
        self.leases.append(sides); return contextlib.nullcontext()
    def close(self, **k):
        self.closed = k; return dict(session_id="fake-episode", cameras={}, joints={}, data_quality={}, task_outcome=k.get("task_outcome"), source_kind="FAKE")


def test_run_session_writes_the_terminal_after_the_recorder_is_released(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075); w = t.World(base, f); rec = Recorder(tmp_path)
    code = p1.run_session(out, execute=True, record=True, limit=L, episode_fn=lambda: rec,
                          session_kw=dict(stage_fn=stage_fn, look_fn=lambda o, tag: w.scene(tag), observe_fn=lambda o: S(out)))
    doc = json.loads((out / "LIMITED_SCOPE_TERMINAL.json").read_text())
    assert code == 0 and doc["terminal"] == "LEFT_LIFT_HELD_CONFIRMED" and doc["stages_after_lift_commanded"] == [] and rec.closed["task_outcome"].endswith("LEFT_LIFT_HELD_CONFIRMED_HOLDING_NEEDS_A_PERSON")
    assert doc["owner"]["recorder"]["released"] is True and doc["owner"]["recorder"]["session_id"] == "fake-episode" and p1.EPISODE is None
    assert doc["normal_home"] is False and doc["torque_off_confirmed"] is False and doc["safe_shutdown"] == "NOT_PERFORMED" and doc["motion_stop_confirmed"] == "UNKNOWN"
    assert doc["operator_required"] is True and doc["next_run_allowed"] is False and doc["last_actual"]["torque"]["gripper"] == 1 and doc["last_goal_ack"]["move_files"] == ["lift/run1/move.json"]
    assert ("left", "right") in rec.leases and "fake-episode" == doc["owner"]["recorder"]["session_id"]
    lift_move = json.loads((out / "lift/run1/move.json").read_text())
    assert lift_move["last_acknowledged_goal"] and lift_move["end"]["torque"]["gripper"] == 1


def test_run_session_failure_terminal_has_no_success_claim(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075, fault_from=30); w = t.World(base, f); rec = Recorder(tmp_path)
    code = p1.run_session(out, execute=True, record=True, limit=L, episode_fn=lambda: rec,
                          session_kw=dict(stage_fn=stage_fn, look_fn=lambda o, tag: w.scene(tag), observe_fn=lambda o: S(out)))
    doc = json.loads((out / "LIMITED_SCOPE_TERMINAL.json").read_text())
    assert code == 3 and doc["terminal"] == "BUS_OR_SERVO_FAULT" and doc["last_stage"]["name"] == "close" and doc["operator_required"] is True and doc["torque_off_confirmed"] is False


def test_overload_0x20_during_close_ends_as_a_fault_without_lift(tmp_path, monkeypatch, base):
    """the 2026-10-15 overload reached the host as a status-packet error on a read; injected here on the fake bus"""
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075); w = t.World(base, f); armed = []
    read = f.sync_read
    def overloaded(reg, normalize=True):
        if armed and len(armed) > 0:
            armed[0] += 1
            if armed[0] == 25:
                raise RuntimeError("servo response error: ID=6 error=0x20 [RxPacketError] Overload error!")
        return read(reg, normalize)
    f.sync_read = overloaded
    def sf(o, name, execute=False):
        if name == "close": armed.append(0)
        return stage_fn(o, name, execute=execute)
    rec = Recorder(tmp_path)
    code = p1.run_session(out, execute=True, record=True, limit=L, episode_fn=lambda: rec,
                          session_kw=dict(stage_fn=sf, look_fn=lambda o, tag: w.scene(tag), observe_fn=lambda o: S(out)))
    st = S(out); doc = json.loads((out / "LIMITED_SCOPE_TERMINAL.json").read_text())
    assert st["stages"]["close"]["controller_result"] == "BUS_LOST" and "lift" not in st["stages"], st["stages"]["close"]["controller_result"]
    assert code == 3 and doc["terminal"] == "BUS_OR_SERVO_FAULT" and doc["operator_required"] is True and doc["torque_off_confirmed"] is False
    assert all(p1.allowed(st, n)[0] is False for n in ("lift", "open", "retreat", "home"))


def test_stopped_state_recorded_forbids_any_further_stage(tmp_path, monkeypatch, base):
    import test_sweepick_pick02_actual as t
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075); w = t.World(base, f)
    real_grip = mv.grip
    def stopped(side, o, **k):
        code, log = real_grip(side, o, **dict(k, execute=False))
        log.update(result="STOPPED_STATE_RECORDED", writes=dict(Goal_Position=3, Torque_Enable=1)); return mv.EXIT["STOPPED_STATE_RECORDED"], log
    sf = lambda o, name, execute=False: (p1.stage(o, name, execute=execute, grip=stopped, cert=lambda a, b, c: dict(identity=json.loads(Path(a).read_text())["identity"], nominal_verdict="CLEAR"),
                                                   run_kw=_kw(stage_fn), torque_now=lambda: f.reg["Torque_Enable"]) if name == "close" else stage_fn(o, name, execute=execute))
    r = limited(out, sf, lambda o, tag: w.scene(tag)); st = S(out)
    assert r["result"] == "STOPPED_IN_CLOSE_HOLDING_NEEDS_A_PERSON" and st["order"][-1] == "close" and st["stages"]["close"]["controller_result"] == "STOPPED_STATE_RECORDED"
    assert all(p1.allowed(st, n)[0] is False for n in ("lift", "open", "retreat", "home", "retreat_arm", "home_arm"))
    assert p1.limited_terminal(r, st) == "ENDED_BEFORE_LIFT"
