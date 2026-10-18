"""prepare (2026-10-18): a session brings its arm to the start pose itself (fake bus, no device). Only the joints that are off
are commanded; the arm is released afterwards (confirmed by a read); a path that touches something, torque already on, or a
fault refuses before any write."""
import json
from pathlib import Path
import pytest
import sweepick_move_a as mv, sweepick_real_command as rc
from test_sweepick_move_a import Fake, Clock, IN

pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")


@pytest.fixture
def p1():
    import sweepick_pick01 as p
    return p


def _setup(p1, side, ticks):
    sp = json.loads(p1.START_POSE.read_text())[side]
    start = dict(sp["arm"], gripper=sp["gripper_open_tick"]); start.update(ticks)
    f = Fake({n: float(v) for n, v in start.items()}); c = Clock()
    mp = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")
    run = lambda mode, s, out, **kw: mv.run(mode, s, out, open_bus=lambda port: f, owner=lambda p: "", mapping=mp, mapping_sha="m", sleep=c.sleep, clock=c.now, other_arm_read=lambda: dict(start), **kw)
    other = {n: 2048.0 for n in rc.JOINTS}
    reads = lambda: {side: dict(ticks=dict(f.reg["Present_Position"]), torque=dict(f.reg["Torque_Enable"]), status=dict(f.reg["Status"])),
                     p1.other(side): dict(ticks=other, torque={n: 0 for n in rc.JOINTS}, status={n: 0 for n in rc.JOINTS})}
    clear = lambda a, b, c_: dict(identity=json.loads(Path(a).read_text())["identity"], nominal_verdict="CLEAR")
    scene = lambda folder, now: dict(ok=True, object=dict(centre_world=[0.2, -0.08, 0.02], yaw_rad=0.0))
    return f, run, reads, clear, scene, sp


@pytest.mark.parametrize("side", ["right", "left"])
def test_already_at_the_start_pose_does_nothing(tmp_path, p1, side):
    f, run, reads, clear, scene, sp = _setup(p1, side, {})
    r = p1.prepare(tmp_path, side, execute=True, run=run, cert=clear, reads=reads, scene_of=scene)
    assert r["result"] == "ALREADY_AT_START" and f.writes == []


def test_a_closed_gripper_is_opened_alone_and_released(tmp_path, p1):
    f, run, reads, clear, scene, sp = _setup(p1, "left", dict(gripper=3200.0))
    plan = p1.prepare(tmp_path / "plan", "left", execute=False, run=run, cert=clear, reads=reads, scene_of=scene)
    assert plan["result"] == "PLANNED" and plan["target"] == dict(gripper=float(sp["gripper_open_tick"])) and f.writes == []
    r = p1.prepare(tmp_path / "run", "left", execute=True, run=run, cert=clear, reads=reads, scene_of=scene)
    assert r["result"] == "RAN" and r["controller_result"] == "RETURNED_RELEASED" and r["at_start_pose"]
    assert {j for _, j, _ in f.writes} == {"gripper"} and not any(f.reg["Torque_Enable"].values())


def test_an_arm_out_of_its_rest_is_folded_with_all_its_joints_powered_then_released(tmp_path, p1):
    f, run, reads, clear, scene, sp = _setup(p1, "left", dict(elbow_flex=2600.0))
    r = p1.prepare(tmp_path, "left", execute=True, run=run, cert=clear, reads=reads, scene_of=scene)
    assert set(r["target"]) == set(p1.ARM) and r["result"] == "RAN" and r["controller_result"] == "RETURNED_RELEASED"
    assert abs(f.reg["Present_Position"]["elbow_flex"] - sp["arm"]["elbow_flex"]) <= 2 and not any(f.reg["Torque_Enable"].values())


def test_a_path_that_touches_something_writes_nothing(tmp_path, p1):
    f, run, reads, clear, scene, sp = _setup(p1, "left", dict(gripper=3200.0))
    hit = lambda a, b, c_: dict(identity=json.loads(Path(a).read_text())["identity"], nominal_verdict="CONTACT", first_nominal_contact="right_pad_1 - red_block")
    r = p1.prepare(tmp_path, "left", execute=True, run=run, cert=hit, reads=reads, scene_of=scene)
    assert r["result"] == "NOT_RUN" and "touches" in r["why"] and f.writes == []


def test_torque_on_or_a_fault_refuses_before_any_write(tmp_path, p1):
    f, run, reads, clear, scene, sp = _setup(p1, "left", dict(gripper=3200.0))
    f.reg["Torque_Enable"]["wrist_roll"] = 1
    assert p1.prepare(tmp_path / "a", "left", execute=True, run=run, cert=clear, reads=reads, scene_of=scene)["result"] == "NOT_RUN"
    f.reg["Torque_Enable"]["wrist_roll"] = 0; f.reg["Status"]["elbow_flex"] = 32
    r = p1.prepare(tmp_path / "b", "left", execute=True, run=run, cert=clear, reads=reads, scene_of=scene)
    assert r["result"] == "NOT_RUN" and "fault" in r["why"] and f.writes == []


@pytest.mark.parametrize("side", ["right", "left"])
def test_the_start_pose_lies_inside_every_servo_range(p1, side):
    sp = json.loads(p1.START_POSE.read_text())[side]; mp = p1.mapping()[side]
    for n, v in dict(sp["arm"], gripper=sp["gripper_open_tick"]).items():
        assert mp[n]["lo"] <= v <= mp[n]["hi"], (side, n, v, mp[n]["lo"], mp[n]["hi"])


def test_a_refusal_before_any_write_is_not_run(tmp_path, p1):
    f, run, reads, clear, scene, sp = _setup(p1, "left", dict(gripper=3200.0))
    refusing = lambda mode, s, out, **kw: run(mode, s, out, **kw) if mode == "plan" else (2, dict(result="REFUSED", writes=dict(Goal_Position=0, Torque_Enable=0), refused=dict(gate="x")))
    r = p1.prepare(tmp_path, "left", execute=True, run=refusing, cert=clear, reads=reads, scene_of=scene)
    assert r["result"] == "NOT_RUN" and f.writes == []
