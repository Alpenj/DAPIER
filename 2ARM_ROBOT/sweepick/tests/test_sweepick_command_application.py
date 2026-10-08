"""The dispatch boundary: every written value is checked, a cycle is all-or-nothing, a failed gate writes nothing."""
import json, math
from pathlib import Path
import numpy as np, pytest
from sweepick.control import sweepick_joint_command_mapping as rc; from sweepick.control import sweepick_command_application as rx; from sweepick.control import sweepick_trajectory_profile as tt

IN = Path.home() / "sweepick_261007_commission/inputs"
pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")
P = dict(left=dict(shoulder_pan=2040, shoulder_lift=1500, elbow_flex=2400, wrist_flex=2043, wrist_roll=2116, gripper=3372), right=dict(shoulder_pan=2103, shoulder_lift=1500, elbow_flex=2400, wrist_flex=2004, wrist_roll=2099, gripper=2981))
LIM = tt.load_profile()
D = lambda a, *x, **k: a.dispatch(*x, **{"servo_status": OK0, **k})      # a real caller always hands over the Status it read
OK0 = {s: {n: 0 for n in rc.JOINTS} for s in ('left', 'right')}
VMAX, AMAX = LIM["v"] / tt.TICK_RAD, LIM["a"] / tt.TICK_RAD


@pytest.fixture(scope="module")
def mp():
    return rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")


def synthetic():
    """A made-up channel table (NOT a real calibration): a servo whose range is the full 0..4095 turn."""
    ch = lambda lo, hi: dict(id=1, sign=1, offset_rad=0.0, lo=lo, hi=hi, model_range=[-9.0, 9.0])
    side = {n: ch(0, 4095) for n in rc.JOINTS[:5]}; side["gripper"] = dict(id=6, closed=0.0, open=100.0, lo=0, hi=4095, model_range=[0.0, 1.0])
    return dict(left=dict(side), right=dict(side))


def follow(a, owner, q, present, n=4000):
    track = []
    for _ in range(n):
        d = D(a, owner, q, {s: {j: a.goal[s][j] for j in rc.JOINTS} for s in rx.SIDES} if present is None else present)
        track.append(d)
        if d.status != "SEND": break
    return track


def test_valid_target_at_the_range_end_never_produces_an_out_of_range_goal():
    """Reproduction of the review: before the repair a target of 4095 could produce the written value 4096."""
    mp_ = synthetic(); start = dict(left={n: 3000.0 for n in rc.JOINTS}, right={n: 3000.0 for n in rc.JOINTS})
    a = rx.Applier(mp_, LIM, start); q = rc.ticks_to_q12(mp_, dict(left={n: 4095.0 for n in rc.JOINTS}, right={n: 3000.0 for n in rc.JOINTS}))
    tr = follow(a, "ACT:X", q, None)
    sent = np.array([d.write["left"]["wrist_roll"] for d in tr])
    assert all(d.status == "SEND" for d in tr) and sent.max() == 4095 and sent[-1] == 4095 and (np.diff(sent) >= 0).all()


def test_far_target_is_followed_within_velocity_and_acceleration_and_is_not_refused(mp):
    a = rx.Applier(mp, LIM, P); q = rc.ticks_to_q12(mp, P); q[5] = 0.20                     # 1087 ticks away
    tr = follow(a, "ACT:START_3V", q, None, 1500)
    exact = np.array([P["left"]["gripper"]] + [d.applied["left"]["gripper"] for d in tr])
    assert all(d.status == "SEND" for d in tr) and tr[-1].write["left"]["gripper"] == round(rc.model_to_tick("gripper", mp["left"]["gripper"], 0.20))
    assert np.abs(np.diff(exact)).max() <= VMAX * LIM["dt"] * 1.000001 and np.abs(np.diff(np.diff(exact))).max() <= AMAX * LIM["dt"] ** 2 * 1.000001


def test_target_reversal_and_owner_change_brake_consistently(mp):
    a = rx.Applier(mp, LIM, P); q1 = rc.ticks_to_q12(mp, P); q1[2] -= 0.8; q2 = rc.ticks_to_q12(mp, P); q2[2] += 0.05
    exact = [P["left"]["elbow_flex"]]
    for k in range(400):
        d = D(a, "ACT:START_3V" if k < 60 else "ACT:RECEIVE_3V", q1 if k < 60 else q2, {s: dict(a.goal[s]) for s in rx.SIDES})   # the new owner asks for the other direction at full speed
        assert d.status == "SEND"; exact.append(d.applied["left"]["elbow_flex"])
    e = np.array(exact)
    assert np.abs(np.diff(np.diff(e))).max() <= AMAX * LIM["dt"] ** 2 * 1.000001 and abs(e[-1] - rc.model_to_tick("elbow_flex", mp["left"]["elbow_flex"], q2[2])) < 0.5
    assert e.min() < e[60] - 1                                                               # it kept going while it braked: no instant reversal


def test_rejected_owner_command_changes_nothing_but_a_controlled_brake_and_is_never_clipped(mp):
    a = rx.Applier(mp, LIM, P); ok = rc.ticks_to_q12(mp, P); ok[2] -= 0.5
    for _ in range(40):
        D(a, "ACT:START_3V", ok, {s: dict(a.goal[s]) for s in rx.SIDES})
    g0, v0 = json.dumps(a.goal), json.dumps(a.vel)
    bad = ok.copy(); bad[9] = 5.0                                                            # the LAST-but-two channel is out of range: earlier channels must not have advanced toward `bad`
    bad[0] += 0.3
    d = D(a, "ACT:START_3V", bad, {s: dict(a.goal[s]) for s in rx.SIDES})
    assert d.status == "BRAKE" and "owner command rejected" in d.reasons[0] and any("right wrist_flex" in r for r in d.reasons) and d.gates[0]["gate"] == "servo_range"
    assert abs(a.goal["left"]["shoulder_pan"] - json.loads(g0)["left"]["shoulder_pan"]) < 1e-9          # did not start toward the rejected command
    assert abs(a.vel["left"]["elbow_flex"]) < abs(json.loads(v0)["left"]["elbow_flex"])                # the moving channel is slowing down
    for _ in range(200):
        d = D(a, "ACT:START_3V", bad, {s: dict(a.goal[s]) for s in rx.SIDES})
    assert d.status == "BRAKE" and all(abs(v) < 1e-9 for s in rx.SIDES for v in a.vel[s].values())


def test_being_far_behind_but_advancing_is_reported_and_never_stops_the_boundary(mp):
    a = rx.Applier(mp, LIM, P); q = rc.ticks_to_q12(mp, P); q[2] -= 0.5
    here = {s: dict(P[s]) for s in rx.SIDES}; sent, worst = 0, 0.0
    for k in range(300):                                                                 # the arm follows at half the commanded speed: far behind, always advancing
        d = D(a, "ACT:START_3V", q, here, closed_loop=True, t_read=k * LIM["dt"], status_required=True, servo_status=OK0)
        assert d.status in ("SEND", "HOLD"), d.reasons
        sent += d.status == "SEND"; worst = max(worst, abs(d.tracking_error["left"]["elbow_flex"]))
        g = a.goal["left"]["elbow_flex"]; here["left"]["elbow_flex"] += max(-1, min(1, round(g) - here["left"]["elbow_flex"]))
    assert sent > 150 and worst > 17, (sent, worst)                                      # far behind: the goal may wait for it, the boundary never stops


def test_an_arm_that_does_not_follow_is_not_sent_a_far_goal(mp):
    """The reviewer's case: actual fixed, target 400 ticks away, fresh readings, Status 0. Before: 500 SEND, goal at the target."""
    a = rx.Applier(mp, LIM, P); tgt = {s: dict(P[s]) for s in rx.SIDES}; tgt["left"]["elbow_flex"] -= 400; q = rc.ticks_to_q12(mp, tgt)
    here = {s: dict(P[s]) for s in rx.SIDES}; out = []
    for k in range(500):
        d = D(a, "ACT:START_3V", q, here, closed_loop=True, t_read=k * LIM["dt"], status_required=True, servo_status=OK0); out.append(d.status)
        if d.status == "STOP": break
    assert out.count("SEND") < 90 and "HOLD" in out and out[-1] == "STOP" and d.gates[0]["gate"] == "no_progress" and d.write is None
    assert P["left"]["elbow_flex"] - a.goal["left"]["elbow_flex"] < 110                    # the goal stayed near the arm; 400 was never accumulated


def test_missing_non_finite_or_old_measurements_and_missing_status_write_nothing(mp):
    a = rx.Applier(mp, LIM, P); q = rc.ticks_to_q12(mp, P); q[2] -= 0.5
    here = lambda: {s: dict(P[s]) for s in rx.SIDES}
    g0 = json.dumps(a.goal)
    bad = here(); bad["left"]["elbow_flex"] = float("nan")
    d = D(a, "ACT:START_3V", q, bad, fresh=True, servo_status=OK0)
    assert d.status == "STOP" and d.write is None and d.gates[0]["gate"] == "measurement_invalid"
    bad = here(); del bad["right"]["gripper"]
    d = D(a, "ACT:START_3V", q, bad, servo_status=OK0)
    assert d.status == "STOP" and d.write is None and d.gates[0]["gate"] == "measurement_invalid"
    d = D(a, "ACT:START_3V", q, here(), servo_status=OK0, t_read=9.98, cycle_start=10.0)          # read before this cycle began
    assert d.status == "STOP" and d.write is None and d.gates[0]["gate"] in ("stale_position", "measurement_invalid")
    d = D(a, "ACT:START_3V", q, here(), servo_status=None, status_required=True)                  # no Status read is not "Status 0"
    assert d.status == "STOP" and d.write is None and d.gates[0]["gate"] == "status_unavailable"
    part = {s: dict(OK0[s]) for s in rx.SIDES}; del part["left"]["wrist_roll"]
    d = D(a, "ACT:START_3V", q, here(), servo_status=part, status_required=True)
    assert d.status == "STOP" and d.gates[0]["gate"] == "status_unavailable" and json.dumps(a.goal) == g0


def test_no_same_cycle_position_or_a_servo_fault_writes_nothing_and_changes_nothing(mp):
    a = rx.Applier(mp, LIM, P); q = rc.ticks_to_q12(mp, P); q[2] -= 0.5
    for _ in range(30):
        D(a, "ACT:START_3V", q, {s: dict(a.goal[s]) for s in rx.SIDES})
    g0, v0, c0 = json.dumps(a.goal), json.dumps(a.vel), a.cycles
    here = {s: dict(a.goal[s]) for s in rx.SIDES}
    for d in (D(a, "ACT:START_3V", q, here, fresh=False), D(a, "ACT:START_3V", q, None)):
        assert d.status == "STOP" and d.write is None and d.gates[0]["gate"] == "stale_position"
    status = {s: {n: 0 for n in rc.JOINTS} for s in rx.SIDES}; status["right"]["gripper"] = 32
    d = D(a, "ACT:START_3V", q, here, servo_status=status)
    assert d.status == "STOP" and d.write is None and d.gates[0]["gate"] == "servo_fault"
    assert json.dumps(a.goal) == g0 and json.dumps(a.vel) == v0 and a.cycles == c0


def test_gate_levels_of_a_rejected_owner_command(mp):
    a = rx.Applier(mp, LIM, P); q = rc.ticks_to_q12(mp, P)
    out = q.copy(); out[2] = 3.0                                                              # beyond the servo's own range
    assert [g["level"] for g in D(a, "ACT:START_3V", out, P).gates] == ["HARD"]
    pan = q.copy(); pan[0] = rc.tick_to_model("shoulder_pan", mp["left"]["shoulder_pan"], mp["left"]["shoulder_pan"]["hi"] - 2)   # inside the servo range, outside the model's
    d = D(a, "ACT:START_3V", pan, P)
    assert d.status == "BRAKE" and [(g["gate"], g["level"]) for g in d.gates] == [("model_joint_range_candidate", "PROVISIONAL")]
    nan = q.copy(); nan[4] = float("nan")
    assert D(a, "ACT:START_3V", nan, P).gates[0]["gate"] == "command_malformed"


def test_sensor_owned_steps_brake_and_say_what_is_missing(mp):
    a = rx.Applier(mp, LIM, P); q = rc.ticks_to_q12(mp, P)
    d = D(a, "CORR:START_SENSOR", q, P)
    assert d.status == "BRAKE" and "REAL JawDevice" in d.reasons[0] and d.write["left"]["gripper"] == 3372
    assert rx.Applier(mp, LIM, P, real_jaw_profile=object()).dispatch("CORR:START_SENSOR", q, P, servo_status=OK0).status == "SEND"


def test_stop_request_brakes_to_rest(mp):
    a = rx.Applier(mp, LIM, P); q = rc.ticks_to_q12(mp, P); q[2] -= 0.8
    for _ in range(60):
        D(a, "ACT:START_3V", q, {s: dict(a.goal[s]) for s in rx.SIDES})
    n = 0
    while any(abs(v) > 1e-9 for s in rx.SIDES for v in a.vel[s].values()):
        d = D(a, "ACT:START_3V", q, {s: dict(a.goal[s]) for s in rx.SIDES}, stop=True); n += 1
        assert d.status == "BRAKE" and n < 200
    assert n > 3


def test_after_a_stop_the_goal_is_relaxed_to_the_measured_position(mp):
    a = rx.Applier(mp, LIM, P); tgt = {s: dict(P[s]) for s in rx.SIDES}; tgt["left"]["elbow_flex"] -= 400; q = rc.ticks_to_q12(mp, tgt)
    here = {s: dict(P[s]) for s in rx.SIDES}
    for k in range(500):
        d = D(a, "ACT:START_3V", q, here, t_read=k * LIM["dt"])
        if d.status == "STOP": break
    assert d.status == "STOP" and P["left"]["elbow_flex"] - a.goal["left"]["elbow_flex"] > 17      # the last goal sent is still ahead of the arm
    w = a.relax(here)
    assert w == {s: {n: int(round(P[s][n])) for n in rc.JOINTS} for s in rx.SIDES} and a.goal["left"]["elbow_flex"] == P["left"]["elbow_flex"] and all(v == 0 for s in rx.SIDES for v in a.vel[s].values())
    bad = {s: dict(P[s]) for s in rx.SIDES}; bad["right"]["gripper"] = float("nan")
    assert a.relax(bad) is None
