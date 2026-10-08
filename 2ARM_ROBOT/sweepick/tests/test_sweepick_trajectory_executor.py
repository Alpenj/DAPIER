"""Behaviour tests of the move executor on a fake bus (no device): normal delay and lag run through, a joint that does not
advance is not driven to a far goal, reaching is confirmed from the readback, and only the named joints are powered."""
import json
from pathlib import Path
import pytest
from sweepick.control import sweepick_trajectory_executor as a; from sweepick.control import sweepick_joint_command_mapping as rc; from sweepick.control import sweepick_trajectory_profile as tt; from sweepick.control import sweepick_motion_progress as PG
from sweepick_261008_control_fake_bus import Fake as Base, IN, START

pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")
MID = dict(START, elbow_flex=2400)
P = PG.load()
BOUND = P["follow"] + 2 * P["start_delay"] * 0.15 / tt.TICK_RAD + 2          # how far a command can get ahead of a joint that does not move: following lead + two start delays at the profile speed


class Clock:
    def __init__(self): self.t = 1000.0
    def now(self): return self.t
    def sleep(self, s): self.t += s if s > 0 else 0.02


class Fake(Base):
    """elbow behaviours: lag (ticks behind), start_lead (does not move until the goal is this far ahead), stuck (never moves),
    stuck_below (cannot pass this tick), step_every (moves one tick every N goal writes), fault_from / fault_clears, die_at"""
    def __init__(self, start=START, **fail):
        super().__init__(**fail); self.reg["Present_Position"] = dict(start)
        self.reg["Status"] = {n: 0 for n in rc.JOINTS}; self.reg["Moving"] = {n: 0 for n in rc.JOINTS}; self.cur, self.k, self.started = 0.0, 0, False
    def sync_read(self, reg, normalize=True):
        r = super().sync_read(reg, normalize)
        if reg == "Present_Current":
            return {n: self.cur for n in rc.JOINTS}
        if reg == "Moving" and "sag" in self.fail:                                                                # the flag is read once after a step and then falls back, as on the servo
            r = dict(r); self.reg["Moving"] = {n: 0 for n in rc.JOINTS}
        if reg == "Present_Position" and "rate" in self.fail and self.reg["Torque_Enable"]["elbow_flex"]:      # a slow joint: moves in time, at most `rate` ticks per read
            p, g = self.reg["Present_Position"]["elbow_flex"], self.reg["Goal_Position"]["elbow_flex"]
            self.reg["Present_Position"]["elbow_flex"] = p + max(-self.fail["rate"], min(self.fail["rate"], round(g) - p)); r = dict(self.reg["Present_Position"])
        if reg == "Present_Position" and self.fail.get("noisy_rest") and not any(self.reg["Torque_Enable"].values()):
            self.k += 1; r = dict(r); r["shoulder_pan"] += self.k % 2
        return r
    def write(self, reg, n, v, normalize=True):
        if reg == "Torque_Enable" and self.fail.get("torque_ack_lost") == int(v):                                  # the servo applies the write, the answer never arrives
            self.reg[reg][n] = int(v); self.dead = True; raise ConnectionError("no status packet")
        if reg == "Goal_Position" and n == "elbow_flex" and self.reg["Torque_Enable"][n]:
            f = self.fail
            if f.get("fault_from") and self.n_goal >= f["fault_from"]:
                self.reg["Status"]["elbow_flex"] = 0 if (self.reg["Status"]["elbow_flex"] and f.get("fault_clears")) else 32
            if any(k in f for k in ("lag", "start_lead", "stuck", "stuck_below", "step_every", "rate", "sag")):
                self.n_goal += 1; self.writes.append((reg, n, v)); self.reg[reg][n] = v
                if f.get("die_at") == self.n_goal: self.dead = True; raise ConnectionError("bus dead")
                p = self.reg["Present_Position"][n]; d = -1 if v < p else 1
                if f.get("stuck") or "rate" in f: new = p
                elif "sag" in f:                                                                          # a loaded joint: the further it has carried its link, the further it rests short of its goal (up to `sag`)
                    self.p0 = getattr(self, "p0", p); short = min(f["sag"], 5 + abs(v - self.p0) / 10)
                    new = min(p, round(v + short)) if d < 0 else max(p, round(v - short))
                elif "start_lead" in f and not self.started:
                    self.started = abs(v - p) >= f["start_lead"]; new = p
                elif "step_every" in f: new = p + d if (self.n_goal % f["step_every"] == 0 and v != p) else p
                else:
                    new = v - d * f.get("lag", 0) if abs(v - p) > f.get("lag", 0) else p
                    if "stuck_below" in f: new = max(new, f["stuck_below"])
                self.reg["Moving"][n] = int(new != p); self.reg["Present_Position"][n] = new
                return
        super().write(reg, n, v, normalize)


def go(tmp_path, fake, mode="out", **kw):
    mp = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")
    c = Clock()
    return a.run(mode, "left", tmp_path, open_bus=lambda port: fake, owner=kw.pop("owner", lambda p: ""), mapping=mp, mapping_sha="m", sleep=c.sleep, clock=c.now, **kw)


ONLY = lambda f, joints: {n for n, v in f.reg["Torque_Enable"].items() if v} == set(joints)
goals = lambda f, n="elbow_flex": [v for r, j, v in f.writes if r == "Goal_Position" and j == n]


def test_plan_reads_the_device_and_writes_nothing(tmp_path):
    f = Fake(); code, log = go(tmp_path / "p", f, mode="plan", clear_range_end=("elbow_flex", 60))
    assert code == 0 and log["result"] == "PLANNED" and f.writes == [] and log["trajectory"]["target"] == dict(elbow_flex=3093.0) and log["validation"]["executable"]


@pytest.mark.parametrize("ticks", [45, 228, 1000])
def test_moves_of_any_size_reach_their_target_and_only_the_moving_joint_is_powered(tmp_path, ticks):
    f = Fake(MID); code, log = go(tmp_path / "o", f, target=dict(elbow_flex=MID["elbow_flex"] - ticks), execute=True)
    assert code == 0 and log["result"] == "HOLDING" and log["outcome"] == dict(command_completed=True, target_reached=True, return_confirmed=None, torque_released=False)
    assert f.reg["Present_Position"]["elbow_flex"] == MID["elbow_flex"] - ticks and ONLY(f, ["elbow_flex"]) and log["torque_scope"]["joints"] == ["elbow_flex"] and log["total_travel_limit"] is None


def test_goal_is_seeded_from_a_fresh_read_before_torque_on_and_only_for_the_powered_joint(tmp_path):
    f = Fake(MID); go(tmp_path / "o", f, target=dict(gripper=3000), execute=True)
    first_torque = next(i for i, w in enumerate(f.writes) if w[0] == "Torque_Enable")
    assert f.writes[:first_torque] == [("Goal_Position", "gripper", MID["gripper"])] and f.writes[first_torque] == ("Torque_Enable", "gripper", 1)
    assert ONLY(f, ["gripper"]) and not any(j != "gripper" for _, j, _ in f.writes)


def test_normal_start_delay_and_lag_run_through_and_reach(tmp_path):
    f = Fake(MID, start_lead=17, lag=10); code, log = go(tmp_path / "o", f, target=dict(elbow_flex=1900), execute=True)      # the longest start seen: 17 ticks of lead; then 10 behind
    assert code == 0 and log["result"] == "HOLDING" and log["outcome"]["target_reached"] and "stopped_by" not in log
    assert log["reached"]["elbow_flex"]["minus_target"] == 10 and any(d.get("diagnostic") == "tracking_error" and d["max_ticks"] >= 10 for d in log["diagnostics"])


def test_slow_quantised_progress_is_not_stopped(tmp_path):
    f = Fake(MID, step_every=8); code, log = go(tmp_path / "o", f, target=dict(elbow_flex=2380), execute=True)                 # one tick every 0.16 s: slow, but always advancing
    assert "no_progress" not in json.dumps(log.get("stopped_by", {})) and log["outcome"]["command_completed"] and code in (0, 6)


def test_a_joint_that_never_moves_is_not_driven_to_the_far_goal(tmp_path):
    f = Fake(MID, stuck=True); code, log = go(tmp_path / "o", f, target=dict(elbow_flex=1600), execute=True)                  # 800 ticks away
    sent = goals(f)
    assert code == 8 and log["result"] == "NO_PROGRESS_HOLDING" and log["stopped_by"]["gate"] == "no_progress" and log["outcome"]["command_completed"] is False and log["outcome"]["target_reached"] is False
    assert min(sent) > 2400 - BOUND and sent[-1] == 2400                      # the command stopped within the following lead plus what it covers in two start delays, and was relaxed to the measured position; 1600 was never sent
    assert ONLY(f, ["elbow_flex"]) and "release" not in log                   # no automatic release


def test_a_joint_that_stops_on_the_way_makes_the_command_wait_then_stops_pushing(tmp_path):
    f = Fake(MID, stuck_below=2300); code, log = go(tmp_path / "o", f, target=dict(elbow_flex=1600), execute=True)
    sent = goals(f)
    assert code == 8 and log["stopped_by"]["gate"] == "no_progress" and min(sent) > 2300 - BOUND and sent[-1] == 2300
    assert any(r.get("command_waiting") for r in log["rows"])


def test_far_behind_but_advancing_is_never_stopped_and_reaches(tmp_path):
    f = Fake(MID, rate=1); code, log = go(tmp_path / "o", f, target=dict(elbow_flex=1900), execute=True)                       # half the commanded cruise speed
    worst = max(abs(r["tracking_error"]["elbow_flex"]) for r in log["rows"])
    assert code == 0 and log["outcome"]["target_reached"] and "stopped_by" not in log and worst > 30, worst


def test_ending_short_of_the_target_is_reported_as_not_reached(tmp_path):
    f = Fake(MID, stuck_below=1916); code, log = go(tmp_path / "o", f, target=dict(elbow_flex=1900), execute=True)            # stands 16 short: within the lead at which joints normally stand, outside the settled error seen
    assert code == 6 and log["result"] == "TARGET_NOT_REACHED_HOLDING" and log["outcome"] == dict(command_completed=True, target_reached=False, return_confirmed=None, torque_released=False)
    assert log["stopped_by"]["gate"] == "target_not_reached" and log["reached"]["elbow_flex"]["minus_target"] == 16


def test_return_is_a_success_only_when_the_readback_is_back(tmp_path):
    f = Fake(MID); go(tmp_path / "o", f, target=dict(elbow_flex=2000), execute=True)
    f.fail["stuck"] = True                                                    # the joint will not move on the way back
    code, log = go(tmp_path / "r", f, mode="return", rest=dict(elbow_flex=2400), execute=True)
    assert code != 0 and log["result"] in ("NO_PROGRESS_HOLDING", "RETURN_NOT_CONFIRMED_HOLDING") and log["outcome"]["torque_released"] is False and f.reg["Torque_Enable"]["elbow_flex"] == 1
    assert f.reg["Present_Position"]["elbow_flex"] == 2000 and log["outcome"]["return_confirmed"] in (None, False)
    f = Fake(MID); go(tmp_path / "o2", f, target=dict(elbow_flex=2000), execute=True)
    code, log = go(tmp_path / "r2", f, mode="return", rest=dict(elbow_flex=2400), execute=True)
    assert code == 0 and log["result"] == "RETURNED_RELEASED" and log["outcome"] == dict(command_completed=True, target_reached=True, return_confirmed=True, torque_released=True) and not any(f.reg["Torque_Enable"].values())


def test_return_that_ends_short_of_the_target_keeps_the_torque_and_is_not_a_success(tmp_path):
    f = Fake(MID); go(tmp_path / "o", f, target=dict(elbow_flex=2000), execute=True)
    f.fail["lag"] = 16                                                        # follows, but ends 16 ticks short (the settled error seen so far is at most 14)
    code, log = go(tmp_path / "r", f, mode="return", rest=dict(elbow_flex=2400), execute=True)
    assert code == 7 and log["result"] == "RETURN_NOT_CONFIRMED_HOLDING" and log["outcome"] == dict(command_completed=True, target_reached=False, return_confirmed=False, torque_released=False) and f.reg["Torque_Enable"]["elbow_flex"] == 1


def test_a_reading_that_flickers_at_rest_does_not_refuse_the_start(tmp_path):
    f = Fake(MID, noisy_rest=True); code, log = go(tmp_path / "o", f, target=dict(elbow_flex=2000), execute=True)
    assert code == 0 and any(d.get("diagnostic") == "start_spread" for d in log["diagnostics"])


def test_servo_fault_operator_stop_and_bus_loss_keep_their_meaning(tmp_path):
    f = Fake(MID, lag=1, fault_from=40); code, log = go(tmp_path / "a", f, target=dict(elbow_flex=2000), execute=True)
    assert code == 14 and log["stopped_by"]["gate"] == "servo_fault" and f.reg["Torque_Enable"]["elbow_flex"] == 0
    f = Fake(MID, lag=1, fault_from=40, fault_clears=True); code, log = go(tmp_path / "b", f, target=dict(elbow_flex=2000), execute=True)
    assert code == 13 and ONLY(f, ["elbow_flex"])
    f = Fake(MID); n = [0]
    def stop():
        n[0] += 1; return n[0] > 40
    code, log = go(tmp_path / "c", f, target=dict(elbow_flex=2000), execute=True, stop_requested=stop)
    assert code == 10 and log["stopped_by"]["gate"] == "operator_stop" and 2000 < f.reg["Present_Position"]["elbow_flex"] < 2400 and "release" not in log
    f = Fake(MID, lag=1, die_at=40); code, log = go(tmp_path / "d", f, target=dict(elbow_flex=2000), execute=True)
    assert code == 15 and log["stopped_by"]["gate"] == "bus_lost" and "NOT known" in log["needs_a_person"]
    f = Fake(MID); f.reg["Status"]["wrist_flex"] = 8
    code, log = go(tmp_path / "e", f, target=dict(elbow_flex=2000), execute=True)
    assert code == 2 and log["refused"]["gate"] == "servo_fault" and f.writes == []


def test_range_mismatch_and_ownership_refuse_without_a_write(tmp_path):
    f = Fake(MID); code, log = go(tmp_path / "a", f, target=dict(elbow_flex=4000), execute=True)
    assert code == 2 and log["refused"]["gate"] == "servo_range" and f.writes == []
    f = Fake(MID); f.reg["Max_Position_Limit"]["elbow_flex"] = 3000
    code, log = go(tmp_path / "b", f, target=dict(elbow_flex=2000), execute=True)
    assert code == 2 and log["refused"]["gate"] == "device_calibration_mismatch" and f.writes == []
    f = Fake(MID); f.reg["Torque_Enable"]["wrist_roll"] = 1
    code, log = go(tmp_path / "c", f, target=dict(elbow_flex=2000), execute=True)
    assert code == 2 and log["refused"]["gate"] == "torque_state" and f.writes == [] and f.reg["Torque_Enable"]["wrist_roll"] == 1
    f = Fake(MID); code, log = go(tmp_path / "d", f, target=dict(elbow_flex=2000), execute=True, owner=lambda p: "4242")
    assert code == 2 and log["refused"]["gate"] == "port_owned"


def test_model_level_adds_the_provisional_gates(tmp_path):
    f = Fake(MID); tgt = dict(elbow_flex=2000)
    code, log = go(tmp_path / "m0", f, target=tgt, level="model", execute=True)
    assert code == 2 and log["refused"]["gate"] == "collision_candidate_frame" and f.writes == []
    go(tmp_path / "plan", f, mode="plan", level="model", target=tgt)
    cert = dict(identity=json.loads((tmp_path / "plan/TRAJECTORY.json").read_text())["identity"], nominal_verdict="CLEAR")
    code, log = go(tmp_path / "m1", f, target=tgt, level="model", cert=cert, execute=True)
    assert code == 0 and log["result"] == "HOLDING"


def test_a_lost_answer_to_torque_on_is_bus_lost_with_the_torque_state_unknown(tmp_path):
    f = Fake(MID, torque_ack_lost=1); f.closed = 0; d0 = f.disconnect
    def disc(*a, **k):
        f.closed += 1; return d0(*a, **k)
    f.disconnect = disc
    code, log = go(tmp_path / "o", f, target=dict(gripper=3000), execute=True)
    assert code == 15 and log["result"] == "BUS_LOST" and log["stopped_by"]["gate"] == "bus_lost" and (tmp_path / "o/move.json").exists() and f.closed >= 1
    assert log["outcome"]["torque_released"] == "UNKNOWN" and log["end"]["torque"] == "UNKNOWN" and "NOT known" in log["needs_a_person"] and log["writes"]["Torque_Enable"] == 0
    assert any(isinstance(e, dict) and e.get("torque_write_acknowledgement") == "UNKNOWN" and e["joint"] == "gripper" and e["requested"] == 1 for e in log["events"])
    assert f.reg["Torque_Enable"]["gripper"] == 1 and "release" not in log and "recovery" not in log            # the servo IS on; nothing claims otherwise, nothing was retried or released
    assert not any(j != "gripper" for _, j, _ in f.writes)


def test_a_lost_answer_to_torque_off_after_a_confirmed_return_is_not_reported_as_released(tmp_path):
    f = Fake(MID); go(tmp_path / "o", f, target=dict(gripper=3000), execute=True)
    f.fail["torque_ack_lost"] = 0
    code, log = go(tmp_path / "r", f, mode="return", rest=dict(gripper=MID["gripper"]), execute=True)
    assert code == 5 and log["result"] == "RELEASE_UNCONFIRMED" and log["outcome"] == dict(command_completed=True, target_reached=True, return_confirmed=True, torque_released="UNKNOWN")
    assert f.reg["Torque_Enable"]["gripper"] == 0                                # the servo did switch off; the report says UNKNOWN, neither released nor still on


def test_next_continues_a_held_arm_without_reseeding_the_held_joints_and_powers_only_what_it_is_told(tmp_path):
    f = Fake(MID); go(tmp_path / "a", f, target=dict(elbow_flex=2300), execute=True)                       # elbow held at 2300
    n0 = len(f.writes)
    code, log = go(tmp_path / "b", f, mode="next", target=dict(gripper=3000), torque_joints=["elbow_flex", "gripper"], execute=True)
    new = f.writes[n0:]
    assert code == 0 and log["result"] == "HOLDING" and log["goal_seed"]["joints"] == ["gripper"] and log["goal_seed"]["already_held"] == ["elbow_flex"]
    assert new[0] == ("Goal_Position", "gripper", MID["gripper"]) and ("Torque_Enable", "elbow_flex", 1) not in new and not any(j == "elbow_flex" for _, j, _ in new) and ONLY(f, ["elbow_flex", "gripper"])
    f2 = Fake(MID); f2.reg["Torque_Enable"]["wrist_roll"] = 1
    code, log = go(tmp_path / "c", f2, mode="next", target=dict(gripper=3000), torque_joints=["gripper"], execute=True)
    assert code == 2 and log["refused"]["gate"] == "torque_state" and f2.writes == []                       # a joint is on that this run was not told about


def test_a_loaded_joint_that_follows_at_a_large_lead_is_not_a_stall_and_the_trim_brings_it_to_the_target(tmp_path):
    f = Fake(MID, sag=23); code, log = go(tmp_path / "a", f, target=dict(elbow_flex=1900), execute=True)                 # as the real elbow: 23 ticks short under its load
    assert code == 6 and log["result"] == "TARGET_NOT_REACHED_HOLDING" and log["outcome"]["command_completed"] and log["reached"]["elbow_flex"]["minus_target"] == 23   # followed to the end; without the trim it is reported as short
    f = Fake(MID, sag=23); code, log = go(tmp_path / "b", f, target=dict(elbow_flex=1900), execute=True, settle_trim=["elbow_flex"])
    assert code == 0 and log["result"] == "HOLDING" and log["reached"]["elbow_flex"]["minus_target"] == 0 and log["settle_trim"][0]["short"] == dict(elbow_flex=-23.0) and f.reg["Goal_Position"]["elbow_flex"] == 1877
    f = Fake(MID, stuck_below=1930); code, log = go(tmp_path / "c", f, target=dict(elbow_flex=1900), execute=True, settle_trim=["elbow_flex"])
    assert code != 0 and log["outcome"]["target_reached"] is False and len(log.get("settle_trim", [])) <= 2            # something in the way: the trim does not keep pushing


def test_a_loaded_joint_that_needs_more_lead_to_start_is_trimmed_up_to_the_following_lead_and_no_further(tmp_path):
    """run 10 of 2026-10-08: a small move up of the shoulder stood 16 ticks short, did not start at a lead of 32 ticks either, and the trim gave up after one step"""
    f = Fake(MID, start_lead=45); code, log = go(tmp_path / "a", f, target=dict(elbow_flex=MID["elbow_flex"] - 16), execute=True, settle_trim=["elbow_flex"])
    assert len(log["settle_trim"]) >= 2 and code == 0 and log["outcome"]["target_reached"], (code, log.get("settle_trim"), log.get("reached"))       # a second trim puts the goal 48 ticks ahead, within the following lead, and the joint starts (the fake then overshoots and is trimmed back)
    f = Fake(MID, stuck=True); code, log = go(tmp_path / "b", f, target=dict(elbow_flex=MID["elbow_flex"] - 16), execute=True, settle_trim=["elbow_flex"])
    assert code != 0 and log["outcome"]["target_reached"] is False and max(abs(v - MID["elbow_flex"]) for v in goals(f)) <= P["follow"]            # a joint that never moves: the goal is never put further ahead than the following lead
