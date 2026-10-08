"""The grip mode on a fake bus (no device): the existing Jaw primitive decides close / squeeze / hold; a jaw that creeps
under the push does not earn a larger push; empty jaws, a fault and a lost answer are not a contact."""
import json
from pathlib import Path
import pytest
from sweepick.control import sweepick_trajectory_executor as mv; from sweepick.control import sweepick_joint_command_mapping as rc; from sweepick.control import sweepick_trajectory_profile as tt
from test_sweepick_trajectory_executor import Fake as Base, Clock, IN, MID

pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")
OPEN = dict(MID, gripper=3366)


class Jaws(Base):
    """the gripper follows its goal 8 ticks behind; `block` = the tick where something is met; `creep` = beyond the block it
    gives way one tick for every `creep` ticks of extra push (a compliant pad / object), never freely"""
    def write(self, reg, n, v, normalize=True):
        if reg == "Torque_Enable" and self.fail.get("torque_ack_lost") == int(v):
            self.reg[reg][n] = int(v); self.dead = True; raise ConnectionError("no status packet")
        if reg == "Goal_Position" and n == "gripper" and self.reg["Torque_Enable"][n]:
            self.n_goal += 1; self.writes.append((reg, n, v)); self.reg[reg][n] = v
            if self.fail.get("fault_from") and self.n_goal >= self.fail["fault_from"]: self.reg["Status"]["gripper"] = 32
            p = self.reg["Present_Position"][n]; new = min(p, v + 8) if v < p else max(p, v - 8)
            if "block" in self.fail and new < self.fail["block"]:
                b = self.fail["block"]; new = b - int((b - 8 - v) / self.fail["creep"]) if self.fail.get("creep") and v < b - 8 else b
                new = min(p, max(new, v + 8))
            self.reg["Present_Position"][n] = new; self.reg["Load"] = None; return
        super(Base, self).write(reg, n, v, normalize)


def go(tmp_path, fake, **kw):
    mp = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json"); c = Clock()
    return mv.grip("left", tmp_path / "g", torque_joints=list(rc.JOINTS), execute=True, hold_s=0.3, open_bus=lambda port: fake, owner=lambda p: "", mapping=mp, mapping_sha="m", sleep=c.sleep, clock=c.now, **kw)


sent = lambda f: [v for r, j, v in f.writes if r == "Goal_Position" and j == "gripper"]


@pytest.mark.parametrize("creep", [None, 8, 3])
def test_a_block_is_a_contact_candidate_and_the_push_does_not_grow_with_a_creeping_jaw(tmp_path, creep):
    f = Jaws(OPEN, block=3075, **({} if creep is None else dict(creep=creep))); code, log = go(tmp_path, f)
    g = log["grip"]
    assert code == 0 and log["result"] == "GRIP_CONTACT_HOLDING" and g["state"] == "CONTACT_CANDIDATE" and log["outcome"]["contact_candidate"] is True
    lead_max = max(r["residual_ticks"] for r in log["rows"])
    assert lead_max <= 17 + 24 + 8, lead_max                                  # onset floor + preload + the fake's own lag: far from the 125 ticks of PICK01 close 1
    assert f.reg["Goal_Position"]["gripper"] == round(g["hold_command_tick"]) and g["at_rest"]["residual_ticks"] > 17 and min(sent(f)) > 3075 - 60
    assert not any(j != "gripper" for _, j, _ in f.writes) and "release" not in log and f.reg["Torque_Enable"]["gripper"] == 1


def test_empty_jaws_end_at_the_end_stop_without_a_contact(tmp_path):
    f = Jaws(OPEN); code, log = go(tmp_path, f)
    assert code == 9 and log["result"] == "GRIP_NO_CONTACT_HOLDING" and log["outcome"]["contact_candidate"] is False and log["grip"]["contact_onset"] is None
    assert min(sent(f)) >= 2001 + 17                                         # the command stays off the calibrated closed end
    peak = max(abs(a - b) for a, b in zip(sent(f), sent(f)[1:]))
    assert peak <= 0.15 / tt.TICK_RAD * 0.02 + 1                             # the closing command keeps the profile speed


def test_a_fault_or_a_lost_answer_is_not_a_contact_and_nothing_is_opened(tmp_path):
    f = Jaws(OPEN, block=3075, fault_from=30); code, log = go(tmp_path / "a", f)
    assert log["result"] == "FAULT_HOLDING" and code == 13 and log["outcome"]["contact_candidate"] is None and max(sent(f)) <= 3366
    f = Jaws(dict(OPEN), torque_ack_lost=1); f.reg["Torque_Enable"] = {n: int(n != "gripper") for n in rc.JOINTS}
    code, log = go(tmp_path / "b", f)
    assert log["result"] == "BUS_LOST" and code == 15 and log["outcome"]["torque_released"] == "UNKNOWN"
    f = Jaws(OPEN); (tmp_path / "s").mkdir(); code, log = go(tmp_path / "c", f, stop_requested=lambda: True)
    assert log["result"] == "STOPPED_HOLDING" and "release" not in log


def test_model_level_needs_a_clear_result_for_the_whole_closing_sweep_at_this_arm_pose(tmp_path):
    f = Jaws(OPEN); code, log = go(tmp_path / "a", f, level="model")
    assert code == 2 and log["refused"]["gate"] == "collision_candidate_frame" and f.writes == []
    cert = dict(nominal_verdict="CLEAR", arm_ticks={n: OPEN[n] for n in rc.JOINTS[:5]}, sweep=[3366, 2018])
    f = Jaws(OPEN, block=3075); code, log = go(tmp_path / "b", f, level="model", cert=cert)
    assert code == 0
    f = Jaws(dict(OPEN, elbow_flex=OPEN["elbow_flex"] + 9)); code, log = go(tmp_path / "c", f, level="model", cert=cert)
    assert code == 2 and log["refused"]["gate"] == "certificate_binding" and f.writes == []


def test_a_trim_is_a_checked_move_and_is_not_applied_when_its_path_is_not_clear(tmp_path):
    from test_sweepick_trajectory_executor import Fake, go as run
    seen = []
    def check(doc):
        seen.append(doc); return dict(nominal_verdict="CONTACT")
    f = Fake(MID, sag=23); code, log = run(tmp_path / "a", f, target=dict(elbow_flex=1900), execute=True, settle_trim=["elbow_flex"], trim_check=check, level="device")
    assert code == 0 and log["settle_trim"][0]["applied"] and seen and seen[0]["trajectory"]["target"] == dict(elbow_flex=1877.0) and seen[0]["trajectory"]["steps"] > 1     # device level: checked for range and profile, the caller's verdict is recorded
    assert log["settle_trim"][0]["collision"] == "CONTACT"


def test_at_model_level_a_trim_goes_through_the_same_validation_as_its_stage(tmp_path):
    """the trim segment needs its own CLEAR collision result bound to it (and passes the model joint range gate); otherwise it is not written"""
    from test_sweepick_trajectory_executor import Fake, go as run
    tgt = dict(elbow_flex=1900)
    def plan_cert(d):
        f = Fake(MID, sag=23); run(d, f, mode="plan", level="model", target=tgt, other_arm_ticks=dict(MID))
        return dict(identity=json.loads((d / "TRAJECTORY.json").read_text())["identity"], nominal_verdict="CLEAR")
    seen = []
    for verdict, applied in (("CONTACT", False), ("CLEAR", True)):
        d = tmp_path / verdict; cert = plan_cert(d / "plan"); f = Fake(MID, sag=23); n0 = None
        def check(doc, v=verdict):
            seen.append(doc["identity"]); return dict(identity=doc["identity"], nominal_verdict=v)
        code, log = run(d / "run", f, target=tgt, level="model", cert=cert, other_arm_ticks=dict(MID), execute=True, settle_trim=["elbow_flex"], trim_check=check)
        t = log["settle_trim"][0]
        assert t["applied"] is applied and (code == 0) is applied and (f.reg["Goal_Position"]["elbow_flex"] == 1877) is applied, (verdict, t)
        assert applied or any(g["gate"] in ("collision_candidate_frame", "certificate_binding") for g in t["provisional"])
    assert len(seen) == 2
