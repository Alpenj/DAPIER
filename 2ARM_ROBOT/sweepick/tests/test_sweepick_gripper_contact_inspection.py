"""LEFT_CONTACT01 wrapper on a fake bus (no device, no camera): the executor's result is kept, a stop is not contact,
a fault / bus loss / operator stop is not followed by the opening."""
import json
from pathlib import Path
import pytest
from sweepick.control import sweepick_gripper_contact_inspection as c1; from sweepick.control import sweepick_joint_command_mapping as rc
from test_sweepick_trajectory_executor import Fake as Base, Clock, IN, MID

pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")
OPEN = dict(MID, gripper=3372)


class Jaw(Base):
    """the gripper follows its goal 8 ticks behind; with `blocked_at` it cannot close past that tick (something is in the jaws)"""
    def write(self, reg, n, v, normalize=True):
        if reg == "Torque_Enable" and self.fail.get("torque_ack_lost") == int(v):
            self.reg[reg][n] = int(v); self.dead = True; raise ConnectionError("no status packet")
        if reg == "Goal_Position" and n == "gripper" and self.reg["Torque_Enable"][n]:
            self.n_goal += 1; self.writes.append((reg, n, v)); self.reg[reg][n] = v
            if self.fail.get("fault_from") and self.n_goal >= self.fail["fault_from"]: self.reg["Status"]["gripper"] = 32
            p = self.reg["Present_Position"][n]; d = -1 if v < p else 1
            new = v - d * 8 if abs(v - p) > 8 else p
            if "blocked_at" in self.fail: new = max(new, self.fail["blocked_at"])
            self.reg["Moving"][n] = int(new != p); self.reg["Present_Position"][n] = new; return
        super(Base, self).write(reg, n, v, normalize)


def go(tmp_path, fake, **kw):
    mp = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json"); c = Clock()
    return c1.session(tmp_path / "s", 2716, execute=True, dwell_s=0.1, cameras=lambda: [], sleep=c.sleep, run_kw=dict(open_bus=lambda port: fake, owner=lambda p: "", mapping=mp, mapping_sha="m", sleep=c.sleep, clock=c.now), **kw)


def test_something_in_the_jaws_keeps_the_executor_result_and_does_not_become_contact_or_grasp(tmp_path):
    f = Jaw(OPEN, blocked_at=3000); code, d = go(tmp_path, f)
    assert d["controller"]["close"]["result"] == "NO_PROGRESS_HOLDING" and d["controller"]["close"]["exit_code"] == 8 and d["controller"]["close"]["stopped_by"]["gate"] == "no_progress"
    o = d["observation"]
    assert o["contact_candidate"] == "unknown" and o["grasp_confirmed"] == "NOT_TESTED" and o["support_confirmed"] == "NOT_TESTED" and o["controller_stop_reason"] == "no_progress"
    assert o["actual_stop"] == 3000 and o["commanded_close"] > 3000 - 110 and "not a held grip force" in o["holding_meaning"]
    assert d["controller"]["open"]["result"] == "RETURNED_RELEASED" and o["opening_completed"] is True and o["final_torque_readback"] is True and code == 0
    assert d["result"] == dict(close="NO_PROGRESS_HOLDING", open="RETURNED_RELEASED", session="RETURNED_RELEASED")
    assert not any(j != "gripper" for _, j, _ in f.writes) and not any(f.reg["Torque_Enable"].values())
    a = c1.annotate(tmp_path / "s", object_visible="true", contact="true", source="USER_OBSERVED", at_stop="both pads on the block", after_relax="block did not move")["observation"]
    assert a["contact_candidate"] == "true" and a["grasp_confirmed"] == "NOT_TESTED" and a["object_state"]["at_stop"]["source"] == "USER_OBSERVED"
    a = c1.annotate(tmp_path / "s", object_visible="unknown", contact="true", source="FRAME_REVIEWED")["observation"]
    assert a["contact_candidate"] == "unknown"                                    # contact claimed without the object seen between the jaws


def test_empty_jaws_reach_the_target_and_are_not_classified_as_contact(tmp_path):
    f = Jaw(OPEN); code, d = go(tmp_path, f)
    assert d["controller"]["close"]["result"] == "HOLDING" and d["observation"]["contact_candidate"] == "unknown" and d["controller"]["open"]["result"] == "RETURNED_RELEASED"
    base = json.loads(c1.BASELINE.read_text()) if c1.BASELINE.exists() else None
    if base:                                                                       # the real no-load trace of 2026-10-08
        o = c1.observation(base, None, [])
        assert o["contact_candidate"] == "unknown" and o["controller_stop_reason"] is None and o["grasp_confirmed"] == "NOT_TESTED"


@pytest.mark.parametrize("fail, result", [(dict(fault_from=40), ("FAULT_JOINTS_RELEASED", "FAULT_HOLDING")), (dict(torque_ack_lost=1), ("BUS_LOST",))])
def test_a_fault_or_a_lost_bus_is_not_an_expected_contact_and_nothing_is_opened(tmp_path, fail, result):
    f = Jaw(OPEN, **fail); code, d = go(tmp_path, f)
    assert d["controller"]["close"]["result"] in result and d["controller"]["open"] is None and code != 0
    assert d["phases"][-1] == dict(phase="planned_open", result="NOT_RUN", why=d["phases"][-1]["why"]) and d["observation"]["contact_candidate"] == "unknown" and d["observation"]["opening_completed"] is None


def test_an_operator_stop_is_not_followed_by_the_opening(tmp_path):
    f = Jaw(OPEN); (tmp_path / "s.STOP").write_text("")
    code, d = go(tmp_path, f)
    assert d["controller"]["close"]["result"] in ("STOPPED_HOLDING", "REFUSED") and d["controller"]["open"] is None
