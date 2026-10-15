"""Error-injection tests of the ARCHIVED commissioning run against a fake bus (no device): preflight, release, exit codes."""
import json
from pathlib import Path
import pytest
import sweepick_commission_v2_archived as c, sweepick_real_command as rc

IN = Path.home() / "sweepick_261007_commission/inputs"
pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")
START = dict(shoulder_pan=2040, shoulder_lift=1031, elbow_flex=3150, wrist_flex=2043, wrist_roll=2116, gripper=3376)
LO = dict(shoulder_pan=726, shoulder_lift=772, elbow_flex=941, wrist_flex=866, wrist_roll=0, gripper=2001)
HI = dict(shoulder_pan=3406, shoulder_lift=3321, elbow_flex=3153, wrist_flex=3160, wrist_roll=4095, gripper=3431)


class Fake:
    """fail: dict of injected faults. Records every write."""
    def __init__(self, **fail):
        self.reg = dict(Goal_Position={n: 0 for n in rc.JOINTS}, Present_Position=dict(START), Torque_Enable={n: 0 for n in rc.JOINTS}, Protection_Current={n: 310 for n in rc.JOINTS}, Min_Position_Limit=dict(LO), Max_Position_Limit=dict(HI))
        self.fail, self.writes, self.n_goal, self.dead, self.disconnected = fail, [], 0, False, None
    def connect(self):
        if self.fail.get("connect"): raise ConnectionError("no port")
    def disconnect(self, disable_torque=True): self.disconnected = disable_torque
    def sync_read(self, reg, normalize=True):
        assert normalize is False
        if self.dead: raise ConnectionError("bus dead")
        if reg in ("Present_Velocity", "Present_Load", "Present_Current"):
            return {n: (self.fail.get("current", 0) if reg == "Present_Current" else 0) for n in rc.JOINTS}
        if reg == "Goal_Position" and self.fail.get("goal_readback") and self.n_goal:   # wrong only after the goals were written
            return {n: 7 for n in rc.JOINTS}
        return dict(self.reg[reg])
    def write(self, reg, n, v, normalize=True):
        assert normalize is False and reg in ("Goal_Position", "Torque_Enable")
        if self.dead: raise ConnectionError("bus dead")
        if reg == "Torque_Enable" and v == 0 and n in self.fail.get("release_fail", ()):
            self.fail["release_fail_count"] = self.fail.get("release_fail_count", 0) + 1
            if self.fail["release_fail_count"] <= self.fail.get("release_fail_times", 99): raise ConnectionError("no status packet")
        if reg == "Goal_Position":
            self.n_goal += 1
            if self.fail.get("write_fail_at") == self.n_goal: raise ConnectionError("write timeout")
            if self.fail.get("die_at") == self.n_goal: self.dead = True; raise ConnectionError("bus dead")
        self.writes.append((reg, n, v)); self.reg[reg][n] = v
        if reg == "Goal_Position" and self.reg["Torque_Enable"][n] and n != self.fail.get("stuck") and not (self.fail.get("no_return") and self.n_goal > self.fail["no_return"]):
            self.reg["Present_Position"][n] = v


def go(tmp_path, fake, **kw):
    mp = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")
    return c.run("left", tmp_path / "out", open_bus=lambda port: fake, owner=kw.pop("owner", lambda p: ""), mapping=mp, sleep=lambda s: None, **kw)


def test_normal_run_completes_and_releases(tmp_path):
    f = Fake(); code, log = go(tmp_path, f)
    assert code == 0 and log["result"] == "COMPLETED" and log["release"]["confirmed"] is True and log["end"]["returned"] and f.disconnected is False
    assert json.loads((tmp_path / "out/commission.json").read_text())["exit_code"] == 0


@pytest.mark.parametrize("setup, text", [(dict(torque_on=True), "torque is already on"), (dict(moving=True), "not at rest"), (dict(limit=True), "limits differ"), (dict(owner=True), "held by process"), (dict(connect=True), "could not be opened")])
def test_refusal_before_the_first_write_preserves_the_state(tmp_path, setup, text):
    f = Fake(connect=setup.get("connect", False))
    if setup.get("torque_on"): f.reg["Torque_Enable"]["elbow_flex"] = 1          # e.g. another controller holds the arm
    if setup.get("limit"): f.reg["Max_Position_Limit"]["elbow_flex"] = 3000
    if setup.get("moving"):
        real = f.sync_read; k = [0]
        def moving(reg, normalize=True):
            r = real(reg, normalize)
            if reg == "Present_Position": k[0] += 1; r["shoulder_pan"] += k[0]
            return r
        f.sync_read = moving
    before = json.dumps(f.reg, sort_keys=True)
    code, log = go(tmp_path, f, delta=setup.get("delta", 20), owner=(lambda p: "1234") if setup.get("owner") else (lambda p: ""))
    assert code == 2 and log["result"] == "REFUSED" and text in log["refused"]
    assert f.writes == [] and json.dumps(f.reg, sort_keys=True) == before          # nothing written: torque that was on stays on
    assert log["writes"] == dict(Goal_Position=0, Torque_Enable=0)


def test_goal_readback_mismatch_restores_the_goal_register_and_enables_nothing(tmp_path):
    f = Fake(goal_readback=True); code, log = go(tmp_path, f)
    assert log["abort_reason"].startswith("Abort: goal read-back") and log["stop_procedure"]["goal_register_restored"]
    assert f.reg["Goal_Position"] == {n: 0 for n in rc.JOINTS} and all(v == 0 for v in f.reg["Torque_Enable"].values()) and not any(r == "Torque_Enable" for r, _, _ in f.writes)
    assert code in (3, 4) and code != 0


def test_stuck_joint_aborts_walks_back_and_reports_returned(tmp_path):
    f = Fake(stuck="elbow_flex"); c.TRACK_ABORT, keep = 5, c.TRACK_ABORT
    try: code, log = go(tmp_path, f)
    finally: c.TRACK_ABORT = keep
    assert code == 3 and log["result"] == "ABORTED_RETURNED" and "tracking error" in log["abort_reason"] and log["stop_procedure"]["held"] and log["end"]["returned"] and log["release"]["confirmed"]


def test_write_failure_mid_move_aborts_and_returns(tmp_path):
    f = Fake(write_fail_at=30); code, log = go(tmp_path, f)
    assert code == 3 and "write timeout" in log["abort_reason"] and log["end"]["returned"] and all(v == 0 for v in f.reg["Torque_Enable"].values())


def test_no_return_after_the_error_is_reported_not_assumed(tmp_path):
    f = Fake(write_fail_at=30, no_return=30); code, log = go(tmp_path, f)       # after the fault the joint no longer follows the goal
    assert code == 4 and log["result"] == "ABORTED_NOT_RETURNED" and log["end"]["returned"] is False and log["release"]["confirmed"] is True


def test_overcurrent_aborts(tmp_path):
    f = Fake(current=200); code, log = go(tmp_path, f)
    assert code == 3 and "protection current" in log["abort_reason"]


def test_one_motor_refusing_release_does_not_stop_the_others_and_is_never_reported_off(tmp_path):
    f = Fake(release_fail=("elbow_flex",)); code, log = go(tmp_path, f)
    assert code == 5 and log["result"] == "TORQUE_RELEASE_UNCONFIRMED" and "needs_a_person" in log
    assert f.reg["Torque_Enable"]["elbow_flex"] == 1 and all(v == 0 for n, v in f.reg["Torque_Enable"].items() if n != "elbow_flex")
    assert len(log["release"]["failures"]) == c.RELEASE_TRIES and log["release"]["confirmed"] is False


def test_release_retry_succeeds_on_a_transient_failure(tmp_path):
    f = Fake(release_fail=("elbow_flex",), release_fail_times=1); code, log = go(tmp_path, f)
    assert code == 0 and log["release"]["confirmed"] is True and len(log["release"]["failures"]) == 1


def test_dead_bus_mid_move_is_unconfirmed_release(tmp_path):
    f = Fake(die_at=30); code, log = go(tmp_path, f)
    assert code == 5 and log["release"]["confirmed"] is False and "end state could not be read" in log["end"]["error"] and log["stop_procedure"]["error"]
