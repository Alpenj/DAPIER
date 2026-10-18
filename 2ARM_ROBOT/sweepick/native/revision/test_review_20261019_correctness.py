"""Correctness fixes after the d98fd9a review (2026-10-19): STOP reaches the bounded seed re-send and the re-read waits, a missing
observation is never a pass, ARM5 coverage, exact zero, finite extrinsic. Fake bus / pure functions only; no device is opened."""
import json
from pathlib import Path

import numpy as np

import sweepick_pick01 as P
import sweepick_setup as setup
from test_sweepick_move_a import Fake, SeedLost, MID, go


# ---- STOP before the one bounded seed re-send ------------------------------------------------------------------------------
class SeedLostThenStop(SeedLost):
    """the seed answer is lost and the operator presses STOP while the executor reads back"""
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw); self.stop = False
    def write(self, reg, n, v, normalize=True):
        if reg == "Goal_Position" and n == self.joint and self.lose > 0:
            self.stop = True
        super().write(reg, n, v, normalize)


def test_stop_during_the_lost_seed_answer_sends_no_re_send(tmp_path):
    f = SeedLostThenStop(MID, "elbow_flex", applied=False)
    code, log = go(tmp_path / "o", f, target=dict(elbow_flex=MID["elbow_flex"] - 228), execute=True, stop_requested=lambda: f.stop)
    seeds = [w for w in f.writes if w[0] == "Goal_Position" and w[1] == "elbow_flex"]
    assert seeds == [] and log["result"] != "HOLDING"
    assert any(isinstance(e, dict) and e.get("recovery") == "NOT_ATTEMPTED: stop requested" for e in log["events"]) or log.get("stopped_by", {}).get("gate") == "operator_stop"


def test_lost_seed_without_stop_is_still_re_sent_once(tmp_path):            # positive control: the fix does not remove the recovery
    f = SeedLost(MID, "elbow_flex", applied=False)
    code, log = go(tmp_path / "o", f, target=dict(elbow_flex=MID["elbow_flex"] - 228), execute=True)
    assert log["result"] == "HOLDING"


# ---- STOP during the loop re-read waits --------------------------------------------------------------------------------------
class SilentAfter(Fake):
    """reads stop answering after `n_ok` reads; STOP is raised at the first failure"""
    def __init__(self, start, n_ok, **kw):
        super().__init__(start, **kw); self.n_ok, self.reads, self.stop = n_ok, 0, False
    def sync_read(self, reg, normalize=True):
        self.reads += 1
        if self.reads > self.n_ok:
            self.stop = True
            raise ConnectionError("Failed to sync read [TxRxResult] There is no status packet!")
        return super().sync_read(reg, normalize)


def test_stop_during_re_read_waits_is_a_stop_not_more_waiting(tmp_path):
    f = SilentAfter(MID, n_ok=60)
    code, log = go(tmp_path / "o", f, target=dict(elbow_flex=MID["elbow_flex"] - 228), execute=True, stop_requested=lambda: f.stop)
    assert log["result"] != "HOLDING"
    det = json.dumps(log.get("stopped_by") or {}) + json.dumps(log.get("refused") or {})
    assert "operator_stop" in det or "stop" in det.lower()


# ---- ARM5 coverage / exact zero / finite extrinsic ------------------------------------------------------------------------
def test_arm5_within_needs_all_five_finite_joints():
    five = {n: dict(minus_target=3.0) for n in P.ARM}
    assert P.arm5_within(five, 20.0)
    assert not P.arm5_within({"gripper": dict(minus_target=0.0)}, 20.0)                 # gripper only
    assert not P.arm5_within({"elbow_flex": dict(minus_target=0.0)}, 20.0)              # one joint
    assert not P.arm5_within(dict(five, wrist_flex=dict(minus_target=float("nan"))), 20.0)
    assert not P.arm5_within(dict(five, wrist_flex=dict(minus_target=None)), 20.0)
    assert not P.arm5_within(dict(five, wrist_flex=dict(minus_target=21.0)), 20.0)
    assert not P.arm5_within({}, 20.0)


def test_measured_head_extrinsic_rejects_non_finite_translation(tmp_path, monkeypatch):
    T = np.eye(4); T[0, 3] = float("nan")
    f = tmp_path / "x.json"; f.write_text(json.dumps(dict(schema="sweepick.head-extrinsic.v1", T_robot_from_camera=[[None if np.isnan(v) else v for v in r] for r in T.tolist()])))
    monkeypatch.setattr(setup, "MEASURED_HEAD_EXTRINSIC", f)
    try:
        setup.measured_head_extrinsic(); ok = False
    except Exception:
        ok = True
    assert ok
    T2 = np.eye(4); T2[0, 3] = 0.1; f.write_text(json.dumps(dict(schema="sweepick.head-extrinsic.v1", T_robot_from_camera=T2.tolist())))
    assert setup.measured_head_extrinsic() is not None                                  # positive control


# ---- the source carries the review rules ---------------------------------------------------------------------------------
SRC = Path(P.__file__).read_text()


def test_taught_gate_does_not_turn_missing_features_into_zero():
    assert 'e.get("row", 0.0)' not in SRC and 'e.get("ang", 0.0)' not in SRC
    assert '.get("ok", True) if gate == "ready_for_final_descent"' not in SRC


def test_exact_zero_lateral_error_is_not_missing():
    assert 'rec.get("lateral_error_mm") or 1e9' not in SRC


def test_carry_hold_needs_held_status():
    assert 'if w["status"] != "HELD"' in SRC and 'return res("UNKNOWN"' in SRC
