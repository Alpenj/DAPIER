"""Pre-approved vs manual observation approval on the fake bus: the approval mode changes only who answers, never the
commands. Forward targets and start ticks are the stored REAL request of 20261015_151937_81d00c."""
import copy, os, sys, json
from pathlib import Path
import pytest
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
os.environ.setdefault("SWEEPICK_RECEIVE_DIR", str(HERE.parent / "pin"))
import RUN_APPROVED_SESSION as runner
import sweepick_move_a as a
from test_obs_reach_20261015 import powered, mapping, ARM, IN
from test_sweepick_move_a import Clock
sys.path.insert(0, str(HERE.parent))
import test_preapproved_view as pv

pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")


def drive(tmp, approved):
    """the owner's next step after the decision: the forward targets on the fake right arm (lag 0)"""
    if not approved:
        return []
    start = {n: float(round(v)) for n, v in pv.REQ["actual_feedback"]["ticks"]["right"].items()}
    f = powered(start, lag=0)
    for k, step in enumerate(pv.REQ["plan"]["forward"]):
        c = Clock()
        code, log = a.run("next", "right", tmp / f"step{k}", target={n: float(v) for n, v in step["target_ticks"].items()}, torque_joints=ARM,
                          open_bus=lambda port: f, owner=lambda p: "", mapping=mapping(), mapping_sha="m", sleep=c.sleep, clock=c.now, execute=True,
                          observation_view=True, settle_trim=())
        assert log["result"] in ("HOLDING", "TARGET_NOT_REACHED_HOLDING"), log.get("refused") or log.get("stopped_by")
    return f.writes


def test_preapproved_and_manual_modes_send_the_same_commands(tmp_path):
    (tmp_path / "m").mkdir(); (tmp_path / "p").mkdir()
    manual = drive(tmp_path / "m", pv.authorize(pv.Owner(tmp_path / "m", decision=True), None))
    pre = drive(tmp_path / "p", pv.authorize(pv.Owner(tmp_path / "p", decision=None), pv.pre(tmp_path / "p")))
    assert manual and manual == pre


def test_bus_fault_at_the_request_never_auto_moves(tmp_path):
    owner = pv.Owner(tmp_path, decision=None)
    plan, raw, cand = pv.inputs(); raw["status"]["left"]["gripper"] = 32.0
    with pytest.raises(TimeoutError):                                                   # falls back to the operator decision; nothing is driven
        runner.authorize_observation_plan(owner, plan, raw, cand, preapproval=pv.pre(tmp_path), manifest_sha=pv.MSHA)
    note = next(n for n in owner.notes if n["kind"] == "observation_move_preapproval_not_applicable")
    assert note["checks"]["bus_status_zero"] is False and not (tmp_path / "OBSERVATION_MOVE_APPROVAL.json").exists()


@pytest.mark.parametrize("purpose", ["RIGHT_INSERT_REPLAN", "RIGHT_CLOSE"])
def test_right_insert_and_close_stay_refused_in_the_observation_scope(purpose):
    import sweepick_full_chain as fc
    owner = type("O", (), {})(); owner.scope, owner.contact_plan = "contact", {"observation_only": True}
    with pytest.raises(RuntimeError, match="outside this observation-only"):
        fc.Owner.segment(owner, "x", "right", target={"gripper": 2000}, purpose=purpose)
