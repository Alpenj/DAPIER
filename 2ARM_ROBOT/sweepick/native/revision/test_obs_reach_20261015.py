"""20261015_140844_3cc05c RIGHT observation moves 03/04 on the existing fake bus (no device): the loaded elbow settles
26 / 23 ticks short. Start/target ticks are the stored move.json values; the elbow's resting shortfall is the fake's
existing `lag` behaviour set to the stored shortfall (a model of the outcome, not a measured servo law)."""
import json
from pathlib import Path
import pytest
import sweepick_move_a as a, sweepick_real_command as rc, sweepick_progress as PG
from test_sweepick_move_a import Fake, Clock, IN

pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
S03 = dict(shoulder_pan=1621, shoulder_lift=1066, elbow_flex=2741, wrist_flex=2692, wrist_roll=1083, gripper=3010)
T03 = dict(shoulder_pan=1621, shoulder_lift=989, elbow_flex=2671, wrist_flex=2827, wrist_roll=1088)
T04 = dict(shoulder_pan=1626, shoulder_lift=995, elbow_flex=2666, wrist_flex=2834, wrist_roll=500)
MP = None


def mapping():
    global MP
    MP = MP or rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")
    return MP


def powered(start, lag):
    f = Fake(dict(start), lag=lag)
    mp = mapping()["right"]                                                     # the RIGHT arm's own calibration snapshot as the device limits
    f.reg["Min_Position_Limit"] = {n: mp[n]["lo"] for n in rc.JOINTS}; f.reg["Max_Position_Limit"] = {n: mp[n]["hi"] for n in rc.JOINTS}
    for n in ARM:
        f.reg["Torque_Enable"][n] = 1; f.reg["Goal_Position"][n] = start[n]
    return f


def view(tmp, f, target, **kw):
    c = Clock()
    return a.run("next", "right", tmp, target=target, torque_joints=ARM, open_bus=lambda port: f, owner=lambda p: "", mapping=mapping(), mapping_sha="m",
                 sleep=c.sleep, clock=c.now, execute=True, **kw)


def test_observation_view_03_completes_settles_and_misses_by_26(tmp_path):
    f = powered(S03, lag=26)
    code, log = view(tmp_path / "o3", f, T03, observation_view=True, settle_trim=())
    r = log["reached"]["elbow_flex"]; v = log["observation_view"]
    assert log["result"] == "TARGET_NOT_REACHED_HOLDING" and code == a.EXIT["TARGET_NOT_REACHED_HOLDING"]
    assert r["minus_target"] == 26.0 and r["within_tolerance"] is False and r["settled"] is True and r["reached"] is False
    assert log["outcome"]["command_completed"] is True and log["outcome"]["target_reached"] is False
    assert v["command_completed"] is True and v["actual_settled"] is True and v["command_state_resolved"] is True and v["nominal_target_reached"] is False
    assert log["settle_trim"] == [] and f.reg["Goal_Position"]["elbow_flex"] == T03["elbow_flex"]      # the Goal stays the uncompensated view target
    assert abs(v["goal_actual_gap_ticks"]["elbow_flex"]) == 26 <= PG.load()["follow"]


def test_observation_view_04_starts_from_the_latest_actual_and_misses_by_23(tmp_path):
    f = powered(S03, lag=26); view(tmp_path / "o3", f, T03, observation_view=True, settle_trim=())
    actual = dict(f.reg["Present_Position"]); assert actual["elbow_flex"] == 2697
    f.fail["lag"] = 23
    code, log = view(tmp_path / "o4", f, T04, observation_view=True, settle_trim=())
    assert log["start"]["ticks"]["elbow_flex"] == 2697 and log["start"]["ticks"] == {n: float(v) for n, v in actual.items()}   # the next path is built from the measured end of 03, not from T03
    traj = json.loads((tmp_path / "o4/TRAJECTORY.json").read_text()) if (tmp_path / "o4/TRAJECTORY.json").exists() else log["trajectory"]
    assert (traj.get("start") or {}).get("elbow_flex", 2697) == 2697
    r = log["reached"]["elbow_flex"]
    assert log["result"] == "TARGET_NOT_REACHED_HOLDING" and r["minus_target"] == 23.0 and r["travelled"] == -8.0 and r["settled"] is True


def test_the_same_move_with_the_existing_settle_trim_reaches_but_moves_the_goal_off_the_view_target(tmp_path):
    """option B: enable the existing integral trim. It reaches, but the Goal register ends beyond the certified view
    target, which the full-chain observation contract refuses ('applied goal differs from the uncompensated view target')."""
    f = powered(S03, lag=26)
    code, log = view(tmp_path / "t3", f, T03, settle_trim=["elbow_flex"])
    t = log["settle_trim"]
    assert t and t[0]["applied"] is True and log["result"] == "HOLDING", (log["result"], t)
    assert f.reg["Goal_Position"]["elbow_flex"] != T03["elbow_flex"]                                     # goal = target - shortfall
    assert abs(f.reg["Present_Position"]["elbow_flex"] - T03["elbow_flex"]) <= PG.load()["reach"]
