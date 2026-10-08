"""Pure tests of the model <-> tick mapping and the command limits (no device)."""
import json, math
from pathlib import Path
import numpy as np, pytest
from sweepick.control import sweepick_joint_command_mapping as rc

IN = Path.home() / "sweepick_261007_commission/inputs"
CAL = Path.home() / ".config/dapier/lerobot-calibration"
pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists() or not (CAL / "dapier_dual_follower_left.json").exists(), reason="candidate config / calibration not on this machine")


@pytest.fixture(scope="module")
def mp():
    return rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", CAL, IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")


def test_round_trip_is_exact_over_the_whole_calibrated_range(mp):
    for side in mp:
        for n, m in mp[side].items():
            for t in np.linspace(m["lo"], m["hi"], 37):
                assert abs(rc.model_to_tick(n, m, rc.tick_to_model(n, m, t)) - t) < 1e-9


def test_inverse_reproduces_the_stored_real_sample(mp):
    """The q12 that the field tool computed from the stored ticks maps back to those ticks (same formula, other direction)."""
    q = np.load(IN / "model_input_r6.npz")["q12"]
    ticks = rc.q12_to_ticks(mp, q)
    want = dict(left=dict(shoulder_pan=2040, shoulder_lift=1031, elbow_flex=3150, wrist_flex=2043, wrist_roll=2116, gripper=3376), right=dict(shoulder_pan=2058, shoulder_lift=1064, elbow_flex=3138, wrist_flex=2030, wrist_roll=2105, gripper=1501))
    for s in want:
        for n in want[s]:
            assert abs(ticks[s][n] - want[s][n]) < 0.01, (s, n, ticks[s][n])


def test_limits_refuse_and_never_clip(mp):
    m = mp["left"]["shoulder_lift"]
    assert rc.check_command(mp, "left", "shoulder_lift", 1031, 1031, margin_ticks=20, max_step_ticks=12) == []
    assert any("calibrated range" in w for w in rc.check_command(mp, "left", "shoulder_lift", m["lo"] + 5, m["lo"] + 5, margin_ticks=20, max_step_ticks=12))
    assert any("step" in w for w in rc.check_command(mp, "left", "shoulder_lift", 1100, 1031, margin_ticks=20, max_step_ticks=12))
    assert any("SIM joint range" in w for w in rc.check_command(mp, "left", "shoulder_pan", mp["left"]["shoulder_pan"]["hi"] - 25, mp["left"]["shoulder_pan"]["hi"] - 25, margin_ticks=20, max_step_ticks=12))
    assert rc.check_command(mp, "left", "gripper", float("nan"), 3376, margin_ticks=20, max_step_ticks=12) == ["non-finite goal"]


def test_dry_run_sends_nothing_and_reports_refusals(mp):
    chunk = np.load(IN / "shadow_r6_chunk_START_3V.npy")
    present = dict(left=dict(shoulder_pan=2040, shoulder_lift=1031, elbow_flex=3150, wrist_flex=2043, wrist_roll=2116, gripper=3376), right=dict(shoulder_pan=2058, shoulder_lift=1064, elbow_flex=3138, wrist_flex=2030, wrist_roll=2105, gripper=1501))
    r = rc.dry_run_chunk(mp, chunk, present)
    assert r["commands_sent"] == 0 and r["steps"] == 25
    bad = chunk.copy(); bad[3, 1] = -3.0
    assert any(x["step"] == 3 and x["joint"] == "shoulder_lift" for x in rc.dry_run_chunk(mp, bad, present)["refused"])


def test_classification_separates_hard_from_temporary(mp):
    present = dict(left=dict(shoulder_pan=2040, shoulder_lift=1031, elbow_flex=3150, wrist_flex=2043, wrist_roll=2116, gripper=3376), right=dict(shoulder_pan=2058, shoulder_lift=1064, elbow_flex=3138, wrist_flex=2030, wrist_roll=2105, gripper=1501))
    s = rc.classify_chunk(mp, np.load(IN / "shadow_r6_chunk_START_3V.npy"), present)
    r = rc.classify_chunk(mp, np.load(IN / "shadow_r6_chunk_RECEIVE_3V.npy"), present)
    assert not any(v["class"].startswith("HARD") for v in s.values()) and s["left elbow_flex"]["class"].startswith("DIAGNOSTIC only")
    assert r["left elbow_flex"]["class"] == "HARD: outside the calibrated range" and r["left gripper"]["class"].startswith("DIAGNOSTIC only")


def test_arms_only_report_uses_only_the_range_gates_and_heuristics_are_diagnostics(mp):
    present = dict(left=dict(shoulder_pan=2040, shoulder_lift=1031, elbow_flex=3150, wrist_flex=2043, wrist_roll=2116, gripper=3376), right=dict(shoulder_pan=2058, shoulder_lift=1064, elbow_flex=3138, wrist_flex=2030, wrist_roll=2105, gripper=1501))
    bad = rc.plan_arms_only(mp, np.load(IN / "shadow_r6_chunk_RECEIVE_3V.npy"), present)                       # left elbow beyond its configured maximum
    assert not bad["in_range"] and bad["commands_sent"] == 0 and any(x["joint"] == "elbow_flex" and x["gate"] == "servo configured range" for x in bad["hard_failures"])
    s = rc.plan_arms_only(mp, np.load(IN / "shadow_r6_chunk_START_3V.npy"), present)                           # inside the range, 3 ticks from its end
    assert s["in_range"] and s["hard_failures"] == [] and any("near the range end" in d["kind"] for d in s["diagnostics"]) and s["executable"] is None
    assert all(r["left"]["gripper"] == 3376 and r["right"]["gripper"] == 1501 for r in s["sequence"])
