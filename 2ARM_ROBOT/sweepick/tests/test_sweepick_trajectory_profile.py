from sweepick.integration.sweepick_resource_paths import source_path
"""Trajectory regression: a move of any size passes when the whole trajectory passes the physical gates, and only then."""
import math
from pathlib import Path
import numpy as np, pytest
from sweepick.control import sweepick_joint_command_mapping as rc; from sweepick.control import sweepick_trajectory_profile as tt

IN = Path.home() / "sweepick_261007_commission/inputs"
pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")
START = dict(shoulder_pan=2040, shoulder_lift=1500, elbow_flex=2400, wrist_flex=2043, wrist_roll=2116, gripper=2700)


@pytest.fixture(scope="module")
def mp():
    return rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")["left"]


LIM = tt.load_profile()
clear = lambda side, traj: dict(identity=tt.identity(side, traj, "m"), nominal_verdict="CLEAR")


@pytest.mark.parametrize("deg", [4, 10, 15, 20, 45])
def test_any_size_is_executable_at_device_level_without_a_model(mp, deg):
    traj = tt.make(START, dict(elbow_flex=START["elbow_flex"] - deg * rc.TICKS / 360), LIM)
    v = tt.validate("left", mp, traj, LIM, mapping_sha="m", level="device")
    assert v["executable"] and v["hard"] == [] and v["provisional"] == [] and v["total_travel_limit"] is None and abs(traj["travel_deg"]["elbow_flex"] - deg) < 1e-9


def test_the_generated_path_keeps_the_selected_profile_and_a_path_that_breaks_it_is_not_executable(mp):
    traj = tt.make(START, dict(elbow_flex=1900), LIM)
    p = tt.peaks(traj)["elbow_flex"]
    assert p["v"] <= LIM["v"] * 1.000001 and p["a"] <= LIM["a"] * 1.000001 and p["j"] <= LIM["j"] * 1.000001
    assert tt.validate("left", mp, traj, LIM, mapping_sha="m", level="device")["executable"]
    fast = dict(traj, goals={n: g[::4] for n, g in traj["goals"].items()}, steps=len(traj["goals"]["elbow_flex"][::4]))     # the same path in a quarter of the time
    v = tt.validate("left", mp, fast, LIM, mapping_sha="m", level="device")
    assert not v["executable"] and any(g["gate"] == "profile_violation" and g["level"] == "CONTRACT" for g in v["hard"])
    short = dict(traj, dt=traj["dt"] / 100)                                                                                 # the reviewer's case: the time axis 100 times shorter
    v = tt.validate("left", mp, short, LIM, mapping_sha="m", level="device")
    assert not v["executable"] and any(g["gate"] == "profile_violation" for g in v["hard"])
    again = tt.make(START, dict(elbow_flex=1900), LIM)                                                                      # retimed by the generator: executable again
    assert tt.validate("left", mp, again, LIM, mapping_sha="m", level="device")["executable"]


def test_only_the_servo_range_is_hard_and_the_model_range_is_provisional_at_model_level(mp):
    t1 = tt.make(START, dict(elbow_flex=mp["elbow_flex"]["hi"] + 5), LIM)
    v1 = tt.validate("left", mp, t1, LIM, mapping_sha="m", level="device")
    assert not v1["executable"] and [g["gate"] for g in v1["hard"]] == ["servo_range"] and max(t1["goals"]["elbow_flex"]) > mp["elbow_flex"]["hi"]
    t2 = tt.make(START, dict(shoulder_pan=mp["shoulder_pan"]["hi"] - 2), LIM)          # inside the servo range, outside the model's +-110 deg
    dev = tt.validate("left", mp, t2, LIM, mapping_sha="m", level="device")
    mod = tt.validate("left", mp, t2, LIM, mapping_sha="m", level="model", cert=clear("left", t2))
    assert dev["executable"] and any(d.get("diagnostic") == "model_range_candidate" for d in dev["diagnostics"])
    assert not mod["executable"] and mod["hard"] == [] and [g["gate"] for g in mod["provisional"]] == ["model_joint_range_candidate"]


def test_being_near_the_range_end_is_a_diagnostic(mp):
    s = dict(START, elbow_flex=mp["elbow_flex"]["hi"] - 4)
    traj = tt.make(s, dict(elbow_flex=mp["elbow_flex"]["hi"] - 12), LIM)
    v = tt.validate("left", mp, traj, LIM, mapping_sha="m", level="device")
    assert v["executable"] and any(d.get("diagnostic") == "distance_to_range_end" and d["ticks"] == 4 for d in v["diagnostics"])


def test_model_level_needs_the_nominal_collision_result_of_exactly_this_trajectory(mp):
    traj = tt.make(START, dict(elbow_flex=2000), LIM)
    gate = lambda cert, **kw: [g["gate"] for g in tt.validate("left", mp, traj, LIM, mapping_sha="m", level="model", cert=cert, **kw)["provisional"]]
    assert gate(None) == ["collision_candidate_frame"]
    other = tt.make(dict(START, elbow_flex=2405), dict(elbow_flex=2000), LIM)
    assert gate(clear("left", other)) == ["certificate_binding"]
    assert gate(dict(clear("left", traj), nominal_verdict="CONTACT", first_nominal_contact=dict(pair="forearm / table"))) == ["collision_candidate_frame"]
    moved = dict(START, shoulder_lift=START["shoulder_lift"] + 200)
    assert gate(dict(clear("left", traj), other_arm_ticks=dict(START)), other_arm_ticks=moved) == ["certificate_binding"]
    assert gate(clear("left", traj)) == []
    tube = dict(clear("left", traj), tube_diagnostic=dict(displacement_ticks=30, contacts=2, first=dict(pair="x")))          # contacts off the commanded path
    v = tt.validate("left", mp, traj, LIM, mapping_sha="m", level="model", cert=tube)
    assert v["executable"] and any(d.get("diagnostic") == "collision_tube" for d in v["diagnostics"])


def test_several_joints_share_one_duration_set_by_the_slowest(mp):
    traj = tt.make(START, dict(shoulder_lift=1900, elbow_flex=2300, wrist_flex=2100), LIM)
    assert traj["moving"] == ["shoulder_lift", "elbow_flex", "wrist_flex"] and traj["held"] == ["shoulder_pan", "wrist_roll", "gripper"]
    assert abs(traj["duration_s"] - math.ceil(tt.duration_for(400 * tt.TICK_RAD, LIM) / LIM["dt"]) * LIM["dt"]) < 1e-9
    assert all(abs(traj["goals"][n][-1] - traj["target"][n]) < 1e-9 for n in traj["moving"]) and set(traj["goals"]["gripper"]) == {2700.0}


def test_a_large_raw_target_step_is_reported_with_its_retimed_duration_not_refused(mp):
    r = tt.retime_report(mp, dict(gripper=3097.0), dict(gripper=3372.0), LIM)["gripper"]
    assert r["raw_step_ticks"] == 275 and r["duration_s_within_limits"] > 1.0 and r["control_periods"] > 50


def test_no_travel_cap_is_left_in_the_command_code():
    for f in ("sweepick_move_a.py", "sweepick_trajectory.py", "sweepick_real_command.py", "sweepick_commission.py", "sweepick_commission_v2_archived.py"):
        s = source_path(f).read_text()
        assert "MAX_TRAVEL" not in s and "max_excursion" not in s and "delta outside" not in s, f
