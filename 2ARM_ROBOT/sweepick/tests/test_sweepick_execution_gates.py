"""Every refusal / stop in the real execution path names a gate from sweepick_HARD_GATE_ALLOWLIST.json. The list manages what each
gate rests on, where it applies and how the executor reacts; it does not ban numbers. What stays banned: a cap on the total
travel under any name, a per-object opening, and the removed 30-tick / 2-tick / 50 % gates coming back under another name."""
import json, re
from pathlib import Path
import pytest
from sweepick.control import sweepick_execution_gates as G

from sweepick.integration.sweepick_resource_paths import source_path
HERE = Path(__file__).resolve().parent
ALLOW = json.loads(source_path("sweepick_HARD_GATE_ALLOWLIST.json").read_text())
PATH = ("sweepick_move_a.py", "sweepick_real_exec.py", "sweepick_progress.py", "sweepick_trajectory.py", "sweepick_collision_cert.py", "sweepick_local_loop.py", "sweepick_kin.py", "sweepick_gates.py")
SRC = {f: source_path(f).read_text() for f in PATH}
code = lambda s: "\n".join(l.split("#", 1)[0] if not l.strip().startswith(('"', "'")) else l for l in s.splitlines())


def test_every_gate_name_in_the_execution_path_is_on_the_allowlist():
    for f, s in SRC.items():
        for kind, section in (("hard", "hard"), ("contract", "contract"), ("provisional", "provisional"), ("diagnostic", "diagnostic")):
            for name in re.findall(r"G\.%s\(\s*\"([a-z_]+)\"" % kind, s):
                assert name in ALLOW[section], f"{f}: {kind} gate '{name}' is not on the allowlist"


def test_a_gate_name_off_the_list_cannot_be_created():
    for bad in ("tracking_error_ticks", "behind_start", "current_over_half_protection", "feedback_timeout", "max_travel"):
        with pytest.raises(G.GateError):
            G.hard(bad)
    with pytest.raises(G.GateError):
        G.provisional("tracking_error_ticks")


def test_every_stop_and_refusal_site_names_a_gate():
    for f in ("sweepick_move_a.py", "sweepick_real_exec.py"):
        for i, line in enumerate(SRC[f].splitlines(), 1):
            l = line.split("#", 1)[0]
            if re.search(r"raise Stop\(", l):
                assert re.search(r"G\.(hard|provisional|contract)\(|check\[\"(hard|provisional)\"\]\[0\]|recheck\[\"hard\"\]\[0\]", l), f"{f}:{i} stops without an allowlisted gate"
            if "refuse(" in l and "def refuse" not in l:
                assert re.search(r"G\.(hard|provisional|contract)\(|e\.gate|p\.gate", l), f"{f}:{i} refuses without an allowlisted gate"
            if 'Dispatch("STOP"' in l:
                assert "[g]" in l or "[gate]" in l or "G.hard(" in l or "G.contract(" in l, f"{f}:{i} returns STOP without an allowlisted gate"


def test_forbidden_threshold_symbols_are_gone_from_the_execution_path():
    forbidden = (r'lim\["track"\]', r'lim\["behind"\]', r"current_fraction", r"feedback_timeout", r"MAX_TRAVEL", r"TRACK_BOUND", r"TRACK_ABORT", r"DRIFT_BOUND", r"DRIFT_ABORT", r"HOLD_DRIFT", r"DEEPER_M", r"max_excursion", r"behind_start_ticks", r"tracking_error_ticks")
    for f, s in SRC.items():
        for pat in forbidden:
            assert not re.search(pat, code(s)), f"{f}: forbidden threshold symbol {pat}"


def test_numbers_that_can_stop_a_run_come_from_a_profile_that_states_basis_and_scope():
    """A numeric condition may stop or hold a run when it is part of the selected execution contract. Its value lives in a
    profile file with its basis and scope, not as a bare literal next to the gate."""
    prog = json.loads(source_path("sweepick_progress_profile.json").read_text())
    for k in ("resolution_ticks", "start_delay_s", "stationary_lead_ticks", "reach_tolerance_ticks"):
        assert prog[k]["basis"] and isinstance(prog[k]["value"], (int, float))
    assert prog["scope"] and prog["reaction"] and "NOT ratings of the servo" in prog["what"] and source_path("sweepick_PROGRESS_EVIDENCE.json").exists()
    for name, text in ALLOW["contract"].items():
        assert len(text) > 40, f"contract gate '{name}' does not say what it rests on"
    for f, s in SRC.items():
        lines = s.splitlines()
        for i, line in enumerate(lines):
            if "G.contract(" not in line:
                continue
            cond = next((lines[k] for k in range(i, max(i - 4, -1), -1) if re.match(r"\s*(if|elif)\b", lines[k])), "").split("#", 1)[0]
            nums = [n for n in re.findall(r"(?:<=|>=|<|>|==|!=)\s*(-?\d+(?:\.\d+)?)\b", cond) if float(n) != 0]
            assert not nums, f"{f}:{i + 1} a CONTRACT gate is guarded by a bare literal {nums}; put the value in a profile with its basis: {cond.strip()}"


def test_no_total_travel_cap_and_no_removed_gate_under_another_name():
    assert any("total travel" in x for x in ALLOW["forbidden"]) and any("per object" in x for x in ALLOW["forbidden"])
    for f, s in SRC.items():
        c = code(s)
        assert not re.search(r"(?i)\b(max|limit|cap)_?(total_?)?(travel|excursion|distance|degrees?)\b\s*=\s*\d", c), f"{f}: looks like a travel cap"
        assert not re.search(r"(?i)opening_for_|object_width_to_opening|OPENING_BY_OBJECT", c), f"{f}: per-object opening"
    prog = json.loads(source_path("sweepick_progress_profile.json").read_text())
    assert "do not limit how far a move may go" in prog["what"]


def test_a_stuck_joint_is_a_stop_not_a_pass_and_missing_data_is_not_normal():
    for name in ("no_progress", "target_not_reached", "measurement_invalid", "status_unavailable", "profile_violation"):
        assert G.contract(name)["level"] == "CONTRACT"
    assert "no_motion_observed" not in ALLOW["diagnostic"]                       # not moving is no longer just a note written after the whole command was sent
    assert 'G.contract("no_progress"' in SRC["sweepick_move_a.py"] and 'G.contract("no_progress"' in SRC["sweepick_real_exec.py"] and 'G.contract("measurement_invalid"' in SRC["sweepick_real_exec.py"]


def test_recovery_heuristics_are_used_only_after_a_stop():
    s = SRC["sweepick_move_a.py"]
    start = s.index("# ---- RECOVERY")
    for name in ALLOW["recovery"]:
        uses = [m.start() for m in re.finditer(r"\b%s\b" % name, s)]
        body = [u for u in uses if "def release" not in s[max(0, u - 400):u] and s[:u].count("\n") > 40]        # skip the definition lines and the release helper
        assert all(u > start or "heuristics" in s[u - 60:u + 60] for u in body if name != "RELEASE_TRIES"), f"{name} is used before the recovery section"


def test_motion_profile_file_holds_profile_parameters_not_limits():
    d = json.loads(source_path("sweepick_motion_profile.json").read_text())
    assert all(k.startswith("profile_") or k in ("schema", "what", "period_s", "device_registers_related_to_speed") for k in d) and "NOT physical limits" in d["what"]
    assert not source_path("sweepick_motion_limits.json").exists() or "sweepick_motion_limits.json" not in "".join(code(s) for s in SRC.values())
