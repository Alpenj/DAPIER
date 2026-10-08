"""Model command -> motor ticks for the two SO-101 followers (candidate q12 mapping), with range checks and a dry-run REPORT.

margin_ticks / max_step_ticks in dry_run_chunk, classify_chunk and plan_arms_only are development heuristics used for
reports only. They are not gates of a real command and no total-travel limit exists: see sweepick_trajectory.py.

  q_model (rad, opening 0..1)  --inverse of the candidate alignment-->  driver (deg, percent)
                               --inverse of the lerobot calibration-->  raw tick (Goal_Position)

This module computes and checks; it opens no device. Sending is done by sweepick_commission.py only.
The mapping is the CANDIDATE config (verified = false). Limits come from existing sources: the calibration file's
range_min / range_max and the joint ranges of the assembled SIM model. Nothing is clipped silently: a command outside a
limit is REFUSED with its reason."""
import json, math
from pathlib import Path
import numpy as np

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
TICKS = 4096


def load_mapping(candidate_file, calibration_dir, model_file):
    cand = json.loads(Path(candidate_file).read_text())
    model = json.loads(Path(model_file).read_text())
    out = {}
    for side in ("left", "right"):
        cal = json.loads((Path(calibration_dir).expanduser() / f"dapier_dual_follower_{side}.json").read_text())
        rows = {}
        for j in cand[side]["joints"]:
            c = cal[j["name"]]
            if c.get("drive_mode"):
                raise ValueError("drive_mode != 0 is not handled here")
            rows[j["name"]] = dict(id=c["id"], sign=int(j["sign"]), offset_rad=float(j["offset_rad"]), lo=int(c["range_min"]), hi=int(c["range_max"]), model_range=model["arms"][side]["joints"][j["name"]]["range_rad"])
        g, c = cand[side]["gripper"], cal["gripper"]
        rows["gripper"] = dict(id=c["id"], closed=float(g["driver_percent_closed"]), open=float(g["driver_percent_open"]), lo=int(c["range_min"]), hi=int(c["range_max"]), model_range=[0.0, 1.0])
        out[side] = rows
    return out


def tick_to_model(name, m, tick):
    if name == "gripper":
        p = (tick - m["lo"]) / (m["hi"] - m["lo"]) * 100
        return (p - m["closed"]) / (m["open"] - m["closed"])
    d = (tick - (m["lo"] + m["hi"]) / 2) * 360 / (TICKS - 1)
    return m["sign"] * math.radians(d) + m["offset_rad"]


def model_to_tick(name, m, q):
    """Exact inverse of tick_to_model (float tick; rounding is done by the caller)."""
    if name == "gripper":
        p = m["closed"] + q * (m["open"] - m["closed"])
        return m["lo"] + p / 100 * (m["hi"] - m["lo"])
    d = math.degrees((q - m["offset_rad"]) / m["sign"])
    return d * (TICKS - 1) / 360 + (m["lo"] + m["hi"]) / 2


def q12_to_ticks(mapping, q12):
    q12 = np.asarray(q12, dtype=float)
    return {side: {n: model_to_tick(n, mapping[side][n], float(q12[i * 6 + k])) for k, n in enumerate(JOINTS)} for i, side in enumerate(("left", "right"))}


def ticks_to_q12(mapping, ticks):
    return np.array([tick_to_model(n, mapping[side][n], ticks[side][n]) for side in ("left", "right") for n in JOINTS])


def check_command(mapping, side, name, goal_tick, present_tick, *, margin_ticks, max_step_ticks):
    """Reasons why this goal must not be sent ([] = allowed). Limits:
       calibrated range with a margin; the SIM joint range of the model value; step from the measured position;
       (no limit on the distance from the start pose exists)."""
    m, why = mapping[side][name], []
    if not math.isfinite(goal_tick):
        return ["non-finite goal"]
    if not (m["lo"] + margin_ticks <= goal_tick <= m["hi"] - margin_ticks):
        why.append(f"goal {goal_tick:.0f} outside the calibrated range [{m['lo']}, {m['hi']}] with margin {margin_ticks}")
    q = tick_to_model(name, m, goal_tick)
    if not (m["model_range"][0] <= q <= m["model_range"][1]):
        why.append(f"model value {q:.4f} outside the SIM joint range {np.round(m['model_range'], 4).tolist()}")
    if abs(goal_tick - present_tick) > max_step_ticks:
        why.append(f"step {goal_tick - present_tick:+.0f} ticks from the measured position exceeds {max_step_ticks}")
    return why


def dry_run_chunk(mapping, chunk, present_ticks, *, steps=25, margin_ticks=20, max_step_ticks=12):
    """What executing the first `steps` commands of a policy chunk would send, checked against the limits. The step limit
    is applied between consecutive commands (the first one against the measured position). Sends nothing."""
    rows, refused = [], []
    prev = {s: dict(present_ticks[s]) for s in present_ticks}
    for k in range(steps):
        goal = q12_to_ticks(mapping, chunk[k])
        row = dict(step=k, goal={s: {n: round(goal[s][n], 1) for n in JOINTS} for s in goal})
        for s in goal:
            for n in JOINTS:
                why = check_command(mapping, s, n, goal[s][n], prev[s][n], margin_ticks=margin_ticks, max_step_ticks=max_step_ticks)
                if why:
                    refused.append(dict(step=k, side=s, joint=n, goal=round(goal[s][n], 1), previous=round(prev[s][n], 1), reasons=why))
        prev = goal
        rows.append(row)
    return dict(steps=steps, limits=dict(margin_ticks=margin_ticks, max_step_ticks=max_step_ticks, step_note="12 ticks per 20 ms = 52.7 deg/s; a bound for the dry-run report, not a tuned controller limit"),
                first=rows[0]["goal"], last=rows[-1]["goal"], refused=refused, would_send=0 if refused else steps, commands_sent=0)


def classify_chunk(mapping, chunk, present_ticks, *, steps=25, margin_ticks=20, max_step_ticks=12):
    """Per joint: is a refusal caused by a HARD limit (the device's calibrated range, the SIM joint range) or only by a
    TEMPORARY dry-run setting (margin inside the range, step size)? Sends nothing."""
    cat, prev = {}, {s: dict(present_ticks[s]) for s in present_ticks}
    for k in range(steps):
        g = q12_to_ticks(mapping, chunk[k])
        for s in g:
            for n in JOINTS:
                m, t = mapping[s][n], g[s][n]
                c = cat.setdefault(f"{s} {n}", dict(outside_calibrated_range=0, outside_SIM_range=0, inside_range_within_margin=0, step_over_limit=0, max_step_ticks=0.0, goal_min=t, goal_max=t, range=[m["lo"], m["hi"]], min_distance_to_range_end=float("inf")))
                if not m["lo"] <= t <= m["hi"]:
                    c["outside_calibrated_range"] += 1
                elif not m["lo"] + margin_ticks <= t <= m["hi"] - margin_ticks:
                    c["inside_range_within_margin"] += 1
                if not m["model_range"][0] <= tick_to_model(n, m, t) <= m["model_range"][1]:
                    c["outside_SIM_range"] += 1
                st = abs(t - prev[s][n])
                c["step_over_limit"] += st > max_step_ticks
                c.update(max_step_ticks=max(c["max_step_ticks"], round(st, 1)), goal_min=min(c["goal_min"], t), goal_max=max(c["goal_max"], t), min_distance_to_range_end=min(c["min_distance_to_range_end"], t - m["lo"], m["hi"] - t))
        prev = g
    for c in cat.values():
        c.update(goal_min=round(c["goal_min"], 1), goal_max=round(c["goal_max"], 1), min_distance_to_range_end=round(c["min_distance_to_range_end"], 1), step_over_limit=int(c["step_over_limit"]))
        c["class"] = ("HARD: outside the calibrated range" if c["outside_calibrated_range"] else "HARD: outside the SIM joint range" if c["outside_SIM_range"] else
                      "DIAGNOSTIC only (near the range end / raw step size): not a gate" if (c["inside_range_within_margin"] or c["step_over_limit"]) else "allowed")
        c["diagnostic_note"] = "margin_ticks and max_step_ticks are development heuristics of this report. Real commands are gated by sweepick_trajectory.validate (range, model range, collision, v / a / j) and by the executor"
    return cat


def plan_arms_only(mapping, chunk, present_ticks, *, steps=25, end_margin_ticks=5, max_step_ticks=12):
    """REPORT of the arm part of the first `steps` chunk commands, grippers held at their measured position. `in_range` is
    decided by the configured range and the SIM joint range only. end_margin_ticks / max_step_ticks are development
    heuristics and produce `diagnostics`, never a refusal. This is not an executable command sequence: a real command is
    retimed and gated by sweepick_trajectory.validate (collision, velocity / acceleration / jerk) and by the executor."""
    seq, hard, diag, prev = [], [], [], {s: dict(present_ticks[s]) for s in present_ticks}
    for k in range(steps):
        g = q12_to_ticks(mapping, chunk[k])
        row = {}
        for s in g:
            row[s] = {}
            for n in JOINTS:
                t = float(present_ticks[s][n]) if n == "gripper" else g[s][n]
                if n != "gripper":
                    m = mapping[s][n]
                    if not (math.isfinite(t) and m["lo"] <= t <= m["hi"]):
                        hard.append(dict(step=k, side=s, joint=n, goal=round(t, 1), gate="servo configured range", range=[m["lo"], m["hi"]]))
                    elif not (m["model_range"][0] <= tick_to_model(n, m, t) <= m["model_range"][1]):
                        hard.append(dict(step=k, side=s, joint=n, goal=round(t, 1), gate="dual model joint range (candidate mapping)"))
                    for why in check_command(mapping, s, n, t, prev[s][n], margin_ticks=end_margin_ticks, max_step_ticks=max_step_ticks):
                        if "outside the calibrated range" in why and m["lo"] <= t <= m["hi"]:
                            diag.append(dict(step=k, side=s, joint=n, kind="near the range end (diagnostic)", distance_ticks=round(min(t - m["lo"], m["hi"] - t), 1)))
                        elif why.startswith("step"):
                            diag.append(dict(step=k, side=s, joint=n, kind="raw step larger than the heuristic (diagnostic; the command layer retimes)", detail=why))
                row[s][n] = int(round(t)) if math.isfinite(t) else None
            prev[s] = dict(row[s])
        seq.append(row)
    ex = {s: {n: max(abs((r[s][n] or 0) - present_ticks[s][n]) for r in seq) for n in JOINTS} for s in present_ticks}
    return dict(in_range=not hard, hard_failures=hard, diagnostics=diag, steps=steps, duration_s=steps * 0.02, largest_move_ticks=ex, largest_move_deg={s: {n: round(v * 360 / TICKS, 2) for n, v in ex[s].items() if n != "gripper"} for s in ex},
                sequence=seq, heuristics=dict(end_margin_ticks=end_margin_ticks, max_step_ticks=max_step_ticks, role="diagnostic only"), grippers="held at the measured position", executable=None,
                executable_note="not decided here: see sweepick_trajectory.validate", commands_sent=0)
