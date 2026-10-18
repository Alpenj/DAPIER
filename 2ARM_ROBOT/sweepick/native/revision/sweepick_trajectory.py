"""Trajectory generation and checking for commands to the real arms.

No limit on the size of a move exists here. What can REFUSE a trajectory is listed in sweepick_HARD_GATE_ALLOWLIST.json and nothing
else: a malformed sample and the servo's own configured range are HARD. Model-dependent checks (joint range and collision
of the assembled model through the CANDIDATE mapping) are PROVISIONAL: they apply to model-dependent paths (level='model')
and are never called physical limits. Everything numeric that was tuned by hand is a DIAGNOSTIC.
Velocity / acceleration / jerk are the SELECTED execution profile (sweepick_motion_profile.json): not ratings of the servo, but the
contract of the run. make() retimes every move to it; a trajectory that does not keep it is refused (profile_violation)."""
import hashlib, json, math
from pathlib import Path
import numpy as np
import sweepick_real_command as rc, sweepick_gates as G

TICK_RAD = 2 * math.pi / rc.TICKS
PROFILE_FILE = Path(__file__).resolve().parent / "sweepick_motion_profile.json"


def load_profile(path=PROFILE_FILE):
    d = json.loads(Path(path).read_text())
    return dict(v=float(d["profile_velocity_rad_s"]["value"]), a=float(d["profile_acceleration_rad_s2"]["value"]), j=float(d["profile_jerk_rad_s3"]["value"]), dt=float(d["period_s"]), sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())


load_limits = load_profile          # old name kept for callers; the values are profile parameters, not limits


def duration_for(length_rad, lim):
    """Shortest rest-to-rest minimum-jerk duration that respects v, a, j (peaks: 1.875 L/T, 5.7735 L/T^2, 60 L/T^3)."""
    if length_rad <= 0:
        return 0.0
    return max(1.875 * length_rad / lim["v"], math.sqrt(5.7735 * length_rad / lim["a"]), (60.0 * length_rad / lim["j"]) ** (1 / 3))


def make(start, target, lim, *, command_start=None):
    """start: {joint: tick} of all six joints as measured NOW. target: {joint: tick} for the joints that move (any subset,
    any distance). command_start, when explicitly supplied, is the range-valid six-joint command seed after a torque-OFF
    actual start; it is never inferred or clipped here. All moving joints share one duration (the slowest joint's)."""
    for n in target:
        if n not in rc.JOINTS:
            raise ValueError(f"unknown joint {n}")
    seed = start if command_start is None else command_start
    delta = {n: float(target[n]) - float(seed[n]) for n in target}
    T = max([duration_for(abs(d) * TICK_RAD, lim) for d in delta.values()] or [0.0])
    steps = int(math.ceil(T / lim["dt"])) if T > 0 else 0
    T = steps * lim["dt"]
    s = np.arange(1, steps + 1) / steps if steps else np.zeros(0)
    u = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    goals = {n: (float(seed[n]) + delta[n] * u if n in delta else np.full(steps, float(seed[n]))).tolist() for n in rc.JOINTS}
    result = dict(schema="tjj.trajectory.v1", start={n: float(start[n]) for n in rc.JOINTS}, target={n: float(v) for n, v in target.items()}, moving=sorted(delta, key=rc.JOINTS.index), held=[n for n in rc.JOINTS if n not in delta],
                profile="rest-to-rest minimum jerk, one duration for all moving joints", dt=lim["dt"], steps=steps, duration_s=T, travel_ticks={n: abs(d) for n, d in delta.items()}, travel_deg={n: abs(d) * 360 / rc.TICKS for n, d in delta.items()}, goals=goals)
    if command_start is not None:
        entry = {n: abs(float(seed[n]) - float(start[n])) for n in rc.JOINTS if float(seed[n]) != float(start[n])}
        result.update(command_start={n: float(seed[n]) for n in rc.JOINTS},
                      command_travel_ticks=dict(result["travel_ticks"]), command_travel_deg=dict(result["travel_deg"]),
                      range_entry_travel_ticks=entry, range_entry_travel_deg={n: d * 360 / rc.TICKS for n, d in entry.items()},
                      travel_ticks={n: entry.get(n, 0.0) + abs(d) for n, d in delta.items()},
                      travel_deg={n: (entry.get(n, 0.0) + abs(d)) * 360 / rc.TICKS for n, d in delta.items()})
    return result


def peaks(traj):
    """Largest |velocity|, |acceleration|, |jerk| of the commanded profile per joint [rad of motor angle], by finite
    differences of the un-rounded samples (start included)."""
    out, dt = {}, traj["dt"]
    for n in traj["moving"]:
        q = np.r_[[traj.get("command_start", traj["start"])[n]] * 4, traj["goals"][n], [traj["goals"][n][-1]] * 4] * TICK_RAD if traj["steps"] else np.zeros(9)
        v = np.diff(q) / dt; a = np.diff(v) / dt; j = np.diff(a) / dt
        out[n] = dict(v=float(np.abs(v).max()), a=float(np.abs(a).max()), j=float(np.abs(j).max()))
    return out


def identity(side, traj, mapping_sha):
    """What a collision result must be bound to (structure only)."""
    fields = dict(side=side, start={n: round(traj["start"][n], 3) for n in rc.JOINTS}, target={n: round(v, 3) for n, v in sorted(traj["target"].items())}, steps=traj["steps"], dt=traj["dt"], mapping=mapping_sha)
    if "command_start" in traj:
        fields["command_start"] = {n: round(traj["command_start"][n], 3) for n in rc.JOINTS}
    key = json.dumps(fields, sort_keys=True)
    return hashlib.sha256(key.encode()).hexdigest()


def validate(side, mp_side, traj, prof, *, mapping_sha, level="device", cert=None, other_arm_ticks=None):
    """level='device': device-level commissioning (ticks on one arm; needs no model).  level='model': a model-dependent move.
    Returns hard (list), provisional (list), diagnostics (list), executable (bool). Nothing is clipped."""
    hard, prov, diag = [], [], []
    command_start = traj.get("command_start", traj["start"])
    for n in rc.JOINTS:
        m = mp_side[n]
        g = np.asarray([traj["start"][n]] + ([command_start[n]] if "command_start" in traj else []) + list(traj["goals"][n]), dtype=float)
        if not np.isfinite(g).all():
            hard.append(G.hard("command_malformed", f"{n}: non-finite sample", joint=n)); continue
        commanded = g[1:]
        commanded_joint = n in traj["moving"] or ("command_start" in traj and command_start[n] != traj["start"][n])
        if commanded_joint and (commanded.min() < m["lo"] or commanded.max() > m["hi"]):
            hard.append(G.hard("servo_range", f"{n}: commanded {commanded.min():.1f}..{commanded.max():.1f} outside the servo range [{m['lo']}, {m['hi']}]", joint=n))
        model_values = commanded if "command_start" in traj else g
        q = [rc.tick_to_model(n, m, t) for t in (model_values.min(), model_values.max())]
        # 2026-10-18 205318_auto: a hand-placed right wrist_flex at 887 ticks (inside the servo range 862..3197) was outside the model range, so the
        # move BACK to the start pose was refused and the session never began. A path whose out-of-range samples are only its first ones, each
        # nearer the range than the one before, and whose target is inside, ENTERS the model range: a diagnostic, not a gate.
        qs = np.array([rc.tick_to_model(n, m, t) for t in model_values]); lo_, hi_ = m["model_range"]
        outside = (qs < lo_) | (qs > hi_); dist = np.where(qs < lo_, lo_ - qs, np.where(qs > hi_, qs - hi_, 0.0))
        entering = bool(outside.any() and not outside[-1] and outside[: int(np.argmin(outside)) if not outside.all() else len(outside)].all()
                        and not outside[int(np.argmin(outside)):].any() and np.all(np.diff(dist[: int(np.argmin(outside)) + 1]) <= 1e-9))
        if commanded_joint and entering:
            diag.append(G.diagnostic("model_range_entry", f"{n}: starts outside the dual model range and only moves into it", joint=n, start_model=float(qs[0]), model_range=[float(lo_), float(hi_)]))
        elif commanded_joint and (min(q) < m["model_range"][0] or max(q) > m["model_range"][1]):
            if level == "model":
                prov.append(G.provisional("model_joint_range_candidate", f"{n}: outside the dual model range through the candidate mapping", joint=n))
            else:
                diag.append(G.diagnostic("model_range_candidate", f"{n}: outside the dual model range through the candidate mapping (not a gate at device level)", joint=n))
        if n in traj["moving"]:
            range_values = commanded if "command_start" in traj else g
            diag.append(G.diagnostic("distance_to_range_end", joint=n, ticks=float(min(range_values.min() - m["lo"], m["hi"] - range_values.max()))))
    pk = peaks(traj)
    diag.append(G.diagnostic("profile_peaks", "peaks of the commanded profile next to the selected profile", peaks=pk, profile={k: prof[k] for k in ("v", "a", "j")}))
    over = {n: {k: p[k] for k in ("v", "a", "j") if p[k] > prof[k] * (1 + 1e-6)} for n, p in pk.items()}
    over = {n: v for n, v in over.items() if v}
    if over or abs(traj["dt"] - prof["dt"]) > 1e-12:                          # the selected profile is the contract of this run, not a rating of the servo
        hard.append(G.contract("profile_violation", f"the trajectory does not keep the selected profile: {over or 'period ' + str(traj['dt']) + ' s instead of ' + str(prof['dt'])}; build it with sweepick_trajectory.make (it retimes to the profile)"))
    ident = identity(side, traj, mapping_sha)
    if level == "model" and traj["moving"]:
        if "command_start" in traj:
            if cert is None:
                prov.append(G.provisional("collision_candidate_frame", "no actual-start to command-seed collision result for this range-entry trajectory", identity=ident))
            elif cert.get("range_entry_checked") is not True or cert.get("range_entry_identity") != ident:
                prov.append(G.provisional("certificate_binding", "the collision result does not prove the actual-start to command-seed range-entry path for this trajectory", identity=ident))
        if cert is None:
            prov.append(G.provisional("collision_candidate_frame", "no collision result for this trajectory", identity=ident))
        elif cert.get("identity") != ident:
            prov.append(G.provisional("certificate_binding", "the collision result belongs to another trajectory (start, target, timing or mapping differ)", identity=ident))
        elif other_arm_ticks is not None and cert.get("other_arm_ticks") is not None and any(other_arm_ticks[n] != cert["other_arm_ticks"][n] for n in rc.JOINTS):
            prov.append(G.provisional("certificate_binding", "the other arm is not at the position the collision result was computed for"))
        elif cert.get("nominal_verdict") != "CLEAR":
            prov.append(G.provisional("collision_candidate_frame", f"contact on the commanded path in the SIM scene (candidate mapping): {cert.get('first_nominal_contact')}"))
        if cert is not None and cert.get("tube_diagnostic"):
            diag.append(G.diagnostic("collision_tube", "contacts found only when joints are displaced from the commanded path", **{k: cert["tube_diagnostic"].get(k) for k in ("displacement_ticks", "contacts", "first")}))
    return dict(executable=not hard and not prov, hard=hard, provisional=prov, diagnostics=diag, level=level, identity=ident, total_travel_limit=None,
                mapping_note="the joint mapping is a CANDIDATE: model range and collision are model-dependent results, not physical limits of the arm")


def retime_report(mp_side, raw_goal_ticks, present_ticks, lim):   # lim = the motion profile
    """A raw (policy) target far from the measured position is NOT refused for its size: this reports how long a
    limit-respecting move to it takes. Diagnostic for the command layer; sends nothing."""
    rows = {}
    for n, g in raw_goal_ticks.items():
        d = abs(float(g) - float(present_ticks[n]))
        rows[n] = dict(raw_step_ticks=d, duration_s_within_limits=duration_for(d * TICK_RAD, lim), control_periods=int(math.ceil(duration_for(d * TICK_RAD, lim) / lim["dt"])) if d else 0)
    return rows
