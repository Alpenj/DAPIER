"""Trajectory generation and checking for commands to the real arms.

No limit on the size of a move exists here. What can REFUSE a trajectory is listed in sweepick_HARD_GATE_ALLOWLIST.json and nothing
else: a malformed sample and the servo's own configured range are HARD. Model-dependent checks (joint range and collision
of the assembled model through the CANDIDATE mapping) are PROVISIONAL: they apply to model-dependent paths (level='model')
and are never called physical limits. Everything numeric that was tuned by hand is a DIAGNOSTIC.
Velocity / acceleration / jerk are the SELECTED execution profile (sweepick_motion_profile.json): not ratings of the servo, but the
contract of the run. make() retimes every move to it; a trajectory that does not keep it is refused (profile_violation)."""
from sweepick.integration.sweepick_resource_paths import source_path
import hashlib, json, math
from pathlib import Path
import numpy as np
from sweepick.control import sweepick_joint_command_mapping as rc
from sweepick.control import sweepick_execution_gates as G

TICK_RAD = 2 * math.pi / rc.TICKS
PROFILE_FILE = source_path("sweepick_motion_profile.json")


def load_profile(path=PROFILE_FILE):
    d = json.loads(Path(path).read_text())
    return dict(v=float(d["profile_velocity_rad_s"]["value"]), a=float(d["profile_acceleration_rad_s2"]["value"]), j=float(d["profile_jerk_rad_s3"]["value"]), dt=float(d["period_s"]), sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())


load_limits = load_profile          # old name kept for callers; the values are profile parameters, not limits


def duration_for(length_rad, lim):
    """Shortest rest-to-rest minimum-jerk duration that respects v, a, j (peaks: 1.875 L/T, 5.7735 L/T^2, 60 L/T^3)."""
    if length_rad <= 0:
        return 0.0
    return max(1.875 * length_rad / lim["v"], math.sqrt(5.7735 * length_rad / lim["a"]), (60.0 * length_rad / lim["j"]) ** (1 / 3))


def make(start, target, lim):
    """start: {joint: tick} of all six joints as measured NOW. target: {joint: tick} for the joints that move (any subset,
    any distance). All moving joints share one duration (the slowest joint's). Returns the commanded trajectory."""
    for n in target:
        if n not in rc.JOINTS:
            raise ValueError(f"unknown joint {n}")
    delta = {n: float(target[n]) - float(start[n]) for n in target}
    T = max([duration_for(abs(d) * TICK_RAD, lim) for d in delta.values()] or [0.0])
    steps = int(math.ceil(T / lim["dt"])) if T > 0 else 0
    T = steps * lim["dt"]
    s = np.arange(1, steps + 1) / steps if steps else np.zeros(0)
    u = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    goals = {n: (float(start[n]) + delta[n] * u if n in delta else np.full(steps, float(start[n]))).tolist() for n in rc.JOINTS}
    return dict(schema="tjj.trajectory.v1", start={n: float(start[n]) for n in rc.JOINTS}, target={n: float(v) for n, v in target.items()}, moving=sorted(delta, key=rc.JOINTS.index), held=[n for n in rc.JOINTS if n not in delta],
                profile="rest-to-rest minimum jerk, one duration for all moving joints", dt=lim["dt"], steps=steps, duration_s=T, travel_ticks={n: abs(d) for n, d in delta.items()}, travel_deg={n: abs(d) * 360 / rc.TICKS for n, d in delta.items()}, goals=goals)


def peaks(traj):
    """Largest |velocity|, |acceleration|, |jerk| of the commanded profile per joint [rad of motor angle], by finite
    differences of the un-rounded samples (start included)."""
    out, dt = {}, traj["dt"]
    for n in traj["moving"]:
        q = np.r_[[traj["start"][n]] * 4, traj["goals"][n], [traj["goals"][n][-1]] * 4] * TICK_RAD if traj["steps"] else np.zeros(9)
        v = np.diff(q) / dt; a = np.diff(v) / dt; j = np.diff(a) / dt
        out[n] = dict(v=float(np.abs(v).max()), a=float(np.abs(a).max()), j=float(np.abs(j).max()))
    return out


def identity(side, traj, mapping_sha):
    """What a collision result must be bound to (structure only)."""
    key = json.dumps(dict(side=side, start={n: round(traj["start"][n], 3) for n in rc.JOINTS}, target={n: round(v, 3) for n, v in sorted(traj["target"].items())}, steps=traj["steps"], dt=traj["dt"], mapping=mapping_sha), sort_keys=True)
    return hashlib.sha256(key.encode()).hexdigest()


def validate(side, mp_side, traj, prof, *, mapping_sha, level="device", cert=None, other_arm_ticks=None):
    """level='device': device-level commissioning (ticks on one arm; needs no model).  level='model': a model-dependent move.
    Returns hard (list), provisional (list), diagnostics (list), executable (bool). Nothing is clipped."""
    hard, prov, diag = [], [], []
    for n in rc.JOINTS:
        m = mp_side[n]
        g = np.asarray([traj["start"][n]] + list(traj["goals"][n]), dtype=float)
        if not np.isfinite(g).all():
            hard.append(G.hard("command_malformed", f"{n}: non-finite sample", joint=n)); continue
        if n in traj["moving"] and (g[1:].min() < m["lo"] or g[1:].max() > m["hi"]):
            hard.append(G.hard("servo_range", f"{n}: commanded {g[1:].min():.1f}..{g[1:].max():.1f} outside the servo range [{m['lo']}, {m['hi']}]", joint=n))
        q = [rc.tick_to_model(n, m, t) for t in (g.min(), g.max())]
        if n in traj["moving"] and (min(q) < m["model_range"][0] or max(q) > m["model_range"][1]):
            if level == "model":
                prov.append(G.provisional("model_joint_range_candidate", f"{n}: outside the dual model range through the candidate mapping", joint=n))
            else:
                diag.append(G.diagnostic("model_range_candidate", f"{n}: outside the dual model range through the candidate mapping (not a gate at device level)", joint=n))
        if n in traj["moving"]:
            diag.append(G.diagnostic("distance_to_range_end", joint=n, ticks=float(min(g.min() - m["lo"], m["hi"] - g.max()))))
    pk = peaks(traj)
    diag.append(G.diagnostic("profile_peaks", "peaks of the commanded profile next to the selected profile", peaks=pk, profile={k: prof[k] for k in ("v", "a", "j")}))
    over = {n: {k: p[k] for k in ("v", "a", "j") if p[k] > prof[k] * (1 + 1e-6)} for n, p in pk.items()}
    over = {n: v for n, v in over.items() if v}
    if over or abs(traj["dt"] - prof["dt"]) > 1e-12:                          # the selected profile is the contract of this run, not a rating of the servo
        hard.append(G.contract("profile_violation", f"the trajectory does not keep the selected profile: {over or 'period ' + str(traj['dt']) + ' s instead of ' + str(prof['dt'])}; build it with sweepick_trajectory.make (it retimes to the profile)"))
    ident = identity(side, traj, mapping_sha)
    if level == "model" and traj["moving"]:
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
