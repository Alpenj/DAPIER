"""Owner-aware application of a canonical command (q12) to the motors: the ONE boundary between the command owners
(ACT student, sensor correctors, grasp primitive - chosen by the existing Arbiter) and Goal_Position.

It does not decide WHAT to do (the owner does) nor WHEN to close / hold (sweepick_grasp.Jaw does). It decides HOW a command is
applied and WHETHER this cycle may be written at all:

  dispatch(owner, q12, present, fresh=..., servo_status=...) -> Dispatch
      SEND      goal ticks for this cycle; every value that would be written was itself checked against the servo range
      BRAKE     the owner's command was not accepted (malformed / outside the servo range / a PROVISIONAL model-dependent
                gate / owner not connected / stop asked): the goal is slowed to rest; these goal ticks may be written
      STOP      nothing is to be written this cycle: the position is not a same-cycle readback, or a servo reports a fault
A writer uses `Dispatch.write` only; it is None unless the status allows a write.
What can stop or reject is sweepick_HARD_GATE_ALLOWLIST.json. Tracking error is reported in every Dispatch and is NOT a gate: an
arm that is behind its goal is not stopped for that. The velocity / acceleration used to follow a target are the motion
PROFILE parameters, not limits of the servo.
One cycle is all-or-nothing: the internal state changes only after every channel of both arms was computed and checked.
A target of any distance is accepted and followed at the velocity / acceleration limits; while following, the goal always
keeps enough room to stop before the end of the servo range (so a valid target can never produce an out-of-range goal).
Jerk is not limited here (stated in the status file). Pure computation; opens no device."""
import math
from dataclasses import dataclass, field
from sweepick.control import sweepick_joint_command_mapping as rc
from sweepick.control import sweepick_trajectory_profile as tt
from sweepick.control import sweepick_execution_gates as G
from sweepick.control import sweepick_motion_progress as PG

SIDES = ("left", "right")


class NotConnected(RuntimeError):
    """An owner whose commands cannot be applied to the real arm yet, with the missing piece named."""


@dataclass
class Dispatch:
    status: str                      # SEND | HOLD | BRAKE | STOP
    write: dict | None               # {side: {joint: int tick}} or None
    owner: str
    reasons: list = field(default_factory=list)
    raw_target: dict | None = None   # ticks of the owner's command (float), for the record
    applied: dict | None = None      # un-rounded goal after this cycle
    tracking_error: dict | None = None       # DIAGNOSTIC: measured - goal, per joint. Never a gate
    gates: list = field(default_factory=list)  # the allowlisted gate records behind the status


class Applier:
    def __init__(self, mapping, limits, present_ticks, *, real_jaw_profile=None):
        self.mp, self.lim, self.real_jaw = mapping, limits, real_jaw_profile
        self.goal = {s: {n: float(present_ticks[s][n]) for n in rc.JOINTS} for s in SIDES}       # starts at the measured position
        self.vel = {s: {n: 0.0 for n in rc.JOINTS} for s in SIDES}                              # ticks / s of the goal
        self.cycles, self.last = 0, None
        self.pg, self.mon, self.seen = None, {}, 0                                              # closed-loop progress monitoring (real plant only)

    # ---- one channel, no side effects -----------------------------------------------------------------
    def _advance(self, m, g, v, target):
        """(goal, velocity) one period later. target=None means 'come to rest'. Never past the target within the step, never
        faster than v, never a larger change of speed than a * dt, and always with room to stop inside [lo, hi]."""
        dt, vmax, amax = self.lim["dt"], self.lim["v"] / tt.TICK_RAD, self.lim["a"] / tt.TICK_RAD
        dv = amax * dt
        brake = v - math.copysign(min(abs(v), dv), v)

        def room(gn, vn):                                                        # where the goal comes to rest if it brakes from here, in whole periods
            k = math.floor(abs(vn) / dv + 1e-12)                                  # periods of full braking still to come after this one
            stop = gn + math.copysign(dt * (k * abs(vn) - dv * k * (k + 1) / 2), vn)
            return m["lo"] - 1e-9 <= min(gn, stop) and max(gn, stop) <= m["hi"] + 1e-9

        if target is None:
            vn = brake
        else:
            e = target - g
            want = math.copysign(min(vmax, math.sqrt(2 * amax * abs(e)), abs(e) / dt), e) if e else 0.0
            vn = v + max(-dv, min(dv, want - v))
            if not room(g + vn * dt, vn):
                vn = brake                                                       # slow down instead of approaching the range end too fast
        gn = g + vn * dt
        if target is not None and abs(target - gn) < 1e-9:
            gn = target
        return gn, vn

    def _check_sent(self, s, n, gn):
        """The value that would be written, checked itself against the servo's own range (HARD)."""
        m, why = self.mp[s][n], []
        t = int(round(gn)) if math.isfinite(gn) else None
        if t is None or not (m["lo"] <= gn <= m["hi"] and m["lo"] <= t <= m["hi"]):
            why.append(f"{s} {n}: goal {gn} outside the servo configured range [{m['lo']}, {m['hi']}]")
        return t, why

    def _commit(self, new_goal, new_vel, status, owner, reasons, raw, err, gates=()):
        write = {s: {} for s in SIDES}
        bad = []
        for s in SIDES:
            for n in rc.JOINTS:
                t, why = self._check_sent(s, n, new_goal[s][n])
                write[s][n] = t; bad += why
        if bad:                                                                  # cannot happen while the 'room to stop' rule holds; if it does, nothing is written
            return Dispatch("STOP", None, owner, ["a computed goal failed the servo range check; nothing written"] + bad + reasons, raw, None, err, [G.hard("servo_range", "; ".join(bad))])
        self.goal, self.vel, self.cycles = new_goal, new_vel, self.cycles + 1       # the only place the state changes
        d = Dispatch(status, write, owner, reasons, raw, {s: dict(new_goal[s]) for s in SIDES}, err, list(gates))
        self.last = d
        return d

    def _brake_all(self, owner, reasons, raw=None, err=None, gates=()):
        g, v = {s: {} for s in SIDES}, {s: {} for s in SIDES}
        for s in SIDES:
            for n in rc.JOINTS:
                g[s][n], v[s][n] = self._advance(self.mp[s][n], self.goal[s][n], self.vel[s][n], None)
        return self._commit(g, v, "BRAKE", owner, reasons, raw, err, gates)

    # ---- the boundary -----------------------------------------------------------------------------------
    def relax(self, present_ticks):
        """The write that follows a STOP: goal = the position just measured, on every channel, velocities zero. It stops
        pushing; it does not release torque and it does not move back. Returns the ticks to write, or None when the
        measurement itself is unusable (then nothing can be written and a person has to act)."""
        ok = present_ticks is not None and all(isinstance((present_ticks.get(s) or {}).get(n), (int, float)) and math.isfinite(present_ticks[s][n]) for s in SIDES for n in rc.JOINTS)
        if not ok:
            return None
        self.goal = {s: {n: float(present_ticks[s][n]) for n in rc.JOINTS} for s in SIDES}
        self.vel = {s: {n: 0.0 for n in rc.JOINTS} for s in SIDES}; self.mon = {}
        return {s: {n: int(round(self.goal[s][n])) for n in rc.JOINTS} for s in SIDES}

    def dispatch(self, owner, q12, present_ticks, *, fresh=True, servo_status=None, status_required=None, t_read=None, cycle_start=None, closed_loop=True, stop=False):
        """present_ticks: the readback of THIS cycle. It is checked here, not trusted: every one of the 12 channels must be
        present and finite, and (when the times are given) read inside this cycle.
        servo_status: {side: {joint: Status}}. With status_required a missing / unread status is NOT a status of 0.
        closed_loop (default): the measurement is the real arm following these goals -> the Status of every channel is
        required and progress monitoring applies. Only a caller whose "measurement" is a synthetic plant (read-only loop,
        fixture, offline mirror: nothing is written, nothing follows) passes closed_loop=False, and says so in its log."""
        status_required = closed_loop if status_required is None else status_required
        def stop_with(gate):
            return Dispatch("STOP", None, owner, [gate["detail"]], None, None, None, [gate])        # no write, no state change
        if present_ticks is None or not fresh:
            return stop_with(G.hard("stale_position", "no same-cycle Present_Position: nothing is written"))
        bad = [f"{s} {n}" for s in SIDES for n in rc.JOINTS if not isinstance((present_ticks.get(s) or {}).get(n), (int, float)) or not math.isfinite((present_ticks.get(s) or {}).get(n))]
        if bad:
            return stop_with(G.contract("measurement_invalid", f"actual position missing or non-finite on {bad}: nothing is written"))
        if t_read is not None and cycle_start is not None and t_read < cycle_start:
            return stop_with(G.contract("measurement_invalid", f"the position was read {cycle_start - t_read:.3f} s before this cycle began: not a same-cycle readback"))
        if status_required:
            miss = [f"{s} {n}" for s in SIDES for n in rc.JOINTS if not isinstance(((servo_status or {}).get(s) or {}).get(n), (int, float))]
            if miss:
                return stop_with(G.contract("status_unavailable", f"Status not read on {miss}: a missing status is not a status of 0"))
        faults = {f"{s} {n}": v for s in (servo_status or {}) for n, v in (servo_status[s] or {}).items() if v}
        if faults:
            return stop_with(G.hard("servo_fault", f"Status register non-zero: {faults}"))
        if closed_loop:
            if self.pg is None:
                self.pg = PG.load()
            self.seen += 1                                                       # one measurement per period when the caller gives no read time (a held cycle counts too)
            now = t_read if t_read is not None else self.seen * self.lim["dt"]
            state = {}
            for s in SIDES:
                for n in rc.JOINTS:
                    lead = self.goal[s][n] - float(present_ticks[s][n])
                    d = 0 if abs(lead) < self.pg["resolution"] else (1 if lead > 0 else -1)
                    m = self.mon.get((s, n))
                    if m is None or m.dir != d:
                        m = self.mon[(s, n)] = PG.Joint(self.pg, present_ticks[s][n], d, now)
                    state[f"{s} {n}"] = m.update(now, float(present_ticks[s][n]), abs(lead))
            stuck = sorted(k for k, v in state.items() if v == PG.NO_PROGRESS)
            if stuck:
                return stop_with(G.contract("no_progress", f"{stuck} did not advance for {2 * self.pg['start_delay']:.2f} s with a goal pending (the goal had stopped advancing after {self.pg['start_delay']:.2f} s): stop pushing", joints=stuck))
            waiting = sorted(k for k, v in state.items() if v == PG.WAIT)
            if waiting:                                                          # do not accumulate a goal the arm is not following
                self.vel = {s: {n: 0.0 for n in rc.JOINTS} for s in SIDES}
                err = {s: {n: float(present_ticks[s][n]) - self.goal[s][n] for n in rc.JOINTS} for s in SIDES}
                d = Dispatch("HOLD", {s: {n: int(round(self.goal[s][n])) for n in rc.JOINTS} for s in SIDES}, owner, [f"waiting for {waiting} to advance: the goal is not moved further"], None, {s: dict(self.goal[s]) for s in SIDES}, err, [G.diagnostic("progress_state", waiting=waiting)])
                self.last = d
                return d
        err = {s: {n: float(present_ticks[s][n]) - self.goal[s][n] for n in rc.JOINTS} for s in SIDES}       # diagnostic
        if stop:
            g = G.hard("operator_stop", "stop requested")
            return self._brake_all(owner, [g["detail"]], None, err, [g])
        if not owner.startswith("ACT:") and self.real_jaw is None:
            g = G.provisional("owner_not_connected", f"{owner} writes its gripper channel through sweepick_grasp.Jaw.next_command, which needs a REAL JawDevice profile; none exists")
            return self._brake_all(owner, ["owner not connected: " + g["detail"]], None, err, [g])
        try:
            raw = rc.q12_to_ticks(self.mp, q12)
            if not all(math.isfinite(raw[s][n]) for s in SIDES for n in rc.JOINTS):
                raise ValueError("non-finite command")
        except Exception as e:
            g = G.hard("command_malformed", f"{type(e).__name__}: {e}")
            return self._brake_all(owner, ["owner command rejected: " + g["detail"]], None, err, [g])
        gates = []
        for s in SIDES:
            for n in rc.JOINTS:
                m, t = self.mp[s][n], raw[s][n]
                if not (m["lo"] <= t <= m["hi"]):
                    gates.append(G.hard("servo_range", f"{s} {n}: owner target {t:.1f} outside the servo configured range [{m['lo']}, {m['hi']}]"))
                elif not (m["model_range"][0] <= rc.tick_to_model(n, m, t) <= m["model_range"][1]):
                    gates.append(G.provisional("model_joint_range_candidate", f"{s} {n}: owner target outside the dual model joint range (candidate mapping; model-dependent gate of the ACT / HYBRID path)"))
        if gates:                                                                # the owner's command is not clipped and not followed
            return self._brake_all(owner, ["owner command rejected"] + [x["detail"] for x in gates], raw, err, gates)
        g, v = {s: {} for s in SIDES}, {s: {} for s in SIDES}
        for s in SIDES:
            for n in rc.JOINTS:
                g[s][n], v[s][n] = self._advance(self.mp[s][n], self.goal[s][n], self.vel[s][n], raw[s][n])
        return self._commit(g, v, "SEND", owner, [], raw, err)
