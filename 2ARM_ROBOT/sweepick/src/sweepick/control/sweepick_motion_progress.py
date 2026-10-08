"""Progress monitoring of a commanded joint from its actual readback: normal start delay / lag versus not moving.

The numbers come from sweepick_progress_profile.json (this robot's own traces, with their scope). A joint that is behind but keeps
advancing is never stopped. A joint that does not advance for longer than the longest normal start delay makes the command
WAIT (nothing further is accumulated); if it still does not advance for as long again, it is reported as NO_PROGRESS."""
from sweepick.integration.sweepick_resource_paths import source_path
import hashlib, json
from pathlib import Path

PROFILE_FILE = source_path("sweepick_progress_profile.json")
OK, WAIT, NO_PROGRESS = "OK", "WAIT", "NO_PROGRESS"


def load(path=PROFILE_FILE):
    d = json.loads(Path(path).read_text())
    return dict(resolution=float(d["resolution_ticks"]["value"]), start_delay=float(d["start_delay_s"]["value"]), lead=float(d["stationary_lead_ticks"]["value"]), follow=float(d["following_lead_ticks"]["value"]), reach=float(d["reach_tolerance_ticks"]["value"]), sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())


class Joint:
    """One commanded ARM joint (or a gripper on a free, planned move). update() once per fresh readback.

    Not for a gripper closing on an object: that is sweepick_grasp.Jaw (contact is judged from the residual of the free stroke,
    and the closing command stops there). A joint that advances a little while the command runs away from it is NOT
    following: nothing here grows with such progress."""
    def __init__(self, prof, start_tick, direction, t0):
        self.p, self.dir, self.best, self.t0 = prof, direction, float(start_tick), t0
        self.state, self.t_mark, self.lead_mark, self.waited_since = OK, t0, None, None

    def update(self, t, present, lead):
        """lead: how far the command is ahead of the measured position in the commanded direction (ticks).
          lead <= stationary lead                      normal standing / start: not timed
          lead <= following lead                       a loaded joint may stand or follow here: not timed (a command that
                                                       has ended leaves it here; reaching is judged afterwards)
          lead >  following lead                       timed, whatever small progress there is; the clock restarts only
                                                       when the lead itself has come down (the joint is catching up)
        Returns OK / WAIT (the command must not advance) / NO_PROGRESS."""
        if self.dir * (present - self.best) >= self.p["resolution"]:
            self.best = float(present)
        if self.dir == 0 or lead <= self.p["follow"]:
            self.state, self.t_mark, self.lead_mark, self.waited_since = OK, t, None, None
            return self.state
        catching_up = self.lead_mark is not None and lead <= self.lead_mark - self.p["resolution"]     # the lead came down since the last reading
        first = self.lead_mark is None
        self.lead_mark = lead
        if first or catching_up:
            self.t_mark = t
            if self.state == WAIT:
                self.waited_since = t
            return self.state
        if self.state == OK and t - self.t_mark > self.p["start_delay"]:
            self.state, self.waited_since = WAIT, t
        elif self.state == WAIT and t - self.waited_since > self.p["start_delay"]:
            self.state = NO_PROGRESS
        return self.state


def reached(prof, present, target):
    return abs(present - target) <= prof["reach"]
