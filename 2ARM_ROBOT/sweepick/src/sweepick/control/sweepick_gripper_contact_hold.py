"""TJJ grasp / support primitive: one logic for closing, holding and losing an object, from motor feedback + vision.
Pure Python (no simulator, no robot, no numpy). The same class drives the close command, the hold command and the
grasp / support state; nothing in it is specific to an object width.

Motor side (per jaw pair), from the command and the SEPARATELY measured position of each control step:
  residual = measured - command (positive: the jaws are more open than commanded = something resists the closing)
  CLOSING            the command moves towards closed
  CONTACT_CANDIDATE  while closing, the residual rose above the residual of the free stroke by the device's preload,
                     or, with the command at rest, the residual stays above the device's no-load floor: an EXTERNAL
                     contact candidate. It says nothing yet about WHAT is touched (object, table, other arm)
  NO_CONTACT         the command is at rest and the residual is at the no-load floor: target reached / nothing held
  EMPTY_END_STOP     the closing reached the jaws' own end stop without a contact candidate
  UNAVAILABLE        feedback stale, torque reported off, non-finite values, or the feedback is a command echo
                     (residual exactly zero through a whole moving stroke): no motor evidence either way

Fusion with vision (facts of the same time window):
  GRASP_CANDIDATE    contact candidate + the task object is observed at this tool point (or valid earlier evidence)
  HELD_CONFIRMED     grasp candidate + the object left its support and moved with the hand (lift / short move)
  SUPPORT_CONFIRMED  the giving hand is unloaded and away, this hand is a contact candidate, the object is observed
                     off the support at this tool point
  LOST               the object is observed resting on the support while this hand should carry it, or the jaws
                     relaxed to no-contact after a confirmed hold
  UNKNOWN            hidden or not enough evidence. UNKNOWN is not LOST.

Device calibration (JawDevice) is about the gripper, not about objects: no-load residual floor, preload residual, closing
rate, end-stop opening, feedback age limit. The values of the SIM gripper are measured on stored SIM runs; a real servo
needs its own (encoder resolution, load / current registers) and they are NOT known yet.
"""
from dataclasses import dataclass
import math

CLOSING, CONTACT_CANDIDATE, NO_CONTACT, EMPTY_END_STOP, UNAVAILABLE, IDLE = "CLOSING", "CONTACT_CANDIDATE", "NO_CONTACT", "EMPTY_END_STOP", "UNAVAILABLE", "IDLE"
NO_DISCRIMINATION = "NO_DISCRIMINATION"      # R6.1: an independent position readback that never differs from the command while moving: no contact-discriminating evidence in this channel
GRASP_CANDIDATE, HELD_CONFIRMED, SUPPORT_CONFIRMED, UNKNOWN, LOST = "GRASP_CANDIDATE", "HELD_CONFIRMED", "SUPPORT_CONFIRMED", "UNKNOWN", "LOST"
HELD_ESTIMATED = "HELD_ESTIMATED"            # R6.1: contact candidate + the hand rose + an EARLIER observation of the object at the tool; the object was not observed moving with the hand
INDEPENDENT, ECHO, UNKNOWN_PROVENANCE = "independent register / sensor", "command echo (the driver copies the command)", "unknown"


@dataclass(frozen=True)
class JawDevice:
    """Calibration of one gripper type (not of an object)."""
    residual_floor: float           # largest |measured - command| at rest with nothing between the jaws
    preload_residual: float         # residual AT REST at which the squeeze stops and holds (the gripper's grip set point)
    creep_step: float               # one squeeze increment of the command after the contact onset (opening units)
    creep_period_s: float           # time between squeeze increments (the residual is read at rest in between)
    close_rate_per_s: float         # closing speed of the command (opening units / s)
    end_stop_opening: float         # smallest commanded opening (the jaws' own end stop, with its margin)
    rest_step: float                # |command change| per step below which the command counts as at rest
    max_feedback_age_s: float       # older feedback is not used
    confirm_s: float                # how long a state must persist
    source: str = ""


SIM_PGRIPPER = JawDevice(residual_floor=1e-5, preload_residual=2.1e-5, creep_step=0.006, creep_period_s=0.1, close_rate_per_s=0.8 / 2.2028, end_stop_opening=0.05, rest_step=1e-4, max_feedback_age_s=0.1, confirm_s=0.2,
                         source="SIM PGripper, from stored runs: no-load residual <= 1.1e-6 at rest and <= 4.9e-6 under a jittering policy command; residual while the teacher holds at its 2 N set point "
                                "1.7e-5 .. 2.5e-5 (floor 1e-5 lies between the two) "
                                "(preload = median 2.1e-5); creep = one closing-rate step every 0.1 s; closing rate = TeacherConfig.close_rate_rad_s; end stop = opening below which the jaws' own pads are closer than 1 mm in the robot model")


class Jaw:
    """Motor-side state of one jaw pair + its closing / holding command."""

    def __init__(self, device: JawDevice, dt: float, provenance: str = UNKNOWN_PROVENANCE):
        """dt = the controller's step (used by the squeeze timer only). provenance = what the measured position IS:
        INDEPENDENT, ECHO or UNKNOWN_PROVENANCE. It is a fact about the driver, not something inferred from the values."""
        self.dev, self.dt, self.provenance = device, dt, provenance
        self.last_stamp = self.last_seq = None
        self.duplicates = self.backward = self.gaps = self.samples = self.recoveries = 0
        self.mode, self.state, self.hold_command = "free", IDLE, None        # mode: free (someone else commands) | close | hold
        self.prev_cmd = self.prev_t = None
        self.rest_s = self.loaded_s = self.relaxed_s = 0.0
        self.stroke, self.echo_steps, self.moving_steps, self.evidence, self.log = [], 0, 0, "none yet", []
        self.contact_opening = self.was_loaded = None
        self.onset, self.creep_t, self.residual, self.latched = 0, 0.0, 0.0, False

    # ---- feedback of one control step ------------------------------------------------------------------
    def update(self, t, command, measured, *, feedback_stamp=None, torque_enabled=None, seq=None):
        """command = what was last sent; measured = the separately measured position; feedback_stamp / seq identify the
        MEASUREMENT (default: t). Durations (at rest, loaded, relaxed) advance by the time between two NEW measurements:
        a repeated sample adds nothing, a sample that goes back in time is ignored, and after a gap longer than the
        device's feedback age limit the duration counters start again (a gap is not evidence either way, and it is not a
        lost object). Returns the motor state."""
        dev = self.dev
        stamp = t if feedback_stamp is None else feedback_stamp
        age = t - stamp
        if not (isinstance(command, (int, float)) and isinstance(measured, (int, float)) and math.isfinite(command) and math.isfinite(measured)):
            return self._set(t, UNAVAILABLE, "non-finite command or feedback")
        if age > dev.max_feedback_age_s or age < 0:
            return self._set(t, UNAVAILABLE, f"feedback {age:.3f} s old (limit {dev.max_feedback_age_s} s)")
        if torque_enabled is False:
            return self._set(t, UNAVAILABLE, "torque reported off: a position residual is not a contact")
        if self.last_stamp is not None:
            if (seq is not None and seq == self.last_seq) or stamp == self.last_stamp:
                self.duplicates += 1                                          # the same measurement again: no new evidence, nothing accumulates
                return self.state
            if stamp < self.last_stamp:
                self.backward += 1                                            # out of order: not used
                return self.state
        elapsed = 0.0 if self.last_stamp is None else stamp - self.last_stamp
        if elapsed > dev.max_feedback_age_s:
            self.gaps += 1                                                    # samples are missing in between: the durations start again
            self.rest_s = self.loaded_s = self.relaxed_s = 0.0
            elapsed = 0.0
        self.last_stamp, self.last_seq, self.samples = stamp, seq, self.samples + 1
        res = measured - command
        step = 0.0 if self.prev_cmd is None else command - self.prev_cmd
        self.prev_cmd, self.residual, self.measured, self.command = command, res, measured, command
        moving = abs(step) > dev.rest_step
        if moving:
            self.moving_steps += 1
            self.echo_steps += res == 0.0
            self.rest_s = 0.0
        else:
            self.rest_s += elapsed
        if self.provenance != ECHO and res != 0.0 and self.moving_steps >= 20 and self.echo_steps == self.moving_steps:
            # READ02: a NEW measurement that differs from the command is new independent evidence. The earlier run of
            # equal values described those samples, not the channel for ever: start the count again and judge this
            # sample by the ordinary time / residual rules below. (A declared echo never takes this path.)
            self.recoveries += 1
            self.moving_steps = self.echo_steps = 0
        if self.moving_steps >= 20 and self.echo_steps == self.moving_steps:
            # Equal numbers alone do not say what the channel is. A declared echo is no measurement; an independent (or
            # unknown) readback that never differs has no contact-discriminating evidence, e.g. its resolution is too coarse.
            if self.provenance == ECHO:
                return self._set(t, UNAVAILABLE, "the driver declares this position a command echo: not an independent measurement")
            return self._set(t, NO_DISCRIMINATION, f"measured == command through {self.moving_steps} moving samples (provenance: {self.provenance}): this position channel shows no residual, "
                                                   "so it cannot tell a contact from free motion (possibly too coarse a resolution); it is not called a fake sensor and it is not a contact")
        closing = step < -dev.rest_step
        if closing:
            self.stroke.append(res)
            base = sorted(self.stroke[:8])[len(self.stroke[:8]) // 2] if len(self.stroke) >= 3 else None
            self.onset = self.onset + 1 if (base is not None and res - base >= dev.residual_floor) else 0
            if self.onset >= 2:                                               # the residual left the free stroke: contact onset
                self.contact_opening = measured
                return self._set(t, CONTACT_CANDIDATE, f"closing: residual {res:.2e} is {res - base:.2e} above the free stroke")
            if command <= dev.end_stop_opening + 1e-12:
                return self._set(t, EMPTY_END_STOP, "the closing reached the jaws' own end stop without a contact candidate")
            return self._set(t, CLOSING, "")
        if not moving:
            self.stroke = []
            if res > dev.residual_floor:
                self.loaded_s += elapsed
                self.relaxed_s = 0.0
                if self.loaded_s >= dev.confirm_s:
                    self.was_loaded = True
                    if self.contact_opening is None:
                        self.contact_opening = measured
                    return self._set(t, CONTACT_CANDIDATE, f"at rest: residual {res:.2e} above the no-load floor {dev.residual_floor:.1e}")
                return self.state
            self.relaxed_s += elapsed
            self.loaded_s = 0.0
            if self.relaxed_s >= dev.confirm_s:
                if command <= dev.end_stop_opening + 1e-12:
                    return self._set(t, EMPTY_END_STOP, "at the end stop with no residual")
                return self._set(t, NO_CONTACT, "at rest: residual at the no-load floor")
            return self.state
        self.loaded_s = self.relaxed_s = 0.0
        return self._set(t, IDLE, "opening")

    def _set(self, t, state, why):
        if state == CONTACT_CANDIDATE:
            self.latched = True                                               # stays through command jitter and arm motion ...
        elif state in (NO_CONTACT, EMPTY_END_STOP, UNAVAILABLE, NO_DISCRIMINATION):
            self.latched = False                                              # ... until the jaws are seen at rest without a residual (or the feedback is unusable)
        if state != self.state:
            self.log.append(dict(t=round(t, 3), state=state, why=why, opening=getattr(self, "measured", None), residual=getattr(self, "residual", None)))
            self.state = state
        self.evidence = ("motor feedback (command and measured position are separate)" if state not in (UNAVAILABLE, NO_DISCRIMINATION) else
                         ("motor evidence UNAVAILABLE: " if state == UNAVAILABLE else "position channel without contact evidence: ") + why)
        return state

    @property
    def loaded(self):
        """An external contact candidate that has not been seen to relax since (latched: a moving command or a moving arm
        does not clear it; a residual at the no-load floor with the command at rest does)."""
        return self.latched

    @property
    def dropped(self):
        return bool(self.was_loaded and self.state in (NO_CONTACT, EMPTY_END_STOP))

    # ---- the closing / holding command -----------------------------------------------------------------------
    def start_close(self):
        self.mode, self.hold_command, self.stroke, self.loaded_s, self.relaxed_s, self.was_loaded = "close", None, [], 0.0, 0.0, None
        self.onset, self.creep_t, self.latched = 0, 0.0, False

    def release_control(self):
        self.mode, self.hold_command = "free", None

    def next_command(self, last_command):
        """The next gripper command while this primitive owns the jaw:
          close    at the device rate until the residual leaves the free stroke (contact onset);
          squeeze  one small step every creep period, reading the residual at rest in between, until it reaches the
                   device preload (a servo on the measured residual; no opening target);
          hold     the command where that happened.
        No target opening comes from an object table."""
        dev = self.dev
        if self.mode == "hold":
            return self.hold_command
        if self.mode == "close":
            if self.state == CONTACT_CANDIDATE:
                self.mode, self.creep_t = "squeeze", 0.0
                return last_command
            if self.state in (EMPTY_END_STOP, UNAVAILABLE, NO_DISCRIMINATION):
                self.mode, self.hold_command = "hold", last_command       # stop moving; the caller reads the state
                return last_command
            return max(dev.end_stop_opening, last_command - dev.close_rate_per_s * self.dt)
        if self.mode == "squeeze":
            if self.state in (UNAVAILABLE, NO_DISCRIMINATION):
                self.mode, self.hold_command = "hold", last_command
                return last_command
            self.creep_t += self.dt
            if self.creep_t < dev.creep_period_s:
                return last_command
            self.creep_t = 0.0
            if self.residual >= dev.preload_residual or last_command <= dev.end_stop_opening + 1e-12:
                self.mode, self.hold_command = "hold", last_command       # hold at the measured grip
                return last_command
            return max(dev.end_stop_opening, last_command - dev.creep_step)
        return last_command

    def closed_and_resting(self):
        return self.mode == "hold" and self.rest_s >= self.dev.confirm_s


def fuse(jaw_state, *, object_at_tool, evidence_valid, object_on_support, co_motion_observed=False, hand_raised_since_contact=False, other_hand_unloaded_and_away=None, was_held=False,
         hand_raised=True):
    """One grasp / support LABEL of a hand from its motor state and the vision facts of the same time window.
    object_at_tool: True / False / None (None = hidden: not observable now).
    co_motion_observed: the object was observed at this tool point at two different tool positions (it moved with the
        hand) - a direct observation. hand_raised_since_contact alone is a pose fact, not that observation.
    HELD_CONFIRMED needs the direct co-motion; a raised hand with only an earlier observation is HELD_ESTIMATED."""
    if object_on_support and hand_raised:
        return LOST
    if was_held and jaw_state in (NO_CONTACT, EMPTY_END_STOP):
        return LOST if object_at_tool is False else UNKNOWN
    contact = jaw_state == CONTACT_CANDIDATE
    if contact and other_hand_unloaded_and_away and object_at_tool and object_on_support is False:
        return SUPPORT_CONFIRMED
    if contact and co_motion_observed and (object_at_tool or evidence_valid) and not object_on_support:
        return HELD_CONFIRMED
    if contact and hand_raised_since_contact and (object_at_tool or evidence_valid) and not object_on_support:
        return HELD_ESTIMATED
    if contact and (object_at_tool or evidence_valid):
        return GRASP_CANDIDATE
    if contact:
        return CONTACT_CANDIDATE                                           # something resists: not known to be the object (table, other arm, ...)
    return UNKNOWN if jaw_state in (UNAVAILABLE, NO_DISCRIMINATION, CLOSING, IDLE) else jaw_state
