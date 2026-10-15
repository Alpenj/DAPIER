#!/usr/bin/env python3
"""TJJ task manager: the state transitions and the message types that connect observation, base approach, manipulation
(pick / hand-over if needed / PLACE), place verification, area re-observation, cleaning and the battery return path.

Pure Python, no simulator and no robot import. Every module is reached through an interface; an implementation says what it
is (`REAL`, `SIM`, `MOCK`, `NOT_IMPLEMENTED`). A run that used a MOCK or NOT_IMPLEMENTED module is never a system success.

Completion is decided from what a robot can sense (image / depth / joint and gripper feedback): `observation_validity`
carries frame, time stamp and age. Simulator ground truth never enters this module; it stays in the scorer of the SIM tool.

RELEASE_B (left hand lets go, right hand holds) is the end of a hand-over, not PLACE. PLACE is complete only when the
object rests inside the target region and the gripper is separate from it (PlaceVerification).
"""
from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field, asdict
from typing import Optional, Protocol


class State(str, enum.Enum):
    IDLE = "IDLE"
    OBSERVE = "OBSERVE"
    APPROACH = "APPROACH"
    BASE_STOPPED = "BASE_STOPPED"
    REOBSERVE = "REOBSERVE"
    PICK = "PICK"
    COORDINATE = "COORDINATE"            # two-arm coordination / hand-over, only if the plan needs it
    RELEASE = "RELEASE"                  # the giving hand lets go and retreats (end of a hand-over); only after COORDINATE
    PLACE = "PLACE"
    VERIFY_PLACE = "VERIFY_PLACE"
    ARM_STOW = "ARM_STOW"                # both arms leave the original area and the top camera's view of it
    REOBSERVE_CLEAR = "REOBSERVE_CLEAR"  # ORIGINAL_AREA_REOBSERVE
    CLEAN_RESUME_REQUEST = "CLEAN_RESUME_REQUEST"   # the request is emitted; with no cleaning device this is where a run ends
    RESUME_CLEANING = "RESUME_CLEANING"
    DONE = "DONE"
    SAFE_STOW = "SAFE_STOW"              # battery path: bring the manipulation to a safe end first
    RETURN = "RETURN"
    DOCK = "DOCK"
    VERIFY_CHARGING = "VERIFY_CHARGING"
    CHARGING = "CHARGING"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class Kind(str, enum.Enum):
    REAL = "REAL"
    SIM = "SIM"
    MOCK = "MOCK"
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"


class Status(str, enum.Enum):
    SUCCEEDED = "SUCCEEDED"
    RUNNING = "RUNNING"
    FAILED = "FAILED"
    REJECTED = "REJECTED"
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"


@dataclass(frozen=True)
class Validity:
    """Whether an observation may be used now. A position without frame and time is not accepted anywhere."""
    frame_id: str
    stamp_s: float
    valid: bool
    source: str                           # e.g. "os30a depth + K/T", "joint feedback"
    reason: str = ""
    calibrated: bool = False              # False: SIM_ONLY / REAL_MODALITY_UNVERIFIED geometry

    def fresh(self, now_s: float, max_age_s: float) -> bool:
        return self.valid and 0.0 <= now_s - self.stamp_s <= max_age_s


@dataclass(frozen=True)
class Region:
    region_id: str
    frame_id: str
    center_xy: tuple
    radius_m: float
    surface_z_m: float
    fixed: bool = True                    # True: the first demo's fixed target; a variable target needs a goal input to the controller


@dataclass(frozen=True)
class ObjectObservation:
    object_id: str
    position_xyz: tuple
    extent_xy_m: tuple
    validity: Validity


@dataclass(frozen=True)
class AreaObservation:
    region_id: str
    clear: bool
    points_above_surface: int
    validity: Validity


@dataclass(frozen=True)
class PlaceVerification:
    object_in_region: bool
    object_supported: bool                # resting on the surface, not carried
    gripper_separated: bool
    evidence: dict
    validity: Validity

    @property
    def placed(self) -> bool:
        return self.validity.valid and self.object_in_region and self.object_supported and self.gripper_separated


@dataclass(frozen=True)
class CommandResult:
    status: Status
    kind: Kind
    detail: str = ""
    stamp_s: float = 0.0
    evidence: dict = field(default_factory=dict)


@dataclass(frozen=True)
class Task:
    task_id: str
    source_region: Region
    target_region: Region
    object_hint: str = ""
    needs_two_arm_coordination: bool = False


class Perception(Protocol):
    kind: Kind
    def observe_object(self, task: Task, now_s: float) -> Optional[ObjectObservation]: ...
    def observe_area(self, region: Region, now_s: float) -> AreaObservation: ...
    def verify_place(self, task: Task, now_s: float) -> PlaceVerification: ...


class Base(Protocol):
    kind: Kind
    def approach(self, target: ObjectObservation, now_s: float) -> CommandResult: ...
    def stopped(self, now_s: float) -> CommandResult: ...
    def return_to_dock(self, now_s: float) -> CommandResult: ...


class Manipulation(Protocol):
    kind: Kind
    def pick(self, target: ObjectObservation, now_s: float) -> CommandResult: ...
    def coordinate(self, task: Task, now_s: float) -> CommandResult: ...
    def release(self, task: Task, now_s: float) -> CommandResult: ...
    def place(self, task: Task, now_s: float) -> CommandResult: ...
    def stow(self, now_s: float) -> CommandResult: ...


class Cleaner(Protocol):
    kind: Kind
    def resume(self, region: Region, now_s: float) -> CommandResult: ...


class Power(Protocol):
    kind: Kind
    def battery_low(self, now_s: float) -> bool: ...
    def dock(self, now_s: float) -> CommandResult: ...
    def charging(self, now_s: float) -> CommandResult: ...


@dataclass
class Event:
    task_id: str
    stamp_s: float
    state: str
    next_state: str
    reason: str
    module_kind: str
    evidence: dict


class TaskManager:
    """One task at a time. `step()` performs one transition and returns the new state; every transition is logged with
    the evidence it used. Nothing here retries silently: a rejected or stale observation is a logged transition."""

    def __init__(self, perception, base, manipulation, cleaner, power, *, max_observation_age_s=1.0, clock=time.monotonic):
        self.perception, self.base, self.manipulation, self.cleaner, self.power = perception, base, manipulation, cleaner, power
        self.max_age, self.clock = max_observation_age_s, clock
        self.state, self.task, self.target, self.events = State.IDLE, None, None, []
        self.failure, self.cancel_requested, self.kinds_used = None, False, set()
        self.request_emitted, self.stopped_at_request = False, False

    # ---- public --------------------------------------------------------------------------------
    def start(self, task: Task):
        if self.state not in (State.IDLE, State.DONE, State.FAILED, State.CANCELLED, State.CHARGING):
            raise RuntimeError(f"a task is running ({self.state.value})")
        self.task, self.target, self.failure, self.cancel_requested = task, None, None, False
        self.request_emitted, self.stopped_at_request = False, False
        self._go(State.OBSERVE, "task accepted", Kind.REAL, dict(task=asdict(task)))

    def cancel(self):
        self.cancel_requested = True

    def resume(self, task: Task, state: State, target, reason: str, kind: Kind, evidence: dict):
        """Continue a task at `state` from a restored execution prefix (a diagnostic of the later stages). The entry is a
        logged event; the stages before it were not run by this manager and are not reported as done by it."""
        if self.state not in (State.IDLE, State.DONE, State.FAILED, State.CANCELLED, State.CHARGING):
            raise RuntimeError(f"a task is running ({self.state.value})")
        self.task, self.target, self.failure, self.cancel_requested = task, target, None, False
        self.request_emitted, self.stopped_at_request = False, False
        self._go(state, f"RESUMED FROM A RESTORED PREFIX: {reason}", kind, evidence)

    def run(self, max_steps=100):
        for _ in range(max_steps):
            if self.state in (State.DONE, State.FAILED, State.CANCELLED, State.CHARGING, State.IDLE) or self.stopped_at_request:
                break
            self.step()
        return self.state

    @property
    def system_success(self) -> bool:
        """True only if the task finished and no MOCK / NOT_IMPLEMENTED module was on the way."""
        return self.state == State.DONE and not ({Kind.MOCK, Kind.NOT_IMPLEMENTED} & self.kinds_used)

    def report(self):
        return dict(task_id=None if self.task is None else self.task.task_id, state=self.state.value, failure=self.failure, system_success=self.system_success,
                    clean_resume_request_emitted=self.request_emitted,
                    module_kinds_used=sorted(k.value for k in self.kinds_used), events=[asdict(e) for e in self.events])

    # ---- transitions ----------------------------------------------------------------------------
    def step(self):
        now, s, t = self.clock(), self.state, self.task
        if self.cancel_requested and s not in (State.DONE, State.FAILED, State.CANCELLED):
            r = self.manipulation.stow(now)
            return self._go(State.CANCELLED, f"cancel requested; stow {r.status.value}", r.kind, r.evidence)
        manipulation_states = (State.PICK, State.COORDINATE, State.RELEASE, State.PLACE, State.VERIFY_PLACE, State.ARM_STOW)
        battery_states = (State.SAFE_STOW, State.RETURN, State.DOCK, State.VERIFY_CHARGING)
        if s not in battery_states and s not in (State.DONE, State.FAILED, State.CANCELLED, State.CHARGING, State.IDLE) and self.power.battery_low(now):
            return self._go(State.SAFE_STOW, "battery low" + (" during manipulation: finish safely first" if s in manipulation_states else ""), self.power.kind, {})
        if s in (State.OBSERVE, State.REOBSERVE):
            obs = self.perception.observe_object(t, now)
            if obs is None or not obs.validity.fresh(now, self.max_age):
                why = "no object" if obs is None else (obs.validity.reason or "stale or invalid observation")
                return self._fail(f"{s.value}: {why}", self.perception.kind, {} if obs is None else dict(validity=asdict(obs.validity)))
            self.target = obs
            return self._go(State.APPROACH if s == State.OBSERVE else State.PICK, "object observed", self.perception.kind, dict(object=asdict(obs)))
        if s == State.APPROACH:
            return self._after(self.base.approach(self.target, now), State.BASE_STOPPED)
        if s == State.BASE_STOPPED:
            return self._after(self.base.stopped(now), State.REOBSERVE)
        if s == State.PICK:
            return self._after(self.manipulation.pick(self.target, now), State.COORDINATE if t.needs_two_arm_coordination else State.PLACE)
        if s == State.COORDINATE:
            return self._after(self.manipulation.coordinate(t, now), State.RELEASE)
        if s == State.RELEASE:
            return self._after(self.manipulation.release(t, now), State.PLACE)
        if s == State.PLACE:
            return self._after(self.manipulation.place(t, now), State.VERIFY_PLACE)
        if s == State.VERIFY_PLACE:
            v = self.perception.verify_place(t, now)
            if not v.validity.fresh(now, self.max_age):
                return self._fail("VERIFY_PLACE: stale or invalid observation", self.perception.kind, dict(verification=asdict(v)))
            if not v.placed:
                return self._fail("VERIFY_PLACE: object not placed (in region / supported / gripper separated)", self.perception.kind, dict(verification=asdict(v)))
            return self._go(State.ARM_STOW, "object placed and released", self.perception.kind, dict(verification=asdict(v)))
        if s == State.ARM_STOW:
            return self._after(self.manipulation.stow(now), State.REOBSERVE_CLEAR)
        if s == State.REOBSERVE_CLEAR:
            a = self.perception.observe_area(t.source_region, now)
            if not a.validity.fresh(now, self.max_age):                      # not observable is not the same as occupied
                return self._fail(f"REOBSERVE_CLEAR: the source region could not be observed ({a.validity.reason or 'stale observation'})", self.perception.kind, dict(area=asdict(a)))
            if not a.clear:
                return self._fail("REOBSERVE_CLEAR: the source region is not clear", self.perception.kind, dict(area=asdict(a)))
            return self._go(State.CLEAN_RESUME_REQUEST, "source region clear: clean-resume request emitted", self.perception.kind, dict(area=asdict(a)))
        if s == State.CLEAN_RESUME_REQUEST:
            r = self.cleaner.resume(t.source_region, now)
            if r.status == Status.NOT_IMPLEMENTED:                           # no cleaning device: the request stays emitted, nothing is called cleaned
                self.kinds_used.add(r.kind)
                self.request_emitted = True
                self.stopped_at_request = True
                return s
            return self._after(r, State.RESUME_CLEANING)
        if s == State.RESUME_CLEANING:
            return self._go(State.DONE, "cleaning module reports it resumed", self.cleaner.kind, {})
        if s == State.SAFE_STOW:
            return self._after(self.manipulation.stow(now), State.RETURN)
        if s == State.RETURN:
            return self._after(self.base.return_to_dock(now), State.DOCK)
        if s == State.DOCK:
            return self._after(self.power.dock(now), State.VERIFY_CHARGING)
        if s == State.VERIFY_CHARGING:
            return self._after(self.power.charging(now), State.CHARGING)
        return s

    # ---- helpers --------------------------------------------------------------------------------
    def _after(self, result: CommandResult, nxt: State):
        if result.status == Status.SUCCEEDED:
            return self._go(nxt, result.detail or "command succeeded", result.kind, result.evidence)
        if result.status == Status.RUNNING:
            self.kinds_used.add(result.kind)
            return self.state
        return self._fail(f"{self.state.value}: {result.status.value} {result.detail}".strip(), result.kind, result.evidence)

    def _go(self, nxt: State, reason, kind, evidence):
        self.kinds_used.add(kind)
        self.events.append(Event(self.task.task_id, self.clock(), self.state.value, nxt.value, reason, kind.value, evidence))
        self.state = nxt
        return nxt

    def _fail(self, reason, kind, evidence):
        self.failure = reason
        return self._go(State.FAILED, reason, kind, evidence)


# --------------------------------------------------------------------------------------------------
# Test doubles. They are MOCK / NOT_IMPLEMENTED on purpose and can never produce a system success.
# --------------------------------------------------------------------------------------------------

class MockBase:
    kind = Kind.MOCK

    def approach(self, target, now_s):
        return CommandResult(Status.SUCCEEDED, self.kind, "MOCK base approach", now_s)

    def stopped(self, now_s):
        return CommandResult(Status.SUCCEEDED, self.kind, "MOCK base stopped", now_s)

    def return_to_dock(self, now_s):
        return CommandResult(Status.SUCCEEDED, self.kind, "MOCK return", now_s)


class StationaryBase:
    """A fixed manipulation scene (the SIM desk): there is no base to move. Approach is a no-op that says so."""
    kind = Kind.SIM

    def approach(self, target, now_s):
        return CommandResult(Status.SUCCEEDED, self.kind, "stationary scene: no base motion", now_s)

    def stopped(self, now_s):
        return CommandResult(Status.SUCCEEDED, self.kind, "stationary scene: base is fixed", now_s)

    def return_to_dock(self, now_s):
        return CommandResult(Status.NOT_IMPLEMENTED, Kind.NOT_IMPLEMENTED, "no mobile base in this scene", now_s)


class MockCleaner:
    kind = Kind.MOCK

    def resume(self, region, now_s):
        return CommandResult(Status.SUCCEEDED, self.kind, "MOCK cleaning resumed (no cleaning device was driven)", now_s)


class NoCleaner:
    """No cleaning device is connected: the request is recorded and nothing is reported as cleaned."""
    kind = Kind.NOT_IMPLEMENTED

    def resume(self, region, now_s):
        return CommandResult(Status.NOT_IMPLEMENTED, self.kind, "no cleaning device connected", now_s)


class PowerExcluded:
    """Charging is not a development target of this MVP: no battery signal is connected, so nothing here ever reports
    a low battery, a dock contact or a charging state."""
    kind = Kind.NOT_IMPLEMENTED

    def battery_low(self, now_s):
        return False

    def dock(self, now_s):
        return CommandResult(Status.NOT_IMPLEMENTED, self.kind, "charging is excluded from this MVP", now_s)

    def charging(self, now_s):
        return CommandResult(Status.NOT_IMPLEMENTED, self.kind, "charging is excluded from this MVP", now_s)


class MockPower:
    kind = Kind.MOCK

    def __init__(self, low_after_calls=None):
        self.calls, self.low_after = 0, low_after_calls

    def battery_low(self, now_s):
        self.calls += 1
        return self.low_after is not None and self.calls > self.low_after

    def dock(self, now_s):
        return CommandResult(Status.SUCCEEDED, self.kind, "MOCK dock contact", now_s)

    def charging(self, now_s):
        return CommandResult(Status.SUCCEEDED, self.kind, "MOCK charging state", now_s)
