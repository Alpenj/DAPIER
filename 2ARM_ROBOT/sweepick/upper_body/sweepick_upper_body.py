"""SWEEPICK upper body (both SO-101 arms on the TurtleBot3) — mission contract, OFFLINE / MOCK.

Scope (2026-10-16 directive SWEEPICK_UPPER_BODY_ONLY): the arms, grippers, upper cameras, arm selection and
PICK -> HOLD -> transport pose -> PLACE/RELEASE, with an explicit boundary to the base team. The base (wheels, OpenCR,
Nav2, RTAB-Map, docking, base TF) belongs to another team: nothing here commands it.

This module is the CONTRACT layer only:
  * the message schema both teams exchange (validated, never trusted by a single bit),
  * SELECT_ARM over per-arm evidence (never "the shortest distance" alone),
  * the mission state machine PICK_AT_STOP -> LIFT_HELD -> SECURE_TRANSPORT_POSE -> HOLD_WHILE_BASE_MOVES ->
    PLACE_AT_STOP -> RELEASE_CHECK -> ARM_SAFE_END.
The arm work itself is done by an injected adapter. The REAL adapter (the existing native Owner / stage / STOP / IK /
collision path of sweepick_pick01 + sweepick_move_a) is NOT wired here; the tests use a mock adapter. Nothing in this module
opens a device, writes a register or starts a session.

No numeric limit is invented here: no distance, speed, torque, tolerance or time window. Freshness is ordering — an
observation or result must come after the base evidence it relies on, in the same clock domain, for the same task and
stop; the timing allowances belong to the base-team interface agreement and are left UNSET."""
import json

SCHEMA = 'sweepick.upper-body-msg.v1'
REQUIRED = ('schema', 'kind', 'task_id', 'sequence_id', 'side', 'source_kind', 'source_identity', 'stamp', 'clock_id', 'status', 'proof_ref')
SOURCE_KINDS = ('REAL', 'SIM', 'MOCK', 'REPLAY')
SIDES = ('left', 'right', 'none')

# received from the base team / the operator
BASE_PARKED_AT_PICKUP = 'BASE_PARKED_AT_PICKUP'
BASE_PARKED_AT_DISPOSAL = 'BASE_PARKED_AT_DISPOSAL'
BASE_MOVING = 'BASE_MOVING'
BASE_STOP_OR_FAULT = 'BASE_STOP_OR_FAULT'
OPERATOR_AUTHORIZATION = 'OPERATOR_AUTHORIZATION'
INBOUND = (BASE_PARKED_AT_PICKUP, BASE_PARKED_AT_DISPOSAL, BASE_MOVING, BASE_STOP_OR_FAULT, OPERATOR_AUTHORIZATION)
# sent by the upper body
ARM_NOT_READY = 'ARM_NOT_READY'
ARM_PICK_CONFIRMED = 'ARM_PICK_CONFIRMED'
ARM_TRANSPORT_READY = 'ARM_TRANSPORT_READY'
ARM_DISPOSAL_CONFIRMED = 'ARM_DISPOSAL_CONFIRMED'
ARM_STOPPED = 'ARM_STOPPED'
ARM_FAULT = 'ARM_FAULT'
ARM_RECOVERY_REQUIRED = 'ARM_RECOVERY_REQUIRED'
BASE_BLOCK_REQUEST = 'BASE_BLOCK_REQUEST'                # "do not start the next base move" — a request, never a base command
OUTBOUND = (ARM_NOT_READY, ARM_PICK_CONFIRMED, ARM_TRANSPORT_READY, ARM_DISPOSAL_CONFIRMED, ARM_STOPPED, ARM_FAULT, ARM_RECOVERY_REQUIRED, BASE_BLOCK_REQUEST)

# the extra evidence each inbound kind must carry (a bare "arrived=true" is not enough)
INBOUND_EVIDENCE = {
    BASE_PARKED_AT_PICKUP: ('stop_location_id', 'base_stopped_proof', 'controller_idle_proof'),
    BASE_PARKED_AT_DISPOSAL: ('stop_location_id', 'base_stopped_proof', 'controller_idle_proof', 'disposal_area_id', 'local_observation_permitted'),
    BASE_MOVING: (),
    BASE_STOP_OR_FAULT: ('reason',),
    OPERATOR_AUTHORIZATION: ('scope', 'native_manifest_sha256', 'supervised', 'recovery_state', 'robot_state_ref'),
}
AUTH_SCOPE = 'UPPER_BODY_PICK_TRANSPORT_PLACE'
UNKNOWN = 'UNKNOWN'

# the mission's arm-side states (an order of behaviour, not an approval to move)
STATES = ('IDLE', 'PICK_AT_STOP', 'LIFT_HELD', 'SECURE_TRANSPORT_POSE', 'HOLD_WHILE_BASE_MOVES', 'PLACE_AT_STOP', 'RELEASE_CHECK', 'ARM_SAFE_END',
          'NOT_READY', 'STOPPED', 'FAULT', 'RECOVERY_REQUIRED')
# per-arm evaluation, in this order; the first that is not True decides that arm (False: refused, None: UNKNOWN)
ARM_CHECKS = ('verified_capability', 'mechanical_state', 'approach_ik', 'collision_and_view', 'grasp_room', 'transport_pose', 'place_pose')


def validate(msg, *, inbound=None):
    """Schema errors of one message (empty list = structurally valid). Never raises on bad input."""
    if not isinstance(msg, dict):
        return ['not a JSON object']
    errors = [f'missing {k}' for k in REQUIRED if msg.get(k) in (None, '')]
    if msg.get('schema') not in (None, SCHEMA):
        errors.append(f"schema {msg.get('schema')!r} is not {SCHEMA}")
    kind = msg.get('kind')
    allowed = INBOUND if inbound is True else OUTBOUND if inbound is False else INBOUND + OUTBOUND
    if kind not in allowed:
        errors.append(f'unknown kind {kind!r}')
    if msg.get('source_kind') not in SOURCE_KINDS:
        errors.append(f"source_kind {msg.get('source_kind')!r} not in {SOURCE_KINDS}")
    if msg.get('side') not in SIDES:
        errors.append(f"side {msg.get('side')!r} not in {SIDES}")
    if not isinstance(msg.get('sequence_id'), int) or isinstance(msg.get('sequence_id'), bool):
        errors.append('sequence_id must be an integer')
    if not isinstance(msg.get('stamp'), (int, float)) or isinstance(msg.get('stamp'), bool) or msg.get('stamp') != msg.get('stamp'):
        errors.append('stamp must be a finite number')
    for field in INBOUND_EVIDENCE.get(kind, ()):
        if msg.get(field) in (None, '', False) and not (field == 'recovery_state' and msg.get(field) is not None):
            errors.append(f'{kind} needs {field}')
    return errors


def message(kind, task_id, sequence_id, side, *, source_kind, source_identity, stamp, clock_id, status, proof_ref, **extra):
    msg = dict(schema=SCHEMA, kind=kind, task_id=task_id, sequence_id=sequence_id, side=side, source_kind=source_kind,
               source_identity=source_identity, stamp=stamp, clock_id=clock_id, status=status, proof_ref=proof_ref, **extra)
    errors = validate(msg, inbound=kind in INBOUND)
    if errors:
        raise ValueError('; '.join(errors))
    return msg


def after(evidence, base_msg):
    """True when `evidence` (an observation / arm result with stamp + clock_id) is newer than `base_msg` in the same clock
    domain. Different clock domains are not comparable: False (UNKNOWN is not 'fresh')."""
    return (isinstance(evidence, dict) and isinstance(base_msg, dict) and evidence.get('clock_id') == base_msg.get('clock_id')
            and isinstance(evidence.get('stamp'), (int, float)) and evidence['stamp'] > base_msg['stamp'])


class BaseView:
    """What the upper body knows about the base, from validated messages of ONE task only. Ordering by sequence_id within
    the task's clock domain; a message with an older sequence_id than the last accepted one is ignored."""

    def __init__(self, task_id):
        self.task_id, self.last, self.rejected = task_id, None, []

    def receive(self, msg):
        errors = validate(msg, inbound=True)
        if not errors and msg['kind'] == OPERATOR_AUTHORIZATION:
            errors.append('an operator authorization is not a base message')
        if not errors and msg['task_id'] != self.task_id:
            errors.append(f"task {msg['task_id']} is not this task {self.task_id}")
        if not errors and self.last is not None and msg['clock_id'] != self.last['clock_id']:
            errors.append('clock domain changed: base messages are no longer comparable')
        if not errors and self.last is not None and msg['sequence_id'] <= self.last['sequence_id']:
            errors.append('older or repeated sequence_id')
        if errors:
            self.rejected.append(dict(msg=msg, errors=errors)); return False
        self.last = msg; return True

    def parked(self, kind, location_id=None):
        """The latest base message, when it says parked at `kind` (and location) and is REAL-sourced evidence of a stop."""
        m = self.last
        if m is None or m['kind'] != kind or m.get('status') != 'PARKED':
            return None
        if location_id is not None and m.get('stop_location_id') != location_id:
            return None
        return m

    def blocks_arm_motion(self):
        """Arm motion is blocked unless the latest base message is a PARKED one (moving, STOP/FAULT, or nothing heard)."""
        m = self.last
        if m is None:
            return 'no base state received for this task'
        if m['kind'] in (BASE_MOVING, BASE_STOP_OR_FAULT):
            return f"base is {m['kind']} ({m.get('reason') or m.get('status')})"
        if m.get('status') != 'PARKED':
            return f"base status {m.get('status')}"
        return None


def authorization_errors(auth, task_id, native_manifest_sha256, mode):
    """The operator's approval for THIS task and THIS native source. A base-team signal never replaces it."""
    errors = validate(auth, inbound=True)
    if errors:
        return errors
    if auth['kind'] != OPERATOR_AUTHORIZATION:
        return ['not an operator authorization']
    if auth['task_id'] != task_id:
        errors.append('authorization is for another task')
    if auth['scope'] != AUTH_SCOPE:
        errors.append(f"authorization scope {auth['scope']!r} is not {AUTH_SCOPE}")
    if auth['native_manifest_sha256'] != native_manifest_sha256:
        errors.append('authorization is bound to another native manifest (new session approval mismatch)')
    if auth.get('supervised') is not True:
        errors.append('no human supervision recorded')
    if auth.get('recovery_state') not in ('NO_OPEN_FAULT', 'OPERATOR_RECOVERY_CONFIRMED'):
        errors.append(f"recovery state {auth.get('recovery_state')!r}: an open FAULT needs the operator's recovery first")
    if mode == 'REAL' and auth['source_kind'] != 'REAL':
        errors.append('a REAL mission needs a REAL operator authorization')
    if auth.get('source_identity', '').startswith('base'):
        errors.append('the base team cannot authorize arm motion')
    return errors


def evaluate_arm(side, checks, mode):
    """One arm's record: every check in ARM_CHECKS order until the first that is not True."""
    rec = dict(side=side, checks={}, eligible=False, decided_by=None)
    for name in ARM_CHECKS:
        c = checks.get(name)
        ok = None if not isinstance(c, dict) else c.get('ok')
        rec['checks'][name] = c if isinstance(c, dict) else dict(ok=None, why='no evidence supplied')
        if name == 'verified_capability' and mode == 'REAL' and ok is True and not (c or {}).get('eligible_for_REAL'):
            ok = False; rec['checks'][name] = dict(c, ok=False, why=(c or {}).get('why') or 'not verified on the real arm: eligible_for_REAL=false')
        if ok is not True:
            rec['decided_by'] = name; rec['status'] = 'UNKNOWN' if ok is None else 'REFUSED'
            rec['why'] = rec['checks'][name].get('why') or ('no evidence' if ok is None else 'refused'); return rec
    rec.update(eligible=True, status='ELIGIBLE', decided_by='all checks')
    return rec


def select_arm(evaluations, mode):
    """SELECT_ARM: each arm judged on its own evidence; only when BOTH are eligible are they compared — by the approach
    joint travel the existing planner reported (the 'nearer' arm in the mission's own terms), never by a straight-line
    distance alone. A tie keeps the arm with REAL evidence (left), and that rule is recorded."""
    recs = {s: evaluate_arm(s, evaluations.get(s) or {}, mode) for s in ('left', 'right')}
    ok = [s for s, r in recs.items() if r['eligible']]
    if not ok:
        return dict(selected=None, decision=ARM_NOT_READY, mode=mode, arms=recs,
                    why='; '.join(f"{s}: {r['decided_by']} {r['status']} ({r['why']})" for s, r in recs.items()))
    if len(ok) == 1:
        return dict(selected=ok[0], decision='SELECTED', mode=mode, arms=recs, why=f'only the {ok[0]} arm passed every check')
    travel = {s: (recs[s]['checks']['approach_ik'] or {}).get('joint_travel_ticks') for s in ok}
    if any(not isinstance(v, (int, float)) for v in travel.values()):
        return dict(selected=None, decision=ARM_NOT_READY, mode=mode, arms=recs, why='both arms eligible but the planner gave no approach travel to compare: not chosen by guess')
    if travel['left'] == travel['right']:
        return dict(selected='left', decision='SELECTED', mode=mode, arms=recs, comparison=travel, why='both eligible, equal approach travel: the arm with REAL PICK evidence (left)')
    pick = min(travel, key=travel.get)
    return dict(selected=pick, decision='SELECTED', mode=mode, arms=recs, comparison=travel, why=f'both eligible: the {pick} arm needs less approach joint travel by the existing planner')


class UpperBodyMission:
    """The arm side of one pickup -> disposal task. Each public step re-checks the base and its own preconditions just before
    it acts (time-of-check vs time-of-use) and gives the adapter a `still_allowed()` to call right before any motion.

    adapter methods (REAL: the existing native path; tests: a mock). Each returns a dict and must NEVER be called while
    still_allowed() is False:
      evaluate_arms(observation) -> {left: {check: {...}}, right: {...}}
      pick(side, still_allowed) -> {result, held: CONFIRMED|UNKNOWN|NOT_HELD, lifted, evidence: {stamp, clock_id, ref}, faults, readback}
      transport_pose(side, still_allowed) -> {reached, readback_ok, torque, gripper_hold: CONFIRMED|UNKNOWN, interference_review: PASS|..., evidence}
      hold_readback(side) -> {readback_ok, gripper_hold, faults, evidence}
      place(side, still_allowed, target) -> {approached, support_and_space: CONFIRMED|..., opened_ack, separated: OBSERVED|UNKNOWN, in_target: OBSERVED|UNKNOWN, retreated, evidence}
      safe_end(side) -> {torque_state, holding_state, evidence}"""

    def __init__(self, task_id, adapter, *, mode='MOCK', native_manifest_sha256, clock_id, source_identity='upper_body'):
        if mode not in ('MOCK', 'REAL'):
            raise ValueError('mode is MOCK or REAL')
        self.task_id, self.a, self.mode, self.manifest, self.clock_id, self.identity = task_id, adapter, mode, native_manifest_sha256, clock_id, source_identity
        self.base = BaseView(task_id); self.state = 'IDLE'; self.side = None; self.sent = []; self.log = []; self.seq = 0
        self.auth = None; self.pickup_park = None; self.stopped = False

    # ---- messages ----------------------------------------------------------------------------------------------------
    def _send(self, kind, status, proof_ref, stamp, **extra):
        self.seq += 1
        m = message(kind, self.task_id, self.seq, self.side or 'none', source_kind=self.mode, source_identity=self.identity, stamp=stamp,
                    clock_id=self.clock_id, status=status, proof_ref=proof_ref, **extra)
        self.sent.append(m); return m

    def _end(self, state, kind, why, stamp, block_base=False, **extra):
        self.state = state; self.log.append(dict(state=state, why=why))
        out = [self._send(kind, state, extra.pop('proof_ref', 'mission log'), stamp, reason=why, **extra)]
        if block_base:
            out.append(self._send(BASE_BLOCK_REQUEST, 'REQUESTED', 'mission log', stamp, reason=f'upper body {state}: do not start the next base move'))
        return out

    def receive(self, msg):
        return self.base.receive(msg)

    def stop(self, stamp, why='operator STOP'):
        """An external STOP: whatever the arm holds stays as it is (UNKNOWN until read back); a person recovers it."""
        self.stopped = True
        return self._end('STOPPED', ARM_STOPPED, why, stamp, block_base=True, holding_state=UNKNOWN, torque_state=UNKNOWN)

    def still_allowed(self):
        return not self.stopped and self.base.blocks_arm_motion() is None

    # ---- steps --------------------------------------------------------------------------------------------------------
    def pick_at_stop(self, auth, observation, stamp):
        if self.state != 'IDLE':
            return self._end('NOT_READY', ARM_NOT_READY, f'pick requested in state {self.state}', stamp)
        park = self.base.parked(BASE_PARKED_AT_PICKUP)
        blocked = self.base.blocks_arm_motion()
        if park is None or blocked:
            return self._end('NOT_READY', ARM_NOT_READY, blocked or 'no valid BASE_PARKED_AT_PICKUP for this task', stamp)
        errors = authorization_errors(auth, self.task_id, self.manifest, self.mode)
        if errors:
            return self._end('NOT_READY', ARM_NOT_READY, 'operator authorization: ' + '; '.join(errors), stamp)
        if not after(observation, park) or observation.get('stop_location_id') != park['stop_location_id']:
            return self._end('NOT_READY', ARM_NOT_READY, 'the observation is not newer than the pickup stop, or from another location (stale / camera from before the stop)', stamp)
        if observation.get('camera_fresh') is not True:
            return self._end('NOT_READY', ARM_NOT_READY, 'camera stale or not re-read after the stop', stamp)
        decision = select_arm(self.a.evaluate_arms(observation), self.mode)
        self.decision = decision
        if decision['decision'] != 'SELECTED':
            return self._end('NOT_READY', ARM_NOT_READY, decision['why'], stamp, decision=decision)
        self.side, self.auth, self.pickup_park, self.state = decision['selected'], auth, park, 'PICK_AT_STOP'
        if not self.still_allowed():                                               # time of use
            return self._end('NOT_READY', ARM_NOT_READY, 'base or STOP changed after the check: no motion', stamp, block_base=True)
        r = self.a.pick(self.side, self.still_allowed)
        return self._after_pick(r, park, stamp)

    def _fault_or_unknown(self, r, stage, stamp):
        """Common fault handling of an arm result. Returns the end messages or None when the result is clean."""
        if r.get('stopped'):
            return self._end('STOPPED', ARM_STOPPED, f'{stage}: STOP', stamp, block_base=True, holding_state=UNKNOWN, torque_state=UNKNOWN)
        if r.get('owner_conflict'):
            return self._end('FAULT', ARM_FAULT, f"{stage}: another owner holds the device ({r['owner_conflict']})", stamp, block_base=True, holding_state=UNKNOWN)
        if r.get('faults'):
            return self._end('RECOVERY_REQUIRED', ARM_RECOVERY_REQUIRED, f"{stage}: servo/bus fault {r['faults']}", stamp, block_base=True,
                             holding_state=UNKNOWN, torque_state=r.get('torque', UNKNOWN), last_safe_state=r.get('last_safe_state', UNKNOWN))
        if r.get('refused'):
            return self._end('NOT_READY' if stage == 'pick' and not r.get('moved') else 'RECOVERY_REQUIRED',
                             ARM_NOT_READY if stage == 'pick' and not r.get('moved') else ARM_RECOVERY_REQUIRED,
                             f"{stage}: refused by the existing checks ({r['refused']})", stamp, block_base=True)
        if r.get('readback_mismatch'):
            return self._end('RECOVERY_REQUIRED', ARM_RECOVERY_REQUIRED, f"{stage}: joint readback does not match the receipt ({r['readback_mismatch']})", stamp, block_base=True, holding_state=UNKNOWN)
        return None

    def _after_pick(self, r, park, stamp):
        bad = self._fault_or_unknown(r, 'pick', stamp)
        if bad:
            return bad
        ev = r.get('evidence') or {}
        if r.get('held') != 'CONFIRMED' or not r.get('lifted') or not after(ev, park):
            return self._end('RECOVERY_REQUIRED' if r.get('moved', True) else 'NOT_READY', ARM_RECOVERY_REQUIRED,
                             f"pick did not confirm a held, lifted block with evidence newer than the stop (held={r.get('held')}, lifted={r.get('lifted')})", stamp, block_base=True)
        self.state = 'LIFT_HELD'
        return [self._send(ARM_PICK_CONFIRMED, 'LIFT_HELD', ev.get('ref', 'pick evidence'), stamp, evidence=ev,
                           note='a held and lifted block at the pickup stop; not a mission completion')]

    def secure_transport(self, stamp):
        if self.state != 'LIFT_HELD':
            return self._end('NOT_READY', ARM_NOT_READY, f'transport pose requested in state {self.state}', stamp, block_base=True)
        blocked = self.base.blocks_arm_motion()
        if blocked or not self.still_allowed():
            return self._end('NOT_READY', ARM_NOT_READY, blocked or 'STOP', stamp, block_base=True)
        self.state = 'SECURE_TRANSPORT_POSE'
        r = self.a.transport_pose(self.side, self.still_allowed)
        bad = self._fault_or_unknown(r, 'transport_pose', stamp)
        if bad:
            return bad
        missing = [k for k, want in (('reached', True), ('readback_ok', True), ('gripper_hold', 'CONFIRMED'), ('interference_review', 'PASS')) if r.get(k) != want]
        if missing:
            return self._end('RECOVERY_REQUIRED', ARM_RECOVERY_REQUIRED, f'transport pose not verified: {missing}', stamp, block_base=True, holding_state=r.get('gripper_hold', UNKNOWN))
        self.state = 'HOLD_WHILE_BASE_MOVES'
        ev = r.get('evidence') or {}
        return [self._send(ARM_TRANSPORT_READY, 'HOLD_WHILE_BASE_MOVES', ev.get('ref', 'transport evidence'), stamp, evidence=ev, torque=r.get('torque'),
                           note='the arm may be carried; this is NOT a base motor command and NOT a mission completion')]

    def hold_check(self, stamp):
        """Called by the mission while the base moves. The arm never moves here; a lost readback or hold blocks the base."""
        if self.state != 'HOLD_WHILE_BASE_MOVES':
            return self._end('NOT_READY', ARM_NOT_READY, f'hold check in state {self.state}', stamp, block_base=True)
        r = self.a.hold_readback(self.side)
        bad = self._fault_or_unknown(r, 'hold', stamp)
        if bad:
            return bad
        if r.get('readback_ok') is not True or r.get('gripper_hold') != 'CONFIRMED':
            return self._end('FAULT', ARM_FAULT, f"holding readback lost during transport (readback_ok={r.get('readback_ok')}, gripper_hold={r.get('gripper_hold')})", stamp,
                             block_base=True, holding_state=r.get('gripper_hold', UNKNOWN))
        return []

    def place_at_stop(self, observation, stamp):
        if self.state != 'HOLD_WHILE_BASE_MOVES':
            return self._end('NOT_READY', ARM_NOT_READY, f'place requested in state {self.state}', stamp, block_base=True)
        park = self.base.parked(BASE_PARKED_AT_DISPOSAL)
        blocked = self.base.blocks_arm_motion()
        if park is None or blocked:
            return self._end('NOT_READY', ARM_NOT_READY, blocked or 'no valid BASE_PARKED_AT_DISPOSAL for this task', stamp, block_base=True)
        if self.pickup_park and park.get('stop_location_id') == self.pickup_park.get('stop_location_id'):
            return self._end('NOT_READY', ARM_NOT_READY, 'the disposal stop is the pickup stop: no independent stop-2 evidence', stamp, block_base=True)
        if not after(observation, park) or observation.get('stop_location_id') != park['stop_location_id'] or observation.get('disposal_area_id') != park.get('disposal_area_id'):
            return self._end('NOT_READY', ARM_NOT_READY, 'no new observation of the disposal area after the stop (an earlier or pickup observation is not reused)', stamp, block_base=True)
        if observation.get('camera_fresh') is not True or observation.get('target_area') != 'OBSERVED':
            return self._end('NOT_READY', ARM_NOT_READY, f"disposal area not observed now (camera_fresh={observation.get('camera_fresh')}, target_area={observation.get('target_area')})", stamp, block_base=True)
        hold = self.a.hold_readback(self.side)
        if hold.get('readback_ok') is not True or hold.get('gripper_hold') != 'CONFIRMED':
            return self._end('FAULT', ARM_FAULT, 'the block is not confirmed in the hand at the disposal stop', stamp, block_base=True, holding_state=hold.get('gripper_hold', UNKNOWN))
        if not self.still_allowed():
            return self._end('NOT_READY', ARM_NOT_READY, 'base or STOP changed after the check: no motion', stamp, block_base=True)
        self.state = 'PLACE_AT_STOP'
        r = self.a.place(self.side, self.still_allowed, observation)
        bad = self._fault_or_unknown(r, 'place', stamp)
        if bad:
            return bad
        self.state = 'RELEASE_CHECK'
        if r.get('support_and_space') != 'CONFIRMED' or not r.get('approached'):
            return self._end('RECOVERY_REQUIRED', ARM_RECOVERY_REQUIRED, 'support / free space at the disposal area not confirmed: the gripper was not to open', stamp, block_base=True, holding_state=UNKNOWN)
        if r.get('separated') != 'OBSERVED' or r.get('in_target') != 'OBSERVED':
            return self._end('RECOVERY_REQUIRED', ARM_RECOVERY_REQUIRED,
                             f"release not confirmed: separated={r.get('separated')}, in_target={r.get('in_target')} (an open command/ACK alone is not a disposal)", stamp, block_base=True)
        ev = r.get('evidence') or {}
        end = self.a.safe_end(self.side)
        self.state = 'ARM_SAFE_END'
        return [self._send(ARM_DISPOSAL_CONFIRMED, 'ARM_SAFE_END', ev.get('ref', 'release evidence'), stamp, evidence=ev, safe_end=end,
                           separated='OBSERVED', in_target='OBSERVED',
                           note='the upper body part only; the mission manager declares the mission complete with the base team\'s stop-2 evidence')]

    def summary(self):
        return dict(task_id=self.task_id, mode=self.mode, state=self.state, side=self.side, sent=[m['kind'] for m in self.sent],
                    mission_completed_by_upper_body=False, log=self.log, base_rejected=len(self.base.rejected))


if __name__ == '__main__':
    print(json.dumps(dict(schema=SCHEMA, required=REQUIRED, inbound=INBOUND, outbound=OUTBOUND, inbound_evidence=INBOUND_EVIDENCE, arm_checks=ARM_CHECKS, states=STATES), indent=1))
