"""Initial range entry through existing stages/executors. Fake bus and collision answers only."""
import json
from pathlib import Path
import pytest

import sweepick_move_a as move
import sweepick_pick01 as pick
from test_a16_close_binding import ACTUAL, certificate
from test_sweepick_grip import Jaws
from test_sweepick_move_a import Clock, IN

OPENINGS = (3344, 3431, 3497, 3501)
# Measured startup Goal/Present residuals, 20261015_112218_f0d866; no force model.
ENTRY_TIMES = (0., .02, .04, .06, .08, .10, .12, .14, .16, .18, .20, .22, .24, .28, .32)
ENTRY_LEADS = (70, 70, 68, 61, 49, 38, 31, 27, 24, 22, 20, 19, 18, 16, 8)


class EntryBus(Jaws):
    """ACK/readback are immediate; only physical range-entry following can lag."""
    def __init__(self, opening, powered, clock, delay=0., block=None, transient=True):
        super().__init__(dict(ACTUAL, gripper=opening))
        self.reg['Torque_Enable'] = {j: int(j != 'gripper' or powered) for j in move.rc.JOINTS}
        self.reg['Goal_Position'] = dict(self.reg['Present_Position'])
        self.clock, self.delay, self.block, self.transient = clock, delay, block, transient
        self.opening, self.first_goal_at, self.entry_goal = opening, None, None
        self.trace, self.phase = [], 'CLOSE'

    def write(self, reg, joint, value, normalize=True):
        if joint != 'gripper':
            super().write(reg, joint, value, normalize)
            if reg == 'Goal_Position':
                self.reg['Torque_Enable'][joint] = 1
            return
        assert normalize is False and reg in ('Goal_Position', 'Torque_Enable')
        self.writes.append((reg, joint, value))
        self.reg[reg][joint] = value
        if reg == 'Goal_Position':
            if self.first_goal_at is None:
                self.first_goal_at, self.entry_goal = self.clock.now(), value
            self.reg['Torque_Enable'][joint] = 1
        self.trace.append({'kind': 'write', 'stage': self.phase, 't': self.clock.now(),
                           'register': reg, 'value': value, 'ack': 'ACK'})

    def sync_read(self, reg, normalize=True):
        if reg == 'Present_Position' and self.first_goal_at is not None:
            t = self.clock.now() - self.first_goal_at
            extra = max(0., self.opening - self.entry_goal - 8)
            if extra and self.transient:
                lead = float(move.np.interp(max(0., t - self.delay), ENTRY_TIMES, ENTRY_LEADS))
                extra *= max(0., (lead - 8) / (ENTRY_LEADS[0] - 8))
            else:
                extra = 0.
            p, goal = self.reg[reg]['gripper'], self.reg['Goal_Position']['gripper']
            candidate = goal + 8 + extra
            if t >= self.delay or self.opening == self.entry_goal:
                present = min(p, round(candidate)) if goal < p else max(p, round(goal - 8))
                if self.block is not None:
                    present = max(self.block, present)
                self.reg[reg]['gripper'] = present
                self.reg['Moving']['gripper'] = int(present != p)
            else:
                self.reg['Moving']['gripper'] = 1
        result = super().sync_read(reg, normalize)
        if reg in ('Present_Position', 'Goal_Position', 'Torque_Enable', 'Status', 'Moving'):
            self.trace.append({'kind': 'read', 'stage': self.phase, 't': self.clock.now(),
                               'register': reg, 'value': result['gripper']})
        return result


def mapping():
    return move.rc.load_mapping(IN / 'CANDIDATE_Q12_CONFIG.json',
        Path.home() / '.config/dapier/lerobot-calibration', IN / 'ASSEMBLED_ZERO_AND_PGRIPPER.json')


def close_case(directory, opening, powered=False, delay=0., block=None, transient=True, setup=None, **kw):
    c = Clock()
    bus = EntryBus(opening, powered, c, delay=delay, block=block, transient=transient)
    if setup is not None:
        setup(bus)
    code, log = move.grip('left', directory, torque_joints=list(move.rc.JOINTS), execute=True,
        hold_s=.3, open_bus=lambda port: bus, owner=lambda port: '', mapping=mapping(),
        sleep=c.sleep, clock=c.now, level='model', cert=certificate(dict(ACTUAL, gripper=opening)), **kw)
    return bus, code, log


@pytest.mark.parametrize('opening', OPENINGS)
@pytest.mark.parametrize('powered', (False, True))
@pytest.mark.parametrize('delay', (0., .12, .32, .4))
def test_initial_correction_cannot_pollute_free_stroke_or_contact(tmp_path, opening, powered, delay):
    bus, code, log = close_case(tmp_path, opening, powered, delay, block=3075)
    assert code == 0 and log['result'] == 'GRIP_CONTACT_HOLDING'
    assert log['certificate_binding']['matched']
    goals = [v for reg, joint, v in bus.writes if reg == 'Goal_Position' and joint == 'gripper']
    assert goals[0] == min(opening, 3431)
    assert all(2001 <= value <= 3431 for value in goals)
    # Same pre-existing bound as test_sweepick_grip: floor + preload + fake free lag.
    assert log['grip']['contact_onset']['residual_ticks'] <= 17 + 24 + 8
    assert all(row['present']['gripper'] <= 3431 + 17 for row in log['rows']
               if row.get('phase') == 'grip')


@pytest.mark.parametrize('opening', OPENINGS)
@pytest.mark.parametrize('powered', (False, True))
@pytest.mark.parametrize('delay', (0., .12, .32, .4))
def test_empty_jaws_do_not_report_contact_during_range_entry(tmp_path, opening, powered, delay):
    bus, code, log = close_case(tmp_path, opening, powered, delay)
    assert code == 9 and log['result'] == 'GRIP_NO_CONTACT_HOLDING'
    assert log['grip']['contact_onset'] is None
    assert not any(row['jaw_state'] == 'CONTACT_CANDIDATE' for row in log['rows'])


@pytest.mark.parametrize('opening', OPENINGS)
def test_existing_stage_pipeline_keeps_gripper_passive_until_close(tmp_path, monkeypatch, opening):
    c = Clock()
    bus = EntryBus(opening, False, c, block=3075)
    bus.reg['Torque_Enable'] = dict.fromkeys(move.rc.JOINTS, 0)
    mp = mapping()
    monkeypatch.setattr(pick, 'mapping', lambda: mp)
    monkeypatch.setattr(pick, 'state_object', lambda out, state: ([0, 0, .02], 0.))
    monkeypatch.setattr(pick, 'read_now', lambda kw=None: {'ticks': dict(bus.reg['Present_Position'])})
    checks = []
    def clear(doc, *args, **kwargs):
        checks.append(doc)
        return dict(identity=doc.get('identity'), nominal_verdict='CLEAR', nominal_steps_checked=doc['trajectory']['steps'],
                    scene='FAKE_COLLISION_ANSWER', allowed=['FAKE only'])
    monkeypatch.setattr(pick, 'object_check', clear)
    def certify(path, *args):
        return clear(json.loads(Path(path).read_text()))
    state = dict(ready=True, stages={}, order=[], evidence=[], home_ticks=dict(bus.reg['Present_Position']),
                 right_ticks=dict(ACTUAL), plan={'ticks': {}}, sources={})
    for index, name in enumerate(('pregrasp', 'descend_a', 'descend'), 1):
        state['plan']['ticks'][name] = {j: ACTUAL[j] - index * 3 for j in pick.ARM}
    tmp_path.mkdir(exist_ok=True)
    (tmp_path / 'scene_block0.mjb').write_bytes(b'FAKE_CACHE_NOT_A_PHYSICS_TEST')
    def save():
        (tmp_path / 'STATE.json').write_text(json.dumps(state))
    save()
    kw = dict(open_bus=lambda port: bus, owner=lambda port: '', mapping=mp,
              sleep=c.sleep, clock=c.now, other_arm_read=lambda: dict(ACTUAL))
    for name in ('pregrasp', 'align', 'descend_a', 'descend', 'close'):
        state = json.loads((tmp_path / 'STATE.json').read_text())
        if name == 'align':
            state['pending_align'] = True
            state['plan']['ticks']['pregrasp']['wrist_roll'] -= 3
        if name in pick.GATE:
            state['evidence'].append({'what': pick.GATE[name], 'ok': True, 'source': 'FAKE_OBSERVATION'})
        save()
        bus.phase = name
        stage_kw = {k: v for k, v in kw.items() if name != 'close' or k != 'other_arm_read'}
        record = pick.stage(tmp_path, name, execute=True, cert=certify, run_kw=stage_kw)
        assert record['controller_result'] == ('GRIP_CONTACT_HOLDING' if name == 'close' else 'HOLDING'), record
        if name != 'close':
            assert bus.reg['Present_Position']['gripper'] == opening
            assert bus.reg['Torque_Enable']['gripper'] == 0
            assert not any(j == 'gripper' for _, j, _ in bus.writes)
    assert json.loads((tmp_path / 'STATE.json').read_text())['order'] == ['pregrasp', 'align', 'descend_a', 'descend', 'close']
    sweep = [d for d in checks if d['trajectory']['moving'] == ['gripper']][-1]
    assert sweep['trajectory']['start']['gripper'] == opening
    assert sweep['trajectory']['goals']['gripper'][0] == opening


@pytest.mark.parametrize('opening', OPENINGS)
def test_powered_pregrasp_retains_existing_ownership_refusal(tmp_path, opening):
    c = Clock()
    bus = EntryBus(opening, True, c)
    code, log = move.run('out', 'left', tmp_path / 'move', target={'wrist_roll': ACTUAL['wrist_roll'] - 3},
        torque_joints=pick.ARM, execute=True, open_bus=lambda port: bus, owner=lambda port: '',
        mapping=mapping(), sleep=c.sleep, clock=c.now, range_entry=True)
    assert code == 2 and log['refused']['gate'] == 'torque_state'
    assert bus.writes == []


@pytest.mark.parametrize('powered', (False, True))
def test_unconfirmed_initial_move_sends_no_closing_goals_or_contact_claim(tmp_path, powered):
    bus, code, log = close_case(tmp_path, 3501, powered, delay=10)
    assert code == 9 and log['result'] == 'GRIP_UNAVAILABLE_HOLDING'
    assert log['outcome']['command_completed'] is False
    assert log['outcome']['contact_candidate'] is None
    assert log['range_entry']['status'] == 'UNCONFIRMED_HOLDING'
    assert [v for reg, joint, v in bus.writes if reg == 'Goal_Position'] == [3431]
    assert not any(reg == 'Torque_Enable' and value == 0 for reg, _, value in bus.writes)


@pytest.mark.parametrize('recovers', (False, True))
def test_initial_goal_overshoot_never_contaminates_contact_or_waits_forever(tmp_path, recovers):
    def setup(bus):
        read = bus.sync_read
        def overshoot(reg, normalize=True):
            values = read(reg, normalize)
            if reg == 'Present_Position' and bus.first_goal_at is not None and (not recovers or bus.clock.now()-bus.first_goal_at < .1):
                bus.reg[reg]['gripper'] = min(bus.reg[reg]['gripper'], 3410)
                values['gripper'] = bus.reg[reg]['gripper']
            return values
        bus.sync_read = overshoot
    bus, code, log = close_case(tmp_path, 3501, setup=setup)
    if recovers:
        assert log['range_entry']['status'] == 'ACTUAL_CONFIRMED'
        assert log['grip']['contact_onset'] is None
        assert log['result'] == 'GRIP_NO_CONTACT_HOLDING'
    else:
        assert log['range_entry']['status'] == 'UNCONFIRMED_HOLDING'
        assert log['result'] == 'GRIP_UNAVAILABLE_HOLDING'
    assert code == 9


@pytest.mark.parametrize('mode', ('ack_lost', 'read_lost', 'status_error'))
def test_initial_goal_application_failure_never_continues_close(tmp_path, mode):
    def setup(bus):
        write, read = bus.write, bus.sync_read
        def injected_write(reg, joint, value, normalize=True):
            write(reg, joint, value, normalize)
            if reg == 'Goal_Position' and mode == 'ack_lost':
                bus.dead = True
                raise ConnectionError('FAKE applied seed; ACK lost')
        def injected_read(reg, normalize=True):
            if bus.first_goal_at is not None and mode == 'read_lost':
                bus.dead = True
            if bus.first_goal_at is not None and mode == 'status_error':
                bus.reg['Status']['gripper'] = 32
            return read(reg, normalize)
        bus.write, bus.sync_read = injected_write, injected_read
    bus, code, log = close_case(tmp_path, 3501, delay=.4, setup=setup)
    assert code != 0 and not log['outcome']['command_completed']
    goals = [v for reg, joint, v in bus.writes if reg == 'Goal_Position']
    assert len(goals) == 1 and goals[0] == 3431
    assert log['outcome']['contact_candidate'] is None


@pytest.mark.parametrize('opening', (3497, 3501))
def test_stop_during_initial_move_never_sends_out_of_range_actual_as_goal(tmp_path, opening):
    seen = []
    def setup(bus):
        seen.append(bus)
    bus, code, log = close_case(tmp_path, opening, delay=.4, setup=setup,
        stop_requested=lambda: bool(seen and seen[0].first_goal_at is not None and
                                    seen[0].clock.now()-seen[0].first_goal_at >= .04))
    assert code == 16 and log['stopped_by']['gate'] == 'operator_stop'
    assert not log['outcome']['command_completed']
    assert log['recovery']['motion_stop_confirmed'] == 'UNKNOWN'
    assert [v for reg, joint, v in bus.writes if reg == 'Goal_Position'] == [3431]
    assert not any(reg == 'Torque_Enable' and v == 0 for reg, _, v in bus.writes)


def test_one_tick_jitter_does_not_extend_initial_wait_forever(tmp_path):
    seen = []
    def setup(bus):
        seen.append(bus)
        read, count = bus.sync_read, [0]
        def jitter(reg, normalize=True):
            values = read(reg, normalize)
            if reg == 'Present_Position' and bus.first_goal_at is not None:
                count[0] += 1
                values['gripper'] = bus.reg[reg]['gripper'] = 3501 - count[0] % 2
            return values
        bus.sync_read = jitter
    bus, code, log = close_case(tmp_path, 3501, setup=setup,
        stop_requested=lambda: bool(seen and seen[0].clock.now() > 1010))
    assert code == 9 and log['result'] == 'GRIP_UNAVAILABLE_HOLDING'
    assert [v for reg, joint, v in bus.writes if reg == 'Goal_Position'] == [3431]


@pytest.mark.parametrize('change,gate', [('arm', 'certificate_binding'), ('torque', 'torque_state'), ('goal', 'bus_lost')])
def test_range_entry_does_not_weaken_binding_or_state_contract(tmp_path, change, gate):
    def setup(bus):
        read = bus.sync_read
        def changed(reg, normalize=True):
            if bus.first_goal_at is not None:
                if change == 'arm':
                    bus.reg['Present_Position']['wrist_flex'] = ACTUAL['wrist_flex'] - 2
                if change == 'torque':
                    bus.reg['Torque_Enable']['wrist_flex'] = 0
                if change == 'goal':
                    bus.reg['Goal_Position']['gripper'] = 3400
            return read(reg, normalize)
        bus.sync_read = changed
    bus, code, log = close_case(tmp_path, 3501, setup=setup)
    assert code != 0 and log['stopped_by']['gate'] == gate
    assert [v for reg, joint, v in bus.writes if reg == 'Goal_Position'] == [3431]
    assert not log['outcome']['command_completed']


def test_initial_negative_residual_rebound_is_not_a_contact(tmp_path):
    def setup(bus):
        read = bus.sync_read
        def rebound(reg, normalize=True):
            values = read(reg, normalize)
            if reg == 'Present_Position' and bus.first_goal_at is not None and bus.clock.now()-bus.first_goal_at < .3:
                bus.reg[reg]['gripper'] = values['gripper'] = 3415
            return values
        bus.sync_read = rebound
    bus, code, log = close_case(tmp_path, 3501, setup=setup)
    assert code == 9 and log['result'] == 'GRIP_NO_CONTACT_HOLDING'
    assert log['grip']['contact_onset'] is None
    assert not any(row['jaw_state'] == 'CONTACT_CANDIDATE' for row in log['rows'])
    assert log['range_entry']['fresh_hold_reseed']['goal_tick'] == 3415
    assert all(2001 <= value <= 3431 for reg, joint, value in bus.writes if reg == 'Goal_Position')


@pytest.mark.parametrize('opening', OPENINGS)
@pytest.mark.parametrize('powered', (False, True))
@pytest.mark.parametrize('delay', (.12, .32, .4))
def test_first_closing_motion_delay_is_not_contact_even_for_in_range_seed(tmp_path, opening, powered, delay):
    def setup(bus):
        read = bus.sync_read
        def delayed(reg, normalize=True):
            values = read(reg, normalize)
            if reg == 'Present_Position' and bus.first_goal_at is not None and bus.clock.now()-bus.first_goal_at < delay:
                bus.reg[reg]['gripper'] = values['gripper'] = opening
                bus.reg['Moving']['gripper'] = 1
            return values
        bus.sync_read = delayed
    bus, code, log = close_case(tmp_path, opening, powered, setup=setup)
    assert code == 9 and log['result'] == 'GRIP_NO_CONTACT_HOLDING'
    assert log['grip']['contact_onset'] is None
    assert not any(row['jaw_state'] == 'CONTACT_CANDIDATE' for row in log['rows'])


def test_already_at_close_end_uses_existing_empty_end_stop_result(tmp_path):
    bus, code, log = close_case(tmp_path, 2018)
    assert code == 9 and log['result'] == 'GRIP_NO_CONTACT_HOLDING'
    assert log['grip']['contact_onset'] is None
