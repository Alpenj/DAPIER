"""A16 recorded CLOSE pose change, using the existing fake bus only."""
import json
import pytest
import sweepick_move_a as mv
from test_sweepick_grip import Jaws, go

ACTUAL = dict(shoulder_pan=2352.0, shoulder_lift=2304.0, elbow_flex=2123.0,
              wrist_flex=2835.0, wrist_roll=2376.0, gripper=3501.0)


def certificate(arm, verdict="CLEAR"):
    return dict(nominal_verdict=verdict, arm_ticks={n: arm[n] for n in mv.rc.JOINTS[:5]},
                sweep=[arm['gripper'], 2018.0])


def fake(gripper=3501.0):
    bus = Jaws(dict(ACTUAL, gripper=gripper), block=3075)
    bus.reg['Torque_Enable'] = {n: int(n != 'gripper') for n in mv.rc.JOINTS}
    return bus


@pytest.mark.parametrize('opening', [3344.0, 3501.0])
def test_old_certificate_refuses_two_tick_arm_change_without_any_write(tmp_path, opening):
    bus = fake(opening)
    cert = certificate(dict(ACTUAL, gripper=opening, wrist_flex=2837.0))
    code, log = go(tmp_path, bus, level='model', cert=cert)
    assert code == 2 and log['refused']['gate'] == 'certificate_binding'
    assert not bus.writes and log['writes'] == dict(Goal_Position=0, Torque_Enable=0)
    assert log['start']['ticks']['gripper'] == opening


def test_changed_start_is_rechecked_once_and_uses_fresh_clear_certificate(tmp_path):
    bus = fake()
    seen = []
    def check(arm):
        assert not bus.writes
        seen.append(dict(arm))
        return certificate(arm)
    code, log = go(tmp_path, bus, level='model', cert=certificate(dict(ACTUAL, wrist_flex=2837.0)), start_recheck=check)
    assert code == 0 and log['result'] == 'GRIP_CONTACT_HOLDING'
    assert seen == [ACTUAL]
    assert log['certificate_binding']['rechecked'] and log['certificate_binding']['matched']
    assert log['goal_seed_normalization']['command_tick'] == 3431.0
    assert log['write_attempts'] == log['writes']
    assert bus.reg['Torque_Enable']['gripper'] == 1
    assert not any(n != 'gripper' for _, n, _ in bus.writes)


@pytest.mark.parametrize('mode', ['contact', 'unknown', 'moves_again', 'fault', 'torque_change', 'bus_lost', 'exception', 'malformed', 'interrupted'])
def test_recheck_never_relaxes_collision_binding_or_state_failures(tmp_path, mode):
    bus = fake()
    calls = []
    def check(arm):
        calls.append(1)
        assert not bus.writes
        if mode == 'moves_again': bus.reg['Present_Position']['wrist_flex'] -= 2
        if mode == 'fault': bus.reg['Status']['wrist_flex'] = 32
        if mode == 'torque_change': bus.reg['Torque_Enable']['wrist_flex'] = 0
        if mode == 'bus_lost': bus.dead = True
        if mode == 'exception': raise RuntimeError('checker failed')
        if mode == 'interrupted': raise KeyboardInterrupt()
        if mode == 'malformed': return ['invalid certificate']
        return certificate(arm, 'CONTACT' if mode == 'contact' else 'UNKNOWN' if mode == 'unknown' else 'CLEAR')
    code, log = go(tmp_path, bus, level='model', cert=certificate(dict(ACTUAL, wrist_flex=2837.0)), start_recheck=check)
    assert code == 2 and log['result'] == 'REFUSED'
    assert calls == [1] and not bus.writes
    assert log['writes'] == dict(Goal_Position=0, Torque_Enable=0)
    if mode == 'bus_lost':
        assert log['refused']['gate'] == 'bus_lost'
        assert log['end']['torque'] == log['outcome']['torque_released'] == 'UNKNOWN'
        assert bus.disconnected is False
    if mode == 'interrupted':
        assert log['refused']['gate'] == 'operator_stop' and bus.disconnected is False


def test_unknown_goal_ack_is_an_attempt_not_an_acknowledged_write(tmp_path):
    bus = fake()
    original = bus.write
    def lost(reg, joint, value, normalize=False):
        if reg == 'Goal_Position':
            original(reg, joint, value, normalize=normalize)
            bus.dead = True
            raise ConnectionError('applied, ACK lost')
        original(reg, joint, value, normalize=normalize)
    bus.write = lost
    code, log = go(tmp_path, bus, level='model', cert=certificate(ACTUAL))
    assert code == 15 and log['result'] == 'BUS_LOST'
    assert log['write_attempts'] == dict(Goal_Position=1, Torque_Enable=0)
    assert log['writes'] == dict(Goal_Position=0, Torque_Enable=0)
    assert log['last_write']['acknowledgement'] == 'UNKNOWN'
    assert len(bus.writes) == 1 and log['end']['torque'] == 'UNKNOWN'


def test_already_bound_certificate_does_not_add_a_check(tmp_path):
    bus = fake()
    code, log = go(tmp_path, bus, level='model', cert=certificate(ACTUAL),
                   start_recheck=lambda _: pytest.fail('no mismatch, no recheck'))
    assert code == 0 and not log['certificate_binding']['rechecked']


def test_pick_close_wires_its_same_scene_sweep_checker_to_grip(tmp_path, monkeypatch):
    import sweepick_pick01 as pick
    from test_sweepick_move_a import Clock
    bus, clock, checked = fake(), Clock(), []
    state = dict(ready=True, sources={}, stages={}, order=[], right_ticks=ACTUAL,
                 evidence=[dict(what='ready_to_close', ok=True)])
    (tmp_path / 'STATE.json').write_text(json.dumps(state))
    monkeypatch.setattr(pick, 'allowed', lambda *_: (True, 'fixture grasp pose held'))
    monkeypatch.setattr(pick, 'stage_request', lambda *_: dict(mode='grip', torque_joints=list(pick.ALL), scene='object: fixture'))
    monkeypatch.setattr(pick, 'state_object', lambda *_: None)
    monkeypatch.setattr(pick, 'read_now', lambda *_: dict(ticks=dict(ACTUAL, wrist_flex=2837.0)))
    def same_checker(doc, obj, **kw):
        checked.append(doc['trajectory']['start'])
        return dict(nominal_verdict='CLEAR', nominal_steps_checked=doc['trajectory']['steps'],
                    scene='same fixture block/desk', allowed=['left pad - block', 'block - desk'])
    monkeypatch.setattr(pick, 'object_check', same_checker)
    record = pick.stage(tmp_path, 'close', execute=True, grip=mv.grip,
                        run_kw=dict(open_bus=lambda _: bus, owner=lambda _: '', sleep=clock.sleep, clock=clock.now))
    assert record['controller_result'] == 'GRIP_CONTACT_HOLDING'
    assert [row['wrist_flex'] for row in checked] == [2837.0, 2835.0]
    assert json.loads((tmp_path / 'close/sweep.CHECK.json').read_text())['arm_ticks']['wrist_flex'] == 2837.0
    assert json.loads((tmp_path / 'close/sweep_recheck.CHECK.json').read_text())['arm_ticks']['wrist_flex'] == 2835.0
