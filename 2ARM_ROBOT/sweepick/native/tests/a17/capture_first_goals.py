"""Reproducible A17 fake-bus evidence; never opens a device."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent
REV = HERE.parents[1] / 'revision'
sys.path.insert(0, str(REV))
import sweepick_move_a as move
import sweepick_pick01 as pick
from test_a16_close_binding import fake, certificate, ACTUAL
from test_sweepick_grip import go


def capture(opening):
    bus, trace, checks = fake(opening), [], []
    read, write = bus.sync_read, bus.write
    def traced_read(register, **kw):
        values = read(register, **kw)
        if register in ('Goal_Position', 'Present_Position', 'Torque_Enable', 'Status'):
            trace.append({'kind': 'read', 'register': register, 'values': dict(values), 'clock': 'FAKE'})
        return values
    def traced_write(register, joint, value, **kw):
        item = {'kind': 'write', 'register': register, 'joint': joint, 'value': value, 'ack': 'UNKNOWN', 'clock': 'FAKE'}
        trace.append(item)
        write(register, joint, value, **kw)
        item['ack'] = 'ACK'
    bus.sync_read, bus.write = traced_read, traced_write
    def check(actual):
        started = time.perf_counter()
        doc = pick.sweep_doc(actual, actual['gripper'], 2018.)
        samples = doc['trajectory']['goals']['gripper']
        checks.append({'actual_six_axes': dict(actual), 'write_count_before_check': len(bus.writes),
            'sweep_from_tick': samples[0], 'sweep_to_tick': samples[-1], 'samples': len(samples),
            'range_entry_3501_to_3431_in_whole_sweep': opening != 3501 or all(t in samples for t in range(3431, 3502)),
            'command_start_present': 'command_start' in doc['trajectory'],
            'result': 'FAKE_CLEAR', 'duration_s': time.perf_counter() - started})
        return certificate(actual)
    with tempfile.TemporaryDirectory(prefix='sweepick_a17_fake_') as directory:
        code, log = go(Path(directory), bus, level='model',
            cert=certificate(dict(ACTUAL, gripper=opening, wrist_flex=2837.)), start_recheck=check)
    goals = [r for r in trace if r['kind'] == 'write' and r['register'] == 'Goal_Position']
    first_index = next(i for i, r in enumerate(trace) if r['kind'] == 'write' and r['register'] == 'Goal_Position')
    first_read = next(r for r in trace[first_index+1:] if r['kind'] == 'read' and r['register'] == 'Goal_Position')
    assert len(checks) == 1 and all(2001 <= r['value'] <= 3431 for r in goals)
    return {'input': {'gripper_actual': opening, 'wrist_certificate': 2837., 'wrist_actual': 2835.},
        'initial_actual': log['start']['ticks'], 'binding': log['certificate_binding'], 'rechecks': checks,
        'first_goal_write': goals[0], 'first_goal_register_readback': first_read['values']['gripper'],
        'all_goal_writes_with_ack': goals, 'all_goal_writes_in_configured_range': True,
        'write_attempts': log['write_attempts'], 'acknowledged_writes': log['writes'],
        'normalization': log.get('goal_seed_normalization'), 'result': log['result'], 'exit_code': code,
        'fake_end': log['end'], 'hardware_result': 'NOT_TESTED',
        'meaning': 'fake contact resistance; not real grasp proof; no physical collision simulation in this capture'}


if __name__ == '__main__':
    payload = {'schema': 'sweepick.a17-first-goal.v1', 'device_access': False,
        'python': sys.executable, 'configured_range': [2001,3431],
        'interpretation': '3501 is a possible manual opening; 3431 is software calibration upper bound, not a proven physical opening limit',
        'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (Path(move.__file__), Path(pick.__file__))},
        'CASE_A': capture(3344.), 'CASE_B': capture(3501.)}
    (HERE / 'A17_FIRST_GOAL_FAKE_BUS.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2)+'\n')
    print({key: {'first_goal': payload[key]['first_goal_write']['value'],
        'first_readback': payload[key]['first_goal_register_readback'],
        'goal_acks': payload[key]['acknowledged_writes']['Goal_Position']} for key in ('CASE_A','CASE_B')})
