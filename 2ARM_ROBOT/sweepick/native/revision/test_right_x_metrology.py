"""RIGHT_X_FINITE_METROLOGY on a fake bus/executor/cameras: T/F/V endpoints, refusals and stops. No device access."""
import copy
import json
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import right_x_metrology as rx
import sweepick_full_chain as fc
import sweepick_pick01 as p1

# measured ticks of a saved REAL RECEIVE_CONTACT observation (after RIGHT_APPROACH), used as the fake bus state
LEFT = {'shoulder_pan': 2566.0, 'shoulder_lift': 1043.0, 'elbow_flex': 3106.0, 'wrist_flex': 2106.0, 'wrist_roll': 2320.0, 'gripper': 3019.0}
RIGHT = {'shoulder_pan': 1621.0, 'shoulder_lift': 1066.0, 'elbow_flex': 2741.0, 'wrist_flex': 2692.0, 'wrist_roll': 1083.0, 'gripper': 3010.0}
CFG = dict(t_pose='AFTER_RIGHT_PREGRASP', x_basis_tool=[1.0, 0.0, 0.0], x_basis_source='A model tool +X of right_cube_grasp; physical direction unconfirmed',
           fit_displacement_m=-0.004, verify_displacement_m=-0.004, verify_from='F')


def approval(out, **over):
    a = dict(scope=rx.SCOPE, execution_scope=rx.SCOPE, session_out=str(out), declared_motor_writes=list(rx.DECLARED_WRITES),
             excluded=list(rx.EXCLUDED), right_gripper_commands_permitted=False, insertion_permitted=False, close_permitted=False,
             left_release_permitted=False, right_x_metrology=dict(CFG))
    a.update(over)
    return a


def test_config_contract_has_no_magnitude_limit_and_refuses_malformed_input():
    assert rx.config_errors(CFG) == []
    assert rx.config_errors(dict(CFG, fit_displacement_m=0.5)) == []                     # size is the operator's decision
    for bad, words in ((dict(CFG, x_basis_tool=[1, 1, 0]), 'unit vector'), (dict(CFG, fit_displacement_m=0), 'non-zero'),
                       (dict(CFG, verify_displacement_m=True), 'non-zero'), (dict(CFG, verify_from='T'), 'verify_from'),
                       (dict(CFG, t_pose='HOME'), 't_pose'), (dict(CFG, x_basis_source=' '), 'x_basis_source'),
                       ({k: v for k, v in CFG.items() if k != 'x_basis_source'}, 'fields'), (dict(CFG, speed=1), 'fields')):
        assert any(words in e for e in rx.config_errors(bad)), (bad, rx.config_errors(bad))
    D = np.asarray(rx.x_relative_tool([0.0, 0.6, 0.8], -0.01))
    np.testing.assert_allclose(D[:3, :3], np.eye(3)); np.testing.assert_allclose(D[:3, 3], [0, -0.006, -0.008])
    assert fc.observation_candidate_errors(rx.move_candidate('F', CFG)) == []


def test_approval_contract(tmp_path):
    assert rx.approval_errors(approval(tmp_path), tmp_path) == []
    for over, words in ((dict(declared_motor_writes=list(rx.DECLARED_WRITES)[:-1]), 'declared_motor_writes'),
                        (dict(scope='RIGHT_RECEIVE_NONCONTACT_OBSERVATION'), 'scope'), (dict(insertion_permitted=True), 'insertion'),
                        (dict(observation_view_preapproval={'approved': True}), 'pre-approval'), (dict(session_out='/elsewhere'), 'session_out'),
                        (dict(right_x_metrology=None), 'object')):
        assert any(words in e for e in rx.approval_errors(approval(tmp_path, **over), tmp_path)), over


def bus(ticks_left=LEFT, ticks_right=RIGHT):
    sides = ('left', 'right')
    return dict(ticks=dict(left=dict(ticks_left), right=dict(ticks_right)), goal=dict(left=dict(ticks_left), right=dict(ticks_right)),
                torque={s: dict.fromkeys(fc.ALL, 1) for s in sides}, status={s: dict.fromkeys(fc.ALL, 0) for s in sides},
                moving={s: dict.fromkeys(fc.ALL, 0) for s in sides}, velocity={s: dict.fromkeys(fc.ALL, 0) for s in sides},
                load={s: dict.fromkeys(fc.ALL, 0) for s in sides}, bus_seq=0, t_bus_monotonic=0.0, cycle_start=0.0)


def owner_on_fake_bus(tmp_path, cfg=CFG, *, plan_refuse=(), decline=(), missing=(), unsupported=(), segment_fault=(), stop_after=None):
    state = bus(); calls = dict(plan=[], authorize=[], segment=[], observation=[], reads=0)
    o = fc.Owner.__new__(fc.Owner)
    o.scope, o.out, o.notes, o.stopped, o.p1, o.mp, o.record_kind = 'contact', tmp_path, [], None, p1, p1.mapping(), 'FAKE_BUS_REGRESSION'
    o.contact_plan = dict(observation_only=True, right_x_metrology=cfg)
    o.last_images, clock = {}, [0.0]
    o.clock = lambda: clock[0]
    o.note = lambda kind, **values: o.notes.append(dict(kind=kind, **values))
    o.track_held = lambda raw: None

    def read():
        calls['reads'] += 1; clock[0] += 0.01
        row = copy.deepcopy(state); row.update(bus_seq=calls['reads'], t_bus_monotonic=clock[0]); return row
    o._read = read

    def plan(candidate, raw, prefix):
        role = candidate['id'][-1]; calls['plan'].append((role, raw['bus_seq'], raw['ticks']['right']['shoulder_lift']))
        if role in plan_refuse:
            return dict(ready=False, forward=[], why=plan_refuse[role], failure='OBSERVATION_PLAN_UNAVAILABLE')
        s = candidate['commanded_tool_displacement_m']
        target = {n: float(raw['ticks']['right'][n]) for n in fc.ARM}
        target.update(shoulder_lift=target['shoulder_lift'] + round(3000 * s), elbow_flex=target['elbow_flex'] + round(2800 * s), wrist_flex=target['wrist_flex'] - round(5900 * s))
        return dict(ready=True, forward=[dict(side='right', start_ticks={n: raw['ticks']['right'][n] for n in fc.ARM}, target_ticks=target,
                                              certificate=dict(nominal_verdict='CLEAR'), native_validation=dict(executable=True, hard=[], provisional=[]), ik={})],
                    normal_return=dict(ready=True))
    o.plan_observation_candidate = plan

    def authorize(proposed, raw, candidate):
        role = candidate['id'][-1]; calls['authorize'].append(role)
        return role not in decline
    o.authorize_right_x_move = authorize

    def segment(name, side, *, target=None, purpose=None, **kw):
        calls['segment'].append((name, side, purpose, dict(target)))
        assert purpose == 'OBSERVATION_VIEW' and side == 'right' and 'gripper' not in target
        if name[-1].upper() in segment_fault:
            raise RuntimeError(f'{name} (right): executor ended BUS_LOST')
        state['ticks']['right'].update(target); state['goal']['right'].update(target)
        if stop_after == name[-1].upper():
            Path(str(tmp_path) + '.STOP').write_text('operator')
        return dict(result='HOLDING', writes={'Goal_Position': 5, 'Torque_Enable': 0}, folder=str(tmp_path / name),
                    observation_view=dict(actual_settled=True, command_state_resolved=True, nominal_target_reached=True))
    o.segment = segment
    o.recovery_ready = lambda step, observation=None, frames=None: ((False, 'left wrist frame stale') if any(f'right X {r}' in step for r in unsupported) else (True, []))

    def contact_observation(stage, *, after, previous=None):
        calls['observation'].append(stage); n = len(calls['observation'])
        frames = {name: dict(n=n, recv_monotonic_s=clock[0] + 0.001 * k, reader_valid=True, fresh=True) for k, name in enumerate(('top', 'left_wrist', 'right_wrist'))}
        observed = dict(decision_record=str(tmp_path / 'decision_observations' / stage), observation_missing={})
        if stage[-1] in missing:
            observed['observation_missing'] = dict(right_wrist='no new frame after the finite move'); frames['right_wrist'] = dict(reader_valid=False, fresh=False)
        o.last_images = frames
        return observed, read(), frames
    o.contact_observation = contact_observation
    return o, calls, state


def run(o):
    return o.run_right_x_metrology({}, lambda raw, phase, *a: None, [])


def saved(tmp_path):
    return json.loads((tmp_path / 'RIGHT_X_METROLOGY.json').read_text())


def test_normal_path_records_T_F_V_from_fresh_actuals_and_generates_no_correction(tmp_path):
    o, calls, state = owner_on_fake_bus(tmp_path)
    record = run(o)
    assert record['status'] == 'ENDPOINTS_RECORDED' and saved(tmp_path)['status'] == 'ENDPOINTS_RECORDED'
    assert [e['role'] for e in record['endpoints']] == ['T', 'F', 'V'] and calls['observation'] == ['RIGHT_X_T', 'RIGHT_X_F', 'RIGHT_X_V']
    assert calls['authorize'] == ['F', 'V'] and [s[0] for s in calls['segment']] == ['right_x_f', 'right_x_v']
    # each move is planned from the latest read: V starts where F ended
    assert calls['plan'][1][2] == calls['segment'][0][3]['shoulder_lift']
    T, F, V = record['endpoints']
    assert T['actual']['bus_seq'] < F['actual']['bus_seq'] < V['actual']['bus_seq']
    for e in record['endpoints']:
        assert set(e['cameras']) == {'top', 'left_wrist', 'right_wrist'} and e['opening']['right_present_tick'] == RIGHT['gripper']
        assert e['actual']['ticks']['left'] == LEFT and 'not equated' in e['time_basis']
        assert len({c['recv_monotonic_s'] for c in e['cameras'].values()} | {e['actual']['t_bus_monotonic']}) == 4   # own clocks kept
    assert F['command']['commanded_tool_displacement_m'] == CFG['fit_displacement_m']
    assert {'command_fk_from_planning_actual', 'basis'} <= set(F['command']) and 'actual_fk_from_previous' in F and 'actual_fk_from_T' in V
    assert record['jacobian'] == dict(delta_u_px=None, J_uX_px_per_mm=None, REAL_J_VALIDATED=False, validated=False, Correction=None,
                                      estimated_tool_correction=None, reason=record['jacobian']['reason'])
    assert record['correction_command_generated'] is False


@pytest.mark.parametrize('case, expect, segments', [
    (dict(plan_refuse={'F': 'TeacherFailure: no IK solution'}), 'REFUSED_PLAN', []),                    # IK impossible
    (dict(plan_refuse={'F': "view finite path refused: {'step': 3}"}), 'REFUSED_PLAN', []),            # collision check failed
    (dict(decline={'F'}), 'DECLINED', []),
    (dict(plan_refuse={'V': 'view normal reverse refused'}), 'REFUSED_PLAN', ['right_x_f']),          # middle stage failure
    (dict(missing={'F'}), 'OBSERVATION_MISSING', ['right_x_f']),                                       # new observation missing
    (dict(missing={'T'}), 'OBSERVATION_MISSING', []),
])
def test_refusals_end_without_further_motion(tmp_path, case, expect, segments):
    o, calls, _ = owner_on_fake_bus(tmp_path, **case)
    record = run(o)
    assert record['status'] == expect and [s[0] for s in calls['segment']] == segments
    assert saved(tmp_path)['status'] == expect and record['jacobian']['Correction'] is None


@pytest.mark.parametrize('case, words, segments', [
    (dict(unsupported={'F'}), 'left hold/bus is unconfirmed before right X F', []),
    (dict(unsupported={'V'}), 'left hold/bus is unconfirmed before right X V', ['right_x_f']),
    (dict(segment_fault={'F'}), 'BUS_LOST', ['right_x_f']),
    (dict(segment_fault={'V'}), 'BUS_LOST', ['right_x_f', 'right_x_v']),
    (dict(stop_after='F'), 'STOP', ['right_x_f']),
])
def test_stop_fault_and_unconfirmed_left_hold_raise_into_the_existing_stop_contract(tmp_path, case, words, segments):
    o, calls, _ = owner_on_fake_bus(tmp_path, **case)
    with pytest.raises(RuntimeError, match=words):
        run(o)
    assert [s[0] for s in calls['segment']] == segments


def test_authorization_exception_is_not_a_decline(tmp_path):
    o, calls, _ = owner_on_fake_bus(tmp_path)
    o.authorize_right_x_move = lambda *a: (_ for _ in ()).throw(RuntimeError('approval wait lost current support/bus'))
    with pytest.raises(RuntimeError, match='approval wait'):
        run(o)
    assert calls['segment'] == [] and any(n['kind'] == 'right_x_metrology_authorization_failed' for n in o.notes)


def test_missing_authorizer_or_bad_config_never_moves(tmp_path):
    o, calls, _ = owner_on_fake_bus(tmp_path); del o.authorize_right_x_move
    assert run(o)['status'] == 'DECLINED' and calls['segment'] == []
    o, calls, _ = owner_on_fake_bus(tmp_path, cfg=dict(CFG, fit_displacement_m=0))
    assert run(o)['status'] == 'REFUSED_CONFIG' and calls['segment'] == calls['plan'] == [] and calls['reads'] == 0


def test_after_adopted_view_requires_an_executed_fresh_view(tmp_path):
    o, calls, _ = owner_on_fake_bus(tmp_path, cfg=dict(CFG, t_pose='AFTER_ADOPTED_VIEW'))
    assert run(o)['status'] == 'REFUSED_T_POSE' and calls['segment'] == calls['observation'] == []
    o, calls, _ = owner_on_fake_bus(tmp_path, cfg=dict(CFG, t_pose='AFTER_ADOPTED_VIEW'))
    o.last_observation_view = dict(fresh_observation_acquired=True)
    assert run(o)['status'] == 'ENDPOINTS_RECORDED'


def test_after_right_pregrasp_skips_the_adopted_view_without_reading_or_moving(tmp_path, monkeypatch):
    o = fc.Owner.__new__(fc.Owner)
    o.scope, o.out, o.stopped, o.notes, o.last_raw = 'contact', tmp_path, None, [], {'ticks': {}}
    o.contact_plan = dict(observation_only=True, right_x_metrology=dict(CFG))
    o.note = lambda kind, **v: o.notes.append(dict(kind=kind, **v))
    o._read = lambda: pytest.fail('no read'); o.segment = lambda *a, **k: pytest.fail('no move')
    observed = {'visible_face_relation': {'status': 'MISSING'}}
    assert o.acquire_insertion_observation(observed, {}, lambda *a: None, [], stage='PLAN') == (observed, o.last_raw, False)
    assert o.notes[-1]['kind'] == 'right_x_metrology_skips_adopted_view'


# ---- the RECEIVE_CONTACT owner around it: no metrology approval -> no metrology write; normal reverse after it ----

def scoped_owner(tmp_path, contact_plan, *, normal_ready=True):
    (tmp_path / 'HANDOFF_SNAPSHOT.json').write_text('{}')
    state = bus(); state['torque']['right'] = dict.fromkeys(fc.ALL, False)

    class Recorder:
        @contextmanager
        def lease(self, *_):
            yield
        def event(self, kind, **values):
            pass
    recorder = Recorder()
    inner = SimpleNamespace(_consumed=False, _readiness={'ready': True}, _ready_out=tmp_path, recorder=recorder, recording_errors=[])
    inner.prepare = lambda snapshot, scene, evidence: {'status': 'PLAN_PREVIEW'}
    o = fc.Owner.__new__(fc.Owner)
    o.out, o.contact_plan, o.notes, o.block_owner, o.stopped, o.done, o.last_images = tmp_path, contact_plan, [], 'left', None, set(), {}
    o.contact_insert_history, o.contact_close_executed, o.insert, o.clock = [], False, None, lambda: 1.0
    o.note = lambda kind, **values: o.notes.append(dict(kind=kind, **values))
    o._read = lambda: copy.deepcopy(state); o.track_held = lambda raw: None
    reverse = [dict(label='RIGHT_RETURN_INITIAL', side='right', target_ticks=dict(RIGHT)), dict(label='LEFT_BACK_TO_LIFT', side='left', target_ticks={n: LEFT[n] for n in fc.ARM})]
    path = dict(ready=True, initial_ticks={s: dict(state['ticks'][s]) for s in ('left', 'right')},
                forward=[dict(label='CARRY_LEFT', side='left', target_q12=[1.0] * 12), dict(label='RIGHT_APPROACH', side='right', target_q12=[2.0] * 12)], reverse=reverse)
    o.plan_approach_prefix = lambda plan, raw: path; o.approach_plan = {'status': 'PLAN_PREVIEW'}

    def go(name, q12, *, arms):
        state['ticks'][arms[0]]['wrist_roll'] += 5.0
        o.notes.append(dict(kind='segment', name=name, side=arms[0], writes={'Goal_Position': 1})); return {arms[0]: {'result': 'HOLDING'}}
    o.go = go

    def contact_observation(stage, **kw):
        o.last_raw = copy.deepcopy(state); return {'q12': [0.0] * 12, 'visible_face_relation': {'status': 'MISSING'}}, copy.deepcopy(state), {}
    o.contact_observation = contact_observation
    o.insertion_ready = lambda current, **kw: False
    o.acquire_insertion_observation = lambda current, *a, **k: (current, copy.deepcopy(state), False)
    o.plan_contact_suffix = lambda raw, prefix, normal_only=False, **kw: dict(ready=normal_ready, reverse=reverse, why=None if normal_ready else 'OBSERVATION_BACK_000: path check is CONTACT')
    segments = []

    def segment(name, side, *, target=None, rest=None, purpose=None, **kw):
        segments.append(name)
        if name.endswith('right_return_initial'):
            state['torque']['right'] = dict.fromkeys(fc.ALL, False); result = 'RETURNED_RELEASED'
        else:
            result = 'HOLDING'
        o.notes.append(dict(kind='segment', name=name, side=side, writes={'Goal_Position': 1}))
        return dict(result=result, end=dict(ticks=copy.deepcopy(state['ticks'][side])))
    o.segment = segment
    o.recovery_ready = lambda *a, **k: (True, [])
    o.run_insert_hops = lambda *a, **k: pytest.fail('insert'); o.contact_close = lambda: pytest.fail('close')
    stops = []; o.on_stop = lambda *a: stops.append(a)
    return fc.Scoped(inner, o, 'contact'), o, recorder, segments, stops


def test_full_chain_without_metrology_approval_never_runs_it(tmp_path, monkeypatch):
    scoped, o, recorder, segments, _ = scoped_owner(tmp_path, {'observation_only': True})
    monkeypatch.setattr(fc.Owner, 'run_right_x_metrology', lambda *a, **k: pytest.fail('no RIGHT_X approval, no metrology write'))
    result = scoped._approach({'out': str(tmp_path), 'episode': recorder, 'scene': {}, 'evidence': {}})
    assert result['result'] == 'RECEIVE_CONTACT_NOT_READY_RETURNED' and result['right_x_metrology'] is None
    assert segments == ['contact_right_return_initial', 'contact_left_back_to_lift']


def test_metrology_runs_before_the_existing_normal_return(tmp_path, monkeypatch):
    scoped, o, recorder, segments, _ = scoped_owner(tmp_path, {'observation_only': True, 'right_x_metrology': dict(CFG)})
    order = []
    monkeypatch.setattr(fc.Owner, 'run_right_x_metrology', lambda self, prefix, live, stages: order.append(list(segments)) or dict(status='ENDPOINTS_RECORDED', why='ok'))
    result = scoped._approach({'out': str(tmp_path), 'episode': recorder, 'scene': {}, 'evidence': {}})
    assert order == [[]] and segments == ['contact_right_return_initial', 'contact_left_back_to_lift']
    assert result['result'] == 'RECEIVE_CONTACT_NOT_READY_RETURNED' and 'RIGHT_X_FINITE_METROLOGY ENDPOINTS_RECORDED' in result['why']


def test_normal_return_failure_after_metrology_keeps_the_existing_stop_contract(tmp_path, monkeypatch):
    scoped, o, recorder, segments, stops = scoped_owner(tmp_path, {'observation_only': True, 'right_x_metrology': dict(CFG)}, normal_ready=False)
    def metrology(self, prefix, live, stages):
        self.right_x_record = dict(status='ENDPOINTS_RECORDED'); return self.right_x_record
    monkeypatch.setattr(fc.Owner, 'run_right_x_metrology', metrology)
    result = scoped._approach({'out': str(tmp_path), 'episode': recorder, 'scene': {}, 'evidence': {}})
    assert result['result'] == 'FAILED' and 'normal return is not ready' in result['why'] and result['end']['returned'] is False
    assert stops and segments == [] and o.block_owner == 'unknown'


def test_fault_inside_metrology_marks_the_record_and_stops(tmp_path, monkeypatch):
    scoped, o, recorder, segments, stops = scoped_owner(tmp_path, {'observation_only': True, 'right_x_metrology': dict(CFG)})
    def metrology(self, prefix, live, stages):
        self.right_x_record = dict(status='STARTED'); self.notes.append(dict(kind='segment', writes={'Goal_Position': 3}))
        raise RuntimeError('right_x_v (right): executor ended BUS_LOST')
    monkeypatch.setattr(fc.Owner, 'run_right_x_metrology', metrology)
    result = scoped._approach({'out': str(tmp_path), 'episode': recorder, 'scene': {}, 'evidence': {}})
    assert result['result'] == 'FAILED' and result['right_x_metrology']['status'] == 'STOPPED_BY_FAULT' and stops and segments == []
    assert json.loads((tmp_path / 'RIGHT_X_METROLOGY.json').read_text())['status'] == 'STOPPED_BY_FAULT'


def test_fake_executor_segment_records_the_move_for_the_existing_normal_reverse(tmp_path):
    """segment(purpose=OBSERVATION_VIEW) with a fake executor: the X move lands in contact_observation_history, which the
    existing plan_contact_suffix turns into an OBSERVATION_BACK reverse step."""
    state = bus(); runs = []
    o = fc.Owner.__new__(fc.Owner)
    o.scope, o.out, o.notes, o.stopped, o.k, o.ep, o.run_kw = 'contact', tmp_path, [], None, 0, None, {}
    o.contact_plan, o.evaluation_mode, o.physical_execution_allowed = dict(observation_only=True, right_x_metrology=dict(CFG)), 'LIVE_REAL', True
    o.notes_sent, o.clock, o.note = [], lambda: 1.0, lambda kind, **v: o.notes.append(dict(kind=kind, **v))
    o._read = lambda: copy.deepcopy(state); o._read_other = lambda side: state['ticks'][side]; o.track_held = lambda raw: None
    o._check = lambda doc, side, raw: dict(nominal_verdict='CLEAR', nominal_steps_checked=max(1, doc['trajectory']['steps']), samples_required=max(1, doc['trajectory']['steps']))
    target = dict({n: RIGHT[n] for n in fc.ARM}, shoulder_lift=RIGHT['shoulder_lift'] - 12, elbow_flex=RIGHT['elbow_flex'] - 11, wrist_flex=RIGHT['wrist_flex'] + 24)
    import sweepick_trajectory as tt

    def executor(mode, side, folder, **kw):
        runs.append((mode, kw.get('observation_view'), kw.get('execute', False)))
        folder.mkdir(parents=True, exist_ok=True)
        if not kw.get('execute'):
            (folder / 'TRAJECTORY.json').write_text(json.dumps(dict(side=side, trajectory=tt.make(state['ticks'][side], target, tt.load_profile()))))
            return 0, dict(result='PLANNED')
        start = copy.deepcopy(state['ticks'][side]); state['ticks'][side].update(target); state['goal'][side].update(target)
        return 0, dict(result='HOLDING', writes={'Goal_Position': 5, 'Torque_Enable': 0}, start=dict(ticks=start), end=dict(ticks=dict(state['ticks'][side]), status=dict.fromkeys(fc.ALL, 0)),
                       rows=[dict(present=dict(state['ticks'][side]), goal=dict(state['goal'][side]))],
                       observation_view=dict(actual_settled=True, command_state_resolved=True, nominal_target_reached=True, command_completed=True))
    o.run = executor
    log = o.segment('right_x_f', 'right', target=target, purpose='OBSERVATION_VIEW')
    assert runs == [('plan', True, False), ('next', True, True)] and log['result'] == 'HOLDING'
    assert o.contact_observation_history == [dict(side='right', start_ticks={n: RIGHT[n] for n in fc.ARM}, last_actual_ticks=state['ticks']['right'],
                                                  last_applied_goal=state['goal']['right'], motion_sequence=1, terminal_result='HOLDING', insert_count=0, after_updated_pregrasp=False)]
    with pytest.raises(RuntimeError, match='observation-only'):
        o.segment('right_x_close', 'right', target={'gripper': 2000.0}, purpose='RIGHT_CLOSE')


SESSION = Path.home() / 'sweepick_console_runs/20261015_140844_3cc05c/session'


@pytest.fixture(scope='module')
def saved_real():
    if not (SESSION / 'RECEIVE_APPROACH_PLAN.json').exists():
        pytest.skip('saved REAL session not on this machine')
    stage = next(d for d in sorted((SESSION / 'decision_observations').iterdir()) if d.name.endswith('_RECEIVE_CONTACT'))
    return (json.loads((stage / 'OBSERVATION.json').read_text())['actual_feedback'], json.loads((SESSION / 'RECEIVE_APPROACH_PLAN.json').read_text()),
            json.loads((SESSION / 'RECEIVE_PLANNER_INPUT.json').read_text()), json.loads((SESSION / 'HELD_POSE.json').read_text()))


def real_planner_owner(tmp_path, saved_real, monkeypatch):
    if not Path(fc.CODEX).exists():                                   # the pin RUN_APPROVED_SESSION.py binds, independent of test order
        monkeypatch.setattr(fc, 'CODEX', Path(fc.__file__).resolve().parents[1] / 'pin')
    fc.codex()
    from sim_data_factory.teacher import HandoverTeacher
    raw, prefix, planner, held = saved_real
    o = fc.Owner(tmp_path, None, {}, p1=p1, read=lambda: pytest.fail('device read'))
    o.planner_options = planner['planning_arguments']['planner_options']
    o.teacher = HandoverTeacher(p1.model(), planner_options=o.planner_options)
    o.held = {'coarse_tool_from_object': held['coarse_tool_from_object']}
    o.scene_look, o.scope, o.record_kind, o.contact_close_executed = planner['scene_look'], 'contact', 'SAVED_REAL_REPLAY_DIAGNOSTIC', False
    o.contact_plan = dict(json.loads((Path(fc.__file__).resolve().parents[1] / 'RECEIVE_FIELD_PLAN.json').read_text()), right_x_metrology=dict(CFG))
    return o, raw, prefix


def test_real_A_ik_collision_and_normal_reverse_plan_an_x_move_from_saved_actual(tmp_path, saved_real, monkeypatch):
    o, raw, prefix = real_planner_owner(tmp_path, saved_real, monkeypatch)
    plan = o.plan_observation_candidate(rx.move_candidate('F', CFG), copy.deepcopy(raw), prefix)
    assert plan['ready'], plan['why']
    step, = plan['forward']
    assert step['certificate']['nominal_verdict'] == 'CLEAR' and step['native_validation']['executable'] and 'gripper' not in step['target_ticks']
    assert [r['label'] for r in plan['normal_return']['reverse']][:2] == ['OBSERVATION_BACK_000', 'RIGHT_BACK_TO_CARRY']
    model = p1.model()
    T0 = fc.tool_pose(model, p1.rc.ticks_to_q12(o.mp, raw['ticks']), site='right_cube_grasp')[0]
    split = fc.Owner.x_split(T0, fc.tool_pose(model, np.asarray(step['target_q12']), site='right_cube_grasp')[0], CFG['x_basis_tool'])
    assert np.sign(split['along_x_m']) == np.sign(CFG['fit_displacement_m'])      # recorded, not a new tolerance gate
    assert not hasattr(o, 'contact_observation_history')                          # planning leaves the executed-move history alone


def test_real_planner_refusals_reach_no_executor(tmp_path, saved_real, monkeypatch):
    from sim_data_factory.teacher import HandoverTeacher, TeacherFailure
    o, raw, prefix = real_planner_owner(tmp_path, saved_real, monkeypatch)
    o._check = lambda doc, side, fake: dict(nominal_verdict='CONTACT', first_nominal_contact=dict(step=1, pairs=[['right_jaw', 'block']]), nominal_steps_checked=1, samples_required=1)
    plan = o.plan_observation_candidate(rx.move_candidate('F', CFG), copy.deepcopy(raw), prefix)
    assert not plan['ready'] and 'view finite path refused' in plan['why']
    o, raw, prefix = real_planner_owner(tmp_path, saved_real, monkeypatch)
    monkeypatch.setattr(HandoverTeacher, 'plan_move', lambda *a, **k: (_ for _ in ()).throw(TeacherFailure('observation view: no IK solution')))
    plan = o.plan_observation_candidate(rx.move_candidate('F', CFG), copy.deepcopy(raw), prefix)
    assert not plan['ready'] and 'TeacherFailure' in plan['why'] and plan['forward'] == []
