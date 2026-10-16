"""2026-10-16 LEFT FOV actual correction through the REAL session(), correct(), plan_pick, stage() and the fake arm of PICK02.

The existing pre-grasp gate and 'the block stays in view after the descent' are judged apart; a FOV risk after a passed gate
reduces the EXISTING lateral error through the EXISTING 1-axis loop at the pre-grasp height. Synthetic camera and bus only.
A synthetic FOV reference is used where a case needs a predictable border; the stored-REAL reference is used for the rest."""
import hashlib
import json
from pathlib import Path

import pytest

import left_fov_check as lf
import sweepick_pick01 as p1
from test_sweepick_pick02_actual import World, base, cell as fake_cell, S, trail, all_off   # noqa: F401  (base is a fixture)

CONTEXT = dict(camera_side='left_wrist', mount_binding_sha256=hashlib.sha256(b'synthetic left wrist mount fixture').hexdigest(),
               image_size_px=[320, 240], object_width_mm=40.0)
FRAME = dict(camera_side='left_wrist', frame_id='synthetic-frame-1', raw_rgb_sha256=hashlib.sha256(b'synthetic raw RGB fixture').hexdigest())
WIDE = dict(schema='sweepick.left-fov-reference.v2', runs=[{}], width_ratio_median=2.0, width_ratio_range=[2.0, 2.0],
            offset_change_mm_median=0.0, offset_change_mm_range=[0.0, 0.0], pre_offset_mm_range=[-3.0, 3.0], context=CONTEXT)
WORLD_SCENE = World.scene
REAL_LOAD_REFERENCE = lf.load_reference


def cell(tmp_path, monkeypatch, base, *, fov_context=CONTEXT, frame_binding=FRAME, reference_context=None, **kwargs):
    def bound_scene(self, tag):
        scene = WORLD_SCENE(self, tag)
        scene['wrist'].update(fov_context=dict(fov_context), frame_binding=dict(frame_binding, frame_id=frame_binding.get('frame_id', tag)))
        return scene
    monkeypatch.setattr(World, 'scene', bound_scene)
    monkeypatch.setattr(lf, 'load_reference', lambda path=lf.REFERENCE_FILE: dict(WIDE, context=dict(fov_context if reference_context is None else reference_context)))
    result = fake_cell(tmp_path, monkeypatch, base, **kwargs)
    state = S(result[0]); state.setdefault('fov_sources', {})[lf.REFERENCE_FILE.name] = hashlib.sha256(lf.REFERENCE_FILE.read_bytes()).hexdigest()
    (result[0] / 'STATE.json').write_text(json.dumps(state))
    return result


def wide(monkeypatch):
    """A reference under which a block more than ~0.5 mm left of the midline at the pre-grasp look leaves the image later."""
    monkeypatch.setattr(lf, 'load_reference', lambda path=lf.REFERENCE_FILE: dict(WIDE))


def own_stage(base, fake, contact_in=None, lost_in=None):
    """The real stage() with the cell's fake bus; the model check of one stage answers CONTACT, or that stage loses the bus."""
    from test_sweepick_move_a import Clock
    c = Clock(); kw = dict(open_bus=lambda port: fake, owner=lambda p_: '', mapping=base['mp'], mapping_sha='m', sleep=c.sleep, clock=c.now)
    def run(o, name, execute=False):
        cert = lambda traj, out_, scene: dict(identity=json.loads(Path(traj).read_text())['identity'], nominal_verdict='CONTACT' if name == contact_in else 'CLEAR')
        if name == lost_in:
            def lost(*a, **k):
                raise ConnectionError('the bus is lost (test)')
            fake.write = lost
        try:
            return p1.stage(o, name, execute=execute, cert=cert, run_kw=kw, torque_now=lambda: fake.reg['Torque_Enable'])
        finally:
            fake.__dict__.pop('write', None)
    return run


def visual(out):
    return trail(out, 'visual')


def test_a_passed_gate_with_a_fov_risk_computes_a_nonzero_correction_and_runs_it_through_the_existing_stage(tmp_path, monkeypatch, base):
    wide(monkeypatch)
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, delta=2.0, block=3075)
    run(); st = S(out); v = visual(out)
    assert v[0]['ok'] is True and v[0]['fov']['status'] == 'FOV_RISK'                       # the existing gate passed
    assert v[0]['correction']['reason'] == 'FOV_RISK_AFTER_A_PASSED_GATE' and v[0]['correction']['tool_move_along_jaw_mm'] > 0
    assert st['order'][:2] == ['pregrasp', 'align'] and v[0]['plan_ok'] is True
    assert abs(v[1]['lateral_error_mm']) < abs(v[0]['lateral_error_mm'])                     # looked again: the error shrank
    assert v[1]['fov']['status'] == 'FOV_OK' and v[1]['fov_before_this_look']['status'] == 'FOV_RISK'
    assert st['order'][2] == 'descend_a'                                                      # then the existing descent
    hand = [c for c in st['correction'] if c['along_jaw_mm']]
    assert hand and all(c['kind'].startswith('HAND TARGET OFFSET') for c in hand) and '[FOV:' in hand[0]['basis']
    assert (st['plan']['tilt_deg'], st['plan']['jaw']) == (base['plan']['tilt_deg'], base['plan']['jaw'])


def test_a_failed_gate_keeps_the_existing_visual_servo_path(tmp_path, monkeypatch, base):
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, delta=4.0, block=3075)
    run(); v = visual(out)
    assert v[0]['ok'] is False and 'fov' not in v[0] and v[0]['correction']['reason'] == 'GATE_NOT_PASSED'


def test_a_normal_view_changes_no_command(tmp_path, monkeypatch, base):
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, delta=0.0, block=3075)
    run(); st = S(out); v = visual(out)
    assert v[0]['ok'] is True and v[0]['fov']['status'] == 'FOV_OK' and 'correction' not in v[0]
    assert st['order'][:2] == ['pregrasp', 'descend_a'] and not [c for c in st.get('correction') or [] if c['along_jaw_mm']]


def test_an_unconfirmed_or_clipped_feature_never_starts_a_correction(tmp_path, monkeypatch, base):
    wide(monkeypatch)
    clipped = lambda o, t, d, g: (dict(o, touches_border=True), t, d, g)
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, over={'01_': clipped}, delta=2.0, block=3075)
    r = run()
    assert r['result'] == 'NOT_ALIGNED_RETURNED_HOME' and S(out)['order'] == ['pregrasp', 'home_arm'] and all_off(f)
    ev = dict(wrist_object=dict(visible=True, stable=True, touches_border=False, width_px=150), wrist_offset=dict(readable=False), lateral_error_mm=None,
              fov_context=CONTEXT, frame_binding=FRAME)
    assert p1.fov_view_check({'jaw_reference': {'midline_px': 154.5}}, 'ready_to_descend', ev)['status'] == 'FOV_UNKNOWN'
    ev = dict(wrist_object=dict(visible=True, stable=True, touches_border=False, width_px=150), lateral_error_mm=-2.0,
              wrist_offset=dict(readable=True, offset_mm=-2.0, reference_confirmed_in_this_frame=False), fov_context=CONTEXT, frame_binding=FRAME)
    risk = p1.fov_view_check({'jaw_reference': {'midline_px': 154.5}}, 'ready_to_descend', ev)
    assert risk['status'] == 'FOV_UNKNOWN' and risk.get('correctable') is not True


def test_missing_or_corrupt_reference_and_sha_mismatch_fail_closed_before_descent(tmp_path, monkeypatch, base):
    out, fake, stage_fn, run = cell(tmp_path / 'missing', monkeypatch, base, delta=0.0, block=3075)
    monkeypatch.setattr(lf, 'load_reference', lambda path=lf.REFERENCE_FILE: None)
    result = run(); row = visual(out)[0]
    assert result['result'] == 'NOT_ALIGNED_RETURNED_HOME' and row['fov']['status'] == 'FOV_UNKNOWN'
    assert S(out)['order'] == ['pregrasp', 'home_arm'] and 'align' not in S(out)['stages'] and 'descend_a' not in S(out)['stages'] and all_off(fake)

    out, fake, stage_fn, run = cell(tmp_path / 'sha', monkeypatch, base, delta=0.0, block=3075)
    monkeypatch.setattr(lf, 'load_reference', lambda path=lf.REFERENCE_FILE: dict(WIDE))
    state = S(out); state['fov_sources'][lf.REFERENCE_FILE.name] = 'mismatch'; (out / 'STATE.json').write_text(json.dumps(state))
    direct = dict(wrist_object=dict(visible=True, stable=True, touches_border=False, width_px=150), lateral_error_mm=0.0,
                  wrist_offset=dict(readable=True, offset_mm=0.0, reference_confirmed_in_this_frame=True), fov_context=CONTEXT, frame_binding=FRAME)
    assert p1.fov_view_check(state, 'ready_to_descend', direct)['status'] == 'FOV_UNKNOWN'
    result = run()
    assert result['result'] == 'NOT_ALIGNED_RETURNED_HOME' and S(out)['order'] == ['pregrasp', 'home_arm']
    assert 'align' not in S(out)['stages'] and 'descend_a' not in S(out)['stages'] and all_off(fake)


def test_ood_context_mismatch_and_model_inside_observed_clip_never_authorize_descent(tmp_path, monkeypatch, base):
    state = {'jaw_reference': {'midline_px': 154.5}, 'fov_sources': {lf.REFERENCE_FILE.name: hashlib.sha256(lf.REFERENCE_FILE.read_bytes()).hexdigest()}}
    base_ev = dict(wrist_object=dict(visible=True, stable=True, touches_border=False, width_px=150), lateral_error_mm=0.0,
                   wrist_offset=dict(readable=True, offset_mm=0.0, reference_confirmed_in_this_frame=True), fov_context=CONTEXT, frame_binding=FRAME)
    assert p1.fov_view_check(state, 'ready_to_descend', dict(base_ev, fov_context=dict(CONTEXT, image_size_px=[640, 480])))['status'] == 'FOV_UNKNOWN'
    clipped = dict(base_ev, wrist_object=dict(base_ev['wrist_object'], touches_border=True), model_prior_envelope='INSIDE')
    assert p1.fov_view_check(state, 'ready_to_descend', clipped)['status'] == 'FOV_UNKNOWN'
    out, fake, stage_fn, run = cell(tmp_path, monkeypatch, base, delta=2.0, block=3075)
    narrow = dict(WIDE, pre_offset_mm_range=[-1.0, 1.0]); monkeypatch.setattr(lf, 'load_reference', lambda path=lf.REFERENCE_FILE: narrow)
    result = run(); row = visual(out)[0]
    assert result['result'] == 'NOT_ALIGNED_RETURNED_HOME' and row['fov']['status'] == 'OUT_OF_SUPPORT'
    assert S(out)['order'] == ['pregrasp', 'home_arm'] and 'align' not in S(out)['stages'] and 'descend_a' not in S(out)['stages'] and all_off(fake)


@pytest.mark.parametrize(('bad_context', 'bad_frame'), [
    (dict(CONTEXT, camera_side='right_wrist', image_size_px=[640, 480], object_width_mm=80.0), dict(FRAME, camera_side='right_wrist')),
    (dict(camera_side='', mount_binding_sha256='', image_size_px=[], object_width_mm=''), dict(camera_side='', frame_id='', raw_rgb_sha256='garbage')),
])
def test_matching_incompatible_or_malformed_context_never_authorizes_descent(tmp_path, monkeypatch, base, bad_context, bad_frame):
    out, fake, stage_fn, run = cell(tmp_path, monkeypatch, base, fov_context=bad_context, frame_binding=bad_frame,
                                    reference_context=bad_context, delta=0.0, block=3075)
    result = run(); row = visual(out)[0]
    assert result['result'] == 'NOT_ALIGNED_RETURNED_HOME' and row['fov']['status'] == 'FOV_UNKNOWN'
    assert S(out)['order'] == ['pregrasp', 'home_arm'] and 'align' not in S(out)['stages'] and 'descend_a' not in S(out)['stages'] and all_off(fake)


def test_a_finite_reference_that_overflows_the_derived_view_is_unknown(tmp_path, monkeypatch):
    reference_file = tmp_path / 'finite_but_overflowing_reference.json'
    reference_file.write_text(json.dumps(dict(WIDE, width_ratio_median=1e308, width_ratio_range=[1e308, 1e308])))
    reference = lf.load_reference(reference_file)
    assert reference is not None
    monkeypatch.setattr(lf, 'load_reference', lambda path=lf.REFERENCE_FILE: reference)
    state = {'jaw_reference': {'midline_px': 154.5}, 'fov_sources': {lf.REFERENCE_FILE.name: hashlib.sha256(lf.REFERENCE_FILE.read_bytes()).hexdigest()}}
    evidence = dict(wrist_object=dict(visible=True, stable=True, touches_border=False, width_px=150), lateral_error_mm=0.0,
                    wrist_offset=dict(readable=True, offset_mm=0.0, reference_confirmed_in_this_frame=True), fov_context=CONTEXT, frame_binding=FRAME)
    assert p1.fov_view_check(state, 'ready_to_descend', evidence)['status'] == 'FOV_UNKNOWN'


@pytest.mark.parametrize('mutation', ('missing', 'corrupt', 'changed', 'oversized_integer'))
def test_reference_damage_after_pregrasp_fails_closed_and_keeps_the_existing_return(tmp_path, monkeypatch, base, mutation):
    out, fake, stage_fn, run = cell(tmp_path, monkeypatch, base, delta=0.0, block=3075)
    reference = tmp_path / 'synthetic_fov_reference.json'; reference.write_text(json.dumps(WIDE))
    monkeypatch.setattr(lf, 'REFERENCE_FILE', reference)
    monkeypatch.setattr(lf, 'load_reference', lambda path=reference: REAL_LOAD_REFERENCE(reference))
    state = S(out); state['fov_sources'] = {reference.name: hashlib.sha256(reference.read_bytes()).hexdigest()}; (out / 'STATE.json').write_text(json.dumps(state))
    world = World(base, fake, delta=0.0)
    def damage_then_look(o, tag):
        scene = world.scene(tag)
        if mutation == 'missing':
            reference.unlink(missing_ok=True)
        elif mutation == 'corrupt':
            reference.write_text('{bad')
        elif mutation == 'oversized_integer':
            reference.write_text(json.dumps(dict(WIDE, width_ratio_median=10**400, width_ratio_range=[10**400, 10**400])))
        else:
            reference.write_text(json.dumps(dict(WIDE, role='synthetic changed fixture')))
        return scene
    result = p1.session(out, execute=True, stage_fn=stage_fn, look_fn=damage_then_look, observe_fn=lambda o: S(out))
    assert result['result'] == 'NOT_ALIGNED_RETURNED_HOME' and visual(out)[0]['fov']['status'] == 'FOV_UNKNOWN'
    assert S(out)['order'] == ['pregrasp', 'home_arm'] and all_off(fake)


def test_the_close_depth_and_the_intermediate_height_never_get_a_fov_move():
    ev = dict(wrist_object=dict(visible=True, stable=True, touches_border=False, width_px=150), lateral_error_mm=-2.0,
              wrist_offset=dict(readable=True, offset_mm=-2.0, reference_confirmed_in_this_frame=True))
    assert p1.fov_view_check({'jaw_reference': {'midline_px': 154.5}}, 'ready_for_final_descent', ev) is None
    assert p1.fov_view_check({'jaw_reference': {'midline_px': 154.5}}, 'ready_to_close', ev) is None
    assert p1.FOV_GATES == ('ready_to_descend',)


def test_a_refused_plan_or_a_contact_on_the_correction_path_stops_the_next_move(tmp_path, monkeypatch, base):
    wide(monkeypatch)
    out, f, stage_fn, run = cell(tmp_path / 'ik', monkeypatch, base, delta=2.0, block=3075)
    real_plan = p1.plan_pick
    calls = []
    def refuse(scene, **k):
        calls.append(k)
        return dict(real_plan(scene, **k), ok=False, why='IK: no solution (test)') if k.get('tool_offset') is not None else real_plan(scene, **k)
    monkeypatch.setattr(p1, 'plan_pick', refuse)
    r = run()
    assert r['result'] == 'NOT_ALIGNED_RETURNED_HOME' and S(out)['order'] == ['pregrasp', 'home_arm'] and visual(out)[0]['plan_ok'] is False
    monkeypatch.setattr(p1, 'plan_pick', real_plan)
    out, f, stage_fn, run = cell(tmp_path / 'col', monkeypatch, base, delta=2.0, block=3075)
    w = World(base, f, delta=2.0)
    r = p1.session(out, execute=True, stage_fn=own_stage(base, f, contact_in='align'), look_fn=lambda o, tag: w.scene(tag), observe_fn=lambda o: S(out))
    assert 'descend_a' not in S(out)['order'] and r['result'] != 'PICKED_LIFTED_AND_PUT_BACK', (r['result'], S(out)['order'])
    align = [json.loads(m_.read_text()) for m_ in (out / 'align').glob('run*/move.json')]
    assert all(m_.get('writes', {}).get('Goal_Position', 0) == 0 for m_ in align), [m_.get('writes') for m_ in align]


def test_an_error_that_does_not_shrink_after_the_fov_move_keeps_the_existing_stop(tmp_path, monkeypatch, base):
    wide(monkeypatch)
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, delta=-2.0, k=-1.0, block=3075)      # the image answers the move the wrong way
    r = run(); v = visual(out)
    assert v[0]['fov']['status'] == 'FOV_RISK' and v[0]['correction']['reason'] == 'FOV_RISK_AFTER_A_PASSED_GATE'
    assert r['result'] == 'NOT_ALIGNED_RETURNED_HOME' and 'end' in v[-1] and 'descend_a' not in S(out)['order'] and all_off(f)


def test_a_bus_fault_in_the_fov_move_keeps_the_existing_holding_contract(tmp_path, monkeypatch, base):
    wide(monkeypatch)
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, delta=2.0, block=3075)
    w = World(base, f, delta=2.0)
    r = p1.session(out, execute=True, stage_fn=own_stage(base, f, lost_in='align'), look_fn=lambda o, tag: w.scene(tag), observe_fn=lambda o: S(out))
    assert r['result'] == 'STOPPED_IN_ALIGN_HOLDING_NEEDS_A_PERSON' and S(out)['order'][-1] == 'align', (r['result'], S(out)['order'])
    assert 'descend_a' not in S(out)['order'] and 'home_arm' not in S(out)['order']        # no automatic way home after a lost bus: the existing contract


def test_an_operator_stop_in_the_fov_move_keeps_the_existing_holding_contract(tmp_path, monkeypatch, base):
    wide(monkeypatch)
    out, fake, stage_fn, run = cell(tmp_path, monkeypatch, base, delta=2.0, block=3075)
    world = World(base, fake, delta=2.0); real_stage = own_stage(base, fake)
    def stopped(o, name, execute=False):
        return dict(result='RAN', controller_result='OPERATOR_STOPPED_HOLDING', exit_code=130, why='operator stop (test)') if name == 'align' else real_stage(o, name, execute=execute)
    result = p1.session(out, execute=True, stage_fn=stopped, look_fn=lambda o, tag: world.scene(tag), observe_fn=lambda o: S(out))
    assert result['result'] == 'STOPPED_IN_ALIGN_HOLDING_NEEDS_A_PERSON'
    assert 'descend_a' not in S(out)['order'] and 'home_arm' not in S(out)['order']


STORED = sorted(Path.home().glob('sweepick_console_runs/*/session/STATE.json')) + sorted(p for p in Path.home().glob('sweepick_26101*/**/STATE.json') if 'REVIEW_BUNDLE' not in str(p))


@pytest.mark.skipif(not lf.REFERENCE_FILE.exists() or not STORED, reason='stored REAL runs / reference not on this machine')
def test_stored_real_pick_runs_keep_their_accepted_path_except_the_named_ones():
    """Offline on stored REAL records only: the executor's FOV check on the LAST accepted pre-grasp look of each run. These are
    not new physical results."""
    flagged = {}
    for path in STORED:
        try:
            st = json.loads(path.read_text())
        except ValueError:
            continue
        pre = [e for e in st.get('evidence') or [] if e.get('what') == 'ready_to_descend' and e.get('ok')]
        if not pre or not any(e.get('what') == 'ready_to_close' for e in st['evidence']):
            continue
        fov = p1.fov_view_check(st, 'ready_to_descend', pre[-1])
        flagged[str(path)] = fov['status']
    assert sorted(flagged.values()).count('OUT_OF_SUPPORT') == 1 and sorted(flagged.values()).count('FOV_UNKNOWN') == 17
    assert [k for k, v in flagged.items() if v == 'OUT_OF_SUPPORT'][0].find('20261016_112020_4d7c2b') >= 0
