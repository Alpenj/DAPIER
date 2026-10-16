"""RIGHT_X_FINITE_METROLOGY console scope: offered only with the tested native contract, its own approval/manifest, the
five operator inputs without defaults, the full write list acknowledged at START, and one field-confirmed operator
decision per F/V request digest. FakeRunner and fake owners only: no device, no tmux launch, no motor write."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import threading
import time
import unittest
from unittest.mock import patch

import control
import test_control
import test_records
from control import FULL_CHAIN, LEFT_TO_LIFT, RIGHT_X, RIGHT_X_MOVE, VIEW_MOVE, FIELD_REQUIRED, ControlError, right_x_backend
from records import atomic_json, run_record

NATIVE = Path(os.environ.get('SWEEPICK_RIGHT_X_NATIVE', Path.home() / 'sweepick_261016_right_x_metrology_native'))
INPUTS = {'t_pose': 'AFTER_RIGHT_PREGRASP', 'x_basis_tool': [1.0, 0.0, 0.0],
          'x_basis_source': 'A model tool +X; physical direction unconfirmed', 'fit_displacement_m': -0.004, 'verify_displacement_m': -0.003}
FIELD = {'direction_observed': True, 'start_pose_observed': True, 'noncontact_space_observed': True,
         'direction_note': 'gripper moves away from the held block'}
ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


def load_native(name, relative):
    spec = importlib.util.spec_from_file_location(name, NATIVE / relative)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module


def native_launcher():
    return load_native('right_x_native_launcher', 'RUN_APPROVED_SESSION.py')


def native_rx():
    return load_native('right_x_native_contract', 'revision/right_x_metrology.py')


def actual(seq, right=2000.0):
    ticks = {'right': dict.fromkeys(ARM, right), 'left': dict.fromkeys(ARM, 1500.0)}
    return {'ticks': ticks, 'status': {'left': dict.fromkeys(ARM, 0), 'right': dict.fromkeys(ARM, 0)}, 'bus_seq': seq, 't_bus_monotonic': 10.0 + seq}


def x_plan(raw, delta=-6.0, **over):
    start = dict(raw['ticks']['right']); target = {n: v + (delta if n in ('shoulder_lift', 'elbow_flex', 'wrist_flex') else 0.0) for n, v in start.items()}
    plan = {'ready': True, 'phase': 'OBSERVATION_CANDIDATE', 'why': None,
            'forward': [{'side': 'right', 'start_ticks': start, 'target_ticks': target, 'certificate': {'nominal_verdict': 'CLEAR'},
                         'native_validation': {'executable': True, 'hard': [], 'provisional': []}}],
            'normal_return': {'ready': True}}
    plan.update(over); return plan


class FakeOwner:
    """The native authorize wait's view of the owner: fresh reads, support check, sleep. Real time, short cadence."""
    def __init__(self, out, raw):
        self.out, self._raw, self.notes, self.idle_limit_s, self.idle_basis, self.last_recovery_check = Path(out), raw, [], 5.0, 'test', {}
        self.supported = True

    def clock(self):
        return time.monotonic()

    def raw(self):
        return self._raw

    def recovery_ready(self, *a, **k):
        self.last_recovery_check = {'register_checks_ok': self.supported}
        return (self.supported, None if self.supported else 'left support lost')

    def note(self, kind, **v):
        self.notes.append(dict(kind=kind, **v))

    def sleep(self, seconds):
        time.sleep(min(float(seconds), 0.02))


def write_decision(out, doc):
    """As the console writes it (temporary file, then replace): the native wait never reads a half-written file."""
    temporary = Path(out) / 'OBSERVATION_MOVE_APPROVAL.tmp'
    temporary.write_text(json.dumps(doc)); temporary.replace(Path(out) / 'OBSERVATION_MOVE_APPROVAL.json')


class Waiter:
    """One native authorize_observation_plan call in a thread, as the launcher runs it for an X move."""
    def __init__(self, runner, owner, plan, raw, candidate, scope=RIGHT_X_MOVE):
        self.result, self.error = None, None
        def run():
            try:
                self.result = runner.authorize_observation_plan(owner, plan, raw, candidate, preapproval=None, scope=scope)
            except Exception as exc:                                          # the native STOP/support exceptions
                self.error = exc
        self.thread = threading.Thread(target=run, daemon=True); self.thread.start()

    def join(self, timeout=5.0):
        self.thread.join(timeout); return not self.thread.is_alive()


class RightXConsole(unittest.TestCase):
    setUp = test_control.ControlContract.setUp

    def native(self, tested=None):
        patcher = patch.object(control, 'A', NATIVE); patcher.start(); self.addCleanup(patcher.stop)
        record = Path(self.temporary.name) / 'RIGHT_X_TESTED_NATIVE.json'
        files = {name: hashlib.sha256((NATIVE / name).read_bytes()).hexdigest() for name in control.RIGHT_X_BOUND_SOURCES}
        files.update(tested or {})
        atomic_json(record, {'files': files})
        tp = patch.object(control, 'RIGHT_X_TESTED_NATIVE', record); tp.start(); self.addCleanup(tp.stop)
        return record

    def launches(self):
        return [c for c in self.runner.calls if c[:2] == ['tmux', 'new-session']]

    def start(self, job, **k):
        return self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True, **k)

    def prepare(self, **over):
        return self.control.prepare(execution_scope=RIGHT_X, right_x=dict(INPUTS, **over))['active']

    def by_id(self):
        return {s['id']: s for s in self.control.operation_contract()['scopes']}

    # ---- P0-1: offered only with the tested native contract -------------------------------------------------------
    def test_active_native_without_the_contract_stays_disabled(self):
        supported, reason = right_x_backend()
        if supported:
            self.skipTest('the active native folder already carries the RIGHT_X contract')
        self.assertFalse(self.by_id()[RIGHT_X]['supported']); self.assertEqual(self.by_id()[RIGHT_X]['backend_status'], 'BACKEND_CHANGE_REQUIRED')
        with self.assertRaisesRegex(ControlError, 'BACKEND_CHANGE_REQUIRED'):
            self.control.prepare(execution_scope=RIGHT_X, right_x=dict(INPUTS))
        self.assertFalse(self.launches())

    def test_marker_alone_is_not_readiness_active_sha_must_equal_tested_source(self):
        self.native(tested={'revision/right_x_metrology.py': '0' * 64})
        supported, reason = right_x_backend()
        self.assertFalse(supported); self.assertIn('revision/right_x_metrology.py', reason)
        with self.assertRaisesRegex(ControlError, 'BACKEND_CHANGE_REQUIRED'):
            self.control.prepare(execution_scope=RIGHT_X, right_x=dict(INPUTS))
        self.native().unlink()
        self.assertIn('시험 기록이 없음', right_x_backend()[1])

    def test_catalog_shows_the_native_write_list_inputs_without_defaults_and_field_requirement(self):
        self.native()
        scope, rx = self.by_id()[RIGHT_X], native_rx()
        self.assertTrue(scope['supported']); self.assertTrue(scope['can_prepare'])
        self.assertEqual(scope['included_stages'], list(rx.DECLARED_WRITES)); self.assertEqual(len(scope['included_stages']), 13)
        self.assertEqual(scope['excluded_stages'], list(rx.EXCLUDED))
        self.assertEqual([i['key'] for i in scope['operator_inputs']], ['t_pose', 'x_basis_tool', 'x_basis_source', 'fit_displacement_m', 'verify_displacement_m'])
        self.assertTrue(all(i['default'] is None for i in scope['operator_inputs']))
        self.assertTrue(all(i.get('limit') is None for i in scope['operator_inputs']))
        self.assertEqual(scope['physical_basis']['status'], FIELD_REQUIRED)
        self.assertIs(scope['physical_basis']['operator_displacement_is_physical_permission'], False)
        self.assertTrue(self.control.operation_contract()['right_x_source_check']['all_match'])
        self.assertTrue(self.by_id()[FULL_CHAIN]['supported'])                     # the existing scopes are unchanged
        self.assertEqual(self.by_id()[LEFT_TO_LIFT]['id'], LEFT_TO_LIFT)

    def test_malformed_or_defaulted_inputs_prepare_nothing(self):
        self.native()
        cases = [None, {}, dict(INPUTS, fit_displacement_m=0), dict(INPUTS, verify_displacement_m=float('nan')),
                 dict(INPUTS, x_basis_tool=[1, 1, 0]), dict(INPUTS, x_basis_tool=[1, 0]), dict(INPUTS, x_basis_source='  '),
                 dict(INPUTS, t_pose='AFTER_ANYTHING'), dict(INPUTS, fit_displacement_m=True), dict(INPUTS, speed=1.0),
                 {k: v for k, v in INPUTS.items() if k != 'verify_displacement_m'}]
        for inputs in cases:
            with self.subTest(inputs=inputs), self.assertRaisesRegex(ControlError, 'RIGHT_X'):
                self.control.prepare(execution_scope=RIGHT_X, right_x=inputs)
        with self.assertRaisesRegex(ControlError, '새 세션'):
            self.control.prepare('session_18', execution_scope=RIGHT_X, right_x=dict(INPUTS))
        with self.assertRaisesRegex(ControlError, 'RIGHT_X 입력은'):
            self.control.prepare(execution_scope=FULL_CHAIN, right_x=dict(INPUTS))
        self.assertIsNone(self.control.status()['active']); self.assertFalse(self.launches())

    def test_prepare_has_its_own_unapproved_approval_and_manifest(self):
        self.native()
        job = self.prepare()
        folder = Path(job['session_out']).parent
        manifest = json.loads((folder / 'MANIFEST.json').read_text()); approval = json.loads((folder / 'APPROVAL.json').read_text())
        self.assertEqual((manifest['scope'], manifest['execution_scope']), (RIGHT_X, RIGHT_X))
        for name in control.RIGHT_X_BOUND_SOURCES:
            self.assertEqual(manifest['files'][str(NATIVE / name)], hashlib.sha256((NATIVE / name).read_bytes()).hexdigest())
        self.assertEqual((approval['scope'], approval['execution_scope']), (RIGHT_X, RIGHT_X))
        self.assertEqual(approval['right_x_metrology'], dict(INPUTS, verify_from='F'))
        self.assertEqual(approval['declared_motor_writes'], list(native_rx().DECLARED_WRITES))
        self.assertFalse(approval['approved']); self.assertFalse(approval['permission_to_open_devices']); self.assertFalse(approval['writes_acknowledged'])
        self.assertNotIn('observation_view_preapproval', approval); self.assertNotIn('previous_attempt', approval)
        self.assertEqual(approval['right_x_physical_basis']['status'], FIELD_REQUIRED)
        self.assertFalse(any(v.get('field_confirmed') for v in approval['right_x_physical_basis'].values() if isinstance(v, dict)))
        self.assertEqual(job['right_x_metrology'], approval['right_x_metrology'])
        self.assertIn('RIGHT X F·V', self.control.status()['scope'])
        self.assertFalse(self.launches()); self.assertFalse(Path(job['session_out']).exists())

    # ---- START: full write list acknowledged, no START pre-approval, exact bytes ---------------------------------------
    def test_start_needs_the_write_list_acknowledged_and_never_takes_a_start_preapproval(self):
        self.native()
        job = self.prepare()
        with self.assertRaisesRegex(ControlError, '전체 조작 단계'):
            self.start(job)
        with self.assertRaisesRegex(ControlError, 'START 사전 승인'):
            self.start(job, writes_acknowledged=True, preapprove_view=True)
        self.assertFalse(self.launches()); self.assertFalse(self.control.status()['active']['consumed'])

    def test_start_refuses_changed_writes_inputs_or_bytes(self):
        self.native()
        for change in ('writes', 'inputs', 'manifest'):
            with self.subTest(change=change):
                job = self.prepare()
                folder = Path(job['session_out']).parent
                if change == 'manifest':
                    doc = json.loads((folder / 'MANIFEST.json').read_text()); doc['files'][str(NATIVE / 'revision/right_x_metrology.py')] = '1' * 64
                    (folder / 'MANIFEST.json').write_text(json.dumps(doc))
                else:
                    doc = json.loads((folder / 'APPROVAL.json').read_text())
                    if change == 'writes':
                        doc['declared_motor_writes'] = doc['declared_motor_writes'][:-1]
                    else:
                        doc['right_x_metrology']['fit_displacement_m'] = -0.04
                    (folder / 'APPROVAL.json').write_text(json.dumps(doc))
                with self.assertRaises(ControlError):
                    self.start(job, writes_acknowledged=True)
                self.assertFalse(self.launches())

    def test_console_files_pass_the_native_right_x_launch_check_after_start(self):
        self.native()
        job = self.prepare()
        self.start(job, execution_scope=RIGHT_X, writes_acknowledged=True)
        self.assertEqual(len(self.launches()), 1); self.assertIn('RUN_APPROVED_SESSION.py --execute --managed-tmux', self.launches()[0][-1])
        folder = Path(job['session_out']).parent
        runner = native_launcher()
        with patch.object(runner, 'APPROVAL', folder / 'APPROVAL.json'), patch.object(runner, 'MANIFEST', folder / 'MANIFEST.json'), \
                patch.dict(os.environ, {'SWEEPICK_SESSION_OUT': job['session_out']}):
            approval, plan, out, manifest = runner.validate_launch()
        self.assertEqual(runner.execution_scope(approval), RIGHT_X); self.assertEqual(str(out), job['session_out'])
        self.assertEqual(plan['right_x_metrology'], dict(INPUTS, verify_from='F'))
        self.assertIsNone(runner.preapproval_record(approval, out, hashlib.sha256((folder / 'MANIFEST.json').read_bytes()).hexdigest()))
        self.assertTrue(approval['writes_acknowledged'])

    def test_other_approvals_cannot_open_right_x(self):
        self.native()
        for scope in (FULL_CHAIN,):
            job = self.control.prepare(execution_scope=scope)['active']
            with self.assertRaisesRegex(ControlError, '범위'):
                self.start(job, execution_scope=RIGHT_X, writes_acknowledged=True)
            job['execution_scope'] = RIGHT_X; job['right_x_metrology'] = dict(INPUTS, verify_from='F'); self.control.save(job)
            with self.assertRaisesRegex(ControlError, '범위가 실행 범위와 다름'):
                self.start(job, writes_acknowledged=True)
        self.assertFalse(self.launches())

    def test_full_chain_prepare_and_start_preapproval_are_unchanged(self):
        self.native()
        job = self.control.prepare(execution_scope=FULL_CHAIN)['active']
        folder = Path(job['session_out']).parent
        self.assertEqual(json.loads((folder / 'MANIFEST.json').read_text())['scope'], 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION')
        self.assertNotIn('right_x_metrology', json.loads((folder / 'APPROVAL.json').read_text()))
        self.assertEqual(self.control.status()['scope'], control.SCOPE)
        self.start(job, preapprove_view=True)
        approval = json.loads((folder / 'APPROVAL.json').read_text())
        self.assertEqual(approval['observation_view_preapproval']['scope'], VIEW_MOVE); self.assertEqual(len(self.launches()), 1)

    def test_no_automatic_next_session_after_a_right_x_run(self):
        self.native()
        job = self.prepare(); self.start(job, writes_acknowledged=True)
        self.runner.alive = False
        with patch.object(self.control, 'off_confirmed', return_value=True):
            status = self.control.auto_prepare_next()
        self.assertEqual(status['active']['id'], job['id']); self.assertNotIn('auto_next', status['active'])

    # ---- P0-2/P0-3: one field-confirmed decision per request digest --------------------------------------------------
    def running(self):
        self.native()
        job = self.prepare(); self.start(job, writes_acknowledged=True)
        Path(job['session_out']).mkdir(parents=True)
        self.assertEqual(self.control.status()['state'], 'RUNNING')
        return job, native_launcher(), native_rx()

    def pending(self, timeout=5.0, previous=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            p = self.control.status()['active'].get('pending_view')
            if p and p['request_sha256'] != previous:
                return p
            time.sleep(0.02)
        self.fail('no pending request')

    def test_f_and_v_are_separate_digests_and_the_f_approval_never_answers_v(self):
        job, runner, rx = self.running()
        cfg = dict(INPUTS, verify_from='F')
        raw_f = actual(1); owner = FakeOwner(job['session_out'], raw_f)
        f = Waiter(runner, owner, x_plan(raw_f), raw_f, rx.move_candidate('F', cfg))
        pf = self.pending()
        self.assertEqual(pf['scope'], RIGHT_X_MOVE); self.assertNotIn('approval_refused', pf)
        s = pf['right_x']
        self.assertEqual((s['role'], s['commanded_tool_displacement_m'], s['x_basis_tool']), ('F', -0.004, [1.0, 0.0, 0.0]))
        self.assertEqual((s['collision'], s['native_executable'], s['normal_return_ready'], s['plan_start_equals_request_actual']), ('CLEAR', True, True, True))
        self.assertEqual(s['planned_from']['bus_seq'], 1); self.assertEqual(s['tick_change']['elbow_flex'], -6.0)
        self.assertEqual(s['physical_status'], FIELD_REQUIRED)
        # P0-2: the operator numbers + model CLEAR are not enough; no decision file, the owner keeps waiting
        for partial in (None, {}, dict(FIELD, noncontact_space_observed=False), dict(FIELD, direction_note=' ')):
            with self.subTest(partial=partial), self.assertRaisesRegex(ControlError, FIELD_REQUIRED):
                self.control.view_decision(job['id'], pf['request_sha256'], True, field_confirmation=partial)
        self.assertFalse((Path(job['session_out']) / 'OBSERVATION_MOVE_APPROVAL.json').exists()); self.assertFalse(f.join(0.2))
        self.control.view_decision(job['id'], pf['request_sha256'], True, field_confirmation=FIELD)
        self.assertTrue(f.join()); self.assertIs(f.result, True)
        decision = json.loads((Path(job['session_out']) / 'OBSERVATION_MOVE_APPROVAL.json').read_text())
        self.assertEqual((decision['role'], decision['scope'], decision['request_sha256']), ('F', RIGHT_X_MOVE, pf['request_sha256']))
        self.assertEqual(decision['field_physical_confirmation']['source'], 'local_operator_field_observation')
        self.assertIs(decision['operator_displacement_is_physical_permission'], False)
        self.assertIsNone(self.control.status()['active']['pending_view'])          # answered
        # V: a new request with its own digest; the F approval file is still there and must not answer it
        raw_v = actual(2, right=1994.0); owner._raw = raw_v
        v = Waiter(runner, owner, x_plan(raw_v, delta=-4.0), raw_v, rx.move_candidate('V', cfg))
        pv = self.pending(previous=pf['request_sha256'])
        self.assertNotEqual(pv['request_sha256'], pf['request_sha256']); self.assertEqual(pv['right_x']['role'], 'V')
        self.assertEqual(pv['right_x']['commanded_tool_displacement_m'], -0.003)
        self.assertFalse(v.join(0.3))                                                # the F decision file is ignored, still waiting
        with self.assertRaisesRegex(ControlError, '일치하지 않음'):                    # replaying the F approval digest
            self.control.view_decision(job['id'], pf['request_sha256'], True, field_confirmation=FIELD)
        self.control.view_decision(job['id'], pv['request_sha256'], False)          # declining needs no field confirmation
        self.assertTrue(v.join()); self.assertIs(v.result, False)
        consumed = [n for n in owner.notes if n['kind'] == 'observation_move_approval_consumed']
        self.assertEqual([(n['approved'], n['request_sha256']) for n in consumed], [(True, pf['request_sha256']), (False, pv['request_sha256'])])

    def test_foreign_decisions_are_ignored_but_stop_still_ends_the_wait(self):
        job, runner, rx = self.running()
        out = Path(job['session_out']); raw = actual(1); owner = FakeOwner(out, raw)
        w = Waiter(runner, owner, x_plan(raw), raw, rx.move_candidate('F', dict(INPUTS, verify_from='F')))
        p = self.pending()
        for foreign in ({'approved': True, 'request_sha256': 'f' * 64, 'scope': RIGHT_X_MOVE, 'session_out': str(out)},   # another digest
                        {'approved': True, 'request_sha256': p['request_sha256'][::-1], 'scope': VIEW_MOVE, 'session_out': str(out)}):
            write_decision(out, foreign)
            self.assertFalse(w.join(0.2))                                            # ignored: still waiting, nothing moved
            self.assertEqual(self.control.status()['active']['pending_view']['request_sha256'], p['request_sha256'])  # still shown
        self.assertTrue(self.control.status()['can_stop'])
        self.control.stop(job['id'])                                                  # the console's STOP: .STOP latch + C-c to the owner
        self.assertTrue(Path(job['session_out'] + '.STOP').exists())
        self.assertIn(['tmux', 'send-keys', '-t', control.MANAGED, 'C-c'], self.runner.calls)
        try:
            runner._STOP_SIGNAL = signal.SIGINT                                       # what that C-c sets in the launcher
            self.assertTrue(w.join(2.0))
        finally:
            runner._STOP_SIGNAL = None
        self.assertIsInstance(w.error, RuntimeError); self.assertIn('STOP', str(w.error)); self.assertIsNone(w.result)

    def test_same_digest_with_another_session_or_scope_is_not_an_approval(self):
        job, runner, rx = self.running()
        out = Path(job['session_out'])
        for wrong in ({'session_out': str(out) + '_other'}, {'scope': VIEW_MOVE}):
            with self.subTest(wrong=wrong):
                raw = actual(len(wrong)); owner = FakeOwner(out, raw)
                w = Waiter(runner, owner, x_plan(raw), raw, rx.move_candidate('F', dict(INPUTS, verify_from='F')))
                p = self.pending(previous=getattr(self, '_last', None)); self._last = p['request_sha256']
                write_decision(out, dict({'approved': True, 'request_sha256': p['request_sha256'], 'scope': RIGHT_X_MOVE, 'session_out': str(out)}, **wrong))
                self.assertTrue(w.join()); self.assertIs(w.result, False, w.error)    # ends the wait as not approved (no move)

    def test_support_loss_while_waiting_raises_into_the_stop_contract(self):
        job, runner, rx = self.running()
        raw = actual(1); owner = FakeOwner(job['session_out'], raw)
        w = Waiter(runner, owner, x_plan(raw), raw, rx.move_candidate('F', dict(INPUTS, verify_from='F')))
        self.pending(); owner.supported = False
        self.assertTrue(w.join()); self.assertIn('lost current support/bus', str(w.error))

    def test_a_request_that_is_not_a_single_clear_returnable_actual_bound_move_cannot_be_approved(self):
        job, runner, rx = self.running()
        raw = actual(1); cfg = dict(INPUTS, verify_from='F')
        bad = [x_plan(raw, normal_return={'ready': False, 'why': 'reverse refused'}),
               x_plan(dict(raw, ticks={'right': dict.fromkeys(ARM, 2100.0), 'left': raw['ticks']['left']})),    # not from this actual
               x_plan(raw, forward=x_plan(raw)['forward'] * 2)]
        bad[0]['forward'][0]['certificate'] = {'nominal_verdict': 'CONTACT'}
        previous = None
        for plan in bad:
            with self.subTest(plan=plan.get('normal_return')):
                owner = FakeOwner(job['session_out'], raw)
                w = Waiter(runner, owner, plan, raw, rx.move_candidate('F', cfg))
                p = self.pending(previous=previous); previous = p['request_sha256']
                with self.assertRaisesRegex(ControlError, '승인하지 않음'):
                    self.control.view_decision(job['id'], p['request_sha256'], True, field_confirmation=FIELD)
                self.control.view_decision(job['id'], p['request_sha256'], False)
                self.assertTrue(w.join()); self.assertIs(w.result, False)

    def test_request_for_another_session_or_a_full_chain_job_is_decline_only(self):
        job, runner, rx = self.running()
        out = Path(job['session_out'])
        proposal = {'scope': RIGHT_X_MOVE, 'session_out': str(out) + '_other', 'candidate': rx.move_candidate('F', dict(INPUTS, verify_from='F')),
                    'plan': x_plan(actual(1)), 'actual_feedback': actual(1)}
        (out / 'OBSERVATION_MOVE_REQUEST.json').write_text(json.dumps(proposal, sort_keys=True))
        p = self.pending()
        self.assertIn('approval_refused', p)
        with self.assertRaisesRegex(ControlError, 'session/scope'):
            self.control.view_decision(job['id'], p['request_sha256'], True, field_confirmation=FIELD)
        self.control.view_decision(job['id'], p['request_sha256'], False)
        self.assertFalse(json.loads((out / 'OBSERVATION_MOVE_APPROVAL.json').read_text())['approved'])

    def test_full_chain_job_never_approves_an_x_move_request(self):
        self.native()
        job = self.control.prepare(execution_scope=FULL_CHAIN)['active']; self.start(job)
        out = Path(job['session_out']); out.mkdir(parents=True)
        rx = native_rx()
        proposal = {'scope': RIGHT_X_MOVE, 'session_out': str(out), 'candidate': rx.move_candidate('F', dict(INPUTS, verify_from='F')),
                    'plan': x_plan(actual(1)), 'actual_feedback': actual(1)}
        (out / 'OBSERVATION_MOVE_REQUEST.json').write_text(json.dumps(proposal, sort_keys=True))
        p = self.pending()
        with self.assertRaisesRegex(ControlError, 'session/scope'):
            self.control.view_decision(job['id'], p['request_sha256'], True, field_confirmation=FIELD)

    def test_adopted_view_request_inside_a_right_x_job_keeps_the_existing_decision(self):
        job, runner, rx = self.running()
        out = Path(job['session_out'])
        (out / 'OBSERVATION_MOVE_REQUEST.json').write_text(json.dumps({'scope': VIEW_MOVE, 'session_out': str(out), 'plan': {}, 'candidate': {}}))
        p = self.pending()
        self.assertNotIn('right_x', p); self.assertNotIn('approval_refused', p)
        self.control.view_decision(job['id'], p['request_sha256'], True)
        decision = json.loads((out / 'OBSERVATION_MOVE_APPROVAL.json').read_text())
        self.assertEqual(decision['scope'], VIEW_MOVE); self.assertNotIn('field_physical_confirmation', decision)

    def test_status_reports_the_measurement_record(self):
        job, runner, rx = self.running()
        atomic_json(Path(job['session_out']) / 'RIGHT_X_METROLOGY.json', {'status': 'DECLINED', 'why': 'V: no explicit operator approval',
                    'endpoints': [{'role': 'T'}, {'role': 'F'}], 'moves': [{'role': 'F', 'plan_ready': True, 'approved': True, 'result': 'HOLDING'}]})
        record = self.control.status()['active']['right_x_record']
        self.assertEqual((record['status'], record['endpoints']), ('DECLINED', ['T', 'F']))


class RightXRecord(unittest.TestCase):
    setUp = test_records.RunEvidence.setUp

    def test_a_completed_measurement_session_is_never_a_success_candidate(self):
        atomic_json(self.directory / 'JOB.json', {'id': self.directory.name, 'consumed': True, 'state': 'RUNNING', 'execution_scope': RIGHT_X})
        atomic_json(self.root / 'RIGHT_X_METROLOGY.json', {'status': 'ENDPOINTS_RECORDED', 'why': 'T, F and V endpoints recorded'})
        record, _ = run_record(self.directory, off_confirmed=True)
        self.assertEqual(record['category'], 'success'); self.assertEqual(record['dataset']['auto_bucket'], 'partial')
        self.assertEqual(record['right_x_metrology']['status'], 'ENDPOINTS_RECORDED')

    def test_full_chain_success_bucket_is_unchanged(self):
        record, _ = run_record(self.directory, off_confirmed=True)
        self.assertEqual(record['dataset']['auto_bucket'], 'success_candidates'); self.assertIsNone(record['right_x_metrology'])


if __name__ == '__main__':
    unittest.main()
