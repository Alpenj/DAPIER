"""Console-17: LEFT_START_TO_LIFT is offered only when the native files carry the limited contract, prepares its own
approval/manifest, starts the same single launcher, and its holding terminal blocks the next run. FakeRunner only."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import unittest
from unittest.mock import patch

import control
import test_control
import test_records
from control import FULL_CHAIN, LEFT_TO_LIFT, ControlError, limited_backend
from records import atomic_json, run_record

NATIVE = Path(os.environ.get('SWEEPICK_CONSOLE17_NATIVE', Path.home() / 'sweepick_261015_console17_native'))


def native_launcher():
    spec = importlib.util.spec_from_file_location('console17_native_launcher', NATIVE / 'RUN_APPROVED_SESSION.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module


class LimitedBackend(unittest.TestCase):
    setUp = test_control.ControlContract.setUp

    def native(self, tested=None):
        patcher = patch.object(control, 'A', NATIVE); patcher.start(); self.addCleanup(patcher.stop)
        record = Path(self.temporary.name) / 'TESTED_NATIVE.json'
        files = {name: hashlib.sha256((NATIVE / name).read_bytes()).hexdigest() for name in control.LIMITED_BOUND_SOURCES}
        files.update(tested or {})
        atomic_json(record, {'files': files})
        tp = patch.object(control, 'TESTED_NATIVE', record); tp.start(); self.addCleanup(tp.stop)
        return record

    def test_marker_alone_is_not_readiness_active_sha_must_equal_tested_source(self):
        self.native(tested={'revision/sweepick_pick01.py': '0' * 64})
        supported, reason = limited_backend()
        self.assertFalse(supported); self.assertIn('revision/sweepick_pick01.py', reason)
        by = {s['id']: s for s in self.control.operation_contract()['scopes']}
        self.assertFalse(by[LEFT_TO_LIFT]['supported'])
        check = self.control.operation_contract()['limited_source_check']
        self.assertFalse(check['all_match']); self.assertFalse(check['files']['revision/sweepick_pick01.py']['match'])
        with self.assertRaisesRegex(ControlError, 'BACKEND_CHANGE_REQUIRED'):
            self.control.prepare(execution_scope=LEFT_TO_LIFT)

    def test_missing_tested_record_is_not_ready(self):
        record = self.native(); record.unlink()
        self.assertFalse(limited_backend()[0]); self.assertIn('시험 기록이 없음', limited_backend()[1])

    def test_start_refuses_a_manifest_that_does_not_bind_the_tested_bytes(self):
        record = self.native()
        job = self.control.prepare(execution_scope=LEFT_TO_LIFT)['active']
        manifest = Path(job['session_out']).parent / 'MANIFEST.json'
        doc = json.loads(manifest.read_text()); doc['files'][str(NATIVE / 'revision/sweepick_pick01.py')] = '1' * 64; manifest.write_text(json.dumps(doc))
        with self.assertRaisesRegex(ControlError, '시험한 native 소스 SHA와 다름'):
            self.start(job)
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))
        self.assertTrue(self.control.operation_contract()['limited_source_check']['all_match'])

    def start(self, job, **k):
        return self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True, **k)

    def test_current_active_native_without_the_contract_stays_disabled(self):
        supported, reason = limited_backend()
        by = {s['id']: s for s in self.control.operation_contract()['scopes']}
        if supported:
            self.skipTest('the active native folder already carries the Console-17 contract')
        self.assertFalse(by[LEFT_TO_LIFT]['supported']); self.assertEqual(by[LEFT_TO_LIFT]['backend_status'], 'BACKEND_CHANGE_REQUIRED')
        # Either the active native lacks the contract, or it is not the source the record says passed the Console-17 tests.
        self.assertRegex(reason, 'Console-17|시험한 소스와 다름')
        with self.assertRaisesRegex(ControlError, 'BACKEND_CHANGE_REQUIRED'):
            self.control.prepare(execution_scope=LEFT_TO_LIFT)

    def test_contract_offers_limited_only_with_the_native_contract(self):
        self.native()
        by = {s['id']: s for s in self.control.operation_contract()['scopes']}
        self.assertTrue(by[LEFT_TO_LIFT]['supported']); self.assertTrue(by[LEFT_TO_LIFT]['can_prepare'])
        self.assertEqual(by[LEFT_TO_LIFT]['backend_status'], 'SUPPORTED_CONSOLE17_LIMITED_BACKEND')
        self.assertIn('CARRY', by[LEFT_TO_LIFT]['excluded_stages']); self.assertIn('LIMITED_SCOPE_TERMINAL', by[LEFT_TO_LIFT]['included_stages'])
        self.assertTrue(by[FULL_CHAIN]['supported'])
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))

    def test_limited_prepare_has_its_own_approval_and_manifest(self):
        self.native()
        job = self.control.prepare(execution_scope=LEFT_TO_LIFT)['active']
        folder = Path(job['session_out']).parent
        manifest = json.loads((folder / 'MANIFEST.json').read_text()); approval = json.loads((folder / 'APPROVAL.json').read_text())
        self.assertEqual((manifest['scope'], manifest['execution_scope']), (LEFT_TO_LIFT, LEFT_TO_LIFT))
        for name in ('RUN_APPROVED_SESSION.py', 'revision/sweepick_pick01.py', 'revision/sweepick_move_a.py', 'revision/sweepick_grasp.py'):
            self.assertIn(str(NATIVE / name), manifest['files'])
        self.assertEqual((approval['scope'], approval['execution_scope']), (LEFT_TO_LIFT, LEFT_TO_LIFT))
        self.assertIs(approval['right_arm_commands_permitted'], False); self.assertIs(approval['automatic_return_after_lift'], False)
        self.assertFalse(approval['approved']); self.assertFalse(approval['permission_to_open_devices'])
        self.assertNotIn('previous_attempt', approval)                 # not copied from the full-chain approval
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))

    def test_console_files_pass_the_native_limited_launch_check_after_start(self):
        self.native()
        job = self.control.prepare(execution_scope=LEFT_TO_LIFT)['active']
        self.start(job, execution_scope=LEFT_TO_LIFT)
        launches = [c for c in self.runner.calls if c[:2] == ['tmux', 'new-session']]
        self.assertEqual(len(launches), 1); self.assertIn('RUN_APPROVED_SESSION.py --execute --managed-tmux', launches[0][-1])
        folder = Path(job['session_out']).parent
        runner = native_launcher()
        with patch.object(runner, 'APPROVAL', folder / 'APPROVAL.json'), patch.object(runner, 'MANIFEST', folder / 'MANIFEST.json'), \
                patch.dict(os.environ, {'SWEEPICK_SESSION_OUT': job['session_out']}):
            approval, plan, out, manifest = runner.validate_launch()
        self.assertEqual(runner.execution_scope(approval), LEFT_TO_LIFT); self.assertEqual(str(out), job['session_out'])

    def test_full_chain_approval_cannot_open_the_limited_scope_even_when_supported(self):
        self.native()
        job = self.control.prepare(execution_scope=FULL_CHAIN)['active']
        with self.assertRaisesRegex(ControlError, '범위'):
            self.start(job, execution_scope=LEFT_TO_LIFT)
        job['execution_scope'] = LEFT_TO_LIFT; self.control.save(job)     # a forged scope on a full-chain approval/manifest
        with self.assertRaisesRegex(ControlError, '범위가 실행 범위와 다름'):
            self.start(job)
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))
        folder = Path(job['session_out']).parent
        approval = json.loads((folder / 'APPROVAL.json').read_text()); approval.update(approved=True, field_ready=True, permission_to_execute=True)
        (folder / 'APPROVAL.json').write_text(json.dumps(approval))
        runner = native_launcher()
        approval.update(execution_scope=LEFT_TO_LIFT); (folder / 'APPROVAL.json').write_text(json.dumps(approval))
        with patch.object(runner, 'APPROVAL', folder / 'APPROVAL.json'), patch.object(runner, 'MANIFEST', folder / 'MANIFEST.json'), \
                patch.dict(os.environ, {'SWEEPICK_SESSION_OUT': job['session_out']}), self.assertRaises(RuntimeError):
            runner.validate_launch()

    def test_limited_job_cannot_be_started_as_full_chain(self):
        self.native()
        job = self.control.prepare(execution_scope=LEFT_TO_LIFT)['active']
        with self.assertRaisesRegex(ControlError, '범위'):
            self.start(job, execution_scope=FULL_CHAIN)
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))

    def ended(self, job, terminal, exit_code=0):
        self.runner.alive = False
        out = Path(job['session_out']); (out / 'review').mkdir(parents=True, exist_ok=True)
        (out / 'review/lifetime.jsonl').write_text(json.dumps({'event': 'owner_end', 'phase': 'ended', 'exit_code': exit_code}) + '\n')
        if terminal:
            atomic_json(out / 'LIMITED_SCOPE_TERMINAL.json', {'terminal': terminal, 'normal_home': False, 'torque_off_confirmed': False,
                        'safe_shutdown': 'NOT_PERFORMED', 'operator_required': True, 'next_run_allowed': False,
                        'last_stage': {'name': 'lift', 'controller_result': 'HOLDING'}, 'last_actual': {'torque': {'gripper': 1}}})

    def test_held_terminal_blocks_the_next_run_until_manual_recovery(self):
        self.native()
        job = self.control.prepare(execution_scope=LEFT_TO_LIFT)['active']; self.start(job)
        self.ended(job, 'LEFT_LIFT_HELD_CONFIRMED')
        status = self.control.status()
        self.assertEqual(status['state'], 'LIMITED_HOLDING'); self.assertFalse(status['can_prepare']); self.assertFalse(status['can_start'])
        self.assertTrue(status['can_confirm_recovery']); self.assertIn('정상 HOME/OFF 아님', status['block_reason'])
        self.assertEqual(status['active']['limited_terminal']['terminal'], 'LEFT_LIFT_HELD_CONFIRMED')
        with self.assertRaises(ControlError):
            self.control.prepare(execution_scope=LEFT_TO_LIFT)
        self.control.confirm_recovery(job['id'], power_isolated=True, workspace_reset=True)
        self.assertTrue(self.control.status()['can_prepare'])

    def test_servo_or_bus_error_wins_over_a_held_terminal_in_status_and_record(self):
        self.native()
        cases = (('none', None, 'LIMITED_HOLDING'),
                 ('feed', {'first_error': {'gate': 'BUS_LOST', 'side': 'left', 'detail': 'ID=6 error=0x20 Overload', 'time_s': 101.0}}, 'FAULT'))
        for name, feed, expected in cases:
            with self.subTest(case=name):
                self.setUp(); self.native()
                job = self.control.prepare(execution_scope=LEFT_TO_LIFT)['active']; self.start(job)
                self.ended(job, 'LEFT_LIFT_HELD_CONFIRMED')
                if feed:
                    atomic_json(Path(job['session_out']) / 'CURRENT_FEEDBACK.json', feed)
                status = self.control.status()
                record, _ = run_record(Path(job['session_out']).parent, current=status['active'], owner_alive=False, off_confirmed=False)
                self.assertEqual(status['state'], expected); self.assertEqual(record['state'], expected)
                self.assertFalse(status['can_prepare']); self.assertNotEqual(record['category'], 'success')
                if feed:
                    self.assertIn('같은 세션 오류 우선', status['block_reason']); self.assertEqual(record['first_error']['gate'], 'BUS_LOST')

    def test_missing_or_fault_terminal_is_a_fault_not_a_hold(self):
        self.native()
        for terminal in (None, 'BUS_OR_SERVO_FAULT', 'LEFT_CLOSE_UNAVAILABLE', 'OPERATOR_STOPPED'):
            with self.subTest(terminal=terminal):
                self.setUp(); self.native()
                job = self.control.prepare(execution_scope=LEFT_TO_LIFT)['active']; self.start(job)
                self.ended(job, terminal, exit_code=3)
                status = self.control.status()
                self.assertEqual(status['state'], 'FAULT'); self.assertFalse(status['can_prepare'])


class LimitedRecord(unittest.TestCase):
    setUp = test_records.RunEvidence.setUp

    def test_holding_record_is_partial_never_home_or_safe_shutdown(self):
        job = json.loads((self.directory / 'JOB.json').read_text()) if (self.directory / 'JOB.json').exists() else {'id': self.directory.name}
        job.update(execution_scope=LEFT_TO_LIFT, consumed=True); atomic_json(self.directory / 'JOB.json', job)
        (self.root / 'review').mkdir(exist_ok=True)
        (self.root / 'review/lifetime.jsonl').write_text(json.dumps({'event': 'owner_end', 'exit_code': 0}) + '\n')
        atomic_json(self.root / 'LIMITED_SCOPE_TERMINAL.json', {'terminal': 'LEFT_LIFT_HELD_CONFIRMED', 'normal_home': False})
        record, _ = run_record(self.directory)
        self.assertEqual(record['execution_scope'], LEFT_TO_LIFT)
        self.assertNotEqual(record['category'], 'success'); self.assertFalse(record['normal_return_confirmed'])
        self.assertEqual(record['limited_terminal']['terminal'], 'LEFT_LIFT_HELD_CONFIRMED')
        self.assertEqual(record['state'], 'LIMITED_HOLDING'); self.assertEqual(record['category'], 'partial')
        self.assertEqual(record['safe_shutdown'], 'NOT_PERFORMED')

    def test_exit_three_with_a_terminal_is_not_an_executor_error(self):
        job = {'id': self.directory.name, 'execution_scope': LEFT_TO_LIFT, 'consumed': True}; atomic_json(self.directory / 'JOB.json', job)
        (self.root / 'review').mkdir(exist_ok=True)
        (self.root / 'review/lifetime.jsonl').write_text(json.dumps({'event': 'owner_end', 'exit_code': 3}) + '\n')
        atomic_json(self.root / 'LIMITED_SCOPE_TERMINAL.json', {'terminal': 'LEFT_LIFT_HOLD_UNKNOWN'})
        record, _ = run_record(self.directory)
        self.assertFalse(record['first_error'] and record['first_error'].get('gate') == 'EXECUTOR_ERROR')
        self.assertEqual(record['limited_terminal']['terminal'], 'LEFT_LIFT_HOLD_UNKNOWN')

    def test_untimed_final_exception_does_not_become_the_first_error_time(self):
        """20261015_112218_f0d866: the owner_end exception had no host time and was given 0, so the export picked the session start"""
        from records import save_review
        atomic_json(self.root / 'CURRENT_FEEDBACK.json', {'first_error': {'gate': 'BUS_LOST', 'side': 'left', 'detail': 'ID=6 error=0x20 Overload', 'time_s': 100.115}})
        (self.root / 'review').mkdir(exist_ok=True)
        (self.root / 'review/lifetime.jsonl').write_text(json.dumps({'event': 'owner_end', 'exit_code': 1, 'timestamp': '2026-10-15T02:24:05.741204+00:00',
            'error': "RuntimeError: FeetechMotorsBus motor check failed on port '/dev/dapier/left_arm': Missing motor IDs: 6"}) + '\n')
        before = {q: q.read_bytes() for q in self.root.rglob('*') if q.is_file()}
        record, evidence = run_record(self.directory)
        save_review(self.directory, record, evidence)
        packet = json.loads((self.directory / 'FAILURE_PACKET.json').read_text())
        self.assertEqual(packet['failure_time_s'], 100.115); self.assertEqual(packet['failure_time_status'], 'RECORDED')
        self.assertEqual(json.loads((self.directory / 'RUN_RECORD.json').read_text())['first_error']['time_s'], 100.115)
        kinds = [row['kind'] for row in packet['failure_sequence']]
        self.assertEqual(kinds, ['FIRST_SERVO_OR_BUS_ERROR', 'HANDSHAKE_FAILURE'])
        self.assertGreater(packet['failure_sequence'][1]['time_s'], 0)
        frame = packet['frames']['left_wrist']
        self.assertTrue(frame['before']['timestamp_s'] is None or frame['before']['timestamp_s'] <= 100.115)
        self.assertTrue(frame['after']['timestamp_s'] is None or frame['after']['timestamp_s'] > 100.115)
        self.assertEqual({q: q.read_bytes() for q in before}, before)          # originals unchanged

    def test_no_recorded_failure_time_stays_unknown(self):
        from records import save_review
        (self.root / 'review').mkdir(exist_ok=True)
        (self.root / 'review/lifetime.jsonl').write_text(json.dumps({'event': 'owner_end', 'exit_code': 1, 'error': 'RuntimeError: x'}) + '\n')
        record, evidence = run_record(self.directory)
        save_review(self.directory, record, evidence)
        packet = json.loads((self.directory / 'FAILURE_PACKET.json').read_text())
        self.assertIsNone(packet['failure_time_s']); self.assertEqual(packet['failure_time_status'], 'UNKNOWN_NOT_RECORDED')
        self.assertTrue(all(v[side]['status'] == 'UNKNOWN' for v in packet['frames'].values() for side in ('before', 'after')))

    def test_full_chain_record_has_no_limited_terminal(self):
        record, _ = run_record(self.directory)
        self.assertIsNone(record['limited_terminal'])


if __name__ == '__main__':
    unittest.main()
