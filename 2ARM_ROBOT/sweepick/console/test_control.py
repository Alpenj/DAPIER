import importlib.util
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace, ModuleType
import tempfile
import time
import unittest
from unittest.mock import patch

from control import Controller, ControlError, A, MANAGED
from live import LiveView
from replay import JOINTS


class FakeRunner:
    def __init__(self):
        self.alive = False
        self.foreign = ''
        self.calls = []

    def __call__(self, argv, **kwargs):
        self.calls.append(argv)
        if argv[0] == 'fuser':
            return SimpleNamespace(returncode=0 if self.foreign else 1, stdout=self.foreign, stderr='')
        if argv[:2] == ['tmux', 'has-session']:
            return SimpleNamespace(returncode=0 if self.alive else 1, stdout='', stderr='')
        if argv[:2] == ['tmux', 'new-session']:
            self.alive = True
        if argv[:2] == ['tmux', 'display-message']:
            return SimpleNamespace(returncode=0, stdout=str(os.getpid()), stderr='')
        return SimpleNamespace(returncode=0, stdout='', stderr='')


class ControlContract(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.runner = FakeRunner()
        self.control = Controller(self.temporary.name, run=self.runner)
        # Storage fidelity is tested with real small files in test_records; controller tests isolate the launch boundary.
        captured = patch('control.capture_sources', return_value={'fixture.py': {'sha256': 'fixture'}})
        self.capture = captured.start()
        self.addCleanup(captured.stop)
        # Device topology is a fixture here (test_wrist_device_check covers the real resolution); fuser stays the FakeRunner.
        devices = patch('control.resolve_devices', return_value={'left_arm': '/dev/ttyACM1', 'right_arm': '/dev/ttyACM0', 'top': '/dev/bus/usb/005/002',
                                                                  'left_wrist': '/dev/video10', 'right_wrist': '/dev/video8'})
        devices.start(); self.addCleanup(devices.stop)

    def prepare(self):
        return self.control.prepare('session_18')['active']

    def start(self, job):
        return self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True)

    def test_prepare_does_not_start_or_approve_and_keeps_current_sources(self):
        job = self.prepare()
        self.assertFalse(job['consumed'])
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))
        manifest = json.loads((Path(job['session_out']).parent / 'MANIFEST.json').read_text())
        self.assertIn(str(A / 'revision/sweepick_move_a.py'), manifest['files'])
        approval = json.loads((Path(job['session_out']).parent / 'APPROVAL.json').read_text())
        self.assertFalse(approval['approved'])
        self.assertEqual(approval['session_out'], job['session_out'])
        self.assertFalse(Path(job['session_out']).exists())

    def test_fresh_session_needs_no_archive_and_survives_controller_restart(self):
        job = self.control.prepare()['active']
        self.assertIsNone(job['source_session'])
        self.assertEqual(job['preparation'], 'FRESH_SESSION')
        self.assertFalse(job['consumed'])
        self.assertTrue((Path(job['session_out']).parent / 'SOURCE_SNAPSHOT.json').exists())
        restarted = Controller(self.temporary.name, run=self.runner)
        self.assertEqual(restarted.status()['active']['id'], job['id'])
        records = restarted.runs()
        self.assertEqual(records[0]['id'], job['id'])
        self.assertEqual(records[0]['recording']['status'], 'NOT_STARTED')
        self.assertFalse(any(call[:2] == ['tmux', 'new-session'] for call in self.runner.calls))

    def test_source_storage_failure_cannot_publish_prepared_or_start(self):
        self.capture.side_effect = OSError('No space left on device')
        with self.assertRaisesRegex(ControlError, '저장 준비 실패'):
            self.control.prepare()
        self.assertFalse(self.control.state_file.exists())
        self.assertFalse(any(call[:2] == ['tmux', 'new-session'] for call in self.runner.calls))

    def test_run_paths_reject_traversal_and_unknown_jobs(self):
        for value in ('../CONTROL', '/etc/passwd', '', None, ['bad']):
            self.assertIsNone(self.control.run_directory(value))
        with self.assertRaises(ControlError):
            self.control.review_file('../CONTROL')

    def test_corrupt_one_run_does_not_hide_other_run_or_claim_release(self):
        damaged = self.control.directory / '20261015_090000_abcdef'
        (damaged / 'session/episode').mkdir(parents=True)
        (damaged / 'APPROVAL.json').write_text('{}')
        (damaged / 'session/episode/EPISODE.json').write_text('{"stages":null,"data_quality":[]}')
        valid = self.control.prepare()['active']
        rows = self.control.runs()
        self.assertEqual(len(rows), 2)
        self.assertIn(valid['id'], [row['id'] for row in rows])
        broken = next(row for row in rows if row['id'] == damaged.name)
        self.assertEqual(broken['recording']['status'], 'INCOMPLETE')
        self.assertFalse(broken['normal_return_confirmed'])
        self.assertEqual(broken['safe_shutdown'], 'UNKNOWN')
        self.assertTrue(self.control.review_file(damaged.name).exists())

    def test_malformed_replay_does_not_break_session_catalog(self):
        import app
        from fastapi import HTTPException
        damaged = self.control.directory / '20261015_090000_abcdef'
        (damaged / 'session/episode').mkdir(parents=True)
        (damaged / 'APPROVAL.json').write_text('{}')
        (damaged / 'session/episode/EPISODE.json').write_text('{"data_quality":null}')
        (damaged / 'session/episode/events.jsonl').write_text('{"kind":"episode_start","t_host_s":100}\n')
        with patch.object(app, 'controller', self.control):
            with self.assertRaises(HTTPException) as error:
                app.get_session(damaged.name)
            self.assertEqual(error.exception.status_code, 422)
            self.assertGreaterEqual(len(app.sessions()['sessions']), 4)
        self.assertIsNone(self.control.runs()[0]['replay_session_id'])

    def test_completed_history_reuses_verified_record_until_inputs_change(self):
        job = self.control.prepare()['active']
        first = self.control.runs()[0]
        with patch('control.run_record', side_effect=AssertionError('must reuse completed index')):
            self.assertEqual(self.control.runs()[0]['verified_at'], first['verified_at'])
        path = Path(job['session_out']).parent / 'APPROVAL.json'
        approval = json.loads(path.read_text())
        approval['timestamp'] = 'changed'
        path.write_text(json.dumps(approval))
        with patch('control.run_record', wraps=__import__('records').run_record) as rebuilt:
            self.control.runs()
            self.assertEqual(rebuilt.call_count, 1)

    def test_start_requires_all_explicit_boolean_conditions(self):
        job = self.prepare()
        for kwargs in ({}, {'field_ready': 'true', 'mounts_unchanged': True, 'supervised': True}):
            with self.assertRaises(ControlError):
                self.control.start(job['id'], **kwargs)
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))

    def test_single_start_passes_only_existing_launcher_and_cannot_reuse(self):
        job = self.prepare()
        status = self.start(job)
        self.assertEqual(status['state'], 'RUNNING')
        calls = [c for c in self.runner.calls if c[:2] == ['tmux', 'new-session']]
        self.assertEqual(len(calls), 1)
        self.assertIn(str(A / 'RUN_APPROVED_SESSION.py'), calls[0][-1])
        self.assertIn('--execute --managed-tmux', calls[0][-1])
        self.assertIn('SWEEPICK_SESSION_OUT=', calls[0][-1])
        with self.assertRaises(ControlError):
            self.start(job)
        with self.assertRaises(ControlError):
            self.control.prepare('observation_08')

    def test_foreign_device_owner_blocks_without_second_owner(self):
        job = self.prepare()
        self.runner.foreign = '12345'
        with self.assertRaisesRegex(ControlError, 'owner'):
            self.start(job)
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))

    def test_stop_only_sets_existing_latch_and_control_c_not_torque(self):
        job = self.prepare()
        self.start(job)
        result = self.control.stop(job['id'])
        self.assertEqual(result['state'], 'STOP_REQUESTED')
        self.assertFalse(result['can_stop'])
        self.assertTrue(Path(job['session_out'] + '.STOP').exists())
        self.assertIn(['tmux', 'send-keys', '-t', MANAGED, 'C-c'], self.runner.calls)
        self.assertFalse(any('Torque_Enable' in ' '.join(c) for c in self.runner.calls))

    def test_reused_managed_name_with_different_process_identity_cannot_be_stopped(self):
        job = self.prepare();self.start(job)
        saved = json.loads(self.control.state_file.read_text())
        saved['owner_start'] = 'different-process-start'
        self.control.save(saved)
        self.assertEqual(self.control.status()['state'], 'BLOCKED')
        with self.assertRaises(ControlError):
            self.control.stop(job['id'])
        self.assertFalse(any(c[:2] == ['tmux', 'send-keys'] for c in self.runner.calls))

    def test_missing_terminal_release_is_fault_even_when_ui_feed_says_zero(self):
        job = self.prepare()
        self.start(job)
        root = Path(job['session_out']);root.mkdir()
        zeros = {n: 0 for n in JOINTS}
        stale_feed = {'sides': {side: {k: {'time_s': 1, 'values': zeros} for k in ('torque', 'status', 'moving', 'velocity')}
                                for side in ('left', 'right')}}
        (root / 'CURRENT_FEEDBACK.json').write_text(json.dumps(stale_feed))
        self.runner.alive = False
        status = self.control.status()
        self.assertEqual(status['state'], 'FAULT')
        self.assertFalse(status['can_prepare'])
        with self.assertRaises(ControlError):
            self.control.prepare('session_18')

    def test_normal_native_release_ledger_allows_new_preparation(self):
        job = self.prepare();self.start(job)
        root = Path(job['session_out']);(root / 'episode').mkdir(parents=True)
        stages = []
        for side in ('left', 'right'):
            folder = root / side;(folder / 'run1').mkdir(parents=True)
            stages.append({'side': side, 'result': 'RETURNED_RELEASED', 'folder': str(folder)})
            (folder / 'run1/move.json').write_text(json.dumps({'result': 'RETURNED_RELEASED', 'outcome': {'torque_released': True},
                'end': {k: {n: 0 for n in JOINTS} for k in ('torque', 'status', 'moving', 'velocity')}}))
        (root / 'episode/EPISODE.json').write_text(json.dumps({'stages': stages}))
        self.runner.alive = False
        self.assertEqual(self.control.status()['state'], 'ENDED')
        self.assertTrue(self.control.status()['can_prepare'])

    def test_extra_view_decision_bound_to_current_request_digest(self):
        job = self.prepare();self.start(job)
        root = Path(job['session_out']);root.mkdir()
        request = {'scope': 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION_MOVE', 'plan': {'actual': 'current'}, 'candidate': {'id': 'real'}}
        (root / 'OBSERVATION_MOVE_REQUEST.json').write_text(json.dumps(request))
        pending = self.control.status()['active']['pending_view']
        with self.assertRaises(ControlError):
            self.control.view_decision(job['id'], 'old-digest', True)
        self.control.view_decision(job['id'], pending['request_sha256'], False)
        decision = json.loads((root / 'OBSERVATION_MOVE_APPROVAL.json').read_text())
        self.assertFalse(decision['approved'])
        self.assertIsNone(self.control.status()['active']['pending_view'])

    def test_partial_view_request_is_unknown_and_does_not_approve(self):
        job = self.prepare();self.start(job)
        root = Path(job['session_out']);root.mkdir()
        (root / 'OBSERVATION_MOVE_REQUEST.json').write_text('{"scope":')
        active = self.control.status()['active']
        self.assertIsNone(active['pending_view'])
        self.assertIn('UNKNOWN', active['pending_view_state'])
        self.assertFalse((root / 'OBSERVATION_MOVE_APPROVAL.json').exists())

    def test_long_hold_journal_keeps_stages_and_stops_approval_at_native_fault(self):
        job = self.prepare();self.start(job)
        root = Path(job['session_out']);(root / 'episode').mkdir(parents=True)
        rows = [{'kind': 'phase', 'step': 'close', 't_host_s': 10},
                {'kind': 'stage_record', 'stage': 'close', 'side': 'left', 'result': 'GRIP_CONTACT_HOLDING', 't_host_s': 11},
                {'kind': 'phase', 'step': 'handoff_approach_right_approach_right', 't_host_s': 12},
                {'kind': 'stage_record', 'stage': 'handoff_approach_right_approach', 'side': 'right', 'result': 'HOLDING', 't_host_s': 13}]
        events = root / 'episode/events.jsonl'
        events.write_text('\n'.join(map(json.dumps, rows)) + '\n')
        view = LiveView(self.control)
        self.assertEqual(view.snapshot()['stage_results'][0]['status'], 'GRIP_CONTACT_HOLDING')
        with events.open('a') as stream:
            for i in range(500):
                stream.write(json.dumps({'kind': 'left_hold_recovery_ready', 'detail': 'x' * 500, 't_host_s': 14}) + '\n')
            stream.write(json.dumps({'kind': 'observation_plan_authorization_failed',
                'reason': 'no valid fresh left wrist frame', 't_host_s': 15}) + '\n')
        (root / 'OBSERVATION_MOVE_REQUEST.json').write_text(json.dumps({'scope': 'extra view'}))
        status = self.control.status()
        self.assertTrue(status['owner_alive'])
        self.assertEqual(status['state'], 'FAULT')
        self.assertIsNone(status['active']['pending_view'])
        with self.assertRaises(ControlError):
            self.control.view_decision(job['id'], 'any-digest', True)
        snap = view.snapshot()
        self.assertEqual(snap['stage_results'][0]['status'], 'GRIP_CONTACT_HOLDING')
        self.assertEqual(snap['stage_results'][3]['status'], 'HOLDING')
        self.assertEqual(snap['current_stage'], 'STOPPED_NEEDS_OPERATOR')
        self.assertEqual(snap['command']['first_error']['gate'], 'LEFT_HOLD_VISION_UNCONFIRMED')
        self.assertEqual(len(self.control.episode_events(root)), 5)

    def test_partial_appended_event_is_read_once_after_completion(self):
        job = self.prepare();root = Path(job['session_out']);(root / 'episode').mkdir(parents=True)
        events = root / 'episode/events.jsonl'
        events.write_text('{"kind":"owner_stop","reasons":')
        self.assertEqual(self.control.episode_events(root), [])
        with events.open('a') as stream:stream.write('["STOP"],"t_host_s":20}\n')
        self.assertEqual(len(self.control.episode_events(root)), 1)
        self.assertEqual(len(self.control.episode_events(root)), 1)

    def test_later_operator_off_is_separate_from_original_failed_shutdown(self):
        job = self.prepare();self.start(job)
        root = Path(job['session_out']);(root / 'episode').mkdir(parents=True);(root / 'review').mkdir()
        (root / 'episode/events.jsonl').write_text(json.dumps({'kind': 'owner_stop', 'reasons': ['lost left support'], 't_host_s': 10})+'\n')
        recovery = {'finished': '2026-10-14T16:15:00+09:00', 'all_off_confirmed': True, 'scope': 'user requested torque OFF',
                    'sides': {side: {'after': {'answered': True, 'state': {k: {n: 0 for n in JOINTS}
                        for k in ('torque', 'status', 'moving', 'present', 'goal')}}} for side in ('left', 'right')}}
        (root / 'review/TORQUE_OFF_OPERATOR_REQUEST.json').write_text(json.dumps(recovery))
        self.runner.alive = False
        snap = LiveView(self.control).snapshot()
        self.assertTrue(all(row['torque'] == 0 for row in snap['joints']))
        self.assertTrue(all('POST_RECOVERY' in row['feedback_status'] for row in snap['joints']))
        self.assertTrue(snap['command']['later_recovery']['all_left_off'])
        self.assertEqual(snap['command']['safe_shutdown'], 'UNKNOWN')
        self.assertEqual(self.control.status()['state'], 'FAULT')

    def stopped_job(self):
        job = self.prepare();self.start(job)
        root = Path(job['session_out']);(root / 'episode').mkdir(parents=True)
        (root / 'episode/events.jsonl').write_text(json.dumps({'kind': 'owner_stop', 'reasons': ['BUS_LOST ID3'], 't_host_s': 10})+'\n')
        self.runner.alive = False
        return job

    def test_manual_recovery_requires_current_operator_confirmation_and_ended_owner(self):
        job = self.stopped_job()
        for kwargs in ({}, {'power_isolated': 'true', 'workspace_reset': True}, {'power_isolated': True}):
            with self.assertRaises(ControlError):
                self.control.confirm_recovery(job['id'], **kwargs)
        self.runner.alive = True
        with self.assertRaises(ControlError):
            self.control.confirm_recovery(job['id'], power_isolated=True, workspace_reset=True)
        self.runner.alive = False;self.runner.foreign = 'OTHER_DEVICE_OWNER'
        with self.assertRaises(ControlError):
            self.control.confirm_recovery(job['id'], power_isolated=True, workspace_reset=True)
        self.assertFalse((Path(job['session_out']) / 'review/MANUAL_RECOVERY_CONFIRMATION.json').exists())

    def test_manual_recovery_keeps_failure_and_unknown_then_prepares_new_unapproved_job(self):
        job = self.stopped_job()
        original_error = self.control.status()['active']['first_error']
        status = self.control.confirm_recovery(job['id'], power_isolated=True, workspace_reset=True)
        self.assertEqual(status['state'], 'FAULT')
        self.assertEqual(status['active']['first_error'], original_error)
        self.assertTrue(status['can_prepare']);self.assertFalse(status['can_start'])
        self.assertFalse(status['can_confirm_recovery'])
        record = status['active']['operator_recovery']
        self.assertEqual(record['torque_readback'], 'UNKNOWN');self.assertFalse(record['normal_return_confirmed'])
        self.assertEqual(record['motor_writes'], 0)
        self.assertEqual(LiveView(self.control).snapshot()['command']['safe_shutdown'], 'UNKNOWN')
        new = self.prepare()
        self.assertNotEqual(new['id'], job['id']);self.assertFalse(new['consumed'])
        directory = Path(new['session_out']).parent
        self.assertFalse(json.loads((directory / 'APPROVAL.json').read_text())['approved'])
        self.assertFalse(Path(new['session_out']).exists())
        previous = json.loads((Path(job['session_out']).parent / 'CONTROL_FINAL.json').read_text())
        self.assertEqual(previous['state'], 'FAULT');self.assertEqual(previous['first_error'], original_error)
        self.assertEqual(sum(call[:2] == ['tmux', 'new-session'] for call in self.runner.calls), 1)

    def test_manual_confirmation_for_another_fault_or_malformed_checkbox_cannot_unlock(self):
        job = self.stopped_job()
        self.control.confirm_recovery(job['id'], power_isolated=True, workspace_reset=True)
        path = Path(job['session_out']) / 'review/MANUAL_RECOVERY_CONFIRMATION.json'
        valid = json.loads(path.read_text())
        for patch_values in ({'job_id': 'old-job'}, {'power_isolated': 'true'}, {'first_error': {'gate': 'old-fault'}}):
            path.write_text(json.dumps({**valid, **patch_values}))
            self.assertFalse(self.control.status()['can_prepare'])

    def test_live_short_cadence_and_future_receipt_not_faked_fresh(self):
        job = self.prepare();self.start(job)
        root = Path(job['session_out']);(root / 'episode').mkdir(parents=True)
        now = time.time()
        rows = [{'camera': 'left_wrist', 'file': 'left.jpg', 'n': i, 'recv_host_s': now - .1 + i * .033}
                for i in range(3)]
        (root / 'episode/frames.jsonl').write_text('\n'.join(map(json.dumps, rows)) + '\n')
        (root / 'episode/left.jpg').write_bytes(b'fixture indexed media')
        view = LiveView(self.control)
        snap = view.snapshot()
        self.assertEqual(snap['cameras'][2]['status'], 'LIVE')
        self.assertAlmostEqual(snap['cameras'][2]['freshness_limit_s'], .066, places=5)
        self.assertEqual(snap['joints'][0]['actual'], 'UNKNOWN')
        rows[-1]['recv_host_s'] = now + 50
        (root / 'episode/frames.jsonl').write_text('\n'.join(map(json.dumps, rows)) + '\n')
        self.assertEqual(view.snapshot()['cameras'][2]['status'], 'UNKNOWN')
        with self.assertRaises(ControlError):
            view.media(job['id'], '../../etc/passwd')

    def test_live_current_phase_is_running_until_last_required_stage(self):
        job = self.prepare();self.start(job)
        root = Path(job['session_out']);(root / 'episode').mkdir(parents=True)
        rows = [{'kind': 'phase', 'step': 'pregrasp', 't_host_s': 10},
                {'kind': 'stage_record', 'stage': 'pregrasp', 'side': 'left', 'result': 'HOLDING', 't_host_s': 11},
                {'kind': 'phase', 'step': 'descend', 't_host_s': 12}]
        path = root / 'episode/events.jsonl'
        path.write_text('\n'.join(map(json.dumps, rows)))
        stages = LiveView(self.control).snapshot()['stage_results']
        self.assertEqual(stages[0]['status'], 'RUNNING')
        self.assertEqual(stages[0]['ended_s'], 'UNKNOWN')
        rows += [{'kind': 'stage_record', 'stage': 'close', 'side': 'left', 'result': 'GRIP_CONTACT_HOLDING', 't_host_s': 13},
                 {'kind': 'phase', 'step': 'after_close', 't_host_s': 13.00006}]
        path.write_text('\n'.join(map(json.dumps, rows)))
        self.assertEqual(LiveView(self.control).snapshot()['stage_results'][0]['status'], 'GRIP_CONTACT_HOLDING')
        rows.append({'kind': 'phase', 'step': 'lift', 't_host_s': 14})
        path.write_text('\n'.join(map(json.dumps, rows)))
        stages = LiveView(self.control).snapshot()['stage_results']
        self.assertEqual(stages[0]['status'], 'GRIP_CONTACT_HOLDING')
        self.assertEqual(stages[1]['status'], 'RUNNING')
        rows += [{'kind': 'stage_record', 'stage': 'lift', 'side': 'left', 'result': 'HOLDING', 't_host_s': 14.5},
                 {'kind': 'phase', 'step': 'after_lift', 't_host_s': 14.500025}]
        path.write_text('\n'.join(map(json.dumps, rows)))
        self.assertEqual(LiveView(self.control).snapshot()['stage_results'][1]['status'], 'HOLDING')
        rows.append({'kind': 'phase', 'step': 'handoff_approach_right_approach_right', 't_host_s': 15})
        path.write_text('\n'.join(map(json.dumps, rows)))
        self.assertEqual(LiveView(self.control).snapshot()['stage_results'][3]['status'], 'RUNNING')

    def test_unknown_ack_stays_unknown_even_when_goal_register_matches(self):
        job = self.prepare();self.start(job)
        root = Path(job['session_out']);root.mkdir()
        feed = {'sides': {'left': {'goal': {'time_s': 20, 'values': {'elbow_flex': 3142}}}},
                'commands': {'left': {'elbow_flex': {'requested_goal': 3142, 'ack': 'UNKNOWN', 'time_s': 19, 'id': 3}}}}
        (root / 'CURRENT_FEEDBACK.json').write_text(json.dumps(feed))
        self.assertEqual(LiveView(self.control).snapshot()['joints'][2]['application'], 'UNKNOWN')

    def test_native_camera_error_and_partial_depth_do_not_look_live(self):
        job = self.prepare();self.start(job)
        root = Path(job['session_out']);(root / 'episode').mkdir(parents=True)
        now = time.time()
        rows = [{'camera': 'top', 'file': 'top.jpg', 'depth_file': 'depth.npy', 'n': i,
                 'recv_host_s': now - .03 + i * .02, 'recv_monotonic_s': 30 + i * .02} for i in range(2)]
        (root / 'episode/frames.jsonl').write_text('\n'.join(map(json.dumps, rows)))
        (root / 'episode/top.jpg').write_bytes(b'fixture indexed image')
        (root / 'episode/depth.npy').write_bytes(b'partial raw depth')
        (root / 'episode/EPISODE.json').write_text(json.dumps({'readers_closed': {'SdkTop': {
            'ended_before_owner_stop': True, 'reader_ended_monotonic_s': 30.03, 'reader_error': 'native SDK receive error'}}}))
        snap = LiveView(self.control).snapshot()
        self.assertEqual(snap['cameras'][0]['status'], 'STALE')
        self.assertEqual(snap['cameras'][0]['connection'], 'RECORDED_ERROR')
        self.assertEqual(snap['cameras'][1]['status'], 'NOT_AVAILABLE')
        self.assertEqual(snap['command']['first_error']['gate'], 'CAMERA_READER_ERROR')

    def test_generated_approval_binds_current_native_launcher_without_running_it(self):
        job = self.prepare()
        directory = Path(job['session_out']).parent
        env = {'SWEEPICK_START_APPROVAL_PATH': str(directory / 'APPROVAL.json'),
               'SWEEPICK_SOURCE_MANIFEST_PATH': str(directory / 'MANIFEST.json'),
               'SWEEPICK_SESSION_OUT': job['session_out'], 'SWEEPICK_MANAGED_SESSION': MANAGED}
        with patch.dict(os.environ, env), patch.object(sys, 'path', sys.path.copy()):
            spec = importlib.util.spec_from_file_location('native_launcher_test', A / 'RUN_APPROVED_SESSION.py')
            module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            with self.assertRaisesRegex(RuntimeError, 'approval incomplete'):
                module.validate_launch()
            approval = json.loads((directory / 'APPROVAL.json').read_text())
            approval.update(approved=True, field_ready=True, permission_to_execute=True, permission_to_open_devices=True)
            (directory / 'APPROVAL.json').write_text(json.dumps(approval))
            _, plan, out, manifest = module.validate_launch()
            self.assertEqual(str(out), job['session_out'])
            self.assertEqual(plan['session_out'], job['session_out'])
            self.assertEqual(module.TMUX_SESSION, MANAGED)
            self.assertIn(str(A / 'revision/console_feedback.py'), manifest['files'])
            self.assertFalse(out.exists())


class FeedContract(unittest.TestCase):
    def test_feed_failure_invalidates_old_positions_and_unknown_ack_is_retained(self):
        spec = importlib.util.spec_from_file_location('console_feed', A / 'revision/console_feedback.py')
        module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            feed = module.Feed(directory)
            feed.read('left', 'Present_Position', {n: 100 for n in JOINTS})
            feed.command('left', 'elbow_flex', 3142, 'UNKNOWN')
            feed.failed('left', 'no status packet')
            data = json.loads((Path(directory) / 'CURRENT_FEEDBACK.json').read_text())
            self.assertEqual(data['sides']['left'], {})
            self.assertEqual(data['commands']['left']['elbow_flex']['ack'], 'UNKNOWN')
            self.assertEqual(data['first_error']['gate'], 'BUS_LOST')

    def test_optional_feed_preserves_original_calls_returns_and_exceptions(self):
        spec = importlib.util.spec_from_file_location('console_feed_wrapper', A / 'revision/console_feedback.py')
        module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        class Bus:
            motors = {'elbow_flex': SimpleNamespace(id=3)}
            error = None
            calls = []
            def write(self, *args, **kwargs):
                self.calls.append(('write', args, kwargs))
                if self.error:
                    raise self.error
                return sentinel
            def sync_read(self, *args, **kwargs):
                self.calls.append(('read', args, kwargs))
                return values
        sentinel, values = object(), {'elbow_flex': 3142}
        fake = ModuleType('lerobot.motors.feetech');fake.FeetechMotorsBus = Bus
        with tempfile.TemporaryDirectory() as directory, patch.dict(sys.modules, {'lerobot.motors.feetech': fake}), patch.object(module.Feed, 'side', return_value='left'):
            feed = module.install(directory);bus = Bus()
            self.assertIs(bus.write('Goal_Position', 'elbow_flex', 3142, normalize=False), sentinel)
            self.assertIs(bus.sync_read('Present_Position', normalize=False), values)
            self.assertEqual(len(bus.calls), 2)
            data = json.loads((Path(directory) / 'CURRENT_FEEDBACK.json').read_text())
            self.assertEqual(data['commands']['left']['elbow_flex']['ack'], 'ACK')
            error = RuntimeError('ID3 no response');bus.error = error
            with self.assertRaises(RuntimeError) as caught:
                bus.write('Goal_Position', 'elbow_flex', 3150, normalize=False)
            self.assertIs(caught.exception, error)
            self.assertEqual(len(bus.calls), 3)
            data = json.loads((Path(directory) / 'CURRENT_FEEDBACK.json').read_text())
            self.assertEqual(data['first_error']['id'], 3)
            self.assertEqual(data['commands']['left']['elbow_flex']['ack'], 'UNKNOWN')
            bus.error = None
            with patch.object(feed, 'save', side_effect=OSError('display file unavailable')):
                self.assertIs(bus.sync_read('Present_Position', normalize=False), values)
            self.assertTrue(feed.disabled)
            self.assertIs(bus.write('Goal_Position', 'elbow_flex', 3142, normalize=False), sentinel)


if __name__ == '__main__':
    unittest.main()
