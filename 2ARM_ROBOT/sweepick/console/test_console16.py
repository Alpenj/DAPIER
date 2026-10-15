"""Limited operation boundary: FakeRunner and saved evidence only, no devices."""
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import app
import test_control
import test_records
from control import FULL_CHAIN, LEFT_TO_LIFT, ControlError
from inspection import inspect_run, observation_evidence
from records import atomic_json, run_record


class LimitedScope(unittest.TestCase):
    """Console-16 contract with the native limited backend ABSENT (pinned; since Console-17 the active native may carry it)."""
    def setUp(self):
        test_control.ControlContract.setUp(self)
        unsupported = patch('control.limited_backend', return_value=(False, 'native 실행본에 Console-17 제한 종료 계약이 없음 (test pin)'))
        unsupported.start(); self.addCleanup(unsupported.stop)

    def test_candidate_and_malformed_scopes_cannot_prepare_or_touch_state(self):
        for scope in (LEFT_TO_LIFT, 'LEFT_START', None, {}, []):
            with self.subTest(scope=scope), self.assertRaises(ControlError):
                self.control.prepare(execution_scope=scope)
        self.assertFalse(self.control.state_file.exists())
        self.assertEqual(self.runner.calls, [])
        self.capture.assert_not_called()

    def test_explicit_candidate_start_cannot_consume_full_approval(self):
        job = self.control.prepare()['active']
        paths = [self.control.state_file, Path(job['session_out']).parent / 'APPROVAL.json']
        original = [p.read_bytes() for p in paths]
        self.runner.calls.clear()
        with self.assertRaisesRegex(ControlError, 'BACKEND_CHANGE_REQUIRED'):
            self.control.start(job['id'], field_ready=True, mounts_unchanged=True,
                               supervised=True, execution_scope=LEFT_TO_LIFT)
        self.assertEqual([p.read_bytes() for p in paths], original)
        self.assertEqual(self.runner.calls, [])

    def test_forged_persisted_candidate_is_rejected_before_launcher(self):
        job = self.control.prepare()['active']
        job['execution_scope'] = LEFT_TO_LIFT
        self.control.save(job)
        approval = Path(job['session_out']).parent / 'APPROVAL.json'
        before = approval.read_bytes()
        self.runner.calls.clear()
        with self.assertRaisesRegex(ControlError, 'BACKEND_CHANGE_REQUIRED'):
            self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True)
        self.assertEqual(approval.read_bytes(), before)
        self.assertEqual(self.runner.calls, [])

    def test_full_scope_uses_same_single_launcher_and_duplicate_owner_blocks(self):
        job = self.control.prepare(execution_scope=FULL_CHAIN)['active']
        self.assertEqual(job['execution_scope'], FULL_CHAIN)
        self.runner.foreign = '12345'
        with self.assertRaisesRegex(ControlError, 'owner'):
            self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True)
        self.runner.foreign = ''
        self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True, execution_scope=FULL_CHAIN)
        with self.assertRaises(ControlError):
            self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True)
        launches = [call for call in self.runner.calls if call[:2] == ['tmux', 'new-session']]
        self.assertEqual(len(launches), 1)
        self.assertIn('RUN_APPROVED_SESSION.py --execute --managed-tmux', launches[0][-1])
        self.assertNotIn('--scope', launches[0][-1])

    def test_legacy_job_without_scope_preserves_full_chain_contract(self):
        job = self.control.prepare()['active']
        del job['execution_scope']
        self.control.save(job)
        result = self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True)
        self.assertEqual(result['state'], 'RUNNING')

    def test_capability_is_read_only_and_never_promotes_live_readiness(self):
        self.control.prepare()
        before = self.control.state_file.read_bytes()
        contract = self.control.operation_contract()
        self.assertEqual(self.control.state_file.read_bytes(), before)
        self.assertFalse(contract['device_access'])
        by_scope = {s['id']: s for s in contract['scopes']}
        self.assertTrue(by_scope[FULL_CHAIN]['supported'])
        self.assertFalse(by_scope[LEFT_TO_LIFT]['supported'])
        self.assertFalse(by_scope[LEFT_TO_LIFT]['can_prepare'])
        self.assertEqual(by_scope[LEFT_TO_LIFT]['backend_status'], 'BACKEND_CHANGE_REQUIRED')
        self.assertEqual(contract['observation']['status'], 'NOT_CHECKED')
        self.assertEqual(contract['calibration']['status'], 'NOT_CHECKED')
        self.assertTrue(all(len(s['sha256']) == 64 for s in contract['sources']))
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))

    def test_unknown_device_occupancy_disables_preparation_without_claiming_empty(self):
        with patch.object(self.control, 'owners', side_effect=OSError('query unavailable')):
            contract = self.control.operation_contract()
        self.assertEqual(contract['owner']['device_occupancy'], 'UNKNOWN')
        self.assertFalse(contract['scopes'][0]['can_prepare'])

    def test_scope_endpoint_returns_controller_contract_without_launch(self):
        with patch.object(app, 'controller', self.control):
            self.assertEqual(app.operation_scopes()['schema'], 'sweepick.operation-contract.v1')
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))


class StoredOperationEvidence(unittest.TestCase):
    setUp = test_records.RunEvidence.setUp

    def test_close_refused_is_not_completed_and_cannot_report_lift(self):
        events = [{'kind': 'episode_start', 't_host_s': 100},
                  {'kind': 'phase', 'step': 'after_close', 't_host_s': 101},
                  {'kind': 'episode_end', 't_host_s': 102}]
        (self.root / 'episode/events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
        (self.root / 'close/run1').mkdir(parents=True)
        atomic_json(self.root / 'close/run1/move.json', {'refused': {'gate': 'certificate_binding'},
                    'writes': {'Goal_Position': 0, 'Torque_Enable': 0}})
        atomic_json(self.root / 'stage_close.json', {'result': 'REFUSED', 'runs': [{'folder': str(self.root / 'close/run1')}],
                    'outcome': {'command_completed': False}})
        ep = {'stages': [{'stage': 'close', 'side': 'left', 'result': 'REFUSED', 't0_host': 100.5,
                         't_host_s': 101, 'folder': str(self.root / 'close')}], 'data_quality': {}}
        atomic_json(self.root / 'episode/EPISODE.json', ep)
        before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in self.root.rglob('*') if p.is_file()}
        summary, _ = inspect_run(self.directory, run_record(self.directory)[0])
        close = next(s for s in summary['stage_results'] if s['name'] == 'CLOSE')
        self.assertEqual(close['completion'], 'BLOCKED_NOT_COMPLETED')
        self.assertFalse(any(s['name'] == 'LIFT' and s['status'] == 'COMPLETED' for s in summary['stage_results']))
        self.assertEqual(before, {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in before})

    def test_stopped_holding_never_becomes_home_or_safe_shutdown(self):
        with (self.root / 'episode/events.jsonl').open('a') as stream:
            stream.write(json.dumps({'kind': 'owner_stop', 'reasons': ['operator STOP'], 't_host_s': 101.5})+'\n')
        summary, _ = inspect_run(self.directory, run_record(self.directory)[0])
        self.assertFalse(summary['shutdown']['normal_return_confirmed'])
        self.assertEqual(summary['shutdown']['torque_off_confirmed'], 'UNKNOWN')
        self.assertEqual(summary['shutdown']['safe_shutdown'], 'UNKNOWN')
        self.assertEqual(summary['grasp']['object_grasp']['current_at_failure'], 'UNKNOWN')
        self.assertFalse(summary['device_access'])

    def test_stale_control_frame_keeps_raw_clock_and_unknown_object_bus(self):
        checks = [{'ok': True} for _ in range(115)] + [{
            'ok': False, 'checked_monotonic_s': 10.268, 't_host_s': 101.5, 'register_identity': {},
            'why': ['no valid fresh left wrist frame showing the block between the left jaws'],
            'left_wrist_frame': {'n': 6266, 'recv_monotonic_s': 10, 'fresh': False,
                                 'freshness_limit_s': .064, 'reader_valid': False}}]
        evidence = observation_evidence(checks)
        self.assertEqual(evidence['classification'], 'CAMERA_STALE')
        self.assertAlmostEqual(evidence['frame_age_s'], .268)
        self.assertEqual(evidence['reader_freshness_limit_s'], .064)
        self.assertEqual(evidence['preceding_consecutive_passes'], 115)
        self.assertEqual(evidence['last_eight_prior_checks'], [True]*8)
        self.assertEqual(evidence['object_departure'], 'UNKNOWN')
        self.assertEqual(evidence['bus_error_evidence'], 'NOT_RECORDED')
        self.assertEqual(checks[-1]['left_wrist_frame']['n'], 6266)
        self.assertEqual(observation_evidence([{'ok': False, 'why': ['BUS_LOST']}])['classification'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
