"""2026-10-15: the START screen's explicit observation-view pre-approval. FakeRunner only."""
import hashlib, importlib.util, json, os
from pathlib import Path
import unittest
from unittest.mock import patch

import control
import test_control
from control import FULL_CHAIN, LEFT_TO_LIFT, ControlError
from records import atomic_json, run_record

NATIVE = Path(os.environ.get('SWEEPICK_PREAPPROVAL_NATIVE', Path.home() / 'sweepick_261015_obs_reach_native'))


def launcher():
    spec = importlib.util.spec_from_file_location('preapproval_native_launcher', NATIVE / 'RUN_APPROVED_SESSION.py')
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m


class PreapprovalStart(unittest.TestCase):
    setUp = test_control.ControlContract.setUp

    def start(self, job, **k):
        return self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True, **k)

    def test_option_binds_session_manifest_and_scope_and_passes_the_launcher_check(self):
        job = self.control.prepare()['active']; self.start(job, preapprove_view=True)
        folder = Path(job['session_out']).parent
        approval = json.loads((folder / 'APPROVAL.json').read_text()); pre = approval['observation_view_preapproval']
        self.assertEqual(pre['session_out'], job['session_out']); self.assertEqual(pre['candidate_id'], 'CURRENT_RECEIVER_CLEAR_THEN_INNER_PAD_VIEW')
        self.assertEqual(pre['prepared_manifest_sha256'], hashlib.sha256((folder / 'MANIFEST.json').read_bytes()).hexdigest())
        self.assertIs(pre['insertion_permitted'], False); self.assertIs(pre['close_permitted'], False)
        self.assertIs(json.loads(self.control.state_file.read_text())['observation_view_preapproved'], True)
        self.assertEqual(launcher().preapproval_record(approval, Path(job['session_out']), pre['prepared_manifest_sha256']), pre)

    def test_without_the_option_the_approval_has_no_preapproval(self):
        job = self.control.prepare()['active']; self.start(job)
        approval = json.loads((Path(job['session_out']).parent / 'APPROVAL.json').read_text())
        self.assertNotIn('observation_view_preapproval', approval)
        self.assertIsNone(launcher().preapproval_record(approval, Path(job['session_out']), approval['prepared_manifest_sha256']))

    def test_option_is_refused_for_the_limited_scope(self):
        with patch('control.limited_backend', return_value=(True, 'test')):
            job = self.control.prepare(execution_scope=LEFT_TO_LIFT)['active']
            with patch.object(control.Controller, 'start', control.Controller.start):
                with self.assertRaisesRegex(ControlError, '사전 승인'):
                    with patch('control.native_source_check', return_value={'files': {}, 'all_match': True, 'tested_record_present': True}):
                        self.start(job, preapprove_view=True)
        self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))

    def test_a_preapproved_decision_shows_no_pending_popup(self):
        job = self.control.prepare()['active']; self.start(job, preapprove_view=True)
        out = Path(job['session_out']); out.mkdir(parents=True, exist_ok=True)
        request = out / 'OBSERVATION_MOVE_REQUEST.json'; request.write_text(json.dumps({'scope': 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION_MOVE', 'plan': {}, 'candidate': {}}))
        self.assertIsNotNone(self.control.status()['active']['pending_view'])       # manual mode: the popup appears
        atomic_json(out / 'OBSERVATION_MOVE_APPROVAL.json', {'approved': True, 'request_sha256': hashlib.sha256(request.read_bytes()).hexdigest(), 'operator_source': 'START_PREAPPROVAL'})
        self.assertIsNone(self.control.status()['active']['pending_view'])

    def test_bus_lost_session_stays_fault_and_blocks_start_until_recovery(self):
        job = self.control.prepare()['active']; self.start(job)
        self.runner.alive = False; out = Path(job['session_out']); (out / 'review').mkdir(parents=True, exist_ok=True)
        (out / 'review/lifetime.jsonl').write_text(json.dumps({'event': 'owner_end', 'exit_code': 3}) + '\n')
        (out / 'episode').mkdir(exist_ok=True)
        (out / 'episode/events.jsonl').write_text(json.dumps({'kind': 'owner_stop', 't_host_s': 100.0, 'reasons': [
            "RECEIVE_CONTACT: RuntimeError: contact_right_return_initial (right): the executor did not plan it: {'gate': 'bus_lost', 'level': 'HARD', 'detail': 'the bus could not be opened: ConnectionError: '}"]}) + '\n')
        status = self.control.status()
        self.assertEqual(status['state'], 'FAULT'); self.assertFalse(status['can_prepare']); self.assertFalse(status['can_start'])
        self.assertEqual(status['active']['first_error']['gate'], 'BUS_LOST')
        record, _ = run_record(out.parent, current=status['active'], owner_alive=False, off_confirmed=False)
        self.assertEqual(record['state'], 'FAULT'); self.assertEqual(record['first_error']['gate'], 'BUS_LOST'); self.assertEqual(record['first_error']['time_s'], 100.0)
        self.assertFalse(record['normal_return_confirmed']); self.assertEqual(record['safe_shutdown'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()


class NewOpenFailureText(unittest.TestCase):
    def test_new_bus_open_detail_keeps_the_bus_lost_classification(self):
        from replay import stop_gate
        new = ("RECEIVE_CONTACT: RuntimeError: contact_right_return_initial (right): the executor did not plan it: {'gate': 'bus_lost', 'level': 'HARD', "
               "'detail': \"the bus could not be opened: ConnectionError: Could not connect on port '/dev/dapier/right_arm'. (from OSError errno 16 Device or resource busy)\"}")
        old = "RECEIVE_CONTACT: RuntimeError: x: {'gate': 'bus_lost', 'level': 'HARD', 'detail': 'the bus could not be opened: ConnectionError: '}"
        self.assertEqual(stop_gate(new), 'BUS_LOST'); self.assertEqual(stop_gate(old), 'BUS_LOST')
        self.assertEqual(stop_gate('existing owner stopped'), 'OWNER_STOP')
