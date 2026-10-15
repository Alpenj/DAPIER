import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from zipfile import ZipFile

from control import ControlError, Controller
from records import atomic_json, save_failure_packet
from replay import application_state, terminal_evidence


class A16OutcomeAndExport(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name) / '20261015_092942_a4628c'
        self.root = self.directory / 'session'
        (self.root / 'episode').mkdir(parents=True)
        (self.root / 'review').mkdir()
        (self.root / 'close/run1').mkdir(parents=True)
        atomic_json(self.root / 'episode/EPISODE.json', {'stages': [{
            'stage': 'close', 'side': 'left', 'result': 'REFUSED', 't_host_s': 20,
            'folder': str(self.root / 'close')}], 'stopped_by': None})
        atomic_json(self.root / 'close/run1/move.json', {'exit_code': 2, 'result': 'REFUSED',
            'refused': {'gate': 'certificate_binding', 'level': 'PROVISIONAL', 'detail': 'no CLEAR result'},
            'stopped_by': None, 'writes': {'Goal_Position': 0, 'Torque_Enable': 0}})
        (self.root / 'review/lifetime.jsonl').write_text(json.dumps(
            {'event': 'owner_end', 'exit_code': 3}) + '\n')
        (self.root / 'episode/events.jsonl').write_text('\n'.join(map(json.dumps, [
            {'kind': 'episode_start', 't_host_s': 10},
            {'kind': 'phase', 'step': 'close', 't_host_s': 19},
            {'kind': 'stage_record', 'stage': 'close', 'side': 'left', 'result': 'REFUSED', 't_host_s': 20},
            {'kind': 'episode_end', 't_host_s': 21}])) + '\n')
        atomic_json(self.root / 'STATE.json', {})

    def test_child_refusal_precedes_parent_exit_and_is_not_executed(self):
        terminal = terminal_evidence(self.root)
        self.assertEqual((terminal['parent_exit_code'], terminal['child_exit_code']), (3, 2))
        self.assertEqual(terminal['error']['gate'], 'certificate_binding')
        self.assertEqual(terminal['command_state'], 'NOT_EXECUTED')

    def test_application_states_keep_readback_and_motion_proofs_distinct(self):
        self.assertEqual(application_state(attempted=False)['state'], 'NOT_EXECUTED')
        self.assertEqual(application_state(ack='UNKNOWN')['state'], 'APPLICATION_UNKNOWN')
        readback = application_state(requested=5, goal=5, actual=4)
        moved = application_state(requested=5, goal=5, actual=4, start_actual=3)
        self.assertEqual((readback['state'], readback['proof']), ('APPLICATION_CONFIRMED', 'GOAL_REGISTER_MATCH'))
        self.assertEqual(moved['proof'], 'PHYSICAL_MOTION')

    def test_first_terminal_failure_wins_and_nonterminal_results_are_ignored(self):
        ep = json.loads((self.root / 'episode/EPISODE.json').read_text())
        (self.root / 'early/run1').mkdir(parents=True)
        atomic_json(self.root / 'early/run1/move.json', {'result': 'BUS_LOST', 'exit_code': 4,
            'writes': {'Goal_Position': 1}})
        ep['stages'] = [
            {'stage': 'return', 'result': 'RETURNED_RELEASED', 't_host_s': 12, 'folder': str(self.root / 'return')},
            {'stage': 'early', 'result': 'BUS_LOST', 't_host_s': 15, 'folder': str(self.root / 'early')},
            *ep['stages']]
        atomic_json(self.root / 'episode/EPISODE.json', ep)
        terminal = terminal_evidence(self.root)
        self.assertEqual(terminal['failed_stage']['stage'], 'early')
        self.assertEqual(terminal['command_state'], 'APPLICATION_UNKNOWN')

    def test_event_fallback_and_unknown_grip_ack_count_as_attempted(self):
        atomic_json(self.root / 'episode/EPISODE.json', {})
        atomic_json(self.root / 'close/run1/move.json', {'result': 'BUS_LOST', 'writes': {'Goal_Position': 0},
            'events': [{'grip_write_acknowledgement': 'UNKNOWN'}]})
        terminal = terminal_evidence(self.root)
        self.assertEqual(terminal['failed_stage']['stage'], 'close')
        self.assertEqual(terminal['command_state'], 'APPLICATION_UNKNOWN')

    def test_missing_write_counters_are_unknown_and_fault_stages_export(self):
        atomic_json(self.root / 'close/run1/move.json', {'result': 'REFUSED',
            'refused': {'gate': 'certificate_binding'}, 'exit_code': 2})
        self.assertEqual(terminal_evidence(self.root)['command_state'], 'APPLICATION_UNKNOWN')
        ep = json.loads((self.root / 'episode/EPISODE.json').read_text())
        ep['stages'][0]['result'] = 'FAULT_HOLDING'
        atomic_json(self.root / 'episode/EPISODE.json', ep)
        atomic_json(self.root / 'close/run1/move.json', {'result': 'FAULT_HOLDING',
            'stopped_by': {'gate': 'servo_fault'}, 'write_attempts': {'Goal_Position': 1}, 'exit_code': 13})
        terminal = terminal_evidence(self.root)
        self.assertEqual(terminal['error']['gate'], 'servo_fault')
        self.assertEqual(terminal['command_state'], 'APPLICATION_UNKNOWN')

    def test_raw_stage_fallback_and_external_move_symlink_guard(self):
        atomic_json(self.root / 'stage_close.json', {'controller_result': 'REFUSED', 'exit_code': 2,
            'refused': {'gate': 'certificate_binding'}, 'writes': {'Goal_Position': 0, 'Torque_Enable': 0}})
        move = self.root / 'close/run1/move.json'
        move.unlink()
        external = Path(self.temp.name) / 'outside.json'
        atomic_json(external, {'exit_code': 99, 'refused': {'gate': 'outside'}})
        move.symlink_to(external)
        terminal = terminal_evidence(self.root)
        self.assertEqual(terminal['error']['gate'], 'certificate_binding')
        self.assertEqual(terminal['child_exit_code'], 2)
        self.assertIsNone(terminal['move_file'])

    def test_wrong_direction_movement_is_not_command_motion_proof(self):
        result = application_state(requested=5, goal=5, actual=2, start_actual=3)
        self.assertEqual(result['proof'], 'GOAL_REGISTER_MATCH')

    def test_review_endpoint_requires_origin_and_token_and_writes_only_review(self):
        import app
        from fastapi import HTTPException
        from starlette.requests import Request
        atomic_json(self.directory / 'APPROVAL.json', {'approved': True})
        atomic_json(self.directory / 'JOB.json', {'consumed': True})
        controller = Controller(Path(self.temp.name), run=lambda *a, **k: type('R', (), {'returncode': 1})())
        scope = {'type': 'http', 'method': 'POST', 'scheme': 'http', 'server': ('localhost', 8765),
                 'path': '/', 'headers': [(b'host', b'localhost:8765')]}
        before = (self.root / 'close/run1/move.json').read_bytes()
        with patch.object(app, 'controller', controller):
            with self.assertRaises(HTTPException) as error:
                app.data_review(self.directory.name, Request(scope), {'purpose': 'EXCLUDE'})
            self.assertEqual(error.exception.status_code, 403)
            self.assertFalse((self.directory / 'DATA_REVIEW.json').exists())
            scope['headers'] += [(b'origin', b'http://localhost:8765'),
                                 (b'x-sweepick-token', controller.csrf_token.encode())]
            result = app.data_review(self.directory.name, Request(scope), {'purpose': 'DIAGNOSTIC_ONLY', 'note': 'checked'})
        self.assertFalse(result['act_approved'])
        self.assertEqual((self.root / 'close/run1/move.json').read_bytes(), before)
        self.assertTrue((self.directory / 'DATA_REVIEW_HISTORY.jsonl').exists())

    def test_review_segment_outside_recorded_range_is_rejected(self):
        atomic_json(self.directory / 'APPROVAL.json', {'approved': True})
        atomic_json(self.directory / 'JOB.json', {'consumed': True})
        controller = Controller(Path(self.temp.name), run=lambda *a, **k: type('R', (), {'returncode': 1})())
        with self.assertRaisesRegex(ControlError, '기록 범위'):
            controller.save_data_review(self.directory.name, {'purpose': 'DIAGNOSTIC_SEGMENT',
                'start_s': 9, 'end_s': 20, 'note': 'outside capture'})
        self.assertFalse((self.directory / 'DATA_REVIEW.json').exists())

    def test_corrupt_move_preserves_stage_error_in_packet(self):
        atomic_json(self.root / 'stage_close.json', {'exit_code': 2,
            'refused': {'gate': 'certificate_binding'}, 'writes': {'Goal_Position': 0, 'Torque_Enable': 0}})
        (self.root / 'close/run1/move.json').write_text('{broken')
        terminal = terminal_evidence(self.root)
        record = {'id': self.directory.name, 'terminal': terminal, 'first_error': terminal['error'],
                  'dataset': {'auto_bucket': 'failed'}}
        save_failure_packet(self.directory, record, {})
        packet = json.loads((self.directory / 'FAILURE_PACKET.json').read_text())
        self.assertEqual(packet['terminal']['error']['gate'], 'certificate_binding')
        self.assertEqual(packet['failed_move']['status'], 'CORRUPT_INPUT')

    def test_failure_zip_has_only_nearest_frames_and_explicit_missing_after(self):
        rows = []
        for camera in ('top', 'left_wrist', 'right_wrist'):
            for when in (10, 21):
                relative = f'{camera}_{when}.jpg'
                (self.root / 'episode' / relative).write_bytes(b'image')
                row = {'camera': camera, 'file': relative, 'recv_host_s': when}
                if camera == 'top' and when == 10:
                    (self.root / 'episode/depth.npy').write_bytes(b'depth')
                    row['depth_file'] = 'depth.npy'
                rows.append(row)
        (self.root / 'episode/frames.jsonl').write_text('\n'.join(map(json.dumps, rows)) + '\n')
        record = {'id': self.directory.name, 'first_error': {'gate': 'certificate_binding'},
                  'terminal': terminal_evidence(self.root),
                  'dataset': {'auto_bucket': 'failed'}}
        atomic_json(self.root / 'stage_close.json', {'stage': 'close', 'controller_result': 'REFUSED'})
        path = save_failure_packet(self.directory, record, {'source_snapshot': {}})
        packet = json.loads((self.directory / 'FAILURE_PACKET.json').read_text())
        self.assertEqual(packet['frames']['left_wrist']['before']['timestamp_s'], 10)
        self.assertEqual(packet['frames']['left_wrist']['after']['timestamp_s'], 21)
        self.assertEqual(packet['frames']['top_depth']['after']['status'], 'UNKNOWN')
        with ZipFile(path) as archive:
            names = archive.namelist()
        self.assertIn('episode/top_10.jpg', names)
        self.assertIn('episode/depth.npy', names)
        self.assertIn('close/run1/move.json', names)
        self.assertIn('stage_close.json', names)
        self.assertFalse(any('..' in name for name in names))

    def test_human_review_rejects_failed_success_candidate(self):
        atomic_json(self.directory / 'APPROVAL.json', {'approved': True})
        atomic_json(self.directory / 'JOB.json', {'consumed': True})
        controller = Controller(Path(self.temp.name), run=lambda *a, **k: type('R', (), {'returncode': 1})())
        with self.assertRaisesRegex(ControlError, 'SUCCESS_CANDIDATE'):
            controller.save_data_review(self.directory.name, {'purpose': 'SUCCESS_CANDIDATE'})
        with self.assertRaisesRegex(ControlError, 'start_s'):
            controller.save_data_review(self.directory.name, {'purpose': 'DIAGNOSTIC_SEGMENT', 'note': 'x'})
        first = controller.save_data_review(self.directory.name, {'purpose': 'DIAGNOSTIC_SEGMENT',
            'start_s': 10, 'end_s': 20, 'note': 'failure segment'})
        second = controller.save_data_review(self.directory.name, {'purpose': 'EXCLUDE', 'note': 'exclude'})
        history = (self.directory / 'DATA_REVIEW_HISTORY.jsonl').read_text().splitlines()
        self.assertEqual((first['review_status'], second['act_approved'], len(history)), ('HUMAN_CONFIRMED', False, 2))

    def test_replay_does_not_expose_future_terminal_record(self):
        from replay import ReplaySession
        archive = ReplaySession(self.directory.name, self.root)
        before = archive.snapshot(19.5)
        after = archive.snapshot(20)
        self.assertIsNone(before['command']['terminal']['error'])
        self.assertEqual(before['command']['state'], 'NOT_EXECUTED')
        self.assertEqual(after['command']['terminal']['error']['gate'], 'certificate_binding')


if __name__ == '__main__':
    unittest.main()
