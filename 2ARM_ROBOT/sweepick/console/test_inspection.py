import hashlib
import json
from pathlib import Path
import unittest
from zipfile import ZipFile
from unittest.mock import patch
import threading

from records import atomic_json, run_record, save_review
from inspection import inspect_run, save_inspection
import test_records


class InspectionEvidence(unittest.TestCase):
    setUp = test_records.RunEvidence.setUp

    def test_small_package_hashes_original_media_and_real_timeseries(self):
        before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in self.root.rglob('*') if p.is_file()}
        record, _ = run_record(self.directory, off_confirmed=True)
        summary = save_inspection(self.directory, record)
        self.assertEqual(summary['window']['motor_sample_count'], 2)
        self.assertEqual(summary['window']['motor_before_count'], 2)
        self.assertEqual(summary['window']['motor_after_count'], 0)
        self.assertEqual(len(summary['joints']), 12)
        self.assertTrue(all(j['goal_error_ticks'] == 0 for j in summary['joints']))
        with ZipFile(self.directory / 'INSPECTION_PACKAGE.zip') as z:
            manifest = json.loads(z.read('FILE_SHA256.json'))
            self.assertEqual({x['file'] for x in manifest['files']}, set(z.namelist()) - {'FILE_SHA256.json'})
            for item in manifest['files']:
                self.assertEqual(hashlib.sha256(z.read(item['file'])).hexdigest(), item['sha256'])
            self.assertIn('window/motor_samples.jsonl', z.namelist())
            self.assertTrue(z.read('window/motor_samples.jsonl'))
            for cam in summary['cameras']:
                item = cam['frames']['before']
                self.assertEqual(hashlib.sha256(z.read(item['file'])).hexdigest(), item['sha256'])
                self.assertEqual(cam['frames']['after']['status'], 'UNKNOWN')
        self.assertEqual(before, {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in before})

    def test_missing_measurement_does_not_become_success_or_synchronized(self):
        (self.root / 'episode/right_wrist.jpg').unlink()
        record, _ = run_record(self.directory)
        summary, window = inspect_run(self.directory, record)
        self.assertEqual(summary['grasp']['object_grasp']['value'], 'UNKNOWN')
        self.assertEqual(summary['grasp']['whole_task_success']['value'], False)  # partial outcome
        self.assertEqual(summary['shutdown']['torque_off_confirmed'], 'UNKNOWN')
        self.assertEqual(summary['shutdown']['safe_shutdown'], 'UNKNOWN')
        self.assertEqual(summary['time_alignment']['physical_capture_sync'], 'UNKNOWN')
        right = next(c for c in summary['cameras'] if c['id'] == 'right_wrist')
        self.assertEqual(right['frames']['before']['status'], 'UNKNOWN')

    def test_missing_original_timestamps_invalidate_old_inspection_export(self):
        atomic_json(self.directory / 'INSPECTION.json', {'status': 'OLD_HEALTHY_REPORT'})
        (self.directory / 'INSPECTION_PACKAGE.zip').write_bytes(b'old export')
        for name in ('events.jsonl', 'joints.jsonl', 'frames.jsonl'):
            (self.root / 'episode' / name).unlink()
        record, evidence = run_record(self.directory)
        save_review(self.directory, record, evidence)
        self.assertEqual(json.loads((self.directory / 'INSPECTION.json').read_text())['status'], 'NOT_AVAILABLE')
        self.assertFalse((self.directory / 'INSPECTION_PACKAGE.zip').exists())

    def test_later_off_never_overwrites_task_failure_or_normal_return(self):
        (self.root / 'episode/events.jsonl').write_text((self.root / 'episode/events.jsonl').read_text() +
            json.dumps({'kind': 'observation_plan_authorization_failed', 'reason': 'no valid fresh left wrist frame', 't_host_s': 101.5})+'\n')
        states = {side: {'after': {'answered': True, 'state': {'torque': dict.fromkeys(
            ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'), 0)}}} for side in ('left','right')}
        atomic_json(self.root / 'review/TORQUE_OFF_OPERATOR_REQUEST.json', {'sides': states,
                    'all_off_confirmed': True, 'finished': '1970-01-01T00:02:00+00:00'})
        record, _ = run_record(self.directory, off_confirmed=True)
        summary, _ = inspect_run(self.directory, record)
        self.assertEqual(summary['first_failure']['time_s'], 101.5)
        self.assertTrue(summary['shutdown']['torque_off_confirmed'])
        self.assertFalse(summary['shutdown']['normal_return_confirmed'])
        self.assertEqual(summary['shutdown']['safe_shutdown'], 'UNKNOWN')
        self.assertEqual(summary['readiness']['limited_scope']['status'], 'NOT_READY')
        self.assertEqual(summary['readiness']['whole_mission']['status'], 'NOT_READY')

    def test_partial_off_record_is_not_both_arms_off(self):
        atomic_json(self.root / 'review/TORQUE_OFF_OPERATOR_REQUEST.json', {'sides': {}, 'all_off_confirmed': True})
        summary, _ = inspect_run(self.directory, run_record(self.directory)[0])
        self.assertEqual(summary['shutdown']['torque_off_confirmed'], 'UNKNOWN')

    def test_export_rejects_active_owner_and_unknown_run_without_regeneration(self):
        import app
        from fastapi import HTTPException
        with patch.object(app, 'controller') as controller:
            controller.run_directory.return_value = self.directory
            controller.status.return_value = {'owner_alive': True, 'active': {'id': self.directory.name}}
            with self.assertRaises(HTTPException) as caught:
                app.run_inspection(self.directory.name)
            self.assertEqual(caught.exception.status_code, 409)
            controller.review_file.assert_not_called()
            controller.run_directory.return_value = None
            with self.assertRaises(HTTPException) as caught:
                app.inspection_package('../outside')
            self.assertEqual(caught.exception.status_code, 404)

    def test_export_uses_cached_matching_evidence(self):
        import app
        summary = save_inspection(self.directory, run_record(self.directory)[0])
        atomic_json(self.directory / 'RUN_RECORD.json', {'index_signature': {'version': 'test'}})
        with patch.object(app, 'controller') as controller:
            controller.run_directory.return_value = self.directory
            controller.status.return_value = {'owner_alive': False, 'active': None}
            controller.records_lock = threading.Lock()
            controller.record_stamp.return_value = {'version': 'test'}
            response = app.inspection_package(self.directory.name)
            self.assertEqual(response.path, self.directory / 'INSPECTION_PACKAGE.zip')
            controller.review_file.assert_not_called()


if __name__ == '__main__':
    unittest.main()
