import hashlib
import asyncio
from io import BytesIO
import json
from pathlib import Path
import tempfile
import tarfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from PIL import Image

from records import atomic_json, capture_sources, run_record, save_review
from replay import JOINTS


class RunEvidence(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.directory = self.base / '20261015_090000_abcdef'
        self.root = self.directory / 'session'
        (self.root / 'episode').mkdir(parents=True)
        atomic_json(self.directory / 'APPROVAL.json', {'approved': True, 'scope': 'NONCONTACT'})
        atomic_json(self.directory / 'JOB.json', {'id': self.directory.name, 'consumed': True, 'state': 'RUNNING'})
        source = self.base / 'fixture.py'
        source.write_text('version = 1\n')
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        atomic_json(self.directory / 'SOURCE_SNAPSHOT.json', {'files': capture_sources({str(source): digest}, self.base / 'source_store')})
        events = [{'kind': 'episode_start', 't_host_s': 100}]
        events.extend({'kind': 'stage_record', 'stage': name, 'result': 'GRIP_CONTACT_HOLDING' if name == 'close' else 'HOLDING',
                       't_host_s': 101} for name in ('close', 'lift', 'handoff_approach_carry_right',
                                                    'handoff_approach_carry_left', 'handoff_approach_right_approach'))
        events.append({'kind': 'episode_end', 't_host_s': 102})
        (self.root / 'episode/events.jsonl').write_text('\n'.join(map(json.dumps, events))+'\n')
        frames = []
        for index, camera in enumerate(('top', 'left_wrist', 'right_wrist')):
            image = camera + '.jpg'
            Image.new('RGB', (16, 16), 'red').save(self.root / 'episode' / image)
            row = {'camera': camera, 'file': image, 'n': 1, 'recv_host_s': 100.1 + index / 100,
                   'recv_monotonic_s': 10.1 + index / 100}
            if camera == 'top':
                np.save(self.root / 'episode/depth.npy', np.full((16, 16), 300))
                row['depth_file'] = 'depth.npy'
            frames.append(row)
        (self.root / 'episode/frames.jsonl').write_text('\n'.join(map(json.dumps, frames))+'\n')
        feedback = [{'side': side, 't_host_s': 101, 'read_s': .01, 'ticks': dict.fromkeys(JOINTS, 2000),
                     'goal': dict.fromkeys(JOINTS, 2000), 'torque': dict.fromkeys(JOINTS, 0),
                     'status': dict.fromkeys(JOINTS, 0), 'moving': dict.fromkeys(JOINTS, 0)}
                    for side in ('left', 'right')]
        (self.root / 'episode/joints.jsonl').write_text('\n'.join(map(json.dumps, feedback))+'\n')
        atomic_json(self.root / 'episode/EPISODE.json', {'task_outcome': 'PARTIAL_OR_DIAGNOSTIC: RECEIVE_CONTACT_DONE_PUT_BACK',
                    'stages': [], 'data_quality': {'complete_three_view_coverage': True,
                                                  'camera_coverage': {'status': 'COMPLETE'}}})
        atomic_json(self.root / 'STATE.json', {})

    def test_saved_review_reopens_original_three_view_and_twelve_axes(self):
        record, evidence = run_record(self.directory, off_confirmed=True)
        self.assertEqual(record['recording']['status'], 'SAVED')
        self.assertEqual(record['recording']['camera_frames'], dict.fromkeys(('top_rgb', 'top_depth', 'left_wrist', 'right_wrist'), 1))
        path = save_review(self.directory, record, evidence)
        review = json.loads(path.read_text())
        self.assertEqual(review['run']['id'], self.directory.name)
        self.assertEqual(len(review['final_snapshot']['joints']), 12)
        self.assertTrue(all(c['image_url'] for c in review['final_snapshot']['cameras']))
        self.assertTrue((self.directory / 'RUN_RECORD.json').exists())
        self.assertFalse(review['device_access'])

    def test_missing_file_and_native_coverage_gap_remain_incomplete(self):
        (self.root / 'episode/right_wrist.jpg').unlink()
        ep = json.loads((self.root / 'episode/EPISODE.json').read_text())
        ep['data_quality']['complete_three_view_coverage'] = False
        ep['data_quality']['camera_coverage']['status'] = 'INCOMPLETE'
        atomic_json(self.root / 'episode/EPISODE.json', ep)
        record, evidence = run_record(self.directory, off_confirmed=True)
        self.assertEqual(record['category'], 'success')
        self.assertEqual(record['recording']['status'], 'INCOMPLETE')
        self.assertIn('episode/right_wrist.jpg', record['recording']['missing'])
        self.assertFalse(evidence['native_data_quality']['complete_three_view_coverage'])

    def test_failed_attempt_keeps_first_error_unknown_and_original_files(self):
        atomic_json(self.root / 'CURRENT_FEEDBACK.json', {'first_error': {'gate': 'BUS_LOST', 'id': 3,
                    'ack': 'UNKNOWN', 'application': 'UNKNOWN', 'time_s': 100.5}})
        (self.root / 'episode/events.jsonl').write_text('{"kind":"owner_stop","reasons":["later fault"],"t_host_s":102}\n{')
        record, evidence = run_record(self.directory)
        self.assertEqual(record['state'], 'FAULT')
        self.assertEqual(record['first_error']['id'], 3)
        self.assertEqual(record['safe_shutdown'], 'UNKNOWN')
        self.assertEqual(record['recording']['status'], 'INCOMPLETE')
        self.assertTrue(record['recording']['errors'])
        save_review(self.directory, record, evidence)
        self.assertTrue((self.root / 'episode/right_wrist.jpg').exists())
        self.assertEqual(json.loads((self.directory / 'AI_REVIEW.json').read_text())['run']['first_error']['application'], 'UNKNOWN')

    def test_running_is_never_complete_and_does_not_need_terminal_manifest(self):
        (self.root / 'episode/EPISODE.json').unlink()
        record, _ = run_record(self.directory, owner_alive=True)
        self.assertEqual(record['state'], 'RUNNING')
        self.assertEqual(record['recording']['status'], 'RECORDING')
        self.assertIsNone(record['replay_session_id'])

    def test_source_snapshot_survives_original_edit_and_reuses_identical_content(self):
        source = self.base / 'executor.py'
        source.write_text('version = 1\n')
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        first = capture_sources({str(source): digest}, self.base / 'source_store')
        second = capture_sources({str(source): digest}, self.base / 'source_store')
        self.assertEqual(first, second)
        self.assertEqual(len(list((self.base / 'source_store').iterdir())), 1)
        source.write_text('version = 2\n')
        self.assertEqual(Path(first[str(source)]['file']).read_text(), 'version = 1\n')
        with self.assertRaisesRegex(ValueError, 'source changed'):
            capture_sources({str(source): digest}, self.base / 'different_store')

    def test_early_start_failure_without_camera_or_episode_is_still_exportable(self):
        for path in (self.root / 'episode').iterdir():
            path.unlink()
        (self.directory / 'owner.log').write_text('launcher failed before capture\n')
        atomic_json(self.directory / 'JOB.json', {'consumed': True, 'state': 'FAULT', 'error': 'startup failed'})
        record, evidence = run_record(self.directory)
        self.assertEqual(record['category'], 'failure')
        self.assertIsNone(record['replay_session_id'])
        self.assertEqual(record['recording']['status'], 'INCOMPLETE')
        self.assertEqual(record['first_error']['detail'], 'startup failed')
        self.assertTrue(save_review(self.directory, record, evidence).is_file())

    def test_initial_off_samples_never_confirm_terminal_release(self):
        record, evidence = run_record(self.directory, off_confirmed=False)
        report = json.loads(save_review(self.directory, record, evidence).read_text())
        self.assertEqual(report['run']['safe_shutdown'], 'UNKNOWN')
        self.assertEqual(report['final_snapshot']['command']['safe_shutdown'], 'UNKNOWN')

    def test_put_back_not_seen_is_partial_even_when_release_is_confirmed(self):
        ep = json.loads((self.root / 'episode/EPISODE.json').read_text())
        ep['task_outcome'] = 'PARTIAL_OR_DIAGNOSTIC: RECEIVE_CONTACT_DONE_PUT_BACK_NOT_SEEN'
        atomic_json(self.root / 'episode/EPISODE.json', ep)
        record, _ = run_record(self.directory, off_confirmed=True)
        self.assertEqual(record['category'], 'partial')

    def test_damaged_joints_or_state_is_not_saved_complete(self):
        for relative, body in [('episode/joints.jsonl', '{'), ('STATE.json', '{')]:
            with self.subTest(relative=relative):
                path = self.root / relative
                original = path.read_text()
                path.write_text(body)
                record, evidence = run_record(self.directory, off_confirmed=True)
                save_review(self.directory, record, evidence)
                self.assertEqual(record['recording']['status'], 'INCOMPLETE')
                self.assertTrue(record['recording']['errors'])
                path.write_text(original)

    def test_malformed_nested_quality_and_frame_path_are_reported(self):
        atomic_json(self.root / 'episode/EPISODE.json', {'data_quality': None})
        with (self.root / 'episode/frames.jsonl').open('a') as stream:
            stream.write(json.dumps({'camera': 'left_wrist', 'file': 123})+'\n')
        record, evidence = run_record(self.directory)
        self.assertEqual(record['recording']['status'], 'INCOMPLETE')
        self.assertTrue(any('data_quality' in error for error in record['recording']['errors']))
        self.assertTrue(save_review(self.directory, record, evidence).exists())

    def test_new_run_is_cataloged_replayed_and_recovered_from_disk(self):
        import app
        from control import Controller
        runner = lambda *args, **kwargs: SimpleNamespace(returncode=1, stdout='', stderr='')
        controller = Controller(self.base, run=runner)
        with patch.object(app, 'controller', controller):
            catalog = app.sessions()['sessions']
            self.assertIn(self.directory.name, [row['id'] for row in catalog])
            snapshot = app.snapshot(self.directory.name, 102)
            self.assertEqual(len(snapshot['joints']), 12)
            self.assertEqual(snapshot['command']['safe_shutdown'], 'UNKNOWN')
            self.assertTrue(all(row['image_url'] for row in snapshot['cameras']))
            self.assertEqual(Path(app.media(self.directory.name, 'episode/left_wrist.jpg').path), self.root / 'episode/left_wrist.jpg')
        restarted = Controller(self.base, run=runner)
        self.assertEqual(restarted.runs()[0]['id'], self.directory.name)
        self.assertTrue(restarted.review_file(self.directory.name).exists())

    def test_missing_source_copy_is_explicitly_incomplete(self):
        snapshot = json.loads((self.directory / 'SOURCE_SNAPSHOT.json').read_text())
        Path(next(iter(snapshot['files'].values()))['file']).unlink()
        record, _ = run_record(self.directory)
        self.assertEqual(record['source_snapshot_status'], 'INCOMPLETE')
        self.assertEqual(record['recording']['status'], 'INCOMPLETE')

    def test_review_rechecks_removed_media_after_cached_history(self):
        from control import Controller
        runner = lambda *args, **kwargs: SimpleNamespace(returncode=1, stdout='', stderr='')
        controller = Controller(self.base, run=runner)
        first = controller.runs()[0]
        self.assertEqual(first['recording']['status'], 'SAVED')
        (self.root / 'episode/left_wrist.jpg').unlink()
        self.assertEqual(controller.runs()[0]['verified_at'], first['verified_at'])
        report = json.loads(controller.review_file(self.directory.name).read_text())
        self.assertEqual(report['run']['recording']['status'], 'INCOMPLETE')
        self.assertIn('episode/left_wrist.jpg', report['run']['recording']['missing'])

    def test_original_data_download_contains_real_files_without_saved_archive(self):
        import app
        from control import Controller
        runner = lambda *args, **kwargs: SimpleNamespace(returncode=1, stdout='', stderr='')
        with patch.object(app, 'controller', Controller(self.base, run=runner)):
            response = app.run_data(self.directory.name)
            async def receive():
                return b''.join([chunk async for chunk in response.body_iterator])
            data = asyncio.run(receive())
        self.assertIn(self.directory.name + '_data.tar', response.headers['content-disposition'])
        with tarfile.open(fileobj=BytesIO(data)) as archive:
            prefix = self.directory.name + '/'
            self.assertEqual(archive.extractfile(prefix + 'session/episode/joints.jsonl').read(),
                             (self.root / 'episode/joints.jsonl').read_bytes())
            self.assertIn(prefix + 'session/episode/right_wrist.jpg', archive.getnames())
            self.assertIn(prefix + 'session/episode/depth.npy', archive.getnames())
            self.assertIn(prefix + 'APPROVAL.json', archive.getnames())
        self.assertEqual(list(self.directory.glob('*.tar')), [])

    def test_original_data_download_rejects_unknown_and_running_owner(self):
        import app
        from control import Controller
        from fastapi import HTTPException
        runner = lambda *args, **kwargs: SimpleNamespace(returncode=1, stdout='', stderr='')
        controller = Controller(self.base, run=runner)
        with patch.object(app, 'controller', controller):
            with self.assertRaises(HTTPException) as error:
                app.run_data('../outside')
            self.assertEqual(error.exception.status_code, 404)
            with patch.object(controller, 'status', return_value={'owner_alive': True, 'active': {'id': self.directory.name}}):
                with self.assertRaises(HTTPException) as error:
                    app.run_data(self.directory.name)
                self.assertEqual(error.exception.status_code, 409)

    def test_cancelled_archive_stops_its_own_tar_process(self):
        import app
        with (self.root / 'large_fixture.bin').open('wb') as stream:
            stream.truncate(2 * 1024 * 1024)
        processes = []
        original = app.subprocess.Popen
        def start(*args, **kwargs):
            process = original(*args, **kwargs)
            processes.append(process)
            return process
        with patch.object(app.subprocess, 'Popen', side_effect=start):
            stream = app.archive_stream(self.directory)
            self.assertTrue(next(stream))
            stream.close()
        self.assertIsNotNone(processes[0].poll())

    def test_http_disconnect_closes_archive_for_both_asgi_versions(self):
        import app
        from starlette.requests import ClientDisconnect
        with (self.root / 'large_fixture.bin').open('wb') as stream:
            stream.truncate(2 * 1024 * 1024)
        processes = []
        original = app.subprocess.Popen
        def start(*args, **kwargs):
            process = original(*args, **kwargs)
            processes.append(process)
            return process
        async def disconnected(version):
            response = app.ArchiveResponse(self.directory)
            started = asyncio.Event()
            async def send(message):
                if message['type'] == 'http.response.body':
                    started.set()
                    if version == '2.4':
                        raise OSError('client disconnected')
            async def receive():
                await started.wait()
                return {'type': 'http.disconnect'}
            try:
                await response({'type': 'http', 'asgi': {'spec_version': version}}, receive, send)
            except ClientDisconnect:
                pass
        with patch.object(app.subprocess, 'Popen', side_effect=start):
            for version in ('2.3', '2.4'):
                with self.subTest(version=version):
                    asyncio.run(disconnected(version))
                    self.assertIsNotNone(processes[-1].poll())


if __name__ == '__main__':
    unittest.main()
