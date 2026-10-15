import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import unittest
from zipfile import ZipFile
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
from PIL import Image

from replay import ReplaySession, clean, depth_png, session, ROOTS


class RecordedSessions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.good = session('session_18')
        cls.camera_fault = session('observation_08')
        cls.bus_fault = session('observation_09')

    def test_success_scope_preserves_partial_episode(self):
        catalog = self.good.catalog()
        self.assertEqual(catalog['category'], 'success')
        self.assertIn('RIGHT PREGRASP', catalog['classification_scope'])
        self.assertIn('PARTIAL', catalog['outcome'])
        self.assertIn('전체 handover 성공이 아니', catalog['warning'])
        last = self.good.snapshot(self.good.trial_end)
        self.assertEqual(last['stage_results'][-1]['status'], 'NOT_READY')
        self.assertEqual(last['command']['first_error']['gate'], 'TARGET_NOT_REACHED')

    def test_three_view_and_measured_12_axes_before_approach(self):
        s = self.camera_fault
        snap = s.snapshot()
        self.assertTrue(all(cam['image_url'] and cam['status'] == 'REPLAY_FRESH' for cam in snap['cameras']))
        self.assertEqual(len(snap['joints']), 12)
        self.assertEqual([r['actual'] for r in snap['joints'][:6]], [2566, 1049, 3102, 2106, 2320, 3042])
        self.assertTrue(all(r['feedback_time_s'] <= snap['time_s'] for r in snap['joints']))
        self.assertEqual(snap['stage_results'][3]['status'], 'PENDING')
        self.assertIsNone(snap['command']['first_error'])

    def test_right_camera_stale_with_age_and_error(self):
        s = self.camera_fault
        t = next(b['t_s'] for b in s.bookmarks if b['name'] == 'RECEIVE_CONTACT')
        snap = s.snapshot(t)
        right = snap['cameras'][3]
        self.assertEqual(right['status'], 'STALE')
        self.assertEqual(right['connection'], 'RECORDED_ERROR')
        self.assertGreater(right['age_s'], 20)
        self.assertIsNotNone(right['image_url'])
        self.assertEqual(snap['stage_results'][-1]['status'], 'NOT_READY')
        self.assertEqual(snap['command']['first_error']['gate'], 'CAMERA_READER_ERROR')

    def test_depth_retains_own_frame_time(self):
        s = self.camera_fault
        rgb = next(r for r in s.frames['top_rgb'] if r['n'] == 4702)
        snap = s.snapshot(rgb['t'])
        top, depth = snap['cameras'][:2]
        self.assertEqual(top['n'], 4702)
        self.assertEqual(depth['n'], 4701)
        self.assertLess(depth['last_received_s'], top['last_received_s'])
        self.assertGreater(depth['age_s'], top['age_s'])
        self.assertGreater(depth['fps'], depth['saved_fps'])
        self.assertEqual(len(depth['depth_range_mm']), 2)

    def test_previous_stage_ack_is_not_attached_to_current_request(self):
        for s, at in [(self.good, 1791778720.8810961), (self.camera_fault, 1791937507.1461072)]:
            row = s.snapshot(at)['joints'][0]
            self.assertIn('align1', row['source'])
            self.assertEqual(row['ack'], 'UNKNOWN')
            self.assertNotEqual(row['requested_goal'], row['last_acknowledged']['requested_goal'])

    def test_bus_failure_is_not_unapplied_or_safe_shutdown(self):
        s = self.bus_fault
        snap = s.snapshot()
        command = snap['command']
        self.assertEqual(command['ack'], 'UNKNOWN')
        self.assertEqual(command['application'], 'UNKNOWN')
        self.assertEqual(command['safe_shutdown'], 'UNKNOWN')
        self.assertEqual(command['first_error']['id'], 3)
        self.assertEqual(command['first_error']['rx_bytes'], 0)
        self.assertEqual(command['first_error']['attempts'], 1)
        self.assertEqual(command['first_error']['retries'], 0)
        self.assertEqual([r['ack'] for r in snap['joints'][:5]], ['ACK', 'ACK', 'UNKNOWN', 'NOT_SENT', 'NOT_SENT'])
        self.assertEqual(snap['joints'][2]['requested_goal'], 3142)
        self.assertTrue(all(r['torque'] == 'UNKNOWN' for r in snap['joints']))
        self.assertEqual(snap['stage_results'][0]['status'], 'BUS_LOST')
        self.assertEqual(snap['stage_results'][3]['status'], 'NOT_REACHED')

    def test_purged_media_is_not_replaced_with_later_capture(self):
        s = self.bus_fault
        self.assertEqual(s.media, set())
        self.assertTrue(all(cam['status'] == 'NOT_AVAILABLE' and cam['image_url'] is None for cam in s.snapshot()['cameras']))

    def test_later_read_is_not_backdated_and_cleanup_does_not_change_error(self):
        s = self.bus_fault
        before = s.snapshot(s.trial_end)
        self.assertEqual(before['joints'][0]['torque'], 'UNKNOWN')
        t = next(b['t_s'] for b in s.bookmarks if b['name'] == '별도 사후 조회')
        later = s.snapshot(t)
        self.assertEqual([r['torque'] for r in later['joints'][:6]], [1, 1, 0, 0, 0, 0])
        final = s.snapshot(s.end)
        self.assertEqual([r['torque'] for r in final['joints'][:6]], [0] * 6)
        self.assertEqual(final['joints'][0]['actual'], 'UNKNOWN')
        self.assertEqual(final['command']['first_error']['ack'], 'UNKNOWN')
        self.assertEqual(final['command']['safe_shutdown'], 'UNKNOWN')
        self.assertTrue(final['command']['later_recovery']['all_left_off'])

    def test_rerun_plan_is_unapproved_and_current_launcher_only(self):
        for key in ROOTS:
            plan = session(key).rerun_plan()
            self.assertFalse(plan['execution_connected'])
            self.assertFalse(plan['approved'])
            self.assertIn('receive_alignment_restore/RUN_APPROVED_SESSION.py', plan['existing_launcher'])
            self.assertIn('저장된 모터값 직접 재전송', plan['prohibited'])

    def test_latest_web_failure_preserves_pregrasp_and_never_promotes_later_off(self):
        archive = session('field_20261014')
        at = 1791961753.71452
        snap = archive.snapshot(at)
        self.assertEqual(archive.catalog()['category'], 'partial')
        self.assertEqual(snap['command']['first_error']['gate'], 'LEFT_HOLD_VISION_UNCONFIRMED')
        self.assertEqual(snap['stage_results'][0]['status'], 'GRIP_CONTACT_HOLDING')
        self.assertTrue(all(s['status'] == 'HOLDING' for s in snap['stage_results'][1:4]))
        self.assertEqual(snap['stage_results'][4]['status'], 'FAILED')
        self.assertEqual(snap['cameras'][2]['n'], 6266)
        self.assertEqual(snap['cameras'][2]['status'], 'STALE')
        self.assertNotIn('later_recovery', snap['command'])
        final = archive.snapshot(archive.end)
        self.assertTrue(all(row['torque'] == 0 for row in final['joints']))
        self.assertEqual(final['command']['safe_shutdown'], 'UNKNOWN')
        self.assertTrue(final['command']['later_recovery']['all_left_off'])

    def test_latest_web_wait_bookmark_is_approval_pending_not_complete_observation(self):
        archive = session('field_20261014')
        at = next(b['t_s'] for b in archive.bookmarks if b['name'] == '추가 관측 이동 승인 대기')
        snap = archive.snapshot(at)
        self.assertEqual(snap['stage_results'][-1]['status'], 'AWAITING_APPROVAL')
        self.assertIsNone(snap['command']['first_error'])


class FixtureTimingAndTrust(unittest.TestCase):
    def test_future_read_and_malformed_line_and_path_escape(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'episode').mkdir()
            (root / 'episode/events.jsonl').write_text(json.dumps({'kind': 'episode_start', 't_host_s': 10}) + '\n')
            raw = {'side': 'left', 't_host_s': 12, 'read_s': 2,
                   'ticks': {'shoulder_pan': 50}, 'torque': {'shoulder_pan': 0}}
            (root / 'episode/joints.jsonl').write_text('invalid line\n' + json.dumps(raw) + '\n')
            s = ReplaySession('fixture', root)
            self.assertEqual(s.snapshot(13)['joints'][0]['actual'], 'UNKNOWN')
            self.assertEqual(s.snapshot(14)['joints'][0]['actual'], 50)
            self.assertEqual(s.snapshot(14)['joints'][1]['torque'], 'UNKNOWN')
            self.assertTrue(any('손상된 행' in w for w in s.warnings))
            self.assertIsNone(s.allow_media(root / '../../etc/passwd'))
            with self.assertRaises(ValueError):
                s.snapshot(float('nan'))

    def test_depth_mm_nonfinite_and_zero_invalid(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'depth.npy'
            np.save(path, np.array([[0, np.nan], [100, 200]]))
            png, bounds = depth_png(str(path))
            self.assertEqual(bounds, [100, 200])
            from io import BytesIO
            image = np.asarray(Image.open(BytesIO(png)))
            self.assertTrue((image[0] == 0).all())
            self.assertEqual(clean({'measurement': float('inf'), 'first_error': None}),
                             {'measurement': 'UNKNOWN', 'first_error': None})


class HttpReadOnly(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        cls.base = f'http://127.0.0.1:{port}'
        cls.log = tempfile.TemporaryFile()
        cls.process = subprocess.Popen([str(Path('.venv/bin/python').absolute()), '-m', 'uvicorn', 'app:app',
            '--host', '127.0.0.1', '--port', str(port)], stdout=cls.log, stderr=cls.log)
        try:
            for _ in range(100):
                if cls.process.poll() is not None:
                    raise RuntimeError('test server exited')
                try:
                    with urlopen(cls.base, timeout=.2) as response:
                        if response.status == 200:
                            return
                except OSError:
                    time.sleep(.05)
            raise RuntimeError('test server did not start')
        except Exception:
            cls.process.terminate()
            cls.process.wait(timeout=5)
            cls.log.seek(0)
            print(cls.log.read().decode())
            cls.log.close()
            raise

    @classmethod
    def tearDownClass(cls):
        cls.process.terminate()
        cls.process.wait(timeout=5)
        cls.log.close()

    def get(self, path):
        with urlopen(self.base + path, timeout=20) as response:
            return json.loads(response.read())

    def test_catalog_and_images_are_real_http(self):
        catalog = self.get('/api/sessions')
        curated = [s for s in catalog['sessions'] if s['id'] in ROOTS]
        self.assertEqual([s['category'] for s in curated], ['success', 'partial', 'failure', 'partial'])
        self.assertEqual(len({s['id'] for s in catalog['sessions']}), len(catalog['sessions']))
        self.assertTrue(all(s['category'] in ('success', 'partial', 'failure') for s in catalog['sessions']))
        snapshot = self.get('/api/sessions/observation_08/snapshot')
        for camera in snapshot['cameras']:
            with urlopen(self.base + camera['image_url'], timeout=10) as response:
                self.assertIn(response.headers['Content-Type'], ['image/jpeg', 'image/png'])
                self.assertGreater(len(response.read()), 100)

    def test_no_start_stop_write_or_arbitrary_files(self):
        for path in ['/api/start', '/api/stop', '/api/sessions/session_18/snapshot', '/api/sessions/session_18/rerun-plan']:
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(self.base + path, data=b'{}', method='POST'), timeout=10)
            self.assertIn(error.exception.code, (404, 405))
        for path in ['/api/media/observation_09/review/environment_stability/top.png',
                     '/api/media/observation_08/STATE.json', '/api/media/observation_08/../../etc/passwd',
                     '/api/sessions/missing/snapshot', '/api/sessions/session_18/snapshot?t=nan']:
            with self.assertRaises(HTTPError) as error:
                urlopen(self.base + path, timeout=10)
            self.assertIn(error.exception.code, (404, 422))

    def test_foreign_host_rejected(self):
        with self.assertRaises(HTTPError) as error:
            urlopen(Request(self.base, headers={'Host': 'foreign.example'}), timeout=10)
        self.assertEqual(error.exception.code, 400)

    def test_operator_cannot_be_framed_by_another_site(self):
        with urlopen(self.base, timeout=5) as response:
            self.assertEqual(response.headers['X-Frame-Options'], 'DENY')
            self.assertEqual(response.headers['Content-Security-Policy'], "frame-ancestors 'none'")

    def test_control_posts_require_origin_token_and_explicit_field_approval(self):
        token = self.get('/api/control')['csrf_token']
        for headers, code in [({}, 403), ({'Origin': 'https://foreign.example', 'X-Sweepick-Token': token}, 403),
                              ({'Origin': self.base, 'X-Sweepick-Token': token}, 409)]:
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(self.base + '/api/control/start', data=b'{}', method='POST',
                                headers={'Content-Type': 'application/json', **headers}), timeout=5)
            self.assertEqual(error.exception.code, code)

    def test_instructor_materials_and_recovery_boundary(self):
        info = self.get('/api/instructor')
        self.assertTrue(info['bundle_verified'])
        with ZipFile('/home/robot/Downloads/SWEEPICK_INSTRUCTOR_REVIEW_20261014.zip') as archive:
            expected = all(archive.read(member) == path.read_bytes() for member, path in (
                ('source/RUN_APPROVED_SESSION.py', Path(info['launcher'])),
                ('source/sweepick_full_chain(2).py', Path(info['launcher']).parent / 'revision/sweepick_full_chain.py')))
        self.assertEqual(info['source_matches'], expected)
        for path, content in [('/instructor', 'SWEEPICK 강사 시연'),
                              ('/instructor-materials/OPERATE.txt', 'AI 호출은 필요 없다')]:
            with urlopen(self.base + path, timeout=5) as response:
                self.assertIn(content, response.read().decode())
        with urlopen(self.base + '/api/instructor/bundle', timeout=5) as response:
            self.assertIn('application/zip', response.headers['Content-Type'])
        with urlopen(self.base + '/instructor-materials/reference/02_TECHNICAL_BRIEF_KO.md', timeout=5) as response:
            self.assertIn('text/plain', response.headers['Content-Type'])
        token = self.get('/api/control')['csrf_token']
        for headers, code in [({}, 403), ({'Origin': self.base, 'X-Sweepick-Token': token}, 409)]:
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(self.base + '/api/control/confirm-recovery', data=b'{}', method='POST',
                                headers={'Content-Type': 'application/json', **headers}), timeout=5)
            self.assertEqual(error.exception.code, code)

    def test_fresh_plan_and_saved_run_review_are_available_without_robot_action(self):
        plan = self.get('/api/control/plan')
        self.assertIsNone(plan['source_session'])
        self.assertFalse(plan['approved'])
        self.assertTrue(plan['execution_connected'])
        self.assertIn('RUN_APPROVED_SESSION.py', plan['existing_launcher'])
        records = self.get('/api/runs')['runs']
        record = next(row for row in records if row['id'] == '20261014_155824_d4017e')
        self.assertEqual(record['recording']['status'], 'INCOMPLETE')
        self.assertEqual(record['replay_session_id'], 'field_20261014')
        with urlopen(self.base + record['review_url'], timeout=20) as response:
            self.assertIn('_AI_REVIEW.json', response.headers['Content-Disposition'])
            review = json.loads(response.read())
        self.assertEqual(review['run']['id'], record['id'])
        self.assertEqual(review['run']['safe_shutdown'], 'UNKNOWN')
        self.assertEqual(review['run']['first_error']['gate'], 'LEFT_HOLD_VISION_UNCONFIRMED')


if __name__ == '__main__':
    unittest.main()
