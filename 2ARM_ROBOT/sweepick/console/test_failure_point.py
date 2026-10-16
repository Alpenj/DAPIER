"""Run history: the step a run stopped at and why, read from the native trail and the first error. No device, no native import."""
import json
from pathlib import Path
import tempfile
import unittest

from records import failure_point, failure_reason

MOTOR = ("RuntimeError: FeetechMotorsBus motor check failed on port '/dev/dapier/right_arm':\n\nMissing motor IDs:\n"
         "  - 1 (expected model: 777)\n  - 2 (expected model: 777)\n\nFull expected motor list (id: model_number):\n{1: 777, 2: 777}")


class FailurePoint(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(); self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def trail(self, *rows):
        (self.root / 'SESSION.json').write_text(json.dumps({'run': 'PICK02', 'trail': [dict(zip(('step', 'ready', 'why'), r)) for r in rows]}))

    def test_a_step_marked_not_ready_is_the_stopping_step_with_a_short_reason(self):
        self.trail(('observe', False, 'initial observation or pick planning failed: ' + MOTOR))
        point = failure_point(self.root, {'gate': 'EXECUTOR_ERROR', 'detail': 'exit 3'})
        self.assertEqual((point['stage'], point['label'], point['stage_failed']), ('observe', '초기 관측·집기 계획', True))
        self.assertEqual(point['reason'], 'right_arm 모터 무응답 (ID 1, 2) — 서보 전원·버스 케이블 확인')
        self.assertIn('Missing motor IDs', point['detail'])          # the full text is kept for the reviewer

    def test_retreat_and_home_after_a_stop_are_listed_after_it_not_taken_as_the_stop(self):
        self.trail(('observe', True, ''), ('descend', None, ''), ('look', None, 'the block is not fully readable after the descent'),
                   ('retreat_arm', None, ''), ('home_arm', None, ''))
        point = failure_point(self.root, {'gate': 'EXECUTOR_ERROR', 'detail': 'exit 3'})
        self.assertEqual((point['stage'], point['after'], point['then']), ('look', '하강', ['팔 후퇴', '팔 홈 복귀']))
        self.assertEqual(point['reason'], 'the block is not fully readable after the descent')

    def test_a_specific_first_error_is_preferred_over_the_progress_note(self):
        self.trail(('descend', None, ''), ('look', None, 'label CONTACT_CANDIDATE'))
        point = failure_point(self.root, {'gate': 'BUS_LOST', 'detail': 'Present_Position: RuntimeError: servo response error: ID=6 error=0x20 [RxPacketError] Overload error!'})
        self.assertEqual((point['stage'], point['reason']), ('look', 'ID 6 서보 과부하(Overload) 오류'))

    def test_the_stage_of_a_move_file_error_is_used(self):
        self.trail(('descend', None, ''), ('look', None, 'aligned'), ('close', None, ''))
        point = failure_point(self.root, {'gate': 'certificate_binding', 'detail': 'no CLEAR collision result', 'source': 'close/run1/move.json'})
        self.assertEqual((point['stage'], point['label'], point['after']), ('close', '그리퍼 닫기', '관측'))

    def test_nothing_failed_means_no_failure_point_and_a_missing_trail_is_not_hidden(self):
        self.trail(('home', None, ''), ('put_back_evidence', None, ''))
        self.assertIsNone(failure_point(self.root, None))
        point = failure_point(self.root / 'missing', {'gate': 'EXECUTOR_ERROR', 'detail': 'exit 3'})
        self.assertEqual((point['stage'], point['label'], point['reason']), ('UNKNOWN', '단계 기록 없음', 'exit 3'))

    def test_reason_summary_keeps_other_errors_as_their_first_line(self):
        self.assertEqual(failure_reason('the bus could not be opened: ConnectionError: '), 'Bus 열기 실패·연결 상실 (bus_lost)')
        self.assertEqual(failure_reason('line one\nline two'), 'line one')
        self.assertIsNone(failure_reason(None))



class SeparateEvidence(unittest.TestCase):
    """LEFT CLOSE FOV P0-5: native gate, native terminal class, recorder errors and per-arm torque are kept apart."""
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(); self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def test_only_the_check_the_session_ended_on_is_the_native_gate(self):
        from records import native_gate
        ev = [{'what': 'ready_to_descend', 'ok': False}, {'what': 'ready_to_descend', 'ok': True}, {'what': 'ready_to_close', 'ok': False, 'look': '02_descended', 'why': 'not readable'}]
        (self.root / 'STATE.json').write_text(json.dumps({'evidence': ev}))
        self.assertEqual(native_gate(self.root)['gate'], 'ready_to_close=false')
        (self.root / 'STATE.json').write_text(json.dumps({'evidence': ev[:2]}))          # re-aligned and went on: no native gate
        self.assertIsNone(native_gate(self.root))

    def test_exit_3_with_a_native_result_is_its_terminal_class(self):
        from records import native_terminal
        (self.root / 'owner.log').write_text('warning line\n{\n "result": "PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME"\n}\n')
        row = native_terminal(self.root, {'exit_code': 3})
        self.assertEqual((row['result'], row['exit_code']), ('PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME', 3))
        self.assertIn('not an executor crash', row['exit_role'])
        self.assertEqual(native_terminal(self.root / 'none', {'exit_code': 3})['result'], 'UNKNOWN')

    def test_recorder_errors_keep_their_own_cause_status(self):
        from records import recorder_errors
        rows = recorder_errors({'errors': [
            {'t_host_s': 1.0, 'what': "right arm read: ConnectionError: \nCould not connect on port '/dev/dapier/right_arm'."},
            {'t_host_s': 2.0, 'what': 'right arm read: ConnectionError: x', 'exception': {'summary': 'ConnectionError: x (from OSError errno 16)', 'chain': [{'type': 'ConnectionError'}, {'type': 'OSError'}]}},
            {'t_host_s': 3.0, 'what': 'right arm read: RuntimeError: motor check failed'}]})
        self.assertEqual([r['cause_status'] for r in rows], ['BUS_OPEN_CAUSE_UNCONFIRMED', 'RECORDED', 'MESSAGE_ONLY'])
        self.assertEqual(rows[1]['cause'], 'ConnectionError: x (from OSError errno 16)')

    def test_a_last_seen_torque_register_is_never_a_final_release(self):
        from records import torque_evidence
        ep = {'stages': [{'stage': 'retreat_arm', 'side': 'left', 'result': 'HOLDING'}, {'stage': 'home_arm', 'side': 'left', 'result': 'RETURNED_RELEASED'}]}
        feed = {'sides': {'right': {'torque': {'time_s': 5.0, 'values': {'shoulder_pan': 0, 'gripper': 0}}}}}
        row = torque_evidence(ep, feed)
        self.assertTrue(row['left']['final_release_certified'])
        self.assertFalse(row['right']['final_release_certified']); self.assertTrue(row['right']['last_seen_torque_all_zero'])
        self.assertIn('not a final torque-off check', row['right']['last_seen_role'])


if __name__ == '__main__':
    unittest.main()
