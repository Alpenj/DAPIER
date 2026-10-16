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


if __name__ == '__main__':
    unittest.main()
