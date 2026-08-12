#!/usr/bin/env python3

from types import SimpleNamespace
import unittest

from action_msgs.msg import GoalStatus

from tb3_real_nav_watch import GoalStatusTracker


def status(identifier: int, value: int, stamp_ns: int):
    return SimpleNamespace(
        status=value,
        goal_info=SimpleNamespace(
            goal_id=SimpleNamespace(uuid=[identifier] * 16),
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
        ),
    )


class GoalStatusTrackerTests(unittest.TestCase):
    def test_ignores_terminal_goal_already_present_at_start(self) -> None:
        tracker = GoalStatusTracker()
        old = status(1, GoalStatus.STATUS_SUCCEEDED, 1_000_000_000)
        tracker.observe([old], baseline=True)
        tracker.observe([old], baseline=False)
        self.assertIsNone(tracker.tracked_id)
        self.assertFalse(tracker.terminal)

    def test_tracks_goal_that_is_active_during_baseline(self) -> None:
        tracker = GoalStatusTracker()
        active = status(2, GoalStatus.STATUS_EXECUTING, 2_000_000_000)
        tracker.observe([active], baseline=True)
        self.assertEqual(tracker.status, GoalStatus.STATUS_EXECUTING)
        done = status(2, GoalStatus.STATUS_SUCCEEDED, 2_000_000_000)
        tracker.observe([done], baseline=False)
        self.assertTrue(tracker.terminal)
        self.assertTrue(tracker.succeeded)

    def test_locks_new_goal_and_reports_abort(self) -> None:
        tracker = GoalStatusTracker()
        old = status(3, GoalStatus.STATUS_SUCCEEDED, 1_000_000_000)
        new = status(4, GoalStatus.STATUS_ACCEPTED, 3_000_000_000)
        tracker.observe([old], baseline=True)
        tracker.observe([old, new], baseline=False)
        self.assertEqual(tracker.status, GoalStatus.STATUS_ACCEPTED)
        aborted = status(4, GoalStatus.STATUS_ABORTED, 3_000_000_000)
        tracker.observe([old, aborted], baseline=False)
        self.assertTrue(tracker.terminal)
        self.assertFalse(tracker.succeeded)
        self.assertEqual(tracker.status_text, "ABORTED")

    def test_chooses_newest_active_goal(self) -> None:
        tracker = GoalStatusTracker()
        first = status(5, GoalStatus.STATUS_EXECUTING, 4_000_000_000)
        newest = status(6, GoalStatus.STATUS_ACCEPTED, 5_000_000_000)
        tracker.observe([first, newest], baseline=True)
        self.assertEqual(tracker.tracked_id, bytes([6] * 16))


if __name__ == "__main__":
    unittest.main()
