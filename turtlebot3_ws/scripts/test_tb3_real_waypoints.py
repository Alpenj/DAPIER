#!/usr/bin/env python3

from types import SimpleNamespace
import math
import unittest

from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints

from tb3_real_waypoints import make_pose, path_length, tour_succeeded


class RealWaypointTests(unittest.TestCase):
    def test_make_pose_uses_map_frame_and_degrees(self) -> None:
        pose = make_pose(1.5, -2.0, 180.0)
        self.assertEqual(pose.header.frame_id, "map")
        self.assertEqual(pose.pose.position.x, 1.5)
        self.assertAlmostEqual(pose.pose.orientation.z, 1.0)
        self.assertAlmostEqual(pose.pose.orientation.w, 0.0, places=7)

    def test_make_pose_rejects_non_finite_values(self) -> None:
        with self.assertRaises(ValueError):
            make_pose(math.nan, 0.0, 0.0)

    def test_path_length_sums_each_segment(self) -> None:
        poses = [make_pose(0, 0, 0), make_pose(3, 4, 0), make_pose(6, 8, 0)]
        self.assertAlmostEqual(path_length(poses), 10.0)

    def test_tour_requires_terminal_success_and_no_misses(self) -> None:
        self.assertTrue(
            tour_succeeded(
                GoalStatus.STATUS_SUCCEEDED, FollowWaypoints.Result.NONE, 0
            )
        )
        self.assertFalse(
            tour_succeeded(
                GoalStatus.STATUS_SUCCEEDED, FollowWaypoints.Result.NONE, 1
            )
        )
        self.assertFalse(
            tour_succeeded(
                GoalStatus.STATUS_ABORTED, FollowWaypoints.Result.NONE, 0
            )
        )


if __name__ == "__main__":
    unittest.main()
