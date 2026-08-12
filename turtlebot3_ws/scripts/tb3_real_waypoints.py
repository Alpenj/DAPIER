#!/usr/bin/env python3
"""Plan, then optionally execute an ordered real-TurtleBot3 waypoint tour.

The explicit ``--execute`` switch is intentional: a typo in a map coordinate
must be inspectable with the planner without causing physical motion.  Even in
execute mode the complete route is planned first and is refused if Nav2 cannot
produce one connected path through every supplied pose.
"""

from __future__ import annotations

import argparse
import math
import time
from collections.abc import Sequence

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathThroughPoses, FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def make_pose(x: float, y: float, yaw_degrees: float) -> PoseStamped:
    """Convert a human-friendly map pose in degrees to a ROS PoseStamped."""
    if not all(math.isfinite(value) for value in (x, y, yaw_degrees)):
        raise ValueError("waypoint values must be finite")
    yaw = math.radians(yaw_degrees)
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def path_length(poses: Sequence[PoseStamped]) -> float:
    """Return the planar length of a Nav2 path for an auditable plan summary."""
    return sum(
        math.hypot(
            right.pose.position.x - left.pose.position.x,
            right.pose.position.y - left.pose.position.y,
        )
        for left, right in zip(poses, poses[1:])
    )


def tour_succeeded(status: int, error_code: int, missed_count: int) -> bool:
    """Require both ROS action success and zero Nav2 waypoint omissions."""
    return (
        status == GoalStatus.STATUS_SUCCEEDED
        and error_code == FollowWaypoints.Result.NONE
        and missed_count == 0
    )


def wait_future(node: Node, future, timeout: float):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    return future.result() if future.done() else None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate an ordered Nav2 route and optionally visit every waypoint."
        )
    )
    parser.add_argument(
        "--pose",
        action="append",
        nargs=3,
        type=float,
        metavar=("X", "Y", "YAW_DEG"),
        required=True,
        help="map-frame waypoint; repeat at least twice",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="physically drive after the complete route passes planning",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=900.0,
        help="tour deadline in seconds (default: 900)",
    )
    args = parser.parse_args()
    if len(args.pose) < 2:
        parser.error("repeat --pose at least twice")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    return args


def main() -> int:
    args = parse_args()
    poses = [make_pose(*values) for values in args.pose]

    rclpy.init(args=["--ros-args", "--log-level", "error"])
    node = Node("tb3_real_waypoints")
    planner = ActionClient(
        node, ComputePathThroughPoses, "/compute_path_through_poses"
    )
    follower = ActionClient(node, FollowWaypoints, "/follow_waypoints")

    if not planner.wait_for_server(timeout_sec=5.0):
        print("ERROR: /compute_path_through_poses is unavailable")
        return 1

    # Plan the exact ordered tour before enabling any wheel command.  This
    # catches unknown-space, occupied-goal, and disconnected-map mistakes.
    plan_goal = ComputePathThroughPoses.Goal()
    plan_goal.goals = poses
    plan_goal.planner_id = "GridBased"
    plan_goal.use_start = False
    plan_handle = wait_future(node, planner.send_goal_async(plan_goal), 8.0)
    if plan_handle is None or not plan_handle.accepted:
        print("ERROR: Nav2 rejected the route-planning request")
        return 1
    plan_wrapper = wait_future(node, plan_handle.get_result_async(), 20.0)
    if plan_wrapper is None:
        print("ERROR: route planning timed out")
        return 1
    plan_result = plan_wrapper.result
    planned_length = path_length(plan_result.path.poses)
    if (
        plan_wrapper.status != GoalStatus.STATUS_SUCCEEDED
        or plan_result.error_code != ComputePathThroughPoses.Result.NONE
        or len(plan_result.path.poses) < 2
    ):
        print(
            "ERROR: no complete route; "
            f"status={plan_wrapper.status} code={plan_result.error_code} "
            f"message={plan_result.error_msg!r}"
        )
        return 1
    print(
        f"PLAN OK: waypoints={len(poses)} poses={len(plan_result.path.poses)} "
        f"length={planned_length:.2f}m"
    )

    if not args.execute:
        print("PLAN ONLY: inspect the route, then repeat with --execute to drive")
        return 0
    if not follower.wait_for_server(timeout_sec=5.0):
        print("ERROR: /follow_waypoints is unavailable")
        return 1

    print("DRIVE STARTING: keep the robot in sight; power off on red LED or alarm")
    last_index: list[int | None] = [None]

    def feedback(message) -> None:
        index = message.feedback.current_waypoint
        if index != last_index[0]:
            target = poses[index].pose.position
            print(
                f"WAYPOINT {index + 1}/{len(poses)} active "
                f"target=({target.x:.3f}, {target.y:.3f})",
                flush=True,
            )
            last_index[0] = index

    tour_goal = FollowWaypoints.Goal()
    # Nav2 defines zero loops as one pass through the supplied list.  Repeated
    # return poses are listed explicitly so the intended route stays visible.
    tour_goal.number_of_loops = 0
    tour_goal.goal_index = 0
    tour_goal.poses = poses
    tour_handle = wait_future(
        node,
        follower.send_goal_async(tour_goal, feedback_callback=feedback),
        8.0,
    )
    if tour_handle is None or not tour_handle.accepted:
        print("ERROR: Nav2 rejected the waypoint tour")
        return 1

    result_future = tour_handle.get_result_async()
    tour_wrapper = wait_future(node, result_future, args.timeout)
    if tour_wrapper is None:
        cancel = tour_handle.cancel_goal_async()
        wait_future(node, cancel, 5.0)
        print("ERROR: tour timeout; cancellation requested")
        return 1

    result = tour_wrapper.result
    missed = [(item.index, item.error_code) for item in result.missed_waypoints]
    print(
        f"TOUR RESULT: status={tour_wrapper.status} code={result.error_code} "
        f"missed={missed} message={result.error_msg!r}"
    )
    if not tour_succeeded(tour_wrapper.status, result.error_code, len(missed)):
        return 1
    print("OK: every waypoint was visited")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        if rclpy.ok():
            rclpy.shutdown()
