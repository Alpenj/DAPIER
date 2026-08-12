#!/usr/bin/env python3
"""Watch one new RViz NavigateToPose goal and prove its terminal status."""

from __future__ import annotations

import argparse
import time
from typing import Iterable

from action_msgs.msg import GoalStatus, GoalStatusArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}
ACTIVE_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}
TERMINAL_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}


def goal_id(status: GoalStatus) -> bytes:
    return bytes(status.goal_info.goal_id.uuid)


def goal_stamp_ns(status: GoalStatus) -> int:
    stamp = status.goal_info.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class GoalStatusTracker:
    def __init__(self) -> None:
        self.baseline_ids: set[bytes] = set()
        self.tracked_id: bytes | None = None
        self.status: int | None = None

    def observe(self, statuses: Iterable[GoalStatus], *, baseline: bool) -> None:
        items = list(statuses)
        if baseline:
            self.baseline_ids.update(goal_id(item) for item in items)

        if self.tracked_id is None:
            active = [item for item in items if item.status in ACTIVE_STATUSES]
            if active:
                selected = max(active, key=goal_stamp_ns)
                self.tracked_id = goal_id(selected)
            elif not baseline:
                new_items = [
                    item for item in items if goal_id(item) not in self.baseline_ids
                ]
                if new_items:
                    selected = max(new_items, key=goal_stamp_ns)
                    self.tracked_id = goal_id(selected)

        if self.tracked_id is not None:
            for item in items:
                if goal_id(item) == self.tracked_id:
                    self.status = item.status
                    break

    @property
    def terminal(self) -> bool:
        return self.status in TERMINAL_STATUSES

    @property
    def succeeded(self) -> bool:
        return self.status == GoalStatus.STATUS_SUCCEEDED

    @property
    def id_text(self) -> str:
        return self.tracked_id.hex() if self.tracked_id is not None else "none"

    @property
    def status_text(self) -> str:
        return STATUS_NAMES.get(self.status, f"INVALID({self.status})")


class NavStatusNode(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("tb3_real_nav_watch")
        self.latest: GoalStatusArray | None = None
        self.create_subscription(
            GoalStatusArray,
            topic,
            self._receive,
            qos_profile_action_status_default,
        )

    def _receive(self, message: GoalStatusArray) -> None:
        self.latest = message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--baseline-seconds", type=float, default=1.0)
    parser.add_argument(
        "--topic", default="/navigate_to_pose/_action/status", help=argparse.SUPPRESS
    )
    args = parser.parse_args()
    if args.timeout <= 0 or args.baseline_seconds <= 0:
        parser.error("timeout and baseline-seconds must be positive")

    rclpy.init(args=["--ros-args", "--log-level", "error"])
    node = NavStatusNode(args.topic)
    tracker = GoalStatusTracker()

    baseline_deadline = time.monotonic() + args.baseline_seconds
    while rclpy.ok() and time.monotonic() < baseline_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest is not None:
            tracker.observe(node.latest.status_list, baseline=True)

    publishers = node.get_publishers_info_by_topic(args.topic)
    if len(publishers) != 1:
        print(
            f"ERROR: NavigateToPose status needs exactly one publisher; "
            f"found {len(publishers)}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if tracker.tracked_id is None:
        print("READY: click Nav2 Goal in RViz; waiting for one new goal")
    else:
        print(f"TRACKING: active goal={tracker.id_text} status={tracker.status_text}")

    deadline = time.monotonic() + args.timeout
    previous: tuple[bytes | None, int | None] = (None, None)
    while rclpy.ok() and time.monotonic() < deadline and not tracker.terminal:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.latest is None:
            continue
        tracker.observe(node.latest.status_list, baseline=False)
        current = (tracker.tracked_id, tracker.status)
        if tracker.tracked_id is not None and current != previous:
            print(f"goal={tracker.id_text} status={tracker.status_text}")
            previous = current

    if tracker.tracked_id is None:
        print("ERROR: no new NavigateToPose goal was observed before timeout")
        result = 1
    elif not tracker.terminal:
        print(
            f"ERROR: goal={tracker.id_text} did not reach a terminal status "
            f"before timeout; last={tracker.status_text}"
        )
        result = 1
    elif tracker.succeeded:
        print(f"OK: NavigateToPose goal={tracker.id_text} result=SUCCEEDED")
        result = 0
    else:
        print(
            f"ERROR: NavigateToPose goal={tracker.id_text} "
            f"result={tracker.status_text}"
        )
        result = 1

    node.destroy_node()
    rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
