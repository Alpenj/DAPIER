#!/usr/bin/env python3
"""Audit whether /odom header timestamps arrive in strictly increasing order."""

import argparse
import sys
import time

from nav_msgs.msg import Odometry
import rclpy


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--min-samples", type=int, default=1)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    if args.seconds <= 0 or args.min_samples <= 0:
        parser.error("seconds and min-samples must be positive")

    rclpy.init()
    node = rclpy.create_node("tb3_odom_timestamp_audit")
    state = {
        "count": 0,
        "regressions": 0,
        "last": None,
        "max_back_ns": 0,
        "seen": set(),
        "duplicates": 0,
    }

    def receive(message: Odometry) -> None:
        stamp = message.header.stamp
        current = stamp.sec * 1_000_000_000 + stamp.nanosec
        previous = state["last"]
        state["count"] += 1
        if current in state["seen"]:
            state["duplicates"] += 1
        else:
            state["seen"].add(current)
        if previous is not None and current < previous:
            backwards = previous - current
            state["regressions"] += 1
            state["max_back_ns"] = max(state["max_back_ns"], backwards)
        state["last"] = current

    subscription = node.create_subscription(Odometry, "/odom", receive, 10)
    del subscription
    deadline = time.monotonic() + args.seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    print(
        "samples={count} unique={unique} duplicates={duplicates} "
        "regressions={regressions} max_back_ms={max_back_ms:.3f}".format(
            count=state["count"],
            unique=len(state["seen"]),
            duplicates=state["duplicates"],
            regressions=state["regressions"],
            max_back_ms=state["max_back_ns"] / 1_000_000.0,
        )
    )
    node.destroy_node()
    rclpy.shutdown()
    if args.strict and (
        state["count"] < args.min_samples
        or state["duplicates"] > 0
        or state["regressions"] > 0
    ):
        print("ERROR: odometry timestamp audit failed", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
