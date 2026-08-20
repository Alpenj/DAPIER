"""ROS 2 subscriber-only CLI for a stationary TurtleBot odometry baseline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Sequence

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from shoe_sorting_data.base_baseline import save_baseline


class StationaryOdomCollector(Node):
    def __init__(self, topic: str, *, warmup_seconds: float) -> None:
        super().__init__("shoe_stationary_odom_collector")
        self.topic = topic
        self.warmup_deadline = time.monotonic() + warmup_seconds
        self.samples: list[dict[str, float | int]] = []
        self.subscription = self.create_subscription(Odometry, topic, self._callback, 20)

    def _callback(self, message: Odometry) -> None:
        if time.monotonic() < self.warmup_deadline:
            return
        self.samples.append(
            {
                "reception_monotonic_ns": time.monotonic_ns(),
                "message_timestamp_ns": (
                    message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                ),
                "linear_x_mps": message.twist.twist.linear.x,
                "angular_z_radps": message.twist.twist.angular.z,
            }
        )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Collect stationary /odom noise without publishing motion commands."
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-seconds", type=float, default=30.0)
    parser.add_argument("--warmup-seconds", type=float, default=3.0)
    parser.add_argument("--topic", default="/odom")
    args, ros_args = parser.parse_known_args(argv)
    if args.duration_seconds <= 0 or args.warmup_seconds < 0:
        parser.error("duration must be positive and warmup must be non-negative")
    if args.output.exists() and any(args.output.iterdir()):
        print(json.dumps({"error": f"output directory is not empty: {args.output}"}, indent=2))
        return 2

    rclpy.init(args=ros_args)
    node = StationaryOdomCollector(args.topic, warmup_seconds=args.warmup_seconds)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.monotonic() + args.warmup_seconds + args.duration_seconds
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        samples_path, report_path = save_baseline(
            args.output,
            node.samples,
            source_topic=args.topic,
            warmup_seconds=args.warmup_seconds,
        )
        report = json.loads(report_path.read_text(encoding="utf-8"))
        print(
            json.dumps(
                {
                    "samples": str(samples_path),
                    "report": str(report_path),
                    "summary": report["summary"],
                },
                indent=2,
            )
        )
        return 0
    except (OSError, ValueError) as error:
        print(json.dumps({"error": str(error)}, indent=2))
        return 1
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
