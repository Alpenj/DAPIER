#!/usr/bin/env python3
"""Verify that the physical LiDAR produces fresh, structurally valid scans."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanAudit:
    def __init__(self) -> None:
        self.count = 0
        self.last_stamp_ns: int | None = None
        self.duplicates = 0
        self.regressions = 0
        self.invalid_samples = 0
        self.finite_points = 0

    def receive_values(
        self,
        *,
        stamp_ns: int,
        frame_id: str,
        angle_increment: float,
        range_min: float,
        range_max: float,
        ranges: list[float] | tuple[float, ...],
    ) -> None:
        self.count += 1
        if self.last_stamp_ns is not None:
            if stamp_ns == self.last_stamp_ns:
                self.duplicates += 1
            elif stamp_ns < self.last_stamp_ns:
                self.regressions += 1
        self.last_stamp_ns = stamp_ns

        structurally_valid = (
            stamp_ns > 0
            and bool(frame_id)
            and math.isfinite(angle_increment)
            and angle_increment > 0
            and math.isfinite(range_min)
            and math.isfinite(range_max)
            and 0 <= range_min < range_max
            and len(ranges) > 0
        )
        if not structurally_valid:
            self.invalid_samples += 1
            return
        self.finite_points += sum(
            math.isfinite(distance) and range_min <= distance <= range_max
            for distance in ranges
        )

    def errors(self, min_samples: int) -> list[str]:
        result = []
        if self.count < min_samples:
            result.append(f"only {self.count} scans arrived; need at least {min_samples}")
        if self.duplicates:
            result.append(f"{self.duplicates} duplicate timestamps")
        if self.regressions:
            result.append(f"{self.regressions} timestamp regressions")
        if self.invalid_samples:
            result.append(f"{self.invalid_samples} structurally invalid scans")
        if self.finite_points == 0:
            result.append("no finite in-range distance was observed")
        return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--min-samples", type=int, default=10)
    args = parser.parse_args()
    if args.seconds <= 0 or args.min_samples <= 0:
        parser.error("seconds and min-samples must be positive")

    rclpy.init()
    node = rclpy.create_node("tb3_scan_probe")
    audit = ScanAudit()

    def receive(message: LaserScan) -> None:
        audit.receive_values(
            stamp_ns=message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec,
            frame_id=message.header.frame_id,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
            ranges=message.ranges,
        )

    subscription = node.create_subscription(
        LaserScan, "/scan", receive, qos_profile_sensor_data
    )
    deadline = time.monotonic() + args.seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    errors = audit.errors(args.min_samples)
    print(
        f"scans={audit.count} duplicates={audit.duplicates} "
        f"regressions={audit.regressions} invalid={audit.invalid_samples} "
        f"finite_points={audit.finite_points}"
    )
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
