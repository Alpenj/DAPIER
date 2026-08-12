#!/usr/bin/env python3
"""Verify live ROS 2 transforms through the typed tf2 API."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TfAudit:
    def __init__(self) -> None:
        self.updates = 0
        self.last_stamp_ns: int | None = None
        self.regressions = 0
        self.invalid = 0

    def receive_values(
        self,
        *,
        stamp_ns: int,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
    ) -> None:
        # Re-reading the latest buffered transform is polling, not a duplicate
        # publication. Count only a newly timestamped transform update.
        if stamp_ns == self.last_stamp_ns:
            return
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.regressions += 1
        self.last_stamp_ns = stamp_ns
        self.updates += 1
        quaternion_norm = math.sqrt(sum(component * component for component in rotation))
        if (
            stamp_ns <= 0
            or not all(math.isfinite(value) for value in translation + rotation)
            or not 0.99 <= quaternion_norm <= 1.01
        ):
            self.invalid += 1

    def errors(self, min_updates: int) -> list[str]:
        result = []
        if self.updates < min_updates:
            result.append(
                f"only {self.updates} transform updates arrived; need {min_updates}"
            )
        if self.regressions:
            result.append(f"{self.regressions} timestamp regressions")
        if self.invalid:
            result.append(f"{self.invalid} invalid transforms")
        return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("target_frame")
    parser.add_argument("source_frame")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--min-updates", type=int, default=2)
    args = parser.parse_args()
    if args.seconds <= 0 or args.min_updates <= 0:
        parser.error("seconds and min-updates must be positive")

    rclpy.init()
    node = rclpy.create_node("tb3_tf_probe")
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)
    audit = TfAudit()
    deadline = time.monotonic() + args.seconds
    last_error = "no transform received"

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform(
                args.target_frame, args.source_frame, Time()
            )
        except TransformException as error:
            last_error = str(error)
            continue
        stamp = transform.header.stamp
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        audit.receive_values(
            stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
            translation=(translation.x, translation.y, translation.z),
            rotation=(rotation.x, rotation.y, rotation.z, rotation.w),
        )
        if not audit.errors(args.min_updates):
            break

    errors = audit.errors(args.min_updates)
    print(
        f"target={args.target_frame} source={args.source_frame} "
        f"updates={audit.updates} regressions={audit.regressions} "
        f"invalid={audit.invalid}"
    )
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        print(f"ERROR: last tf2 result: {last_error}")
    del listener
    node.destroy_node()
    rclpy.shutdown()
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
