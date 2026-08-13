#!/usr/bin/env python3
"""Run SO-101 follower calibration with an explicit torque-off interlock."""

from __future__ import annotations

import argparse
import select
import sys
import time
from types import MethodType
from typing import Any

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

# These are deliberately much smaller than the normal SO-101 mechanical ranges,
# but large enough to reject a buffered ENTER or a joint that was not moved.
MIN_RANGE_SPAN_TICKS = {
    "shoulder_pan": 1000,
    "shoulder_lift": 1000,
    "elbow_flex": 800,
    "wrist_flex": 1000,
    "gripper": 500,
}
DISPLAY_PERIOD_SECONDS = 0.25


class CalibrationIncompleteError(RuntimeError):
    """The operator stopped range recording before every span was valid."""


class CalibrationCancelledError(RuntimeError):
    """The operator explicitly cancelled range recording."""


def _read_command() -> str | None:
    """Return one pending terminal command without blocking."""
    ready = select.select([sys.stdin], [], [], 0)[0]
    if not ready:
        return None
    return sys.stdin.readline().strip().lower()


def _safe_record_ranges(
    bus: Any,
    motors: list[str] | tuple[str, ...] | None = None,
    display_values: bool = True,
) -> tuple[dict[str, int], dict[str, int]]:
    """Record ranges and stop immediately when the operator requests a decision."""
    motor_names = list(motors) if motors is not None else list(bus.motors)
    expected = set(MIN_RANGE_SPAN_TICKS)
    if set(motor_names) != expected:
        raise RuntimeError(
            f"Safety recorder expected {sorted(expected)}, got {sorted(motor_names)}"
        )

    positions = bus.sync_read("Present_Position", motor_names, normalize=False)
    mins = {name: int(value) for name, value in positions.items()}
    maxes = mins.copy()
    last_display = 0.0
    last_status_width = 0

    print("\nSafety recorder is active.")
    print("Move every listed joint through its safe full range now.")
    print("The status stays on one line: ENTER=check and stop, q=cancel safely.")

    while True:
        positions = {
            name: int(value)
            for name, value in bus.sync_read(
                "Present_Position", motor_names, normalize=False
            ).items()
        }
        for name, value in positions.items():
            mins[name] = min(mins[name], value)
            maxes[name] = max(maxes[name], value)

        now = time.monotonic()
        spans = {name: maxes[name] - mins[name] for name in motor_names}
        ready = all(spans[name] >= MIN_RANGE_SPAN_TICKS[name] for name in motor_names)

        if display_values and now - last_display >= DISPLAY_PERIOD_SECONDS:
            status = " | ".join(
                f"{name}:{spans[name]}/{MIN_RANGE_SPAN_TICKS[name]}"
                + (" OK" if spans[name] >= MIN_RANGE_SPAN_TICKS[name] else "")
                for name in motor_names
            )
            line = "SPAN " + status + "  [Enter=check, q=cancel]"
            sys.stdout.write("\r" + line.ljust(last_status_width))
            sys.stdout.flush()
            last_status_width = max(last_status_width, len(line))
            last_display = now

        command = _read_command()
        if command is not None:
            if display_values:
                sys.stdout.write("\n")
                sys.stdout.flush()
            if command in {"q", "quit"}:
                raise CalibrationCancelledError("Range recording cancelled by operator.")
            if command == "":
                if ready:
                    return mins, maxes
                missing = [
                    f"{name}:{spans[name]}/{MIN_RANGE_SPAN_TICKS[name]}"
                    for name in motor_names
                    if spans[name] < MIN_RANGE_SPAN_TICKS[name]
                ]
                raise CalibrationIncompleteError(
                    "Incomplete spans: " + ", ".join(missing)
                )
            print(f"Unknown command {command!r}; use ENTER or q.")

        time.sleep(0.02)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safely recalibrate an SO-101 follower; torque is disabled before calibration starts."
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Explicit follower serial path, preferably /dev/serial/by-id/...",
    )
    parser.add_argument("--id", default="so101_follower_main")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    device = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.id,
            cameras={},
            disable_torque_on_disconnect=True,
        )
    )

    try:
        print("Connecting to the motor bus...")
        device.bus.connect()

        before = device.bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
        print("Torque state before safety step:", before)
        print("Disabling torque on all six motors...")
        device.bus.disable_torque(num_retry=3)
        after = device.bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
        print("Torque state after safety step:", after)

        enabled = [name for name in MOTOR_NAMES if int(after[name]) != 0]
        if enabled:
            raise RuntimeError(
                "Torque-off interlock failed on: " + ", ".join(enabled) + ". "
                "Keep servo power off and do not move the arm."
            )

        print("All six motors report torque OFF.")
        print(
            "Starting a fresh calibration; the existing JSON will be replaced after completion."
        )
        print(
            "Keep the arm supported and do not press Enter until the displayed pose is safe."
        )

        # Suppress the normal 'use existing calibration?' prompt. The user asked
        # for a fresh calibration, and the explicit torque interlock above has
        # already run before entering the calibration routine.
        device.calibration = {}
        device.bus.record_ranges_of_motion = MethodType(
            _safe_record_ranges, device.bus
        )
        device.calibrate()
        print("Calibration routine completed.")
        return 0
    except CalibrationIncompleteError as exc:
        print(f"\nCalibration stopped without saving: {exc}")
        print("Move every listed joint farther and rerun the same command.")
        return 2
    except CalibrationCancelledError as exc:
        print(f"\n{exc} Nothing was saved.")
        return 130
    except KeyboardInterrupt:
        print("\nInterrupted. Torque will be disabled before disconnecting.")
        return 130
    finally:
        if device.bus.is_connected:
            try:
                device.bus.disable_torque(num_retry=5)
            finally:
                # Torque was explicitly disabled above; avoid another write
                # during close while still guaranteeing a closed serial port.
                device.bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    raise SystemExit(main())
