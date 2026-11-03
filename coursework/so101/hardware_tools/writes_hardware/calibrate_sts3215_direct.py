#!/usr/bin/env python3
"""Calibrate an SO-101 STS3215 bus without importing LeRobot.

This expert fallback talks through scservo_sdk and can emit a
LeRobot-compatible JSON file, but it never imports or calls LeRobot code.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import select
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import scservo_sdk as scs
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing scservo_sdk. Use the project venv that provides "
        "feetech-servo-sdk; this script still does not import LeRobot."
    ) from exc


MOTORS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}
RANGE_MOTORS = tuple(name for name in MOTORS if name != "wrist_roll")
EXPECTED_MODEL_NUMBER = 777
ENCODER_MAX = 4095
ENCODER_MIDPOINT = ENCODER_MAX // 2
BAUDRATE = 1_000_000

ADDR_MIN_POSITION_LIMIT = 9
ADDR_MAX_POSITION_LIMIT = 11
ADDR_HOMING_OFFSET = 31
ADDR_OPERATING_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56

POSITION_MODE = 0
TORQUE_DISABLED = 0
EEPROM_UNLOCKED = 0
CONFIRM_TEXT = "WRITE_STS3215_CALIBRATION"

MIN_RANGE_SPAN_TICKS = {
    "shoulder_pan": 1000,
    "shoulder_lift": 1000,
    "elbow_flex": 800,
    "wrist_flex": 1000,
    "gripper": 500,
}


def encode_sign_magnitude(value: int, sign_bit: int) -> int:
    max_magnitude = (1 << sign_bit) - 1
    if abs(value) > max_magnitude:
        raise ValueError(f"{value=} exceeds sign-magnitude bit {sign_bit}")
    return abs(value) | ((1 << sign_bit) if value < 0 else 0)


def decode_sign_magnitude(value: int, sign_bit: int) -> int:
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if value & (1 << sign_bit) else magnitude


def _enter_pressed() -> bool:
    ready = select.select([sys.stdin], [], [], 0)[0]
    return bool(ready and sys.stdin.readline().strip() == "")


def atomic_write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, ensure_ascii=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        with contextlib.suppress(FileNotFoundError):
            os.unlink(temporary)
        raise


class DirectSTS3215Bus:
    """Checked scservo_sdk protocol-0 wrapper."""

    def __init__(self, port: str, *, retries: int = 3) -> None:
        self.port_name = port
        self.retries = retries
        self.port = scs.PortHandler(port)
        self.packet = scs.PacketHandler(0)

    def __enter__(self) -> DirectSTS3215Bus:
        if not self.port.setBaudRate(BAUDRATE):
            raise RuntimeError(f"Could not open {self.port_name} at {BAUDRATE} baud")
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        if self.port.is_open:
            self.port.closePort()

    def _error(
        self, operation: str, motor_id: int, result: int, error: int
    ) -> RuntimeError:
        return RuntimeError(
            f"{operation} failed for motor {motor_id}: result={result} "
            f"{self.packet.getTxRxResult(result)}; error={error} "
            f"{self.packet.getRxPacketError(error)}"
        )

    def ping(self, motor_id: int) -> int:
        last: tuple[int, int] | None = None
        for _ in range(self.retries):
            model, result, error = self.packet.ping(self.port, motor_id)
            if result == scs.COMM_SUCCESS and error == 0:
                return int(model)
            last = (result, error)
            time.sleep(0.01)
        assert last is not None
        raise self._error("ping", motor_id, *last)

    def read(self, motor_id: int, address: int, size: int) -> int:
        reader = {
            1: self.packet.read1ByteTxRx,
            2: self.packet.read2ByteTxRx,
        }.get(size)
        if reader is None:
            raise ValueError(f"Unsupported register size: {size}")
        last: tuple[int, int] | None = None
        for _ in range(self.retries):
            value, result, error = reader(self.port, motor_id, address)
            if result == scs.COMM_SUCCESS and error == 0:
                return int(value)
            last = (result, error)
            time.sleep(0.01)
        assert last is not None
        raise self._error(f"read address {address}", motor_id, *last)

    def write(self, motor_id: int, address: int, size: int, value: int) -> None:
        writer = {
            1: self.packet.write1ByteTxRx,
            2: self.packet.write2ByteTxRx,
        }.get(size)
        if writer is None:
            raise ValueError(f"Unsupported register size: {size}")
        last: tuple[int, int] | None = None
        for _ in range(self.retries):
            result, error = writer(self.port, motor_id, address, value)
            if result == scs.COMM_SUCCESS and error == 0:
                return
            last = (result, error)
            time.sleep(0.01)
        assert last is not None
        raise self._error(f"write address {address}", motor_id, *last)

    def require_expected_motors(self) -> dict[str, int]:
        models = {}
        for name, motor_id in MOTORS.items():
            model = self.ping(motor_id)
            if model != EXPECTED_MODEL_NUMBER:
                raise RuntimeError(
                    f"{name} id={motor_id}: expected STS3215 model "
                    f"{EXPECTED_MODEL_NUMBER}, got {model}"
                )
            models[name] = model
        return models

    def read_position(self, motor_id: int) -> int:
        return decode_sign_magnitude(
            self.read(motor_id, ADDR_PRESENT_POSITION, 2), 15
        )

    def read_calibration(self) -> dict[str, dict[str, int]]:
        calibration = {}
        for name, motor_id in MOTORS.items():
            calibration[name] = {
                "id": motor_id,
                "drive_mode": 0,
                "homing_offset": decode_sign_magnitude(
                    self.read(motor_id, ADDR_HOMING_OFFSET, 2), 11
                ),
                "range_min": self.read(
                    motor_id, ADDR_MIN_POSITION_LIMIT, 2
                ),
                "range_max": self.read(
                    motor_id, ADDR_MAX_POSITION_LIMIT, 2
                ),
            }
        return calibration

    def torque_state(self) -> dict[str, int]:
        return {
            name: self.read(motor_id, ADDR_TORQUE_ENABLE, 1)
            for name, motor_id in MOTORS.items()
        }

    def disable_torque_and_unlock(self) -> None:
        for motor_id in MOTORS.values():
            self.write(motor_id, ADDR_TORQUE_ENABLE, 1, TORQUE_DISABLED)
            self.write(motor_id, ADDR_LOCK, 1, EEPROM_UNLOCKED)
        enabled = [name for name, value in self.torque_state().items() if value]
        if enabled:
            raise RuntimeError(
                "Torque-off verification failed: " + ", ".join(enabled)
            )

    def write_calibration(self, calibration: dict[str, dict[str, int]]) -> None:
        for name, motor_id in MOTORS.items():
            record = calibration[name]
            self.write(
                motor_id,
                ADDR_HOMING_OFFSET,
                2,
                encode_sign_magnitude(int(record["homing_offset"]), 11),
            )
            self.write(
                motor_id,
                ADDR_MIN_POSITION_LIMIT,
                2,
                int(record["range_min"]),
            )
            self.write(
                motor_id,
                ADDR_MAX_POSITION_LIMIT,
                2,
                int(record["range_max"]),
            )


def validate_calibration(calibration: dict[str, Any]) -> None:
    if set(calibration) != set(MOTORS):
        raise ValueError("Calibration motor names do not match SO-101")
    for name, motor_id in MOTORS.items():
        record = calibration[name]
        if int(record["id"]) != motor_id:
            raise ValueError(f"{name}: expected id {motor_id}, got {record['id']}")
        minimum = int(record["range_min"])
        maximum = int(record["range_max"])
        if not 0 <= minimum < maximum <= ENCODER_MAX:
            raise ValueError(f"{name}: invalid range {minimum}..{maximum}")
        if name == "wrist_roll" and (minimum, maximum) != (0, ENCODER_MAX):
            raise ValueError("wrist_roll must use 0..4095")
        if (
            name != "wrist_roll"
            and maximum - minimum < MIN_RANGE_SPAN_TICKS[name]
        ):
            raise ValueError(
                f"{name}: span {maximum - minimum} is below "
                f"{MIN_RANGE_SPAN_TICKS[name]}"
            )
        encode_sign_magnitude(int(record["homing_offset"]), 11)


def record_ranges(
    bus: DirectSTS3215Bus, *, minimum_seconds: float
) -> tuple[dict[str, int], dict[str, int]]:
    if not sys.stdin.isatty():
        raise RuntimeError("Interactive range recording requires a TTY")
    positions = {
        name: bus.read_position(MOTORS[name]) for name in RANGE_MOTORS
    }
    mins = positions.copy()
    maxes = positions.copy()
    started = time.monotonic()
    last_display = 0.0

    print("\nDirect SDK range recorder is active.")
    print("Move every listed joint through its safe full range.")
    print(
        f"ENTER is ignored for {minimum_seconds:.0f}s and until every STATE is OK."
    )
    while True:
        positions = {
            name: bus.read_position(MOTORS[name]) for name in RANGE_MOTORS
        }
        for name, value in positions.items():
            mins[name] = min(mins[name], value)
            maxes[name] = max(maxes[name], value)
        spans = {name: maxes[name] - mins[name] for name in RANGE_MOTORS}
        ready = all(
            spans[name] >= MIN_RANGE_SPAN_TICKS[name] for name in RANGE_MOTORS
        )
        now = time.monotonic()

        if now - last_display >= 0.25:
            print("\n-------------------------------------------------------")
            print(
                f"{'NAME':<15} | {'MIN':>5} | {'POS':>5} | "
                f"{'MAX':>5} | {'SPAN':>5} | STATE"
            )
            for name in RANGE_MOTORS:
                state = (
                    "OK"
                    if spans[name] >= MIN_RANGE_SPAN_TICKS[name]
                    else "MOVE"
                )
                print(
                    f"{name:<15} | {mins[name]:>5} | {positions[name]:>5} | "
                    f"{maxes[name]:>5} | {spans[name]:>5} | {state}"
                )
            print("When every STATE is OK, press ENTER once to save.")
            last_display = now

        if _enter_pressed():
            elapsed = now - started
            if elapsed < minimum_seconds:
                print(f"Ignored early ENTER at {elapsed:.1f}s.")
            elif not ready:
                missing = [
                    f"{name}:{spans[name]}/{MIN_RANGE_SPAN_TICKS[name]}"
                    for name in RANGE_MOTORS
                    if spans[name] < MIN_RANGE_SPAN_TICKS[name]
                ]
                print("Not saved; incomplete spans: " + ", ".join(missing))
            else:
                return mins, maxes
        time.sleep(0.02)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("inspect", "calibrate", "restore"), default="inspect"
    )
    parser.add_argument(
        "--role", choices=("follower", "leader"), required=True
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--backup", type=Path)
    parser.add_argument("--receipt", type=Path)
    parser.add_argument("--minimum-seconds", type=float, default=5.0)
    parser.add_argument("--confirm", default="")
    return parser


def inspect_bus(bus: DirectSTS3215Bus, role: str) -> dict[str, Any]:
    result = {
        "timestamp": datetime.now().astimezone().isoformat(),
        "role": role,
        "port": bus.port_name,
        "baudrate": BAUDRATE,
        "models": bus.require_expected_motors(),
        "torque_enable": bus.torque_state(),
        "positions": {
            name: bus.read_position(motor_id)
            for name, motor_id in MOTORS.items()
        },
        "calibration": bus.read_calibration(),
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return result


def run_calibration(
    bus: DirectSTS3215Bus, args: argparse.Namespace
) -> int:
    if args.output is None:
        raise SystemExit("--output is required for --mode calibrate")
    if args.confirm != CONFIRM_TEXT:
        raise SystemExit(
            f"Refusing EEPROM writes. Add --confirm {CONFIRM_TEXT} "
            "after reviewing the backup path."
        )
    if args.minimum_seconds < 5:
        raise SystemExit("--minimum-seconds must be at least 5")

    models = bus.require_expected_motors()
    before = bus.read_calibration()
    backup = args.backup or args.output.with_name(
        f"{args.output.stem}.direct-before.json"
    )
    atomic_write_json(backup, before)
    print(f"Register backup saved: {backup}")

    bus.disable_torque_and_unlock()
    print("All six motors report torque OFF.")
    for motor_id in MOTORS.values():
        bus.write(motor_id, ADDR_OPERATING_MODE, 1, POSITION_MODE)
        bus.write(motor_id, ADDR_HOMING_OFFSET, 2, 0)
        bus.write(motor_id, ADDR_MIN_POSITION_LIMIT, 2, 0)
        bus.write(motor_id, ADDR_MAX_POSITION_LIMIT, 2, ENCODER_MAX)

    input(
        "Support the arm, move every joint to the middle of its range, "
        "then press ENTER once..."
    )
    homing_offsets = {}
    for name, motor_id in MOTORS.items():
        actual = bus.read_position(motor_id)
        offset = actual - ENCODER_MIDPOINT
        bus.write(
            motor_id,
            ADDR_HOMING_OFFSET,
            2,
            encode_sign_magnitude(offset, 11),
        )
        homing_offsets[name] = offset

    centered = {
        name: bus.read_position(motor_id)
        for name, motor_id in MOTORS.items()
    }
    bad_center = {
        name: value
        for name, value in centered.items()
        if abs(value - ENCODER_MIDPOINT) > 20
    }
    if bad_center:
        raise RuntimeError(f"Homing verification failed: {bad_center}")

    mins, maxes = record_ranges(
        bus, minimum_seconds=args.minimum_seconds
    )
    calibration = {}
    for name, motor_id in MOTORS.items():
        calibration[name] = {
            "id": motor_id,
            "drive_mode": 0,
            "homing_offset": homing_offsets[name],
            "range_min": 0 if name == "wrist_roll" else mins[name],
            "range_max": ENCODER_MAX if name == "wrist_roll" else maxes[name],
        }
    validate_calibration(calibration)
    bus.write_calibration(calibration)
    observed = bus.read_calibration()
    if observed != calibration:
        raise RuntimeError(
            "Register verification differs from requested calibration"
        )
    atomic_write_json(args.output, calibration)

    receipt = {
        "timestamp": datetime.now().astimezone().isoformat(),
        "method": "direct-scservo-sdk-no-lerobot-import",
        "role": args.role,
        "port": args.port,
        "baudrate": BAUDRATE,
        "models": models,
        "backup": str(backup),
        "output": str(args.output),
        "torque_enable_after": bus.torque_state(),
    }
    receipt_path = args.receipt or args.output.with_name(
        f"{args.output.stem}.direct-receipt.json"
    )
    atomic_write_json(receipt_path, receipt)
    print(f"Calibration saved: {args.output}")
    print(f"Receipt saved: {receipt_path}")
    return 0


def run_restore(bus: DirectSTS3215Bus, args: argparse.Namespace) -> int:
    if args.input is None:
        raise SystemExit("--input is required for --mode restore")
    if args.confirm != CONFIRM_TEXT:
        raise SystemExit(
            f"Refusing EEPROM writes. Add --confirm {CONFIRM_TEXT}."
        )
    calibration = json.loads(args.input.read_text(encoding="utf-8"))
    validate_calibration(calibration)
    bus.require_expected_motors()
    bus.disable_torque_and_unlock()
    bus.write_calibration(calibration)
    if bus.read_calibration() != calibration:
        raise RuntimeError("Restore verification failed")
    print(f"Restored and verified calibration from {args.input}")
    return 0


def main() -> int:
    args = build_parser().parse_args()
    with DirectSTS3215Bus(args.port) as bus:
        try:
            if args.mode == "inspect":
                result = inspect_bus(bus, args.role)
                if args.output is not None:
                    atomic_write_json(args.output, result)
                return 0
            if args.mode == "calibrate":
                return run_calibration(bus, args)
            return run_restore(bus, args)
        finally:
            if args.mode in {"calibrate", "restore"}:
                try:
                    bus.disable_torque_and_unlock()
                except Exception as exc:
                    print(
                        f"WARNING: final torque-off verification failed: {exc}",
                        file=sys.stderr,
                    )


if __name__ == "__main__":
    raise SystemExit(main())
