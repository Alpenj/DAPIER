"""Fail-closed single-joint micro-jog for a confirmed JDcobot200 arm.

The only writable addresses are STS3215 RAM registers 40-49. EEPROM, IDs,
offsets, limits, and calibration are deliberately outside this module.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import time
from typing import Any, Mapping, Sequence

from shoe_sorting_data.arm_probe import DEFAULT_MOTOR_IDS, _read_position


WRITE_INSTRUCTION = 0x03
TORQUE_ENABLE_REGISTER = 40
CONTROL_BLOCK_LAST_REGISTER = 49
CONFIRM_TEXT = "JOG_SINGLE_ARM_MICRO"
EXPECTED_MODEL_NUMBER = 777
SCHEMA_VERSION = "dapier.jdcobot-micro-jog.v0.1"
_CONTROLLER_NAME = re.compile(r"usb-1a86_USB_Single_Serial_(?P<serial>[A-Za-z0-9]+)-if00")
FULL_RANGE_PROFILE = "joint-full-range"
TICKS_PER_REVOLUTION = 4096
FULL_TORQUE_LIMIT_RAW = 1000


class JogSafetyError(ValueError):
    """A fail-closed preflight or runtime interlock rejected the jog."""


@dataclass(frozen=True)
class JogPolicy:
    max_abs_delta_ticks: int | None = 48
    max_speed_raw: int = 120
    max_acceleration_raw: int = 40
    max_torque_limit_raw: int = FULL_TORQUE_LIMIT_RAW
    min_voltage_volts: float = 11.0
    max_voltage_volts: float = 13.0
    max_temperature_celsius: int = 45
    max_start_current_milliamps: float = 100.0
    max_runtime_current_milliamps: float = 650.0
    max_runtime_load_fraction: float = 0.35
    position_margin_ticks: int = 64
    max_extra_travel_ticks: int = 8
    target_tolerance_ticks: int = 3
    max_hold_seconds: float = 1.0


def _checked_int(name: str, value: int, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise JogSafetyError(f"{name} must be an integer from {minimum} to {maximum}")
    return value


def _write_packet(motor_id: int, register: int, payload: bytes) -> bytes:
    _checked_int("motor_id", motor_id, 1, 253)
    _checked_int("register", register, TORQUE_ENABLE_REGISTER, CONTROL_BLOCK_LAST_REGISTER)
    if not payload:
        raise JogSafetyError("RAM write payload must not be empty")
    if register + len(payload) - 1 > CONTROL_BLOCK_LAST_REGISTER:
        raise JogSafetyError("RAM write must stay inside STS3215 registers 40-49")
    parameters = bytes([register]) + payload
    body = bytes([motor_id, len(parameters) + 2, WRITE_INSTRUCTION]) + parameters
    checksum = (~(sum(body) & 0xFF)) & 0xFF
    return bytes([0xFF, 0xFF]) + body + bytes([checksum])


def build_torque_disable_packet(motor_id: int) -> bytes:
    """Build the only standalone write allowed: Torque_Enable=0."""

    return _write_packet(motor_id, TORQUE_ENABLE_REGISTER, bytes([0]))


def build_guarded_motion_packet(
    motor_id: int,
    *,
    target_tick: int,
    speed_raw: int,
    acceleration_raw: int,
    torque_limit_raw: int,
) -> bytes:
    """Write torque, profile, goal, and torque limit in one RAM transaction."""

    _checked_int("target_tick", target_tick, 0, 4095)
    _checked_int("speed_raw", speed_raw, 1, 4095)
    _checked_int("acceleration_raw", acceleration_raw, 1, 254)
    _checked_int("torque_limit_raw", torque_limit_raw, 1, 1000)
    payload = bytes(
        [
            1,
            acceleration_raw,
            target_tick & 0xFF,
            (target_tick >> 8) & 0xFF,
            0,
            0,
            speed_raw & 0xFF,
            (speed_raw >> 8) & 0xFF,
            torque_limit_raw & 0xFF,
            (torque_limit_raw >> 8) & 0xFF,
        ]
    )
    return _write_packet(motor_id, TORQUE_ENABLE_REGISTER, payload)


def _validate_command(
    *,
    motor_id: int,
    delta_ticks: int,
    speed_raw: int,
    acceleration_raw: int,
    torque_limit_raw: int,
    hold_seconds: float,
    policy: JogPolicy,
) -> None:
    _checked_int("motor_id", motor_id, 1, 6)
    if isinstance(delta_ticks, bool) or not isinstance(delta_ticks, int) or delta_ticks == 0:
        raise JogSafetyError("delta_ticks must be a non-zero integer")
    if policy.max_abs_delta_ticks is not None and abs(delta_ticks) > policy.max_abs_delta_ticks:
        raise JogSafetyError(f"absolute delta_ticks must not exceed {policy.max_abs_delta_ticks}")
    _checked_int("speed_raw", speed_raw, 1, policy.max_speed_raw)
    _checked_int("acceleration_raw", acceleration_raw, 1, policy.max_acceleration_raw)
    _checked_int("torque_limit_raw", torque_limit_raw, 1, policy.max_torque_limit_raw)
    if isinstance(hold_seconds, bool) or not 0.1 <= hold_seconds <= policy.max_hold_seconds:
        raise JogSafetyError(f"hold_seconds must be from 0.1 to {policy.max_hold_seconds}")


def _validate_preflight_state(
    motor_id: int,
    state: Mapping[str, object],
    *,
    policy: JogPolicy,
) -> None:
    if state.get("status") != "ok":
        raise JogSafetyError(f"motor {motor_id} did not return a valid state")
    if state.get("model_number_raw") != EXPECTED_MODEL_NUMBER:
        raise JogSafetyError(f"motor {motor_id} is not the confirmed STS3215 model")
    if state.get("operating_mode_raw") != 0:
        raise JogSafetyError(f"motor {motor_id} is not in position mode")
    if state.get("torque_enabled") is not False:
        raise JogSafetyError(f"motor {motor_id} torque is already enabled")
    if state.get("moving") is not False:
        raise JogSafetyError(f"motor {motor_id} is already moving")
    if state.get("hardware_error_status") != 0:
        raise JogSafetyError(f"motor {motor_id} reports a hardware error")
    voltage = float(state["voltage_volts"])
    if not policy.min_voltage_volts <= voltage <= policy.max_voltage_volts:
        raise JogSafetyError(f"motor {motor_id} voltage is outside the guarded range: {voltage}")
    temperature = int(state["temperature_celsius"])
    if temperature > policy.max_temperature_celsius:
        raise JogSafetyError(f"motor {motor_id} temperature is too high: {temperature}")
    if abs(float(state["current_milliamps"])) > policy.max_start_current_milliamps:
        raise JogSafetyError(f"motor {motor_id} has unexpected current while torque is off")
    position = int(state["position_tick"])
    lower = int(state["minimum_position_limit_tick"])
    upper = int(state["maximum_position_limit_tick"])
    if not lower <= position <= upper:
        raise JogSafetyError(
            f"motor {motor_id} position {position} is outside stored limits {lower}..{upper}"
        )


def validate_preflight(
    states: Mapping[int, Mapping[str, object]],
    *,
    target_motor_id: int,
    delta_ticks: int,
    policy: JogPolicy,
) -> int:
    """Validate all six motors and return a guarded target tick."""

    if set(states) != set(DEFAULT_MOTOR_IDS):
        raise JogSafetyError("exactly motor IDs 1-6 must respond before a jog")
    for motor_id in DEFAULT_MOTOR_IDS:
        _validate_preflight_state(motor_id, states[motor_id], policy=policy)
    target_state = states[target_motor_id]
    start = int(target_state["position_tick"])
    target = start + delta_ticks
    lower = int(target_state["minimum_position_limit_tick"]) + policy.position_margin_ticks
    upper = int(target_state["maximum_position_limit_tick"]) - policy.position_margin_ticks
    if lower >= upper or not lower <= start <= upper or not lower <= target <= upper:
        raise JogSafetyError(
            f"motor {target_motor_id} start/target must stay inside guarded limits {lower}..{upper}"
        )
    return target


def confirm_temperature_sample(
    state: Mapping[str, object],
    read_again,
    *,
    maximum_celsius: int,
) -> dict[str, object]:
    """Retry one physically implausible high sample; preserve confirmed high values."""

    current = dict(state)
    if current.get("status") != "ok" or int(current["temperature_celsius"]) <= maximum_celsius:
        return current
    confirmation = dict(read_again())
    if confirmation.get("status") == "ok":
        confirmation["temperature_retry_triggered"] = True
        confirmation["first_temperature_celsius"] = current["temperature_celsius"]
        return confirmation
    return current


def _public_state(state: Mapping[str, object]) -> dict[str, object]:
    keys = (
        "position_tick",
        "minimum_position_limit_tick",
        "maximum_position_limit_tick",
        "position_offset_tick",
        "torque_enabled",
        "moving",
        "voltage_volts",
        "temperature_celsius",
        "current_milliamps",
        "load_fraction",
        "hardware_error_status",
    )
    public = {key: state[key] for key in keys}
    for key in ("temperature_retry_triggered", "first_temperature_celsius"):
        if key in state:
            public[key] = state[key]
    return public


def read_preflight(bus, *, target_motor_id: int, delta_ticks: int, policy: JogPolicy) -> dict[str, Any]:
    states = {motor_id: bus.read_state(motor_id) for motor_id in DEFAULT_MOTOR_IDS}
    target = validate_preflight(
        states,
        target_motor_id=target_motor_id,
        delta_ticks=delta_ticks,
        policy=policy,
    )
    return {
        "status": "PASS",
        "read_only": True,
        "hardware_writes_sent": False,
        "target_motor_id": target_motor_id,
        "start_tick": states[target_motor_id]["position_tick"],
        "target_tick": target,
        "motors": {str(motor_id): _public_state(states[motor_id]) for motor_id in DEFAULT_MOTOR_IDS},
    }


def _runtime_interlock(
    state: Mapping[str, object],
    *,
    start_tick: int,
    delta_ticks: int,
    policy: JogPolicy,
) -> None:
    if state.get("status") != "ok":
        raise JogSafetyError("motor state became unreadable during jog")
    if state.get("hardware_error_status") != 0:
        raise JogSafetyError("hardware error appeared during jog")
    if not policy.min_voltage_volts <= float(state["voltage_volts"]) <= policy.max_voltage_volts:
        raise JogSafetyError("voltage left the guarded range during jog")
    if int(state["temperature_celsius"]) > policy.max_temperature_celsius:
        raise JogSafetyError("temperature exceeded the guarded limit during jog")
    if abs(float(state["current_milliamps"])) > policy.max_runtime_current_milliamps:
        raise JogSafetyError("current exceeded the guarded limit during jog")
    if abs(float(state["load_fraction"])) > policy.max_runtime_load_fraction:
        raise JogSafetyError("load exceeded the guarded limit during jog")
    if abs(int(state["position_tick"]) - start_tick) > abs(delta_ticks) + policy.max_extra_travel_ticks:
        raise JogSafetyError("position exceeded the guarded travel envelope")


def execute_guarded_jog(
    bus,
    *,
    motor_id: int = 1,
    delta_ticks: int = 8,
    speed_raw: int = 30,
    acceleration_raw: int = 10,
    torque_limit_raw: int = FULL_TORQUE_LIMIT_RAW,
    hold_seconds: float = 1.0,
    policy: JogPolicy = JogPolicy(),
    clock=time.monotonic,
    sleep=time.sleep,
) -> dict[str, Any]:
    """Run one bounded jog and verify that every motor ends torque-off."""

    _validate_command(
        motor_id=motor_id,
        delta_ticks=delta_ticks,
        speed_raw=speed_raw,
        acceleration_raw=acceleration_raw,
        torque_limit_raw=torque_limit_raw,
        hold_seconds=hold_seconds,
        policy=policy,
    )
    result: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "created_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "status": "BLOCKED",
        "motor_id": motor_id,
        "delta_ticks": delta_ticks,
        "speed_raw": speed_raw,
        "acceleration_raw": acceleration_raw,
        "torque_limit_raw": torque_limit_raw,
        "motion_command_sent": False,
        "eeprom_writes_sent": False,
        "torque_off_verified": False,
        "samples": [],
    }
    motion_started = False
    failure: str | None = None
    try:
        preflight = read_preflight(
            bus,
            target_motor_id=motor_id,
            delta_ticks=delta_ticks,
            policy=policy,
        )
        result["preflight"] = preflight
        start_tick = int(preflight["start_tick"])
        target_tick = int(preflight["target_tick"])
        bus.start_motion(
            motor_id,
            target_tick=target_tick,
            speed_raw=speed_raw,
            acceleration_raw=acceleration_raw,
            torque_limit_raw=torque_limit_raw,
        )
        motion_started = True
        result["motion_command_sent"] = True
        deadline = clock() + hold_seconds
        reached = False
        while clock() < deadline:
            state = bus.read_state(motor_id)
            sample = _public_state(state)
            sample["elapsed_seconds"] = max(0.0, hold_seconds - max(0.0, deadline - clock()))
            result["samples"].append(sample)
            _runtime_interlock(
                state,
                start_tick=start_tick,
                delta_ticks=delta_ticks,
                policy=policy,
            )
            if abs(int(state["position_tick"]) - target_tick) <= policy.target_tolerance_ticks:
                reached = True
                break
            sleep(0.02)
        result["target_reached"] = reached
        if not reached:
            failure = "target was not reached before the bounded hold deadline"
    except (JogSafetyError, OSError, ValueError, KeyboardInterrupt) as error:
        failure = "operator interrupt" if isinstance(error, KeyboardInterrupt) else str(error)
    finally:
        if motion_started:
            try:
                bus.disable_all_torque(DEFAULT_MOTOR_IDS)
                final_states = {motor_id: bus.read_state(motor_id) for motor_id in DEFAULT_MOTOR_IDS}
                result["final_motors"] = {
                    str(item): _public_state(final_states[item]) for item in DEFAULT_MOTOR_IDS
                }
                result["torque_off_verified"] = all(
                    state.get("status") == "ok" and state.get("torque_enabled") is False
                    for state in final_states.values()
                )
                if not result["torque_off_verified"]:
                    failure = "final torque-off verification failed"
            except (JogSafetyError, OSError, ValueError, KeyError) as error:
                result["shutdown_error"] = str(error)
                failure = "final torque-off verification could not complete"
    if failure is None and result.get("target_reached") and result["torque_off_verified"]:
        result["status"] = "PASS"
    else:
        result["status"] = "FAIL" if motion_started else "BLOCKED"
        result["failure"] = failure or "jog did not satisfy the guarded contract"
    return result


def execute_joint_full_range(
    bus,
    *,
    motor_id: int,
    delta_degrees: float = 40.0,
    clock=time.monotonic,
    sleep=time.sleep,
) -> dict[str, Any]:
    """Move any motor ID 1-6 across its full stored position range."""

    if isinstance(delta_degrees, bool) or not isinstance(delta_degrees, (int, float)):
        raise JogSafetyError("delta_degrees must be a finite non-zero number")
    if not math.isfinite(delta_degrees) or delta_degrees == 0:
        raise JogSafetyError("delta_degrees must be a finite non-zero number")
    delta_ticks = round(float(delta_degrees) * TICKS_PER_REVOLUTION / 360.0)
    if delta_ticks == 0:
        raise JogSafetyError("delta_degrees is smaller than one motor tick")
    policy = JogPolicy(
        max_abs_delta_ticks=None,
        position_margin_ticks=0,
        target_tolerance_ticks=5,
        max_hold_seconds=15.0,
    )
    hold_seconds = min(15.0, max(5.0, abs(delta_ticks) / 90.0))
    result = execute_guarded_jog(
        bus,
        motor_id=motor_id,
        delta_ticks=delta_ticks,
        speed_raw=120,
        acceleration_raw=40,
        torque_limit_raw=FULL_TORQUE_LIMIT_RAW,
        hold_seconds=hold_seconds,
        policy=policy,
        clock=clock,
        sleep=sleep,
    )
    result["profile"] = FULL_RANGE_PROFILE
    result["requested_degrees"] = float(delta_degrees)
    result["commanded_degrees"] = delta_ticks * 360.0 / TICKS_PER_REVOLUTION
    result["artificial_delta_limit"] = None
    result["position_margin_ticks"] = 0
    return result


class STS3215Bus:
    """Narrow serial transport that cannot write outside the guarded RAM block."""

    def __init__(self, port: str, *, timeout_seconds: float = 0.08) -> None:
        try:
            import serial
        except ImportError as error:
            raise JogSafetyError("pyserial is missing; do not install it automatically") from error
        self.connection = serial.Serial()
        self.connection.port = port
        self.connection.baudrate = 1_000_000
        self.connection.timeout = timeout_seconds
        self.connection.write_timeout = timeout_seconds
        self.connection.exclusive = True
        self.connection.dtr = False
        self.connection.rts = False
        self.connection.open()
        time.sleep(0.05)

    def read_state(self, motor_id: int) -> dict[str, object]:
        initial = _read_position(self.connection, motor_id, retries=3)
        return confirm_temperature_sample(
            initial,
            lambda: _read_position(self.connection, motor_id, retries=3),
            maximum_celsius=JogPolicy().max_temperature_celsius,
        )

    def _send(self, packet: bytes) -> None:
        written = self.connection.write(packet)
        self.connection.flush()
        if written != len(packet):
            raise OSError(f"short serial write: {written}/{len(packet)}")

    def start_motion(
        self,
        motor_id: int,
        *,
        target_tick: int,
        speed_raw: int,
        acceleration_raw: int,
        torque_limit_raw: int,
    ) -> None:
        self._send(
            build_guarded_motion_packet(
                motor_id,
                target_tick=target_tick,
                speed_raw=speed_raw,
                acceleration_raw=acceleration_raw,
                torque_limit_raw=torque_limit_raw,
            )
        )

    def disable_all_torque(self, motor_ids: Sequence[int]) -> None:
        for _ in range(2):
            for motor_id in motor_ids:
                self._send(build_torque_disable_packet(motor_id))
            time.sleep(0.02)

    def close(self) -> None:
        if self.connection.is_open:
            self.connection.close()


def verify_controller_identity(port: str, expected_serial: str) -> dict[str, str]:
    """Require one exact QinHeng by-id path without returning its serial."""

    path = Path(port)
    if path.parent != Path("/dev/serial/by-id"):
        raise JogSafetyError("port must use a stable /dev/serial/by-id path")
    match = _CONTROLLER_NAME.fullmatch(path.name)
    if match is None:
        raise JogSafetyError("port is not the confirmed QinHeng USB Single Serial controller")
    candidates = list(path.parent.glob("usb-1a86_USB_Single_Serial_*-if00"))
    if path not in candidates:
        raise JogSafetyError("selected JDcobot controller is not connected")
    observed_serial = match.group("serial")
    if observed_serial != expected_serial:
        raise JogSafetyError("controller serial does not match the operator-approved arm")
    resolved = path.resolve(strict=True)
    if not resolved.name.startswith("ttyACM"):
        raise JogSafetyError("controller did not resolve to a ttyACM device")
    return {
        "identity_sha256": hashlib.sha256(observed_serial.encode("ascii")).hexdigest(),
        "resolved_device": str(resolved),
    }


def _run_hardware(args: argparse.Namespace) -> dict[str, Any]:
    identity = verify_controller_identity(args.port, args.expected_serial)
    bus = STS3215Bus(args.port)
    try:
        if args.command == "preflight":
            result = read_preflight(
                bus,
                target_motor_id=args.motor_id,
                delta_ticks=args.delta_ticks,
                policy=JogPolicy(),
            )
        elif args.command == FULL_RANGE_PROFILE:
            result = execute_joint_full_range(
                bus,
                motor_id=args.motor_id,
                delta_degrees=args.delta_degrees,
            )
        else:
            result = execute_guarded_jog(
                bus,
                motor_id=args.motor_id,
                delta_ticks=args.delta_ticks,
                speed_raw=args.speed_raw,
                acceleration_raw=args.acceleration_raw,
                torque_limit_raw=args.torque_limit_raw,
                hold_seconds=args.hold_seconds,
            )
    finally:
        bus.close()
    result["controller"] = identity
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Fail-closed JDcobot200 single-joint micro-jog")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name in ("preflight", "jog"):
        command = subparsers.add_parser(name)
        command.add_argument("--port", required=True)
        command.add_argument("--expected-serial", required=True)
        command.add_argument("--motor-id", type=int, default=1)
        command.add_argument("--delta-ticks", type=int, default=8)
        if name == "jog":
            command.add_argument("--speed-raw", type=int, default=30)
            command.add_argument("--acceleration-raw", type=int, default=10)
            command.add_argument(
                "--torque-limit-raw", type=int, default=FULL_TORQUE_LIMIT_RAW
            )
            command.add_argument("--hold-seconds", type=float, default=1.0)
            command.add_argument("--confirm", required=True)
    full_range = subparsers.add_parser(
        FULL_RANGE_PROFILE,
        help="move motor ID 1-6 across its full stored position range",
    )
    full_range.add_argument("--port", required=True)
    full_range.add_argument("--expected-serial", required=True)
    full_range.add_argument("--motor-id", type=int, choices=DEFAULT_MOTOR_IDS, required=True)
    full_range.add_argument("--delta-degrees", type=float, default=40.0)
    full_range.add_argument("--confirm", required=True)
    args = parser.parse_args(argv)
    if args.command in ("jog", FULL_RANGE_PROFILE) and args.confirm != CONFIRM_TEXT:
        parser.error(f"--confirm must be exactly {CONFIRM_TEXT}")

    previous_handlers: dict[int, Any] = {}

    def _abort(_signum, _frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, _abort)
    try:
        result = _run_hardware(args)
    except (JogSafetyError, OSError, KeyboardInterrupt) as error:
        result = {
            "schema_version": SCHEMA_VERSION,
            "status": "BLOCKED",
            "motion_command_sent": False,
            "error": "operator interrupt" if isinstance(error, KeyboardInterrupt) else str(error),
        }
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
