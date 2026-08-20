"""Read-only Feetech serial probe for identifying connected teaching arms."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Sequence


READ_INSTRUCTION = 0x02
MODEL_NUMBER_REGISTER = 3
ANGLE_LIMITS_REGISTER = 9
ANGLE_LIMITS_SIZE = 4
CALIBRATION_REGISTER = 31
CALIBRATION_SIZE = 3
CONTROL_STATE_REGISTER = 40
CONTROL_STATE_SIZE = 10
PRESENT_POSITION_REGISTER = 56
TELEMETRY_REGISTER = 56
TELEMETRY_SIZE = 15
DEFAULT_MOTOR_IDS = (1, 2, 3, 4, 5, 6)


def build_read_packet(motor_id: int, register: int, size: int = 2) -> bytes:
    """Build one Feetech READ packet; this function cannot encode a write command."""
    for name, value in (("motor_id", motor_id), ("register", register), ("size", size)):
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFF:
            raise ValueError(f"{name} must be an integer from 0 to 255")
    length = 4
    body = [motor_id, length, READ_INSTRUCTION, register, size]
    checksum = (~(sum(body) & 0xFF)) & 0xFF
    return bytes([0xFF, 0xFF, *body, checksum])


def parse_read_response(packet: bytes, *, motor_id: int, payload_size: int) -> bytes:
    """Validate one Feetech READ response and return its payload bytes."""
    expected_size = payload_size + 6
    if len(packet) != expected_size:
        raise ValueError(f"expected {expected_size} response bytes, received {len(packet)}")
    if packet[:2] != b"\xff\xff":
        raise ValueError("response header is invalid")
    if packet[2] != motor_id:
        raise ValueError(f"response motor ID {packet[2]} does not match {motor_id}")
    if packet[3] != payload_size + 2:
        raise ValueError(f"unexpected response length field: {packet[3]}")
    expected_checksum = (~(sum(packet[2:-1]) & 0xFF)) & 0xFF
    if packet[-1] != expected_checksum:
        raise ValueError("response checksum is invalid")
    if packet[4] != 0:
        raise ValueError(f"motor returned error byte 0x{packet[4]:02x}")
    return packet[5:-1]


def parse_position_response(packet: bytes, *, motor_id: int) -> int:
    """Validate an eight-byte status packet and return its 12-bit position."""
    payload = parse_read_response(packet, motor_id=motor_id, payload_size=2)
    position = payload[0] | (payload[1] << 8)
    if not 0 <= position <= 4095:
        raise ValueError(f"position is outside the 12-bit range: {position}")
    return position


def _little_u16(payload: bytes, offset: int) -> int:
    return payload[offset] | (payload[offset + 1] << 8)


def _sign_magnitude(raw: int, sign_bit: int) -> int:
    return -(raw & ~sign_bit) if raw & sign_bit else raw


def decode_telemetry(payload: bytes) -> dict[str, object]:
    """Decode the official STS feedback block without estimating joint torque."""
    if len(payload) != TELEMETRY_SIZE:
        raise ValueError(f"expected {TELEMETRY_SIZE} telemetry bytes, received {len(payload)}")
    position = _sign_magnitude(_little_u16(payload, 0), 1 << 15)
    speed = _sign_magnitude(_little_u16(payload, 2), 1 << 15)
    load = _sign_magnitude(_little_u16(payload, 4), 1 << 10)
    current = _sign_magnitude(_little_u16(payload, 13), 1 << 15)
    return {
        "position_tick": position,
        "speed_raw": speed,
        "load_raw": load,
        "load_fraction": load / 1000.0,
        "voltage_raw": payload[6],
        "voltage_volts": payload[6] * 0.1,
        "temperature_celsius": payload[7],
        "async_write_flag": payload[8],
        "hardware_error_status": payload[9],
        "moving": bool(payload[10]),
        "current_raw": current,
        "current_milliamps": current * 6.5,
        "estimated_joint_torque_nm": None,
    }


def decode_angle_limits(payload: bytes) -> dict[str, int]:
    if len(payload) != ANGLE_LIMITS_SIZE:
        raise ValueError(f"expected {ANGLE_LIMITS_SIZE} angle-limit bytes, received {len(payload)}")
    return {
        "minimum_position_limit_tick": _little_u16(payload, 0),
        "maximum_position_limit_tick": _little_u16(payload, 2),
    }


def decode_calibration(payload: bytes) -> dict[str, int]:
    if len(payload) != CALIBRATION_SIZE:
        raise ValueError(f"expected {CALIBRATION_SIZE} calibration bytes, received {len(payload)}")
    return {
        "position_offset_raw": _little_u16(payload, 0),
        "position_offset_tick": int.from_bytes(payload[0:2], "little", signed=True),
        "operating_mode_raw": payload[2],
    }


def decode_control_state(payload: bytes) -> dict[str, object]:
    if len(payload) != CONTROL_STATE_SIZE:
        raise ValueError(f"expected {CONTROL_STATE_SIZE} control-state bytes, received {len(payload)}")
    torque_limit = _little_u16(payload, 8)
    return {
        "torque_enabled": bool(payload[0]),
        "torque_enable_raw": payload[0],
        "configured_acceleration_raw": payload[1],
        "goal_position_tick": _little_u16(payload, 2),
        "goal_time_raw": _little_u16(payload, 4),
        "goal_speed_raw": _little_u16(payload, 6),
        "torque_limit_raw": torque_limit,
        "torque_limit_fraction": torque_limit / 1000.0,
    }


def _read_registers(
    connection: object,
    motor_id: int,
    *,
    register: int,
    size: int,
    retries: int,
) -> bytes:
    request = build_read_packet(motor_id, register, size)
    last_error = "no response"
    for _ in range(retries):
        connection.reset_input_buffer()
        connection.write(request)
        connection.flush()
        response = connection.read(size + 6)
        try:
            return parse_read_response(response, motor_id=motor_id, payload_size=size)
        except ValueError as error:
            last_error = str(error)
            time.sleep(0.01)
    raise ValueError(last_error)


def _read_position(connection: object, motor_id: int, *, retries: int) -> dict[str, object]:
    try:
        model_payload = _read_registers(
            connection,
            motor_id,
            register=MODEL_NUMBER_REGISTER,
            size=2,
            retries=retries,
        )
        limits_payload = _read_registers(
            connection,
            motor_id,
            register=ANGLE_LIMITS_REGISTER,
            size=ANGLE_LIMITS_SIZE,
            retries=retries,
        )
        calibration_payload = _read_registers(
            connection,
            motor_id,
            register=CALIBRATION_REGISTER,
            size=CALIBRATION_SIZE,
            retries=retries,
        )
        control_payload = _read_registers(
            connection,
            motor_id,
            register=CONTROL_STATE_REGISTER,
            size=CONTROL_STATE_SIZE,
            retries=retries,
        )
        telemetry_payload = _read_registers(
            connection,
            motor_id,
            register=TELEMETRY_REGISTER,
            size=TELEMETRY_SIZE,
            retries=retries,
        )
    except ValueError as error:
        return {"status": "no_valid_response", "error": str(error)}
    return {
        "status": "ok",
        "model_number_raw": _little_u16(model_payload, 0),
        **decode_angle_limits(limits_payload),
        **decode_calibration(calibration_payload),
        **decode_control_state(control_payload),
        **decode_telemetry(telemetry_payload),
    }


def probe_port(
    port: str,
    *,
    baudrate: int = 1_000_000,
    motor_ids: Sequence[int] = DEFAULT_MOTOR_IDS,
    timeout_seconds: float = 0.08,
    retries: int = 3,
) -> dict[str, object]:
    """Open one controller and issue only present-position READ requests."""
    if baudrate <= 0:
        raise ValueError("baudrate must be positive")
    if timeout_seconds <= 0:
        raise ValueError("timeout_seconds must be positive")
    if retries <= 0:
        raise ValueError("retries must be positive")
    if not motor_ids:
        raise ValueError("at least one motor ID is required")

    try:
        import serial
    except ImportError as error:
        raise ValueError("pyserial is missing; do not install it automatically") from error

    result: dict[str, object] = {
        "port": port,
        "baudrate": baudrate,
        "operation": "read_model_configuration_and_telemetry_only",
        "motion_commands_sent": False,
        "torque_commands_sent": False,
        "motors": {},
    }
    connection = serial.Serial()
    connection.port = port
    connection.baudrate = baudrate
    connection.timeout = timeout_seconds
    connection.write_timeout = timeout_seconds
    connection.exclusive = True
    connection.dtr = False
    connection.rts = False
    try:
        connection.open()
        time.sleep(0.05)
        motors = {
            str(motor_id): _read_position(connection, motor_id, retries=retries)
            for motor_id in motor_ids
        }
        result["motors"] = motors
        result["responding_motor_count"] = sum(
            motor["status"] == "ok" for motor in motors.values()
        )
    except (OSError, serial.SerialException) as error:
        result["port_error"] = str(error)
        result["responding_motor_count"] = 0
    finally:
        if connection.is_open:
            connection.close()
    return result


def _motor_ids(value: str) -> tuple[int, ...]:
    try:
        ids = tuple(int(item.strip()) for item in value.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError("motor IDs must be comma-separated integers") from error
    if not ids or any(not 1 <= motor_id <= 253 for motor_id in ids) or len(set(ids)) != len(ids):
        raise argparse.ArgumentTypeError("motor IDs must be unique values from 1 to 253")
    return ids


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Probe Feetech motor model and telemetry without sending torque or motion commands. "
            "Use only with the instructor-confirmed JDcobot protocol."
        )
    )
    parser.add_argument("--port", action="append", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--motor-ids", type=_motor_ids, default=DEFAULT_MOTOR_IDS)
    parser.add_argument("--timeout-seconds", type=float, default=0.08)
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    try:
        results = [
            probe_port(
                port,
                baudrate=args.baudrate,
                motor_ids=args.motor_ids,
                timeout_seconds=args.timeout_seconds,
                retries=args.retries,
            )
            for port in args.port
        ]
    except ValueError as error:
        print(json.dumps({"error": str(error)}, indent=2))
        return 2
    summary = {
        "protocol_basis": "JD-edu/jdcobot200_imitation_learning",
        "read_only": True,
        "results": results,
    }
    if args.output is not None:
        if args.output.exists():
            print(json.dumps({"error": f"output already exists: {args.output}"}, indent=2))
            return 2
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if all(result["responding_motor_count"] > 0 for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
