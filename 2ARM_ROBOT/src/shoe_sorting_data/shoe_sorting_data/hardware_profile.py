"""Build a DAPIER-specific hardware profile from read-only measurements."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from typing import Mapping, Sequence

from shoe_sorting_data.base_baseline import derive_stationary_tolerances


PROFILE_SCHEMA_VERSION = "dapier.hardware-profile.v0.1"
ARM_MOTOR_ROLES = {
    "1": "base_yaw",
    "2": "shoulder_pitch",
    "3": "elbow_pitch",
    "4": "wrist_pitch",
    "5": "wrist_roll",
    "6": "gripper",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_object(path: Path) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"input not found: {path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON in {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"JSON top level must be an object: {path}")
    return value


def build_hardware_profile(
    arm_snapshot: Mapping[str, object],
    base_baseline: Mapping[str, object],
    *,
    arm_snapshot_sha256: str,
    base_baseline_sha256: str,
) -> dict[str, object]:
    if arm_snapshot.get("read_only") is not True:
        raise ValueError("arm snapshot must declare read_only=true")
    if base_baseline.get("motion_commands_sent") is not False:
        raise ValueError("base baseline must declare motion_commands_sent=false")
    results = arm_snapshot.get("results")
    if not isinstance(results, list) or len(results) != 2:
        raise ValueError("arm snapshot must contain exactly two arm results")
    baseline_summary = base_baseline.get("summary")
    if not isinstance(baseline_summary, Mapping):
        raise ValueError("base baseline summary must be an object")
    tolerances = derive_stationary_tolerances(baseline_summary)

    arms: dict[str, object] = {}
    for index, raw_result in enumerate(results):
        if not isinstance(raw_result, Mapping):
            raise ValueError("each arm result must be an object")
        motors = raw_result.get("motors")
        if not isinstance(motors, Mapping) or set(motors) != set(ARM_MOTOR_ROLES):
            raise ValueError("each arm must contain motor IDs 1 through 6")
        if any(
            not isinstance(motor, Mapping)
            or motor.get("status") != "ok"
            or motor.get("model_number_raw") != 777
            for motor in motors.values()
        ):
            raise ValueError("all arm motors must be responding STS3215 devices (model 777)")
        alias = f"arm_{chr(ord('a') + index)}"
        observed_voltages = [float(motor["voltage_volts"]) for motor in motors.values()]
        supply_class = "12V" if min(observed_voltages) >= 10.0 else "below_10V"
        arms[alias] = {
            "semantic_side": None,
            "semantic_side_status": "pending_physical_label",
            "serial_device_path": raw_result.get("port"),
            "baudrate": raw_result.get("baudrate"),
            "servo_model": "STS3215",
            "servo_model_number": 777,
            "supply_class": supply_class,
            "observed_supply_voltage_range": [min(observed_voltages), max(observed_voltages)],
            "motor_roles": ARM_MOTOR_ROLES,
            "motor_configuration": motors,
            "joint_axis_direction_status": "pending_physical_validation",
            "kinematic_zero_status": "device_offsets_captured_not_motion_validated",
        }

    return {
        "schema_version": PROFILE_SCHEMA_VERSION,
        "created_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "platform": "JDcobot200_dual_arm_on_turtlebot3_waffle_pi",
        "arms": arms,
        "base": {
            "model": "turtlebot3_waffle_pi",
            "ros_domain_id": 101,
            "state_topic": "/odom",
            "command_topic": "/cmd_vel",
            "state_vector": ["linear_x_mps", "angular_z_radps"],
            "stationary_tolerances": tolerances,
            "stationary_tolerance_derivation": {
                "formula": "ceil(max(2*abs_max,3*abs_p99),0.0001)",
                "baseline_summary": baseline_summary,
            },
        },
        "reference_facts": {
            "arm_motor_order": "JD-edu reference: IDs 1-5 are ordered arm joints; ID 6 is gripper",
            "arm_protocol": "Feetech STS READ protocol at 1000000 baud",
            "reference_code_copied": False,
        },
        "evidence": {
            "arm_snapshot_sha256": arm_snapshot_sha256,
            "base_baseline_report_sha256": base_baseline_sha256,
            "arm_motion_commands_sent": False,
            "arm_torque_commands_sent": False,
            "base_motion_commands_sent": False,
        },
        "pending_calibration": [
            "physical arm_a/arm_b side labels",
            "joint positive directions and mechanical zero",
            "safe per-joint motion limits",
            "current-to-joint-torque calibration",
            "link mass/inertia/friction identification",
            "TurtleBot wheel deadband, tracking, current, and safe operating maximum",
            "RGB-D camera driver, intrinsics, depth scale, and base extrinsics",
        ],
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build a DAPIER hardware profile from read-only evidence.")
    parser.add_argument("--arm-snapshot", type=Path, required=True)
    parser.add_argument("--base-baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        print(json.dumps({"error": f"output already exists: {args.output}"}, indent=2))
        return 2
    try:
        profile = build_hardware_profile(
            _load_object(args.arm_snapshot),
            _load_object(args.base_baseline),
            arm_snapshot_sha256=_sha256(args.arm_snapshot),
            base_baseline_sha256=_sha256(args.base_baseline),
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(profile, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    except (OSError, ValueError) as error:
        print(json.dumps({"error": str(error)}, indent=2))
        return 1
    print(json.dumps({"profile": str(args.output), "schema_version": PROFILE_SCHEMA_VERSION}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
