#!/usr/bin/env python3
"""Run the reviewed MuJoCo right-arm box pre-grasp as a bounded real motion."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import runpy
import signal
import time


ROOT = Path(__file__).resolve().parents[3]
SMOKE = runpy.run_path(str(ROOT / "2ARM_ROBOT/scripts/dual_so101_smoke"))
CONFIRMATION = "VISIBLE_RIGHT_SO101_MUJOCO_BOX_PREGRASP"
# Invalidated after the endpoint labelled right physically moved the left arm.
# Set only after a witnessed role-remapping check and a new confirmation token.
PHYSICAL_ROLE_MAPPING_VERIFIED = False
MOVING_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
# Relative joint motion from compact_mobile_dual_so101.py at target
# (0.285, -0.090, 0.200) m; IK residual 0.061 mm.
MUJOCO_DELTA_DEG = {
    "shoulder_pan": 3.010,
    "shoulder_lift": 86.614,
    "elbow_flex": -53.270,
    "wrist_flex": -6.282,
    "wrist_roll": -3.379,
}
DURATION_S = 8.0
HOLD_S = 2.0
PERIOD_S = 0.05
MAX_TRACKING_ERROR_DEG = 18.0
MAX_TEMPERATURE_C = 50
VOLTAGE_RAW_RANGE = (110, 130)
MAX_ABS_LOAD_RAW = 500
MAX_CURRENT_RAW = 300
MAX_TARGET_VELOCITY_DEG_S = 25.0
MAX_TARGET_ACCELERATION_DEG_S2 = 12.0
MAX_TARGET_JERK_DEG_S3 = 10.0


def _septic_weight(progress: float) -> float:
    u = min(1.0, max(0.0, progress))
    return 35 * u**4 - 84 * u**5 + 70 * u**6 - 20 * u**7


def build_goal(start: dict[str, float]) -> dict[str, float]:
    return {name: start[name] + MUJOCO_DELTA_DEG.get(name, 0.0) for name in start}


def validate_goal(goal: dict[str, float], calibration: dict[str, dict]) -> None:
    for name in MOVING_JOINTS:
        item = calibration[name]
        midpoint = (item["range_min"] + item["range_max"]) / 2.0
        minimum = (item["range_min"] - midpoint) * 360.0 / 4095.0
        maximum = (item["range_max"] - midpoint) * 360.0 / 4095.0
        if not minimum <= goal[name] <= maximum:
            raise RuntimeError(f"{name} target exceeds calibrated range")


def validate_trajectory_limits() -> None:
    distance = max(abs(value) for value in MUJOCO_DELTA_DEG.values())
    limits = (
        (2.1875 * distance / DURATION_S, MAX_TARGET_VELOCITY_DEG_S, "velocity"),
        (7.514 * distance / DURATION_S**2, MAX_TARGET_ACCELERATION_DEG_S2, "acceleration"),
        (52.5 * distance / DURATION_S**3, MAX_TARGET_JERK_DEG_S3, "jerk"),
    )
    for actual, maximum, label in limits:
        if actual > maximum:
            raise RuntimeError(f"target {label} limit exceeded")


def validate_health(snapshot: dict[str, dict[str, float]], *, stationary: bool) -> None:
    for register, expected in (("Status", 0), ("Operating_Mode", 0)):
        if any(value != expected for value in snapshot[register].values()):
            raise RuntimeError(f"unexpected {register}")
    if stationary and any(snapshot["Moving"].values()):
        raise RuntimeError("arm must be stationary before motion")
    if max(snapshot["Present_Temperature"].values()) > MAX_TEMPERATURE_C:
        raise RuntimeError("motor temperature limit exceeded")
    low, high = VOLTAGE_RAW_RANGE
    if any(not low <= value <= high for value in snapshot["Present_Voltage"].values()):
        raise RuntimeError("motor voltage is outside the commissioning range")
    if any(abs(value) > MAX_ABS_LOAD_RAW for value in snapshot["Present_Load"].values()):
        raise RuntimeError("motor load limit exceeded")
    if any(abs(value) > MAX_CURRENT_RAW for value in snapshot["Present_Current"].values()):
        raise RuntimeError("motor current limit exceeded")


def _runtime_health(bus) -> dict[str, dict[str, float]]:
    return {
        name: bus.sync_read(name, normalize=False, num_retry=2)
        for name in (
            "Operating_Mode",
            "Present_Load",
            "Present_Current",
            "Present_Velocity",
            "Present_Temperature",
            "Present_Voltage",
            "Torque_Enable",
            "Status",
            "Moving",
        )
    }


def _sample(bus, goal: dict[str, float]) -> dict[str, object]:
    observed = bus.sync_read("Present_Position", MOVING_JOINTS, num_retry=2)
    snapshot = _runtime_health(bus)
    validate_health(snapshot, stationary=False)
    if any(snapshot["Torque_Enable"][name] != 1 for name in MOVING_JOINTS):
        raise RuntimeError("moving-joint torque was lost")
    error = max(abs(observed[name] - goal[name]) for name in MOVING_JOINTS)
    if error > MAX_TRACKING_ERROR_DEG:
        raise RuntimeError("tracking error limit exceeded")
    return {
        "timestamp": datetime.now().astimezone().isoformat(),
        "goal_deg": goal,
        "observed_deg": observed,
        "max_tracking_error_deg": error,
        "load_raw": {name: snapshot["Present_Load"][name] for name in MOVING_JOINTS},
        "current_raw": {name: snapshot["Present_Current"][name] for name in MOVING_JOINTS},
        "velocity_raw": {name: snapshot["Present_Velocity"][name] for name in MOVING_JOINTS},
    }


def run_leg(
    bus,
    start: dict[str, float],
    goal: dict[str, float],
    trace: list,
    should_stop=lambda: False,
) -> None:
    steps = round(DURATION_S / PERIOD_S)
    started = time.monotonic()
    deadline = started + DURATION_S + 1.0
    for index in range(1, steps + 1):
        if should_stop():
            raise RuntimeError("operator stop requested")
        if time.monotonic() > deadline:
            raise RuntimeError("motion deadline exceeded")
        weight = _septic_weight(index / steps)
        command = {
            name: start[name] + weight * (goal[name] - start[name])
            for name in MOVING_JOINTS
        }
        bus.sync_write("Goal_Position", command, num_retry=2)
        time.sleep(max(0.0, started + index * PERIOD_S - time.monotonic()))
        if index % 5 == 0 or index == steps:
            trace.append(_sample(bus, command))


def _require_request(confirm: str, operator_present: bool) -> None:
    if confirm != CONFIRMATION:
        raise ValueError(f"motion requires --confirm {CONFIRMATION}")
    if not operator_present:
        raise ValueError("motion requires --operator-present")
    if not (os.isatty(0) and os.isatty(1)):
        raise ValueError("motion requires an interactive TTY")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--confirm", default="")
    parser.add_argument("--operator-present", action="store_true")
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    try:
        _require_request(args.confirm, args.operator_present)
        if not PHYSICAL_ROLE_MAPPING_VERIFIED:
            raise RuntimeError("physical left/right role mapping is unverified")
        profile = SMOKE["load_trusted_profile"](args.profile)
        validate_trajectory_limits()
        ports = {side: item["port"] for side, item in profile.items()}
        identities = {side: item["controller_serial"] for side, item in profile.items()}
        SMOKE["verify_arm_identities"](ports, identities)
        bus = SMOKE["load_bus"](profile["right"]["port"], profile["right"]["calibration"])
        log_stream = SMOKE["_open_new_log"](args.log)
    except (ValueError, RuntimeError) as error:
        raise SystemExit(str(error)) from error

    record: dict[str, object] = {
        "schema_version": "dapier.right-box-pregrasp.v1",
        "started_at": datetime.now().astimezone().isoformat(),
        "confirmation": CONFIRMATION,
        "mujoco_target_m": [0.285, -0.090, 0.200],
        "mujoco_delta_deg": MUJOCO_DELTA_DEG,
        "duration_s_each_way": DURATION_S,
        "target_limits": {
            "velocity_deg_s": MAX_TARGET_VELOCITY_DEG_S,
            "acceleration_deg_s2": MAX_TARGET_ACCELERATION_DEG_S2,
            "jerk_deg_s3": MAX_TARGET_JERK_DEG_S3,
        },
        "hardware_execution": False,
        "motion_completed": False,
        "returned_to_start": False,
        "trace": [],
    }
    failure: BaseException | None = None
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {
        sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        bus.connect()
        SMOKE["verify_arm_identities"](
            {"right": ports["right"]}, {"right": identities["right"]}
        )
        start = bus.sync_read("Present_Position", normalize=True, num_retry=2)
        before = SMOKE["health"](bus)
        validate_health(before, stationary=True)
        if any(before["Torque_Enable"].values()):
            raise RuntimeError("all right-arm motors must start torque-off")
        goal = build_goal(start)
        validate_goal(goal, profile["right"]["calibration"])
        record["start_deg"] = start
        record["goal_deg"] = goal

        bus.sync_write("Goal_Position", {name: start[name] for name in MOVING_JOINTS})
        bus.enable_torque(list(MOVING_JOINTS), num_retry=3)
        record["hardware_execution"] = True
        run_leg(bus, start, goal, record["trace"], lambda: stop_requested)
        time.sleep(HOLD_S)
        run_leg(bus, goal, start, record["trace"], lambda: stop_requested)
        record["returned_to_start"] = True
        record["motion_completed"] = True
    except BaseException as error:
        record["error"] = f"{type(error).__name__}: {error}"
        failure = error
    finally:
        try:
            if bus.is_connected:
                bus.disable_torque(list(MOVING_JOINTS), num_retry=5)
                torque = bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
                record["torque_off_verified"] = all(
                    torque[name] == 0 for name in MOVING_JOINTS
                )
                bus.disconnect(disable_torque=False)
        except BaseException as error:
            record["shutdown_error"] = type(error).__name__
            failure = failure or error
        finally:
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
            record["finished_at"] = datetime.now().astimezone().isoformat()
            with log_stream:
                json.dump(record, log_stream, ensure_ascii=False, indent=2)
                log_stream.write("\n")

    if failure is not None or not record.get("torque_off_verified"):
        raise RuntimeError("right-arm pre-grasp failed; inspect the private log") from None
    print("right_box_pregrasp=completed_and_returned")
    print("torque_off_verified=true")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
