"""MuJoCo actuator-slider teleop for one or two JDcobot200 arms.

Simulation-only is the default. Hardware dispatch requires an explicit flag
and confirmation text. The bridge treats the hardware pose at startup as the
MuJoCo reference pose, so it does not invent unmeasured absolute offsets.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import importlib.util
import json
import math
from pathlib import Path
import time
from typing import Any, Mapping, Sequence

from shoe_sorting_data.arm_jog import (
    FULL_TORQUE_LIMIT_RAW,
    DEFAULT_MOTOR_IDS,
    EXPECTED_MODEL_NUMBER,
    JogPolicy,
    JogSafetyError,
    STS3215Bus,
    _validate_preflight_state,
    verify_controller_identity,
)


CONFIRM_TEXT = "AUTHORIZE_MUJOCO_JDCOBOT_TELEOP"
CONTROL_NAMES = (
    "base",
    "shoulder",
    "elbow",
    "wrist_pitch",
    "wrist_roll",
    "gripper_motor",
)
TICKS_PER_RADIAN = 4096.0 / (2.0 * math.pi)
TELEOP_POLICY = JogPolicy(
    max_abs_delta_ticks=None,
    position_margin_ticks=0,
    target_tolerance_ticks=5,
    max_hold_seconds=15.0,
)


@dataclass(frozen=True)
class ArmEndpoint:
    side: str
    port: str
    expected_serial: str
    control_offset: int


@dataclass
class ActiveArm:
    endpoint: ArmEndpoint
    bus: STS3215Bus
    start_ticks: dict[int, int]
    limits: dict[int, tuple[int, int]]
    last_targets: dict[int, int]
    calibration: dict[str, Any] | None = None
    saturation_events: int = 0


def find_project_root(start: Path | None = None) -> Path:
    """Find the source checkout containing the MuJoCo dual-arm model."""

    origin = (start or Path(__file__)).resolve()
    for candidate in (origin, *origin.parents):
        if (candidate / "sim/jdcobot200_dual/dual_model.py").is_file():
            return candidate
    raise FileNotFoundError("could not find sim/jdcobot200_dual/dual_model.py")


def relative_ctrl_to_ticks(
    current_ctrl: Sequence[float],
    reference_ctrl: Sequence[float],
    start_ticks: Mapping[int, int],
    limits: Mapping[int, tuple[int, int]],
) -> dict[int, int]:
    """Convert six relative MuJoCo radians, saturating at stored endpoints."""

    targets, _ = _relative_ctrl_mapping(
        current_ctrl,
        reference_ctrl,
        start_ticks,
        limits,
    )
    return targets


def _relative_ctrl_mapping(
    current_ctrl: Sequence[float],
    reference_ctrl: Sequence[float],
    start_ticks: Mapping[int, int],
    limits: Mapping[int, tuple[int, int]],
) -> tuple[dict[int, int], tuple[int, ...]]:
    """Return targets and motor IDs saturated to their stored endpoints."""

    if len(current_ctrl) != 6 or len(reference_ctrl) != 6:
        raise JogSafetyError("exactly six MuJoCo controls are required per arm")
    if set(start_ticks) != set(DEFAULT_MOTOR_IDS) or set(limits) != set(DEFAULT_MOTOR_IDS):
        raise JogSafetyError("motor IDs 1-6 must have start positions and stored limits")

    targets: dict[int, int] = {}
    saturated = []
    for index, motor_id in enumerate(DEFAULT_MOTOR_IDS):
        current = float(current_ctrl[index])
        reference = float(reference_ctrl[index])
        if not math.isfinite(current) or not math.isfinite(reference):
            raise JogSafetyError("MuJoCo control contains a non-finite value")
        requested = int(start_ticks[motor_id]) + round(
            (current - reference) * TICKS_PER_RADIAN
        )
        lower, upper = limits[motor_id]
        target = max(int(lower), min(int(upper), requested))
        if target != requested:
            saturated.append(motor_id)
        targets[motor_id] = target
    return targets, tuple(saturated)


def load_arm_calibration(
    project_root: Path,
    *,
    side: str,
    identity_sha256: str,
) -> dict[str, Any] | None:
    """Load a side-specific calibration only when controller identity matches."""

    path = project_root / f"config/jdcobot_calibration/{side}.json"
    if not path.is_file():
        return None
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("schema_version") != "dapier.jdcobot-calibration.v0.1":
        raise JogSafetyError(f"unsupported calibration schema: {path}")
    if value.get("arm_side") != side:
        raise JogSafetyError(f"calibration side mismatch: {path}")
    if value.get("controller_identity_sha256") != identity_sha256:
        raise JogSafetyError(f"calibration controller identity mismatch: {path}")
    zero = value.get("joint_zero_ticks")
    signs = value.get("joint_signs")
    gripper = value.get("gripper")
    if not isinstance(zero, dict) or set(zero) != {str(i) for i in range(1, 6)}:
        raise JogSafetyError(f"calibration must define joint zero ticks 1-5: {path}")
    if not isinstance(signs, dict) or set(signs) != {str(i) for i in range(1, 6)}:
        raise JogSafetyError(f"calibration must define joint signs 1-5: {path}")
    if any(signs[str(i)] not in (-1, 1) for i in range(1, 6)):
        raise JogSafetyError(f"calibration joint signs must be -1 or 1: {path}")
    if not isinstance(gripper, dict) or gripper.get("motor_id") != 6:
        raise JogSafetyError(f"calibration must define motor 6 gripper endpoints: {path}")
    if int(gripper["open_tick"]) == int(gripper["closed_tick"]):
        raise JogSafetyError(f"gripper open and closed ticks must differ: {path}")
    if float(gripper["model_open_radians"]) == float(gripper["model_closed_radians"]):
        raise JogSafetyError(f"gripper model endpoints must differ: {path}")
    return value


def calibrated_ctrl_to_ticks(
    controls: Sequence[float],
    calibration: Mapping[str, Any],
    limits: Mapping[int, tuple[int, int]],
) -> tuple[dict[int, int], tuple[int, ...]]:
    """Map MuJoCo absolute controls to measured servo zero/endpoints."""

    if len(controls) != 6:
        raise JogSafetyError("exactly six MuJoCo controls are required per arm")
    zero = calibration["joint_zero_ticks"]
    signs = calibration["joint_signs"]
    targets: dict[int, int] = {}
    saturated = []
    for index, motor_id in enumerate(range(1, 6)):
        control = float(controls[index])
        if not math.isfinite(control):
            raise JogSafetyError("MuJoCo control contains a non-finite value")
        requested = int(zero[str(motor_id)]) + round(
            control * int(signs[str(motor_id)]) * TICKS_PER_RADIAN
        )
        lower, upper = limits[motor_id]
        target = max(int(lower), min(int(upper), requested))
        if target != requested:
            saturated.append(motor_id)
        targets[motor_id] = target

    gripper = calibration["gripper"]
    control = float(controls[5])
    model_open = float(gripper["model_open_radians"])
    model_closed = float(gripper["model_closed_radians"])
    ratio = (control - model_open) / (model_closed - model_open)
    bounded_ratio = max(0.0, min(1.0, ratio))
    requested = round(
        int(gripper["open_tick"])
        + bounded_ratio * (int(gripper["closed_tick"]) - int(gripper["open_tick"]))
    )
    lower, upper = limits[6]
    targets[6] = max(int(lower), min(int(upper), requested))
    if bounded_ratio != ratio or targets[6] != requested:
        saturated.append(6)
    return targets, tuple(saturated)


def validate_runtime_state(motor_id: int, state: Mapping[str, object]) -> None:
    """Apply the same telemetry interlocks while torque may be enabled."""

    if state.get("status") != "ok":
        raise JogSafetyError(f"motor {motor_id} became unreadable")
    if state.get("model_number_raw") != EXPECTED_MODEL_NUMBER:
        raise JogSafetyError(f"motor {motor_id} model changed during teleop")
    if state.get("operating_mode_raw") != 0:
        raise JogSafetyError(f"motor {motor_id} left position mode")
    if state.get("hardware_error_status") != 0:
        raise JogSafetyError(f"motor {motor_id} reports a hardware error")
    voltage = float(state["voltage_volts"])
    if not TELEOP_POLICY.min_voltage_volts <= voltage <= TELEOP_POLICY.max_voltage_volts:
        raise JogSafetyError(
            f"motor {motor_id} voltage {voltage:.1f}V left the guarded range "
            f"{TELEOP_POLICY.min_voltage_volts:.1f}.."
            f"{TELEOP_POLICY.max_voltage_volts:.1f}V"
        )
    if int(state["temperature_celsius"]) > TELEOP_POLICY.max_temperature_celsius:
        raise JogSafetyError(f"motor {motor_id} temperature exceeded the guarded limit")
    if abs(float(state["current_milliamps"])) > TELEOP_POLICY.max_runtime_current_milliamps:
        raise JogSafetyError(f"motor {motor_id} current exceeded the guarded limit")
    if abs(float(state["load_fraction"])) > TELEOP_POLICY.max_runtime_load_fraction:
        raise JogSafetyError(f"motor {motor_id} load exceeded the guarded limit")
    position = int(state["position_tick"])
    lower = int(state["minimum_position_limit_tick"])
    upper = int(state["maximum_position_limit_tick"])
    if not lower <= position <= upper:
        raise JogSafetyError(f"motor {motor_id} left its stored position range")


def validate_hardware_authorization(args: argparse.Namespace) -> tuple[ArmEndpoint, ...]:
    """Resolve selected arms, rejecting partial or implicit hardware authority."""

    if not args.hardware_dispatch_authorized:
        return ()
    if args.confirm != CONFIRM_TEXT:
        raise JogSafetyError(f"--confirm must be exactly {CONFIRM_TEXT}")

    endpoints = []
    for side, offset in (("left", 0), ("right", 6)):
        port = getattr(args, f"{side}_port")
        serial = getattr(args, f"{side}_expected_serial")
        if bool(port) != bool(serial):
            raise JogSafetyError(f"{side} port and expected serial must be provided together")
        if port:
            endpoints.append(ArmEndpoint(side, port, serial, offset))
    if not endpoints:
        raise JogSafetyError("at least one arm endpoint is required for hardware teleop")
    if len({endpoint.port for endpoint in endpoints}) != len(endpoints):
        raise JogSafetyError("left and right arms must use different controller paths")
    return tuple(endpoints)


def _load_dual_model(project_root: Path):
    module_path = project_root / "sim/jdcobot200_dual/dual_model.py"
    spec = importlib.util.spec_from_file_location("dapier_jdcobot200_dual", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load MuJoCo model module: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _open_arm(endpoint: ArmEndpoint, project_root: Path) -> ActiveArm:
    identity = verify_controller_identity(endpoint.port, endpoint.expected_serial)
    bus = STS3215Bus(endpoint.port)
    try:
        states = {motor_id: bus.read_state(motor_id) for motor_id in DEFAULT_MOTOR_IDS}
        for motor_id, state in states.items():
            _validate_preflight_state(motor_id, state, policy=TELEOP_POLICY)
        start_ticks = {
            motor_id: int(state["position_tick"]) for motor_id, state in states.items()
        }
        limits = {
            motor_id: (
                int(state["minimum_position_limit_tick"]),
                int(state["maximum_position_limit_tick"]),
            )
            for motor_id, state in states.items()
        }
        # Force the first dispatch to send all six current-pose targets. Leaving
        # untouched joints torque-off lets gravity move the rest of the chain.
        pending_initial_hold = {motor_id: -1 for motor_id in DEFAULT_MOTOR_IDS}
        calibration = load_arm_calibration(
            project_root,
            side=endpoint.side,
            identity_sha256=identity["identity_sha256"],
        )
        return ActiveArm(
            endpoint,
            bus,
            start_ticks,
            limits,
            pending_initial_hold,
            calibration,
        )
    except Exception:
        bus.close()
        raise


def _poll_arm(arm: ActiveArm) -> None:
    for motor_id in DEFAULT_MOTOR_IDS:
        state = arm.bus.read_state(motor_id)
        if state.get("status") == "ok":
            voltage = float(state["voltage_volts"])
            if not TELEOP_POLICY.min_voltage_volts <= voltage <= TELEOP_POLICY.max_voltage_volts:
                state = arm.bus.read_state(motor_id)
        validate_runtime_state(motor_id, state)


def _dispatch_arm(arm: ActiveArm, controls: Sequence[float], reference: Sequence[float]) -> int:
    if arm.calibration is None:
        targets, saturated = _relative_ctrl_mapping(
            controls,
            reference,
            arm.start_ticks,
            arm.limits,
        )
    else:
        targets, saturated = calibrated_ctrl_to_ticks(
            controls,
            arm.calibration,
            arm.limits,
        )
    arm.saturation_events += len(saturated)
    writes = 0
    for motor_id, target in targets.items():
        if target == arm.last_targets[motor_id]:
            continue
        arm.bus.start_motion(
            motor_id,
            target_tick=target,
            speed_raw=120,
            acceleration_raw=40,
            torque_limit_raw=FULL_TORQUE_LIMIT_RAW,
        )
        arm.last_targets[motor_id] = target
        writes += 1
    return writes


def _shutdown_arm(arm: ActiveArm) -> bool:
    try:
        for _ in range(3):
            try:
                arm.bus.disable_all_torque(DEFAULT_MOTOR_IDS)
                states = {
                    motor_id: arm.bus.read_state(motor_id)
                    for motor_id in DEFAULT_MOTOR_IDS
                }
                if all(
                    state.get("status") == "ok" and state.get("torque_enabled") is False
                    for state in states.values()
                ):
                    return True
            except (JogSafetyError, OSError, ValueError, KeyError):
                pass
            time.sleep(0.05)
        return False
    finally:
        arm.bus.close()


def run_teleop(args: argparse.Namespace) -> dict[str, Any]:
    endpoints = validate_hardware_authorization(args)
    project_root = Path(args.project_root).resolve() if args.project_root else find_project_root()
    dual_model = _load_dual_model(project_root)

    import mujoco

    model = dual_model.build_model()
    dual_model.validate_model(model, smoke_steps=0)
    data = mujoco.MjData(model)
    data.ctrl[:] = dual_model.actuator_targets_from_qpos(model, data.qpos)
    reference_ctrl = tuple(float(value) for value in data.ctrl)

    if args.headless_smoke_steps:
        if endpoints:
            raise JogSafetyError("headless smoke mode never permits hardware dispatch")
        for _ in range(args.headless_smoke_steps):
            mujoco.mj_step(model, data)
        return {
            "status": "PASS",
            "mode": "headless-simulation",
            "published": False,
            "control_authorized": False,
            "hardware_dispatch_authorized": False,
            "executed_action": False,
            "hardware_execution": False,
            "steps": args.headless_smoke_steps,
        }

    import mujoco.viewer

    active_arms: list[ActiveArm] = []
    writes = 0
    torque_off_verified = True
    failure: str | None = None
    try:
        for endpoint in endpoints:
            active_arms.append(_open_arm(endpoint, project_root))
        dispatch_period = 1.0 / args.dispatch_hz
        next_dispatch = time.monotonic()
        next_telemetry = time.monotonic()
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                now = time.monotonic()
                if active_arms and now >= next_dispatch:
                    for arm in active_arms:
                        start = arm.endpoint.control_offset
                        stop = start + len(CONTROL_NAMES)
                        writes += _dispatch_arm(
                            arm,
                            data.ctrl[start:stop],
                            reference_ctrl[start:stop],
                        )
                    next_dispatch = now + dispatch_period
                if active_arms and now >= next_telemetry:
                    for arm in active_arms:
                        _poll_arm(arm)
                    next_telemetry = now + 0.5
                viewer.sync()
    except (JogSafetyError, OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        failure = "operator interrupt" if isinstance(error, KeyboardInterrupt) else str(error)
    finally:
        for arm in active_arms:
            try:
                torque_off_verified = _shutdown_arm(arm) and torque_off_verified
            except Exception:
                torque_off_verified = False

    authorized = bool(endpoints)
    executed = writes > 0
    if failure is None and torque_off_verified:
        status = "PASS"
    elif executed or not torque_off_verified:
        status = "FAIL"
    else:
        status = "BLOCKED"
    result = {
        "status": status,
        "mode": "hardware-teleop" if authorized else "simulation-only",
        "arms": [arm.endpoint.side for arm in active_arms],
        "calibrated_arms": [
            arm.endpoint.side for arm in active_arms if arm.calibration is not None
        ],
        "published": executed,
        "control_authorized": authorized,
        "hardware_dispatch_authorized": authorized,
        "executed_action": executed,
        "hardware_execution": executed,
        "hardware_writes": writes,
        "saturation_events": sum(arm.saturation_events for arm in active_arms),
        "torque_off_verified": torque_off_verified,
    }
    if failure is not None:
        result["failure"] = failure
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Drive JDcobot200 arms from MuJoCo actuator sliders; simulation-only by default."
    )
    parser.add_argument("--project-root")
    parser.add_argument("--hardware-dispatch-authorized", action="store_true")
    parser.add_argument("--confirm")
    parser.add_argument("--left-port")
    parser.add_argument("--left-expected-serial")
    parser.add_argument("--right-port")
    parser.add_argument("--right-expected-serial")
    parser.add_argument("--dispatch-hz", type=float, default=10.0)
    parser.add_argument("--headless-smoke-steps", type=int, default=0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if not 1.0 <= args.dispatch_hz <= 50.0:
            raise JogSafetyError("dispatch_hz must be from 1 to 50")
        if args.headless_smoke_steps < 0:
            raise JogSafetyError("headless_smoke_steps must be non-negative")
        result = run_teleop(args)
    except (JogSafetyError, OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        result = {
            "status": "BLOCKED",
            "error": "operator interrupt" if isinstance(error, KeyboardInterrupt) else str(error),
            "published": False,
            "control_authorized": False,
            "hardware_dispatch_authorized": False,
            "executed_action": False,
            "hardware_execution": False,
        }
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
