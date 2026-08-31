#!/usr/bin/env python3
"""Keyboard teleoperation for the MuJoCo model only.

This module never imports serial, ROS, LeRobot hardware, or device-discovery
APIs. Key events update bounded simulator actuator targets after the existing
protected-clearance guard accepts the interpolated path.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import threading
import time
from typing import Sequence

import mujoco

from collision_guard import (
    DEFAULT_CLEARANCE_M,
    DEFAULT_MAX_JOINT_STEP_RAD,
    check_bimanual_path,
)
from mobile_dual_so101 import ACTION_NAMES


KEY_RIGHT_ARROW = 262
KEY_LEFT_ARROW = 263
KEY_LEFT_ARM = KEY_LEFT_ARROW
KEY_RIGHT_ARM = KEY_RIGHT_ARROW
KEY_DOWN_ARROW = 264
KEY_UP_ARROW = 265
KEY_INCREASE = KEY_UP_ARROW
KEY_DECREASE = KEY_DOWN_ARROW
KEY_OPEN_GRIPPER = ord("O")
KEY_CLOSE_GRIPPER = ord("C")
KEY_HOME = ord("H")
KEY_STOP = ord(" ")
KEYPAD_1 = 321
DEFAULT_STEP_RAD = math.radians(5.0)
DEFAULT_MIN_KEY_INTERVAL_S = 0.0
DEFAULT_MAX_SPEED_RAD_S = math.radians(45.0)
DEFAULT_ACCEL_RAD_S2 = math.radians(180.0)
DEFAULT_CONTROL_PERIOD_S = 0.02


@dataclass(frozen=True)
class TeleopUpdate:
    accepted: bool
    reason: str
    action_name: str
    targets: tuple[float, ...]
    selected_arm: str
    selected_joint: str
    stopped: bool
    monotonic_timestamp_ns: int
    minimum_clearance_m: float | None = None
    published: bool = False
    control_authorized: bool = False
    hardware_dispatch_authorized: bool = False
    executed_action: bool = False
    hardware_execution: bool = False

    def as_report(self) -> dict[str, object]:
        return asdict(self)


def _normalize_key(keycode: int) -> int:
    if ord("a") <= keycode <= ord("z"):
        return keycode - ord("a") + ord("A")
    return keycode


def _bounded_targets(
    model: mujoco.MjModel,
    targets: Sequence[float],
) -> tuple[float, ...]:
    if model.nu != len(ACTION_NAMES):
        raise RuntimeError(f"expected {len(ACTION_NAMES)} actuators, got {model.nu}")
    if len(targets) != len(ACTION_NAMES):
        raise ValueError(f"expected {len(ACTION_NAMES)} targets, got {len(targets)}")
    bounded = []
    for actuator_id, raw_value in enumerate(targets):
        value = float(raw_value)
        if not math.isfinite(value):
            raise ValueError("teleop targets must contain only finite numbers")
        lower, upper = model.actuator_ctrlrange[actuator_id]
        bounded.append(min(float(upper), max(float(lower), value)))
    return tuple(bounded)


def apply_teleop_targets(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    targets: Sequence[float],
) -> None:
    """Set actuator targets without writing qpos or contacting hardware."""

    requested = tuple(float(value) for value in targets)
    bounded = _bounded_targets(model, requested)
    if requested != bounded:
        raise ValueError("teleop targets must already be inside actuator bounds")
    data.ctrl[:] = bounded


def _actuator_positions(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> tuple[float, ...]:
    positions = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            raise RuntimeError(f"actuator {actuator_id} is not joint-backed")
        positions.append(float(data.qpos[int(model.jnt_qposadr[joint_id])]))
    return tuple(positions)


class SimTeleopController:
    """Thread-safe, fail-closed target state for keyboard teleoperation."""

    def __init__(
        self,
        model: mujoco.MjModel,
        *,
        initial_action: Sequence[float],
        step_rad: float = DEFAULT_STEP_RAD,
        min_key_interval_s: float = DEFAULT_MIN_KEY_INTERVAL_S,
        required_clearance_m: float = DEFAULT_CLEARANCE_M,
        max_speed_rad_s: float = DEFAULT_MAX_SPEED_RAD_S,
        accel_rad_s2: float = DEFAULT_ACCEL_RAD_S2,
    ) -> None:
        if not math.isfinite(step_rad) or step_rad <= 0:
            raise ValueError("step_rad must be finite and positive")
        if not math.isfinite(min_key_interval_s) or min_key_interval_s < 0:
            raise ValueError("min_key_interval_s must be finite and non-negative")
        if not math.isfinite(required_clearance_m) or required_clearance_m <= 0:
            raise ValueError("required_clearance_m must be finite and positive")
        if not math.isfinite(max_speed_rad_s) or max_speed_rad_s <= 0:
            raise ValueError("max_speed_rad_s must be finite and positive")
        if not math.isfinite(accel_rad_s2) or accel_rad_s2 <= 0:
            raise ValueError("accel_rad_s2 must be finite and positive")
        self._model = model
        self._home = _bounded_targets(model, initial_action)
        self._targets = self._home
        self._control_targets = self._home
        self._control_velocities = [0.0] * len(self._home)
        self._step_rad = float(step_rad)
        self._min_key_interval_ns = int(min_key_interval_s * 1_000_000_000)
        self._required_clearance_m = float(required_clearance_m)
        self._max_speed_rad_s = float(max_speed_rad_s)
        self._accel_rad_s2 = float(accel_rad_s2)
        self._selected_arm = 0
        self._selected_joint = 0
        self._stopped = False
        self._last_motion_ns: int | None = None
        self._lock = threading.RLock()

    @property
    def targets(self) -> tuple[float, ...]:
        with self._lock:
            return self._targets

    @property
    def stopped(self) -> bool:
        with self._lock:
            return self._stopped

    @property
    def control_targets(self) -> tuple[float, ...]:
        with self._lock:
            return self._control_targets

    def _action_index(self) -> int:
        return self._selected_arm * 6 + self._selected_joint

    def _selection(self) -> tuple[str, str, str]:
        action_name = ACTION_NAMES[self._action_index()]
        selected_arm, selected_joint = action_name.split("_", maxsplit=1)
        return action_name, selected_arm, selected_joint

    def _update(
        self,
        *,
        accepted: bool,
        reason: str,
        now_ns: int,
        minimum_clearance_m: float | None = None,
        action_name: str | None = None,
    ) -> TeleopUpdate:
        selected_action, selected_arm, selected_joint = self._selection()
        return TeleopUpdate(
            accepted=accepted,
            reason=reason,
            action_name=action_name or selected_action,
            targets=self._targets,
            selected_arm=selected_arm,
            selected_joint=selected_joint,
            stopped=self._stopped,
            monotonic_timestamp_ns=now_ns,
            minimum_clearance_m=minimum_clearance_m,
        )

    def status(self) -> dict[str, object]:
        with self._lock:
            action_name, selected_arm, selected_joint = self._selection()
            return {
                "schema_version": "dapier.mujoco-sim-teleop.v0.1",
                "action_order": ACTION_NAMES,
                "selected_arm": selected_arm,
                "selected_joint": selected_joint,
                "selected_action": action_name,
                "targets_rad": self._targets,
                "control_targets_rad": self._control_targets,
                "stopped": self._stopped,
                "published": False,
                "control_authorized": False,
                "hardware_dispatch_authorized": False,
                "executed_action": False,
                "hardware_execution": False,
            }

    def hold_current(self, current_action: Sequence[float]) -> None:
        """Adopt measured simulator qpos as the target while stopped."""

        with self._lock:
            self._targets = _bounded_targets(self._model, current_action)
            self._control_targets = self._targets
            self._control_velocities = [0.0] * len(self._targets)
    def handle_control_panel(
        self,
        panel_targets: Sequence[float],
        *,
        now_ns: int | None = None,
    ) -> TeleopUpdate:
        """Adopt a MuJoCo Control-panel target without writing simulator qpos."""

        resolved_now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        if resolved_now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        requested = tuple(float(value) for value in panel_targets)
        bounded = _bounded_targets(self._model, requested)
        with self._lock:
            if self._stopped:
                return self._update(
                    accepted=False,
                    reason="sim teleop is stopped",
                    now_ns=resolved_now_ns,
                    action_name="control_panel",
                )
            if requested != bounded:
                return self._update(
                    accepted=False,
                    reason="control panel target is outside actuator bounds",
                    now_ns=resolved_now_ns,
                    action_name="control_panel",
                )
            if bounded == self._control_targets and bounded == self._targets:
                return self._update(
                    accepted=False,
                    reason="control panel target is unchanged",
                    now_ns=resolved_now_ns,
                    action_name="control_panel",
                )
            assessment = check_bimanual_path(
                self._model,
                self._control_targets,
                bounded,
                required_clearance_m=self._required_clearance_m,
                max_joint_step_rad=DEFAULT_MAX_JOINT_STEP_RAD,
            )
            if not assessment.safe:
                return self._update(
                    accepted=False,
                    reason=(
                        f"{assessment.reason}: {assessment.first_body} vs "
                        f"{assessment.second_body}"
                    ),
                    now_ns=resolved_now_ns,
                    minimum_clearance_m=assessment.minimum_clearance_m,
                    action_name="control_panel",
                )
            self._targets = bounded
            self._control_targets = bounded
            self._control_velocities = [0.0] * len(bounded)
            return self._update(
                accepted=True,
                reason="control panel target accepted",
                now_ns=resolved_now_ns,
                minimum_clearance_m=assessment.minimum_clearance_m,
                action_name="control_panel",
            )

    def advance(self, dt_s: float) -> tuple[float, ...]:
        """Slew controls toward requested targets with acceleration limits."""

        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be finite and positive")
        with self._lock:
            if self._stopped:
                return self._control_targets
            candidate = list(self._control_targets)
            velocities = list(self._control_velocities)
            for index, (current, target) in enumerate(
                zip(self._control_targets, self._targets, strict=True)
            ):
                error = target - current
                if abs(error) <= 1e-12:
                    candidate[index] = target
                    velocities[index] = 0.0
                    continue
                direction = math.copysign(1.0, error)
                braking_speed = math.sqrt(2.0 * self._accel_rad_s2 * abs(error))
                desired_velocity = direction * min(
                    self._max_speed_rad_s, braking_speed
                )
                velocity_delta = self._accel_rad_s2 * dt_s
                velocity = velocities[index]
                velocity += min(
                    velocity_delta,
                    max(-velocity_delta, desired_velocity - velocity),
                )
                step = velocity * dt_s
                if direction * step >= abs(error):
                    candidate[index] = target
                    velocities[index] = 0.0
                else:
                    candidate[index] = current + step
                    velocities[index] = velocity

            raw_candidate = tuple(candidate)
            resolved_candidate = _bounded_targets(self._model, raw_candidate)
            for index, (raw_value, bounded_value) in enumerate(
                zip(raw_candidate, resolved_candidate, strict=True)
            ):
                if raw_value != bounded_value:
                    velocities[index] = 0.0
            if resolved_candidate == self._control_targets:
                return self._control_targets
            assessment = check_bimanual_path(
                self._model,
                self._control_targets,
                resolved_candidate,
                required_clearance_m=self._required_clearance_m,
                max_joint_step_rad=DEFAULT_MAX_JOINT_STEP_RAD,
            )
            if not assessment.safe:
                self._targets = self._control_targets
                self._control_velocities = [0.0] * len(self._targets)
                return self._control_targets
            self._control_targets = resolved_candidate
            self._control_velocities = velocities
            return self._control_targets

    def handle_key(
        self,
        keycode: int,
        *,
        now_ns: int | None = None,
    ) -> TeleopUpdate:
        resolved_now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        if resolved_now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        key = _normalize_key(int(keycode))
        with self._lock:
            if key in {KEY_LEFT_ARM, KEY_LEFT_ARROW}:
                self._selected_arm = 0
                return self._update(
                    accepted=True,
                    reason="left arm selected",
                    now_ns=resolved_now_ns,
                )
            if key in {KEY_RIGHT_ARM, KEY_RIGHT_ARROW}:
                self._selected_arm = 1
                return self._update(
                    accepted=True,
                    reason="right arm selected",
                    now_ns=resolved_now_ns,
                )
            if ord("1") <= key <= ord("6"):
                self._selected_joint = key - ord("1")
                return self._update(
                    accepted=True,
                    reason="joint selected",
                    now_ns=resolved_now_ns,
                )
            if KEYPAD_1 <= key <= KEYPAD_1 + 5:
                self._selected_joint = key - KEYPAD_1
                return self._update(
                    accepted=True,
                    reason="joint selected",
                    now_ns=resolved_now_ns,
                )
            if key == KEY_STOP:
                self._stopped = not self._stopped
                return self._update(
                    accepted=True,
                    reason="sim stop engaged" if self._stopped else "sim stop released",
                    now_ns=resolved_now_ns,
                )

            motion_key = key in {
                KEY_INCREASE,
                KEY_DECREASE,
                KEY_OPEN_GRIPPER,
                KEY_CLOSE_GRIPPER,
                KEY_HOME,
            }
            if not motion_key:
                return self._update(
                    accepted=False,
                    reason=f"unknown keycode {key}",
                    now_ns=resolved_now_ns,
                )
            if self._stopped:
                return self._update(
                    accepted=False,
                    reason="sim teleop is stopped",
                    now_ns=resolved_now_ns,
                )
            if (
                self._last_motion_ns is not None
                and resolved_now_ns - self._last_motion_ns < self._min_key_interval_ns
            ):
                return self._update(
                    accepted=False,
                    reason="motion key rate limit",
                    now_ns=resolved_now_ns,
                )

            candidate = list(self._targets)
            action_name: str | None = None
            if key == KEY_HOME:
                candidate = list(self._home)
                action_name = "all_home"
            else:
                if key in {KEY_OPEN_GRIPPER, KEY_CLOSE_GRIPPER}:
                    self._selected_joint = 5
                action_index = self._action_index()
                lower, upper = self._model.actuator_ctrlrange[action_index]
                if key == KEY_OPEN_GRIPPER:
                    candidate[action_index] = float(upper)
                elif key == KEY_CLOSE_GRIPPER:
                    candidate[action_index] = float(lower)
                else:
                    direction = 1.0 if key == KEY_INCREASE else -1.0
                    candidate[action_index] += direction * self._step_rad
                candidate = list(_bounded_targets(self._model, candidate))
                action_name = ACTION_NAMES[action_index]

            resolved_candidate = tuple(candidate)
            if resolved_candidate == self._targets:
                return self._update(
                    accepted=False,
                    reason="target is already at the requested limit or home",
                    now_ns=resolved_now_ns,
                    action_name=action_name,
                )
            assessment = check_bimanual_path(
                self._model,
                self._control_targets,
                resolved_candidate,
                required_clearance_m=self._required_clearance_m,
                max_joint_step_rad=DEFAULT_MAX_JOINT_STEP_RAD,
            )
            if not assessment.safe:
                return self._update(
                    accepted=False,
                    reason=(
                        f"{assessment.reason}: {assessment.first_body} vs "
                        f"{assessment.second_body}"
                    ),
                    now_ns=resolved_now_ns,
                    minimum_clearance_m=assessment.minimum_clearance_m,
                    action_name=action_name,
                )
            self._targets = resolved_candidate
            self._last_motion_ns = resolved_now_ns
            return self._update(
                accepted=True,
                reason="simulator target accepted",
                now_ns=resolved_now_ns,
                minimum_clearance_m=assessment.minimum_clearance_m,
                action_name=action_name,
            )


def synchronize_stop_hold(
    controller: SimTeleopController,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    was_stopped: bool,
) -> bool:
    """Latch simulator qpos once when stop changes from released to engaged."""

    stopped = controller.stopped
    if stopped and not was_stopped:
        controller.hold_current(_actuator_positions(model, data))
    return stopped

def synchronize_control_panel(
    controller: SimTeleopController,
    data: mujoco.MjData,
    last_applied_targets: Sequence[float],
    *,
    now_ns: int | None = None,
) -> TeleopUpdate | None:
    """Detect a viewer Control-panel edit relative to the last applied target."""

    observed = tuple(float(value) for value in data.ctrl)
    previous = tuple(float(value) for value in last_applied_targets)
    if len(observed) != len(previous):
        raise ValueError("control panel and applied targets must have equal length")
    if all(
        abs(observed_value - previous_value) <= 1e-12
        for observed_value, previous_value in zip(observed, previous, strict=True)
    ):
        return None
    return controller.handle_control_panel(observed, now_ns=now_ns)


def teleop_help() -> str:
    return (
        "MuJoCo SIM teleop: Control sliders or keyboard | "
        "Left/Right arm | 1-6 joint | hold Up/Down to move smoothly | "
        "O/C gripper | H home | Space stop/resume | Esc close"
    )


def run_sim_teleop(
    model: mujoco.MjModel,
    *,
    initial_action: Sequence[float],
    step_rad: float = DEFAULT_STEP_RAD,
    min_key_interval_s: float = DEFAULT_MIN_KEY_INTERVAL_S,
    required_clearance_m: float = DEFAULT_CLEARANCE_M,
    max_speed_rad_s: float = DEFAULT_MAX_SPEED_RAD_S,
    accel_rad_s2: float = DEFAULT_ACCEL_RAD_S2,
) -> dict[str, object]:
    """Run the passive viewer with Control-panel and keyboard targets."""

    import mujoco.viewer

    controller = SimTeleopController(
        model,
        initial_action=initial_action,
        step_rad=step_rad,
        min_key_interval_s=min_key_interval_s,
        required_clearance_m=required_clearance_m,
        max_speed_rad_s=max_speed_rad_s,
        accel_rad_s2=accel_rad_s2,
    )
    data = mujoco.MjData(model)
    initial = controller.targets
    apply_teleop_targets(model, data, initial)
    for actuator_id, target in enumerate(initial):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        data.qpos[int(model.jnt_qposadr[joint_id])] = target
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    print(teleop_help(), flush=True)
    print(json.dumps(controller.status(), ensure_ascii=False), flush=True)

    def key_callback(keycode: int) -> None:
        update = controller.handle_key(keycode)
        print(json.dumps(update.as_report(), ensure_ascii=False), flush=True)

    was_stopped = False
    control_elapsed_s = DEFAULT_CONTROL_PERIOD_S
    last_applied_controls = initial
    with mujoco.viewer.launch_passive(
        model,
        data,
        key_callback=key_callback,
    ) as viewer:
        while viewer.is_running():
            started = time.monotonic()
            panel_update = None
            with viewer.lock():
                panel_update = synchronize_control_panel(
                    controller,
                    data,
                    last_applied_controls,
                )
                was_stopped = synchronize_stop_hold(
                    controller, model, data, was_stopped=was_stopped
                )
                control_elapsed_s += float(model.opt.timestep)
                if control_elapsed_s >= DEFAULT_CONTROL_PERIOD_S:
                    control_targets = controller.advance(control_elapsed_s)
                    control_elapsed_s = 0.0
                else:
                    control_targets = controller.control_targets
                apply_teleop_targets(model, data, control_targets)
                mujoco.mj_step(model, data)
                last_applied_controls = control_targets
            viewer.sync()
            if panel_update is not None:
                print(json.dumps(panel_update.as_report(), ensure_ascii=False), flush=True)
            remaining = float(model.opt.timestep) - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)

    report = controller.status()
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return report


__all__ = [
    "KEY_CLOSE_GRIPPER",
    "KEY_DECREASE",
    "KEY_DOWN_ARROW",
    "KEY_HOME",
    "KEY_INCREASE",
    "KEY_LEFT_ARM",
    "KEY_LEFT_ARROW",
    "KEY_OPEN_GRIPPER",
    "KEY_RIGHT_ARM",
    "KEY_RIGHT_ARROW",
    "KEY_STOP",
    "KEY_UP_ARROW",
    "KEYPAD_1",
    "SimTeleopController",
    "TeleopUpdate",
    "apply_teleop_targets",
    "run_sim_teleop",
    "synchronize_control_panel",
    "synchronize_stop_hold",
    "teleop_help",
]
