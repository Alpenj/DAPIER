#!/usr/bin/env python3
"""Interactive MuJoCo controller for the DAPIER ros_dd four-wheel car."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import glfw
import mujoco
import numpy as np


WHEEL_RADIUS = 0.075
TRACK_WIDTH = 0.35
DEFAULT_SPEED = 0.8
MIN_SPEED = 0.2
MAX_SPEED = 3.0
SPEED_STEP = 0.2
TURN_RATE = 1.8


def wheel_targets(linear: float, angular: float) -> np.ndarray:
    """Convert body linear/angular targets to four wheel angular velocities."""
    left = linear - angular * TRACK_WIDTH / 2.0
    right = linear + angular * TRACK_WIDTH / 2.0
    # Hinge axes point +Y. Positive angular velocity rolls the car toward +X.
    return np.array([left, left, right, right], dtype=np.float64) / WHEEL_RADIUS


def command_values(command: str, speed: float) -> tuple[float, float]:
    commands = {
        "forward": (speed, 0.0),
        "reverse": (-speed, 0.0),
        "left": (0.0, TURN_RATE),
        "right": (0.0, -TURN_RATE),
        "stop": (0.0, 0.0),
    }
    return commands[command]


def load_model(path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    if model.nu != 4:
        raise RuntimeError(f"Expected four wheel actuators, found {model.nu}")
    return model, data


def run_headless(model_path: Path, command: str, seconds: float) -> int:
    model, data = load_model(model_path)
    linear, angular = command_values(command, DEFAULT_SPEED)
    target = wheel_targets(linear, angular)
    steps = max(1, math.ceil(seconds / model.opt.timestep))
    for _ in range(steps):
        data.ctrl[:] = target
        mujoco.mj_step(model, data)

    position = data.qpos[:3]
    quaternion = data.qpos[3:7]
    yaw = math.atan2(
        2.0 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
        1.0 - 2.0 * (quaternion[2] ** 2 + quaternion[3] ** 2),
    )
    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    print(
        f"model={model_path.name} command={command} seconds={seconds:.2f} "
        f"position=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) "
        f"yaw={yaw:.3f} finite={finite}"
    )
    return 0 if finite else 1


class InteractiveCar:
    def __init__(self, model_path: Path) -> None:
        self.model, self.data = load_model(model_path)
        self.speed = DEFAULT_SPEED
        self.pressed: set[int] = set()
        self.last_cursor = (0.0, 0.0)
        self.last_title_update = 0.0

        if not glfw.init():
            raise RuntimeError("GLFW initialization failed. Run from a graphical desktop.")
        self.window = glfw.create_window(1280, 800, "ros_dd MuJoCo 4-wheel", None, None)
        if self.window is None:
            glfw.terminate()
            raise RuntimeError("Could not create the MuJoCo window.")
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        self.camera = mujoco.MjvCamera()
        self.option = mujoco.MjvOption()
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.context = mujoco.MjrContext(
            self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value
        )
        mujoco.mjv_defaultCamera(self.camera)
        mujoco.mjv_defaultOption(self.option)
        self.camera.azimuth = 135.0
        self.camera.elevation = -22.0
        self.camera.distance = 2.2
        self.camera.lookat[:] = [0.0, 0.0, 0.15]

        glfw.set_key_callback(self.window, self.on_key)
        glfw.set_scroll_callback(self.window, self.on_scroll)
        glfw.set_cursor_pos_callback(self.window, self.on_cursor)
        glfw.set_mouse_button_callback(self.window, self.on_mouse_button)

    def on_key(self, _window, key: int, _scancode: int, action: int, _mods: int) -> None:
        if action == glfw.PRESS:
            if key == glfw.KEY_ESCAPE:
                glfw.set_window_should_close(self.window, True)
            elif key == glfw.KEY_SPACE:
                self.pressed.clear()
                self.data.ctrl[:] = 0.0
            elif key == glfw.KEY_R:
                mujoco.mj_resetData(self.model, self.data)
                mujoco.mj_forward(self.model, self.data)
            else:
                self.pressed.add(key)
        elif action == glfw.RELEASE:
            self.pressed.discard(key)

    def on_scroll(self, _window, _xoffset: float, yoffset: float) -> None:
        if glfw.get_key(self.window, glfw.KEY_LEFT_CONTROL) == glfw.PRESS:
            mujoco.mjv_moveCamera(
                self.model,
                mujoco.mjtMouse.mjMOUSE_ZOOM,
                0.0,
                -0.05 * yoffset,
                self.scene,
                self.camera,
            )
            return
        self.speed = float(
            np.clip(self.speed + SPEED_STEP * yoffset, MIN_SPEED, MAX_SPEED)
        )

    def on_mouse_button(self, window, _button: int, _action: int, _mods: int) -> None:
        self.last_cursor = glfw.get_cursor_pos(window)

    def on_cursor(self, window, xpos: float, ypos: float) -> None:
        dx = xpos - self.last_cursor[0]
        dy = ypos - self.last_cursor[1]
        self.last_cursor = (xpos, ypos)
        left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        middle = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
        if not (left or right or middle):
            return
        width, height = glfw.get_window_size(window)
        shift = (
            glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
            or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        )
        if right:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_H if shift else mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif left:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if shift else mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM
        mujoco.mjv_moveCamera(
            self.model, action, dx / max(height, 1), dy / max(height, 1), self.scene, self.camera
        )

    def drive_targets(self) -> np.ndarray:
        linear = self.speed * (
            int(glfw.KEY_W in self.pressed) - int(glfw.KEY_S in self.pressed)
        )
        angular = TURN_RATE * (
            int(glfw.KEY_A in self.pressed) - int(glfw.KEY_D in self.pressed)
        )
        return wheel_targets(linear, angular)

    def render(self) -> None:
        width, height = glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, width, height)
        self.camera.lookat[0] += 0.05 * (self.data.qpos[0] - self.camera.lookat[0])
        self.camera.lookat[1] += 0.05 * (self.data.qpos[1] - self.camera.lookat[1])
        mujoco.mjv_updateScene(
            self.model,
            self.data,
            self.option,
            None,
            self.camera,
            mujoco.mjtCatBit.mjCAT_ALL.value,
            self.scene,
        )
        mujoco.mjr_render(viewport, self.scene, self.context)
        left = "W/S  forward/reverse\nA/D  turn\nSpace stop\nR reset\nEsc quit"
        right = (
            f"speed {self.speed:.1f} m/s\n"
            "wheel: speed +/-\nCtrl+wheel: camera zoom\n"
            "drag: rotate/pan"
        )
        mujoco.mjr_overlay(
            mujoco.mjtFontScale.mjFONTSCALE_150,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            viewport,
            left,
            right,
            self.context,
        )

    def run(self) -> None:
        last_wall = time.perf_counter()
        try:
            while not glfw.window_should_close(self.window):
                frame_start = time.perf_counter()
                elapsed = min(frame_start - last_wall, 0.05)
                last_wall = frame_start
                end_time = self.data.time + elapsed
                target = self.drive_targets()
                while self.data.time < end_time:
                    self.data.ctrl[:] = target
                    mujoco.mj_step(self.model, self.data)
                self.render()
                glfw.swap_buffers(self.window)
                glfw.poll_events()
                if frame_start - self.last_title_update > 0.25:
                    glfw.set_window_title(
                        self.window,
                        f"ros_dd MuJoCo 4-wheel | speed {self.speed:.1f} m/s",
                    )
                    self.last_title_update = frame_start
        finally:
            glfw.destroy_window(self.window)
            glfw.terminate()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(__file__).with_name("ros_dd_4wheel.xml"),
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--command",
        choices=("forward", "reverse", "left", "right", "stop"),
        default="forward",
    )
    parser.add_argument("--seconds", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.headless:
        return run_headless(args.model, args.command, args.seconds)
    InteractiveCar(args.model).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
