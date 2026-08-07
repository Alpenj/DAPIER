#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Teleoperate SO-101 in MuJoCo with a keyboard or an optional physical leader."""

from __future__ import annotations

import argparse
import contextlib
import ctypes
import ctypes.util
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.envs.so101_mujoco import (
    CAMERA_NAMES,
    CAMERA_OBSERVATION_KEYS,
    JOINT_NAMES,
    SO101MujocoEnv,
)
from lerobot.envs.so101_mujoco.teleop import (
    PICK_APPROACH_ACTION,
    PICK_CLEAR_ACTION,
    PICK_LIFT_FRAMES,
    CartesianJogController,
    JointJogController,
    SO101LeaderActionSource,
    scripted_pick_lift_action,
    should_save_episode,
)
from lerobot.utils.constants import ACTION, OBS_IMAGES, OBS_STATE


class X11HeldKeyPoller:
    """Read held-key state on X11 without taking ownership of MuJoCo's GLFW window."""

    def __init__(self, key_names: dict[int, str]) -> None:
        library_path = ctypes.util.find_library("X11")
        if library_path is None:
            raise RuntimeError("Continuous keyboard input requires libX11 on this Linux viewer")
        self._x11 = ctypes.CDLL(library_path)
        self._x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self._x11.XOpenDisplay.restype = ctypes.c_void_p
        self._x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self._x11.XStringToKeysym.restype = ctypes.c_ulong
        self._x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self._x11.XKeysymToKeycode.restype = ctypes.c_ubyte
        self._x11.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_char)]
        self._x11.XQueryKeymap.restype = ctypes.c_int
        self._x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self._x11.XCloseDisplay.restype = ctypes.c_int

        self._display = self._x11.XOpenDisplay(None)
        if not self._display:
            raise RuntimeError("Could not open DISPLAY for continuous keyboard input")
        self._keycodes: dict[int, int] = {}
        for glfw_key, name in key_names.items():
            keysym = self._x11.XStringToKeysym(name.encode("ascii"))
            keycode = int(self._x11.XKeysymToKeycode(self._display, keysym))
            if keysym == 0 or keycode == 0:
                self.close()
                raise RuntimeError(f"Could not map X11 key {name!r}")
            self._keycodes[glfw_key] = keycode

    def pressed(self) -> set[int]:
        if not self._display:
            return set()
        key_vector = ctypes.create_string_buffer(32)
        self._x11.XQueryKeymap(self._display, key_vector)
        bits = bytes(key_vector)
        return {
            glfw_key
            for glfw_key, keycode in self._keycodes.items()
            if bits[keycode // 8] & (1 << (keycode % 8))
        }

    def close(self) -> None:
        if getattr(self, "_display", None):
            self._x11.XCloseDisplay(self._display)
            self._display = None


class ViewerKeyboard:
    """Thread-safe edge and held-key controls for MuJoCo's passive viewer."""

    def __init__(
        self,
        controller: CartesianJogController,
        *,
        joint_speed_degrees: float,
        gripper_speed_percent: float,
        cartesian_speed_m: float,
    ) -> None:
        import glfw

        self.controller = controller
        self.joint_speed_degrees = float(joint_speed_degrees)
        self.gripper_speed_percent = float(gripper_speed_percent)
        self.cartesian_speed_m = float(cartesian_speed_m)
        self._lock = threading.Lock()
        self._new_episode_requested = False
        self._stop_requested = False
        self._armed_keys: set[int] = set()
        self._pending_edge_keys: set[int] = set()
        self._handled_edge_keys: set[int] = set()
        self._scripted_frame: int | None = None
        self._freeze_after_current_step = False
        self._physics_paused = False
        self._mode = "manual"
        self._continuous_keys = {
            glfw.KEY_UP,
            glfw.KEY_DOWN,
            glfw.KEY_W,
            glfw.KEY_S,
            glfw.KEY_A,
            glfw.KEY_D,
            glfw.KEY_R,
            glfw.KEY_F,
            glfw.KEY_O,
            glfw.KEY_L,
        }
        self._edge_keys = {
            glfw.KEY_J,
            glfw.KEY_K,
            glfw.KEY_H,
            glfw.KEY_G,
            glfw.KEY_P,
            glfw.KEY_N,
            glfw.KEY_Q,
        }
        self._command_keys = self._continuous_keys | self._edge_keys
        self._shift_keys = {glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT}
        self.key_names = {
            glfw.KEY_UP: "Up",
            glfw.KEY_DOWN: "Down",
            glfw.KEY_W: "w",
            glfw.KEY_S: "s",
            glfw.KEY_A: "a",
            glfw.KEY_D: "d",
            glfw.KEY_R: "r",
            glfw.KEY_F: "f",
            glfw.KEY_O: "o",
            glfw.KEY_L: "l",
            glfw.KEY_J: "j",
            glfw.KEY_K: "k",
            glfw.KEY_H: "h",
            glfw.KEY_G: "g",
            glfw.KEY_P: "p",
            glfw.KEY_N: "n",
            glfw.KEY_Q: "q",
            glfw.KEY_LEFT_SHIFT: "Shift_L",
            glfw.KEY_RIGHT_SHIFT: "Shift_R",
        }

    def on_key(self, keycode: int) -> None:
        with self._lock:
            if keycode in self._command_keys:
                # MuJoCo exposes press events but not releases to this callback.
                # X11HeldKeyPoller supplies the release/held state on the main loop.
                self._armed_keys.add(keycode)
                if keycode in self._edge_keys:
                    self._pending_edge_keys.add(keycode)

    def poll_controls(self, poller: X11HeldKeyPoller, elapsed_seconds: float) -> None:
        import glfw

        pressed = poller.pressed()
        shift_pressed = bool(self._shift_keys & pressed)
        with self._lock:
            released = self._armed_keys - pressed
            self._armed_keys.intersection_update(pressed)
            self._handled_edge_keys.difference_update(released)
            active = self._armed_keys & pressed if shift_pressed else set()
            edge_candidates = active & self._edge_keys
            if shift_pressed:
                edge_candidates |= self._pending_edge_keys
            self._pending_edge_keys.clear()
            new_edges = edge_candidates - self._handled_edge_keys
            self._handled_edge_keys.update(new_edges)

            if glfw.KEY_J in new_edges:
                self.controller.select_previous_joint()
                print(f"selected_joint={self.controller.selected_joint_name}")
            if glfw.KEY_K in new_edges:
                self.controller.select_next_joint()
                print(f"selected_joint={self.controller.selected_joint_name}")
            if glfw.KEY_H in new_edges:
                self.controller.reset()
                self._scripted_frame = None
                self._physics_paused = False
                self._mode = "home"
                print("target=home")
            if glfw.KEY_G in new_edges:
                self.controller.set_action(PICK_APPROACH_ACTION)
                self._scripted_frame = None
                self._physics_paused = False
                self._mode = "cube approach"
                print("target=cube_approach")
            if glfw.KEY_P in new_edges:
                self._scripted_frame = 0
                self._freeze_after_current_step = False
                self._physics_paused = False
                self._mode = "scripted pick 0/300"
                print("scripted_pick=started (300 frames)")
            if glfw.KEY_N in new_edges:
                self._scripted_frame = None
                self._physics_paused = False
                self._mode = "new episode requested"
                self._new_episode_requested = True
            if glfw.KEY_Q in new_edges:
                self._stop_requested = True

            active_continuous = active & self._continuous_keys
            if not active_continuous:
                return
            self._scripted_frame = None
            self._physics_paused = False
            self._mode = "manual (Shift chord held)"

            selected_direction = int(glfw.KEY_UP in active_continuous) - int(
                glfw.KEY_DOWN in active_continuous
            )
            if selected_direction:
                speed = (
                    self.gripper_speed_percent
                    if self.controller.selected_joint == 5
                    else self.joint_speed_degrees
                )
                self.controller.jog_selected_joint(selected_direction * speed * elapsed_seconds)

            gripper_direction = int(glfw.KEY_O in active_continuous) - int(glfw.KEY_L in active_continuous)
            if gripper_direction:
                self.controller.adjust_gripper(
                    gripper_direction * self.gripper_speed_percent * elapsed_seconds
                )

            delta = (
                self.cartesian_speed_m
                * elapsed_seconds
                * np.array(
                    [
                        int(glfw.KEY_W in active_continuous) - int(glfw.KEY_S in active_continuous),
                        int(glfw.KEY_A in active_continuous) - int(glfw.KEY_D in active_continuous),
                        int(glfw.KEY_R in active_continuous) - int(glfw.KEY_F in active_continuous),
                    ],
                    dtype=np.float64,
                )
            )
            if np.any(delta):
                self.controller.move(delta)

    def get_action(self) -> np.ndarray:
        with self._lock:
            if self._scripted_frame is not None:
                frame = self._scripted_frame
                self.controller.set_action(scripted_pick_lift_action(frame))
                self._scripted_frame += 1
                if self._scripted_frame >= PICK_LIFT_FRAMES:
                    self._scripted_frame = None
                    self._freeze_after_current_step = True
                    self._mode = "scripted lift complete (pausing physics)"
                else:
                    self._mode = f"scripted pick {self._scripted_frame}/300"
            return self.controller.get_action()

    def after_step(self) -> None:
        with self._lock:
            if self._freeze_after_current_step:
                self._freeze_after_current_step = False
                self._physics_paused = True
                self._mode = "verified frame 299 (physics paused)"

    def resume_physics(self) -> None:
        with self._lock:
            self._freeze_after_current_step = False
            self._physics_paused = False

    @property
    def physics_paused(self) -> bool:
        with self._lock:
            return self._physics_paused

    def consume_new_episode_request(self) -> bool:
        with self._lock:
            requested = self._new_episode_requested
            self._new_episode_requested = False
            return requested

    @property
    def stop_requested(self) -> bool:
        with self._lock:
            return self._stop_requested

    @property
    def status_text(self) -> str:
        with self._lock:
            action = self.controller.get_action()
            return (
                f"mode: {self._mode}\n"
                f"selected: {self.controller.selected_joint_name}\n"
                f"target: {np.array2string(action, precision=1, suppress_small=True)}"
            )


def create_dataset(
    root: Path,
    repo_id: str,
    fps: int,
    height: int,
    width: int,
    camera_names: tuple[str, ...],
):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features: dict[str, dict[str, Any]] = {
        OBS_STATE: {"dtype": "float32", "shape": (6,), "names": list(JOINT_NAMES)},
        ACTION: {"dtype": "float32", "shape": (6,), "names": list(JOINT_NAMES)},
        "next.reward": {"dtype": "float32", "shape": (1,), "names": None},
        "next.success": {"dtype": "bool", "shape": (1,), "names": None},
        "next.done": {"dtype": "bool", "shape": (1,), "names": None},
    }
    for camera_name in camera_names:
        features[f"{OBS_IMAGES}.{camera_name}"] = {
            "dtype": "image",
            "shape": (height, width, 3),
            "names": ["height", "width", "channel"],
        }
    return LeRobotDataset.create(
        repo_id=repo_id,
        root=root,
        fps=fps,
        robot_type="so101_mujoco",
        features=features,
        use_videos=False,
        image_writer_threads=max(2, len(camera_names)),
    )


def finish_episode(dataset, *, success: bool, save_mode: str) -> bool:
    """Save a completed episode or discard its buffered frames."""
    if dataset is None or not dataset.has_pending_frames():
        return False
    should_save = should_save_episode(success=success, save_mode=save_mode)
    if should_save:
        dataset.save_episode(parallel_encoding=False)
    else:
        dataset.clear_episode_buffer(delete_images=True)
    return should_save


def add_dataset_frame(dataset, observation, action, reward, info, done, camera_names) -> None:
    frame = {
        OBS_STATE: observation["agent_pos"],
        ACTION: action,
        "next.reward": np.array([reward], dtype=np.float32),
        "next.success": np.array([info["is_success"]], dtype=bool),
        "next.done": np.array([done], dtype=bool),
        "task": "Pick up the blue cube and place it in the green tray.",
    }
    for camera_name in camera_names:
        frame[f"{OBS_IMAGES}.{camera_name}"] = observation[CAMERA_OBSERVATION_KEYS[camera_name]]
    dataset.add_frame(frame)


def wait_for_viewer_shutdown(viewer_handle, timeout_seconds: float = 2.0) -> None:
    """Let MuJoCo's render thread destroy GLFW before Python process teardown."""
    deadline = time.monotonic() + timeout_seconds
    while viewer_handle._sim() is not None and time.monotonic() < deadline:
        time.sleep(0.01)


def run(args: argparse.Namespace) -> None:
    if args.input == "keyboard" and args.no_viewer:
        raise ValueError("Keyboard input requires the MuJoCo viewer; remove --no-viewer")
    if args.input == "leader" and not args.leader_port:
        raise ValueError("--leader-port is required when --input=leader")

    camera_names = ("front",) if args.front_only else CAMERA_NAMES
    recording = args.record_root is not None
    env = SO101MujocoEnv(
        obs_type="pixels_agent_pos" if recording else "state",
        camera_names=camera_names,
        observation_height=args.height,
        observation_width=args.width,
        fps=args.fps,
        max_episode_steps=args.steps,
        terminate_on_success=True,
        cube_xy_randomization=args.cube_randomization,
        home_action=PICK_CLEAR_ACTION,
    )
    dataset = (
        create_dataset(args.record_root, args.repo_id, args.fps, args.height, args.width, camera_names)
        if recording
        else None
    )
    observation, info = env.reset(seed=args.seed)
    assert env.model is not None and env.data is not None
    joint_controller = JointJogController(
        home_action=PICK_CLEAR_ACTION,
        joint_step_degrees=args.joint_step_degrees,
        gripper_step_percent=args.gripper_step_percent,
    )
    controller = CartesianJogController(env.model, joint_controller=joint_controller)
    keyboard = ViewerKeyboard(
        controller,
        joint_speed_degrees=args.joint_speed_degrees,
        gripper_speed_percent=args.gripper_speed_percent,
        cartesian_speed_m=args.cartesian_speed_m,
    )
    action_source = (
        SO101LeaderActionSource(port=args.leader_port, leader_id=args.leader_id)
        if args.input == "leader"
        else keyboard
    )

    attempts = 0
    successful_episodes = 0
    saved_episodes = 0
    print(
        "keys: hold Shift+W/S X, Shift+A/D Y, Shift+R/F Z | Shift+O/L gripper | "
        "Shift+J/K joint select | Shift+Up/Down joint | Shift+G approach | "
        "Shift+P scripted pick/lift | Shift+H home | Shift+N new | Shift+Q quit"
    )

    try:
        with contextlib.ExitStack() as stack:
            if args.input == "leader":
                action_source = stack.enter_context(action_source)

            viewer_handle = None
            key_poller = None
            if not args.no_viewer:
                import mujoco
                import mujoco.viewer

                viewer_handle = stack.enter_context(
                    mujoco.viewer.launch_passive(
                        env.model,
                        env.data,
                        key_callback=keyboard.on_key,
                        show_left_ui=False,
                        show_right_ui=False,
                    )
                )
                if args.input == "keyboard":
                    key_poller = X11HeldKeyPoller(keyboard.key_names)
                    stack.callback(key_poller.close)
                    viewer_handle.set_texts(
                        [
                            (
                                None,
                                mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                                "HOLD\nHOLD\nHOLD\nHOLD\nPRESS\nPRESS\nPRESS",
                                "Shift+W/S: X   Shift+A/D: Y   Shift+R/F: Z\n"
                                "Shift+O/L: gripper open/close\n"
                                "Shift+Up/Down: selected joint\n"
                                "Shift+J/K: previous/next joint\n"
                                "Shift+G: approach cube\n"
                                "Shift+P: verified scripted pick + lift\n"
                                "Shift+H: home   Shift+N: new   Shift+Q: quit",
                            ),
                            (
                                None,
                                mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                                "SO-101 SIM",
                                keyboard.status_text,
                            ),
                        ]
                    )

            while attempts < args.episodes and not keyboard.stop_requested:
                if viewer_handle is not None and not viewer_handle.is_running():
                    break
                frame_started = time.perf_counter()
                if key_poller is not None:
                    keyboard.poll_controls(key_poller, 1.0 / args.fps)
                if keyboard.stop_requested:
                    break
                if args.input == "keyboard" and keyboard.physics_paused:
                    if viewer_handle is not None:
                        viewer_handle.sync()
                    remaining = 1.0 / args.fps - (time.perf_counter() - frame_started)
                    if remaining > 0:
                        time.sleep(remaining)
                    continue
                action = action_source.get_action()
                with viewer_handle.lock() if viewer_handle is not None else contextlib.nullcontext():
                    next_observation, reward, terminated, truncated, info = env.step(action)
                if args.input == "keyboard":
                    keyboard.after_step()
                manually_ended = keyboard.consume_new_episode_request()
                done = terminated or truncated or manually_ended

                if dataset is not None:
                    add_dataset_frame(dataset, observation, action, reward, info, done, camera_names)
                observation = next_observation

                if viewer_handle is not None:
                    if args.input == "keyboard":
                        viewer_handle.set_texts(
                            [
                                (
                                    None,
                                    mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                                    "HOLD\nHOLD\nHOLD\nHOLD\nPRESS\nPRESS\nPRESS",
                                    "Shift+W/S: X   Shift+A/D: Y   Shift+R/F: Z\n"
                                    "Shift+O/L: gripper open/close\n"
                                    "Shift+Up/Down: selected joint\n"
                                    "Shift+J/K: previous/next joint\n"
                                    "Shift+G: approach cube\n"
                                    "Shift+P: verified scripted pick + lift\n"
                                    "Shift+H: home   Shift+N: new   Shift+Q: quit",
                                ),
                                (
                                    None,
                                    mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                                    "SO-101 SIM",
                                    keyboard.status_text,
                                ),
                            ]
                        )
                    viewer_handle.sync()

                if done:
                    attempts += 1
                    success = bool(info["is_success"])
                    successful_episodes += int(success)
                    saved = finish_episode(dataset, success=success, save_mode=args.save_mode)
                    saved_episodes += int(saved)
                    print(
                        f"attempt={attempts}/{args.episodes} success={success} "
                        f"saved={saved} total_saved={saved_episodes}"
                    )
                    if attempts >= args.episodes:
                        break
                    if args.input == "keyboard":
                        keyboard.resume_physics()
                        controller.reset()
                    with viewer_handle.lock() if viewer_handle is not None else contextlib.nullcontext():
                        observation, _ = env.reset(seed=args.seed + attempts)

                remaining = 1.0 / args.fps - (time.perf_counter() - frame_started)
                if remaining > 0:
                    time.sleep(remaining)
        if viewer_handle is not None:
            wait_for_viewer_shutdown(viewer_handle)
    finally:
        if dataset is not None:
            if dataset.has_pending_frames():
                dataset.clear_episode_buffer(delete_images=True)
            dataset.finalize()
        env.close()

    print(f"finished attempts={attempts} successes={successful_episodes} saved_episodes={saved_episodes}")
    print(
        "final_cube_position="
        f"{np.array2string(info['cube_position'], precision=4, suppress_small=True)} "
        "final_gripper_position="
        f"{np.array2string(info['gripper_position'], precision=4, suppress_small=True)}"
    )
    if dataset is not None:
        print(f"dataset_root={args.record_root.resolve()}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", choices=("keyboard", "leader"), default="keyboard")
    parser.add_argument("--leader-port", help="Serial port used only with --input=leader")
    parser.add_argument("--leader-id", default="so101_leader_main")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--steps", type=int, default=1800)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--joint-step-degrees", type=float, default=2.0)
    parser.add_argument("--gripper-step-percent", type=float, default=5.0)
    parser.add_argument("--joint-speed-degrees", type=float, default=35.0)
    parser.add_argument("--gripper-speed-percent", type=float, default=60.0)
    parser.add_argument("--cartesian-speed-m", type=float, default=0.08)
    parser.add_argument(
        "--cube-randomization",
        type=float,
        default=0.0,
        help="Half-width of cube XY randomization; zero keeps the verified interactive layout",
    )
    parser.add_argument("--front-only", action="store_true", help="Record only the front camera")
    parser.add_argument("--no-viewer", action="store_true", help="Useful for unattended leader runs")
    parser.add_argument("--record-root", type=Path, help="Optional new dataset directory")
    parser.add_argument("--repo-id", default="local/so101_mujoco_teleop")
    parser.add_argument(
        "--save-mode",
        choices=("successful", "all"),
        default="successful",
        help="By default, failed and interrupted episodes are discarded",
    )
    args = parser.parse_args()
    if args.episodes <= 0 or args.steps <= 0 or args.fps <= 0:
        parser.error("--episodes, --steps, and --fps must be positive")
    if args.joint_step_degrees <= 0 or args.gripper_step_percent <= 0:
        parser.error("Jog step sizes must be positive")
    if args.joint_speed_degrees <= 0 or args.gripper_speed_percent <= 0:
        parser.error("Joint and gripper speeds must be positive")
    if args.cartesian_speed_m <= 0:
        parser.error("--cartesian-speed-m must be positive")
    if args.cube_randomization < 0:
        parser.error("--cube-randomization must be non-negative")
    return args


if __name__ == "__main__":
    run(parse_args())
