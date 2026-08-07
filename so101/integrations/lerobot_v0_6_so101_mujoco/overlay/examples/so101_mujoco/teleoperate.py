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
    JointJogController,
    SO101LeaderActionSource,
    should_save_episode,
)
from lerobot.utils.constants import ACTION, OBS_IMAGES, OBS_STATE


class ViewerKeyboard:
    """Thread-safe keyboard state used by MuJoCo's passive viewer callback."""

    def __init__(self, controller: JointJogController) -> None:
        self.controller = controller
        self._lock = threading.Lock()
        self._new_episode_requested = False
        self._stop_requested = False

    def on_key(self, keycode: int) -> None:
        import glfw

        with self._lock:
            if glfw.KEY_1 <= keycode <= glfw.KEY_6:
                self.controller.select_joint(keycode - glfw.KEY_1)
                print(f"selected_joint={self.controller.selected_joint_name}")
            elif keycode == glfw.KEY_UP:
                action = self.controller.jog(1)
                print(
                    f"target[{self.controller.selected_joint_name}]={action[self.controller.selected_joint]:.1f}"
                )
            elif keycode == glfw.KEY_DOWN:
                action = self.controller.jog(-1)
                print(
                    f"target[{self.controller.selected_joint_name}]={action[self.controller.selected_joint]:.1f}"
                )
            elif keycode == glfw.KEY_HOME:
                self.controller.reset()
                print("target=home")
            elif keycode == glfw.KEY_N:
                self._new_episode_requested = True
            elif keycode in {glfw.KEY_Q, glfw.KEY_ESCAPE}:
                self._stop_requested = True

    def get_action(self) -> np.ndarray:
        with self._lock:
            return self.controller.get_action()

    def consume_new_episode_request(self) -> bool:
        with self._lock:
            requested = self._new_episode_requested
            self._new_episode_requested = False
            return requested

    @property
    def stop_requested(self) -> bool:
        with self._lock:
            return self._stop_requested


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
    )
    dataset = (
        create_dataset(args.record_root, args.repo_id, args.fps, args.height, args.width, camera_names)
        if recording
        else None
    )
    controller = JointJogController(
        joint_step_degrees=args.joint_step_degrees,
        gripper_step_percent=args.gripper_step_percent,
    )
    keyboard = ViewerKeyboard(controller)
    action_source = (
        SO101LeaderActionSource(port=args.leader_port, leader_id=args.leader_id)
        if args.input == "leader"
        else keyboard
    )

    observation, _ = env.reset(seed=args.seed)
    assert env.model is not None and env.data is not None
    attempts = 0
    successful_episodes = 0
    saved_episodes = 0
    print("keys: 1..6 select joint | Up/Down jog | Home reset target | N new episode | Q/Esc quit")

    try:
        with contextlib.ExitStack() as stack:
            if args.input == "leader":
                action_source = stack.enter_context(action_source)

            viewer_handle = None
            if not args.no_viewer:
                import mujoco.viewer

                viewer_handle = stack.enter_context(
                    mujoco.viewer.launch_passive(env.model, env.data, key_callback=keyboard.on_key)
                )

            while attempts < args.episodes and not keyboard.stop_requested:
                if viewer_handle is not None and not viewer_handle.is_running():
                    break
                frame_started = time.perf_counter()
                action = action_source.get_action()
                with viewer_handle.lock() if viewer_handle is not None else contextlib.nullcontext():
                    next_observation, reward, terminated, truncated, info = env.step(action)
                manually_ended = keyboard.consume_new_episode_request()
                done = terminated or truncated or manually_ended

                if dataset is not None:
                    add_dataset_frame(dataset, observation, action, reward, info, done, camera_names)
                observation = next_observation

                if viewer_handle is not None:
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
                        controller.reset()
                    with viewer_handle.lock() if viewer_handle is not None else contextlib.nullcontext():
                        observation, _ = env.reset(seed=args.seed + attempts)

                remaining = 1.0 / args.fps - (time.perf_counter() - frame_started)
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        if dataset is not None:
            if dataset.has_pending_frames():
                dataset.clear_episode_buffer(delete_images=True)
            dataset.finalize()
        env.close()

    print(f"finished attempts={attempts} successes={successful_episodes} saved_episodes={saved_episodes}")
    if dataset is not None:
        print(f"dataset_root={args.record_root.resolve()}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", choices=("keyboard", "leader"), default="keyboard")
    parser.add_argument("--leader-port", help="Serial port used only with --input=leader")
    parser.add_argument("--leader-id", default="so101_leader_main")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--joint-step-degrees", type=float, default=2.0)
    parser.add_argument("--gripper-step-percent", type=float, default=5.0)
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
    return args


if __name__ == "__main__":
    run(parse_args())
