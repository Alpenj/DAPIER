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

"""Run a leader-free SO-101 trajectory and optionally record a LeRobot dataset."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from lerobot.envs.so101_mujoco import ACTION_HIGH, ACTION_LOW, JOINT_NAMES, SO101MujocoEnv
from lerobot.utils.constants import ACTION, OBS_IMAGES, OBS_STATE


def scripted_joint_sweep(step: int, episode_steps: int) -> np.ndarray:
    """A conservative, deterministic motion used to test the full data path."""
    phase = 2.0 * np.pi * step / max(episode_steps, 1)
    action = np.array(
        [
            25.0 * np.sin(phase),
            -35.0 + 8.0 * np.sin(phase + np.pi / 3),
            55.0 + 10.0 * np.sin(phase + 2 * np.pi / 3),
            35.0 + 8.0 * np.sin(phase + np.pi),
            20.0 * np.sin(phase + 4 * np.pi / 3),
            65.0 + 30.0 * np.sin(phase + 5 * np.pi / 3),
        ],
        dtype=np.float32,
    )
    return np.clip(action, ACTION_LOW, ACTION_HIGH)


def _create_dataset(root: Path, repo_id: str, fps: int, height: int, width: int):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features = {
        OBS_STATE: {"dtype": "float32", "shape": (6,), "names": list(JOINT_NAMES)},
        f"{OBS_IMAGES}.front": {
            "dtype": "image",
            "shape": (height, width, 3),
            "names": ["height", "width", "channel"],
        },
        ACTION: {"dtype": "float32", "shape": (6,), "names": list(JOINT_NAMES)},
        "next.reward": {"dtype": "float32", "shape": (1,), "names": None},
        "next.success": {"dtype": "bool", "shape": (1,), "names": None},
        "next.done": {"dtype": "bool", "shape": (1,), "names": None},
    }
    return LeRobotDataset.create(
        repo_id=repo_id,
        root=root,
        fps=fps,
        robot_type="so101_mujoco",
        features=features,
        use_videos=False,
        image_writer_threads=2,
    )


def run(args: argparse.Namespace) -> None:
    recording = args.record_root is not None
    env = SO101MujocoEnv(
        obs_type="pixels_agent_pos" if recording else "state",
        observation_height=args.height,
        observation_width=args.width,
        fps=args.fps,
        max_episode_steps=args.steps,
        terminate_on_success=False,
    )
    dataset = (
        _create_dataset(args.record_root, args.repo_id, args.fps, args.height, args.width)
        if recording
        else None
    )

    try:
        for episode_index in range(args.episodes):
            observation, _ = env.reset(seed=args.seed + episode_index)
            total_reward = 0.0
            for step in range(args.steps):
                action = scripted_joint_sweep(step, args.steps)
                next_observation, reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated or step == args.steps - 1
                total_reward += reward

                if dataset is not None:
                    dataset.add_frame(
                        {
                            OBS_STATE: observation["agent_pos"],
                            f"{OBS_IMAGES}.front": observation["pixels"],
                            ACTION: action,
                            "next.reward": np.array([reward], dtype=np.float32),
                            "next.success": np.array([info["is_success"]], dtype=bool),
                            "next.done": np.array([done], dtype=bool),
                            "task": "Move every SO-101 joint through a conservative diagnostic trajectory.",
                        }
                    )
                observation = next_observation
                if done:
                    break

            if dataset is not None:
                dataset.save_episode(parallel_encoding=False)
            print(
                f"episode={episode_index} steps={step + 1} "
                f"mean_reward={total_reward / (step + 1):.4f} recorded={recording}"
            )
    finally:
        env.close()
        if dataset is not None:
            dataset.finalize()

    if dataset is not None:
        print(f"dataset_root={args.record_root.resolve()}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--steps", type=int, default=90)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--record-root",
        type=Path,
        help="Optional output directory. Omit it for a simulation-only smoke run.",
    )
    parser.add_argument("--repo-id", default="local/so101_mujoco_joint_sweep")
    args = parser.parse_args()
    if args.episodes <= 0 or args.steps <= 0 or args.fps <= 0:
        parser.error("--episodes, --steps, and --fps must be positive")
    return args


if __name__ == "__main__":
    run(parse_args())
