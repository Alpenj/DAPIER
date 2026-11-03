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

"""Collect successful top-RGB IK demonstrations with synchronized wrist images."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.envs.so101_mujoco import (
    FINGER_PAD_CUBE_CONTACT_FRICTION,
    FINGER_PAD_CUBE_CONTACT_SOLREF,
    GOAL_TRAY_POSITION,
    IK_OBSERVE_ACTION,
    JOINT_NAMES,
    PICK_CLEAR_ACTION,
    POLICY_CAMERA_NAMES,
    TOP_CAMERA_PROFILE_ID,
    VISION_GRASP_CLOSE_PERCENT,
    VISION_GRASP_Z_OFFSET_M,
    VISION_MAX_PAD_PENETRATION_M,
    VISION_SETTLE_FRAMES,
    WRIST_CAMERA_PROFILE_ID,
    SO101MujocoEnv,
    build_vision_pick_place_plan,
    estimate_blue_cube_world_position,
    mark_ik_expert_dataset_verified,
    resolve_control_route,
    write_ik_expert_dataset_contract,
)
from lerobot.utils.constants import ACTION, OBS_IMAGES, OBS_STATE

TASK = "Pick up the blue cube and place it in the green tray."


def create_dataset(root: Path, repo_id: str, fps: int, height: int, width: int):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features: dict[str, dict[str, Any]] = {
        OBS_STATE: {"dtype": "float32", "shape": (6,), "names": list(JOINT_NAMES)},
        ACTION: {"dtype": "float32", "shape": (6,), "names": list(JOINT_NAMES)},
        "next.reward": {"dtype": "float32", "shape": (1,), "names": None},
        "next.success": {"dtype": "bool", "shape": (1,), "names": None},
        "next.done": {"dtype": "bool", "shape": (1,), "names": None},
    }
    for camera_name in POLICY_CAMERA_NAMES:
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
        image_writer_threads=4,
    )


def add_frame(dataset, observation, action, reward, info, done: bool) -> None:
    frame = {
        OBS_STATE: observation["agent_pos"],
        ACTION: np.asarray(action, dtype=np.float32),
        "next.reward": np.array([reward], dtype=np.float32),
        "next.success": np.array([info["is_success"]], dtype=bool),
        "next.done": np.array([done], dtype=bool),
        "task": TASK,
    }
    for camera_name in POLICY_CAMERA_NAMES:
        frame[f"{OBS_IMAGES}.{camera_name}"] = observation["pixels"][camera_name]
    dataset.add_frame(frame)


def run(args: argparse.Namespace) -> None:
    resolve_control_route(POLICY_CAMERA_NAMES, requested_mode="ik_expert")
    if args.root.exists():
        raise FileExistsError(f"Refusing to overwrite dataset root: {args.root}")

    dataset = create_dataset(args.root, args.repo_id, args.fps, args.height, args.width)
    write_ik_expert_dataset_contract(
        args.root,
        wrist_camera_profile_id=WRIST_CAMERA_PROFILE_ID,
        top_camera_profile_id=TOP_CAMERA_PROFILE_ID,
    )
    env = SO101MujocoEnv(
        obs_type="pixels_agent_pos",
        camera_names=POLICY_CAMERA_NAMES,
        observation_height=args.height,
        observation_width=args.width,
        fps=args.fps,
        max_episode_steps=VISION_SETTLE_FRAMES + 1000,
        terminate_on_success=False,
        cube_xy_randomization=args.cube_randomization,
        home_action=PICK_CLEAR_ACTION,
    )
    total_frames = 0
    errors_mm: list[float] = []
    penetrations_mm: list[float] = []
    completed = False
    try:
        for episode_index in range(args.episodes):
            seed = args.seed + episode_index
            observation, info = env.reset(seed=seed)
            episode_frames = 0
            for _ in range(VISION_SETTLE_FRAMES):
                next_observation, reward, _, _, info = env.step(IK_OBSERVE_ACTION)
                add_frame(dataset, observation, IK_OBSERVE_ACTION, reward, info, done=False)
                observation = next_observation
                episode_frames += 1

            estimate = estimate_blue_cube_world_position(env.render("top"), env.camera_calibration("top"))
            error_mm = float(
                np.linalg.norm(estimate.world_xyz[:2] - np.asarray(info["cube_position"])[:2]) * 1000
            )
            plan = build_vision_pick_place_plan(
                env.model,
                estimate.world_xyz[:2],
                goal_xy=GOAL_TRAY_POSITION[:2],
            )
            max_penetration_m = 0.0
            bilateral_contact_frames = 0
            for frame_index, action in enumerate(plan.actions):
                is_last = frame_index == len(plan.actions) - 1
                next_observation, reward, _, _, info = env.step(action)
                add_frame(dataset, observation, action, reward, info, done=is_last)
                observation = next_observation
                episode_frames += 1
                max_penetration_m = max(
                    max_penetration_m,
                    float(info["finger_pad_cube_max_penetration_m"]),
                )
                bilateral_contact_frames += int(info["finger_pad_cube_bilateral_contact"])

            if not info["is_success"]:
                dataset.clear_episode_buffer(delete_images=True)
                raise RuntimeError(f"IK expert failed at episode={episode_index} seed={seed}")
            if max_penetration_m > VISION_MAX_PAD_PENETRATION_M:
                dataset.clear_episode_buffer(delete_images=True)
                raise RuntimeError(
                    f"IK expert exceeded penetration gate at episode={episode_index} seed={seed}: "
                    f"{max_penetration_m * 1000:.3f} mm"
                )
            if bilateral_contact_frames == 0:
                dataset.clear_episode_buffer(delete_images=True)
                raise RuntimeError(
                    f"IK expert never achieved bilateral contact at episode={episode_index} seed={seed}"
                )
            dataset.save_episode(parallel_encoding=False)
            total_frames += episode_frames
            errors_mm.append(error_mm)
            penetrations_mm.append(max_penetration_m * 1000)
            print(
                f"episode={episode_index + 1}/{args.episodes} seed={seed} "
                f"frames={episode_frames} rgb_xy_error_mm={error_mm:.3f} "
                f"max_penetration_mm={max_penetration_m * 1000:.3f} "
                f"bilateral_contact_frames={bilateral_contact_frames} success=True"
            )
        completed = True
    finally:
        if dataset.has_pending_frames():
            dataset.clear_episode_buffer(delete_images=True)
        dataset.finalize()
        env.close()

    if completed:
        mark_ik_expert_dataset_verified(args.root, episodes=args.episodes, frames=total_frames)
        validation_path = args.root / "meta" / "dapier_corrected_ik_validation.json"
        validation_path.write_text(
            json.dumps(
                {
                    "schema_version": "dapier.so101.corrected-ik-validation.v1",
                    "execution": {
                        "seed_start": args.seed,
                        "seed_end": args.seed + args.episodes - 1,
                        "episodes": args.episodes,
                        "cube_xy_randomization_m": args.cube_randomization,
                    },
                    "teacher": {
                        "grasp_close_percent": VISION_GRASP_CLOSE_PERCENT,
                        "grasp_z_offset_m": VISION_GRASP_Z_OFFSET_M,
                        "max_allowed_pad_penetration_m": VISION_MAX_PAD_PENETRATION_M,
                        "finger_pad_cube_contact_friction": list(FINGER_PAD_CUBE_CONTACT_FRICTION),
                        "finger_pad_cube_contact_solref": list(FINGER_PAD_CUBE_CONTACT_SOLREF),
                    },
                    "results": {
                        "successful_episodes": args.episodes,
                        "recorded_frames": total_frames,
                        "mean_rgb_xy_error_mm": float(np.mean(errors_mm)),
                        "max_rgb_xy_error_mm": float(np.max(errors_mm)),
                        "max_observed_pad_penetration_mm": float(np.max(penetrations_mm)),
                    },
                    "claims": {
                        "simulation_only": True,
                        "physical_grasp_verified": False,
                        "ready_for_student_dataset_derivation": True,
                        "vla_optimizer_updates": 0,
                    },
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(
            f"summary={args.episodes}/{args.episodes} frames={total_frames} "
            f"mean_error_mm={np.mean(errors_mm):.3f} max_error_mm={np.max(errors_mm):.3f} "
            f"max_penetration_mm={np.max(penetrations_mm):.3f} "
            f"grasp_close_percent={VISION_GRASP_CLOSE_PERCENT:.1f} "
            f"grasp_z_offset_mm={VISION_GRASP_Z_OFFSET_M * 1000:.1f}"
        )
        print(f"corrected_ik_validation={validation_path.resolve()}")
        print(f"dataset_root={args.root.resolve()}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-id", default="local/so101_ik_teacher")
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--seed", type=int, default=200)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--cube-randomization", type=float, default=0.025)
    args = parser.parse_args()
    if min(args.episodes, args.fps, args.height, args.width) <= 0:
        parser.error("--episodes, --fps, --height, and --width must be positive")
    if args.seed < 0 or args.cube_randomization < 0:
        parser.error("--seed and --cube-randomization must be non-negative")
    return args


if __name__ == "__main__":
    run(parse_args())
