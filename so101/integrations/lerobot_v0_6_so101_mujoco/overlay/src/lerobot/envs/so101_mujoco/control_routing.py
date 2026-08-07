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

"""Fail-closed camera-to-controller routing for SO-101 data and inference."""

from __future__ import annotations

import json
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

CONTROL_CONTRACT_SCHEMA_VERSION = "dapier.so101.control-route.v1"
ControlMode = Literal["ik_expert", "vla"]
RequestedControlMode = Literal["auto", "ik_expert", "vla"]


@dataclass(frozen=True)
class ControlRoute:
    """A controller choice derived from cameras, never from a silent fallback."""

    mode: ControlMode
    camera_names: tuple[str, ...]
    perception_camera: str
    dataset_role: str


def resolve_control_route(
    camera_names: Sequence[str],
    requested_mode: RequestedControlMode = "auto",
) -> ControlRoute:
    """Select IK expert for top+wrist or VLA for wrist-only observations.

    ``front`` is a viewer camera and does not participate in policy routing.
    IK requires wrist frames as student targets in addition to the top camera.
    VLA intentionally rejects a top camera so evaluation cannot accidentally
    depend on a view that will be absent on the real wrist-only setup.
    """
    cameras = tuple(camera_names)
    if not cameras or len(cameras) != len(set(cameras)):
        raise ValueError("camera_names must be non-empty and contain no duplicates")
    unknown = set(cameras) - {"front", "top", "wrist"}
    if unknown:
        raise ValueError(f"Unsupported control cameras: {sorted(unknown)}")
    policy_cameras = set(cameras) - {"front"}
    if policy_cameras == {"top", "wrist"}:
        derived_mode: ControlMode = "ik_expert"
        route = ControlRoute(
            mode=derived_mode,
            camera_names=cameras,
            perception_camera="top",
            dataset_role="teacher_demonstration",
        )
    elif policy_cameras == {"wrist"}:
        derived_mode = "vla"
        route = ControlRoute(
            mode=derived_mode,
            camera_names=cameras,
            perception_camera="wrist",
            dataset_role="student_inference",
        )
    else:
        raise ValueError(
            "Control routing requires top+wrist for IK expert collection or wrist-only for VLA inference"
        )
    if requested_mode not in {"auto", "ik_expert", "vla"}:
        raise ValueError(f"Unsupported requested control mode: {requested_mode!r}")
    if requested_mode != "auto" and requested_mode != derived_mode:
        raise ValueError(f"Requested {requested_mode!r}, but cameras {cameras!r} require {derived_mode!r}")
    return route


def build_ik_expert_dataset_contract(
    *,
    wrist_camera_profile_id: str,
    top_camera_profile_id: str,
) -> dict:
    """Describe how an IK teacher dataset becomes a wrist-only VLA dataset."""
    return {
        "schema_version": CONTROL_CONTRACT_SCHEMA_VERSION,
        "teacher": {
            "controller": "ik",
            "perception_camera": "top",
            "recorded_cameras": ["top", "wrist"],
            "uses_privileged_sim_object_state": False,
        },
        "student": {
            "controller": "vla",
            "inference_cameras": ["wrist"],
            "language_feature": "task",
            "state_feature": "observation.state",
            "action_feature": "action",
        },
        "camera_profiles": {
            "top": top_camera_profile_id,
            "wrist": wrist_camera_profile_id,
        },
        "student_dataset_derivation": {
            "remove_features": ["observation.images.top"],
            "retain_features": ["observation.images.wrist", "observation.state", "action", "task"],
        },
        "claims": {
            "ik_teacher_verified_in_sim": False,
            "vla_trained": False,
            "vla_evaluated": False,
            "physical_camera_alignment_verified": False,
        },
    }


def write_ik_expert_dataset_contract(
    dataset_root: Path,
    *,
    wrist_camera_profile_id: str,
    top_camera_profile_id: str,
) -> Path:
    """Write a sidecar without modifying LeRobot's own ``meta/info.json`` schema."""
    contract = build_ik_expert_dataset_contract(
        wrist_camera_profile_id=wrist_camera_profile_id,
        top_camera_profile_id=top_camera_profile_id,
    )
    output_path = Path(dataset_root) / "meta" / "dapier_control_route.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(contract, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return output_path


def build_wrist_student_dataset_command(
    *,
    python_executable: str,
    teacher_repo_id: str,
    teacher_root: Path,
    student_repo_id: str,
    student_root: Path,
) -> list[str]:
    """Build the LeRobot edit command that removes the privileged top view."""
    resolve_control_route(("top", "wrist"), requested_mode="ik_expert")
    resolve_control_route(("wrist",), requested_mode="vla")
    if not python_executable or not teacher_repo_id or not student_repo_id:
        raise ValueError("python_executable and dataset repo ids must be non-empty")
    if Path(teacher_root).resolve() == Path(student_root).resolve():
        raise ValueError("student_root must differ from teacher_root")
    return [
        python_executable,
        "-m",
        "lerobot.scripts.lerobot_edit_dataset",
        f"--repo_id={teacher_repo_id}",
        f"--root={teacher_root}",
        f"--new_repo_id={student_repo_id}",
        f"--new_root={student_root}",
        "--operation.type=remove_feature",
        '--operation.feature_names=["observation.images.top"]',
    ]


def build_wrist_vla_train_command(
    *,
    python_executable: str,
    dataset_repo_id: str,
    dataset_root: Path,
    output_dir: Path,
    steps: int,
    batch_size: int,
    seed: int,
) -> list[str]:
    """Build a local SmolVLA training command for the wrist-only student data."""
    resolve_control_route(("wrist",), requested_mode="vla")
    if not python_executable or not dataset_repo_id:
        raise ValueError("python_executable and dataset_repo_id must be non-empty")
    if min(steps, batch_size) <= 0 or seed < 0:
        raise ValueError("steps and batch_size must be positive and seed must be non-negative")
    return [
        python_executable,
        "-m",
        "lerobot.scripts.lerobot_train",
        f"--dataset.repo_id={dataset_repo_id}",
        f"--dataset.root={dataset_root}",
        "--policy.type=smolvla",
        "--policy.push_to_hub=false",
        "--wandb.enable=false",
        f"--output_dir={output_dir}",
        "--job_name=so101_wrist_smolvla",
        f"--steps={steps}",
        f"--batch_size={batch_size}",
        f"--seed={seed}",
    ]


def build_wrist_vla_eval_command(
    *,
    python_executable: str,
    policy_path: Path,
    output_dir: Path,
    episodes: int,
    steps: int,
    height: int,
    width: int,
    seed: int,
    cube_randomization: float,
) -> list[str]:
    """Build the standard LeRobot evaluator command for wrist-only VLA rollout."""
    resolve_control_route(("wrist",), requested_mode="vla")
    if not python_executable:
        raise ValueError("python_executable must be non-empty")
    if not str(policy_path):
        raise ValueError("policy_path must be non-empty")
    if min(episodes, steps, height, width) <= 0:
        raise ValueError("episodes, steps, height, and width must be positive")
    if seed < 0 or cube_randomization < 0:
        raise ValueError("seed and cube_randomization must be non-negative")
    return [
        python_executable,
        "-m",
        "lerobot.scripts.lerobot_eval",
        f"--policy.path={policy_path}",
        "--env.type=so101_mujoco",
        "--env.camera_names=[wrist]",
        "--env.obs_type=pixels_agent_pos",
        f"--env.episode_length={steps}",
        f"--env.observation_height={height}",
        f"--env.observation_width={width}",
        f"--env.cube_xy_randomization={cube_randomization}",
        f"--eval.n_episodes={episodes}",
        "--eval.batch_size=1",
        f"--seed={seed}",
        f"--output_dir={output_dir}",
    ]
