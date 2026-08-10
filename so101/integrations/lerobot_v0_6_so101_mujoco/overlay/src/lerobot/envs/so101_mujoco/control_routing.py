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

import hashlib
import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

CONTROL_CONTRACT_SCHEMA_VERSION = "dapier.so101.control-route.v1"
VLA_DATASET_HOME_ACTION_CLI = "[0,-45,17.5,90,0,100]"
VLA_EVALUATION_ACTION_STEPS = 25
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
            "vla_training_smoke_completed": False,
            "vla_inference_smoke_completed": False,
            "vla_trained": False,
            "vla_evaluated": False,
            "vla_success_threshold_met": False,
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


def mark_ik_expert_dataset_verified(dataset_root: Path, *, episodes: int, frames: int) -> Path:
    """Mark a completed all-success sim collection without overstating VLA or physical claims."""
    if min(episodes, frames) <= 0:
        raise ValueError("episodes and frames must be positive")
    output_path = Path(dataset_root) / "meta" / "dapier_control_route.json"
    if not output_path.is_file():
        raise FileNotFoundError(f"Missing IK expert dataset contract: {output_path}")
    contract = json.loads(output_path.read_text(encoding="utf-8"))
    if contract.get("schema_version") != CONTROL_CONTRACT_SCHEMA_VERSION:
        raise ValueError("Cannot verify a dataset with an unsupported control contract")
    contract["claims"]["ik_teacher_verified_in_sim"] = True
    contract["ik_teacher_verification"] = {
        "successful_episodes": episodes,
        "recorded_frames": frames,
        "success_filter": "all_saved_episodes_passed_environment_success_condition",
    }
    output_path.write_text(json.dumps(contract, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return output_path


def write_wrist_student_dataset_contract(
    teacher_root: Path,
    student_root: Path,
    *,
    student_features: Sequence[str],
    episodes: int,
    frames: int,
) -> Path:
    """Carry verified IK provenance into a top-free wrist student dataset."""
    if min(episodes, frames) <= 0:
        raise ValueError("episodes and frames must be positive")
    features = set(student_features)
    required = {"observation.images.wrist", "observation.state", "action"}
    if not required.issubset(features) or "observation.images.top" in features:
        raise ValueError("Student features must retain wrist/state/action and remove the top image")
    teacher_path = Path(teacher_root) / "meta" / "dapier_control_route.json"
    if not teacher_path.is_file():
        raise FileNotFoundError(f"Missing IK teacher contract: {teacher_path}")
    teacher_bytes = teacher_path.read_bytes()
    contract = json.loads(teacher_bytes)
    if contract.get("schema_version") != CONTROL_CONTRACT_SCHEMA_VERSION:
        raise ValueError("Cannot derive a student from an unsupported control contract")
    if contract.get("claims", {}).get("ik_teacher_verified_in_sim") is not True:
        raise ValueError("IK teacher must be verified before deriving a VLA student dataset")
    contract["student_dataset_derivation"].update(
        {
            "verified": True,
            "teacher_contract_sha256": hashlib.sha256(teacher_bytes).hexdigest(),
            "episodes": episodes,
            "frames": frames,
            "result_features": sorted(features),
        }
    )
    output_path = Path(student_root) / "meta" / "dapier_control_route.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(contract, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return output_path


def mark_wrist_vla_smoke_completed(
    student_root: Path,
    *,
    training_steps: int,
    rollout_steps: int,
    rollout_success: bool,
) -> Path:
    """Record an executable VLA smoke while keeping full training/evaluation claims false."""
    if min(training_steps, rollout_steps) <= 0:
        raise ValueError("training_steps and rollout_steps must be positive")
    output_path = Path(student_root) / "meta" / "dapier_control_route.json"
    if not output_path.is_file():
        raise FileNotFoundError(f"Missing wrist student contract: {output_path}")
    contract = json.loads(output_path.read_text(encoding="utf-8"))
    if contract.get("student_dataset_derivation", {}).get("verified") is not True:
        raise ValueError("Wrist student derivation must be verified before a VLA smoke")
    contract["claims"]["vla_training_smoke_completed"] = True
    contract["claims"]["vla_inference_smoke_completed"] = True
    contract["vla_smoke_verification"] = {
        "training_steps": training_steps,
        "rollout_steps": rollout_steps,
        "rollout_success": bool(rollout_success),
        "scope": "pipeline_smoke_not_trained_policy_performance",
    }
    output_path.write_text(json.dumps(contract, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return output_path


def mark_wrist_vla_training_evaluated(
    student_root: Path,
    *,
    checkpoint_path: Path,
    evaluation_output_path: Path,
    training_updates: int,
    batch_size: int,
    dataset_episodes: int,
    dataset_frames: int,
    training_seed_start: int,
    training_seed_end: int,
    evaluation_seed_start: int,
    evaluation_episodes: int,
    evaluation_action_steps: int,
    evaluation_home_action: Sequence[float],
    evaluation_cube_xy_randomization_m: float,
    successful_episodes: int,
    average_max_reward: float,
    average_sum_reward: float,
    success_threshold: float = 0.8,
) -> Path:
    """Record bounded wrist-only training and held-out evaluation evidence.

    A trained checkpoint is not the same claim as a task-qualified policy. The
    latter is recorded independently against ``success_threshold`` so a weak
    policy cannot be presented as a successful physical or sim-to-real result.
    """
    if training_updates <= 1 or min(batch_size, dataset_episodes, dataset_frames) <= 0:
        raise ValueError("training_updates must exceed smoke scope and dataset counts must be positive")
    if min(training_seed_start, evaluation_seed_start) < 0 or training_seed_end < training_seed_start:
        raise ValueError("seed ranges must be non-negative and ordered")
    if evaluation_episodes <= 0 or not 0 <= successful_episodes <= evaluation_episodes:
        raise ValueError("evaluation episode counts are invalid")
    if (
        isinstance(evaluation_action_steps, bool)
        or not isinstance(evaluation_action_steps, int)
        or evaluation_action_steps <= 0
    ):
        raise ValueError("evaluation_action_steps must be a positive integer")
    evaluation_home = tuple(float(value) for value in evaluation_home_action)
    if len(evaluation_home) != 6 or not all(math.isfinite(value) for value in evaluation_home):
        raise ValueError("evaluation_home_action must contain six finite values")
    evaluation_randomization = float(evaluation_cube_xy_randomization_m)
    if not math.isfinite(evaluation_randomization) or evaluation_randomization < 0:
        raise ValueError("evaluation_cube_xy_randomization_m must be finite and non-negative")
    if not 0 < success_threshold <= 1:
        raise ValueError("success_threshold must be in (0, 1]")
    metrics = (float(average_max_reward), float(average_sum_reward))
    if not all(math.isfinite(value) for value in metrics):
        raise ValueError("evaluation rewards must be finite")
    evaluation_seed_end = evaluation_seed_start + evaluation_episodes - 1
    ranges_overlap = not (
        evaluation_seed_end < training_seed_start or evaluation_seed_start > training_seed_end
    )
    if ranges_overlap:
        raise ValueError("held-out evaluation seeds must not overlap the expert collection seeds")

    checkpoint = Path(checkpoint_path)
    evaluation_output = Path(evaluation_output_path)
    if not checkpoint.is_dir():
        raise FileNotFoundError(f"Missing trained checkpoint directory: {checkpoint}")
    if not evaluation_output.is_dir():
        raise FileNotFoundError(f"Missing evaluation output directory: {evaluation_output}")

    output_path = Path(student_root) / "meta" / "dapier_control_route.json"
    if not output_path.is_file():
        raise FileNotFoundError(f"Missing wrist student contract: {output_path}")
    contract = json.loads(output_path.read_text(encoding="utf-8"))
    if contract.get("schema_version") != CONTROL_CONTRACT_SCHEMA_VERSION:
        raise ValueError("Cannot record VLA evidence for an unsupported control contract")
    if contract.get("student_dataset_derivation", {}).get("verified") is not True:
        raise ValueError("Wrist student derivation must be verified before recording VLA evidence")

    success_rate = successful_episodes / evaluation_episodes
    threshold_met = success_rate >= success_threshold
    contract["claims"]["vla_trained"] = True
    contract["claims"]["vla_evaluated"] = True
    contract["claims"]["vla_success_threshold_met"] = threshold_met
    contract["vla_training_evaluation"] = {
        "scope": "bounded_local_wrist_only_training_not_physical_validation",
        "checkpoint_path": str(checkpoint.resolve()),
        "evaluation_output_path": str(evaluation_output.resolve()),
        "training_updates": training_updates,
        "batch_size": batch_size,
        "training_samples_seen": training_updates * batch_size,
        "dataset_episodes": dataset_episodes,
        "dataset_frames": dataset_frames,
        "approximate_dataset_epochs": round(training_updates * batch_size / dataset_frames, 6),
        "expert_collection_seed_range": [training_seed_start, training_seed_end],
        "held_out_evaluation_seed_range": [evaluation_seed_start, evaluation_seed_end],
        "evaluation_episodes": evaluation_episodes,
        "evaluation_action_steps": evaluation_action_steps,
        "evaluation_home_action": list(evaluation_home),
        "evaluation_cube_xy_randomization_m": evaluation_randomization,
        "successful_episodes": successful_episodes,
        "success_rate": success_rate,
        "success_threshold": success_threshold,
        "success_threshold_met": threshold_met,
        "average_max_reward": metrics[0],
        "average_sum_reward": metrics[1],
        "physical_rollout_executed": False,
    }
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
        "--policy.load_vlm_weights=true",
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
        f"--policy.n_action_steps={VLA_EVALUATION_ACTION_STEPS}",
        "--env.type=so101_mujoco",
        "--env.camera_names=[wrist]",
        "--env.obs_type=pixels_agent_pos",
        f"--env.home_action={VLA_DATASET_HOME_ACTION_CLI}",
        f"--env.episode_length={steps}",
        f"--env.observation_height={height}",
        f"--env.observation_width={width}",
        f"--env.cube_xy_randomization={cube_randomization}",
        f"--eval.n_episodes={episodes}",
        "--eval.batch_size=1",
        f"--seed={seed}",
        f"--output_dir={output_dir}",
    ]
