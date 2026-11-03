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

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from lerobot.envs.configs import SO101MujocoEnvConfig
from lerobot.envs.so101_mujoco import (
    ACTION_HIGH,
    ACTION_LOW,
    ACTION_TRACE_CONTRACT_ID,
    CUBE_SPAWN_POSITION,
    CUBE_TOP_PLANE_Z_M,
    DEFAULT_VLA_ACTION_MAX_DELTA,
    FINGER_PAD_CUBE_CONTACT_FRICTION,
    FINGER_PAD_CUBE_CONTACT_SOLREF,
    FINGER_PAD_GEOM_NAMES,
    GOAL_TRAY_POSITION,
    HUMAN_AUTHORITY,
    IK_OBSERVE_ACTION,
    JOINT_NAMES,
    PICK_CLEAR_ACTION,
    PICK_LIFT_FRAMES,
    POLICY_AUTHORITY,
    POLICY_CAMERA_NAMES,
    TOP_CAMERA_PROFILE_ID,
    VISION_GRASP_CLOSE_PERCENT,
    VISION_GRASP_Z_OFFSET_M,
    VISION_MAX_PAD_PENETRATION_M,
    VISION_PICK_PLACE_FRAMES,
    WRIST_CAMERA_HOUSING_GEOM_NAME,
    WRIST_CAMERA_LENS_GEOM_NAME,
    WRIST_CAMERA_MOUNT_GEOM_NAME,
    WRIST_CAMERA_PROFILE_ID,
    CameraCalibration,
    CartesianJogController,
    HardwareInventory,
    InterventionEpisodeRecorder,
    JointJogController,
    ResetSeedSequence,
    SO101MujocoEnv,
    VideoDevice,
    VLAActionFilter,
    VLAInterventionSession,
    build_ik_expert_dataset_contract,
    build_physical_wrist_gate_receipt,
    build_vision_pick_place_plan,
    build_wrist_student_dataset_command,
    build_wrist_vla_eval_command,
    build_wrist_vla_train_command,
    camera_profile,
    detect_blue_cube,
    estimate_blue_cube_world_position,
    leader_action_dict_to_array,
    lerobot_action_to_qpos,
    mark_ik_expert_dataset_verified,
    mark_wrist_vla_smoke_completed,
    mark_wrist_vla_training_evaluated,
    project_pixel_to_horizontal_plane,
    qpos_to_lerobot_state,
    resolve_control_route,
    scripted_pick_lift_action,
    should_save_episode,
    write_ik_expert_dataset_contract,
    write_parallel_rollout_manifest,
    write_physical_wrist_gate_receipt,
    write_wrist_student_dataset_contract,
)
from lerobot.envs.utils import preprocess_observation


def test_joint_contract_and_round_trip():
    assert JOINT_NAMES == (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    )
    for action in (
        ACTION_LOW,
        ACTION_HIGH,
        np.array([0, -35, 55, 35, 0, 50], dtype=np.float32),
    ):
        np.testing.assert_allclose(qpos_to_lerobot_state(lerobot_action_to_qpos(action)), action, atol=1e-4)


def test_joint_contract_rejects_wrong_shapes():
    with pytest.raises(ValueError, match="shape \\(6,\\)"):
        lerobot_action_to_qpos(np.zeros(5))
    with pytest.raises(ValueError, match="shape \\(6,\\)"):
        qpos_to_lerobot_state(np.zeros(7))


def test_vla_intervention_authority_switch_discards_implicit_policy_control():
    session = VLAInterventionSession()
    policy_action = np.array([1, -40, 20, 80, 0, 90], dtype=np.float32)
    measured_action = np.array([2, -39, 21, 79, 1, 88], dtype=np.float32)
    human_action = measured_action.copy()
    human_action[5] = 92

    policy_decision = session.choose_action(policy_action=policy_action)
    assert policy_decision.source == POLICY_AUTHORITY
    np.testing.assert_array_equal(policy_decision.action, policy_action)

    assert session.take_over(measured_action) is True
    assert session.take_over(measured_action) is False
    human_decision = session.choose_action(policy_action=None, human_action=human_action)
    assert human_decision.source == HUMAN_AUTHORITY
    assert human_decision.intervention_segment == 1
    np.testing.assert_array_equal(human_decision.action, human_action)
    assert session.intervention_frames == 1

    assert session.resume_policy() is True
    assert session.resume_policy() is False
    with pytest.raises(ValueError, match="policy_action is required"):
        session.choose_action(policy_action=None)


def test_intervention_recorder_writes_source_labeled_evidence(tmp_path: Path):
    recorder = InterventionEpisodeRecorder(tmp_path)
    episode_dir = recorder.start_episode(episode_index=0, seed=17, task="pick")
    action = np.array([0, -45, 17.5, 90, 0, 100], dtype=np.float32)
    recorder.record_frame(
        step_index=0,
        source=HUMAN_AUTHORITY,
        intervention_segment=1,
        observation_state=action,
        wrist_rgb=np.zeros((8, 10, 3), dtype=np.uint8),
        requested_action=action,
        applied_action=action,
        last_policy_action=None,
        reward=0.5,
        success=False,
        done=False,
    )
    manifest_path = recorder.finish_episode(
        success=False, termination_reason="manual_next", intervention_segments=1
    )

    event = json.loads((episode_dir / "events.jsonl").read_text())
    manifest = json.loads(manifest_path.read_text())
    assert event["source"] == HUMAN_AUTHORITY
    assert event["intervention_segment"] == 1
    assert (episode_dir / event["wrist_image"]).is_file()
    assert manifest["human_intervention_frames"] == 1
    assert manifest["training_status"] == "evidence_only_requires_dataset_conversion"


def test_vla_action_filter_blends_chunk_boundaries_and_limits_outliers():
    action_filter = VLAActionFilter(
        enabled=True,
        action_chunk_steps=4,
        action_blend_steps=2,
        action_max_delta=(1, 1, 1, 1, 1, 2),
        gripper_action_deadband=0.5,
    )
    action_filter.reset(np.zeros(6, dtype=np.float32))

    results = [action_filter.apply(np.array([4, 0, 0, 0, 0, 0.2], dtype=np.float32)) for _ in range(4)]
    assert results[0].chunk_boundary is True
    assert results[0].blend_weight == pytest.approx(0.5)
    assert results[0].applied_action[0] == pytest.approx(1.0)
    assert results[0].applied_action[5] == pytest.approx(0.0)
    assert results[0].gripper_deadband_applied is True
    assert all(np.max(np.abs(result.applied_delta[:5])) <= 1.0 for result in results)

    next_chunk = action_filter.apply(np.array([-4, 0, 0, 0, 0, 0], dtype=np.float32))
    assert next_chunk.chunk_boundary is True
    assert next_chunk.applied_delta[0] == pytest.approx(-1.0)
    assert next_chunk.slew_limited_axes[0]


def test_disabled_vla_action_filter_preserves_bounded_actions():
    action_filter = VLAActionFilter(
        enabled=False,
        action_chunk_steps=25,
        action_blend_steps=3,
        action_max_delta=np.ones(6),
        gripper_action_deadband=1.0,
    )
    action_filter.reset(np.zeros(6, dtype=np.float32))
    result = action_filter.apply(np.array([4, 3, 2, 1, 0, 0.2], dtype=np.float32))
    np.testing.assert_allclose(result.applied_action, [4, 3, 2, 1, 0, 0.2])
    assert result.action_filtered is False
    assert not np.any(result.slew_limited_axes)


def test_keyboard_jog_controller_is_bounded_and_resettable():
    controller = JointJogController(joint_step_degrees=10, gripper_step_percent=25)
    controller.select_joint(0)
    for _ in range(100):
        controller.jog(1)
    assert controller.get_action()[0] == ACTION_HIGH[0]

    controller.select_joint(5)
    controller.jog(-1)
    assert controller.get_action()[5] == 75
    np.testing.assert_array_equal(controller.reset(), np.array([0, -35, 55, 35, 0, 100]))


def test_joint_selection_wraps_and_direct_adjustment_is_bounded():
    controller = JointJogController()
    assert controller.select_previous_joint() == 5
    assert controller.select_next_joint() == 0
    controller.adjust_joint(5, -1000)
    assert controller.get_action()[5] == ACTION_LOW[5]
    with pytest.raises(ValueError, match="shape"):
        controller.set_action(np.zeros(5))


def test_official_leader_action_mapping():
    action_dict = {f"{name}.pos": value for name, value in zip(JOINT_NAMES, [1, 2, 3, 4, 5, 6], strict=True)}
    np.testing.assert_array_equal(leader_action_dict_to_array(action_dict), [1, 2, 3, 4, 5, 6])
    with pytest.raises(ValueError, match="missing keys"):
        leader_action_dict_to_array({"shoulder_pan.pos": 0})


def test_success_only_episode_selection():
    assert should_save_episode(success=True, save_mode="successful")
    assert not should_save_episode(success=False, save_mode="successful")
    assert should_save_episode(success=False, save_mode="all")
    with pytest.raises(ValueError, match="Unsupported"):
        should_save_episode(success=True, save_mode="unknown")


def test_every_scene_reset_gets_the_next_seed_without_rewinding():
    sequence = ResetSeedSequence(101)
    assert sequence.initial_seed == 101
    assert [sequence.next_seed() for _ in range(4)] == [102, 103, 104, 105]
    assert sequence.reset_count == 4
    with pytest.raises(ValueError, match="non-negative"):
        ResetSeedSequence(-1)


def test_wrist_camera_profile_preserves_cad_provenance_without_claiming_physical_calibration():
    profile = camera_profile("wrist")
    assert profile.profile_id == WRIST_CAMERA_PROFILE_ID
    assert profile.parent_body == "gripper"
    np.testing.assert_allclose(profile.position_m, [0.0025, -0.072057361, 0.004150235])
    np.testing.assert_allclose(
        -np.cross(profile.xyaxes[:3], profile.xyaxes[3:]),
        [0, 0.422618262, -0.906307787],
    )
    assert profile.provenance["source_revision"] == "7629d2ad9853d10fb903093a33ef6114099d97e5"
    assert profile.provenance["source_sha256"] == (
        "b4345ccf23f1f2ed3f4885c205cac5afbed6ddd1b183617c4801751e3bafb7b4"
    )
    assert profile.verification["lens_optical_center"] == "unverified_zero_offset"
    assert profile.physical_alignment_verified is False


def test_camera_set_routes_top_wrist_to_ik_and_wrist_only_to_vla():
    expert = resolve_control_route(("top", "wrist"))
    assert expert.mode == "ik_expert"
    assert expert.perception_camera == "top"
    student = resolve_control_route(("wrist",))
    assert student.mode == "vla"
    assert student.perception_camera == "wrist"
    with pytest.raises(ValueError, match="require"):
        resolve_control_route(("top",))
    with pytest.raises(ValueError, match="Requested"):
        resolve_control_route(("top", "wrist"), requested_mode="vla")


def test_ik_expert_sidecar_defines_wrist_only_student_derivation(tmp_path):
    contract = build_ik_expert_dataset_contract(
        wrist_camera_profile_id=WRIST_CAMERA_PROFILE_ID,
        top_camera_profile_id=TOP_CAMERA_PROFILE_ID,
    )
    assert contract["teacher"]["controller"] == "ik"
    assert contract["teacher"]["recorded_cameras"] == ["top", "wrist"]
    assert contract["student"]["controller"] == "vla"
    assert contract["student"]["inference_cameras"] == ["wrist"]
    assert contract["student_dataset_derivation"]["remove_features"] == ["observation.images.top"]
    assert contract["claims"]["vla_trained"] is False
    path = write_ik_expert_dataset_contract(
        tmp_path,
        wrist_camera_profile_id=WRIST_CAMERA_PROFILE_ID,
        top_camera_profile_id=TOP_CAMERA_PROFILE_ID,
    )
    assert path == tmp_path / "meta" / "dapier_control_route.json"
    assert path.is_file()
    mark_ik_expert_dataset_verified(tmp_path, episodes=3, frames=1980)
    verified = json.loads(path.read_text())
    assert verified["claims"]["ik_teacher_verified_in_sim"] is True
    assert verified["ik_teacher_verification"]["successful_episodes"] == 3
    assert verified["claims"]["vla_trained"] is False
    student_path = write_wrist_student_dataset_contract(
        tmp_path,
        tmp_path / "student",
        student_features=("observation.images.wrist", "observation.state", "action"),
        episodes=3,
        frames=1980,
    )
    student = json.loads(student_path.read_text())
    assert student["student_dataset_derivation"]["verified"] is True
    assert len(student["student_dataset_derivation"]["teacher_contract_sha256"]) == 64
    with pytest.raises(ValueError, match="remove the top"):
        write_wrist_student_dataset_contract(
            tmp_path,
            tmp_path / "bad-student",
            student_features=("observation.images.top", "observation.images.wrist"),
            episodes=3,
            frames=1980,
        )
    smoke_path = mark_wrist_vla_smoke_completed(
        tmp_path / "student", training_steps=1, rollout_steps=5, rollout_success=False
    )
    smoke = json.loads(smoke_path.read_text())
    assert smoke["claims"]["vla_training_smoke_completed"] is True
    assert smoke["claims"]["vla_inference_smoke_completed"] is True
    assert smoke["claims"]["vla_trained"] is False
    assert smoke["claims"]["vla_evaluated"] is False
    assert smoke["vla_smoke_verification"]["rollout_success"] is False


def test_wrist_vla_route_delegates_to_standard_lerobot_evaluator(tmp_path):
    command = build_wrist_vla_eval_command(
        python_executable="python",
        policy_path=Path("checkpoints/wrist-smolvla"),
        output_dir=tmp_path / "eval",
        episodes=3,
        steps=700,
        height=240,
        width=320,
        seed=7,
        cube_randomization=0.025,
    )
    assert command[:3] == ["python", "-m", "lerobot.scripts.lerobot_eval"]
    assert "--env.camera_names=[wrist]" in command
    assert "--env.cube_xy_randomization=0.025" in command
    assert "--env.home_action=[0,-45,17.5,90,0,100]" in command
    assert "--policy.n_action_steps=25" in command
    assert "--env.action_smoothing=true" in command
    assert "--env.action_chunk_steps=25" in command
    assert "--env.action_blend_steps=3" in command
    assert "--env.action_max_delta=[1.75,0.65,0.30,0.35,0.12,5.50]" in command
    assert "--env.gripper_action_deadband=1.0" in command
    assert f"--env.action_trace_path={tmp_path / 'eval' / 'action_trace.jsonl'}" in command
    assert "--policy.path=checkpoints/wrist-smolvla" in command
    assert "--eval.batch_size=1" in command


def test_wrist_vla_parallel_route_uses_batched_policy_and_worker_traces(tmp_path):
    command = build_wrist_vla_eval_command(
        python_executable="python",
        policy_path=Path("checkpoints/wrist-smolvla"),
        output_dir=tmp_path / "eval",
        episodes=8,
        steps=700,
        height=240,
        width=320,
        seed=1600,
        cube_randomization=0.025,
        parallel_envs=4,
    )
    assert "--eval.batch_size=4" in command
    assert (
        f"--env.action_trace_path={tmp_path / 'eval' / 'action_traces' / 'env_{env_index}.jsonl'}" in command
    )
    with pytest.raises(ValueError, match="cannot exceed"):
        build_wrist_vla_eval_command(
            python_executable="python",
            policy_path=Path("checkpoints/wrist-smolvla"),
            output_dir=tmp_path / "bad",
            episodes=2,
            steps=10,
            height=24,
            width=32,
            seed=0,
            cube_randomization=0,
            parallel_envs=3,
        )


def test_parallel_rollout_manifest_labels_experience_without_claiming_training(
    tmp_path,
):
    policy_path = tmp_path / "policy"
    policy_path.mkdir()
    eval_dir = tmp_path / "eval"
    eval_dir.mkdir()
    trace_dir = eval_dir / "action_traces"
    trace_dir.mkdir()
    (trace_dir / "env_0.jsonl").write_text(
        json.dumps({"episode_seed": 1700, "episode_index": 0})
        + "\n"
        + json.dumps({"episode_seed": 1702, "episode_index": 3})
        + "\n",
        encoding="utf-8",
    )
    (trace_dir / "env_1.jsonl").write_text(
        json.dumps({"episode_seed": 1701, "episode_index": 0})
        + "\n"
        + json.dumps({"episode_seed": 1703, "episode_index": 2})
        + "\n",
        encoding="utf-8",
    )
    eval_info_path = eval_dir / "eval_info.json"
    eval_info_path.write_text(
        json.dumps(
            {
                "per_task": [
                    {
                        "task_group": "so101_mujoco",
                        "task_id": 0,
                        "metrics": {
                            "sum_rewards": [10.0, 20.0, 30.0, 40.0],
                            "max_rewards": [0.1, 0.2, 0.3, 0.4],
                            "successes": [False, True, True, False],
                        },
                    }
                ],
                "overall": {"eval_s": 8.0, "eval_ep_s": 2.0},
            }
        ),
        encoding="utf-8",
    )
    manifest_path = write_parallel_rollout_manifest(
        eval_info_path=eval_info_path,
        policy_path=policy_path,
        episodes=4,
        parallel_envs=2,
        seed=1700,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["execution"]["architecture"] == ("one_policy_batched_inference_with_async_mujoco_workers")
    assert manifest["results"]["successful_episodes"] == 2
    assert manifest["results"]["success_rate"] == 0.5
    assert [row["seed"] for row in manifest["results"]["per_episode"]] == [
        1700,
        1701,
        1702,
        1703,
    ]
    assert manifest["results"]["per_episode"][1]["action_trace"].endswith("action_traces/env_1.jsonl")
    assert manifest["results"]["per_episode"][1]["trace_episode_index"] == 0
    assert manifest["results"]["per_episode"][2]["action_trace"].endswith("action_traces/env_0.jsonl")
    assert manifest["results"]["per_episode"][2]["trace_episode_index"] == 3
    assert manifest["learning_boundary"]["optimizer_updates"] == 0
    assert manifest["learning_boundary"]["dataset_conversion_required"] is True


def test_full_vla_evidence_separates_training_from_success_threshold(tmp_path):
    teacher_root = tmp_path / "teacher"
    student_root = tmp_path / "student"
    write_ik_expert_dataset_contract(
        teacher_root,
        wrist_camera_profile_id=WRIST_CAMERA_PROFILE_ID,
        top_camera_profile_id=TOP_CAMERA_PROFILE_ID,
    )
    mark_ik_expert_dataset_verified(teacher_root, episodes=30, frames=19800)
    sidecar_path = write_wrist_student_dataset_contract(
        teacher_root,
        student_root,
        student_features=("observation.images.wrist", "observation.state", "action"),
        episodes=30,
        frames=19800,
    )
    checkpoint = tmp_path / "checkpoint"
    evaluation = tmp_path / "evaluation"
    checkpoint.mkdir()
    evaluation.mkdir()
    mark_wrist_vla_training_evaluated(
        student_root,
        checkpoint_path=checkpoint,
        evaluation_output_path=evaluation,
        training_updates=10000,
        batch_size=4,
        dataset_episodes=30,
        dataset_frames=19800,
        training_seed_start=400,
        training_seed_end=429,
        evaluation_seed_start=800,
        evaluation_episodes=10,
        evaluation_action_steps=25,
        evaluation_home_action=PICK_CLEAR_ACTION,
        evaluation_cube_xy_randomization_m=0.025,
        successful_episodes=2,
        average_max_reward=0.525,
        average_sum_reward=147.69,
    )
    evidence = json.loads(sidecar_path.read_text())
    assert evidence["claims"]["vla_trained"] is True
    assert evidence["claims"]["vla_evaluated"] is True
    assert evidence["claims"]["vla_success_threshold_met"] is False
    assert evidence["claims"]["physical_camera_alignment_verified"] is False
    assert evidence["vla_training_evaluation"]["training_samples_seen"] == 40000
    assert evidence["vla_training_evaluation"]["success_rate"] == 0.2
    assert evidence["vla_training_evaluation"]["evaluation_action_steps"] == 25
    assert evidence["vla_training_evaluation"]["evaluation_home_action"] == PICK_CLEAR_ACTION.tolist()
    assert evidence["vla_training_evaluation"]["evaluation_cube_xy_randomization_m"] == 0.025
    assert evidence["vla_training_evaluation"]["physical_rollout_executed"] is False
    with pytest.raises(ValueError, match="must not overlap"):
        mark_wrist_vla_training_evaluated(
            student_root,
            checkpoint_path=checkpoint,
            evaluation_output_path=evaluation,
            training_updates=10000,
            batch_size=4,
            dataset_episodes=30,
            dataset_frames=19800,
            training_seed_start=400,
            training_seed_end=429,
            evaluation_seed_start=420,
            evaluation_episodes=10,
            evaluation_action_steps=25,
            evaluation_home_action=PICK_CLEAR_ACTION,
            evaluation_cube_xy_randomization_m=0.025,
            successful_episodes=2,
            average_max_reward=0.525,
            average_sum_reward=147.69,
        )
    with pytest.raises(ValueError, match="positive integer"):
        mark_wrist_vla_training_evaluated(
            student_root,
            checkpoint_path=checkpoint,
            evaluation_output_path=evaluation,
            training_updates=10000,
            batch_size=4,
            dataset_episodes=30,
            dataset_frames=19800,
            training_seed_start=400,
            training_seed_end=429,
            evaluation_seed_start=800,
            evaluation_episodes=10,
            evaluation_action_steps=25.0,
            evaluation_home_action=PICK_CLEAR_ACTION,
            evaluation_cube_xy_randomization_m=0.025,
            successful_episodes=2,
            average_max_reward=0.525,
            average_sum_reward=147.69,
        )


def test_physical_wrist_gate_is_read_only_and_fail_closed(tmp_path):
    blocked = build_physical_wrist_gate_receipt(
        HardwareInventory(
            video_devices=(VideoDevice(device="/dev/video0", name="ASUS FHD webcam"),),
            serial_by_id=(),
        ),
        expected_camera_name_substrings=("SO101", "32x32"),
        camera_profile_id=WRIST_CAMERA_PROFILE_ID,
        physical_alignment_verified=False,
    )
    assert blocked["status"] == "blocked"
    assert set(blocked["blocking_reasons"]) == {
        "expected_wrist_camera_not_detected",
        "stable_robot_serial_device_not_detected",
        "wrist_camera_profile_physical_alignment_unverified",
    }
    assert blocked["claims"]["device_nodes_opened"] is False
    assert blocked["claims"]["motor_commands_sent"] is False
    assert blocked["claims"]["physical_motion_authorized"] is False
    output = write_physical_wrist_gate_receipt(tmp_path / "receipt.json", blocked)
    assert json.loads(output.read_text())["status"] == "blocked"

    ready_for_human = build_physical_wrist_gate_receipt(
        HardwareInventory(
            video_devices=(VideoDevice(device="/dev/video2", name="SO101 32x32 wrist camera"),),
            serial_by_id=("/dev/serial/by-id/usb-robot",),
        ),
        expected_camera_name_substrings=("SO101",),
        camera_profile_id=WRIST_CAMERA_PROFILE_ID,
        physical_alignment_verified=True,
    )
    assert ready_for_human["status"] == "ready_for_operator_validation"
    assert ready_for_human["claims"]["physical_motion_authorized"] is False
    assert ready_for_human["claims"]["physical_rollout_executed"] is False


def test_ik_dataset_derivation_removes_only_the_top_camera(tmp_path):
    command = build_wrist_student_dataset_command(
        python_executable="python",
        teacher_repo_id="local/so101_ik_teacher",
        teacher_root=tmp_path / "teacher",
        student_repo_id="local/so101_wrist_student",
        student_root=tmp_path / "student",
    )
    assert command[:3] == ["python", "-m", "lerobot.scripts.lerobot_edit_dataset"]
    assert "--operation.type=remove_feature" in command
    assert '--operation.feature_names=["observation.images.top"]' in command
    assert all("observation.images.wrist" not in argument for argument in command)


def test_wrist_student_trains_with_standard_smolvla_pipeline(tmp_path):
    command = build_wrist_vla_train_command(
        python_executable="python",
        dataset_repo_id="local/so101_wrist_student",
        dataset_root=tmp_path / "student",
        output_dir=tmp_path / "train",
        steps=1000,
        batch_size=8,
        seed=23,
    )
    assert command[:3] == ["python", "-m", "lerobot.scripts.lerobot_train"]
    assert "--policy.type=smolvla" in command
    assert "--policy.load_vlm_weights=true" in command
    assert "--policy.push_to_hub=false" in command
    assert "--steps=1000" in command


def test_config_exposes_real_robot_compatible_features():
    cfg = SO101MujocoEnvConfig(observation_height=120, observation_width=160)
    assert cfg.type == "so101_mujoco"
    assert cfg.features["action"].shape == (6,)
    assert cfg.features["agent_pos"].shape == (6,)
    assert cfg.features["pixels/top"].shape == (120, 160, 3)
    assert cfg.features["pixels/wrist"].shape == (120, 160, 3)
    assert cfg.features_map["pixels/top"] == "observation.images.top"
    assert cfg.features_map["pixels/wrist"] == "observation.images.wrist"
    assert cfg.camera_names == POLICY_CAMERA_NAMES
    np.testing.assert_allclose(cfg.gym_kwargs["home_action"], PICK_CLEAR_ACTION)
    assert cfg.gym_kwargs["cube_xy_randomization"] == pytest.approx(0.025)
    assert cfg.gym_kwargs["action_smoothing"] is False
    np.testing.assert_allclose(cfg.gym_kwargs["action_max_delta"], DEFAULT_VLA_ACTION_MAX_DELTA)


@pytest.mark.timeout(30)
def test_env_writes_rcs_compatible_action_trace(tmp_path):
    pytest.importorskip("mujoco")
    trace_path = tmp_path / "action_trace.jsonl"
    env = SO101MujocoEnv(
        obs_type="state",
        max_episode_steps=1,
        action_smoothing=True,
        action_chunk_steps=2,
        action_blend_steps=1,
        action_max_delta=(1, 1, 1, 1, 1, 2),
        gripper_action_deadband=0.5,
        action_trace_path=str(trace_path),
    )
    try:
        env.reset(seed=7)
        raw_action = env.home_action.copy()
        raw_action[0] += 10
        _, _, _, _, info = env.step(raw_action)
        assert info["action_filtered"] is True
        assert info["action_slew_limited_axes"][0]
        env.reset(seed=8)
        env.step(raw_action)
    finally:
        env.close()

    records = [json.loads(line) for line in trace_path.read_text().splitlines()]
    assert len(records) == 2
    record = records[0]
    assert record["contract_id"] == ACTION_TRACE_CONTRACT_ID
    assert record["schema_version"] == "dapier.so101.vla-action-trace.v1"
    assert record["episode_seed"] == 7
    assert record["joint_names"] == list(JOINT_NAMES)
    assert record["action_smoothing"] is True
    assert record["chunk_boundary"] is True
    assert len(record["command_positions_rad"]) == 6
    assert len(record["simulation_positions_rad"]) == 6
    assert len(record["cube_position_m"]) == 3
    assert len(record["gripper_position_m"]) == 3
    assert len(record["tray_position_m"]) == 3
    assert isinstance(record["finger_pad_cube_bilateral_contact"], bool)
    assert record["finger_pad_cube_max_penetration_m"] >= 0.0
    assert isinstance(record["reward"], float)
    assert record["is_success"] is False
    assert record["terminated"] is False
    assert record["truncated"] is True
    assert record["episode_done"] is True
    assert [item["trace_sample_index"] for item in records] == [1, 2]
    assert records[0]["timestamp_ns"] < records[1]["timestamp_ns"]
    assert records[0]["episode_timestamp_ns"] == records[1]["episode_timestamp_ns"]


@pytest.mark.timeout(30)
def test_env_reset_step_and_determinism():
    pytest.importorskip("mujoco")
    env = SO101MujocoEnv(obs_type="state", max_episode_steps=2)
    try:
        check_env(env, skip_render_check=True)
        obs_a, info_a = env.reset(seed=7)
        cube_a = info_a["cube_position"].copy()
        obs_b, info_b = env.reset(seed=7)
        np.testing.assert_allclose(info_b["cube_position"], cube_a)
        assert env.observation_space.contains(obs_a)
        assert env.observation_space.contains(obs_b)

        obs, reward, terminated, truncated, info = env.step(np.zeros(6, dtype=np.float32))
        assert env.observation_space.contains(obs)
        assert np.isfinite(reward)
        assert isinstance(terminated, bool)
        assert truncated is False
        assert info["action_clipped"] is False
    finally:
        env.close()


@pytest.mark.timeout(30)
def test_reachable_scene_contains_support_goal_and_finger_pads():
    mujoco = pytest.importorskip("mujoco")
    env = SO101MujocoEnv(obs_type="state", cube_xy_randomization=0)
    try:
        _, info = env.reset(seed=101)
        np.testing.assert_allclose(info["cube_position"], CUBE_SPAWN_POSITION, atol=1e-7)
        assert info["cube_position"][0] > 0
        assert np.linalg.norm(info["cube_position"][:2] - info["tray_position"][:2]) > 0.1
        for name in (
            *FINGER_PAD_GEOM_NAMES,
            "tray_floor",
            "goal_tray_floor",
        ):
            assert mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0
        expected_pads = {
            FINGER_PAD_GEOM_NAMES[0]: (
                [-0.0109, -0.0002221, -0.097517],
                [0.012, 0.008, 0.003],
            ),
            FINGER_PAD_GEOM_NAMES[1]: (
                [-0.0093, -0.0750583, 0.0188972],
                [0.008, 0.012, 0.003],
            ),
        }
        for name, (expected_pos, expected_size) in expected_pads.items():
            geom_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            assert env.model.geom_group[geom_id] == 2
            assert env.model.geom_contype[geom_id] == 1
            assert env.model.geom_conaffinity[geom_id] == 1
            assert env.model.geom_rgba[geom_id, 3] == 1
            np.testing.assert_allclose(env.model.geom_pos[geom_id], expected_pos)
            np.testing.assert_allclose(env.model.geom_size[geom_id], expected_size)
            pair_id = mujoco.mj_name2id(
                env.model,
                mujoco.mjtObj.mjOBJ_PAIR,
                f"{name}_cube_contact",
            )
            assert pair_id >= 0
            np.testing.assert_allclose(env.model.pair_friction[pair_id], FINGER_PAD_CUBE_CONTACT_FRICTION)
            np.testing.assert_allclose(env.model.pair_solref[pair_id], FINGER_PAD_CUBE_CONTACT_SOLREF)
        for wall_name in (
            "tray_wall_left",
            "tray_wall_right",
            "tray_wall_near",
            "tray_wall_far",
        ):
            wall_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, wall_name)
            assert env.model.geom_pos[wall_id, 2] == pytest.approx(0.006)
            assert env.model.geom_size[wall_id, 2] == pytest.approx(0.006)

        for fingertip_body_name in ("gripper", "moving_jaw_so101_v1"):
            body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, fingertip_body_name)
            mesh_ids = np.flatnonzero(
                (env.model.geom_bodyid == body_id) & (env.model.geom_type == mujoco.mjtGeom.mjGEOM_MESH)
            )
            assert len(mesh_ids) > 0
            assert np.all(env.model.geom_contype[mesh_ids] == 0)
            assert np.all(env.model.geom_conaffinity[mesh_ids] == 0)

        camera_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
        top_camera_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "top")
        gripper_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        housing_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, WRIST_CAMERA_HOUSING_GEOM_NAME)
        lens_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, WRIST_CAMERA_LENS_GEOM_NAME)
        mount_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, WRIST_CAMERA_MOUNT_GEOM_NAME)
        assert env.model.cam_bodyid[camera_id] == gripper_id
        assert env.model.cam_bodyid[top_camera_id] == 0
        wrist_profile = camera_profile("wrist")
        top_profile = camera_profile("top")
        np.testing.assert_allclose(env.model.cam_pos[camera_id], wrist_profile.position_m)
        np.testing.assert_allclose(env.model.cam_pos[top_camera_id], top_profile.position_m)
        wrist_calibration = env.camera_calibration("wrist")
        top_calibration = env.camera_calibration("top")
        assert wrist_calibration.profile_id == WRIST_CAMERA_PROFILE_ID
        assert top_calibration.profile_id == TOP_CAMERA_PROFILE_ID
        assert wrist_calibration.physical_alignment_verified is False
        assert top_calibration.physical_alignment_verified is False
        assert housing_id >= 0
        assert lens_id >= 0
        assert mount_id >= 0
        assert env.model.geom_contype[housing_id] == 0
        assert env.model.geom_contype[lens_id] == 0
        assert env.model.geom_contype[mount_id] == 0
    finally:
        env.close()


@pytest.mark.timeout(30)
def test_cartesian_jog_moves_gripper_in_world_xyz():
    pytest.importorskip("mujoco")
    env = SO101MujocoEnv(obs_type="state", cube_xy_randomization=0)
    try:
        env.reset(seed=0)
        controller = CartesianJogController(env.model)
        start = controller.site_position()
        action = controller.move([0.01, 0.01, 0.01])
        np.testing.assert_allclose(controller.site_position(), start + 0.01, atol=1e-3)
        assert np.all(action >= ACTION_LOW)
        assert np.all(action <= ACTION_HIGH)
    finally:
        env.close()


@pytest.mark.timeout(30)
def test_cartesian_pose_move_preserves_gripper_orientation():
    pytest.importorskip("mujoco")
    env = SO101MujocoEnv(obs_type="state", cube_xy_randomization=0)
    try:
        env.reset(seed=0)
        controller = CartesianJogController(env.model, max_iterations=200)
        controller.set_action(np.array([0, -12.78, 19.83, 69.84, 0, 100], dtype=np.float32))
        start_position = controller.site_position()
        start_rotation = controller.site_rotation()
        controller.move_preserving_orientation([0, 0, 0.05])
        np.testing.assert_allclose(controller.site_position(), start_position + [0, 0, 0.05], atol=1e-3)
        np.testing.assert_allclose(controller.site_rotation(), start_rotation, atol=3e-3)
        assert controller.last_orientation_error_rad < 0.005
    finally:
        env.close()


def test_scripted_pick_lift_trace_is_bounded_and_deterministic():
    first = [scripted_pick_lift_action(index) for index in range(PICK_LIFT_FRAMES)]
    second = [scripted_pick_lift_action(index) for index in range(PICK_LIFT_FRAMES)]
    np.testing.assert_array_equal(first, second)
    assert len(first) == 390
    assert all(np.all(action >= ACTION_LOW) and np.all(action <= ACTION_HIGH) for action in first)
    np.testing.assert_array_equal(first[0], first[29])
    np.testing.assert_array_equal(first[360], first[389])


@pytest.mark.timeout(30)
def test_scripted_pick_lifts_and_holds_cube_with_bilateral_contact():
    mujoco = pytest.importorskip("mujoco")
    env = SO101MujocoEnv(
        obs_type="state",
        cube_xy_randomization=0,
        max_episode_steps=PICK_LIFT_FRAMES,
        terminate_on_success=False,
        home_action=PICK_CLEAR_ACTION,
    )
    try:
        env.reset(seed=101)
        cube_geom_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        pad_geom_ids = {
            mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in FINGER_PAD_GEOM_NAMES
        }
        support_geom_ids = {
            mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in (
                "workbench",
                "tray_floor",
                "tray_wall_left",
                "tray_wall_right",
                "tray_wall_near",
                "tray_wall_far",
            )
        }
        cube_z: list[float] = []
        bilateral_contact: list[bool] = []
        support_contact: list[bool] = []
        for frame_index in range(PICK_LIFT_FRAMES):
            _, _, _, _, info = env.step(scripted_pick_lift_action(frame_index))
            cube_z.append(float(info["cube_position"][2]))
            other_geoms: set[int] = set()
            for contact_index in range(env.data.ncon):
                contact = env.data.contact[contact_index]
                geom1, geom2 = int(contact.geom1), int(contact.geom2)
                if geom1 == cube_geom_id:
                    other_geoms.add(geom2)
                elif geom2 == cube_geom_id:
                    other_geoms.add(geom1)
            bilateral_contact.append(pad_geom_ids <= other_geoms)
            support_contact.append(bool(support_geom_ids & other_geoms))

        settled_z = float(np.median(cube_z[20:30]))
        hold_lift = np.asarray(cube_z[360:390]) - settled_z
        assert env._simulated_substeps == 6500
        assert env.data.time == pytest.approx(13.0, abs=1e-9)
        assert hold_lift.min() >= 0.02
        assert all(bilateral_contact[360:390])
        assert not any(support_contact[360:390])
    finally:
        env.close()


@pytest.mark.timeout(30)
def test_headless_dual_camera_render():
    os.environ.setdefault("MUJOCO_GL", "egl")
    pytest.importorskip("mujoco")
    env = SO101MujocoEnv(
        obs_type="pixels",
        observation_height=96,
        observation_width=128,
        camera_names=POLICY_CAMERA_NAMES,
    )
    try:
        obs, _ = env.reset(seed=1)
        assert obs["pixels"]["top"].shape == (96, 128, 3)
        assert obs["pixels"]["top"].dtype == np.uint8
        assert obs["pixels"]["top"].max() > obs["pixels"]["top"].min()
        assert obs["pixels"]["wrist"].shape == (96, 128, 3)
        assert obs["pixels"]["wrist"].dtype == np.uint8
        assert obs["pixels"]["wrist"].max() > obs["pixels"]["wrist"].min()
        assert not np.array_equal(obs["pixels"]["top"], obs["pixels"]["wrist"])
        policy_obs = preprocess_observation(obs)
        assert policy_obs["observation.images.top"].shape == (1, 3, 96, 128)
        assert policy_obs["observation.images.wrist"].shape == (1, 3, 96, 128)
    finally:
        env.close()


def test_camera_configuration_validation():
    with pytest.raises(ValueError, match="at least one"):
        SO101MujocoEnv(camera_names=())
    with pytest.raises(ValueError, match="duplicates"):
        SO101MujocoEnv(camera_names=("front", "front"))
    with pytest.raises(ValueError, match="Unsupported"):
        SO101MujocoEnv(camera_names=("overhead",))


def test_rgb_detector_and_pinhole_plane_projection_are_auditable():
    image = np.zeros((100, 120, 3), dtype=np.uint8)
    image[40:61, 50:71] = [40, 180, 250]
    detection = detect_blue_cube(image)
    assert detection.center_pixel_xy == (60.0, 50.0)
    assert detection.bbox_xyxy == (50, 40, 70, 60)
    assert detection.pixel_count == 21 * 21

    calibration = CameraCalibration(
        name="test",
        position=np.array([0.0, 0.0, 1.0]),
        rotation=np.eye(3),
        vertical_fov_degrees=90.0,
        image_height=101,
        image_width=101,
    )
    intersection = project_pixel_to_horizontal_plane((50.0, 50.0), calibration, plane_z_m=0.0)
    np.testing.assert_allclose(intersection, [0.0, 0.0, 0.0], atol=1e-12)
    with pytest.raises(RuntimeError, match="not found"):
        detect_blue_cube(np.zeros((32, 32, 3), dtype=np.uint8))


@pytest.mark.timeout(30)
def test_cad_mount_surface_wrist_view_moves_with_gripper_and_sees_cube():
    os.environ.setdefault("MUJOCO_GL", "egl")
    pytest.importorskip("mujoco")
    env = SO101MujocoEnv(
        obs_type="state",
        camera_names=("wrist",),
        observation_height=240,
        observation_width=320,
        cube_xy_randomization=0.025,
        terminate_on_success=False,
        home_action=PICK_CLEAR_ACTION,
    )
    try:
        _, info = env.reset(seed=4)
        start_position = env.camera_calibration("wrist").position
        for _ in range(30):
            _, _, _, _, info = env.step(PICK_CLEAR_ACTION)
        estimate = estimate_blue_cube_world_position(env.render("wrist"), env.camera_calibration("wrist"))
        assert np.linalg.norm(estimate.world_xyz[:2] - info["cube_position"][:2]) < 0.003
        for _ in range(15):
            env.step(IK_OBSERVE_ACTION)
        moved_position = env.camera_calibration("wrist").position
        assert np.linalg.norm(moved_position - start_position) > 0.01
    finally:
        env.close()


@pytest.mark.timeout(30)
def test_top_rgb_estimate_drives_a_bounded_ik_plan_without_cube_state():
    os.environ.setdefault("MUJOCO_GL", "egl")
    pytest.importorskip("mujoco")
    env = SO101MujocoEnv(
        obs_type="state",
        camera_names=("top", "wrist"),
        observation_height=240,
        observation_width=320,
        cube_xy_randomization=0.025,
        terminate_on_success=False,
        home_action=PICK_CLEAR_ACTION,
    )
    try:
        _, info = env.reset(seed=3)
        for _ in range(30):
            _, _, _, _, info = env.step(IK_OBSERVE_ACTION)
        estimate = estimate_blue_cube_world_position(env.render("top"), env.camera_calibration("top"))
        assert estimate.world_xyz[2] == pytest.approx(CUBE_TOP_PLANE_Z_M)
        assert np.linalg.norm(estimate.world_xyz[:2] - info["cube_position"][:2]) < 0.003

        plan = build_vision_pick_place_plan(env.model, estimate.world_xyz[:2])
        assert plan.actions.shape == (VISION_PICK_PLACE_FRAMES, 6)
        assert np.all(plan.actions >= ACTION_LOW)
        assert np.all(plan.actions <= ACTION_HIGH)
        assert plan.approach_action[5] == 100
        assert plan.lift_action[5] == VISION_GRASP_CLOSE_PERCENT
        assert pytest.approx(-0.015) == VISION_GRASP_Z_OFFSET_M
        shifted = build_vision_pick_place_plan(env.model, estimate.world_xyz[:2] + np.array([0.005, 0.0]))
        assert not np.array_equal(plan.approach_action, shifted.approach_action)
        with pytest.raises(ValueError, match="outside the verified pick workspace"):
            build_vision_pick_place_plan(env.model, CUBE_SPAWN_POSITION[:2] + 0.1)
    finally:
        env.close()


@pytest.mark.timeout(60)
@pytest.mark.parametrize("seed", range(5))
def test_top_rgb_ik_expert_places_randomized_cube(seed):
    os.environ.setdefault("MUJOCO_GL", "egl")
    pytest.importorskip("mujoco")
    env = SO101MujocoEnv(
        obs_type="state",
        camera_names=("top", "wrist"),
        observation_height=240,
        observation_width=320,
        cube_xy_randomization=0.025,
        max_episode_steps=700,
        terminate_on_success=False,
        home_action=PICK_CLEAR_ACTION,
    )
    try:
        env.reset(seed=seed)
        for _ in range(30):
            env.step(IK_OBSERVE_ACTION)
        estimate = estimate_blue_cube_world_position(env.render("top"), env.camera_calibration("top"))
        plan = build_vision_pick_place_plan(env.model, estimate.world_xyz[:2], goal_xy=GOAL_TRAY_POSITION[:2])
        info = None
        max_penetration_m = 0.0
        bilateral_contact_frames = 0
        for action in plan.actions:
            _, _, _, _, info = env.step(action)
            max_penetration_m = max(
                max_penetration_m,
                float(info["finger_pad_cube_max_penetration_m"]),
            )
            bilateral_contact_frames += int(info["finger_pad_cube_bilateral_contact"])
        assert info is not None
        assert info["is_success"] is True
        assert np.all(np.abs(info["cube_position"][:2] - GOAL_TRAY_POSITION[:2]) < 0.05)
        assert bilateral_contact_frames > 0
        assert max_penetration_m <= VISION_MAX_PAD_PENETRATION_M
    finally:
        env.close()


@pytest.mark.timeout(30)
def test_config_builds_vector_env():
    pytest.importorskip("mujoco")
    cfg = SO101MujocoEnvConfig(obs_type="state", episode_length=2)
    envs = cfg.create_envs(n_envs=1)
    vector_env = envs["so101_mujoco"][0]
    try:
        obs, _ = vector_env.reset(seed=3)
        assert obs["agent_pos"].shape == (1, 6)
        action = np.zeros((1, 6), dtype=np.float32)
        _, reward, _, _, _ = vector_env.step(action)
        assert reward.shape == (1,)
    finally:
        vector_env.close()
