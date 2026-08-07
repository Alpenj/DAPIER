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

import os

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from lerobot.envs.configs import SO101MujocoEnvConfig
from lerobot.envs.so101_mujoco import (
    ACTION_HIGH,
    ACTION_LOW,
    CAMERA_NAMES,
    JOINT_NAMES,
    JointJogController,
    SO101MujocoEnv,
    leader_action_dict_to_array,
    lerobot_action_to_qpos,
    qpos_to_lerobot_state,
    should_save_episode,
)


def test_joint_contract_and_round_trip():
    assert JOINT_NAMES == (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    )
    for action in (ACTION_LOW, ACTION_HIGH, np.array([0, -35, 55, 35, 0, 50], dtype=np.float32)):
        np.testing.assert_allclose(qpos_to_lerobot_state(lerobot_action_to_qpos(action)), action, atol=1e-4)


def test_joint_contract_rejects_wrong_shapes():
    with pytest.raises(ValueError, match="shape \\(6,\\)"):
        lerobot_action_to_qpos(np.zeros(5))
    with pytest.raises(ValueError, match="shape \\(6,\\)"):
        qpos_to_lerobot_state(np.zeros(7))


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


def test_config_exposes_real_robot_compatible_features():
    cfg = SO101MujocoEnvConfig(observation_height=120, observation_width=160)
    assert cfg.type == "so101_mujoco"
    assert cfg.features["action"].shape == (6,)
    assert cfg.features["agent_pos"].shape == (6,)
    assert cfg.features["pixels"].shape == (120, 160, 3)
    assert cfg.features["pixels_wrist"].shape == (120, 160, 3)
    assert cfg.features_map["pixels"] == "observation.images.front"
    assert cfg.features_map["pixels_wrist"] == "observation.images.wrist"
    assert cfg.camera_names == CAMERA_NAMES


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
def test_headless_dual_camera_render():
    os.environ.setdefault("MUJOCO_GL", "egl")
    pytest.importorskip("mujoco")
    env = SO101MujocoEnv(
        obs_type="pixels",
        observation_height=96,
        observation_width=128,
        camera_names=CAMERA_NAMES,
    )
    try:
        obs, _ = env.reset(seed=1)
        assert obs["pixels"].shape == (96, 128, 3)
        assert obs["pixels"].dtype == np.uint8
        assert obs["pixels"].max() > obs["pixels"].min()
        assert obs["pixels_wrist"].shape == (96, 128, 3)
        assert obs["pixels_wrist"].dtype == np.uint8
        assert obs["pixels_wrist"].max() > obs["pixels_wrist"].min()
        assert not np.array_equal(obs["pixels"], obs["pixels_wrist"])
    finally:
        env.close()


def test_camera_configuration_validation():
    with pytest.raises(ValueError, match="at least one"):
        SO101MujocoEnv(camera_names=())
    with pytest.raises(ValueError, match="duplicates"):
        SO101MujocoEnv(camera_names=("front", "front"))
    with pytest.raises(ValueError, match="Unsupported"):
        SO101MujocoEnv(camera_names=("overhead",))


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
