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
    CUBE_SPAWN_POSITION,
    CUBE_TOP_PLANE_Z_M,
    FINGER_PAD_GEOM_NAMES,
    FINGER_PAD_VISUAL_GEOM_NAMES,
    GOAL_TRAY_POSITION,
    JOINT_NAMES,
    PICK_CLEAR_ACTION,
    PICK_LIFT_FRAMES,
    VISION_PICK_PLACE_FRAMES,
    WRIST_CAMERA_HOUSING_GEOM_NAME,
    WRIST_CAMERA_LENS_GEOM_NAME,
    WRIST_CAMERA_MOUNT_GEOM_NAME,
    CameraCalibration,
    CartesianJogController,
    JointJogController,
    SO101MujocoEnv,
    build_vision_pick_place_plan,
    detect_blue_cube,
    estimate_blue_cube_world_position,
    leader_action_dict_to_array,
    lerobot_action_to_qpos,
    project_pixel_to_horizontal_plane,
    qpos_to_lerobot_state,
    scripted_pick_lift_action,
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
            *FINGER_PAD_VISUAL_GEOM_NAMES,
            "tray_floor",
            "goal_tray_floor",
        ):
            assert mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0
        for name in FINGER_PAD_GEOM_NAMES:
            geom_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            assert env.model.geom_group[geom_id] == 3
            assert env.model.geom_rgba[geom_id, 3] == 0
            np.testing.assert_allclose(env.model.geom_size[geom_id], [0.0275, 0.02, 0.002])
        for name in FINGER_PAD_VISUAL_GEOM_NAMES:
            geom_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            assert env.model.geom_group[geom_id] == 2
            assert env.model.geom_contype[geom_id] == 0
            np.testing.assert_allclose(env.model.geom_size[geom_id], [0.018, 0.0065, 0.0021])

        camera_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
        gripper_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        housing_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, WRIST_CAMERA_HOUSING_GEOM_NAME)
        lens_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, WRIST_CAMERA_LENS_GEOM_NAME)
        mount_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, WRIST_CAMERA_MOUNT_GEOM_NAME)
        assert env.model.cam_bodyid[camera_id] == gripper_id
        np.testing.assert_allclose(env.model.cam_pos[camera_id], [0.05, -0.07, 0.04])
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


def test_scripted_pick_lift_trace_is_bounded_and_deterministic():
    first = [scripted_pick_lift_action(index) for index in range(PICK_LIFT_FRAMES)]
    second = [scripted_pick_lift_action(index) for index in range(PICK_LIFT_FRAMES)]
    np.testing.assert_array_equal(first, second)
    assert len(first) == 300
    assert all(np.all(action >= ACTION_LOW) and np.all(action <= ACTION_HIGH) for action in first)
    np.testing.assert_array_equal(first[0], first[29])
    np.testing.assert_array_equal(first[260], first[299])


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
        hold_lift = np.asarray(cube_z[270:300]) - settled_z
        assert env._simulated_substeps == 5000
        assert env.data.time == pytest.approx(10.0, abs=1e-9)
        assert hold_lift.min() >= 0.02
        assert all(bilateral_contact[270:300])
        assert not any(support_contact[270:300])
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
def test_wrist_rgb_estimate_drives_a_bounded_plan_without_cube_state():
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
        _, info = env.reset(seed=3)
        for _ in range(30):
            _, _, _, _, info = env.step(PICK_CLEAR_ACTION)
        estimate = estimate_blue_cube_world_position(env.render("wrist"), env.camera_calibration("wrist"))
        assert estimate.world_xyz[2] == pytest.approx(CUBE_TOP_PLANE_Z_M)
        assert np.linalg.norm(estimate.world_xyz[:2] - info["cube_position"][:2]) < 0.012

        plan = build_vision_pick_place_plan(env.model, estimate.world_xyz[:2])
        assert plan.actions.shape == (VISION_PICK_PLACE_FRAMES, 6)
        assert np.all(plan.actions >= ACTION_LOW)
        assert np.all(plan.actions <= ACTION_HIGH)
        shifted = build_vision_pick_place_plan(env.model, estimate.world_xyz[:2] + np.array([0.005, 0.0]))
        assert not np.array_equal(plan.approach_action, shifted.approach_action)
        with pytest.raises(ValueError, match="outside the verified pick workspace"):
            build_vision_pick_place_plan(env.model, CUBE_SPAWN_POSITION[:2] + 0.1)
    finally:
        env.close()


@pytest.mark.timeout(60)
@pytest.mark.parametrize("seed", range(5))
def test_wrist_rgb_closed_loop_places_randomized_cube(seed):
    os.environ.setdefault("MUJOCO_GL", "egl")
    pytest.importorskip("mujoco")
    env = SO101MujocoEnv(
        obs_type="state",
        camera_names=("wrist",),
        observation_height=240,
        observation_width=320,
        cube_xy_randomization=0.025,
        max_episode_steps=500,
        terminate_on_success=False,
        home_action=PICK_CLEAR_ACTION,
    )
    try:
        env.reset(seed=seed)
        for _ in range(30):
            env.step(PICK_CLEAR_ACTION)
        estimate = estimate_blue_cube_world_position(env.render("wrist"), env.camera_calibration("wrist"))
        plan = build_vision_pick_place_plan(env.model, estimate.world_xyz[:2], goal_xy=GOAL_TRAY_POSITION[:2])
        info = None
        for action in plan.actions:
            _, _, _, _, info = env.step(action)
        assert info is not None
        assert info["is_success"] is True
        assert np.linalg.norm(info["cube_position"][:2] - GOAL_TRAY_POSITION[:2]) < 0.055
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
