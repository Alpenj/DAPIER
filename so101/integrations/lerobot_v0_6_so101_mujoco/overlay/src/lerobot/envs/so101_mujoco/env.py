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

"""A small, leader-free SO-101 MuJoCo environment.

The public action/state contract intentionally matches LeRobot's calibrated
SO-101 convention:

    [shoulder_pan_deg, shoulder_lift_deg, elbow_flex_deg,
     wrist_flex_deg, wrist_roll_deg, gripper_percent]

The first five values are degrees. The gripper is 0 (closed) to 100 (open).
Only the adapter in this module deals with MuJoCo radians.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

CAMERA_NAMES = ("front", "wrist")
CAMERA_OBSERVATION_KEYS = {
    "front": "pixels",
    "wrist": "pixels_wrist",
}

# Limits are taken from the upstream SO-101 new-calibration MJCF.
_JOINT_LOW_RAD = np.array(
    [-1.9198621772, -1.7453292520, -1.69, -1.6580628495, -2.7438472970, -0.1745329776],
    dtype=np.float64,
)
_JOINT_HIGH_RAD = np.array(
    [1.9198621772, 1.7453292520, 1.69, 1.6580627293, 2.8412063094, 1.7453291996],
    dtype=np.float64,
)

ACTION_LOW = np.concatenate((np.rad2deg(_JOINT_LOW_RAD[:5]), np.array([0.0]))).astype(np.float32)
ACTION_HIGH = np.concatenate((np.rad2deg(_JOINT_HIGH_RAD[:5]), np.array([100.0]))).astype(np.float32)
DEFAULT_HOME_ACTION = np.array([0.0, -35.0, 55.0, 35.0, 0.0, 100.0], dtype=np.float32)
CUBE_SPAWN_POSITION = np.array([0.25453126220736555, -0.002930872758779989, 0.075], dtype=np.float64)
GOAL_TRAY_POSITION = np.array([0.20, 0.18, 0.031], dtype=np.float64)
CUBE_HALF_SIZE_M = 0.025
CUBE_SETTLED_CENTER_Z_M = 0.06881588
CUBE_TOP_PLANE_Z_M = CUBE_SETTLED_CENTER_Z_M + CUBE_HALF_SIZE_M
FINGER_PAD_GEOM_NAMES = ("dapier_fixed_finger_pad", "dapier_moving_finger_pad")
FINGER_PAD_VISUAL_GEOM_NAMES = (
    "dapier_fixed_finger_pad_visual",
    "dapier_moving_finger_pad_visual",
)
WRIST_CAMERA_HOUSING_GEOM_NAME = "dapier_wrist_camera_housing"
WRIST_CAMERA_LENS_GEOM_NAME = "dapier_wrist_camera_lens"
WRIST_CAMERA_MOUNT_GEOM_NAME = "dapier_wrist_camera_mount"
_WRIST_CAMERA_POSITION = [0.05, -0.07, 0.04]
_WRIST_CAMERA_QUATERNION = [
    0.8976243763874222,
    0.0994776440004103,
    0.1271154174228951,
    -0.4101418631554479,
]
_WRIST_CAMERA_HOUSING_POSITION = [
    0.051612642922897835,
    -0.07311143607055529,
    0.05042680911794568,
]
_FINGER_PAD_SPECS = (
    {
        "body": "gripper",
        "name": FINGER_PAD_GEOM_NAMES[0],
        "visual_name": FINGER_PAD_VISUAL_GEOM_NAMES[0],
        "pos": [0.0251, -0.000218121, -0.0831274],
        "quat": [0.707107, 0.0, 0.707107, 0.0],
    },
    {
        "body": "moving_jaw_so101_v1",
        "name": FINGER_PAD_GEOM_NAMES[1],
        "visual_name": FINGER_PAD_VISUAL_GEOM_NAMES[1],
        "pos": [-0.0165797936, -0.0822294661, 0.0190181],
        "quat": [-0.206738, 0.206738, -0.67621, 0.67621],
    },
)


@dataclass(frozen=True)
class CameraCalibration:
    """Current calibrated pinhole pose for one MuJoCo RGB camera."""

    name: str
    position: np.ndarray
    rotation: np.ndarray
    vertical_fov_degrees: float
    image_height: int
    image_width: int


def lerobot_action_to_qpos(action: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    """Convert the LeRobot SO-101 six-value convention to MuJoCo radians."""
    action_array = np.asarray(action, dtype=np.float64)
    if action_array.shape != (6,):
        raise ValueError(f"Expected an SO-101 action with shape (6,), got {action_array.shape}")

    clipped = np.clip(action_array, ACTION_LOW, ACTION_HIGH)
    qpos = np.empty(6, dtype=np.float64)
    qpos[:5] = np.deg2rad(clipped[:5])
    qpos[5] = _JOINT_LOW_RAD[5] + clipped[5] / 100.0 * (_JOINT_HIGH_RAD[5] - _JOINT_LOW_RAD[5])
    return qpos


def qpos_to_lerobot_state(qpos: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    """Convert the six MuJoCo joint values to LeRobot degrees + gripper percent."""
    qpos_array = np.asarray(qpos, dtype=np.float64)
    if qpos_array.shape != (6,):
        raise ValueError(f"Expected SO-101 joint positions with shape (6,), got {qpos_array.shape}")

    state = np.empty(6, dtype=np.float32)
    state[:5] = np.rad2deg(qpos_array[:5])
    state[5] = (qpos_array[5] - _JOINT_LOW_RAD[5]) / (_JOINT_HIGH_RAD[5] - _JOINT_LOW_RAD[5]) * 100.0
    return np.clip(state, ACTION_LOW, ACTION_HIGH).astype(np.float32)


def _require_mujoco():
    try:
        import mujoco
    except ImportError as exc:
        raise ImportError(
            "SO101MujocoEnv requires MuJoCo. Install it with "
            "`pip install 'lerobot[so101_mujoco]'` or `uv sync --extra so101_mujoco`."
        ) from exc
    return mujoco


class SO101MujocoEnv(gym.Env):
    """Joint-position SO-101 pick-and-place environment."""

    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(
        self,
        *,
        task: str = "PickCube-v0",
        obs_type: str = "pixels_agent_pos",
        render_mode: str | None = "rgb_array",
        observation_height: int = 480,
        observation_width: int = 640,
        fps: int = 30,
        max_episode_steps: int = 300,
        reward_type: str = "dense",
        terminate_on_success: bool = True,
        cube_xy_randomization: float = 0.04,
        camera_names: tuple[str, ...] | list[str] = ("front",),
        home_action: tuple[float, ...] | list[float] | np.ndarray = DEFAULT_HOME_ACTION,
    ):
        super().__init__()
        if task != "PickCube-v0":
            raise ValueError(f"Unsupported SO-101 MuJoCo task: {task!r}")
        if obs_type not in {"state", "pixels", "pixels_agent_pos"}:
            raise ValueError(f"Unsupported obs_type: {obs_type!r}")
        if render_mode not in {None, "rgb_array"}:
            raise ValueError(f"Unsupported render_mode: {render_mode!r}")
        if reward_type not in {"dense", "sparse"}:
            raise ValueError(f"Unsupported reward_type: {reward_type!r}")
        if fps <= 0 or max_episode_steps <= 0:
            raise ValueError("fps and max_episode_steps must be positive")
        if cube_xy_randomization < 0:
            raise ValueError("cube_xy_randomization must be non-negative")
        camera_names = tuple(camera_names)
        if not camera_names:
            raise ValueError("camera_names must contain at least one camera")
        if len(camera_names) != len(set(camera_names)):
            raise ValueError(f"camera_names contains duplicates: {camera_names}")
        unknown_cameras = set(camera_names) - set(CAMERA_NAMES)
        if unknown_cameras:
            raise ValueError(f"Unsupported camera_names: {sorted(unknown_cameras)}")

        self.task = task
        self.obs_type = obs_type
        self.render_mode = render_mode
        self.observation_height = observation_height
        self.observation_width = observation_width
        self.fps = fps
        self.max_episode_steps = max_episode_steps
        self._max_episode_steps = max_episode_steps
        self.task_description = "Pick up the blue cube and place it in the green tray."
        self.reward_type = reward_type
        self.terminate_on_success = terminate_on_success
        self.cube_xy_randomization = cube_xy_randomization
        self.camera_names = camera_names
        self.home_action = np.clip(np.asarray(home_action, dtype=np.float32), ACTION_LOW, ACTION_HIGH)
        if self.home_action.shape != (6,):
            raise ValueError(f"home_action must have shape (6,), got {self.home_action.shape}")

        self.metadata = {**self.metadata, "render_fps": fps}
        self.action_space = gym.spaces.Box(low=ACTION_LOW, high=ACTION_HIGH, dtype=np.float32)
        observation_spaces: dict[str, gym.Space] = {}
        if obs_type in {"state", "pixels_agent_pos"}:
            observation_spaces["agent_pos"] = gym.spaces.Box(
                low=ACTION_LOW, high=ACTION_HIGH, dtype=np.float32
            )
        if obs_type in {"pixels", "pixels_agent_pos"}:
            for camera_name in self.camera_names:
                observation_spaces[CAMERA_OBSERVATION_KEYS[camera_name]] = gym.spaces.Box(
                    low=0,
                    high=255,
                    shape=(observation_height, observation_width, 3),
                    dtype=np.uint8,
                )
        self.observation_space = gym.spaces.Dict(observation_spaces)

        self._mujoco: Any | None = None
        self.model: Any | None = None
        self.data: Any | None = None
        self._renderer: Any | None = None
        self._joint_qpos_addresses: np.ndarray | None = None
        self._actuator_ids: np.ndarray | None = None
        self._cube_qpos_address: int | None = None
        self._cube_body_id: int | None = None
        self._tray_site_id: int | None = None
        self._gripper_site_id: int | None = None
        self._simulated_substeps = 0
        self._step_count = 0

    @property
    def model_path(self) -> Path:
        return Path(__file__).resolve().parent / "assets" / "pick_cube.xml"

    def _load_model(self) -> None:
        if self.model is not None:
            return

        mujoco = _require_mujoco()
        self._mujoco = mujoco
        model_spec = mujoco.MjSpec.from_file(str(self.model_path))
        for pad in _FINGER_PAD_SPECS:
            # The larger transparent contact envelope keeps the cube seated
            # during transfer. A smaller, opaque rubber lining below shows the
            # surface the operator should visually treat as the fingertip.
            model_spec.body(pad["body"]).add_geom(
                name=pad["name"],
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=pad["pos"],
                quat=pad["quat"],
                size=[0.0275, 0.02, 0.002],
                contype=1,
                conaffinity=1,
                friction=[2.0, 0.01, 0.001],
                rgba=[0.08, 0.08, 0.08, 0.0],
                group=3,
                density=0,
            )
            model_spec.body(pad["body"]).add_geom(
                name=pad["visual_name"],
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=pad["pos"],
                quat=pad["quat"],
                size=[0.018, 0.0065, 0.0021],
                contype=0,
                conaffinity=0,
                rgba=[0.07, 0.07, 0.07, 1.0],
                group=2,
                density=0,
            )

        # Put the eye-in-hand camera above and slightly beside the fingers.
        # A dead-centre overhead camera is physically occluded by the fixed
        # finger, so this small lateral offset preserves the top-down view.
        wrist_camera = model_spec.camera("wrist")
        wrist_camera.pos = _WRIST_CAMERA_POSITION
        wrist_camera.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
        wrist_camera.quat = _WRIST_CAMERA_QUATERNION
        model_spec.body("gripper").add_geom(
            name=WRIST_CAMERA_MOUNT_GEOM_NAME,
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[0.008, -0.018, 0.0, 0.0515, -0.073, 0.0505],
            size=[0.0035, 0.0, 0.0],
            contype=0,
            conaffinity=0,
            rgba=[0.7, 0.55, 0.08, 1.0],
            group=2,
            density=0,
        )
        model_spec.body("gripper").add_geom(
            name=WRIST_CAMERA_HOUSING_GEOM_NAME,
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=_WRIST_CAMERA_HOUSING_POSITION,
            quat=_WRIST_CAMERA_QUATERNION,
            size=[0.014, 0.01, 0.008],
            contype=0,
            conaffinity=0,
            rgba=[0.025, 0.025, 0.025, 1.0],
            group=2,
            density=0,
        )
        model_spec.body("gripper").add_geom(
            name=WRIST_CAMERA_LENS_GEOM_NAME,
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            pos=[0.050219905853122436, -0.07042428673689391, 0.04142183760699259],
            quat=_WRIST_CAMERA_QUATERNION,
            size=[0.005, 0.001, 0.0],
            contype=0,
            conaffinity=0,
            rgba=[0.04, 0.13, 0.18, 1.0],
            group=2,
            density=0,
        )
        self.model = model_spec.compile()
        self.data = mujoco.MjData(self.model)

        joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES]
        actuator_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in JOINT_NAMES
        ]
        if min(joint_ids) < 0 or min(actuator_ids) < 0:
            raise RuntimeError("The SO-101 MJCF is missing a required joint or actuator")
        self._joint_qpos_addresses = self.model.jnt_qposadr[joint_ids].astype(np.int32)
        self._actuator_ids = np.asarray(actuator_ids, dtype=np.int32)

        cube_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
        self._cube_qpos_address = int(self.model.jnt_qposadr[cube_joint_id])
        self._cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self._tray_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "tray_target")
        self._gripper_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")

    def _substeps_for_next_frame(self) -> int:
        """Keep the requested FPS without accumulating timestep drift."""
        assert self.model is not None
        target_total = int((self._step_count + 1) / (self.fps * self.model.opt.timestep) + 1e-9)
        return max(1, target_total - self._simulated_substeps)

    def _joint_qpos(self) -> np.ndarray:
        assert self.data is not None and self._joint_qpos_addresses is not None
        return self.data.qpos[self._joint_qpos_addresses]

    def _get_observation(self) -> dict[str, np.ndarray]:
        observation: dict[str, np.ndarray] = {}
        if self.obs_type in {"state", "pixels_agent_pos"}:
            observation["agent_pos"] = qpos_to_lerobot_state(self._joint_qpos())
        if self.obs_type in {"pixels", "pixels_agent_pos"}:
            for camera_name in self.camera_names:
                observation[CAMERA_OBSERVATION_KEYS[camera_name]] = self.render(camera_name)
        return observation

    def _task_metrics(self) -> tuple[bool, float]:
        assert self.data is not None and self._cube_body_id is not None and self._tray_site_id is not None
        cube_position = self.data.xpos[self._cube_body_id]
        tray_position = self.data.site_xpos[self._tray_site_id]
        distance = float(np.linalg.norm(cube_position - tray_position))
        xy_distance = float(np.linalg.norm(cube_position[:2] - tray_position[:2]))
        success = xy_distance < 0.055 and 0.012 < cube_position[2] < 0.09
        dense_reward = float(1.0 - np.tanh(5.0 * distance)) + float(success)
        return bool(success), dense_reward

    def _get_info(self, action_clipped: bool = False) -> dict[str, Any]:
        assert self.data is not None and self._cube_body_id is not None and self._tray_site_id is not None
        success, dense_reward = self._task_metrics()
        gripper_position = (
            self.data.site_xpos[self._gripper_site_id].astype(np.float32).copy()
            if self._gripper_site_id is not None
            else np.full(3, np.nan, dtype=np.float32)
        )
        return {
            "is_success": success,
            "task": self.task,
            "action_clipped": action_clipped,
            "dense_reward": dense_reward,
            "cube_position": self.data.xpos[self._cube_body_id].astype(np.float32).copy(),
            "tray_position": self.data.site_xpos[self._tray_site_id].astype(np.float32).copy(),
            "gripper_position": gripper_position,
        }

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        super().reset(seed=seed)
        self._load_model()
        assert self._mujoco is not None and self.model is not None and self.data is not None
        assert self._joint_qpos_addresses is not None and self._actuator_ids is not None
        assert self._cube_qpos_address is not None

        self._mujoco.mj_resetData(self.model, self.data)
        home_qpos = lerobot_action_to_qpos(self.home_action)
        self.data.qpos[self._joint_qpos_addresses] = home_qpos
        self.data.ctrl[self._actuator_ids] = home_qpos

        cube_xy = CUBE_SPAWN_POSITION[:2].copy()
        if self.cube_xy_randomization > 0:
            cube_xy += self.np_random.uniform(-self.cube_xy_randomization, self.cube_xy_randomization, size=2)
        cube_adr = self._cube_qpos_address
        self.data.qpos[cube_adr : cube_adr + 3] = [cube_xy[0], cube_xy[1], CUBE_SPAWN_POSITION[2]]
        self.data.qpos[cube_adr + 3 : cube_adr + 7] = [1.0, 0.0, 0.0, 0.0]
        self._mujoco.mj_forward(self.model, self.data)
        self._step_count = 0
        self._simulated_substeps = 0
        return self._get_observation(), self._get_info()

    def step(self, action: np.ndarray) -> tuple[dict[str, np.ndarray], float, bool, bool, dict[str, Any]]:
        self._load_model()
        assert self._mujoco is not None and self.model is not None and self.data is not None
        assert self._actuator_ids is not None

        action_array = np.asarray(action, dtype=np.float32)
        if action_array.shape != (6,):
            raise ValueError(f"Expected action shape (6,), got {action_array.shape}")
        clipped_action = np.clip(action_array, ACTION_LOW, ACTION_HIGH)
        action_clipped = not np.array_equal(action_array, clipped_action)
        self.data.ctrl[self._actuator_ids] = lerobot_action_to_qpos(clipped_action)

        substeps = self._substeps_for_next_frame()
        for _ in range(substeps):
            self._mujoco.mj_step(self.model, self.data)
        self._simulated_substeps += substeps

        self._step_count += 1
        success, dense_reward = self._task_metrics()
        reward = float(success) if self.reward_type == "sparse" else dense_reward
        terminated = bool(success and self.terminate_on_success)
        truncated = self._step_count >= self.max_episode_steps
        return self._get_observation(), reward, terminated, truncated, self._get_info(action_clipped)

    def render(self, camera_name: str = "front") -> np.ndarray:
        if camera_name not in CAMERA_NAMES:
            raise ValueError(f"Unsupported camera_name: {camera_name!r}")
        self._load_model()
        assert self._mujoco is not None and self.model is not None and self.data is not None
        if self._renderer is None:
            self._renderer = self._mujoco.Renderer(
                self.model,
                height=self.observation_height,
                width=self.observation_width,
            )
        self._renderer.update_scene(self.data, camera=camera_name)
        return self._renderer.render().copy()

    def camera_calibration(self, camera_name: str = "wrist") -> CameraCalibration:
        """Return the current camera pose and intrinsic field of view.

        The pose changes with the measured robot state because the wrist camera
        is a child of the gripper body. Object state is intentionally absent.
        """
        if camera_name not in CAMERA_NAMES:
            raise ValueError(f"Unsupported camera_name: {camera_name!r}")
        self._load_model()
        assert self._mujoco is not None and self.model is not None and self.data is not None
        camera_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if camera_id < 0:
            raise RuntimeError(f"The MuJoCo model is missing camera {camera_name!r}")
        self._mujoco.mj_forward(self.model, self.data)
        return CameraCalibration(
            name=camera_name,
            position=self.data.cam_xpos[camera_id].astype(np.float64).copy(),
            rotation=self.data.cam_xmat[camera_id].reshape(3, 3).astype(np.float64).copy(),
            vertical_fov_degrees=float(self.model.cam_fovy[camera_id]),
            image_height=self.observation_height,
            image_width=self.observation_width,
        )

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def create_so101_mujoco_envs(
    *,
    n_envs: int,
    gym_kwargs: dict[str, Any],
    env_cls: type[gym.vector.VectorEnv],
) -> dict[str, dict[int, gym.vector.VectorEnv]]:
    """Build the nested environment mapping expected by LeRobot."""

    def _make_one():
        return SO101MujocoEnv(**gym_kwargs)

    extra_kwargs: dict[str, Any] = {}
    if env_cls is gym.vector.AsyncVectorEnv:
        extra_kwargs["context"] = "forkserver"
    try:
        from gymnasium.vector import AutoresetMode

        vector_env = env_cls(
            [_make_one for _ in range(n_envs)],
            autoreset_mode=AutoresetMode.SAME_STEP,
            **extra_kwargs,
        )
    except ImportError:
        vector_env = env_cls([_make_one for _ in range(n_envs)], **extra_kwargs)
    return {"so101_mujoco": {0: vector_env}}
