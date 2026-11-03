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

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

import gymnasium as gym
import numpy as np

from .camera_profiles import (
    TOP_CAMERA_PROFILE_ID,
    WRIST_CAMERA_PROFILE_ID,
    CameraProfile,
    load_camera_profile,
)

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

CAMERA_NAMES = ("front", "top", "wrist")
POLICY_CAMERA_NAMES = ("top", "wrist")
CAMERA_OBSERVATION_KEYS = {
    "front": "pixels/front",
    "top": "pixels/top",
    "wrist": "pixels/wrist",
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
# Measured maxima from the verified 60-episode v2 IK teacher dataset. These
# are simulation rollout stability ceilings, not physical motor safety limits.
DEFAULT_VLA_ACTION_MAX_DELTA = np.array([1.75, 0.65, 0.30, 0.35, 0.12, 5.50], dtype=np.float32)
CUBE_SPAWN_POSITION = np.array([0.25453126220736555, -0.002930872758779989, 0.075], dtype=np.float64)
GOAL_TRAY_POSITION = np.array([0.20, 0.18, 0.031], dtype=np.float64)
CUBE_HALF_SIZE_M = 0.025
CUBE_SETTLED_CENTER_Z_M = 0.06881588
CUBE_TOP_PLANE_Z_M = CUBE_SETTLED_CENTER_Z_M + CUBE_HALF_SIZE_M
_GOAL_TRAY_CUBE_CENTER_HALF_EXTENT_M = 0.05
FINGER_PAD_GEOM_NAMES = ("dapier_fixed_finger_pad", "dapier_moving_finger_pad")
FINGER_PAD_CUBE_CONTACT_FRICTION = (1.6, 1.6, 0.02, 0.001, 0.001)
FINGER_PAD_CUBE_CONTACT_SOLREF = (-200_000.0, -400.0)
WRIST_CAMERA_HOUSING_GEOM_NAME = "dapier_wrist_camera_housing"
WRIST_CAMERA_LENS_GEOM_NAME = "dapier_wrist_camera_lens"
WRIST_CAMERA_MOUNT_GEOM_NAME = "dapier_wrist_camera_mount"
_CAMERA_PROFILE_IDS = {
    "top": TOP_CAMERA_PROFILE_ID,
    "wrist": WRIST_CAMERA_PROFILE_ID,
}
_FINGER_PAD_SPECS = (
    {
        "body": "gripper",
        "name": FINGER_PAD_GEOM_NAMES[0],
        # The outward +X face is exactly coplanar with the fixed fingertip STL.
        "pos": [-0.0109, -0.0002221, -0.097517],
        "quat": [0.70710678, 0.0, 0.70710678, 0.0],
        "size": [0.012, 0.008, 0.003],
    },
    {
        "body": "moving_jaw_so101_v1",
        "name": FINGER_PAD_GEOM_NAMES[1],
        # The outward -X face is exactly coplanar with the moving fingertip STL.
        "pos": [-0.0093, -0.0750583, 0.0188972],
        "quat": [-0.5, -0.5, 0.5, 0.5],
        "size": [0.008, 0.012, 0.003],
    },
)


def _action_trace_contract_id() -> str:
    payload = {
        "joint_names": JOINT_NAMES,
        "position_unit": "radian",
        "step_mode": "synchronous",
        "observation_alignment": "post_action_readback",
        "action_reference": "absolute_target",
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


ACTION_TRACE_CONTRACT_ID = _action_trace_contract_id()


@dataclass(frozen=True)
class ActionFilterResult:
    """One raw-to-applied action transformation with auditable deltas."""

    step_index: int
    chunk_step: int
    chunk_boundary: bool
    blend_weight: float
    raw_action: np.ndarray
    bounded_action: np.ndarray
    applied_action: np.ndarray
    raw_delta: np.ndarray
    applied_delta: np.ndarray
    slew_limited_axes: np.ndarray
    gripper_deadband_applied: bool

    @property
    def action_filtered(self) -> bool:
        return not np.allclose(self.bounded_action, self.applied_action, atol=1e-6)


class VLAActionFilter:
    """Blend action-chunk boundaries and reject frame-to-frame outliers."""

    def __init__(
        self,
        *,
        enabled: bool,
        action_chunk_steps: int,
        action_blend_steps: int,
        action_max_delta: tuple[float, ...] | list[float] | np.ndarray,
        gripper_action_deadband: float,
    ) -> None:
        if action_chunk_steps <= 0:
            raise ValueError("action_chunk_steps must be positive")
        if not 0 <= action_blend_steps <= action_chunk_steps:
            raise ValueError("action_blend_steps must be between zero and action_chunk_steps")
        max_delta = np.asarray(action_max_delta, dtype=np.float32)
        if max_delta.shape != (6,) or not np.all(np.isfinite(max_delta)) or np.any(max_delta <= 0):
            raise ValueError("action_max_delta must contain six positive finite values")
        if not np.isfinite(gripper_action_deadband) or gripper_action_deadband < 0:
            raise ValueError("gripper_action_deadband must be finite and non-negative")

        self.enabled = enabled
        self.action_chunk_steps = action_chunk_steps
        self.action_blend_steps = action_blend_steps
        self.action_max_delta = max_delta
        self.gripper_action_deadband = float(gripper_action_deadband)
        self._step_index = 0
        self._previous_raw: np.ndarray | None = None
        self._previous_applied: np.ndarray | None = None
        self._chunk_anchor: np.ndarray | None = None

    def reset(self, initial_action: np.ndarray) -> None:
        initial = np.asarray(initial_action, dtype=np.float32)
        if initial.shape != (6,) or not np.all(np.isfinite(initial)):
            raise ValueError("initial_action must contain six finite values")
        initial = np.clip(initial, ACTION_LOW, ACTION_HIGH)
        self._step_index = 0
        self._previous_raw = initial.copy()
        self._previous_applied = initial.copy()
        self._chunk_anchor = initial.copy()

    def apply(self, raw_action: np.ndarray) -> ActionFilterResult:
        raw = np.asarray(raw_action, dtype=np.float32)
        if raw.shape != (6,):
            raise ValueError(f"Expected action shape (6,), got {raw.shape}")
        if not np.all(np.isfinite(raw)):
            raise ValueError("action must contain only finite values")
        if self._previous_raw is None or self._previous_applied is None:
            raise RuntimeError("VLAActionFilter.reset() must be called before apply()")

        bounded = np.clip(raw, ACTION_LOW, ACTION_HIGH)
        chunk_step = self._step_index % self.action_chunk_steps
        chunk_boundary = chunk_step == 0
        if chunk_boundary:
            self._chunk_anchor = self._previous_applied.copy()

        candidate = bounded.copy()
        blend_weight = 1.0
        if self.enabled and self.action_blend_steps > 0 and chunk_step < self.action_blend_steps:
            assert self._chunk_anchor is not None
            blend_weight = (chunk_step + 1) / self.action_blend_steps
            candidate = self._chunk_anchor + blend_weight * (candidate - self._chunk_anchor)

        gripper_deadband_applied = False
        if (
            self.enabled
            and abs(float(candidate[5] - self._previous_applied[5])) < self.gripper_action_deadband
        ):
            candidate[5] = self._previous_applied[5]
            gripper_deadband_applied = True

        requested_delta = candidate - self._previous_applied
        if self.enabled:
            applied_delta = np.clip(requested_delta, -self.action_max_delta, self.action_max_delta)
        else:
            applied_delta = requested_delta
        applied = np.clip(self._previous_applied + applied_delta, ACTION_LOW, ACTION_HIGH)
        slew_limited_axes = self.enabled & (np.abs(requested_delta) > self.action_max_delta + 1e-6)

        result = ActionFilterResult(
            step_index=self._step_index,
            chunk_step=chunk_step,
            chunk_boundary=chunk_boundary,
            blend_weight=blend_weight,
            raw_action=raw.copy(),
            bounded_action=bounded.copy(),
            applied_action=applied.copy(),
            raw_delta=(bounded - self._previous_raw).copy(),
            applied_delta=(applied - self._previous_applied).copy(),
            slew_limited_axes=slew_limited_axes.copy(),
            gripper_deadband_applied=gripper_deadband_applied,
        )
        self._previous_raw = bounded.copy()
        self._previous_applied = applied.copy()
        self._step_index += 1
        return result


@dataclass(frozen=True)
class CameraCalibration:
    """Current calibrated pinhole pose for one MuJoCo RGB camera."""

    name: str
    position: np.ndarray
    rotation: np.ndarray
    vertical_fov_degrees: float
    image_height: int
    image_width: int
    profile_id: str = "untracked"
    physical_alignment_verified: bool = False


def camera_profile(camera_name: str) -> CameraProfile:
    """Return the repository-owned pose contract for a calibrated sim camera."""
    profile_id = _CAMERA_PROFILE_IDS.get(camera_name)
    if profile_id is None:
        raise ValueError(f"Camera {camera_name!r} has no auditable profile")
    return load_camera_profile(profile_id)


def _apply_camera_profile(camera: Any, profile: CameraProfile, mujoco: Any) -> None:
    camera.pos = profile.position_m
    camera.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    camera.alt.xyaxes = profile.xyaxes
    camera.fovy = profile.vertical_fov_degrees


def _profile_quaternion(profile: CameraProfile, mujoco: Any) -> np.ndarray:
    x_axis, y_axis = profile.xyaxes[:3], profile.xyaxes[3:]
    z_axis = np.cross(x_axis, y_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, rotation.reshape(-1))
    return quaternion


def lerobot_action_to_qpos(
    action: np.ndarray | list[float] | tuple[float, ...],
) -> np.ndarray:
    """Convert the LeRobot SO-101 six-value convention to MuJoCo radians."""
    action_array = np.asarray(action, dtype=np.float64)
    if action_array.shape != (6,):
        raise ValueError(f"Expected an SO-101 action with shape (6,), got {action_array.shape}")

    clipped = np.clip(action_array, ACTION_LOW, ACTION_HIGH)
    qpos = np.empty(6, dtype=np.float64)
    qpos[:5] = np.deg2rad(clipped[:5])
    qpos[5] = _JOINT_LOW_RAD[5] + clipped[5] / 100.0 * (_JOINT_HIGH_RAD[5] - _JOINT_LOW_RAD[5])
    return qpos


def qpos_to_lerobot_state(
    qpos: np.ndarray | list[float] | tuple[float, ...],
) -> np.ndarray:
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
        cube_xy_randomization: float = 0.025,
        camera_names: tuple[str, ...] | list[str] = POLICY_CAMERA_NAMES,
        home_action: tuple[float, ...] | list[float] | np.ndarray = DEFAULT_HOME_ACTION,
        action_smoothing: bool = False,
        action_chunk_steps: int = 25,
        action_blend_steps: int = 3,
        action_max_delta: tuple[float, ...] | list[float] | np.ndarray = DEFAULT_VLA_ACTION_MAX_DELTA,
        gripper_action_deadband: float = 1.0,
        action_trace_path: str | None = None,
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
        if action_trace_path is not None and not action_trace_path:
            raise ValueError("action_trace_path must be non-empty when provided")
        self.action_smoothing = action_smoothing
        self._action_filter = VLAActionFilter(
            enabled=action_smoothing,
            action_chunk_steps=action_chunk_steps,
            action_blend_steps=action_blend_steps,
            action_max_delta=action_max_delta,
            gripper_action_deadband=gripper_action_deadband,
        )
        self.action_trace_path = (
            Path(action_trace_path).expanduser().resolve() if action_trace_path is not None else None
        )
        self._action_trace_file: TextIO | None = None
        self._trace_sample_index = 0
        self._episode_index = -1
        self._episode_seed: int | None = None
        self._model_source_revision = "sha256:" + hashlib.sha256(self.model_path.read_bytes()).hexdigest()

        self.metadata = {**self.metadata, "render_fps": fps}
        self.action_space = gym.spaces.Box(low=ACTION_LOW, high=ACTION_HIGH, dtype=np.float32)
        observation_spaces: dict[str, gym.Space] = {}
        if obs_type in {"state", "pixels_agent_pos"}:
            observation_spaces["agent_pos"] = gym.spaces.Box(
                low=ACTION_LOW, high=ACTION_HIGH, dtype=np.float32
            )
        if obs_type in {"pixels", "pixels_agent_pos"}:
            observation_spaces["pixels"] = gym.spaces.Dict(
                {
                    camera_name: gym.spaces.Box(
                        low=0,
                        high=255,
                        shape=(observation_height, observation_width, 3),
                        dtype=np.uint8,
                    )
                    for camera_name in self.camera_names
                }
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
        self._cube_geom_id: int | None = None
        self._finger_pad_geom_ids: frozenset[int] = frozenset()
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
        for fingertip_body_name in ("gripper", "moving_jaw_so101_v1"):
            for geom in model_spec.body(fingertip_body_name).geoms:
                if geom.type == mujoco.mjtGeom.mjGEOM_MESH and geom.contype:
                    # MuJoCo collides against a mesh's convex hull. The hollow
                    # CAD fingers therefore produce contact in visually empty
                    # space; the measured flush pads below are the contact
                    # proxies for these two fingertip bodies instead.
                    geom.contype = 0
                    geom.conaffinity = 0
        for pad in _FINGER_PAD_SPECS:
            # One visible geom is also the collision geom. Its contact face is
            # flush with the measured STL fingertip plane, so the viewer and
            # physics engine agree about where contact occurs.
            model_spec.body(pad["body"]).add_geom(
                name=pad["name"],
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=pad["pos"],
                quat=pad["quat"],
                size=pad["size"],
                contype=1,
                conaffinity=1,
                condim=4,
                friction=[4.0, 0.02, 0.001],
                solref=[0.005, 1.0],
                solimp=[0.95, 0.99, 0.001, 0.5, 2.0],
                rgba=[0.07, 0.07, 0.07, 1.0],
                group=2,
                density=0,
            )
        for pad_name in FINGER_PAD_GEOM_NAMES:
            model_spec.add_pair(
                name=f"{pad_name}_cube_contact",
                geomname1=pad_name,
                geomname2="cube_geom",
                condim=4,
                friction=FINGER_PAD_CUBE_CONTACT_FRICTION,
                solref=FINGER_PAD_CUBE_CONTACT_SOLREF,
                solimp=[0.95, 0.99, 0.001, 0.5, 2.0],
            )

        wrist_profile = camera_profile("wrist")
        top_profile = camera_profile("top")
        wrist_camera = model_spec.camera("wrist")
        top_camera = model_spec.camera("top")
        if wrist_camera is None:
            wrist_camera = model_spec.body(wrist_profile.parent_body).add_camera(name="wrist")
        if top_camera is None:
            raise RuntimeError("The scene is missing the top camera declared by its camera profile")
        _apply_camera_profile(wrist_camera, wrist_profile, mujoco)
        _apply_camera_profile(top_camera, top_profile, mujoco)

        wrist_quaternion = _profile_quaternion(wrist_profile, mujoco)
        wrist_x_axis, wrist_y_axis = wrist_profile.xyaxes[:3], wrist_profile.xyaxes[3:]
        wrist_look_axis = -np.cross(wrist_x_axis, wrist_y_axis)
        housing_position = wrist_profile.position_m - 0.004 * wrist_look_axis
        lens_position = wrist_profile.position_m + 0.002 * wrist_look_axis
        model_spec.body("gripper").add_geom(
            name=WRIST_CAMERA_MOUNT_GEOM_NAME,
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[0.0025, -0.022, 0.0009, *housing_position],
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
            pos=housing_position,
            quat=wrist_quaternion,
            size=[0.0175, 0.0175, 0.002],
            contype=0,
            conaffinity=0,
            rgba=[0.025, 0.025, 0.025, 1.0],
            group=2,
            density=0,
        )
        model_spec.body("gripper").add_geom(
            name=WRIST_CAMERA_LENS_GEOM_NAME,
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            pos=lens_position,
            quat=wrist_quaternion,
            size=[0.005, 0.002, 0.0],
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
        self._cube_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        self._finger_pad_geom_ids = frozenset(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in FINGER_PAD_GEOM_NAMES
        )
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

    def _get_observation(self) -> dict[str, Any]:
        observation: dict[str, Any] = {}
        if self.obs_type in {"state", "pixels_agent_pos"}:
            observation["agent_pos"] = qpos_to_lerobot_state(self._joint_qpos())
        if self.obs_type in {"pixels", "pixels_agent_pos"}:
            observation["pixels"] = {
                camera_name: self.render(camera_name) for camera_name in self.camera_names
            }
        return observation

    def _task_metrics(self) -> tuple[bool, float]:
        assert self.data is not None and self._cube_body_id is not None and self._tray_site_id is not None
        cube_position = self.data.xpos[self._cube_body_id]
        tray_position = self.data.site_xpos[self._tray_site_id]
        distance = float(np.linalg.norm(cube_position - tray_position))
        xy_error = np.abs(cube_position[:2] - tray_position[:2])
        # The goal is a square tray, so success follows its usable square
        # footprint rather than an inscribed Euclidean circle.
        success = bool(
            np.all(xy_error < _GOAL_TRAY_CUBE_CENTER_HALF_EXTENT_M) and 0.012 < cube_position[2] < 0.09
        )
        dense_reward = float(1.0 - np.tanh(5.0 * distance)) + float(success)
        return bool(success), dense_reward

    def _finger_pad_cube_contact_metrics(self) -> tuple[bool, float]:
        assert self.data is not None and self._cube_geom_id is not None
        touching_pads: set[int] = set()
        max_penetration_m = 0.0
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            geom_pair = {int(contact.geom1), int(contact.geom2)}
            if self._cube_geom_id not in geom_pair:
                continue
            touched = geom_pair & self._finger_pad_geom_ids
            if not touched:
                continue
            touching_pads.update(touched)
            max_penetration_m = max(max_penetration_m, -float(contact.dist))
        return touching_pads == self._finger_pad_geom_ids, max_penetration_m

    def _get_info(
        self,
        action_clipped: bool = False,
        action_filter_result: ActionFilterResult | None = None,
    ) -> dict[str, Any]:
        assert self.data is not None and self._cube_body_id is not None and self._tray_site_id is not None
        success, dense_reward = self._task_metrics()
        bilateral_contact, max_penetration_m = self._finger_pad_cube_contact_metrics()
        gripper_position = (
            self.data.site_xpos[self._gripper_site_id].astype(np.float32).copy()
            if self._gripper_site_id is not None
            else np.full(3, np.nan, dtype=np.float32)
        )
        info = {
            "is_success": success,
            "task": self.task,
            "action_clipped": action_clipped,
            "dense_reward": dense_reward,
            "cube_position": self.data.xpos[self._cube_body_id].astype(np.float32).copy(),
            "tray_position": self.data.site_xpos[self._tray_site_id].astype(np.float32).copy(),
            "gripper_position": gripper_position,
            "finger_pad_cube_bilateral_contact": bilateral_contact,
            "finger_pad_cube_max_penetration_m": max_penetration_m,
        }
        if action_filter_result is not None:
            info.update(
                {
                    "action_filtered": action_filter_result.action_filtered,
                    "action_chunk_boundary": action_filter_result.chunk_boundary,
                    "action_raw": action_filter_result.raw_action.copy(),
                    "action_bounded": action_filter_result.bounded_action.copy(),
                    "action_applied": action_filter_result.applied_action.copy(),
                    "action_raw_delta": action_filter_result.raw_delta.copy(),
                    "action_applied_delta": action_filter_result.applied_delta.copy(),
                    "action_slew_limited_axes": action_filter_result.slew_limited_axes.copy(),
                    "action_gripper_deadband_applied": (action_filter_result.gripper_deadband_applied),
                }
            )
        return info

    def _write_action_trace(
        self,
        result: ActionFilterResult,
        *,
        reward: float,
        success: bool,
        terminated: bool,
        truncated: bool,
    ) -> None:
        if self.action_trace_path is None:
            return
        assert self.data is not None
        assert self._cube_body_id is not None and self._gripper_site_id is not None and self._tray_site_id is not None
        if self._action_trace_file is None:
            self.action_trace_path.parent.mkdir(parents=True, exist_ok=True)
            self._action_trace_file = self.action_trace_path.open("w", encoding="utf-8", buffering=1)
        self._trace_sample_index += 1
        bilateral_contact, max_penetration_m = self._finger_pad_cube_contact_metrics()
        record = {
            "schema_version": "dapier.so101.vla-action-trace.v1",
            "contract_id": ACTION_TRACE_CONTRACT_ID,
            "source_revision": self._model_source_revision,
            "joint_names": list(JOINT_NAMES),
            "episode_index": self._episode_index,
            "episode_seed": self._episode_seed,
            "step_index": result.step_index,
            # RCS JointTrace requires timestamps to increase across the whole
            # trace. Keep the reset-local MuJoCo clock as separate evidence.
            "trace_sample_index": self._trace_sample_index,
            "timestamp_ns": int(round(self._trace_sample_index / self.fps * 1_000_000_000)),
            "episode_timestamp_ns": int(round(float(self.data.time) * 1_000_000_000)),
            "chunk_step": result.chunk_step,
            "chunk_boundary": result.chunk_boundary,
            "blend_weight": result.blend_weight,
            "action_smoothing": self.action_smoothing,
            "raw_action_lerobot": result.raw_action.tolist(),
            "bounded_action_lerobot": result.bounded_action.tolist(),
            "applied_action_lerobot": result.applied_action.tolist(),
            "raw_delta_lerobot": result.raw_delta.tolist(),
            "applied_delta_lerobot": result.applied_delta.tolist(),
            "slew_limited_axes": result.slew_limited_axes.tolist(),
            "gripper_deadband_applied": result.gripper_deadband_applied,
            "command_positions_rad": lerobot_action_to_qpos(result.applied_action).tolist(),
            "simulation_positions_rad": self._joint_qpos().astype(np.float64).tolist(),
            "cube_position_m": self.data.xpos[self._cube_body_id].astype(np.float64).tolist(),
            "gripper_position_m": self.data.site_xpos[self._gripper_site_id].astype(np.float64).tolist(),
            "tray_position_m": self.data.site_xpos[self._tray_site_id].astype(np.float64).tolist(),
            "finger_pad_cube_bilateral_contact": bilateral_contact,
            "finger_pad_cube_max_penetration_m": max_penetration_m,
            "reward": reward,
            "is_success": success,
            "terminated": terminated,
            "truncated": truncated,
            "episode_done": terminated or truncated,
        }
        self._action_trace_file.write(json.dumps(record, separators=(",", ":")) + "\n")

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        super().reset(seed=seed)
        self._episode_seed = seed
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
        self.data.qpos[cube_adr : cube_adr + 3] = [
            cube_xy[0],
            cube_xy[1],
            CUBE_SPAWN_POSITION[2],
        ]
        self.data.qpos[cube_adr + 3 : cube_adr + 7] = [1.0, 0.0, 0.0, 0.0]
        self._mujoco.mj_forward(self.model, self.data)
        self._step_count = 0
        self._simulated_substeps = 0
        self._episode_index += 1
        self._action_filter.reset(self.home_action)
        return self._get_observation(), self._get_info()

    def step(self, action: np.ndarray) -> tuple[dict[str, np.ndarray], float, bool, bool, dict[str, Any]]:
        self._load_model()
        assert self._mujoco is not None and self.model is not None and self.data is not None
        assert self._actuator_ids is not None

        action_array = np.asarray(action, dtype=np.float32)
        if action_array.shape != (6,):
            raise ValueError(f"Expected action shape (6,), got {action_array.shape}")
        filter_result = self._action_filter.apply(action_array)
        action_clipped = not np.array_equal(action_array, filter_result.bounded_action)
        self.data.ctrl[self._actuator_ids] = lerobot_action_to_qpos(filter_result.applied_action)

        substeps = self._substeps_for_next_frame()
        for _ in range(substeps):
            self._mujoco.mj_step(self.model, self.data)
        self._simulated_substeps += substeps

        self._step_count += 1
        success, dense_reward = self._task_metrics()
        reward = float(success) if self.reward_type == "sparse" else dense_reward
        terminated = bool(success and self.terminate_on_success)
        truncated = self._step_count >= self.max_episode_steps
        self._write_action_trace(
            filter_result,
            reward=reward,
            success=success,
            terminated=terminated,
            truncated=truncated,
        )
        return (
            self._get_observation(),
            reward,
            terminated,
            truncated,
            self._get_info(action_clipped, filter_result),
        )

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

        The wrist pose changes with measured robot state; the top pose is fixed.
        Object state is intentionally absent from both calibrations.
        """
        if camera_name not in CAMERA_NAMES:
            raise ValueError(f"Unsupported camera_name: {camera_name!r}")
        self._load_model()
        assert self._mujoco is not None and self.model is not None and self.data is not None
        camera_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if camera_id < 0:
            raise RuntimeError(f"The MuJoCo model is missing camera {camera_name!r}")
        self._mujoco.mj_forward(self.model, self.data)
        profile_id = _CAMERA_PROFILE_IDS.get(camera_name, "untracked_front_sim")
        profile = load_camera_profile(profile_id) if camera_name in _CAMERA_PROFILE_IDS else None
        return CameraCalibration(
            name=camera_name,
            position=self.data.cam_xpos[camera_id].astype(np.float64).copy(),
            rotation=self.data.cam_xmat[camera_id].reshape(3, 3).astype(np.float64).copy(),
            vertical_fov_degrees=float(self.model.cam_fovy[camera_id]),
            image_height=self.observation_height,
            image_width=self.observation_width,
            profile_id=profile_id,
            physical_alignment_verified=(
                profile.physical_alignment_verified if profile is not None else False
            ),
        )

    def close(self) -> None:
        if self._action_trace_file is not None:
            self._action_trace_file.close()
            self._action_trace_file = None
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

    def _make_one(env_index: int):
        worker_kwargs = dict(gym_kwargs)
        action_trace_path = worker_kwargs.get("action_trace_path")
        if action_trace_path is not None:
            worker_kwargs["action_trace_path"] = str(action_trace_path).format(env_index=env_index)
        return SO101MujocoEnv(**worker_kwargs)

    extra_kwargs: dict[str, Any] = {}
    if env_cls is gym.vector.AsyncVectorEnv:
        extra_kwargs["context"] = "forkserver"
    try:
        from gymnasium.vector import AutoresetMode

        vector_env = env_cls(
            [lambda env_index=env_index: _make_one(env_index) for env_index in range(n_envs)],
            autoreset_mode=AutoresetMode.SAME_STEP,
            **extra_kwargs,
        )
    except ImportError:
        vector_env = env_cls(
            [lambda env_index=env_index: _make_one(env_index) for env_index in range(n_envs)],
            **extra_kwargs,
        )
    return {"so101_mujoco": {0: vector_env}}
