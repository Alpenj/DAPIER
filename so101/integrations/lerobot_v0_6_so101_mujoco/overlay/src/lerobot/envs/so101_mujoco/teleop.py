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

"""Input adapters shared by the interactive SO-101 MuJoCo examples."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from .env import (
    ACTION_HIGH,
    ACTION_LOW,
    CUBE_SPAWN_POSITION,
    DEFAULT_HOME_ACTION,
    GOAL_TRAY_POSITION,
    JOINT_NAMES,
    lerobot_action_to_qpos,
    qpos_to_lerobot_state,
)

PICK_APPROACH_ACTION = np.array([0.0, -14.8461, 24.4459, 67.3505, 0.0, 100.0], dtype=np.float32)
PICK_CLEAR_ACTION = np.array([0.0, -45.0, 17.5, 90.0, 0.0, 100.0], dtype=np.float32)
IK_OBSERVE_ACTION = np.array([70.0, -45.0, 20.0, 90.0, 0.0, 100.0], dtype=np.float32)
PICK_LIFT_FRAMES = 390
_PICK_PHASE_BOUNDARIES = (0, 30, 130, 210, 240, 360, PICK_LIFT_FRAMES)
_SCRIPTED_PICK_CLOSED_PERCENT = 27.0
_PICK_NOMINAL_LIFT_ACTION = np.array(
    [0.0, -13.5897, 1.8477, 88.5845, 0.0, _SCRIPTED_PICK_CLOSED_PERCENT], dtype=np.float32
)
VISION_SETTLE_FRAMES = 30
VISION_MAX_CUBE_OFFSET_M = 0.045
VISION_GRASP_CLOSE_PERCENT = 35.0
VISION_GRASP_Z_OFFSET_M = -0.015
VISION_MAX_PAD_PENETRATION_M = 0.001
VISION_PHASES = (
    ("leave top observation pose", 60),
    ("approach", 100),
    ("close", 80),
    ("grasp", 30),
    ("lift", 120),
    ("lift hold", 30),
    ("transfer", 90),
    ("transfer hold", 20),
    ("release", 20),
    ("settle", 80),
)
VISION_PICK_PLACE_FRAMES = sum(frame_count for _, frame_count in VISION_PHASES)


@dataclass
class ResetSeedSequence:
    """Issue one deterministic, never-repeated seed for every scene reset."""

    base_seed: int
    reset_count: int = 0

    def __post_init__(self) -> None:
        if isinstance(self.base_seed, bool) or not isinstance(self.base_seed, int):
            raise ValueError("base_seed must be an integer")
        if self.base_seed < 0:
            raise ValueError("base_seed must be non-negative")
        if self.reset_count != 0:
            raise ValueError("reset_count must start at zero")

    @property
    def initial_seed(self) -> int:
        return self.base_seed

    def next_seed(self) -> int:
        self.reset_count += 1
        return self.base_seed + self.reset_count


@dataclass(frozen=True)
class VisionPickPlacePlan:
    """Pixel-conditioned action plan with auditable kinematic waypoints."""

    actions: np.ndarray
    estimated_cube_xy: np.ndarray
    approach_action: np.ndarray
    lift_action: np.ndarray
    goal_closed_action: np.ndarray
    phase_ends: tuple[int, ...]

    def stage_for_frame(self, frame_index: int) -> str:
        if isinstance(frame_index, bool) or not isinstance(frame_index, int):
            raise ValueError("frame_index must be an integer")
        if not 0 <= frame_index < len(self.actions):
            raise ValueError(f"frame_index must be in [0, {len(self.actions) - 1}]")
        for (stage, _), phase_end in zip(VISION_PHASES, self.phase_ends, strict=True):
            if frame_index < phase_end:
                return stage
        raise AssertionError("unreachable vision phase")


class JointJogController:
    """Maintain a bounded six-joint target for keyboard jogging."""

    def __init__(
        self,
        *,
        home_action: np.ndarray = DEFAULT_HOME_ACTION,
        joint_step_degrees: float = 2.0,
        gripper_step_percent: float = 5.0,
    ) -> None:
        if joint_step_degrees <= 0 or gripper_step_percent <= 0:
            raise ValueError("Jog step sizes must be positive")
        home = np.asarray(home_action, dtype=np.float32)
        if home.shape != (6,):
            raise ValueError(f"home_action must have shape (6,), got {home.shape}")

        self.home_action = np.clip(home, ACTION_LOW, ACTION_HIGH).astype(np.float32)
        self.joint_step_degrees = float(joint_step_degrees)
        self.gripper_step_percent = float(gripper_step_percent)
        self.selected_joint = 0
        self._action = self.home_action.copy()

    @property
    def selected_joint_name(self) -> str:
        return JOINT_NAMES[self.selected_joint]

    def select_joint(self, index: int) -> None:
        if not 0 <= index < len(JOINT_NAMES):
            raise ValueError(f"Joint index must be in [0, {len(JOINT_NAMES) - 1}], got {index}")
        self.selected_joint = index

    def select_previous_joint(self) -> int:
        self.selected_joint = (self.selected_joint - 1) % len(JOINT_NAMES)
        return self.selected_joint

    def select_next_joint(self) -> int:
        self.selected_joint = (self.selected_joint + 1) % len(JOINT_NAMES)
        return self.selected_joint

    def jog(self, direction: int, *, scale: float = 1.0) -> np.ndarray:
        if direction not in {-1, 1}:
            raise ValueError(f"direction must be -1 or 1, got {direction}")
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError(f"scale must be positive and finite, got {scale}")
        step = self.gripper_step_percent if self.selected_joint == 5 else self.joint_step_degrees
        self._action[self.selected_joint] += direction * step * scale
        self._action = np.clip(self._action, ACTION_LOW, ACTION_HIGH).astype(np.float32)
        return self.get_action()

    def adjust_joint(self, index: int, amount: float) -> np.ndarray:
        if not 0 <= index < len(JOINT_NAMES):
            raise ValueError(f"Joint index must be in [0, {len(JOINT_NAMES) - 1}], got {index}")
        if not np.isfinite(amount):
            raise ValueError(f"amount must be finite, got {amount}")
        self._action[index] += amount
        self._action = np.clip(self._action, ACTION_LOW, ACTION_HIGH).astype(np.float32)
        return self.get_action()

    def set_action(self, action: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        action_array = np.asarray(action, dtype=np.float32)
        if action_array.shape != (6,):
            raise ValueError(f"action must have shape (6,), got {action_array.shape}")
        if not np.all(np.isfinite(action_array)):
            raise ValueError(f"action contains non-finite values: {action_array}")
        self._action = np.clip(action_array, ACTION_LOW, ACTION_HIGH).astype(np.float32)
        return self.get_action()

    def reset(self) -> np.ndarray:
        self._action = self.home_action.copy()
        return self.get_action()

    def get_action(self) -> np.ndarray:
        return self._action.copy()


class CartesianJogController:
    """Move the gripper site in world XYZ with bounded damped-least-squares IK."""

    def __init__(
        self,
        model: Any,
        *,
        joint_controller: JointJogController | None = None,
        site_name: str = "gripperframe",
        damping: float = 0.02,
        tolerance_m: float = 5e-4,
        max_iterations: int = 40,
    ) -> None:
        if damping <= 0 or tolerance_m <= 0 or max_iterations <= 0:
            raise ValueError("IK damping, tolerance, and max_iterations must be positive")

        import mujoco

        self._mujoco = mujoco
        self.model = model
        self.joints = joint_controller or JointJogController()
        self.damping = float(damping)
        self.tolerance_m = float(tolerance_m)
        self.max_iterations = int(max_iterations)
        self.last_cartesian_error_m = 0.0
        self.last_orientation_error_rad = 0.0

        self._site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES]
        if self._site_id < 0 or min(joint_ids) < 0:
            raise ValueError("The MuJoCo model is missing the SO-101 joints or gripperframe site")
        self._joint_ids = np.asarray(joint_ids, dtype=np.int32)
        self._qpos_addresses = model.jnt_qposadr[self._joint_ids].astype(np.int32)
        # Wrist roll is preserved: the first four joints are sufficient for XYZ motion.
        self._ik_dof_addresses = model.jnt_dofadr[self._joint_ids[:4]].astype(np.int32)
        self._pose_dof_addresses = model.jnt_dofadr[self._joint_ids[:5]].astype(np.int32)
        self._ik_data = mujoco.MjData(model)

    @property
    def selected_joint(self) -> int:
        return self.joints.selected_joint

    @property
    def selected_joint_name(self) -> str:
        return self.joints.selected_joint_name

    def select_previous_joint(self) -> int:
        return self.joints.select_previous_joint()

    def select_next_joint(self) -> int:
        return self.joints.select_next_joint()

    def jog_selected_joint(self, amount: float) -> np.ndarray:
        return self.joints.adjust_joint(self.selected_joint, amount)

    def adjust_gripper(self, amount: float) -> np.ndarray:
        return self.joints.adjust_joint(5, amount)

    def set_action(self, action: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        return self.joints.set_action(action)

    def reset(self) -> np.ndarray:
        return self.joints.reset()

    def get_action(self) -> np.ndarray:
        return self.joints.get_action()

    def site_position(self, action: np.ndarray | None = None) -> np.ndarray:
        action_array = self.get_action() if action is None else np.asarray(action, dtype=np.float32)
        qpos = lerobot_action_to_qpos(action_array)
        self._mujoco.mj_resetData(self.model, self._ik_data)
        self._ik_data.qpos[self._qpos_addresses] = qpos
        self._mujoco.mj_forward(self.model, self._ik_data)
        return self._ik_data.site_xpos[self._site_id].copy()

    def site_rotation(self, action: np.ndarray | None = None) -> np.ndarray:
        """Return the gripper site's 3x3 world rotation matrix."""
        action_array = self.get_action() if action is None else np.asarray(action, dtype=np.float32)
        qpos = lerobot_action_to_qpos(action_array)
        self._mujoco.mj_resetData(self.model, self._ik_data)
        self._ik_data.qpos[self._qpos_addresses] = qpos
        self._mujoco.mj_forward(self.model, self._ik_data)
        return self._ik_data.site_xmat[self._site_id].reshape(3, 3).copy()

    def move(self, delta_xyz_m: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        delta = np.asarray(delta_xyz_m, dtype=np.float64)
        if delta.shape != (3,):
            raise ValueError(f"delta_xyz_m must have shape (3,), got {delta.shape}")
        if not np.all(np.isfinite(delta)):
            raise ValueError(f"delta_xyz_m contains non-finite values: {delta}")
        return self.move_to(self.site_position() + delta)

    def move_to(self, target_xyz_m: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        target = np.asarray(target_xyz_m, dtype=np.float64)
        if target.shape != (3,):
            raise ValueError(f"target_xyz_m must have shape (3,), got {target.shape}")
        if not np.all(np.isfinite(target)):
            raise ValueError(f"target_xyz_m contains non-finite values: {target}")

        current_action = self.get_action()
        robot_qpos = lerobot_action_to_qpos(current_action)
        joint_ranges = self.model.jnt_range[self._joint_ids[:4]]
        jacobian_position = np.zeros((3, self.model.nv), dtype=np.float64)

        for _ in range(self.max_iterations):
            self._mujoco.mj_resetData(self.model, self._ik_data)
            self._ik_data.qpos[self._qpos_addresses] = robot_qpos
            self._mujoco.mj_forward(self.model, self._ik_data)
            error = target - self._ik_data.site_xpos[self._site_id]
            if np.linalg.norm(error) <= self.tolerance_m:
                break

            jacobian_position.fill(0.0)
            self._mujoco.mj_jacSite(
                self.model,
                self._ik_data,
                jacobian_position,
                None,
                self._site_id,
            )
            jacobian = jacobian_position[:, self._ik_dof_addresses]
            regularized = jacobian @ jacobian.T + self.damping**2 * np.eye(3)
            joint_delta = jacobian.T @ np.linalg.solve(regularized, error)
            robot_qpos[:4] += np.clip(joint_delta, -0.04, 0.04)
            robot_qpos[:4] = np.clip(robot_qpos[:4], joint_ranges[:, 0], joint_ranges[:, 1])

        solved_action = qpos_to_lerobot_state(robot_qpos)
        solved_action[4:] = current_action[4:]
        self.set_action(solved_action)
        self.last_cartesian_error_m = float(np.linalg.norm(target - self.site_position()))
        self.last_orientation_error_rad = 0.0
        return self.get_action()

    def move_preserving_orientation(
        self,
        delta_xyz_m: np.ndarray | list[float] | tuple[float, ...],
    ) -> np.ndarray:
        """Move in world XYZ while preserving the current gripper orientation."""
        delta = np.asarray(delta_xyz_m, dtype=np.float64)
        if delta.shape != (3,):
            raise ValueError(f"delta_xyz_m must have shape (3,), got {delta.shape}")
        if not np.all(np.isfinite(delta)):
            raise ValueError(f"delta_xyz_m contains non-finite values: {delta}")
        return self.move_to_pose(self.site_position() + delta, self.site_rotation())

    def move_to_pose(
        self,
        target_xyz_m: np.ndarray | list[float] | tuple[float, ...],
        target_rotation: np.ndarray,
        *,
        orientation_weight_m: float = 0.05,
    ) -> np.ndarray:
        """Solve a bounded five-joint position-and-orientation waypoint.

        The rotational residual is scaled to metres so translation and
        orientation can share one damped-least-squares solve. This is intended
        for short grasped-object moves where allowing wrist pitch drift would
        open one side of the grasp.
        """
        target = np.asarray(target_xyz_m, dtype=np.float64)
        rotation = np.asarray(target_rotation, dtype=np.float64)
        if target.shape != (3,):
            raise ValueError(f"target_xyz_m must have shape (3,), got {target.shape}")
        if rotation.shape != (3, 3):
            raise ValueError(f"target_rotation must have shape (3, 3), got {rotation.shape}")
        if not np.all(np.isfinite(target)) or not np.all(np.isfinite(rotation)):
            raise ValueError("pose target contains non-finite values")
        if orientation_weight_m <= 0 or not np.isfinite(orientation_weight_m):
            raise ValueError("orientation_weight_m must be positive and finite")

        current_action = self.get_action()
        robot_qpos = lerobot_action_to_qpos(current_action)
        joint_ranges = self.model.jnt_range[self._joint_ids[:5]]
        jacobian_position = np.zeros((3, self.model.nv), dtype=np.float64)
        jacobian_rotation = np.zeros((3, self.model.nv), dtype=np.float64)
        position_error = np.full(3, np.inf)
        orientation_error = np.full(3, np.inf)

        for _ in range(max(self.max_iterations, 150)):
            self._mujoco.mj_resetData(self.model, self._ik_data)
            self._ik_data.qpos[self._qpos_addresses] = robot_qpos
            self._mujoco.mj_forward(self.model, self._ik_data)
            position_error = target - self._ik_data.site_xpos[self._site_id]
            current_rotation = self._ik_data.site_xmat[self._site_id].reshape(3, 3)
            orientation_error = 0.5 * sum(
                np.cross(current_rotation[:, axis], rotation[:, axis]) for axis in range(3)
            )
            if (
                np.linalg.norm(position_error) <= self.tolerance_m
                and np.linalg.norm(orientation_error) <= 2e-3
            ):
                break

            jacobian_position.fill(0.0)
            jacobian_rotation.fill(0.0)
            self._mujoco.mj_jacSite(
                self.model,
                self._ik_data,
                jacobian_position,
                jacobian_rotation,
                self._site_id,
            )
            jacobian = np.vstack(
                (
                    jacobian_position[:, self._pose_dof_addresses],
                    orientation_weight_m * jacobian_rotation[:, self._pose_dof_addresses],
                )
            )
            residual = np.concatenate((position_error, orientation_weight_m * orientation_error))
            regularized = jacobian.T @ jacobian + self.damping**2 * np.eye(5)
            joint_delta = np.linalg.solve(regularized, jacobian.T @ residual)
            robot_qpos[:5] += np.clip(joint_delta, -0.06, 0.06)
            robot_qpos[:5] = np.clip(robot_qpos[:5], joint_ranges[:, 0], joint_ranges[:, 1])

        solved_action = qpos_to_lerobot_state(robot_qpos)
        solved_action[5] = current_action[5]
        self.set_action(solved_action)
        self.last_cartesian_error_m = float(np.linalg.norm(target - self.site_position()))
        solved_rotation = self.site_rotation()
        trace_cosine = np.clip((np.trace(solved_rotation.T @ rotation) - 1.0) * 0.5, -1.0, 1.0)
        self.last_orientation_error_rad = float(np.arccos(trace_cosine))
        return self.get_action()


def _smoothstep_actions(start: np.ndarray, stop: np.ndarray, fraction: float) -> np.ndarray:
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("smoothstep fraction must be in [0, 1]")
    weight = fraction * fraction * (3.0 - 2.0 * fraction)
    interpolated = start.astype(np.float64) * (1.0 - weight) + stop.astype(np.float64) * weight
    return np.clip(interpolated, ACTION_LOW, ACTION_HIGH).astype(np.float32)


def _action_segment(start: np.ndarray, stop: np.ndarray, frames: int) -> np.ndarray:
    if frames <= 0:
        raise ValueError("frames must be positive")
    return np.stack([_smoothstep_actions(start, stop, (index + 1) / frames) for index in range(frames)])


def build_vision_pick_place_plan(
    model: Any,
    estimated_cube_xy: np.ndarray | list[float] | tuple[float, float],
    *,
    goal_xy: np.ndarray | list[float] | tuple[float, float] = GOAL_TRAY_POSITION[:2],
) -> VisionPickPlacePlan:
    """Build a camera-conditioned pick-and-place plan without object-state access.

    ``estimated_cube_xy`` is expected to come from a calibrated RGB detector. The
    green tray is a fixed task destination, so its calibrated XY is supplied as
    task configuration rather than read from MuJoCo state.
    """
    cube_xy = np.asarray(estimated_cube_xy, dtype=np.float64)
    destination_xy = np.asarray(goal_xy, dtype=np.float64)
    if cube_xy.shape != (2,) or destination_xy.shape != (2,):
        raise ValueError("estimated_cube_xy and goal_xy must both have shape (2,)")
    if not np.all(np.isfinite(cube_xy)) or not np.all(np.isfinite(destination_xy)):
        raise ValueError("estimated_cube_xy and goal_xy must be finite")

    cube_offset = cube_xy - CUBE_SPAWN_POSITION[:2]
    if np.any(np.abs(cube_offset) > VISION_MAX_CUBE_OFFSET_M):
        raise ValueError(
            "Vision estimate is outside the verified pick workspace: "
            f"offset={np.array2string(cube_offset, precision=4)} m"
        )

    controller = CartesianJogController(model, max_iterations=250)
    controller.set_action(PICK_APPROACH_ACTION)
    approach_action = controller.move_preserving_orientation(
        [cube_offset[0], cube_offset[1], VISION_GRASP_Z_OFFSET_M]
    )
    if controller.last_cartesian_error_m > 0.002 or controller.last_orientation_error_rad > 0.04:
        raise RuntimeError(
            "Could not solve camera-guided approach waypoint: "
            f"position={controller.last_cartesian_error_m:.6f} m "
            f"orientation={controller.last_orientation_error_rad:.6f} rad"
        )
    approach_action[5] = 100.0
    grasp_action = approach_action.copy()
    grasp_action[5] = VISION_GRASP_CLOSE_PERCENT

    controller.set_action(approach_action)
    lift_action = controller.move_preserving_orientation([0.0, 0.0, 0.05])
    if controller.last_cartesian_error_m > 0.002 or controller.last_orientation_error_rad > 0.02:
        raise RuntimeError(
            "Could not solve camera-guided lift waypoint: "
            f"position={controller.last_cartesian_error_m:.6f} m "
            f"orientation={controller.last_orientation_error_rad:.6f} rad"
        )
    lift_action[5] = VISION_GRASP_CLOSE_PERCENT

    # The grasp transform is calibrated by the CAD-aligned padded task. Translating
    # the gripper by destination - visually estimated cube position does not use
    # the privileged cube body pose and leaves the final tray tolerance to the
    # task evaluator.
    lift_site = controller.site_position(lift_action)
    goal_site = lift_site.copy()
    goal_site[:2] += destination_xy - cube_xy
    goal_site[2] += 0.01
    controller.set_action(lift_action)
    goal_closed_action = controller.move_to(goal_site)
    if controller.last_cartesian_error_m > 0.003:
        raise RuntimeError(
            f"Could not solve camera-guided transfer waypoint: {controller.last_cartesian_error_m:.6f} m"
        )
    goal_closed_action[5] = VISION_GRASP_CLOSE_PERCENT
    goal_open_action = goal_closed_action.copy()
    goal_open_action[5] = 100.0

    phase_actions = (
        _action_segment(IK_OBSERVE_ACTION, PICK_CLEAR_ACTION, 60),
        _action_segment(PICK_CLEAR_ACTION, approach_action, 100),
        _action_segment(approach_action, grasp_action, 80),
        np.repeat(grasp_action[None, :], 30, axis=0),
        _action_segment(grasp_action, lift_action, 120),
        np.repeat(lift_action[None, :], 30, axis=0),
        _action_segment(lift_action, goal_closed_action, 90),
        np.repeat(goal_closed_action[None, :], 20, axis=0),
        _action_segment(goal_closed_action, goal_open_action, 20),
        np.repeat(goal_open_action[None, :], 80, axis=0),
    )
    actions = np.concatenate(phase_actions, axis=0).astype(np.float32)
    if actions.shape != (VISION_PICK_PLACE_FRAMES, 6):
        raise AssertionError(f"Unexpected vision plan shape: {actions.shape}")
    phase_ends = tuple(np.cumsum([frame_count for _, frame_count in VISION_PHASES]).tolist())
    return VisionPickPlacePlan(
        actions=actions,
        estimated_cube_xy=cube_xy.copy(),
        approach_action=approach_action.copy(),
        lift_action=lift_action.copy(),
        goal_closed_action=goal_closed_action.copy(),
        phase_ends=phase_ends,
    )


def scripted_pick_lift_action(frame_index: int) -> np.ndarray:
    """Return one command from the verified CAD-aligned pick-and-lift trace."""
    if isinstance(frame_index, bool) or not isinstance(frame_index, int):
        raise ValueError("frame_index must be an integer")
    if not 0 <= frame_index < PICK_LIFT_FRAMES:
        raise ValueError(f"frame_index must be in [0, {PICK_LIFT_FRAMES - 1}]")

    high_open = PICK_CLEAR_ACTION
    low_open = PICK_APPROACH_ACTION
    low_closed = low_open.copy()
    low_closed[5] = _SCRIPTED_PICK_CLOSED_PERCENT
    lifted = _PICK_NOMINAL_LIFT_ACTION
    _, settle, descend, close, grasp, lift, end = _PICK_PHASE_BOUNDARIES

    if frame_index < settle:
        return high_open.copy()
    if frame_index < descend:
        return _smoothstep_actions(high_open, low_open, (frame_index - settle) / (descend - settle))
    if frame_index < close:
        return _smoothstep_actions(low_open, low_closed, (frame_index - descend) / (close - descend))
    if frame_index < grasp:
        return low_closed.copy()
    if frame_index < lift:
        return _smoothstep_actions(low_closed, lifted, (frame_index - grasp) / (lift - grasp))
    if frame_index < end:
        return lifted.copy()
    raise AssertionError("unreachable scripted pick phase")


def leader_action_dict_to_array(action: Mapping[str, Any]) -> np.ndarray:
    """Convert an official SO101Leader action dictionary to the simulator contract."""
    keys = [f"{joint_name}.pos" for joint_name in JOINT_NAMES]
    missing = [key for key in keys if key not in action]
    if missing:
        raise ValueError(f"Leader action is missing keys: {missing}")
    values = np.asarray([action[key] for key in keys], dtype=np.float32)
    if not np.all(np.isfinite(values)):
        raise ValueError(f"Leader action contains non-finite values: {values}")
    return np.clip(values, ACTION_LOW, ACTION_HIGH).astype(np.float32)


def should_save_episode(*, success: bool, save_mode: str) -> bool:
    """Return whether the recorder should commit the current episode."""
    if save_mode not in {"successful", "all"}:
        raise ValueError(f"Unsupported save_mode: {save_mode!r}")
    return save_mode == "all" or success


class SO101LeaderActionSource:
    """Read a calibrated leader through LeRobot's maintained teleoperator API."""

    def __init__(self, *, port: str, leader_id: str) -> None:
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

        self._leader = SO101Leader(SO101LeaderConfig(port=port, id=leader_id, use_degrees=True))

    def __enter__(self) -> SO101LeaderActionSource:
        self._leader.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def get_action(self) -> np.ndarray:
        return leader_action_dict_to_array(self._leader.get_action())

    def close(self) -> None:
        if self._leader.is_connected:
            self._leader.disconnect()
