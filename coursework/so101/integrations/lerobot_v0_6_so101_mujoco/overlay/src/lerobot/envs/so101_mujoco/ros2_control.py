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

"""ROS 2-neutral control primitives for the SO-101 bridge.

ROS ``JointState`` and ``JointTrajectory`` positions use radians. LeRobot's
calibrated SO-101 contract uses degrees for the five arm joints and a 0..100
range for the gripper. Keeping all conversions here makes the ROS node small
and lets the safety-critical parts run in tests without a ROS installation.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from .env import (
    ACTION_HIGH,
    ACTION_LOW,
    JOINT_NAMES,
    SO101MujocoEnv,
    lerobot_action_to_qpos,
    qpos_to_lerobot_state,
)

ROS_POSITION_LOW = lerobot_action_to_qpos(ACTION_LOW)
ROS_POSITION_HIGH = lerobot_action_to_qpos(ACTION_HIGH)


def reorder_joint_positions(
    joint_names: Sequence[str], positions: Sequence[float] | np.ndarray
) -> np.ndarray:
    """Validate and reorder a full named ROS joint vector to ``JOINT_NAMES``."""
    names = tuple(joint_names)
    values = np.asarray(positions, dtype=np.float64)
    if values.shape != (len(names),):
        raise ValueError(
            f"Expected one position per joint name, got names={len(names)}, shape={values.shape}"
        )
    if len(names) != len(set(names)):
        raise ValueError(f"Joint names contain duplicates: {names}")

    expected = set(JOINT_NAMES)
    received = set(names)
    if received != expected:
        missing = sorted(expected - received)
        extra = sorted(received - expected)
        raise ValueError(f"Trajectory must contain all SO-101 joints; missing={missing}, extra={extra}")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"Joint positions contain non-finite values: {values}")

    by_name = dict(zip(names, values, strict=True))
    return np.asarray([by_name[name] for name in JOINT_NAMES], dtype=np.float64)


def ros_positions_to_lerobot_action(
    joint_names: Sequence[str], positions: Sequence[float] | np.ndarray
) -> np.ndarray:
    """Convert a named ROS radian vector to the calibrated LeRobot action."""
    ordered = reorder_joint_positions(joint_names, positions)
    outside = (ordered < ROS_POSITION_LOW - 1e-9) | (ordered > ROS_POSITION_HIGH + 1e-9)
    if np.any(outside):
        violations = {JOINT_NAMES[index]: float(ordered[index]) for index in np.flatnonzero(outside)}
        raise ValueError(f"Joint command exceeds the SO-101 limits (radians): {violations}")
    return qpos_to_lerobot_state(ordered)


def lerobot_state_to_ros_positions(state: Sequence[float] | np.ndarray) -> np.ndarray:
    """Convert LeRobot degrees + gripper percent to ROS joint radians."""
    return lerobot_action_to_qpos(np.asarray(state, dtype=np.float64))


def lerobot_mapping_to_ros_positions(values: dict[str, Any]) -> np.ndarray:
    """Convert a LeRobot ``*.pos`` observation dictionary to ROS radians."""
    missing = [f"{name}.pos" for name in JOINT_NAMES if f"{name}.pos" not in values]
    if missing:
        raise ValueError(f"SO-101 observation is missing keys: {missing}")
    state = np.asarray([values[f"{name}.pos"] for name in JOINT_NAMES], dtype=np.float64)
    if not np.all(np.isfinite(state)):
        raise ValueError(f"SO-101 observation contains non-finite positions: {state}")
    return lerobot_state_to_ros_positions(state)


@dataclass(frozen=True)
class TrajectoryPoint:
    """One validated ROS-radian waypoint."""

    positions: np.ndarray
    time_from_start: float


class LinearJointTrajectory:
    """Sample a full SO-101 trajectory with position-linear interpolation."""

    def __init__(
        self,
        *,
        joint_names: Sequence[str],
        positions: Sequence[Sequence[float] | np.ndarray],
        times_from_start: Sequence[float],
        current_positions: Sequence[float] | np.ndarray,
        start_time: float,
    ) -> None:
        if len(positions) == 0:
            raise ValueError("A trajectory must contain at least one point")
        if len(positions) != len(times_from_start):
            raise ValueError("Trajectory positions and timestamps must have the same length")
        if not np.isfinite(start_time):
            raise ValueError(f"start_time must be finite, got {start_time}")

        current = np.asarray(current_positions, dtype=np.float64)
        if current.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(current)):
            raise ValueError(f"current_positions must be a finite shape-(6,) vector, got {current}")

        times = np.asarray(times_from_start, dtype=np.float64)
        if not np.all(np.isfinite(times)) or np.any(times < 0):
            raise ValueError(f"Trajectory timestamps must be finite and non-negative: {times}")
        if np.any(np.diff(times) <= 0):
            raise ValueError(f"Trajectory timestamps must be strictly increasing: {times}")

        points = [
            TrajectoryPoint(reorder_joint_positions(joint_names, row), float(time_value))
            for row, time_value in zip(positions, times, strict=True)
        ]
        for point in points:
            ros_positions_to_lerobot_action(JOINT_NAMES, point.positions)

        if points[0].time_from_start > 0:
            points.insert(0, TrajectoryPoint(current.copy(), 0.0))

        self.points = tuple(points)
        self.start_time = float(start_time)

    @property
    def duration(self) -> float:
        return self.points[-1].time_from_start

    def sample(self, now: float) -> tuple[np.ndarray, bool]:
        """Return interpolated ROS positions and whether the path is complete."""
        if not np.isfinite(now):
            raise ValueError(f"now must be finite, got {now}")
        elapsed = max(0.0, float(now) - self.start_time)
        if elapsed >= self.duration:
            return self.points[-1].positions.copy(), True
        if elapsed <= self.points[0].time_from_start:
            return self.points[0].positions.copy(), False

        right = next(index for index, point in enumerate(self.points) if point.time_from_start > elapsed)
        left_point = self.points[right - 1]
        right_point = self.points[right]
        span = right_point.time_from_start - left_point.time_from_start
        fraction = (elapsed - left_point.time_from_start) / span
        positions = left_point.positions + fraction * (right_point.positions - left_point.positions)
        return positions.astype(np.float64), False


class SO101MujocoROSBackend:
    """A ROS-facing backend that advances the local SO-101 MuJoCo model."""

    def __init__(self, *, fps: int = 30) -> None:
        self.env = SO101MujocoEnv(
            obs_type="state",
            render_mode=None,
            fps=fps,
            max_episode_steps=2**31 - 1,
            terminate_on_success=False,
            cube_xy_randomization=0.0,
        )
        observation, _ = self.env.reset(seed=0)
        self._positions = lerobot_state_to_ros_positions(observation["agent_pos"])

    def read_positions(self) -> np.ndarray:
        return self._positions.copy()

    def command_positions(self, positions: Sequence[float] | np.ndarray) -> np.ndarray:
        action = ros_positions_to_lerobot_action(JOINT_NAMES, positions)
        observation, _, _, _, _ = self.env.step(action)
        self._positions = lerobot_state_to_ros_positions(observation["agent_pos"])
        return self.read_positions()

    def reset(self) -> np.ndarray:
        observation, _ = self.env.reset(seed=0)
        self._positions = lerobot_state_to_ros_positions(observation["agent_pos"])
        return self.read_positions()

    def close(self) -> None:
        self.env.close()


class SO101FollowerROSBackend:
    """A ROS-facing adapter around LeRobot's maintained SO101Follower API."""

    def __init__(
        self,
        *,
        port: str,
        robot_id: str,
        max_relative_target: float = 5.0,
        calibrate: bool = True,
    ) -> None:
        if not port:
            raise ValueError("A non-empty follower port is required for the hardware backend")
        if max_relative_target <= 0:
            raise ValueError("max_relative_target must be positive")

        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        config = SO101FollowerConfig(
            port=port,
            id=robot_id,
            use_degrees=True,
            max_relative_target=max_relative_target,
        )
        self.robot = SO101Follower(config)
        self.robot.connect(calibrate=calibrate)

    def read_positions(self) -> np.ndarray:
        return lerobot_mapping_to_ros_positions(self.robot.get_observation())

    def command_positions(self, positions: Sequence[float] | np.ndarray) -> np.ndarray:
        action = ros_positions_to_lerobot_action(JOINT_NAMES, positions)
        command = {f"{name}.pos": float(value) for name, value in zip(JOINT_NAMES, action, strict=True)}
        self.robot.send_action(command)
        return self.read_positions()

    def close(self) -> None:
        if self.robot.is_connected:
            self.robot.disconnect()
