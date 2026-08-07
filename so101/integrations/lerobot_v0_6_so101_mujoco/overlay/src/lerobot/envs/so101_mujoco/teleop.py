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
from typing import Any

import numpy as np

from .env import ACTION_HIGH, ACTION_LOW, DEFAULT_HOME_ACTION, JOINT_NAMES


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

    def jog(self, direction: int) -> np.ndarray:
        if direction not in {-1, 1}:
            raise ValueError(f"direction must be -1 or 1, got {direction}")
        step = self.gripper_step_percent if self.selected_joint == 5 else self.joint_step_degrees
        self._action[self.selected_joint] += direction * step
        self._action = np.clip(self._action, ACTION_LOW, ACTION_HIGH).astype(np.float32)
        return self.get_action()

    def reset(self) -> np.ndarray:
        self._action = self.home_action.copy()
        return self.get_action()

    def get_action(self) -> np.ndarray:
        return self._action.copy()


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
