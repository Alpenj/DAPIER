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

"""State and evidence helpers for human intervention in SO-101 VLA rollouts."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

import numpy as np

from .env import ACTION_HIGH, ACTION_LOW, JOINT_NAMES

INTERVENTION_SCHEMA_VERSION = "dapier.so101.vla-intervention.v1"
POLICY_AUTHORITY = "policy"
HUMAN_AUTHORITY = "human"


def _validated_action(action: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    action_array = np.asarray(action, dtype=np.float32)
    if action_array.shape != (6,):
        raise ValueError(f"action must have shape (6,), got {action_array.shape}")
    if not np.all(np.isfinite(action_array)):
        raise ValueError("action must contain only finite values")
    return np.clip(action_array, ACTION_LOW, ACTION_HIGH).astype(np.float32)


@dataclass(frozen=True)
class InterventionDecision:
    """One action and the controller that owned it."""

    action: np.ndarray
    source: str
    intervention_segment: int | None


class VLAInterventionSession:
    """Fail-explicit authority switch between a policy and a human controller."""

    def __init__(self) -> None:
        self.authority = POLICY_AUTHORITY
        self.intervention_segments = 0
        self.intervention_frames = 0
        self._human_action: np.ndarray | None = None

    @property
    def human_action(self) -> np.ndarray | None:
        return None if self._human_action is None else self._human_action.copy()

    def reset_episode(self) -> None:
        self.authority = POLICY_AUTHORITY
        self.intervention_segments = 0
        self.intervention_frames = 0
        self._human_action = None

    def take_over(self, current_action: np.ndarray | list[float] | tuple[float, ...]) -> bool:
        """Anchor manual control to measured state and return whether authority changed."""
        self._human_action = _validated_action(current_action)
        if self.authority == HUMAN_AUTHORITY:
            return False
        self.authority = HUMAN_AUTHORITY
        self.intervention_segments += 1
        return True

    def update_human_action(self, action: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        if self.authority != HUMAN_AUTHORITY:
            raise RuntimeError("human action can be updated only during an intervention")
        self._human_action = _validated_action(action)
        return self._human_action.copy()

    def resume_policy(self) -> bool:
        """Return whether policy authority was restored."""
        if self.authority == POLICY_AUTHORITY:
            return False
        self.authority = POLICY_AUTHORITY
        self._human_action = None
        return True

    def choose_action(
        self,
        *,
        policy_action: np.ndarray | list[float] | tuple[float, ...] | None,
        human_action: np.ndarray | list[float] | tuple[float, ...] | None = None,
    ) -> InterventionDecision:
        if self.authority == POLICY_AUTHORITY:
            if policy_action is None:
                raise ValueError("policy_action is required while policy owns control")
            return InterventionDecision(_validated_action(policy_action), POLICY_AUTHORITY, None)

        if human_action is not None:
            self.update_human_action(human_action)
        if self._human_action is None:
            raise RuntimeError("human intervention has no anchored action")
        self.intervention_frames += 1
        return InterventionDecision(self._human_action.copy(), HUMAN_AUTHORITY, self.intervention_segments)


class InterventionEpisodeRecorder:
    """Write crash-readable JSONL plus wrist RGB frames for one rollout at a time."""

    def __init__(self, root: Path) -> None:
        self.root = root.expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._events_file: TextIO | None = None
        self._episode_dir: Path | None = None
        self._episode_index: int | None = None
        self._frame_count = 0
        self._human_frame_count = 0

    def start_episode(self, *, episode_index: int, seed: int, task: str) -> Path:
        if self._events_file is not None:
            raise RuntimeError("finish the active episode before starting another")
        self._episode_index = int(episode_index)
        self._episode_dir = self.root / f"episode_{episode_index:04d}"
        (self._episode_dir / "wrist").mkdir(parents=True, exist_ok=True)
        self._events_file = (self._episode_dir / "events.jsonl").open("w", encoding="utf-8", buffering=1)
        self._frame_count = 0
        self._human_frame_count = 0
        metadata = {
            "schema_version": INTERVENTION_SCHEMA_VERSION,
            "episode_index": episode_index,
            "seed": int(seed),
            "task": task,
            "joint_names": list(JOINT_NAMES),
            "observation_camera": "wrist",
        }
        (self._episode_dir / "episode.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return self._episode_dir

    def record_frame(
        self,
        *,
        step_index: int,
        source: str,
        intervention_segment: int | None,
        observation_state: np.ndarray,
        wrist_rgb: np.ndarray,
        requested_action: np.ndarray,
        applied_action: np.ndarray,
        last_policy_action: np.ndarray | None,
        reward: float,
        success: bool,
        done: bool,
    ) -> None:
        if self._events_file is None or self._episode_dir is None or self._episode_index is None:
            raise RuntimeError("start_episode must be called before record_frame")
        if source not in {POLICY_AUTHORITY, HUMAN_AUTHORITY}:
            raise ValueError(f"unsupported action source: {source!r}")
        state = _validated_action(observation_state)
        requested = _validated_action(requested_action)
        applied = _validated_action(applied_action)
        image = np.asarray(wrist_rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("wrist_rgb must be an HxWx3 uint8 image")

        from PIL import Image

        image_relative = Path("wrist") / f"{step_index:06d}.png"
        Image.fromarray(image).save(self._episode_dir / image_relative)
        record: dict[str, Any] = {
            "schema_version": INTERVENTION_SCHEMA_VERSION,
            "episode_index": self._episode_index,
            "step_index": int(step_index),
            "timestamp_ns": int(round(step_index / 30 * 1_000_000_000)),
            "source": source,
            "intervention_segment": intervention_segment,
            "observation_state_lerobot": state.tolist(),
            "wrist_image": image_relative.as_posix(),
            "requested_action_lerobot": requested.tolist(),
            "applied_action_lerobot": applied.tolist(),
            "last_policy_action_lerobot": (
                None if last_policy_action is None else _validated_action(last_policy_action).tolist()
            ),
            "reward": float(reward),
            "success": bool(success),
            "done": bool(done),
        }
        self._events_file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._frame_count += 1
        self._human_frame_count += int(source == HUMAN_AUTHORITY)

    def finish_episode(self, *, success: bool, termination_reason: str, intervention_segments: int) -> Path:
        if self._events_file is None or self._episode_dir is None or self._episode_index is None:
            raise RuntimeError("no active episode to finish")
        self._events_file.close()
        self._events_file = None
        manifest = {
            "schema_version": INTERVENTION_SCHEMA_VERSION,
            "episode_index": self._episode_index,
            "frames": self._frame_count,
            "human_intervention_frames": self._human_frame_count,
            "intervention_segments": int(intervention_segments),
            "success": bool(success),
            "termination_reason": termination_reason,
            "training_status": "evidence_only_requires_dataset_conversion",
        }
        manifest_path = self._episode_dir / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self._episode_dir = None
        self._episode_index = None
        return manifest_path

    def close(self) -> None:
        if self._events_file is not None:
            self._events_file.close()
            self._events_file = None
