#!/usr/bin/env python3
"""MuJoCo sensor observation adapter for the strict sim-to-real policy boundary.

This module deliberately sanitizes simulator state before policy use. It keeps
only portable proprioception, adds explicit sensor provenance, and never falls
back to MuJoCo object truth when RGB-D detection fails.
"""

from __future__ import annotations

from pathlib import Path
import sys
from typing import Mapping


PROJECT_DIR = Path(__file__).resolve().parent
RESEARCH_SRC = PROJECT_DIR.parents[1] / "research" / "src"
if str(RESEARCH_SRC) not in sys.path:
    sys.path.insert(0, str(RESEARCH_SRC))

from dapier_research.observation_contract import (  # noqa: E402
    attach_sensor_runtime_provenance,
    validate_observation_for_use,
)
from dapier_research.sim_to_real_policy import SensorPolicyExecutor  # noqa: E402
from vision_guided_reach import (  # noqa: E402
    RenderedRgbdFrame,
    SimBlueShoeDetector,
    build_vision_policy_observation,
)


SIM_TO_REAL_OBSERVATION_ADAPTER_VERSION = (
    "dapier.mujoco-sim-to-real-observation-adapter.v1"
)
_PORTABLE_ROBOT_KEYS = (
    "left_arm_rad",
    "left_gripper_normalized",
    "right_arm_rad",
    "right_gripper_normalized",
    "base_velocity",
)


def portable_robot_state(robot_state: Mapping[str, object]) -> dict[str, object]:
    """Remove simulator-only world pose fields from policy proprioception."""

    if not isinstance(robot_state, Mapping):
        raise ValueError("robot_state must be an object")
    required = _PORTABLE_ROBOT_KEYS[:-1]
    missing = [key for key in required if key not in robot_state]
    if missing:
        raise ValueError(f"robot_state is missing portable fields: {missing}")
    result = {key: robot_state[key] for key in required}
    result["base_velocity"] = robot_state.get("base_velocity", (0.0, 0.0))
    return result


def build_sim_to_real_policy_observation(
    frame: RenderedRgbdFrame,
    *,
    robot_state: Mapping[str, object],
    detector: SimBlueShoeDetector | None = None,
) -> dict[str, object]:
    """Build one policy observation with explicit, sensor-only provenance."""

    observation = build_vision_policy_observation(
        frame,
        robot_state=portable_robot_state(robot_state),
        detector=detector,
    )
    observation["adapter_version"] = SIM_TO_REAL_OBSERVATION_ADAPTER_VERSION
    observation["sim_to_real_observation_compatible"] = True
    observation = attach_sensor_runtime_provenance(
        observation,
        producer="mujoco_rgbd_policy_adapter_v1",
        sensor_frames=(
            frame.camera_frame,
            "left_joint_state",
            "right_joint_state",
            "base_twist",
        ),
    )
    validate_observation_for_use(observation, use="policy_runtime")
    return observation


__all__ = [
    "SIM_TO_REAL_OBSERVATION_ADAPTER_VERSION",
    "SensorPolicyExecutor",
    "build_sim_to_real_policy_observation",
    "portable_robot_state",
]
