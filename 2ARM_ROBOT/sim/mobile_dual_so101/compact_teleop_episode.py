#!/usr/bin/env python3
"""Replay compact-box teleop controls into a training-gated DAPIER episode."""

from __future__ import annotations

from contextlib import closing
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Sequence

import mujoco
import numpy as np

from box_shoe_scene import BOX_GEOM_NAMES
from collision_guard import check_bimanual_path
from compact_mobile_dual_so101 import create_compact_mobile_data
from mobile_dual_so101 import ACTION_NAMES, actuator_targets_from_qpos
from mujoco_mission_adapters import MuJoCoMultiCameraAdapter
from sim_episode import (
    CAMERA_STREAMS,
    SHOE_DATA_SOURCE,
    _split_policy_action,
    _write_samples,
    write_sim_camera_sample,
)
from sim_policy import actuator_targets_to_policy_action


if str(SHOE_DATA_SOURCE) not in sys.path:
    sys.path.insert(0, str(SHOE_DATA_SOURCE))

from shoe_sorting_data.contract import (  # noqa: E402
    build_manifest,
    save_manifest,
    validate_manifest,
)
from shoe_sorting_data.quality import validate_episode  # noqa: E402


def _split_state(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, list[float]]:
    action = actuator_targets_to_policy_action(
        model, actuator_targets_from_qpos(model, data.qpos)
    )
    return _split_policy_action(action)


def _active_box_geoms(model: mujoco.MjModel) -> tuple[str, ...]:
    active = []
    for name in BOX_GEOM_NAMES:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id >= 0 and int(model.geom_contype[geom_id]) != 0:
            active.append(name)
    return tuple(active)


def record_compact_pregrasp_episode(
    output_dir: str | Path,
    *,
    episode_id: str,
    model: mujoco.MjModel,
    actions_rad: Sequence[Sequence[float]],
    target_base_m: Sequence[float],
    fps: int = 20,
    width: int = 320,
    height: int = 240,
    success_tolerance_m: float = 0.015,
) -> Path:
    """Record one replay; only a stable terminal pre-grasp becomes accepted."""

    actions = tuple(tuple(float(value) for value in row) for row in actions_rad)
    target = np.asarray(target_base_m, dtype=np.float64)
    if len(actions) < 3 or any(
        len(row) != len(ACTION_NAMES) or not all(map(math.isfinite, row))
        for row in actions
    ):
        raise ValueError("teleop replay needs at least three finite 12-axis actions")
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("pre-grasp target must contain three finite values")
    if isinstance(fps, bool) or not isinstance(fps, int) or fps <= 0:
        raise ValueError("fps must be a positive integer")
    if width <= 0 or height <= 0 or not 0.0 < success_tolerance_m <= 0.03:
        raise ValueError("image dimensions and success tolerance are invalid")

    output = Path(output_dir).expanduser().resolve()
    if output.exists():
        raise ValueError(f"output directory already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.staging-", dir=output.parent))
    try:
        manifest = _record(
            staging,
            episode_id,
            model,
            actions,
            target,
            fps,
            width,
            height,
            success_tolerance_m,
        )
        os.replace(staging, output)
        return output / manifest.name
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def _record(
    root: Path,
    episode_id: str,
    model: mujoco.MjModel,
    actions: tuple[tuple[float, ...], ...],
    target: np.ndarray,
    fps: int,
    width: int,
    height: int,
    success_tolerance_m: float,
) -> Path:
    data = create_compact_mobile_data(model)
    steps_per_sample = round(1.0 / (fps * float(model.opt.timestep)))
    if steps_per_sample <= 0:
        raise ValueError("fps is faster than the MuJoCo timestep")
    period_ns = round(steps_per_sample * float(model.opt.timestep) * 1e9)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "right_gripperframe")
    if site_id < 0:
        raise ValueError("right gripper frame is missing")
    obstacles = _active_box_geoms(model)
    samples = []
    terminal_distances = []
    last_requested = tuple(actuator_targets_from_qpos(model, data.qpos))

    with closing(
        MuJoCoMultiCameraAdapter(model, data, width=width, height=height)
    ) as camera:
        for index, requested in enumerate(actions):
            timestamp_ns = round(float(data.time) * 1e9)
            guard = check_bimanual_path(
                model,
                last_requested,
                requested,
                required_clearance_m=0.005,
                obstacle_geom_names=obstacles,
            )
            if not guard.safe:
                raise ValueError(
                    f"teleop replay sample {index} collision rejected: "
                    f"{guard.reason}: {guard.first_body} vs {guard.second_body}"
                )
            cameras, camera_times, camera_receipts = write_sim_camera_sample(
                root, camera, index
            )
            stream_times = {
                name: timestamp_ns
                for name in (
                    "left_joint_state",
                    "right_joint_state",
                    "left_joint_action",
                    "right_joint_action",
                    "base_velocity",
                    "base_command",
                )
            }
            stream_times.update(camera_times)
            stream_receipts = dict(stream_times)
            stream_receipts.update(camera_receipts)
            samples.append(
                {
                    "timestamp_ns": timestamp_ns,
                    "timing": {
                        "anchor_timestamp_ns": timestamp_ns,
                        "sync_delta_ns": 0,
                        "stream_timestamps_ns": stream_times,
                        "stream_received_monotonic_ns": stream_receipts,
                    },
                    "state": _split_state(model, data),
                    "action": _split_policy_action(
                        actuator_targets_to_policy_action(model, requested)
                    ),
                    "cameras": cameras,
                    "simulation": {
                        "hardware_execution": False,
                        "skill": "right_front_flap_pregrasp",
                        "collision_guard": guard.as_report(),
                    },
                }
            )
            data.ctrl[:] = requested
            for _ in range(steps_per_sample):
                mujoco.mj_step(model, data)
            last_requested = requested
            terminal_distances.append(float(np.linalg.norm(data.site_xpos[site_id] - target)))

    stable_success = all(
        distance <= success_tolerance_m for distance in terminal_distances[-3:]
    )
    digest = _write_samples(root / "samples.jsonl", samples)
    manifest = build_manifest(
        episode_id=episode_id,
        sample_count=len(samples),
        samples_sha256=digest,
        operator_id="mujoco_keyboard_teleop",
        session_id=f"{episode_id}_session",
        shoe_pair_id="sim_box_front_flap",
        source_split="train",
        outcome_status="accepted" if stable_success else "recorded",
        success=stable_success,
        synthetic=True,
        object_instance_id="sim_measured_shoe_box",
        background_id="mujoco_compact_workcell_v1",
        fixture_id="compact_dual_so101_v1",
        recording_span_id=f"{episode_id}_span",
        attempt_id=f"{episode_id}_attempt",
        camera_payload_mode="required",
        camera_streams=CAMERA_STREAMS,
    )
    manifest["task"] = {
        "name": "shoe_box_opening",
        "skill": "right_front_flap_pregrasp",
        "shoe_pair_id": "sim_box_front_flap",
        "language_instruction": "Move the right gripper to the front box flap pre-grasp.",
        "base_motion_allowed": False,
    }
    manifest["robot"] = {
        "platform": "SO101_dual_arm_on_turtlebot3_waffle_pi",
        "robot_config_version": "mujoco_compact_dual_so101_v1",
        "controller_version": "mujoco_keyboard_teleop_v1",
        "calibration_version": "simulation_so101_new_calib_v1",
    }
    manifest["recording"]["expected_period_ns"] = period_ns
    manifest["provenance"]["pipeline_version"] = "compact_teleop_replay_v1"
    manifest["simulation_validation"] = {
        "terminal_distance_m": terminal_distances[-1],
        "terminal_stable_sample_count": 3,
        "success_tolerance_m": success_tolerance_m,
        "hardware_execution": False,
    }
    validate_manifest(manifest)
    manifest_path = root / "episode_manifest.json"
    save_manifest(manifest_path, manifest)
    if stable_success and not validate_episode(manifest_path).usable:
        raise RuntimeError("accepted teleop episode failed the existing quality gate")
    return manifest_path


__all__ = ["record_compact_pregrasp_episode"]
