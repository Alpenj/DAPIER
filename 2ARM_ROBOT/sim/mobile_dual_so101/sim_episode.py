#!/usr/bin/env python3
"""Record deterministic MuJoCo RGB-D episodes in the DAPIER data contract."""

from __future__ import annotations

import argparse
from contextlib import closing
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Callable, Mapping, Sequence

import mujoco

from mobile_dual_so101 import actuator_targets_from_qpos
from mission_modules.camera import CameraRole
from mujoco_mission_adapters import MuJoCoMultiCameraAdapter, build_mobile_shoe_mission_model
from shoe_task import ShoeTaskEnv, ground_truth_observation, task_metrics
from sim_policy import actuator_targets_to_policy_action


SHOE_DATA_SOURCE = Path(__file__).resolve().parents[2] / "src" / "shoe_sorting_data"
if str(SHOE_DATA_SOURCE) not in sys.path:
    sys.path.insert(0, str(SHOE_DATA_SOURCE))

from shoe_sorting_data.camera_payload import (  # noqa: E402
    CameraFramePayload,
    write_camera_payload,
)
from shoe_sorting_data.contract import (  # noqa: E402
    build_manifest,
    save_manifest,
    validate_manifest,
)


ActionSource = Callable[[int, Mapping[str, object]], Sequence[float]]
CAMERA_PREFIXES = {
    CameraRole.FRONT_RGBD: "front",
    CameraRole.WORKSPACE_RGBD: "workspace",
    CameraRole.LEFT_GRIPPER_RGB: "left_gripper",
    CameraRole.RIGHT_GRIPPER_RGB: "right_gripper",
}
CAMERA_STREAMS = (
    "front_rgb",
    "front_depth",
    "workspace_rgb",
    "workspace_depth",
    "left_gripper_rgb",
    "right_gripper_rgb",
)


@dataclass(frozen=True)
class SimEpisodeConfig:
    episode_id: str
    sample_count: int = 20
    fps: int = 20
    width: int = 64
    height: int = 48
    seed: int = 0
    source_split: str = "train"

    def validate(self) -> None:
        if not self.episode_id.strip():
            raise ValueError("episode_id must not be empty")
        if self.sample_count < 2:
            raise ValueError("sample_count must be at least 2")
        if self.fps <= 0 or self.width <= 0 or self.height <= 0:
            raise ValueError("fps, width, and height must be positive")
        if self.source_split not in {"train", "validation", "test"}:
            raise ValueError("source_split must be train, validation, or test")


def _split_policy_action(action: Sequence[float]) -> dict[str, list[float]]:
    if len(action) != 12:
        raise ValueError("policy action must contain 12 values")
    return {
        "left_arm": [float(value) for value in action[0:5]],
        "left_gripper": [float(action[5])],
        "right_arm": [float(value) for value in action[6:11]],
        "right_gripper": [float(action[11])],
        "base_velocity": [0.0, 0.0],
    }


def _state_streams(observation: Mapping[str, object]) -> dict[str, list[float]]:
    robot = observation["robot"]
    if not isinstance(robot, Mapping):
        raise ValueError("observation.robot must be an object")
    return {
        "left_arm": [float(value) for value in robot["left_arm_rad"]],
        "left_gripper": [float(robot["left_gripper_normalized"])],
        "right_arm": [float(value) for value in robot["right_arm_rad"]],
        "right_gripper": [float(robot["right_gripper_normalized"])],
        "base_velocity": [0.0, 0.0],
    }


def _continuous_carry_success(
    frame_carry_supported: Sequence[bool], terminal_success: bool
) -> bool:
    first_carry = next(
        (index for index, carried in enumerate(frame_carry_supported) if carried),
        None,
    )
    return (
        terminal_success
        and first_carry is not None
        and all(frame_carry_supported[first_carry:])
    )


def _write_samples(path: Path, samples: Sequence[Mapping[str, object]]) -> str:
    payload = "".join(
        json.dumps(sample, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        + "\n"
        for sample in samples
    ).encode("utf-8")
    try:
        with path.open("xb") as stream:
            stream.write(payload)
    except FileExistsError as error:
        raise ValueError(f"refusing to overwrite sample file: {path}") from error
    return hashlib.sha256(payload).hexdigest()


def write_sim_camera_sample(
    root: Path,
    camera: MuJoCoMultiCameraAdapter,
    index: int,
) -> tuple[dict[str, object], dict[str, int], dict[str, int]]:
    camera_records: dict[str, object] = {}
    camera_timestamps: dict[str, int] = {}
    camera_receipts: dict[str, int] = {}
    for frame in camera.capture().frames:
        prefix = CAMERA_PREFIXES[frame.role]
        streams = [(f"{prefix}_rgb", "rgb8", frame.width * 3, frame.rgb)]
        if frame.depth_m_le_f32 is not None:
            streams.append(
                (
                    f"{prefix}_depth",
                    "32FC1",
                    frame.width * 4,
                    frame.depth_m_le_f32,
                )
            )
        for stream, encoding, step, payload in streams:
            timestamp_ns = (
                frame.rgb_timestamp_ns
                if encoding == "rgb8"
                else int(frame.depth_timestamp_ns)
            )
            camera_records[stream] = {
                "timestamp_ns": timestamp_ns,
                "received_monotonic_ns": frame.received_monotonic_ns,
                "frame_id": frame.frame_id,
                "valid": frame.valid,
                "optical_frame": frame.optical_frame,
                "calibration_id": frame.calibration_id,
                "payload": write_camera_payload(
                    root,
                    stream,
                    index,
                    CameraFramePayload(
                        width=frame.width,
                        height=frame.height,
                        encoding=encoding,
                        is_bigendian=0,
                        step=step,
                        data=payload,
                    ),
                ),
            }
            camera_timestamps[stream] = timestamp_ns
            camera_receipts[stream] = frame.received_monotonic_ns
    return camera_records, camera_timestamps, camera_receipts


def record_sim_episode(
    output_dir: str | Path,
    config: SimEpisodeConfig,
    *,
    action_source: ActionSource | None = None,
) -> Path:
    """Record one simulation-only episode and return its manifest path."""

    config.validate()
    target = Path(output_dir).expanduser().resolve()
    if target.exists():
        raise ValueError(f"output directory already exists: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    root = Path(
        tempfile.mkdtemp(prefix=f".{target.name}.staging-", dir=target.parent)
    )
    try:
        return _record_sim_episode(root, target, config, action_source)
    except BaseException:
        shutil.rmtree(root, ignore_errors=True)
        raise


def _record_sim_episode(
    root: Path,
    target: Path,
    config: SimEpisodeConfig,
    action_source: ActionSource | None,
) -> Path:

    env = ShoeTaskEnv(model=build_mobile_shoe_mission_model())
    observation, reset_info = env.reset(seed=config.seed)
    if reset_info.get("hardware_execution") is not False:
        raise RuntimeError("simulation episode unexpectedly reported hardware execution")
    timestep_s = float(env.model.opt.timestep)
    steps_per_sample = round(1.0 / (config.fps * timestep_s))
    if steps_per_sample <= 0:
        raise ValueError("fps is faster than the MuJoCo physics timestep")
    period_ns = round(steps_per_sample * timestep_s * 1_000_000_000)
    requested_period_ns = round(1_000_000_000 / config.fps)
    if abs(period_ns - requested_period_ns) > max(1, requested_period_ns // 1000):
        raise ValueError("fps does not map to the MuJoCo timestep within 0.1 percent")

    initial_targets = tuple(actuator_targets_from_qpos(env.model, env.data.qpos))
    source = action_source or (lambda _index, _observation: initial_targets)
    samples: list[dict[str, object]] = []

    with closing(MuJoCoMultiCameraAdapter(
        env.model,
        env.data,
        height=config.height,
        width=config.width,
    )) as camera:
        for index in range(config.sample_count):
            requested = tuple(float(value) for value in source(index, observation))
            if len(requested) != env.model.nu or not all(
                math.isfinite(value) for value in requested
            ):
                raise ValueError("action source must return 12 finite actuator targets")
            for actuator_id, value in enumerate(requested):
                lower, upper = (
                    float(bound)
                    for bound in env.model.actuator_ctrlrange[actuator_id]
                )
                if not lower <= value <= upper:
                    raise ValueError(
                        f"action source target {actuator_id} is outside actuator range"
                    )
            timestamp_ns = round(float(env.data.time) * 1_000_000_000)
            frame_metrics = task_metrics(env.model, env.data)
            camera_records, camera_timestamps, camera_receipts = (
                write_sim_camera_sample(root, camera, index)
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
            stream_times.update(camera_timestamps)
            stream_receipts = dict(stream_times)
            stream_receipts.update(camera_receipts)
            policy_action = actuator_targets_to_policy_action(env.model, requested)
            samples.append(
                {
                    "timestamp_ns": timestamp_ns,
                    "timing": {
                        "anchor_timestamp_ns": timestamp_ns,
                        "sync_delta_ns": 0,
                        "stream_timestamps_ns": stream_times,
                        "stream_received_monotonic_ns": stream_receipts,
                    },
                    "state": _state_streams(observation),
                    "action": _split_policy_action(policy_action),
                    "cameras": camera_records,
                    "simulation": {
                        "hardware_execution": False,
                        "task_success": bool(frame_metrics["success"]),
                        "carry_supported": bool(frame_metrics["carry_supported"]),
                        "bilateral_gripper_contact": bool(
                            frame_metrics["bilateral_gripper_contact"]
                        ),
                        "shoe_gripper_attachment": bool(
                            frame_metrics["shoe_gripper_attachment"]
                        ),
                        "reward": float(frame_metrics["reward"]),
                    },
                }
            )
            _, assessment = env.apply_action(
                requested,
                physics_steps=steps_per_sample,
            )
            samples[-1]["simulation"]["collision_guard"] = assessment.as_report()
            observation = ground_truth_observation(env.model, env.data)

    digest = _write_samples(root / "samples.jsonl", samples)
    success = _continuous_carry_success(
        [bool(sample["simulation"]["carry_supported"]) for sample in samples],
        bool(samples[-1]["simulation"]["task_success"]),
    )
    manifest = build_manifest(
        episode_id=config.episode_id,
        sample_count=config.sample_count,
        samples_sha256=digest,
        arm_dof=5,
        gripper_dof=1,
        operator_id="mujoco_sim_recorder",
        session_id=f"mujoco_seed_{config.seed}",
        shoe_pair_id=f"sim_shoe_{config.seed}",
        source_split=config.source_split,
        outcome_status="accepted" if success else "recorded",
        success=success,
        synthetic=True,
        object_instance_id=f"sim_shoe_{config.seed}",
        background_id="mujoco_tower_scene_v1",
        fixture_id="mobile_dual_so101_tower_v1",
        recording_span_id=f"mujoco_span_{config.seed}",
        attempt_id=f"mujoco_attempt_{config.seed}",
        camera_payload_mode="required",
        camera_streams=CAMERA_STREAMS,
    )
    manifest["robot"] = {
        "platform": "SO101_dual_arm_on_turtlebot3_waffle_pi",
        "robot_config_version": "mujoco_mobile_dual_so101_v1",
        "controller_version": "mujoco_position_actuator_v1",
        "calibration_version": "simulation_so101_new_calib_v1",
    }
    manifest["recording"]["expected_period_ns"] = period_ns
    manifest["provenance"]["pipeline_version"] = "mujoco_episode_recorder_v1"
    validate_manifest(manifest)
    manifest_path = root / "episode_manifest.json"
    save_manifest(manifest_path, manifest)
    os.replace(root, target)
    finalized_manifest = target / "episode_manifest.json"
    return finalized_manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--episode-id", required=True)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=64)
    parser.add_argument("--height", type=int, default=48)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--split", choices=("train", "validation", "test"), default="train")
    args = parser.parse_args()
    manifest = record_sim_episode(
        args.output,
        SimEpisodeConfig(
            episode_id=args.episode_id,
            sample_count=args.samples,
            fps=args.fps,
            width=args.width,
            height=args.height,
            seed=args.seed,
            source_split=args.split,
        ),
    )
    print(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
