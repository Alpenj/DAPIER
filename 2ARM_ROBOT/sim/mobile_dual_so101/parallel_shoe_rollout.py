#!/usr/bin/env python3
"""Process-parallel MuJoCo rollouts for the simulation-only shoe task."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
import json
import math
import multiprocessing
import os
import time
from typing import Sequence

from mobile_dual_so101 import ACTION_NAMES, actuator_targets_from_qpos
from shoe_task import OBSERVATION_NAMES, ShoeTaskConfig, ShoeTaskEnv


@dataclass(frozen=True)
class ParallelRolloutConfig:
    workers: int = 4
    episodes: int = 8
    steps_per_episode: int = 100

    def validate(self) -> None:
        if self.workers <= 0:
            raise ValueError("workers must be positive")
        if self.episodes <= 0:
            raise ValueError("episodes must be positive")
        if self.steps_per_episode <= 0:
            raise ValueError("steps_per_episode must be positive")


def _rollout_worker(
    worker_id: int,
    episode_count: int,
    steps_per_episode: int,
    shoe_config: ShoeTaskConfig,
) -> dict[str, object]:
    env = ShoeTaskEnv(shoe_config)
    transitions = 0
    observation_checksum = 0.0
    finite = True
    hardware_execution = False
    final_shoe_y = 0.0
    for episode_index in range(episode_count):
        observation, reset_info = env.reset(
            seed=worker_id * 100_000 + episode_index
        )
        hardware_execution = hardware_execution or bool(
            reset_info["hardware_execution"]
        )
        hold = actuator_targets_from_qpos(env.model, env.data.qpos)
        for _ in range(steps_per_episode):
            observation, _, _, _, info = env.step(hold)
            hardware_execution = hardware_execution or bool(
                info["hardware_execution"]
            )
            vector = tuple(float(value) for value in observation["vector"])
            finite = finite and all(math.isfinite(value) for value in vector)
            observation_checksum += sum(vector)
            transitions += 1
        final_shoe_y = float(observation["shoe"]["position_map_m"][1])
    return {
        "worker_id": worker_id,
        "pid": os.getpid(),
        "episodes": episode_count,
        "transitions": transitions,
        "observation_checksum": observation_checksum,
        "finite_observations": finite,
        "hardware_execution": hardware_execution,
        "final_shoe_y_m": final_shoe_y,
        "nq": env.model.nq,
        "nv": env.model.nv,
        "nu": env.model.nu,
    }


def run_parallel_rollouts(
    config: ParallelRolloutConfig,
    *,
    shoe_config: ShoeTaskConfig | None = None,
) -> dict[str, object]:
    config.validate()
    base_config = shoe_config or ShoeTaskConfig()
    base_config.validate()
    worker_count = min(config.workers, config.episodes)
    episode_counts = [config.episodes // worker_count] * worker_count
    for index in range(config.episodes % worker_count):
        episode_counts[index] += 1

    jobs = []
    for worker_id, episode_count in enumerate(episode_counts):
        y_offset = (worker_id - (worker_count - 1) / 2.0) * 0.01
        x, y, z = base_config.shoe_position_m
        worker_config = replace(
            base_config,
            shoe_position_m=(x, y + y_offset, z),
        )
        jobs.append((worker_id, episode_count, worker_config))

    started = time.perf_counter()
    context = multiprocessing.get_context("spawn")
    with ProcessPoolExecutor(
        max_workers=worker_count,
        mp_context=context,
    ) as executor:
        futures = [
            executor.submit(
                _rollout_worker,
                worker_id,
                episode_count,
                config.steps_per_episode,
                worker_config,
            )
            for worker_id, episode_count, worker_config in jobs
        ]
        workers = [future.result() for future in futures]
    wall_seconds = time.perf_counter() - started

    transitions = sum(int(worker["transitions"]) for worker in workers)
    pids = {int(worker["pid"]) for worker in workers}
    finite = all(bool(worker["finite_observations"]) for worker in workers)
    hardware_execution = any(
        bool(worker["hardware_execution"]) for worker in workers
    )
    if len(pids) != worker_count:
        raise RuntimeError("parallel rollouts did not use independent processes")
    if not finite:
        raise RuntimeError("parallel rollout produced a non-finite observation")
    if hardware_execution:
        raise RuntimeError("parallel simulation reported hardware execution")
    return {
        "parallel_backend": "spawned_processes",
        "workers_requested": config.workers,
        "workers_used": worker_count,
        "worker_pids": sorted(pids),
        "episodes": config.episodes,
        "steps_per_episode": config.steps_per_episode,
        "transitions": transitions,
        "wall_seconds": wall_seconds,
        "transitions_per_second": transitions / wall_seconds,
        "independent_processes": True,
        "finite_observations": finite,
        "hardware_execution": hardware_execution,
        "mount_layout": base_config.mount_layout,
        "observation_dimension": len(OBSERVATION_NAMES),
        "action_dimension": len(ACTION_NAMES),
        "workers": workers,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--episodes", type=int, default=8)
    parser.add_argument("--steps", type=int, default=100)
    args = parser.parse_args(argv)
    report = run_parallel_rollouts(
        ParallelRolloutConfig(
            workers=args.workers,
            episodes=args.episodes,
            steps_per_episode=args.steps,
        )
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
