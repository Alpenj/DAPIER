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
from shoe_task import OBSERVATION_NAMES, ShoeTaskConfig, ShoeTaskEnv, UnsafeActionError
from sim_policy import (
    ActionChunkExecutor,
    EXECUTION_MODES,
    HoldChunkPolicy,
    actuator_targets_to_policy_action,
    policy_action_to_actuator_targets,
)


@dataclass(frozen=True)
class ParallelRolloutConfig:
    workers: int = 4
    episodes: int = 8
    steps_per_episode: int = 100
    execution_mode: str = "receding_horizon"
    chunk_size: int = 4
    n_action_steps: int = 1

    def validate(self) -> None:
        if self.workers <= 0:
            raise ValueError("workers must be positive")
        if self.episodes <= 0:
            raise ValueError("episodes must be positive")
        if self.steps_per_episode <= 0:
            raise ValueError("steps_per_episode must be positive")
        if self.execution_mode not in EXECUTION_MODES:
            raise ValueError(f"execution_mode must be one of {EXECUTION_MODES}")
        if self.chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        if self.n_action_steps <= 0 or self.n_action_steps > self.chunk_size:
            raise ValueError("n_action_steps must be in [1, chunk_size]")


def _rollout_worker(
    worker_id: int,
    episode_count: int,
    steps_per_episode: int,
    shoe_config: ShoeTaskConfig,
    execution_mode: str,
    chunk_size: int,
    n_action_steps: int,
) -> dict[str, object]:
    env = ShoeTaskEnv(shoe_config)
    transitions = 0
    observation_checksum = 0.0
    finite = True
    hardware_execution = False
    final_shoe_y = 0.0
    policy_queries = 0
    unsafe_rejections = 0
    for episode_index in range(episode_count):
        observation, reset_info = env.reset(
            seed=worker_id * 100_000 + episode_index
        )
        hardware_execution = hardware_execution or bool(
            reset_info["hardware_execution"]
        )
        hold = actuator_targets_to_policy_action(
            env.model,
            actuator_targets_from_qpos(env.model, env.data.qpos),
        )
        executor = ActionChunkExecutor(
            HoldChunkPolicy(hold, chunk_size=chunk_size),
            mode=execution_mode,
            n_action_steps=n_action_steps,
        )
        executor.reset()
        for _ in range(steps_per_episode):
            policy_action = executor.next_action(observation)
            actuator_targets = policy_action_to_actuator_targets(
                env.model, policy_action
            )
            try:
                observation, _, _, _, info = env.step(actuator_targets)
            except UnsafeActionError as error:
                unsafe_rejections += 1
                hardware_execution = hardware_execution or bool(
                    error.assessment.hardware_execution
                )
                break
            hardware_execution = hardware_execution or bool(
                info["hardware_execution"]
            )
            vector = tuple(float(value) for value in observation["vector"])
            finite = finite and all(math.isfinite(value) for value in vector)
            observation_checksum += sum(vector)
            transitions += 1
        policy_queries += executor.policy_queries
        final_shoe_y = float(observation["shoe"]["position_map_m"][1])
    return {
        "worker_id": worker_id,
        "pid": os.getpid(),
        "episodes": episode_count,
        "transitions": transitions,
        "policy_queries": policy_queries,
        "unsafe_rejections": unsafe_rejections,
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
                config.execution_mode,
                config.chunk_size,
                config.n_action_steps,
            )
            for worker_id, episode_count, worker_config in jobs
        ]
        workers = [future.result() for future in futures]
    wall_seconds = time.perf_counter() - started

    transitions = sum(int(worker["transitions"]) for worker in workers)
    policy_queries = sum(int(worker["policy_queries"]) for worker in workers)
    unsafe_rejections = sum(int(worker["unsafe_rejections"]) for worker in workers)
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
        "policy_adapter": "hold_chunk_fixture",
        "execution_mode": config.execution_mode,
        "chunk_size": config.chunk_size,
        "n_action_steps": config.n_action_steps,
        "policy_queries": policy_queries,
        "unsafe_rejections": unsafe_rejections,
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
    parser.add_argument(
        "--execution-mode",
        choices=EXECUTION_MODES,
        default="receding_horizon",
    )
    parser.add_argument("--chunk-size", type=int, default=4)
    parser.add_argument("--n-action-steps", type=int, default=1)
    args = parser.parse_args(argv)
    report = run_parallel_rollouts(
        ParallelRolloutConfig(
            workers=args.workers,
            episodes=args.episodes,
            steps_per_episode=args.steps,
            execution_mode=args.execution_mode,
            chunk_size=args.chunk_size,
            n_action_steps=args.n_action_steps,
        )
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
