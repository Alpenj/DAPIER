#!/usr/bin/env python3
"""Copy verified LeRobot evaluation metrics into the wrist-student sidecar."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from lerobot.envs.so101_mujoco import mark_wrist_vla_training_evaluated


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--student-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--evaluation-output", type=Path, required=True)
    parser.add_argument("--training-updates", type=int, required=True)
    parser.add_argument("--batch-size", type=int, required=True)
    parser.add_argument("--dataset-episodes", type=int, required=True)
    parser.add_argument("--dataset-frames", type=int, required=True)
    parser.add_argument("--training-seed-start", type=int, required=True)
    parser.add_argument("--training-seed-end", type=int, required=True)
    parser.add_argument("--evaluation-seed-start", type=int, required=True)
    parser.add_argument("--success-threshold", type=float, default=0.8)
    return parser.parse_args()


def _load_evaluation_metrics(evaluation_output: Path) -> dict:
    eval_info_path = evaluation_output / "eval_info.json"
    payload = json.loads(eval_info_path.read_text(encoding="utf-8"))
    overall = payload.get("overall")
    per_task = payload.get("per_task")
    if not isinstance(overall, dict) or not isinstance(per_task, list) or not per_task:
        raise ValueError("eval_info.json is missing overall or per_task metrics")
    successes: list[bool] = []
    for task in per_task:
        task_successes = task.get("metrics", {}).get("successes")
        if not isinstance(task_successes, list) or not all(
            isinstance(value, bool) for value in task_successes
        ):
            raise ValueError("eval_info.json contains invalid success values")
        successes.extend(task_successes)
    episodes = overall.get("n_episodes")
    if (
        isinstance(episodes, bool)
        or not isinstance(episodes, int)
        or episodes != len(successes)
    ):
        raise ValueError("overall episode count does not match per-task success values")
    successful_episodes = sum(successes)
    expected_percent = 100 * successful_episodes / episodes
    if not math.isclose(
        float(overall.get("pc_success")), expected_percent, abs_tol=1e-9
    ):
        raise ValueError(
            "overall success percentage does not match per-task success values"
        )
    return {
        "evaluation_episodes": episodes,
        "successful_episodes": successful_episodes,
        "average_max_reward": float(overall["avg_max_reward"]),
        "average_sum_reward": float(overall["avg_sum_reward"]),
    }


def main() -> int:
    args = parse_args()
    metrics = _load_evaluation_metrics(args.evaluation_output)
    sidecar_path = mark_wrist_vla_training_evaluated(
        args.student_root,
        checkpoint_path=args.checkpoint,
        evaluation_output_path=args.evaluation_output,
        training_updates=args.training_updates,
        batch_size=args.batch_size,
        dataset_episodes=args.dataset_episodes,
        dataset_frames=args.dataset_frames,
        training_seed_start=args.training_seed_start,
        training_seed_end=args.training_seed_end,
        evaluation_seed_start=args.evaluation_seed_start,
        success_threshold=args.success_threshold,
        **metrics,
    )
    print(sidecar_path)
    print(
        json.dumps(
            json.loads(sidecar_path.read_text(encoding="utf-8"))[
                "vla_training_evaluation"
            ],
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
