#!/usr/bin/env python3
"""Project 4: verify or serve TensorBoard logs produced by Project 3."""

import argparse
import subprocess
from pathlib import Path

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--logdir", type=Path, default=Path(__file__).parent / "artifacts/tensorboard"
    )
    parser.add_argument(
        "--serve",
        action="store_true",
        help="Start TensorBoard after validating all runs",
    )
    parser.add_argument("--port", type=int, default=6006)
    args = parser.parse_args()
    run_dirs = (
        sorted(path for path in args.logdir.iterdir() if path.is_dir())
        if args.logdir.is_dir()
        else []
    )
    if not run_dirs:
        raise FileNotFoundError(
            "No TensorBoard runs found. Run project3_ablation.py first."
        )
    for run_dir in run_dirs:
        accumulator = EventAccumulator(str(run_dir))
        accumulator.Reload()
        tags = accumulator.Tags()
        print(
            f"{run_dir.name}: scalars={tags['scalars']} histograms={tags['histograms']}"
        )
    if args.serve:
        subprocess.run(
            ["tensorboard", "--logdir", str(args.logdir), "--port", str(args.port)],
            check=True,
        )


if __name__ == "__main__":
    main()
