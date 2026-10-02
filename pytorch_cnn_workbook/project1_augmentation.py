#!/usr/bin/env python3
"""Project 1: visualize CIFAR-10 augmentation and normalization."""

import argparse
from pathlib import Path

from runtime_bootstrap import ensure_course_environment

ensure_course_environment()

from cifar10_common import default_data_dir, prepare_output_dirs, run_p1


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=default_data_dir())
    parser.add_argument(
        "--output-dir", type=Path, default=Path(__file__).parent / "artifacts"
    )
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    run_p1(
        args.data_dir,
        prepare_output_dirs(args.output_dir.resolve()),
        args.batch_size,
        args.num_workers,
        args.seed,
    )


if __name__ == "__main__":
    main()
