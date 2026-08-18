#!/usr/bin/env python3
"""Project 3: run a fair BatchNorm/Dropout ablation study."""

import argparse
from pathlib import Path

from runtime_bootstrap import ensure_course_environment

ensure_course_environment()

import torch

from cifar10_common import (
    default_data_dir,
    prepare_output_dirs,
    resolve_device,
    run_p3_p4,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=default_data_dir())
    parser.add_argument(
        "--output-dir", type=Path, default=Path(__file__).parent / "artifacts"
    )
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()
    if args.epochs < 1:
        raise ValueError("epochs must be positive")
    device = resolve_device(args.device)
    print(f"device={device}")
    if device.type == "cuda":
        print(
            f"gpu={torch.cuda.get_device_name(device)} torch_cuda={torch.version.cuda}"
        )
    run_p3_p4(
        args.data_dir,
        prepare_output_dirs(args.output_dir.resolve()),
        args.epochs,
        args.batch_size,
        args.num_workers,
        args.seed,
        device,
    )


if __name__ == "__main__":
    main()
