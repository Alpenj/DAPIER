#!/usr/bin/env python3
"""Project 2: inspect post-ReLU feature maps with removable hooks."""

import argparse
from pathlib import Path

import torch

from cifar10_common import prepare_output_dirs, resolve_device, run_p2


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-dir", type=Path, default=Path.home() / ".cache/dapier/cifar10"
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path(__file__).parent / "artifacts"
    )
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()
    device = resolve_device(args.device)
    print(f"device={device}")
    if device.type == "cuda":
        print(f"gpu={torch.cuda.get_device_name(device)}")
    run_p2(
        args.data_dir,
        prepare_output_dirs(args.output_dir.resolve()),
        args.batch_size,
        args.num_workers,
        args.seed,
        device,
    )


if __name__ == "__main__":
    main()
