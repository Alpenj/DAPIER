"""CLI for the non-physical CardBench one-card kinematic baseline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from casino_dealer.card_sim import run_one_card_baseline


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the deterministic one-card bimanual kinematic baseline. "
            "This command does not open cameras, serial ports, or actuators."
        )
    )
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.output.exists():
        raise FileExistsError(
            f"refusing to replace existing receipt: {args.output}"
        )
    receipt = run_one_card_baseline(args.episodes, args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(receipt["summary"], sort_keys=True))
    return 0 if receipt["summary"]["passed"] == args.episodes else 1


if __name__ == "__main__":
    raise SystemExit(main())
