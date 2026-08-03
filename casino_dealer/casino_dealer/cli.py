"""Command-line entry points for inspecting deterministic dealer plans."""

from __future__ import annotations

import argparse
import json
from typing import Sequence

from casino_dealer.planner import (
    MAX_PLAYER_COUNT,
    MIN_PLAYER_COUNT,
    build_blackjack_opening_plan,
)


def player_count_argument(value: str) -> int:
    """Parse a bounded player count for argparse."""
    try:
        player_count = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            'players must be an integer'
        ) from error
    if not MIN_PLAYER_COUNT <= player_count <= MAX_PLAYER_COUNT:
        raise argparse.ArgumentTypeError(
            f'players must be between '
            f'{MIN_PLAYER_COUNT} and {MAX_PLAYER_COUNT}'
        )
    return player_count


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Generate a DAPIER blackjack opening deal plan.',
    )
    parser.add_argument(
        '--players',
        type=player_count_argument,
        default=2,
        help='number of player seats, from 1 to 7 (default: 2)',
    )
    parser.add_argument(
        '--compact',
        action='store_true',
        help='print compact JSON instead of indented JSON',
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    plan = build_blackjack_opening_plan(args.players)
    indent = None if args.compact else 2
    print(json.dumps(plan.to_dict(), indent=indent))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
