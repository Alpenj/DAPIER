"""Command-line entry point for Phase 0 data work."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from shoe_sorting_data.index import build_index, query_index
from shoe_sorting_data.quality import validate_episode
from shoe_sorting_data.synthetic import FAULTS, generate_dataset


def _boolean(value: str) -> bool:
    lowered = value.lower()
    if lowered in {"true", "1", "yes"}:
        return True
    if lowered in {"false", "0", "no"}:
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="shoe_episode",
        description="Generate, validate, index, and query DYNA-lite shoe episodes.",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    generate = commands.add_parser("generate", help="create deterministic synthetic episodes")
    generate.add_argument("--root", type=Path, required=True)
    generate.add_argument("--count", type=int, default=20)
    generate.add_argument("--samples", type=int, default=40)
    generate.add_argument("--arm-dof", type=int, default=6)
    generate.add_argument("--gripper-dof", type=int, default=1)
    generate.add_argument("--seed", type=int, default=42)
    generate.add_argument("--fault", choices=sorted(FAULTS), default="none")

    validate = commands.add_parser("validate", help="validate one episode manifest")
    validate.add_argument("--manifest", type=Path, required=True)

    index = commands.add_parser("index", help="validate a tree and build a SQLite manifest")
    index.add_argument("--root", type=Path, required=True)
    index.add_argument("--db", type=Path, required=True)

    query = commands.add_parser("query", help="query a SQLite manifest")
    query.add_argument("--db", type=Path, required=True)
    query.add_argument("--usable", type=_boolean)
    query.add_argument("--success", type=_boolean)
    query.add_argument("--split")
    query.add_argument("--shoe-pair-id")
    query.add_argument("--limit", type=int, default=100)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "generate":
            paths = generate_dataset(
                args.root,
                count=args.count,
                sample_count=args.samples,
                arm_dof=args.arm_dof,
                gripper_dof=args.gripper_dof,
                seed=args.seed,
                fault=args.fault,
            )
            print(json.dumps({"generated": len(paths), "root": str(args.root)}, indent=2))
            return 0
        if args.command == "validate":
            report = validate_episode(args.manifest)
            print(json.dumps(report.to_dict(), indent=2))
            return 0 if report.usable else 1
        if args.command == "index":
            summary = build_index(args.root, args.db)
            print(json.dumps(summary, indent=2))
            return 0 if summary["usable"] == summary["indexed"] and summary["invalid_manifest"] == 0 else 1
        if args.command == "query":
            rows = query_index(
                args.db,
                usable=args.usable,
                source_split=args.split,
                success=args.success,
                shoe_pair_id=args.shoe_pair_id,
                limit=args.limit,
            )
            print(json.dumps(rows, indent=2))
            return 0
    except (OSError, ValueError) as error:
        print(json.dumps({"error": str(error)}, indent=2))
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
