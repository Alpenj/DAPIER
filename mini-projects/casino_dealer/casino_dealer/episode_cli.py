"""Human-friendly commands for CardBench episode sidecar manifests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from casino_dealer.episode_manifest import (
    VALID_SOURCES,
    VALID_STATUSES,
    build_manifest,
    load_manifest,
    parse_arm_spec,
    save_manifest,
    validate_manifest,
)


DEFAULT_ARM_SPEC = "right,so101_follower_main,so101_leader_main"


def _manifest_episode_id(path: Path) -> str:
    """Use the episode directory name unless the caller supplies an ID."""
    return path.parent.name or path.stem


def _parse_success(value: str) -> bool:
    normalized = value.lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    raise argparse.ArgumentTypeError("success must be true or false")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create and validate DAPIER episode manifests."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    init = subparsers.add_parser(
        "init", help="create a manifest before or immediately after recording"
    )
    init.add_argument("--path", type=Path, required=True)
    init.add_argument("--episode-id", default=None)
    init.add_argument("--task", required=True)
    init.add_argument("--skill", required=True)
    init.add_argument("--source", choices=VALID_SOURCES, default="lerobot")
    init.add_argument("--fps", type=float, default=30.0)
    init.add_argument("--camera", action="append", dest="cameras")
    init.add_argument(
        "--arm-spec",
        action="append",
        help="name,follower_id,leader_id; repeat for a second arm",
    )
    init.add_argument("--calibration-ref", action="append", default=[])
    init.add_argument("--data-path", default="")
    init.add_argument("--notes", default="")
    init.add_argument("--overwrite", action="store_true")

    validate = subparsers.add_parser("validate", help="validate one manifest")
    validate.add_argument("--path", type=Path, required=True)
    validate.add_argument(
        "--check-data",
        action="store_true",
        help="also require recording.data_path to exist locally",
    )

    validate_tree = subparsers.add_parser(
        "validate-tree", help="validate every episode_manifest.json below a directory"
    )
    validate_tree.add_argument("--root", type=Path, required=True)

    mark = subparsers.add_parser(
        "mark", help="set the review result after watching an episode"
    )
    mark.add_argument("--path", type=Path, required=True)
    mark.add_argument("--status", choices=VALID_STATUSES[1:], required=True)
    mark.add_argument("--success", type=_parse_success, required=True)
    mark.add_argument("--failure-reason", default=None)
    mark.add_argument("--notes", default=None)

    return parser


def _run_init(args: argparse.Namespace) -> int:
    if args.path.exists() and not args.overwrite:
        raise ValueError(f"refusing to overwrite existing file: {args.path}")
    specs = args.arm_spec or [DEFAULT_ARM_SPEC]
    arms = [parse_arm_spec(spec) for spec in specs]
    manifest = build_manifest(
        episode_id=args.episode_id or _manifest_episode_id(args.path),
        task=args.task,
        skill=args.skill,
        source=args.source,
        fps=args.fps,
        cameras=args.cameras or ["front"],
        arms=arms,
        calibration_refs=args.calibration_ref,
        data_path=args.data_path,
        notes=args.notes,
    )
    save_manifest(args.path, manifest)
    print(f"MANIFEST CREATED: {args.path}")
    return 0


def _run_validate(args: argparse.Namespace) -> int:
    manifest = load_manifest(args.path)
    if args.check_data:
        data_path = manifest["recording"]["data_path"]
        if not data_path:
            raise ValueError("recording.data_path is empty")
        if not Path(data_path).exists():
            raise ValueError(f"recording.data_path does not exist: {data_path}")
    print(f"MANIFEST VALID: {args.path}")
    return 0


def _run_validate_tree(args: argparse.Namespace) -> int:
    paths = sorted(args.root.rglob("episode_manifest.json"))
    if not paths:
        raise ValueError(f"no episode_manifest.json files found below: {args.root}")
    errors: list[str] = []
    for path in paths:
        try:
            load_manifest(path)
        except ValueError as error:
            errors.append(f"{path}: {error}")
    if errors:
        print("MANIFESTS INVALID")
        for error in errors:
            print(f"- {error}")
        return 1
    print(f"MANIFESTS VALID: {len(paths)}")
    return 0


def _run_mark(args: argparse.Namespace) -> int:
    manifest = load_manifest(args.path)
    outcome = dict(manifest["outcome"])
    outcome["status"] = args.status
    outcome["success"] = args.success
    if args.failure_reason is not None:
        outcome["failure_reason"] = args.failure_reason
    if args.notes is not None:
        manifest["notes"] = args.notes
    manifest["outcome"] = outcome
    validate_manifest(manifest)
    save_manifest(args.path, manifest)
    print(f"MANIFEST UPDATED: {args.path}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "init":
            return _run_init(args)
        if args.command == "validate":
            return _run_validate(args)
        if args.command == "validate-tree":
            return _run_validate_tree(args)
        if args.command == "mark":
            return _run_mark(args)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}")
        return 1
    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
