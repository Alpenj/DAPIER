"""Command-line entry point for Phase 0 data work."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from shoe_sorting_data.index import build_index, query_index
from shoe_sorting_data.perception_exemplar import (
    add_perception_exemplar,
    build_perception_registry,
    load_perception_registry,
    match_perception_exemplar,
    save_perception_registry,
)
from shoe_sorting_data.quality import validate_episode
from shoe_sorting_data.skill_exemplar import (
    audit_exemplar_leakage,
    build_skill_exemplar,
    retrieve_skill_exemplars,
    save_skill_exemplar,
)
from shoe_sorting_data.synthetic import FAULTS, generate_dataset


def _boolean(value: str) -> bool:
    lowered = value.lower()
    if lowered in {"true", "1", "yes"}:
        return True
    if lowered in {"false", "0", "no"}:
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def _embedding(value: str) -> list[float]:
    try:
        result = [float(item.strip()) for item in value.split(",")]
    except ValueError as error:
        raise argparse.ArgumentTypeError("embedding must be comma-separated numbers") from error
    if not result:
        raise argparse.ArgumentTypeError("embedding must not be empty")
    return result


def _key_values(values: Sequence[str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for value in values:
        key, separator, item = value.partition("=")
        if not separator or not key.strip() or not item.strip():
            raise ValueError(f"expected key=value, received: {value!r}")
        if key in result:
            raise ValueError(f"duplicate key: {key}")
        result[key] = item
    return result


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
    generate.add_argument("--arm-dof", type=int, default=5)
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
    query.add_argument("--object-instance-id")
    query.add_argument("--session-id")
    query.add_argument("--background-id")
    query.add_argument("--limit", type=int, default=100)

    pair_add = commands.add_parser("pair-add", help="add a one-shot perception exemplar")
    pair_add.add_argument("--registry", type=Path, required=True)
    pair_add.add_argument("--embedding-model", default="mock_embedding_v0")
    pair_add.add_argument("--exemplar-id", required=True)
    pair_add.add_argument("--pair-id", required=True)
    pair_add.add_argument("--object-instance-id", required=True)
    pair_add.add_argument("--embedding", type=_embedding, required=True)
    pair_add.add_argument("--session-id", required=True)
    pair_add.add_argument("--background-id", required=True)
    pair_add.add_argument("--split", default="train")
    pair_add.add_argument("--feature", action="append", default=[])

    pair_match = commands.add_parser("pair-match", help="match a pair exemplar or abstain")
    pair_match.add_argument("--registry", type=Path, required=True)
    pair_match.add_argument("--embedding", type=_embedding, required=True)
    pair_match.add_argument("--min-similarity", type=float, default=0.8)
    pair_match.add_argument("--min-margin", type=float, default=0.05)

    skill_register = commands.add_parser(
        "skill-register", help="register an accepted episode as a typed skill exemplar"
    )
    skill_register.add_argument("--manifest", type=Path, required=True)
    skill_register.add_argument("--output", type=Path, required=True)
    skill_register.add_argument("--exemplar-id", required=True)
    skill_register.add_argument("--skill-id")
    skill_register.add_argument("--precondition", action="append", required=True)
    skill_register.add_argument("--postcondition", action="append", required=True)
    skill_register.add_argument("--timeout-ms", type=int, required=True)
    skill_register.add_argument("--tag", action="append", default=[])
    skill_register.add_argument("--parameter", action="append", default=[])

    skill_retrieve = commands.add_parser("skill-retrieve", help="retrieve compatible typed skill exemplars")
    skill_retrieve.add_argument("--root", type=Path, required=True)
    skill_retrieve.add_argument("--manifest", type=Path, required=True)
    skill_retrieve.add_argument("--skill-id", required=True)
    skill_retrieve.add_argument("--tag", action="append", default=[])
    skill_retrieve.add_argument("--limit", type=int, default=5)
    skill_retrieve.add_argument("--allow-same-object", action="store_true")

    leakage = commands.add_parser("exemplar-audit", help="audit exemplar/evaluation split leakage")
    leakage.add_argument("--exemplar-root", type=Path, required=True)
    leakage.add_argument("--evaluation-root", type=Path, required=True)
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
                object_instance_id=args.object_instance_id,
                session_id=args.session_id,
                background_id=args.background_id,
                limit=args.limit,
            )
            print(json.dumps(rows, indent=2))
            return 0
        if args.command == "pair-add":
            registry = (
                load_perception_registry(args.registry)
                if args.registry.is_file()
                else build_perception_registry(args.embedding_model)
            )
            registry = add_perception_exemplar(
                registry,
                exemplar_id=args.exemplar_id,
                pair_id=args.pair_id,
                object_instance_id=args.object_instance_id,
                embedding=args.embedding,
                session_id=args.session_id,
                background_id=args.background_id,
                source_split=args.split,
                features=_key_values(args.feature),
            )
            save_perception_registry(args.registry, registry)
            print(json.dumps({"saved": str(args.registry), "exemplar_count": len(registry["exemplars"])}, indent=2))
            return 0
        if args.command == "pair-match":
            result = match_perception_exemplar(
                load_perception_registry(args.registry),
                args.embedding,
                min_similarity=args.min_similarity,
                min_margin=args.min_margin,
            )
            print(json.dumps(result, indent=2))
            return 0 if result["decision"] == "match" else 1
        if args.command == "skill-register":
            exemplar = build_skill_exemplar(
                args.manifest,
                exemplar_id=args.exemplar_id,
                skill_id=args.skill_id,
                preconditions=args.precondition,
                postconditions=args.postcondition,
                timeout_ms=args.timeout_ms,
                tags=args.tag,
                parameters=_key_values(args.parameter),
            )
            save_skill_exemplar(args.output, exemplar)
            print(json.dumps({"saved": str(args.output), "exemplar_id": exemplar["exemplar_id"]}, indent=2))
            return 0
        if args.command == "skill-retrieve":
            results = retrieve_skill_exemplars(
                args.root,
                args.manifest,
                skill_id=args.skill_id,
                tags=args.tag,
                limit=args.limit,
                exclude_same_object=not args.allow_same_object,
            )
            print(json.dumps(results, indent=2))
            return 0
        if args.command == "exemplar-audit":
            report = audit_exemplar_leakage(args.exemplar_root, args.evaluation_root)
            print(json.dumps(report, indent=2))
            return 0 if report["passed"] else 1
    except (OSError, ValueError) as error:
        print(json.dumps({"error": str(error)}, indent=2))
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
