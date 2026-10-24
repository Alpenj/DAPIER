"""Command-line entry point for Phase 0 data work."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from shoe_sorting_data.act_interchange import export_act_interchange, verify_act_interchange
from shoe_sorting_data.index import build_index, query_index
from shoe_sorting_data.lerobot_v3_encoder import (
    build_native_encoder_plan,
    encode_native_lerobot_v3,
    native_dependency_status,
)
from shoe_sorting_data.native_act_smoke import run_native_act_smoke
from shoe_sorting_data.offline_evaluator import build_offline_evaluator_fixture, evaluate_action_chunks
from shoe_sorting_data.rollout_safety import run_rollout_safety_smoke
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
    generate.add_argument("--camera-payload", action="store_true", help="write small lossless RGB-D raw fixtures")
    generate.add_argument("--camera-width", type=int, default=8)
    generate.add_argument("--camera-height", type=int, default=6)

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

    act_export = commands.add_parser("act-export", help="export a verified ACT interchange without source mutation")
    act_export.add_argument("--root", type=Path, required=True)
    act_export.add_argument("--output", type=Path, required=True)

    act_verify = commands.add_parser("act-verify", help="verify ACT interchange output hashes")
    act_verify.add_argument("--root", type=Path, required=True)

    commands.add_parser("native-status", help="report optional LeRobot v3 encoder dependencies")

    native_preflight = commands.add_parser("native-preflight", help="validate raw episodes for native v3 export")
    native_preflight.add_argument("--root", type=Path, required=True)
    native_preflight.add_argument("--depth-unit", choices=("mm", "m"), required=True)

    native_export = commands.add_parser("native-export", help="encode finalized episodes as derived LeRobot v3")
    native_export.add_argument("--root", type=Path, required=True)
    native_export.add_argument("--output", type=Path, required=True)
    native_export.add_argument("--repo-id", required=True)
    native_export.add_argument("--depth-unit", choices=("mm", "m"), required=True)

    native_smoke = commands.add_parser(
        "native-act-smoke",
        help="reopen native v3 data and verify ACT temporal/padding contracts",
    )
    native_smoke.add_argument("--root", type=Path, required=True)
    native_smoke.add_argument("--repo-id", required=True)
    native_smoke.add_argument("--chunk-size", type=int, default=3)

    offline_fixture = commands.add_parser("offline-eval-fixture", help="create a deterministic evaluator fixture")
    offline_fixture.add_argument("--root", type=Path, required=True)
    offline_fixture.add_argument("--padded-prediction", type=float, default=999.0)

    offline_eval = commands.add_parser("offline-eval", help="evaluate held-out ACT action chunks")
    offline_eval.add_argument("--manifest", type=Path, required=True)
    offline_eval.add_argument("--output", type=Path, required=True)
    offline_eval.add_argument("--inspection-top-k", type=int, default=3)

    rollout_smoke = commands.add_parser(
        "rollout-safety-smoke",
        help="run policy-independent safety mutations through a dry-run JDcobot ROS2 adapter",
    )
    rollout_smoke.add_argument("--output", type=Path, required=True)
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
                include_camera_payload=args.camera_payload,
                camera_width=args.camera_width,
                camera_height=args.camera_height,
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
        if args.command == "act-export":
            report = export_act_interchange(args.root, args.output)
            print(json.dumps(report.to_dict(), indent=2))
            return 0
        if args.command == "act-verify":
            report = verify_act_interchange(args.root)
            print(json.dumps(report, indent=2))
            return 0 if report["passed"] else 1
        if args.command == "native-status":
            print(json.dumps(native_dependency_status(), indent=2))
            return 0
        if args.command == "native-preflight":
            plan = build_native_encoder_plan(args.root, depth_unit=args.depth_unit)
            print(json.dumps(plan.to_dict(), indent=2))
            return 0
        if args.command == "native-export":
            receipt = encode_native_lerobot_v3(
                args.root,
                args.output,
                repo_id=args.repo_id,
                depth_unit=args.depth_unit,
            )
            print(
                json.dumps(
                    {
                        "published": receipt["published"],
                        "output": str(args.output),
                        "episode_count": receipt["plan"]["episode_count"],
                        "frame_count": receipt["plan"]["frame_count"],
                        "round_trip": receipt["round_trip"],
                    },
                    indent=2,
                )
            )
            return 0
        if args.command == "native-act-smoke":
            receipt = run_native_act_smoke(
                args.root,
                repo_id=args.repo_id,
                chunk_size=args.chunk_size,
            )
            print(json.dumps(receipt, indent=2))
            return 0 if receipt["status"] == "PASS" else 1
        if args.command == "offline-eval-fixture":
            manifest_path = build_offline_evaluator_fixture(
                args.root,
                padded_prediction=args.padded_prediction,
            )
            print(json.dumps({"manifest": str(manifest_path), "synthetic_fixture_only": True}, indent=2))
            return 0
        if args.command == "offline-eval":
            report = evaluate_action_chunks(
                args.manifest,
                args.output,
                inspection_top_k=args.inspection_top_k,
            )
            print(
                json.dumps(
                    {
                        "status": report["status"],
                        "output": str(args.output),
                        "record_count": report["input"]["record_count"],
                        "valid_timestep_count": report["mask_accounting"]["valid_timestep_count"],
                        "masked_timestep_count": report["mask_accounting"]["masked_timestep_count"],
                        "closed_loop_status": report["closed_loop_metrics"]["status"],
                    },
                    indent=2,
                )
            )
            return 0
        if args.command == "rollout-safety-smoke":
            report = run_rollout_safety_smoke(args.output)
            print(
                json.dumps(
                    {
                        "status": report["status"],
                        "output": str(args.output),
                        "scenario_count": report["scenario_count"],
                        "safety_pass_count": report["safety_pass_count"],
                        "reject_count": report["reject_count"],
                        "published_command_count": report["published_command_count"],
                        "hardware_dispatch_authorized_count": report["hardware_dispatch_authorized_count"],
                    },
                    indent=2,
                )
            )
            return 0
    except (OSError, RuntimeError, ValueError) as error:
        print(json.dumps({"error": str(error)}, indent=2))
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
