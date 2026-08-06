"""Create and execute only the SO-101 sim-first G0 environment gate."""

from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime, timezone
from hashlib import sha256
import json
from math import isclose
import os
from pathlib import Path
import secrets
import subprocess
import sys
from typing import Any, Sequence

from .embodiment import SO101_CHANNEL_NAMES, EmbodimentSpec, so101_new_calibration_spec
from .environment import collect_environment
from .protocols import Frame, FrameContractError, Leader, validate_frame

RECORD_ID = "DAPIER-2026-08-07-so101-g0"
MANIFEST_SCHEMA_VERSION = 1
CONTROL_PERIOD_NS = 33_333_333

# The five source facts fixed by the work contract.  G0 checks exact manifest
# values; it does not claim that all five repositories are checked out locally.
EXPECTED_SOURCE_REVISIONS = {
    "dapier_source_base": "0baa32ca7c5e4c16ab4d3797c7d803144f00ab95",
    "so_arm100": "7629d2ad9853d10fb903093a33ef6114099d97e5",
    "so101_nexus_0_5_1": "3619f7dce086445dc31311edd593a4de93b21c47",
    "so101_nexus_vendored_menagerie": "4c358ef9d9d7f32ca58b40b490884a0c1726a440",
    "aloha_sim": "d02904607cca1bf6dfb72f30b522506ac7ca0f91",
}


def _utc_now() -> str:
    return (
        datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")
    )


def _canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _digest_value(value: Any) -> str:
    return f"sha256:{sha256(_canonical_bytes(value)).hexdigest()}"


def _digest_file(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return f"sha256:{digest.hexdigest()}"


def _write_json_exclusive(path: Path, payload: Any) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o444)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(
                payload,
                handle,
                allow_nan=False,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            handle.write("\n")
    except BaseException:
        try:
            os.close(descriptor)
        except OSError:
            pass
        raise


def _read_json_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"expected a JSON object in {path}")
    return payload


def _git_revision(repo_root: Path) -> str:
    completed = subprocess.run(
        ["git", "-C", str(repo_root), "rev-parse", "HEAD"],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise ValueError(f"cannot resolve Git revision for {repo_root}")
    return completed.stdout.strip()


def _git_has_changes(repo_root: Path) -> bool:
    completed = subprocess.run(
        ["git", "-C", str(repo_root), "status", "--porcelain"],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise ValueError(f"cannot inspect Git status for {repo_root}")
    return bool(completed.stdout.strip())


def _embodiment_mapping(spec: EmbodimentSpec) -> dict[str, Any]:
    return {
        "embodiment_id": spec.embodiment_id,
        "embodiment_revision": spec.embodiment_revision,
        "channel_names": list(spec.channel_names),
        "action_units": list(spec.action_units),
        "sim_units": list(spec.sim_units),
        "calibration_id": spec.calibration_id,
        "sim_lower": list(spec.sim_lower),
        "sim_upper": list(spec.sim_upper),
        "action_bounds_digest": spec.bounds_digest(),
    }


def _manifest_input_payload(manifest: dict[str, Any]) -> dict[str, Any]:
    return {
        "gate": manifest["gate"],
        "seed": manifest["seed"],
        "implementation_revision": manifest["implementation"]["revision"],
        "source_revisions": manifest["source_revisions"],
        "embodiment": manifest["embodiment"],
        "model_sha256": manifest["model"]["sha256"],
        "calibration_sha256": manifest["model"]["calibration_sha256"],
    }


def initialize_g0_manifest(
    *,
    run_root: Path,
    repo_root: Path,
    model_path: Path,
    calibration_path: Path,
) -> Path:
    run_root = run_root.expanduser().resolve()
    repo_root = repo_root.expanduser().resolve(strict=True)
    model_path = model_path.expanduser().resolve(strict=True)
    calibration_path = calibration_path.expanduser().resolve(strict=True)

    if run_root.exists():
        raise ValueError(f"RUN_ROOT must not already exist: {run_root}")
    if run_root == repo_root or run_root.is_relative_to(repo_root):
        raise ValueError("RUN_ROOT must be outside the DAPIER repository")
    if not model_path.is_file() or not calibration_path.is_file():
        raise ValueError("model and calibration inputs must be regular files")

    calibration_digest = _digest_file(calibration_path)
    spec = so101_new_calibration_spec(calibration_digest)
    manifest: dict[str, Any] = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "record_id": RECORD_ID,
        "created_at_utc": _utc_now(),
        "run_id": run_root.name,
        "nonce": secrets.token_hex(16),
        "gate": "G0",
        "seed": 0,
        "repo_root": str(repo_root),
        "implementation": {
            "revision": _git_revision(repo_root),
            "worktree_has_changes_at_init": _git_has_changes(repo_root),
        },
        "source_revisions": dict(EXPECTED_SOURCE_REVISIONS),
        "source_revision_verification_mode": (
            "exact manifest match to the work contract; upstream checkout presence is not claimed"
        ),
        "embodiment": _embodiment_mapping(spec),
        "model": {
            "path": str(model_path),
            "filename": model_path.name,
            "sha256": _digest_file(model_path),
            "calibration_path": str(calibration_path),
            "calibration_filename": calibration_path.name,
            "calibration_sha256": calibration_digest,
            "provenance_boundary": (
                "existing local external asset; source revision is asserted by the work contract and file identity by SHA-256"
            ),
        },
    }
    manifest["input_digest"] = _digest_value(_manifest_input_payload(manifest))

    run_root.mkdir(parents=True, exist_ok=False)
    manifest_path = run_root / "run-manifest.json"
    _write_json_exclusive(manifest_path, manifest)
    return manifest_path


def _check_source_revisions(
    manifest: dict[str, Any],
) -> tuple[list[dict[str, Any]], int]:
    actual = manifest.get("source_revisions")
    checks: list[dict[str, Any]] = []
    passed = 0
    for name, expected in EXPECTED_SOURCE_REVISIONS.items():
        actual_value = actual.get(name) if isinstance(actual, dict) else None
        matches = actual_value == expected
        passed += int(matches)
        checks.append(
            {
                "name": name,
                "expected": expected,
                "manifest_value": actual_value,
                "passed": matches,
                "mode": "contract_manifest_exact_match",
            }
        )
    if isinstance(actual, dict):
        extra = sorted(set(actual) - set(EXPECTED_SOURCE_REVISIONS))
        if extra:
            checks.append(
                {"name": "unexpected_entries", "values": extra, "passed": False}
            )
    return checks, passed


def _frame_contract_checks(spec: EmbodimentSpec) -> dict[str, Any]:
    now_ns = 1_000_000_000
    previous = Frame(
        embodiment_id=spec.embodiment_id,
        embodiment_revision=spec.embodiment_revision,
        channel_names=spec.channel_names,
        values=tuple(
            (low + high) / 2
            for low, high in zip(spec.action_lower, spec.action_upper, strict=True)
        ),
        units=spec.action_units,
        calibration_id=spec.calibration_id,
        monotonic_timestamp_ns=now_ns - CONTROL_PERIOD_NS,
        sequence_id=10,
        source="scripted",
    )
    current = replace(previous, sequence_id=11)
    boundary_age = replace(
        previous, monotonic_timestamp_ns=now_ns - 2 * CONTROL_PERIOD_NS, sequence_id=12
    )

    details: list[dict[str, Any]] = []
    violations = 0
    accepted = 0
    for label, frame, prior in (
        ("valid_current", current, previous),
        ("age_equal_to_2T", boundary_age, None),
    ):
        try:
            validate_frame(
                frame,
                spec=spec,
                now_ns=now_ns,
                control_period_ns=CONTROL_PERIOD_NS,
                previous=prior,
            )
        except FrameContractError as exc:
            violations += 1
            details.append({"check": label, "passed": False, "error": str(exc)})
        else:
            accepted += 1
            details.append({"check": label, "passed": True})

    invalid_frames = (
        ("embodiment_id", replace(current, embodiment_id="wrong"), previous),
        (
            "embodiment_revision",
            replace(current, embodiment_revision="wrong"),
            previous,
        ),
        (
            "calibration_id",
            replace(current, calibration_id="sha256:" + "0" * 64),
            previous,
        ),
        (
            "channel_order",
            replace(current, channel_names=tuple(reversed(current.channel_names))),
            previous,
        ),
        ("units", replace(current, units=("radian",) * 6), previous),
        ("sequence", replace(current, sequence_id=10), previous),
        (
            "timestamp_order",
            replace(
                current, monotonic_timestamp_ns=previous.monotonic_timestamp_ns - 1
            ),
            previous,
        ),
        (
            "stale_age",
            replace(current, monotonic_timestamp_ns=now_ns - 2 * CONTROL_PERIOD_NS - 1),
            None,
        ),
        ("future_timestamp", replace(current, monotonic_timestamp_ns=now_ns + 1), None),
        ("source", replace(current, source="unknown"), previous),
        (
            "finite_values",
            replace(current, values=(float("nan"),) + current.values[1:]),
            previous,
        ),
        ("value_width", replace(current, values=current.values[:-1]), previous),
    )

    rejected = 0
    for label, frame, prior in invalid_frames:
        try:
            validate_frame(
                frame,
                spec=spec,
                now_ns=now_ns,
                control_period_ns=CONTROL_PERIOD_NS,
                previous=prior,
            )
        except FrameContractError:
            rejected += 1
            details.append({"check": f"reject_{label}", "passed": True})
        else:
            violations += 1
            details.append(
                {
                    "check": f"reject_{label}",
                    "passed": False,
                    "error": "invalid frame accepted",
                }
            )

    schema_payload = current.to_mapping()
    schema_payload["unexpected"] = True
    try:
        Frame.from_mapping(schema_payload)
    except FrameContractError:
        rejected += 1
        details.append({"check": "reject_schema_extra_field", "passed": True})
    else:
        violations += 1
        details.append(
            {
                "check": "reject_schema_extra_field",
                "passed": False,
                "error": "invalid schema accepted",
            }
        )

    class FakeLeader:
        def connect(self) -> None:
            return None

        def disconnect(self) -> None:
            return None

        def get_action(self) -> Frame:
            return current

    leader_protocol_passed = isinstance(FakeLeader(), Leader)
    if not leader_protocol_passed:
        violations += 1
    details.append(
        {"check": "leader_protocol_structure", "passed": leader_protocol_passed}
    )

    return {
        "accepted_checks_passed": accepted,
        "accepted_checks_total": 2,
        "rejection_checks_passed": rejected,
        "rejection_checks_total": len(invalid_frames) + 1,
        "leader_protocol_passed": leader_protocol_passed,
        "violation_count": violations,
        "details": details,
    }


def _load_and_check_model(model_path: Path, spec: EmbodimentSpec) -> dict[str, Any]:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path))
    expected_set = set(spec.channel_names)
    model_joint_order = tuple(
        name
        for index in range(model.njnt)
        if (name := mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index))
        in expected_set
    )
    model_actuator_order = tuple(
        name
        for index in range(model.nu)
        if (name := mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index))
        in expected_set
    )

    channel_checks: list[dict[str, Any]] = []
    channel_passed = 0
    actual_ranges: list[tuple[float, float] | None] = []
    for index, name in enumerate(spec.channel_names):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        actual_range: tuple[float, float] | None = None
        if joint_id >= 0:
            actual_range = (
                float(model.jnt_range[joint_id][0]),
                float(model.jnt_range[joint_id][1]),
            )
        actual_ranges.append(actual_range)
        range_matches = bool(
            actual_range
            and isclose(
                actual_range[0], spec.sim_lower[index], rel_tol=0.0, abs_tol=1e-10
            )
            and isclose(
                actual_range[1], spec.sim_upper[index], rel_tol=0.0, abs_tol=1e-10
            )
        )
        passed = (
            joint_id >= 0
            and actuator_id >= 0
            and model_joint_order == spec.channel_names
            and model_actuator_order == spec.channel_names
            and range_matches
        )
        channel_passed += int(passed)
        channel_checks.append(
            {
                "channel": name,
                "joint_present": joint_id >= 0,
                "actuator_present": actuator_id >= 0,
                "range": list(actual_range) if actual_range else None,
                "range_matches": range_matches,
                "passed": passed,
            }
        )

    conversion_checks: list[dict[str, Any]] = []
    conversion_passed = 0
    midpoint = tuple(
        (low + high) / 2
        for low, high in zip(spec.action_lower, spec.action_upper, strict=True)
    )
    for index, name in enumerate(spec.channel_names):
        max_error = 0.0
        passed = True
        for value in (
            spec.action_lower[index],
            midpoint[index],
            spec.action_upper[index],
        ):
            action = list(midpoint)
            action[index] = value
            round_trip = spec.sim_to_action(spec.action_to_sim(action))
            error = abs(round_trip[index] - value)
            max_error = max(max_error, error)
            passed = passed and error <= 1e-9
        conversion_passed += int(passed)
        conversion_checks.append(
            {"channel": name, "max_round_trip_error": max_error, "passed": passed}
        )

    return {
        "loaded": True,
        "mujoco_version": getattr(mujoco, "__version__", None),
        "dimensions": {"nq": int(model.nq), "nv": int(model.nv), "nu": int(model.nu)},
        "joint_order": list(model_joint_order),
        "actuator_order": list(model_actuator_order),
        "channel_checks": channel_checks,
        "channel_passed": channel_passed,
        "channel_total": len(spec.channel_names),
        "conversion_checks": conversion_checks,
        "conversion_passed": conversion_passed,
        "conversion_total": len(spec.channel_names),
    }


def run_g0(*, manifest_path: Path, out_path: Path) -> tuple[str, Path]:
    manifest_path = manifest_path.expanduser().resolve(strict=True)
    if manifest_path.name != "run-manifest.json":
        raise ValueError("manifest filename must be run-manifest.json")
    run_root = manifest_path.parent
    out_path = out_path.expanduser().resolve()
    if out_path.parent != run_root or out_path.name != "G0":
        raise ValueError("G0 output must be exactly $RUN_ROOT/G0")
    existing_entries = list(run_root.iterdir())
    if len(existing_entries) != 1 or existing_entries[0] != manifest_path:
        raise ValueError(
            "RUN_ROOT contains an existing artifact or receipt; refusing reuse"
        )
    if out_path.exists():
        raise ValueError("G0 output already exists; refusing reuse")

    manifest = _read_json_object(manifest_path)
    repo_root = (
        Path(str(manifest.get("repo_root", ""))).expanduser().resolve(strict=True)
    )
    if run_root == repo_root or run_root.is_relative_to(repo_root):
        raise ValueError("RUN_ROOT must be outside the DAPIER repository")

    out_path.mkdir(mode=0o755)
    errors: list[str] = []
    revision_checks, revision_passed = _check_source_revisions(manifest)

    if manifest.get("schema_version") != MANIFEST_SCHEMA_VERSION:
        errors.append("manifest schema_version mismatch")
    if manifest.get("record_id") != RECORD_ID:
        errors.append("manifest record_id mismatch")
    if manifest.get("gate") != "G0" or manifest.get("seed") != 0:
        errors.append("manifest must select G0 with seed 0")
    if revision_passed != len(EXPECTED_SOURCE_REVISIONS) or len(revision_checks) != len(
        EXPECTED_SOURCE_REVISIONS
    ):
        errors.append("one or more pinned source revisions mismatch")

    try:
        expected_input_digest = _digest_value(_manifest_input_payload(manifest))
    except (KeyError, TypeError) as exc:
        expected_input_digest = None
        errors.append(f"manifest input fields are incomplete: {exc}")
    if manifest.get("input_digest") != expected_input_digest:
        errors.append("manifest input_digest mismatch")

    implementation_revision = _git_revision(repo_root)
    if manifest.get("implementation", {}).get("revision") != implementation_revision:
        errors.append("DAPIER implementation revision changed after manifest creation")

    model_data = (
        manifest.get("model") if isinstance(manifest.get("model"), dict) else {}
    )
    calibration_id = model_data.get("calibration_sha256")
    try:
        spec = so101_new_calibration_spec(calibration_id)
    except (TypeError, ValueError) as exc:
        errors.append(f"invalid calibration identity: {exc}")
        spec = so101_new_calibration_spec("sha256:" + "0" * 64)

    expected_embodiment = _embodiment_mapping(spec)
    if manifest.get("embodiment") != expected_embodiment:
        errors.append("SO-101 embodiment manifest mismatch")

    model_path = Path(str(model_data.get("path", ""))).expanduser()
    calibration_path = Path(str(model_data.get("calibration_path", ""))).expanduser()
    model_identity_passed = False
    calibration_identity_passed = False
    if not model_path.is_file():
        errors.append("model file is missing")
    else:
        model_identity_passed = _digest_file(model_path) == model_data.get("sha256")
        if not model_identity_passed:
            errors.append("model file digest mismatch")
    if not calibration_path.is_file():
        errors.append("calibration file is missing")
    else:
        calibration_identity_passed = (
            _digest_file(calibration_path) == calibration_id == spec.calibration_id
        )
        if not calibration_identity_passed:
            errors.append("calibration identity mismatch")

    environment = collect_environment(repo_root)
    environment.update(
        {
            "schema_version": 1,
            "record_id": RECORD_ID,
            "gate": "G0",
            "captured_at_utc": _utc_now(),
        }
    )

    frame_checks = _frame_contract_checks(spec)
    try:
        model_checks = _load_and_check_model(model_path, spec)
    except (
        Exception
    ) as exc:  # MuJoCo reports model/compiler errors through several exception types.
        model_checks = {
            "loaded": False,
            "channel_passed": 0,
            "channel_total": len(SO101_CHANNEL_NAMES),
            "conversion_passed": 0,
            "conversion_total": len(SO101_CHANNEL_NAMES),
            "error": f"{type(exc).__name__}: {exc}",
        }
        errors.append(f"MuJoCo import/model load failed: {type(exc).__name__}: {exc}")

    if model_checks["channel_passed"] != len(SO101_CHANNEL_NAMES):
        errors.append("SO-101 channel/order/model-range check did not pass 6/6")
    if model_checks["conversion_passed"] != len(SO101_CHANNEL_NAMES):
        errors.append("unit conversion check did not pass 6/6")
    if frame_checks["violation_count"] != 0:
        errors.append("frame schema/rejection-rule verification has violations")

    pass_status = (
        not errors
        and model_checks["loaded"]
        and model_identity_passed
        and calibration_identity_passed
        and revision_passed == len(EXPECTED_SOURCE_REVISIONS)
        and model_checks["channel_passed"] == len(SO101_CHANNEL_NAMES)
        and model_checks["conversion_passed"] == len(SO101_CHANNEL_NAMES)
        and frame_checks["violation_count"] == 0
    )
    status = "PASS" if pass_status else "FAIL"

    contract = {
        "schema_version": 1,
        "record_id": RECORD_ID,
        "gate": "G0",
        "checked_at_utc": _utc_now(),
        "status": status,
        "revision_checks": revision_checks,
        "revision_verification_boundary": manifest.get(
            "source_revision_verification_mode"
        ),
        "implementation_revision": {
            "manifest": manifest.get("implementation", {}).get("revision"),
            "actual": implementation_revision,
            "passed": manifest.get("implementation", {}).get("revision")
            == implementation_revision,
        },
        "model": {
            "filename": model_data.get("filename"),
            "sha256": model_data.get("sha256"),
            "identity_passed": model_identity_passed,
            "calibration_filename": model_data.get("calibration_filename"),
            "calibration_id": calibration_id,
            "calibration_identity_passed": calibration_identity_passed,
            "provenance_boundary": model_data.get("provenance_boundary"),
            "load_checks": model_checks,
        },
        "frame_contract": frame_checks,
        "errors": errors,
        "claims_not_made": [
            "upstream checkout presence for all five declared revisions",
            "GUI or camera rendering",
            "recording or LeRobotDataset compatibility",
            "policy training or evaluation",
            "ROS 2 adapter compatibility",
            "serial access or physical hardware control",
            "sim-to-real success",
        ],
    }

    manifest_hash = _digest_file(manifest_path)
    receipt = {
        "schema_version": 1,
        "record_id": RECORD_ID,
        "gate": "G0",
        "run_id": manifest.get("run_id"),
        "nonce": secrets.token_hex(16),
        "created_at_utc": _utc_now(),
        "manifest_hash": manifest_hash,
        "input_hash": manifest.get("input_digest"),
        "metrics": {
            "revision": {
                "passed": revision_passed,
                "total": len(EXPECTED_SOURCE_REVISIONS),
            },
            "channel_order": {
                "passed": model_checks["channel_passed"],
                "total": len(SO101_CHANNEL_NAMES),
            },
            "unit_conversion": {
                "passed": model_checks["conversion_passed"],
                "total": len(SO101_CHANNEL_NAMES),
            },
            "calibration_identity": {
                "passed": int(calibration_identity_passed),
                "total": 1,
            },
            "model_load": {"passed": int(bool(model_checks["loaded"])), "total": 1},
            "schema_rejection_rule_violation_count": frame_checks["violation_count"],
        },
        "status": status,
        "errors": errors,
    }

    _write_json_exclusive(out_path / "environment.json", environment)
    _write_json_exclusive(out_path / "contract.json", contract)
    receipt_path = out_path / "receipt.json"
    _write_json_exclusive(receipt_path, receipt)
    return status, receipt_path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Initialize or execute the DAPIER SO-101 G0 environment smoke gate only."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    init = subparsers.add_parser(
        "init-g0", help="Create a new G0 RUN_ROOT and immutable manifest"
    )
    init.add_argument("--run-root", type=Path, required=True)
    init.add_argument("--repo", type=Path, default=Path.cwd())
    init.add_argument("--model", type=Path, required=True)
    init.add_argument("--calibration", type=Path, required=True)

    g0 = subparsers.add_parser(
        "g0", help="Run only G0 and write environment/contract/receipt JSON"
    )
    g0.add_argument("--manifest", type=Path, required=True)
    g0.add_argument("--out", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "init-g0":
            manifest_path = initialize_g0_manifest(
                run_root=args.run_root,
                repo_root=args.repo,
                model_path=args.model,
                calibration_path=args.calibration,
            )
            print(f"G0 manifest created: {manifest_path}")
            return 0
        if args.command == "g0":
            status, receipt_path = run_g0(
                manifest_path=args.manifest, out_path=args.out
            )
            print(f"G0 {status}: {receipt_path}")
            return 0 if status == "PASS" else 1
    except (OSError, ValueError) as exc:
        print(f"G0 refused: {exc}", file=sys.stderr)
        return 2
    raise AssertionError(f"unsupported command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
