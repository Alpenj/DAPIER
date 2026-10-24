"""Typed skill exemplars derived from accepted, quality-gated robot episodes."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

from shoe_sorting_data.contract import load_manifest
from shoe_sorting_data.quality import validate_episode


SKILL_EXEMPLAR_SCHEMA_VERSION = "dapier.skill-exemplar.v0.1"
REQUIRED_LEAKAGE_KEYS = (
    "object_instance_id",
    "session_id",
    "background_id",
    "fixture_id",
    "recording_span_id",
)
ERROR_LEAKAGE_KEYS = {"object_instance_id", "session_id", "recording_span_id"}
WARNING_LEAKAGE_KEYS = {"background_id", "fixture_id"}
PLACEHOLDER_SUFFIXES = ("_unknown", "_pending")


def _require_text(parent: Mapping[str, Any], key: str) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return value


def _text_list(value: Any, label: str) -> list[str]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{label} must be an array of strings")
    result = []
    for item in value:
        if not isinstance(item, str) or not item.strip():
            raise ValueError(f"{label} entries must be non-empty strings")
        result.append(item)
    if len(result) != len(set(result)):
        raise ValueError(f"{label} entries must be unique")
    return result


def _canonical_digest(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def episode_contract_fingerprint(manifest: Mapping[str, Any]) -> str:
    """Fingerprint observation/action compatibility, not episode content."""
    recording = manifest["recording"]
    robot = manifest["robot"]
    value = {
        "episode_schema_version": manifest["schema_version"],
        "platform": robot["platform"],
        "robot_config_version": robot["robot_config_version"],
        "calibration_version": robot["calibration_version"],
        "clock": recording["clock"],
        "camera_streams": recording["camera_streams"],
        "state_streams": recording["state_streams"],
        "action_streams": recording["action_streams"],
    }
    return _canonical_digest(value)


def _leakage_keys(manifest: Mapping[str, Any]) -> dict[str, str]:
    provenance = manifest["provenance"]
    result: dict[str, str] = {}
    for key in REQUIRED_LEAKAGE_KEYS:
        value = provenance.get(key)
        if not isinstance(value, str) or not value.strip() or value.lower().endswith(PLACEHOLDER_SUFFIXES):
            raise ValueError(f"provenance.{key} must be resolved before exemplar registration")
        result[key] = value
    return result


def build_skill_exemplar(
    manifest_path: str | Path,
    *,
    exemplar_id: str,
    skill_id: str | None = None,
    preconditions: Sequence[str],
    postconditions: Sequence[str],
    timeout_ms: int,
    tags: Sequence[str] = (),
    parameters: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Create a retrieval artifact only from an accepted usable episode."""
    path = Path(manifest_path)
    report = validate_episode(path)
    if not report.usable:
        codes = sorted({issue.code for issue in report.errors})
        raise ValueError(f"source episode is not usable: {codes}")
    manifest = load_manifest(path)
    source_split = manifest["provenance"]["source_split"]
    if source_split not in {"train", "exemplar"}:
        raise ValueError("skill exemplars may only be registered from train or exemplar source_split")
    if not isinstance(exemplar_id, str) or not exemplar_id.strip():
        raise ValueError("exemplar_id must be a non-empty string")
    resolved_skill = skill_id or manifest["task"]["skill"]
    if not isinstance(resolved_skill, str) or not resolved_skill.strip():
        raise ValueError("skill_id must be a non-empty string")
    if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or timeout_ms <= 0:
        raise ValueError("timeout_ms must be a positive integer")
    precondition_list = _text_list(preconditions, "preconditions")
    postcondition_list = _text_list(postconditions, "postconditions")
    if not precondition_list or not postcondition_list:
        raise ValueError("preconditions and postconditions must not be empty")
    tag_list = sorted(_text_list(tags, "tags"))
    leakage = _leakage_keys(manifest)
    duration_seconds = report.duration_ns / 1_000_000_000
    return {
        "schema_version": SKILL_EXEMPLAR_SCHEMA_VERSION,
        "exemplar_id": exemplar_id,
        "source": {
            "episode_id": manifest["episode_id"],
            "samples_sha256": manifest["checksums"]["samples_sha256"],
            "duration_ns": report.duration_ns,
            "reference_duration_class": (
                "short_3_to_12_seconds" if 3.0 <= duration_seconds <= 12.0 else "outside_3_to_12_second_reference"
            ),
        },
        "skill": {
            "skill_id": resolved_skill,
            "preconditions": precondition_list,
            "postconditions": postcondition_list,
            "timeout_ms": timeout_ms,
            "tags": tag_list,
            "parameters": dict(parameters or {}),
        },
        "compatibility": {
            "episode_schema_version": manifest["schema_version"],
            "platform": manifest["robot"]["platform"],
            "robot_config_version": manifest["robot"]["robot_config_version"],
            "calibration_version": manifest["robot"]["calibration_version"],
            "contract_fingerprint": episode_contract_fingerprint(manifest),
        },
        "leakage_keys": leakage,
        "source_split": source_split,
        "outcome": {"status": "accepted", "success": True},
        "execution_policy": {
            "retrieval_only": True,
            "control_authorized": False,
            "requires_safety_supervisor": True,
        },
    }


def validate_skill_exemplar(exemplar: Mapping[str, Any]) -> None:
    if exemplar.get("schema_version") != SKILL_EXEMPLAR_SCHEMA_VERSION:
        raise ValueError(f"unsupported skill exemplar schema: {exemplar.get('schema_version')!r}")
    _require_text(exemplar, "exemplar_id")
    source = exemplar.get("source")
    if not isinstance(source, Mapping):
        raise ValueError("source must be an object")
    for key in ("episode_id", "samples_sha256", "reference_duration_class"):
        _require_text(source, key)
    duration = source.get("duration_ns")
    if isinstance(duration, bool) or not isinstance(duration, int) or duration <= 0:
        raise ValueError("source.duration_ns must be a positive integer")
    skill = exemplar.get("skill")
    if not isinstance(skill, Mapping):
        raise ValueError("skill must be an object")
    _require_text(skill, "skill_id")
    if not _text_list(skill.get("preconditions"), "preconditions"):
        raise ValueError("skill.preconditions must not be empty")
    if not _text_list(skill.get("postconditions"), "postconditions"):
        raise ValueError("skill.postconditions must not be empty")
    _text_list(skill.get("tags"), "tags")
    timeout = skill.get("timeout_ms")
    if isinstance(timeout, bool) or not isinstance(timeout, int) or timeout <= 0:
        raise ValueError("skill.timeout_ms must be a positive integer")
    if not isinstance(skill.get("parameters"), Mapping):
        raise ValueError("skill.parameters must be an object")
    try:
        json.dumps(skill["parameters"], ensure_ascii=False)
    except (TypeError, ValueError) as error:
        raise ValueError("skill.parameters must be JSON-serializable") from error
    compatibility = exemplar.get("compatibility")
    if not isinstance(compatibility, Mapping):
        raise ValueError("compatibility must be an object")
    for key in (
        "episode_schema_version",
        "platform",
        "robot_config_version",
        "calibration_version",
        "contract_fingerprint",
    ):
        _require_text(compatibility, key)
    leakage = exemplar.get("leakage_keys")
    if not isinstance(leakage, Mapping):
        raise ValueError("leakage_keys must be an object")
    for key in REQUIRED_LEAKAGE_KEYS:
        _require_text(leakage, key)
    _require_text(exemplar, "source_split")
    outcome = exemplar.get("outcome")
    if outcome != {"status": "accepted", "success": True}:
        raise ValueError("skill exemplar outcome must be accepted and successful")
    policy = exemplar.get("execution_policy")
    expected_policy = {
        "retrieval_only": True,
        "control_authorized": False,
        "requires_safety_supervisor": True,
    }
    if policy != expected_policy:
        raise ValueError("execution_policy must preserve retrieval-only safety boundaries")


def load_skill_exemplar(path: str | Path) -> dict[str, Any]:
    exemplar_path = Path(path)
    try:
        value = json.loads(exemplar_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"skill exemplar not found: {exemplar_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid skill exemplar JSON: {error}") from error
    if not isinstance(value, Mapping):
        raise ValueError("skill exemplar top level must be an object")
    validate_skill_exemplar(value)
    return dict(value)


def save_skill_exemplar(path: str | Path, exemplar: Mapping[str, Any]) -> None:
    validate_skill_exemplar(exemplar)
    exemplar_path = Path(path)
    exemplar_path.parent.mkdir(parents=True, exist_ok=True)
    exemplar_path.write_text(
        json.dumps(exemplar, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def retrieve_skill_exemplars(
    root: str | Path,
    manifest_path: str | Path,
    *,
    skill_id: str,
    tags: Sequence[str] = (),
    limit: int = 5,
    exclude_same_object: bool = True,
) -> list[dict[str, Any]]:
    """Return compatible exemplars ranked by tag overlap."""
    if isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0:
        raise ValueError("limit must be a positive integer")
    if not isinstance(skill_id, str) or not skill_id.strip():
        raise ValueError("skill_id must be a non-empty string")
    requested_tags = set(_text_list(tags, "tags"))
    manifest = load_manifest(manifest_path)
    current_fingerprint = episode_contract_fingerprint(manifest)
    current_object = manifest["provenance"].get("object_instance_id")
    expected = {
        "episode_schema_version": manifest["schema_version"],
        "platform": manifest["robot"]["platform"],
        "robot_config_version": manifest["robot"]["robot_config_version"],
        "calibration_version": manifest["robot"]["calibration_version"],
        "contract_fingerprint": current_fingerprint,
    }
    results: list[dict[str, Any]] = []
    for path in sorted(Path(root).rglob("skill_exemplar.json")):
        exemplar = load_skill_exemplar(path)
        if exemplar["skill"]["skill_id"] != skill_id:
            continue
        if any(exemplar["compatibility"][key] != value for key, value in expected.items()):
            continue
        if exclude_same_object and current_object and exemplar["leakage_keys"]["object_instance_id"] == current_object:
            continue
        exemplar_tags = set(exemplar["skill"]["tags"])
        union = requested_tags | exemplar_tags
        score = len(requested_tags & exemplar_tags) / len(union) if union else 1.0
        results.append(
            {
                "exemplar_id": exemplar["exemplar_id"],
                "skill_id": skill_id,
                "score": score,
                "path": str(path),
                "source_episode_id": exemplar["source"]["episode_id"],
                "control_authorized": False,
            }
        )
    return sorted(results, key=lambda item: (-item["score"], item["exemplar_id"]))[:limit]


def _comparable(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip()) and not value.lower().endswith(PLACEHOLDER_SUFFIXES)


def audit_exemplar_leakage(exemplar_root: str | Path, evaluation_root: str | Path) -> dict[str, Any]:
    """Detect object/session/span leakage and background/fixture overlap."""
    exemplar_paths = sorted(Path(exemplar_root).rglob("skill_exemplar.json"))
    evaluation_paths = sorted(Path(evaluation_root).rglob("episode_manifest.json"))
    if not exemplar_paths:
        raise ValueError(f"no skill_exemplar.json files found below: {exemplar_root}")
    if not evaluation_paths:
        raise ValueError(f"no episode_manifest.json files found below: {evaluation_root}")
    issues: list[dict[str, Any]] = []
    for exemplar_path in exemplar_paths:
        exemplar = load_skill_exemplar(exemplar_path)
        for evaluation_path in evaluation_paths:
            manifest = load_manifest(evaluation_path)
            provenance = manifest["provenance"]
            for key in REQUIRED_LEAKAGE_KEYS:
                exemplar_value = exemplar["leakage_keys"][key]
                evaluation_value = provenance.get(key)
                if _comparable(exemplar_value) and exemplar_value == evaluation_value:
                    severity = "error" if key in ERROR_LEAKAGE_KEYS else "warning"
                    issues.append(
                        {
                            "severity": severity,
                            "key": key,
                            "value": exemplar_value,
                            "exemplar_id": exemplar["exemplar_id"],
                            "evaluation_episode_id": manifest["episode_id"],
                            "evaluation_manifest": str(evaluation_path),
                        }
                    )
    error_count = sum(issue["severity"] == "error" for issue in issues)
    warning_count = sum(issue["severity"] == "warning" for issue in issues)
    return {
        "passed": error_count == 0,
        "exemplar_count": len(exemplar_paths),
        "evaluation_episode_count": len(evaluation_paths),
        "error_count": error_count,
        "warning_count": warning_count,
        "issues": issues,
    }
