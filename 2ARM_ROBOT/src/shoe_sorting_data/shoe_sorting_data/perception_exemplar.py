"""One-shot shoe-pair perception exemplars with explicit abstention.

This module matches embedding vectors only. It never authorizes robot actions and
must not be described as one-shot robot control or GEN-1.5 reproduction.
"""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


PERCEPTION_REGISTRY_SCHEMA_VERSION = "dapier.perception-exemplar-registry.v0.1"


def _require_text(parent: Mapping[str, Any], key: str) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return value


def _embedding(values: Any, *, expected_dimension: int | None = None) -> list[float]:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
        raise ValueError("embedding must be an array")
    if expected_dimension is not None and len(values) != expected_dimension:
        raise ValueError(f"embedding expected {expected_dimension} values but found {len(values)}")
    if not values:
        raise ValueError("embedding must not be empty")
    converted: list[float] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("embedding values must be finite numbers")
        converted.append(float(value))
    if math.sqrt(sum(value * value for value in converted)) == 0.0:
        raise ValueError("embedding norm must be greater than zero")
    return converted


def build_perception_registry(embedding_model: str) -> dict[str, Any]:
    if not isinstance(embedding_model, str) or not embedding_model.strip():
        raise ValueError("embedding_model must be a non-empty string")
    return {
        "schema_version": PERCEPTION_REGISTRY_SCHEMA_VERSION,
        "embedding_model": embedding_model,
        "embedding_dimension": None,
        "exemplars": [],
    }


def validate_perception_registry(registry: Mapping[str, Any]) -> None:
    if registry.get("schema_version") != PERCEPTION_REGISTRY_SCHEMA_VERSION:
        raise ValueError(f"unsupported perception registry schema: {registry.get('schema_version')!r}")
    _require_text(registry, "embedding_model")
    dimension = registry.get("embedding_dimension")
    if dimension is not None and (isinstance(dimension, bool) or not isinstance(dimension, int) or dimension <= 0):
        raise ValueError("embedding_dimension must be null or a positive integer")
    exemplars = registry.get("exemplars")
    if not isinstance(exemplars, list):
        raise ValueError("exemplars must be an array")
    if exemplars and dimension is None:
        raise ValueError("embedding_dimension is required when exemplars are present")
    seen_ids: set[str] = set()
    for index, exemplar in enumerate(exemplars):
        if not isinstance(exemplar, Mapping):
            raise ValueError(f"exemplars[{index}] must be an object")
        exemplar_id = _require_text(exemplar, "exemplar_id")
        if exemplar_id in seen_ids:
            raise ValueError(f"duplicate exemplar_id: {exemplar_id}")
        seen_ids.add(exemplar_id)
        for key in ("pair_id", "object_instance_id"):
            _require_text(exemplar, key)
        _embedding(exemplar.get("embedding"), expected_dimension=dimension)
        features = exemplar.get("features")
        if not isinstance(features, Mapping):
            raise ValueError(f"exemplars[{index}].features must be an object")
        try:
            json.dumps(features, ensure_ascii=False)
        except (TypeError, ValueError) as error:
            raise ValueError(f"exemplars[{index}].features must be JSON-serializable") from error
        provenance = exemplar.get("provenance")
        if not isinstance(provenance, Mapping):
            raise ValueError(f"exemplars[{index}].provenance must be an object")
        for key in ("session_id", "background_id", "source_split"):
            _require_text(provenance, key)


def add_perception_exemplar(
    registry: Mapping[str, Any],
    *,
    exemplar_id: str,
    pair_id: str,
    object_instance_id: str,
    embedding: Sequence[float],
    session_id: str,
    background_id: str,
    source_split: str = "train",
    features: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a registry copy with one exemplar added."""
    validate_perception_registry(registry)
    result = deepcopy(dict(registry))
    for key, value in (
        ("exemplar_id", exemplar_id),
        ("pair_id", pair_id),
        ("object_instance_id", object_instance_id),
        ("session_id", session_id),
        ("background_id", background_id),
        ("source_split", source_split),
    ):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{key} must be a non-empty string")
    if any(item["exemplar_id"] == exemplar_id for item in result["exemplars"]):
        raise ValueError(f"duplicate exemplar_id: {exemplar_id}")
    vector = _embedding(embedding, expected_dimension=result["embedding_dimension"])
    if result["embedding_dimension"] is None:
        result["embedding_dimension"] = len(vector)
    result["exemplars"].append(
        {
            "exemplar_id": exemplar_id,
            "pair_id": pair_id,
            "object_instance_id": object_instance_id,
            "embedding": vector,
            "features": dict(features or {}),
            "provenance": {
                "session_id": session_id,
                "background_id": background_id,
                "source_split": source_split,
            },
        }
    )
    validate_perception_registry(result)
    return result


def _cosine(left: Sequence[float], right: Sequence[float]) -> float:
    dot = sum(a * b for a, b in zip(left, right))
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    return dot / (left_norm * right_norm)


def match_perception_exemplar(
    registry: Mapping[str, Any],
    query_embedding: Sequence[float],
    *,
    min_similarity: float = 0.8,
    min_margin: float = 0.05,
) -> dict[str, Any]:
    """Match a pair ID or abstain when confidence or separation is too low."""
    validate_perception_registry(registry)
    for name, value in (("min_similarity", min_similarity), ("min_margin", min_margin)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
    if not -1.0 <= min_similarity <= 1.0:
        raise ValueError("min_similarity must be between -1 and 1")
    if not 0.0 <= min_margin <= 2.0:
        raise ValueError("min_margin must be between 0 and 2")
    query = _embedding(query_embedding, expected_dimension=registry["embedding_dimension"])
    pair_scores: dict[str, float] = {}
    pair_exemplars: dict[str, str] = {}
    for exemplar in registry["exemplars"]:
        score = _cosine(query, exemplar["embedding"])
        pair_id = exemplar["pair_id"]
        if pair_id not in pair_scores or score > pair_scores[pair_id]:
            pair_scores[pair_id] = score
            pair_exemplars[pair_id] = exemplar["exemplar_id"]
    ranked = sorted(pair_scores.items(), key=lambda item: (-item[1], item[0]))
    candidates = [
        {"pair_id": pair_id, "similarity": score, "exemplar_id": pair_exemplars[pair_id]}
        for pair_id, score in ranked
    ]
    if not candidates:
        return {
            "decision": "abstain",
            "pair_id": None,
            "confidence": None,
            "margin": None,
            "reason": "empty_registry",
            "control_authorized": False,
            "candidates": [],
        }
    top = candidates[0]
    second_score = candidates[1]["similarity"] if len(candidates) > 1 else -1.0
    margin = top["similarity"] - second_score
    if top["similarity"] < min_similarity:
        decision, reason, pair_id = "abstain", "below_similarity_threshold", None
    elif margin < min_margin:
        decision, reason, pair_id = "abstain", "ambiguous_margin", None
    else:
        decision, reason, pair_id = "match", "matched", top["pair_id"]
    return {
        "decision": decision,
        "pair_id": pair_id,
        "confidence": top["similarity"],
        "margin": margin,
        "reason": reason,
        "control_authorized": False,
        "candidates": candidates,
    }


def load_perception_registry(path: str | Path) -> dict[str, Any]:
    registry_path = Path(path)
    try:
        value = json.loads(registry_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"perception registry not found: {registry_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid perception registry JSON: {error}") from error
    if not isinstance(value, Mapping):
        raise ValueError("perception registry top level must be an object")
    validate_perception_registry(value)
    return dict(value)


def save_perception_registry(path: str | Path, registry: Mapping[str, Any]) -> None:
    validate_perception_registry(registry)
    registry_path = Path(path)
    registry_path.parent.mkdir(parents=True, exist_ok=True)
    registry_path.write_text(
        json.dumps(registry, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
