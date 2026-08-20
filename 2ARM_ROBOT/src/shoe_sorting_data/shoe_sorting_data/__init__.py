"""DYNA-lite data tools for the DAPIER shoe-sorting robot."""

from shoe_sorting_data.contract import (
    EPISODE_SCHEMA_VERSION,
    build_manifest,
    load_manifest,
    save_manifest,
    validate_manifest,
)
from shoe_sorting_data.quality import ValidationReport, validate_episode
from shoe_sorting_data.perception_exemplar import (
    add_perception_exemplar,
    build_perception_registry,
    match_perception_exemplar,
)
from shoe_sorting_data.synthetic import generate_dataset, generate_episode
from shoe_sorting_data.skill_exemplar import (
    audit_exemplar_leakage,
    build_skill_exemplar,
    retrieve_skill_exemplars,
)

__all__ = [
    "EPISODE_SCHEMA_VERSION",
    "ValidationReport",
    "add_perception_exemplar",
    "build_perception_registry",
    "build_manifest",
    "build_skill_exemplar",
    "audit_exemplar_leakage",
    "generate_dataset",
    "generate_episode",
    "load_manifest",
    "match_perception_exemplar",
    "save_manifest",
    "retrieve_skill_exemplars",
    "validate_episode",
    "validate_manifest",
]
