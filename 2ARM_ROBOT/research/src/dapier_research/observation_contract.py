"""Fail-closed observation provenance for sim-to-real consumers.

Simulator truth may support reset, reward, and evaluation, but it must never
enter policy-runtime, training-episode, or control-monitor boundaries.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence


OBSERVATION_PROVENANCE_SCHEMA_VERSION = "dapier.observation-provenance.v1"
OBSERVATION_SOURCE_KINDS = ("sensor_runtime", "simulator_privileged")
OBSERVATION_USES = (
    "policy_runtime",
    "training_episode",
    "control_monitor",
    "evaluation_oracle",
    "reset_reward",
)
STRICT_SENSOR_USES = frozenset(
    {"policy_runtime", "training_episode", "control_monitor"}
)

_REQUIRED_KEYS = frozenset(
    {
        "schema_version",
        "source_kind",
        "producer",
        "ground_truth_used",
        "privileged",
        "sensor_frames",
        "privileged_fields",
    }
)
_RUNTIME_FORBIDDEN_KEYS = frozenset(
    {
        "body_id",
        "body_ids",
        "body_pose",
        "body_position",
        "body_quaternion",
        "geom_id",
        "geom_ids",
        "object_world_pose",
        "qpos",
        "segmentation_id",
        "segmentation_ids",
        "shoe",
        "simulator_object_pose",
        "xpos",
        "xquat",
    }
)


@dataclass(frozen=True)
class ObservationProvenance:
    schema_version: str
    source_kind: str
    producer: str
    ground_truth_used: bool
    privileged: bool
    sensor_frames: tuple[str, ...]
    privileged_fields: tuple[str, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "source_kind": self.source_kind,
            "producer": self.producer,
            "ground_truth_used": self.ground_truth_used,
            "privileged": self.privileged,
            "sensor_frames": list(self.sensor_frames),
            "privileged_fields": list(self.privileged_fields),
        }


def _text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _boolean(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be boolean")
    return value


def _text_array(value: object, name: str) -> tuple[str, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must be an array")
    result = tuple(_text(item, f"{name} entry") for item in value)
    if len(set(result)) != len(result):
        raise ValueError(f"{name} entries must be unique")
    return result


def provenance_from_mapping(value: Mapping[str, object]) -> ObservationProvenance:
    if not isinstance(value, Mapping):
        raise ValueError("observation_provenance must be an object")
    keys = frozenset(value)
    if keys != _REQUIRED_KEYS:
        raise ValueError(
            "observation_provenance keys differ; "
            f"missing={sorted(_REQUIRED_KEYS - keys)}, "
            f"extra={sorted(keys - _REQUIRED_KEYS)}"
        )
    result = ObservationProvenance(
        schema_version=_text(value["schema_version"], "schema_version"),
        source_kind=_text(value["source_kind"], "source_kind"),
        producer=_text(value["producer"], "producer"),
        ground_truth_used=_boolean(
            value["ground_truth_used"], "ground_truth_used"
        ),
        privileged=_boolean(value["privileged"], "privileged"),
        sensor_frames=_text_array(value["sensor_frames"], "sensor_frames"),
        privileged_fields=_text_array(
            value["privileged_fields"], "privileged_fields"
        ),
    )
    if result.schema_version != OBSERVATION_PROVENANCE_SCHEMA_VERSION:
        raise ValueError("unsupported observation provenance schema_version")
    if result.source_kind not in OBSERVATION_SOURCE_KINDS:
        raise ValueError(f"source_kind must be one of {OBSERVATION_SOURCE_KINDS}")
    if result.source_kind == "sensor_runtime":
        if result.privileged or result.ground_truth_used:
            raise ValueError("sensor_runtime provenance cannot use privileged truth")
        if not result.sensor_frames:
            raise ValueError("sensor_runtime provenance requires sensor_frames")
        if result.privileged_fields:
            raise ValueError("sensor_runtime provenance cannot list privileged_fields")
    else:
        if not result.privileged or not result.ground_truth_used:
            raise ValueError(
                "simulator_privileged provenance must declare privileged truth"
            )
        if not result.privileged_fields:
            raise ValueError(
                "simulator_privileged provenance requires privileged_fields"
            )
    return result


def sensor_runtime_provenance(
    *, producer: str, sensor_frames: Sequence[str]
) -> ObservationProvenance:
    return provenance_from_mapping(
        {
            "schema_version": OBSERVATION_PROVENANCE_SCHEMA_VERSION,
            "source_kind": "sensor_runtime",
            "producer": producer,
            "ground_truth_used": False,
            "privileged": False,
            "sensor_frames": list(sensor_frames),
            "privileged_fields": [],
        }
    )


def simulator_privileged_provenance(
    *,
    producer: str,
    privileged_fields: Sequence[str],
    sensor_frames: Sequence[str] = (),
) -> ObservationProvenance:
    return provenance_from_mapping(
        {
            "schema_version": OBSERVATION_PROVENANCE_SCHEMA_VERSION,
            "source_kind": "simulator_privileged",
            "producer": producer,
            "ground_truth_used": True,
            "privileged": True,
            "sensor_frames": list(sensor_frames),
            "privileged_fields": list(privileged_fields),
        }
    )


def _walk_items(value: object, path: str = "observation"):
    if isinstance(value, Mapping):
        for key, child in value.items():
            key_text = str(key)
            child_path = f"{path}.{key_text}"
            yield child_path, key_text, child
            yield from _walk_items(child, child_path)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for index, child in enumerate(value):
            yield from _walk_items(child, f"{path}[{index}]")


def _reject_privileged_runtime_fields(observation: Mapping[str, object]) -> None:
    for path, key, value in _walk_items(observation):
        if key in _RUNTIME_FORBIDDEN_KEYS:
            raise ValueError(f"runtime observation contains privileged field: {path}")
        if key in {"ground_truth", "ground_truth_used", "uses_privileged_labels"}:
            if value is not False:
                raise ValueError(f"runtime observation truth flag must be false: {path}")
        if key in {"control_authorized", "hardware_execution"} and value is not False:
            raise ValueError(f"runtime observation authorization flag must be false: {path}")


def validate_observation_for_use(
    observation: Mapping[str, object], *, use: str
) -> ObservationProvenance:
    if use not in OBSERVATION_USES:
        raise ValueError(f"use must be one of {OBSERVATION_USES}")
    if not isinstance(observation, Mapping):
        raise ValueError("observation must be an object")
    raw = observation.get("observation_provenance")
    if not isinstance(raw, Mapping):
        raise ValueError("observation requires explicit observation_provenance")
    provenance = provenance_from_mapping(raw)
    ground_truth = observation.get("ground_truth")
    if not isinstance(ground_truth, bool):
        raise ValueError("observation.ground_truth must be boolean")
    if ground_truth != provenance.ground_truth_used:
        raise ValueError("observation ground_truth flag conflicts with provenance")
    if use in STRICT_SENSOR_USES:
        if provenance.source_kind != "sensor_runtime":
            raise ValueError(f"{use} requires sensor_runtime provenance")
        _reject_privileged_runtime_fields(observation)
    return provenance


def attach_sensor_runtime_provenance(
    observation: Mapping[str, object],
    *,
    producer: str,
    sensor_frames: Sequence[str],
) -> dict[str, object]:
    if not isinstance(observation, Mapping):
        raise ValueError("observation must be an object")
    if "observation_provenance" in observation:
        raise ValueError("observation already contains observation_provenance")
    result = dict(observation)
    if result.get("ground_truth", False) is not False:
        raise ValueError("sensor provenance cannot be attached to ground truth")
    result["ground_truth"] = False
    result["observation_provenance"] = sensor_runtime_provenance(
        producer=producer, sensor_frames=sensor_frames
    ).as_dict()
    validate_observation_for_use(result, use="policy_runtime")
    return result


def attach_simulator_privileged_provenance(
    observation: Mapping[str, object],
    *,
    producer: str,
    privileged_fields: Sequence[str],
    sensor_frames: Sequence[str] = (),
) -> dict[str, object]:
    if not isinstance(observation, Mapping):
        raise ValueError("observation must be an object")
    if "observation_provenance" in observation:
        raise ValueError("observation already contains observation_provenance")
    result = dict(observation)
    if result.get("ground_truth", True) is not True:
        raise ValueError("privileged provenance requires ground_truth=true")
    result["ground_truth"] = True
    result["observation_provenance"] = simulator_privileged_provenance(
        producer=producer,
        privileged_fields=privileged_fields,
        sensor_frames=sensor_frames,
    ).as_dict()
    validate_observation_for_use(result, use="evaluation_oracle")
    return result
