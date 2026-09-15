"""Dataset gate for sensor-only sim-to-real training episodes."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Mapping, Sequence

from .observation_contract import (
    OBSERVATION_PROVENANCE_SCHEMA_VERSION,
    validate_observation_for_use,
)


SIM_TO_REAL_DATASET_GATE_SCHEMA_VERSION = "dapier.sim-to-real-dataset-gate.v1"


@dataclass(frozen=True)
class SimToRealDatasetGateReport:
    schema_version: str
    accepted: bool
    sample_count: int
    producers: tuple[str, ...]
    sensor_frames: tuple[str, ...]
    ground_truth_used_for_policy: bool = False
    hardware_execution: bool = False

    def as_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "accepted": self.accepted,
            "sample_count": self.sample_count,
            "producers": list(self.producers),
            "sensor_frames": list(self.sensor_frames),
            "ground_truth_used_for_policy": self.ground_truth_used_for_policy,
            "hardware_execution": self.hardware_execution,
        }


def validate_sim_to_real_episode(
    manifest: Mapping[str, object],
    samples: Sequence[Mapping[str, object]],
) -> SimToRealDatasetGateReport:
    """Reject legacy/privileged episodes before sim-to-real training use."""

    if not isinstance(manifest, Mapping):
        raise ValueError("manifest must be an object")
    provenance = manifest.get("provenance")
    if not isinstance(provenance, Mapping):
        raise ValueError("manifest.provenance must be an object")
    if provenance.get("sim_to_real_observation_compatible") is not True:
        raise ValueError(
            "manifest is not marked sim_to_real_observation_compatible=true"
        )
    if provenance.get("ground_truth_used_for_policy") is not False:
        raise ValueError("manifest must declare ground_truth_used_for_policy=false")
    if (
        provenance.get("observation_provenance_schema_version")
        != OBSERVATION_PROVENANCE_SCHEMA_VERSION
    ):
        raise ValueError("manifest observation provenance schema is unsupported")

    recording = manifest.get("recording")
    if not isinstance(recording, Mapping):
        raise ValueError("manifest.recording must be an object")
    camera_payload = recording.get("camera_payload")
    if not isinstance(camera_payload, Mapping) or camera_payload.get("mode") != "required":
        raise ValueError("sim-to-real episode requires camera_payload.mode=required")
    expected_count = recording.get("sample_count")
    if isinstance(expected_count, bool) or not isinstance(expected_count, int):
        raise ValueError("recording.sample_count must be an integer")
    if expected_count <= 0 or len(samples) != expected_count:
        raise ValueError("sample count does not match manifest")

    producers: set[str] = set()
    frames: set[str] = set()
    for index, sample in enumerate(samples):
        if not isinstance(sample, Mapping):
            raise ValueError(f"sample {index} must be an object")
        observation = sample.get("policy_observation")
        if not isinstance(observation, Mapping):
            raise ValueError(
                f"sample {index} requires an explicit policy_observation"
            )
        sample_provenance = validate_observation_for_use(
            observation, use="training_episode"
        )
        producers.add(sample_provenance.producer)
        frames.update(sample_provenance.sensor_frames)
        simulation = sample.get("simulation")
        if not isinstance(simulation, Mapping):
            raise ValueError(f"sample {index}.simulation must be an object")
        if simulation.get("hardware_execution") is not False:
            raise ValueError(f"sample {index} reported hardware execution")

    return SimToRealDatasetGateReport(
        schema_version=SIM_TO_REAL_DATASET_GATE_SCHEMA_VERSION,
        accepted=True,
        sample_count=len(samples),
        producers=tuple(sorted(producers)),
        sensor_frames=tuple(sorted(frames)),
    )


def validate_sim_to_real_episode_files(
    manifest_path: str | Path,
    samples_path: str | Path,
) -> SimToRealDatasetGateReport:
    try:
        manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot load manifest: {error}") from error
    if not isinstance(manifest, Mapping):
        raise ValueError("manifest top level must be an object")

    samples: list[Mapping[str, object]] = []
    try:
        lines = Path(samples_path).read_text(encoding="utf-8").splitlines()
        for line_number, line in enumerate(lines, start=1):
            value = json.loads(line)
            if not isinstance(value, Mapping):
                raise ValueError(f"sample line {line_number} must be an object")
            samples.append(value)
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot load samples: {error}") from error
    return validate_sim_to_real_episode(manifest, samples)
