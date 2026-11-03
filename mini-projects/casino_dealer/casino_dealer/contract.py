"""Load and validate the CardBench observation/action contract."""

from __future__ import annotations

from importlib import resources
import json
from pathlib import Path
from typing import Any, Mapping


CARD_BENCH_SCHEMA_VERSION = 'dapier.cardbench.v0'
_CONTRACT_RESOURCE = 'contracts/cardbench_v0.json'


def load_cardbench_contract(
    path: str | Path | None = None,
) -> dict[str, Any]:
    """Load the default packaged contract or a contract from ``path``."""
    if path is None:
        text = resources.files('casino_dealer').joinpath(
            _CONTRACT_RESOURCE
        ).read_text(encoding='utf-8')
    else:
        text = Path(path).read_text(encoding='utf-8')

    contract = json.loads(text)
    validate_cardbench_contract(contract)
    return contract


def validate_cardbench_contract(contract: Mapping[str, Any]) -> None:
    """Reject contracts that cannot represent the CardBench v0 task."""
    if contract.get('schema_version') != CARD_BENCH_SCHEMA_VERSION:
        raise ValueError(
            f'schema_version must be {CARD_BENCH_SCHEMA_VERSION!r}'
        )

    frequency = contract.get('control_frequency_hz')
    if isinstance(frequency, bool) or not isinstance(frequency, (int, float)):
        raise ValueError('control_frequency_hz must be numeric')
    if frequency <= 0:
        raise ValueError('control_frequency_hz must be positive')

    observation = _require_mapping(contract, 'observation')
    state = _require_mapping(observation, 'state')
    images = _require_mapping(observation, 'images')
    action = _require_mapping(contract, 'action')
    task = _require_mapping(contract, 'task')

    expected_shapes = {
        'left_arm.joint_position': [4],
        'right_arm.joint_position': [4],
        'left_vacuum.pressure': [1],
        'right_vacuum.pressure': [1],
    }
    for name, expected_shape in expected_shapes.items():
        feature = _require_mapping(state, name)
        if feature.get('shape') != expected_shape:
            raise ValueError(f'{name} shape must be {expected_shape}')

    for name in ('left_arm.joint_position', 'right_arm.joint_position'):
        if state[name].get('source') != 'measured':
            raise ValueError(f'{name} source must be measured')

    overhead = _require_mapping(images, 'overhead')
    if overhead.get('required') is not True:
        raise ValueError('overhead image must be required')

    expected_action_shapes = {
        'left_arm.joint_target': [4],
        'right_arm.joint_target': [4],
        'left_vacuum.command': [1],
        'right_vacuum.command': [1],
    }
    for name, expected_shape in expected_action_shapes.items():
        feature = _require_mapping(action, name)
        if feature.get('shape') != expected_shape:
            raise ValueError(f'{name} shape must be {expected_shape}')

    for name in ('left_vacuum.command', 'right_vacuum.command'):
        if action[name].get('dtype') != 'float32':
            raise ValueError(f'{name} dtype must be float32')
        if action[name].get('range') != [0.0, 1.0]:
            raise ValueError(f'{name} range must be [0.0, 1.0]')

    instruction = _require_mapping(task, 'language_instruction')
    if instruction.get('required') is not True:
        raise ValueError('language_instruction must be required')


def _require_mapping(
    parent: Mapping[str, Any],
    key: str,
) -> Mapping[str, Any]:
    value = parent.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f'{key} must be an object')
    return value
