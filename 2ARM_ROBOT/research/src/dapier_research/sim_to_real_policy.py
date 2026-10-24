"""Strict policy boundary that accepts sensor-runtime observations only."""

from __future__ import annotations

from typing import Mapping, Protocol, Sequence

from .observation_contract import validate_observation_for_use


class ResettableChunkExecutor(Protocol):
    @property
    def policy_queries(self) -> int: ...

    @property
    def executed_actions(self) -> int: ...

    def reset(self) -> None: ...

    def next_action(
        self, observation: Mapping[str, object]
    ) -> Sequence[float]: ...


class SensorPolicyExecutor:
    """Validate provenance before delegating to an existing chunk executor.

    The wrapper does not infer provenance and never falls back to simulator
    truth. A missing or privileged observation is rejected before the policy is
    queried.
    """

    def __init__(self, delegate: ResettableChunkExecutor) -> None:
        self._delegate = delegate

    @property
    def policy_queries(self) -> int:
        return int(self._delegate.policy_queries)

    @property
    def executed_actions(self) -> int:
        return int(self._delegate.executed_actions)

    def reset(self) -> None:
        self._delegate.reset()

    def next_action(
        self, observation: Mapping[str, object]
    ) -> tuple[float, ...]:
        validate_observation_for_use(observation, use="policy_runtime")
        return tuple(float(value) for value in self._delegate.next_action(observation))


__all__ = ["ResettableChunkExecutor", "SensorPolicyExecutor"]
