"""Serializable domain objects shared by dealer adapters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class Arm(str, Enum):
    """Logical arm assignments used by the first CardBench task."""

    LEFT = 'left'
    RIGHT = 'right'


class Skill(str, Enum):
    """High-level skills that a controller or learned policy must execute."""

    STABILIZE_DECK = 'stabilize_deck'
    PICK_TOP_CARD = 'pick_top_card'
    PLACE_CARD = 'place_card'
    RELEASE_DECK = 'release_deck'


@dataclass(frozen=True)
class DealerCommand:
    """One deterministic high-level command in a deal plan."""

    step: int
    arm: Arm
    skill: Skill
    target: str
    card_index: int | None = None
    face_up: bool | None = None

    def __post_init__(self) -> None:
        if self.step < 1:
            raise ValueError('step must be at least 1')
        if not self.target:
            raise ValueError('target must not be empty')

        card_skill = self.skill in (Skill.PICK_TOP_CARD, Skill.PLACE_CARD)
        if card_skill and self.card_index is None:
            raise ValueError(f'{self.skill.value} requires card_index')
        if not card_skill and self.card_index is not None:
            raise ValueError(f'{self.skill.value} must not include card_index')
        if self.card_index is not None and self.card_index < 0:
            raise ValueError('card_index must not be negative')

        if self.skill is Skill.PLACE_CARD and self.face_up is None:
            raise ValueError('place_card requires face_up')
        if self.skill is not Skill.PLACE_CARD and self.face_up is not None:
            raise ValueError(f'{self.skill.value} must not include face_up')

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable command dictionary."""
        result: dict[str, Any] = {
            'step': self.step,
            'arm': self.arm.value,
            'skill': self.skill.value,
            'target': self.target,
        }
        if self.card_index is not None:
            result['card_index'] = self.card_index
        if self.face_up is not None:
            result['face_up'] = self.face_up
        return result


@dataclass(frozen=True)
class DealPlan:
    """A versioned, deterministic plan for one card-table task."""

    schema_version: str
    task_name: str
    language_instruction: str
    game_name: str
    player_count: int
    commands: tuple[DealerCommand, ...]

    def __post_init__(self) -> None:
        expected_steps = tuple(range(1, len(self.commands) + 1))
        actual_steps = tuple(command.step for command in self.commands)
        if actual_steps != expected_steps:
            raise ValueError('command steps must be contiguous and start at 1')

    def to_dict(self) -> dict[str, Any]:
        """Return the stable JSON representation consumed by adapters."""
        return {
            'schema_version': self.schema_version,
            'task': {
                'name': self.task_name,
                'language_instruction': self.language_instruction,
            },
            'game': {
                'name': self.game_name,
                'player_count': self.player_count,
            },
            'commands': [command.to_dict() for command in self.commands],
        }
