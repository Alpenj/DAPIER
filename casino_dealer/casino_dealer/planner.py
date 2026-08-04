"""Deterministic task planner used for the current CardBench experiment."""

from __future__ import annotations

from casino_dealer.model import Arm, DealPlan, DealerCommand, Skill


DEAL_PLAN_SCHEMA_VERSION = 'dapier.deal-plan.v0'
MIN_PLAYER_COUNT = 1
MAX_PLAYER_COUNT = 7


def build_blackjack_opening_plan(player_count: int) -> DealPlan:
    """Build a two-round opening deal using a fixed bimanual role split.

    The left arm stabilizes the deck for the full deal. The right arm moves
    every card. Player cards and the dealer's first card are face up; the
    dealer's second card is the face-down hole card.
    """
    _validate_player_count(player_count)

    commands: list[DealerCommand] = []

    def append(
        arm: Arm,
        skill: Skill,
        target: str,
        card_index: int | None = None,
        face_up: bool | None = None,
    ) -> None:
        commands.append(
            DealerCommand(
                step=len(commands) + 1,
                arm=arm,
                skill=skill,
                target=target,
                card_index=card_index,
                face_up=face_up,
            )
        )

    append(Arm.LEFT, Skill.STABILIZE_DECK, 'deck')

    seat_order = [
        *(f'player_{index}' for index in range(1, player_count + 1)),
        'dealer',
    ]
    card_index = 0
    for round_index in range(2):
        for seat in seat_order:
            append(
                Arm.RIGHT,
                Skill.PICK_TOP_CARD,
                'deck',
                card_index=card_index,
            )
            is_hole_card = round_index == 1 and seat == 'dealer'
            append(
                Arm.RIGHT,
                Skill.PLACE_CARD,
                seat,
                card_index=card_index,
                face_up=not is_hole_card,
            )
            card_index += 1

    append(Arm.LEFT, Skill.RELEASE_DECK, 'deck')

    return DealPlan(
        schema_version=DEAL_PLAN_SCHEMA_VERSION,
        task_name='blackjack_opening_deal',
        language_instruction=(
            f'Deal the opening blackjack hand to {player_count} '
            f'{"player" if player_count == 1 else "players"}.'
        ),
        game_name='blackjack',
        player_count=player_count,
        commands=tuple(commands),
    )


def _validate_player_count(player_count: int) -> None:
    if isinstance(player_count, bool) or not isinstance(player_count, int):
        raise TypeError('player_count must be an integer')
    if not MIN_PLAYER_COUNT <= player_count <= MAX_PLAYER_COUNT:
        raise ValueError(
            f'player_count must be between '
            f'{MIN_PLAYER_COUNT} and {MAX_PLAYER_COUNT}'
        )
