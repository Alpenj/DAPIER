"""ROS 2-free command transport safety contract between workstation and Pi."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol, runtime_checkable


class EdgeCommandKind(str, Enum):
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    HOLD = "hold"
    SAFE_STOP = "safe_stop"
    LOAD_SHEDDING = "load_shedding"


class AckStatus(str, Enum):
    ACCEPTED = "accepted"
    DUPLICATE = "duplicate"
    EXPIRED = "expired"
    OUT_OF_ORDER = "out_of_order"
    INVALID = "invalid"


@dataclass(frozen=True)
class CommandEnvelope:
    sequence: int
    request_id: str
    kind: EdgeCommandKind
    issued_monotonic_ns: int
    ttl_ms: float
    payload_digest: str

    def validate(self) -> None:
        for label, value in (
            ("sequence", self.sequence),
            ("issued_monotonic_ns", self.issued_monotonic_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{label} must be a non-negative integer")
        if (
            not isinstance(self.request_id, str)
            or not self.request_id.strip()
            or len(self.request_id) > 100
        ):
            raise ValueError("request_id must contain 1 to 100 characters")
        if not isinstance(self.kind, EdgeCommandKind):
            raise ValueError("kind must be an EdgeCommandKind")
        if (
            isinstance(self.ttl_ms, bool)
            or not isinstance(self.ttl_ms, (int, float))
            or not math.isfinite(self.ttl_ms)
            or self.ttl_ms <= 0
        ):
            raise ValueError("ttl_ms must be finite and positive")
        if (
            not isinstance(self.payload_digest, str)
            or len(self.payload_digest) != 64
            or any(
                character not in "0123456789abcdef"
                for character in self.payload_digest
            )
        ):
            raise ValueError("payload_digest must be a lowercase SHA-256 hex digest")

    def age_ms(self, *, now_monotonic_ns: int) -> float:
        if (
            isinstance(now_monotonic_ns, bool)
            or not isinstance(now_monotonic_ns, int)
            or now_monotonic_ns < self.issued_monotonic_ns
        ):
            raise ValueError("now_monotonic_ns must not precede command issue time")
        return (now_monotonic_ns - self.issued_monotonic_ns) / 1_000_000.0


@dataclass(frozen=True)
class CommandAck:
    sequence: int
    request_id: str
    status: AckStatus
    received_monotonic_ns: int
    requires_safe_state: bool
    detail: str = ""

    def validate(self) -> None:
        for label, value in (
            ("sequence", self.sequence),
            ("received_monotonic_ns", self.received_monotonic_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{label} must be a non-negative integer")
        if (
            not isinstance(self.request_id, str)
            or not self.request_id.strip()
            or len(self.request_id) > 100
        ):
            raise ValueError("request_id must contain 1 to 100 characters")
        if not isinstance(self.status, AckStatus):
            raise ValueError("status must be an AckStatus")
        if not isinstance(self.requires_safe_state, bool):
            raise ValueError("requires_safe_state must be a boolean")
        if not isinstance(self.detail, str) or len(self.detail) > 500:
            raise ValueError("ack detail is too long")


class CommandSequenceGate:
    """Pi-side no-replay gate for already validated typed command payloads."""

    def __init__(self) -> None:
        self._last_accepted_sequence = -1
        self._accepted_digests: dict[int, str] = {}

    @property
    def last_accepted_sequence(self) -> int:
        return self._last_accepted_sequence

    def inspect(
        self,
        envelope: object,
        *,
        now_monotonic_ns: int,
    ) -> CommandAck:
        if (
            isinstance(now_monotonic_ns, bool)
            or not isinstance(now_monotonic_ns, int)
            or now_monotonic_ns < 0
        ):
            return self._invalid_ack(0, "now_monotonic_ns is invalid")
        if not isinstance(envelope, CommandEnvelope):
            return self._invalid_ack(
                now_monotonic_ns,
                "command must be a CommandEnvelope",
            )
        try:
            envelope.validate()
            age_ms = envelope.age_ms(now_monotonic_ns=now_monotonic_ns)
        except ValueError as error:
            return self._invalid_ack(now_monotonic_ns, str(error))
        if age_ms > envelope.ttl_ms:
            return self._ack(
                envelope,
                AckStatus.EXPIRED,
                now_monotonic_ns,
                requires_safe_state=True,
                detail=f"command age {age_ms:.3f} ms exceeded TTL",
            )
        if envelope.sequence in self._accepted_digests:
            same_payload = (
                self._accepted_digests[envelope.sequence] == envelope.payload_digest
            )
            return self._ack(
                envelope,
                AckStatus.DUPLICATE if same_payload else AckStatus.OUT_OF_ORDER,
                now_monotonic_ns,
                requires_safe_state=not same_payload,
                detail="already accepted; command will not be replayed",
            )
        if envelope.sequence <= self._last_accepted_sequence:
            return self._ack(
                envelope,
                AckStatus.OUT_OF_ORDER,
                now_monotonic_ns,
                requires_safe_state=True,
                detail="sequence is older than last accepted command",
            )
        self._last_accepted_sequence = envelope.sequence
        self._accepted_digests[envelope.sequence] = envelope.payload_digest
        return self._ack(
            envelope,
            AckStatus.ACCEPTED,
            now_monotonic_ns,
            requires_safe_state=False,
        )

    @staticmethod
    def _invalid_ack(
        received_monotonic_ns: int,
        detail: str,
    ) -> CommandAck:
        ack = CommandAck(
            sequence=0,
            request_id="invalid",
            status=AckStatus.INVALID,
            received_monotonic_ns=received_monotonic_ns,
            requires_safe_state=True,
            detail=detail,
        )
        ack.validate()
        return ack

    @staticmethod
    def _ack(
        envelope: CommandEnvelope,
        status: AckStatus,
        received_monotonic_ns: int,
        *,
        requires_safe_state: bool,
        detail: str = "",
    ) -> CommandAck:
        ack = CommandAck(
            sequence=envelope.sequence,
            request_id=envelope.request_id,
            status=status,
            received_monotonic_ns=received_monotonic_ns,
            requires_safe_state=requires_safe_state,
            detail=detail,
        )
        ack.validate()
        return ack


@dataclass(frozen=True)
class LinkHeartbeat:
    sequence: int
    received_monotonic_ns: int
    timeout_ms: float

    def validate(self) -> None:
        for label, value in (
            ("sequence", self.sequence),
            ("received_monotonic_ns", self.received_monotonic_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{label} must be a non-negative integer")
        if (
            isinstance(self.timeout_ms, bool)
            or not isinstance(self.timeout_ms, (int, float))
            or not math.isfinite(self.timeout_ms)
            or self.timeout_ms <= 0
        ):
            raise ValueError("heartbeat timeout_ms must be finite and positive")

    def alive(self, *, now_monotonic_ns: int) -> bool:
        self.validate()
        if (
            isinstance(now_monotonic_ns, bool)
            or not isinstance(now_monotonic_ns, int)
            or now_monotonic_ns < self.received_monotonic_ns
        ):
            raise ValueError("now_monotonic_ns precedes heartbeat receipt")
        age_ms = (now_monotonic_ns - self.received_monotonic_ns) / 1_000_000.0
        return age_ms <= self.timeout_ms


@runtime_checkable
class EdgeTransportPort(Protocol):
    @property
    def simulation_only(self) -> bool: ...

    def send_command(self, envelope: CommandEnvelope) -> CommandAck: ...

    def latest_heartbeat(self) -> LinkHeartbeat: ...


__all__ = [
    "AckStatus",
    "CommandAck",
    "CommandEnvelope",
    "CommandSequenceGate",
    "EdgeCommandKind",
    "EdgeTransportPort",
    "LinkHeartbeat",
]
