from __future__ import annotations

import ast
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_modules.transport import (
    AckStatus,
    CommandEnvelope,
    CommandSequenceGate,
    EdgeCommandKind,
    LinkHeartbeat,
)


DIGEST_A = "a" * 64
DIGEST_B = "b" * 64


def command(**overrides) -> CommandEnvelope:
    values = {
        "sequence": 1,
        "request_id": "navigate-b-1",
        "kind": EdgeCommandKind.NAVIGATION,
        "issued_monotonic_ns": 1_000_000_000,
        "ttl_ms": 100.0,
        "payload_digest": DIGEST_A,
    }
    values.update(overrides)
    return CommandEnvelope(**values)


class TransportContractTest(unittest.TestCase):
    def test_fresh_command_is_accepted_once(self) -> None:
        gate = CommandSequenceGate()
        accepted = gate.inspect(command(), now_monotonic_ns=1_010_000_000)
        self.assertEqual(accepted.status, AckStatus.ACCEPTED)
        self.assertFalse(accepted.requires_safe_state)
        duplicate = gate.inspect(command(), now_monotonic_ns=1_020_000_000)
        self.assertEqual(duplicate.status, AckStatus.DUPLICATE)
        self.assertFalse(duplicate.requires_safe_state)

    def test_same_sequence_with_different_payload_fails_closed(self) -> None:
        gate = CommandSequenceGate()
        gate.inspect(command(), now_monotonic_ns=1_010_000_000)
        mismatch = gate.inspect(
            command(payload_digest=DIGEST_B),
            now_monotonic_ns=1_020_000_000,
        )
        self.assertEqual(mismatch.status, AckStatus.OUT_OF_ORDER)
        self.assertTrue(mismatch.requires_safe_state)

    def test_expired_and_out_of_order_commands_require_safe_state(self) -> None:
        gate = CommandSequenceGate()
        expired = gate.inspect(command(), now_monotonic_ns=1_101_000_000)
        self.assertEqual(expired.status, AckStatus.EXPIRED)
        self.assertTrue(expired.requires_safe_state)
        accepted = gate.inspect(
            command(sequence=2, request_id="hold-2", kind=EdgeCommandKind.HOLD),
            now_monotonic_ns=1_010_000_000,
        )
        self.assertEqual(accepted.status, AckStatus.ACCEPTED)
        old = gate.inspect(command(sequence=1), now_monotonic_ns=1_020_000_000)
        self.assertEqual(old.status, AckStatus.OUT_OF_ORDER)
        self.assertTrue(old.requires_safe_state)

    def test_heartbeat_timeout_is_edge_local(self) -> None:
        heartbeat = LinkHeartbeat(
            sequence=4,
            received_monotonic_ns=1_000_000_000,
            timeout_ms=250.0,
        )
        self.assertTrue(heartbeat.alive(now_monotonic_ns=1_250_000_000))
        self.assertFalse(heartbeat.alive(now_monotonic_ns=1_250_000_001))

    def test_contract_imports_no_runtime_backend(self) -> None:
        tree = ast.parse(
            (PROJECT_DIR / "mission_modules" / "transport.py").read_text(
                encoding="utf-8"
            )
        )
        imported: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split(".")[0])
        self.assertTrue(
            {"mujoco", "rclpy", "serial", "zmq", "grpc"}.isdisjoint(imported)
        )


if __name__ == "__main__":
    unittest.main()
