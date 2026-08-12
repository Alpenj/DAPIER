#!/usr/bin/env python3

from types import SimpleNamespace
import unittest

from tb3_real_ready import OdomAudit


def odom(stamp_ns: int, *, x: float = 0.0, qw: float = 1.0):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            )
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=qw),
            )
        ),
    )


class OdomAuditTests(unittest.TestCase):
    def test_accepts_five_unique_monotonic_valid_samples(self) -> None:
        audit = OdomAudit()
        for index in range(5):
            audit.receive(odom(1_000_000_000 + index * 50_000_000, x=index / 10))
        self.assertEqual(audit.errors(5), [])

    def test_rejects_duplicate_and_regressing_timestamps(self) -> None:
        audit = OdomAudit()
        for stamp in (1_000_000_000, 1_050_000_000, 1_050_000_000, 900_000_000):
            audit.receive(odom(stamp))
        errors = " ".join(audit.errors(4))
        self.assertIn("duplicate", errors)
        self.assertIn("regressions", errors)

    def test_rejects_invalid_pose(self) -> None:
        audit = OdomAudit()
        audit.receive(odom(1_000_000_000, x=float("nan"), qw=0.0))
        self.assertIn("invalid", " ".join(audit.errors(1)))


if __name__ == "__main__":
    unittest.main()
