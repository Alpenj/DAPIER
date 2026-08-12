#!/usr/bin/env python3

import unittest

from tb3_tf_probe import TfAudit


class TfAuditTests(unittest.TestCase):
    def add_valid(self, audit: TfAudit, stamp_ns: int) -> None:
        audit.receive_values(
            stamp_ns=stamp_ns,
            translation=(0.1, -0.2, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
        )

    def test_accepts_two_monotonic_typed_updates(self) -> None:
        audit = TfAudit()
        self.add_valid(audit, 1_000)
        self.add_valid(audit, 2_000)
        self.assertEqual(audit.errors(2), [])

    def test_polling_same_buffered_transform_is_not_an_update(self) -> None:
        audit = TfAudit()
        self.add_valid(audit, 1_000)
        self.add_valid(audit, 1_000)
        self.assertIn("only 1", " ".join(audit.errors(2)))

    def test_rejects_regressing_timestamp(self) -> None:
        audit = TfAudit()
        self.add_valid(audit, 2_000)
        self.add_valid(audit, 1_000)
        self.assertIn("regression", " ".join(audit.errors(2)))

    def test_rejects_invalid_quaternion(self) -> None:
        audit = TfAudit()
        audit.receive_values(
            stamp_ns=1_000,
            translation=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 0.0),
        )
        self.assertIn("invalid", " ".join(audit.errors(1)))


if __name__ == "__main__":
    unittest.main()
