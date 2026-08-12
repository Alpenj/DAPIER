#!/usr/bin/env python3

import unittest

from tb3_scan_probe import ScanAudit


class ScanAuditTests(unittest.TestCase):
    def add_valid(self, audit: ScanAudit, stamp_ns: int) -> None:
        audit.receive_values(
            stamp_ns=stamp_ns,
            frame_id="base_scan",
            angle_increment=0.01,
            range_min=0.12,
            range_max=3.5,
            ranges=[float("inf"), 0.8, 1.2],
        )

    def test_accepts_fresh_valid_scans(self) -> None:
        audit = ScanAudit()
        for index in range(10):
            self.add_valid(audit, 1_000_000_000 + index * 200_000_000)
        self.assertEqual(audit.errors(10), [])

    def test_rejects_duplicate_and_regressing_timestamps(self) -> None:
        audit = ScanAudit()
        for stamp in (1_000, 1_000, 900):
            self.add_valid(audit, stamp)
        errors = " ".join(audit.errors(3))
        self.assertIn("duplicate", errors)
        self.assertIn("regression", errors)

    def test_rejects_empty_or_non_finite_scan(self) -> None:
        audit = ScanAudit()
        audit.receive_values(
            stamp_ns=1_000,
            frame_id="base_scan",
            angle_increment=0.01,
            range_min=0.12,
            range_max=3.5,
            ranges=[],
        )
        errors = " ".join(audit.errors(1))
        self.assertIn("structurally invalid", errors)
        self.assertIn("no finite", errors)


if __name__ == "__main__":
    unittest.main()
