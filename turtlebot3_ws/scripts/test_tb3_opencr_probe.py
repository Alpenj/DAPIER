#!/usr/bin/env python3

import unittest

from tb3_opencr_probe import OpenCrAudit


class OpenCrAuditTests(unittest.TestCase):
    def test_accepts_multiple_charged_torque_on_samples(self) -> None:
        audit = OpenCrAudit()
        for voltage in (12.4, 12.3, 12.2):
            audit.receive_battery(voltage)
            audit.receive_torque(True)
        self.assertEqual(audit.errors(min_samples=3, min_voltage=11.1), [])

    def test_rejects_low_voltage(self) -> None:
        audit = OpenCrAudit()
        for voltage in (12.0, 11.0, 11.8):
            audit.receive_battery(voltage)
            audit.receive_torque(True)
        self.assertIn(
            "below", " ".join(audit.errors(min_samples=3, min_voltage=11.1))
        )

    def test_rejects_any_torque_off_sample(self) -> None:
        audit = OpenCrAudit()
        for torque in (True, False, True):
            audit.receive_battery(12.0)
            audit.receive_torque(torque)
        self.assertIn(
            "torque", " ".join(audit.errors(min_samples=3, min_voltage=11.1))
        )

    def test_rejects_missing_samples(self) -> None:
        audit = OpenCrAudit()
        errors = " ".join(audit.errors(min_samples=3, min_voltage=11.1))
        self.assertIn("battery samples", errors)
        self.assertIn("torque samples", errors)


if __name__ == "__main__":
    unittest.main()
