from __future__ import annotations

import ast
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_modules.manipulator import ArmSide
from mission_modules.tactile import (
    TactileChannelStatus,
    TactileLimits,
    TactileModality,
    TactileRigStatus,
    TactileRole,
)


LIMITS = TactileLimits()


def channel(
    role: TactileRole,
    *,
    enabled: bool = True,
    force_n: float = 3.0,
    slip_probability: float = 0.1,
    contact: bool = True,
    overpressure: bool = False,
    age_ms: float = 5.0,
) -> TactileChannelStatus:
    if not enabled:
        return TactileChannelStatus(
            role=role,
            enabled=False,
            modality=None,
            sensor_id="",
            calibration_id="",
            sequence=0,
            normal_force_n=0.0,
            slip_probability=0.0,
            contact=False,
            overpressure=False,
            valid=False,
            observation_age_ms=0.0,
        )
    return TactileChannelStatus(
        role=role,
        enabled=True,
        modality=TactileModality.NORMAL_FORCE,
        sensor_id=f"{role.value}-sensor",
        calibration_id=f"{role.value}-cal-v1",
        sequence=1,
        normal_force_n=force_n,
        slip_probability=slip_probability,
        contact=contact,
        overpressure=overpressure,
        valid=True,
        observation_age_ms=age_ms,
    )


def rig(left: TactileChannelStatus, right: TactileChannelStatus) -> TactileRigStatus:
    return TactileRigStatus(channels=(left, right))


class TactileContractTest(unittest.TestCase):
    def test_selected_gripper_contact_verifies_grasp(self) -> None:
        status = rig(
            channel(TactileRole.LEFT_GRIPPER),
            channel(TactileRole.RIGHT_GRIPPER, enabled=False),
        )
        self.assertTrue(status.grasp_verified(ArmSide.LEFT, LIMITS))
        self.assertFalse(status.grasp_verified(ArmSide.RIGHT, LIMITS))

    def test_slip_overpressure_contact_loss_and_stale_are_alerted(self) -> None:
        unsafe = (
            channel(TactileRole.LEFT_GRIPPER, slip_probability=0.8),
            channel(TactileRole.LEFT_GRIPPER, force_n=25.0, overpressure=True),
            channel(TactileRole.LEFT_GRIPPER, contact=False),
            channel(TactileRole.LEFT_GRIPPER, age_ms=60.0),
        )
        for left in unsafe:
            with self.subTest(left=left):
                status = rig(left, channel(TactileRole.RIGHT_GRIPPER, enabled=False))
                self.assertFalse(status.grasp_verified(ArmSide.LEFT, LIMITS))
                self.assertTrue(status.alert_codes(LIMITS))

    def test_missing_role_and_active_data_on_disabled_channel_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "each gripper role"):
            TactileRigStatus(
                channels=(channel(TactileRole.LEFT_GRIPPER),)
            ).validate()
        invalid = TactileChannelStatus(
            role=TactileRole.RIGHT_GRIPPER,
            enabled=False,
            modality=None,
            sensor_id="",
            calibration_id="",
            sequence=0,
            normal_force_n=1.0,
            slip_probability=0.0,
            contact=False,
            overpressure=False,
            valid=False,
            observation_age_ms=0.0,
        )
        with self.assertRaisesRegex(ValueError, "no active measurement"):
            invalid.validate()

    def test_contract_supports_sensor_modalities_without_backend_imports(self) -> None:
        self.assertEqual(
            set(TactileModality),
            {
                TactileModality.NORMAL_FORCE,
                TactileModality.TAXEL_ARRAY,
                TactileModality.SIX_AXIS_FORCE_TORQUE,
            },
        )
        tree = ast.parse(
            (PROJECT_DIR / "mission_modules" / "tactile.py").read_text(
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
            {"mujoco", "rclpy", "serial", "smbus", "spidev"}.isdisjoint(imported)
        )


if __name__ == "__main__":
    unittest.main()
