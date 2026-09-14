from __future__ import annotations

import ast
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_modules.manipulator import (
    ArmSide,
    ArmTelemetry,
    ManipulationAction,
    ManipulationRequest,
    ManipulatorStatus,
    PlanningBackend,
)
from mission_modules.types import Pose3D


def arm(side: ArmSide, *, hot: bool = False, error: bool = False) -> ArmTelemetry:
    return ArmTelemetry(
        side=side,
        joint_position_rad=(0.0,) * 5,
        joint_velocity_radps=(0.0,) * 5,
        motor_temperature_c=((80.0,) + (30.0,) * 4) if hot else (30.0,) * 5,
        motor_error=((True,) + (False,) * 4) if error else (False,) * 5,
        gripper_position_rad=0.3,
        gripper_holding=side == ArmSide.LEFT,
    )


def status(**overrides) -> ManipulatorStatus:
    values = {
        "left": arm(ArmSide.LEFT),
        "right": arm(ArmSide.RIGHT),
        "active_request_id": "pick-1",
        "trajectory_complete": True,
        "collision_free": True,
        "watchdog_ok": True,
        "object_lifted": True,
        "carry_pose_clear": True,
        "observation_age_ms": 10.0,
    }
    values.update(overrides)
    return ManipulatorStatus(**values)


class ManipulatorContractTest(unittest.TestCase):
    def test_pick_request_is_semantic_and_pose_bounded(self) -> None:
        request = ManipulationRequest(
            request_id="pick-1",
            action=ManipulationAction.PICK,
            preferred_arm=ArmSide.LEFT,
            target_pose_map=Pose3D(0.5, 0.1, 0.02, 1.0, 0.0, 0.0, 0.0),
            planning_backend=PlanningBackend.NATIVE_IK,
        )
        request.validate()
        self.assertNotIn("joint", request.__dataclass_fields__)

    def test_pick_and_place_require_target_pose(self) -> None:
        request = ManipulationRequest(
            request_id="pick-1",
            action=ManipulationAction.PICK,
            preferred_arm=ArmSide.LEFT,
            target_pose_map=None,
            planning_backend=PlanningBackend.MOVEIT2,
        )
        with self.assertRaisesRegex(ValueError, "target_pose_map"):
            request.validate()

    def test_motor_fault_and_temperature_fail_motion_gate(self) -> None:
        self.assertTrue(
            status().safe_for_motion(
                max_observation_age_ms=100.0,
                max_motor_temperature_c=70.0,
            )
        )
        for unsafe in (
            status(left=arm(ArmSide.LEFT, hot=True)),
            status(right=arm(ArmSide.RIGHT, error=True)),
        ):
            self.assertFalse(
                unsafe.safe_for_motion(
                    max_observation_age_ms=100.0,
                    max_motor_temperature_c=70.0,
                )
            )

    def test_verified_carry_requires_lift_hold_and_clear_envelope(self) -> None:
        self.assertTrue(status().verified_carry(max_observation_age_ms=100.0))
        self.assertFalse(
            status(object_lifted=False).verified_carry(
                max_observation_age_ms=100.0
            )
        )
        self.assertFalse(
            status(carry_pose_clear=False).verified_carry(
                max_observation_age_ms=100.0
            )
        )
        self.assertFalse(
            status(collision_free=False).verified_carry(
                max_observation_age_ms=100.0
            )
        )
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            status().verified_carry(max_observation_age_ms=float("inf"))

    def test_contract_imports_no_runtime_backend(self) -> None:
        tree = ast.parse(
            (PROJECT_DIR / "mission_modules" / "manipulator.py").read_text(
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
            {"mujoco", "moveit", "rclpy", "serial", "torch"}.isdisjoint(imported)
        )


if __name__ == "__main__":
    unittest.main()
