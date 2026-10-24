from __future__ import annotations

import ast
from dataclasses import fields
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_core import MissionLocation
from mission_modules.mobility import (
    MobilityPort,
    MobilityStatus,
    NavigationRequest,
)
from mission_modules.types import Pose2D


class FakeMobility:
    simulation_only = True

    def __init__(self) -> None:
        self.request: NavigationRequest | None = None
        self.stop_reason = ""

    def request_navigation(self, request: NavigationRequest) -> None:
        request.validate()
        self.request = request

    def read_status(self) -> MobilityStatus:
        return MobilityStatus(
            pose_map=Pose2D(0.0, 0.0, 0.0),
            linear_velocity_mps=0.0,
            angular_velocity_radps=0.0,
            active_goal=MissionLocation.A,
            goal_reached=True,
            watchdog_ok=True,
            observation_age_ms=10.0,
        )

    def safe_stop(self, reason: str) -> None:
        self.stop_reason = reason


class MobilityContractTest(unittest.TestCase):
    def test_public_request_has_no_wheel_level_command(self) -> None:
        names = {field.name for field in fields(NavigationRequest)}
        self.assertEqual(
            names,
            {
                "goal",
                "target_pose",
                "position_tolerance_m",
                "yaw_tolerance_rad",
                "timeout_s",
            },
        )
        self.assertFalse(any("wheel" in name for name in names))

    def test_goal_ready_requires_settled_fresh_watchdog_state(self) -> None:
        ready = MobilityStatus(
            pose_map=Pose2D(1.0, 0.0, 0.0),
            linear_velocity_mps=0.0,
            angular_velocity_radps=0.0,
            active_goal=MissionLocation.B,
            goal_reached=True,
            watchdog_ok=True,
            observation_age_ms=20.0,
        )
        self.assertTrue(ready.ready_at_goal(max_observation_age_ms=100.0))
        moving = MobilityStatus(
            **{
                **ready.__dict__,
                "linear_velocity_mps": 0.01,
            }
        )
        self.assertFalse(moving.ready_at_goal(max_observation_age_ms=100.0))
        stale = MobilityStatus(
            **{
                **ready.__dict__,
                "observation_age_ms": 101.0,
            }
        )
        self.assertFalse(stale.ready_at_goal(max_observation_age_ms=100.0))
        watchdog_failed = MobilityStatus(
            **{
                **ready.__dict__,
                "watchdog_ok": False,
            }
        )
        self.assertFalse(
            watchdog_failed.ready_at_goal(max_observation_age_ms=100.0)
        )

    def test_invalid_units_and_status_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            NavigationRequest(
                goal=MissionLocation.B,
                target_pose=Pose2D(1.0, 0.0, 0.0),
                timeout_s=0.0,
            ).validate()
        with self.assertRaisesRegex(ValueError, "Pose2D"):
            NavigationRequest(
                goal=MissionLocation.B,
                target_pose=(1.0, 0.0, 0.0),
            ).validate()
        with self.assertRaisesRegex(ValueError, "active_goal"):
            MobilityStatus(
                pose_map=Pose2D(0.0, 0.0, 0.0),
                linear_velocity_mps=0.0,
                angular_velocity_radps=0.0,
                active_goal=None,
                goal_reached=True,
                watchdog_ok=True,
                observation_age_ms=0.0,
            ).validate()

    def test_protocol_fixture_is_simulation_only(self) -> None:
        port = FakeMobility()
        self.assertIsInstance(port, MobilityPort)
        request = NavigationRequest(
            goal=MissionLocation.B,
            target_pose=Pose2D(1.0, 0.0, 0.0),
        )
        port.request_navigation(request)
        self.assertEqual(port.request, request)
        self.assertTrue(port.read_status().ready_at_goal(max_observation_age_ms=100.0))
        port.safe_stop("watchdog")
        self.assertEqual(port.stop_reason, "watchdog")
        self.assertTrue(port.simulation_only)

    def test_contract_imports_no_runtime_backend(self) -> None:
        forbidden = {"mujoco", "rclpy", "rospy", "serial"}
        imported: set[str] = set()
        for filename in ("types.py", "mobility.py"):
            tree = ast.parse(
                (PROJECT_DIR / "mission_modules" / filename).read_text(
                    encoding="utf-8"
                )
            )
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    imported.update(alias.name.split(".")[0] for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.module:
                    imported.add(node.module.split(".")[0])
        self.assertTrue(forbidden.isdisjoint(imported))


if __name__ == "__main__":
    unittest.main()
