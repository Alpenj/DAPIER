from __future__ import annotations

import ast
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_modules.health import (
    EdgeComputeHealth,
    EdgeResourceLimits,
    HealthIssue,
    HealthSeverity,
    RuntimeHealthSnapshot,
)


LIMITS = EdgeResourceLimits()


def edge(**overrides) -> EdgeComputeHealth:
    values = {
        "cpu_utilization": 0.30,
        "memory_utilization": 0.40,
        "temperature_c": 50.0,
        "control_loop_lag_ms": 2.0,
        "camera_queue_depth": 1,
        "under_voltage": False,
        "observation_age_ms": 10.0,
    }
    values.update(overrides)
    return EdgeComputeHealth(**values)


class HealthContractTest(unittest.TestCase):
    def test_nominal_edge_runtime_allows_motion(self) -> None:
        snapshot = RuntimeHealthSnapshot(sequence=1, edge_compute=edge())
        self.assertTrue(snapshot.motion_allowed(LIMITS))
        self.assertFalse(snapshot.load_shedding_required(LIMITS))

    def test_soft_load_requests_shedding_without_immediate_stop(self) -> None:
        snapshot = RuntimeHealthSnapshot(
            sequence=1,
            edge_compute=edge(cpu_utilization=0.75),
        )
        self.assertTrue(snapshot.motion_allowed(LIMITS))
        self.assertTrue(snapshot.load_shedding_required(LIMITS))

    def test_resource_pressure_alerts_without_blocking_motion(self) -> None:
        overloaded = (
            edge(cpu_utilization=0.95),
            edge(memory_utilization=0.95),
            edge(camera_queue_depth=4),
        )
        for health in overloaded:
            with self.subTest(health=health):
                snapshot = RuntimeHealthSnapshot(sequence=1, edge_compute=health)
                self.assertTrue(snapshot.motion_allowed(LIMITS))
                self.assertTrue(snapshot.load_shedding_required(LIMITS))
                self.assertTrue(snapshot.alert_codes(LIMITS))

    def test_realtime_electrical_or_stale_fault_blocks_motion(self) -> None:
        unsafe = (
            edge(temperature_c=81.0),
            edge(control_loop_lag_ms=21.0),
            edge(under_voltage=True),
            edge(observation_age_ms=201.0),
        )
        for health in unsafe:
            with self.subTest(health=health):
                snapshot = RuntimeHealthSnapshot(sequence=1, edge_compute=health)
                self.assertFalse(snapshot.motion_allowed(LIMITS))

    def test_component_fault_blocks_motion_and_is_reportable(self) -> None:
        snapshot = RuntimeHealthSnapshot(
            sequence=1,
            edge_compute=edge(),
            issues=(
                HealthIssue(
                    component="right_arm",
                    code="motor_over_temperature",
                    severity=HealthSeverity.FAULT,
                    detail="joint=2 temperature_c=82",
                ),
            ),
        )
        self.assertFalse(snapshot.motion_allowed(LIMITS))
        self.assertEqual(snapshot.fault_codes, ("right_arm:motor_over_temperature",))

    def test_contract_imports_no_runtime_backend(self) -> None:
        tree = ast.parse(
            (PROJECT_DIR / "mission_modules" / "health.py").read_text(
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
            {"mujoco", "rclpy", "serial", "psutil", "cv2"}.isdisjoint(imported)
        )


if __name__ == "__main__":
    unittest.main()
