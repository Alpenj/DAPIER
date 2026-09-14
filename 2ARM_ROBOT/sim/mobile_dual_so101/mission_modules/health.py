"""Runtime health and Raspberry Pi load-shedding contract."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol, runtime_checkable


class HealthSeverity(str, Enum):
    OK = "ok"
    DEGRADED = "degraded"
    FAULT = "fault"


@dataclass(frozen=True)
class EdgeResourceLimits:
    alert_cpu_utilization: float = 0.70
    alert_memory_utilization: float = 0.75
    max_temperature_c: float = 80.0
    max_control_loop_lag_ms: float = 20.0
    max_camera_queue_depth: int = 3
    max_observation_age_ms: float = 200.0

    def validate(self) -> None:
        fractions = (
            self.alert_cpu_utilization,
            self.alert_memory_utilization,
        )
        if not all(math.isfinite(value) and 0.0 < value <= 1.0 for value in fractions):
            raise ValueError("resource utilization limits must be inside (0, 1]")
        numeric = (
            self.max_temperature_c,
            self.max_control_loop_lag_ms,
            self.max_observation_age_ms,
        )
        if not all(math.isfinite(value) and value > 0 for value in numeric):
            raise ValueError("temperature, lag, and age limits must be positive")
        if (
            isinstance(self.max_camera_queue_depth, bool)
            or not isinstance(self.max_camera_queue_depth, int)
            or self.max_camera_queue_depth < 1
        ):
            raise ValueError("max_camera_queue_depth must be a positive integer")


@dataclass(frozen=True)
class EdgeComputeHealth:
    cpu_utilization: float
    memory_utilization: float
    temperature_c: float
    control_loop_lag_ms: float
    camera_queue_depth: int
    under_voltage: bool
    observation_age_ms: float

    def validate(self) -> None:
        for label, value in (
            ("cpu_utilization", self.cpu_utilization),
            ("memory_utilization", self.memory_utilization),
        ):
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{label} must be inside [0, 1]")
        numeric = (
            self.temperature_c,
            self.control_loop_lag_ms,
            self.observation_age_ms,
        )
        if not all(math.isfinite(value) and value >= 0 for value in numeric):
            raise ValueError("temperature, lag, and age must be non-negative")
        if (
            isinstance(self.camera_queue_depth, bool)
            or not isinstance(self.camera_queue_depth, int)
            or self.camera_queue_depth < 0
        ):
            raise ValueError("camera_queue_depth must be a non-negative integer")
        if not isinstance(self.under_voltage, bool):
            raise ValueError("under_voltage must be a boolean")

    def severity(self, limits: EdgeResourceLimits) -> HealthSeverity:
        self.validate()
        limits.validate()
        safety_fault = (
            self.temperature_c >= limits.max_temperature_c
            or self.control_loop_lag_ms >= limits.max_control_loop_lag_ms
            or self.under_voltage
            or self.observation_age_ms > limits.max_observation_age_ms
        )
        if safety_fault:
            return HealthSeverity.FAULT
        degraded = (
            self.cpu_utilization >= limits.alert_cpu_utilization
            or self.memory_utilization >= limits.alert_memory_utilization
            or self.camera_queue_depth > limits.max_camera_queue_depth
        )
        return HealthSeverity.DEGRADED if degraded else HealthSeverity.OK

    def resource_alert_codes(self, limits: EdgeResourceLimits) -> tuple[str, ...]:
        self.validate()
        limits.validate()
        alerts: list[str] = []
        if self.cpu_utilization >= limits.alert_cpu_utilization:
            alerts.append("edge_cpu_pressure")
        if self.memory_utilization >= limits.alert_memory_utilization:
            alerts.append("edge_memory_pressure")
        if self.camera_queue_depth > limits.max_camera_queue_depth:
            alerts.append("camera_queue_backlog")
        return tuple(alerts)


@dataclass(frozen=True)
class HealthIssue:
    component: str
    code: str
    severity: HealthSeverity
    detail: str = ""

    def validate(self) -> None:
        if not self.component.strip() or len(self.component) > 100:
            raise ValueError("health issue component must contain 1 to 100 characters")
        if not self.code.strip() or len(self.code) > 100:
            raise ValueError("health issue code must contain 1 to 100 characters")
        if not isinstance(self.severity, HealthSeverity):
            raise ValueError("health issue severity must be a HealthSeverity")
        if len(self.detail) > 500:
            raise ValueError("health issue detail is too long")


@dataclass(frozen=True)
class RuntimeHealthSnapshot:
    sequence: int
    edge_compute: EdgeComputeHealth
    issues: tuple[HealthIssue, ...] = ()

    def validate(self) -> None:
        if (
            isinstance(self.sequence, bool)
            or not isinstance(self.sequence, int)
            or self.sequence < 0
        ):
            raise ValueError("health sequence must be a non-negative integer")
        self.edge_compute.validate()
        for issue in self.issues:
            issue.validate()
        identities = [(issue.component, issue.code) for issue in self.issues]
        if len(set(identities)) != len(identities):
            raise ValueError("health issues must not contain duplicate component/code pairs")

    def motion_allowed(self, limits: EdgeResourceLimits) -> bool:
        self.validate()
        return (
            self.edge_compute.severity(limits) != HealthSeverity.FAULT
            and not any(issue.severity == HealthSeverity.FAULT for issue in self.issues)
        )

    def load_shedding_required(self, limits: EdgeResourceLimits) -> bool:
        self.validate()
        return (
            self.edge_compute.severity(limits) == HealthSeverity.DEGRADED
            or any(issue.severity == HealthSeverity.DEGRADED for issue in self.issues)
        )

    def alert_codes(self, limits: EdgeResourceLimits) -> tuple[str, ...]:
        self.validate()
        component_alerts = tuple(
            f"{issue.component}:{issue.code}"
            for issue in self.issues
            if issue.severity == HealthSeverity.DEGRADED
        )
        return self.edge_compute.resource_alert_codes(limits) + component_alerts

    @property
    def fault_codes(self) -> tuple[str, ...]:
        return tuple(
            f"{issue.component}:{issue.code}"
            for issue in self.issues
            if issue.severity == HealthSeverity.FAULT
        )


@runtime_checkable
class HealthPort(Protocol):
    @property
    def simulation_only(self) -> bool: ...

    def read_health(self) -> RuntimeHealthSnapshot: ...


__all__ = [
    "EdgeComputeHealth",
    "EdgeResourceLimits",
    "HealthIssue",
    "HealthPort",
    "HealthSeverity",
    "RuntimeHealthSnapshot",
]
