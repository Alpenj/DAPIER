"""Shared unit-explicit data types for mission capability boundaries."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    yaw_rad: float
    frame_id: str = "map"

    def validate(self) -> None:
        if not all(math.isfinite(value) for value in (self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError("Pose2D values must be finite")
        if not self.frame_id.strip() or len(self.frame_id) > 100:
            raise ValueError("Pose2D frame_id must contain 1 to 100 characters")


@dataclass(frozen=True)
class Pose3D:
    x_m: float
    y_m: float
    z_m: float
    qw: float
    qx: float
    qy: float
    qz: float
    frame_id: str = "map"

    def validate(self) -> None:
        values = (
            self.x_m,
            self.y_m,
            self.z_m,
            self.qw,
            self.qx,
            self.qy,
            self.qz,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Pose3D values must be finite")
        quaternion_norm = math.sqrt(
            self.qw * self.qw
            + self.qx * self.qx
            + self.qy * self.qy
            + self.qz * self.qz
        )
        if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-3):
            raise ValueError("Pose3D quaternion must be normalized")
        if not self.frame_id.strip() or len(self.frame_id) > 100:
            raise ValueError("Pose3D frame_id must contain 1 to 100 characters")


__all__ = ["Pose2D", "Pose3D"]
