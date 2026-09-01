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


__all__ = ["Pose2D"]
