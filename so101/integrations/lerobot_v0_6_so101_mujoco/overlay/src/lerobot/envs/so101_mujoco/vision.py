# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Small, auditable RGB perception helpers for the SO-101 MuJoCo task.

The detector accepts a calibrated RGB camera (top or wrist). It does not read
the simulated cube body pose, depth buffer, segmentation IDs, or contacts. The
known top-plane height is a task calibration, equivalent to knowing the work
surface and object dimensions.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .env import CUBE_TOP_PLANE_Z_M, CameraCalibration


@dataclass(frozen=True)
class BlueCubeDetection:
    """Image-space evidence returned by the deterministic color detector."""

    center_pixel_xy: tuple[float, float]
    bbox_xyxy: tuple[int, int, int, int]
    pixel_count: int
    fill_fraction: float


@dataclass(frozen=True)
class CubeVisionEstimate:
    """World-space cube estimate derived from one calibrated RGB frame."""

    world_xyz: np.ndarray
    detection: BlueCubeDetection


def detect_blue_cube(rgb: np.ndarray, *, minimum_pixels: int | None = None) -> BlueCubeDetection:
    """Locate the visible bright-blue cube top without simulator labels.

    The center of the color-mask bounding box is used instead of its raw pixel
    centroid. This is less biased when a real or simulated gripper finger
    partially occludes one part of the top face.
    """
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"rgb must have shape (height, width, 3), got {image.shape}")
    if image.shape[0] < 8 or image.shape[1] < 8:
        raise ValueError(f"rgb image is too small for cube detection: {image.shape}")
    if not np.issubdtype(image.dtype, np.number):
        raise ValueError(f"rgb must contain numeric values, got {image.dtype}")

    values = image.astype(np.int16, copy=False)
    red, green, blue = values[..., 0], values[..., 1], values[..., 2]
    # The top camera's direct light makes the simulated blue material appear
    # cyan, while the green tray still has green > blue. Integer differences
    # keep that distinction without depending on a camera-specific exposure.
    mask = (blue > 180) & (green > 140) & (blue > green + 8) & (green > red + 80)
    rows, columns = np.nonzero(mask)
    required = (
        max(24, int(image.shape[0] * image.shape[1] * 0.0004))
        if minimum_pixels is None
        else int(minimum_pixels)
    )
    if required <= 0:
        raise ValueError("minimum_pixels must be positive")
    if columns.size < required:
        raise RuntimeError(f"Blue cube not found: {columns.size} matching pixels, need at least {required}")

    x_min, x_max = int(columns.min()), int(columns.max())
    y_min, y_max = int(rows.min()), int(rows.max())
    box_width = x_max - x_min + 1
    box_height = y_max - y_min + 1
    if box_width < 4 or box_height < 4:
        raise RuntimeError(
            f"Blue region is too small or thin to be a cube: bbox={(x_min, y_min, x_max, y_max)}"
        )
    box_area = box_width * box_height
    return BlueCubeDetection(
        center_pixel_xy=((x_min + x_max) / 2.0, (y_min + y_max) / 2.0),
        bbox_xyxy=(x_min, y_min, x_max, y_max),
        pixel_count=int(columns.size),
        fill_fraction=float(columns.size / box_area),
    )


def project_pixel_to_horizontal_plane(
    pixel_xy: tuple[float, float] | np.ndarray,
    calibration: CameraCalibration,
    *,
    plane_z_m: float,
) -> np.ndarray:
    """Intersect one calibrated pinhole ray with a horizontal world plane."""
    pixel = np.asarray(pixel_xy, dtype=np.float64)
    position = np.asarray(calibration.position, dtype=np.float64)
    rotation = np.asarray(calibration.rotation, dtype=np.float64)
    if pixel.shape != (2,):
        raise ValueError(f"pixel_xy must have shape (2,), got {pixel.shape}")
    if position.shape != (3,) or rotation.shape != (3, 3):
        raise ValueError("camera position and rotation must have shapes (3,) and (3, 3)")
    if (
        not np.all(np.isfinite(pixel))
        or not np.all(np.isfinite(position))
        or not np.all(np.isfinite(rotation))
    ):
        raise ValueError("pixel and camera calibration must be finite")
    if not np.isfinite(plane_z_m):
        raise ValueError("plane_z_m must be finite")
    if calibration.image_height <= 0 or calibration.image_width <= 0:
        raise ValueError("camera image dimensions must be positive")
    if not 0.0 < calibration.vertical_fov_degrees < 180.0:
        raise ValueError("camera vertical field of view must be in (0, 180) degrees")

    focal_pixels = 0.5 * calibration.image_height / np.tan(np.deg2rad(calibration.vertical_fov_degrees) / 2.0)
    center_x = (calibration.image_width - 1) / 2.0
    center_y = (calibration.image_height - 1) / 2.0
    camera_ray = np.array(
        [
            (pixel[0] - center_x) / focal_pixels,
            -(pixel[1] - center_y) / focal_pixels,
            -1.0,
        ],
        dtype=np.float64,
    )
    world_ray = rotation @ camera_ray
    if abs(world_ray[2]) < 1e-9:
        raise RuntimeError("Camera ray is parallel to the calibrated horizontal plane")
    distance = (float(plane_z_m) - position[2]) / world_ray[2]
    if distance <= 0:
        raise RuntimeError("Calibrated horizontal plane is behind the camera")
    return position + distance * world_ray


def estimate_blue_cube_world_position(
    rgb: np.ndarray,
    calibration: CameraCalibration,
    *,
    cube_top_plane_z_m: float = CUBE_TOP_PLANE_Z_M,
) -> CubeVisionEstimate:
    """Estimate cube XY from calibrated RGB and the known cube-top plane."""
    image = np.asarray(rgb)
    if image.shape[:2] != (calibration.image_height, calibration.image_width):
        raise ValueError(
            "RGB dimensions do not match camera calibration: "
            f"image={image.shape[:2]} calibration="
            f"{(calibration.image_height, calibration.image_width)}"
        )
    detection = detect_blue_cube(image)
    world_xyz = project_pixel_to_horizontal_plane(
        detection.center_pixel_xy,
        calibration,
        plane_z_m=cube_top_plane_z_m,
    )
    return CubeVisionEstimate(world_xyz=world_xyz, detection=detection)
