# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Auditable camera poses used by the SO-101 MuJoCo environment."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

CAMERA_PROFILE_SCHEMA_VERSION = "dapier.so101.camera-profiles.v1"
WRIST_CAMERA_PROFILE_ID = "therobotstudio_integrated_32x32_mount_surface_v1"
TOP_CAMERA_PROFILE_ID = "top_sim_v1"


@dataclass(frozen=True)
class CameraProfile:
    """One camera pose plus enough provenance to avoid claiming false calibration."""

    profile_id: str
    camera_name: str
    parent_body: str
    position_m: np.ndarray
    xyaxes: np.ndarray
    vertical_fov_degrees: float
    provenance: dict[str, Any]
    verification: dict[str, Any]

    @property
    def physical_alignment_verified(self) -> bool:
        return self.verification.get("physical_alignment") is True


def camera_profiles_path() -> Path:
    return Path(__file__).resolve().parent / "assets" / "camera_profiles.json"


def load_camera_profile(profile_id: str) -> CameraProfile:
    """Load and validate a named profile from the repository-owned JSON contract."""
    payload = json.loads(camera_profiles_path().read_text(encoding="utf-8"))
    if payload.get("schema_version") != CAMERA_PROFILE_SCHEMA_VERSION:
        raise ValueError(f"Unsupported camera profile schema: {payload.get('schema_version')!r}")
    profiles = payload.get("profiles")
    if not isinstance(profiles, dict) or profile_id not in profiles:
        raise ValueError(f"Unknown SO-101 camera profile: {profile_id!r}")
    raw = profiles[profile_id]
    if not isinstance(raw, dict):
        raise ValueError(f"Camera profile {profile_id!r} must be an object")

    position = _vector(raw, "position_m", 3)
    xyaxes = _vector(raw, "xyaxes", 6)
    x_axis, y_axis = xyaxes[:3], xyaxes[3:]
    if not np.isclose(np.linalg.norm(x_axis), 1.0, atol=1e-6):
        raise ValueError(f"Camera profile {profile_id!r} x axis must be unit length")
    if not np.isclose(np.linalg.norm(y_axis), 1.0, atol=1e-6):
        raise ValueError(f"Camera profile {profile_id!r} y axis must be unit length")
    if not np.isclose(np.dot(x_axis, y_axis), 0.0, atol=1e-6):
        raise ValueError(f"Camera profile {profile_id!r} axes must be orthogonal")

    fovy = raw.get("vertical_fov_degrees")
    if isinstance(fovy, bool) or not isinstance(fovy, (int, float)) or not 0 < fovy < 180:
        raise ValueError(f"Camera profile {profile_id!r} vertical FOV must be in (0, 180)")
    provenance = raw.get("provenance")
    verification = raw.get("verification")
    if not isinstance(provenance, dict) or not isinstance(verification, dict):
        raise ValueError(f"Camera profile {profile_id!r} requires provenance and verification objects")
    if "physical_alignment" not in verification or not isinstance(verification["physical_alignment"], bool):
        raise ValueError(f"Camera profile {profile_id!r} must state physical_alignment")

    camera_name = raw.get("camera_name")
    parent_body = raw.get("parent_body")
    if not isinstance(camera_name, str) or not camera_name:
        raise ValueError(f"Camera profile {profile_id!r} requires camera_name")
    if not isinstance(parent_body, str) or not parent_body:
        raise ValueError(f"Camera profile {profile_id!r} requires parent_body")
    return CameraProfile(
        profile_id=profile_id,
        camera_name=camera_name,
        parent_body=parent_body,
        position_m=position,
        xyaxes=xyaxes,
        vertical_fov_degrees=float(fovy),
        provenance=dict(provenance),
        verification=dict(verification),
    )


def _vector(parent: dict[str, Any], key: str, length: int) -> np.ndarray:
    value = np.asarray(parent.get(key), dtype=np.float64)
    if value.shape != (length,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{key} must contain {length} finite numbers")
    return value
