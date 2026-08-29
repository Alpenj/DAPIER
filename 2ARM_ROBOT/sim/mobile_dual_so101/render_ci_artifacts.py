#!/usr/bin/env python3
"""Render review artifacts for the simulation-only mobile dual SO-101 model."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any

import mujoco
from PIL import Image

from mobile_dual_so101 import (
    HUMANOID_HOME_ACTION,
    TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
    apply_control_as_pose,
    build_model,
    model_provenance,
    validate_model,
)


WIDTH = 640
HEIGHT = 480
EXTERNAL_VIEWS: tuple[tuple[str, float, float, float], ...] = (
    ("front", 180.0, -5.0, 1.05),
    ("front_left", 135.0, -12.0, 1.05),
    ("front_right", 225.0, -12.0, 1.05),
    ("top", 180.0, -60.0, 1.15),
)
LOOKAT = (-0.06, 0.0, 0.34)
SO101_ASSET_SOURCE = {
    "repository": "https://github.com/TheRobotStudio/SO-ARM100",
    "revision": "7629d2ad9853d10fb903093a33ef6114099d97e5",
    "path": "Simulation/SO101",
    "license": "Apache-2.0",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_png(path: Path, pixels: Any) -> dict[str, object]:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite artifact: {path}")
    Image.fromarray(pixels).save(path, format="PNG", optimize=False)
    return {
        "file": path.name,
        "sha256": _sha256(path),
        "width": WIDTH,
        "height": HEIGHT,
    }


def render_artifacts(output_dir: Path, model_path: Path | None) -> Path:
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "mujoco-artifacts.json"
    expected = [output_dir / f"{name}.png" for name, *_ in EXTERNAL_VIEWS]
    expected.extend((output_dir / "depth_camera_rgb.png", report_path))
    existing = [str(path) for path in expected if path.exists()]
    if existing:
        raise FileExistsError(
            "refusing to overwrite existing artifacts: " + ", ".join(existing)
        )

    model, source = build_model(
        arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
        mount_layout="tower",
        model_path=model_path,
    )
    validation = validate_model(model, source=source, smoke_steps=1000)
    data = mujoco.MjData(model)
    apply_control_as_pose(model, data, HUMANOID_HOME_ACTION)

    rendered: dict[str, dict[str, object]] = {}
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        for name, azimuth, elevation, distance in EXTERNAL_VIEWS:
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.lookat[:] = LOOKAT
            camera.azimuth = azimuth
            camera.elevation = elevation
            camera.distance = distance
            renderer.update_scene(data, camera=camera)
            rendered[name] = {
                **_write_png(output_dir / f"{name}.png", renderer.render()),
                "camera": {
                    "type": "free",
                    "lookat_m": LOOKAT,
                    "azimuth_deg": azimuth,
                    "elevation_deg": elevation,
                    "distance_m": distance,
                },
            }

        renderer.update_scene(data, camera="front_depth_camera")
        rendered["depth_camera_rgb"] = {
            **_write_png(output_dir / "depth_camera_rgb.png", renderer.render()),
            "camera": {"type": "model", "name": "front_depth_camera"},
        }

    report = {
        "schema_version": "dapier.mobile-dual-so101.ci-artifacts.v1",
        "source_head_commit": (
            os.environ.get("DAPIER_SOURCE_HEAD_SHA")
            or os.environ.get("GITHUB_SHA")
        ),
        "tested_commit": os.environ.get("GITHUB_SHA"),
        "mount_layout": "tower",
        "arm_mount_height_m": TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
        "home_action_radians": list(HUMANOID_HOME_ACTION),
        "asset_source": SO101_ASSET_SOURCE,
        "model": model_provenance(source),
        "validation": validation,
        "artifacts": rendered,
        "published": False,
        "control_authorized": False,
        "hardware_dispatch_authorized": False,
        "hardware_execution": False,
    }
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    args = parser.parse_args()
    report_path = render_artifacts(args.output_dir, args.model)
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
