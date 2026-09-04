#!/usr/bin/env python3
"""Install the tracked DAPIER overlay into an official RoboTwin checkout."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import sys

import yaml

from build_dual_so101 import build, validate_with_sapien_subprocess


def _merge_yaml(path: Path, values: dict) -> None:
    current = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    current.update(values)
    path.write_text(yaml.safe_dump(current, sort_keys=False), encoding="utf-8")


def _apply_patch(repo: Path, patch: Path) -> None:
    check = subprocess.run(
        ["git", "apply", "--reverse", "--check", patch],
        cwd=repo,
        check=False,
    )
    if check.returncode == 0:
        return
    subprocess.run(["git", "apply", "--check", patch], cwd=repo, check=True)
    subprocess.run(["git", "apply", patch], cwd=repo, check=True)


def install(robotwin: Path, source_urdf: Path, mesh_source: Path) -> None:
    here = Path(__file__).resolve().parent
    overlay = here / "overlay"
    for patch in sorted((here / "patches").glob("*.patch")):
        _apply_patch(robotwin, patch)
    asset = robotwin / "assets/embodiments/dapier-dual-so101"
    urdf = build(source_urdf, mesh_source, asset)
    for source in overlay.rglob("*"):
        if source.is_file() and source.suffix != ".pyc":
            target = robotwin / source.relative_to(overlay)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    _merge_yaml(
        robotwin / "env_cfg/task_config/_embodiment_config.yml",
        {"dapier-dual-so101": {"file_path": "./assets/embodiments/dapier-dual-so101/"}},
    )
    _merge_yaml(
        robotwin / "env_cfg/task_config/_camera_config.yml",
        {
            "DAPIER_H201": {"fovy": 60, "w": 640, "h": 460},
            # ponytail: provisional FOV; replace after real wrist calibration.
            "DAPIER_WRIST": {"fovy": 60, "w": 320, "h": 240},
        },
    )
    subprocess.run(
        [sys.executable, "scripts/update_embodiment_config_path.py"],
        cwd=robotwin,
        check=True,
    )
    validate_with_sapien_subprocess(urdf)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robotwin", type=Path, required=True)
    parser.add_argument("--source-urdf", type=Path, required=True)
    parser.add_argument("--mesh-source", type=Path, required=True)
    args = parser.parse_args()
    install(
        args.robotwin.resolve(),
        args.source_urdf.resolve(),
        args.mesh_source.resolve(),
    )


if __name__ == "__main__":
    main()
