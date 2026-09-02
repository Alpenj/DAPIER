#!/usr/bin/env python3
"""DAPIER scene profile for RGB-D shoe detection and reach planning.

The current tower camera geometry predates the vision-first policy path.  This
module constructs a *provisional simulation profile* with a 35-degree downward
camera tilt so the default near-floor shoe enters the real camera frustum.  The
same mounting angle must be measured and confirmed on the physical camera
before sim-to-real use.

No simulator object pose or segmentation ID is used to create the target.  The
object body pose remains available to tests only as an error-measurement oracle.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import math
from pathlib import Path
from typing import Iterator, Sequence

import numpy as np

import mobile_dual_so101
from shoe_task import ShoeTaskEnv
from vision_guided_reach import (
    SimBlueShoeDetector,
    VisionGuidedReachPlan,
    plan_vision_guided_reach_from_frame,
    render_rgbd_frame,
)


DAPIER_VISION_CAMERA_DOWN_TILT_RAD = math.radians(35.0)
DAPIER_VISION_CAMERA_PROFILE = "tower-near-floor-rgbd-v1"
DAPIER_VISION_DETECTOR = SimBlueShoeDetector(
    minimum_blue=100,
    blue_over_red=60,
    blue_over_green=40,
    minimum_component_pixels=6,
    ignore_top_fraction=0.20,
)


@contextmanager
def _temporary_tower_camera_tilt(down_tilt_rad: float) -> Iterator[None]:
    """Override model construction only, then restore the module constant."""

    if not math.isfinite(down_tilt_rad) or not 0.0 < down_tilt_rad < math.pi / 2.0:
        raise ValueError("down_tilt_rad must be finite and inside (0, pi/2)")
    original = mobile_dual_so101.TOWER_CAMERA_DOWN_TILT_RAD
    mobile_dual_so101.TOWER_CAMERA_DOWN_TILT_RAD = down_tilt_rad
    try:
        yield
    finally:
        mobile_dual_so101.TOWER_CAMERA_DOWN_TILT_RAD = original


def build_dapier_vision_env() -> ShoeTaskEnv:
    """Build the default shoe task with the explicit near-floor camera profile."""

    with _temporary_tower_camera_tilt(DAPIER_VISION_CAMERA_DOWN_TILT_RAD):
        return ShoeTaskEnv()


def capture_dapier_vision_plan(
    *,
    side: str = "left",
    width: int = 320,
    height: int = 240,
) -> tuple[ShoeTaskEnv, VisionGuidedReachPlan]:
    env = build_dapier_vision_env()
    env.reset(seed=0)
    frame = render_rgbd_frame(
        env.model,
        env.data,
        width=width,
        height=height,
        camera_name="front_depth_camera",
        target_body_name="tb3_base_link",
    )
    start_action = tuple(float(value) for value in env.data.ctrl)
    plan = plan_vision_guided_reach_from_frame(
        env.model,
        frame,
        start_action,
        side=side,
        detector=DAPIER_VISION_DETECTOR,
    )
    return env, plan


def write_vision_artifacts(
    output_dir: Path,
    *,
    width: int = 320,
    height: int = 240,
) -> Path:
    """Write RGB, mask and estimate metadata for human review.

    This helper deliberately writes no hardware credentials or commands.
    """

    from PIL import Image

    env = build_dapier_vision_env()
    env.reset(seed=0)
    frame = render_rgbd_frame(
        env.model,
        env.data,
        width=width,
        height=height,
        camera_name="front_depth_camera",
        target_body_name="tb3_base_link",
    )
    detection = DAPIER_VISION_DETECTOR.detect(frame.rgb)
    from vision_guided_reach import estimate_shoe_from_frame

    estimate = estimate_shoe_from_frame(frame, detector=DAPIER_VISION_DETECTOR)
    output_dir.mkdir(parents=True, exist_ok=True)
    Image.fromarray(frame.rgb, mode="RGB").save(output_dir / "vision_rgb.png")
    Image.fromarray((detection.mask.astype(np.uint8) * 255), mode="L").save(
        output_dir / "vision_mask.png"
    )
    report = {
        "schema_version": "dapier.vision-review-artifact.v1",
        "camera_profile": DAPIER_VISION_CAMERA_PROFILE,
        "camera_down_tilt_deg": math.degrees(DAPIER_VISION_CAMERA_DOWN_TILT_RAD),
        "estimate": estimate.as_dict(),
        "ground_truth_used_for_target": False,
        "control_authorized": False,
        "hardware_execution": False,
    }
    report_path = output_dir / "vision_target_report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the provisional DAPIER near-floor RGB-D planning profile."
    )
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--artifact-dir", type=Path)
    args = parser.parse_args(argv)

    if args.artifact_dir is not None:
        write_vision_artifacts(
            args.artifact_dir,
            width=args.width,
            height=args.height,
        )
    _, plan = capture_dapier_vision_plan(
        side=args.side,
        width=args.width,
        height=args.height,
    )
    print(json.dumps(plan.as_dict(), ensure_ascii=False, indent=2))
    return 0 if plan.planning_accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
