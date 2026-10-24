"""Fail-closed TurtleBot wheel characterization CLI."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence


PHYSICAL_MOTION_DISABLED = (
    "physical motion is disabled until local safety integration is complete"
)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="TurtleBot wheel characterization (physical motion disabled)."
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--wheels-off-ground-confirmed", action="store_true")
    parser.add_argument("--stage-seconds", type=float, default=1.2)
    args, _ros_args = parser.parse_known_args(argv)
    if not args.wheels_off_ground_confirmed:
        parser.error("--wheels-off-ground-confirmed is required")
    if args.stage_seconds < 0.8:
        parser.error("--stage-seconds must be at least 0.8")
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    parser.error(PHYSICAL_MOTION_DISABLED)


if __name__ == "__main__":
    raise SystemExit(main())
