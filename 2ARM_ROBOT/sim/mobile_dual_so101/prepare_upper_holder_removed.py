#!/usr/bin/env python3
"""Remove the integrated SO-101 holder envelopes from the upper STEP solid."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re

try:
    import cadquery as cq
except ModuleNotFoundError as error:
    raise SystemExit(
        "CadQuery 2.6.1 is required to regenerate the holder-removed assets"
    ) from error


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_SOURCE = PROJECT_DIR / "assets" / "assem_base.step"
DEFAULT_OUTPUT_STEP = (
    PROJECT_DIR
    / "assets/alternatives/holder_removed_experiment/assem_base_upper_holder_removed.step"
)
DEFAULT_OUTPUT_STL = (
    PROJECT_DIR
    / "assets/alternatives/holder_removed_experiment/assem_base_upper_holder_removed.stl"
)
EXPECTED_SOURCE_SHA256 = (
    "f9f77f71a77f962aac3c7a3898bf5df7232b12982be3f16e3e2fc20d39c1bb3b"
)
UPPER_ASSEMBLY_Z_MIN_MM = 160.0
HOLDER_CUT_Y_INNER_MM = 65.0
HOLDER_CUT_Y_OUTER_MM = 127.5
HOLDER_CUT_LOCAL_Z_MIN_MM = 71.5
HOLDER_CUT_LOCAL_Z_MAX_MM = 156.5
CANONICAL_STEP_TIMESTAMP = "2000-01-01T00:00:00"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonicalize_step_header(path: Path) -> None:
    """Remove exporter wall-clock metadata while preserving STEP geometry."""

    text = path.read_text(encoding="utf-8")
    canonical, replacements = re.subn(
        r"(FILE_NAME\('Open CASCADE Shape Model',')[^']+(')",
        rf"\g<1>{CANONICAL_STEP_TIMESTAMP}\g<2>",
        text,
        count=1,
    )
    if replacements != 1:
        raise RuntimeError("cannot canonicalize generated STEP FILE_NAME timestamp")
    path.write_text(canonical, encoding="utf-8", newline="\n")


def generate(source: Path, output_step: Path, output_stl: Path) -> tuple[str, str]:
    source = source.resolve()
    if sha256(source) != EXPECTED_SOURCE_SHA256:
        raise RuntimeError(f"unexpected assem_base.step revision: {source}")

    solids = cq.importers.importStep(str(source)).solids().vals()
    upper_solids = [
        solid
        for solid in solids
        if solid.BoundingBox().zmin >= UPPER_ASSEMBLY_Z_MIN_MM - 0.1
    ]
    if len(upper_solids) != 2:
        raise RuntimeError(
            f"expected two upper solids in assem_base.step, found {len(upper_solids)}"
        )

    cut_width = HOLDER_CUT_Y_OUTER_MM - HOLDER_CUT_Y_INNER_MM
    cut_height = HOLDER_CUT_LOCAL_Z_MAX_MM - HOLDER_CUT_LOCAL_Z_MIN_MM
    cut_z = (
        UPPER_ASSEMBLY_Z_MIN_MM
        + (HOLDER_CUT_LOCAL_Z_MIN_MM + HOLDER_CUT_LOCAL_Z_MAX_MM) / 2.0
    )
    cutters = []
    for sign in (-1.0, 1.0):
        cut_y = sign * (
            HOLDER_CUT_Y_INNER_MM + HOLDER_CUT_Y_OUTER_MM
        ) / 2.0
        cutters.append(
            cq.Workplane("XY")
            .box(120.0, cut_width, cut_height)
            .translate((0.0, cut_y, cut_z))
            .val()
        )

    carved = []
    for solid in upper_solids:
        result = solid
        for cutter in cutters:
            result = result.cut(cutter)
        carved.append(
            result.translate(cq.Vector(0.0, 0.0, -UPPER_ASSEMBLY_Z_MIN_MM))
        )
    result = cq.Compound.makeCompound(carved)
    output_step.parent.mkdir(parents=True, exist_ok=True)
    output_stl.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(result, str(output_step))
    canonicalize_step_header(output_step)
    cq.exporters.export(
        result,
        str(output_stl),
        tolerance=0.01,
        angularTolerance=0.1,
    )
    return sha256(output_step), sha256(output_stl)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output-step", type=Path, default=DEFAULT_OUTPUT_STEP)
    parser.add_argument("--output-stl", type=Path, default=DEFAULT_OUTPUT_STL)
    args = parser.parse_args()
    step_digest, stl_digest = generate(
        args.source,
        args.output_step,
        args.output_stl,
    )
    print(f"{step_digest}  {args.output_step}")
    print(f"{stl_digest}  {args.output_stl}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
