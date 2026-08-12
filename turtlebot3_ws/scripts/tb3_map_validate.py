#!/usr/bin/env python3
"""Validate a Nav2 YAML + PGM map pair before saving or navigation."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import yaml


class MapValidationError(ValueError):
    pass


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MapValidationError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise MapValidationError(f"{name} must be finite")
    return result


def _next_pgm_token(data: bytes, offset: int) -> tuple[bytes, int]:
    while offset < len(data):
        if data[offset] in b" \t\r\n":
            offset += 1
            continue
        if data[offset] == ord("#"):
            newline = data.find(b"\n", offset)
            if newline < 0:
                raise MapValidationError("unterminated PGM comment")
            offset = newline + 1
            continue
        break
    start = offset
    while offset < len(data) and data[offset] not in b" \t\r\n#":
        offset += 1
    if start == offset:
        raise MapValidationError("truncated PGM header")
    return data[start:offset], offset


def _pgm_metadata(image_path: Path) -> tuple[int, int]:
    data = image_path.read_bytes()
    magic, offset = _next_pgm_token(data, 0)
    width_token, offset = _next_pgm_token(data, offset)
    height_token, offset = _next_pgm_token(data, offset)
    maxval_token, offset = _next_pgm_token(data, offset)
    if magic != b"P5":
        raise MapValidationError("map image must be binary PGM (P5)")
    try:
        width = int(width_token)
        height = int(height_token)
        maxval = int(maxval_token)
    except ValueError as error:
        raise MapValidationError("PGM dimensions/maxval must be integers") from error
    if width <= 0 or height <= 0:
        raise MapValidationError("PGM dimensions must be positive")
    if not 0 < maxval <= 65535:
        raise MapValidationError("PGM maxval must be between 1 and 65535")
    if offset >= len(data) or data[offset] not in b" \t\r\n":
        raise MapValidationError("PGM header has no raster separator")
    if data[offset : offset + 2] == b"\r\n":
        offset += 2
    else:
        offset += 1
    bytes_per_pixel = 1 if maxval < 256 else 2
    expected_bytes = width * height * bytes_per_pixel
    actual_bytes = len(data) - offset
    if actual_bytes != expected_bytes:
        raise MapValidationError(
            f"PGM raster size mismatch: expected {expected_bytes}, got {actual_bytes}"
        )
    return width, height


def validate_map(yaml_path: Path) -> tuple[int, int, float, Path]:
    yaml_path = yaml_path.expanduser().resolve(strict=True)
    try:
        document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise MapValidationError(f"cannot read map YAML: {error}") from error
    if not isinstance(document, dict):
        raise MapValidationError("map YAML root must be a mapping")

    image = document.get("image")
    if not isinstance(image, str) or not image.strip():
        raise MapValidationError("image must be a non-empty path")
    image_path = Path(image).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    try:
        image_path = image_path.resolve(strict=True)
    except OSError as error:
        raise MapValidationError(f"map image does not exist: {image_path}") from error
    if not image_path.is_file():
        raise MapValidationError(f"map image is not a file: {image_path}")

    mode = document.get("mode", "trinary")
    if mode not in {"trinary", "scale", "raw"}:
        raise MapValidationError(f"unsupported map mode: {mode!r}")
    resolution = _number(document.get("resolution"), "resolution")
    if resolution <= 0:
        raise MapValidationError("resolution must be positive")
    origin = document.get("origin")
    if not isinstance(origin, list) or len(origin) != 3:
        raise MapValidationError("origin must contain exactly three numbers")
    for index, value in enumerate(origin):
        _number(value, f"origin[{index}]")
    negate = document.get("negate")
    if negate not in (0, 1, False, True):
        raise MapValidationError("negate must be 0 or 1")
    occupied = _number(document.get("occupied_thresh"), "occupied_thresh")
    free = _number(document.get("free_thresh"), "free_thresh")
    if not 0 <= free < occupied <= 1:
        raise MapValidationError(
            "thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1"
        )

    width, height = _pgm_metadata(image_path)
    return width, height, resolution, image_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("--machine", action="store_true")
    args = parser.parse_args()
    try:
        width, height, resolution, image_path = validate_map(args.map_yaml)
    except (MapValidationError, OSError) as error:
        print(f"ERROR: invalid map: {error}", file=sys.stderr)
        return 1
    if args.machine:
        print(f"width={width}")
        print(f"height={height}")
        print(f"resolution={resolution:.12g}")
        print(f"image={image_path}")
    else:
        print(
            f"OK: valid map {width}x{height} @ {resolution:.12g}m/cell; "
            f"image={image_path}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
