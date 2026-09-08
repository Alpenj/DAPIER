#!/usr/bin/env python3
"""Deterministic, hardware-free A4 ChArUco target; no camera or motor access."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DEFINITION = {
    "schema_version": 1, "dictionary": "DICT_4X4_50", "squares_x": 10, "squares_y": 7,
    "square_length_m": 0.025, "marker_length_m": 0.018, "legacy_pattern": False,
    "paper_mm": [297, 210], "board_mm": [250, 175], "board_top_left_mm": [23.5, 17.5],
    "print_scale": "100 percent / actual size; disable fit-to-page",
    "physical_dimensions_verified": False,
    "purpose": "camera calibration target, not a camera calibration result",
}


def make_board():
    board = cv2.aruco.CharucoBoard((DEFINITION["squares_x"], DEFINITION["squares_y"]),
        DEFINITION["square_length_m"], DEFINITION["marker_length_m"],
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DEFINITION["dictionary"])))
    board.setLegacyPattern(DEFINITION["legacy_pattern"])
    return board


def target():
    board = make_board()
    page = Image.new("L", (2970, 2100), 255)  # 10 pixels/mm = 254 DPI, exact A4.
    page.paste(Image.fromarray(board.generateImage((2500, 1750), marginSize=0, borderBits=1)), (235, 175))
    draw = ImageDraw.Draw(page)
    font = ImageFont.load_default(size=24)
    draw.text((235, 70), "DAPIER ChArUco 10x7 | square 25 mm | marker 18 mm | PRINT ACTUAL SIZE 100%", fill=0, font=font)
    draw.line([(235, 1990), (1235, 1990)], fill=0, width=3)
    for x in (235, 1235):
        draw.line([(x, 1975), (x, 2005)], fill=0, width=3)
    draw.text((1280, 1970), "Measure this line: exactly 100 mm", fill=0, font=font)
    corners, ids, _, markers = cv2.aruco.CharucoDetector(board).detectBoard(np.asarray(page))
    if ids is None or markers is None or len(ids) != 54 or len(markers) != 35:
        raise RuntimeError("generated page does not detect all expected corners/markers")
    if set(ids.ravel()) != set(range(54)) or set(markers.ravel()) != set(range(35)):
        raise RuntimeError("generated target IDs differ from the board definition")
    return page


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if not args.self_test and args.output is None:
        parser.error("--output must name a new directory")
    page = target()
    assert page.size == (2970, 2100)
    if args.self_test:
        print("PASS: A4 254 DPI, 35 marker IDs and 54 ChArUco corner IDs detected")
        return
    args.output.mkdir(parents=True, exist_ok=False)
    png, pdf = args.output / "charuco-a4.png", args.output / "charuco-a4.pdf"
    page.save(png, dpi=(254, 254))
    page.convert("RGB").save(pdf, "PDF", resolution=254.0, quality=100)
    report = {**DEFINITION, "opencv_version": cv2.__version__,
              "sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (png, pdf)}}
    (args.output / "board.json").write_text(json.dumps(report, indent=2) + "\n")
    print(pdf)


if __name__ == "__main__":
    main()
