#!/usr/bin/env python3
"""Image-only ChArUco intrinsics candidate. Never opens cameras or applies calibration."""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile

import cv2
import numpy as np

from prepare_camera_board import DEFINITION, make_board


def estimate(paths, measured_line_mm):
    max_line_mm = 100 * min(p / b for p, b in zip(DEFINITION["paper_mm"], DEFINITION["board_mm"]))
    if not np.isfinite(measured_line_mm) or not 0 < measured_line_mm <= max_line_mm:
        raise ValueError("measured 100-mm reference line must be positive, finite and fit this A4 target; check mm units")
    # ponytail: one reference line assumes uniform X/Y scaling; measure both axes before metric use.
    print_scale = measured_line_mm / 100
    if not 10 <= len(paths) <= 300:
        raise ValueError("provide 10..300 images from ONE unchanged camera configuration")
    board = make_board()
    detector = cv2.aruco.CharucoDetector(board)
    object_points, image_points, accepted, rejected = [], [], [], []
    hashes, pixels_seen, image_size = {}, set(), None
    for path in paths:
        contents = path.read_bytes()
        hashes[str(path)] = hashlib.sha256(contents).hexdigest()
        frame = cv2.imdecode(np.frombuffer(contents, np.uint8), cv2.IMREAD_UNCHANGED)
        # UNCHANGED preserves bit depth and does not apply EXIF rotation.
        if frame is None or frame.dtype != np.uint8:
            raise ValueError(f"cannot decode an 8-bit image: {path.name}")
        if frame.ndim == 2:
            gray = frame
        elif frame.ndim == 3 and frame.shape[2] in (3, 4):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY if frame.shape[2] == 3 else cv2.COLOR_BGRA2GRAY)
        else:
            raise ValueError(f"unsupported image channels: {path.name}")
        size = (gray.shape[1], gray.shape[0])
        if image_size is not None and size != image_size:
            raise ValueError("mixed image sizes; never resize calibration inputs implicitly")
        image_size = size
        digest = hashlib.sha256(gray.tobytes()).hexdigest()
        if digest in pixels_seen:
            rejected.append({"image": path.name, "reason": "duplicate decoded pixels"})
            continue
        pixels_seen.add(digest)
        corners, ids, _, _ = detector.detectBoard(gray)
        if ids is None or len(ids) < 12 or board.checkCharucoCornersCollinear(ids):
            rejected.append({"image": path.name, "reason": "need at least 12 non-collinear ChArUco corners"})
            continue
        if len(set(ids.ravel())) != len(ids) or np.any(ids < 0) or np.any(ids >= 54):
            raise ValueError("unexpected or duplicate ChArUco IDs")
        obj, img = board.matchImagePoints(corners, ids)
        if not np.isfinite(img).all():
            raise ValueError("non-finite corner coordinates")
        # Planar image correspondences stay unchanged; only metric board coordinates scale.
        object_points.append(obj * np.float32(print_scale))
        image_points.append(img)
        accepted.append({"image": path.name, "corners": len(ids)})
    if len(accepted) < 10:
        raise ValueError(f"only {len(accepted)} usable distinct views; need at least 10")
    result = cv2.calibrateCameraExtended(object_points, image_points, image_size, None, None,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9))
    rms, matrix, distortion, rotations, translations, std_intrinsics, _, view_errors = result
    if (not all(np.isfinite(x).all() for x in (matrix, distortion, std_intrinsics, view_errors))
            or not np.isfinite(rms) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0):
        raise ValueError("calibration is numerically invalid; acquire more varied views")
    normals = np.array([cv2.Rodrigues(r)[0][:, 2] for r in rotations])
    span_deg = float(np.rad2deg(np.arccos(np.clip(normals @ normals[0], -1, 1))).max())
    coverage = np.ptp(np.concatenate(image_points).reshape(-1, 2), axis=0) / image_size
    relative_focal_std = std_intrinsics.ravel()[:2] / [matrix[0, 0], matrix[1, 1]]
    # ponytail: these are acquisition heuristics, not a calibration certificate;
    # validate on unseen images/known dimensions before any runtime adoption.
    warnings = []
    if rms > 1:
        warnings.append("reprojection RMS exceeds 1 pixel")
    if span_deg < 15:
        warnings.append("board normal changes less than 15 degrees from the first view")
    if np.any(coverage < .5):
        warnings.append("corners cover less than half the image width or height")
    if np.any(relative_focal_std > .1):
        warnings.append("estimated focal-length standard deviation exceeds 10 percent")
    if not (0 <= matrix[0, 2] < image_size[0] and 0 <= matrix[1, 2] < image_size[1]):
        warnings.append("estimated principal point is outside the image")
    for row, error, rvec, tvec in zip(accepted, view_errors, rotations, translations):
        row.update(rms_px=float(error[0]), board_to_camera_rvec=rvec.ravel().tolist(),
                   board_to_camera_tvec_m=tvec.ravel().tolist())
    return {"schema_version": 2, "kind": "ChArUco pinhole intrinsics candidate",
        "opencv_version": cv2.__version__, "image_size_wh": list(image_size),
        "camera_matrix": matrix.tolist(), "distortion_order": ["k1", "k2", "p1", "p2", "k3"],
        "distortion_coefficients": distortion.ravel().tolist(), "fit_rms_px": float(rms),
        "focal_length_relative_stddev": relative_focal_std.tolist(),
        "board_normal_span_from_first_deg": span_deg, "corner_coverage_xy": coverage.tolist(),
        "accepted_images": accepted, "rejected_images": rejected, "input_sha256": hashes,
        "source_images_unchanged": all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p, h in hashes.items()),
        "nominal_board_definition": DEFINITION, "quality_warnings": warnings,
        "print_measurement": {"nominal_line_mm": 100, "measured_line_mm": float(measured_line_mm),
            "uniform_scale": print_scale, "uniform_xy_scale_verified": False,
            "derived_square_length_mm": DEFINITION["square_length_m"] * 1000 * print_scale,
            "derived_marker_length_mm": DEFINITION["marker_length_m"] * 1000 * print_scale,
            "derived_board_size_mm": [b * print_scale for b in DEFINITION["board_mm"]]},
        "candidate_only": True, "physical_camera_identity_verified": False,
        "unseen_image_validation_performed": False, "camera_to_robot_calibrated": False,
        "hardware_execution": False, "runtime_calibration_updated": False,
        "limitations": ["One physical camera, fixed focus/resolution/crop/rotation must be confirmed separately.",
            "Only the reference line is supplied; uniform X/Y print scale and flatness need separate checks.",
            "Per-view poses map board to optical camera, not camera to gripper/base.",
            "Do not use RGB intrinsics for H201 depth or undistort already rectified images twice."]}


def run(args):
    source, output = args.images.resolve(), args.output.resolve()
    if not source.is_dir() or output.exists() or source == output or source in output.parents or output in source.parents:
        raise ValueError("use an existing image folder and a separate, new output folder")
    if not args.capture_notes.strip():
        raise ValueError("record camera identity, focus, raw/rectified mode, crop and rotation in capture notes")
    paths = sorted(p for p in source.iterdir() if p.is_file() and p.suffix.lower() in (".png", ".jpg", ".jpeg"))
    report = estimate(paths, args.measured_line_mm)
    if not report["source_images_unchanged"]:
        raise ValueError("source images changed during calibration; output not written")
    report.update(camera_role=args.camera, capture_notes=args.capture_notes)
    output.mkdir(parents=True, exist_ok=False)
    (output / "intrinsics-candidate.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"accepted": len(report["accepted_images"]), "rejected": len(report["rejected_images"]),
        "fit_rms_px": report["fit_rms_px"], "quality_warnings": report["quality_warnings"],
        "candidate_only": True, "output": str(output)}, ensure_ascii=False))
    return 2 if report["quality_warnings"] else 0


def self_test():
    from types import SimpleNamespace
    board = make_board()
    texture = board.generateImage((1000, 700), marginSize=0, borderBits=1)
    pixels = np.float32([[0, 0], [999, 0], [999, 699], [0, 699]])
    object_corners = np.float32([[0, 0, 0], [.25, 0, 0], [.25, .175, 0], [0, .175, 0]])
    expected = np.array([[600., 0, 320], [0, 610., 240], [0, 0, 1]])
    with tempfile.TemporaryDirectory(prefix="charuco-offline-check-") as directory:
        root = Path(directory) / "images"
        root.mkdir()
        paths = []
        for i in range(20):
            rvec = np.array([-.4 + .2 * (i % 5), -.3 + .2 * (i // 5), .03 * (i % 3)])
            rotation = cv2.Rodrigues(rvec)[0]
            tvec = np.array([-.08 + .04 * (i % 5), -.04 + .027 * (i // 5), .5 + .03 * (i % 4)]) - rotation @ [.125, .0875, 0]
            projected = cv2.projectPoints(object_corners, rvec, tvec, expected, np.zeros(5))[0].reshape(4, 2)
            transform = cv2.getPerspectiveTransform(pixels, projected.astype(np.float32))
            frame = cv2.warpPerspective(texture, transform, (640, 480), borderValue=255)
            path = root / f"view-{i:02d}.png"
            assert cv2.imwrite(str(path), frame)
            paths.append(path)
        report = estimate(paths + [paths[0]], 100)
        actual = np.array(report["camera_matrix"])
        assert len(report["accepted_images"]) >= 18 and len(report["rejected_images"]) == 1, report
        assert np.allclose(actual[:2, :2], expected[:2, :2], rtol=.03, atol=1), actual
        assert np.max(np.abs(actual[:2, 2] - expected[:2, 2])) < 3, actual
        assert report["fit_rms_px"] < .5 and not report["quality_warnings"], report
        assert report["candidate_only"] and not report["camera_to_robot_calibrated"]
        scaled = estimate(paths, 95)
        assert np.allclose(scaled["camera_matrix"], actual, rtol=1e-4, atol=1e-3), scaled
        assert np.allclose(scaled["distortion_coefficients"], report["distortion_coefficients"], atol=1e-4)
        for nominal_view, scaled_view in zip(report["accepted_images"], scaled["accepted_images"]):
            assert np.allclose(scaled_view["board_to_camera_tvec_m"],
                               np.array(nominal_view["board_to_camera_tvec_m"]) * .95, rtol=1e-4, atol=1e-5)
        assert scaled["print_measurement"]["derived_square_length_mm"] == 23.75
        assert scaled["print_measurement"]["derived_board_size_mm"] == [237.5, 166.25]
        assert DEFINITION["board_mm"] == [250, 175]  # Never rewrite the nominal PDF definition.
        for invalid_line in (0, -95, 950, float("nan"), float("inf")):
            try:
                estimate(paths, invalid_line)
            except ValueError:
                pass
            else:
                raise AssertionError("invalid printed dimension accepted")
        args = SimpleNamespace(images=root, output=Path(directory) / "candidate",
                               measured_line_mm=95, camera="left_wrist",
                               capture_notes="SYNTHETIC: fixed optics, raw, no crop/rotation")
        assert run(args) == 0
        saved = json.loads((args.output / "intrinsics-candidate.json").read_text())
        assert saved["source_images_unchanged"] and saved["camera_role"] == "left_wrist"
        assert saved["print_measurement"]["measured_line_mm"] == 95
        assert not saved["print_measurement"]["uniform_xy_scale_verified"]
        try:
            run(args)
        except ValueError:
            pass
        else:
            raise AssertionError("existing output overwritten")
        small = root / "mixed-size.png"
        assert cv2.imwrite(str(small), np.zeros((240, 320), dtype=np.uint8))
        depth = root / "depth16.png"
        assert cv2.imwrite(str(depth), np.zeros((480, 640), dtype=np.uint16))
        for invalid in (paths[:2], [paths[0]] * 10, paths + [small], paths + [depth]):
            try:
                estimate(invalid, 95)
            except ValueError:
                pass
            else:
                raise AssertionError("insufficient/duplicate/mixed-resolution input accepted")
        print(json.dumps({"synthetic_only": True, "views": len(report["accepted_images"]),
                          "fit_rms_px": report["fit_rms_px"], "camera_matrix": actual.tolist()}))
    print("PASS: image calibration; 95/100 print scale preserves intrinsics and scales translation; invalid input rejection; no hardware")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, help="one camera's unchanged PNG/JPEG captures")
    parser.add_argument("--camera", choices=("left_wrist", "right_wrist"))
    parser.add_argument("--capture-notes", help="physical camera/configuration identity; raw/rectified, focus, crop, rotation")
    parser.add_argument("--measured-line-mm", type=float, help="actual length of the printed 100-mm line; assumes uniform X/Y scale")
    parser.add_argument("--output", type=Path, help="separate new folder, never auto-applied to a runtime")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif any(getattr(args, k) is None for k in ("images", "camera", "capture_notes", "measured_line_mm", "output")):
        parser.error("images, camera, capture-notes, measured-line-mm and new output are required")
    else:
        raise SystemExit(run(args))
