"""Offline right-wrist intrinsic/eye-in-hand calibration from owner-supplied snapshots.

This module never opens a camera, bus, SDK, or robot model.  ``capture_snapshot``
accepts one already paired snapshot through a callback; ``solve_dataset`` only
reads the files that callback produced.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


SCHEMA = "sweepick.wrist-optical.v1"
CAPTURE_SCHEMA = "sweepick.wrist-optical-captures.v1"
SAMPLE_SCHEMA = "sweepick.wrist-optical-capture.v1"
CAMERA_KEY = "right_wrist"
TOOL_FRAME = "right_cube_grasp"
IMAGE_SIZE_HW = (240, 320)


def _finite(name, value, shape=None):
    a = np.asarray(value, dtype=float)
    if shape is not None and a.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {a.shape}")
    if not np.isfinite(a).all():
        raise ValueError(f"{name} must be finite")
    return a


def _rigid(name, value):
    T = _finite(name, value, (4, 4))
    R = T[:3, :3]
    if not np.allclose(T[3], [0, 0, 0, 1]) or not np.allclose(R.T @ R, np.eye(3)) or np.linalg.det(R) <= 0:
        raise ValueError(f"{name} must be a proper rigid transform")
    return T


def _binding(value):
    if not isinstance(value, dict) or set(("ref", "physical_pairing", "mode")) - set(value):
        raise ValueError("binding must contain ref, physical_pairing, and mode")
    if not value["ref"] or not value["physical_pairing"] or not value["mode"]:
        raise ValueError("binding ref, physical_pairing, and mode must be non-empty")
    mode = value["mode"]
    negotiated = mode.get("negotiated") if isinstance(mode, dict) else None
    if (not isinstance(negotiated, dict) or "requested" not in mode
            or set(("fourcc", "width", "height", "fps")) - set(negotiated)):
        raise ValueError("binding mode must preserve requested and negotiated fourcc/width/height/fps")
    if (negotiated["height"], negotiated["width"]) != IMAGE_SIZE_HW:
        raise ValueError("binding negotiated image size is not 320x240")
    return value


def _points(record):
    obj = _finite("object_points_m", record.get("object_points_m"))
    img = _finite("image_points_px", record.get("image_points_px"))
    if obj.ndim != 2 or obj.shape[1] != 3 or img.shape != (len(obj), 2) or len(obj) < 4:
        raise ValueError("each sample needs at least four paired 3-D/2-D board points")
    if np.linalg.matrix_rank(obj - obj.mean(axis=0)) != 2:
        raise ValueError("board object points must span one plane")
    return obj, img


def _validate_records(records):
    if not records:
        raise ValueError("no calibration samples")
    first = records[0]
    expected_binding = _binding(first.get("binding"))
    expected_kind = first.get("source_kind")
    expected_board = first.get("board")
    if not expected_kind:
        raise ValueError("source_kind is required and is preserved unchanged")
    if not isinstance(expected_board, dict) or not expected_board.get("dimension_source"):
        raise ValueError("known board dimensions and dimension_source are required")
    for i, record in enumerate(records):
        if record.get("schema") not in (None, SAMPLE_SCHEMA):
            raise ValueError(f"sample {i}: wrong schema")
        if record.get("camera_key") != CAMERA_KEY or record.get("tool_frame") != TOOL_FRAME:
            raise ValueError(f"sample {i}: camera_key/tool_frame binding differs")
        if tuple(record.get("image_size_hw") or ()) != IMAGE_SIZE_HW or record.get("encoding") not in ("BGR8", "RGB8"):
            raise ValueError(f"sample {i}: geometry requires unresized BGR8 or RGB8 320x240")
        if record.get("mode") != expected_binding["mode"] or record.get("binding") != expected_binding:
            raise ValueError(f"sample {i}: physical camera binding or mode differs")
        if record.get("source_kind") != expected_kind:
            raise ValueError(f"sample {i}: source_kind differs")
        if record.get("board") != expected_board:
            raise ValueError(f"sample {i}: board dimensions/source differ")
        if record.get("split") not in ("train", "holdout"):
            raise ValueError(f"sample {i}: split must be train or holdout")
        q = _finite("q", record.get("q"))
        if q.shape != (12,):
            raise ValueError(f"sample {i}: q must be the actual q12")
        if not record.get("frame_stamps") or not record.get("q_stamps"):
            raise ValueError(f"sample {i}: frame and actual-q stamps are required")
        if not isinstance(record.get("q_mapping"), dict) or not isinstance(record.get("geometry"), dict):
            raise ValueError(f"sample {i}: q_mapping and geometry provenance are required")
        _rigid("T_base_tool", record.get("T_base_tool"))
        _points(record)
    return expected_kind, expected_binding, expected_board


def _vij(H, i, j):
    hi, hj = H[:, i], H[:, j]
    return np.array((hi[0] * hj[0], hi[0] * hj[1] + hi[1] * hj[0], hi[1] * hj[1],
                     hi[2] * hj[0] + hi[0] * hj[2], hi[2] * hj[1] + hi[1] * hj[2], hi[2] * hj[2]))


def _intrinsic_design_rank(records):
    rows = []
    for record in records:
        obj, img = _points(record)
        H, _ = cv2.findHomography(obj[:, :2], img, 0)
        if H is None or not np.isfinite(H).all():
            raise ValueError("a training view does not determine a finite board homography")
        for row in (_vij(H, 0, 1), _vij(H, 0, 0) - _vij(H, 1, 1)):
            norm = np.linalg.norm(row)
            if norm == 0:
                raise ValueError("intrinsic calibration design is degenerate")
            rows.append(row / norm)
    return int(np.linalg.matrix_rank(np.asarray(rows)))


def _rotation_vector(R):
    return cv2.Rodrigues(np.asarray(R, dtype=float))[0].reshape(3)


def _hand_eye_axis_rank(tool_poses):
    motions = []
    for i in range(len(tool_poses)):
        for j in range(i + 1, len(tool_poses)):
            v = _rotation_vector(tool_poses[i][:3, :3].T @ tool_poses[j][:3, :3])
            if np.linalg.norm(v) > np.finfo(float).eps * 32:
                motions.append(v)
    return 0 if not motions else int(np.linalg.matrix_rank(np.asarray(motions)))


def _target_pose(record, K, D):
    obj, img = _points(record)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
        raise ValueError("solvePnP did not produce a finite target-to-camera pose")
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, tvec.reshape(3)
    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    error = np.linalg.norm(projected.reshape(-1, 2) - img, axis=1)
    return T, float(np.sqrt(np.mean(error ** 2)))


def _transform_residual(reference, value):
    delta = np.linalg.inv(reference) @ value
    cosine = float(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1, 1))
    return dict(translation_residual_m=float(np.linalg.norm(delta[:3, 3])),
                rotation_residual_deg=float(math.degrees(math.acos(cosine))))


def solve_records(records):
    """Solve from capture records. Holdout records never enter either calibration call."""
    source_kind, binding, board = _validate_records(records)
    train = [r for r in records if r["split"] == "train"]
    holdout = [r for r in records if r["split"] == "holdout"]
    if len(train) < 3:
        raise ValueError("OpenCV hand-eye calibration requires at least three training poses")
    intrinsic_rank = _intrinsic_design_rank(train)
    if intrinsic_rank < 5:
        raise ValueError(f"intrinsic calibration is unobservable: planar design rank {intrinsic_rank}, need 5")
    tool_poses = [_rigid("T_base_tool", r["T_base_tool"]) for r in train]
    axis_rank = _hand_eye_axis_rank(tool_poses)
    if axis_rank < 2:
        raise ValueError(f"hand-eye calibration is unobservable: relative rotation-axis rank {axis_rank}, need 2 non-parallel axes")

    object_points, image_points = zip(*[_points(r) for r in train])
    try:
        intrinsic_rms, K, D, _, _ = cv2.calibrateCamera(
            [p.astype(np.float32) for p in object_points],
            [p.astype(np.float32) for p in image_points],
            IMAGE_SIZE_HW[::-1], None, None)
    except cv2.error as e:
        raise ValueError(f"OpenCV calibrateCamera failed: {e}") from e
    if not np.isfinite(K).all() or not np.isfinite(D).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError("OpenCV intrinsic result is not finite with positive focal lengths")

    target_poses, train_reprojection = zip(*[_target_pose(r, K, D) for r in train])
    try:
        R, t = cv2.calibrateHandEye(
            [T[:3, :3] for T in tool_poses], [T[:3, 3] for T in tool_poses],
            [T[:3, :3] for T in target_poses], [T[:3, 3] for T in target_poses],
            method=cv2.CALIB_HAND_EYE_TSAI)
    except cv2.error as e:
        raise ValueError(f"OpenCV calibrateHandEye failed: {e}") from e
    T_tool_camera = np.eye(4)
    T_tool_camera[:3, :3], T_tool_camera[:3, 3] = R, np.asarray(t).reshape(3)
    _rigid("T_tool_camera", T_tool_camera)

    train_targets = [Tbt @ T_tool_camera @ Tct for Tbt, Tct in zip(tool_poses, target_poses)]
    reference_target = train_targets[0]
    train_static = [_transform_residual(reference_target, T) for T in train_targets]
    holdout_rows = []
    for record in holdout:
        Tct, reprojection = _target_pose(record, K, D)
        Tbt = _rigid("T_base_tool", record["T_base_tool"]) @ T_tool_camera @ Tct
        holdout_rows.append(dict(sample_id=record.get("sample_id"), reprojection_rmse_px=reprojection,
                                 target_static_consistency=_transform_residual(reference_target, Tbt)))

    finite = bool(np.isfinite(K).all() and np.isfinite(D).all() and np.isfinite(T_tool_camera).all())
    report = dict(
        status="SOLVED_OFFLINE_NOT_FIELD_VERIFIED",
        valid_finite=finite,
        observability=dict(training_poses=len(train), intrinsic_planar_design_rank=intrinsic_rank,
                           intrinsic_required_rank=5, hand_eye_relative_rotation_axis_rank=axis_rank,
                           hand_eye_required_axis_rank=2,
                           basis="planar intrinsic design rank; OpenCV hand-eye requires at least 3 poses with rotations about at least 2 non-parallel axes"),
        intrinsic=dict(opencv_rms_px=float(intrinsic_rms), training_reprojection_rmse_px=list(train_reprojection)),
        hand_eye=dict(method="CALIB_HAND_EYE_TSAI", training_target_static_consistency=train_static),
        holdout=dict(status="MEASURED" if holdout_rows else "NOT_PERFORMED: collect independently labelled holdout board views",
                     used_in_solve=False, samples=holdout_rows),
        residual_interpretation="pixel and transform-consistency residuals are diagnostics; they are not a physical millimetre error bound",
        input_provenance=dict(
            board=board,
            q_mapping_verified=all(r["q_mapping"].get("verified") is True for r in records),
            geometry_verified=all(r["geometry"].get("verified") is True for r in records),
            samples=[dict(sample_id=r.get("sample_id"), base_frame=r.get("base_frame"),
                          q_mapping=r["q_mapping"], geometry=r["geometry"]) for r in records]),
        timing_interpretation="frame and actual-q stamps are preserved; exposure simultaneity has not been independently measured")
    return dict(schema=SCHEMA, camera_key=CAMERA_KEY, tool_frame=TOOL_FRAME,
                K=K.tolist(), D=D.reshape(-1).tolist(), T_tool_camera=T_tool_camera.tolist(),
                image_size_hw=list(IMAGE_SIZE_HW), source_kind=source_kind, binding=binding,
                validation_report=report, live_execution_permitted=False,
                independent_field_verification_not_performed=True)


def _dictionary(name):
    if not isinstance(name, str) or not hasattr(cv2.aruco, name):
        raise ValueError(f"unknown OpenCV ArUco dictionary {name!r}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def _detect_board(image, encoding, board):
    if not isinstance(board, dict) or not board.get("dimension_source"):
        raise ValueError("board known dimensions and dimension_source are required")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY if encoding == "RGB8" else cv2.COLOR_BGR2GRAY)
    kind = board.get("kind")
    if kind == "charuco":
        squares = tuple(board.get("squares") or ())
        if len(squares) != 2:
            raise ValueError("Charuco board squares=[columns, rows] is required")
        model = cv2.aruco.CharucoBoard(squares, float(board["square_length_m"]),
                                      float(board["marker_length_m"]), _dictionary(board["dictionary"]))
        corners, ids, _, _ = cv2.aruco.CharucoDetector(model).detectBoard(gray)
        if ids is None or len(ids) < 4:
            raise ValueError("fewer than four Charuco corners were detected")
        obj = model.getChessboardCorners()[ids.reshape(-1)]
        img = corners.reshape(-1, 2)
    elif kind == "checkerboard":
        size = tuple(board.get("inner_corners") or ())
        if len(size) != 2:
            raise ValueError("checkerboard inner_corners=[columns, rows] is required")
        ok, corners = cv2.findChessboardCornersSB(gray, size)
        if not ok:
            raise ValueError("checkerboard corners were not detected")
        obj = np.zeros((size[0] * size[1], 3), np.float32)
        obj[:, :2] = np.mgrid[:size[0], :size[1]].T.reshape(-1, 2) * float(board["square_length_m"])
        img = corners.reshape(-1, 2)
    else:
        raise ValueError("board kind must be charuco or checkerboard")
    if not np.isfinite(obj).all() or not np.isfinite(img).all() or np.linalg.matrix_rank(obj - obj.mean(axis=0)) != 2:
        raise ValueError("detected board points are not a finite planar correspondence")
    return np.asarray(obj, float), np.asarray(img, float)


def capture_snapshot(provider, out_dir, *, split, board):
    """Read one owner-resident snapshot through ``provider`` and append it to a file dataset."""
    if split not in ("train", "holdout"):
        raise ValueError("split must be train or holdout")
    sample = provider()
    if not isinstance(sample, dict):
        raise ValueError("provider must return a mapping")
    if sample.get("camera_key") != CAMERA_KEY or sample.get("tool_frame") != TOOL_FRAME:
        raise ValueError("provider must bind right_wrist to right_cube_grasp")
    rgb = np.asarray(sample.get("rgb"))
    encoding = sample.get("encoding")
    if rgb.shape != (*IMAGE_SIZE_HW, 3) or rgb.dtype != np.uint8 or encoding not in ("BGR8", "RGB8"):
        raise ValueError("provider must supply unresized BGR8 or RGB8 uint8 320x240")
    binding = _binding(sample.get("binding"))
    if sample.get("mode") != binding["mode"]:
        raise ValueError("provider mode differs from binding mode")
    if sample.get("source_kind") != "REAL":
        raise ValueError("actual capture provider must preserve source_kind REAL")
    q = _finite("q", sample.get("q"))
    if q.shape != (12,):
        raise ValueError("provider q must be the actual q12")
    if not sample.get("frame_stamps") or not sample.get("q_stamps"):
        raise ValueError("provider must preserve frame and actual-q stamps")
    if not isinstance(sample.get("q_mapping"), dict) or not isinstance(sample.get("geometry"), dict):
        raise ValueError("provider must preserve q_mapping and geometry provenance")
    T_base_tool = _rigid("T_base_tool", sample.get("T_base_tool"))
    obj, img = _detect_board(rgb, encoding, board)

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    manifest_path = out / "dataset.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else dict(schema=CAPTURE_SCHEMA, samples=[])
    if manifest.get("schema") != CAPTURE_SCHEMA:
        raise ValueError("existing dataset has the wrong schema")
    index = len(manifest["samples"])
    stem = f"sample_{index:04d}_{split}"
    if (out / f"{stem}.json").exists() or (out / f"{stem}.png").exists():
        raise FileExistsError(f"capture target {stem} already exists")
    stored_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if encoding == "RGB8" else rgb
    image_path = out / f"{stem}.png"
    if not cv2.imwrite(str(image_path), stored_bgr):
        raise OSError("RGB capture PNG was not written")
    record = dict(schema=SAMPLE_SCHEMA, sample_id=stem, split=split, camera_key=CAMERA_KEY,
                  tool_frame=TOOL_FRAME, source_kind="REAL", image_file=f"{stem}.png",
                  image_sha256=hashlib.sha256(image_path.read_bytes()).hexdigest(),
                  image_size_hw=list(IMAGE_SIZE_HW), encoding=encoding, mode=sample["mode"], binding=binding,
                  frame_stamps=sample["frame_stamps"], q=q.tolist(), q_stamps=sample["q_stamps"],
                  base_frame=sample.get("base_frame"),
                  q_mapping=sample["q_mapping"], geometry=sample["geometry"],
                  T_base_tool=T_base_tool.tolist(), board=board,
                  object_points_m=obj.tolist(), image_points_px=img.tolist())
    (out / f"{stem}.json").write_text(json.dumps(record, indent=2))
    manifest["samples"].append(f"{stem}.json")
    manifest_tmp = manifest_path.with_suffix(".json.tmp")
    manifest_tmp.write_text(json.dumps(manifest, indent=2))
    manifest_tmp.replace(manifest_path)
    return record


def load_dataset(path):
    root = Path(path)
    manifest = json.loads((root / "dataset.json").read_text())
    if manifest.get("schema") != CAPTURE_SCHEMA or not isinstance(manifest.get("samples"), list):
        raise ValueError("dataset.json has the wrong schema")
    records = []
    for relative in manifest["samples"]:
        record = json.loads((root / relative).read_text())
        image_path = root / record.get("image_file", "")
        if not record.get("image_sha256") or hashlib.sha256(image_path.read_bytes()).hexdigest() != record["image_sha256"]:
            raise ValueError(f"{relative}: stored image SHA differs")
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None or image.shape != (*IMAGE_SIZE_HW, 3):
            raise ValueError(f"{relative}: stored image is missing or not raw 320x240 colour")
        records.append(record)
    return records


def solve_dataset(dataset, output=None):
    result = solve_records(load_dataset(dataset))
    if output is not None:
        output = Path(output)
        if output.exists():
            raise FileExistsError(f"calibration output already exists: {output}")
        output.write_text(json.dumps(result, indent=2))
    return result


def compare_static_records(reference, current, calibration):
    """Compare stationary snapshots without solving or approving a new calibration."""
    source_kind, binding, _ = _validate_records([reference, current])
    if (calibration.get("schema") != SCHEMA or calibration.get("source_kind") != source_kind
            or calibration.get("binding") != binding or calibration.get("tool_frame") != TOOL_FRAME
            or calibration.get("camera_key") != CAMERA_KEY):
        raise ValueError("static comparison calibration/source/mount identity differs")
    if reference.get("base_frame") != current.get("base_frame"):
        raise ValueError("static snapshots use different base frames")
    K = _finite("K", calibration.get("K"), (3, 3))
    D = _finite("D", calibration.get("D"))
    if min(K[0, 0], K[1, 1]) <= 0:
        raise ValueError("static comparison requires positive focal lengths")
    X = _rigid("T_tool_camera", calibration.get("T_tool_camera"))
    ref_tool = _rigid("reference T_base_tool", reference["T_base_tool"])
    cur_tool = _rigid("current T_base_tool", current["T_base_tool"])
    ref_board, ref_pixel_rms = _target_pose(reference, K, D)
    cur_board, cur_pixel_rms = _target_pose(current, K, D)
    reference_world_board = ref_tool @ X @ ref_board
    predicted_current_board = np.linalg.inv(cur_tool @ X) @ reference_world_board
    obj, pixels = _points(current)
    projected, _ = cv2.projectPoints(obj, _rotation_vector(predicted_current_board[:3, :3]),
                                    predicted_current_board[:3, 3], K, D)
    A = np.linalg.inv(cur_tool) @ ref_tool
    B = cur_board @ np.linalg.inv(ref_board)
    angle_A = _transform_residual(np.eye(4), A)["rotation_residual_deg"]
    angle_B = _transform_residual(np.eye(4), B)["rotation_residual_deg"]
    return dict(
        status="DIAGNOSTIC_NOT_CALIBRATION_OR_EXECUTION_PERMISSION", source_kind=source_kind,
        hardware_access=False, motor_writes=0, live_execution_permitted=False,
        per_frame_pnp_reprojection_rmse_px=dict(reference=ref_pixel_rms, current=cur_pixel_rms),
        fixed_board_projection_rmse_px=float(np.sqrt(np.mean(np.sum(
            (projected.reshape(-1, 2) - pixels) ** 2, axis=1)))),
        target_static_consistency=_transform_residual(reference_world_board, cur_tool @ X @ cur_board),
        relative_motion=dict(FK_angle_deg=angle_A, camera_PnP_angle_deg=angle_B,
                             angle_difference_deg=angle_A - angle_B,
                             AX_XB=_transform_residual(A @ X, X @ B)),
        actual_q_delta=(np.asarray(current["q"]) - np.asarray(reference["q"])).tolist(),
        predicted_current_board_pixels=projected.reshape(-1, 2).tolist(),
        reference_role="snapshot-derived world-board estimate; not independent ground truth",
        timing=[dict(sample_id=r.get("sample_id"), frame=r["frame_stamps"], actual_q=r["q_stamps"])
                for r in (reference, current)],
        interpretation=("Repeated static frames test repeatability/timing. Persistent between-pose mismatch "
                        "can still come from FK/q or pose-dependent K/D/PnP bias; it does not isolate one cause."),
        physical_error_bound_m=None)


def capture_description():
    return dict(
        callback="capture_snapshot(provider, out_dir, split='train|holdout', board=known_board)",
        provider_fields=dict(camera_key=CAMERA_KEY, tool_frame=TOOL_FRAME, source_kind="REAL",
                             rgb="uint8 colour, shape [240,320,3], before any ACT resize/crop", encoding="BGR8 or RGB8, exactly as supplied",
                             mode="requested YUYV 320x240 plus negotiated fourcc/width/height/fps from the resident reader",
                             binding=["ref", "physical_pairing", "mode"],
                             frame_stamps="frame seq, capture/receipt times and their clocks",
                             q="actual q12 paired to this frame", q_stamps="actual-feedback time/sequence and clock",
                             q_mapping="existing candidate q12 mapping metadata, including verified state",
                             geometry="existing FK/model geometry metadata, including verified state",
                             T_base_tool="4x4 base<-right_cube_grasp FK from that actual q"),
        collection=("Keep one known-dimension target stationary. Collect at least three training poses with board coverage, "
                    "distance/tilt diversity and rotations about at least two non-parallel axes; collect separately labelled "
                    "holdout poses that are not used by either calibration call. More diverse poses are recommended."),
        existing_board_reference=dict(kind="charuco", squares=[10, 7], square_length_m=0.024,
                                      marker_length_m=0.01728, dictionary="DICT_4X4_50",
                                      dimension_source="revision/sweepick_read03_capture.py: operator-confirmed 2026-09-18; pass explicitly after confirming the physical print"),
        restrictions=["no camera/serial/SDK reader in this tool", "no robot or motor action", "no inferred FOV/default K",
                      "no new marker generation or camera setting change", "holdout residuals are diagnostics, not physical bounds"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("capture-description", help="print the callback and one-pass field collection contract")
    solve = sub.add_parser("solve", help="solve a file dataset; performs no live I/O")
    solve.add_argument("dataset")
    solve.add_argument("output")
    compare = sub.add_parser("compare-static", help="diagnose two saved static snapshots; no hand-eye/intrinsic solve or live I/O")
    compare.add_argument("calibration")
    compare.add_argument("reference_snapshot")
    compare.add_argument("current_snapshot")
    compare.add_argument("output")
    args = parser.parse_args(argv)
    if args.command == "capture-description":
        print(json.dumps(capture_description(), indent=2))
    elif args.command == "compare-static":
        records = []
        for name in (args.reference_snapshot, args.current_snapshot):
            path = Path(name)
            record = json.loads(path.read_text())
            if hashlib.sha256((path.parent / record["image_file"]).read_bytes()).hexdigest() != record["image_sha256"]:
                raise ValueError(f"{path}: snapshot image SHA differs")
            records.append(record)
        result = compare_static_records(*records, json.loads(Path(args.calibration).read_text()))
        output = Path(args.output)
        with output.open("x") as stream:
            json.dump(result, stream, indent=2)
        print(json.dumps(dict(output=str(output), status=result["status"], live_execution_permitted=False)))
    else:
        result = solve_dataset(args.dataset, args.output)
        print(json.dumps(dict(output=args.output, schema=result["schema"], validation_report=result["validation_report"]), indent=2))


if __name__ == "__main__":
    main()
