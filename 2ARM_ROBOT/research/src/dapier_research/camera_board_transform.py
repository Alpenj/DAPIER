"""Offline composition of measured camera/board/arm transforms; no hardware I/O."""
from __future__ import annotations

from typing import Sequence

import numpy as np

from .vision_target import PinholeIntrinsics, VisionTargetError, _finite_transform


def camera_from_board_pose(rvec, tvec_m):
    """Convert existing OpenCV board→camera pose; tvec_m uses measured board metres."""
    import cv2

    rotation_vector = np.asarray(rvec, dtype=np.float64)
    translation = np.asarray(tvec_m, dtype=np.float64)
    if (rotation_vector.size != 3 or translation.size != 3
            or not np.isfinite(rotation_vector).all()
            or not np.isfinite(translation).all()):
        raise VisionTargetError("invalid_pose", "finite 3D board-to-camera pose required")
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rotation_vector.reshape(3))[0]
    transform[:3, 3] = translation.reshape(3)
    return _finite_transform(transform)


def camera_to_arm_chain(T_camera_from_board, T_arm_from_board, *,
                        board_to_arm_verified, board_to_arm_revision, arm_frame):
    """Compose a candidate using separately measured board→arm calibration.

    The verified flag records caller evidence; it does not authenticate calibration
    or establish camera identity, accuracy, freshness, reachability or clearance.
    """
    if (board_to_arm_verified is not True
            or not isinstance(board_to_arm_revision, str) or not board_to_arm_revision.strip()
            or not isinstance(arm_frame, str) or not arm_frame.strip()):
        raise VisionTargetError(
            "unverified_extrinsic", "measured board-to-arm transform, frame and revision required")
    camera_from_board = _finite_transform(T_camera_from_board)
    arm_from_board = _finite_transform(T_arm_from_board)
    # OpenCV pose maps board→camera. Invert it to compose camera→board→arm.
    arm_from_camera = arm_from_board @ np.linalg.inv(camera_from_board)
    return {
        "T_arm_from_camera": arm_from_camera.tolist(),
        "T_board_from_camera": np.linalg.inv(camera_from_board).tolist(),
        "arm_frame": arm_frame.strip(),
        "board_to_arm_revision": board_to_arm_revision.strip(),
        "candidate_only": True,
        "independently_validated": False,
        "control_authorized": False,
        "hardware_execution": False,
    }


def measured_board_to_left_arm_transform(delta_board_mm: Sequence[float]) -> np.ndarray:
    """Compose a candidate from arm-origin coordinates in the board frame (mm).

    The input is board-origin -> arm-origin, already expressed in the OpenCV
    board axes, not a vector in arm axes or a separate gravity-up convention.
    No implicit Z sign change is applied. With the candidate rotation below,
    translation is [dY, dX, dZ] / 1000. This arithmetic does not verify the
    physical axis directions or identify a motor-axis point with the MJCF base.
    """
    delta = np.asarray(delta_board_mm, dtype=np.float64)
    if delta.shape != (3,) or not np.all(np.isfinite(delta)):
        raise VisionTargetError("invalid_delta", "delta_board_mm must be finite 3D coordinates in mm")

    a_B = delta * 1e-3  # [dX, dY, dZ] in metres from board origin to arm shoulder pan axis

    R_A_from_B = np.array([
        [ 0.0, -1.0,  0.0],
        [-1.0,  0.0,  0.0],
        [ 0.0,  0.0, -1.0],
    ], dtype=np.float64)

    t_A_from_B = - (R_A_from_B @ a_B)

    T_arm_from_board = np.eye(4, dtype=np.float64)
    T_arm_from_board[:3, :3] = R_A_from_B
    T_arm_from_board[:3, 3] = t_A_from_B
    return _finite_transform(T_arm_from_board)


def known_cube_from_board(observation: dict) -> dict:
    """Reuse the saved-observation ray/plane method for a new rectified frame.

    Four cyclic top-face corners describe a confirmed 40 mm cube resting on the
    board plane. This computes a candidate; reprojection is not execution approval.
    """
    import cv2

    if (observation.get("camera_frame") != "os30a_rectified_left_optical"
            or observation.get("cube_side_m") != .04
            or observation.get("clock") != "unix_ns"
            or type(observation.get("timestamp_ns")) is not int
            or observation["timestamp_ns"] <= 0
            or not isinstance(observation.get("calibration_revision"), str)
            or not observation["calibration_revision"].strip()):
        raise ValueError("rectified frame, confirmed 40mm cube, timestamp and calibration revision required")
    intrinsics = PinholeIntrinsics(**observation["intrinsics"])
    intrinsics.validate()
    pixels = np.asarray(observation["top_corners_uv"], dtype=float)
    if (pixels.shape != (4, 2) or not np.isfinite(pixels).all()
            or np.any(pixels < 0) or np.any(pixels[:, 0] >= intrinsics.width)
            or np.any(pixels[:, 1] >= intrinsics.height)):
        raise ValueError("four finite top corners within the calibrated image required")
    edges = np.roll(pixels, -1, axis=0) - pixels
    following = np.roll(edges, -1, axis=0)
    cross = edges[:, 0]*following[:, 1] - edges[:, 1]*following[:, 0]
    if not (np.all(cross > 0) or np.all(cross < 0)):
        raise ValueError("top corners must form a cyclic convex quadrilateral")
    camera_board = _finite_transform(observation["T_camera_from_board"])
    datum_board = _finite_transform(observation["T_datum_from_board"])
    rotation, translation = camera_board[:3, :3], camera_board[:3, 3]
    if not np.allclose(datum_board[:3, 2], [0., 0., -1.], rtol=0, atol=1e-9):
        raise ValueError("current desk adapter requires board +Z into the horizontal desk")
    rays = np.column_stack(((pixels[:, 0]-intrinsics.cx)/intrinsics.fx,
                            (pixels[:, 1]-intrinsics.cy)/intrinsics.fy, np.ones(4)))
    denominator = rays @ rotation[:, 2]
    if np.any(np.abs(denominator) <= 1e-9):
        raise ValueError("top rays are parallel to the board plane")
    scale = (rotation[:, 2] @ translation - .04) / denominator
    if not np.isfinite(scale).all() or np.any(scale <= 0):
        raise ValueError("top plane intersections must be in front of the camera")
    points = (rays*scale[:, None] - translation) @ rotation
    surface = points.mean(axis=0)
    center = surface + [0., 0., .02]
    # Fit only XY/yaw; never change cube size, calibration or measured datum.
    if np.cross(points[1]-points[0], points[3]-points[0])[2] < 0:
        points, pixels = points[[0,3,2,1]], pixels[[0,3,2,1]]
    square = np.array([[-.02,-.02],[.02,-.02],[.02,.02],[-.02,.02]])
    u, _, vt = np.linalg.svd(square.T @ (points[:, :2]-surface[:2]))
    planar = u @ vt
    if np.linalg.det(planar) <= 0:
        raise ValueError("top corners do not define a proper cube rotation")
    fitted = np.column_stack((square @ planar + surface[:2], np.full(4, -.04)))
    optical = fitted @ rotation.T + translation
    if not np.isfinite(optical).all() or np.any(optical[:, 2] <= 0):
        raise ValueError("fitted cube must remain in front of the camera")
    projected = np.column_stack((intrinsics.fx*optical[:,0]/optical[:,2]+intrinsics.cx,
                                 intrinsics.fy*optical[:,1]/optical[:,2]+intrinsics.cy))
    cube_board = np.eye(3)
    cube_board[:2, :2] = planar.T
    rv = cv2.Rodrigues(datum_board[:3, :3] @ cube_board)[0].ravel()
    angle = float(np.linalg.norm(rv))
    quat = (np.r_[np.cos(angle/2), rv*np.sin(angle/2)/angle] if angle > 1e-12
            else np.array([1., 0., 0., 0.]))
    point_in_datum = lambda p: datum_board[:3, :3] @ p + datum_board[:3, 3]
    return {
        "rgb_timestamp_ns": observation["timestamp_ns"],
        "arm_mapping": {"candidate_frame":"left_motor1_datum", "physically_verified":False},
        "target_arm_xyz_candidate_m": point_in_datum(surface).tolist(),
        "target_position_semantics":"block_top_surface_center; pregrasp offset is separate",
        "scene_object": {"frame":"left_motor1_datum", "position_semantics":"object_center",
            "center_xyz_m":point_in_datum(center).tolist(), "size_m":[.04]*3,
            "quaternion_wxyz":quat.tolist()},
        "scene_support": {"frame":"left_motor1_datum", "normal_xyz":[0.,0.,1.],
            "top_z_m":float(datum_board[2,3]), "revision":observation["calibration_revision"]},
        "geometry_evidence": {"method":"known40mm_cube_on_board_plane", "direct_depth_used":False,
            "surface_center_optical_m":(rotation@surface+translation).tolist(),
            "cube_center_optical_m":(rotation@center+translation).tolist(),
            "maximum_corner_reprojection_px":float(np.max(np.linalg.norm(projected-pixels, axis=1))),
            "calibration_revision":observation["calibration_revision"],
            "hard_total_error_bound_claimed":False},
        "accepted_for_execution":False,
    }
