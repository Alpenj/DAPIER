"""Offline composition of measured camera/board/arm transforms; no hardware I/O."""
from __future__ import annotations

import numpy as np

from .vision_target import VisionTargetError, _finite_transform


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
