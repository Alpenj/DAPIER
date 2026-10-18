"""Which physical setup the executor works in. Selected by revision/sweepick_setup.json ({"setup": ...}); the environment
variable SWEEPICK_SETUP overrides it (tests). Both are bound sources: the choice is part of what a session ran.

DESK            the 2026-09/10 desk: the assembled dual desk scene, the top camera placed through the printed board and the
                operator's ruler record of the board against the left-arm datum (T_DATUM_FROM_BOARD), desk top -14.6 mm.
MOUNTED_FLOOR   2026-10-16: both arms on the TurtleBot3, the block on the floor. World = the robot base frame of the A
                runtime 'mobile' scene (sweepick_mounted_floor_scene.mjb, layout USER_DIMENSIONS_UNVERIFIED). The top camera
                is on the robot, so camera -> robot is fixed: its position and downward pitch are the scene's, its in-image
                rotation is image-up = robot forward / image-right = robot right (the scene camera's in-plane rotation is
                90 deg off: with it the observed board came out tilted 34 deg; with this one the board lies level at
                -12 mm and the model arms projected onto the top image fall on the real arms — run 20261016_164052_9b3c13).
                The board is not needed; when seen it is a diagnostic only."""
import json
import os
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DESK, MOUNTED_FLOOR = 'DESK', 'MOUNTED_FLOOR'
CHOICE_FILE = HERE / 'sweepick_setup.json'


def current():
    value = os.environ.get('SWEEPICK_SETUP')
    if not value:
        try:
            value = json.loads(CHOICE_FILE.read_text()).get('setup')
        except (OSError, ValueError, AttributeError):
            value = None
    if value not in (DESK, MOUNTED_FLOOR):
        raise RuntimeError(f'sweepick_setup: unknown or missing setup {value!r} ({CHOICE_FILE.name} or SWEEPICK_SETUP)')
    return value


SETUP = current()
MOUNTED_SCENE = HERE / 'sweepick_mounted_floor_scene.mjb'
FLOOR_Z = 0.0                                                   # the floor in the mounted scene (its 'table' top)
CAMERA = 'os30a_UNVERIFIED'


MEASURED_HEAD_EXTRINSIC = HERE / 'sweepick_head_extrinsic_measured.json'


def measured_head_extrinsic():
    """The measured top-camera pose (2026-10-18: ICP of the head depth on the arms, validated on poses and grasps not used in the
    fit), or None when the file is absent. A file that is present but malformed is an error, never a silent fallback."""
    if not MEASURED_HEAD_EXTRINSIC.exists():
        return None
    doc = json.loads(MEASURED_HEAD_EXTRINSIC.read_text())
    if doc.get('schema') != 'sweepick.head-extrinsic.v1':
        raise RuntimeError(f'{MEASURED_HEAD_EXTRINSIC.name}: unknown schema {doc.get("schema")!r}')
    T = np.asarray(doc['T_robot_from_camera'], float)
    if T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1]) or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6) or abs(np.linalg.det(T[:3, :3]) - 1) > 1e-6:
        raise RuntimeError(f'{MEASURED_HEAD_EXTRINSIC.name}: T_robot_from_camera is not a rigid transform')
    return T, doc


def mounted_camera_source():
    m = measured_head_extrinsic()
    return dict(source='MEASURED', file=MEASURED_HEAD_EXTRINSIC.name, status=m[1].get('status')) if m else dict(source='SCENE_CAMERA', camera=CAMERA, status='UNVERIFIED')


def mounted_robot_from_camera(m=None):
    measured = measured_head_extrinsic()
    if measured is not None:                                    # the measured pose replaces the scene camera (USER_DIMENSIONS_UNVERIFIED) when present
        return measured[0].copy()
    return scene_robot_from_camera(m)


def scene_robot_from_camera(m=None):
    """T (4x4) from the top camera's OpenCV frame (x right, y down, z forward) to the robot base frame of the mounted scene."""
    import mujoco
    m = m or mujoco.MjModel.from_binary_path(str(MOUNTED_SCENE))
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    c = m.camera(CAMERA).id
    p = d.cam_xpos[c].copy()
    optical = -d.cam_xmat[c].reshape(3, 3)[:, 2]                # a MuJoCo camera looks along its -z
    right = np.array([0.0, -1.0, 0.0])                          # image right = robot right
    right -= optical * float(np.dot(right, optical)); right /= np.linalg.norm(right)
    down = np.cross(optical, right)                             # OpenCV: z = x cross y  ->  y = z cross x
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = right, down, optical, p
    return T
