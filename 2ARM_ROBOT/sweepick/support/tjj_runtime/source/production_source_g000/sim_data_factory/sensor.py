"""Virtual camera observation and point-cloud generation for DAPIER SIM data factory.

Implements the sensor interface defined in integration_contract.md:
  sensor.observe(model, data, context) -> dict[camera_id, dict[field, np.ndarray | float]]

Owned exclusively by Gemini. Does not advance physics or mutate authoritative state.
"""
from __future__ import annotations

import math
from typing import Any, Mapping, Sequence
import numpy as np
import mujoco

CAMERA_CAPABILITY_MAP: dict[str, dict[str, Any]] = {
    "os30a_UNVERIFIED": {
        "real_correspondence": "HP-ASC-H201 (OS30A) overhead desk RGB-D camera",
        "attachment": "static_stand_plate",
        "appearance": "monochrome intensity replicated in3 channels; os30a-mono-v1",
        "parent_body": "world (stand plate pos=[0.0226, 0, 0.38], angle=-25 deg)",
        "modalities_supported_sim": ["rgb", "depth_m"],
        "real_hardware_status": "real OS30A provides RGB + Depth; SIM extrinsics/fovy are PROVISIONAL estimates",
        "optical_convention": "+X right, +Y down, +Z forward",
        "depth_definition": "orthogonal optical-Z distance in meters",
        "student_eligible": True,
    },
    "left_wrist_rgb": {
        "real_correspondence": "/dev/dapier/left_wrist_rgb (video10)",
        "attachment": "left_arm_wrist_gripper",
        "parent_body": "left_gripper",
        "modalities_supported_sim": ["rgb"],
        "real_hardware_status": "real hardware is RGB only; physical TCP offset unverified",
        "optical_convention": "+X right, +Y down, +Z forward",
        "depth_definition": "none (RGB only; depth omitted for student observation)",
        "student_eligible": True,
    },
    "right_wrist_rgb": {
        "real_correspondence": "/dev/dapier/right_wrist_rgb (video8)",
        "attachment": "right_arm_wrist_gripper",
        "parent_body": "right_gripper",
        "modalities_supported_sim": ["rgb"],
        "real_hardware_status": "real hardware is RGB only; physical TCP offset unverified",
        "optical_convention": "+X right, +Y down, +Z forward",
        "depth_definition": "none (RGB only; depth omitted for student observation)",
        "student_eligible": True,
    },
}

OBSERVATION_REVISION = "os30a-mono-v1"


def canonical_rgb(camera_id, image, *, channel_order="RGB"):
    """MuJoCo render is RGB; gray3 represents intensity, not a color/IR calibration."""
    image=np.asarray(image)
    if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:
        raise ValueError("uint8 HWC3 required")
    if channel_order not in ("RGB","BGR"):raise ValueError("unknown channel order")
    rgb=image if channel_order=="RGB" else image[...,::-1]
    if camera_id!="os30a_UNVERIFIED":return rgb.copy()
    if np.array_equal(rgb[...,0],rgb[...,1]) and np.array_equal(rgb[...,1],rgb[...,2]):return rgb.copy()
    gray=((rgb.astype(np.uint32)*[19595,38470,7471]).sum(-1)+32768)//65536
    return np.repeat(gray.astype(np.uint8)[...,None],3,axis=-1)


R_CAM_FROM_OPTICAL = np.diag([1.0, -1.0, -1.0])


def compute_camera_intrinsics(
    model: mujoco.MjModel,
    camera_name: str,
    width: int,
    height: int,
) -> np.ndarray:
    """Compute 3x3 intrinsic matrix K from MuJoCo camera vertical field of view."""
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id < 0:
        raise ValueError(f"camera not found in model: {camera_name}")
    fovy_deg = float(model.cam_fovy[cam_id])
    if not math.isfinite(fovy_deg) or fovy_deg <= 0.0 or fovy_deg >= 180.0:
        raise ValueError(f"invalid fovy: {fovy_deg}")
    fy = 0.5 * height / math.tan(math.radians(fovy_deg) / 2.0)
    fx = fy  # MuJoCo assumes square pixels
    cx = (float(width) - 1.0) / 2.0
    cy = (float(height) - 1.0) / 2.0
    return np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def compute_camera_optical_transform(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_name: str,
) -> np.ndarray:
    """Compute 4x4 homogeneous transform T_world_camera_optical.

    Converts MuJoCo camera coordinate (+X right, +Y up, -Z looking direction)
    to standard optical coordinate (+X right, +Y down, +Z looking direction).
    """
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id < 0:
        raise ValueError(f"camera not found in model: {camera_name}")
    cam_pos = data.cam_xpos[cam_id].copy()
    cam_mat = data.cam_xmat[cam_id].reshape(3, 3).copy()

    r_world_optical = cam_mat @ R_CAM_FROM_OPTICAL
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = r_world_optical
    transform[:3, 3] = cam_pos
    return transform


def depth_to_point_cloud(
    depth_m: np.ndarray,
    k: np.ndarray,
    t_world_camera_optical: np.ndarray,
    rgb: np.ndarray | None = None,
    min_depth: float = 0.05,
    max_depth: float = 2.5,
) -> dict[str, np.ndarray]:
    """Back-project metric optical-Z depth map to 3D point cloud in optical and world frames."""
    depth = np.asarray(depth_m, dtype=np.float64)
    height, width = depth.shape
    fx = float(k[0, 0])
    fy = float(k[1, 1])
    cx = float(k[0, 2])
    cy = float(k[1, 2])

    valid_mask = np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
    v_idx, u_idx = np.nonzero(valid_mask)
    z = depth[v_idx, u_idx]
    x = (u_idx.astype(np.float64) - cx) * z / fx
    y = (v_idx.astype(np.float64) - cy) * z / fy

    points_optical = np.column_stack([x, y, z])
    rot = t_world_camera_optical[:3, :3]
    trans = t_world_camera_optical[:3, 3]
    points_world = points_optical @ rot.T + trans

    result = {
        "points_optical": points_optical.astype(np.float32),
        "points_world": points_world.astype(np.float32),
        "valid_mask": valid_mask,
    }
    if rgb is not None:
        rgb_arr = np.asarray(rgb, dtype=np.uint8)
        result["colors"] = rgb_arr[v_idx, u_idx]
    return result


class SimulatedCameraSensor:
    """Multi-camera observation provider adhering to integration_contract.md."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        camera_ids: Sequence[str] = ("left_wrist_rgb", "right_wrist_rgb", "os30a_UNVERIFIED"),
        depth_camera_ids: Sequence[str] = ("os30a_UNVERIFIED",),
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.camera_ids = tuple(camera_ids)
        self.depth_camera_ids = set(depth_camera_ids)
        self._renderer: mujoco.Renderer | None = None
        self._current_model_id: int | None = None

    def _ensure_renderer(self, model: mujoco.MjModel) -> mujoco.Renderer:
        model_id = id(model)
        if self._renderer is None or self._current_model_id != model_id:
            if self._renderer is not None:
                try:
                    self._renderer.close()
                except Exception:
                    pass
            self._renderer = mujoco.Renderer(model, self.height, self.width)
            self._current_model_id = model_id
        return self._renderer

    def observe(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        context: Mapping[str, Any] | None = None,
    ) -> dict[str, dict[str, np.ndarray | float]]:
        """Render all configured cameras from the given MuJoCo state without mutating state."""
        del context  # Unused, conforms to contract interface
        renderer = self._ensure_renderer(model)
        capture_time = float(data.time)
        observations: dict[str, dict[str, np.ndarray | float]] = {}

        for cam_name in self.camera_ids:
            intrinsics = compute_camera_intrinsics(model, cam_name, self.width, self.height)
            transform = compute_camera_optical_transform(model, data, cam_name)

            renderer.disable_depth_rendering()
            renderer.update_scene(data, camera=cam_name)
            rgb = canonical_rgb(cam_name, renderer.render(), channel_order="RGB")

            cam_dict: dict[str, np.ndarray | float] = {
                "rgb": rgb,
                "K": intrinsics,
                "T_world_camera_optical": transform,
                "capture_time": capture_time,
            }

            if cam_name in self.depth_camera_ids:
                renderer.enable_depth_rendering()
                renderer.update_scene(data, camera=cam_name)
                depth_m = renderer.render().copy()
                renderer.disable_depth_rendering()
                cam_dict["depth_m"] = depth_m.astype(np.float32)

            observations[cam_name] = cam_dict

        return observations

    def close(self) -> None:
        """Release offscreen rendering context."""
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None
            self._current_model_id = None

    def __del__(self) -> None:
        self.close()


_default_sensor: SimulatedCameraSensor | None = None


def get_default_sensor() -> SimulatedCameraSensor:
    global _default_sensor
    if _default_sensor is None:
        _default_sensor = SimulatedCameraSensor()
    return _default_sensor


def observe(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    context: Mapping[str, Any] | None = None,
) -> dict[str, dict[str, np.ndarray | float]]:
    """Module-level observe endpoint compatible with sensor.observe(...) contracts."""
    return get_default_sensor().observe(model, data, context)
