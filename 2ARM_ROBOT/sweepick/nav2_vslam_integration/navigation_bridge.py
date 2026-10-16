"""Navigation -> Manipulation Bridge reusing `llm_semantic_docker` modules.

Directly reuses:
- `command_validator.py` (`CommandValidator`, `ActionEnum`, `RobotIntent`, `ValidationResult`)
- `station_manager.py` (`StationManager`)
- `semantic_waypoints.yaml`, `docking_profiles.yaml`, `docking_stations.yaml`, `color_profiles.yaml`
- `vslam_navigation.launch.py` static TF (`camera_link -> camera_color_optical_frame`:
  roll=-pi/2, pitch=0, yaw=-pi/2)
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .contracts import (
    ApprovedSweepickScope,
    BaseStopEvidence,
    CameraObservation3D,
    IntegrationOutcomeRecord,
    NavigationArrivalEvidence,
)
from .pipeline_coordinator import VslamSweepickCoordinator


DEFAULT_NAV_REPO_ROOT = (
    Path.home()
    / "DAPIER/.local-workspaces/pro/vslam-nav2-c-integration-20261016"
)
FALLBACK_NAV_REPO_ROOT = (
    Path.home()
    / "DAPIER/.local-artifacts/deps/Dapier_project_visaul_slam"
)


def resolve_llm_semantic_docker_root(nav_repo_root: Optional[Path] = None) -> Path:
    """Locate `robot_ws/src/llm_semantic_docker` in the Navigation worktree."""
    candidates = []
    if nav_repo_root is not None:
        candidates.append(Path(nav_repo_root))
    candidates.extend([DEFAULT_NAV_REPO_ROOT, FALLBACK_NAV_REPO_ROOT])
    for root in candidates:
        pkg = root / "robot_ws/src/llm_semantic_docker"
        if (pkg / "llm_semantic_docker/command_validator.py").is_file():
            return pkg
    raise FileNotFoundError("Could not locate robot_ws/src/llm_semantic_docker")


def load_nav_module(module_name: str, file_path: Path):
    spec = importlib.util.spec_from_file_location(module_name, str(file_path))
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load module {module_name} from {file_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def optical_to_camera_link_rotation() -> np.ndarray:
    """Return 3x3 rotation R_camera_link_from_optical from `vslam_navigation.launch.py`.

    In `vslam_navigation.launch.py`:
      static_transform_publisher: camera_link -> camera_color_optical_frame
      roll = -1.5708 (-pi/2), pitch = 0.0, yaw = -1.5708 (-pi/2)
    For an optical point p_opt = (x_right, y_down, z_forward):
      p_camera_link = (z_forward, -x_right, -y_down).
    """
    return np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=float,
    )


def transform_optical_pose_to_base_footprint(
    position_optical_m: Tuple[float, float, float],
    normal_optical: Tuple[float, float, float] = (0.0, 0.0, -1.0),
    *,
    camera_link_in_base_footprint_m: Tuple[float, float, float] = (0.073, 0.0, 0.084),
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Transform a 3D point and surface normal from `camera_color_optical_frame` to `base_footprint`
    using the existing TurtleBot3 / `vslam_navigation.launch.py` TF2 chain.
    Does NOT invent any `base_footprint <-> arm_base` transform.
    """
    r_mat = optical_to_camera_link_rotation()
    p_opt = np.asarray(position_optical_m, dtype=float).reshape(3)
    n_opt = np.asarray(normal_optical, dtype=float).reshape(3)
    t_base_cam = np.asarray(camera_link_in_base_footprint_m, dtype=float).reshape(3)

    p_base = r_mat @ p_opt + t_base_cam
    n_base = r_mat @ n_opt
    norm = float(np.linalg.norm(n_base))
    if norm > 1e-9:
        n_base = n_base / norm
    return (
        (float(p_base[0]), float(p_base[1]), float(p_base[2])),
        (float(n_base[0]), float(n_base[1]), float(n_base[2])),
    )


class NavigationManipulationBridge:
    """Reuses `CommandValidator` and `StationManager` to validate navigation/docking intents
    and hand off to `VslamSweepickCoordinator` after arrival, base stop, and fresh re-observation.
    """

    def __init__(
        self,
        *,
        coordinator: VslamSweepickCoordinator,
        nav_repo_root: Optional[Path] = None,
    ) -> None:
        self.coordinator = coordinator
        self.pkg_root = resolve_llm_semantic_docker_root(nav_repo_root)
        self.config_dir = self.pkg_root / "config"
        self.templates_dir = self.pkg_root / "templates"

        cv_mod = load_nav_module(
            "dapier_nav_command_validator",
            self.pkg_root / "llm_semantic_docker/command_validator.py",
        )
        sm_mod = load_nav_module(
            "dapier_nav_station_manager",
            self.pkg_root / "llm_semantic_docker/station_manager.py",
        )
        self.CommandValidator = cv_mod.CommandValidator
        self.ActionEnum = cv_mod.ActionEnum
        self.StationManager = sm_mod.StationManager

        self.validator = self.CommandValidator(
            waypoints_file=str(self.config_dir / "semantic_waypoints.yaml"),
            profiles_file=str(self.config_dir / "docking_profiles.yaml"),
            colors_file=str(self.config_dir / "color_profiles.yaml"),
        )
        self.station_manager = self.StationManager(
            config_path=str(self.config_dir / "docking_stations.yaml"),
            templates_dir=str(self.templates_dir),
        )

    def resolve_navigation_goal(self, raw_intent: Any) -> Dict[str, Any]:
        """Validate LLM/operator intent using existing `CommandValidator` and `StationManager`."""
        res = self.validator.validate(raw_intent)
        if not res.is_valid:
            return {
                "valid": False,
                "error_code": res.error_code,
                "reason": res.error_msg,
                "resolved_task": None,
                "staging_pose_map": None,
                "docking_profile": None,
            }

        task = res.resolved_task
        action = res.action
        staging_pose = None
        docking_profile = None
        target_label = (
            task.get("waypoint_name")
            or task.get("station_name")
            or task.get("target_name")
            or action
        )

        if action == "NAVIGATE":
            tp = task.get("target_pose") or {}
            if "x" in tp and "y" in tp:
                staging_pose = (
                    float(tp.get("x", 0.0)),
                    float(tp.get("y", 0.0)),
                    math.radians(float(tp.get("yaw_deg", 0.0))),
                )
        elif action == "DOCK":
            docking_profile = task.get("docking_profile")
            sp = task.get("staging_pose")
            if isinstance(sp, dict) and "x" in sp and "y" in sp:
                staging_pose = (
                    float(sp.get("x", 0.0)),
                    float(sp.get("y", 0.0)),
                    math.radians(float(sp.get("yaw_deg", 0.0))),
                )
            elif task.get("profile_name"):
                staging_wp = (docking_profile or {}).get("staging_waypoint")
                wp = self.validator.waypoints.get(staging_wp) or {}
                if "x" in wp and "y" in wp:
                    staging_pose = (
                        float(wp.get("x", 0.0)),
                        float(wp.get("y", 0.0)),
                        math.radians(float(wp.get("yaw_deg", 0.0))),
                    )

        return {
            "valid": True,
            "error_code": "",
            "reason": "VALIDATED",
            "action": action,
            "target_label": target_label,
            "resolved_task": task,
            "staging_pose_map": staging_pose,
            "docking_profile": docking_profile,
        }

    def execute_integrated_task(
        self,
        *,
        raw_intent_json: str,
        initial_detection_optical: Tuple[float, float, float],
        initial_detected_at_s: float,
        initial_frame_seq: int,
        arrival: NavigationArrivalEvidence,
        base_stop: BaseStopEvidence,
        post_stop_detection_optical: Optional[Tuple[float, float, float]],
        post_stop_detected_at_s: Optional[float],
        post_stop_frame_seq: Optional[int],
        post_stop_inliers: int = 85,
        post_stop_rms_m: float = 0.008,
        post_stop_is_ema_from_motion: bool = False,
        post_stop_is_persistence_latched: bool = False,
        post_stop_detector_mode: str = "hsv",
        post_stop_qwen_bbox_at_s: Optional[float] = None,
        reobserve_checked_at_s: float = 0.0,
        scope: ApprovedSweepickScope = ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT,
        require_full_handover: bool = False,
        handoff_mode: str = "INDEPENDENT_LOCAL_CAMERA_OBSERVATION",
    ) -> IntegrationOutcomeRecord:
        """Validate navigation intent, run the 8-stage pipeline, and return the integration outcome."""
        resolved = self.resolve_navigation_goal(raw_intent_json)
        if not resolved["valid"]:
            return self.coordinator.reject_at_detect(
                error_code=resolved.get("error_code") or "COMMAND_VALIDATOR_REJECTED",
                detail=str(resolved["reason"]),
                block_subsequent=False,
            )

        init_pos_base, init_norm_base = transform_optical_pose_to_base_footprint(
            initial_detection_optical
        )
        initial_obs = CameraObservation3D(
            observation_id=f"pre_nav_seq_{initial_frame_seq}",
            frame_seq=initial_frame_seq,
            captured_at_s=initial_detected_at_s,
            clock_id=base_stop.clock_id,
            frame_id="base_footprint",
            position_in_base_footprint_m=init_pos_base,
            normal_in_base_footprint=init_norm_base,
            inlier_count=60,
            rms_m=0.012,
            detector_mode=post_stop_detector_mode,
        )

        post_obs: Optional[CameraObservation3D] = None
        if (
            post_stop_detection_optical is not None
            and post_stop_detected_at_s is not None
            and post_stop_frame_seq is not None
        ):
            post_pos_base, post_norm_base = transform_optical_pose_to_base_footprint(
                post_stop_detection_optical
            )
            post_obs = CameraObservation3D(
                observation_id=f"post_stop_seq_{post_stop_frame_seq}",
                frame_seq=post_stop_frame_seq,
                captured_at_s=post_stop_detected_at_s,
                clock_id=base_stop.clock_id,
                frame_id="base_footprint",
                position_in_base_footprint_m=post_pos_base,
                normal_in_base_footprint=post_norm_base,
                inlier_count=post_stop_inliers,
                rms_m=post_stop_rms_m,
                detector_mode=post_stop_detector_mode,
                is_ema_filtered_from_motion=post_stop_is_ema_from_motion,
                is_persistence_latched=post_stop_is_persistence_latched,
                qwen_bbox_captured_at_s=post_stop_qwen_bbox_at_s,
            )

        return self.coordinator.execute_pipeline(
            initial_detection=initial_obs,
            arrival=arrival,
            base_stop=base_stop,
            post_stop_observation=post_obs,
            reobserve_checked_at_s=reobserve_checked_at_s,
            scope=scope,
            require_full_handover=require_full_handover,
            handoff_mode=handoff_mode,
        )
