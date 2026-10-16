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
from .tf_provenance import (
    DetectionProvenance,
    SensorFrameLedger,
    VerifiedTransform,
    build_post_stop_observation,
)


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
    camera_link_in_base_footprint_m: Tuple[float, float, float],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Pure geometry: rotate an optical-frame point/normal into `camera_link` axes and add an
    explicitly supplied `camera_link` origin. C-2: there is NO default origin any more; the
    C-1 value (0.073, 0.0, 0.084) was neither the Waffle Pi URDF chain
    (base_footprint->base_link (0,0,0.010), base_link->camera_link (0.073,-0.011,0.084)) nor a
    measurement of the real camera mount. Real evidence must come from a TF2 lookup at the
    message time (`tf_provenance.VerifiedTransform`); this helper is for math regression only.
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
        arrival: NavigationArrivalEvidence,
        base_stop: BaseStopEvidence,
        initial_detection_optical: Optional[Tuple[float, float, float]] = None,
        initial_provenance: Optional[DetectionProvenance] = None,
        initial_transform: Optional[VerifiedTransform] = None,
        post_stop_detection_optical: Optional[Tuple[float, float, float]] = None,
        post_stop_normal_optical: Tuple[float, float, float] = (0.0, 0.0, -1.0),
        post_stop_provenance: Optional[DetectionProvenance] = None,
        post_stop_transform: Optional[VerifiedTransform] = None,
        sensor_frame_id: str = "camera_color_optical_frame",
        reobserve_checked_at_s: float = 0.0,
        scope: ApprovedSweepickScope = ApprovedSweepickScope.FULL_CHAIN_RECEIVE_CONTACT,
        require_full_handover: bool = False,
        handoff_mode: str = "INDEPENDENT_LOCAL_CAMERA_OBSERVATION",
        allow_mock_evidence: bool = False,
        sensor_frame_ledger: Optional[SensorFrameLedger] = None,
    ) -> IntegrationOutcomeRecord:
        """Validate navigation intent, run the 8-stage pipeline, and return the integration outcome.

        C-2 P0-3: the post-stop observation is built only from (a) detector provenance that names
        the sensor frame, EMA/latch state, anchor source, inlier count and RMS, and (b) a TF2
        transform looked up at that sensor time. When either is missing the pipeline reaches
        REOBSERVE and stops there with `POST_STOP_OBSERVATION_UNVERIFIED` / `TF_UNAVAILABLE...`;
        no value is synthesised and the arm is never requested.
        """
        resolved = self.resolve_navigation_goal(raw_intent_json)
        if not resolved["valid"]:
            return self.coordinator.reject_at_detect(
                error_code=resolved.get("error_code") or "COMMAND_VALIDATOR_REJECTED",
                detail=str(resolved["reason"]),
                block_subsequent=False,
            )

        initial_obs: Optional[CameraObservation3D] = None
        self.last_initial_rejection: Tuple[str, ...] = ()
        if initial_detection_optical is not None and initial_provenance is not None:
            initial_obs, self.last_initial_rejection = build_post_stop_observation(
                position_in_sensor_frame_m=initial_detection_optical,
                normal_in_sensor_frame=(0.0, 0.0, -1.0),
                sensor_frame_id=sensor_frame_id,
                provenance=initial_provenance,
                transform=initial_transform,
                ledger=None,
                allow_mock=allow_mock_evidence,
            )

        post_obs: Optional[CameraObservation3D] = None
        self.last_post_stop_rejection: Tuple[str, ...] = ()
        if post_stop_detection_optical is not None and post_stop_provenance is not None:
            post_obs, self.last_post_stop_rejection = build_post_stop_observation(
                position_in_sensor_frame_m=post_stop_detection_optical,
                normal_in_sensor_frame=post_stop_normal_optical,
                sensor_frame_id=sensor_frame_id,
                provenance=post_stop_provenance,
                transform=post_stop_transform,
                ledger=sensor_frame_ledger,
                allow_mock=allow_mock_evidence,
            )
        elif post_stop_detection_optical is not None:
            self.last_post_stop_rejection = ("POST_STOP_OBSERVATION_UNVERIFIED:provenance_missing",)

        if post_obs is None and self.last_post_stop_rejection:
            # Reach REOBSERVE through the normal stages and refuse there with the provenance reasons.
            return self._refuse_at_reobserve(
                initial_obs=initial_obs,
                arrival=arrival,
                base_stop=base_stop,
                reasons=self.last_post_stop_rejection,
                reobserve_checked_at_s=reobserve_checked_at_s,
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

    def _refuse_at_reobserve(
        self,
        *,
        initial_obs: Optional[CameraObservation3D],
        arrival: NavigationArrivalEvidence,
        base_stop: BaseStopEvidence,
        reasons: Tuple[str, ...],
        reobserve_checked_at_s: float,
    ) -> IntegrationOutcomeRecord:
        from .contracts import IntegrationStage, ManipulationResultStatus

        coord = self.coordinator
        blocked = coord._check_not_blocked("execute_integrated_task")
        if blocked is not None:
            return blocked
        coord.reset_for_next_cycle()
        coord.step_detect(initial_obs)
        coord.step_navigate()
        arr_abort = coord.step_arrived(arrival)
        if arr_abort is not None:
            return arr_abort
        stop_abort = coord.step_base_stopped(base_stop)
        if stop_abort is not None:
            return stop_abort
        coord._transition_to(IntegrationStage.REOBSERVE)
        return coord._abort_with_error(
            failed_stage=IntegrationStage.REOBSERVE,
            status=ManipulationResultStatus.OBSERVATION_FAILED,
            error_code="POST_STOP_OBSERVATION_UNVERIFIED",
            detail="Post-stop observation refused (" + "; ".join(reasons) + "); arm motion forbidden",
            block_subsequent=False,
            requires_recovery=False,
        )
