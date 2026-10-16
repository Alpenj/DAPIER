"""P0 & P1 integration contracts for VSLAM/Nav2 x Sweepick Visual Servo."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import math
from typing import Any, Dict, List, Optional, Tuple


class IntegrationStage(str, Enum):
    """P0 mandatory 8-stage pipeline."""

    DETECT = "DETECT"
    NAVIGATE = "NAVIGATE"
    ARRIVED = "ARRIVED"
    BASE_STOPPED = "BASE_STOPPED"
    REOBSERVE = "REOBSERVE"
    MANIPULATION_READY = "MANIPULATION_READY"
    ARM_TASK = "ARM_TASK"
    RESULT = "RESULT"


ALLOWED_STAGE_TRANSITIONS: Dict[IntegrationStage, Tuple[IntegrationStage, ...]] = {
    IntegrationStage.DETECT: (IntegrationStage.NAVIGATE, IntegrationStage.RESULT),
    IntegrationStage.NAVIGATE: (IntegrationStage.ARRIVED, IntegrationStage.RESULT),
    IntegrationStage.ARRIVED: (IntegrationStage.BASE_STOPPED, IntegrationStage.RESULT),
    IntegrationStage.BASE_STOPPED: (IntegrationStage.REOBSERVE, IntegrationStage.RESULT),
    IntegrationStage.REOBSERVE: (IntegrationStage.MANIPULATION_READY, IntegrationStage.RESULT),
    IntegrationStage.MANIPULATION_READY: (IntegrationStage.ARM_TASK, IntegrationStage.RESULT),
    IntegrationStage.ARM_TASK: (IntegrationStage.RESULT,),
    IntegrationStage.RESULT: (),
}


def validate_stage_transition(current: IntegrationStage, nxt: IntegrationStage) -> None:
    """Enforce strict sequential progression across the P0 state machine."""
    allowed = ALLOWED_STAGE_TRANSITIONS.get(current, ())
    if nxt not in allowed:
        raise ValueError(
            f"Illegal pipeline transition {current.value} -> {nxt.value}; "
            f"allowed targets: {[s.value for s in allowed]}"
        )


class ManipulationResultStatus(str, Enum):
    """P1 mandatory execution outcome classification."""

    WAITING_FOR_OPERATOR_APPROVAL = "WAITING_FOR_OPERATOR_APPROVAL"
    COMPLETED = "COMPLETED"                 # 작업 완료 (requires confirmed LEFT PICK/LIFT or RIGHT handover)
    PARTIAL_COMPLETED = "PARTIAL_COMPLETED" # 부분 완료 (limited scope holding or diagnostic/non-handover completion)
    OBSERVATION_FAILED = "OBSERVATION_FAILED"  # 관측 실패 (post-stop REOBSERVE failure or Sweepick look/observe failure)
    EXECUTION_REJECTED = "EXECUTION_REJECTED"  # 실행 거부 (validator/owner/manifest/TF/recovery gate rejection)
    STOP = "STOP"                           # STOP (operator or owner STOP latch)
    FAULT = "FAULT"                         # FAULT (servo/bus fault, missing motor, or unconfirmed torque-off)


class ApprovedSweepickScope(str, Enum):
    """Approved execution scopes defined by Sweepick Console & Native owner."""

    FULL_CHAIN_RECEIVE_CONTACT = "FULL_CHAIN_RECEIVE_CONTACT"
    LEFT_START_TO_LIFT = "LEFT_START_TO_LIFT"
    RIGHT_X_FINITE_METROLOGY = "RIGHT_X_FINITE_METROLOGY"


NAV_TF2_FRAMES: Tuple[str, ...] = (
    "map",
    "odom",
    "base_footprint",
    "base_link",
    "camera_link",
    "camera_color_optical_frame",
)

SWEEPICK_MANIPULATION_FRAMES: Tuple[str, ...] = (
    "assembled dual model world (left base at its model pose)",
    "aruco_board",
    "arm_datum",
    "left_cube_grasp",
    "right_cube_grasp",
    "CURRENT_TOOL",
)


@dataclass(frozen=True)
class HardwareReadinessState:
    """Explicitly separates device-free integration verification from physical hardware readiness."""

    integration_mode: str = "MOCK_DEVICE_FREE"  # "MOCK_DEVICE_FREE" or "PHYSICAL_FIELD"
    base_to_arm_mounting_verified: bool = False
    base_to_arm_tf_calibrated: bool = False
    nav_cam_to_top_cam_extrinsics_verified: bool = False
    wrist_camera_cover_test_confirmed: bool = False
    operator_start_approved: bool = False

    def physical_cross_tree_tf_ready(self) -> bool:
        return (
            self.base_to_arm_mounting_verified
            and self.base_to_arm_tf_calibrated
            and self.nav_cam_to_top_cam_extrinsics_verified
        )

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["physical_cross_tree_tf_ready"] = self.physical_cross_tree_tf_ready()
        return data


@dataclass(frozen=True)
class CoordinateFrameContract:
    """Enforces P0 TF2 and coordinate frame rules without inventing unverified transforms."""

    nav_global_frame: str = "map"
    nav_odom_frame: str = "odom"
    nav_base_frame: str = "base_footprint"
    nav_camera_frame: str = "camera_color_optical_frame"
    sweepick_world_frame: str = "assembled dual model world (left base at its model pose)"
    sweepick_tool_frame: str = "CURRENT_TOOL"
    hardware_readiness: HardwareReadinessState = field(default_factory=HardwareReadinessState)

    def validate_nav_frame(self, frame_id: str) -> Tuple[bool, Optional[str]]:
        if frame_id not in NAV_TF2_FRAMES:
            return False, f"UNKNOWN_NAV_TF2_FRAME:{frame_id}"
        return True, None

    def validate_handoff_mode(
        self,
        *,
        handoff_mode: str,
        target_frame: str,
    ) -> Tuple[bool, Optional[str]]:
        """Validate how Navigation passes control to Sweepick at MANIPULATION_READY.

        - 'INDEPENDENT_LOCAL_CAMERA_OBSERVATION': Navigation verifies arrival, base stop, and
          post-stop target presence in `base_footprint`, then triggers Sweepick to run its own
          calibrated local observation (`top_rgb`/`top_depth` ArUco board chain + wrist cameras).
          This requires NO fabricated `base_footprint -> arm_base` transform.
        - 'DIRECT_BASE_TO_ARM_METRIC_TF': Attempts to transform a `base_footprint` 3D point directly
          into `arm_base` / `assembled dual model world`. Strictly forbidden unless physical
          Base-Arm mounting & TF calibration have been verified.
        """
        if handoff_mode == "INDEPENDENT_LOCAL_CAMERA_OBSERVATION":
            if target_frame not in SWEEPICK_MANIPULATION_FRAMES:
                return False, f"UNSUPPORTED_SWEEPICK_FRAME:{target_frame}"
            return True, None
        if handoff_mode == "DIRECT_BASE_TO_ARM_METRIC_TF":
            if not self.hardware_readiness.physical_cross_tree_tf_ready():
                return (
                    False,
                    "UNVERIFIED_BASE_TO_ARM_TF_FORBIDDEN: base_footprint <-> arm_base mounting "
                    "and extrinsic calibration are not verified; fabricating a transform is prohibited",
                )
            return True, None
        return False, f"UNKNOWN_HANDOFF_MODE:{handoff_mode}"


@dataclass(frozen=True)
class CameraObservation3D:
    """A 3D object observation from Navigation (`object_detector_3d.py`) or post-stop re-observation."""

    observation_id: str
    frame_seq: int
    captured_at_s: float
    clock_id: str
    frame_id: str
    position_in_base_footprint_m: Tuple[float, float, float]
    normal_in_base_footprint: Tuple[float, float, float] = (-1.0, 0.0, 0.0)
    inlier_count: int = 0
    rms_m: float = 0.0
    detector_mode: str = "hsv"
    is_ema_filtered_from_motion: bool = False
    is_persistence_latched: bool = False
    qwen_bbox_captured_at_s: Optional[float] = None
    invalidated_by_navigation: bool = False

    def distance_xy_m(self) -> float:
        x, y, _ = self.position_in_base_footprint_m
        return float(math.hypot(x, y))

    def bearing_deg(self) -> float:
        x, y, _ = self.position_in_base_footprint_m
        return float(math.degrees(math.atan2(y, x)))

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["distance_xy_m"] = self.distance_xy_m()
        data["bearing_deg"] = self.bearing_deg()
        return data


@dataclass(frozen=True)
class NavigationArrivalEvidence:
    """Nav2 / PrecisionApproacher arrival evidence. Never interpreted as manipulation success."""

    goal_label: str
    nav2_status: str  # e.g., "STATUS_SUCCEEDED", "DOCKED", "STATUS_ABORTED"
    arrived_at_s: float
    clock_id: str
    final_pose_map: Tuple[float, float, float]  # (x, y, yaw_rad)
    last_nav_frame_seq: int
    manipulation_succeeded: bool = False
    final_pose_frame: str = "map"  # C-2: "map" only when map<-base_footprint TF was looked up; else "odom"
    evidence_source: str = ""      # C-2: e.g. "nav2:/navigate_to_pose/_action/status" or "MOCK"

    def __post_init__(self) -> None:
        if self.manipulation_succeeded is not False:
            raise ValueError("Nav2 arrival evidence must never set manipulation_succeeded=True")

    def is_arrived(self) -> bool:
        return self.nav2_status in ("STATUS_SUCCEEDED", "DOCKED")


@dataclass(frozen=True)
class BaseStopEvidence:
    """Evidence that the mobile base has fully stopped after Nav2 arrival."""

    arrived_at_s: float
    stopped_at_s: float
    clock_id: str
    linear_vel_mps: float
    angular_vel_radps: float
    cmd_vel_zero_published: bool
    nav2_controller_idle: bool
    settled_duration_s: float
    last_nav_frame_seq: int
    pre_arrival_coordinates_invalidated: bool = True
    evidence_source: str = ""  # C-2: observed topics that produced this evidence, or "MOCK"


@dataclass(frozen=True)
class ReobserveVerdict:
    """Outcome of the P0 REOBSERVE gate after BASE_STOPPED."""

    passed: bool
    reason: str
    observation: Optional[CameraObservation3D]
    checked_at_s: float
    clock_id: str
    rejected_reasons: Tuple[str, ...] = ()
    distance_xy_m: Optional[float] = None
    bearing_deg: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TaskManagerErrorReport:
    """Structured error propagated to the Navigation Task Manager upon failure/STOP/rejection."""

    failed_stage: IntegrationStage
    result_status: ManipulationResultStatus
    error_code: str
    detail: str
    subsequent_commands_blocked: bool
    requires_operator_recovery: bool
    run_id: Optional[str] = None
    native_gate: Optional[str] = None
    timestamp_s: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["failed_stage"] = self.failed_stage.value
        data["result_status"] = self.result_status.value
        return data


@dataclass(frozen=True)
class IntegrationOutcomeRecord:
    """Final P0/P1/P2 result emitted at IntegrationStage.RESULT."""

    current_stage: IntegrationStage
    stage_history: Tuple[str, ...]
    result_status: ManipulationResultStatus
    manipulation_succeeded: bool
    left_pick_lift_confirmed: bool
    right_handover_confirmed: bool
    approved_scope: str
    motor_authority_created_by_integration: bool
    pre_nav_coordinate_reused: bool
    post_stop_reobserved: bool
    subsequent_commands_blocked: bool
    hardware_readiness: Dict[str, Any]
    reason: str
    error_report: Optional[Dict[str, Any]] = None
    sweepick_run_id: Optional[str] = None
    sweepick_session_result: Optional[str] = None
    reobserve_verdict: Optional[Dict[str, Any]] = None

    def __post_init__(self) -> None:
        if self.motor_authority_created_by_integration is not False:
            raise ValueError("Integration layer must never create motor/torque authority")
        if self.pre_nav_coordinate_reused is not False:
            raise ValueError("Pre-navigation coordinates must never be reused after motion")
        if self.result_status == ManipulationResultStatus.COMPLETED:
            if not (self.left_pick_lift_confirmed or self.right_handover_confirmed):
                raise ValueError(
                    "COMPLETED status is forbidden unless actual left PICK/LIFT or right handover is confirmed"
                )
            if not self.post_stop_reobserved:
                raise ValueError(
                    "COMPLETED status is forbidden without a verified post-stop REOBSERVE"
                )

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["current_stage"] = self.current_stage.value
        data["result_status"] = self.result_status.value
        return data
