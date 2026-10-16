"""P0 BASE_STOPPED verifier and PostStopReobserveGate.

Prevents pre-navigation or in-motion object observations, EMA state, 0.5s
persistence latches, or 180s latched Qwen BBoxes from leaking into
MANIPULATION_READY after Nav2 arrival.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Set, Tuple

from .contracts import (
    BaseStopEvidence,
    CameraObservation3D,
    CoordinateFrameContract,
    ReobserveVerdict,
)


@dataclass(frozen=True)
class ReobserveGateConfig:
    """Thresholds for verifying BASE_STOPPED and fresh post-stop 3D re-observation."""

    max_linear_vel_mps: float = 0.005
    max_angular_vel_radps: float = 0.01
    min_settle_duration_s: float = 0.5
    min_post_stop_margin_s: float = 0.02
    max_observation_age_s: float = 1.5
    min_target_dist_m: float = 0.12
    max_target_dist_m: float = 0.85
    max_bearing_deg: float = 35.0
    min_plane_inliers: int = 25
    max_plane_rms_m: float = 0.035


class BaseStopVerifier:
    """Verifies that the robot base has settled at zero velocity after Nav2 arrival."""

    def __init__(self, config: Optional[ReobserveGateConfig] = None) -> None:
        self.config = config or ReobserveGateConfig()

    def verify(self, evidence: BaseStopEvidence) -> Tuple[bool, List[str]]:
        problems: List[str] = []
        if not evidence.clock_id or "UNKNOWN" in evidence.clock_id.upper():
            problems.append("BASE_STOP_CLOCK_UNQUALIFIED")
        if not (math.isfinite(evidence.arrived_at_s) and math.isfinite(evidence.stopped_at_s)):
            problems.append("BASE_STOP_TIMESTAMP_NONFINITE")
        elif evidence.stopped_at_s < evidence.arrived_at_s:
            problems.append("BASE_STOPPED_BEFORE_ARRIVED_TIMESTAMP")
        if not math.isfinite(evidence.linear_vel_mps) or abs(evidence.linear_vel_mps) > self.config.max_linear_vel_mps:
            problems.append(
                f"LINEAR_VELOCITY_NONZERO:{evidence.linear_vel_mps:.4f}>{self.config.max_linear_vel_mps:.4f}"
            )
        if not math.isfinite(evidence.angular_vel_radps) or abs(evidence.angular_vel_radps) > self.config.max_angular_vel_radps:
            problems.append(
                f"ANGULAR_VELOCITY_NONZERO:{evidence.angular_vel_radps:.4f}>{self.config.max_angular_vel_radps:.4f}"
            )
        if not evidence.cmd_vel_zero_published:
            problems.append("CMD_VEL_ZERO_NOT_PUBLISHED")
        if not evidence.nav2_controller_idle:
            problems.append("NAV2_CONTROLLER_STILL_ACTIVE")
        if not math.isfinite(evidence.settled_duration_s) or evidence.settled_duration_s < self.config.min_settle_duration_s:
            problems.append(
                f"BASE_SETTLE_DURATION_TOO_SHORT:{evidence.settled_duration_s:.3f}<{self.config.min_settle_duration_s:.3f}"
            )
        if not evidence.pre_arrival_coordinates_invalidated:
            problems.append("PRE_ARRIVAL_COORDINATES_NOT_INVALIDATED")
        return len(problems) == 0, problems


class PostStopReobserveGate:
    """Enforces that MANIPULATION_READY is only entered with a fresh post-stop camera observation."""

    def __init__(
        self,
        config: Optional[ReobserveGateConfig] = None,
        frame_contract: Optional[CoordinateFrameContract] = None,
    ) -> None:
        self.config = config or ReobserveGateConfig()
        self.frame_contract = frame_contract or CoordinateFrameContract()
        self._invalidated_observation_ids: Set[str] = set()

    def invalidate_observation(self, observation: Optional[CameraObservation3D]) -> None:
        """Mark a pre-navigation or in-motion observation as permanently unusable after motion."""
        if observation is not None and observation.observation_id:
            self._invalidated_observation_ids.add(observation.observation_id)

    def evaluate(
        self,
        *,
        base_stop: BaseStopEvidence,
        observation: Optional[CameraObservation3D],
        checked_at_s: float,
        pre_nav_observation: Optional[CameraObservation3D] = None,
    ) -> ReobserveVerdict:
        """Validate that `observation` was captured strictly after `base_stop` and meets 3D gates."""
        self.invalidate_observation(pre_nav_observation)
        rejected: List[str] = []

        if observation is None:
            return ReobserveVerdict(
                passed=False,
                reason="NO_POST_STOP_OBSERVATION_AVAILABLE",
                observation=None,
                checked_at_s=checked_at_s,
                clock_id=base_stop.clock_id,
                rejected_reasons=("NO_POST_STOP_OBSERVATION_AVAILABLE",),
            )

        if (
            observation.invalidated_by_navigation
            or observation.observation_id in self._invalidated_observation_ids
            or (
                pre_nav_observation is not None
                and observation.observation_id == pre_nav_observation.observation_id
            )
        ):
            rejected.append("PRE_NAVIGATION_OBSERVATION_REUSE_FORBIDDEN")

        if observation.clock_id != base_stop.clock_id:
            rejected.append(
                f"CLOCK_DOMAIN_MISMATCH:{observation.clock_id}!={base_stop.clock_id}"
            )

        min_required_time = base_stop.stopped_at_s + self.config.min_post_stop_margin_s
        if not math.isfinite(observation.captured_at_s) or observation.captured_at_s < min_required_time:
            rejected.append(
                f"STALE_PRE_STOP_OBSERVATION:captured_at={observation.captured_at_s:.3f}"
                f"<required_after_stop={min_required_time:.3f}"
            )

        if math.isfinite(checked_at_s) and math.isfinite(observation.captured_at_s):
            age_s = checked_at_s - observation.captured_at_s
            if age_s < 0:
                rejected.append("OBSERVATION_TIMESTAMP_IN_FUTURE")
            elif age_s > self.config.max_observation_age_s:
                rejected.append(
                    f"OBSERVATION_TOO_OLD:age={age_s:.3f}>{self.config.max_observation_age_s:.3f}"
                )

        if observation.frame_seq <= base_stop.last_nav_frame_seq:
            rejected.append(
                f"STALE_CAMERA_FRAME_SEQ:{observation.frame_seq}<=last_nav_seq={base_stop.last_nav_frame_seq}"
            )

        if observation.is_ema_filtered_from_motion:
            rejected.append("EMA_FILTER_CARRYOVER_FROM_MOTION_FORBIDDEN")

        if observation.is_persistence_latched:
            rejected.append("PERSISTENCE_LATCHED_OBSERVATION_FORBIDDEN")

        if observation.detector_mode == "qwen_vlm":
            if observation.qwen_bbox_captured_at_s is None:
                rejected.append("QWEN_BBOX_TIMESTAMP_MISSING")
            elif observation.qwen_bbox_captured_at_s <= base_stop.stopped_at_s:
                rejected.append(
                    f"STALE_LATCHED_QWEN_BBOX:bbox_at={observation.qwen_bbox_captured_at_s:.3f}"
                    f"<=stopped_at={base_stop.stopped_at_s:.3f}"
                )

        frame_ok, frame_err = self.frame_contract.validate_nav_frame(observation.frame_id)
        if not frame_ok and frame_err:
            rejected.append(frame_err)

        coords = observation.position_in_base_footprint_m
        if len(coords) != 3 or not all(math.isfinite(c) for c in coords):
            rejected.append("NONFINITE_3D_POSITION")
            dist_xy, bearing = None, None
        else:
            dist_xy = observation.distance_xy_m()
            bearing = observation.bearing_deg()
            if dist_xy < self.config.min_target_dist_m or dist_xy > self.config.max_target_dist_m:
                rejected.append(
                    f"TARGET_DISTANCE_OUT_OF_RANGE:{dist_xy:.3f}m not in "
                    f"[{self.config.min_target_dist_m:.3f}, {self.config.max_target_dist_m:.3f}]"
                )
            if abs(bearing) > self.config.max_bearing_deg:
                rejected.append(
                    f"TARGET_BEARING_OUT_OF_FOV:{bearing:+.1f}deg exceeds +-{self.config.max_bearing_deg:.1f}deg"
                )

        if observation.inlier_count < self.config.min_plane_inliers:
            rejected.append(
                f"INSUFFICIENT_PLANE_INLIERS:{observation.inlier_count}<{self.config.min_plane_inliers}"
            )

        if not math.isfinite(observation.rms_m) or observation.rms_m > self.config.max_plane_rms_m:
            rejected.append(
                f"PLANE_FIT_RMS_TOO_HIGH:{observation.rms_m:.4f}>{self.config.max_plane_rms_m:.4f}"
            )

        passed = len(rejected) == 0
        reason = (
            "FRESH_POST_STOP_OBSERVATION_CONFIRMED"
            if passed
            else "; ".join(rejected)
        )
        return ReobserveVerdict(
            passed=passed,
            reason=reason,
            observation=observation if passed else None,
            checked_at_s=checked_at_s,
            clock_id=base_stop.clock_id,
            rejected_reasons=tuple(rejected),
            distance_xy_m=dist_xy,
            bearing_deg=bearing,
        )
