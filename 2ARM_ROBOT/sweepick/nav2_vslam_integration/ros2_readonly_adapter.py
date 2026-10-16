"""C-2 P0-4: ROS 2 Jazzy read-only adapter for the VSLAM/Nav2 x Sweepick integration.

Two layers:
- `ReadOnlyGraphState` is pure Python (no rclpy). It ingests plain-dict copies of the
  messages the existing Navigation stack already publishes and derives
  `NavigationArrivalEvidence`, `BaseStopEvidence`, an arm-task gate and a post-stop
  observation candidate. Unit tests use this layer.
- `SweepickNavReadOnlyNode` (only when `rclpy` is importable) subscribes to the existing
  topics with the publishers' own QoS, keeps a tf2 buffer and feeds the state. It creates
  NO publishers, NO action clients and NO service clients: it cannot send a Nav2 goal,
  publish `/cmd_vel`, or write to a motor.

Existing topics consumed (names, types and QoS as found in
`Dapier_project_visaul_slam/robot_ws/src` at commit 3416c7c, ROS 2 Jazzy):
  /navigate_to_pose/_action/status   action_msgs/msg/GoalStatusArray        (rcl action status QoS: reliable, transient_local, depth 1)
  /navigate_to_pose/_action/feedback nav2_msgs/action/NavigateToPose_FeedbackMessage
  /odom                              nav_msgs/msg/Odometry                  (turtlebot3_node; task_manager subscribes depth 10)
  /cmd_vel                           geometry_msgs/msg/TwistStamped         (enable_stamped_cmd_vel: true everywhere)
  /cmd_vel_unstamped, /cmd_vel_nav   geometry_msgs/msg/Twist                (raw halt copies from task_manager / docking controller)
  /docking/status                    std_msgs/msg/String  "STATE=<...>, ..." (precision_approacher_node / precision_docking_controller_node)
  /task/status                       std_msgs/msg/String  "STATE=<TaskState>, task=..., target=..."
  /docking/target_object_pose        geometry_msgs/msg/PoseStamped          (object_detector_3d: camera_color_optical_frame, stamp=publish time, EMA+latch;
                                                                             precision_docking_controller_node: base_footprint)
  /docking/object_detection_status   std_msgs/msg/String  "... inliers=N, fps=F"
  /camera/color/camera_info          sensor_msgs/msg/CameraInfo             (sensor frame cadence; BEST_EFFORT)
  /docking/target_object_provenance  std_msgs/msg/String (JSON)             (C-2 PROPOSED sidecar; not published by the current detector)
  /tf, /tf_static                    via tf2_ros.Buffer / TransformListener
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, field
import json
import math
import re
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from .contracts import BaseStopEvidence, NavigationArrivalEvidence
from .reobserve_gate import ReobserveGateConfig
from .tf_provenance import (
    PROVENANCE_SOURCE_POSE_HEADER_ONLY,
    PROVENANCE_SOURCE_SIDECAR,
    DetectionProvenance,
    SensorFrameLedger,
    VerifiedTransform,
    build_post_stop_observation,
)


# action_msgs/msg/GoalStatus constants (Jazzy)
GOAL_STATUS_UNKNOWN = 0
GOAL_STATUS_ACCEPTED = 1
GOAL_STATUS_EXECUTING = 2
GOAL_STATUS_CANCELING = 3
GOAL_STATUS_SUCCEEDED = 4
GOAL_STATUS_CANCELED = 5
GOAL_STATUS_ABORTED = 6
GOAL_STATUS_NAMES = {
    0: "STATUS_UNKNOWN", 1: "STATUS_ACCEPTED", 2: "STATUS_EXECUTING", 3: "STATUS_CANCELING",
    4: "STATUS_SUCCEEDED", 5: "STATUS_CANCELED", 6: "STATUS_ABORTED",
}
NAV2_ACTIVE_STATUSES = frozenset({GOAL_STATUS_ACCEPTED, GOAL_STATUS_EXECUTING, GOAL_STATUS_CANCELING})

# precision_approacher_node / precision_docking_controller_node states that still command the base
DOCKING_ACTIVE_STATES = frozenset({
    "INIT", "SEARCH_MARKER", "ALIGN", "APPROACH", "STANDOFF_SETTLE", "FINAL_APPROACH", "CRAWL_CONTACT",
    "BACKUP_STANDOFF", "BACKUP_RETRY", "ABORT_TO_NAV2", "ACQUIRING", "ALIGNING", "APPROACHING", "DOCKING",
})
DOCKING_ARRIVED_STATES = frozenset({"DOCKED"})
DOCKING_IDLE_STATES = frozenset({"NAV2_READY", "IDLE", "FAILSAFE"})

TOPIC_TARGET_POSE = "/docking/target_object_pose"
TOPIC_PROVENANCE_SIDECAR = "/docking/target_object_provenance"

_STATE_RE = re.compile(r"STATE=([A-Z0-9_]+)")
_INLIERS_RE = re.compile(r"inliers=(\d+)")


def parse_state_field(text: str) -> Optional[str]:
    m = _STATE_RE.search(text or "")
    return m.group(1) if m else None


@dataclass(frozen=True)
class OdomSample:
    stamp_s: float
    frame_id: str
    child_frame_id: str
    linear_x: float
    linear_y: float
    angular_z: float
    pose_xy_yaw: Tuple[float, float, float]


@dataclass(frozen=True)
class CmdVelSample:
    topic: str
    stamp_s: float   # header stamp for TwistStamped, receive time for Twist
    receive_s: float
    linear_x: float
    angular_z: float
    stamped: bool
    publisher_node: Optional[str] = None

    @property
    def is_zero(self) -> bool:
        return abs(self.linear_x) < 1e-6 and abs(self.angular_z) < 1e-6


@dataclass(frozen=True)
class TargetPoseSample:
    receive_s: float
    receive_index: int
    header_stamp_s: float
    frame_id: str
    position: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float]
    publisher_node: Optional[str]


@dataclass
class ArmTaskGateDecision:
    allowed: bool
    reasons: List[str] = field(default_factory=list)
    checked_at_s: float = float("nan")

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class ReadOnlyGraphState:
    """Pure-Python consumer of the existing ROS 2 graph (no publishing, no rclpy dependency)."""

    def __init__(
        self,
        *,
        clock_id: str,
        gate_config: Optional[ReobserveGateConfig] = None,
        transform_lookup: Optional[Callable[[str, str, float], Optional[VerifiedTransform]]] = None,
        odom_history: int = 400,
    ) -> None:
        self.clock_id = clock_id
        self.config = gate_config or ReobserveGateConfig()
        self.transform_lookup = transform_lookup
        self.odom: Deque[OdomSample] = deque(maxlen=odom_history)
        self.cmd_vel: Deque[CmdVelSample] = deque(maxlen=200)
        self.nav2_goals: Dict[str, Tuple[int, float]] = {}       # goal_id -> (status, stamp)
        self.nav2_last_terminal: Optional[Tuple[str, int, float]] = None
        self.nav2_feedback_count = 0
        self.docking_state: Optional[str] = None
        self.docking_state_since_s: Optional[float] = None
        self.docking_docked_at_s: Optional[float] = None
        self.docking_last_text: Optional[str] = None
        self.task_state: Optional[str] = None
        self.task_last_text: Optional[str] = None
        self.sensor_frames: Deque[Tuple[float, float]] = deque(maxlen=400)  # (header_stamp, receive)
        self.sensor_frame_count = 0
        self.sensor_frame_index_by_stamp_ns: Dict[int, int] = {}
        self.target_poses: Deque[TargetPoseSample] = deque(maxlen=200)
        self.target_pose_count = 0
        self.detection_status_inliers: Optional[int] = None
        self.detection_status_text: Optional[str] = None
        self.provenance_sidecars: Deque[Dict[str, Any]] = deque(maxlen=200)
        self.ledger = SensorFrameLedger()
        self.console_status: Optional[Dict[str, Any]] = None
        self.publisher_nodes_seen: Dict[str, set] = {}
        self.counts: Dict[str, int] = {}
        self.first_receive_s: Optional[float] = None
        self.last_receive_s: Optional[float] = None

    # ------------------------------------------------------------------ ingest
    def _count(self, topic: str, receive_s: float, publisher_node: Optional[str] = None) -> None:
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if publisher_node:
            self.publisher_nodes_seen.setdefault(topic, set()).add(publisher_node)
        self.first_receive_s = receive_s if self.first_receive_s is None else min(self.first_receive_s, receive_s)
        self.last_receive_s = receive_s if self.last_receive_s is None else max(self.last_receive_s, receive_s)

    def ingest_odom(self, *, stamp_s: float, receive_s: float, frame_id: str, child_frame_id: str,
                    linear_x: float, linear_y: float, angular_z: float,
                    pose_xy_yaw: Tuple[float, float, float], publisher_node: Optional[str] = None) -> None:
        self._count("/odom", receive_s, publisher_node)
        self.odom.append(OdomSample(stamp_s, frame_id, child_frame_id, linear_x, linear_y, angular_z, pose_xy_yaw))

    def ingest_cmd_vel(self, *, topic: str, stamp_s: float, receive_s: float, linear_x: float, angular_z: float,
                       stamped: bool, publisher_node: Optional[str] = None) -> None:
        self._count(topic, receive_s, publisher_node)
        self.cmd_vel.append(CmdVelSample(topic, stamp_s, receive_s, linear_x, angular_z, stamped, publisher_node))

    def ingest_nav2_status(self, *, receive_s: float, statuses: List[Tuple[str, int, float]],
                           publisher_node: Optional[str] = None) -> None:
        self._count("/navigate_to_pose/_action/status", receive_s, publisher_node)
        for goal_id, status, stamp_s in statuses:
            self.nav2_goals[goal_id] = (int(status), float(stamp_s))
            if status in (GOAL_STATUS_SUCCEEDED, GOAL_STATUS_ABORTED, GOAL_STATUS_CANCELED):
                if self.nav2_last_terminal is None or stamp_s >= self.nav2_last_terminal[2]:
                    self.nav2_last_terminal = (goal_id, int(status), float(stamp_s))

    def ingest_nav2_feedback(self, *, receive_s: float, publisher_node: Optional[str] = None) -> None:
        self._count("/navigate_to_pose/_action/feedback", receive_s, publisher_node)
        self.nav2_feedback_count += 1

    def ingest_docking_status(self, *, receive_s: float, text: str, publisher_node: Optional[str] = None) -> None:
        self._count("/docking/status", receive_s, publisher_node)
        state = parse_state_field(text)
        self.docking_last_text = text
        if state and state != self.docking_state:
            self.docking_state, self.docking_state_since_s = state, receive_s
            if state in DOCKING_ARRIVED_STATES:
                self.docking_docked_at_s = receive_s

    def ingest_task_status(self, *, receive_s: float, text: str, publisher_node: Optional[str] = None) -> None:
        self._count("/task/status", receive_s, publisher_node)
        self.task_last_text = text
        self.task_state = parse_state_field(text) or self.task_state

    def ingest_sensor_frame(self, *, header_stamp_s: float, receive_s: float, publisher_node: Optional[str] = None) -> None:
        self._count("/camera/color/camera_info", receive_s, publisher_node)
        self.sensor_frame_count += 1
        self.sensor_frames.append((header_stamp_s, receive_s))
        self.sensor_frame_index_by_stamp_ns[int(round(header_stamp_s * 1e9))] = self.sensor_frame_count

    def sensor_frame_index_at(self, stamp_s: float) -> int:
        """Index (1-based receive count) of the sensor frame with this header stamp; or the count of
        frames stamped at/before it when no exact frame is known. 0 when no frame is known at all."""
        exact = self.sensor_frame_index_by_stamp_ns.get(int(round(stamp_s * 1e9)))
        if exact is not None:
            return exact
        before = [idx for ns, idx in self.sensor_frame_index_by_stamp_ns.items() if ns * 1e-9 <= stamp_s]
        return max(before) if before else 0

    def ingest_target_pose(self, *, receive_s: float, header_stamp_s: float, frame_id: str,
                           position: Tuple[float, float, float],
                           orientation_xyzw: Tuple[float, float, float, float],
                           publisher_node: Optional[str] = None) -> TargetPoseSample:
        self._count(TOPIC_TARGET_POSE, receive_s, publisher_node)
        self.target_pose_count += 1
        sample = TargetPoseSample(receive_s, self.target_pose_count, header_stamp_s, frame_id, position,
                                  orientation_xyzw, publisher_node)
        self.target_poses.append(sample)
        return sample

    def ingest_detection_status(self, *, receive_s: float, text: str, publisher_node: Optional[str] = None) -> None:
        self._count("/docking/object_detection_status", receive_s, publisher_node)
        self.detection_status_text = text
        m = _INLIERS_RE.search(text or "")
        self.detection_status_inliers = int(m.group(1)) if m else None

    def ingest_provenance_sidecar(self, *, receive_s: float, payload: Dict[str, Any],
                                  publisher_node: Optional[str] = None) -> None:
        self._count(TOPIC_PROVENANCE_SIDECAR, receive_s, publisher_node)
        self.provenance_sidecars.append(dict(payload, _receive_s=receive_s, _publisher=publisher_node))

    def ingest_console_status(self, status: Dict[str, Any]) -> None:
        self.console_status = status

    # ------------------------------------------------------------------ derived evidence
    def nav2_active_goal_ids(self) -> List[str]:
        return [gid for gid, (status, _) in self.nav2_goals.items() if status in NAV2_ACTIVE_STATUSES]

    def nav2_controller_idle(self) -> bool:
        return not self.nav2_active_goal_ids()

    def docking_commands_base(self) -> bool:
        return self.docking_state in DOCKING_ACTIVE_STATES

    def last_sensor_frame_index(self) -> int:
        return self.sensor_frame_count

    def base_motion_window(self, now_s: float, window_s: float) -> Dict[str, Any]:
        samples = [o for o in self.odom if now_s - o.stamp_s <= window_s]
        if not samples:
            return {"samples": 0}
        max_lin = max(math.hypot(o.linear_x, o.linear_y) for o in samples)
        max_ang = max(abs(o.angular_z) for o in samples)
        return {"samples": len(samples), "max_linear_mps": max_lin, "max_angular_radps": max_ang,
                "first_stamp_s": samples[0].stamp_s, "last_stamp_s": samples[-1].stamp_s}

    def settled_since(self, now_s: float) -> Optional[float]:
        """Stamp of the first odom sample after the last sample that exceeded the stop thresholds."""
        last_moving: Optional[float] = None
        first_quiet_after: Optional[float] = None
        for o in self.odom:
            moving = math.hypot(o.linear_x, o.linear_y) > self.config.max_linear_vel_mps or abs(o.angular_z) > self.config.max_angular_vel_radps
            if moving:
                last_moving, first_quiet_after = o.stamp_s, None
            elif first_quiet_after is None:
                first_quiet_after = o.stamp_s
        if not self.odom:
            return None
        return first_quiet_after

    def last_cmd_vel(self) -> Optional[CmdVelSample]:
        return self.cmd_vel[-1] if self.cmd_vel else None

    def arrival_evidence(self, *, goal_label: str, map_pose_lookup: Optional[Callable[[float], Optional[Tuple[float, float, float]]]] = None) -> Tuple[Optional[NavigationArrivalEvidence], List[str]]:
        reasons: List[str] = []
        if self.nav2_last_terminal is None and self.docking_docked_at_s is None:
            reasons.append("NO_NAV2_TERMINAL_STATUS_OBSERVED")
            return None, reasons
        if self.nav2_last_terminal is not None:
            goal_id, status, stamp_s = self.nav2_last_terminal
            status_name = GOAL_STATUS_NAMES.get(status, str(status))
            source = f"nav2:/navigate_to_pose/_action/status:{goal_id}"
        else:
            status_name, stamp_s, source = "DOCKED", float(self.docking_docked_at_s), "docking:/docking/status"
        if self.docking_docked_at_s is not None and self.docking_docked_at_s >= stamp_s:
            status_name, stamp_s, source = "DOCKED", float(self.docking_docked_at_s), source + "+docking:/docking/status"
        pose_frame = "map"
        pose = map_pose_lookup(stamp_s) if map_pose_lookup else None
        if pose is None:
            pose_frame = "odom"
            if self.odom:
                pose = self.odom[-1].pose_xy_yaw
            else:
                reasons.append("NO_POSE_AVAILABLE_FOR_ARRIVAL")
                return None, reasons
        evidence = NavigationArrivalEvidence(
            goal_label=goal_label,
            nav2_status=status_name,
            arrived_at_s=stamp_s,
            clock_id=self.clock_id,
            final_pose_map=pose,
            last_nav_frame_seq=self.sensor_frame_index_at(stamp_s),
            final_pose_frame=pose_frame,
            evidence_source=source,
        )
        return evidence, reasons

    def base_stop_evidence(self, *, now_s: float, arrived_at_s: float) -> Tuple[Optional[BaseStopEvidence], List[str]]:
        """Build BASE_STOPPED evidence from OBSERVED state only. Nothing is published to stop the base."""
        reasons: List[str] = []
        if not self.odom:
            reasons.append("NO_ODOM_OBSERVED")
            return None, reasons
        last = self.odom[-1]
        if now_s - last.stamp_s > 1.0:
            reasons.append(f"ODOM_STALE:{now_s - last.stamp_s:.2f}s")
        quiet_since = self.settled_since(now_s)
        if quiet_since is None:
            reasons.append("BASE_STILL_MOVING")
            settled = 0.0
            stopped_at = float("nan")
        else:
            # The base is usually already at rest when Nav2 reports SUCCEEDED or the docking
            # controller reports DOCKED; the stop is CONFIRMED from the arrival instant onward,
            # so stopped_at = max(first quiet odom sample, arrival) and the settle time counts
            # only the quiet time after that instant.
            stopped_at = max(quiet_since, arrived_at_s)
            settled = max(0.0, last.stamp_s - stopped_at)
        cmd = self.last_cmd_vel()
        cmd_zero_observed = bool(cmd is not None and cmd.is_zero)
        cmd_quiet = cmd is None or (now_s - cmd.receive_s) >= self.config.min_settle_duration_s
        if cmd is not None and not cmd.is_zero and not cmd_quiet:
            reasons.append(f"CMD_VEL_NONZERO_OBSERVED:{cmd.topic}:{cmd.linear_x:.3f},{cmd.angular_z:.3f}")
        nav_idle = self.nav2_controller_idle() and not self.docking_commands_base()
        if not nav_idle:
            reasons.append("NAV2_OR_DOCKING_STILL_COMMANDING_BASE")
        evidence = BaseStopEvidence(
            arrived_at_s=arrived_at_s,
            stopped_at_s=stopped_at if math.isfinite(stopped_at) else max(arrived_at_s, last.stamp_s),
            clock_id=self.clock_id,
            linear_vel_mps=math.hypot(last.linear_x, last.linear_y),
            angular_vel_radps=last.angular_z,
            cmd_vel_zero_published=cmd_zero_observed or (cmd is None and nav_idle),
            nav2_controller_idle=nav_idle,
            settled_duration_s=settled,
            last_nav_frame_seq=self.sensor_frame_index_at(stopped_at) if math.isfinite(stopped_at) else self.last_sensor_frame_index(),
            pre_arrival_coordinates_invalidated=True,
            evidence_source=(
                "observed:/odom,/cmd_vel,/navigate_to_pose/_action/status,/docking/status"
                + (f";first_quiet_odom_s={quiet_since:.3f}" if quiet_since is not None else "")
            ),
        )
        return evidence, reasons

    def arm_task_gate(self, *, now_s: float) -> ArmTaskGateDecision:
        """Whether this layer may even ASK Sweepick for an arm task (it never starts one)."""
        decision = ArmTaskGateDecision(allowed=True, checked_at_s=now_s)
        quiet_since = self.settled_since(now_s)
        if not self.odom:
            decision.reasons.append("NO_ODOM_OBSERVED")
        elif quiet_since is None or (self.odom[-1].stamp_s - quiet_since) < self.config.min_settle_duration_s:
            decision.reasons.append("BASE_STILL_MOVING_ARM_REQUEST_FORBIDDEN")
        if not self.nav2_controller_idle():
            decision.reasons.append("NAV2_GOAL_ACTIVE_ARM_REQUEST_FORBIDDEN")
        if self.docking_commands_base():
            decision.reasons.append(f"DOCKING_ACTIVE_ARM_REQUEST_FORBIDDEN:{self.docking_state}")
        cmd = self.last_cmd_vel()
        if cmd is not None and not cmd.is_zero and (now_s - cmd.receive_s) < self.config.min_settle_duration_s:
            decision.reasons.append("CMD_VEL_NONZERO_ARM_REQUEST_FORBIDDEN")
        cs = self.console_status
        if cs is not None:
            state = cs.get("state")
            active = cs.get("active") if isinstance(cs.get("active"), dict) else {}
            if state in ("FAULT", "LIMITED_HOLDING") and not isinstance(active.get("operator_recovery"), dict):
                decision.reasons.append(f"PRIOR_SWEEPICK_{state}_UNRECOVERED_NEW_TASK_FORBIDDEN:{active.get('id')}")
            if state == "BLOCKED":
                decision.reasons.append("SWEEPICK_FOREIGN_OWNER_ACTIVE")
            if cs.get("owner_alive") is True:
                decision.reasons.append("SWEEPICK_OWNER_ALIVE_NEW_TASK_FORBIDDEN")
        decision.allowed = not decision.reasons
        return decision

    def latest_post_stop_candidate(self, *, base_stop: BaseStopEvidence) -> Dict[str, Any]:
        """Evaluate the newest `/docking/target_object_pose` as a post-stop observation.

        Provenance comes only from the sidecar topic; the PoseStamped alone yields
        `POSE_HEADER_ONLY` provenance, which is refused (`POST_STOP_OBSERVATION_UNVERIFIED`).
        """
        if not self.target_poses:
            return {"observation": None, "rejected": ["NO_POST_STOP_OBSERVATION_AVAILABLE"], "provenance": None}
        sample = self.target_poses[-1]
        sidecar = self._sidecar_for(sample)
        if sidecar is not None:
            sensor_stamp = _float_or_none(sidecar.get("sensor_stamp_s"))
            provenance = DetectionProvenance(
                source_topic=TOPIC_TARGET_POSE,
                message_stamp_s=sample.header_stamp_s,
                receive_index=self.sensor_frame_index_at(sensor_stamp) if sensor_stamp is not None else sample.receive_index,
                clock_id=self.clock_id,
                provenance_source=PROVENANCE_SOURCE_SIDECAR,
                publisher_node=sample.publisher_node,
                sensor_stamp_s=_float_or_none(sidecar.get("sensor_stamp_s")),
                anchor_source=sidecar.get("anchor_source"),
                is_ema_filtered=_bool_or_none(sidecar.get("is_ema_filtered")),
                is_persistence_latched=_bool_or_none(sidecar.get("is_persistence_latched")),
                inlier_count=_int_or_none(sidecar.get("inlier_count")),
                rms_m=_float_or_none(sidecar.get("rms_m")),
                detector_mode=sidecar.get("detector_mode"),
                qwen_bbox_stamp_s=_float_or_none(sidecar.get("qwen_bbox_stamp_s")),
                notes=("sidecar matched by pose_stamp_s",),
            )
        else:
            provenance = DetectionProvenance(
                source_topic=TOPIC_TARGET_POSE,
                message_stamp_s=sample.header_stamp_s,
                receive_index=sample.receive_index,
                clock_id=self.clock_id,
                provenance_source=PROVENANCE_SOURCE_POSE_HEADER_ONLY,
                publisher_node=sample.publisher_node,
                sensor_stamp_s=None,
                inlier_count=self.detection_status_inliers,
                notes=(
                    "object_detector_3d.py stamps the pose with publish time, applies ema_alpha=0.25 and a 0.5 s "
                    "persistence latch; EMA/latch state, sensor frame and RMS are not on any topic",
                ),
            )
        transform = None
        if self.transform_lookup is not None and provenance.sensor_stamp_s is not None:
            transform = self.transform_lookup("base_footprint", sample.frame_id, provenance.sensor_stamp_s)
        normal = _normal_from_orientation(sample.orientation_xyzw)
        observation, rejected = build_post_stop_observation(
            position_in_sensor_frame_m=sample.position,
            normal_in_sensor_frame=normal,
            sensor_frame_id=sample.frame_id,
            provenance=provenance,
            transform=transform,
            ledger=self.ledger,
            allow_mock=False,
        )
        return {
            "observation": observation.to_dict() if observation else None,
            "observation_obj": observation,
            "rejected": list(rejected),
            "provenance": provenance.to_dict(),
            "transform": transform.to_dict() if transform else None,
            "pose_sample": asdict(sample),
        }

    def _sidecar_for(self, sample: TargetPoseSample) -> Optional[Dict[str, Any]]:
        for row in reversed(self.provenance_sidecars):
            ps = _float_or_none(row.get("pose_stamp_s"))
            if ps is not None and abs(ps - sample.header_stamp_s) < 1e-6:
                return row
        return None

    def snapshot(self) -> Dict[str, Any]:
        return {
            "clock_id": self.clock_id,
            "message_counts": dict(self.counts),
            "publisher_nodes_seen": {k: sorted(v) for k, v in self.publisher_nodes_seen.items()},
            "nav2_goals": {k: {"status": GOAL_STATUS_NAMES.get(v[0], v[0]), "stamp_s": v[1]} for k, v in self.nav2_goals.items()},
            "nav2_last_terminal": self.nav2_last_terminal,
            "nav2_feedback_count": self.nav2_feedback_count,
            "docking_state": self.docking_state,
            "docking_last_text": self.docking_last_text,
            "task_state": self.task_state,
            "task_last_text": self.task_last_text,
            "last_odom": asdict(self.odom[-1]) if self.odom else None,
            "last_cmd_vel": asdict(self.cmd_vel[-1]) if self.cmd_vel else None,
            "sensor_frame_count": self.sensor_frame_count,
            "target_pose_count": self.target_pose_count,
            "detection_status_text": self.detection_status_text,
            "provenance_sidecar_count": len(self.provenance_sidecars),
            "first_receive_s": self.first_receive_s,
            "last_receive_s": self.last_receive_s,
        }


def _float_or_none(v: Any) -> Optional[float]:
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def _int_or_none(v: Any) -> Optional[int]:
    try:
        return None if v is None else int(v)
    except (TypeError, ValueError):
        return None


def _bool_or_none(v: Any) -> Optional[bool]:
    return v if isinstance(v, bool) else None


def _normal_from_orientation(q_xyzw: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    """object_detector_3d publishes yaw about the optical Y axis from normal=(nx,·,nz) via atan2(nx,-nz);
    recover the outward normal (towards the camera, -Z optical) rotated by that yaw."""
    x, y, z, w = q_xyzw
    yaw = 2.0 * math.atan2(y, w) if abs(x) < 1e-9 and abs(z) < 1e-9 else 0.0
    return (math.sin(yaw), 0.0, -math.cos(yaw))


# ============================================================================ rclpy layer
try:  # pragma: no cover - exercised by the dry-run script, not by unit tests
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data  # type: ignore
    _RCLPY_AVAILABLE = True
except Exception:  # noqa: BLE001
    _RCLPY_AVAILABLE = False


def rclpy_available() -> bool:
    return _RCLPY_AVAILABLE


if _RCLPY_AVAILABLE:  # pragma: no cover
    from action_msgs.msg import GoalStatusArray  # type: ignore
    from geometry_msgs.msg import PoseStamped, Twist, TwistStamped  # type: ignore
    from nav_msgs.msg import Odometry  # type: ignore
    from sensor_msgs.msg import CameraInfo  # type: ignore
    from std_msgs.msg import String  # type: ignore
    from nav2_msgs.action import NavigateToPose  # type: ignore
    import rclpy.time  # type: ignore
    from rclpy.duration import Duration  # type: ignore
    from tf2_ros import TransformException  # type: ignore
    from tf2_ros.buffer import Buffer  # type: ignore
    from tf2_ros.transform_listener import TransformListener  # type: ignore
    from .tf_provenance import TF_SOURCE_LOOKUP

    ACTION_STATUS_QOS = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def _stamp_to_s(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _yaw_from_q(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    class SweepickNavReadOnlyNode(Node):
        """Subscribes only. Creates no publisher, action client or service client."""

        def __init__(self, *, node_name: str = "sweepick_c2_readonly_adapter", gate_config: Optional[ReobserveGateConfig] = None,
                     subscribe_action_feedback: bool = True, discovery_wait_s: float = 3.0) -> None:
            super().__init__(node_name)
            self.user_publishers_created = 0
            self.user_clients_created = 0
            # Give this node's own graph cache time to discover the existing publishers, so every
            # subscription below can copy the publisher's QoS instead of guessing.
            import time as _time
            t_end = _time.monotonic() + max(0.0, discovery_wait_s)
            self.discovery_waited_s = 0.0
            while _time.monotonic() < t_end:
                if self.get_publishers_info_by_topic("/odom") or self.get_publishers_info_by_topic("/docking/status"):
                    break
                _time.sleep(0.1)
            self.discovery_waited_s = round(max(0.0, discovery_wait_s - (t_end - _time.monotonic())), 2)
            clock_kind = "ROS_TIME(use_sim_time=%s)" % bool(self.get_parameter("use_sim_time").value)
            self.state = ReadOnlyGraphState(
                clock_id=f"ros2:{clock_kind}", gate_config=gate_config, transform_lookup=self.lookup_transform_at,
            )
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
            self.tf_lookup_log: List[Dict[str, Any]] = []
            self.subscriptions_made: List[Dict[str, Any]] = []
            self._sub(GoalStatusArray, "/navigate_to_pose/_action/status", self._on_nav2_status, ACTION_STATUS_QOS)
            if subscribe_action_feedback:
                self._sub(NavigateToPose.Impl.FeedbackMessage, "/navigate_to_pose/_action/feedback", self._on_nav2_feedback, QoSProfile(depth=10))
            self._sub(Odometry, "/odom", self._on_odom, QoSProfile(depth=10))
            self._sub(TwistStamped, "/cmd_vel", self._on_cmd_vel_stamped, QoSProfile(depth=10))
            self._sub(Twist, "/cmd_vel_unstamped", lambda m: self._on_cmd_vel_raw("/cmd_vel_unstamped", m), QoSProfile(depth=10))
            self._sub(Twist, "/cmd_vel_nav", lambda m: self._on_cmd_vel_raw("/cmd_vel_nav", m), QoSProfile(depth=10))
            self._sub(String, "/docking/status", self._on_docking_status, QoSProfile(depth=10))
            self._sub(String, "/task/status", self._on_task_status, QoSProfile(depth=10))
            self._sub(PoseStamped, TOPIC_TARGET_POSE, self._on_target_pose, QoSProfile(depth=10))
            self._sub(String, "/docking/object_detection_status", self._on_detection_status, QoSProfile(depth=10))
            self._sub(String, TOPIC_PROVENANCE_SIDECAR, self._on_provenance_sidecar, QoSProfile(depth=10))
            self._sub(CameraInfo, "/camera/color/camera_info", self._on_camera_info, qos_profile_sensor_data)

        # QoS is copied from the first discovered publisher so the existing graph is reused as-is.
        def _sub(self, msg_type, topic: str, cb, fallback_qos: QoSProfile) -> None:
            infos = self.get_publishers_info_by_topic(topic)
            qos = fallback_qos
            matched_from = None
            if infos:
                pq = infos[0].qos_profile
                qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=max(1, fallback_qos.depth),
                                 reliability=pq.reliability, durability=pq.durability)
                matched_from = infos[0].node_name
            self.create_subscription(msg_type, topic, cb, qos)
            self.subscriptions_made.append({
                "topic": topic, "type": f"{msg_type.__module__}.{msg_type.__name__}",
                "qos": {"reliability": str(qos.reliability), "durability": str(qos.durability), "depth": qos.depth},
                "qos_copied_from_publisher": matched_from,
                "publishers_at_subscribe": [{"node": i.node_name, "type": i.topic_type} for i in infos],
            })

        def _publisher_of(self, topic: str) -> Optional[str]:
            infos = self.get_publishers_info_by_topic(topic)
            return infos[0].node_name if infos else None

        def now_s(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def lookup_transform_at(self, parent: str, child: str, stamp_s: float) -> Optional[VerifiedTransform]:
            """TF2 lookup at the exact message time. No fallback to latest, no constants."""
            t_req = rclpy.time.Time(nanoseconds=int(round(stamp_s * 1e9)))
            try:
                tf = self.tf_buffer.lookup_transform(parent, child, t_req, timeout=Duration(seconds=0.2))
            except TransformException as exc:  # Lookup/Connectivity/Extrapolation
                self.tf_lookup_log.append({"parent": parent, "child": child, "stamp_s": stamp_s, "ok": False,
                                           "error": f"{type(exc).__name__}: {exc}"})
                return None
            tr, rot = tf.transform.translation, tf.transform.rotation
            out = VerifiedTransform(
                parent_frame=tf.header.frame_id, child_frame=tf.child_frame_id,
                stamp_s=_stamp_to_s(tf.header.stamp) if (tf.header.stamp.sec or tf.header.stamp.nanosec) else stamp_s,
                clock_id=self.state.clock_id,
                translation_m=(tr.x, tr.y, tr.z), rotation_xyzw=(rot.x, rot.y, rot.z, rot.w),
                source=TF_SOURCE_LOOKUP, lookup_time_s=self.now_s(),
                tf_authority="tf2_ros.Buffer(/tf,/tf_static)",
            )
            self.tf_lookup_log.append({"parent": parent, "child": child, "stamp_s": stamp_s, "ok": True,
                                       "translation_m": out.translation_m, "rotation_xyzw": out.rotation_xyzw})
            return out

        def map_pose_at(self, stamp_s: float) -> Optional[Tuple[float, float, float]]:
            tf = self.lookup_transform_at("map", "base_footprint", stamp_s)
            if tf is None:
                return None
            x, y, z, w = tf.rotation_xyzw
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            return (tf.translation_m[0], tf.translation_m[1], yaw)

        # ---- callbacks (plain-dict copies into the pure state) ----
        def _on_nav2_status(self, msg) -> None:
            self.state.ingest_nav2_status(
                receive_s=self.now_s(),
                statuses=[(bytes(s.goal_info.goal_id.uuid).hex(), int(s.status), _stamp_to_s(s.goal_info.stamp)) for s in msg.status_list],
                publisher_node=self._publisher_of("/navigate_to_pose/_action/status"),
            )

        def _on_nav2_feedback(self, msg) -> None:
            self.state.ingest_nav2_feedback(receive_s=self.now_s(), publisher_node=self._publisher_of("/navigate_to_pose/_action/feedback"))

        def _on_odom(self, msg) -> None:
            p, q, tw = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
            self.state.ingest_odom(
                stamp_s=_stamp_to_s(msg.header.stamp), receive_s=self.now_s(), frame_id=msg.header.frame_id,
                child_frame_id=msg.child_frame_id, linear_x=tw.linear.x, linear_y=tw.linear.y, angular_z=tw.angular.z,
                pose_xy_yaw=(p.x, p.y, _yaw_from_q(q)), publisher_node=self._publisher_of("/odom"),
            )

        def _on_cmd_vel_stamped(self, msg) -> None:
            self.state.ingest_cmd_vel(topic="/cmd_vel", stamp_s=_stamp_to_s(msg.header.stamp), receive_s=self.now_s(),
                                      linear_x=msg.twist.linear.x, angular_z=msg.twist.angular.z, stamped=True,
                                      publisher_node=self._publisher_of("/cmd_vel"))

        def _on_cmd_vel_raw(self, topic: str, msg) -> None:
            now = self.now_s()
            self.state.ingest_cmd_vel(topic=topic, stamp_s=now, receive_s=now, linear_x=msg.linear.x,
                                      angular_z=msg.angular.z, stamped=False, publisher_node=self._publisher_of(topic))

        def _on_docking_status(self, msg) -> None:
            self.state.ingest_docking_status(receive_s=self.now_s(), text=msg.data, publisher_node=self._publisher_of("/docking/status"))

        def _on_task_status(self, msg) -> None:
            self.state.ingest_task_status(receive_s=self.now_s(), text=msg.data, publisher_node=self._publisher_of("/task/status"))

        def _on_target_pose(self, msg) -> None:
            p, q = msg.pose.position, msg.pose.orientation
            self.state.ingest_target_pose(
                receive_s=self.now_s(), header_stamp_s=_stamp_to_s(msg.header.stamp), frame_id=msg.header.frame_id,
                position=(p.x, p.y, p.z), orientation_xyzw=(q.x, q.y, q.z, q.w), publisher_node=self._publisher_of(TOPIC_TARGET_POSE),
            )

        def _on_detection_status(self, msg) -> None:
            self.state.ingest_detection_status(receive_s=self.now_s(), text=msg.data, publisher_node=self._publisher_of("/docking/object_detection_status"))

        def _on_provenance_sidecar(self, msg) -> None:
            try:
                payload = json.loads(msg.data)
            except ValueError:
                payload = {"parse_error": msg.data[:200]}
            self.state.ingest_provenance_sidecar(receive_s=self.now_s(), payload=payload, publisher_node=self._publisher_of(TOPIC_PROVENANCE_SIDECAR))

        def _on_camera_info(self, msg) -> None:
            self.state.ingest_sensor_frame(header_stamp_s=_stamp_to_s(msg.header.stamp), receive_s=self.now_s(),
                                           publisher_node=self._publisher_of("/camera/color/camera_info"))

        def own_interfaces(self) -> Dict[str, Any]:
            name, ns = self.get_name(), self.get_namespace()
            return {
                "publishers": [{"topic": t, "types": ty} for t, ty in self.get_publisher_names_and_types_by_node(name, ns)],
                "subscriptions": [{"topic": t, "types": ty} for t, ty in self.get_subscriber_names_and_types_by_node(name, ns)],
                "service_clients": [{"service": s, "types": ty} for s, ty in self.get_client_names_and_types_by_node(name, ns)],
                "user_publishers_created": self.user_publishers_created,
                "user_clients_created": self.user_clients_created,
            }
