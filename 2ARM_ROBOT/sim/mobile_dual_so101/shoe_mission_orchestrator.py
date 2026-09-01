#!/usr/bin/env python3
"""Headless A-to-B shoe mission integration runner.

This revision combines kinematic differential-drive navigation, ground-truth
SIM object localization, collision-checked IK planning, and a scripted FSR
grasp state.  It does not claim contact-physics grasp success or hardware
execution; those modes are explicit in every report.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Sequence

import mujoco

from collision_guard import check_bimanual_path
from mission_core import (
    MissionController,
    MissionEvent,
    MissionEventType,
    MissionLocation,
    MissionPhase,
)
from mission_modules.mobility import NavigationRequest
from mission_modules.types import Pose2D
from mobile_dual_so101 import (
    HUMANOID_HOME_ACTION,
    TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
    apply_control_as_pose,
    build_model as build_stationary_arm_model,
)
from mujoco_mission_adapters import (
    MuJoCoMobilityAdapter,
    build_mobile_shoe_mission_model,
)
from physics_ik import (
    MotionLimits,
    plan_septic_joint_trajectory,
    solve_bimanual_position_ik,
)
from shoe_task import ShoeTaskConfig


SCHEMA_VERSION = "dapier.shoe-mission-run.v0.1"
PERCEPTION_MODE = "SIM_GROUND_TRUTH"
MOBILITY_MODE = "SIM_KINEMATIC_DIFFERENTIAL_DRIVE"
MANIPULATION_MODE = "IK_PLANNED_SCRIPTED_GRASP"
TACTILE_MODE = "SIM_SCRIPTED_DUAL_FSR"


@dataclass(frozen=True)
class ShoeMissionScenario:
    start_pose_map: Pose2D = Pose2D(0.0, 0.0, 0.0)
    shoe_position_map_m: tuple[float, float, float] = (0.26, 0.0, 0.015)
    docking_shoe_position_base_m: tuple[float, float, float] = (
        -0.055,
        0.246,
        0.070,
    )
    docking_yaw_rad: float = 0.0
    navigation_position_tolerance_m: float = 0.015
    navigation_yaw_tolerance_rad: float = 0.025
    navigation_timeout_s: float = 10.0
    ik_tolerance_m: float = 0.002
    required_clearance_m: float = 0.010

    def validate(self) -> None:
        self.start_pose_map.validate()
        vectors = (self.shoe_position_map_m, self.docking_shoe_position_base_m)
        if any(
            len(vector) != 3
            or not all(math.isfinite(float(value)) for value in vector)
            for vector in vectors
        ):
            raise ValueError("shoe and docking positions must contain three finite values")
        scalars = (
            self.navigation_position_tolerance_m,
            self.navigation_yaw_tolerance_rad,
            self.navigation_timeout_s,
            self.ik_tolerance_m,
            self.required_clearance_m,
        )
        if not all(math.isfinite(value) and value > 0 for value in scalars):
            raise ValueError("mission tolerances and timeout must be positive")
        if not math.isfinite(self.docking_yaw_rad):
            raise ValueError("docking_yaw_rad must be finite")

    def docking_pose_map(self) -> Pose2D:
        """Place the shoe at the requested left-gripper target in base frame."""

        self.validate()
        local_x, local_y, _ = self.docking_shoe_position_base_m
        cosine = math.cos(self.docking_yaw_rad)
        sine = math.sin(self.docking_yaw_rad)
        offset_x = cosine * local_x - sine * local_y
        offset_y = sine * local_x + cosine * local_y
        pose = Pose2D(
            self.shoe_position_map_m[0] - offset_x,
            self.shoe_position_map_m[1] - offset_y,
            self.docking_yaw_rad,
        )
        pose.validate()
        return pose


@dataclass(frozen=True)
class ShoeMissionRunReport:
    schema_version: str
    completed: bool
    final_phase: str
    transition_reports: tuple[dict[str, object], ...]
    docking_pose_map: tuple[float, float, float]
    final_base_pose_map: tuple[float, float, float]
    ik_converged: bool
    ik_iterations: int
    ik_residual_m: float
    collision_path_safe: bool
    minimum_clearance_m: float
    planned_trajectory_duration_s: float
    perception_mode: str
    mobility_mode: str
    manipulation_mode: str
    tactile_mode: str
    contact_physics_grasp_verified: bool = False
    hardware_dispatch_authorized: bool = False
    hardware_execution: bool = False

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def _mission_event(
    sequence: int,
    kind: MissionEventType,
    **overrides: object,
) -> MissionEvent:
    values: dict[str, object] = {
        "kind": kind,
        "observation_seq": sequence,
        "observation_age_ms": 0.0,
        "base_stationary": True,
    }
    values.update(overrides)
    return MissionEvent(**values)


def _navigate(
    adapter: MuJoCoMobilityAdapter,
    request: NavigationRequest,
) -> tuple[bool, Pose2D]:
    adapter.request_navigation(request)
    maximum_steps = math.ceil(
        (request.timeout_s + 0.1) / float(adapter.model.opt.timestep)
    )
    for _ in range(maximum_steps):
        adapter.advance()
        status = adapter.read_status()
        if status.goal_reached or not status.watchdog_ok:
            return status.goal_reached, status.pose_map
    adapter.safe_stop("navigation step budget exhausted")
    return False, adapter.read_status().pose_map


def run_scripted_ik_mission(
    scenario: ShoeMissionScenario | None = None,
) -> ShoeMissionRunReport:
    resolved = scenario or ShoeMissionScenario()
    resolved.validate()
    shoe_config = ShoeTaskConfig(shoe_position_m=resolved.shoe_position_map_m)
    mobile_model = build_mobile_shoe_mission_model(shoe_config)
    mobile_data = mujoco.MjData(mobile_model)
    apply_control_as_pose(mobile_model, mobile_data, HUMANOID_HOME_ACTION)
    mobility = MuJoCoMobilityAdapter(mobile_model, mobile_data)
    mission = MissionController()
    transitions = []
    sequence = 0

    transitions.append(
        mission.dispatch(_mission_event(sequence, MissionEventType.START))
    )
    sequence += 1
    docking_pose = resolved.docking_pose_map()
    reached_b, _ = _navigate(
        mobility,
        NavigationRequest(
            goal=MissionLocation.B,
            target_pose=docking_pose,
            position_tolerance_m=resolved.navigation_position_tolerance_m,
            yaw_tolerance_rad=resolved.navigation_yaw_tolerance_rad,
            timeout_s=resolved.navigation_timeout_s,
        ),
    )
    transitions.append(
        mission.dispatch(
            _mission_event(
                sequence,
                MissionEventType.NAVIGATION_RESULT,
                location=MissionLocation.B,
                success=reached_b,
                recoverable=False,
                failure_code="" if reached_b else "navigation_to_b_failed",
            )
        )
    )
    sequence += 1
    if not reached_b:
        return _partial_report(
            mission,
            transitions,
            docking_pose,
            mobility.read_status().pose_map,
        )

    transitions.append(
        mission.dispatch(
            _mission_event(
                sequence,
                MissionEventType.POSE_RESULT,
                pose_confidence=1.0,
            )
        )
    )
    sequence += 1

    stationary_model, _ = build_stationary_arm_model(
        arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
        mount_layout="tower",
    )
    target = resolved.docking_shoe_position_base_m
    ik = solve_bimanual_position_ik(
        stationary_model,
        HUMANOID_HOME_ACTION,
        {"left": target},
        tolerance_m=resolved.ik_tolerance_m,
        max_iterations=200,
    )
    collision = check_bimanual_path(
        stationary_model,
        HUMANOID_HOME_ACTION,
        ik.action_rad,
        required_clearance_m=resolved.required_clearance_m,
    )
    trajectory = plan_septic_joint_trajectory(
        stationary_model,
        HUMANOID_HOME_ACTION,
        ik.action_rad,
        limits=MotionLimits(),
    )
    pick_ready = ik.converged and collision.safe
    transitions.append(
        mission.dispatch(
            _mission_event(
                sequence,
                MissionEventType.PICK_RESULT,
                success=pick_ready,
                recoverable=False,
                failure_code="" if pick_ready else "ik_or_collision_rejected",
            )
        )
    )
    sequence += 1
    if pick_ready:
        transitions.append(
            mission.dispatch(
                _mission_event(
                    sequence,
                    MissionEventType.GRASP_RESULT,
                    object_lifted=True,
                    gripper_holding=True,
                    carry_pose_clear=True,
                    tactile_available=True,
                    tactile_contact=True,
                )
            )
        )
        sequence += 1

    reached_a = False
    if mission.phase == MissionPhase.NAVIGATING_TO_A:
        reached_a, _ = _navigate(
            mobility,
            NavigationRequest(
                goal=MissionLocation.A,
                target_pose=resolved.start_pose_map,
                position_tolerance_m=resolved.navigation_position_tolerance_m,
                yaw_tolerance_rad=resolved.navigation_yaw_tolerance_rad,
                timeout_s=resolved.navigation_timeout_s,
            ),
        )
        transitions.append(
            mission.dispatch(
                _mission_event(
                    sequence,
                    MissionEventType.NAVIGATION_RESULT,
                    location=MissionLocation.A,
                    success=reached_a,
                    recoverable=False,
                    failure_code="" if reached_a else "navigation_to_a_failed",
                    transport_hold_ok=True,
                    tactile_available=True,
                    tactile_contact=True,
                )
            )
        )
        sequence += 1
    if reached_a and mission.phase == MissionPhase.PLACING_AT_A:
        transitions.append(
            mission.dispatch(
                _mission_event(sequence, MissionEventType.PLACE_RESULT)
            )
        )
        sequence += 1
        transitions.append(
            mission.dispatch(
                _mission_event(
                    sequence,
                    MissionEventType.RELEASE_RESULT,
                    object_released=True,
                    at_start_zone=True,
                )
            )
        )

    final_pose = mobility.read_status().pose_map
    return ShoeMissionRunReport(
        schema_version=SCHEMA_VERSION,
        completed=mission.phase == MissionPhase.COMPLETED,
        final_phase=mission.phase.value,
        transition_reports=tuple(
            transition.as_report() for transition in transitions
        ),
        docking_pose_map=(docking_pose.x_m, docking_pose.y_m, docking_pose.yaw_rad),
        final_base_pose_map=(final_pose.x_m, final_pose.y_m, final_pose.yaw_rad),
        ik_converged=ik.converged,
        ik_iterations=ik.iterations,
        ik_residual_m=max(ik.residual_m_by_side.values()),
        collision_path_safe=collision.safe,
        minimum_clearance_m=collision.minimum_clearance_m,
        planned_trajectory_duration_s=trajectory.duration_s,
        perception_mode=PERCEPTION_MODE,
        mobility_mode=MOBILITY_MODE,
        manipulation_mode=MANIPULATION_MODE,
        tactile_mode=TACTILE_MODE,
    )


def _partial_report(
    mission: MissionController,
    transitions: Sequence,
    docking_pose: Pose2D,
    final_pose: Pose2D,
) -> ShoeMissionRunReport:
    return ShoeMissionRunReport(
        schema_version=SCHEMA_VERSION,
        completed=False,
        final_phase=mission.phase.value,
        transition_reports=tuple(
            transition.as_report() for transition in transitions
        ),
        docking_pose_map=(docking_pose.x_m, docking_pose.y_m, docking_pose.yaw_rad),
        final_base_pose_map=(final_pose.x_m, final_pose.y_m, final_pose.yaw_rad),
        ik_converged=False,
        ik_iterations=0,
        ik_residual_m=math.inf,
        collision_path_safe=False,
        minimum_clearance_m=0.0,
        planned_trajectory_duration_s=0.0,
        perception_mode=PERCEPTION_MODE,
        mobility_mode=MOBILITY_MODE,
        manipulation_mode=MANIPULATION_MODE,
        tactile_mode=TACTILE_MODE,
    )


__all__ = [
    "MANIPULATION_MODE",
    "MOBILITY_MODE",
    "PERCEPTION_MODE",
    "SCHEMA_VERSION",
    "TACTILE_MODE",
    "ShoeMissionRunReport",
    "ShoeMissionScenario",
    "run_scripted_ik_mission",
]
