#!/usr/bin/env python3
"""Physics-executed dual-arm IK primitives for the stationary tower model.

IK planning uses a private MuJoCo data object and may set qpos while solving.
Runtime execution initializes qpos once, then changes actuator controls only and
advances the rigid-body dynamics with ``mj_step``. There is no ROS, serial, or
physical-hardware dispatch path in this module.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import sys
from typing import Mapping, Sequence

import mujoco
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR))

from collision_guard import check_bimanual_path  # noqa: E402
from mobile_dual_so101 import ACTION_NAMES  # noqa: E402


SEPTIC_MAX_VELOCITY_FACTOR = 2.1875
SEPTIC_MAX_ACCELERATION_FACTOR = 7.514
SEPTIC_MAX_JERK_FACTOR = 52.5
WAFFLE_SUPPORT_POLYGON_XY_M = np.asarray(
    ((-0.192, -0.064), (0.0, -0.153), (0.0, 0.153), (-0.192, 0.064)),
    dtype=np.float64,
)


@dataclass(frozen=True)
class IKResult:
    action_rad: tuple[float, ...]
    converged: bool
    iterations: int
    residual_m_by_side: dict[str, float]
    tool_axis_error_rad_by_side: dict[str, float]
    planning_qpos_writes: int
    runtime_qpos_writes: int = 0
    hardware_execution: bool = False


@dataclass(frozen=True)
class MotionLimits:
    max_velocity_rad_s: float = 0.50
    max_acceleration_rad_s2: float = 1.50
    max_jerk_rad_s3: float = 8.0

    def validate(self) -> None:
        values = (
            self.max_velocity_rad_s,
            self.max_acceleration_rad_s2,
            self.max_jerk_rad_s3,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("motion limits must be finite and positive")


@dataclass(frozen=True)
class PhysicsExecutionReport:
    duration_s: float
    simulated_steps: int
    pre_settle_steps: int
    runtime_qpos_writes: int
    maximum_tracking_error_rad: float
    final_tracking_error_rad: float
    maximum_actual_velocity_rad_s: float
    maximum_actual_acceleration_rad_s2: float
    maximum_actual_jerk_rad_s3: float
    maximum_target_velocity_rad_s: float
    maximum_target_acceleration_rad_s2: float
    maximum_target_jerk_rad_s3: float
    maximum_actuator_force_ratio: float
    torque_saturated: bool
    minimum_support_margin_m: float
    forbidden_contact_count: int
    finite_state: bool
    dynamics_limits_satisfied: bool
    dynamics_limit_violations: tuple[str, ...]
    simulation_motion_accepted: bool
    hardware_execution: bool = False

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class SepticJointTrajectory:
    start_rad: np.ndarray
    goal_rad: np.ndarray
    duration_s: float
    limits: MotionLimits

    def sample(
        self, time_s: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if not math.isfinite(time_s):
            raise ValueError("trajectory time must be finite")
        u = min(1.0, max(0.0, time_s / self.duration_s))
        u2, u3, u4 = u * u, u * u * u, u * u * u * u
        weight = 35.0 * u4 - 84.0 * u4 * u + 70.0 * u4 * u2 - 20.0 * u4 * u3
        d1 = 140.0 * u3 - 420.0 * u4 + 420.0 * u4 * u - 140.0 * u4 * u2
        d2 = 420.0 * u2 - 1680.0 * u3 + 2100.0 * u4 - 840.0 * u4 * u
        d3 = 840.0 * u - 5040.0 * u2 + 8400.0 * u3 - 4200.0 * u4
        delta = self.goal_rad - self.start_rad
        velocity_scale = 1.0 / self.duration_s
        acceleration_scale = velocity_scale**2
        jerk_scale = velocity_scale**3
        return (
            self.start_rad + weight * delta,
            d1 * velocity_scale * delta,
            d2 * acceleration_scale * delta,
            d3 * jerk_scale * delta,
        )


def _validated_action(model: mujoco.MjModel, action: Sequence[float]) -> np.ndarray:
    values = np.asarray(action, dtype=np.float64)
    if values.shape != (len(ACTION_NAMES),):
        raise ValueError(f"expected {len(ACTION_NAMES)} action values")
    if not np.all(np.isfinite(values)):
        raise ValueError("action contains non-finite values")
    lower = model.actuator_ctrlrange[:, 0]
    upper = model.actuator_ctrlrange[:, 1]
    if np.any(values < lower - 1e-9) or np.any(values > upper + 1e-9):
        raise ValueError("action exceeds actuator control range")
    return np.clip(values, lower, upper)


def _actuated_addresses(
    model: mujoco.MjModel,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    joint_ids = model.actuator_trnid[:, 0].astype(np.int32)
    if np.any(joint_ids < 0):
        raise RuntimeError("every actuator must address a joint")
    return (
        joint_ids,
        model.jnt_qposadr[joint_ids].astype(np.int32),
        model.jnt_dofadr[joint_ids].astype(np.int32),
    )


def _set_planning_action(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    action: np.ndarray,
    qpos_addresses: np.ndarray,
) -> None:
    data.qpos[qpos_addresses] = action
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def solve_bimanual_position_ik(
    model: mujoco.MjModel,
    start_action_rad: Sequence[float],
    targets_m: Mapping[str, Sequence[float]],
    *,
    site_names: Mapping[str, str] | None = None,
    tool_axis_targets: Mapping[str, Sequence[float]] | None = None,
    tool_axis_weight_m: float = 0.05,
    tool_axis_tolerance_rad: float = math.radians(2.0),
    damping: float = 0.02,
    tolerance_m: float = 5e-4,
    max_iterations: int = 100,
    max_joint_step_rad: float = 0.05,
) -> IKResult:
    """Solve arm-site XYZ and optional local-X direction using bounded DLS IK."""

    if not targets_m or any(side not in ("left", "right") for side in targets_m):
        raise ValueError("targets_m must contain left and/or right")
    if site_names is not None and set(site_names) != set(targets_m):
        raise ValueError("site_names must contain the same sides as targets_m")
    if tool_axis_targets is not None and not set(tool_axis_targets).issubset(
        targets_m
    ):
        raise ValueError("tool_axis_targets sides must also have position targets")
    scalars = (
        damping,
        tolerance_m,
        max_joint_step_rad,
        tool_axis_weight_m,
        tool_axis_tolerance_rad,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in scalars):
        raise ValueError("IK scalar parameters must be finite and positive")
    if max_iterations <= 0:
        raise ValueError("max_iterations must be positive")

    action = _validated_action(model, start_action_rad).copy()
    _, qpos_addresses, dof_addresses = _actuated_addresses(model)
    sides = tuple(side for side in ("left", "right") if side in targets_m)
    targets = {
        side: np.asarray(targets_m[side], dtype=np.float64) for side in sides
    }
    if any(target.shape != (3,) for target in targets.values()):
        raise ValueError("each IK target must contain XYZ")
    if any(not np.all(np.isfinite(target)) for target in targets.values()):
        raise ValueError("IK targets must be finite")
    axis_targets = {
        side: np.asarray(target, dtype=np.float64)
        for side, target in (tool_axis_targets or {}).items()
    }
    if any(target.shape != (3,) for target in axis_targets.values()):
        raise ValueError("each tool axis target must contain XYZ")
    if any(
        not np.all(np.isfinite(target)) or np.linalg.norm(target) <= 1e-12
        for target in axis_targets.values()
    ):
        raise ValueError("tool axis targets must be finite non-zero vectors")
    axis_targets = {
        side: target / np.linalg.norm(target)
        for side, target in axis_targets.items()
    }

    actuator_offsets = {"left": 0, "right": 6}
    actuator_ids = np.asarray(
        [
            index
            for side in sides
            for index in range(actuator_offsets[side], actuator_offsets[side] + 5)
        ],
        dtype=np.int32,
    )
    selected_qpos = qpos_addresses[actuator_ids]
    selected_dofs = dof_addresses[actuator_ids]
    selected_ranges = model.actuator_ctrlrange[actuator_ids]
    site_ids = {
        side: mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            site_names[side] if site_names is not None else f"{side}_gripperframe",
        )
        for side in sides
    }
    if min(site_ids.values()) < 0:
        raise RuntimeError("requested IK site is missing")

    planning_data = mujoco.MjData(model)
    mujoco.mj_resetData(model, planning_data)
    planning_writes = 0
    converged = False
    iterations = 0
    residuals: dict[str, float] = {}
    axis_errors: dict[str, float] = {}

    for iteration in range(max_iterations + 1):
        _set_planning_action(model, planning_data, action, qpos_addresses)
        planning_writes += 1
        errors = [
            targets[side] - planning_data.site_xpos[site_ids[side]]
            for side in sides
        ]
        residuals = {
            side: float(np.linalg.norm(error))
            for side, error in zip(sides, errors, strict=True)
        }
        current_axes = {
            side: planning_data.site_xmat[site_ids[side]].reshape(3, 3)[:, 0]
            for side in axis_targets
        }
        axis_errors = {
            side: math.acos(
                float(np.clip(np.dot(current_axes[side], target), -1.0, 1.0))
            )
            for side, target in axis_targets.items()
        }
        if max(residuals.values()) <= tolerance_m and (
            not axis_errors
            or max(axis_errors.values()) <= tool_axis_tolerance_rad
        ):
            converged = True
            iterations = iteration
            break
        if iteration == max_iterations:
            iterations = iteration
            break

        jacobian_rows = []
        residual_rows = []
        for side_index, side in enumerate(sides):
            position_jacobian = np.zeros((3, model.nv), dtype=np.float64)
            rotation_jacobian = np.zeros((3, model.nv), dtype=np.float64)
            mujoco.mj_jacSite(
                model,
                planning_data,
                position_jacobian,
                rotation_jacobian,
                site_ids[side],
            )
            jacobian_rows.append(position_jacobian[:, selected_dofs])
            residual_rows.append(errors[side_index])
            if side in axis_targets:
                jacobian_rows.append(
                    tool_axis_weight_m * rotation_jacobian[:, selected_dofs]
                )
                residual_rows.append(
                    tool_axis_weight_m
                    * np.cross(current_axes[side], axis_targets[side])
                )
        jacobian = np.vstack(jacobian_rows)
        residual = np.concatenate(residual_rows)
        regularized = jacobian @ jacobian.T + damping**2 * np.eye(len(residual))
        joint_delta = jacobian.T @ np.linalg.solve(regularized, residual)
        action[actuator_ids] += np.clip(
            joint_delta, -max_joint_step_rad, max_joint_step_rad
        )
        action[actuator_ids] = np.clip(
            action[actuator_ids], selected_ranges[:, 0], selected_ranges[:, 1]
        )

    return IKResult(
        action_rad=tuple(float(value) for value in action),
        converged=converged,
        iterations=iterations,
        residual_m_by_side=residuals,
        tool_axis_error_rad_by_side=axis_errors,
        planning_qpos_writes=planning_writes,
    )


def plan_septic_joint_trajectory(
    model: mujoco.MjModel,
    start_action_rad: Sequence[float],
    goal_action_rad: Sequence[float],
    *,
    limits: MotionLimits = MotionLimits(),
    minimum_duration_s: float = 0.50,
) -> SepticJointTrajectory:
    limits.validate()
    if not math.isfinite(minimum_duration_s) or minimum_duration_s <= 0.0:
        raise ValueError("minimum duration must be finite and positive")
    start = _validated_action(model, start_action_rad)
    goal = _validated_action(model, goal_action_rad)
    distance = np.abs(goal - start)
    duration = max(
        minimum_duration_s,
        float(
            np.max(
                distance
                * SEPTIC_MAX_VELOCITY_FACTOR
                / limits.max_velocity_rad_s
            )
        ),
        float(
            np.max(
                np.sqrt(
                    distance
                    * SEPTIC_MAX_ACCELERATION_FACTOR
                    / limits.max_acceleration_rad_s2
                )
            )
        ),
        float(
            np.max(
                np.cbrt(
                    distance * SEPTIC_MAX_JERK_FACTOR / limits.max_jerk_rad_s3
                )
            )
        ),
    )
    return SepticJointTrajectory(start, goal, duration, limits)


def _signed_support_margin(point_xy: np.ndarray) -> float:
    margins = []
    for start, end in zip(
        WAFFLE_SUPPORT_POLYGON_XY_M,
        np.roll(WAFFLE_SUPPORT_POLYGON_XY_M, -1, axis=0),
    ):
        edge = end - start
        relative = point_xy - start
        cross = edge[0] * relative[1] - edge[1] * relative[0]
        margins.append(float(cross / np.linalg.norm(edge)))
    return min(margins)


def _combined_center_of_mass(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    ids = np.arange(1, model.nbody)
    masses = model.body_mass[ids]
    return np.sum(data.xipos[ids] * masses[:, None], axis=0) / np.sum(masses)


def _forbidden_contact_count(model: mujoco.MjModel, data: mujoco.MjData) -> int:
    result = 0
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        geom_ids = (int(contact.geom1), int(contact.geom2))
        body_names = tuple(
            mujoco.mj_id2name(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                int(model.geom_bodyid[geom_id]),
            )
            or ""
            for geom_id in geom_ids
        )
        geom_names = tuple(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            for geom_id in geom_ids
        )
        left_right = (
            body_names[0].startswith("left_") and body_names[1].startswith("right_")
        ) or (
            body_names[0].startswith("right_") and body_names[1].startswith("left_")
        )
        arm_structure = any(
            name.startswith(("left_", "right_")) for name in body_names
        ) and any(
            name.endswith("_depth_camera_collision") or name.startswith("tower_")
            for name in geom_names
        )
        if left_right or arm_structure:
            result += 1
    return result


def initialize_physics_state(
    model: mujoco.MjModel, start_action_rad: Sequence[float]
) -> mujoco.MjData:
    """Reset simulation and perform the only runtime-state qpos initialization."""

    start = _validated_action(model, start_action_rad)
    _, qpos_addresses, _ = _actuated_addresses(model)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[qpos_addresses] = start
    data.qvel[:] = 0.0
    data.ctrl[:] = start
    mujoco.mj_forward(model, data)
    return data


def execute_physics_trajectory(
    model: mujoco.MjModel,
    trajectory: SepticJointTrajectory,
    *,
    initial_data: mujoco.MjData | None = None,
    pre_settle_time_s: float = 0.50,
    settle_time_s: float = 0.25,
    required_clearance_m: float = 0.030,
) -> tuple[mujoco.MjData, PhysicsExecutionReport]:
    """Execute actuator targets through MuJoCo dynamics without runtime qpos writes."""

    times = (pre_settle_time_s, settle_time_s)
    if not all(math.isfinite(value) and value >= 0.0 for value in times):
        raise ValueError("settle times must be finite and non-negative")
    collision = check_bimanual_path(
        model,
        trajectory.start_rad,
        trajectory.goal_rad,
        required_clearance_m=required_clearance_m,
    )
    if not collision.safe:
        raise ValueError(
            "trajectory rejected by collision guard: "
            f"clearance={collision.minimum_clearance_m:.6f} m"
        )

    _, qpos_addresses, dof_addresses = _actuated_addresses(model)
    data = initial_data or initialize_physics_state(model, trajectory.start_rad)
    if initial_data is not None:
        if (
            data.qpos.shape != (model.nq,)
            or data.qvel.shape != (model.nv,)
            or data.ctrl.shape != (model.nu,)
        ):
            raise ValueError("initial_data dimensions do not match model")
        if not np.all(np.isfinite(data.qpos)):
            raise ValueError("initial_data contains non-finite qpos")
        if np.max(np.abs(data.qpos[qpos_addresses] - trajectory.start_rad)) > 0.10:
            raise ValueError("initial_data is not near trajectory start")
    timestep = float(model.opt.timestep)
    pre_settle_steps = math.ceil(pre_settle_time_s / timestep)
    motion_steps = max(1, math.ceil(trajectory.duration_s / timestep))
    settle_steps = math.ceil(settle_time_s / timestep)
    for _ in range(pre_settle_steps):
        data.ctrl[:] = trajectory.start_rad
        mujoco.mj_step(model, data)
    previous_acceleration = data.qacc[dof_addresses].copy()
    maximum_tracking = 0.0
    maximum_velocity = 0.0
    maximum_acceleration = 0.0
    maximum_jerk = 0.0
    maximum_target_velocity = 0.0
    maximum_target_acceleration = 0.0
    maximum_target_jerk = 0.0
    maximum_force_ratio = 0.0
    minimum_support_margin = math.inf
    forbidden_contacts = 0

    for step_index in range(motion_steps + settle_steps):
        time_s = min((step_index + 1) * timestep, trajectory.duration_s)
        target, target_velocity, target_acceleration, target_jerk = (
            trajectory.sample(time_s)
        )
        data.ctrl[:] = target
        mujoco.mj_step(model, data)

        actual = data.qpos[qpos_addresses]
        velocity = data.qvel[dof_addresses]
        acceleration = data.qacc[dof_addresses]
        jerk = (acceleration - previous_acceleration) / timestep
        previous_acceleration = acceleration.copy()
        maximum_tracking = max(maximum_tracking, float(np.max(np.abs(actual - target))))
        maximum_velocity = max(maximum_velocity, float(np.max(np.abs(velocity))))
        maximum_acceleration = max(
            maximum_acceleration, float(np.max(np.abs(acceleration)))
        )
        maximum_jerk = max(maximum_jerk, float(np.max(np.abs(jerk))))
        maximum_target_velocity = max(
            maximum_target_velocity, float(np.max(np.abs(target_velocity)))
        )
        maximum_target_acceleration = max(
            maximum_target_acceleration, float(np.max(np.abs(target_acceleration)))
        )
        maximum_target_jerk = max(
            maximum_target_jerk, float(np.max(np.abs(target_jerk)))
        )
        force_limit = np.max(np.abs(model.actuator_forcerange), axis=1)
        limited = (model.actuator_forcelimited != 0) & (force_limit > 0.0)
        if np.any(limited):
            maximum_force_ratio = max(
                maximum_force_ratio,
                float(
                    np.max(
                        np.abs(data.actuator_force[limited]) / force_limit[limited]
                    )
                ),
            )
        minimum_support_margin = min(
            minimum_support_margin,
            _signed_support_margin(_combined_center_of_mass(model, data)[:2]),
        )
        forbidden_contacts += _forbidden_contact_count(model, data)

    final_error = float(
        np.max(np.abs(data.qpos[qpos_addresses] - trajectory.goal_rad))
    )
    finite_state = bool(
        np.all(np.isfinite(data.qpos))
        and np.all(np.isfinite(data.qvel))
        and np.all(np.isfinite(data.qacc))
    )
    numerical_tolerance = 1e-3
    dynamics_limit_violations = []
    if maximum_velocity > trajectory.limits.max_velocity_rad_s + numerical_tolerance:
        dynamics_limit_violations.append("velocity")
    if (
        maximum_acceleration
        > trajectory.limits.max_acceleration_rad_s2 + numerical_tolerance
    ):
        dynamics_limit_violations.append("acceleration")
    if maximum_jerk > trajectory.limits.max_jerk_rad_s3 + numerical_tolerance:
        dynamics_limit_violations.append("jerk")
    dynamics_limits_satisfied = not dynamics_limit_violations
    simulation_motion_accepted = (
        finite_state
        and dynamics_limits_satisfied
        and maximum_force_ratio < 0.999
        and minimum_support_margin > 0.0
        and forbidden_contacts == 0
    )
    report = PhysicsExecutionReport(
        duration_s=trajectory.duration_s,
        simulated_steps=pre_settle_steps + motion_steps + settle_steps,
        pre_settle_steps=pre_settle_steps,
        runtime_qpos_writes=0,
        maximum_tracking_error_rad=maximum_tracking,
        final_tracking_error_rad=final_error,
        maximum_actual_velocity_rad_s=maximum_velocity,
        maximum_actual_acceleration_rad_s2=maximum_acceleration,
        maximum_actual_jerk_rad_s3=maximum_jerk,
        maximum_target_velocity_rad_s=maximum_target_velocity,
        maximum_target_acceleration_rad_s2=maximum_target_acceleration,
        maximum_target_jerk_rad_s3=maximum_target_jerk,
        maximum_actuator_force_ratio=maximum_force_ratio,
        torque_saturated=maximum_force_ratio >= 0.999,
        minimum_support_margin_m=minimum_support_margin,
        forbidden_contact_count=forbidden_contacts,
        finite_state=finite_state,
        dynamics_limits_satisfied=dynamics_limits_satisfied,
        dynamics_limit_violations=tuple(dynamics_limit_violations),
        simulation_motion_accepted=simulation_motion_accepted,
    )
    return data, report
