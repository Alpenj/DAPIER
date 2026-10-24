#!/usr/bin/env python3
"""SIM-only right-lid-open and friction-only left-shoe extraction demo.

The arms are initialized once at a verified right-wing contact pose. Runtime
motion changes actuator controls only and advances MuJoCo dynamics. The lid
uses a contact-gated hold constraint; the shoe must be pinched and lifted by
the two physical gripper sides without a weld or attachment constraint.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import time

import mujoco
import numpy as np

from box_shoe_scene import (
    BOX_BODY_NAME,
    BOX_LID_BODY_NAME,
    BOX_LID_JOINT_NAME,
    LID_GRASP_SITE_NAME,
    RIGHT_LID_GRASP_EQUALITY_NAME,
    RIGHT_LID_GRASP_SITE_NAME,
    SHOE_BODY_NAME,
    SHOE_GEOM_NAME,
    BoxShoeSceneConfig,
    build_box_shoe_scene_model,
)
from mobile_dual_so101 import ACTION_NAMES, HUMANOID_HOME_ACTION
from physics_ik import solve_bimanual_position_ik


SCHEMA_VERSION = "dapier.box-shoe-physics-demo.v0.1"
RIGHT_WING_GEOM_NAME = "box_lid_left_dust_flap"
RIGHT_CONTACT_TARGET_M = (0.18, -0.17, 0.15)
RIGHT_OPEN_WAYPOINT_ANGLES_DEG = (30.0, 60.0, 95.0)
LEFT_PREGRASP_TARGET_M = (0.20, 0.08, 0.20)
LEFT_CONTACT_TARGET_M = (0.20, 0.08, 0.082)
LEFT_WRIST_ROLL_SEED_RAD = 1.3
LEFT_GRIPPER_OPEN_RAD = 1.2
LEFT_GRIPPER_CLOSED_RAD = -0.1745
MAX_OPPOSING_CONTACT_NORMAL_DOT = -0.5
MAX_CONTACT_PENETRATION_M = 0.001
MAX_GRASP_TRANSLATION_DRIFT_M = 0.03
MAX_GRASP_ROTATION_DRIFT_RAD = 0.35
MAX_ACTUATOR_TRACKING_ERROR_RAD = 0.10
LEFT_LIFT_TARGETS_M = (
    (0.20, 0.10, 0.14),
    (0.20, 0.12, 0.20),
    (0.20, 0.12, 0.26),
    (0.18, 0.16, 0.32),
)
LEFT_EXTRACT_TARGET_M = (0.16, 0.22, 0.32)


@dataclass(frozen=True)
class DemoConfig:
    physics_timestep_s: float = 0.01
    move_steps: int = 300
    hold_steps: int = 60
    minimum_open_angle_deg: float = 90.0
    clear_height_margin_m: float = 0.005

    def validate(self) -> None:
        if not math.isfinite(self.physics_timestep_s) or not (
            0.0 < self.physics_timestep_s <= 0.02
        ):
            raise ValueError("physics_timestep_s must be inside (0, 0.02]")
        if (
            isinstance(self.move_steps, bool)
            or not isinstance(self.move_steps, int)
            or self.move_steps <= 0
            or isinstance(self.hold_steps, bool)
            or not isinstance(self.hold_steps, int)
            or self.hold_steps < 0
        ):
            raise ValueError("move_steps must be positive and hold_steps non-negative")
        if not 0.0 < self.minimum_open_angle_deg <= 120.0:
            raise ValueError("minimum_open_angle_deg must be inside (0, 120]")
        if (
            not math.isfinite(self.clear_height_margin_m)
            or self.clear_height_margin_m <= 0.0
        ):
            raise ValueError("clear_height_margin_m must be finite and positive")


@dataclass(frozen=True)
class DemoReport:
    schema_version: str
    success: bool
    completed_phase: str
    right_wing_contact_count: int
    left_shoe_contact_count: int
    right_latch_contact_gated: bool
    left_static_finger_contact_count: int
    left_moving_finger_contact_count: int
    bilateral_finger_contact_verified: bool
    finger_contact_normal_dot: float | None
    opposing_finger_contact_verified: bool
    shoe_grasp_weld_present: bool
    friction_lift_verified: bool
    maximum_grasp_relative_translation_drift_m: float | None
    maximum_grasp_relative_rotation_drift_rad: float | None
    maximum_actuator_tracking_error_rad: float
    lid_open_angle_deg: float
    shoe_initial_xyz_m: tuple[float, float, float]
    shoe_final_xyz_m: tuple[float, float, float]
    shoe_bottom_clearance_m: float
    shoe_clear_of_box: bool
    final_shoe_box_contact_count: int
    finite_state: bool
    final_tracking_error_rad: float
    maximum_actuator_force_ratio: float
    actuator_saturation_fraction_by_name: dict[str, float]
    runtime_arm_qpos_writes: int
    initialization_arm_qpos_writes: int
    unintended_contact_pairs: tuple[tuple[str, str], ...]
    unintended_contact_phases: tuple[tuple[str, str, str], ...]
    hardware_execution: bool = False

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def _object_id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise RuntimeError(f"MuJoCo object missing: {name}")
    return int(object_id)


def _body_name(model: mujoco.MjModel, body_id: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""


def _geom_name(model: mujoco.MjModel, geom_id: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""


def _is_descendant(model: mujoco.MjModel, body_id: int, ancestor_id: int) -> bool:
    current = body_id
    while current > 0:
        if current == ancestor_id:
            return True
        current = int(model.body_parentid[current])
    return False


def _matching_contacts(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    geom_name: str,
    arm_side: str,
) -> list[mujoco.MjContact]:
    gripper_id = _object_id(
        model, mujoco.mjtObj.mjOBJ_BODY, f"{arm_side}_gripper"
    )
    moving_jaw_id = _object_id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        f"{arm_side}_moving_jaw_so101_v1",
    )
    matches = []
    for index in range(data.ncon):
        contact = data.contact[index]
        geom_ids = (int(contact.geom1), int(contact.geom2))
        if geom_name not in {_geom_name(model, geom_id) for geom_id in geom_ids}:
            continue
        body_ids = tuple(int(model.geom_bodyid[geom_id]) for geom_id in geom_ids)
        if any(
            _is_descendant(model, body_id, gripper_id)
            or _is_descendant(model, body_id, moving_jaw_id)
            for body_id in body_ids
        ):
            matches.append(contact)
    return matches


def _shoe_finger_contacts(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> tuple[list[mujoco.MjContact], list[mujoco.MjContact]]:
    """Return shoe contacts on the fixed and moving gripper sides separately."""

    shoe_geom = _object_id(model, mujoco.mjtObj.mjOBJ_GEOM, SHOE_GEOM_NAME)
    fixed_body = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, "left_gripper")
    moving_body = _object_id(
        model, mujoco.mjtObj.mjOBJ_BODY, "left_moving_jaw_so101_v1"
    )
    fixed_contacts: list[mujoco.MjContact] = []
    moving_contacts: list[mujoco.MjContact] = []
    for contact in data.contact[: data.ncon]:
        geom_ids = (int(contact.geom1), int(contact.geom2))
        if shoe_geom not in geom_ids:
            continue
        other_geom = geom_ids[1] if geom_ids[0] == shoe_geom else geom_ids[0]
        other_body = int(model.geom_bodyid[other_geom])
        if other_body == fixed_body:
            fixed_contacts.append(contact)
        elif _is_descendant(model, other_body, moving_body):
            moving_contacts.append(contact)
    return fixed_contacts, moving_contacts


def _finger_contact_normal_dot(
    model: mujoco.MjModel,
    fixed_contacts: list[mujoco.MjContact],
    moving_contacts: list[mujoco.MjContact],
) -> float | None:
    """Return the most opposing pair of normals, both oriented away from the shoe."""

    if not fixed_contacts or not moving_contacts:
        return None
    shoe_geom = _object_id(model, mujoco.mjtObj.mjOBJ_GEOM, SHOE_GEOM_NAME)

    def shoe_outward(contact: mujoco.MjContact) -> np.ndarray:
        normal = np.asarray(contact.frame[:3], dtype=np.float64)
        return normal if int(contact.geom1) == shoe_geom else -normal

    return _most_opposing_normal_dot(
        [shoe_outward(contact) for contact in fixed_contacts],
        [shoe_outward(contact) for contact in moving_contacts],
    )


def _most_opposing_normal_dot(
    fixed_normals: list[np.ndarray], moving_normals: list[np.ndarray]
) -> float | None:
    if not fixed_normals or not moving_normals:
        return None
    return min(
        float(np.dot(fixed, moving))
        for fixed in fixed_normals
        for moving in moving_normals
    )


def _shoe_pose_in_left_gripper(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> tuple[np.ndarray, np.ndarray]:
    gripper_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, "left_gripper")
    shoe_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, SHOE_BODY_NAME)
    gripper_rotation = data.xmat[gripper_id].reshape(3, 3)
    shoe_rotation = data.xmat[shoe_id].reshape(3, 3)
    return (
        gripper_rotation.T @ (data.xpos[shoe_id] - data.xpos[gripper_id]),
        gripper_rotation.T @ shoe_rotation,
    )


def _rotation_distance_rad(first: np.ndarray, second: np.ndarray) -> float:
    relative = first.T @ second
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(cosine)


def _continuous_lift_evidence_ok(
    translation_drift_m: float | None,
    rotation_drift_rad: float | None,
    actuator_tracking_error_rad: float,
) -> bool:
    return (
        translation_drift_m is not None
        and rotation_drift_rad is not None
        and translation_drift_m <= MAX_GRASP_TRANSLATION_DRIFT_M
        and rotation_drift_rad <= MAX_GRASP_ROTATION_DRIFT_RAD
        and actuator_tracking_error_rad <= MAX_ACTUATOR_TRACKING_ERROR_RAD
    )


def _classify_unintended_contact(
    body_names: tuple[str, str],
    *,
    distance_m: float,
    right_lid_hold_active: bool,
) -> tuple[str, str] | None:
    pair = tuple(sorted(body_names))
    if distance_m >= -MAX_CONTACT_PENETRATION_M or len(set(pair)) < 2:
        return None
    if not (
        any(name.startswith(("left_", "right_")) for name in pair)
        or SHOE_BODY_NAME in pair
        and (BOX_BODY_NAME in pair or BOX_LID_BODY_NAME in pair)
    ):
        return None
    return pair


def _configure_connect_sites_at_contact(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    contact_position: np.ndarray,
) -> None:
    for site_name in (RIGHT_LID_GRASP_SITE_NAME, LID_GRASP_SITE_NAME):
        site_id = _object_id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        body_id = int(model.site_bodyid[site_id])
        body_rotation = data.xmat[body_id].reshape(3, 3)
        model.site_sameframe[site_id] = 0
        model.site_pos[site_id] = body_rotation.T @ (
            contact_position - data.xpos[body_id]
        )
    mujoco.mj_forward(model, data)


def _lid_site_target_at_angle(
    model: mujoco.MjModel,
    action: np.ndarray,
    actuated_qpos_addresses: np.ndarray,
    angle_deg: float,
) -> np.ndarray:
    planning_data = mujoco.MjData(model)
    mujoco.mj_resetData(model, planning_data)
    planning_data.qpos[actuated_qpos_addresses] = action
    lid_joint = _object_id(
        model, mujoco.mjtObj.mjOBJ_JOINT, BOX_LID_JOINT_NAME
    )
    planning_data.qpos[int(model.jnt_qposadr[lid_joint])] = math.radians(angle_deg)
    mujoco.mj_forward(model, planning_data)
    lid_site = _object_id(
        model, mujoco.mjtObj.mjOBJ_SITE, LID_GRASP_SITE_NAME
    )
    return planning_data.site_xpos[lid_site].copy()


def _quintic_weight(fraction: float) -> float:
    return 10.0 * fraction**3 - 15.0 * fraction**4 + 6.0 * fraction**5


class BoxShoePhysicsDemo:
    def __init__(self, config: DemoConfig | None = None) -> None:
        self.config = config or DemoConfig()
        self.config.validate()
        self.scene_config = BoxShoeSceneConfig()
        self.model = build_box_shoe_scene_model(self.scene_config)
        self.model.opt.timestep = self.config.physics_timestep_s
        self.data = mujoco.MjData(self.model)
        self.actuated_joint_ids = self.model.actuator_trnid[:, 0].astype(np.int32)
        self.actuated_qpos_addresses = self.model.jnt_qposadr[
            self.actuated_joint_ids
        ].astype(np.int32)
        self.unintended_contacts: set[tuple[str, str]] = set()
        self.unintended_contact_phases: set[tuple[str, str, str]] = set()
        self._grasp_reference: tuple[np.ndarray, np.ndarray] | None = None
        self.maximum_grasp_relative_translation_drift_m: float | None = None
        self.maximum_grasp_relative_rotation_drift_rad: float | None = None
        self.maximum_actuator_tracking_error_rad = 0.0
        self.maximum_actuator_force_ratio = 0.0
        self.dynamics_sample_count = 0
        self.actuator_saturation_steps = np.zeros(self.model.nu, dtype=np.int64)

    def _start_grasp_tracking(self) -> None:
        self._grasp_reference = _shoe_pose_in_left_gripper(self.model, self.data)
        self.maximum_grasp_relative_translation_drift_m = 0.0
        self.maximum_grasp_relative_rotation_drift_rad = 0.0

    def _sample_dynamics(self) -> None:
        force_limit = np.maximum(
            np.abs(self.model.actuator_forcerange[:, 0]),
            np.abs(self.model.actuator_forcerange[:, 1]),
        )
        force_ratio = np.abs(self.data.actuator_force) / force_limit
        self.maximum_actuator_force_ratio = max(
            self.maximum_actuator_force_ratio,
            float(np.max(force_ratio)),
        )
        self.dynamics_sample_count += 1
        self.actuator_saturation_steps += force_ratio >= 0.999
        self.maximum_actuator_tracking_error_rad = max(
            self.maximum_actuator_tracking_error_rad,
            float(
                np.max(
                    np.abs(
                        self.data.qpos[self.actuated_qpos_addresses] - self.data.ctrl
                    )
                )
            ),
        )
        if self._grasp_reference is None:
            return
        relative_position, relative_rotation = _shoe_pose_in_left_gripper(
            self.model, self.data
        )
        self.maximum_grasp_relative_translation_drift_m = max(
            self.maximum_grasp_relative_translation_drift_m or 0.0,
            float(np.linalg.norm(relative_position - self._grasp_reference[0])),
        )
        self.maximum_grasp_relative_rotation_drift_rad = max(
            self.maximum_grasp_relative_rotation_drift_rad or 0.0,
            _rotation_distance_rad(self._grasp_reference[1], relative_rotation),
        )

    def _emit(self, phase: str, **fields: object) -> None:
        print(
            json.dumps(
                {
                    "schema_version": SCHEMA_VERSION,
                    "phase": phase,
                    "sim_time_s": round(float(self.data.time), 4),
                    **fields,
                },
                ensure_ascii=False,
                sort_keys=True,
            ),
            flush=True,
        )

    def _sample_unintended_contacts(self, phase: str) -> None:
        right_lid_hold = _object_id(
            self.model,
            mujoco.mjtObj.mjOBJ_EQUALITY,
            RIGHT_LID_GRASP_EQUALITY_NAME,
        )
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom_ids = (int(contact.geom1), int(contact.geom2))
            body_names = tuple(
                _body_name(self.model, int(self.model.geom_bodyid[geom_id]))
                for geom_id in geom_ids
            )
            pair = _classify_unintended_contact(
                body_names,
                distance_m=float(contact.dist),
                right_lid_hold_active=bool(self.data.eq_active[right_lid_hold]),
            )
            if pair is not None:
                self.unintended_contacts.add(pair)
                self.unintended_contact_phases.add((phase, *pair))

    def _move(
        self,
        start_action: np.ndarray,
        goal_action: np.ndarray,
        *,
        viewer: mujoco.viewer.Handle | None,
        phase: str,
    ) -> None:
        for index in range(self.config.move_steps):
            fraction = (index + 1) / self.config.move_steps
            self.data.ctrl[:] = start_action + _quintic_weight(fraction) * (
                goal_action - start_action
            )
            mujoco.mj_step(self.model, self.data)
            self._sample_dynamics()
            self._sample_unintended_contacts(phase)
            if viewer is not None:
                viewer.sync()
                time.sleep(self.config.physics_timestep_s)
        self._emit(phase)

    def _hold(
        self,
        action: np.ndarray,
        viewer: mujoco.viewer.Handle | None,
        *,
        phase: str,
    ) -> None:
        for _ in range(self.config.hold_steps):
            self.data.ctrl[:] = action
            mujoco.mj_step(self.model, self.data)
            self._sample_dynamics()
            self._sample_unintended_contacts(phase)
            if viewer is not None:
                viewer.sync()
                time.sleep(self.config.physics_timestep_s)

    def _close_left_gripper(
        self,
        start_action: np.ndarray,
        *,
        viewer: mujoco.viewer.Handle | None,
    ) -> tuple[np.ndarray, int, int, float | None]:
        goal_action = start_action.copy()
        goal_action[5] = LEFT_GRIPPER_CLOSED_RAD
        peak_fixed = 0
        peak_moving = 0
        for index in range(self.config.move_steps):
            fraction = (index + 1) / self.config.move_steps
            self.data.ctrl[:] = start_action + _quintic_weight(fraction) * (
                goal_action - start_action
            )
            mujoco.mj_step(self.model, self.data)
            self._sample_dynamics()
            self._sample_unintended_contacts("left_gripper_close")
            fixed, moving = _shoe_finger_contacts(self.model, self.data)
            peak_fixed = max(peak_fixed, len(fixed))
            peak_moving = max(peak_moving, len(moving))
            if viewer is not None:
                viewer.sync()
                time.sleep(self.config.physics_timestep_s)
        self._hold(goal_action, viewer, phase="left_gripper_close")
        fixed, moving = _shoe_finger_contacts(self.model, self.data)
        normal_dot = _finger_contact_normal_dot(self.model, fixed, moving)
        self._emit(
            "left_gripper_closed",
            fixed_contacts=len(fixed),
            moving_contacts=len(moving),
            peak_fixed_contacts=peak_fixed,
            peak_moving_contacts=peak_moving,
            finger_contact_normal_dot=normal_dot,
        )
        return goal_action, len(fixed), len(moving), normal_dot

    def run(self, viewer: mujoco.viewer.Handle | None = None) -> DemoReport:
        right_start = list(HUMANOID_HOME_ACTION)
        right_start[10] = -1.57
        right_start[11] = 0.5
        right_contact_ik = solve_bimanual_position_ik(
            self.model,
            right_start,
            {"right": RIGHT_CONTACT_TARGET_M},
            max_iterations=250,
        )
        if not right_contact_ik.converged:
            raise RuntimeError(f"right contact IK failed: {right_contact_ik}")

        mujoco.mj_resetData(self.model, self.data)
        right_contact_action = np.asarray(right_contact_ik.action_rad)
        self.data.qpos[self.actuated_qpos_addresses] = right_contact_action
        self.data.ctrl[:] = right_contact_action
        mujoco.mj_forward(self.model, self.data)

        shoe_id = _object_id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, SHOE_BODY_NAME
        )
        shoe_initial = self.data.xpos[shoe_id].copy()
        right_contacts = _matching_contacts(
            self.model,
            self.data,
            geom_name=RIGHT_WING_GEOM_NAME,
            arm_side="right",
        )
        if not right_contacts:
            raise RuntimeError("right wing contact was not observed; latch refused")
        _configure_connect_sites_at_contact(
            self.model, self.data, np.asarray(right_contacts[0].pos)
        )
        right_equality = _object_id(
            self.model,
            mujoco.mjtObj.mjOBJ_EQUALITY,
            RIGHT_LID_GRASP_EQUALITY_NAME,
        )
        self.data.eq_active[right_equality] = 1
        mujoco.mj_forward(self.model, self.data)
        self._sample_unintended_contacts("right_wing_contact")
        self._emit("right_wing_contact", contacts=len(right_contacts))

        right_open_action = right_contact_action
        for angle_deg in RIGHT_OPEN_WAYPOINT_ANGLES_DEG:
            contact_target = _lid_site_target_at_angle(
                self.model,
                right_open_action,
                self.actuated_qpos_addresses,
                angle_deg,
            )
            right_open_ik = solve_bimanual_position_ik(
                self.model,
                right_open_action,
                {"right": contact_target},
                site_names={"right": RIGHT_LID_GRASP_SITE_NAME},
                max_iterations=250,
            )
            if not right_open_ik.converged:
                raise RuntimeError(
                    f"right open IK failed at {angle_deg} degrees: {right_open_ik}"
                )
            next_right_action = np.asarray(right_open_ik.action_rad)
            self._move(
                right_open_action,
                next_right_action,
                viewer=viewer,
                phase=f"right_open_{angle_deg:g}deg",
            )
            right_open_action = next_right_action
        self._hold(right_open_action, viewer, phase="right_lid_hold")
        lid_joint = _object_id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, BOX_LID_JOINT_NAME
        )
        lid_angle_deg = math.degrees(
            float(self.data.qpos[int(self.model.jnt_qposadr[lid_joint])])
        )
        self._emit("right_lid_hold", lid_open_angle_deg=lid_angle_deg)
        if lid_angle_deg < self.config.minimum_open_angle_deg:
            return self._report(
                "right_lid_hold_failed",
                len(right_contacts),
                0,
                shoe_initial,
            )

        left_contact_seed = right_open_action.copy()
        left_contact_seed[4] = LEFT_WRIST_ROLL_SEED_RAD
        left_contact_seed[5] = LEFT_GRIPPER_OPEN_RAD
        left_pregrasp_ik = solve_bimanual_position_ik(
            self.model,
            left_contact_seed,
            {"left": LEFT_PREGRASP_TARGET_M},
            max_iterations=250,
        )
        if not left_pregrasp_ik.converged:
            raise RuntimeError(f"left pre-grasp IK failed: {left_pregrasp_ik}")
        left_pregrasp_action = np.asarray(left_pregrasp_ik.action_rad)
        self._move(
            right_open_action,
            left_pregrasp_action,
            viewer=viewer,
            phase="left_shoe_pregrasp",
        )
        self._hold(left_pregrasp_action, viewer, phase="left_shoe_pregrasp")

        left_contact_ik = solve_bimanual_position_ik(
            self.model,
            left_pregrasp_action,
            {"left": LEFT_CONTACT_TARGET_M},
            max_iterations=250,
        )
        if not left_contact_ik.converged:
            raise RuntimeError(f"left contact IK failed: {left_contact_ik}")
        left_contact_action = np.asarray(left_contact_ik.action_rad)
        self._move(
            left_pregrasp_action,
            left_contact_action,
            viewer=viewer,
            phase="left_shoe_approach",
        )
        self._hold(left_contact_action, viewer, phase="left_shoe_approach")
        closed_action, fixed_contacts, moving_contacts, contact_normal_dot = (
            self._close_left_gripper(
                left_contact_action,
                viewer=viewer,
            )
        )
        if fixed_contacts == 0 or moving_contacts == 0:
            return self._report(
                "left_bilateral_contact_failed",
                len(right_contacts),
                fixed_contacts + moving_contacts,
                shoe_initial,
                fixed_contact_count=fixed_contacts,
                moving_contact_count=moving_contacts,
                contact_normal_dot=contact_normal_dot,
            )
        if (
            contact_normal_dot is None
            or contact_normal_dot > MAX_OPPOSING_CONTACT_NORMAL_DOT
        ):
            return self._report(
                "left_opposing_contact_failed",
                len(right_contacts),
                fixed_contacts + moving_contacts,
                shoe_initial,
                fixed_contact_count=fixed_contacts,
                moving_contact_count=moving_contacts,
                contact_normal_dot=contact_normal_dot,
            )
        self._start_grasp_tracking()
        self._emit(
            "left_opposing_contact",
            fixed_contacts=fixed_contacts,
            moving_contacts=moving_contacts,
            finger_contact_normal_dot=contact_normal_dot,
            shoe_grasp_weld_present=False,
        )

        current_action = closed_action
        motion_targets = [
            (f"left_shoe_lift_{index}", target)
            for index, target in enumerate(LEFT_LIFT_TARGETS_M, start=1)
        ]
        motion_targets.append(("left_shoe_extract", LEFT_EXTRACT_TARGET_M))
        for phase, target in motion_targets:
            ik = solve_bimanual_position_ik(
                self.model,
                current_action,
                {"left": target},
                max_iterations=250,
            )
            if not ik.converged:
                raise RuntimeError(f"{phase} IK failed: {ik}")
            goal_action = np.asarray(ik.action_rad)
            goal_action[5] = LEFT_GRIPPER_CLOSED_RAD
            self._move(
                current_action,
                goal_action,
                viewer=viewer,
                phase=phase,
            )
            self._hold(goal_action, viewer, phase=phase)
            current_action = goal_action

        final_fixed_contacts, final_moving_contacts = _shoe_finger_contacts(
            self.model, self.data
        )
        final_contact_normal_dot = _finger_contact_normal_dot(
            self.model, final_fixed_contacts, final_moving_contacts
        )
        return self._report(
            "completed",
            len(right_contacts),
            len(final_fixed_contacts) + len(final_moving_contacts),
            shoe_initial,
            fixed_contact_count=len(final_fixed_contacts),
            moving_contact_count=len(final_moving_contacts),
            contact_normal_dot=final_contact_normal_dot,
        )

    def _report(
        self,
        phase: str,
        right_contact_count: int,
        left_contact_count: int,
        shoe_initial: np.ndarray,
        *,
        fixed_contact_count: int = 0,
        moving_contact_count: int = 0,
        contact_normal_dot: float | None = None,
    ) -> DemoReport:
        lid_joint = _object_id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, BOX_LID_JOINT_NAME
        )
        lid_angle_deg = math.degrees(
            float(self.data.qpos[int(self.model.jnt_qposadr[lid_joint])])
        )
        shoe_id = _object_id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, SHOE_BODY_NAME
        )
        shoe_final = self.data.xpos[shoe_id].copy()
        shoe_geom = _object_id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, SHOE_GEOM_NAME
        )
        shoe_rotation = self.data.geom_xmat[shoe_geom].reshape(3, 3)
        world_half_extents = np.abs(shoe_rotation) @ self.model.geom_size[shoe_geom]
        shoe_bottom = float(self.data.geom_xpos[shoe_geom, 2] - world_half_extents[2])
        clearance = shoe_bottom - self.scene_config.box_outer_size_m[2]
        final_shoe_box_contacts = 0
        for contact in self.data.contact[: self.data.ncon]:
            body_names = {
                _body_name(
                    self.model, int(self.model.geom_bodyid[int(geom_id)])
                )
                for geom_id in (contact.geom1, contact.geom2)
            }
            if SHOE_BODY_NAME in body_names and body_names.intersection(
                {BOX_LID_BODY_NAME, "box_fixture"}
            ):
                final_shoe_box_contacts += 1
        clear = (
            clearance >= self.config.clear_height_margin_m
            and final_shoe_box_contacts == 0
        )
        final_tracking_error = float(
            np.max(
                np.abs(
                    self.data.qpos[self.actuated_qpos_addresses] - self.data.ctrl
                )
            )
        )
        finite = bool(
            np.all(np.isfinite(self.data.qpos))
            and np.all(np.isfinite(self.data.qvel))
            and np.all(np.isfinite(self.data.qacc))
        )
        bilateral = fixed_contact_count > 0 and moving_contact_count > 0
        opposing = (
            bilateral
            and contact_normal_dot is not None
            and contact_normal_dot <= MAX_OPPOSING_CONTACT_NORMAL_DOT
        )
        shoe_grasp_weld_present = (
            mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_EQUALITY,
                "left_shoe_grasp_latch",
            )
            >= 0
        )
        continuous_evidence = _continuous_lift_evidence_ok(
            self.maximum_grasp_relative_translation_drift_m,
            self.maximum_grasp_relative_rotation_drift_rad,
            self.maximum_actuator_tracking_error_rad,
        )
        friction_lift = (
            opposing
            and not shoe_grasp_weld_present
            and clear
            and continuous_evidence
        )
        success = (
            phase == "completed"
            and right_contact_count > 0
            and friction_lift
            and lid_angle_deg >= self.config.minimum_open_angle_deg
            and clear
            and finite
            and final_tracking_error <= MAX_ACTUATOR_TRACKING_ERROR_RAD
            and not self.unintended_contacts
        )
        report = DemoReport(
            schema_version=SCHEMA_VERSION,
            success=success,
            completed_phase=phase,
            right_wing_contact_count=right_contact_count,
            left_shoe_contact_count=left_contact_count,
            right_latch_contact_gated=right_contact_count > 0,
            left_static_finger_contact_count=fixed_contact_count,
            left_moving_finger_contact_count=moving_contact_count,
            bilateral_finger_contact_verified=bilateral,
            finger_contact_normal_dot=contact_normal_dot,
            opposing_finger_contact_verified=opposing,
            shoe_grasp_weld_present=shoe_grasp_weld_present,
            friction_lift_verified=friction_lift,
            maximum_grasp_relative_translation_drift_m=(
                self.maximum_grasp_relative_translation_drift_m
            ),
            maximum_grasp_relative_rotation_drift_rad=(
                self.maximum_grasp_relative_rotation_drift_rad
            ),
            maximum_actuator_tracking_error_rad=(
                self.maximum_actuator_tracking_error_rad
            ),
            lid_open_angle_deg=lid_angle_deg,
            shoe_initial_xyz_m=tuple(float(value) for value in shoe_initial),
            shoe_final_xyz_m=tuple(float(value) for value in shoe_final),
            shoe_bottom_clearance_m=clearance,
            shoe_clear_of_box=clear,
            final_shoe_box_contact_count=final_shoe_box_contacts,
            finite_state=finite,
            final_tracking_error_rad=final_tracking_error,
            maximum_actuator_force_ratio=self.maximum_actuator_force_ratio,
            actuator_saturation_fraction_by_name={
                name: float(count / max(self.dynamics_sample_count, 1))
                for name, count in zip(
                    ACTION_NAMES, self.actuator_saturation_steps, strict=True
                )
            },
            runtime_arm_qpos_writes=0,
            initialization_arm_qpos_writes=1,
            unintended_contact_pairs=tuple(sorted(self.unintended_contacts)),
            unintended_contact_phases=tuple(
                sorted(self.unintended_contact_phases)
            ),
        )
        self._emit("report", report=report.as_dict())
        return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--move-steps", type=int, default=DemoConfig.move_steps)
    args = parser.parse_args()
    demo = BoxShoePhysicsDemo(DemoConfig(move_steps=args.move_steps))
    if args.viewer:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(demo.model, demo.data) as viewer:
            viewer.cam.lookat[:] = [0.10, 0.0, 0.20]
            viewer.cam.distance = 0.72
            viewer.cam.azimuth = 145
            viewer.cam.elevation = -18
            report = demo.run(viewer)
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.01)
    else:
        report = demo.run()
    print(json.dumps(report.as_dict(), ensure_ascii=False, sort_keys=True))
    return 0 if report.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
