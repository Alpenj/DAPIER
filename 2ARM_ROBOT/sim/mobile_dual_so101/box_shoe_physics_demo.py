#!/usr/bin/env python3
"""SIM-only right-lid-open and left-shoe-extract physics demonstration.

The arms are initialized once at a verified right-wing contact pose. Runtime
motion changes actuator controls only and advances MuJoCo dynamics. Grasp
latches remain inactive until the matching contact is observed.
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
    BOX_LID_BODY_NAME,
    BOX_LID_JOINT_NAME,
    LEFT_SHOE_GRASP_EQUALITY_NAME,
    LID_GRASP_SITE_NAME,
    RIGHT_LID_GRASP_EQUALITY_NAME,
    RIGHT_LID_GRASP_SITE_NAME,
    SHOE_BODY_NAME,
    SHOE_GEOM_NAME,
    BoxShoeSceneConfig,
    build_box_shoe_scene_model,
)
from mobile_dual_so101 import HUMANOID_HOME_ACTION
from physics_ik import solve_bimanual_position_ik


SCHEMA_VERSION = "dapier.box-shoe-physics-demo.v0.1"
RIGHT_WING_GEOM_NAME = "box_lid_right_dust_flap"
RIGHT_CONTACT_TARGET_M = (0.18, -0.17, 0.15)
RIGHT_OPEN_TARGET_M = (-0.01, -0.18, 0.265)
LEFT_PREGRASP_TARGET_M = (0.20, 0.08, 0.20)
LEFT_CONTACT_TARGET_M = (0.20, 0.08, 0.09)
LEFT_LIFT_TARGET_M = (0.16, 0.14, 0.20)
LEFT_EXTRACT_TARGET_M = (0.08, 0.26, 0.24)


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
        if self.move_steps <= 0 or self.hold_steps < 0:
            raise ValueError("move_steps must be positive and hold_steps non-negative")
        if not 0.0 < self.minimum_open_angle_deg <= 120.0:
            raise ValueError("minimum_open_angle_deg must be inside (0, 120]")


@dataclass(frozen=True)
class DemoReport:
    schema_version: str
    success: bool
    completed_phase: str
    right_wing_contact_count: int
    left_shoe_contact_count: int
    right_latch_contact_gated: bool
    left_latch_contact_gated: bool
    lid_open_angle_deg: float
    shoe_initial_xyz_m: tuple[float, float, float]
    shoe_final_xyz_m: tuple[float, float, float]
    shoe_bottom_clearance_m: float
    shoe_clear_of_box: bool
    finite_state: bool
    runtime_arm_qpos_writes: int
    initialization_arm_qpos_writes: int
    unintended_contact_pairs: tuple[tuple[str, str], ...]
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


def _activate_body_weld(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    equality_name: str,
) -> None:
    equality_id = _object_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, equality_name)
    body1 = int(model.eq_obj1id[equality_id])
    body2 = int(model.eq_obj2id[equality_id])
    inverse_position = np.zeros(3)
    inverse_quaternion = np.zeros(4)
    relative_position = np.zeros(3)
    relative_quaternion = np.zeros(4)
    mujoco.mju_negPose(
        inverse_position,
        inverse_quaternion,
        data.xpos[body1],
        data.xquat[body1],
    )
    mujoco.mju_mulPose(
        relative_position,
        relative_quaternion,
        inverse_position,
        inverse_quaternion,
        data.xpos[body2],
        data.xquat[body2],
    )
    model.eq_data[equality_id, 3:6] = relative_position
    model.eq_data[equality_id, 6:10] = relative_quaternion
    data.eq_active[equality_id] = 1
    mujoco.mj_forward(model, data)


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

    def _sample_unintended_contacts(self) -> None:
        intended_geoms = {RIGHT_WING_GEOM_NAME, SHOE_GEOM_NAME}
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom_ids = (int(contact.geom1), int(contact.geom2))
            geom_names = tuple(_geom_name(self.model, geom_id) for geom_id in geom_ids)
            if intended_geoms.intersection(geom_names):
                continue
            body_names = tuple(
                _body_name(self.model, int(self.model.geom_bodyid[geom_id]))
                for geom_id in geom_ids
            )
            right_holds_lid = (
                BOX_LID_BODY_NAME in body_names
                and any(name.startswith("right_") for name in body_names)
            )
            if right_holds_lid:
                continue
            arm_structure = any(
                name.startswith(("left_", "right_")) for name in body_names
            )
            box_structure = any(
                name == BOX_LID_BODY_NAME or name == "box_fixture"
                for name in body_names
            )
            if arm_structure and box_structure and float(contact.dist) < -0.001:
                self.unintended_contacts.add(tuple(sorted(body_names)))

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
            if index % 10 == 0:
                self._sample_unintended_contacts()
            if viewer is not None:
                viewer.sync()
        self._emit(phase)

    def _hold(self, action: np.ndarray, viewer: mujoco.viewer.Handle | None) -> None:
        for _ in range(self.config.hold_steps):
            self.data.ctrl[:] = action
            mujoco.mj_step(self.model, self.data)
            if viewer is not None:
                viewer.sync()

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
        self._emit("right_wing_contact", contacts=len(right_contacts))

        right_open_ik = solve_bimanual_position_ik(
            self.model,
            right_contact_action,
            {"right": RIGHT_OPEN_TARGET_M},
            max_iterations=250,
        )
        if not right_open_ik.converged:
            raise RuntimeError(f"right open IK failed: {right_open_ik}")
        right_open_action = np.asarray(right_open_ik.action_rad)
        self._move(
            right_contact_action,
            right_open_action,
            viewer=viewer,
            phase="right_open_motion",
        )
        self._hold(right_open_action, viewer)
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
        left_contact_seed[4] = -1.57
        left_contact_seed[5] = 0.2
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
        self._hold(left_pregrasp_action, viewer)

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
        self._hold(left_contact_action, viewer)
        left_contacts = _matching_contacts(
            self.model,
            self.data,
            geom_name=SHOE_GEOM_NAME,
            arm_side="left",
        )
        if not left_contacts:
            return self._report(
                "left_shoe_contact_failed",
                len(right_contacts),
                0,
                shoe_initial,
            )
        _activate_body_weld(
            self.model, self.data, LEFT_SHOE_GRASP_EQUALITY_NAME
        )
        self._emit("left_shoe_contact", contacts=len(left_contacts))

        current_action = left_contact_action
        for phase, target in (
            ("left_shoe_lift", LEFT_LIFT_TARGET_M),
            ("left_shoe_extract", LEFT_EXTRACT_TARGET_M),
        ):
            ik = solve_bimanual_position_ik(
                self.model,
                current_action,
                {"left": target},
                max_iterations=250,
            )
            if not ik.converged:
                raise RuntimeError(f"{phase} IK failed: {ik}")
            goal_action = np.asarray(ik.action_rad)
            goal_action[5] = -0.1745
            self._move(
                current_action,
                goal_action,
                viewer=viewer,
                phase=phase,
            )
            self._hold(goal_action, viewer)
            current_action = goal_action

        return self._report(
            "completed",
            len(right_contacts),
            len(left_contacts),
            shoe_initial,
        )

    def _report(
        self,
        phase: str,
        right_contact_count: int,
        left_contact_count: int,
        shoe_initial: np.ndarray,
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
        shoe_bottom = float(
            shoe_final[2] - self.scene_config.shoe_half_size_m[2]
        )
        clearance = shoe_bottom - self.scene_config.box_outer_size_m[2]
        clear = clearance >= self.config.clear_height_margin_m
        finite = bool(
            np.all(np.isfinite(self.data.qpos))
            and np.all(np.isfinite(self.data.qvel))
            and np.all(np.isfinite(self.data.qacc))
        )
        success = (
            phase == "completed"
            and right_contact_count > 0
            and left_contact_count > 0
            and lid_angle_deg >= self.config.minimum_open_angle_deg
            and clear
            and finite
            and not self.unintended_contacts
        )
        report = DemoReport(
            schema_version=SCHEMA_VERSION,
            success=success,
            completed_phase=phase,
            right_wing_contact_count=right_contact_count,
            left_shoe_contact_count=left_contact_count,
            right_latch_contact_gated=right_contact_count > 0,
            left_latch_contact_gated=left_contact_count > 0,
            lid_open_angle_deg=lid_angle_deg,
            shoe_initial_xyz_m=tuple(float(value) for value in shoe_initial),
            shoe_final_xyz_m=tuple(float(value) for value in shoe_final),
            shoe_bottom_clearance_m=clearance,
            shoe_clear_of_box=clear,
            finite_state=finite,
            runtime_arm_qpos_writes=0,
            initialization_arm_qpos_writes=1,
            unintended_contact_pairs=tuple(sorted(self.unintended_contacts)),
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
