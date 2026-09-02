"""Primitive MuJoCo scene for the box-open-and-extract task.

The shoe is deliberately one free cuboid. The box is a fixed five-panel
container with a passive hinged lid. This is a geometry and coordination
baseline, not a claim of realistic footwear contact physics.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import mujoco

from mobile_dual_so101 import (
    ACTION_NAMES,
    HUMANOID_HOME_ACTION,
    TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
    apply_control_as_pose,
    build_spec as build_mobile_spec,
)


SCHEMA_VERSION = "dapier.box-shoe-scene.v0.1"
SHOE_REPRESENTATION = "CUBOID_PROXY"
SHOE_BODY_NAME = "cuboid_shoe_proxy"
SHOE_JOINT_NAME = "cuboid_shoe_free"
SHOE_GEOM_NAME = "cuboid_shoe_geom"
BOX_BODY_NAME = "box_fixture"
BOX_LID_BODY_NAME = "box_lid"
BOX_LID_JOINT_NAME = "box_lid_hinge"
RIGHT_LID_GRASP_EQUALITY_NAME = "right_lid_grasp_latch"
LEFT_SHOE_GRASP_EQUALITY_NAME = "left_shoe_grasp_latch"
RIGHT_LID_GRASP_SITE_NAME = "right_lid_grasp_site"
LID_GRASP_SITE_NAME = "lid_grasp_site"
BOX_GEOM_NAMES = (
    "box_floor",
    "box_wall_left",
    "box_wall_right",
    "box_wall_front",
    "box_wall_back",
    "box_lid_panel",
    "box_lid_front_tuck_flap",
    "box_lid_left_dust_flap",
    "box_lid_right_dust_flap",
)


@dataclass(frozen=True)
class BoxShoeSceneConfig:
    box_center_xy_m: tuple[float, float] = (0.20, 0.0)
    box_yaw_deg: float = 90.0
    box_outer_size_m: tuple[float, float, float] = (0.282, 0.210, 0.105)
    cardboard_thickness_m: float = 0.0015
    cardboard_density_kg_m3: float = 280.0
    lid_front_tuck_depth_m: float = 0.055
    lid_dust_flap_width_m: float = 0.040
    lid_max_angle_deg: float = 120.0
    shoe_half_size_m: tuple[float, float, float] = (0.11, 0.04, 0.04)
    shoe_density_kg_m3: float = 220.0

    def validate(self) -> None:
        vectors = (
            self.box_center_xy_m,
            self.box_outer_size_m,
            self.shoe_half_size_m,
        )
        if any(
            not all(math.isfinite(float(value)) for value in vector)
            for vector in vectors
        ):
            raise ValueError("scene dimensions must be finite")
        if len(self.box_center_xy_m) != 2 or len(self.box_outer_size_m) != 3:
            raise ValueError("box center and outer size dimensions do not match")
        if len(self.shoe_half_size_m) != 3:
            raise ValueError("shoe_half_size_m must contain three values")
        positive = (
            *self.box_outer_size_m,
            self.cardboard_thickness_m,
            self.cardboard_density_kg_m3,
            self.lid_front_tuck_depth_m,
            self.lid_dust_flap_width_m,
            *self.shoe_half_size_m,
            self.shoe_density_kg_m3,
            self.lid_max_angle_deg,
        )
        if not math.isfinite(float(self.box_yaw_deg)):
            raise ValueError("box_yaw_deg must be finite")
        if not all(
            math.isfinite(float(value)) and value > 0.0 for value in positive
        ):
            raise ValueError("scene sizes, density, and lid range must be positive")
        if not 0.0 < self.lid_max_angle_deg <= 180.0:
            raise ValueError("lid_max_angle_deg must be inside (0, 180]")
        outer_x, outer_y, outer_z = self.box_outer_size_m
        thickness = self.cardboard_thickness_m
        if thickness * 2.0 >= min(outer_x, outer_y, outer_z):
            raise ValueError("cardboard thickness is too large for box dimensions")
        inner_half_x = outer_x / 2.0 - thickness
        inner_half_y = outer_y / 2.0 - thickness
        shoe_x, shoe_y, shoe_z = self.shoe_half_size_m
        if shoe_x >= inner_half_x or shoe_y >= inner_half_y:
            raise ValueError("cuboid shoe must fit inside the box footprint")
        if 2.0 * shoe_z >= outer_z - thickness:
            raise ValueError("cuboid shoe must fit below the closed lid")


def _add_box(spec: mujoco.MjSpec, config: BoxShoeSceneConfig) -> None:
    center_x, center_y = config.box_center_xy_m
    outer_x, outer_y, outer_z = config.box_outer_size_m
    half_x, half_y = outer_x / 2.0, outer_y / 2.0
    thickness = config.cardboard_thickness_m
    half_t = thickness / 2.0
    yaw_rad = math.radians(config.box_yaw_deg)
    yaw_quat = [math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)]
    inner_half_y = half_y - thickness
    wall_half_z = (outer_z - thickness) / 2.0
    box = spec.worldbody.add_body(
        name=BOX_BODY_NAME,
        pos=[center_x, center_y, 0.0],
        quat=yaw_quat,
    )
    common = {
        "type": mujoco.mjtGeom.mjGEOM_BOX,
        "mass": 0.0,
        "contype": 3,
        "conaffinity": 3,
        "friction": [0.8, 0.02, 0.002],
        "rgba": [0.48, 0.31, 0.16, 1.0],
    }
    box.add_geom(
        name="box_floor",
        pos=[0.0, 0.0, half_t],
        size=[half_x, half_y, half_t],
        **common,
    )
    wall_z = thickness + wall_half_z
    box.add_geom(
        name="box_wall_left",
        pos=[0.0, half_y - half_t, wall_z],
        size=[half_x, half_t, wall_half_z],
        **common,
    )
    box.add_geom(
        name="box_wall_right",
        pos=[0.0, -half_y + half_t, wall_z],
        size=[half_x, half_t, wall_half_z],
        **common,
    )
    box.add_geom(
        name="box_wall_front",
        pos=[half_x - half_t, 0.0, wall_z],
        size=[half_t, inner_half_y, wall_half_z],
        **common,
    )
    box.add_geom(
        name="box_wall_back",
        pos=[-half_x + half_t, 0.0, wall_z],
        size=[half_t, inner_half_y, wall_half_z],
        **common,
    )

    lid = box.add_body(
        name=BOX_LID_BODY_NAME,
        pos=[0.0, half_y, outer_z],
    )
    lid.add_site(name=LID_GRASP_SITE_NAME, size=[0.004, 0.0, 0.0])
    spec.body("right_gripper").add_site(
        name=RIGHT_LID_GRASP_SITE_NAME,
        size=[0.004, 0.0, 0.0],
    )
    lid.add_joint(
        name=BOX_LID_JOINT_NAME,
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=[-1.0, 0.0, 0.0],
        range=[0.0, config.lid_max_angle_deg],
        damping=0.10,
        armature=0.002,
    )
    lid_common = {
        "type": mujoco.mjtGeom.mjGEOM_BOX,
        "density": config.cardboard_density_kg_m3,
        "contype": 3,
        "conaffinity": 3,
        "friction": [0.8, 0.02, 0.002],
    }
    flexible_visual_common = {
        **lid_common,
        "contype": 0,
        "conaffinity": 0,
    }
    lid.add_geom(
        name="box_lid_panel",
        pos=[0.0, -half_y, 0.0],
        size=[half_x, half_y, half_t],
        rgba=[0.55, 0.36, 0.18, 1.0],
        **lid_common,
    )
    lid.add_geom(
        name="box_lid_front_tuck_flap",
        pos=[0.0, -outer_y - config.lid_front_tuck_depth_m / 2.0, 0.0],
        size=[half_x, config.lid_front_tuck_depth_m / 2.0, half_t],
        rgba=[0.58, 0.39, 0.20, 1.0],
        **flexible_visual_common,
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        wing_common = lid_common if side == "right" else flexible_visual_common
        lid.add_geom(
            name=f"box_lid_{side}_dust_flap",
            pos=[
                sign * (half_x + config.lid_dust_flap_width_m / 2.0),
                -half_y,
                0.0,
            ],
            size=[config.lid_dust_flap_width_m / 2.0, half_y, half_t],
            rgba=[0.58, 0.39, 0.20, 1.0],
            **wing_common,
        )
    spec.add_equality(
        name=RIGHT_LID_GRASP_EQUALITY_NAME,
        type=mujoco.mjtEq.mjEQ_CONNECT,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        name1=RIGHT_LID_GRASP_SITE_NAME,
        name2=LID_GRASP_SITE_NAME,
        active=False,
    )


def _add_cuboid_shoe(spec: mujoco.MjSpec, config: BoxShoeSceneConfig) -> None:
    center_x, center_y = config.box_center_xy_m
    half_x, half_y, half_z = config.shoe_half_size_m
    yaw_rad = math.radians(config.box_yaw_deg)
    shoe = spec.worldbody.add_body(
        name=SHOE_BODY_NAME,
        pos=[
            center_x,
            center_y,
            config.cardboard_thickness_m + half_z + 0.002,
        ],
        quat=[math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)],
    )
    shoe.add_freejoint(name=SHOE_JOINT_NAME)
    shoe.add_geom(
        name=SHOE_GEOM_NAME,
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[half_x, half_y, half_z],
        density=config.shoe_density_kg_m3,
        friction=[0.9, 0.02, 0.002],
        condim=4,
        contype=3,
        conaffinity=3,
        rgba=[0.18, 0.32, 0.75, 1.0],
    )
    spec.add_equality(
        name=LEFT_SHOE_GRASP_EQUALITY_NAME,
        type=mujoco.mjtEq.mjEQ_WELD,
        objtype=mujoco.mjtObj.mjOBJ_BODY,
        name1="left_gripper",
        name2=SHOE_BODY_NAME,
        active=False,
        data=[
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.05
        ],
    )


def build_box_shoe_scene_model(
    config: BoxShoeSceneConfig | None = None,
    *,
    model_path: str | None = None,
) -> mujoco.MjModel:
    resolved = config or BoxShoeSceneConfig()
    resolved.validate()
    spec, _ = build_mobile_spec(
        arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
        mount_layout="tower",
        model_path=model_path,
    )
    _add_box(spec, resolved)
    _add_cuboid_shoe(spec, resolved)
    return spec.compile()


def create_box_shoe_scene_data(model: mujoco.MjModel) -> mujoco.MjData:
    """Create scene state with the CAD-matched dual-arm home pose applied."""

    data = mujoco.MjData(model)
    apply_control_as_pose(model, data, HUMANOID_HOME_ACTION)
    return data


def box_shoe_scene_contract(model: mujoco.MjModel) -> dict[str, object]:
    object_ids = {
        "shoe_body": mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, SHOE_BODY_NAME
        ),
        "shoe_free_joint": mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, SHOE_JOINT_NAME
        ),
        "lid_hinge": mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, BOX_LID_JOINT_NAME
        ),
    }
    if min(object_ids.values()) < 0:
        raise RuntimeError(f"box-shoe scene contract object missing: {object_ids}")
    lid_joint = object_ids["lid_hinge"]
    return {
        "schema_version": SCHEMA_VERSION,
        "shoe_representation": SHOE_REPRESENTATION,
        "shoe_single_cuboid": True,
        "shoe_proxy_dimensions_status": "PROVISIONAL_UNTIL_SHOE_MEASUREMENT",
        "box_primitive_geometry": True,
        "box_outer_size_m": BoxShoeSceneConfig().box_outer_size_m,
        "box_yaw_deg": BoxShoeSceneConfig().box_yaw_deg,
        "cardboard_thickness_m": BoxShoeSceneConfig().cardboard_thickness_m,
        "flap_dimensions_status": "PROVISIONAL_FROM_TWO_PHOTOS",
        "lid_passive_hinge": True,
        "contact_gated_grasp_latches": True,
        "lid_range_rad": tuple(float(value) for value in model.jnt_range[lid_joint]),
        "robot_actuator_count": model.nu,
        "expected_robot_actuator_count": len(ACTION_NAMES),
        "box_actuator_present": model.nu != len(ACTION_NAMES),
        "contact_physics_task_verified": False,
        "hardware_dispatch_authorized": False,
        "hardware_execution": False,
    }


__all__ = [
    "BOX_BODY_NAME",
    "BOX_GEOM_NAMES",
    "BOX_LID_BODY_NAME",
    "BOX_LID_JOINT_NAME",
    "LEFT_SHOE_GRASP_EQUALITY_NAME",
    "LID_GRASP_SITE_NAME",
    "RIGHT_LID_GRASP_EQUALITY_NAME",
    "RIGHT_LID_GRASP_SITE_NAME",
    "BoxShoeSceneConfig",
    "SCHEMA_VERSION",
    "SHOE_BODY_NAME",
    "SHOE_GEOM_NAME",
    "SHOE_JOINT_NAME",
    "SHOE_REPRESENTATION",
    "box_shoe_scene_contract",
    "build_box_shoe_scene_model",
    "create_box_shoe_scene_data",
]
