#!/usr/bin/env python3
"""Approved integration geometry shared by viewer and SIM task; no hardware."""
import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np

from mobile_dual_so101 import resolve_so101_model, actuator_targets_from_qpos, build_waffle_pi_spec

# Latest visual-layout correction supersedes the overlapping literal-mm draft.
# These layout estimates are NOT revised physical measurements.
RAIL_WIDTH = .400
FRAME_DEPTH = .112
PROFILE = .020
ARM_OFFSET = .150
REAR_HOLE_X = -.01386471
ARM_MOUNT_X = -.046 - REAR_HOLE_X
DESK_EDGE_X = ARM_MOUNT_X - .02236471


def add_camera_plate(spec, parent, installation_z=0, camera_x=.0226408355, desk=False):
    from camera_stand_cad import add_stand
    add_stand(spec, parent, installation_z, camera_x, desk=desk)
    # Photo: plate projects into workspace, camera underneath. Offset/size and
    # optical normal remain estimates; angle now follows the original CAD.
    angle = -np.deg2rad(25)
    assembly = parent.add_body(
        name="os30a_plate_frame", pos=[camera_x, 0, installation_z + .38],
        quat=[np.cos(angle / 2), 0, np.sin(angle / 2), 0])
    assembly.add_geom(name="os30a_enclosure_UNVERIFIED",
                      type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, 0, -.015],
                      size=[.013, .045, .013], rgba=[.65, .65, .68, 1])
    assembly.add_camera(name="os30a_UNVERIFIED", pos=[0, 0, -.029],
                        quat=[0, 0, 0, 1], fovy=45.5)


def build_scene(kind, *, grippers="both"):
    if kind not in ("desk", "mobile"):
        raise ValueError("unknown integration scene")
    from pgripper import replace_gripper, selected_sides
    sides = selected_sides(grippers)
    source = resolve_so101_model()
    if kind == "desk":
        from replay_recorded_episode import build_tabletop_spec
        profile = json.loads(Path(__file__).with_name("tabletop_replay.json").read_text())
        profile["camera_height_above_table_m"] = .38
        profile["table_front_edge_x_m"] = DESK_EDGE_X
        spec = build_tabletop_spec(source, profile, grippers=grippers)
        for side in ("left", "right"):
            spec.frame(side + "_mount").pos[0] = ARM_MOUNT_X
        spec.modelname = "desk_learning_OS30A_UNVERIFIED"
        spec.delete(spec.camera("top_h201_reference"))
        spec.delete(spec.geom("camera_body"))
        spec.delete(spec.geom("camera_support"))
        spec.delete(spec.geom("camera_mast"))
        # Same stand-to-arm XY layout as mobile, per latest user photo decision.
        add_camera_plate(spec, spec.worldbody, desk=True)
        return spec
    spec = build_waffle_pi_spec()
    spec.modelname = "mobile_aluminum_USER_DIMENSIONS_UNVERIFIED"
    parent = spec.body("tb3_base_link").add_body(
        name="aluminum_mount_UNVERIFIED", pos=[-.064, 0, .094])
    spec.worldbody.add_light(pos=[0, 0, 2], dir=[0, 0, -1])
    box = mujoco.mjtGeom.mjGEOM_BOX

    def geom(name, pos, size, color):
        return parent.add_geom(
            name=name, type=box, pos=pos, size=size, rgba=color)

    for label, x in (("front", -.046), ("back", .046)):
        geom("profile_" + label, [x, 0, .010],
             [PROFILE / 2, RAIL_WIDTH / 2, PROFILE / 2], [.12, .13, .14, 1])
    # Upright mount yaw=0 and z=profile top are provisional assembly assumptions.
    for side, sign in (("left", 1), ("right", -1)):
        arm = mujoco.MjSpec.from_file(str(source))
        # Preserve source collision meshes, filters, joint limits and actuators.
        for mesh in arm.meshes:
            mesh.file = str((source.parent / arm.meshdir / mesh.file).resolve())
        if side in sides:
            replace_gripper(arm)
        frame = parent.add_frame(
            name=side + "_desk_mount", pos=[ARM_MOUNT_X, sign * ARM_OFFSET, PROFILE])
        spec.attach(arm, prefix=side + "_", frame=frame)

    # Mast/enclosure are collision-active proxies, dimensions/extrinsics UNVERIFIED.
    add_camera_plate(spec, parent, PROFILE)
    return spec



def task_env(kind="desk", *, grippers="both"):
    """Bind approved geometry; never rebuild the old tower task."""
    if kind != "desk":
        raise ValueError("mobile has no target block/work surface; mobile-to-desk transform UNVERIFIED")
    from shoe_task import ShoeTaskConfig, ShoeTaskEnv
    model = build_scene(kind, grippers=grippers).compile()
    config = ShoeTaskConfig(
        scene_id="integration_desk", grippers=grippers, object_kind="block",
        shoe_position_m=tuple(model.body("red_block").pos),
        initial_home_pose=False, shoe_xy_range_m=0, shoe_yaw_range_rad=0)
    env = ShoeTaskEnv(config, model=model)
    env.data.ctrl[:] = actuator_targets_from_qpos(model, env.data.qpos)
    mujoco.mj_forward(model, env.data)
    return env


def task_provenance(env):
    import hashlib
    import subprocess
    import tempfile
    root = Path(__file__).resolve().parents[3]
    with tempfile.TemporaryDirectory() as tmp:
        binary = Path(tmp) / "scene.mjb"
        mujoco.mj_saveModel(env.model, str(binary), None)
        model_hash = hashlib.sha256(binary.read_bytes()).hexdigest()
    source = resolve_so101_model()
    arm = mujoco.MjSpec.from_file(str(source))
    assets = {source}
    assets.update((source.parent / arm.meshdir / mesh.file).resolve()
                  for mesh in arm.meshes if mesh.file)
    assets.update(Path(__file__).with_name("assets").joinpath("camera_stand").glob("*.3mf"))
    if env.config.grippers != "stock":
        from pgripper import ASSETS
        assets.update(p for p in ASSETS.rglob("*") if p.is_file())
    sources = ("pgripper.py", "integration_scenes.py", "camera_stand_cad.py", "tabletop_replay.json",
               "replay_recorded_episode.py", "shoe_task.py", "center_block_teacher.py",
               "collision_guard.py", "collision_diagnostics.py", "integration_desk_near_support.json", "physics_ik.py")
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    return dict(
        scene_id=env.config.scene_id, builder="integration_scenes.build_scene(desk)",
        git_sha=subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip(),
        source_dirty=bool(subprocess.check_output(
            ["git", "-C", str(root), "diff", "--", "2ARM_ROBOT/sim/mobile_dual_so101"])),
        model_sha256=model_hash,
        asset_sha256={p.name: digest(p) for p in sorted(assets)},
        source_sha256={name: digest(Path(__file__).with_name(name)) for name in sources},
        mujoco_version=mujoco.__version__,
        gripper_revision=("NORMA PGripper both; pinned CAD + RGB camera stand"
                          if env.config.grippers == "both" else "SO101 stock"),
        gripper_variant=env.config.grippers,
        wrist_rgb_camera_stand=("NORMA CameraMount_square_27mm STL + RGB optics proxy"
                                if env.config.grippers != "stock" else "stock wrist camera"),
        camera_extrinsics_physically_verified=False,
        target_body="red_block", target_geom="red_block_geom",
        support_geom="table", gripper_site="left_gripperframe", ik_site="left_cube_grasp",
        observation_source="SIM truth for offline teacher validation only; not perception/runtime",
        actuator_mapping=[dict(actuator=env.model.actuator(i).name,
            joint=env.model.joint(int(env.model.actuator_trnid[i, 0])).name,
            qpos_address=int(env.model.jnt_qposadr[int(env.model.actuator_trnid[i, 0])]))
            for i in range(env.model.nu)],
    )

def inspect(model, data, kind):
    mujoco.mj_forward(model, data)
    contacts = []
    for c in data.contact:
        contacts.append({
            "geom_ids": [int(g) for g in c.geom],
            "geom_pair": [model.geom(int(g)).name for g in c.geom],
            "body_pair": [model.body(int(model.geom_bodyid[g])).name for g in c.geom],
            "signed_distance_m": float(c.dist)})
    return {
        "status": "UNVERIFIED_STATIC_DRAFT_NOT_TASK_READY",
        "scene": kind,
        "arm_mount_x_correction_m": ARM_MOUNT_X,
        "rear_mount_hole_local_x_m": REAR_HOLE_X,
        "rear_hole_pitch_m": .0635,
        "rear_profile_center_x_m": -.046,
        "desk_front_edge_x_m": DESK_EDGE_X if kind == "desk" else None,
        "stand_height_m": .38,
        "plate_angle_source": "original cam_mount_top2.3mf; supersedes earlier user165deg",
        "plate_angle_datum": "horizontal; CAD plane inclination25deg",
        "height_datum": "installation surface to plate centre; mobile profile top / desk top",
        "plate_world_position_m": data.body("os30a_plate_frame").xpos.tolist(),
        "plate_world_rotation": data.body("os30a_plate_frame").xmat.reshape(3, 3).tolist(),
        "camera_assumption": "Original 3MF geometry and native plate angle retained; inter-file assembly overlap and optical normal UNVERIFIED",
        "camera_enclosure_center_world_m": data.geom("os30a_enclosure_UNVERIFIED").xpos.tolist(),
        "cad_plate_native_tilt_deg": 25,
        "cad_plate_applied_tilt_deg": 25,
        "physics_steps": 0, "time": float(data.time),
        "frame": "desk top z=0; +X into desk, +Y left, +Z up; TurtleBot transform UNVERIFIED",
        "layout_m": {"rail_width": RAIL_WIDTH, "frame_depth": FRAME_DEPTH,
                        "profile_thickness": PROFILE, "inner_gap": FRAME_DEPTH - 2 * PROFILE,
                        "stand_to_arm_center": ARM_OFFSET},
        "layout_provenance": "User requested separated arms like desk reference; 150 mm offset and 400 mm rail are provisional layout estimates, superseding 15/40 mm draft",
        "unverified": ["desk dimensions/height", "mount orientation/height",
                       "camera optical extrinsics/intrinsics", "final gripper revision",
                       "block start/workspace", "joint zero/HOME", "TurtleBot-to-desk transform"],
        "arm_mounts_world": {s: data.body(s + "_base").xpos.tolist() for s in ("left", "right")},
        "qpos": data.qpos.tolist(), "qvel": data.qvel.tolist(),
        "ctrl": data.ctrl.tolist(),
        "ctrl_qpos_max_error": float(np.max(np.abs(data.ctrl - actuator_targets_from_qpos(model, data.qpos)))),
        "contacts": contacts,
        "minimum_contact_distance_m": min((c["signed_distance_m"] for c in contacts), default=None),
        "note": "Contact inventory is not a full clearance certificate. Zero pose is not validated HOME."
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=("mobile", "desk"), required=True)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--grippers", choices=("stock","both"), default="both")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--render", type=Path)
    args = parser.parse_args()
    model = build_scene(args.scene, grippers=args.grippers).compile()
    data = mujoco.MjData(model)
    data.ctrl[:] = actuator_targets_from_qpos(model, data.qpos)
    report = inspect(model, data, args.scene)
    report["grippers"] = args.grippers
    report["frame"] = "desk top z=0" if args.scene == "desk" else "TurtleBot floor frame; CAD centre reused"
    if args.scene == "desk":
        report["layout_m"] = {"historical_desk_arm_offset": .15, "block_size": .04}
        report["layout_provenance"] = "Same relative arm/stand XY layout as mobile; supersedes camera20mm ahead of base requirement"
        report["note"] += (" Existing NORMA PGripper + wrist RGB stand retained."
                           if args.grippers == "both" else " Stock tabletop finger pad proxies retained.")
    assert model.nu == 12 and data.time == 0 and np.all(data.qvel == 0)
    assert report["ctrl_qpos_max_error"] == 0
    assert np.isclose(FRAME_DEPTH - 2 * PROFILE, .072)
    assert np.isclose(ARM_MOUNT_X + REAR_HOLE_X, -.046)
    anchor_x = 0 if args.scene == "desk" else data.body("aluminum_mount_UNVERIFIED").xpos[0]
    for side in ("left", "right"):
        assert np.isclose(data.body(side + "_base").xpos[0] - anchor_x, ARM_MOUNT_X)
    plate = data.body("os30a_plate_frame")
    installation_z = 0 if args.scene == "desk" else data.body("aluminum_mount_UNVERIFIED").xpos[2] + PROFILE
    assert np.isclose(plate.xpos[2] - installation_z, .38)
    assert np.isclose(np.rad2deg(np.arccos(plate.xmat.reshape(3, 3)[2, 2])), 25)
    assert model.geom("stand_cad_top_3").contype == 1
    if args.scene == "desk":
        assert np.isclose(data.body("os30a_plate_frame").xpos[0], .0226408355)
    assert np.isclose(data.body("left_base").xpos[1] - data.body("right_base").xpos[1], .300)
    output = json.dumps(report, indent=2)
    print(output)
    if args.report:
        args.report.write_text(output + "\n")
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.12, 0, .15]
    camera.distance, camera.azimuth, camera.elevation = 1.35, 135, -25
    if args.render:
        from PIL import Image
        with mujoco.Renderer(model, height=480, width=640) as renderer:
            renderer.update_scene(data, camera=camera)
            Image.fromarray(renderer.render()).save(args.render)
    if args.viewer:
        from mujoco.viewer import launch_passive
        with launch_passive(model, data) as viewer:
            viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME] = 0
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance = camera.distance
            viewer.cam.azimuth = camera.azimuth
            viewer.cam.elevation = camera.elevation
            while viewer.is_running():
                # Geometry review only: deliberately no mj_step/controller path.
                viewer.sync()
                time.sleep(.03)


if __name__ == "__main__":
    main()
