#!/usr/bin/env python3
"""Offline LeRobot-v3 episode replay. No robot, serial, ROS or training imports.

The measured-state panel is kinematic; only the second panel uses mj_step.
Neither is a calibrated reconstruction of the photographed scene.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from mobile_dual_so101 import ACTION_NAMES, apply_control_as_pose, model_names
from sim_policy import policy_action_to_actuator_targets

ARM = np.array([0, 1, 2, 3, 4, 6, 7, 8, 9, 10])
GRIPPER = np.array([5, 11])
FEATURE_NAMES = [f"{name}.pos" for name in ACTION_NAMES]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_vector(value, size, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {size} finite values")
    return result


def mapping(profile):
    signs = finite_vector(profile["arm_signs"], 10, "arm_signs")
    offsets = finite_vector(profile["arm_zero_offsets_deg"], 10, "arm_zero_offsets_deg")
    if not np.isin(signs, [-1, 1]).all():
        raise ValueError("arm_signs must be -1 or +1")
    return signs, offsets


def recorded_to_sim(model, values, profile):
    values = finite_vector(values, 12, "recorded positions").copy()
    signs, offsets = mapping(profile)
    values[ARM] = np.deg2rad(values[ARM] * signs + offsets)
    values[GRIPPER] /= 100.0
    # Stored action is already the actual sent follower command. Never add
    # leader-to-follower shoulder-pan alignment again at the simulation boundary.
    return np.asarray(policy_action_to_actuator_targets(model, values))


def sim_to_recorded(model, values, profile):
    # Measured qpos may exceed MuJoCo's soft limits. Decode it faithfully;
    # command limits belong to recorded_to_sim, not the observation boundary.
    policy = finite_vector(values, 12, "measured simulator positions").copy()
    ranges = np.asarray(model.actuator_ctrlrange)[GRIPPER]
    widths = ranges[:, 1] - ranges[:, 0]
    if not np.isfinite(ranges).all() or np.any(widths <= 0):
        raise ValueError("invalid gripper ranges")
    policy[GRIPPER] = (policy[GRIPPER] - ranges[:, 0]) / widths
    signs, offsets = mapping(profile)
    policy[ARM] = (np.rad2deg(policy[ARM]) - offsets) / signs
    policy[GRIPPER] *= 100.0
    return policy


def load_episode(root, episode):
    if episode < 0:
        raise ValueError("episode must be non-negative")
    info = json.loads((root / "meta/info.json").read_text())
    if info["robot_type"] != "bi_so_follower" or info["codebase_version"] != "v3.0":
        raise ValueError("expected a LeRobot-v3 bi_so_follower dataset")
    for key in ("action", "observation.state"):
        if info["features"][key]["names"] != FEATURE_NAMES:
            raise ValueError(f"unexpected joint order for {key}")
    fps = float(info["fps"])
    if not np.isfinite(fps) or not 1 <= fps <= 240:
        raise ValueError("invalid recording fps")
    columns = ["episode_index", "frame_index", "timestamp", "observation.state", "action"]
    paths = sorted((root / "data").rglob("*.parquet"))
    if not paths:
        raise ValueError("dataset has no parquet files")
    table = pa.concat_tables([pq.read_table(p, columns=columns) for p in paths])
    table = table.filter(pa.compute.equal(table["episode_index"], episode)).sort_by("frame_index")
    if len(table) < 2:
        raise ValueError("episode must have at least two frames")
    frames = np.array(table["frame_index"].to_pylist())
    timestamps = np.array(table["timestamp"].to_pylist())
    if not np.array_equal(frames, np.arange(len(table))):
        raise ValueError("episode has missing or duplicate frame indices")
    if not np.isfinite(timestamps).all() or not np.allclose(timestamps, frames / fps, atol=1e-4, rtol=0):
        raise ValueError("episode timestamps do not follow the declared fps")
    state = np.asarray(table["observation.state"].to_pylist(), dtype=float)
    action = np.asarray(table["action"].to_pylist(), dtype=float)
    if state.shape != (len(table), 12) or action.shape != state.shape:
        raise ValueError("expected finite Nx12 state/action arrays")
    if not np.isfinite(state).all() or not np.isfinite(action).all():
        raise ValueError("non-finite state/action values")
    alignment = json.loads((root / "meta/dapier_base_alignment.json").read_text())
    if alignment.get("stored_action") != "actual sent command in configured follower units":
        raise ValueError("actual-sent-action provenance is missing")
    return state, action, fps, alignment


def build_tabletop(model_path, profile):
    if profile.get("schema_version") != 2:
        raise ValueError("unknown scene profile")
    from lerobot.envs.so101_mujoco.camera_profiles import load_camera_profile, WRIST_CAMERA_PROFILE_ID
    from lerobot.envs.so101_mujoco.env import (
        _FINGER_PAD_SPECS, _apply_camera_profile, FINGER_PAD_CUBE_CONTACT_SOLREF,
    )
    mapping(profile)
    distance = float(profile["camera_to_base_horizontal_m"])
    size = finite_vector(profile["table_size_m"], 3, "table size")
    height = float(profile["camera_height_above_table_m"])
    front = float(profile["table_front_edge_x_m"])
    tilt = float(profile["camera_down_tilt_deg"])
    fov = float(profile["camera_vertical_fov_deg"])
    yaws = finite_vector(profile["arm_yaw_deg"], 2, "mount yaw")
    block_size = finite_vector(profile["reference_block_size_m"], 3, "block size")
    block_center = finite_vector(profile["reference_block_center_m"], 3, "block center")
    block_mass = float(profile["block_mass_kg"])
    block_friction = finite_vector(profile["block_friction"], 3, "block friction")
    contact_solref = finite_vector(profile.get("contact_solref", FINGER_PAD_CUBE_CONTACT_SOLREF), 2, "contact solref")
    if np.any(contact_solref >= 0):
        raise ValueError("expected negative direct-format contact stiffness/damping")
    if not np.isfinite(block_mass) or block_mass <= 0 or np.any(block_friction < 0):
        raise ValueError("block requires positive mass and non-negative friction")
    if not np.isfinite([distance, height, front, tilt, fov]).all():
        raise ValueError("scene dimensions must be finite")
    if distance <= 0 or height <= 0 or np.any(size <= 0) or np.any(block_size <= 0):
        raise ValueError("scene dimensions must be positive")
    if not 0 < tilt <= 90 or not 0 < fov < 180:
        raise ValueError("invalid camera tilt/FOV")
    spec = mujoco.MjSpec()
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    cones = {"pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL, "elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC}
    cone = profile.get("friction_cone", "pyramidal")
    tolerance = float(profile.get("solver_tolerance", spec.option.tolerance))
    if cone not in cones or not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("invalid friction cone or solver tolerance")
    spec.option.cone = cones[cone]
    spec.option.tolerance = tolerance
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.28, 0.32, 0.38], rgb2=[0.65, 0.68, 0.70], width=256, height=1536)
    spec.worldbody.add_light(pos=[0.3, 0, 1.5], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
    spec.worldbody.add_light(pos=[-0.5, 0, 1], dir=[0.5, 0, -1], diffuse=[0.4, 0.4, 0.4])
    spec.worldbody.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX,
                            size=size / 2, pos=[front + size[0] / 2, 0, -size[2] / 2],
                            rgba=[0.91, 0.91, 0.89, 1])
    for side, y, yaw in zip(("left", "right"), (distance, -distance), yaws):
        arm = mujoco.MjSpec.from_file(str(model_path))
        # Reuse the existing CAD-aligned fingertip proxies. A convex hull of
        # the hollow finger mesh otherwise collides in visibly empty space.
        for pad in _FINGER_PAD_SPECS:
            body = arm.body(pad["body"])
            for geom in body.geoms:
                # Only replace hollow fingers. Preserve the motor housing's
                # original collision even when it shares the gripper body.
                if (geom.type == mujoco.mjtGeom.mjGEOM_MESH and geom.contype
                        and geom.meshname in ("wrist_roll_follower_so101_v1", "moving_jaw_so101_v1")):
                    geom.contype = 0
                    geom.conaffinity = 0
            body.add_geom(name=pad["name"], type=mujoco.mjtGeom.mjGEOM_BOX,
                          pos=pad["pos"], quat=pad["quat"], size=pad["size"],
                          contype=1, conaffinity=1, condim=4, friction=block_friction,
                          solref=contact_solref, solimp=[0.95, 0.99, 0.001, 0.5, 2.0],
                          rgba=[0.07, 0.07, 0.07, 1], density=0)
        wrist = load_camera_profile(WRIST_CAMERA_PROFILE_ID)
        fixed = _FINGER_PAD_SPECS[0]
        # A target at the cube center when its face meets the fixed pad.
        # This site is geometry only; it cannot constrain or attach the object.
        pinch = np.asarray(fixed["pos"]) + np.array([fixed["size"][2] + block_size[0] / 2, 0, 0])
        arm.body("gripper").add_site(name="cube_grasp", pos=pinch,
            quat=arm.site("gripperframe").quat, size=[.002, .002, .002], group=3)
        _apply_camera_profile(arm.body(wrist.parent_body).add_camera(name="wrist_rgb"), wrist, mujoco)
        for material in arm.materials:
            if material.rgba[0] > 0.8 and material.rgba[1] > 0.7 and material.rgba[2] < 0.3:
                material.rgba = [0.88, 0.89, 0.88, 1]
        angle = np.deg2rad(yaw) / 2
        frame = spec.worldbody.add_frame(name=f"{side}_mount", pos=[0, y, 0],
                                         quat=[np.cos(angle), 0, 0, np.sin(angle)])
        spec.attach(arm, prefix=f"{side}_", frame=frame)
    # ponytail: camera optics and physical parameters remain estimates;
    # replace them with measured profiles before claiming sim-to-real validity.
    spec.worldbody.add_geom(name="camera_mast", type=mujoco.mjtGeom.mjGEOM_BOX,
                            size=[0.012, 0.02, height / 2], pos=[front, 0, height / 2],
                            rgba=[0.08, 0.08, 0.08, 1])
    spec.worldbody.add_geom(name="camera_body", type=mujoco.mjtGeom.mjGEOM_BOX,
                            size=[0.013, 0.045, 0.013], pos=[0, 0, height],
                            rgba=[0.12, 0.12, 0.12, 1], contype=0, conaffinity=0)
    if front != 0:
        spec.worldbody.add_geom(name="camera_support", type=mujoco.mjtGeom.mjGEOM_BOX,
                                size=[abs(front) / 2, 0.015, 0.01], pos=[front / 2, 0, height],
                                rgba=[0.08, 0.08, 0.08, 1])
    theta = np.deg2rad(90 - tilt) / 2
    spec.worldbody.add_camera(name="top_h201_reference", pos=[0, 0, height],
                              quat=[np.cos(theta), 0, -np.sin(theta), 0], fovy=fov)
    block = spec.worldbody.add_body(name="red_block", pos=block_center)
    block.add_freejoint(name="red_block_free")
    block.add_geom(name="red_block_geom", type=mujoco.mjtGeom.mjGEOM_BOX,
                   size=block_size / 2, mass=block_mass, rgba=[0.65, 0.03, 0.025, 1],
                   contype=1, conaffinity=1, condim=4, friction=block_friction,
                   solref=contact_solref, solimp=[0.95, 0.99, 0.001, 0.5, 2.0])
    model = spec.compile()
    if model_names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu) != ACTION_NAMES:
        raise ValueError("MJCF actuator order mismatch")
    return model


def physics_settings(model):
    return {"mujoco_version": mujoco.__version__, "timestep_s": float(model.opt.timestep),
            "friction_cone": mujoco.mjtCone(model.opt.cone).name,
            "solver": mujoco.mjtSolver(model.opt.solver).name,
            "tolerance": float(model.opt.tolerance), "impratio": float(model.opt.impratio),
            "noslip_iterations": int(model.opt.noslip_iterations)}


def block_contacts(model, data):
    """Contact evidence only: never used to attach or teleport the block."""
    block = model.geom("red_block_geom").id
    touched = set()
    for i, contact in enumerate(data.contact):
        pair = {int(contact.geom1), int(contact.geom2)}
        if block in pair:
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, force)
            if force[0] > 0:
                touched.update(model.geom(g).name for g in pair if g != block)
    return {side: all(f"{side}_dapier_{finger}_finger_pad" in touched for finger in ("fixed", "moving"))
            for side in ("left", "right")}


def inspect_contacts(model, data, deepest, frame):
    for contact in data.contact:
        if -float(contact.dist) > deepest["depth_m"]:
            geoms = [int(contact.geom1), int(contact.geom2)]
            deepest.update(depth_m=-float(contact.dist), frame=frame, time_s=float(data.time),
                           bodies=[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                   int(model.geom_bodyid[g])) for g in geoms],
                           geoms=[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) for g in geoms])


def run(args):
    root = args.dataset.resolve()
    output = args.output.resolve()
    # Never write into a dataset, even when a caller supplies an unsafe output.
    if output == root or root in output.parents or output in root.parents:
        raise ValueError("output must be separate from the source dataset")
    profile = json.loads(args.scene.read_text())
    protected = [p for p in root.rglob("*") if p.is_file()]
    protected += list(args.calibration_dir.glob("*.json"))
    if not list(args.calibration_dir.glob("*.json")):
        raise ValueError("no calibration JSON files found for preservation check")
    protected += [args.model, args.scene]
    before = {str(p): sha256(p) for p in protected}
    state, action, fps, alignment = load_episode(root, args.episode)
    model = build_tabletop(args.model, profile)
    state_q = np.array([recorded_to_sim(model, row, profile) for row in state])
    action_q = np.array([recorded_to_sim(model, row, profile) for row in action])
    roundtrip = max(float(np.max(np.abs(sim_to_recorded(model, q, profile) - v)))
                    for q, v in zip(state_q, state))
    substeps = max(1, int(np.ceil(1 / fps / 0.002)))
    model.opt.timestep = 1 / (fps * substeps)
    observed, physics = mujoco.MjData(model), mujoco.MjData(model)
    apply_control_as_pose(model, physics, state_q[0])
    addresses = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    output.mkdir(parents=True, exist_ok=False)
    (output / "scene.json").write_text(json.dumps(profile, indent=2) + "\n")
    renderer = mujoco.Renderer(model, height=360, width=480)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.13, 0, 0.22]
    camera.distance, camera.azimuth, camera.elevation = 1.25, 140, -35
    video = subprocess.Popen([
        "ffmpeg", "-v", "error", "-n", "-f", "rawvideo", "-pixel_format", "rgb24",
        "-video_size", "960x400", "-framerate", str(fps), "-i", "pipe:0", "-an",
        "-c:v", "libx264", "-preset", "fast", "-crf", "23", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(output / "replay.mp4")], stdin=subprocess.PIPE)
    import cv2
    simulated = []
    block_positions = []
    block_contact_frames = {"left": 0, "right": 0}
    deepest_contact = {"depth_m": 0.0}
    warning_counts = np.zeros(len(physics.warning), dtype=int)
    try:
        for i, (measured, target) in enumerate(zip(state_q, action_q)):
            apply_control_as_pose(model, observed, measured)
            simulated.append(physics.qpos[addresses].copy())
            block_positions.append(physics.body("red_block").xpos.copy())
            for side, touching in block_contacts(model, physics).items():
                block_contact_frames[side] += int(touching)
            inspect_contacts(model, physics, deepest_contact, i)
            panels = []
            for data in (observed, physics):
                renderer.update_scene(data, camera=camera)
                panels.append(renderer.render().copy())
            frame = np.zeros((400, 960, 3), dtype=np.uint8)
            frame[40:] = np.concatenate(panels, axis=1)
            for x, title in [(8, "JOINT REPLAY; BLOCK AT INITIAL POSE"), (488, "SENT ACTION + FREE BLOCK (physics)")]:
                cv2.putText(frame, title, (x, 17), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
            spacing_mm = 2000 * profile["camera_to_base_horizontal_m"]
            cv2.putText(frame, f"SIM ONLY | {i / fps:.2f}s | base spacing {spacing_mm:g}mm | scene/joint zero NOT calibrated", (8, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 205, 120), 1)
            video.stdin.write(frame.tobytes())
            if i in {0, len(state) // 2, len(state) - 1}:
                if not cv2.imwrite(str(output / f"frame-{i:04d}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError("snapshot write failed")
            physics.ctrl[:] = target
            for _ in range(substeps):
                mujoco.mj_step(model, physics)
                inspect_contacts(model, physics, deepest_contact, i)
            if not np.isfinite(physics.qpos).all() or not np.isfinite(physics.qvel).all():
                raise RuntimeError(f"non-finite physics at frame {i}")
            warning_counts = np.maximum(warning_counts, [w.number for w in physics.warning])
    finally:
        video.stdin.close()
        video_code = video.wait()
        renderer.close()
    if video_code:
        raise RuntimeError(f"video encoder failed: {video_code}")
    simulated = np.asarray(simulated)
    error = simulated - state_q
    np.savez_compressed(output / "joint_trace.npz", recorded_state_rad=state_q,
                        sent_action_rad=action_q, simulated_state_rad=simulated,
                        simulated_block_position_m=np.asarray(block_positions),
                        timestamp_s=np.arange(len(state)) / fps)
    preserved = all(sha256(Path(path)) == digest for path, digest in before.items())
    report = {
        "schema_version": 1, "hardware_execution": False, "learning_executed": False,
        "episode_index": args.episode, "frames": len(state), "fps": fps,
        "last_observation_time_s": (len(state) - 1) / fps,
        "simulation_duration_s": float(physics.time), "substeps_per_frame": substeps,
        "timestamp_basis": "dataset frame_index/fps, not measured device capture latency",
        "joint_names": ACTION_NAMES, "source_arm_units": "degrees",
        "source_gripper_units": "percent_0_to_100", "model_units": "all actuator joints in radians",
        "alignment_metadata": alignment, "leader_alignment_reapplied": False,
        "roundtrip_max_error_source_units": roundtrip,
        "input_range_check_passed": True, "source_and_calibration_hashes_unchanged": preserved,
        "source_hashes": before, "scene": profile, "physics_settings": physics_settings(model),
        "arm_rmse_deg": dict(zip([ACTION_NAMES[i] for i in ARM], np.rad2deg(np.sqrt(np.mean(error[:, ARM] ** 2, axis=0))).tolist())),
        "gripper_rmse_percent": (np.sqrt(np.mean(error[:, GRIPPER] ** 2, axis=0)) /
                                  np.diff(model.actuator_ctrlrange[GRIPPER], axis=1).ravel() * 100).tolist(),
        "max_contact_penetration_m": deepest_contact["depth_m"],
        "block_dynamics": {"free_joint": True, "mass_kg": profile["block_mass_kg"],
                           "size_m": profile["reference_block_size_m"],
                           "bilateral_contact_frames": block_contact_frames,
                           "max_center_height_m": float(np.max(np.asarray(block_positions)[:, 2])),
                           "final_position_m": physics.body("red_block").xpos.tolist(),
                           "attachment_or_pose_updates": False,
                           "grasp_success_verified": False},
        "deepest_contact": deepest_contact,
        "mujoco_warning_counts": warning_counts.tolist(),
        "physical_mapping_verified": profile["joint_mapping_physically_verified"],
        "ready_for_policy_evaluation": False,
        "limitations": ["No measured MJCF neutral/sign/gripper correspondence",
                        "Camera parameters and desk size are estimates; cube side is user-confirmed 4 cm",
                        "Object moves only under physics; initial XY/mass/friction remain estimates",
                        "Wrist camera CAD profile is not physically calibrated; no policy is executing",
                        "Original MuJoCo motor parameters, no system identification"],
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    if not preserved or warning_counts.any():
        raise RuntimeError("preservation or simulation warning gate failed; inspect report.json")
    print(json.dumps({k: report[k] for k in ["frames", "fps", "simulation_duration_s",
                     "source_and_calibration_hashes_unchanged", "roundtrip_max_error_source_units",
                     "arm_rmse_deg", "max_contact_penetration_m", "ready_for_policy_evaluation"]}, indent=2))
    print(output)


def self_test():
    from types import SimpleNamespace
    model = SimpleNamespace(actuator_ctrlrange=np.tile([-np.pi, np.pi], (12, 1)))
    profile = {"arm_signs": [-1] + [1] * 9, "arm_zero_offsets_deg": [7] + [0] * 9}
    row = np.array([30, -90, 45, 10, -20, 0, -30, 90, -45, 20, 0, 100.])
    q = recorded_to_sim(model, row, profile)
    assert np.isclose(q[0], np.deg2rad(-23))
    assert np.allclose(q[GRIPPER], [-np.pi, np.pi])
    assert np.allclose(sim_to_recorded(model, q, profile), row)
    overshoot = q.copy()
    overshoot[11] += .001
    assert sim_to_recorded(model, overshoot, profile)[11] > 100
    for index, bad in [(0, np.nan), (0, 1000), (5, -1), (11, 101)]:
        invalid = row.copy()
        invalid[index] = bad
        try:
            recorded_to_sim(model, invalid, profile)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid input accepted")
    assert np.isclose(15 * 34 * (1 / (15 * 34)), 1)
    print("PASS: units/sign/zero/gripper endpoints, roundtrip, invalid inputs, 15Hz timing")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--dataset", type=Path)
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--scene", type=Path, default=Path(__file__).with_name("tabletop_replay.json"))
    parser.add_argument("--calibration-dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        if any(getattr(args, key) is None for key in ("dataset", "model", "calibration_dir", "output")):
            parser.error("--dataset, --model, --calibration-dir, --output are required")
        run(args)


if __name__ == "__main__":
    main()
