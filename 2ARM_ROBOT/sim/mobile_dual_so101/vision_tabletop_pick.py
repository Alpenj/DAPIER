#!/usr/bin/env python3
"""SIM-only depth + IK + virtual-contact pickup baseline, NOT a trained ACT policy."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from mobile_dual_so101 import apply_control_as_pose
from physics_ik import solve_bimanual_position_ik, plan_septic_joint_trajectory
from replay_recorded_episode import build_tabletop, recorded_to_sim, block_contacts, sha256, inspect_contacts, physics_settings
from vision_box_pregrasp import (
    calibration_from_mujoco, estimate_box_pose_from_depth, DepthBoxDetectorConfig,
)


def pad_forces(model, data):
    """Virtual fingertip force sensors; no object pose/identity is read here."""
    ids = [model.geom(f"left_dapier_{finger}_finger_pad").id for finger in ("fixed", "moving")]
    forces = np.zeros(2)
    for index, contact in enumerate(data.contact):
        if any(g in (contact.geom1, contact.geom2) for g in ids):
            wrench = np.zeros(6)
            mujoco.mj_contactForce(model, data, index, wrench)
            for i, g in enumerate(ids):
                if g in (contact.geom1, contact.geom2):
                    forces[i] += max(0, wrench[0])
    return forces


def run(args):
    from PIL import Image
    if not np.isfinite(args.entry_clearance) or not 0 <= args.entry_clearance <= .02:
        raise ValueError("entry clearance must be finite and within 0..20mm")
    if not np.isfinite(args.grasp_force) or not .05 <= args.grasp_force <= 5:
        raise ValueError("virtual grasp force must be finite and within 0.05..5N")
    profile = json.loads(args.scene.read_text())
    if not np.allclose(profile["reference_block_size_m"], .04) or profile["block_mass_kg"] != .02:
        raise ValueError("this baseline requires the user's 4cm, approximately 20g cube")
    sources = (args.model, args.scene, Path(__file__), Path(__file__).with_name("replay_recorded_episode.py"))
    before = {str(p): sha256(p) for p in sources}
    output = args.output.resolve()
    if output.exists():
        raise ValueError("use a new output directory")
    model = build_tabletop(args.model, profile)
    if args.disable_finger_contact:
        for side in ("left", "right"):
            for finger in ("fixed", "moving"):
                geom = model.geom(f"{side}_dapier_{finger}_finger_pad")
                geom.contype = geom.conaffinity = 0
    assert not any(model.eq_type == mujoco.mjtEq.mjEQ_WELD)
    assert model.body("red_block").mocapid == -1
    data = mujoco.MjData(model)
    home = np.tile([0, -35, 55, 35, 0, 100.], 2)
    home[4] = np.rad2deg(.05)
    command = recorded_to_sim(model, home, profile)
    command[5] = .50  # Model joint radians, not normalized gripper opening.
    apply_control_as_pose(model, data, command)  # Only runtime qpos initialization.
    model.opt.timestep = 1 / (15 * 34)
    renderer = mujoco.Renderer(model, width=640, height=460)
    detector = DepthBoxDetectorConfig(workspace_x_m=(.08, .35), workspace_y_m=(-.10, .10),
        top_height_m=(.035, .045), box_size_m=(.04, .04, .04), lid_side_flap_m=.001,
        lid_front_flap_m=.001, dimension_tolerance_m=.01, minimum_points=80, pixel_stride=1)
    camera = calibration_from_mujoco(model, data, "top_h201_reference", width=640, height=460)
    output.mkdir(parents=True, exist_ok=False)
    (output / "scene.json").write_text(json.dumps(profile, indent=2) + "\n")
    viewer, trace, detections, plans = None, [], [], []
    failure, phase = None, "settle"
    deepest = {"depth_m": 0.0}
    phase_peaks = {}
    block_dof = model.joint("red_block_free").dofadr[0]
    if args.viewer:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
        viewer.cam.lookat[:] = [.14, 0, .11]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = .95, 135, -35

    def tick(target):
        started = time.perf_counter()
        if viewer is not None and not viewer.is_running():
            raise ValueError("viewer closed by user")
        target = np.asarray(target)
        if target.shape != (12,) or not np.isfinite(target).all():
            raise ValueError("invalid joint command")
        if np.any(target < model.actuator_ctrlrange[:, 0]) or np.any(target > model.actuator_ctrlrange[:, 1]):
            raise ValueError("joint command outside range")
        data.ctrl[:] = target
        for _ in range(34):
            mujoco.mj_step(model, data)
            inspect_contacts(model, data, deepest, len(trace))
            peak = phase_peaks.setdefault(phase, {"pad_normal_force_N": 0.0, "block_speed_m_s": 0.0})
            peak["pad_normal_force_N"] = max(peak["pad_normal_force_N"], float(pad_forces(model, data).max()))
            peak["block_speed_m_s"] = max(peak["block_speed_m_s"], float(np.linalg.norm(data.qvel[block_dof:block_dof + 3])))
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
            raise ValueError("invalid physics or MuJoCo warning")
        # Truth is confined to the verifier log, never the camera/IK target.
        trace.append({"time_s": float(data.time), "phase": phase,
            "ctrl_rad": target.tolist(), "qpos": data.qpos.tolist(),
            "qvel": data.qvel.tolist(),
            "virtual_pad_forces_N": pad_forces(model, data).tolist(),
            "verifier_block_position_m": data.body("red_block").xpos.tolist(),
            "verifier_bilateral_contact": block_contacts(model, data)["left"]})
        if viewer is not None:
            viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                "DEPTH + IK + CONTACT | NOT ACT | SIM ONLY", f"{phase} | {data.time:.2f}s | 4cm / 20g")])
            viewer.sync()
            time.sleep(max(0, 1 / 15 - (time.perf_counter() - started)))

    def observe():
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera="top_h201_reference")
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
        i = len(detections)
        np.save(output / f"depth-{i:02d}.npy", depth)
        renderer.update_scene(data, camera="top_h201_reference")
        Image.fromarray(renderer.render()).save(output / f"top-{i:02d}.png")
        estimate = estimate_box_pose_from_depth(depth, camera, detector)
        detections.append(asdict(estimate))
        return np.asarray(estimate.top_center_base_m) - [0, 0, .02]

    def move(goal, minimum_duration_s=.50):
        nonlocal command
        trajectory = plan_septic_joint_trajectory(model, command, goal, minimum_duration_s=minimum_duration_s)
        plans.append({"phase": phase, "duration_s": trajectory.duration_s, "target_rad": list(goal)})
        for i in range(int(np.ceil(trajectory.duration_s * 15))):
            target, _, _, _ = trajectory.sample((i + 1) / 15)
            tick(target)
        command = np.asarray(goal).copy()
        for _ in range(8):
            tick(command)

    def target_at(center):
        result = solve_bimanual_position_ik(model, command, {"left": center},
            site_names={"left": "left_cube_grasp"}, tool_axis_targets={"left": [0, 0, -1]},
            max_iterations=300)
        if not result.converged:
            raise ValueError(f"IK did not converge: {result.residual_m_by_side}")
        return np.asarray(result.action_rad)

    try:
        for _ in range(15):
            tick(command)
        center = observe()
        phase = "vision approach"
        move(target_at(center + [0, .06, .06]))
        # Look beside the cube before the gripper occludes the top view.
        center = observe()
        # ponytail: this baseline grasps the axis-aligned cube along world X;
        # derive this direction from a verified object/grasp frame for rotated objects.
        entry = center - [args.entry_clearance, 0, 0]
        phase = "align above last observed cube"
        move(target_at(entry + [0, 0, .06]))
        phase = "descent with lateral finger clearance"
        move(target_at(entry + [0, 0, .015]), minimum_duration_s=2.5)
        phase = "slow final 15mm approach"
        move(target_at(entry), minimum_duration_s=2.0)
        phase = "slow lateral seating at cube center height"
        move(target_at(center), minimum_duration_s=2.0)
        phase = "slow gripper preshape"
        goal = command.copy()
        goal[5] = .40
        move(goal, minimum_duration_s=1.0)
        phase = "close until virtual bilateral force"
        contacted = False
        stable_contacts = 0
        for _ in range(270):
            if np.any(pad_forces(model, data) < args.grasp_force):
                command[5] = max(model.actuator_ctrlrange[5, 0], command[5] - .00025)
            tick(command)
            stable_contacts = stable_contacts + 1 if np.all(pad_forces(model, data) >= args.grasp_force) else 0
            if stable_contacts >= 5:
                contacted = True
                break
        if not contacted:
            raise ValueError("bilateral fingertip force not established; no lift commanded")
        phase = "grasp settle"
        for _ in range(15):
            tick(command)
        phase = "lift"
        # Lift target derives from the last image estimate, not cube ground truth.
        move(target_at(center + [0, 0, .05]), minimum_duration_s=3.0)
        phase = "hold"
        for _ in range(45):
            tick(command)
    except ValueError as error:
        failure = {"phase": phase, "reason": str(error)}
    finally:
        renderer.close()
    held = [row for row in trace if row["phase"] == "hold"]
    success = (failure is None and len(held) >= 30
        and deepest["depth_m"] <= .001
        and all(row["verifier_block_position_m"][2] > .05 and row["verifier_bilateral_contact"] for row in held))
    report = {"kind": "vision+IK+virtual-contact scripted physics baseline", "hardware_execution": False,
        "learned_act_success": False, "sim_to_real_valid": False,
        "cube_side_m": .04, "cube_mass_kg": float(model.body("red_block").mass[0]),
        "gravity_m_s2": model.opt.gravity.tolist(), "finger_contact_disabled": args.disable_finger_contact,
        "table_pick_and_3s_hold_success": success, "failure": failure,
        "deepest_contact": deepest,
        "phase_peak_metrics": phase_peaks,
        "max_cube_center_height_m": max(row["verifier_block_position_m"][2] for row in trace),
        "final_cube_position_m": data.body("red_block").xpos.tolist(), "simulation_seconds": float(data.time),
        "runtime_qpos_writes_after_initialization": 0, "object_pose_writes_or_attachments": False,
        "target_source": "metric depth images + virtual camera calibration + known 4cm cube dimensions",
        "object_ground_truth_used_for_control": False, "virtual_contact_sensors_used_for_closure": True,
        "virtual_contact_sensors_available_on_real_robot": False,
        "detections": detections, "plans": plans,
        "mujoco_warning_counts": [int(w.number) for w in data.warning],
        "source_hashes": before, "physics_settings": physics_settings(model),
        "source_hashes_unchanged": before == {str(p): sha256(p) for p in sources},
        "entry_clearance_m": args.entry_clearance,
        "virtual_grasp_force_target_N": args.grasp_force,
        "limitations": ["Camera/joint mapping and friction are not physically calibrated.",
            "Finger shafts are not fully represented by collision proxies; no collision-safety claim.",
            "A successful lift here is not ACT learning, handover, or real-hardware validation."]}
    (output / "trace.json").write_text(json.dumps(trace, indent=2) + "\n")
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    if viewer is not None:
        viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
            "SIM PICK PASS" if success else "SIM PICK FAILED", "NOT ACT / NOT REAL VALIDATION")])
        while viewer.is_running():
            viewer.sync()
            time.sleep(.05)
        viewer.close()
    return 0 if success else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--scene", type=Path, default=Path(__file__).with_name("tabletop_replay.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--entry-clearance", type=float, default=.003,
                        help="SIM-only lateral entry clearance in meters for this axis-aligned cube")
    parser.add_argument("--grasp-force", type=float, default=1.0,
                        help="SIM-only normal force on each pad, required for five consecutive 15Hz ticks")
    parser.add_argument("--disable-finger-contact", action="store_true", help="negative control: pickup must fail")
    raise SystemExit(run(parser.parse_args()))
