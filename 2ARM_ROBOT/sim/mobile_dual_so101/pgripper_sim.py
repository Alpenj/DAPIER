#!/usr/bin/env python3
"""Current right-only PGripper tabletop scene; offline SIM only, no dataset replay."""
import argparse
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from mobile_dual_so101 import apply_control_as_pose, resolve_so101_model, model_provenance
from pgripper import ASSETS, MOTOR_MAX_RAD, selected_sides, home_action, jaw_gap_m
from replay_recorded_episode import build_tabletop_spec, physics_settings


def run(args):
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError("use a new output directory; stock results are never overwritten")
    source = resolve_so101_model(args.model)
    if output == source.parent or output in source.parents:
        raise ValueError("output must be separate from source model files")
    profile = json.loads(Path(__file__).with_name("tabletop_replay.json").read_text())
    spec = build_tabletop_spec(source, profile, grippers=args.grippers)
    model = spec.compile()
    data = mujoco.MjData(model)
    initial = np.tile(np.deg2rad([0, -35, 55, 35, 0, 30]), 2)
    initial = np.asarray(home_action(model, initial))
    apply_control_as_pose(model, data, initial)
    initial_qpos = data.qpos.copy()
    # One initialization only. All subsequent motion uses controls + the physics solver.
    sides = selected_sides(args.grippers)
    trace, warnings = [], np.zeros(len(data.warning), dtype=int)
    for fraction in np.concatenate((np.linspace(1, 0, 750), np.linspace(0, 1, 750))):
        for side in sides:
            data.ctrl[5 if side == "left" else 11] = MOTOR_MAX_RAD * fraction
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError("non-finite PGripper simulation")
        warnings = np.maximum(warnings, [w.number for w in data.warning])
        if len(trace) == 0 or data.time >= trace[-1]["time_s"] + .02:
            trace.append({"time_s": float(data.time),
                          "gap_m": {side: jaw_gap_m(model, data, side) for side in sides}})
    if warnings.any():
        raise RuntimeError(f"MuJoCo warning gate failed: {warnings.tolist()}")
    output.mkdir(parents=True, exist_ok=False)
    xml = output / f"dual-so101-pgripper-{args.grippers}.xml"
    spec.add_key(name="home", qpos=initial_qpos, ctrl=initial)
    spec.compile()
    spec.to_file(str(xml))
    reloaded = mujoco.MjModel.from_xml_path(str(xml))
    if (reloaded.nq, reloaded.nu, reloaded.neq) != (model.nq, model.nu, model.neq):
        raise RuntimeError("saved MJCF reload contract changed")
    # Separate pose-only pictures avoid presenting kinematic endpoints as dynamic grasp evidence.
    from PIL import Image
    pose = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, width=640, height=480)
    camera = mujoco.MjvCamera()
    endpoint_gaps = {}
    try:
        for label, fraction in (("open", 1), ("closed", 0)):
            action = initial.copy()
            for side in sides:
                action[5 if side == "left" else 11] = MOTOR_MAX_RAD * fraction
            apply_control_as_pose(model, pose, action)
            endpoint_gaps[label] = {side: jaw_gap_m(model, pose, side) for side in sides}
            camera.lookat[:] = [.12, 0, .15]
            camera.distance, camera.azimuth, camera.elevation = .9, 135, -30
            renderer.update_scene(pose, camera=camera)
            Image.fromarray(renderer.render()).save(output / f"tabletop-{label}.png")
            if sides:
                camera.lookat[:] = (pose.site(f"{sides[-1]}_cube_grasp").xpos
                                    + pose.body(f"{sides[-1]}_gripper").xpos) / 2
                camera.distance, camera.azimuth, camera.elevation = .34, 140, -25
                renderer.update_scene(pose, camera=camera)
                Image.fromarray(renderer.render()).save(output / f"pgripper-{label}.png")
                for side in sides:
                    renderer.update_scene(pose, camera=f"{side}_wrist_rgb")
                    Image.fromarray(renderer.render()).save(output / f"{side}-wrist-{label}.png")
    finally:
        renderer.close()
    report = {"grippers": args.grippers, "matches_operator_reported_gripper_selection": args.grippers == "right",
        **model_provenance(source), "upstream": json.loads((ASSETS / "provenance.json").read_text()),
        "physics": physics_settings(model), "nq": model.nq, "nv": model.nv, "nu": model.nu, "neq": model.neq,
        "physics_steps": 1500, "warning_counts": warnings.tolist(), "endpoint_gap_m": endpoint_gaps,
        "trace": trace, "saved_mjcf_reloaded": True, "runtime_qpos_writes_after_initialization": 0,
        "gripper_command": "opening-positive radians; upstream motor angle = 2.2028 - command",
        "camera_mount_parameters_preserved": False, "camera_mount_physically_verified": False,
        "camera_mount_basis": "photo-estimated rigid NORMA bracket; not target-tracking",
        "sim_to_real_valid": False, "grasp_success_verified": False, "hardware_execution": False,
        "limitations": ["SO-101 mounting transform follows the supplied URDF; physical assembly/encoder zero remain unverified",
            "Camera bracket seat, lens center, image roll and FOV are estimates; camera mass/collision are not modeled",
            "Dynamics are source/stock model estimates; gear inertia approximated from its bounding box",
            "Convex distal fingertip hulls and a housing box; full finger/rack/gear tooth contact is not simulated",
            "Endpoint PNGs are pose-only; dynamic trace is a no-object opening/closing smoke test",
            "Stock recordings and ACT checkpoints cannot be reused without new calibration/data provenance"]}
    (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: report[k] for k in ("grippers", "nu", "neq", "physics_steps", "endpoint_gap_m",
                                          "saved_mjcf_reloaded", "hardware_execution")}, indent=2))
    print(output)
    if args.viewer:
        from mujoco import viewer as mj_viewer
        apply_control_as_pose(model, data, initial)
        with mj_viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [.12, 0, .15]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = .9, 135, -30
            deadline = time.monotonic() + 1800
            while viewer.is_running() and time.monotonic() < deadline:
                started = time.monotonic()
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(max(0, model.opt.timestep - (time.monotonic() - started)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--grippers", choices=("right", "both"), default="right",
                        help="right matches the current assembly; both previews the planned replacement")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--viewer", action="store_true", help="open simulation-only controls for up to 30 minutes")
    run(parser.parse_args())
