#!/usr/bin/env python3
"""SIM-only LEFT +20 mm/hold/return rehearsal; never dispatches hardware."""
import argparse
from contextlib import nullcontext
from dataclasses import asdict
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from collision_guard import check_bimanual_path, task_clearance_status
from integration_scenes import task_env
from mobile_dual_so101 import actuator_targets_from_qpos
from physics_ik import MotionLimits, plan_septic_joint_trajectory, solve_bimanual_position_ik


# Reserve the observed command-FK to measured-TCP deviation; acceptance remains 0.5 mm.
PLANNING_POSITION_TOLERANCE_M = .0005 - .0003465963856126189


def plan(env):
    m, d = env.model, env.data
    site = m.site("left_cube_grasp").id
    start_xyz = d.site_xpos[site].copy()
    axis = d.site_xmat[site].reshape(3, 3)[:, 0].copy()
    target_xyz = start_xyz + [0., 0., .020]
    ik = solve_bimanual_position_ik(
        m, actuator_targets_from_qpos(m, d.qpos), {"left": target_xyz}, site_names={"left": "left_cube_grasp"},
        tool_axis_targets={"left": axis}, axis_formulation="axis_direction",
        max_iterations=300, tolerance_m=PLANNING_POSITION_TOLERANCE_M)
    if not ik.converged:
        raise ValueError(f"+20 mm IK failed: {ik}")
    # Measured q defines the collision start. Unoperated channels retain explicit ctrl.
    target = np.array(ik.action_rad)
    target[5:] = d.ctrl[5:]
    measured = actuator_targets_from_qpos(m, d.qpos)
    guard = check_bimanual_path(m, measured, target, reference_data=d, task_phase="SAFE_STAGE",
                                 required_clearance_m=env.config.required_clearance_m)
    if not guard.safe:
        raise ValueError(f"+20 mm path failed: {guard}")
    trajectory = plan_septic_joint_trajectory(m, d.ctrl.copy(), target, minimum_duration_s=.8)
    return site, start_xyz, target_xyz, axis, trajectory, asdict(ik), asdict(guard)


def run(env, report, viewer=None):
    m, d = env.model, env.data
    report.update(scene_id=env.config.scene_id, hardware_execution=False,
                  real_start_pose_verified=False, runtime_qpos_writes=0,
                  start_provenance="SIM model_default RESET; NOT measured REAL pose",
                  passed_scope="SIM endpoint/path/joint/clearance/warning/tracking only; NOT full dynamics or hardware",
                  qpos_write_count_basis="source contract; reset/planning writes excluded",
                  solver=dict(timestep=float(m.opt.timestep), noslip_iterations=int(m.opt.noslip_iterations)),
                  frames="metres/radians; desk world +Z up; base +Z parallel",
                  phases=[], steps=[])
    env.reset(seed=0)
    env.collision_phase = "SAFE_STAGE"  # No manipulation proximity exemptions.
    phase = "SETTLE"
    start_xyz = d.site("left_cube_grasp").xpos.copy()
    wall = time.monotonic()
    last_draw = -1.

    def observe():
        nonlocal last_draw
        measured = np.array(actuator_targets_from_qpos(m, d.qpos))
        clearance = task_clearance_status(m, d, "SAFE_STAGE")
        row = dict(phase=phase, sim_time_s=float(d.time), command=d.ctrl.tolist(),
                   measured=measured.tolist(), raw_qpos=d.qpos.tolist(),
                   tcp_xyz=d.site("left_cube_grasp").xpos.tolist(),
                   clearance=clearance,
                   max_tracking_error_rad=float(np.max(np.abs(d.ctrl-measured))))
        row["joint_margins_rad"] = {m.joint(j).name: [float(d.qpos[m.jnt_qposadr[j]]-m.jnt_range[j,0]),
                float(m.jnt_range[j,1]-d.qpos[m.jnt_qposadr[j]])] for j in range(m.njnt)
                if m.jnt_limited[j] and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE}
        report["steps"].append(row)
        if np.any(d.warning.number):
            raise RuntimeError("MuJoCo warning")
        if row["max_tracking_error_rad"] > MotionLimits().max_tracking_error_rad:
            raise RuntimeError("existing tracking bound exceeded")
        if viewer is not None and d.time-last_draw >= 1/30:
            if not viewer.is_running():
                raise RuntimeError("viewer closed; rehearsal stopped")
            lines = ["SIM PHYSICS / ctrl + mj_step / NO HARDWARE", phase,
                     f"time {d.time:.3f} s | general {clearance['general_clearance_m']*1000:.3f} mm",
                     f"TCP dz {(d.site('left_cube_grasp').xpos[2]-start_xyz[2])*1000:.3f} mm",
                     "target LEFT " + " ".join(f"{q:.3f}" for q in d.ctrl[:6]),
                     "measured    " + " ".join(f"{q:.3f}" for q in measured[:6]),
                     "REAL start pose / joint mapping: NOT VERIFIED"]
            viewer.set_texts([(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               "\n".join(lines), "")])
            viewer.sync()
            time.sleep(max(0., wall+float(d.time)-time.monotonic()))
            last_draw = float(d.time)

    env.physics_observer = observe
    report["settle"] = env.settle()
    site, start_xyz, target_xyz, axis, up, ik, guard = plan(env)
    start_command = d.ctrl.copy()
    report.update(start_q=start_command.tolist(), target_q=up.goal_rad.tolist(),
                  return_q=start_command.tolist(), ik=ik, path=guard,
                  left_delta_rad=(up.goal_rad[:5]-start_command[:5]).tolist(),
                  left_delta_deg=np.degrees(up.goal_rad[:5]-start_command[:5]).tolist(),
                  start_xyz=start_xyz.tolist(), target_xyz=target_xyz.tolist(),
                  duration_s=up.duration_s, hold_s=.5,
                  planning_position_tolerance_m=PLANNING_POSITION_TOLERANCE_M,
                  reserve_basis="baseline observed FK-measured deviation; not a robustness bound", profile="existing septic; MotionLimits unchanged",
                  actuator_names=[m.actuator(i).name for i in range(m.nu)])

    def endpoint(expected):
        position_error = float(np.linalg.norm(d.site_xpos[site]-expected))
        tracking = float(np.max(np.abs(d.ctrl-np.array(actuator_targets_from_qpos(m, d.qpos)))))
        actual_axis = d.site_xmat[site].reshape(3, 3)[:, 0]
        approach_error = float(np.degrees(np.arccos(np.clip(actual_axis@axis, -1., 1.))))
        result = dict(phase=phase, sim_time_s=float(d.time), tcp_xyz=d.site_xpos[site].tolist(),
                      position_error_m=position_error, approach_error_deg=approach_error,
                      final_tracking_error_rad=tracking)
        report["phases"].append(result)
        print(json.dumps(result), flush=True)
        if tracking > MotionLimits().max_final_tracking_error_rad:
            raise RuntimeError("existing final tracking bound exceeded")
        if position_error > .0005 or approach_error > 2.:
            raise RuntimeError("existing 0.5 mm / 2 deg endpoint acceptance failed")

    def move(trajectory):
        duration_steps = int(np.ceil(trajectory.duration_s/m.opt.timestep))
        for i in range(1, duration_steps+1):
            command = trajectory.sample(min(i*m.opt.timestep, trajectory.duration_s))[0]
            env.apply_action(command, physics_steps=1)

    phase = "UP_20MM"
    print(phase, flush=True)
    move(up)
    endpoint(target_xyz)
    phase = "NONCONTACT_HOLD"
    for _ in range(round(.5/m.opt.timestep)):
        env.apply_action(up.goal_rad, physics_steps=1)
    endpoint(target_xyz)
    phase = "RETURN"
    print(phase, flush=True)
    measured = actuator_targets_from_qpos(m, d.qpos)
    guard = check_bimanual_path(m, measured, start_command, reference_data=d, task_phase="SAFE_STAGE",
                                 required_clearance_m=env.config.required_clearance_m)
    report["return_guard"] = asdict(guard)
    if not guard.safe:
        raise RuntimeError("return path rejected")
    move(plan_septic_joint_trajectory(m, d.ctrl.copy(), start_command, minimum_duration_s=.8))
    endpoint(start_xyz)
    report["passed"] = True
    env.physics_observer = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    env = task_env("desk")
    report = {"passed": False}
    if args.viewer:
        from mujoco.viewer import launch_passive
        context = launch_passive(env.model, env.data, show_left_ui=False, show_right_ui=False)
    else:
        context = nullcontext(None)
    with context as viewer:
        if viewer:
            viewer.cam.lookat[:] = [.18, 0, .18]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.1, 135, -30
        try:
            run(env, report, viewer)
        except (ValueError, RuntimeError) as error:
            report["failure"] = str(error)
            print("STOP: " + str(error), flush=True)
            if viewer:
                viewer.set_texts([(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                   "SIM PHYSICS STOP / NO HARDWARE\n"+str(error), "")])
        finally:
            env.physics_observer = None
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2)+"\n")
        if viewer:
            print("Physics stopped; preserving final state for 60 s", flush=True)
            until = time.monotonic()+60
            while viewer.is_running() and time.monotonic()<until:
                viewer.sync()
                time.sleep(.05)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
