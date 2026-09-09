#!/usr/bin/env python3
"""SIM-only left pickup -> right handover -> right placement; not camera-guided, ACT or hardware."""
import argparse
import json
from pathlib import Path
from threading import Event
import time
import mujoco
import numpy as np
from mobile_dual_so101 import apply_control_as_pose, resolve_so101_model
from pgripper import home_action
from collision_guard import _collision_geoms_for_arm, _same_arm_geom_pairs
from physics_ik import plan_septic_joint_trajectory, solve_bimanual_position_ik
from replay_recorded_episode import build_tabletop_spec, inspect_contacts, physics_settings, sha256


# Offline 5-DOF IK for the fixed scene: donor present, recipient pre, recipient grasp.
# Recipient approaches along donor pad width; no object pose is read by the planner.
WAYPOINTS = {
    "right": (
        [-.574844801314, -.022102205000, -.451632013407, 1.375045807261, 2.175441934218],
        [.582809236600, -.157804654739, 1.397516068787, -1.575496886858, -2.125094839219],
        [.581398950131, -.113460085301, 1.239113617885, -1.465675010949, -2.126297118725]),
    "left": (
        [.580052084060, -.013975119061, -.465292296251, 1.366124686554, -2.027834427009],
        [-.578384314568, -.149694934652, 1.395817181374, -1.589436172226, -.904607424748],
        [-.578174359657, -.106353721142, 1.237549063593, -1.478845851889, -.904437071030]),
}


def target_pad_forces(model, data, pairs):
    """Count exact block/pad pairs; table or opposite-arm contacts cannot pass."""
    forces = np.zeros(4)
    for index, contact in enumerate(data.contact):
        pad = pairs.get(frozenset((contact.geom1, contact.geom2)))
        if pad is not None:
            wrench = np.zeros(6)
            mujoco.mj_contactForce(model, data, index, wrench)
            if not np.isfinite(wrench).all():
                raise ValueError("non-finite contact wrench")
            forces[pad] += max(0.0, wrench[0])
    return forces.reshape(2, 2)


def finite_list(values):
    """Keep a failure report writable even after invalid simulator state."""
    values = np.asarray(values)
    return np.where(np.isfinite(values), values, None).tolist()


def table_support_force(model, data):
    pair = {model.geom("table").id, model.geom("red_block_geom").id}
    total = 0.0
    for index, contact in enumerate(data.contact):
        if {contact.geom1, contact.geom2} == pair:
            wrench = np.zeros(6)
            mujoco.mj_contactForce(model, data, index, wrench)
            if not np.isfinite(wrench).all():
                raise ValueError("non-finite table support force")
            total += max(0.0, wrench[0])
    return total


def run(args, *, replay_requested=None, observer=None, initial_arm_offset=None):
    replay_requested = replay_requested if replay_requested is not None else Event()
    if args.donor != "left" and not args.handover_only:
        raise ValueError("the full task is left pickup -> right receive -> right place; use --handover-only for reverse")
    output = args.output.resolve()
    if output.exists():
        raise ValueError("use a new output directory")
    sources = [Path(__file__), Path(__file__).with_name("pgripper.py"),
               Path(__file__).with_name("physics_ik.py"), Path(__file__).with_name("tabletop_replay.json")]
    hashes = {p.name: sha256(p) for p in sources}
    profile = json.loads(Path(__file__).with_name("tabletop_replay.json").read_text())
    if (not np.allclose(profile["reference_block_center_m"], [.20, 0, .02])
            or not np.allclose(profile["reference_block_size_m"], [.04] * 3)
            or profile["block_mass_kg"] != .02 or profile["camera_to_base_horizontal_m"] != .15
            or profile["arm_yaw_deg"] != [0.0, 0.0]):
        raise ValueError("offline handover waypoints require the documented 4cm/20g fixed scene")
    model = build_tabletop_spec(resolve_so101_model(args.model), profile, grippers="both").compile()
    # Reduce soft-contact tangential drift, without changing friction or adding attachments.
    model.opt.impratio = 10
    donor = args.donor
    recipient = "left" if donor == "right" else "right"
    sides = {"left": 0, "right": 1}
    donor_arm = slice(sides[donor] * 6, sides[donor] * 6 + 5)
    recipient_arm = slice(sides[recipient] * 6, sides[recipient] * 6 + 5)
    donor_grip = sides[donor] * 6 + 5
    recipient_grip = sides[recipient] * 6 + 5
    geom_sides = [(model.body(int(model.geom_bodyid[g])).name or "").split("_")[0]
                  for g in range(model.ngeom)]
    self_collision_pairs = {frozenset(pair) for side in sides
        for pair in _same_arm_geom_pairs(model, _collision_geoms_for_arm(model, side))}
    block_id = model.geom("red_block_geom").id
    pairs = {frozenset((block_id, model.geom(f"{side}_pgripper_pad_{finger}").id)): i * 2 + finger - 1
             for side, i in sides.items() for finger in (1, 2)}
    if args.disable_recipient_contact:
        for finger in (1, 2):
            geom = model.geom(f"{recipient}_pgripper_pad_{finger}")
            geom.contype = geom.conaffinity = 0
    if any(model.eq_type == mujoco.mjtEq.mjEQ_WELD) or model.body("red_block").mocapid != -1:
        raise ValueError("the block must be an unattached free body")
    data = mujoco.MjData(model)
    command = np.asarray(home_action(model, np.tile(np.deg2rad([0, -35, 55, 35, 0, 0]), 2)))
    if initial_arm_offset is not None:
        offset = np.asarray(initial_arm_offset, dtype=float)
        if offset.shape != (12,) or not np.isfinite(offset).all() or np.max(np.abs(offset)) > np.deg2rad(1):
            raise ValueError("initial arm perturbation must be finite and at most 1 degree")
        if np.any(offset[[5, 11]]):
            raise ValueError("initial perturbation cannot change grippers")
        command += offset
        command = np.clip(command, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
    apply_control_as_pose(model, data, command)  # Only runtime pose initialization.
    output.mkdir(parents=True, exist_ok=False)
    trace, plans = [], []
    deepest = {"depth_m": 0.0}
    phase, failure = "settle", None
    interrupted_for_replay = False
    donor_released = False
    donor_release_started = False
    success = False
    task_success = False
    place_release_started = False
    table_hold_steps = 0
    hold_commands = {}
    grip_confirmations = {}
    recipient_only_hold_steps = 0
    force_peak = np.zeros((2, 2))
    dt = float(model.opt.timestep)
    viewer = None
    if args.viewer:
        from mujoco import viewer as mj_viewer
        def key_callback(keycode):
            if keycode in (ord("R"), ord("r")):
                replay_requested.set()  # The physics thread performs the restart, not the GUI callback.
        viewer = mj_viewer.launch_passive(model, data, key_callback=key_callback)
        viewer.cam.lookat[:] = [.19, 0, .14]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = .85, 135, -30
    wall_started = time.monotonic()

    def tick(target):
        nonlocal recipient_only_hold_steps
        if viewer is not None and replay_requested.is_set():
            raise InterruptedError("viewer replay requested")
        if viewer is not None and not viewer.is_running():
            raise ValueError("viewer closed by user")
        target = np.asarray(target).copy()
        if np.shape(target) != (12,) or not np.isfinite(target).all():
            raise ValueError("invalid joint command")
        if np.any(target < model.actuator_ctrlrange[:, 0]) or np.any(target > model.actuator_ctrlrange[:, 1]):
            raise ValueError("joint command outside range")
        for side, grip in hold_commands.items():
            target[sides[side] * 6 + 5] = grip
        if observer is not None:
            observer(model, data, target.copy(), phase)
        data.ctrl[:] = target
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
            raise ValueError("invalid physics or MuJoCo warning")
        inspect_contacts(model, data, deepest, len(trace))
        forces = target_pad_forces(model, data, pairs)
        np.maximum(force_peak, forces, out=force_peak)
        if forces.max() > 10:
            raise ValueError(f"virtual pad overload in {phase}: {forces.tolist()}")
        if donor in grip_confirmations and not donor_release_started and forces[sides[donor]].min() < .3:
            raise ValueError(f"donor bilateral force lost during {phase}")
        if phase in ("donor release", "donor retreat", "recipient carry", "place approach",
                     "place seat", "place support confirm") and forces[sides[recipient]].min() < .3:
            raise ValueError(f"recipient bilateral force lost during {phase}")
        if phase == "donor retreat":
            if forces[sides[donor]].max() > .01 or data.body("red_block").xpos[2] < .06:
                raise ValueError("recipient-only hold lost during donor retreat")
            recipient_only_hold_steps += 1  # Verify while the empty arm moves, without an extra pause.
        for side in hold_commands:
            minimum = forces[sides[side]].min()
            if minimum < 1.5:
                speed = .08 if minimum < 1.0 else .02
                hold_commands[side] = max(0, hold_commands[side] - speed * dt)
        for contact in data.contact:
            names = [model.geom(g).name for g in (contact.geom1, contact.geom2)]
            if (frozenset((contact.geom1, contact.geom2)) in self_collision_pairs
                    or {geom_sides[contact.geom1], geom_sides[contact.geom2]} == {"left", "right"}
                    or ("table" in names and any(geom_sides[g] in sides for g in (contact.geom1, contact.geom2)))):
                raise ValueError(f"forbidden arm collision: {names}")
        if not trace or data.time >= trace[-1]["time_s"] + .02 - 1e-9:
            trace.append({"time_s": float(data.time), "phase": phase, "forces_N": forces.tolist(),
                          "ctrl_rad": list(target), "block_position_m": data.body("red_block").xpos.tolist()})
            if viewer is not None:
                viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    f"DUAL PGRIPPER | {donor} -> {recipient} | SIM ONLY",
                    f"{phase} | {data.time:.1f}s | R: restart | NOT ACT")])
                viewer.sync()
                time.sleep(max(0, wall_started + data.time - time.monotonic()))
        if deepest["depth_m"] > .001:
            raise ValueError(f"penetration exceeds 1mm: {deepest}")
        return forces

    def hold(seconds):
        for _ in range(int(np.ceil(seconds / dt))):
            tick(command)

    def move(goal, seconds=1.0):
        nonlocal command
        trajectory = plan_septic_joint_trajectory(model, command, goal, minimum_duration_s=seconds)
        plans.append({"phase": phase, "duration_s": trajectory.duration_s, "goal_rad": list(goal)})
        print(phase, "duration", round(trajectory.duration_s, 2), flush=True)
        for step in range(int(np.ceil(trajectory.duration_s / dt))):
            target, _, _, _ = trajectory.sample((step + 1) * dt)
            tick(target)
        command = np.asarray(goal).copy()
        for side, grip in hold_commands.items():
            command[sides[side] * 6 + 5] = grip

    def target_at(side, center, axis=(0, 0, -1)):
        result = solve_bimanual_position_ik(model, command, {side: center},
            site_names={side: f"{side}_cube_grasp"}, tool_axis_targets={side: axis}, max_iterations=500)
        if not result.converged:
            raise ValueError(f"IK failed: {result.residual_m_by_side}, {result.tool_axis_error_rad_by_side}")
        return np.asarray(result.action_rad)

    def close(side):
        index, stable = sides[side], 0.0
        for _ in range(int(np.ceil(45 / dt))):
            forces = target_pad_forces(model, data, pairs)[index]
            if forces.max() > 10:
                raise ValueError(f"{side} virtual pad overload: {forces.tolist()}")
            if forces.min() < 1.5:  # Closure target above the 1 N acceptance floor prevents chatter.
                # ponytail: fixture-scale speed; tune with measured sensors before real use.
                speed = .20 if forces.max() < .05 else .04
                command[index * 6 + 5] = max(0, command[index * 6 + 5] - speed * dt)
            forces = tick(command)[index]
            stable = stable + dt if forces.min() >= 1.0 else 0.0
            if stable >= .33:
                command[:] = data.ctrl  # Preserve the other hand's live grip correction.
                hold_commands[side] = command[index * 6 + 5]
                grip_confirmations[side] = {"time_s": float(data.time), "stable_for_s": stable,
                                            "minimum_pad_force_N": float(forces.min())}
                print(side, "bilateral grip", forces.tolist(), flush=True)
                return
        raise ValueError(f"{side} bilateral force timeout; donor release forbidden")

    try:
        hold(.5)
        center = np.asarray(profile["reference_block_center_m"])
        phase = "donor approach"
        move(target_at(donor, center + [0, 0, .06]))
        phase = "donor descend"
        move(target_at(donor, center + [0, 0, .01]), 3.0)
        phase = "donor close"
        close(donor)
        phase = "donor lift"
        move(target_at(donor, center + [0, 0, .06]), 3.0)
        phase = "donor present"
        presentation = command.copy()
        presentation[donor_arm] = WAYPOINTS[donor][0]
        move(presentation, 6.0)
        # Known-scene, offline-IK waypoints: receiver approaches along donor pad width.
        # Cube center approximately (.27, 0, .17); no cube truth is used as a target.
        receiving = command.copy()
        receiving[recipient_arm] = WAYPOINTS[donor][1]
        phase = "recipient orient away"
        clearance = receiving.copy()
        clearance[sides[recipient] * 6] = 0.0
        move(clearance, 5.0)
        phase = "recipient approach"
        move(receiving, 3.0)
        phase = "recipient insert"
        receiving[recipient_arm] = WAYPOINTS[donor][2]
        move(receiving, 3.0)
        phase = "recipient close"
        close(recipient)
        phase = "donor release"
        donor_release_started = True  # Reachable only after recipient bilateral-force gate.
        hold_commands.pop(donor)
        released = command.copy()
        released[donor_grip] = model.actuator_ctrlrange[donor_grip, 1]
        move(released, 2.0)
        if target_pad_forces(model, data, pairs)[sides[donor]].max() > .01:
            raise ValueError("donor contact persisted after opening")
        donor_released = True
        phase = "donor retreat"
        donor_up = data.body(f"{donor}_gripper").xmat.reshape(3, 3)[:, 2].copy()
        move(target_at(donor, data.site(f"{donor}_cube_grasp").xpos + .04 * donor_up, -donor_up), 3.0)
        phase = "recipient hold"
        for _ in range(max(0, int(np.ceil(3.0 / dt)) - recipient_only_hold_steps)):
            forces = tick(command)
            if forces[sides[recipient]].min() < .3 or forces[sides[donor]].max() > .01:
                raise ValueError("recipient-only bilateral hold lost")
            if data.body("red_block").xpos[2] < .06:
                raise ValueError("recipient hold did not keep block clear of table")
            recipient_only_hold_steps += 1
        success = True
        if not args.handover_only:
            # Safe offline-IK branch switch; the nominal high vertical target is not reachable.
            phase = "recipient carry"
            carry = command.copy()
            carry[recipient_arm] = [-.371009057, .005871891, -.299537741, 1.658060000, -1.028668818]
            move(carry, 6.0)
            phase = "place approach"
            move(target_at(recipient, [.22, -.08, .060]), 3.0)
            phase = "place seat"
            endpoint = target_at(recipient, [.22, -.08, .040])
            landing = plan_septic_joint_trajectory(model, command, endpoint, minimum_duration_s=8.0)
            plans.append({"phase": phase, "duration_s": landing.duration_s,
                          "goal_rad": endpoint.tolist(), "stop_condition": "block-table normal force >= 0.1 N"})
            supported = False
            for step in range(int(np.ceil(landing.duration_s / dt))):
                target, _, _, _ = landing.sample((step + 1) * dt)
                tick(target)
                if table_support_force(model, data) >= .1:
                    command = data.ctrl.copy()
                    supported = True
                    break
            if not supported:
                raise ValueError("table support not established; recipient release forbidden")
            phase = "place support confirm"
            for _ in range(int(np.ceil(.2 / dt))):
                tick(command)
                if table_support_force(model, data) < .05:
                    raise ValueError("table support lost; recipient release forbidden")
            phase = "place release"
            hold_commands.pop(recipient)  # Confirmed table support now owns the block.
            place_release_started = True
            opened = command.copy()
            opened[recipient_grip] = model.actuator_ctrlrange[recipient_grip, 1]
            move(opened, 3.0)
            phase = "place retreat"
            move(target_at(recipient, data.site(f"{recipient}_cube_grasp").xpos + [0, 0, .03]), 3.0)
            phase = "table hold"
            block_dof = int(model.joint("red_block_free").dofadr[0])
            for _ in range(int(np.ceil(3.0 / dt))):
                forces = tick(command)
                height = data.body("red_block").xpos[2]
                speed = np.linalg.norm(data.qvel[block_dof:block_dof + 3])
                if (forces.max() > .01 or table_support_force(model, data) < .1
                        or not .019 <= height <= .022 or speed > .01):
                    raise ValueError("released block is not settled on the table")
                table_hold_steps += 1
            task_success = True
    except (ValueError, RuntimeError, InterruptedError) as error:
        failure = {"phase": phase, "reason": str(error)}
        interrupted_for_replay = isinstance(error, InterruptedError) and replay_requested.is_set()
    report = {"kind": "known-scene IK + virtual-contact SIM baseline", "donor": donor,
        "recipient": recipient, "handover_success": success, "failure": failure,
        "task_success": task_success, "handover_only": args.handover_only,
        "interrupted_for_replay": interrupted_for_replay,
        "place_release_started": place_release_started, "table_supported_hold_s": table_hold_steps * dt,
        "task_sequence": [f"{donor} pickup", f"handover to {recipient}"]
            + ([] if args.handover_only else ["right place on table"]),
        "donor_released": donor_released, "recipient_contact_disabled": args.disable_recipient_contact,
        "donor_release_started": donor_release_started,
        "grip_confirmations": grip_confirmations, "recipient_only_hold_s": recipient_only_hold_steps * dt,
        "deepest_contact": deepest, "pad_force_peak_N": force_peak.tolist(),
        "final_block_position_m": finite_list(data.body("red_block").xpos), "final_command_rad": finite_list(data.ctrl),
        "final_block_quaternion_wxyz": finite_list(data.body("red_block").xquat),
        "final_tcp_positions_m": {side: finite_list(data.site(f"{side}_cube_grasp").xpos) for side in sides},
        "verifier_block_in_donor_frame_m": finite_list(data.body(f"{donor}_gripper").xmat.reshape(3, 3).T
            @ (data.body("red_block").xpos - data.site(f"{donor}_cube_grasp").xpos)),
        "plans": plans, "simulation_seconds": float(data.time) if np.isfinite(data.time) else None,
        "physics": physics_settings(model), "source_hashes": hashes,
        "source_hashes_unchanged": hashes == {p.name: sha256(p) for p in sources},
        "warning_counts": [int(w.number) for w in data.warning], "runtime_qpos_writes_after_initialization": 0,
        "object_pose_writes_or_attachments": False, "object_pose_used_for_control": False,
        "target_source": "known scene fixture + kinematics", "camera_driven": False, "learned_ACT": False,
        "virtual_contact_feedback_used": True, "virtual_contact_sensors_available_on_real_robot": False,
        "hardware_execution": False, "sim_to_real_valid": False,
        "limitations": ["Fixed-scene scripted IK/contact baseline; not camera-guided or learned ACT.",
            "Mount/camera calibration and motor/material parameters are not physically verified.",
            "Collision checks cover existing housing/fingertip proxies, not full rack/camera geometry.",
            "Contact solver impratio=10 reduces numerical drift; friction remains unmeasured."]}
    (output / "trace.json").write_text(json.dumps(trace, indent=2, allow_nan=False) + "\n")
    (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    accepted = success if args.handover_only else task_success
    print(json.dumps({key: report[key] for key in ("donor", "recipient", "handover_success", "task_success", "failure",
        "recipient_only_hold_s", "deepest_contact", "simulation_seconds")}, indent=2), flush=True)
    print(output, flush=True)
    if args.render and accepted:
        from PIL import Image
        renderer = mujoco.Renderer(model, width=640, height=480)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [.19, 0, .14]
        camera.distance, camera.azimuth, camera.elevation = .85, 135, -30
        try:
            for label, view in (("tabletop", camera), ("left-wrist", "left_wrist_rgb"),
                                ("right-wrist", "right_wrist_rgb")):
                renderer.update_scene(data, camera=view)
                Image.fromarray(renderer.render()).save(output / f"final-{label}.png")
        finally:
            renderer.close()
    if viewer is not None:
        viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
            "SIM TASK PASS" if accepted else "SIM TASK STOPPED", "R: restart | Close window: exit | SIM ONLY")])
        while viewer.is_running() and not replay_requested.is_set():
            viewer.sync()
            time.sleep(.05)
        viewer.close()
    return 0 if accepted else 2


def main(args):
    replay_requested = Event()
    attempt = 0
    while True:
        current = argparse.Namespace(**vars(args))
        if attempt:
            current.output = args.output.with_name(f"{args.output.name}-replay-{attempt:03d}")
        replay_requested.clear()
        code = run(current, replay_requested=replay_requested)
        if not args.viewer or not replay_requested.is_set():
            return code
        attempt += 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--donor", choices=("left", "right"), default="left",
                        help="task default: left to right; right selects the reverse verification")
    parser.add_argument("--disable-recipient-contact", action="store_true")
    parser.add_argument("--handover-only", action="store_true", help="diagnostic: stop after handover and hold")
    parser.add_argument("--render", action="store_true", help="save final physics-state images on success")
    parser.add_argument("--viewer", action="store_true", help="show motion; R restarts, closing the window exits")
    raise SystemExit(main(parser.parse_args()))
