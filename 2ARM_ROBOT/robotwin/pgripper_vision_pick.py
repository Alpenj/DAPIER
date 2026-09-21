"""SIM-only RGB-D localization + wrist-RGB refinement + contact-checked PGripper pickup."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from pgripper_collect import make_scene, pose_robot, contacts, Recorder


def transform(position, quaternion):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(np.roll(quaternion, -1)).as_matrix()
    matrix[:3, 3] = position
    return matrix


def arm_fk(reference, action, side):
    """Pure kinematics: never changes a simulator state."""
    values = dict(zip(reference["joints"], action))
    pose = np.eye(4)
    for body in reference["bodies"]:
        if not body["name"].startswith(side+"_"):
            continue
        pose = pose @ transform(body["pos"], body["quat"])
        joint = body.get("joint")
        if joint:
            motion = np.eye(4)
            rotation = Rotation.from_rotvec(np.asarray(joint["axis"]) * (values[joint["name"]]-joint["ref"])).as_matrix()
            motion[:3, :3] = rotation
            motion[:3, 3] = np.asarray(joint["pos"])-rotation@joint["pos"]
            pose = pose @ motion
        if body["name"] == side+"_gripper":
            return pose
    raise ValueError("Missing gripper frame")


def target_action(reference, tcp, start, side, center, keep_vertical=True, jaw_axis=None):
    index = 0 if side == "left" else 6
    limits = np.asarray(reference["ctrlrange"])[index:index+5]
    local = transform(tcp[side]["pos"], tcp[side]["quat"])
    start = np.asarray(start).copy()
    def residual(angles):
        action = start.copy()
        action[index:index+5] = angles
        pose = arm_fk(reference, action, side) @ local
        orientation = .05*(pose[:3, 2]-jaw_axis) if jaw_axis is not None else (
            .05*(pose[:3, 0]-[0, 0, -1]) if keep_vertical else .001*(angles-start[index:index+5]))
        return np.r_[pose[:3, 3]-center, orientation]
    fit = least_squares(residual, start[index:index+5], bounds=limits.T, max_nfev=250,
                        ftol=1e-11, xtol=1e-11, gtol=1e-11)
    error = residual(fit.x)
    if np.linalg.norm(error[:3]) > .001 or (keep_vertical and np.linalg.norm(error[3:]) > .002):
        raise ValueError(f"IK did not converge: {error.tolist()}")
    start[index:index+5] = fit.x
    if jaw_axis is not None and (arm_fk(reference, start, side) @ local)[2, 0] > -.95:
        raise ValueError("Face-aligned grasp tilts more than 18 degrees")
    return start


def red_mask(rgb):
    hsv = cv2.cvtColor(np.asarray(rgb, np.uint8), cv2.COLOR_RGB2HSV)
    mask = (((hsv[..., 0] < 12) | (hsv[..., 0] > 170)) & (hsv[..., 1] > 130) & (hsv[..., 2] > 35)).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        raise ValueError("No red block visible")
    chosen = 1+int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[chosen, cv2.CC_STAT_AREA] < 30:
        raise ValueError("Too few block pixels")
    return labels == chosen


def estimate_rgbd(rgb, depth, intrinsic, world_from_gl):
    """Use RGB and metric depth only; the known cube is 4 cm tall."""
    mask = red_mask(rgb) & np.isfinite(depth) & (depth > .05) & (depth < 2)
    rows, columns = np.nonzero(mask)
    z = depth[mask]
    camera = np.column_stack([(columns+.5-intrinsic[0, 2])*z/intrinsic[0, 0],
                              -(rows+.5-intrinsic[1, 2])*z/intrinsic[1, 1], -z])
    points = camera @ world_from_gl[:3, :3].T + world_from_gl[:3, 3]
    top = points[(points[:, 2] > .037) & (points[:, 2] < .043)]
    if len(top) < 30:
        raise ValueError("Insufficient top-surface depth")
    rectangle = cv2.minAreaRect(top[:, :2].astype(np.float32))
    size = np.asarray(rectangle[1])
    if size.min() < .026 or size.max() > .05:
        raise ValueError(f"Unexpected cube footprint: {size.tolist()}")
    center = np.r_[rectangle[0], np.median(top[:, 2])-.02]
    if not (.12 < center[0] < .29 and abs(center[1]) < .10 and .016 < center[2] < .024):
        raise ValueError("RGB-D location outside calibrated workspace")
    return center, np.deg2rad(rectangle[2]), dict(top_points=len(top), footprint_m=size.tolist())


def wrist_center(rgb, intrinsic, world_from_gl, prior, yaw):
    """Fit the known cube silhouette to wrist RGB; no wrist depth or actor pose."""
    mask = red_mask(rgb)
    rows, columns = np.nonzero(mask)
    observed = np.array([columns.min()+.5, rows.min()+.5, columns.max()+.5, rows.max()+.5])
    if columns.min() <= 1 or rows.min() <= 1 or columns.max() >= rgb.shape[1]-2 or rows.max() >= rgb.shape[0]-2:
        raise ValueError("Wrist block silhouette clipped at image boundary")
    corners = np.array(list(itertools.product([-.02, .02], repeat=3)))
    corners = corners @ Rotation.from_euler("z", yaw).as_matrix().T
    rotation, translation = world_from_gl[:3, :3], world_from_gl[:3, 3]
    def box_pixels(xy):
        center = np.r_[xy, prior[2]]
        camera = (corners+center-translation) @ rotation
        depth = -camera[:, 2]
        if np.any(depth <= .01):
            raise ValueError("Cube behind wrist camera")
        pixel = np.column_stack([intrinsic[0, 0]*camera[:, 0]/depth+intrinsic[0, 2],
                                 -intrinsic[1, 1]*camera[:, 1]/depth+intrinsic[1, 2]])
        return np.r_[pixel.min(0), pixel.max(0)]
    fit = least_squares(lambda xy: box_pixels(xy)-observed, prior[:2],
        bounds=(prior[:2]-.012, prior[:2]+.012), max_nfev=80)
    error = float(np.linalg.norm(box_pixels(fit.x)-observed))
    if error > 5:
        raise ValueError(f"Wrist silhouette fit unreliable: {error:.2f}px")
    return np.r_[fit.x, prior[2]], dict(observed_bbox_px=observed.tolist(),
        before_error_px=float(np.linalg.norm(box_pixels(prior[:2])-observed)), fit_error_px=error)


def observe(recorder, output, number, blank_top=False, blank_wrist=None):
    for camera, parent, local in recorder.cameras:
        camera.set_entity_pose(local if parent is None else parent.entity_pose*local)
    recorder.scene.update_render()
    frames = {}
    for camera, _, _ in recorder.cameras:
        camera.take_picture()
        rgb = np.clip(camera.get_picture("Color")[..., :3]*255, 0, 255).astype(np.uint8)
        if (blank_top and camera.name == "top_h201_reference") or camera.name == blank_wrist:
            rgb[:] = 0
        frame = dict(rgb=rgb, intrinsic=camera.get_intrinsic_matrix(), transform=camera.get_model_matrix())
        if camera.name == "top_h201_reference":
            position = camera.get_picture("Position")
            frame["depth"] = np.maximum(0, -position[..., 2])
            frame["depth"][position[..., 3] >= 1] = 0
            np.save(output/f"observation-{number:02d}-depth.npy", frame["depth"])
        cv2.imwrite(str(output/f"observation-{number:02d}-{camera.name}.jpg"), rgb[..., ::-1])
        frames[camera.name] = frame
    return frames


def run(args):
    import sapien
    args.reference, args.runtime, args.output = [p.resolve() for p in (args.reference, args.runtime, args.output)]
    args.output.mkdir(parents=True, exist_ok=False)
    source = Path(__file__).read_bytes()
    (args.output/"source-pgripper_vision_pick.py").write_bytes(source)
    task, ref, robot, joints, order, links, block = make_scene(args.reference, args.runtime)
    tcp = json.loads((args.reference/"vision_tcp.json").read_text())
    # Spawn randomization belongs to environment setup; it is never passed to localization or IK.
    rng = np.random.default_rng(args.seed)
    spawn = [float(rng.uniform(.175, .23)), float(rng.uniform(-.055, .055)), .02]
    yaw = float(rng.uniform(-np.deg2rad(8), np.deg2rad(8)))
    block.set_pose(sapien.Pose(spawn, np.roll(Rotation.from_euler("z", yaw).as_quat(), 1)))
    ref["randomization"] = dict(seed=args.seed, block_position_m=spawn, block_yaw_rad=yaw)
    for sample in ref["fk_samples"]:
        for side in ("left", "right"):
            actual = arm_fk(ref, sample["action"], side)
            expected = sample["poses"][side+"_gripper"]
            assert np.linalg.norm(actual[:3, 3]-expected[:3]) < 1e-6
            assert np.allclose(actual[:3, :3], transform(expected[:3], expected[3:])[:3, :3], atol=1e-6)
    pose_robot(robot, joints, ref, ref["home"])
    recorder = Recorder(task.scene, ref, links, block, robot, order, args.output, .002)
    side_index = 0 if args.arm == "left" else 1
    grip_index = side_index*6+5
    command = np.asarray(ref["home"]).copy()
    phase, step, hold_grip, deepest, peak, coupling_peak = "settle", 0, None, 0., 0., 0.
    detections, movements, hold_evidence = [], [], []
    joint_indices = {name: i for i, name in enumerate(joints)}

    def tick(target):
        nonlocal step, hold_grip, deepest, peak, coupling_peak
        target = np.asarray(target).copy()
        if hold_grip is not None:
            target[grip_index] = hold_grip
        limits = np.asarray(ref["ctrlrange"])
        if not np.isfinite(target).all() or np.any(target < limits[:, 0]-1e-8) or np.any(target > limits[:, 1]+1e-8):
            raise ValueError("Invalid bounded joint target")
        recorder.before_step(step, target, phase)
        for name, value in zip(ref["joints"], target):
            joints[name].set_drive_target(float(value))
        task.scene.step()
        step += 1
        forces, support, depth, forbidden = contacts(task.scene, .002)
        recorder.after_step(forces, support, depth)
        deepest, peak = max(deepest, depth), max(peak, float(forces.max()))
        values = robot.get_qpos()
        if not np.isfinite(values).all() or not np.isfinite(forces).all():
            raise ValueError("Nonfinite physics")
        for c in ref["couplings"]:
            error = abs(values[joint_indices[c["follower"]]]-c["factor"]*values[joint_indices[c["driver"]]]-c["offset"])
            coupling_peak = max(coupling_peak, float(error))
            if error > .0003:
                raise ValueError("Jaw coupling error")
        if forbidden or depth > .001 or forces.max() > 10:
            raise ValueError(f"Collision/contact guard: {forbidden}, {depth}, {forces.tolist()}")
        if hold_grip is not None:
            if forces[side_index].min() < .3:
                raise ValueError("Bilateral grip lost")
            if forces[side_index].min() < 1.5:
                hold_grip = max(0, hold_grip-(.08 if forces[side_index].min() < 1 else .02)*.002)
        return forces

    def move(name, target, minimum=1.):
        nonlocal phase, command
        phase = name
        if hold_grip is not None:
            command[grip_index] = target[grip_index] = hold_grip
        delta = float(np.max(np.abs(target-command)))
        duration = max(minimum, 2.1875*delta/.5, np.sqrt(8*delta/1.5), (53*delta/8)**(1/3))
        count = int(np.ceil(duration/.002))
        movements.append(dict(phase=phase, goal_rad=target.tolist(), start_step=step, duration_s=count*.002))
        print(phase, "duration", round(count*.002, 3), flush=True)
        start = command.copy()
        for i in range(count):
            t = (i+1)/count
            blend = t**4*(35-84*t+70*t*t-20*t**3)
            tick(start+(target-start)*blend)
        command = target.copy()
        for _ in range(100):
            tick(command)

    def capture():
        return observe(recorder, args.output, len(detections), args.blank_top,
                       args.arm+"_wrist_rgb" if args.blank_wrist else None)

    failure, success = None, False
    try:
        for _ in range(250):
            tick(command)
        phase = "rgbd locate"
        frames = capture()
        top = frames["top_h201_reference"]
        center, detected_yaw, evidence = estimate_rgbd(top["rgb"], top["depth"], top["intrinsic"], top["transform"])
        detections.append(dict(source="top RGB + metric depth", step=step, center_m=center.tolist(), yaw_rad=float(detected_yaw), **evidence))
        print("RGB-D center", center.tolist(), flush=True)
        phase = "plan approach"
        approach = None
        for clearance in (.06, .05, .04):
            candidates = []
            for angle in detected_yaw+np.arange(4)*np.pi/2:
                axis = np.array([np.cos(angle), np.sin(angle), 0.])
                try:
                    goal = target_action(ref, tcp, command, args.arm, center+[0, 0, clearance], jaw_axis=axis)
                    candidates.append((float(np.linalg.norm(goal-command)), goal, axis))
                except ValueError:
                    continue
            if candidates:
                _, approach, jaw_axis = min(candidates, key=lambda candidate: candidate[0])
                break
        if approach is None:
            raise ValueError("No reachable pregrasp with at least 20mm top-surface clearance")
        move("vision approach", approach)
        for iteration in range(3):
            phase = "wrist refine"
            frames = capture()
            wrist = frames[args.arm+"_wrist_rgb"]
            refined, evidence = wrist_center(wrist["rgb"], wrist["intrinsic"], wrist["transform"], center, detected_yaw)
            correction = refined-center
            if np.linalg.norm(correction) > .008:
                raise ValueError("Wrist correction exceeds 8mm")
            detections.append(dict(source=args.arm+" wrist RGB", step=step, center_m=refined.tolist(),
                correction_m=correction.tolist(), **evidence))
            print("wrist refinement", refined.tolist(), evidence, flush=True)
            center = refined
            move(f"wrist align {iteration+1}", target_action(ref, tcp, command, args.arm, center+[0, 0, .04], jaw_axis=jaw_axis), 1.5)
            if np.linalg.norm(correction) < .001:
                break
        phase = "wrist presence"
        frames = capture()
        visible = int(red_mask(frames[args.arm+"_wrist_rgb"]["rgb"]).sum())
        detections.append(dict(source=args.arm+" wrist RGB visibility", step=step, pixels=visible))
        move("vision descend", target_action(ref, tcp, command, args.arm, center+[0, 0, .01], jaw_axis=jaw_axis), 3.)
        phase = "close"
        stable = 0
        for _ in range(15000):
            force = contacts(task.scene, .002)[0][side_index]
            if force.min() < 1.5:
                command[grip_index] = max(0, command[grip_index]-(.2 if force.max() < .05 else .04)*.002)
            force = tick(command)[side_index]
            stable = stable+1 if force.min() >= 1 else 0
            if stable >= 165:
                hold_grip = command[grip_index]
                break
        if hold_grip is None:
            raise ValueError("Contact confirmation failed; lift forbidden")
        move("vision lift", target_action(ref, tcp, command, args.arm, center+[0, 0, .08], keep_vertical=False), 3.)
        phase = "hold"
        for _ in range(1500):
            forces = tick(command)
            # Object truth is restricted to this success verifier, never a motion target.
            hold_evidence.append(dict(z=float(block.pose.p[2]), forces=forces[side_index].tolist()))
            if block.pose.p[2] < .06:
                raise ValueError("Block did not stay lifted")
        success = True
    except (ValueError, AssertionError, RuntimeError) as error:
        failure = dict(phase=phase, reason=str(error))
    frames = recorder.finish(success)
    report = dict(success=success, failure=failure, seed=args.seed, arm=args.arm, frames=frames,
        physics_steps=step, simulation_seconds=step*.002, randomization=ref["randomization"],
        camera_driven=True, target_source="top RGB-D, wrist RGB + known 4cm geometry, encoder FK",
        object_ground_truth_used_for_control=False, object_attachments=False,
        object_pose_writes_after_initialization=0, runtime_qpos_writes_after_initialization=0,
        hardware_execution=False, learned_policy=False, detections=detections, movements=movements,
        hold_seconds=len(hold_evidence)*.002, hold_minimum_height_m=min((x["z"] for x in hold_evidence), default=None),
        maximum_penetration_m=deepest, maximum_pad_force_N=peak, maximum_coupling_error_m=coupling_peak,
        final_block_pose=np.r_[block.pose.p, block.pose.q].tolist(),
        source_sha256=hashlib.sha256(source).hexdigest(), blank_top=args.blank_top, blank_wrist=args.blank_wrist)
    (args.output/"report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k:report[k] for k in ("success","failure","arm","frames","simulation_seconds","hold_minimum_height_m")}, indent=2), flush=True)
    return 0 if success else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--arm", choices=("left","right"), default="left")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--blank-top", action="store_true")
    parser.add_argument("--blank-wrist", action="store_true")
    raise SystemExit(run(parser.parse_args()))
