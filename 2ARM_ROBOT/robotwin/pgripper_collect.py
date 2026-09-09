"""RoboTwin/SAPIEN PGripper data collection from a frozen MuJoCo reference.

Uses RoboTwin Base_Task scene setup; the expert and recording contract are local.
No serial, ROS, learned policy, object attachment, or runtime state teleportation.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation


def quat(rotation):
    return np.roll(rotation.as_quat(), 1)


def make_scene(reference_dir, runtime, seed=0, disable_recipient_contact=False):
    import sapien
    sys.path.insert(0, str(runtime))
    os.chdir(runtime)
    from envs._base_task import Base_Task

    reference = json.loads((reference_dir / "reference.json").read_text())
    task = Base_Task()
    task.task_name, task.random_light, task.render_freq = "dapier_pgripper", False, 0
    task.setup_scene(timestep=.002, ground_height=-1, static_friction=1, dynamic_friction=1)
    scene = task.scene
    builder = scene.create_articulation_builder()
    root = builder.create_link_builder()
    root.set_name("world")
    builders = {"world": root}
    materials = {}
    for body in reference["bodies"]:
        b = builder.create_link_builder(builders[body["parent"]])
        builders[body["name"]] = b
        b.set_name(body["name"])
        body_pose = sapien.Pose(body["pos"], body["quat"])
        j = body.get("joint")
        if j:
            axis = np.asarray(j["axis"])
            axis /= np.linalg.norm(axis)
            # Joint X is SAPIEN's native hinge/slide axis. Keep the CAD link frame.
            v = np.cross([1., 0, 0], axis)
            if np.linalg.norm(v) < 1e-8:
                alignment = Rotation.identity() if axis[0] > 0 else Rotation.from_rotvec([0, 0, np.pi])
            else:
                alignment = Rotation.from_rotvec(v / np.linalg.norm(v) * np.arccos(np.clip(axis[0], -1, 1)))
            child = sapien.Pose(j["pos"], quat(alignment))
            ref = sapien.Pose(q=quat(Rotation.from_rotvec([-j["ref"], 0, 0]))) if j["type"] == 3 else sapien.Pose([-j["ref"], 0, 0])
            b.set_joint_name(j["name"])
            b.set_joint_properties("revolute" if j["type"] == 3 else "prismatic", [j["limits"]],
                body_pose * child * ref, child, friction=j["friction"], damping=j["damping"])
        else:
            b.set_joint_properties("fixed", [], body_pose, sapien.Pose())
        b.set_mass_and_inertia(body["mass"], sapien.Pose(body["ipos"], body["iquat"]), body["inertia"])
        for g in body["geoms"]:
            pose = sapien.Pose(g["pos"], g["quat"])
            if g["collision"]:
                friction = g["friction"][0]
                if friction not in materials:
                    materials[friction] = scene.create_physical_material(friction, friction, 0)
                material = materials[friction]
                if g["type"] == 7:
                    b.add_convex_collision_from_file(str(reference_dir / g["mesh"]), pose=pose, material=material)
                elif g["type"] == 6:
                    b.add_box_collision(pose=pose, half_size=g["size"], material=material)
                else:
                    raise ValueError(f"Unsupported collision geometry: {g['name']}")
            elif g["rgba"][3] > 0:
                material = sapien.render.RenderMaterial(base_color=g["rgba"])
                if g["type"] == 7:
                    b.add_visual_from_file(str(reference_dir / g["mesh"]), pose=pose, material=material)
                elif g["type"] == 6:
                    b.add_box_visual(pose=pose, half_size=g["size"], material=material)
                elif g["type"] == 5:
                    pose = pose * sapien.Pose(q=quat(Rotation.from_euler("y", np.pi/2)))
                    b.add_cylinder_visual(pose=pose, radius=g["size"][0], half_length=g["size"][1], material=material)
    robot = builder.build(fix_root_link=True)
    links = {b.name: b for b in robot.get_links()}
    active = robot.get_active_joints()
    joints = {j.name: j for j in active}
    order = [list(joints).index(n) for n in reference["joints"]]
    for c in reference["couplings"]:
        driver, follower = joints[c["driver"]], joints[c["follower"]]
        assert driver.parent_link == follower.parent_link
        # Native tendon: follower - factor*driver = offset, including motor ref.
        # Force coefficients follow its gradient: .0115 Nm per N on the motor.
        robot.create_fixed_tendon([driver.parent_link, driver.child_link, follower.child_link],
            [0, -c["factor"], 1], [0, -c["factor"], 1], rest_length=c["offset"], stiffness=1e5, damping=20)
    by_joint = {b["joint"]["name"]: b["joint"] for b in reference["bodies"] if "joint" in b}
    for name, joint in joints.items():
        spec = by_joint[name]
        joint.set_friction(spec["friction"])
        joint.set_armature([spec["armature"]])
        joint.set_drive_property(0, spec["damping"])
    for drive in reference["drives"]:
        joint = joints[drive["name"]]
        joint.set_drive_property(drive["kp"], drive["kv"] + by_joint[joint.name]["damping"], force_limit=drive["force_limit"])
    for link in links.values():
        for shape in link.get_collision_shapes():
            shape.set_contact_offset(.001)
    profile = reference["profile"]
    if disable_recipient_contact:
        for finger in (1, 2):
            for shape in links[f"right_pgripper_jaw_{finger}"].get_collision_shapes():
                shape.set_collision_groups([0, 0, 0, 0])
    def box(name, size, position, color, dynamic=False):
        b = scene.create_actor_builder()
        material = scene.create_physical_material(profile["block_friction"][0] if dynamic else 1,
                                                  profile["block_friction"][0] if dynamic else 1, 0)
        b.add_box_collision(half_size=np.asarray(size)/2, material=material)
        b.add_box_visual(half_size=np.asarray(size)/2, material=color)
        actor = b.build(name) if dynamic else b.build_static(name)
        actor.set_pose(sapien.Pose(position))
        return actor
    size = profile["table_size_m"]
    box("table", size, [profile["table_front_edge_x_m"]+size[0]/2, 0, -size[2]/2], [.9, .9, .88])
    position = np.asarray(profile["reference_block_center_m"]).copy()
    rng = np.random.default_rng(seed)
    yaw = 0.
    if seed:
        position[:2] += rng.uniform(-.001, .001, 2)
        yaw = rng.uniform(-np.deg2rad(1), np.deg2rad(1))
    block = box("red_block", profile["reference_block_size_m"], position, [.65, .03, .025], True)
    block.set_pose(sapien.Pose(position, quat(Rotation.from_euler("z", yaw))))
    reference["randomization"] = dict(seed=seed, block_position_m=position.tolist(), block_yaw_rad=float(yaw))
    body = block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
    body.mass = profile["block_mass_kg"]
    body.inertia = np.full(3, body.mass * .04**2/6)
    return task, reference, robot, joints, order, links, block


def pose_robot(robot, joints, reference, action):
    """Initialization/FK verification only; never call inside a physics rollout."""
    values = dict(zip(reference["joints"], action))
    for c in reference["couplings"]:
        values[c["follower"]] = c["factor"] * values[c["driver"]] + c["offset"]
    robot.set_qpos([values[j] for j in joints])
    robot.set_qvel(np.zeros(len(joints)))


def check_fk(robot, joints, reference, links):
    errors, angles = [], []
    for sample in reference["fk_samples"]:
        pose_robot(robot, joints, reference, sample["action"])
        for name, pose in sample["poses"].items():
            p = links[name].entity_pose
            errors.append(float(np.linalg.norm(p.p - pose[:3])))
            angles.append(float(min(np.linalg.norm(p.q - pose[3:]), np.linalg.norm(p.q + np.asarray(pose[3:])))))
    result = dict(samples=len(reference["fk_samples"]), links=len(reference["bodies"]),
                  max_position_error_m=max(errors), max_quaternion_error=max(angles))
    assert max(errors) < 1e-6 and max(angles) < 1e-5, result
    return result


def contacts(scene, dt):
    forces = np.zeros((2, 2))
    support, penetration = 0., 0.
    forbidden = []
    for c in scene.get_contacts():
        names = [b.entity.name for b in c.bodies]
        impulse = sum(np.linalg.norm(p.impulse) for p in c.points)
        depth = max([0.] + [-float(p.separation) for p in c.points])
        penetration = max(penetration, depth)
        if "red_block" in names:
            other = names[1-names.index("red_block")]
            if other == "table":
                support += sum(abs(float(p.impulse[2])) for p in c.points) / dt
            for side, prefix in enumerate(("left", "right")):
                for finger in (1, 2):
                    if other == f"{prefix}_pgripper_jaw_{finger}":
                        forces[side, finger-1] += sum(abs(float(np.dot(p.impulse, p.normal))) for p in c.points) / dt
        sides = [n.split("_", 1)[0] for n in names]
        if sides[0] == sides[1] and sides[0] in ("left", "right") and (depth > .0001 or impulse > 1e-6):
            forbidden.append(names)
        if {sides[0], sides[1]} == {"left", "right"} and (depth > .0001 or impulse > 1e-6):
            forbidden.append(names)
        if "table" in names:
            other = names[1-names.index("table")]
            if other.startswith(("left_", "right_")) and not other.endswith("_base") and (depth > .0001 or impulse > 1e-6):
                forbidden.append(names)
    return forces, support, penetration, forbidden



class Recorder:
    """Record pre-action observations plus every applied 500 Hz substep command."""
    def __init__(self, scene, reference, links, block, robot, order, output, dt):
        import cv2
        import h5py
        import sapien
        self.scene, self.robot, self.order, self.block = scene, robot, order, block
        self.dt, self.output, self.frames, self.last_contacts = dt, output, 0, None
        self.file = h5py.File(output / "episode.hdf5", "x")
        self.datasets, self.pending = {}, {}
        self.file.attrs.update(schema="dapier.pgripper.sapien.v1", control_hz=500, observation_hz=25,
            ordering="left5, left_gripper, right5, right_gripper",
            action_semantics="observation precedes action; expert/action_rad contains every 2ms applied command",
            gripper_semantics="raw motor rad, opening-positive 0..2.2028; normalized policy field divides by 2.2028",
            grippers="both NORMA PGripper", simulator_truth_group="expert", hardware_execution=False)
        self.file.attrs["reference_sha256"] = hashlib.sha256(json.dumps(reference, sort_keys=True).encode()).hexdigest()
        self.file.attrs["randomization"] = json.dumps(reference["randomization"])
        self.cameras = []
        # MuJoCo: right +X, up +Y, view -Z; SAPIEN: view +X, left +Y, up +Z.
        conversion = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
        for spec in reference["cameras"]:
            cam = scene.add_camera(spec["name"], 320, 240, np.deg2rad(spec["fovy"]), .01, 10)
            rotation = Rotation.from_quat(np.roll(spec["quat"], -1)).as_matrix() @ conversion
            local = sapien.Pose(spec["pos"], quat(Rotation.from_matrix(rotation)))
            parent = links.get(spec["body"])
            self.cameras.append((cam, parent, local))
            group = self.file.require_group("camera_metadata/" + spec["name"])
            group.create_dataset("intrinsics", data=cam.get_intrinsic_matrix())
            group.attrs["parent_body"] = spec["body"]
            group.attrs["local_transform_convention"] = "SAPIEN: forward +X, left +Y, up +Z"
            group.attrs["world_transform_convention"] = "OpenGL: right +X, up +Y, view -Z"
            group.create_dataset("local_transform", data=local.to_transformation_matrix())
            self.datasets["observation/images/"+spec["name"]] = self.file.create_dataset(
                "observation/images/"+spec["name"], shape=(0,), maxshape=(None,),
                dtype=h5py.vlen_dtype(np.dtype("uint8")))
        self.overview = scene.add_camera("overview", 640, 480, np.deg2rad(48), .01, 10)
        eye, target = np.array([.78, .72, .57]), np.array([.19, 0, .14])
        forward = target-eye
        forward /= np.linalg.norm(forward)
        left = np.cross([0, 0, 1], forward)
        left /= np.linalg.norm(left)
        self.overview.set_entity_pose(sapien.Pose(eye, quat(Rotation.from_matrix(np.column_stack([forward, left, np.cross(forward, left)])))))
        self.video = cv2.VideoWriter(str(output/"preview.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 25, (640, 480))
        if not self.video.isOpened():
            raise RuntimeError("Video writer failed")
        self.snapshots = set()

    def append(self, name, value):
        value = np.asarray(value)
        if "500hz" in name or name == "expert/action_rad":
            self.pending.setdefault(name, []).append(value.copy())
            return
        if name not in self.datasets:
            self.datasets[name] = self.file.create_dataset(name, shape=(0, *value.shape),
                maxshape=(None, *value.shape), dtype=value.dtype, chunks=(1, *value.shape),
                compression="gzip" if value.ndim >= 2 else None)
        dataset = self.datasets[name]
        dataset.resize(len(dataset)+1, axis=0)
        dataset[-1] = value

    def before_step(self, step, target, phase):
        import cv2
        if step % 20 == 0:
            state = self.robot.get_qpos()[self.order]
            self.append("observation/state_rad", state)
            normalized = state.copy()
            normalized[[5, 11]] /= 2.2028
            self.append("observation/state", normalized)
            self.append("action_rad", target)
            normalized_action = target.copy()
            normalized_action[[5, 11]] /= 2.2028
            self.append("action", normalized_action)
            self.append("timestamp_s", step*self.dt)
            self.append("physics_step", step)
            self.append("expert/block_pose", np.r_[self.block.pose.p, self.block.pose.q])
            self.append("expert/contact_valid", step > 0)
            f, support, depth = self.last_contacts if self.last_contacts is not None else (np.full((2, 2), np.nan), np.nan, np.nan)
            self.append("expert/pad_force_N", f)
            self.append("expert/table_support_N", support)
            self.append("expert/penetration_m", depth)
            self.append("expert/phase", np.bytes_(phase.ljust(32)))
            for cam, parent, local in self.cameras:
                cam.set_entity_pose(local if parent is None else parent.entity_pose*local)
            self.scene.update_render()
            for cam, _, _ in self.cameras:
                cam.take_picture()
                rgb = np.clip(cam.get_picture("Color")[..., :3]*255, 0, 255).astype(np.uint8)
                ok, encoded = cv2.imencode(".jpg", rgb[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not ok:
                    raise RuntimeError("JPEG encoding failed")
                self.append("observation/images/"+cam.name, encoded)
                self.append("camera_metadata/"+cam.name+"/world_transform", cam.get_model_matrix())
                if cam.name == "top_h201_reference":
                    position = cam.get_picture("Position")
                    depth = np.maximum(0, -position[..., 2]).astype(np.float32)
                    depth[position[..., 3] >= 1] = 0
                    self.append("observation/depth_m", depth)
            self.overview.take_picture()
            bgr = np.clip(self.overview.get_picture("Color")[..., :3][..., ::-1]*255, 0, 255).astype(np.uint8)
            cv2.putText(bgr, f"SIM | {phase} | {step*self.dt:.1f}s", (12, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (245, 245, 245), 1, cv2.LINE_AA)
            self.video.write(bgr)
            if phase not in self.snapshots:
                cv2.imwrite(str(self.output / (phase.replace(" ", "-")+".jpg")), bgr)
                self.snapshots.add(phase)
            self.frames += 1
        self.append("expert/action_rad", target)
        self.append("expert/phase_500hz", np.bytes_(phase.ljust(32)))

    def after_step(self, forces, support, depth):
        self.last_contacts = forces.copy(), support, depth
        self.append("expert/pad_force_500hz_N", forces)
        self.append("expert/table_support_500hz_N", support)
        self.append("expert/penetration_500hz_m", depth)
        self.append("expert/state_500hz_rad", self.robot.get_qpos()[self.order])
        self.append("expert/block_pose_500hz", np.r_[self.block.pose.p, self.block.pose.q])

    def finish(self, success):
        for name, values in self.pending.items():
            self.file.create_dataset(name, data=np.asarray(values), compression="gzip", shuffle=True)
        self.file.attrs["success"] = bool(success)
        self.file.attrs["frames"] = self.frames
        self.file.flush()
        self.file.close()
        self.video.release()
        return self.frames


def main(args):
    args.reference, args.runtime, args.output = [p.resolve() for p in (args.reference, args.runtime, args.output)]
    args.output.mkdir(parents=True, exist_ok=False)
    source_bytes = Path(__file__).read_bytes()
    (args.output / "source-pgripper_collect.py").write_bytes(source_bytes)
    source_hash = hashlib.sha256(source_bytes).hexdigest()
    task, ref, robot, joints, order, links, block = make_scene(args.reference, args.runtime, args.seed, args.disable_recipient_contact)
    fk = check_fk(robot, joints, ref, links)
    print("FK", fk, flush=True)
    (args.output / "fk.json").write_text(json.dumps(fk, indent=2))
    if args.fk_only:
        return 0
    pose_robot(robot, joints, ref, ref["home"])
    scene, dt = task.scene, .002
    command = np.asarray(ref["home"]).copy()
    hold_grips = {}
    phase, step, deepest, peak = "settle", 0, 0., 0.
    trace = []
    recorder = Recorder(scene, ref, links, block, robot, order, args.output, dt) if args.record else None
    hold_steps, table_hold_steps = 0, 0
    maximum_coupling_error = 0.
    phase_events = []
    q_indices = {name: i for i, name in enumerate(joints)}
    plans = {p["phase"]: p for p in json.loads((args.reference / "mujoco_baseline/report.json").read_text())["plans"]}

    def tick(target):
        nonlocal step, deepest, peak, maximum_coupling_error
        target = np.asarray(target).copy()
        for side, grip in hold_grips.items():
            target[side*6+5] = grip
        limits = np.asarray(ref["ctrlrange"])
        assert target.shape == (12,) and np.isfinite(target).all()
        assert np.all(target >= limits[:, 0]-1e-8) and np.all(target <= limits[:, 1]+1e-8)
        if recorder is not None:
            recorder.before_step(step, target, phase)
        for name, value in zip(ref["joints"], target):
            joints[name].set_drive_target(float(value))
        scene.step()
        step += 1
        forces, support, depth, forbidden = contacts(scene, dt)
        deepest, peak = max(deepest, depth), max(peak, float(forces.max()))
        all_q = robot.get_qpos()
        q = all_q[order]
        for coupling in ref["couplings"]:
            error = abs(float(all_q[q_indices[coupling["follower"]]] -
                        coupling["factor"]*all_q[q_indices[coupling["driver"]]] - coupling["offset"]))
            maximum_coupling_error = max(maximum_coupling_error, error)
            if error > .0003:
                raise ValueError(f"Jaw coupling error {error:.6f} m")
        if recorder is not None:
            recorder.after_step(forces, support, depth)
        if not np.isfinite(q).all() or not np.isfinite(block.pose.p).all():
            raise ValueError("Nonfinite simulation state")
        if forbidden:
            raise ValueError(f"Forbidden collision: {forbidden}")
        if depth > .001:
            raise ValueError(f"Penetration {depth:.6f} m")
        if phase in ("place release", "place retreat") and support < .05:
            raise ValueError("Table support lost during release or retreat")
        if forces.max() > 10:
            raise ValueError(f"Pad overload: {forces.tolist()}")
        for side in hold_grips:
            if forces[side].min() < .3:
                raise ValueError(f"Grasp lost: {side}, {forces.tolist()}")
            if forces[side].min() < 1.5:
                hold_grips[side] = max(0, hold_grips[side] - (.08 if forces[side].min() < 1 else .02)*dt)
        if step % 10 == 0:
            trace.append(dict(time_s=step*dt, phase=phase, state_rad=q.tolist(), action_rad=target.tolist(),
                forces_N=forces.tolist(), support_N=support, block_pose=np.r_[block.pose.p, block.pose.q].tolist()))
        return forces, support

    def move(name, goal=None):
        nonlocal command, phase
        phase = name
        phase_events.append(dict(phase=phase, start_step=step))
        spec = plans[name]
        target = np.asarray(spec["goal_rad"] if goal is None else goal).copy()
        # Other-hand grasp corrections must survive precomputed whole-arm waypoints.
        for side, grip in hold_grips.items():
            command[side*6+5] = target[side*6+5] = grip
        count = int(np.ceil(spec["duration_s"]/dt))
        print(name, "duration", count*dt, flush=True)
        start = command.copy()
        for i in range(count):
            t = (i+1)/count
            blend = t**4*(35 - 84*t + 70*t*t - 20*t**3)
            _, support = tick(start + (target-start)*blend)
            if name == "place seat" and support >= .1:
                command = start + (target-start)*blend
                return
        command = target
        if name == "place seat":
            raise ValueError("No table support; release forbidden")

    def close(side):
        nonlocal phase
        phase = "donor close" if side == 0 else "recipient close"
        stable = 0.
        print(phase, flush=True)
        for _ in range(int(45/dt)):
            f = contacts(scene, dt)[0][side]
            if f.min() < 1.5:
                command[side*6+5] = max(0, command[side*6+5] - (.2 if f.max() < .05 else .04)*dt)
            f = tick(command)[0][side]
            stable = stable+dt if f.min() >= 1 else 0
            if stable >= .33:
                hold_grips[side] = command[side*6+5]
                phase_events.append(dict(phase=phase, accepted_step=step, stable_s=stable, forces_N=f.tolist()))
                print("grasp", side, f.tolist(), flush=True)
                return
        raise ValueError(f"Bilateral contact timeout: {f.tolist()}")

    success, failure = False, None
    try:
        for _ in range(250):
            tick(command)
        move("donor approach")
        move("donor descend")
        close(0)
        for name in ("donor lift", "donor present", "recipient orient away", "recipient approach", "recipient insert"):
            move(name)
        close(1)
        hold_grips.pop(0)
        move("donor release")
        if contacts(scene, dt)[0][0].max() > .01:
            raise ValueError("Donor contact remains after opening")
        if args.drop_recipient_contact:
            for finger in (1, 2):
                for shape in links[f"right_pgripper_jaw_{finger}"].get_collision_shapes():
                    shape.set_collision_groups([0, 0, 0, 0])
        move("donor retreat")
        phase = "recipient hold"
        for _ in range(1500):
            f, _ = tick(command)
            if f[0].max() > .01 or block.pose.p[2] < .06:
                raise ValueError("Recipient-only hold failed")
            hold_steps += 1
        for name in ("recipient carry", "place approach", "place seat"):
            move(name)
        phase = "place support confirm"
        for _ in range(100):
            if tick(command)[1] < .05:
                raise ValueError("Table support lost; release forbidden")
        hold_grips.pop(1)
        move("place release")
        move("place retreat")
        phase = "table hold"
        import sapien
        block_body = block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        resting_position = block.pose.p.copy()
        resting_quaternion = block.pose.q.copy()
        for _ in range(1500):
            f, support = tick(command)
            if (f.max() > .01 or support < .1 or not .019 <= block.pose.p[2] <= .022
                    or np.linalg.norm(block_body.linear_velocity) > .01
                    or np.linalg.norm(block_body.angular_velocity) > .05
                    or np.linalg.norm(block.pose.p-resting_position) > .001
                    or min(np.linalg.norm(block.pose.q-resting_quaternion), np.linalg.norm(block.pose.q+resting_quaternion)) > .01):
                raise ValueError("Released block not resting on table")
            table_hold_steps += 1
        success = True
    except (ValueError, AssertionError, RuntimeError) as error:
        failure = dict(phase=phase, reason=str(error))
    report = dict(success=success, failure=failure, physics_steps=step, simulation_seconds=step*dt,
        maximum_penetration_m=deepest, maximum_pad_force_N=peak, final_block_pose=np.r_[block.pose.p, block.pose.q].tolist(),
        final_joint_position=dict(zip(joints, robot.get_qpos().tolist())),
        source_hashes=ref["source_hashes"], geometry="MuJoCo both PGripper frozen mesh adaptation",
        engine="RoboTwin Base_Task + SAPIEN/PhysX", hardware_execution=False,
        sapien_version=__import__("sapien").__version__, seed=args.seed, randomization=ref["randomization"],
        recipient_contact_disabled=args.disable_recipient_contact, recipient_contact_drop=args.drop_recipient_contact,
        recipient_only_hold_s=hold_steps*dt, table_rest_s=table_hold_steps*dt,
        maximum_coupling_error_m=maximum_coupling_error, events=phase_events,
        control_hz=500, observation_hz=25, camera_driven=False, learned_policy=False,
        expert="frozen MuJoCo IK waypoints + SAPIEN contact feedback",
        collector_sha256=source_hash,
        runtime_qpos_writes=0, object_pose_writes_after_initialization=0, object_attachments=False)
    if recorder is not None:
        report["frames"] = recorder.finish(success)
    (args.output / "trace.json").write_text(json.dumps(trace))
    (args.output / "report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k: report[k] for k in ("success", "failure", "physics_steps", "simulation_seconds", "maximum_penetration_m", "maximum_coupling_error_m")}, indent=2), flush=True)
    return 0 if success else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fk-only", action="store_true")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--disable-recipient-contact", action="store_true")
    parser.add_argument("--drop-recipient-contact", action="store_true")
    raise SystemExit(main(parser.parse_args()))
