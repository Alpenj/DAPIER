"""Small-object bimanual handover sized for the real Dual SO-101 workspace."""

from copy import deepcopy

import numpy as np
import torch
import transforms3d as t3d
from curobo.geom.types import Cuboid
from curobo.types.math import Pose as CuroboPose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig, PoseCostMetric

from envs.handover_block import handover_block
from envs.utils import ArmTag, create_box, create_cylinder, rand_pose
from envs.utils.actor_utils import Actor


class dapier_handover_block(handover_block):
    def setup_demo(self, **kwargs):
        # Keep RoboTwin's table at world z=0.74 and place the robot root there.
        # Relative to the robot this is the real mobile-base ground plane.
        self.path_limits = kwargs["left_embodiment_config"]["joint_path_limits"]
        if any(float(value) <= 0 for value in self.path_limits.values()):
            raise ValueError("joint_path_limits values must be positive")
        self.pregrasp_clearance_m = float(kwargs["pregrasp_clearance_m"])
        if self.pregrasp_clearance_m <= 0:
            raise ValueError("pregrasp_clearance_m must be positive")
        self.grasp_height_offset_m = float(kwargs["grasp_height_offset_m"])
        if abs(self.grasp_height_offset_m) > 0.1:
            raise ValueError("grasp_height_offset_m must be within 0.1 m")
        self.grasp_lateral_offset_m = float(kwargs["grasp_lateral_offset_m"])
        if abs(self.grasp_lateral_offset_m) > 0.1:
            raise ValueError("grasp_lateral_offset_m must be within 0.1 m")
        self.grasp_gripper_position = float(kwargs["grasp_gripper_position"])
        if not 0 <= self.grasp_gripper_position <= 1:
            raise ValueError("grasp_gripper_position must be in [0, 1]")
        self.min_approach_alignment = float(kwargs["min_approach_alignment"])
        self.min_jaw_axis_alignment = float(kwargs["min_jaw_axis_alignment"])
        if not all(
            -1 <= value <= 1
            for value in (
                self.min_approach_alignment,
                self.min_jaw_axis_alignment,
            )
        ):
            raise ValueError("alignment thresholds must be in [-1, 1]")
        self._pregrasp_obstacle = None
        super()._init_task_env_(**kwargs)
        # Scale ALOHA's handover point to the shorter, symmetric SO-101 reach.
        self.block_middle_pose = [-0.035, 0.05, 0.82, 0, 1, 0, 0]

    def get_obs(self):
        observation = super().get_obs()
        measured = {}
        for side in ("left", "right"):
            entity = getattr(self.robot, f"{side}_entity")
            names = [joint.get_name() for joint in entity.get_active_joints()]
            qpos = entity.get_qpos()
            arm_names = getattr(self.robot, f"{side}_arm_joints_name")
            arm = [float(qpos[names.index(name)]) for name in arm_names]
            scale = getattr(self.robot, f"{side}_gripper_scale")
            raw_gripper = float(qpos[names.index(f"{side}_gripper")])
            gripper = float(
                np.clip(
                    (raw_gripper - scale[0]) / (scale[1] - scale[0]), 0, 1
                )
            )
            measured[side] = (arm, gripper)
        observation["joint_action"].update(
            left_arm=np.asarray(measured["left"][0]),
            left_gripper=measured["left"][1],
            right_arm=np.asarray(measured["right"][0]),
            right_gripper=measured["right"][1],
            vector=np.asarray(
                measured["left"][0]
                + [measured["left"][1]]
                + measured["right"][0]
                + [measured["right"][1]]
            ),
        )
        return observation

    def _five_dof_plan(self, arm_tag, target_pose, use_graph=False):
        """Solve position + approach-direction IK, then validate a smooth path."""
        robot = self.robot
        planner = robot.left_planner if arm_tag == "left" else robot.right_planner
        entity = robot.left_entity if arm_tag == "left" else robot.right_entity
        joint_names = (
            robot.left_arm_joints_name
            if arm_tag == "left"
            else robot.right_arm_joints_name
        )
        target = robot._trans_from_gripper_to_endlink(target_pose, arm_tag=arm_tag)
        base = (
            robot.left_entity_origion_pose
            if arm_tag == "left"
            else robot.right_entity_origion_pose
        )
        target_position, target_quaternion = planner._trans_from_world_to_base(
            np.r_[base.p, base.q], np.r_[target.p, target.q]
        )
        goal = CuroboPose.from_list(
            target_position.tolist() + target_quaternion.tolist()
        )

        active_names = [joint.get_name() for joint in entity.get_active_joints()]
        qpos = entity.get_qpos()
        planner.motion_gen.update_locked_joints(
            {
                name: float(qpos[index])
                for index, name in enumerate(active_names)
                if name not in joint_names
            },
            planner.yml_path,
        )
        current = torch.tensor(
            [[round(float(qpos[active_names.index(name)]), 5) for name in joint_names]],
            device="cuda",
        )
        ik = planner.motion_gen.ik_solver
        ik_filter_counts = {}

        def solve_candidates(label, weights):
            ik.update_pose_cost_metric(
                PoseCostMetric(
                    reach_partial_pose=True,
                    reach_vec_weight=planner.motion_gen.tensor_args.to_device(weights),
                )
            )
            try:
                result = ik.solve_single(
                    goal,
                    retract_config=current,
                    seed_config=current[:, None, :],
                    return_seeds=64,
                    num_seeds=256,
                )
            finally:
                ik.update_pose_cost_metric(PoseCostMetric.reset_metric())
            candidates = result.solution[0]
            states = JointState.from_position(candidates, joint_names=joint_names)
            rollout = ik.rollout_fn
            primitive = rollout.primitive_collision_constraint
            self_collision = rollout.robot_self_collision_constraint

            def constraint_mask(use_world, use_self):
                (primitive.enable_cost() if use_world else primitive.disable_cost())
                (self_collision.enable_cost() if use_self else self_collision.disable_cost())
                return planner.motion_gen.check_constraints(states).feasible.flatten()

            try:
                joint_ok = constraint_mask(False, False)
                world_ok = constraint_mask(True, False)
                self_ok = constraint_mask(False, True)
                feasible = constraint_mask(True, True)
            finally:
                primitive.enable_cost()
                self_collision.enable_cost()
            position_ok = result.position_error[0] <= ik.position_threshold
            ik_filter_counts[label] = dict(
                returned=len(candidates),
                position_ok=int(position_ok.sum().item()),
                joint_ok=int(joint_ok.sum().item()),
                world_ok=int(world_ok.sum().item()),
                self_ok=int(self_ok.sum().item()),
                feasible=int(feasible.sum().item()),
                accepted=int((feasible & position_ok).sum().item()),
            )
            return candidates[feasible & position_ok]

        # CuRobo's result.success still applies a full-orientation threshold
        # to partial-pose IK. Select position-converged, collision-free
        # candidates ourselves, then rank their real FK orientation.
        axis_constrained = solve_candidates(
            "axis_constrained", [1, 1, 0, 1, 1, 1]
        )
        position_only = solve_candidates("position_only", [0, 0, 0, 1, 1, 1])
        position_only = torch.cat((axis_constrained, position_only))
        position_only_raw = len(position_only)
        best_raw_alignment = None
        delta_matrix = (
            robot.left_delta_matrix
            if arm_tag == "left"
            else robot.right_delta_matrix
        )
        desired_rotation = (
            t3d.quaternions.quat2mat(target_quaternion) @ delta_matrix
        )
        if len(position_only):
            kinematics = planner.motion_gen.compute_kinematics(
                JointState.from_position(position_only, joint_names=joint_names)
            )
            scores = []
            keep = []
            for index, quaternion in enumerate(
                kinematics.ee_pose.quaternion.cpu().numpy()
            ):
                rotation = t3d.quaternions.quat2mat(quaternion) @ delta_matrix
                approach = float(
                    np.dot(desired_rotation[:, 0], rotation[:, 0])
                )
                jaw = float(np.dot(desired_rotation[:, 1], rotation[:, 1]))
                distance = torch.linalg.vector_norm(
                    position_only[index] - current[0]
                ).item()
                raw_score = 2 * approach + jaw - 0.05 * distance
                if best_raw_alignment is None or raw_score > best_raw_alignment[0]:
                    best_raw_alignment = (
                        raw_score,
                        approach,
                        jaw,
                        rotation[:, 0].tolist(),
                        rotation[:, 1].tolist(),
                    )
                if (
                    approach >= self.min_approach_alignment
                    and jaw >= self.min_jaw_axis_alignment
                ):
                    keep.append(index)
                    scores.append(2 * approach + jaw - 0.05 * distance)
            order = np.argsort(-np.asarray(scores)) if scores else []
            position_only = position_only[
                torch.as_tensor(
                    [keep[index] for index in order],
                    device=position_only.device,
                    dtype=torch.long,
                )
            ]

        limits = self.path_limits

        def segment(start, end):
            delta = end - start
            distance = torch.abs(delta).max().item()
            duration = max(
                0.5,
                1.875 * distance / limits["max_velocity_rad_s"],
                (5.773503 * distance / limits["max_acceleration_rad_s2"]) ** 0.5,
                (60.0 * distance / limits["max_jerk_rad_s3"]) ** (1.0 / 3.0),
            )
            steps = int(np.ceil(duration * limits["control_hz"])) + 1
            phase = torch.linspace(0, 1, steps, device="cuda")
            blend = 10 * phase**3 - 15 * phase**4 + 6 * phase**5
            path = start + blend[:, None] * delta
            states = JointState.from_position(path, joint_names=joint_names)
            feasible = planner.motion_gen.check_constraints(states).feasible.flatten()
            if not feasible.all():
                first_clear = torch.nonzero(feasible, as_tuple=False)
                if (
                    not len(first_clear)
                    or first_clear[0].item() > len(feasible) // 4
                    or not feasible[first_clear[0].item() :].all()
                ):
                    return None
                print(
                    f"DAPIER_EXIT_CONTACT_PATH: arm={arm_tag} "
                    f"clear_at={first_clear[0].item()}/{len(feasible)}",
                    flush=True,
                )
            blend_rate = (30 * phase**2 - 60 * phase**3 + 30 * phase**4) / duration
            return path, blend_rate[:, None] * delta

        def finish_path(path, velocity, candidate):
            settle_steps = int(np.ceil(limits["settle_s"] * limits["control_hz"]))
            if settle_steps:
                path = torch.cat((path, candidate.repeat(settle_steps, 1)))
                velocity = torch.cat((velocity, torch.zeros_like(path[-settle_steps:])))
            return {
                "status": "Success",
                "position": path.cpu().numpy(),
                "velocity": velocity.cpu().numpy(),
            }

        def finish(segments, candidate):
            path = torch.cat(
                [
                    part[0] if index == 0 else part[0][1:]
                    for index, part in enumerate(segments)
                ]
            )
            velocity = torch.cat(
                [
                    part[1] if index == 0 else part[1][1:]
                    for index, part in enumerate(segments)
                ]
            )
            return finish_path(path, velocity, candidate)

        current_q = current[0]
        for candidate in position_only:
            direct = segment(current_q, candidate)
            if direct is not None:
                return finish([direct], candidate)

        if use_graph:
            start = JointState.from_position(current, joint_names=joint_names)
            for candidate in position_only:
                graph = planner.motion_gen.plan_single_js(
                    start,
                    JointState.from_position(
                        candidate[None], joint_names=joint_names
                    ),
                    MotionGenPlanConfig(
                        max_attempts=1, enable_graph=True, enable_opt=False
                    ),
                )
                if graph.success.item():
                    plan = graph.interpolated_plan
                    states = JointState.from_position(
                        plan.position, joint_names=joint_names
                    )
                    if not planner.motion_gen.check_constraints(states).feasible.all():
                        continue
                    connector = segment(current_q, plan.position[0])
                    if connector is None:
                        continue
                    return finish(
                        [connector, (plan.position, plan.velocity)], candidate
                    )
        print(
            f"DAPIER_5DOF_PATH_FAIL: arm={arm_tag} "
            f"position_only={len(position_only)}/"
            f"{position_only_raw} filters={ik_filter_counts} "
            f"best_alignment={best_raw_alignment}",
            flush=True,
        )
        return {"status": "Fail"}

    def _move_to_pose(self, arm_tag, pose):
        if not self.plan_success:
            return None
        paths = self.left_joint_path if arm_tag == "left" else self.right_joint_path
        counter = self.left_cnt if arm_tag == "left" else self.right_cnt
        if self.need_plan:
            planner = (
                self.robot.left_planner
                if arm_tag == "left"
                else self.robot.right_planner
            )
            static_world = None
            actor = self._pregrasp_obstacle
            self._pregrasp_obstacle = None
            if actor is not None:
                static_world = planner.motion_gen.world_model.clone()
                actor_pose = actor.get_pose()
                base = (
                    self.robot.left_entity_origion_pose
                    if arm_tag == "left"
                    else self.robot.right_entity_origion_pose
                )
                position, quaternion = planner._trans_from_world_to_base(
                    np.r_[base.p, base.q], np.r_[actor_pose.p, actor_pose.q]
                )
                world = static_world.clone()
                world.cuboid.append(
                    Cuboid(
                        name="dapier_pregrasp_obstacle",
                        pose=position.tolist() + quaternion.tolist(),
                        dims=(2 * np.asarray(actor.config["extents"])).tolist(),
                    )
                )
                planner.motion_gen.update_world(world)
            try:
                result = self._five_dof_plan(arm_tag, pose, use_graph=True)
            finally:
                if static_world is not None:
                    planner.motion_gen.update_world(static_world)
            paths.append(deepcopy(result))
        else:
            result = deepcopy(paths[counter])
            if arm_tag == "left":
                self.left_cnt += 1
            else:
                self.right_cnt += 1
        if result["status"] != "Success":
            self.plan_success = False
            return None
        return result

    def left_move_to_pose(self, pose, **kwargs):
        return self._move_to_pose("left", pose)

    def right_move_to_pose(self, pose, **kwargs):
        return self._move_to_pose("right", pose)

    def grasp_actor(
        self,
        actor,
        arm_tag,
        pre_grasp_dis=0.1,
        grasp_dis=0,
        gripper_pos=0.0,
        contact_point_id=None,
    ):
        gripper_pos = self.grasp_gripper_position
        action = super().grasp_actor(
            actor,
            arm_tag,
            pre_grasp_dis,
            grasp_dis,
            gripper_pos,
            contact_point_id,
        )
        if self.need_plan and arm_tag == "left" and pre_grasp_dis != grasp_dis:
            self._pregrasp_obstacle = actor
        return action

    def play_once(self):
        left, right = ArmTag("left"), ArmTag("right")
        self._handover_verified = False
        self.box.actor.set_pose(
            rand_pose(
                xlim=[-0.0685],
                ylim=[0.0307],
                zlim=[0.820001],
                qpos=[2**-0.5, 0, 2**-0.5, 0],
            )
        )
        for component in self.box.actor.get_components():
            if hasattr(component, "linear_velocity"):
                component.linear_velocity = np.zeros(3)
                component.angular_velocity = np.zeros(3)
        if not self.move(
            self.grasp_actor(
                self.box,
                left,
                pre_grasp_dis=self.pregrasp_clearance_m,
                contact_point_id=[0, 1, 2, 3],
            )
        ):
            return self.info
        if not self.move(self.move_by_displacement(left, z=0.08)):
            return self.info

        expected = np.array(
            [-0.035, 0.071, self.block_middle_pose[2] + 0.0815]
        )
        pregrasp = self._right_handover_pose(expected, distance=0.08)
        if not self.move(self.move_to_pose(right, pregrasp)):
            return self.info
        if not self.move(self.move_by_displacement(left, x=0.03, y=0.04, z=0.025)):
            return self.info

        object_pose = self.box.get_pose()
        axis = t3d.quaternions.quat2mat(object_pose.q)[:, 0]
        down = axis if axis[2] < 0 else -axis
        contact = self._right_handover_pose(
            object_pose.p - down * 0.03,
            distance=0,
            z_offset=-0.02,
            lateral=0.045,
        )
        for step in range(1, 7):
            target = pregrasp + (contact - pregrasp) * (step / 8)
            if not self.move(self.move_to_pose(right, target)):
                return self.info

        if not self.move(self.close_gripper(right, pos=0.2)):
            return self.info
        for opening in (0.30, 0.40, 0.50, 0.65, 0.80, 1.0):
            if not self.move(self.close_gripper(left, pos=opening)):
                return self.info
        for _ in range(120):
            self.scene.step()

        if not self.move(self.move_by_displacement(left, x=-0.03, y=-0.04, z=0.02)):
            return self.info
        if not self.move(self.back_to_origin(left)):
            return self.info
        self._handover_verified = self._right_holds_object()
        return self.info

    @staticmethod
    def _right_handover_pose(position, distance, z_offset=-0.025, lateral=0.02):
        angle = np.deg2rad(30)
        cosine, sine = np.cos(angle), np.sin(angle)
        rotation = np.array(
            [[-cosine, -sine, 0], [sine, -cosine, 0], [0, 0, 1]],
            dtype=float,
        )
        contact = np.asarray(position, dtype=float).copy()
        contact[2] += z_offset
        target = contact - rotation[:, 0] * distance + rotation[:, 1] * lateral
        return np.r_[target, t3d.quaternions.mat2quat(rotation)]

    def _right_holds_object(self):
        jaws = {"right_gripper_link", "right_moving_jaw_so101_v1_link"}
        jaw_impulses = {}
        jaw_normals = {}
        inter_arm_impulse = 0.0
        for contact in self.scene.get_contacts():
            names = tuple(body.entity.name for body in contact.bodies)
            impulse = sum(float(np.linalg.norm(point.impulse)) for point in contact.points)
            if any(name.startswith("left_") for name in names) and any(
                name.startswith("right_") for name in names
            ):
                inter_arm_impulse += impulse
            jaw = next((name for name in names if name in jaws), None)
            if jaw is None or "handover_object" not in names:
                continue
            jaw_impulses[jaw] = jaw_impulses.get(jaw, 0.0) + impulse
            strongest = max(contact.points, key=lambda point: np.linalg.norm(point.impulse))
            jaw_normals[jaw] = np.asarray(strongest.normal, dtype=float)

        normals = (
            [jaw_normals[name] for name in sorted(jaws)]
            if jaw_impulses.keys() == jaws
            else []
        )
        normal_dot = float(np.dot(*normals)) if normals else None
        success = (
            jaw_impulses.keys() == jaws
            and inter_arm_impulse <= 1e-5
            and min(jaw_impulses.values()) > 1e-4
            and normal_dot < -0.8
            and self.box.get_pose().p[2] > 0.82
        )
        print(
            "DAPIER_HANDOVER_GATE: "
            f"success={success} jaw_impulses={jaw_impulses} "
            f"normal_dot={normal_dot} inter_arm_impulse={inter_arm_impulse:.6f} "
            f"object_z={self.box.get_pose().p[2]:.6f}",
            flush=True,
        )
        return success

    def choose_grasp_pose(
        self, actor, arm_tag, pre_dis=0.1, target_dis=0, contact_point_id=None
    ):
        contact = np.array(actor.get_pose().p, dtype=float)
        if arm_tag == "right" and contact[2] > 0.84:
            # Approach diagonally from the right mount while the left wrist stays above.
            angle = np.deg2rad(30)
            cosine, sine = np.cos(angle), np.sin(angle)
            rotation = np.array(
                [[-cosine, -sine, 0], [sine, -cosine, 0], [0, 0, 1]]
            )
            contact[2] -= 0.025
        else:
            # The official gripper frame already contains its 180-degree flip.
            rotation = np.array(
                [[0, 0, -1], [0, -1, 0], [-1, 0, 0]], dtype=float
            )
            contact[2] += self.grasp_height_offset_m
        quaternion = t3d.quaternions.mat2quat(rotation).tolist()

        def pose(distance):
            position = (
                contact
                - rotation[:, 0] * distance
                + rotation[:, 1] * self.grasp_lateral_offset_m
            )
            return position.tolist() + quaternion

        return pose(pre_dis), pose(target_dis)

    def move(self, first, second=None, save_freq=-1):
        was_successful = self.plan_success
        result = super().move(first, second, save_freq)
        if was_successful and not self.plan_success:
            actions = [first] + ([] if second is None else [second])
            summary = "; ".join(
                f"{arm}: " + ", ".join(str(action) for action in arm_actions)
                for arm, arm_actions in actions
            )
            print(f"DAPIER_PLAN_STAGE_FAIL: {summary}", flush=True)
        return result

    def load_actors(self):
        entity = create_cylinder(
            scene=self,
            pose=rand_pose(
                xlim=[-0.0685],
                ylim=[0.0307],
                zlim=[0.822],
                qpos=[2**-0.5, 0, 2**-0.5, 0],
                rotate_rand=False,
            ),
            radius=0.015,
            half_length=0.08,
            color=(1, 0, 0),
            name="handover_object",
        )
        self.box = Actor(
            entity,
            {
                "center": [0, 0, 0],
                "extents": [0.015, 0.015, 0.08],
                "scale": [0.015, 0.015, 0.08],
            },
        )
        self.target_box = create_box(
            scene=self,
            pose=rand_pose(xlim=[0.04, 0.08], ylim=[0.04, 0.08]),
            half_size=(0.04, 0.04, 0.005),
            color=(0, 0, 1),
            name="handover_target",
            is_static=True,
        )
        self.add_prohibit_area(self.box, padding=0.05)
        self.add_prohibit_area(self.target_box, padding=0.05)

    def check_success(self):
        return bool(getattr(self, "_handover_verified", False))
