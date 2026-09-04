"""Small-object bimanual handover sized for the real Dual SO-101 workspace."""

import numpy as np
import transforms3d as t3d

from envs.handover_block import handover_block
from envs.utils import create_box, rand_pose


class dapier_handover_block(handover_block):
    def setup_demo(self, **kwargs):
        # Keep RoboTwin's table at world z=0.74 and place the robot root there.
        # Relative to the robot this is the real mobile-base ground plane.
        super()._init_task_env_(**kwargs)
        self.block_middle_pose = [0, 0.08, 0.90, 0, 1, 0, 0]

    def choose_grasp_pose(
        self, actor, arm_tag, pre_dis=0.1, target_dis=0, contact_point_id=None
    ):
        # Keep the current reachable wrist orientation and approach along the
        # RoboTwin grasp-frame X axis. A 5-DoF SO-101 cannot realize the stock
        # ALOHA task's arbitrary 6-DoF contact orientations.
        tcp = (
            self.robot.get_left_ee_pose()
            if arm_tag == "left"
            else self.robot.get_right_ee_pose()
        )
        rotation = t3d.quaternions.quat2mat(tcp[-4:])
        contact = np.array(actor.get_pose().p, dtype=float)
        contact[2] += 0.02

        def pose(distance):
            position = contact + rotation[:, 0] * distance
            return position.tolist() + tcp[-4:]

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
        self.box = create_box(
            scene=self,
            pose=rand_pose(
                xlim=[-0.08, -0.04],
                ylim=[0.04, 0.08],
                zlim=[0.802],
                qpos=[0.981, 0, 0, 0.195],
                rotate_rand=True,
                rotate_lim=[0, 0, 0.12],
            ),
            half_size=(0.02, 0.02, 0.06),
            color=(1, 0, 0),
            name="handover_object",
            boxtype="long",
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
        box_pos = self.box.get_functional_point(0, "pose").p
        target = self.target_box.get_functional_point(1, "pose").p
        return (
            np.all(np.abs(box_pos[:2] - target[:2]) < 0.03)
            and abs(box_pos[2] - target[2]) < 0.01
            and self.is_right_gripper_open()
        )
