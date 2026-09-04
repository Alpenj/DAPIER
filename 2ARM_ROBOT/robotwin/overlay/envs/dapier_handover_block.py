"""Small-object bimanual handover sized for the real Dual SO-101 workspace."""

import numpy as np

from envs.handover_block import handover_block
from envs.utils import create_box, rand_pose


class dapier_handover_block(handover_block):
    def setup_demo(self, **kwargs):
        # RoboTwin's stock table is 740 mm high; the mobile SO-101 works near its
        # ground-referenced base, so shift the whole task surface to z=0.
        super()._init_task_env_(table_height_bias=-0.74, **kwargs)
        self.block_middle_pose = [0, 0.08, 0.16, 0, 1, 0, 0]

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
