import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from waypoint_block_teacher import WaypointBlockTeacher
from pgripper import jaw_gap_m
from mobile_dual_so101 import apply_control_as_pose,actuator_targets_from_qpos
from physics_ik import plan_septic_joint_trajectory

class TaskOpenTest(unittest.TestCase):
    def setUp(self):
        self.reference=json.loads((Path(__file__).parent/"fixtures/task_open_reference.json").read_text())
        self.t=WaypointBlockTeacher(self.reference["candidate"],self.reference)
        self.t.configure_task_open()

    def test_opening_and_both_command_margins_without_reset_change(self):
        t=self.t;raw=t.d.qpos.copy();q=t.d.ctrl.copy();q[[5,11]]=t.gripper_hold_reference
        import mujoco
        preview=mujoco.MjData(t.m);preview.qpos[:]=raw
        apply_control_as_pose(t.m,preview,q)
        for side,a in (("left",5),("right",11)):
            self.assertLess(q[a],t.m.actuator_ctrlrange[a,1])
            self.assertGreater(q[a],t.m.actuator_ctrlrange[a,0])
            self.assertGreater(jaw_gap_m(t.m,preview,side),t.report["task_open"]["required_opening_m"])
            self.assertLess(jaw_gap_m(t.m,preview,side),t.report["task_open"]["maximum_opening_m"]+1e-12)
        np.testing.assert_array_equal(t.d.qpos,raw)
        self.assertGreater(t.report["task_open"]["command_upper_margin_rad"],1e-4)

    def test_trajectory_start_is_command_but_path_start_is_measured(self):
        t=self.t;t.env.reset(seed=0);t.phase="SAFE_STAGE";t.env.collision_phase="SAFE_STAGE"
        t.d.qpos[5]-=1e-5;t.d.qpos[13]-=2e-5
        import mujoco
        mujoco.mj_forward(t.m,t.d)
        measured=np.asarray(actuator_targets_from_qpos(t.m,t.d.qpos))
        commanded=t.d.ctrl.copy()
        with patch("center_block_teacher.plan_septic_joint_trajectory",wraps=plan_septic_joint_trajectory) as trajectory, patch.object(t.env,"apply_action",side_effect=RuntimeError("test stop before physics")), patch("center_block_teacher.check_bimanual_path", wraps=__import__("collision_guard").check_bimanual_path) as path:
            with self.assertRaisesRegex(RuntimeError,"test stop"):
                t.move(commanded)
        np.testing.assert_array_equal(path.call_args.args[1],measured)
        np.testing.assert_array_equal(trajectory.call_args.args[1][[5,11]],commanded[[5,11]])
        np.testing.assert_array_equal(trajectory.call_args.args[2][5:12:6],t.gripper_hold_reference)

if __name__=="__main__":unittest.main()
