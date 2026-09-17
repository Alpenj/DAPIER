import unittest
from unittest.mock import patch
import numpy as np
import mujoco
from integration_scenes import task_env
from shoe_task import measured_joint_state
from mobile_dual_so101 import apply_control_as_pose

class MeasuredStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.env=task_env()

    def setUp(self):
        self.env.reset(seed=0)

    def test_commands_strict_both_sides(self):
        for a in (5,11):
            command=self.env.data.ctrl.copy()
            command[a]=np.nextafter(self.env.model.actuator_ctrlrange[a,1],np.inf)
            with self.subTest(a=a), patch("shoe_task.mujoco.mj_step") as step:
                with self.assertRaisesRegex(ValueError,"outside actuator range"):
                    self.env.apply_action(command,physics_steps=1)
                step.assert_not_called()

    def test_raw_measured_boundary_and_invalid_both_sides(self):
        m,d=self.env.model,self.env.data
        for a in (5,11):
            j=int(m.actuator_trnid[a,0]); q=int(m.jnt_qposadr[j]); upper=m.jnt_range[j,1]
            for delta in (1.20686e-9,8.9633e-9):
                d.qpos[q]=upper+delta
                current,rows=measured_joint_state(m,d,integration_desk=True)
                self.assertEqual(current[a],d.qpos[q])
                self.assertEqual(next(r for r in rows if r["actuator"]==m.actuator(a).name)["status"],"boundary numerical overshoot")
                preview=mujoco.MjData(m)
                apply_control_as_pose(m,preview,current,preserve_raw_pose=True)
                self.assertEqual(preview.qpos[q],d.qpos[q])
            for value in (upper+1.01e-8,upper+1e-4,np.nan,np.inf):
                d.qpos[q]=value
                with self.subTest(a=a,value=value),self.assertRaises(ValueError):
                    measured_joint_state(m,d,integration_desk=True)
            d.qpos[q]=upper
        d.qvel[0]=np.inf
        with self.assertRaises(ValueError):measured_joint_state(m,d,integration_desk=True)


    def test_dynamic_pregrasp_boundary_fixture_still_rejects(self):
        import json
        from pathlib import Path
        f=json.loads((Path(__file__).parent/"fixtures/dynamic_pregrasp_boundary.json").read_text())
        m,d=self.env.model,self.env.data
        d.qpos[:]=f["qpos"];d.qvel[:]=f["qvel"];d.ctrl[:]=f["ctrl"];d.time=f["time_s"]
        raw=d.qpos.copy()
        self.assertTrue(np.all(d.ctrl>=m.actuator_ctrlrange[:,0]))
        self.assertTrue(np.all(d.ctrl<=m.actuator_ctrlrange[:,1]))
        with self.assertRaisesRegex(ValueError,"measured joint left_gripper exceeds joint range"):
            measured_joint_state(m,d,integration_desk=True)
        np.testing.assert_array_equal(d.qpos,raw)

    def test_mapping_fail_closed(self):
        m,d=self.env.model,self.env.data
        original=m.actuator_gear[5].copy()
        try:
            m.actuator_gear[5,0]=2.
            with self.assertRaisesRegex(ValueError,"transmission"):
                measured_joint_state(m,d,integration_desk=True)
        finally:m.actuator_gear[5]=original

if __name__=="__main__":unittest.main()
