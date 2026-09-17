import json
from pathlib import Path
import unittest
from unittest.mock import patch
import mujoco
import numpy as np
from collision_guard import certified_box_box_separation_lower_bound as certificate, minimum_protected_clearance
from integration_scenes import task_env

class BoxBoxCertificateTest(unittest.TestCase):
    def test_saved_failure(self):
        f=json.loads((Path(__file__).parent/"fixtures/box_box_false_zero.json").read_text())
        e=task_env(); m,d=e.model,e.data
        d.qpos[:]=f["qpos"]; d.qvel[:]=f["qvel"]; d.ctrl[:]=f["target_q"]
        mujoco.mj_forward(m,d)
        for pair in (f["pair"],f["pair"][::-1]):
            self.assertEqual(mujoco.mj_geomDistance(m,d,*pair,2.,None),0.)
            bound=certificate(m,d,*pair)
            self.assertAlmostEqual(bound,f["expected_lower_bound_m"],places=10)
            self.assertGreater(minimum_protected_clearance(m,d,[pair])[0],.03)

    def test_rotated_touching_penetration_parallel(self):
        m=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
        <geom type="box" size=".1 .1 .1" quat=".9238795325112867 0 .3826834323650898 0"/>
        <body><freejoint/><geom type="box" size=".1 .1 .1" mass="1"/></body>
        </worldbody></mujoco>''')
        d=mujoco.MjData(m)
        for epsilon in (0.,1e-14,1e-9):
            d.qpos[3:]=m.geom_quat[0]
            d.qpos[4]+=epsilon; d.qpos[3:]/=np.linalg.norm(d.qpos[3:])
            mujoco.mj_forward(m,d)
            axis=d.geom_xmat[0].reshape(3,3)[:,0]
            radius=float(np.abs(axis @ d.geom_xmat[1].reshape(3,3)) @ m.geom_size[1])
            for gap in (.04,0.,-1e-8,-.05):
                d.qpos[:3]=axis*(.1+radius+gap)
                mujoco.mj_forward(m,d)
                with self.subTest(epsilon=epsilon,gap=gap):
                    bound=certificate(m,d,0,1)
                    if gap>0:
                        self.assertAlmostEqual(bound,gap,places=10)
                    else:
                        self.assertEqual(bound,0.)
                        with patch("collision_guard.mujoco.mj_geomDistance",return_value=0.):
                            self.assertLessEqual(minimum_protected_clearance(m,d,[(0,1)])[0],0.)
                        self.assertLess(minimum_protected_clearance(m,d,[(0,1)])[0],.03)
            # Native positive must skip the SAT certificate.
            with patch("collision_guard.mujoco.mj_geomDistance",return_value=.02), patch(
                    "collision_guard.certified_box_box_separation_lower_bound",side_effect=AssertionError):
                self.assertEqual(minimum_protected_clearance(m,d,[(0,1)])[0],.02)
        d.geom_xpos[1,0]=np.nan
        self.assertEqual(certificate(m,d,0,1),0.)

if __name__=="__main__":
    unittest.main()
