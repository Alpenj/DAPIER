import unittest
import mujoco
import numpy as np
from physics_ik import axis_direction_task
from integration_scenes import task_env
from mobile_dual_so101 import apply_control_as_pose

class AxisDirectionTest(unittest.TestCase):
    def test_site_axis_finite_difference(self):
        env=task_env();m,d=env.model,env.data
        site=m.site("left_cube_grasp").id
        for pose in ([0,0,0,0,0],[.6,.4,-.5,1.5,-.3]):
            q=d.ctrl.copy();q[:5]=pose
            apply_control_as_pose(m,d,q)
            jp=np.zeros((3,m.nv));jw=jp.copy()
            mujoco.mj_jacSite(m,d,jp,jw,site)
            u=d.site_xmat[site].reshape(3,3)[:,0].copy()
            analytic,_=axis_direction_task(u,np.array([0.,0.,-1.]),jw)
            for a in range(5):
                j=int(m.actuator_trnid[a,0]);address=int(m.jnt_qposadr[j]);dof=int(m.jnt_dofadr[j])
                original=d.qpos[address]; h=1e-6
                d.qpos[address]=original+h;mujoco.mj_forward(m,d)
                plus=d.site_xmat[site].reshape(3,3)[:,0].copy()
                d.qpos[address]=original-h;mujoco.mj_forward(m,d)
                minus=d.site_xmat[site].reshape(3,3)[:,0].copy()
                d.qpos[address]=original;mujoco.mj_forward(m,d)
                np.testing.assert_allclose(analytic[:,dof],(plus-minus)/(2*h),atol=1e-9,rtol=1e-7)

    def test_roll_nullspace_and_equivalent_frames(self):
        u=np.array([1.,2.,3.]);u/=np.linalg.norm(u)
        target=np.array([0.,0.,-1.])
        J,e=axis_direction_task(u,target,np.eye(3))
        np.testing.assert_allclose(J@u,0,atol=1e-15)
        self.assertEqual(np.linalg.matrix_rank(J),2)
        self.assertAlmostEqual(float(e@u),0.,places=14)
        # Same tool axis, different transverse frame (pure roll): same axis task.
        m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint axis="1 0 0"/><geom size=".01"/><site name="tcp"/></body></worldbody></mujoco>')
        d=mujoco.MjData(m);values=[]
        for roll in (0.,1.2):
            d.qpos[0]=roll;mujoco.mj_forward(m,d)
            jp=np.zeros((3,m.nv));jw=jp.copy();mujoco.mj_jacSite(m,d,jp,jw,0)
            axis=d.site_xmat[0].reshape(3,3)[:,0]
            values.append(axis_direction_task(axis,np.array([0.,0.,-1.]),jw))
        np.testing.assert_allclose(values[0][0],values[1][0],atol=1e-15)
        np.testing.assert_allclose(values[0][1],values[1][1],atol=1e-15)
        np.testing.assert_allclose(values[0][0],0,atol=1e-15)

if __name__=="__main__":unittest.main()
