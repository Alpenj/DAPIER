import unittest
from unittest.mock import patch
import mujoco
import numpy as np

from integration_scenes import task_env
from collision_guard import (structural_near_support_pairs, structural_near_support_status,
                             protected_geom_pairs, check_bimanual_path)
from mobile_dual_so101 import actuator_targets_from_qpos


class NearSupportTest(unittest.TestCase):
    def test_exact_identity_home_and_nominal_geometry(self):
        e=task_env(); m,d=e.model,e.data
        self.assertEqual(structural_near_support_pairs(m), ((13,0),(48,0)))
        for row in structural_near_support_status(m,d):
            self.assertTrue(row["safe"])
            self.assertIn(tuple(row["pair"]), protected_geom_pairs(m))
            self.assertAlmostEqual(row["positive_separation_evidence_m"], .0161999981, places=9)
            self.assertIn("HARDWARE_UNVERIFIED",row["scope"])
        q=actuator_targets_from_qpos(m,d.qpos)
        self.assertTrue(check_bimanual_path(m,q,q).safe)
        # A sibling shoulder geom is NOT structural: its 30mm rule stays active.
        g=9
        body=int(m.geom_bodyid[g])
        m.geom_pos[g] += d.xmat[body].reshape(3,3).T @ np.array([0.,0.,-.06])
        bad=check_bimanual_path(m,q,q)
        self.assertFalse(bad.safe)
        self.assertEqual(bad.required_clearance_m,.03)
        self.assertEqual(bad.first_geom_id,g)

    def test_poststep_observer_has_integrated_geometry(self):
        e=task_env();e.reset(seed=0)
        m,d=e.model,e.data
        j=m.joint("left_shoulder_pan").id
        d.qvel[m.jnt_dofadr[j]]=.1
        observed=[]
        def observe():
            kind=mujoco.mjtState.mjSTATE_INTEGRATION
            state=np.empty(mujoco.mj_stateSize(m,kind))
            mujoco.mj_getState(m,d,state,kind)
            fresh=mujoco.MjData(m)
            mujoco.mj_setState(m,fresh,state,kind)
            mujoco.mj_forward(m,fresh)
            np.testing.assert_allclose(d.geom_xpos,fresh.geom_xpos,atol=1e-12,rtol=0)
            observed.append(float(d.time))
        e.physics_observer=observe
        e.apply_action(tuple(d.ctrl),physics_steps=1)
        self.assertEqual(observed,[float(m.opt.timestep)])

    def test_geometry_change_fails_closed(self):
        e=task_env()
        e.model.geom_pos[e.model.geom("table").id,2] += .001
        with self.assertRaisesRegex(RuntimeError,"geometry changed"):
            structural_near_support_pairs(e.model)

    def test_any_contact_and_nonfinite_evidence_rejected(self):
        e=task_env();m,d=e.model,e.data
        for distance in (0., -.001):
            mujoco.mj_forward(m,d)
            c=mujoco.MjContact();c.geom1=13;c.geom2=0;c.dist=distance
            mujoco.mj_addContact(m,d,c)
            rows=structural_near_support_status(m,d)
            self.assertFalse(rows[0]["safe"])
            self.assertEqual(rows[0]["penetration"],distance<0)
        mujoco.mj_forward(m,d)
        with patch("collision_guard.certified_mesh_box_separation_lower_bound",return_value=float("nan")):
            self.assertFalse(structural_near_support_status(m,d)[0]["safe"])


if __name__=="__main__":
    unittest.main()
