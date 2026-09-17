import json
from pathlib import Path
import unittest
import mujoco
import numpy as np
from integration_scenes import task_env
from collision_guard import manipulation_pair_status,check_bimanual_path
from mobile_dual_so101 import apply_control_as_pose

class ManipulationPolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.env=task_env()
        f=json.loads((Path(__file__).parent/"fixtures/manipulation_grasp.json").read_text())
        cls.pose=f["q"];cls.block_qpos=f["block_qpos"];cls.pregrasp=f["pregrasp_q"]
    def setUp(self):
        self.m,self.d=self.env.model,self.env.data
        mujoco.mj_resetData(self.m,self.d)
        self.d.qpos[-7:]=self.block_qpos
        apply_control_as_pose(self.m,self.d,self.pose)
    def rows(self,phase):
        return {tuple(r["names"]):r for r in manipulation_pair_status(self.m,self.d,phase)}
    def contact(self,a,b,depth):
        c=mujoco.MjContact();c.geom1=self.m.geom(a).id;c.geom2=self.m.geom(b).id;c.dist=depth
        mujoco.mj_addContact(self.m,self.d,c)
    def test_geometry_and_positive_near(self):
        rows=self.rows("APPROACH_FINE")
        self.assertTrue(all(r["safe"] for r in rows.values()))
        self.assertAlmostEqual(rows[("left_pgripper_pad_1","table")]["clearance_m"],.00416654677,places=8)
        self.assertAlmostEqual(rows[("left_pgripper_housing","red_block_geom")]["clearance_m"],.011864144,places=8)
    def test_saved_tilted_approach_full_segment(self):
        guard=check_bimanual_path(self.m,self.pregrasp,self.pose,
            task_phase="APPROACH_FINE",reference_data=self.d)
        self.assertTrue(guard.safe,guard.reason)

    def test_support_and_housing_contact_rejected(self):
        for a,b in (("left_pgripper_pad_1","table"),("left_pgripper_housing","red_block_geom")):
            for depth in (0.,-.0001,-.002):
                self.setUp();self.contact(a,b,depth)
                for phase in ("APPROACH_FINE","CLOSE","HOLD"):
                    self.assertFalse(self.rows(phase)[(a,b)]["safe"])
    def test_finger_contact_phase_and_depth(self):
        pair=("left_pgripper_pad_1","red_block_geom")
        self.contact(*pair,-.0001)
        self.assertFalse(self.rows("APPROACH_FINE")[pair]["safe"])
        self.assertTrue(self.rows("CLOSE")[pair]["safe"])
        self.assertTrue(self.rows("GRASP_CONFIRM")[pair]["safe"])
        self.contact(*pair,-.00101)
        self.assertFalse(self.rows("CLOSE")[pair]["safe"])
    def test_wrong_phase_and_general_pair(self):
        self.assertEqual(self.rows("HOME"),{})
        old=check_bimanual_path(self.m,self.pose,self.pose,task_phase="HOME",reference_data=self.d)
        self.assertFalse(old.safe);self.assertEqual(old.required_clearance_m,.03)
        # Exact policy membership cannot absorb any right arm or unrelated arm/table pair.
        pairs={tuple(r["pair"]) for r in self.rows("APPROACH_FINE").values()}
        self.assertNotIn((self.m.geom("right_pgripper_pad_1").id,self.m.geom("table").id),pairs)
        self.assertNotIn((9,0),pairs)
    def test_unrelated_arm_table_still_requires_30mm_in_approach(self):
        e=task_env();m,d=e.model,e.data
        g=9;body=int(m.geom_bodyid[g])
        m.geom_pos[g]+=d.xmat[body].reshape(3,3).T@np.array([0.,0.,-.06])
        q=tuple(d.ctrl)
        bad=check_bimanual_path(m,q,q,task_phase="APPROACH_FINE",reference_data=d)
        self.assertFalse(bad.safe)
        self.assertEqual(bad.required_clearance_m,.03)
        self.assertEqual(bad.first_geom_id,g)

    def test_changed_geometry_fails_closed(self):
        g=self.m.geom("left_pgripper_housing").id
        old=self.m.geom_size[g,0]
        try:
            self.m.geom_size[g,0]+=1e-4
            with self.assertRaisesRegex(ValueError,"geometry changed"):self.rows("CLOSE")
        finally:self.m.geom_size[g,0]=old
if __name__=="__main__":unittest.main()
