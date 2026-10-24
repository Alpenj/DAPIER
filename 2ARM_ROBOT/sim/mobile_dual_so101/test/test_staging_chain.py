import json
from pathlib import Path
import unittest
import mujoco
import numpy as np
from waypoint_block_teacher import WaypointBlockTeacher
from collision_guard import manipulation_pair_status, task_clearance_status

class StagingChainTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture=json.loads((Path(__file__).parent/"fixtures/staging_chain.json").read_text())
        cls.t=t=WaypointBlockTeacher(fixture["candidate"])
        d=t.d;s=fixture["settled"]
        d.qpos[:]=s["qpos"];d.qvel[:]=s["qvel"];d.ctrl[:]=s["target_q"];d.time=s["time_s"]
        mujoco.mj_forward(t.m,d)
        t.closing=d.xmat[t.block_body].reshape(3,3)[:,0].copy()
        kind=mujoco.mjtState.mjSTATE_INTEGRATION
        before=np.empty(mujoco.mj_stateSize(t.m,kind));after=np.empty_like(before)
        mujoco.mj_getState(t.m,d,before,kind)
        cls.sequence=t.staging_plan(np.array(fixture["teacher_input"]["pregrasp_xyz"]),
                                   np.array(fixture["teacher_input"]["grasp_xyz"]))
        mujoco.mj_getState(t.m,d,after,kind)
        cls.before,cls.after=before,after

    def test_planning_does_not_modify_physics_state(self):
        np.testing.assert_array_equal(self.before,self.after)

    def test_continuous_chain_and_entire_segments(self):
        self.assertEqual([r["phase"] for r in self.sequence],
                         ["SAFE_STAGE","ALIGN_HIGH","PREGRASP_NEAR","APPROACH_COARSE","APPROACH_FINE"])
        for i,row in enumerate(self.sequence):
            self.assertTrue(row["eligible"])
            self.assertTrue(row["guard"]["safe"])
            self.assertGreaterEqual(row["path_general_minimum_m"],.03)
            self.assertLessEqual(row["position_error_m"],.0005)
            if i:
                np.testing.assert_array_equal(row["seed_q"],self.sequence[i-1]["ik"]["action_rad"])
                self.assertLessEqual(row["approach_error_rad"],np.deg2rad(2))
                self.assertLessEqual(row["closing_error_rad"],np.deg2rad(15))
        for row in self.sequence[:2]:
            self.assertEqual(row["task_policy"]["pairs"],[])

    def test_raw_physics_tracking_clearance_failure_is_preserved(self):
        fixture=json.loads((Path(__file__).parent/"fixtures/staging_physics_blocker.json").read_text())
        p=mujoco.MjData(self.t.m)
        # Passive jaw coordinates are part of measured physics. Actuator-only
        # FK projects them onto the ideal linkage and cannot replay this failure.
        p.qpos[:]=fixture["qpos"];p.qvel[:]=fixture["qvel"];p.ctrl[:]=fixture["ctrl"]
        p.time=fixture["time_s"];mujoco.mj_forward(self.t.m,p)
        status=task_clearance_status(self.t.m,p,"ALIGN_HIGH")
        self.assertFalse(status["safe"])
        self.assertEqual(status["closest_general_pair"],fixture["pair"])
        self.assertAlmostEqual(status["general_clearance_m"],fixture["expected_gap_m"],places=12)
        self.assertLess(status["general_clearance_m"],.03)
        self.assertGreater(status["general_clearance_m"],0.)
        self.assertEqual(status["pairs"],[])
        self.assertFalse(any({int(c.geom1),int(c.geom2)}==set(fixture["pair"]) for c in p.contact))

    def test_zero_retreat_and_wrong_phase_fail_closed(self):
        trial=self.t.report["staging_search"]["trials"][0]
        self.assertEqual(trial["offset_m"],0.)
        self.assertFalse(trial["eligible"])
        self.assertFalse(trial["segments"][0]["guard"]["safe"])
        self.assertEqual(trial["segments"][0]["guard"]["required_clearance_m"],.03)
        for phase in ("SAFE_STAGE","ALIGN_HIGH"):
            self.assertEqual(manipulation_pair_status(self.t.m,self.t.d,phase),[])

if __name__=="__main__":unittest.main()
