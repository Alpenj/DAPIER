import json
from pathlib import Path
import unittest
import numpy as np
import mujoco
from waypoint_block_teacher import WaypointBlockTeacher
from dynamic_preflight import full_state_preflight,integration_state

class DynamicPreflightTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        f=json.loads((Path(__file__).parent/"fixtures/dynamic_preflight.json").read_text())
        t=WaypointBlockTeacher(f["candidate"]);t.env.reset(seed=0)
        if t.report["provenance"]["model_sha256"]!=f["model_sha256"]:
            raise AssertionError("dynamic fixture model changed")
        state=np.asarray(f["initial_state"])
        kind=mujoco.mjtState.mjSTATE_INTEGRATION
        mujoco.mj_setState(t.m,t.d,state,kind);mujoco.mj_forward(t.m,t.d)
        mujoco.mj_setState(t.m,t.d,state,kind)
        t.env.settle_info=f["settle_info"]
        t.closing=t.d.xmat[t.block_body].reshape(3,3)[:,0].copy()
        cls.before=integration_state(t.m,t.d)
        cls.failed=full_state_preflight(t,f["baseline_segments"])
        cls.passed=full_state_preflight(t,f["sufficient_segments"])
        cls.after=integration_state(t.m,t.d)
        cls.env_valid=t.env._reset_valid
        cls.original_trace_length=len(t.step_telemetry)
        cls.expected_time=f["failure_time_s"]
        cls.expected_gap=f["failure_gap_m"]

    def test_copied_full_state_reproduces_actual_failure(self):
        r=self.failed
        self.assertFalse(r["passed"])
        self.assertEqual(r["failure"]["phase"],"ALIGN_HIGH")
        last=r["telemetry"][-1]
        self.assertAlmostEqual(last["time_s"],self.expected_time,places=9)
        self.assertAlmostEqual(last["measured_clearance_m"],self.expected_gap,places=12)
        self.assertLess(last["measured_clearance_m"],.03)
        self.assertGreaterEqual(last["target_fk_clearance_m"],.03)
        self.assertTrue(last["passive_jaw_state"])
        self.assertFalse(last["command_fk_is_safety_evidence"])
        self.assertEqual(last["closest_pair"],[36,74])

    def test_sufficient_retreat_and_every_step(self):
        r=self.passed
        self.assertTrue(r["passed"],r["failure"])
        self.assertGreaterEqual(r["minimum_measured_clearance_m"],.03)
        times=[row["time_s"] for row in r["telemetry"]]
        np.testing.assert_allclose(np.diff(times),.002,atol=1e-12,rtol=0)
        for row in r["telemetry"]:
            self.assertGreaterEqual(row["measured_clearance_m"],.03)
            self.assertEqual(row["task_pairs"],[])  # no near exemption in staging/alignment
            self.assertTrue(row["policy_safe"])
            self.assertTrue(np.isfinite(row["raw_qpos"]).all())

    def test_live_physics_and_python_state_are_not_contaminated(self):
        np.testing.assert_array_equal(self.before,self.after)
        self.assertTrue(self.env_valid)
        self.assertEqual(self.original_trace_length,0)
        for result in (self.failed,self.passed):
            self.assertTrue(result["live_state_unchanged"])
            np.testing.assert_array_equal(result["initial_state"],self.before)

if __name__=="__main__":unittest.main()
