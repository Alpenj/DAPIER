import json,math,copy
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import unittest
import numpy as np
import mujoco
from waypoint_block_teacher import WaypointBlockTeacher
from mobile_dual_so101 import actuator_targets_from_qpos
from dynamic_preflight import measured_axis_reserve,full_state_preflight,integration_state

from integration_scenes import same_audited_desk_model

class ApproachTrackingTest(unittest.TestCase):
    def test_observed_reserve_and_copied_endpoint(self):
        f=json.loads((Path(__file__).parent/"fixtures/approach_tracking.json").read_text())
        t=WaypointBlockTeacher(f["candidate"]);t.env.reset(seed=0)
        self.assertTrue(same_audited_desk_model(t.report["provenance"]["portable_model_sha256"],f["portable_model_sha256"]))
        t.gripper_hold_reference=(f["task_open"]["reference_rad"],)*2
        t.env.settle_info=f["settle"];t.closing=np.array([1.,0,0])
        state=np.array(f["initial_state"]);kind=mujoco.mjtState.mjSTATE_INTEGRATION
        mujoco.mj_setState(t.m,t.d,state,kind);mujoco.mj_forward(t.m,t.d);mujoco.mj_setState(t.m,t.d,state,kind)
        reserve=measured_axis_reserve(t.m,t.site,f)
        self.assertAlmostEqual(math.degrees(reserve["observed_axis_deviation_rad"]),.05335700564978,places=9)
        self.assertEqual(reserve["acceptance_rad"],math.radians(2))
        self.assertLess(reserve["internal_tolerance_rad"],reserve["acceptance_rad"])
        row=t.evaluate_waypoint(f["xyz"],actuator_targets_from_qpos(t.m,t.d.qpos),"PREGRASP_NEAR",axis_tolerance_rad=reserve["internal_tolerance_rad"])
        self.assertTrue(row["eligible"])
        self.assertLessEqual(row["approach_error_rad"],reserve["internal_tolerance_rad"])
        before=integration_state(t.m,t.d)
        result=full_state_preflight(t,[row])
        self.assertTrue(result["passed"],result["failure"])
        gate=result["waypoint_gates"][-1]
        self.assertLessEqual(gate["position_error_m"],.0005)
        self.assertLessEqual(gate["approach_error_rad"],math.radians(2))
        self.assertLessEqual(gate["closing_error_rad"],math.radians(15))
        self.assertTrue(gate["status"]["safe"])
        np.testing.assert_array_equal(before,integration_state(t.m,t.d))
        bad=copy.deepcopy(f);bad["execution_telemetry"]=[]
        with self.assertRaises(ValueError):measured_axis_reserve(t.m,t.site,bad)

if __name__=="__main__":unittest.main()
