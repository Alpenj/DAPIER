import json,unittest
from pathlib import Path
import numpy as np
from waypoint_block_teacher import WaypointBlockTeacher
from close_contact_diagnostic import restore
from dynamic_preflight import full_state_preflight,integration_state
from mobile_dual_so101 import actuator_targets_from_qpos

class PostContactCloseTest(unittest.TestCase):
    def test_fixed_reference_forms_physical_bilateral_contact(self):
        f=json.loads((Path(__file__).parent/"fixtures/post_contact_close.json").read_text())
        t=WaypointBlockTeacher(f["candidate"]);t.env.reset(seed=0)
        self.assertEqual(t.report["provenance"]["model_sha256"],f["model_sha256"])
        t.env.settle_info=f["settle"];t.gripper_hold_reference=(f["task_open"]["reference_rad"],)*2
        restore(t.m,t.d,f["start_state"])
        t.closing=np.array([1.,0,0]);t.waypoint_xyz=np.array(f["xyz"])
        t.close_arm_reference=np.array(f["reference"]);t.close_start_tcp=t.d.site_xpos[t.site].copy()
        t.close_contact_origin={k:np.array(v) for k,v in f["origin"].items()}
        t.phase="CLOSE";t.env.collision_phase="CLOSE"
        self.assertTrue(all(x["target_equals_previous_measured"] for x in f["baseline_q_proof"]))
        for stage in f["stages"]:
            t.close_previous_command=t.d.ctrl.copy()
            t.close_previous_measured=np.array(actuator_targets_from_qpos(t.m,t.d.qpos))
            before=integration_state(t.m,t.d)
            p=full_state_preflight(t,[dict(eligible=True,phase="CLOSE",xyz=f["xyz"],ik=dict(action_rad=stage["target"]),duration_s=.1)])
            self.assertTrue(p["passed"],p["failure"])
            np.testing.assert_array_equal(before,integration_state(t.m,t.d))
            row=p["telemetry"][-1]
            self.assertAlmostEqual(row["executed_tcp_error_m"],stage["expected_error"],places=10)
            self.assertEqual(row["arm_reference_drift_rad"],0.)
            self.assertLessEqual(row["executed_tcp_error_m"],.0005)
            self.assertLessEqual(row["measured_approach_error_rad"],np.deg2rad(2))
            self.assertLessEqual(row["measured_closing_error_rad"],np.deg2rad(15))
            self.assertFalse(row["non_target_protected_contacts"])
            self.assertTrue(all(x["policy_safe"] for x in p["telemetry"]))
            self.assertFalse(t.report["success"])
            restore(t.m,t.d,p["final_state"])
        self.assertEqual(row["contact_state"],"bilateral")
        self.assertTrue(all(v>0 for v in row["finger_force_N"].values()))
        self.assertTrue(all(row["finger_contacts"].values()))
        self.assertLessEqual(max(c["penetration_m"] for c in row["contacts"]),.001)

if __name__=="__main__":unittest.main()
