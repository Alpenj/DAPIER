import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from waypoint_block_teacher import WaypointBlockTeacher
from mobile_dual_so101 import actuator_targets_from_qpos,apply_control_as_pose
from dynamic_preflight import integration_state

from integration_scenes import same_audited_desk_model

class CloseReferenceTest(unittest.TestCase):
    def test_precontact_commands_do_not_reseed_from_measured(self):
        fixture=json.loads((Path(__file__).parent/"fixtures/approach_tracking.json").read_text())
        t=WaypointBlockTeacher(fixture["candidate"]);t.env.reset(seed=0)
        t.gripper_hold_reference=(fixture["task_open"]["reference_rad"],)*2
        t.waypoint_xyz=np.zeros(3)
        reference=t.d.ctrl.copy()
        arm=[i for i in range(t.m.nu) if i not in (5,11)]
        j=int(t.m.actuator_trnid[1,0]);t.d.qpos[t.m.jnt_qposadr[j]]+=.0002
        commands=[];starts=[]
        def move(target,duration):
            starts.append(np.asarray(actuator_targets_from_qpos(t.m,t.d.qpos)).copy())
            commands.append(np.array(target))
            if len(commands)==3:raise RuntimeError("test stop, no fabricated grasp")
            t.d.ctrl[:]=target
            t.d.qpos[t.m.jnt_qposadr[j]]+=.0001
        t.move=move
        del t.step_telemetry  # Unit test command generation only; real copied/live check runs separately.
        with patch.object(t,"snapshot",return_value={"finger_force_N":{"fixed":0.,"moving":0.}}):
            with self.assertRaisesRegex(RuntimeError,"test stop"):t.finish_task(np.zeros(3))
        for q in commands:np.testing.assert_array_equal(q[arm],reference[arm])
        self.assertTrue(all(commands[i+1][5]<commands[i][5] for i in range(2)))
        self.assertNotEqual(starts[0][1],commands[0][1])
        self.assertGreater(starts[2][1],starts[0][1])
        self.assertFalse(t.report["success"])
        self.assertNotIn("GRASP_CONFIRM",[r["phase"] for r in t.report["transitions"]])

    def test_close_path_uses_raw_state_trajectory_uses_reference(self):
        from physics_ik import plan_septic_joint_trajectory
        from collision_guard import check_bimanual_path
        import mujoco
        f=json.loads((Path(__file__).parent/"fixtures/approach_tracking.json").read_text())
        t=WaypointBlockTeacher(f["candidate"]);t.env.reset(seed=0)
        t.phase="CLOSE";t.env.collision_phase="CLOSE"
        t.close_arm_reference=t.d.ctrl.copy()
        j=int(t.m.actuator_trnid[1,0]);t.d.qpos[t.m.jnt_qposadr[j]]+=.0002
        mujoco.mj_forward(t.m,t.d)
        raw=t.d.qpos.copy();measured=np.array(actuator_targets_from_qpos(t.m,raw))
        with patch("center_block_teacher.plan_septic_joint_trajectory",wraps=plan_septic_joint_trajectory) as trajectory,patch("center_block_teacher.check_bimanual_path",wraps=check_bimanual_path) as path,patch.object(t.env,"apply_action",side_effect=RuntimeError("test stop")):
            with self.assertRaisesRegex(RuntimeError,"test stop"):t.move(t.close_arm_reference,duration=.1)
        arms=[i for i in range(t.m.nu) if i not in (5,11)]
        np.testing.assert_array_equal(path.call_args.args[1],measured)
        np.testing.assert_array_equal(trajectory.call_args.args[1][arms],t.close_arm_reference[arms])
        np.testing.assert_array_equal(raw,t.d.qpos)
        np.testing.assert_array_equal(t.step_telemetry[-1]["raw_qpos"],raw)

    def test_saved_reseeding_failure(self):
        f=json.loads((Path(__file__).parent/"fixtures/close_reference.json").read_text())
        self.assertTrue(all(p["next_target_equals_previous_measured"] for p in f["proof"]))
        last=f["stages"][-1]["FK"]
        self.assertLess(last["command"]["position_error_m"],.0005)
        self.assertGreater(last["measured"]["position_error_m"],.0005)
        self.assertTrue(all(v==0 for v in f["stages"][-1]["finger_force_N"].values()))

    def test_full_state_preflight_matches_live_without_fake_contact(self):
        import mujoco
        from dynamic_preflight import full_state_preflight
        f=json.loads((Path(__file__).parent/"fixtures/close_physics.json").read_text())
        t=WaypointBlockTeacher(f["candidate"]);t.env.reset(seed=0)
        self.assertTrue(same_audited_desk_model(t.report["provenance"]["portable_model_sha256"],f["portable_model_sha256"]))
        t.env.settle_info=f["settle"];t.gripper_hold_reference=(f["task_open"]["reference_rad"],)*2
        state=np.array(f["initial_state"]);kind=mujoco.mjtState.mjSTATE_INTEGRATION
        mujoco.mj_setState(t.m,t.d,state,kind);mujoco.mj_forward(t.m,t.d);mujoco.mj_setState(t.m,t.d,state,kind)
        t.close_arm_reference=np.array(f["reference"]);t.close_start_tcp=t.d.site_xpos[t.site].copy()
        t.waypoint_xyz=np.array(f["xyz"]);t.closing=np.array([1.,0,0])
        t.transition("CLOSE")
        for target in f["targets"]:
            before=integration_state(t.m,t.d)
            pref=full_state_preflight(t,[dict(eligible=True,phase="CLOSE",xyz=f["xyz"],ik=dict(action_rad=target),duration_s=.1)])
            self.assertTrue(pref["passed"],pref["failure"])
            np.testing.assert_array_equal(before,integration_state(t.m,t.d))
            t.move(target,duration=.1)
            np.testing.assert_array_equal(integration_state(t.m,t.d),pref["final_state"])
            row=t.step_telemetry[-1]
            self.assertLess(row["executed_tcp_error_m"],.0005)
            self.assertEqual(row["arm_reference_drift_rad"],0.)
            self.assertTrue(all(v==0 for v in row["finger_force_N"].values()))
            self.assertFalse(t.report["success"])

if __name__=="__main__":unittest.main()
