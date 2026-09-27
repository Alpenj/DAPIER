from pathlib import Path
import json
import sys
import unittest
import mujoco
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from lift_refinement_audit import refine_once
from dynamic_preflight import integration_state
from mobile_dual_so101 import build_model, HUMANOID_HOME_ACTION, TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M, apply_control_as_pose
from physics_ik import solve_bimanual_position_ik


class EndpointRefinementTest(unittest.TestCase):
    def test_one_extra_update_matches_existing_solver_without_mutating_physics(self):
        m,_=build_model(arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,mount_layout='tower')
        d=mujoco.MjData(m);apply_control_as_pose(m,d,HUMANOID_HOME_ACTION)
        site=m.site('left_gripperframe').id
        q=d.ctrl.copy();xyz=d.site_xpos[site].copy()+[.0004,0,0]
        axis=d.site_xmat[site].reshape(3,3)[:,0].copy()
        initial=integration_state(m,d)
        common=dict(site_names={'left':'left_gripperframe'},tool_axis_targets={'left':axis},axis_formulation='axis_direction')
        accepted=solve_bimanual_position_ik(m,q,{'left':xyz},**common)
        self.assertTrue(accepted.converged)
        self.assertEqual(accepted.iterations,0)  # Within acceptance is not minimum residual.
        candidate,settings=refine_once(m,d,site,q,xyz,axis)
        np.testing.assert_array_equal(integration_state(m,d),initial)
        np.testing.assert_array_equal(candidate[5:],q[5:])
        # Test oracle forces exactly one original DLS update; task acceptance is untouched.
        reference=solve_bimanual_position_ik(m,q,{'left':xyz},tolerance_m=np.finfo(float).eps,max_iterations=1,**common)
        np.testing.assert_allclose(candidate,reference.action_rad,rtol=0,atol=1e-14)
        self.assertLess(reference.residual_m_by_side['left'],accepted.residual_m_by_side['left'])
        self.assertEqual(settings,dict(damping=.02,tool_axis_weight_m=.05,max_joint_step_rad=.05))

    def test_saved_ab_preserves_gate_and_support_contract(self):
        r=json.loads((Path(__file__).parent/'fixtures/lift15_refinement.json').read_text())
        a,b=(r['cases'][k] for k in ('original','refined'))
        self.assertTrue(r['same_initial_full_state'])
        self.assertFalse(r['live_task_success'] or r['hardware_execution'])
        self.assertEqual(a['options'],b['options'])
        for c in (a,b):
            self.assertTrue(c['model_unchanged'] and c['plan']['guard']['safe'] and c['plan']['task_policy']['safe'])
            self.assertEqual(c['options']['noslip_iterations'],0)
            self.assertLess(c['plan']['approach_error_rad'],np.deg2rad(2))
            self.assertLess(c['plan']['closing_error_rad'],np.deg2rad(15))
            self.assertGreaterEqual(min(c['plan']['joint_margins_rad']),0)
            self.assertGreater(min(c['minimum_finger_forces_N']),0)
            self.assertTrue(c['table_always_absent'])
            self.assertFalse(c['any_saturation'] or any(c['max_warning_counts']))
            self.assertLess(c['max_penetration_m'],.001)
        self.assertLess(b['plan']['position_error_m'],a['plan']['position_error_m'])
        np.testing.assert_array_equal(a['plan']['q'][5:],b['plan']['q'][5:])
        self.assertGreater(a['lift15_endpoint']['errors']['total_m'],.0005)
        self.assertLess(b['lift15_endpoint']['errors']['total_m'],.0005)
        self.assertIsNotNone(b['lift15_gate'])
        self.assertFalse(a['lift15_gate_pass'])
        self.assertTrue(b['lift15_gate_pass'])


    def test_lift30_gate_pass_does_not_hide_height_loss_during_hold(self):
        r=json.loads((Path(__file__).parent/'fixtures/lift15_refinement.json').read_text())['cases']['refined']
        self.assertEqual([g['phase'] for g in r['waypoint_gates']],['LIFT_15MM','LIFT_30MM'])
        for g in r['waypoint_gates']:
            self.assertLess(g['position_error_m'],.0005)
            self.assertLess(g['approach_error_rad'],np.deg2rad(2))
            self.assertLess(g['closing_error_rad'],np.deg2rad(15))
            self.assertTrue(g['collision_safe'])
        self.assertGreater(r['maximum_bottom_lift_m'],.03)
        self.assertGreaterEqual(r['hold_observed_s'],3)
        self.assertLess(r['maximum_continuous_supported_lift_s'],3)
        lost=r['first_hold_support_criterion_failure']
        self.assertFalse(lost['lift_supported'])
        self.assertLess(lost['block_bottom_world_m'],.03)
        self.assertGreater(min(lost['finger_forces_N']),0)
        self.assertEqual(lost['table_count'],0)
        self.assertEqual(r['final']['continuous_hold_s'],0)
        self.assertLess(r['final']['bottom_lift_m'],.03)
        self.assertLess(r['final']['block_vz_m_s'],0)
        self.assertTrue(r['gripper_command_constant'])
        self.assertFalse(r['copied_success'])
        self.assertEqual(r['failure'],'continuous 3-second supported lift not reached')


if __name__=='__main__':unittest.main()
