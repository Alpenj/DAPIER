import contextlib
import io
import json
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lift_transition_diagnostic import run, next_close_diagnostic


class LiftTransitionTest(unittest.TestCase):
    def test_actual_failure_and_bounded_contact_diagnostics(self):
        fixture=json.loads((Path(__file__).parent/'fixtures/lift_transition.json').read_text())
        with contextlib.redirect_stdout(io.StringIO()):
            report=run(fixture)
        self.assertFalse(report['live_success'])
        same_kernel=report['provenance']['portable_model_sha256']==fixture['provenance']['portable_model_sha256']
        # The audited second BLAS identity reproduced saved qpos within 3.08e-16.
        # This four-epsilon historical comparison never changes runtime state limits.
        limit=0. if same_kernel else 4*np.finfo(float).eps
        self.assertLessEqual(report['confirmation_replay_max_qpos_difference'],limit)
        self.assertLessEqual(report['first_lift_replay_max_qpos_difference'],limit)
        self.assertEqual(report['close_terminal_command'],report['confirm_terminal_command'])
        self.assertEqual(set(report['states']),{'A_close','B_confirm','C_first_lift'})
        hold=report['cases']['hold'];baseline=report['cases']['baseline_lift']
        continuous=report['cases']['command_continuous_lift']
        self.assertTrue(all(r['gate_failure'] is None for r in hold))
        self.assertTrue(all(r['maximum_command_delta_rad']==0 for r in hold))
        self.assertTrue(all(v==0 for v in baseline[0]['finger_force_N'].values()))
        self.assertGreater(baseline[0]['maximum_command_delta_rad'],.0005)
        self.assertLess(continuous[0]['maximum_command_delta_rad'],1e-8)
        self.assertEqual(next(r['step'] for r in continuous if r['gate_failure']),36)
        event=report['chronology']['command_continuous_lift']
        self.assertIsNone(event['table_support_loss'])
        self.assertEqual(event['first_finger_support_loss'],36)
        self.assertEqual(event['both_finger_support_loss'],42)
        self.assertIsNone(event['both_geometric_contacts_lost'])
        self.assertTrue(any(continuous[41]['finger_contacts'].values()))
        for rows in report['cases'].values():
            self.assertEqual(len(rows),50)
            for r in rows:
                self.assertTrue(r['policy_safe'])
                self.assertLess(r['block_bottom_table_gap_m'],0.)
                self.assertTrue(np.isfinite(r['tcp_linear_velocity_world_m_s']).all())
                np.testing.assert_array_equal(r['closing_center_world_m'],np.mean(r['jaw_inner_face_centers_world_m'],axis=0))
                for contact in r['contacts']:
                    self.assertTrue(np.isfinite(contact['friction_coefficients']).all())
                    self.assertEqual(len(contact['contact_frame_wrench']),6)
                    relative=np.asarray(contact['relative_velocity_contact_frame_m_s'])
                    self.assertAlmostEqual(contact['relative_tangent_speed_m_s'],np.linalg.norm(relative[1:]))
                self.assertEqual(r['gripper_command_delta_from_close_rad'],0.)
                self.assertTrue(np.isfinite(r['raw_qpos']).all())
                self.assertTrue(np.isfinite(r['raw_qvel']).all())
                self.assertFalse(r['non_target_protected_contacts'])
                self.assertLessEqual(max((c['penetration_m'] for c in r['contacts']),default=0),.001)
        self.assertEqual(report['grasp_quality']['block_table_contact_count'],4)

        quality=report['grasp_quality']['load_evidence']
        self.assertFalse(quality['is_load_ready_gate'])
        self.assertEqual(continuous[41]['load_evidence']['optimistic_vertical_upper_bound_N'],0.)
        self.assertEqual(continuous[41]['load_evidence']['finger_vertical_force_N'],0.)
        self.assertAlmostEqual(quality['block_weight_N'],.1962)
        self.assertLess(quality['finger_vertical_force_N']/quality['block_weight_N'],.003)
        self.assertAlmostEqual(quality['finger_vertical_force_N']+report['grasp_quality']['block_table_normal_force_N'],.1962,places=5)
        self.assertLess(hold[-1]['load_evidence']['optimistic_vertical_upper_bound_N'],.1962)
        self.assertLess(hold[-1]['grippers'][0]['jaw_opening_m'],report['grasp_quality']['grippers'][0]['jaw_opening_m'])
        points=quality['finger_contacts']
        self.assertLess(points[0]['block_com_local_m'][2],-.015)
        self.assertGreater(points[1]['block_com_local_m'][2],.015)
        self.assertGreater(np.linalg.norm(quality['finger_torque_about_com_world_Nm']),.002)
        with contextlib.redirect_stdout(io.StringIO()):
            candidate=next_close_diagnostic(fixture,report)
        self.assertFalse(candidate['live_success'])
        self.assertEqual(candidate['candidate_close_stage'],17)
        self.assertTrue(candidate['close_preflight']['passed'])
        self.assertTrue(candidate['close_preflight']['live_state_unchanged'])
        self.assertTrue(all(v>0 for v in candidate['cases']['extra_close_confirm'][-1]['finger_force_N'].values()))
        self.assertEqual(candidate['failure']['phase'],'LIFT_5MM')
        self.assertEqual(candidate['failure']['step'],36)
        self.assertIn('bilateral finger contact lost',candidate['failure']['reason'])
        self.assertGreater(candidate['cases']['extra_close_continuous_lift'][-1]['block_table_normal_force_N'],0)



if __name__=='__main__':unittest.main()
