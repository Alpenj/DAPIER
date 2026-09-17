import contextlib
import io
import json
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lift_transition_diagnostic import run


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


if __name__=='__main__':unittest.main()
