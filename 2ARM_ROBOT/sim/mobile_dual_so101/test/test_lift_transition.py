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
        self.assertEqual(report['confirmation_replay_max_qpos_difference'],0.)
        self.assertEqual(report['first_lift_replay_max_qpos_difference'],0.)
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
        for rows in report['cases'].values():
            self.assertEqual(len(rows),50)
            for r in rows:
                self.assertTrue(r['policy_safe'])
                self.assertEqual(r['gripper_command_delta_from_close_rad'],0.)
                self.assertTrue(np.isfinite(r['raw_qpos']).all())
                self.assertTrue(np.isfinite(r['raw_qvel']).all())
                self.assertFalse(r['non_target_protected_contacts'])
                self.assertLessEqual(max((c['penetration_m'] for c in r['contacts']),default=0),.001)
        self.assertEqual(report['grasp_quality']['block_table_contact_count'],4)


if __name__=='__main__':unittest.main()
