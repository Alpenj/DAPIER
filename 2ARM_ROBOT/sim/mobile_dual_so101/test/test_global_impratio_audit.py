from pathlib import Path
import json
import math
import unittest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from global_impratio_audit import outcome


class GlobalCandidateContractTest(unittest.TestCase):
    def test_formation_or_lift_failure_is_not_hold_slip_or_success(self):
        self.assertEqual(outcome(False,False,False),'B')
        self.assertEqual(outcome(True,False,False),'INCOMPLETE_LIFT')
        self.assertEqual(outcome(True,True,False),'C')
        self.assertEqual(outcome(True,True,True),'A')

    def test_saved_candidate_is_one_continuous_fixed_configuration_path(self):
        r=json.loads((Path(__file__).parent/'fixtures/global_impratio.json').read_text())
        self.assertTrue(r['continuous_state_verified'] and r['options_constant_every_step'])
        self.assertTrue(r['model_arrays_unchanged'])
        a,b=r['original_options'],r['fixed_options']
        self.assertEqual([k for k in a if a[k]!=b[k]],['impratio'])
        self.assertEqual((b['noslip_iterations'],b['impratio']),(0,100.))
        self.assertFalse(r['live_task_success'] or r['hardware_execution'] or r['runtime_adopted'])
        self.assertTrue(r['times_contiguous'] and r['frozen_lift_endpoints'])
        self.assertEqual(r['close_commands_rad'][-1],1.716112023423022)
        self.assertLess(r['close_commands_rad'][-1],r['close_commands_rad'][-2])

    def test_saved_classification_requires_existing_gates(self):
        r=json.loads((Path(__file__).parent/'fixtures/global_impratio.json').read_text())
        self.assertEqual(r['classification'],outcome(r['close_confirm_pass'],r['lift_pass'],r['copied_hold_pass']))
        if r['classification']=='A':
            self.assertIsNone(r['failure'])
            self.assertEqual(r['hold_steps'],1500)
            self.assertTrue(r['hold_all_supported'] and r['hold_table_always_absent'])
            self.assertGreaterEqual(r['hold_min_bottom_m'],.03)
            self.assertGreaterEqual(r['hold_final_counter_s'],3.)
        else:
            self.assertIsNotNone(r['failure'])
            self.assertFalse(r['copied_hold_pass'])
        for g in r['waypoint_gates']:
            if not r['failure'] or g['time_s']<r['failure']['time_s']:
                self.assertLessEqual(g['position_error_m'],.0005)
                self.assertLessEqual(g['approach_error_rad'],math.radians(2))
                self.assertLessEqual(g['closing_error_rad'],math.radians(15))



    def test_first_table_free_sample_does_not_hide_recontact(self):
        r=json.loads((Path(__file__).parent/'fixtures/global_impratio.json').read_text())
        self.assertFalse(r['table_absent_since_first_detachment'])
        self.assertGreater(len(r['lift_table_recontacts']),0)
        self.assertGreater(r['permanent_table_free_time_s'],r['first_table_detachment_time_s'])
        self.assertTrue(r['hold_table_always_absent'])
        self.assertGreater(r['hold_initial_counter_s'],0.)
        self.assertGreaterEqual(r['hold_final_counter_s']-r['hold_initial_counter_s'],3.-1e-12)

if __name__=='__main__':unittest.main()
