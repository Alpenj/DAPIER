from pathlib import Path
import json
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from postcontact_squeeze_audit import closing_budget, matched_target, experiment


class SqueezeAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = json.loads((Path(__file__).parent/'fixtures/postcontact_squeeze.json').read_text())

    def test_budget_is_closing_amplitude_not_phase_name(self):
        r=self.r; a,b=(r['cases'][k] for k in ('A0','A2'))
        self.assertTrue(a['confirm_command_unchanged'] and b['confirm_command_unchanged'])
        self.assertEqual(closing_budget(b['close_end'],b['confirm_end']),0)
        self.assertGreater(a['measured_closing_rad'],b['measured_closing_rad'])
        self.assertGreater(a['opening_reduction_m'],b['opening_reduction_m'])
        q=matched_target(a['first_bilateral'],a['confirm_end'],b['first_bilateral'],b['confirm_end'],r['actuator_contract']['ctrlrange'])
        self.assertAlmostEqual(q,r['matched_target_rad'],places=14)
        self.assertAlmostEqual(b['first_bilateral']['command_rad']-q,a['command_budget_rad'],places=14)

    def test_no_experiment_without_smaller_budget_or_valid_bounds(self):
        a={'command_rad':1.0};b={'command_rad':.9}
        with self.assertRaises(ValueError):matched_target(a,b,a,b,[0,2])
        with self.assertRaises(ValueError):matched_target(a,b,a,a,[.95,2])
        with self.assertRaises(ValueError):matched_target(a,{'command_rad':float('nan')},a,a,[0,2])
        with self.assertRaises(ValueError):
            experiment({'less_actual_squeeze':False},[],None,None,None,None,Path('/nonexistent-dapier-test-output'))

    def test_existing_evidence_cannot_be_overwritten(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory);(out/'experiment.json').write_text('preserve')
            with self.assertRaises(FileExistsError):experiment({},[],None,None,None,None,out)
            self.assertEqual((out/'experiment.json').read_text(),'preserve')

    def test_one_ab_keeps_contract_and_does_not_claim_lift(self):
        r=self.r
        self.assertEqual(r['physics_runs'],2)
        self.assertTrue(r['same_initial_full_state'] and r['ab_same_options'] and r['baseline_bitwise_reproduced'])
        self.assertFalse(r['task_success'] or r['load_bearing_verified'])
        for case in r['ab'].values():
            self.assertIsNone(case['failure'])
            self.assertEqual(case['noslip'],0)
            self.assertEqual(case['confirm_steps'],50)
            self.assertTrue(case['model_unchanged'] and case['other_commands_unchanged'] and case['confirm_ctrl_unchanged'])
            self.assertFalse(any(case['warning_counts_max']))
            self.assertLess(case['max_penetration_m'],.001)
            self.assertLess(case['max_tcp_error_m'],.0005)
            self.assertLess(case['max_actuator_force_Nm'],r['actuator_contract']['forcerange'][1])
        base,new=(r['ab'][k]['final'] for k in ('baseline','matched_budget'))
        old=[x['summed_normal_force_N'] for x in base['normalized']['pads'].values()]
        force=[x['summed_normal_force_N'] for x in new['normalized']['pads'].values()]
        self.assertGreater(min(force),max(old))
        table=lambda row:sum(c['wrench_contact'][0] for c in row['contacts'] if 'table' in c['names'])
        self.assertGreater(table(new),table(base))
        np.testing.assert_array_equal(np.delete(base['ctrl'],5),np.delete(new['ctrl'],5))

if __name__=='__main__': unittest.main()
