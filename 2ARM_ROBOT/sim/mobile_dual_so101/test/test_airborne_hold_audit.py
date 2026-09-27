from pathlib import Path
import json
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from airborne_hold_audit import elliptic_utilization


class ContactConeMetricTest(unittest.TestCase):
    def test_anisotropic_sliding_and_torsional_components(self):
        c=dict(wrench_contact=[2.,.6,1.6,0.,0.,0.],friction=[.5,1.,.01,.001,.001],condim=3)
        self.assertAlmostEqual(elliptic_utilization(c),1.)  # normalized tangent (.6,.8)
        c.update(condim=4,wrench_contact=[2.,0.,0.,.02,0.,0.])
        self.assertAlmostEqual(elliptic_utilization(c),1.)  # torsion also consumes the cone
        c.update(condim=6,wrench_contact=[2.,0.,0.,0.,.0012,.0016])
        self.assertAlmostEqual(elliptic_utilization(c),1.)

    def test_unloaded_or_invalid_contact_is_not_false_safe_evidence(self):
        c=dict(wrench_contact=[0.,0.,0.,0.,0.,0.],friction=[1.]*5,condim=3)
        self.assertIsNone(elliptic_utilization(c))
        c['wrench_contact'][0]=float('nan')
        with self.assertRaises(ValueError):elliptic_utilization(c)
        c['wrench_contact']=[1.,1.,0.,0.,0.,0.];c['friction'][0]=0
        with self.assertRaises(ValueError):elliptic_utilization(c)


class AirborneEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.r=json.loads((Path(__file__).parent/'fixtures/airborne_hold.json').read_text())

    def test_identical_checkpoint_command_and_single_option_change(self):
        r=self.r;a,b=(r['cases'][k] for k in ('0','5'))
        self.assertTrue(r['same_initial_full_state'] and r['checkpoint_prior_trace_bitwise_match'])
        self.assertEqual(r['checkpoint_replayed_steps'],435)
        self.assertTrue(a['prior_hold_bitwise_match'])
        self.assertEqual(a['initial_bottom_m'],b['initial_bottom_m'])
        self.assertEqual([k for k in a['options'] if a['options'][k]!=b['options'][k]],['noslip_iterations'])
        self.assertEqual((a['options']['noslip_iterations'],b['options']['noslip_iterations']),(0,5))
        for c in (a,b):
            self.assertTrue(c['model_arrays_unchanged'] and c['all_commands_identical'])
            self.assertEqual(c['steps'],1500)
            self.assertEqual(c['duration_s'],3.)

    def test_contact_alone_does_not_hide_baseline_height_failure(self):
        r=self.r;a,b=(r['cases'][k] for k in ('0','5'))
        self.assertEqual(r['required_world_bottom_m'],.03)
        self.assertFalse(a['copied_hold_pass'] or a['all_steps_lift_supported'])
        self.assertLess(a['final_bottom_m'],.03)
        self.assertIsNotNone(a['first_height_failure_elapsed_s'])
        self.assertTrue(b['copied_hold_pass'] and b['all_steps_lift_supported'])
        self.assertGreaterEqual(b['minimum_bottom_m'],.03)
        self.assertGreaterEqual(b['final_hold_counter_s']-r['inherited_hold_counter_s'],3.-1e-12)
        self.assertLess(abs(b['bottom_displacement_m']),abs(a['bottom_displacement_m']))
        self.assertLess(abs(b['final_vz_m_s']),abs(a['final_vz_m_s']))
        self.assertFalse(r['live_task_success'] or r['hardware_execution'] or r['runtime_adopted'])

    def test_force_timing_and_existing_safety_contract(self):
        for c in self.r['cases'].values():
            self.assertIsNone(c['failure'])
            self.assertTrue(c['table_always_absent'])
            self.assertGreater(min(c['minimum_forces_N']),0)
            self.assertLess(c['maximum_tcp_error_m'],.0005)
            self.assertLess(c['max_penetration_m'],.001)
            self.assertFalse(c['any_saturation'] or any(c['max_warning_counts']))
            self.assertLessEqual(c['max_elliptic_utilization'],1.)
            # Applied solver-cache forces, not post-forward gate forces, balance m*dv/dt.
            self.assertLess(c['force_acceleration_residual_max_N'],1e-12)


if __name__=='__main__':unittest.main()
