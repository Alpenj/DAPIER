from pathlib import Path
import json
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from postcontact_squeeze_audit import closing_budget, matched_target, experiment, lift_outcome


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



class LiftClassificationTest(unittest.TestCase):
    def row(self, phase='LIFT_5MM', table=1, forces=(.45,.45), supported=False, hold=0):
        return dict(phase=phase,table_contact_count=table,table_force_N=.1962 if table else 0.,
                    finger_forces_N=list(forces),lift_supported=supported,continuous_hold_s=hold)

    def test_success_needs_actual_support_free_three_seconds(self):
        r=self.row('HOLD',0,supported=True,hold=3)
        self.assertEqual(lift_outcome([r],True),'A')
        for key,value in [('continuous_hold_s',2.99),('table_contact_count',1),('table_force_N',.01),('finger_forces_N',[.4,0])]:
            self.assertNotEqual(lift_outcome([dict(r,**{key:value})],True),'A')

    def test_loss_before_detachment_vs_no_detachment(self):
        self.assertEqual(lift_outcome([self.row(forces=(0,.4))],False),'B')
        self.assertEqual(lift_outcome([self.row()],False),'C')

    def test_recorded_lift_survives_detachment_but_tcp_gate_is_not_relaxed(self):
        r=json.loads((Path(__file__).parent/'fixtures/matched_budget_lift.json').read_text())
        self.assertEqual(r['classification'],'UNCLASSIFIED_PARTIAL_LIFT')
        self.assertEqual(r['failure'],'measured waypoint position tolerance failed')
        self.assertTrue(r['matched_reproduction_exact'] and r['model_unchanged'])
        self.assertEqual(r['noslip'],0)
        self.assertEqual(r['phase_steps']['CLOSE'],188)
        self.assertEqual(r['phase_steps']['GRASP_CONFIRM'],50)
        self.assertEqual(r['first_table_detachment']['lift_step'],44)
        self.assertAlmostEqual(r['first_table_detachment']['elapsed_s'],.088)
        self.assertIsNone(r['first_force_loss'])
        self.assertIsNone(r['first_geometric_contact_loss'])
        self.assertTrue(r['table_recontact_after_detachment'])
        self.assertEqual(r['first_table_recontact']['lift_step'],45)
        self.assertEqual(r['continuous_support_free_start']['lift_step'],46)
        self.assertAlmostEqual(r['continuous_support_free_start']['elapsed_s'],.092)
        self.assertAlmostEqual(r['observed_support_free_s'],1.094)
        self.assertGreater(min(r['minimum_forces_after_detachment_N']),0)
        self.assertTrue(r['gripper_command_constant_during_lift'])
        self.assertFalse(r['any_actuator_saturation'] or any(r['maximum_warning_counts']))
        self.assertLess(r['maximum_penetration_m'],.001)
        self.assertLess(r['maximum_bottom_lift_m'],.03)
        self.assertEqual(r['final']['table_contact_count'],0)
        self.assertEqual(r['final']['table_force_N'],0)
        gates={g['phase']:g for g in r['waypoint_gates']}
        self.assertLess(gates['LIFT_5MM']['position_error_m'],.0005)
        self.assertGreater(gates['LIFT_15MM']['position_error_m'],.0005)
        self.assertTrue(all(g['collision_safe'] for g in gates.values()))
        self.assertNotIn('HOLD',r['phase_steps'])
        self.assertFalse(r['copied_lift_success'] or r['live_task_success'] or r['hardware_execution'])

    def test_hold_slip_is_not_pre_detachment_loss(self):
        r=self.row(table=0,supported=True)
        self.assertEqual(lift_outcome([r,self.row('HOLD',0,forces=(0,.4))],False),'D')
        self.assertEqual(lift_outcome([r,self.row(table=0,forces=(0,.4))],False),'UNCLASSIFIED_PARTIAL_LIFT')



class LiftEndpointAuditTest(unittest.TestCase):
    def test_planning_and_tracking_residuals_add_as_vectors(self):
        from lift_endpoint_audit import error_components
        r=error_components(np.zeros(3),[.0004,0,0],[.0001,0,0])
        self.assertAlmostEqual(r['planned_m'],.0004)
        self.assertAlmostEqual(r['tracking_m'],.0003)
        self.assertAlmostEqual(r['total_m'],.0001)
        np.testing.assert_allclose(np.array(r['planned_vector_m'])+r['tracking_vector_m'],r['total_vector_m'])

    def test_saved_same_command_hold_never_passes_endpoint_gate(self):
        r=json.loads((Path(__file__).parent/'fixtures/lift15_endpoint_hold.json').read_text())
        self.assertEqual(r['physics_steps'],250)
        self.assertAlmostEqual(r['duration_s'],.5)
        self.assertEqual(r['options']['noslip_iterations'],0)
        self.assertTrue(r['model_unchanged'] and r['command_unchanged'])
        np.testing.assert_array_equal(r['target_q'],r['command_q'])
        np.testing.assert_array_equal(r['initial']['ctrl'],r['final']['ctrl'])
        np.testing.assert_array_equal(r['planned_xyz_m'],r['command_xyz_m'])
        self.assertLess(r['initial']['errors']['planned_m'],.0005)
        self.assertGreater(r['minimum_tcp_error_m'],.0005)
        self.assertIsNone(r['first_gate_pass_s'])
        self.assertFalse(r['continued_to_30mm'] or r['task_success'])
        # A stationary joint velocity does not remove steady-state tracking error.
        self.assertLess(max(abs(x) for x in r['final']['joint_qvel'][:5]),1e-5)
        self.assertGreater(r['final']['errors']['tracking_m'],.0002)

    def test_retained_contacts_are_not_a_success_or_stable_hold_claim(self):
        r=json.loads((Path(__file__).parent/'fixtures/lift15_endpoint_hold.json').read_text())
        self.assertIsNone(r['failure'])  # Physical safety checks passed, endpoint pose gate did not.
        self.assertTrue(r['table_always_absent'])
        self.assertGreater(min(r['minimum_finger_forces_N']),0)
        self.assertFalse(r['any_saturation'] or any(r['max_warning_counts']))
        self.assertLess(r['max_penetration_m'],.001)
        self.assertLess(r['additional_block_descent_m'],0)
        self.assertLess(r['final']['block_vz_m_s'],0)
        self.assertLess(r['final']['bottom_lift_m'],.03)


if __name__=='__main__': unittest.main()
