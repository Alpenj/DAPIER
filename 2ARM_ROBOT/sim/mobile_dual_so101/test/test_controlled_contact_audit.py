from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
import numpy as np
from controlled_contact_audit import contact_metrics

class NativeContactMetricTest(unittest.TestCase):
    def test_all_contacts_geom_order_and_intrinsic_torque(self):
        def c(pad, point, force, reverse=False):
            return dict(names=['red_block_geom',pad] if reverse else [pad,'red_block_geom'],
                        position_world_m=point,frame=np.eye(3).tolist(),
                        wrench_contact=force,distance_m=-1e-5)
        contacts=[c('left_pgripper_pad_1',[0,0,1],[2,0,1,0,0,.2]),
                  c('left_pgripper_pad_1',[0,0,3],[1,0,0,0,0,0]),
                  c('left_pgripper_pad_2',[0,0,-1],[3,0,0,0,0,.1],True)]
        m=contact_metrics(contacts,[0,0,0],np.eye(3))
        np.testing.assert_allclose(m['F_net_world_N'],[0,0,1])
        np.testing.assert_allclose(m['tau_COM_r_cross_f_world_Nm'],[0,8,0])
        np.testing.assert_allclose(m['full_constraint_tau_COM_world_Nm'],[0,8,.1])
        self.assertEqual(m['pads']['left_pgripper_pad_1']['contact_count'],2)
        self.assertAlmostEqual(m['pads']['left_pgripper_pad_1']['normal_weighted_centroid_world_m'][2],5/3)
        self.assertEqual(m['first_native_height_world_m'],2)
        self.assertIsNone(m['geometric_surface_overlap'])
    def test_empty_zero_force_and_nonfinite(self):
        m=contact_metrics([],np.zeros(3),np.eye(3))
        self.assertIsNone(m['first_native_height_world_m'])
        np.testing.assert_array_equal(m['F_net_world_N'],np.zeros(3))
        c=dict(names=['left_pgripper_pad_1','red_block_geom'],position_world_m=[0,0,0],frame=np.eye(3),wrench_contact=[0]*6,distance_m=0)
        self.assertIsNone(contact_metrics([c],[0]*3,np.eye(3))['pads']['left_pgripper_pad_1']['normal_weighted_centroid_world_m'])
        c['wrench_contact'][0]=float('nan')
        with self.assertRaises(ValueError):contact_metrics([c],[0]*3,np.eye(3))
class StoredAuditTest(unittest.TestCase):
    def test_all_four_cases_recompute_same_force_and_metric(self):
        import json
        data=json.loads((Path(__file__).parent/'fixtures/contact_audit.json').read_text())
        for case in data['four_state']['states'].values():
            m=case['metric']
            recomputed=contact_metrics(case['native_contact_inputs'],m['block_COM_world_m'],m['block_rotation_world'])
            for key in ['F_net_world_N','tau_COM_r_cross_f_world_Nm','full_constraint_tau_COM_world_Nm','first_native_height_world_m','first_native_height_block_m']:
                np.testing.assert_allclose(recomputed[key],m[key],rtol=0,atol=1e-14)
            # Saved world force/lever are independently checked by all-contact sums.
            force=np.sum([c['force_world_N'] for c in m['contacts']],axis=0)
            np.testing.assert_allclose(force,m['F_net_world_N'],atol=1e-14)
            self.assertEqual(sum(v['contact_count'] for v in m['pads'].values()),len(m['contacts']))
            self.assertIsNone(m['geometric_surface_overlap'])
        self.assertGreater(data['four_state']['states']['A0']['metric']['F_net_world_N'][2],.19619)
        for name in ['A1','A2','A3']:
            self.assertLess(data['four_state']['states'][name]['metric']['F_net_world_N'][2],.001)
    def test_basic_change_is_planning_not_cartesian_compensation(self):
        import json
        d=json.loads((Path(__file__).parent/'fixtures/contact_audit.json').read_text())['basic_planning_diff']
        np.testing.assert_allclose(np.array(d['same_target_xyz'])-d['same_start_xyz'],[0,0,.02],atol=1e-15)
        self.assertTrue(d['no_empirical_cartesian_offset'])
        self.assertFalse(d['real_absolute_q_copy_allowed'])
        self.assertLess(d['refined_ik']['residual_m_by_side']['left'],d['baseline_ik']['residual_m_by_side']['left'])
        self.assertNotEqual(d['baseline_duration_s'],d['refined_duration_s'])

    def test_controlled_ab_does_not_turn_unilateral_into_success(self):
        import json
        d=json.loads((Path(__file__).parent/'fixtures/contact_audit.json').read_text())['controlled_ab']
        self.assertEqual(d['pairs_executed'],1)
        self.assertEqual(d['only_option_delta'],{'noslip_iterations':[0,5]})
        self.assertEqual(d['identical_command_common_prefix_steps'],9697)
        self.assertIsNone(d['cases']['0']['failure'])
        self.assertIsNone(d['cases']['5']['first_bilateral_step'])
        self.assertEqual(d['cases']['5']['failure']['phase'],'GRASP_CONFIRM')
        for case in d['cases'].values():
            self.assertLessEqual(case['maximum_penetration_m'],.001)
            self.assertFalse(any(case['warning_counts']))
            for phase in ['close_terminal','first_confirm','last']:
                self.assertNotIn(case[phase]['phase'],['LIFT','HOLD'])

if __name__=='__main__':unittest.main()
