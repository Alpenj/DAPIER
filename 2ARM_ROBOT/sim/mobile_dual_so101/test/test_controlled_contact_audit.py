from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
import numpy as np
from controlled_contact_audit import contact_metrics, locked_step

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
class ArmLockTest(unittest.TestCase):
    def test_repeated_projection_leaves_jaw_block_and_control_physical(self):
        from types import SimpleNamespace
        d=SimpleNamespace(qpos=np.arange(8,dtype=float),qvel=np.ones(8),ctrl=np.array([.4,.5]))
        qa=np.array([0,2]);va=np.array([0,2]);reference=d.qpos[qa].copy()
        def step(model,data):
            np.testing.assert_array_equal(data.qpos[qa],reference)
            np.testing.assert_array_equal(data.qvel[va],[0,0])
            data.qpos+=.1;data.qvel+=.2
        for _ in range(3):
            locked_step(None,d,reference,qa,va,step)
            np.testing.assert_array_equal(d.qpos[qa],reference)
            np.testing.assert_array_equal(d.qvel[va],[0,0])
        np.testing.assert_allclose(d.qpos[[1,3,4,5,6,7]],np.array([1,3,4,5,6,7])+.3)
        np.testing.assert_allclose(d.qvel[[1,3,4,5,6,7]],1.6)
        np.testing.assert_array_equal(d.ctrl,[.4,.5])

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

    def test_preclose_lock_preserves_weak_grasp_and_contact_chronology(self):
        import json
        d=json.loads((Path(__file__).parent/'fixtures/preclose_arm_lock.json').read_text())
        self.assertEqual(d['locked_runs'],1)
        self.assertFalse(d['normal_arm_rerun'])
        self.assertFalse(d['task_success'])
        for key in ['same_preclose_fullstate','same_compiled_arrays','same_options']:
            self.assertTrue(d[key])
        self.assertEqual(d['identical_commands_steps'],9696)
        self.assertEqual(d['branch'],'B_WEAK_GRASP_REMAINS')
        self.assertIsNone(d['failure'])
        for name,case in d['cases'].items():
            row=case['final_row']
            m=contact_metrics(row['contacts'],row['block_com_world_m'],row['block_rotation'])
            forces=[m['pads'][p]['summed_normal_force_N'] for p in ('left_pgripper_pad_1','left_pgripper_pad_2')]
            np.testing.assert_allclose(forces,case['normal_force_N'],rtol=0,atol=1e-14)
            self.assertTrue(case['bilateral'])
            self.assertEqual(case['phase'],'CLOSE')
            self.assertLess(case['maximum_penetration_m'],.001)
            self.assertFalse(any(case['warning_counts']))
            trace=d['first_contact_trace'][name]
            self.assertLess(trace['first_positive_force_steps'][1],trace['first_positive_force_steps'][0])
            self.assertAlmostEqual(trace['force_delta_pad1_minus_pad2_s'],.506,places=10)
            self.assertGreater(trace['events'][-1]['table']['summed_normal_N'],.1962)
        locked=d['cases']['locked'];normal=d['cases']['normal']
        np.testing.assert_array_equal(locked['max_arm_displacement_rad'],np.zeros(5))
        self.assertEqual(locked['max_tcp_displacement_m'],0)
        self.assertGreater(normal['max_tcp_displacement_m'],0)
        # Stored negative result: projection did not restore bench-like force.
        self.assertLess(sum(locked['normal_force_N'])/sum(normal['normal_force_N']),1.02)
        p=d['planned_error_projection']
        axes=p['target_axes'];e=np.array(p['error_vector_world_m'])
        for name,axis in axes.items():
            self.assertAlmostEqual(e@axis,p['components_m'][name],places=14)
        self.assertAlmostEqual(np.linalg.norm(list(p['components_m'].values())),p['magnitude_m'],places=14)
        self.assertLess(abs(p['components_m']['closing']),p['magnitude_m']/2)

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
