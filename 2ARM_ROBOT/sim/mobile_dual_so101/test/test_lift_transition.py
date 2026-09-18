import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lift_transition_diagnostic import run, next_close_diagnostic


class LiftTransitionTest(unittest.TestCase):
    def test_arm_hold_retention_is_not_unsupported_grasp(self):
        f = json.loads((Path(__file__).parent/'fixtures/lift_transition.json').read_text())
        e = f['arm_coupling_diagnostic']
        self.assertEqual(e['mode'], 'DIAGNOSTIC COPY / NOT LIVE TASK SUCCESS')
        self.assertEqual(e['lock_type'], 'SOFT_LOCK')
        self.assertFalse(e['live_success'])
        self.assertFalse(e['runtime_noslip_adopted'])
        for key in ('initial_state_bitexact', 'donor_unchanged', 'no_reset_physics',
                    'original_gripper_equalities_unchanged'):
            self.assertTrue(e[key])
        self.assertTrue(all(e['all_option_fields_exact'].values()))
        for rows in e['cases'].values():
            self.assertEqual([r['step'] for r in rows], [1, 25, 50])
            for r in rows:
                np.testing.assert_array_equal(r['target_q'], e['terminal_command'])
                self.assertTrue(np.isfinite(r['raw_qpos']).all())
                self.assertTrue(np.isfinite(r['raw_qvel']).all())
                self.assertFalse(any(r['warnings']))
                self.assertGreater(min(r['finger_force_N'].values()), 0)
                self.assertGreater(r['block_table_contact_count'], 0)
                self.assertGreater(r['block_table_normal_force_N'], .19)
                force, torque = np.zeros(3), np.zeros(3)
                for contact in r['contacts']:
                    if not any('pgripper_pad' in n for n in contact['names']):
                        continue
                    sign = 1 if contact['names'][1] == 'red_block_geom' else -1
                    frame = np.asarray(contact['contact_frame_world']).reshape(3, 3)
                    wrench = np.asarray(contact['contact_frame_wrench'])
                    f_world = sign * frame.T @ wrench[:3]
                    force += f_world
                    lever = np.asarray(contact['position_m']) - r['block_position_m']
                    torque += np.cross(lever, f_world) + sign * frame.T @ wrench[3:]
                np.testing.assert_allclose(force, r['load_evidence']['finger_force_world_N'], atol=1e-12, rtol=0)
                np.testing.assert_allclose(torque, r['load_evidence']['finger_torque_about_com_world_Nm'], atol=1e-12, rtol=0)
        arm = e['case_summaries']['terminal_hold']['initial']
        bench = e['bench_selected'][-1]
        for row in (arm, bench):
            height = np.ptp([c['point_block_COM_m'][2] for c in row['contacts']])
            self.assertAlmostEqual(height, row['height_difference_block_m'], places=12)
        # Equal friction settings do not make different grasps an arm-only comparison.
        self.assertGreater(arm['height_difference_block_m'], .03)
        self.assertLess(bench['height_difference_block_m'], .00002)
        self.assertEqual(bench['support_count'], 0)
        self.assertEqual(bench['support_Fn_N'], 0)
        np.testing.assert_allclose(bench['pad_force_world_N'], [0, 0, .1962], atol=1e-10, rtol=0)
        self.assertLess(np.linalg.norm(bench['pad_torque_COM_world_Nm']), 1e-10)

    def check_arm_noslip_evidence(self, e):
        self.assertEqual(e['mode'], 'DIAGNOSTIC COPY / NOT LIVE TASK SUCCESS')
        self.assertEqual(e['option_delta'], {'noslip_iterations': [0, 5]})
        self.assertTrue(e['donor_unchanged'])
        self.assertFalse(e['endpoint_reached'])
        rows = e['rows']
        self.assertEqual(len(rows), 50)
        self.assertEqual(e['extension_steps'], list(range(34, 51)))
        start, end = np.asarray(e['terminal_command']), np.asarray(e['target'])
        for i, row in enumerate(rows, 1):
            self.assertEqual(row['step'], i)
            t = i * .002 / e['trajectory_duration_s']
            blend = 35*t**4 - 84*t**5 + 70*t**6 - 20*t**7
            np.testing.assert_allclose(row['target_q'], start + (end-start)*blend,
                                       atol=1e-15, rtol=0)
            self.assertAlmostEqual(row['time_s'] - e['initial_full_state'][0], i*.002, places=11)
            for index in (5, 11):
                self.assertEqual(row['target_q'][index], start[index])
            self.assertTrue(row['policy_safe'])
            self.assertGreaterEqual(row['measured_clearance_m'], .03)
            self.assertLessEqual(row['maximum_penetration_m'], .001)
            self.assertFalse(any(row['warnings']))
            for name, force in row['finger_force_N'].items():
                pad = 'left_pgripper_pad_' + name[-1]
                observed = sum(c['normal_force_N'] for c in row['contacts'] if pad in c['names'])
                self.assertAlmostEqual(force, observed, places=12)
            table = [c for c in row['contacts'] if 'table' in c['names']]
            self.assertEqual(row['block_table_contact_count'], len(table))
            self.assertAlmostEqual(row['block_table_normal_force_N'],
                                   sum(c['normal_force_N'] for c in table), places=12)
        first_loss = next(row['step'] for row in rows if min(row['finger_force_N'].values()) <= 0)
        self.assertEqual(first_loss, e['first_blocker_step'])
        self.assertEqual(first_loss, 33)
        # A single separated sample is not sustained load-bearing lift evidence.
        separated = [row['step'] for row in rows if row['block_table_contact_count'] == 0
                     and row['block_table_normal_force_N'] == 0 and row['block_bottom_table_gap_m'] > 0]
        self.assertEqual(separated, [32])
        self.assertGreater(rows[32]['block_table_normal_force_N'], 0)
        self.assertLess(rows[-1]['block_lift_m'], 0)

    def test_arm_noslip_copy_does_not_establish_load_bearing(self):
        fixture = json.loads((Path(__file__).parent/'fixtures/lift_transition.json').read_text())
        evidence = fixture['arm_noslip_diagnostic']
        self.check_arm_noslip_evidence(evidence)
        for mutation in ('force', 'support', 'command', 'success', 'clearance'):
            bad = copy.deepcopy(evidence)
            if mutation == 'force':
                bad['rows'][32]['finger_force_N']['jaw_1'] = .1
            elif mutation == 'support':
                bad['rows'][32]['block_table_contact_count'] = 0
            elif mutation == 'command':
                bad['rows'][0]['target_q'][5] -= .001
            elif mutation == 'success':
                bad['endpoint_reached'] = True
            else:
                bad['rows'][0]['measured_clearance_m'] = .029
            with self.assertRaises(AssertionError):
                self.check_arm_noslip_evidence(bad)

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
        self.assertAlmostEqual(abs(points[1]['contact_world_m'][2]-points[0]['contact_world_m'][2]),.031057826,places=8)
        expected_body_mm=([2.571241,7.059810,-41.002735],[-2.571192,-15.069648,-16.002898])
        for point, expected in zip(points, expected_body_mm):
            np.testing.assert_allclose(np.asarray(point['contact_pad_body_local_m'])*1000,expected,atol=1e-6,rtol=0)
            self.assertTrue(point['pad_mesh'])
            self.assertTrue(point['pad_body'].startswith('left_'))
            self.assertTrue(np.isfinite(point['contact_pad_geom_local_m']).all())
            self.assertTrue(np.isfinite(point['contact_pad_body_local_m']).all())
            self.assertGreaterEqual(point['pad_geom_id'],0)
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
