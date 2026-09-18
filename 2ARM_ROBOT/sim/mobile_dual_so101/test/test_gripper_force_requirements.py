"""Frozen gripper-only equilibrium evidence, not a runtime grasp-success gate."""
import copy
import json
from pathlib import Path
import unittest

import numpy as np


def verify_requirement(case, objective):
    com = np.asarray(case['com_world_m'], dtype=float)
    gravity = np.asarray(case['gravity_m_s2'], dtype=float)
    mass = case['mass_kg']
    assert com.shape == gravity.shape == (3,)
    assert np.isfinite(np.r_[com, gravity, mass]).all() and mass > 0
    target = np.r_[-mass * gravity, np.zeros(3)]
    result = case['objectives'][objective]
    contacts, forces = case['contacts'], result['contact_forces']
    assert contacts and len(contacts) == len(forces)
    nu = np.asarray(result['lower_certificate']['covector_world'], dtype=float)
    assert nu.shape == (6,) and np.isfinite(nu).all()
    total = np.zeros(6)
    loads, supports = {}, {}
    for contact, witness in zip(contacts, forces):
        frame = np.asarray(contact['frame'], dtype=float)
        mu = np.asarray(contact['mu3'], dtype=float)
        point = np.asarray(contact['point'], dtype=float)
        sign = contact['sign']
        assert frame.shape == (3, 3) and mu.shape == point.shape == (3,)
        assert np.isfinite(np.r_[frame.ravel(), mu, point]).all()
        assert sign in (-1, 1) and np.all(mu > 0)
        assert contact['condim'] == 4 and contact['cone'] == 'elliptic'
        np.testing.assert_allclose(frame @ frame.T, np.eye(3), atol=1e-9, rtol=0)
        force = np.column_stack((sign * frame[0], sign * mu[0] * frame[1],
                                 sign * mu[1] * frame[2], np.zeros(3)))
        moment = np.cross(point - com, force.T).T
        moment[:, 3] += sign * mu[2] * frame[0]
        basis = np.vstack((force, moment))
        n = witness['Fn_N']
        u = np.asarray(witness['normalized_tangent_torsion_N'], dtype=float)
        assert u.shape == (3,) and np.isfinite(np.r_[n, u]).all()
        assert n >= 0 and np.linalg.norm(u) <= n + 1e-9
        total += basis @ np.r_[n, u]
        finger = contact['finger']
        loads[finger] = loads.get(finger, 0.) + n
        a = nu @ basis
        support = float(a[0] + np.linalg.norm(a[1:]))
        supports[finger] = max(supports.get(finger, 0.), support)
        if objective == 'minsum_Fn':
            assert support <= 1 + 1e-9
    # Full COM torque balance matters; a scalar mu*sum(Fn) check is insufficient.
    np.testing.assert_allclose(total, target, atol=1e-9, rtol=0)
    if objective == 'minmax_per_finger':
        assert sum(supports.values()) <= 1 + 1e-9
    upper = sum(loads.values()) if objective == 'minsum_Fn' else max(loads.values())
    lower = float(nu @ target)
    assert abs(upper - result['claimed_witness_objective_N']) < 1e-9
    assert abs(lower - result['lower_certificate']['lower_bound_N']) < 1e-9
    assert 0 <= lower <= upper + 1e-9
    return lower, upper


class GripperForceRequirementsTest(unittest.TestCase):
    def test_exact_cone_primal_and_dual_witnesses(self):
        fixture = json.loads((Path(__file__).parent / 'fixtures/gripper_force_requirements.json').read_text())
        for case in fixture['cases']:
            for objective in ('minsum_Fn', 'minmax_per_finger'):
                with self.subTest(case=case['id'], objective=objective):
                    verify_requirement(case, objective)
                    for mutation in ('force', 'torque', 'dual', 'nan', 'missing'):
                        bad = copy.deepcopy(case)
                        if mutation == 'force':

                            for force in bad['objectives'][objective]['contact_forces']:
                                force['Fn_N'] *= .1
                        elif mutation == 'torque':
                            bad['com_world_m'][0] += .01
                        elif mutation == 'dual':
                            bad['objectives'][objective]['lower_certificate']['covector_world'] = [0.] * 6
                        elif mutation == 'nan':
                            bad['mass_kg'] = float('nan')
                        else:
                            bad['objectives'][objective]['contact_forces'].pop()
                        with self.assertRaises(AssertionError):
                            verify_requirement(bad, objective)



class GripperSlipEvidenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture = json.loads((Path(__file__).parent / 'fixtures/gripper_force_requirements.json').read_text())
        cls.evidences = [fixture['observed_slip'], fixture['normal_close_slip']]

    def check_event(self, e):
        force, moment = np.zeros(3), np.zeros(3)
        rotation = np.asarray(e['body_rotation_world'])
        inertia = np.asarray(e['body_inertia_diag_kg_m2'])
        omega_body = np.asarray(e['body_local_angular_velocity_pre'])
        omega = rotation @ omega_body
        np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-9, rtol=0)
        self.assertEqual(e['support_contact_count'], 0)
        self.assertEqual(e['support_force_N'], 0.)
        self.assertGreater(len(e['contacts']), 0)
        for c in e['contacts']:
            frame, w = np.asarray(c['frame']), np.asarray(c['contact_wrench'])
            lever, mu = np.asarray(c['lever_COM_world_m']), np.asarray(c['mu3'])
            self.assertTrue(np.isfinite(np.r_[frame.ravel(), w, lever, mu]).all())
            np.testing.assert_allclose(frame @ frame.T, np.eye(3), atol=1e-9, rtol=0)
            self.assertIn(c['sign'], (-1, 1))
            self.assertGreater(w[0], 0.)
            self.assertTrue(np.all(mu > 0))
            self.assertLessEqual(np.linalg.norm(w[[1, 2, 3]] / mu), w[0] + 1e-9)
            f = c['sign'] * frame.T @ w[:3]
            force += f
            moment += np.cross(lever, f) + c['sign'] * frame.T @ w[3:]
            velocity = np.asarray(e['block_linear_velocity_world_pre']) + np.cross(omega, lever) - c['pad_velocity_world_m_s']
            normal = c['sign'] * frame[0]
            slip = np.linalg.norm(velocity - normal * (normal @ velocity)) * 1000
            self.assertAlmostEqual(slip, c['claimed_tangent_relative_speed_mm_s'], places=9)
        # A feasible static wrench does not mean the observed motion is at rest.
        np.testing.assert_allclose(force, e['claimed_force_world_N'], atol=1e-12, rtol=0)
        np.testing.assert_allclose(moment, e['claimed_moment_COM_Nm'], atol=1e-12, rtol=0)
        np.testing.assert_allclose(force + e['mass_kg'] * np.asarray(e['gravity']),
                                   e['mass_kg'] * np.asarray(e['qacc_linear_world']), atol=1e-12, rtol=0)
        np.testing.assert_allclose(moment, rotation @ (inertia * e['qacc_angular_body_local'] +
                                   np.cross(omega_body, inertia * omega_body)), atol=1e-12, rtol=0)
        dt = e['post_time_s'] - e['force_evaluation_time_s']
        self.assertGreater(dt, 0.)
        vz = (e['block_linear_velocity_world_pre'][2] + dt * e['qacc_linear_world'][2]) * 1000
        self.assertAlmostEqual(vz, e['post_world_vz_mm_s'], places=9)
        self.assertLess(vz, 0.)

    def test_all_contact_wrench_and_nonzero_slip(self):
        for event in [event for evidence in self.evidences for event in evidence['events']]:
            with self.subTest(event=event['id']):
                self.check_event(event)
                for mutation in ('contact', 'torque', 'support_contact', 'support_force', 'velocity'):
                    bad = copy.deepcopy(event)
                    if mutation == 'contact':
                        bad['contacts'].pop()
                    elif mutation == 'torque':
                        bad['contacts'][0]['lever_COM_world_m'][0] += .01
                    elif mutation == 'support_contact':
                        bad['support_contact_count'] = 1
                    elif mutation == 'support_force':
                        bad['support_force_N'] = 1e-9
                    else:
                        bad['post_world_vz_mm_s'] = 0.
                    with self.assertRaises(AssertionError):
                        self.check_event(bad)

    def test_persistent_slip_despite_bilateral_force(self):
        for evidence in self.evidences:
            chronology = evidence['chronology']
            self.assertTrue(chronology['all_rows_bilateral'])
            self.assertEqual(chronology['other_block_contacts_total'], 0)
            for window in chronology['windows']:
                moments = window['regression_centered_moments']
                slope = moments['sum_t_centered_z_centered_mm_s'] / moments['sum_t_centered_squared_s2']
                self.assertAlmostEqual(slope, window['claimed_z_regression_slope_mm_s'], places=12)
                self.assertLess(slope, 0.)
                self.assertLess(window['vz_range_mm_s'][1], 0.)
                self.assertLess(window['end_z_mm'], window['start_z_mm'])
                self.assertEqual(window['bilateral_count'], window['sample_count'])
                self.assertGreater(min(window['Fn_min_N']), 0.)
            self.assertLess(chronology['final_z_mm'], chronology['initial_z_mm'])
            self.assertLess(chronology['final_vz_mm_s'], 0.)



class SolverContactDiagnosticTest(unittest.TestCase):
    def verify(self, evidence):
        cases = evidence['cases']
        self.assertEqual(len(cases), 3)
        expected_changes = ({}, {'iterations': 1000, 'tolerance': 1e-12},
                            {'noslip_iterations': 5})
        for case, changes in zip(cases, expected_changes):
            expected = dict(evidence['baseline_options'], **changes)
            self.assertEqual(case['options'], expected)
            for key in ('start_sha256', 'model_snapshot_sha256', 'commands_sha256'):
                self.assertEqual(case[key], cases[0][key])
            self.assertEqual(case['support_count_max'], 0)
            self.assertEqual(case['support_force_max_N'], 0.)
            self.assertEqual(case['warnings'], 0)
            self.assertTrue(case['all_bilateral'])
            self.assertTrue(np.isfinite(case['max_penetration_m']))
            self.assertLessEqual(case['max_penetration_m'], .001)
            self.assertAlmostEqual(sum(w['duration_s'] for w in case['windows']), 3., places=9)
            for w in case['windows']:
                self.assertTrue(np.isfinite(list(w.values())).all())
                self.assertGreater(w['sum_tt'], 0)
                self.assertAlmostEqual(w['sum_tz'] / w['sum_tt'], w['slope_mm_s'], places=10)
                self.assertAlmostEqual(w['end_z_mm'] - w['start_z_mm'],
                                       w['integrated_vz_mm'], places=9)
        # Solver sensitivity is evidence about a copied bench, never a task-success gate.
        self.assertEqual(cases[0]['windows'], cases[1]['windows'])
        self.assertEqual(cases[0]['terminal_vz_mm_s'], cases[1]['terminal_vz_mm_s'])
        self.assertGreater(abs(cases[0]['windows'][-1]['slope_mm_s']),
                           abs(cases[2]['windows'][-1]['slope_mm_s']))
        self.assertNotEqual(cases[2]['terminal_vz_mm_s'], 0.)

    def test_same_state_parameter_isolation_and_observed_slip(self):
        evidence = json.loads((Path(__file__).parent / 'fixtures/gripper_force_requirements.json').read_text())['solver_contact_diagnostic']
        self.verify(evidence)
        for mutation in ('state', 'command', 'model', 'option', 'support', 'integral'):
            bad = copy.deepcopy(evidence)
            case = bad['cases'][2]
            if mutation in ('state', 'command', 'model'):
                key = {'state': 'start_sha256', 'command': 'commands_sha256',
                       'model': 'model_snapshot_sha256'}[mutation]
                case[key] = 'corrupted'
            elif mutation == 'option':
                case['options']['impratio'] += 1
            elif mutation == 'support':
                case['support_force_max_N'] = 1e-9
            else:
                case['windows'][0]['integrated_vz_mm'] = 0.
            with self.assertRaises(AssertionError):
                self.verify(bad)


if __name__ == '__main__':
    unittest.main()
