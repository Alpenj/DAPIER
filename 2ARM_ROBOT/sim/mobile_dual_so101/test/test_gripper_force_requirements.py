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


if __name__ == '__main__':
    unittest.main()
