"""Offline force/torque evidence; never a task gate."""
import copy
import json
from pathlib import Path
import unittest
import numpy as np

def verify_certificate(case, atol=1e-10):
    nu=np.asarray(case['equilibrium_covector_world'],dtype=float)
    mass=case['mass_kg'];gravity=np.asarray(case['gravity'],dtype=float)
    com=np.asarray(case['com_world_m'],dtype=float)
    if nu.shape!=(6,) or not np.all(np.isfinite(nu)) or not np.isfinite(mass) or mass<=0 or gravity.shape!=(3,) or com.shape!=(3,) or not np.all(np.isfinite(np.r_[gravity,com])):raise ValueError('invalid covector/mass/gravity/com')
    target=-np.r_[mass*gravity,np.zeros(3)]
    alpha=float(nu@target)
    if alpha<=0:raise ValueError('positive gravity normalization required')
    supports=[]
    for c in case['contacts']:
        frame=np.asarray(c['frame'],dtype=float);mu=np.asarray(c['mu3'],dtype=float);cap=c['Fn_cap'];sign=c['sign']
        if frame.shape!=(3,3) or mu.shape!=(3,) or not np.isfinite(cap) or cap<0 or not np.all(np.isfinite(mu)) or np.any(mu<=0) or sign not in (-1,1):raise ValueError('invalid contact')
        if not np.all(np.isfinite(frame)) or not np.allclose(frame@frame.T,np.eye(3),atol=1e-9,rtol=0):raise ValueError('invalid frame')
        f=np.column_stack([sign*frame[0],sign*mu[0]*frame[1],sign*mu[1]*frame[2],np.zeros(3)])
        point=np.asarray(c['point'],dtype=float)
        if point.shape!=(3,) or not np.all(np.isfinite(point)):raise ValueError('invalid point')
        r=point-com
        t=np.column_stack([np.cross(r,f[:,i]) for i in range(4)])
        t[:,3]+=sign*mu[2]*frame[0]
        a=nu@np.vstack([f,t])
        # For ||u||<=Fn, nu.w <= Fn*(a0+||a1:||), with 0<=Fn<=cap.
        support=cap*max(0.,float(a[0]+np.linalg.norm(a[1:])))
        supports.append(support)
    upper=float(sum(supports)/alpha)
    if not np.isfinite(upper) or abs(upper-case['claimed_lambda_upper'])>atol:raise ValueError('claimed support bound disagrees')
    return dict(alpha=alpha,contact_support= supports,lambda_upper=upper,infeasible_unit_gravity=upper<1-atol)

class GraspWrenchEvidenceTest(unittest.TestCase):
    def test_saved_contacts_and_corrupt_evidence(self):
        fixture=json.loads((Path(__file__).parent/'fixtures/grasp_wrench.json').read_text())
        for name,case in fixture['cases'].items():
            with self.subTest(case=name):
                self.assertTrue(verify_certificate(case)['infeasible_unit_gravity'])
                bad=copy.deepcopy(case)
                bad['claimed_lambda_upper']+=.01
                with self.assertRaises(ValueError):verify_certificate(bad)
                bad=copy.deepcopy(case)
                bad['equilibrium_covector_world']=[0]*6
                with self.assertRaises(ValueError):verify_certificate(bad)
                bad=copy.deepcopy(case)
                bad['contacts'][0]['Fn_cap']=float('nan')
                with self.assertRaises(ValueError):verify_certificate(bad)

    def test_centered_contacts_are_not_rejected_by_upper_bound(self):
        case=dict(mass_kg=.02,gravity=[0,0,-9.81],com_world_m=[0,0,0],
            equilibrium_covector_world=[0,0,1/.1962,0,0,0],
            claimed_lambda_upper=.32/.1962,contacts=[
                dict(point=[-.02,0,0],frame=np.eye(3).tolist(),sign=1,mu3=[1.6,1.6,.02],Fn_cap=.1),
                dict(point=[.02,0,0],frame=np.diag([-1,1,-1]).tolist(),sign=1,mu3=[1.6,1.6,.02],Fn_cap=.1)])
        result=verify_certificate(case)
        self.assertFalse(result['infeasible_unit_gravity'])
        # An upper bound above weight does not certify feasibility or dynamic success.
        self.assertAlmostEqual(result['lambda_upper'],.32/.1962)

if __name__=='__main__':unittest.main()
