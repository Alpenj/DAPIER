from pathlib import Path
import sys,json,unittest,ast
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from preclose_geometry_audit import face_gap,onset,FACE_GROUP_TOL_M

class PadFaceAuditTest(unittest.TestCase):
    def test_mirrored_sign_and_overlap_without_any_inside_vertex(self):
        # Large pad surrounds the block projection: no pad vertex lies inside it.
        for side in (-1,1):
            p=np.array([[side*.03,y,z] for y,z in [(-.03,-.03),(.03,-.03),(.03,.03),(-.03,.03)]])
            r=face_gap(p,[-side,0,0],[.02]*3,side)
            self.assertTrue(r['overlap_exists'])
            np.testing.assert_allclose(r['overlap_extent_yz_m'],[.04,.04])
            self.assertAlmostEqual(r['effective_min_gap_m'],.01)
            self.assertAlmostEqual(r['normal_angle_deg'],0)
    def test_overlap_restricts_wedge_minimum(self):
        p=np.array([[.03+.5*y,y,z] for y,z in [(-.06,-.01),(.06,-.01),(.06,.01),(-.06,.01)]])
        r=face_gap(p,[-1,0,0],[.02]*3,1)
        self.assertLess(r['min_gap_m'],0)
        self.assertAlmostEqual(r['effective_min_gap_m'],0)
        self.assertGreater(r['mean_vertex_gap_m'],0)
    def test_no_overlap_and_touch_penetration_are_not_separation(self):
        p=np.array([[.01,y,z] for y,z in [(.03,-.01),(.04,-.01),(.04,.01),(.03,.01)]])
        r=face_gap(p,[-1,0,0],[.02]*3,1)
        self.assertFalse(r['overlap_exists']);self.assertIsNone(r['effective_min_gap_m'])
        p[:,1]-=.035
        for x in (.02,.01999):
            p[:,0]=x;r=face_gap(p,[-1,0,0],[.02]*3,1)
            self.assertLessEqual(r['effective_min_gap_m'],0)
        self.assertIsNone(onset([0,1,2],[None,None,None]))
        self.assertEqual(onset([0,.002,.004],[1e-5,0,-1e-5])['bracket_s'],[0,.002])
    def test_saved_geometry_predicts_order_but_not_finite_second_onset(self):
        r=json.loads((Path(__file__).parent/'fixtures/preclose_geometry.json').read_text())
        self.assertEqual(r['new_physics_steps'],0)
        self.assertTrue(r['measured_passive_jaw_trace']);self.assertEqual(r['decision'],'PARTIAL')
        for state in r['states'].values():
            for p,face in zip(state['pads'],state['compiled_faces']):
                self.assertLess(face['grouping_residual_m'],FACE_GROUP_TOL_M)
                value=face_gap(p['vertices_block_m'],p['normal_block'],state['block_half_size_m'],face['side'])
                for key in ('min_gap_m','mean_vertex_gap_m','wedge_m','effective_min_gap_m','normal_angle_deg'):
                    self.assertAlmostEqual(value[key],p[key],places=12)
        self.assertLess(abs(r['states']['A0']['min_gap_delta_pad1_minus_pad2_m']),FACE_GROUP_TOL_M)
        self.assertGreater(r['states']['A2']['min_gap_delta_pad1_minus_pad2_m'],.0003)
        pred=r['predicted']['fixed_preclose'];self.assertIsNone(pred['onset'][0]);self.assertIsNone(pred['delta_pad1_minus_pad2_s'])
        self.assertAlmostEqual(pred['onset'][1]['time_s'],18.794,places=8)
        for p,t in zip(r['predicted']['actual']['onset'],r['observed']['contact_count']['times_s']):
            self.assertAlmostEqual(p['time_s'],t,places=10)
        event=next(x for x in r['block_motion_events'] if x['step']==9651)
        self.assertLess(event['block_motion_gap_contribution_m'][0],0)
        self.assertGreater(event['block_motion_gap_contribution_m'][1],0)
        self.assertLessEqual(event['actual_effective_gap_m'][0],0)
        self.assertGreater(event['block_frozen_effective_gap_m'][0],0)
        tree=ast.parse((Path(__file__).resolve().parents[1]/'preclose_geometry_audit.py').read_text())
        self.assertFalse(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in ('mj_step','mj_forward') for n in ast.walk(tree)))

if __name__=='__main__':unittest.main()
