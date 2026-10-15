"""Feature validity and the file-record adapter; positive images are explicitly synthetic."""
import copy
import unittest
from unittest.mock import patch

import numpy as np

from .a_shadow import proposal_from_A_record
from .features import observation_binding
from .right_relative import RightRelativeServoProposal
from .test_b19 import right_fixture
from .test_visual_servo import FIXTURE_SHA


def native_fixture():
    obs,_,_,_=right_fixture('REAL')
    image=obs['views']['right_wrist']['image_bgr']
    ticks={s:dict(gripper=100) for s in ('left','right')}
    document=dict(source_kind='REAL',stage='NEGATIVE_SCHEMA_FIXTURE_NOT_REAL_MEASUREMENT',q12=obs['q12'],
        observation=dict(q12=obs['q12'],present_ticks=ticks,object_evidence=dict(evidence_valid=False)),
        actual_feedback=dict(ticks=ticks,usable=True,t_bus_monotonic=1),
        files={k:dict(rgb=k+'.png') for k in obs['views']},
        camera_frames={k:dict(n=1,fresh=True,reader_valid=True,recv_monotonic_s=1) for k in obs['views']})
    return document,{k:image.copy() for k in obs['views']},dict(record_identity=obs['observation_identity'],
        record_sha256=FIXTURE_SHA,image_sha256={k:FIXTURE_SHA for k in obs['views']})


class FeatureValidityTests(unittest.TestCase):
    def test_unclipped_X_positive_does_not_require_Y_or_face_graph(self):
        obs,held,pad,j=right_fixture()
        p=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
        self.assertEqual(p['observed_control_dimensions'],1)
        self.assertIsNone(p['relative_feature_error_px']['v'])
        self.assertIsNone(p['feature_evidence']['visible_approach_region']['graph'])
        self.assertFalse(p['feature_evidence']['visible_approach_region']['graph_requested'])

    def test_each_clipped_boundary_invalidates_centroid_with_other_evidence_present(self):
        for edge in ('left','right','top','bottom'):
            with self.subTest(edge=edge):
                obs,held,pad,j=right_fixture();image=obs['views']['right_wrist']['image_bgr'];image[:]=255
                regions=dict(left=(slice(70,150),slice(0,110)),right=(slice(70,150),slice(210,320)),
                    top=(slice(0,80),slice(100,210)),bottom=(slice(160,240),slice(100,210)))
                image[regions[edge]]=[0,0,255]
                for evidence in (held,pad):evidence.update(observation_binding(obs,camera_key='right_wrist'))
                p=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
                self.assertEqual(p['observed_control_dimensions'],0);self.assertIsNone(p['estimated_tool_correction'])
                self.assertIn('RIGHT_COMPONENT_CENTROID_FRAME_CLIPPED',p['missing_evidence'])
                self.assertIsNotNone(p['feature_evidence']['object_component_centroid_uv']['u'])

    def test_diagnostic_graph_cannot_change_control_features(self):
        obs,held,pad,j=right_fixture();fast=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
        graph=(np.array([[1,2]]),[],None,None,dict(scope='TEST_FIXTURE'))
        from .right_relative import _right_functions
        from .features import ROOT,sha256
        source=(ROOT/'native/revision').resolve()
        digests=tuple(sha256(source/name) for name in ('sweepick_pick01.py','right_wrist_relative_geometry.py',
            'sweepick_261009_offsite_relative_features.py'))
        _,geometry,_=_right_functions(source,digests)
        with patch.object(geometry,'observed_face_graph',return_value=graph) as called:
            again=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
            called.assert_not_called()
            diagnostic=RightRelativeServoProposal(obs['q12'],obs,held,pad,j,include_geometry=True)
            called.assert_called_once()
        for key in ('relative_feature_error_px','observed_control_dimensions','estimated_tool_correction'):
            self.assertEqual(fast[key],again[key]);self.assertEqual(fast[key],diagnostic[key])


class AShadowTests(unittest.TestCase):
    def test_missing_REAL_reference_preserves_zero_and_inputs(self):
        document,images,kwargs=native_fixture();before=copy.deepcopy(document)
        result=proposal_from_A_record(document,images,**kwargs)
        self.assertEqual(document,before);self.assertEqual(result['observed_control_dimensions'],0)
        self.assertIsNone(result['estimated_tool_correction']);self.assertFalse(result['executable'])
        self.assertFalse(result['motor_write_authorized']);self.assertFalse(result['A_record_provenance']['command_fields_consumed'])

    def test_command_or_inconsistent_state_cannot_replace_actual_feedback(self):
        for change in ('q','ticks','usable'):
            document,images,kwargs=native_fixture()
            if change=='q':document['observation']['q12']=[1]*12
            elif change=='ticks':document['observation']['present_ticks']={}
            else:document['actual_feedback']['usable']=False
            with self.assertRaises(ValueError):proposal_from_A_record(document,images,**kwargs)

    def test_missing_or_malformed_view_is_not_filled(self):
        for change in ('missing','float'):
            document,images,kwargs=native_fixture()
            if change=='missing':del images['right_wrist']
            else:images['right_wrist']=images['right_wrist'].astype(float)
            with self.assertRaises(ValueError):proposal_from_A_record(document,images,**kwargs)


if __name__=='__main__':unittest.main(verbosity=2)
