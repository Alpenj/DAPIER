"""Right RECEIVE and performance contracts; synthetic fixtures are not REAL measurements."""
import copy
import json
import unittest

import numpy as np

from .features import a_image_functions,observation_binding
from .image_jacobian import jacobian_record
from .performance import benchmark,candidate_correction,roi_object_feature,scalar_response_comparison
from .right_relative import RIGHT_FEATURE_DEFINITIONS,RightRelativeServoProposal,extract_right_features
from .servo_proposal import evaluate_response
from .test_visual_servo import FIXTURE_SHA,fixture


def right_fixture(kind='TEST_FIXTURE'):
    obs,_,_=fixture();obs['source_kind']=kind
    for view in obs['views'].values():view['source_kind']=kind
    obs['views']['right_wrist']['image_bgr']=obs['views']['left_wrist']['image_bgr'].copy()
    binding=observation_binding(obs,camera_key='right_wrist')
    evidence=dict(**binding,source_kind=kind,source_sha256=FIXTURE_SHA)
    held=dict(evidence,held_by_left=True,target_association_verified=True)
    pad=dict(evidence,camera_key='right_wrist',role='CURRENT_OBSERVED_PAD',
        physical_correspondence_verified=True,values_px=dict(u=155,v=None),observed_axes=['u'],
        feature_definitions=RIGHT_FEATURE_DEFINITIONS,context=dict(id='RIGHT_TEST_FIXTURE_ONLY'))
    j=jacobian_record([[2]],feature_axes=['u'],motion_basis_model=[[1],[0],[0]],
        source='TEST_FIXTURE',context=pad['context'],feature_definitions=RIGHT_FEATURE_DEFINITIONS,
        source_sha256=dict(fixture=FIXTURE_SHA))
    return obs,held,pad,j


class RightTests(unittest.TestCase):
    def test_observed_relative_X_only(self):
        obs,held,pad,j=right_fixture();p=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
        self.assertEqual(p['observed_control_dimensions'],1)
        self.assertAlmostEqual(p['relative_feature_error_px']['u'],-.5)
        self.assertIsNone(p['relative_feature_error_px']['v'])
        self.assertAlmostEqual(p['estimated_tool_correction']['coordinates_mm'][0],.25)
        self.assertEqual(p['feature_evidence']['camera_key'],'right_wrist')
        self.assertFalse(p['executable']);self.assertFalse(p['motor_write_authorized'])

    def test_missing_or_model_pad_cannot_supply_REAL_reference(self):
        for replacement in ({},dict(role='MODEL_PRIOR',physical_correspondence_verified=False)):
            obs,held,pad,j=right_fixture()
            pad=replacement if not replacement else dict(pad,**replacement)
            p=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
            self.assertEqual(p['observed_control_dimensions'],0);self.assertIsNone(p['estimated_tool_correction'])
            self.assertIn('RIGHT_PAD_REFERENCE_UNOBSERVED_OR_UNIDENTIFIED',p['missing_evidence'])

    def test_unbound_held_target_cannot_supply_relative_error(self):
        for field in ('feedback_sha256','camera_state_binding','source_kind'):
            obs,held,pad,j=right_fixture();held[field]='different'
            p=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
            self.assertIsNone(p['estimated_tool_correction'])
            self.assertIn('CURRENT_LEFT_HELD_TARGET_ASSOCIATION_UNVERIFIED',p['missing_evidence'])

    def test_PICK_reference_is_not_RECEIVE_reference(self):
        obs,held,pad,j=right_fixture();pad['camera_key']='left_wrist'
        self.assertEqual(RightRelativeServoProposal(obs['q12'],obs,held,pad,j)['observed_control_dimensions'],0)

    def test_REAL_schema_cannot_use_model_or_unvalidated_J(self):
        # Negative schema fixtures exercise an untrusted REAL label; no image is admitted as REAL evidence.
        for source,validated in (('MODEL_ONLY_JACOBIAN',False),('REAL_MEASURED',False)):
            obs,held,pad,j=right_fixture(kind='REAL');j.update(source=source,validated=validated)
            p=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
            self.assertIsNone(p['estimated_tool_correction'])
            self.assertIn('VALIDATED_REAL_RIGHT_JACOBIAN_UNAVAILABLE',p['missing_evidence'])

    def test_actual_q_must_match_camera_packet(self):
        obs,held,pad,j=right_fixture();q=obs['q12'].copy();q[6]=.1
        with self.assertRaisesRegex(ValueError,'exactly bind'):RightRelativeServoProposal(q,obs,held,pad,j)

    def test_stale_or_unknown_REAL_frame_cannot_correct(self):
        for camera in ({},{'fresh':False,'reader_valid':True},{'fresh':True,'reader_valid':False}):
            obs,held,pad,j=right_fixture(kind='REAL');obs['views']['right_wrist']['recorded_camera']=camera
            j.update(source='REAL_MEASURED',validated=True)
            p=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
            self.assertIsNone(p['estimated_tool_correction'])
            self.assertIn('CURRENT_RIGHT_IMAGE_FRESHNESS_UNVERIFIED',p['missing_evidence'])

    def test_no_object_or_clipped_object_is_explicit(self):
        obs,held,pad,_=right_fixture();image=obs['views']['right_wrist']['image_bgr'];image[:]=255
        f=extract_right_features(obs,held,pad)
        self.assertIn('RIGHT_OBJECT_FEATURE_UNOBSERVED',f['missing_evidence'])
        image[:,100:210]=[0,0,255];f=extract_right_features(obs,held,pad)
        self.assertTrue(f['visibility']['frame_clipped'])
        self.assertFalse(f['grasp_region']['physically_graspable_region_verified'])

    def test_model_projection_is_not_measured_pad_error(self):
        obs,held,pad,_=right_fixture()
        obs['model_assisted_relative_geometry']=dict(status='CANDIDATE',
            right_pad_model_prior=[dict(source='MODEL_PRIOR_ACTUAL_Q_CAD',currently_measured=False)],
            fit_diagnostics=dict(reprojection_rms_px=.6,residual_is_physical_bound=False))
        f=extract_right_features(obs,held,{})
        self.assertIsNone(f['model_projection_diagnostic']['actual_pad_pixel_residual_px'])
        self.assertEqual(f['observed_image_axes'],[])

    def test_nonfinite_pad_and_mismatched_feature_definition(self):
        obs,held,pad,j=right_fixture();pad['values_px']['u']=float('nan')
        with self.assertRaises(ValueError):RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
        pad['values_px']['u']=155;pad['feature_definitions']={'u':'LEFT_PICK'}
        self.assertIsNone(RightRelativeServoProposal(obs['q12'],obs,held,pad,j)['estimated_tool_correction'])


class ResponseTests(unittest.TestCase):
    def response(self,before,after,expected):
        old=dict(image_identity='before',source_kind='TEST_FIXTURE',camera_key='right_wrist',
            feature_definitions=RIGHT_FEATURE_DEFINITIONS,values_px=dict(zip(('u','v'),before)),
            target=dict(values_px=dict(u=0,v=0)))
        new=dict(old,image_identity='after',values_px=dict(zip(('u','v'),after)))
        proposal=dict(estimated_tool_correction={},jacobian=dict(feature_axes=['u','v'],matrix_px_per_mm=np.diag(expected).tolist()))
        return evaluate_response(old,new,proposal,[1,1])

    def test_global_dot_cannot_hide_reversed_axis(self):
        r=self.response([2,20],[3,10],[-1,-10])
        self.assertGreater(r['direction_dot_px_squared'],0)
        self.assertEqual(r['status'],'REVERSED_IMAGE_RESPONSE');self.assertTrue(r['jacobian_revalidation_required'])

    def test_consistent_axis_direction_with_larger_total_error_requires_review(self):
        r=self.response([10,0],[-10,20],[-1,1])
        self.assertEqual(r['status'],'IMAGE_ERROR_INCREASED');self.assertTrue(r['image_error_increased'])
        self.assertTrue(r['jacobian_revalidation_required'])

    def test_both_axis_directions_and_total_error_decrease(self):
        r=self.response([10,20],[5,10],[-1,-1])
        self.assertEqual(r['status'],'DIRECTION_CONSISTENT');self.assertFalse(r['jacobian_revalidation_required'])
        self.assertLess(r['image_error_norm_px'][1],r['image_error_norm_px'][0])

    def test_unpredicted_axis_change_is_unresolved(self):
        r=self.response([10,20],[5,19],[-1,0])
        self.assertEqual(r['status'],'DIRECTION_UNRESOLVED');self.assertTrue(r['jacobian_revalidation_required'])

    def test_lost_response_axis_cannot_be_silently_removed(self):
        r=self.response([10,20],[5,None],[-1,-1])
        self.assertEqual(r['status'],'RESPONSE_FEATURE_UNOBSERVED')
        self.assertEqual(r['missing_response_axes'],['v']);self.assertTrue(r['jacobian_revalidation_required'])

    def test_rank_one_J_does_not_hide_defined_Y_task_error(self):
        obs,held,pad,j=right_fixture();pad.update(observed_axes=['u','v'],values_px=dict(u=144.5,v=89.5))
        before=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
        old=before['feature_evidence'];new=copy.deepcopy(old);new.update(image_identity='new',values_px=dict(u=5,v=70))
        r=evaluate_response(old,new,before,[-2.5])
        self.assertEqual(r['image_error_norm_axes'],['u','v'])
        np.testing.assert_allclose(r['image_error_norm_px'],[np.hypot(10,20),np.hypot(5,70)])
        self.assertEqual(r['status'],'IMAGE_ERROR_INCREASED');self.assertTrue(r['jacobian_revalidation_required'])

    def test_unused_Y_Jacobian_row_is_not_new_required_evidence(self):
        obs,held,pad,j=right_fixture()
        j.update(matrix_px_per_mm=[[2],[3]],feature_axes=['u','v'])
        p=RightRelativeServoProposal(obs['q12'],obs,held,pad,j)
        self.assertEqual(list(p['expected_image_error_change']),['u'])
        old=p['feature_evidence'];new=copy.deepcopy(old);new.update(image_identity='new',values_px=dict(u=0,v=None))
        r=evaluate_response(old,new,p,[.25])
        self.assertEqual(r['status'],'DIRECTION_CONSISTENT')
        self.assertEqual(r['feature_axes'],['u']);self.assertEqual(r['image_error_norm_axes'],['u'])
        self.assertFalse(r['jacobian_revalidation_required'])


class PerformanceTests(unittest.TestCase):
    def test_ROI_coordinates_preserve_existing_A_feature(self):
        obs,_,_=fixture();a,_=a_image_functions();image=obs['views']['left_wrist']['image_bgr']
        full=a.wrist_object(image);roi,meta=roi_object_feature(image,a)
        self.assertEqual(full['centre_px'],roi['centre_px']);self.assertEqual(full['edges_px'],roi['edges_px'])
        self.assertEqual(full['row'],roi['row']);self.assertEqual(full['touches_border'],roi['touches_border'])
        self.assertTrue(meta['global_component_scan_retained'])

    def test_ROI_no_object_remains_unobserved(self):
        a,_=a_image_functions();result,_=roi_object_feature(np.full((240,320,3),255,np.uint8),a)
        self.assertFalse(result['visible'])

    def test_same_opening_confirmation_is_separate_from_visible_Jaw_matching(self):
        obs,reference,_=fixture();a,_=a_image_functions();image=obs['views']['left_wrist']['image_bgr']
        obj=a.wrist_object(image);offset=a.wrist_offset(obj,reference,{},reference['gripper_tick'])
        self.assertTrue(offset['reference_confirmed_in_this_frame'])
        self.assertFalse({'left','right'}&set(offset['confirmed_by']))

    def test_gain_candidates_are_explicit_numerical_calculations(self):
        self.assertEqual(candidate_correction(10,2,4,10,'A'),-5)
        self.assertEqual(candidate_correction(10,2,4,10,'B'),-2.5)
        self.assertAlmostEqual(candidate_correction(10,2,4,10,'C'),-2.5)
        self.assertEqual(candidate_correction(10,2,4,10,'D'),-2.5)
        self.assertEqual(candidate_correction(10,2,4,10,'E'),-5)

    def test_counterfactual_is_not_a_REAL_success_or_certified_path(self):
        pair=dict(before_image_error_px=-10,normalized_error_response_px_per_mm=4,
            actual_model_FK_projected_tool_move_mm=4,commanded_tool_move_mm=5,scale_mm_per_px=[.25,.24])
        d=scalar_response_comparison([dict(pair,id='TEST_A'),dict(pair,id='TEST_B')])
        self.assertEqual(len(d['comparisons']),10)
        self.assertTrue(all(x['REAL_success'] is False and x['physical_time_saved_s'] is None for x in d['comparisons']))
        self.assertTrue(all(x['IK_or_collision_certified'] is False for x in d['comparisons']))
        json.dumps(d,allow_nan=False)

    def test_CPU_benchmark_reports_actual_samples(self):
        calls=[];d=benchmark(lambda:calls.append(1),repeats=3)
        self.assertEqual(len(calls),4);self.assertEqual(len(d['samples_ms']),3)
        self.assertGreaterEqual(d['median_ms'],0);self.assertEqual(d['unit'],'ms')


if __name__=='__main__':unittest.main(verbosity=2)
