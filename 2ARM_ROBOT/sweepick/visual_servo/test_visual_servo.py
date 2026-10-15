"""Synthetic contract fixtures and optional private A-model regressions; no REAL success labels."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from visual_servo.features import FEATURE_DEFINITIONS,extract_features
from visual_servo.image_jacobian import estimate_jacobian,jacobian_record,solve_correction
from visual_servo.servo_proposal import propose_servo,evaluate_response

MODEL_FIXTURE=None
MODEL_EVIDENCE=[]
FIXTURE_SHA=hashlib.sha256(b'TEST_FIXTURE_NOT_REAL').hexdigest()


def fixture():
    image=np.full((240,320,3),255,np.uint8)
    image[70:150,100:210]=[0,0,255]
    image[230:,:65]=0;image[230:,245:]=0
    identity=dict(run_id='TEST_FIXTURE_ONLY',observation_id='one')
    stamp=dict(value=1,unit='s',clock='TEST_FIXTURE_MONOTONIC')
    observation=dict(observation_identity=identity,source_kind='TEST_FIXTURE',q12=[0.0]*12,
        reference_times=dict(joint_state={s:copy.deepcopy(stamp) for s in ('left','right')},sensors=[]),
        views={v:dict(observation_identity=copy.deepcopy(identity),source_kind='TEST_FIXTURE',
                      image_identity=v+'_1',image_bgr=image if v=='left_wrist' else None,
                      source_sha256=FIXTURE_SHA,gripper_tick=100) for v in ('top','left_wrist','right_wrist')})
    reference=dict(readable=True,inner_px=[64,246],midline_px=155,gap_px=182,
        gripper_tick=100,camera_key='left_wrist',source_kind='TEST_FIXTURE',source_sha256=FIXTURE_SHA)
    target=dict(values_px=dict(u=155,v=None),observed_axes=['u'],camera_key='left_wrist',
        source_kind='TEST_FIXTURE',source_sha256=FIXTURE_SHA,context=dict(id='TEST_LOCAL_CONTEXT'),
        feature_definitions=copy.deepcopy(FEATURE_DEFINITIONS))
    return observation,reference,target


def matrix_record(matrix,axes=('u','v'),basis=None):
    return jacobian_record(matrix,feature_axes=axes,
        motion_basis_model=np.eye(3)[:,:len(matrix[0])] if basis is None else basis,
        source='TEST_FIXTURE',context=dict(id='TEST_LOCAL_CONTEXT'),
        feature_definitions=FEATURE_DEFINITIONS,source_sha256=dict(fixture=FIXTURE_SHA))


class FeatureTests(unittest.TestCase):
    def test_observed_component_centre_contour_and_existing_X(self):
        obs,ref,target=fixture();f=extract_features(obs,jaw_reference=ref,target=target)
        self.assertTrue(f['object']['visible']);self.assertTrue(f['object']['contour_uv'])
        np.testing.assert_allclose(f['object']['observed_component_centroid_uv'],[154.5,109.5])
        self.assertEqual(f['observed_image_axes'],['u']);self.assertIn('Y_REFERENCE_UNOBSERVED',f['missing_evidence'])
        self.assertFalse(f['jaw']['bottom_row_is_task_y_reference'])

    def test_no_default_image_centre_or_Y_target(self):
        obs,ref,_=fixture();f=extract_features(obs,jaw_reference=ref)
        self.assertEqual(f['observed_image_axes'],[])
        self.assertEqual(f['feature_error_px'],dict(u=None,v=None))

    def test_object_detection_failure(self):
        obs,ref,target=fixture();obs['views']['left_wrist']['image_bgr'][:]=255
        f=extract_features(obs,jaw_reference=ref,target=target)
        self.assertIn('OBJECT_FEATURE_UNOBSERVED',f['missing_evidence'])
        self.assertEqual(f['observed_image_axes'],[])

    def test_gripper_reference_detection_failure(self):
        obs,ref,target=fixture();image=obs['views']['left_wrist']['image_bgr'];image[230:]=255
        obs['views']['left_wrist']['gripper_tick']=None
        f=extract_features(obs,jaw_reference=ref,target=target)
        self.assertIn('X_REFERENCE_UNOBSERVED',f['missing_evidence'])

    def test_frame_clipping_is_reported(self):
        obs,ref,target=fixture();image=obs['views']['left_wrist']['image_bgr'];image[100:,100:210]=[0,0,255]
        self.assertTrue(extract_features(obs,jaw_reference=ref,target=target)['object']['frame_clipped'])

    def test_detection_stability_uses_actual_burst(self):
        obs,ref,target=fixture();image=obs['views']['left_wrist']['image_bgr']
        self.assertEqual(extract_features(obs,jaw_reference=ref,target=target)['stability']['status'],'UNMEASURED')
        obs['views']['left_wrist']['frame_burst_bgr']=[image.copy() for _ in range(3)]
        self.assertEqual(extract_features(obs,jaw_reference=ref,target=target)['stability']['status'],'STABLE')

    def test_camera_state_mismatch(self):
        obs,ref,target=fixture();obs['views']['left_wrist']['observation_identity']['observation_id']='wrong'
        with self.assertRaisesRegex(ValueError,'identity mismatch'):extract_features(obs,jaw_reference=ref,target=target)

    def test_normal_async_is_not_order_failure(self):
        obs,ref,target=fixture();obs['reference_times']['joint_state']['right']['value']=2
        f=extract_features(obs,jaw_reference=ref,target=target)
        self.assertIn('ASYNCHRONOUS_BIMANUAL_FEEDBACK',f['temporal_alignment']['diagnostics'])
        self.assertEqual(f['observed_image_axes'],['u'])

    def test_foreign_provenance_cannot_supply_Y_reference(self):
        obs,ref,target=fixture();target.update(source_kind='MODEL',observed_axes=['u','v'],values_px=dict(u=155,v=100))
        f=extract_features(obs,jaw_reference=ref,target=target)
        self.assertIn('Y_REFERENCE_UNOBSERVED',f['missing_evidence'])
        self.assertEqual(f['observed_image_axes'],[])


class JacobianTests(unittest.TestCase):
    def test_rank_two_estimate_and_residual(self):
        result=estimate_jacobian([[1,0],[0,1],[1,1]],[[2,0],[0,3],[2,3]],
            feature_axes=['u','v'],motion_basis_model=np.eye(3)[:,:2],context={},
            feature_definitions=FEATURE_DEFINITIONS,source_sha256={},source='TEST_FIXTURE',comparable_context=True)
        self.assertEqual(result['independent_motion_directions'],2)
        np.testing.assert_allclose(result['jacobian']['matrix_px_per_mm'],[[2,0],[0,3]],atol=1e-14)
        self.assertFalse(result['jacobian']['validated']);self.assertIsNone(result['measurement_noise_px'])

    def test_rank_one_never_fills_unobserved_column(self):
        result=estimate_jacobian([[1,0],[2,0]],[[2,3],[4,6]],feature_axes=['u','v'],
            motion_basis_model=np.eye(3)[:,:2],context={},feature_definitions=FEATURE_DEFINITIONS,
            source_sha256={},source='TEST_FIXTURE',comparable_context=True)
        self.assertEqual(result['independent_motion_directions'],1)
        self.assertEqual(np.shape(result['jacobian']['matrix_px_per_mm']),(2,1))

    def test_mixed_height_context_is_insufficient(self):
        result=estimate_jacobian([[1],[2]],[[2],[4]],feature_axes=['u'],motion_basis_model=[[1],[0],[0]],
            context={},feature_definitions=FEATURE_DEFINITIONS,source_sha256={},comparable_context=False)
        self.assertEqual(result['status'],'INSUFFICIENT_PAIRED_OBSERVATIONS');self.assertIsNone(result['jacobian'])
        self.assertEqual(result['candidate_pair_count'],2);self.assertEqual(result['valid_pair_count'],0)

    def test_singular_zero_and_rank_one(self):
        self.assertEqual(solve_correction([[0,0],[0,0]],[1,2])['control_dimensions'],0)
        r=solve_correction([[2,0],[0,0]],[4,8]);self.assertEqual(r['control_dimensions'],1)
        self.assertEqual(len(r['coefficients_mm']),1)

    def test_numerical_instability_and_caller_damping(self):
        truncated=solve_correction([[1,0],[0,1e-20]],[1,1]);self.assertEqual(truncated['control_dimensions'],1)
        plain=solve_correction([[1,0],[0,1e-8]],[1,1])
        damped=solve_correction([[1,0],[0,1e-8]],[1,1],damping=.01)
        self.assertLess(np.linalg.norm(damped['coefficients_mm']),np.linalg.norm(plain['coefficients_mm']))
        self.assertFalse(damped['clipping_applied']);json.dumps(truncated,allow_nan=False)

    def test_malformed_numeric_input(self):
        for value in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):solve_correction([[value]],[1])
        with self.assertRaises(ValueError):solve_correction([[1]],[1],damping=-1)


class ProposalTests(unittest.TestCase):
    def test_positive_negative_X_and_Y_errors(self):
        for axis in ('u','v'):
            for sign in (-1,1):
                with self.subTest(axis=axis,sign=sign):
                    obs,ref,target=fixture();target['observed_axes']=['u','v'];target['values_px']['v']=109.5
                    target['values_px'][axis]-=sign*8
                    f=extract_features(obs,jaw_reference=ref,target=target)
                    p=propose_servo(obs,f,target,matrix_record([[2,0],[0,3]]))
                    self.assertEqual(p['observed_control_dimensions'],2)
                    self.assertLess(p['feature_error_px'][axis]*p['expected_image_error_change'][axis],0)
                    self.assertFalse(p['executable']);self.assertFalse(p['motor_write_authorized'])

    def test_Y_unobserved_keeps_one_control(self):
        obs,ref,target=fixture();f=extract_features(obs,jaw_reference=ref,target=target)
        p=propose_servo(obs,f,target,matrix_record([[2]],axes=['u']))
        self.assertEqual(p['observed_control_dimensions'],1)
        self.assertEqual(len(p['estimated_tool_correction']['coordinates_mm']),1)
        self.assertIsNone(p['feature_error_px']['v'])

    def test_missing_object_or_reference_never_corrects(self):
        for missing in ('object','jaw'):
            obs,ref,target=fixture()
            if missing=='object':obs['views']['left_wrist']['image_bgr'][:]=255
            else:ref={}
            f=extract_features(obs,jaw_reference=ref,target=target)
            self.assertIsNone(propose_servo(obs,f,target,matrix_record([[2]],axes=['u']))['estimated_tool_correction'])

    def test_no_new_image_or_state_mismatch(self):
        obs,ref,target=fixture();f=extract_features(obs,jaw_reference=ref,target=target);j=matrix_record([[2]],axes=['u'])
        self.assertIn('NO_NEW_IMAGE',propose_servo(obs,f,target,j,previous_features=f)['missing_evidence'])
        obs['q12'][0]=.1
        self.assertEqual(propose_servo(obs,f,target,j)['validity'],'INVALID_OBSERVATION')

    def test_opposite_actual_image_response_requires_reidentification(self):
        obs,ref,target=fixture();f=extract_features(obs,jaw_reference=ref,target=target);j=matrix_record([[2]],axes=['u'])
        p=propose_servo(obs,f,target,j);after=copy.deepcopy(f);after['image_identity']='new'
        after['values_px']['u']-=2
        response=evaluate_response(f,after,p,[1])
        self.assertEqual(response['status'],'REVERSED_IMAGE_RESPONSE')
        next_p=propose_servo(obs,f,target,j,previous_response=response)
        self.assertIsNone(next_p['estimated_tool_correction']);self.assertFalse(response['physical_stop_requested'])

    def test_unvalidated_REAL_jacobian_is_not_accepted(self):
        obs,ref,target=fixture();f=extract_features(obs,jaw_reference=ref,target=target);j=matrix_record([[2]],axes=['u'])
        j.update(source='REAL_MEASURED',model_only=False,validated=False)
        self.assertIn('REAL_JACOBIAN_VALIDATION_UNVERIFIED',propose_servo(obs,f,target,j)['missing_evidence'])

    def test_target_context_and_definition_mismatch(self):
        obs,ref,target=fixture();f=extract_features(obs,jaw_reference=ref,target=target);j=matrix_record([[2]],axes=['u'])
        j['context']={}
        self.assertIn('JACOBIAN_CONTEXT_MISMATCH',propose_servo(obs,f,target,j)['missing_evidence'])
        j['context']=target['context'];j['feature_definitions']={'u':'another_feature'}
        self.assertIn('FEATURE_DEFINITION_MISMATCH',propose_servo(obs,f,target,j)['missing_evidence'])

    def test_changed_frame_cannot_reuse_extracted_features(self):
        for field in ('image_identity','source_sha256','image_bgr','image_shape'):
            with self.subTest(field=field):
                obs,ref,target=fixture();f=extract_features(obs,jaw_reference=ref,target=target)
                view=obs['views']['left_wrist']
                if field=='image_bgr':view[field][0,0]=0
                elif field=='image_shape':view['image_bgr']=view['image_bgr'].reshape(320,240,3)
                else:view[field]='changed'
                p=propose_servo(obs,f,target,matrix_record([[2]],axes=['u']))
                self.assertEqual(p['validity'],'INVALID_OBSERVATION')
                self.assertIsNone(p['estimated_tool_correction'])

    def test_fixture_or_model_cannot_be_promoted_to_REAL(self):
        for kind in ('TEST_FIXTURE','MODEL'):
            obs,ref,target=fixture();obs['source_kind']=kind;ref['source_kind']=kind;target['source_kind']=kind
            for view in obs['views'].values():view['source_kind']=kind
            f=extract_features(obs,jaw_reference=ref,target=target);j=matrix_record([[2]],axes=['u'])
            j.update(source='REAL_MEASURED',validated=True)
            p=propose_servo(obs,f,target,j)
            self.assertIn('JACOBIAN_SOURCE_MISMATCH',p['missing_evidence'])
            self.assertIsNone(p['estimated_tool_correction']);self.assertTrue(p['model_only'])

    def test_external_Jacobian_basis_is_revalidated(self):
        for basis in ([[float('nan')],[0],[0]],[[2],[0],[0]],[[1,0],[0,1],[0,0]]):
            obs,ref,target=fixture();f=extract_features(obs,jaw_reference=ref,target=target)
            j=matrix_record([[2]],axes=['u']);j['motion_basis_model']=basis
            p=propose_servo(obs,f,target,j)
            self.assertEqual(p['validity'],'INVALID_JACOBIAN');self.assertIsNone(p['estimated_tool_correction'])
            json.dumps(p,allow_nan=False)

    def test_nonfinite_response_requires_revalidation(self):
        for value in (float('nan'),float('inf')):
            obs,ref,target=fixture();f=extract_features(obs,jaw_reference=ref,target=target)
            j=matrix_record([[2]],axes=['u']);p=propose_servo(obs,f,target,j)
            after=copy.deepcopy(f);after['image_identity']='new';after['values_px']['u']=value
            response=evaluate_response(f,after,p,[1])
            self.assertEqual(response['status'],'INVALID_IMAGE_RESPONSE')
            self.assertTrue(response['jacobian_revalidation_required']);json.dumps(response,allow_nan=False)
            self.assertIsNone(propose_servo(obs,f,target,j,previous_response=response)['estimated_tool_correction'])

    def test_nonfinite_feature_error_never_proposes(self):
        obs,ref,target=fixture();f=extract_features(obs,jaw_reference=ref,target=target)
        f['feature_error_px']['u']=float('nan')
        p=propose_servo(obs,f,target,matrix_record([[2]],axes=['u']))
        self.assertEqual(p['validity'],'INVALID_CALCULATION');self.assertIsNone(p['estimated_tool_correction'])
        json.dumps(p,allow_nan=False)


class ActualAModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if MODEL_FIXTURE is None:raise unittest.SkipTest('requires private frozen A model and explicit model fixture')

    def test_model_jacobian_rank_and_real_sign_are_separate(self):
        loop,basis,q=MODEL_FIXTURE
        j=loop.jacobian(q,basis,probe_mm=1.0)
        self.assertEqual(j['jacobian']['source'],'MODEL_ONLY_JACOBIAN')
        self.assertEqual(j['jacobian']['diagnostics']['rank'],2)
        self.assertFalse(j['physical_match_verified']);MODEL_EVIDENCE.append(j)

    def test_closed_loop_through_existing_A_IK_collision_and_reprojection(self):
        from visual_servo.offline_model import MODEL_FEATURES
        from visual_servo.features import observation_binding
        loop,basis,q_ref=MODEL_FIXTURE;target_uv,_=loop.observe(q_ref)
        for direction in range(2):
            for sign in (-1,1):
                q,check=loop.move(q_ref,basis[:,direction]*(sign*.003))
                self.assertIsNotNone(q,check);history=[]
                for step in range(8):
                    uv,_=loop.observe(q);error=uv-target_uv;history.append(float(np.linalg.norm(error)))
                    if history[-1]<=.2:break
                    fit=loop.jacobian(q,basis,probe_mm=1.0);j=fit['jacobian'];self.assertIsNotNone(j,fit)
                    obs,_,_=fixture();obs['source_kind']='MODEL';obs['q12']=q.tolist()
                    obs['observation_identity']['observation_id']=f'model_{direction}_{sign}_{step}'
                    for view in obs['views'].values():view['source_kind']='MODEL';view['observation_identity']=copy.deepcopy(obs['observation_identity'])
                    obs['views']['left_wrist']['image_identity']=f'model_frame_{step}'
                    target=dict(context=loop.context,source_kind='MODEL',values_px=dict(zip(('u','v'),target_uv.tolist())))
                    f=dict(**observation_binding(obs),source_kind='MODEL',image_identity=f'model_frame_{step}',
                        values_px=dict(zip(('u','v'),uv.tolist())),feature_error_px=dict(zip(('u','v'),error.tolist())),
                        observed_image_axes=['u','v'],feature_definitions=MODEL_FEATURES,target=target,source_sha256={},missing_evidence=[])
                    p=propose_servo(obs,f,target,j);self.assertEqual(p['validity'],'MODEL_ONLY_PROPOSAL')
                    q,check=loop.move(q,p['estimated_tool_correction']['delta_model_m']);self.assertIsNotNone(q,check)
                self.assertLess(history[-1],history[0])
                self.assertLessEqual(history[-1],.2)
                MODEL_EVIDENCE.append(dict(source='MODEL_ONLY',status='SIM_OR_MODEL_CONVERGED',direction=direction,sign=sign,
                    image_error_norm_px=history,test_target_px=.2,
                    criterion_role='virtual-model test acceptance only; no REAL task or safety threshold',
                    iteration_budget=8,real_convergence='REAL_CONVERGENCE_UNVERIFIED'))


if __name__=='__main__':unittest.main(verbosity=2)
