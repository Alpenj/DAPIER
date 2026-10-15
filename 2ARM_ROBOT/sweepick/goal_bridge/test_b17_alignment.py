"""Clock and model-preview regressions; these fixtures are not REAL measurements."""
import copy
import json
import unittest

from b16_goal_bridge import GoalProposalBridge, b15_times, stamp_difference, temporal_alignment
from test_b16_goal_bridge import contract_fixture


class TemporalTests(unittest.TestCase):
    def test_units_and_large_ns_are_subtracted_before_float_conversion(self):
        self.assertEqual(stamp_difference(
            dict(value=1000000000000000001, unit='ns', clock='qualified'),
            dict(value=1000000000000000000, unit='ns', clock='qualified'))['seconds'], 1e-9)
        self.assertEqual(stamp_difference(dict(value=2, unit='s', clock='qualified'),
            dict(value=1500, unit='ms', clock='qualified'))['seconds'], .5)

    def test_unknown_and_different_clocks_are_not_subtracted(self):
        for clock in ('UNKNOWN', 'UNKNOWN_DEVICE_TIMESTAMP', 'different'):
            self.assertIsNone(stamp_difference(dict(value=1, unit='s', clock=clock),
                dict(value=0, unit='s', clock='qualified'))['seconds'])

    def test_camera_specific_references(self):
        p = dict(source_clock=dict(host_id='fixture', run_id='fixture'),
            actual_feedback=dict(per_arm={s:dict(feedback_received_ns=n)
                                         for s,n in [('left',10),('right',20)]}),
            camera_provenance={v:dict(received_ns=15) for v in ('left_wrist','right_wrist','top')})
        times = b15_times(p)
        self.assertEqual([set(s['state_references']) for s in times['sensors']],
                         [{'left'}, {'right'}, {'left','right'}])
        report = temporal_alignment(times)
        self.assertIsNone(report['right_minus_left_measurement_s']['seconds'])
        self.assertEqual(report['cameras'][0]['receipt_minus_state_s']['left']['seconds'],5e-9)
        self.assertEqual(report['cameras'][1]['receipt_minus_state_s']['right']['seconds'],-5e-9)

    def test_known_measurement_and_exposure_are_separate_from_receipt(self):
        stamp = lambda n:dict(value=n,unit='s',clock='fixture:monotonic')
        times = dict(joint_state={s:dict(stamp(10),measurement_time=stamp(n))
                                 for s,n in [('left',1),('right',2)]},
            sensors=[dict(stamp(12),view='top',state_references={s:stamp(10) for s in ('left','right')},
                          exposure_time=stamp(1.5),native_exposure_known=True)])
        report = temporal_alignment(times)
        self.assertEqual(report['right_minus_left_receipt_s']['seconds'],0)
        self.assertEqual(report['right_minus_left_measurement_s']['seconds'],1)
        self.assertEqual(report['cameras'][0]['exposure_minus_joint_measurement_s']['left']['seconds'],.5)
        self.assertEqual(report['cameras'][0]['exposure_minus_joint_measurement_s']['right']['seconds'],-.5)

    def test_false_calibration_and_missing_exposure_allow_pure_preview(self):
        record = contract_fixture()
        record['geometry_evidence']['calibration_verified'] = False
        record['reference_times']['sensors'][0]['native_exposure_known'] = False
        result = GoalProposalBridge().propose(record,lambda _:dict(ik_solution_exists=True,collision_result='CLEAR'))
        self.assertEqual(result['status'],'MODEL_PREVIEW_ONLY')
        self.assertFalse(result['physical_execution_requirements']['physical_approval'])
        self.assertFalse(result['executable'])

    def test_missing_geometry_model_context_keeps_needs_observation(self):
        record = contract_fixture()
        record['held_pose']['object_pose_model'] = None
        record['geometry_evidence']['centre_identified'] = False
        record['model_preview_context'] = dict(available=True,role='MODEL_PRIOR')
        result = GoalProposalBridge().propose(record,lambda _:dict(ik_solution_exists=True,collision_result='CLEAR'))
        self.assertEqual(result['status'],'NEEDS_OBSERVATION')
        self.assertEqual(result['targets'],[])
        self.assertEqual(result['goal_expression'],'MISSING')
        self.assertEqual(result['target_reference_frame'],'MISSING')
        self.assertEqual(result['model_view_candidate']['role'],'MODEL_VIEW_CANDIDATE')
        self.assertFalse(result['model_view_candidate']['measured_goal'])
        self.assertTrue(result['planning_preview']['ik_solution_exists'])

    def test_model_context_cannot_bypass_missing_frame_or_invalid_state(self):
        for field,value in [('coordinate_frame',None),('canonical_state',[float('nan')]*12)]:
            r=contract_fixture();r[field]=value;r['model_preview_context']=dict(available=True)
            result=GoalProposalBridge().propose(r,lambda _:self.fail('invalid basic context previewed'))
            self.assertIn(result['status'],('NEEDS_OBSERVATION','EXCLUDED'))

    def test_malformed_time_is_excluded(self):
        for value in (float('nan'),True,'bad'):
            record=contract_fixture();record['reference_times']['sensors'][0]['value']=value
            result=GoalProposalBridge().propose(record)
            self.assertEqual(result['status'],'EXCLUDED')
            self.assertIn('REFERENCE_TIME_MALFORMED',result['invalid_reasons'])
            json.dumps(result,allow_nan=False)


if __name__ == '__main__':
    unittest.main(verbosity=2)
