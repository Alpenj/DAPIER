"""Left/right evidence semantics; all constructed records are schema fixtures only."""
import copy
import unittest

from .a_shadow import proposal_from_A_record
from .test_b20 import native_fixture


def left_current_fixture():
    document,images,kwargs=native_fixture();native=document['observation']
    native.update(feedback_stamp=1,seq=1)
    camera=document['camera_frames']['top']
    native['held_object']=dict(source_kind='REAL',pose_role='CURRENT_OBSERVATION',transforms=dict(tool='left_cube_grasp'),
        feedback_stamp=1,seq=1,observation_diagnostic=dict(feedback_identity=dict(feedback_stamp=1,seq=1)),
        grasp_evidence=dict(held_after_lift=dict(label='HELD_CONFIRMED',ok=True,
            wrist_same_place_as_before_the_lift=True,jaw_during_lift=dict(loaded_throughout=True))),
        vision_evidence=dict(camera,observed=True))
    return document,images,kwargs


def held_result(document,images,kwargs,**extra):
    proposal=proposal_from_A_record(document,images,**kwargs,**extra)
    return proposal['reference_provenance']['object_target_association'],proposal


class LeftHoldBindingTests(unittest.TestCase):
    def test_right_tool_evaluation_never_confirms_left_hold(self):
        for at_tool in (True,False,None):
            document,images,kwargs=native_fixture()
            document['observation']['object_evidence']=dict(evidence_valid=True,object_at_tool=at_tool)
            held,proposal=held_result(document,images,kwargs)
            self.assertIsNone(held['held_by_left']);self.assertIsNone(held['target_association_verified'])
            self.assertEqual(held['recorded_right_tool_object_evidence']['object_at_tool'],at_tool)
            self.assertIsNone(proposal['estimated_tool_correction'])

    def test_historical_lift_without_current_observation_remains_unknown(self):
        document,images,kwargs=left_current_fixture()
        document['observation']['held_object']['pose_role']='CURRENT_TRACKED_ESTIMATE'
        held,_=held_result(document,images,kwargs)
        self.assertEqual(held['left_hold_status'],'UNKNOWN');self.assertIsNone(held['held_by_left'])

    def test_current_left_observation_and_lift_support_hold_but_not_cross_view_identity(self):
        document,images,kwargs=left_current_fixture();before=copy.deepcopy(document)
        held,proposal=held_result(document,images,kwargs)
        self.assertIs(held['held_by_left'],True);self.assertIsNone(held['target_association_verified'])
        self.assertFalse(held['physical_6D_pose_verified']);self.assertEqual(document,before)
        self.assertIsNone(proposal['estimated_tool_correction']);self.assertFalse(proposal['executable'])

    def test_stale_wrong_arm_or_missing_current_evidence_cannot_confirm_hold(self):
        for case in ('feedback','camera','tool','vision','lift','source'):
            document,images,kwargs=left_current_fixture();held=document['observation']['held_object']
            if case=='feedback':held['seq']=2
            elif case=='camera':held['vision_evidence']['n']=2
            elif case=='tool':held['transforms']['tool']='right_cube_grasp'
            elif case=='vision':held['vision_evidence']['observed']=False
            elif case=='lift':held['grasp_evidence']['held_after_lift']['jaw_during_lift']['loaded_throughout']=False
            else:held['source_kind']='MODEL'
            result,_=held_result(document,images,kwargs);self.assertIsNone(result['held_by_left'],case)

    def test_torque_and_search_prior_are_only_context(self):
        document,images,kwargs=native_fixture()
        document['observation'].update(left=dict(torque=True),held_object=dict(search_prior=dict(role='SEARCH_PRIOR')))
        held,_=held_result(document,images,kwargs)
        self.assertTrue(held['native_left_powered_hold_context']);self.assertIsNone(held['held_by_left'])

if __name__=='__main__':unittest.main(verbosity=2)
