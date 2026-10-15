"""Contract fixtures are synthetic tests only; they never become REAL goals or labels."""
import copy
import os
from pathlib import Path
import unittest

import numpy as np

from b16_goal_bridge import GoalProposalBridge, MODEL_FRAME, read_json, load_module

PRIVATE_ROOT = os.environ.get('B16_PRIVATE_ROOT')
HERE = Path(PRIVATE_ROOT) if PRIVATE_ROOT else None
AOfflineGeometry = (load_module('b16_private_geometry', HERE/'b16_goal_bridge.py').AOfflineGeometry
                    if HERE else None)

A_GEOMETRY = None
TEST_EVIDENCE = {}


def contract_fixture():
    stamp = dict(value=1, unit='ns', clock='TEST_FIXTURE_CLOCK')
    return dict(
        run_id='TEST_FIXTURE_NOT_A_REAL_RUN', observation_id='contract_only',
        measurement_source_kinds=['REAL'], coordinate_frame=MODEL_FRAME,
        canonical_state=[0.0]*12, target_object=dict(id='TEST_FIXTURE_OBJECT'),
        held_pose=dict(status='OBSERVED_TEST_FIXTURE', object_pose_model=np.eye(4).tolist()),
        geometry_evidence=dict(centre_identified=True, full_rotation_observed=True,
                               pad_object_relation_status='OK', calibration_verified=True),
        reference_times=dict(joint_state=dict(left=stamp, right=copy.deepcopy(stamp)),
            sensors=[dict(stamp, state_reference=copy.deepcopy(stamp), native_exposure_known=True)]),
        a_observation_candidate=dict(source_kind='SENSOR_SIM_CANDIDATE',
            targets=[dict(side='right', reference_frame='CURRENT_TOOL',
                          reference_from_target_tool=np.eye(4).tolist())],
            expected_observed_components=['TEST_FIXTURE_EDGE'], observation_policy='FINAL_TARGET_ONLY'),
        provenance=dict(observed=[], estimated=[], model_assumptions=['TEST_FIXTURE_ONLY'], unidentified=[]),
        source_sha256={}, source_files=[], required_additional_observations=[])


class GoalContractTests(unittest.TestCase):
    def setUp(self):
        self.bridge = GoalProposalBridge()
        self.record = contract_fixture()

    def check_missing(self, expected):
        result = self.bridge.propose(self.record, lambda _: self.fail('preview called with missing evidence'))
        self.assertEqual(result['status'], 'NEEDS_OBSERVATION')
        self.assertIn(expected, result['unknown_geometry'])
        self.assertEqual(result['targets'], [])
        self.assertFalse(result['executable'])
        self.assertFalse(result['motion_requested'])

    def check_model_preview(self, expected):
        result = self.bridge.propose(self.record, lambda _: dict(
            ik_solution_exists=True, collision_result='CLEAR'))
        self.assertEqual(result['status'], 'MODEL_PREVIEW_ONLY')
        self.assertIn(expected, result['temporal_alignment']['diagnostics'])
        self.assertNotIn(expected, result['unknown_geometry'])
        self.assertFalse(result['executable'])

    def test_01_missing_goal_frame(self):
        self.record['coordinate_frame'] = None
        self.check_missing('TARGET_COORDINATE_FRAME_MISSING')

    def test_02_nonexistent_object_position(self):
        self.record['held_pose']['object_pose_model'] = None
        self.check_missing('OBJECT_POSITION_MISSING')

    def test_03_held_pose_missing(self):
        self.record['held_pose']['status'] = 'MISSING'
        self.check_missing('HELD_POSE_MISSING')

    def test_04_sensor_before_stored_state(self):
        self.record['reference_times']['sensors'][0]['value'] = 0
        self.check_model_preview('CAMERA_RECEIVED_BEFORE_REFERENCED_STATE')

    def test_05_bimanual_feedback_different_times(self):
        self.record['reference_times']['joint_state']['left']['value'] = 0
        self.check_model_preview('ASYNCHRONOUS_BIMANUAL_FEEDBACK')

    def test_07_ik_failure_never_promotes(self):
        result = self.bridge.propose(self.record, lambda _: dict(
            status='IK_FAILED_TEST_FIXTURE', ik_solution_exists=False, collision_result='NOT_EVALUATED'))
        self.assertEqual(result['status'], 'PREVIEW_FAILED')
        self.assertFalse(result['executable'])

    def test_08_nonclear_collision_never_promotes(self):
        for verdict in ('COLLISION', 'UNKNOWN', 'INCOMPLETE'):
            with self.subTest(verdict=verdict):
                result = GoalProposalBridge().propose(self.record, lambda _: dict(
                    status='MODEL_TEST_FIXTURE', ik_solution_exists=True, collision_result=verdict))
                self.assertEqual(result['status'], 'PREVIEW_FAILED')
                self.assertFalse(result['executable'])

    def test_09_REAL_SIM_measurement_mix(self):
        self.record['measurement_source_kinds'] = ['REAL', 'SIM']
        result = self.bridge.propose(self.record)
        self.assertEqual(result['status'], 'EXCLUDED')
        self.assertEqual(result['targets'], [])

    def test_10_duplicate_observation_id_is_scoped_to_run(self):
        first = self.bridge.propose(self.record)
        self.assertEqual(first['status'], 'PREVIEW_UNSUPPORTED')
        second = self.bridge.propose(self.record)
        self.assertEqual(second['status'], 'EXCLUDED')
        self.assertIn('DUPLICATE_OBSERVATION_ID_WITHIN_RUN', second['invalid_reasons'])
        self.record['run_id'] = 'TEST_FIXTURE_OTHER_RUN'
        self.assertEqual(self.bridge.propose(self.record)['status'], 'PREVIEW_UNSUPPORTED')

    def test_clear_model_preview_stays_shadow(self):
        result = self.bridge.propose(self.record, lambda _: dict(
            status='MODEL_TEST_FIXTURE', ik_solution_exists=True, collision_result='CLEAR'))
        self.assertEqual(result['status'], 'MODEL_PREVIEW_ONLY')
        for key in ('executable', 'motor_write_authorized', 'motion_requested', 'insertion_requested'):
            self.assertFalse(result[key])
        self.assertTrue(result['shadow_only'])

    def test_partial_pose_is_not_measured_pose(self):
        self.record['geometry_evidence']['centre_identified'] = False
        self.check_missing('HELD_CENTRE_NOT_OBSERVED')

    def test_different_clock_not_subtracted(self):
        self.record['reference_times']['joint_state']['left']['clock'] = 'OTHER_TEST_CLOCK'
        self.check_model_preview('JOINT_CLOCKS_UNCOMPARABLE')

    def test_malformed_relative_transform_is_excluded(self):
        self.record['a_observation_candidate']['targets'][0]['reference_from_target_tool'][0][0] = 2
        self.assertEqual(self.bridge.propose(self.record)['status'], 'EXCLUDED')

    def test_malformed_A_candidate_is_excluded_without_preview(self):
        for candidate in ([], {}, {'source_kind':'SENSOR_SIM_CANDIDATE', 'targets':'bad'}):
            with self.subTest(candidate=candidate):
                record = contract_fixture()
                record['a_observation_candidate'] = candidate
                result = GoalProposalBridge().propose(record, lambda _: self.fail('malformed candidate previewed'))
                self.assertEqual(result['status'], 'EXCLUDED')
                self.assertEqual(result['targets'], [])


@unittest.skipUnless(PRIVATE_ROOT, 'requires local frozen B16 evidence/model via B16_PRIVATE_ROOT')
class ActualAPlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        global A_GEOMETRY
        if A_GEOMETRY is None:
            A_GEOMETRY = AOfflineGeometry()
        cls.geometry = A_GEOMETRY

    def test_06_ACT_left_right_elbow_exceedance_unclipped(self):
        prediction = read_json(HERE/'inputs/B15/B15_SHADOW_BRIDGE_REPORT.json')['prediction_results'][0]
        result = self.geometry.act_diagnostics(prediction)
        axes = {x['axis'] for x in result['saved_servo_range_exceedances']}
        self.assertIn('left_elbow_flex', axes)
        self.assertIn('right_elbow_flex', axes)
        self.assertTrue(result['B15_left_right_elbow_reproduced'])
        self.assertEqual(result['source_canonical_chunk'], prediction['proposal']['action_chunk_absolute_canonical'])
        self.assertFalse(result['joint_clipping'])
        self.assertFalse(result['usable_goal_generated'])
        self.assertEqual(result['A_tool_pose_max_abs_difference'], dict(left=0.0, right=0.0))
        TEST_EVIDENCE['ACT_range_case'] = dict(source_request=prediction['request_id'],
            exceedance_count=len(result['saved_servo_range_exceedances']), axes=sorted(axes), clipped=False)

    def test_existing_A_IK_really_fails_for_unreachable_test_target(self):
        g = self.geometry
        request = read_json(HERE/'inputs/A_web_20261014/OBSERVATION_MOVE_REQUEST.json')
        teacher = g.tm.HandoverTeacher(g.model)
        seed = g.tm.decode_command(request['plan']['command_seed_q12'], g.model)
        base = g.model.qpos0.copy()
        teacher.plan.pose(base, seed)
        _, rotation = teacher.plan.site_frame('right')
        goal, reason = teacher.plan.solve(base, seed, 'right', np.array([1000., 1000., 1000.]),
                                         rotation[:, 0], [rotation[:, 2]], teacher.cfg)
        self.assertIsNone(goal)
        TEST_EVIDENCE['unreachable_IK'] = dict(source_kind='TEST_FIXTURE',
            target_m=[1000., 1000., 1000.], solver='A Planner.solve', ik_solution_exists=False, reason=reason)

    def test_existing_A_collision_really_nonclear_for_test_obstacle(self):
        g = self.geometry
        request = read_json(HERE/'inputs/A_web_20261014/OBSERVATION_MOVE_REQUEST.json')
        seed = g.tm.decode_command(request['plan']['command_seed_q12'], g.model)
        planner = g.tm.Planner(g.model)
        base = g.model.qpos0.copy()
        data = planner.pose(base, seed)
        position = data.geom_xpos[g.model.geom('right_pgripper_housing').id].copy()
        block = np.r_[position, [1., 0., 0., 0.]]
        hit = planner.path_collisions(base, seed, seed, block_qpos=block)
        self.assertIsNotNone(hit)
        TEST_EVIDENCE['nonclear_collision'] = dict(source_kind='TEST_FIXTURE',
            description='block intersects right housing on private MjData; not a REAL inferred object',
            checker='A Planner.path_collisions', verdict='COLLISION', hit=hit)


if __name__ == '__main__':
    unittest.main(verbosity=2)
