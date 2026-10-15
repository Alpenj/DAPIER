"""Synthetic positives and source-integrity negatives; never physical confirmations."""
import copy
import json
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from .target_image import (CURRENT_TARGET, FIELD_TARGET, FEATURES, JOINTS,
                           TargetImageServoProposal, snapshot_observation, source_blob)
from .image_jacobian import jacobian_record
from .test_b19 import right_fixture
from .right_relative import RightRelativeServoProposal


def json_blob(value):
    return source_blob(json.dumps(value, sort_keys=True, allow_nan=False).encode())


def document(snapshot):
    return json.loads(snapshot['record']['bytes'])


def evidence(snapshot):
    return json.loads(snapshot['evidence']['bytes'])


def rebind(snapshot, value=None):
    value = evidence(snapshot) if value is None else copy.deepcopy(value)
    _, value['binding'], opening = snapshot_observation(snapshot)
    value['context']['right_gripper_present_tick'] = opening
    snapshot['evidence'] = json_blob(value)


def snapshot_fixture(name, shift=0, *, role=CURRENT_TARGET):
    ticks = {side: {n: 100.0 for n in JOINTS} for side in ('left', 'right')}
    q = [0.0] * 12
    camera = dict(n=1 + shift, fresh=True, reader_valid=True, recv_monotonic_s=10.0)
    d = dict(source_kind='TEST_FIXTURE', stage='SYNTHETIC_NOT_PHYSICAL', q12=q,
        files=dict(right_wrist=dict(rgb='right_wrist_rgb.png')),
        camera_frames=dict(right_wrist=camera),
        actual_feedback=dict(ticks=copy.deepcopy(ticks), usable=True, snapshot_only=False,
                             t_bus_monotonic=10.0, bus_seq=1),
        observation=dict(q12=q, present_ticks=ticks, fresh=True, feedback_stamp=10.0, seq=1))
    image = np.full((240, 320, 3), 255, np.uint8)
    image[70:150, 100 + shift:210 + shift] = [0, 0, 255]
    ok, encoded = cv2.imencode('.png', image)
    assert ok
    snapshot = dict(record=json_blob(d), image=dict(source_blob(encoded.tobytes()), name='right_wrist_rgb.png'),
                    observation_identity=dict(run_id='SYNTHETIC_FIXTURE', observation_id=name))
    value = dict(role=role, source_kind='TEST_FIXTURE', verified=True,
        verification_origin='TEST_FIXTURE',
        verification_source='SYNTHETIC_FIXTURE_NOT_PHYSICAL_CONFIRMATION', noncontact_target_verified=True,
        held_by_left=True, object_identity_verified=True, feature_correspondence_verified=True,
        mounting_verified=True, feature_unoccluded=True,
        context=dict(object_id='FIXTURE_OBJECT', physical_feature_id='FIXTURE_VISIBLE_RED_COMPONENT',
            mounting_condition='FIXTURE_MOUNT_EPOCH', camera_mode='RAW_BGR_320x240',
            calibration_identity='FIXTURE_CALIBRATION', right_gripper_present_tick=100.0))
    rebind(snapshot, value)
    return snapshot


def fixture_pair():
    target = snapshot_fixture('target', role=FIELD_TARGET)
    current = snapshot_fixture('current', 4)
    return current, dict(role=FIELD_TARGET, reference_id='SYNTHETIC_TARGET', snapshot=target)


def motion_pair(before, after, dx, basis):
    from .target_image import _sample
    b, a = _sample(before), _sample(after)
    return dict(before=before, after=after, motion=json_blob(dict(source_kind='TEST_FIXTURE',
        role='MEASURED_PHYSICAL_X_RESPONSE', axis_scale_verified=True,
        command_substituted_for_actual=False, local_translation_comparable=True,
        before_binding=b['binding'], after_binding=a['binding'], motion_basis_model=basis,
        actual_displacement_mm=dx)))


def jacobian_fixture(current, reference):
    comparison = TargetImageServoProposal(current, reference)
    basis = [[1.0], [0.0], [0.0]]
    fit = motion_pair(reference['snapshot'], current, 2.0, basis)
    independent = motion_pair(current, snapshot_fixture('independent', 6), 1.0, basis)
    sources = dict(reference=reference['snapshot']['evidence']['sha256'],
        fit_motion=[fit['motion']['sha256']], independent_motion=independent['motion']['sha256'])
    j = jacobian_record([[2.0]], feature_axes=['u'], motion_basis_model=basis, source='TEST_FIXTURE',
        context=comparison['comparison_context'], feature_definitions=FEATURES, source_sha256=sources)
    return dict(jacobian=j, fit_pairs=[fit], independent_pair=independent)


class TargetImageTests(unittest.TestCase):
    def assert_negative(self, current, reference, reason, j=None, *, image_error=False):
        result = TargetImageServoProposal(current, reference, j)
        self.assertIsNone(result['estimated_tool_correction'])
        self.assertFalse(result['REAL_J_VALIDATED'])
        self.assertFalse(result['executable']); self.assertFalse(result['motor_write_authorized'])
        self.assertTrue(any(reason in x for x in result['missing_evidence']), result['missing_evidence'])
        if not image_error:
            self.assertIsNone(result['feature_error_px']['u'])
        return result

    def test_synthetic_positive_uses_independent_bindings_and_only_X(self):
        current, reference = fixture_pair(); before = copy.deepcopy((current, reference))
        result = TargetImageServoProposal(current, reference)
        self.assertEqual(result['validity'], 'TARGET_IMAGE_ERROR_ONLY')
        self.assertEqual(result['feature_error_px'], dict(u=4.0, v=None))
        self.assertEqual(result['observed_image_axes'], ['u'])
        self.assertNotEqual(result['reference_binding'], result['current_binding'])
        self.assertFalse(result['graph_requested']); self.assertTrue(result['model_only'])
        self.assertIsNone(result['estimated_tool_correction'])
        self.assertEqual((current, reference), before)

    def test_missing_physical_target_confirmation(self):
        for field, replacement in (('noncontact_target_verified', False), ('verified', False),
                                   ('verification_source', ''), ('held_by_left', False)):
            with self.subTest(field=field):
                current, reference = fixture_pair(); value = evidence(reference['snapshot'])
                value[field] = replacement; reference['snapshot']['evidence'] = json_blob(value)
                reason = 'FIELD_PHYSICAL' if field == 'noncontact_target_verified' else (
                    'HELD_BY_LEFT' if field == 'held_by_left' else 'PHYSICAL_CONFIRMATION')
                self.assert_negative(current, reference, reason)

    def test_reference_types_remain_independent(self):
        current, reference = fixture_pair(); reference['role'] = 'CURRENT_OBSERVED_PAD'
        self.assert_negative(current, reference, 'INDEPENDENT_FIELD_TARGET')
        obs, held, pad, j = right_fixture()
        self.assertEqual(RightRelativeServoProposal(obs['q12'], obs, held, pad, j)['observed_control_dimensions'], 1)

    def test_copied_binding_and_reused_identity_are_rejected(self):
        current, reference = fixture_pair(); value = evidence(current)
        value['binding'] = evidence(reference['snapshot'])['binding']; current['evidence'] = json_blob(value)
        self.assert_negative(current, reference, 'EVIDENCE_SOURCE_BINDING')
        current, reference = fixture_pair(); current['observation_identity'] = reference['snapshot']['observation_identity']
        rebind(current)
        self.assert_negative(current, reference, 'OBSERVATION_IDENTITY_REUSED')

    def test_object_feature_mount_mode_and_calibration_mismatch(self):
        for field, reason in (('object_id', 'OBJECT_IDENTITY'), ('physical_feature_id', 'PHYSICAL_FEATURE'),
                              ('mounting_condition', 'CAMERA_MOUNT'), ('camera_mode', 'CAMERA_MODE'),
                              ('calibration_identity', 'CALIBRATION_CONTEXT')):
            with self.subTest(field=field):
                current, reference = fixture_pair(); value = evidence(current)
                value['context'][field] = 'DIFFERENT'; current['evidence'] = json_blob(value)
                self.assert_negative(current, reference, reason)

    def test_actual_opening_change_is_a_comparison_failure_not_a_motion_stop(self):
        current, reference = fixture_pair(); d = document(current)
        for ticks in (d['actual_feedback']['ticks'], d['observation']['present_ticks']):
            ticks['right']['gripper'] = 101.0
        current['record'] = json_blob(d); rebind(current)
        self.assert_negative(current, reference, 'OPENING_CONTEXT')

    def test_original_source_SHA_and_actual_feedback_are_checked_on_both_sides(self):
        for side in ('current', 'reference'):
            for fault in ('record_sha', 'image_sha', 'evidence_sha', 'q12', 'ticks', 'seq'):
                with self.subTest(side=side, fault=fault):
                    current, reference = fixture_pair(); sample = current if side == 'current' else reference['snapshot']
                    if fault.endswith('_sha'):
                        sample[fault[:-4]]['sha256'] = '0' * 64; reason = 'SOURCE_SHA'
                    else:
                        d = document(sample)
                        if fault == 'q12': d['q12'][6] = 0.5
                        elif fault == 'ticks': d['actual_feedback']['ticks']['right']['wrist_roll'] = 101.0
                        else: d['actual_feedback']['bus_seq'] = 2
                        sample['record'] = json_blob(d); reason = 'ACTUAL_FEEDBACK_SOURCE'
                    self.assert_negative(current, reference, reason)

    def test_clipped_occluded_and_missing_features_are_unusable(self):
        for side in ('current', 'reference'):
            for fault in ('clipped', 'occluded', 'invisible'):
                with self.subTest(side=side, fault=fault):
                    current, reference = fixture_pair(); sample = current if side == 'current' else reference['snapshot']
                    if fault == 'occluded':
                        value = evidence(sample); value['feature_unoccluded'] = False
                        sample['evidence'] = json_blob(value); reason = 'FEATURE_UNOCCLUDED'
                    else:
                        image = np.full((240, 320, 3), 255, np.uint8)
                        if fault == 'clipped': image[70:150, :110] = [0, 0, 255]
                        sample['image'] = dict(source_blob(cv2.imencode('.png', image)[1].tobytes()), name='right_wrist_rgb.png')
                        rebind(sample); reason = 'UNUSABLE_OBJECT_CENTROID'
                    self.assert_negative(current, reference, reason)

    def test_model_real_and_fixture_provenance_cannot_mix(self):
        for kind in ('MODEL', 'REAL'):
            current, reference = fixture_pair(); d = document(current); d['source_kind'] = kind
            current['record'] = json_blob(d)
            if kind == 'REAL':
                value = evidence(current); value['source_kind'] = kind; rebind(current, value)
            self.assert_negative(current, reference, 'REAL_OR_EXPLICIT' if kind == 'MODEL' else 'PHYSICAL_CONFIRMATION_SOURCE')

    def test_J_flag_alone_never_produces_a_correction(self):
        for j in (None, {}, dict(jacobian=dict(source='REAL_MEASURED', validated=False))):
            current, reference = fixture_pair()
            result = TargetImageServoProposal(current, reference, j)
            self.assertEqual(result['feature_error_px']['u'], 4.0)
            self.assertIsNone(result['estimated_tool_correction']); self.assertFalse(result['REAL_J_VALIDATED'])

    def test_REAL_J_validation_flag_and_REAL_fit_sources_are_separate_requirements(self):
        # Negative schema probes stop before any synthetic observation can become REAL evidence.
        from .target_image import _sample, _validated_jacobian
        current, reference = fixture_pair(); envelope = jacobian_fixture(current, reference)
        sample = _sample(current); sample['observation']['source_kind'] = 'REAL'
        envelope['jacobian']['source'] = 'REAL_MEASURED'
        for validated, reason in ((False, 'VALIDATED_REAL_TARGET_IMAGE'), (True, 'MODEL_REAL_OR_FIXTURE_MIX')):
            envelope['jacobian']['validated'] = validated
            with self.assertRaisesRegex(ValueError, reason):
                _validated_jacobian(envelope, sample, envelope['jacobian']['context'])

    def test_independent_fit_and_sign_are_recomputed_from_their_sources(self):
        current, reference = fixture_pair(); j = jacobian_fixture(current, reference)
        self.assert_negative(current, reference, 'TEST_FIXTURE_IS_NOT_A_REAL_CORRECTION', j, image_error=True)
        for fault, reason in (('reuse', 'FIT_INDEPENDENT'), ('source', 'JACOBIAN_EVIDENCE_SOURCE'),
                              ('matrix', 'JACOBIAN_FIT_SOURCE'), ('reversed', 'INDEPENDENT_SIGN')):
            with self.subTest(fault=fault):
                altered = copy.deepcopy(j)
                if fault == 'reuse': altered['independent_pair'] = altered['fit_pairs'][0]
                elif fault == 'source': altered['jacobian']['source_sha256']['independent_motion'] = '0' * 64
                elif fault == 'matrix': altered['jacobian']['matrix_px_per_mm'] = [[3.0]]
                else:
                    altered['independent_pair'] = motion_pair(current, snapshot_fixture('reversed', 2), 1.0, [[1.0], [0.0], [0.0]])
                    altered['jacobian']['source_sha256']['independent_motion'] = altered['independent_pair']['motion']['sha256']
                self.assert_negative(current, reference, reason, altered, image_error=True)

    def test_feedback_and_camera_freshness_are_capture_evidence(self):
        for fault in ('usable', 'snapshot_only', 'fresh', 'reader_valid'):
            current, reference = fixture_pair(); d = document(current)
            if fault == 'usable': d['actual_feedback']['usable'] = False
            elif fault == 'snapshot_only': d['actual_feedback']['snapshot_only'] = True
            else: d['camera_frames']['right_wrist'][fault] = False
            current['record'] = json_blob(d)
            self.assert_negative(current, reference, 'ACTUAL_FEEDBACK' if fault in ('usable', 'snapshot_only') else 'CAMERA_FRESHNESS')

    def test_unknown_physical_context_is_not_comparable(self):
        for field in ('object_id', 'physical_feature_id', 'mounting_condition', 'camera_mode', 'calibration_identity'):
            for unknown in ('UNKNOWN', ' missing ', 'unavailable', 'UNVERIFIED'):
                current, reference = fixture_pair()
                for sample in (current, reference['snapshot']):
                    value = evidence(sample); value['context'][field] = unknown; sample['evidence'] = json_blob(value)
                self.assert_negative(current, reference, '_UNIDENTIFIED')

    def test_missing_feedback_or_capture_identity_is_not_equal_null(self):
        for fault in ('stamp', 'seq', 'frame', 'receipt', 'tick_list'):
            current, reference = fixture_pair(); d = document(current)
            if fault == 'stamp':
                d['actual_feedback']['t_bus_monotonic'] = d['observation']['feedback_stamp'] = None
            elif fault == 'seq':
                d['actual_feedback']['bus_seq'] = d['observation']['seq'] = None
            elif fault in ('frame', 'receipt'):
                d['camera_frames']['right_wrist']['n' if fault == 'frame' else 'recv_monotonic_s'] = None
            else:
                d['actual_feedback']['ticks']['right']['gripper'] = [100.0]
                d['observation']['present_ticks']['right']['gripper'] = [100.0]
            current['record'] = json_blob(d)
            self.assert_negative(current, reference, 'FINITE_MEASURED_PRESENT' if fault == 'tick_list' else 'IDENTITY_REQUIRED')

    def test_malformed_nested_documents_fail_with_structured_evidence_status(self):
        for fault in ('observation', 'actual_feedback', 'camera', 'context', 'J'):
            current, reference = fixture_pair(); j = None
            if fault == 'context':
                value = evidence(current); value['context'] = []; current['evidence'] = json_blob(value)
            elif fault == 'J':
                j = dict(jacobian=[])
            else:
                d = document(current)
                if fault == 'camera': d['camera_frames']['right_wrist'] = []
                else: d[fault] = []
                current['record'] = json_blob(d)
            self.assert_negative(current, reference, 'DOCUMENT_REQUIRED' if fault != 'camera' else 'CAPTURE_IDENTITY', j,
                                 image_error=fault == 'J')

    def test_reencoded_fit_capture_is_not_an_independent_sign_frame(self):
        current, reference = fixture_pair(); j = jacobian_fixture(current, reference)
        duplicate = copy.deepcopy(current); d = document(duplicate); d['stage'] = 'REPACKAGED_SAME_CAPTURE'
        duplicate['record'] = json_blob(d); duplicate['observation_identity']['observation_id'] = 'repackaged'
        image = cv2.imdecode(np.frombuffer(duplicate['image']['bytes'], np.uint8), cv2.IMREAD_COLOR)
        duplicate['image'] = dict(source_blob(cv2.imencode('.png', image, [cv2.IMWRITE_PNG_COMPRESSION, 9])[1].tobytes()),
                                  name='right_wrist_rgb.png')
        self.assertNotEqual(duplicate['image']['sha256'], current['image']['sha256'])
        rebind(duplicate)
        j['independent_pair'] = motion_pair(reference['snapshot'], duplicate, 2.0, [[1.0], [0.0], [0.0]])
        j['jacobian']['source_sha256']['independent_motion'] = j['independent_pair']['motion']['sha256']
        self.assert_negative(current, reference, 'FIT_INDEPENDENT', j, image_error=True)

    def test_new_capture_may_revisit_identical_pixels(self):
        current, reference = fixture_pair(); j = jacobian_fixture(current, reference)
        revisit = snapshot_fixture('independent_return', role=CURRENT_TARGET)
        d = document(revisit); d['camera_frames']['right_wrist']['n'] = 20
        revisit['record'] = json_blob(d); rebind(revisit)
        self.assertEqual(revisit['image']['sha256'], reference['snapshot']['image']['sha256'])
        j['independent_pair'] = motion_pair(current, revisit, -2.0, [[1.0], [0.0], [0.0]])
        j['jacobian']['source_sha256']['independent_motion'] = j['independent_pair']['motion']['sha256']
        self.assert_negative(current, reference, 'TEST_FIXTURE_IS_NOT_A_REAL_CORRECTION', j, image_error=True)

    def test_unverified_or_mixed_X_motion_never_validates_J(self):
        for fault in ('axis_scale_verified', 'local_translation_comparable', 'command_substituted_for_actual'):
            current, reference = fixture_pair(); j = jacobian_fixture(current, reference)
            motion = json.loads(j['fit_pairs'][0]['motion']['bytes'])
            motion[fault] = fault == 'command_substituted_for_actual'; j['fit_pairs'][0]['motion'] = json_blob(motion)
            self.assert_negative(current, reference, 'ACTUAL_X_RESPONSE_SOURCE', j, image_error=True)

    def test_malformed_or_unidentified_reference_is_not_a_partial_target(self):
        current, reference = fixture_pair()
        for wrong in (None, [], {}, dict(reference, reference_id='')):
            self.assert_negative(current, wrong, 'REFERENCE' if not isinstance(wrong, dict) else 'INDEPENDENT_FIELD')

    def test_target_consumer_preserves_the_fast_path(self):
        current, reference = fixture_pair()
        from .right_relative import _right_functions
        from .features import ROOT, sha256
        source = (ROOT / 'native/revision').resolve()
        digests = tuple(sha256(source / name) for name in ('sweepick_pick01.py',
            'right_wrist_relative_geometry.py', 'sweepick_261009_offsite_relative_features.py'))
        _, geometry, _ = _right_functions(source, digests)
        with patch.object(geometry, 'observed_face_graph', side_effect=AssertionError('graph must stay optional')) as graph:
            TargetImageServoProposal(current, reference)
            graph.assert_not_called()
        self.assertFalse(TargetImageServoProposal(current, reference)['graph_requested'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
