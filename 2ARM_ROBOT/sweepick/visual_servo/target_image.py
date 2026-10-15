"""Source-bound target-image X comparisons; no observed-Pad substitution or execution."""
import copy
import hashlib
import json

import cv2
import numpy as np

from .features import observation_binding
from .image_jacobian import estimate_jacobian, jacobian_record
from .right_relative import extract_right_features
from .servo_proposal import propose_servo

FIELD_TARGET = 'FIELD_VERIFIED_TARGET_IMAGE'
CURRENT_TARGET = 'CURRENT_TARGET_IMAGE_OBSERVATION'
FEATURES = dict(u='RIGHT_HELD_COMPONENT_MINUS_FIELD_TARGET_U', v=None)
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
UNIDENTIFIED = {'unknown', 'missing', 'unavailable', 'unverified', 'none', 'null'}


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and np.isfinite(value)


def source_blob(data):
    """Package original bytes; this establishes content identity, not physical truth."""
    if not isinstance(data, bytes) or not data:
        raise ValueError('NONEMPTY_SOURCE_BYTES_REQUIRED')
    return dict(bytes=data, sha256=hashlib.sha256(data).hexdigest())


def _bytes(blob):
    data = blob['bytes']
    if source_blob(data)['sha256'] != blob['sha256']:
        raise ValueError('SOURCE_SHA_MISMATCH')
    return data


def _json(blob):
    def invalid(value):
        raise ValueError('NONFINITE_SOURCE_JSON: ' + value)
    value = json.loads(_bytes(blob), parse_constant=invalid)
    if not isinstance(value, dict):
        raise ValueError('SOURCE_DOCUMENT_REQUIRED')
    return value


def snapshot_observation(snapshot):
    """Rebuild each binding from its own original A record and encoded right image."""
    d = _json(snapshot['record'])
    kind = d['source_kind']
    if kind not in ('REAL', 'TEST_FIXTURE'):
        raise ValueError('REAL_OR_EXPLICIT_TEST_FIXTURE_REQUIRED')
    native, feedback = d['observation'], d['actual_feedback']
    if not isinstance(native, dict) or not isinstance(feedback, dict):
        raise ValueError('ACTUAL_FEEDBACK_DOCUMENT_REQUIRED')
    if native.get('source_kind', kind) != kind or feedback.get('source_kind', kind) != kind:
        raise ValueError('FEEDBACK_PROVENANCE_MIX')
    for raw_q in (d['q12'], native['q12']):
        if not isinstance(raw_q, list) or len(raw_q) != 12 or not all(_number(v) for v in raw_q):
            raise ValueError('FINITE_MEASURED_Q12_REQUIRED')
    if (not _number(feedback.get('t_bus_monotonic')) or not _number(native.get('feedback_stamp'))
            or any(not isinstance(value, int) or isinstance(value, bool) for value in
                   (feedback.get('bus_seq'), native.get('seq')))):
        raise ValueError('ACTUAL_FEEDBACK_IDENTITY_REQUIRED')
    q = np.asarray(d['q12'], float)
    if (q.shape != (12,) or not np.isfinite(q).all()
            or not np.array_equal(q, np.asarray(native['q12'], float))
            or feedback.get('usable') is not True or feedback.get('snapshot_only') is not False
            or native.get('fresh') is not True or feedback['ticks'] != native['present_ticks']
            or feedback['t_bus_monotonic'] != native['feedback_stamp']
            or feedback['bus_seq'] != native['seq']):
        raise ValueError('ACTUAL_FEEDBACK_SOURCE_MISMATCH')
    ticks = feedback['ticks']
    for side in ('left', 'right'):
        if (not isinstance(ticks[side], dict) or set(ticks[side]) != set(JOINTS)
                or not all(_number(ticks[side][n]) for n in JOINTS)):
            raise ValueError('FINITE_MEASURED_PRESENT_TICKS_REQUIRED')
    camera = d['camera_frames']['right_wrist']
    if (not isinstance(camera, dict) or not isinstance(camera.get('n'), int)
            or isinstance(camera['n'], bool) or not _number(camera.get('recv_monotonic_s'))):
        raise ValueError('CAMERA_CAPTURE_IDENTITY_REQUIRED')
    if camera.get('fresh') is not True or camera.get('reader_valid') is not True:
        raise ValueError('RECORDED_CAMERA_FRESHNESS_UNVERIFIED')
    image = cv2.imdecode(np.frombuffer(_bytes(snapshot['image']), np.uint8), cv2.IMREAD_COLOR)
    if image is None or snapshot['image']['name'] != d['files']['right_wrist']['rgb']:
        raise ValueError('SOURCE_IMAGE_MISMATCH')
    identity = snapshot['observation_identity']
    views = {key: dict(source_kind=kind, observation_identity=copy.deepcopy(identity), unavailable=True)
             for key in ('top', 'left_wrist', 'right_wrist')}
    views['right_wrist'] = dict(source_kind=kind, observation_identity=copy.deepcopy(identity),
        image_bgr=image, image_identity=dict(stream='right_wrist', n=camera['n'], file=snapshot['image']['name']),
        source_sha256=snapshot['image']['sha256'], recorded_camera=copy.deepcopy(camera))
    observation = dict(source_kind=kind, observation_identity=copy.deepcopy(identity), q12=q.tolist(),
                       views=views, reference_times={})
    binding = observation_binding(observation, camera_key='right_wrist')
    binding = {key: binding[key] for key in ('observation_identity', 'feedback_sha256', 'camera_state_binding')}
    binding.update(record_sha256=snapshot['record']['sha256'], image_sha256=snapshot['image']['sha256'])
    return observation, binding, ticks['right']['gripper']


def _sample(snapshot, *, role=None, a_source=None):
    observation, binding, opening = snapshot_observation(snapshot)
    if 'evidence' not in snapshot:
        raise ValueError('FIELD_PHYSICAL_CONFIRMATION_REQUIRED' if role == FIELD_TARGET else 'CURRENT_PHYSICAL_CONTEXT_REQUIRED')
    evidence = _json(snapshot['evidence'])
    if (evidence.get('source_kind') != observation['source_kind'] or evidence.get('binding') != binding
            or evidence.get('role') not in (FIELD_TARGET, CURRENT_TARGET)
            or role is not None and evidence.get('role') != role):
        raise ValueError('EVIDENCE_SOURCE_BINDING_MISMATCH')
    origin = 'OPERATOR_PHYSICAL_REVIEW' if observation['source_kind'] == 'REAL' else 'TEST_FIXTURE'
    if (evidence.get('verification_origin') != origin
            or evidence.get('verified') is not True or not isinstance(evidence.get('verification_source'), str)
            or not evidence['verification_source'].strip()):
        raise ValueError('PHYSICAL_CONFIRMATION_SOURCE_REQUIRED')
    if evidence['role'] == FIELD_TARGET and evidence.get('noncontact_target_verified') is not True:
        raise ValueError('FIELD_PHYSICAL_CONFIRMATION_REQUIRED')
    for field in ('held_by_left', 'object_identity_verified', 'feature_correspondence_verified',
                  'mounting_verified', 'feature_unoccluded'):
        if evidence.get(field) is not True:
            raise ValueError(field.upper() + '_UNVERIFIED')
    context = evidence['context']
    if not isinstance(context, dict):
        raise ValueError('PHYSICAL_CONTEXT_DOCUMENT_REQUIRED')
    for field in ('object_id', 'physical_feature_id', 'mounting_condition', 'camera_mode', 'calibration_identity'):
        if (not isinstance(context.get(field), str) or not context[field].strip()
                or context[field].strip().casefold() in UNIDENTIFIED):
            raise ValueError(field.upper() + '_UNIDENTIFIED')
    if (isinstance(context.get('right_gripper_present_tick'), bool)
            or context.get('right_gripper_present_tick') != opening):
        raise ValueError('OPENING_FEEDBACK_SOURCE_MISMATCH')
    features = extract_right_features(observation, {}, {}, a_source=a_source)
    if not features['feature_validity']['centroid_usable_for_control']:
        raise ValueError('UNUSABLE_OBJECT_CENTROID')
    binding['evidence_sha256'] = snapshot['evidence']['sha256']
    return dict(observation=observation, binding=binding, context=copy.deepcopy(context),
                u=features['object_component_centroid_uv']['u'],
                capture_identity=(observation['observation_identity']['run_id'], 'right_wrist',
                                  binding['camera_state_binding']['image_identity']['n']),
                graph_requested=features['visible_approach_region']['graph_requested'])


def _same_context(before, after):
    if before['observation']['source_kind'] != after['observation']['source_kind']:
        raise ValueError('MODEL_REAL_OR_FIXTURE_MIX')
    if (before['binding']['observation_identity'] == after['binding']['observation_identity']
            and any(before['binding'][k] != after['binding'][k] for k in before['binding'] if k != 'evidence_sha256')):
        raise ValueError('OBSERVATION_IDENTITY_REUSED_FOR_DIFFERENT_SOURCE')
    labels = dict(object_id='OBJECT_IDENTITY_MISMATCH', physical_feature_id='PHYSICAL_FEATURE_MISMATCH',
        right_gripper_present_tick='OPENING_CONTEXT_MISMATCH', mounting_condition='CAMERA_MOUNT_MISMATCH',
        camera_mode='CAMERA_MODE_MISMATCH', calibration_identity='CALIBRATION_CONTEXT_MISMATCH')
    for field, message in labels.items():
        if before['context'][field] != after['context'][field]:
            raise ValueError(message)
    if (before['binding']['camera_state_binding']['image_shape'] != after['binding']['camera_state_binding']['image_shape']
            or before['binding']['camera_state_binding']['image_dtype'] != after['binding']['camera_state_binding']['image_dtype']):
        raise ValueError('CAMERA_IMAGE_MODE_MISMATCH')


def _pair(pair, sample, basis, *, a_source=None):
    before, after = (_sample(pair[key], a_source=a_source) for key in ('before', 'after'))
    for value in (before, after):
        _same_context(sample, value)
    if (before['binding']['record_sha256'] == after['binding']['record_sha256']
            or before['capture_identity'] == after['capture_identity']):
        raise ValueError('NEW_RESPONSE_OBSERVATION_REQUIRED')
    measurement = _json(pair['motion'])
    if (measurement.get('source_kind') != sample['observation']['source_kind']
            or measurement.get('role') != 'MEASURED_PHYSICAL_X_RESPONSE'
            or measurement.get('axis_scale_verified') is not True
            or measurement.get('local_translation_comparable') is not True
            or measurement.get('command_substituted_for_actual') is not False
            or measurement.get('before_binding') != before['binding']
            or measurement.get('after_binding') != after['binding']
            or measurement.get('motion_basis_model') != basis):
        raise ValueError('ACTUAL_X_RESPONSE_SOURCE_UNVERIFIED')
    dx = measurement['actual_displacement_mm']
    if isinstance(dx, bool) or not np.isscalar(dx) or not np.isfinite(dx) or dx == 0:
        raise ValueError('IDENTIFIABLE_ACTUAL_X_REQUIRED')
    return float(dx), after['u'] - before['u'], before, after


def _validated_jacobian(envelope, sample, context, *, a_source=None):
    j = envelope['jacobian']
    if not isinstance(j, dict):
        raise ValueError('JACOBIAN_DOCUMENT_REQUIRED')
    kind = sample['observation']['source_kind']
    source = 'REAL_MEASURED' if kind == 'REAL' else 'TEST_FIXTURE'
    if (j.get('source') != source or kind == 'REAL' and j.get('validated') is not True
            or j.get('context') != context or j.get('feature_definitions') != FEATURES
            or j.get('feature_axes') != ['u'] or np.asarray(j['matrix_px_per_mm']).shape != (1, 1)):
        raise ValueError('VALIDATED_REAL_TARGET_IMAGE_JACOBIAN_REQUIRED')
    checked = jacobian_record(j['matrix_px_per_mm'], feature_axes=j['feature_axes'],
        motion_basis_model=j['motion_basis_model'], source=source, context=context,
        feature_definitions=FEATURES, source_sha256=j['source_sha256'], validated=j.get('validated') is True)
    if not envelope['fit_pairs']:
        raise ValueError('FIT_RESPONSE_EVIDENCE_REQUIRED')
    fitted = [_pair(pair, sample, checked['motion_basis_model'], a_source=a_source) for pair in envelope['fit_pairs']]
    sign = _pair(envelope['independent_pair'], sample, checked['motion_basis_model'], a_source=a_source)
    used_records = {value['binding']['record_sha256'] for pair in fitted for value in pair[2:]}
    used_captures = {value['capture_identity'] for pair in fitted for value in pair[2:]}
    used_ids = [value['binding']['observation_identity'] for pair in fitted for value in pair[2:]]
    if (sign[3]['binding']['record_sha256'] in used_records or sign[3]['capture_identity'] in used_captures
            or sign[3]['binding']['observation_identity'] in used_ids):
        raise ValueError('FIT_INDEPENDENT_SIGN_OBSERVATION_REQUIRED')
    expected_sources = dict(reference=context['reference_evidence_sha256'],
        fit_motion=[pair['motion']['sha256'] for pair in envelope['fit_pairs']],
        independent_motion=envelope['independent_pair']['motion']['sha256'])
    if checked['source_sha256'] != expected_sources:
        raise ValueError('JACOBIAN_EVIDENCE_SOURCE_MISMATCH')
    estimate = estimate_jacobian([[p[0]] for p in fitted], [[p[1]] for p in fitted],
        feature_axes=['u'], motion_basis_model=checked['motion_basis_model'], context=context,
        feature_definitions=FEATURES, source_sha256=expected_sources, source=source, comparable_context=True)
    matrix = estimate['jacobian']['matrix_px_per_mm']
    if not np.array_equal(np.asarray(matrix), np.asarray(checked['matrix_px_per_mm'])):
        raise ValueError('JACOBIAN_FIT_SOURCE_MISMATCH')
    predicted = checked['matrix_px_per_mm'][0][0] * sign[0]
    if not np.isfinite(predicted) or predicted == 0 or sign[1] == 0 or np.sign(predicted) != np.sign(sign[1]):
        raise ValueError('INDEPENDENT_SIGN_UNVERIFIED')
    return checked


def TargetImageServoProposal(current_snapshot, reference, jacobian_evidence=None, *, a_source=None):
    """Compare two independently bound images; verified external facts remain a trust boundary."""
    result = dict(schema='sweepick.b20.target-image-consumer/1', type='TargetImageServoProposal',
        reference_type=reference.get('role') if isinstance(reference, dict) else None,
        validity='NEEDS_EVIDENCE', missing_evidence=[],
        feature_error_px=dict(u=None, v=None), relative_feature_error_px=dict(u=None, v=None),
        observed_image_axes=[], observed_control_dimensions=0, estimated_tool_correction=None,
        REAL_J_VALIDATED=False, model_only=True, executable=False, motor_write_authorized=False)
    try:
        if not isinstance(reference, dict):
            raise ValueError('REFERENCE_DOCUMENT_REQUIRED')
        if reference.get('role') != FIELD_TARGET or not isinstance(reference.get('reference_id'), str) or not reference['reference_id'].strip():
            raise ValueError('INDEPENDENT_FIELD_TARGET_REFERENCE_REQUIRED')
        target = _sample(reference['snapshot'], role=FIELD_TARGET, a_source=a_source)
        current = _sample(current_snapshot, role=CURRENT_TARGET, a_source=a_source)
        _same_context(target, current)
        context = dict(reference_id=reference['reference_id'], reference_binding=copy.deepcopy(target['binding']),
            reference_evidence_sha256=target['binding']['evidence_sha256'], feature_definition=FEATURES['u'],
            comparison_context=copy.deepcopy(target['context']))
        error = current['u'] - target['u']
        result.update(validity='TARGET_IMAGE_ERROR_ONLY', model_only=current['observation']['source_kind'] != 'REAL',
            source_kind=current['observation']['source_kind'], feature_error_px=dict(u=error, v=None),
            relative_feature_error_px=dict(u=error, v=None), observed_image_axes=['u'],
            reference_binding=target['binding'], current_binding=current['binding'], comparison_context=context,
            reference_centroid_u=target['u'], current_centroid_u=current['u'],
            graph_requested=target['graph_requested'] or current['graph_requested'])
        if jacobian_evidence is None:
            raise ValueError('VALIDATED_REAL_JACOBIAN_AND_INDEPENDENT_SIGN_REQUIRED')
        checked = _validated_jacobian(jacobian_evidence, current, context, a_source=a_source)
        if current['observation']['source_kind'] != 'REAL':
            raise ValueError('TEST_FIXTURE_IS_NOT_A_REAL_CORRECTION')
        binding = observation_binding(current['observation'], camera_key='right_wrist')
        target_features = dict(values_px=dict(u=0.0, v=None), source_kind='REAL', context=context,
                               feature_definitions=FEATURES)
        features = dict(binding, source_kind='REAL', feature_definitions=FEATURES,
            image_identity=binding['camera_state_binding']['image_identity'], target=target_features,
            feature_error_px=result['feature_error_px'], observed_image_axes=['u'], missing_evidence=[])
        proposal = propose_servo(current['observation'], features, target_features, checked, camera_key='right_wrist')
        result.update(validity=proposal['validity'], observed_control_dimensions=proposal['observed_control_dimensions'],
            estimated_tool_correction=proposal['estimated_tool_correction'],
            REAL_J_VALIDATED=proposal['estimated_tool_correction'] is not None,
            missing_evidence=proposal['missing_evidence'])
    except (KeyError, ValueError, TypeError, OverflowError, cv2.error, np.linalg.LinAlgError) as exc:
        result['missing_evidence'].append(str(exc))
    return result
