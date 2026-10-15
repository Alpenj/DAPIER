"""Unapproved tool-translation calculations; A retains stage/IK/collision/write authority."""
import copy

import numpy as np

from .features import observation_binding
from .image_jacobian import jacobian_record,solve_correction


def propose_servo(observation,features,target_features,jacobian,*,previous_features=None,
                  previous_response=None,damping=0.0):
    result=dict(schema='sweepick.b18.visual-servo-proposal/1',type='VisualServoProposal',
        observation_identity=copy.deepcopy(observation.get('observation_identity')),
        feature_error_px=dict(u=None,v=None),observed_control_dimensions=0,
        jacobian=None,jacobian_source=None if jacobian is None else jacobian.get('source'),
        estimated_tool_correction=None,expected_image_error_change=None,missing_evidence=[],
        validity='NEEDS_EVIDENCE',source_sha256=copy.deepcopy(features.get('source_sha256',{})),
        model_only=True,executable=False,motor_write_authorized=False)
    try:
        binding=observation_binding(observation)
    except (ValueError,TypeError) as exc:
        result.update(validity='INVALID_OBSERVATION',missing_evidence=[str(exc)]);return result
    result['temporal_alignment']=binding['temporal_alignment']
    if (features.get('observation_identity')!=binding['observation_identity']
            or features.get('feedback_sha256')!=binding['feedback_sha256']
            or features.get('camera_state_binding')!=binding['camera_state_binding']
            or features.get('image_identity')!=binding['camera_state_binding']['image_identity']
            or features.get('source_kind')!=observation['source_kind']):
        result.update(validity='INVALID_OBSERVATION',missing_evidence=['FEATURE_CAMERA_STATE_MISMATCH']);return result
    result['feature_error_px']=copy.deepcopy(features.get('feature_error_px',{}))
    result['observed_image_axes']=copy.deepcopy(features.get('observed_image_axes',[]))
    missing=list(features.get('missing_evidence',[]))
    if previous_response is not None and previous_response.get('jacobian_revalidation_required'):
        result['missing_evidence']=list(dict.fromkeys(missing+[previous_response['status']]));return result
    if target_features!=features.get('target'):
        missing.append('TARGET_FEATURES_CHANGED_SINCE_EXTRACTION')
    if previous_features is not None:
        if (previous_features.get('image_identity')==features.get('image_identity')
                or previous_features.get('observation_identity')==features.get('observation_identity')):
            missing.append('NO_NEW_IMAGE')
    if 'NO_NEW_IMAGE' in missing or 'TARGET_FEATURES_CHANGED_SINCE_EXTRACTION' in missing:
        result['missing_evidence']=list(dict.fromkeys(missing));return result
    if jacobian is None:
        result['missing_evidence']=list(dict.fromkeys(missing+['JACOBIAN_UNIDENTIFIED']));return result
    try:
        jacobian=jacobian_record(jacobian['matrix_px_per_mm'],**{k:jacobian[k] for k in
            ('feature_axes','motion_basis_model','source','context','feature_definitions','source_sha256')},
            validated=jacobian.get('validated') is True)
    except (KeyError,ValueError,TypeError,np.linalg.LinAlgError):
        result.update(validity='INVALID_JACOBIAN',missing_evidence=missing+['MALFORMED_JACOBIAN']);return result
    result['jacobian']=jacobian
    source=jacobian.get('source');result['model_only']=source!='REAL_MEASURED'
    if source=='REAL_MEASURED' and jacobian.get('validated') is not True:
        result['missing_evidence']=list(dict.fromkeys(missing+['REAL_JACOBIAN_VALIDATION_UNVERIFIED']));return result
    kind=observation['source_kind']
    if (target_features.get('source_kind')!=kind
            or source=='REAL_MEASURED' and kind!='REAL'
            or (source=='TEST_FIXTURE')!=(kind=='TEST_FIXTURE')):
        result.update(model_only=True,missing_evidence=missing+['JACOBIAN_SOURCE_MISMATCH']);return result
    if jacobian.get('context')!=target_features.get('context'):
        result['missing_evidence']=list(dict.fromkeys(missing+['JACOBIAN_CONTEXT_MISMATCH']));return result
    axes=[a for a in jacobian['feature_axes'] if a in features.get('observed_image_axes',[])]
    if not axes:
        result['missing_evidence']=list(dict.fromkeys(missing+['OBSERVED_ERROR_UNAVAILABLE']));return result
    if any(jacobian['feature_definitions'].get(a)!=features['feature_definitions'].get(a) for a in axes):
        result['missing_evidence']=list(dict.fromkeys(missing+['FEATURE_DEFINITION_MISMATCH']));return result
    errors={a:features['feature_error_px'][a] for a in axes}
    matrix=np.asarray(jacobian['matrix_px_per_mm'],float)[[jacobian['feature_axes'].index(a) for a in axes]]
    try:
        with np.errstate(over='raise',invalid='raise'):
            calculation=solve_correction(matrix,[errors[a] for a in axes],damping=damping)
    except (ValueError,TypeError,FloatingPointError,np.linalg.LinAlgError):
        result.update(validity='INVALID_CALCULATION',feature_error_px=dict(u=None,v=None),
            missing_evidence=missing+['NONFINITE_OR_MALFORMED_CALCULATION']);return result
    result.update(feature_error_px={a:errors.get(a) for a in ('u','v')},
                  observed_control_dimensions=calculation['control_dimensions'],calculation=calculation)
    if calculation['control_dimensions']==0:
        result['missing_evidence']=list(dict.fromkeys(missing+['JACOBIAN_NUMERICALLY_UNOBSERVABLE']));return result
    basis=np.asarray(jacobian['motion_basis_model'])@np.asarray(calculation['control_subspace_coordinates'])
    coefficient=np.asarray(calculation['coefficients_mm'])
    with np.errstate(over='ignore',invalid='ignore'):
        delta=basis@coefficient/1000
    if not np.isfinite(delta).all() or not np.isfinite(calculation['expected_image_change_px']).all():
        result.update(validity='INVALID_CALCULATION',observed_control_dimensions=0,
            missing_evidence=missing+['NONFINITE_OR_MALFORMED_CALCULATION']);return result
    result.update(validity='MODEL_ONLY_PROPOSAL' if result['model_only'] else 'SHADOW_PROPOSAL',
        estimated_tool_correction=dict(coordinates_mm=coefficient.tolist(),basis_model=basis.tolist(),
            delta_model_m=delta.tolist(),
            coordinate_frame='assembled dual model world; orthonormal columns define independent controls',
            joint_target_generated=False),
        expected_image_error_change={a:float(v) for a,v in zip(axes,calculation['expected_image_change_px'])},
        missing_evidence=list(dict.fromkeys(missing)))
    return result


def evaluate_response(previous_features,current_features,proposal,actual_control_displacement_mm):
    """Check measured response direction; returns evidence, never commands a stop or retry."""
    if previous_features.get('image_identity')==current_features.get('image_identity'):
        return dict(status='NO_NEW_IMAGE',jacobian_revalidation_required=True)
    if previous_features.get('target')!=current_features.get('target'):
        return dict(status='TARGET_CHANGED_NOT_COMPARABLE',jacobian_revalidation_required=True)
    if proposal.get('estimated_tool_correction') is None:
        return dict(status='NO_PREVIOUS_CORRECTION',jacobian_revalidation_required=True)
    j=proposal['jacobian'];axes=[a for a in j['feature_axes'] if
        previous_features['values_px'].get(a) is not None and current_features['values_px'].get(a) is not None]
    actual=np.asarray(actual_control_displacement_mm,float)
    matrix=np.asarray(j['matrix_px_per_mm'],float)
    if actual.shape!=(matrix.shape[1],) or not np.isfinite(actual).all():
        raise ValueError('actual displacement must use the recorded Jacobian input basis')
    indices=[j['feature_axes'].index(a) for a in axes]
    predicted=matrix[indices]@actual
    observed=np.array([current_features['values_px'][a]-previous_features['values_px'][a] for a in axes])
    dot=float(predicted@observed)
    if not np.isfinite(predicted).all() or not np.isfinite(observed).all() or not np.isfinite(dot):
        return dict(status='INVALID_IMAGE_RESPONSE',jacobian_revalidation_required=True,
                    physical_stop_requested=False,physical_success=False)
    return dict(status='REVERSED_IMAGE_RESPONSE' if dot<0 else 'DIRECTION_CONSISTENT' if dot>0 else 'DIRECTION_UNRESOLVED',
        feature_axes=axes,predicted_image_change_px=predicted.tolist(),observed_image_change_px=observed.tolist(),
        direction_dot_px_squared=dot,jacobian_revalidation_required=dot<=0,
        physical_stop_requested=False,physical_success=False)
