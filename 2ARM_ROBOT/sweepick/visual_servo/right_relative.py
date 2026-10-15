"""RECEIVE pixel evidence, separate from PICK Jaw features and model pad projections."""
import copy

import numpy as np

from .features import ROOT,observation_binding,sha256,vision_functions
from .servo_proposal import propose_servo

RIGHT_FEATURE_DEFINITIONS=dict(u='RIGHT_HELD_COMPONENT_MINUS_OBSERVED_PAD_TARGET_U',
                               v='RIGHT_HELD_COMPONENT_MINUS_OBSERVED_PAD_TARGET_V')


def extract_right_features(observation,held_object_evidence,right_pad_reference,*,a_source=None):
    binding=observation_binding(observation,camera_key='right_wrist')
    view=observation['views']['right_wrist'];image=np.asarray(view.get('image_bgr'))
    if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3 or not image.size or not view.get('image_identity'):
        raise ValueError('right wrist needs an identified uint8 BGR frame')
    source=ROOT/'native/revision' if a_source is None else a_source
    from pathlib import Path
    source=Path(source)
    _,geometry,functions=vision_functions(source)
    surface=functions.wrist_object_surfaces(image)
    missing=[];values=dict(u=None,v=None);graph=None
    if surface['visible']:
        v,u=np.nonzero(surface['_mask']);values=dict(u=float(u.mean()),v=float(v.mean()))
        try:
            points,edges,_,_,diagnostics=geometry.observed_face_graph(image,surface['_mask'])
            graph=dict(junctions_uv=points.tolist(),edges=edges,diagnostics=diagnostics)
        except (ValueError,RuntimeError) as exc:
            missing.append('RIGHT_FACE_GRAPH_UNAVAILABLE: '+str(exc))
    else:missing.append('RIGHT_OBJECT_FEATURE_UNOBSERVED')
    kind=observation['source_kind'];held=held_object_evidence or {};ref=copy.deepcopy(right_pad_reference or {})
    camera=view.get('recorded_camera') or {}
    current=(kind!='REAL' or camera.get('fresh') is True and camera.get('reader_valid') is True)
    if not current:missing.append('CURRENT_RIGHT_IMAGE_FRESHNESS_UNVERIFIED')
    def bound(evidence):
        return (evidence.get('source_kind')==kind and bool(evidence.get('source_sha256')) and
                all(evidence.get(k)==binding[k] for k in
                    ('observation_identity','feedback_sha256','camera_state_binding')))
    held_usable=bound(held) and held.get('held_by_left') is True and held.get('target_association_verified') is True
    if not held_usable:missing.append('CURRENT_LEFT_HELD_TARGET_ASSOCIATION_UNVERIFIED')
    measured_ref=(ref.get('role')=='CURRENT_OBSERVED_PAD' and
                  ref.get('physical_correspondence_verified') is True)
    model_ref=kind=='MODEL' and ref.get('role')=='MODEL_PRIOR'
    reference_usable=bound(ref) and ref.get('camera_key')=='right_wrist' and (measured_ref or model_ref)
    if not reference_usable:missing.append('RIGHT_PAD_REFERENCE_UNOBSERVED_OR_UNIDENTIFIED')
    errors=dict(u=None,v=None);axes=[]
    for axis in ('u','v'):
        target=ref.get('values_px',{}).get(axis)
        if target is not None and (isinstance(target,bool) or not np.isscalar(target) or not np.isfinite(target)):
            raise ValueError('finite right Pad target pixels required')
        if (current and held_usable and reference_usable and values[axis] is not None and target is not None and
                axis in ref.get('observed_axes',[]) and
                ref.get('feature_definitions',{}).get(axis)==RIGHT_FEATURE_DEFINITIONS[axis]):
            errors[axis]=values[axis]-float(target);axes.append(axis)
        else:missing.append('RIGHT_'+axis.upper()+'_REFERENCE_UNOBSERVED')
    model=observation.get('model_assisted_relative_geometry') or {}
    return dict(schema='sweepick.b19.right-image-features/1',**binding,source_kind=kind,
        camera_key='right_wrist',image_identity=copy.deepcopy(view['image_identity']),
        feature_role='RIGHT_RECEIVE',feature_definitions=copy.deepcopy(RIGHT_FEATURE_DEFINITIONS),
        coordinate_frame='right wrist raw pixels; u right, v down',units='pixel',values_px=errors,
        object_component_centroid_uv=values,
        feature_error_px=errors,observed_image_axes=axes,
        target=dict(values_px={a:0.0 if a in axes else None for a in ('u','v')},
            source_kind=kind,camera_key='right_wrist',context=copy.deepcopy(ref.get('context')),
            feature_definitions=copy.deepcopy(RIGHT_FEATURE_DEFINITIONS),
            role='explicit observed Pad/task correspondence; zero relative error, never image centre'),
        visible_approach_region=dict(visible=surface['visible'],status=surface['status'],
            contour_uv=surface['contour_points_px'],graph=graph,
            component_area_px=surface.get('component_area_px'),
            physical_face_identity_verified=False,complete_object_centre_observed=False,
            identification_prior=surface['colour_role']),
        grasp_region=dict(candidate_contour_uv=surface['contour_points_px'],
            physically_graspable_region_verified=False,hidden_surface_filled=False),
        visibility=dict(frame_clipped=surface['frame_clipped'],clipped_boundary=surface['clipped_boundary'],
            physical_occlusion_identity='UNOBSERVED'),
        reference_provenance=dict(held_target_bound=held_usable,pad_reference_bound=reference_usable,
            measured_pad_reference=bool(reference_usable and measured_ref),
            pad_reference=ref,object_target_association=copy.deepcopy(held)),
        model_projection_diagnostic=dict(status=model.get('status','UNAVAILABLE'),
            classification='MODEL_ASSISTED_DIAGNOSTIC_ONLY',
            fit_diagnostics=copy.deepcopy(model.get('fit_diagnostics')),
            right_pad_model_prior=copy.deepcopy(model.get('right_pad_model_prior')),
            actual_pad_pixel_residual_px=None,physical_pad_correspondence_verified=False,
            fitted_model_residual_is_physical_accuracy=False),
        source_sha256=dict(image=view.get('source_sha256'),
            image_array=binding['camera_state_binding']['image_array_sha256'],
            A_features=sha256(source/'sweepick_pick01.py'),
            A_relative_geometry=sha256(source/'right_wrist_relative_geometry.py')),
        missing_evidence=list(dict.fromkeys(missing)),executable=False,motor_write_authorized=False)


def RightRelativeServoProposal(actual_q12,right_wrist_observation,held_object_evidence,
                               right_pad_reference,jacobian_evidence,*,a_source=None):
    """Return an unapproved translation only; A owns actual-pose IK/path/collision checks."""
    observation=right_wrist_observation
    actual=np.asarray(actual_q12,float);recorded=np.asarray(observation.get('q12'),float)
    if actual.shape!=(12,) or not np.isfinite(actual).all() or not np.array_equal(actual,recorded):
        raise ValueError('actual q12 must exactly bind this right-camera observation')
    features=extract_right_features(observation,held_object_evidence,right_pad_reference,a_source=a_source)
    jacobian=jacobian_evidence
    if observation['source_kind']=='REAL' and (jacobian is None or jacobian.get('source')!='REAL_MEASURED'
                                               or jacobian.get('validated') is not True):
        jacobian=None
        features['missing_evidence'].append('VALIDATED_REAL_RIGHT_JACOBIAN_UNAVAILABLE')
    proposal=propose_servo(observation,features,features['target'],jacobian,camera_key='right_wrist')
    proposal.update(schema='sweepick.b19.right-relative-servo-proposal/1',type='RightRelativeServoProposal',
        relative_feature_error_px=features['feature_error_px'],
        reference_provenance=features['reference_provenance'],feature_evidence=features,
        actual_q12_for_A_IK=actual.tolist(),A_final_execution_authority=True)
    return proposal
