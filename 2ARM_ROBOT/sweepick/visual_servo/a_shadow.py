"""Pure adapter for an existing A decision-observation record and its saved RGB arrays."""
import copy

import numpy as np

from .features import observation_binding
from .right_relative import RightRelativeServoProposal


def proposal_from_A_record(document,images_bgr,*,record_identity,record_sha256,image_sha256,
                           right_pad_reference=None,jacobian_evidence=None,a_source=None,
                           include_geometry=False):
    """Callers verify saved files; extraction reads pure source, never devices or executors."""
    native=document['observation'];feedback=document['actual_feedback']
    q=np.asarray(document['q12'],float)
    if (document.get('source_kind')!='REAL' or q.shape!=(12,) or not np.isfinite(q).all()
            or not np.array_equal(q,np.asarray(native.get('q12'),float))
            or feedback.get('usable') is not True
            or not isinstance(feedback.get('ticks'),dict) or not feedback['ticks']
            or feedback['ticks']!=native.get('present_ticks')):
        raise ValueError('A record needs bound usable REAL actual feedback; no commanded-state substitution')
    keys={'top','left_wrist','right_wrist'}
    if set(images_bgr)!=keys or set(image_sha256)!=keys or not record_sha256:
        raise ValueError('three identified recorded images and record source required')
    clock=record_identity['run_id']+':HOST_MONOTONIC'
    unknown=dict(value=None,unit='s',clock=clock,measurement_time=None)
    views={};sensors=[]
    for key in ('top','left_wrist','right_wrist'):
        image=np.asarray(images_bgr[key]);camera=document['camera_frames'][key]
        if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3 or not image.size or not image_sha256[key]:
            raise ValueError('identified uint8 BGR recorded views required')
        views[key]=dict(image_bgr=image,image_identity=dict(stream=key,n=camera['n'],
            file=document['files'][key]['rgb']),source_kind='REAL',source_sha256=image_sha256[key],
            observation_identity=copy.deepcopy(record_identity),recorded_camera=copy.deepcopy(camera))
        sides=('left',) if key=='left_wrist' else ('right',) if key=='right_wrist' else ('left','right')
        sensors.append(dict(view=key,value=camera.get('recv_monotonic_s'),unit='s',clock=clock,
            kind='HOST_CAMERA_RECEIPT_NOT_EXPOSURE',state_references={s:copy.deepcopy(unknown) for s in sides},
            native_exposure_known=False))
    observation=dict(observation_identity=copy.deepcopy(record_identity),source_kind='REAL',q12=q.tolist(),views=views,
        reference_times=dict(joint_state={s:copy.deepcopy(unknown) for s in ('left','right')},sensors=sensors,
            packet_receipt=dict(value=feedback.get('t_bus_monotonic'),unit='s',clock=clock)),
        model_assisted_relative_geometry=copy.deepcopy(native.get('model_assisted_relative_geometry')))
    evidence=native.get('object_evidence') or {}
    held=dict(**observation_binding(observation,camera_key='right_wrist'),source_kind='REAL',source_sha256=record_sha256,
        held_by_left=True if evidence.get('evidence_valid') is True else None,
        target_association_verified=evidence.get('evidence_valid') is True,
        native_left_powered_hold_context=bool((native.get('left') or {}).get('torque') and native.get('held_object')),
        recorded_native_held=copy.deepcopy(native.get('held_object')),recorded_object_evidence=copy.deepcopy(evidence))
    result=RightRelativeServoProposal(q,observation,held,right_pad_reference,jacobian_evidence,
        a_source=a_source,include_geometry=include_geometry)
    result['A_record_provenance']=dict(record_sha256=record_sha256,image_sha256=copy.deepcopy(image_sha256),
        command_fields_consumed=False,actual_feedback_usable=True,
        right_gripper_measured_tick=feedback['ticks'].get('right',{}).get('gripper'),
        native_stage=document.get('stage'),native_seq=native.get('seq'))
    return result
