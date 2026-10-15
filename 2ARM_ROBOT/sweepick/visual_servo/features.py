"""Left-wrist image measurements; target references must have their own evidence."""
import ast
import copy
import hashlib
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT/'goal_bridge') not in sys.path:
    sys.path.insert(0,str(ROOT/'goal_bridge'))
from b16_goal_bridge import temporal_alignment, sha256
from stored_replay import vision_functions

FEATURE_DEFINITIONS = dict(u='A_WIDEST_ROW_OBJECT_MIDPOINT_X',
                           v='VISIBLE_COMPONENT_CENTROID_Y')


def a_image_functions(source=None):
    source = Path(source) if source is not None else ROOT/'native/revision'
    _,_,functions = vision_functions(source)
    names = {'wrist_object','jaw_tips','jaw_reference','wrist_offset','wrist_features'}
    tree = ast.parse((source/'sweepick_pick01.py').read_text())
    nodes = [n for n in tree.body if
             isinstance(n,ast.FunctionDef) and n.name in names or
             isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='WRIST' for t in n.targets)]
    if {n.name for n in nodes if isinstance(n,ast.FunctionDef)} != names:
        raise ValueError('existing A image function contract changed')
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(source/'sweepick_pick01.py'),'exec'),functions.__dict__)
    return functions,sha256(source/'sweepick_pick01.py')


def observation_binding(observation):
    identity = observation.get('observation_identity')
    if (not isinstance(identity,dict) or any(not isinstance(identity.get(k),str) or not identity[k]
                                           for k in ('run_id','observation_id'))):
        raise ValueError('run_id and observation_id are required')
    q = np.asarray(observation.get('q12'),float)
    if q.shape != (12,) or not np.isfinite(q).all():
        raise ValueError('finite measured canonical q12 is required')
    views = observation.get('views')
    if not isinstance(views,dict) or set(views) != {'top','left_wrist','right_wrist'}:
        raise ValueError('three view entries are required; unavailable views must be explicit')
    if observation.get('source_kind') not in ('REAL','MODEL','TEST_FIXTURE'):
        raise ValueError('explicit observation source kind required')
    for key,view in views.items():
        if view.get('source_kind') != observation['source_kind']:
            raise ValueError('camera/observation source mismatch')
        if view.get('observation_identity') != identity:
            raise ValueError(f'{key} camera/state observation identity mismatch')
    left=views['left_wrist'];image=np.asarray(left.get('image_bgr'))
    return dict(observation_identity=copy.deepcopy(identity),
        feedback_sha256=hashlib.sha256(q.astype('<f8').tobytes()).hexdigest(),
        camera_state_binding=dict(source_kind=observation['source_kind'],
            image_identity=copy.deepcopy(left.get('image_identity')),source_sha256=left.get('source_sha256'),
            image_shape=list(image.shape),image_dtype=str(image.dtype),
            image_array_sha256=hashlib.sha256(image.tobytes()).hexdigest()
                if image.dtype==np.uint8 and image.ndim==3 else None),
        temporal_alignment=temporal_alignment(observation.get('reference_times') or {}))


def extract_features(observation, *, jaw_reference=None, target=None, previous=None, a_source=None):
    """Keep legacy X and observed component Y distinct; a bottom-row jaw X is not a Y target."""
    binding = observation_binding(observation)
    view = observation['views']['left_wrist'];image=np.asarray(view.get('image_bgr'))
    if (image.ndim!=3 or image.shape[2]!=3 or image.dtype!=np.uint8 or not image.size):
        raise ValueError('left wrist needs a saved uint8 BGR frame')
    if not view.get('image_identity'):
        raise ValueError('left wrist image identity required')
    a,source = a_image_functions(a_source)
    surface = a.wrist_object_surfaces(image);obj=a.wrist_object(image);tips=a.jaw_tips(image)
    missing=[];values=dict(u=None,v=None);centroid=None;contour=[]
    if surface.get('visible') and obj.get('visible'):
        v,u=np.nonzero(surface['_mask']);centroid=[float(u.mean()),float(v.mean())]
        values=dict(u=float(obj['centre_px']),v=centroid[1]);contour=surface['contour_points_px']
    else:
        missing.append('OBJECT_FEATURE_UNOBSERVED')
    target=copy.deepcopy(target or {});reference=copy.deepcopy(jaw_reference or {})
    offset=a.wrist_offset(obj,reference,tips,gripper_tick=view.get('gripper_tick'))
    reference_bound=(reference.get('camera_key')=='left_wrist' and bool(reference.get('source_sha256'))
                     and reference.get('source_kind')==observation['source_kind'])
    observed=[];errors=dict(u=None,v=None)
    definitions=target.get('feature_definitions',{})
    for axis in ('u','v'):
        value=target.get('values_px',{}).get(axis)
        target_bound=(target.get('camera_key')=='left_wrist' and bool(target.get('source_sha256'))
                      and target.get('source_kind')==observation['source_kind']
                      and definitions.get(axis)==FEATURE_DEFINITIONS[axis]
                      and axis in target.get('observed_axes',[]))
        if axis=='u':
            target_bound=target_bound and reference_bound and offset.get('reference_confirmed_in_this_frame') is True
        if value is not None and (isinstance(value,bool) or not np.isfinite(value)):
            raise ValueError('target pixels must be finite')
        if target_bound and value is not None and values[axis] is not None:
            errors[axis]=values[axis]-float(value);observed.append(axis)
        else:
            missing.append('X_REFERENCE_UNOBSERVED' if axis=='u' else 'Y_REFERENCE_UNOBSERVED')
    burst=view.get('frame_burst_bgr')
    stability=dict(status='UNMEASURED',source='one saved frame does not establish detection stability')
    if burst is not None:
        if not isinstance(burst,list) or not burst:
            raise ValueError('nonempty recorded frame burst required')
        legacy=a.wrist_features(burst,ids=view.get('frame_burst_identities'))
        stability=dict(status='STABLE' if legacy['object'].get('stable') else 'UNSTABLE',
            frame_count=len(burst),source='unchanged A wrist_features criterion; image diagnostic only',
            object=legacy['object'])
    comparison=None
    if previous is not None:
        if previous.get('image_identity')==view['image_identity']:
            missing.append('NO_NEW_IMAGE')
        comparison={axis:None if previous['values_px'].get(axis) is None or values[axis] is None else
                    values[axis]-previous['values_px'][axis] for axis in ('u','v')}
    return dict(schema='sweepick.b18.image-features/1',**binding,
        source_kind=observation['source_kind'],camera_key='left_wrist',image_identity=copy.deepcopy(view['image_identity']),
        coordinate_frame='left wrist raw image pixels: u right, v down',units='pixel',
        source_sha256=dict(image_file=view.get('source_sha256'),
            image_array=hashlib.sha256(image.tobytes()).hexdigest(),A_feature_source=source),
        feature_definitions=copy.deepcopy(FEATURE_DEFINITIONS),values_px=values,
        object=dict(observed_component_centroid_uv=centroid,centroid_is_complete_object_centre=False,
            contour_uv=contour,visible=bool(surface.get('visible')),frame_clipped=surface.get('frame_clipped'),
            component_area_px=surface.get('component_area_px'),physical_edge_identity_verified=False,
            identification_prior=surface['colour_role']),
        jaw=dict(current_tips_x_px=tips,current_bright_background_candidate=a.jaw_reference(image),
            reference=reference,reference_bound=reference_bound,offset=offset,
            y_reference_observed='v' in observed,bottom_row_is_task_y_reference=False),
        target=target,feature_error_px=errors,observed_image_axes=observed,
        visibility_and_occlusion=dict(frame_clipped=surface.get('frame_clipped'),
            hidden_surface_identity='UNOBSERVED',jaw_tips_visible={k:v is not None for k,v in tips.items()}),
        stability=stability,frame_to_frame_feature_change_px=comparison,
        view_roles=dict(top='object/worktable spatial context only',left_wrist='left proximity alignment',
                        right_wrist='retained as a separate input; no verified left control contribution'),
        view_provenance={k:{key:value for key,value in data.items() if key not in ('image_bgr','frame_burst_bgr')}
                         for k,data in observation['views'].items()},
        missing_evidence=list(dict.fromkeys(missing)),executable=False,motor_write_authorized=False)
