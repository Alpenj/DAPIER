"""Raw-image diagnostics only: observed pixels, model projections and unknowns stay separate."""
import ast
import copy
import hashlib
from pathlib import Path

import cv2
import numpy as np

from .features import ROOT, a_image_functions, sha256
from .right_relative import _right_functions


def frame_binding(frame):
    if not isinstance(frame, dict) or frame.get('artifact_role') != 'RAW_RGB':
        raise ValueError('RAW_RGB_REQUIRED_OVERLAY_IS_NOT_DETECTOR_INPUT')
    image = np.asarray(frame.get('image_bgr'))
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or not image.size:
        raise ValueError('NONEMPTY_RAW_UINT8_BGR_REQUIRED')
    if not frame.get('image_identity') or not frame.get('source_sha256'):
        raise ValueError('RAW_IMAGE_IDENTITY_AND_SOURCE_REQUIRED')
    if frame.get('camera_key') not in ('left_wrist','right_wrist') or frame.get('source_kind') not in ('REAL','MODEL','TEST_FIXTURE'):
        raise ValueError('EXPLICIT_WRIST_CAMERA_AND_SOURCE_KIND_REQUIRED')
    return dict(image_identity=copy.deepcopy(frame['image_identity']), source_sha256=frame['source_sha256'],
        camera_key=frame['camera_key'],source_kind=frame['source_kind'],
        image_array_sha256=hashlib.sha256(image.tobytes()).hexdigest(), shape=list(image.shape))


def _alignment(a, source, plan, scene):
    tree = ast.parse((source / 'sweepick_pick01.py').read_text())
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'current_wrist_alignment']
    if len(nodes) != 1:
        raise ValueError('A_CURRENT_WRIST_ALIGNMENT_CONTRACT_CHANGED')
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source / 'sweepick_pick01.py'), 'exec'), a.__dict__)
    return a.current_wrist_alignment(plan, scene)


def _points(value):
    points = np.asarray(value, float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('FINITE_UV_POINTS_REQUIRED')
    return points


def _bound(value, binding):
    return isinstance(value, dict) and value.get('binding') == binding


def measure_wrist(frame, *, side, jaw_reference=None, context=None, model_projection=None,
                  edge_evidence=None, next_descent=None, a_source=None):
    """No default image-centre reference; external evidence must bind this raw frame."""
    binding = frame_binding(frame); image = frame['image_bgr']; h, w = image.shape[:2]
    if side not in ('left', 'right'):
        raise ValueError('EXPLICIT_WRIST_SIDE_REQUIRED')
    if frame['camera_key'] != side+'_wrist':
        raise ValueError('WRIST_CAMERA_SIDE_MISMATCH')
    source = Path(a_source) if a_source else ROOT / 'native/revision'
    a, a_sha = a_image_functions(source)
    digests = tuple(sha256(source/n) for n in ('sweepick_pick01.py', 'right_wrist_relative_geometry.py',
                                             'sweepick_261009_offsite_relative_features.py'))
    _, geometry, fast = _right_functions(source.resolve(), digests)
    surface = fast.wrist_object_surfaces(image)
    obj = a.wrist_object(image); tips = a.jaw_tips(image)
    centroid = None
    if surface['visible']:
        vv, uu = np.nonzero(surface['_mask']); centroid = [float(uu.mean()), float(vv.mean())]
    result = dict(schema='sweepick.dual-wrist-overlay-features/1', side=side, binding=binding,
        coordinate_frame='raw wrist pixels: u right, v down', source_sha256=dict(A_image=a_sha, A_geometry=digests[1]),
        object=dict(visible=surface['visible'], status=surface['status'], role='OBSERVED',
            contour_uv=surface['contour_points_px'], visible_edges=surface['visible_edges'],
            clipped_boundary=surface['clipped_boundary'], frame_clipped=surface['frame_clipped'],
            component_centroid_uv=centroid, centroid_is_physical_centre=False,
            widest_row_midpoint_u=obj.get('centre_px'), raw_far_edge_slope_deg=obj.get('far_edge_slope_deg'),
            widest_row_horizontal_clipped=obj.get('touches_border'),
            edge_correspondence_verified=False, occlusion='UNKNOWN', physical_3D_yaw=None),
        gripper=dict(observed_grasp_centre_uv=None, observed_midline_u=None,
            observed_reference_role='UNKNOWN', observed_reference_row=None, closing_line_uv=None,
            closing_line_role='UNKNOWN', model_grasp_centre_uv=None, model_pad_boundaries_uv=[]),
        image_error_px=dict(u=None, v=None), model_relative_image_error_px=dict(u=None, v=None),
        relative_angle=dict(status='UNKNOWN', value_deg=None, role='UNKNOWN', physical_3D_yaw=None,
            reason='unoccluded corresponding edge and gripper direction are not established'),
        legacy_alignment=None, fov=dict(bounds_uv=[0,0,w-1,h-1], current_frame_clipped=surface['frame_clipped'],
            next_descent=dict(status='UNKNOWN', role='UNKNOWN', reason='no bound next-descent geometry', contour_uv=None)),
        graph_requested=False, Correction=None, estimated_tool_correction=None, executable=False, motor_write_authorized=False)
    grip = result['gripper']; offset = None; scene = None
    if side == 'left' and _bound(context, binding):
        ref = jaw_reference or {}
        scene = copy.deepcopy(context.get('scene') or {})
        stable = (scene.get('wrist') or {}).get('object', {}).get('stable')
        measured_obj = dict(obj)
        if stable is not None: measured_obj['stable'] = stable
        offset = a.wrist_offset(measured_obj, ref, tips, gripper_tick=context.get('gripper_tick'))
        reference_bound = (ref.get('camera_key') == 'left_wrist' and bool(ref.get('source_sha256'))
                           and ref.get('source_kind') == frame.get('source_kind')
                           and ref.get('reference_shape_hw') == [h,w])
        if reference_bound and ref.get('readable') is True and offset.get('reference_confirmed_in_this_frame') is True:
            grip.update(observed_midline_u=ref['midline_px'], observed_reference_role='OBSERVED',
                observed_reference_row=h-2, closing_line_uv=[[ref['inner_px'][0],h-2],[ref['inner_px'][1],h-2]],
                closing_line_role='OBSERVED_REFERENCE_TIP_ROW_X_ONLY',
                current_confirmation=offset.get('confirmed_by'), reference_source_sha256=ref['source_sha256'])
            if surface['visible'] and not obj.get('touches_border') and offset.get('readable'):
                result['image_error_px']['u'] = offset['offset_px']
        scene['wrist'] = dict(scene.get('wrist') or {}, object=measured_obj, offset=offset)
        legacy = _alignment(a, source, context.get('plan') or {}, scene)
        result['legacy_alignment'] = {k:legacy.get(k) for k in
            ('ok','edge_slope_deg','square_at_deg','error_deg','image_error_role','physical_rotation_verified')}
    if model_projection is not None:
        if not _bound(model_projection, binding) or model_projection.get('role') != 'MODEL_PRIOR':
            raise ValueError('BOUND_MODEL_PRIOR_REQUIRED')
        grip['model_grasp_centre_uv'] = (_points([model_projection['centre_uv']])[0].tolist()
            if model_projection.get('centre_uv') is not None else None)
        grip['model_pad_boundaries_uv'] = [_points(p).tolist() for p in model_projection.get('pad_boundaries_uv',[])]
        grip['model_closing_line_uv'] = (_points(model_projection['closing_line_uv']).tolist()
            if model_projection.get('closing_line_uv') is not None else None)
        grip['model_source'] = copy.deepcopy(model_projection.get('source'))
        if centroid and grip['model_grasp_centre_uv'] and not surface['frame_clipped']:
            result['model_relative_image_error_px'] = dict(zip(('u','v'),
                (np.asarray(centroid)-grip['model_grasp_centre_uv']).tolist()))
    if (surface['visible'] and not surface['frame_clipped'] and _bound(edge_evidence,binding)
            and edge_evidence.get('unoccluded') is True and edge_evidence.get('correspondence_verified') is True):
        points = _points(edge_evidence['points_uv'])
        visible=np.zeros((h,w),np.uint8)
        for run in surface['visible_edges']:
            for x,y in run['points_px']:visible[y,x]=1
        contour={tuple(p) for p in surface['contour_points_px']}
        segment=np.zeros((h,w),np.uint8)
        valid=(len(points)>=2 and not np.array_equal(points[0],points[-1]) and all(tuple(p) in contour for p in points))
        if valid:
            cv2.line(segment,tuple(points[0].astype(int)),tuple(points[-1].astype(int)),1,1,cv2.LINE_8)
            valid=not np.any((segment!=0)&(visible==0)) and all(segment[int(p[1]),int(p[0])] for p in points)
        if not valid:
            result['relative_angle']['reason']='EDGE_IS_NOT_ONE_STRAIGHT_VISIBLE_CONTOUR_SEGMENT'
        else:
            slope = float(np.degrees(np.arctan2(*(points[-1]-points[0])[::-1])))
            line = grip.get('model_closing_line_uv'); ref_angle = None; role = 'UNKNOWN'
            if grip['observed_midline_u'] is not None and result['legacy_alignment'] is not None:
                if result['legacy_alignment'].get('ok') is True:
                    ref_angle=result['legacy_alignment']['square_at_deg']; role='OBSERVED_EDGE_VS_EXISTING_IMAGE_REFERENCE'
            elif line is not None:
                dv=np.asarray(line[1])-line[0]
                if np.linalg.norm(dv)>0:
                    ref_angle=float(np.degrees(np.arctan2(dv[1],dv[0])));role='OBSERVED_EDGE_VS_MODEL_PRIOR_DIRECTION'
            if ref_angle is not None:
                result['object'].update(edge_correspondence_verified=True,occlusion='VERIFIED_UNOCCLUDED_FOR_THIS_EDGE',valid_reference_edge_uv=points.tolist())
                result['relative_angle']=dict(status='AVAILABLE_IMAGE_ONLY',value_deg=float((slope-ref_angle+90)%180-90),
                    role=role,reference_angle_deg=ref_angle,edge_angle_deg=slope,physical_3D_yaw=None)
    if next_descent is not None:
        if not _bound(next_descent,binding) or next_descent.get('role')!='MODEL_PRIOR':
            raise ValueError('BOUND_NEXT_DESCENT_MODEL_PRIOR_REQUIRED')
        prediction=dict(status='UNKNOWN',role='MODEL_PRIOR',contour_uv=None,physical_validated=False,
            assumptions='visible top-envelope candidates on existing MODEL_PRIOR plane; candidate camera/mapping; nominal next endpoint',
            forecast_scope='VISIBLE_TOP_ENVELOPE_ONLY_NOT_WHOLE_OBJECT_OR_OCCLUSION',
            input_role=next_descent.get('input_role'),source=copy.deepcopy(next_descent.get('source')))
        if surface['visible']:
            try:
                mask=surface['_mask'];columns=np.flatnonzero(mask.any(axis=0))
                envelope=np.array([[x,np.flatnonzero(mask[:,x])[0]] for x in columns],float)
                envelope=envelope[(envelope[:,0]>0)&(envelope[:,0]<w-1)&(envelope[:,1]>0)&(envelope[:,1]<h-1)]
                if not len(envelope):raise ValueError('NO_VISIBLE_TOP_ENVELOPE')
                K=np.asarray(next_descent['K'],float);D=np.asarray(next_descent['D'],float)
                points,_=geometry.lift_pixels_to_plane(envelope,K,D,
                    next_descent['T_world_camera'],next_descent['plane_centre'],next_descent['plane_normal'])
                uv=geometry.project_points(points,K,D,next_descent['T_world_camera_next'])
                uv=_points(uv); outside=(uv[:,0]<0)|(uv[:,0]>w-1)|(uv[:,1]<0)|(uv[:,1]>h-1)
                prediction.update(status='MODEL_PRIOR_ENVELOPE_EXIT' if outside.any() else 'MODEL_PRIOR_ENVELOPE_INSIDE',
                    contour_uv=uv.tolist(),outside_fraction=float(outside.mean()),
                    bbox_uv=[uv.min(0).tolist(),uv.max(0).tolist()],full_object_FOV='UNKNOWN',valid_for_control=False)
            except (KeyError,ValueError,TypeError,cv2.error,np.linalg.LinAlgError) as exc:
                prediction['reason']=str(exc)
        else: prediction['reason']='CURRENT_COMPONENT_UNOBSERVED'
        result['fov']['next_descent']=prediction
    return result


def render_wrist(frame, result):
    """Return a separate debug artifact; draw only on a copy of the raw pixels."""
    if frame_binding(frame)!=result['binding']:
        raise ValueError('OVERLAY_FEATURE_RAW_IMAGE_BINDING_MISMATCH')
    raw=frame['image_bgr']; image=raw.copy(); h,w=image.shape[:2]
    green=(50,240,80); orange=(0,180,255); gray=(180,180,180)
    def line(points,color,closed=False):
        p=_points(points)
        if len(p)<2:return
        pairs=list(zip(p,p[1:]))+([(p[-1],p[0])] if closed else [])
        for a,b in pairs:
            if np.max(np.abs([a,b]))>np.iinfo(np.int32).max:continue
            ok,a,b=cv2.clipLine((0,0,w,h),tuple(np.rint(a).astype(int).tolist()),tuple(np.rint(b).astype(int).tolist()))
            if ok:cv2.line(image,a,b,color,1,cv2.LINE_AA)
    def cross(point,color):
        if point is not None and 0<=point[0]<w and 0<=point[1]<h:
            cv2.drawMarker(image,tuple(np.rint(point).astype(int).tolist()),color,cv2.MARKER_CROSS,13,1)
    cv2.rectangle(image,(0,0),(w-1,h-1),gray,1)
    obj=result['object']; grip=result['gripper']
    for edge in obj['visible_edges']:line(edge['points_px'],green)
    for edge in obj['clipped_boundary']:line(edge['points_px'],(0,0,255))
    cross(obj['component_centroid_uv'],green)
    if grip['observed_midline_u'] is not None:
        x=grip['observed_midline_u'];line([[x,0],[x,h-1]],green)
        line(grip['closing_line_uv'],green)
        cross([x,grip['observed_reference_row']],green)
    for edge in grip['model_pad_boundaries_uv']:line(edge,orange)
    cross(grip['model_grasp_centre_uv'],orange)
    if grip.get('model_closing_line_uv') is not None:
        points=_points(grip['model_closing_line_uv']);line(points,orange)
        mid=points.mean(0)
        for end in points:
            if np.max(np.abs([end,mid]))<np.iinfo(np.int32).max:
                cv2.arrowedLine(image,tuple(np.rint(end).astype(int).tolist()),tuple(np.rint(mid).astype(int).tolist()),orange,1,tipLength=.2)
    if obj.get('valid_reference_edge_uv'):line(obj['valid_reference_edge_uv'],(255,255,0))
    predicted=result['fov']['next_descent'].get('contour_uv')
    if predicted is not None:line(predicted,(255,160,255))
    lines=[result['side'].upper()+' OBSERVED green / MODEL_PRIOR orange',
        'green +: visible centroid; green |: observed X',
        'orange +: MODEL grasp; arrows: MODEL closing',
        'pink: MODEL next top envelope, NOT full FOV',
        'OBS grasp2D UNKNOWN; X '+str(grip['observed_midline_u']),
        'OBS error px '+str(result['image_error_px']),
        'MODEL error '+str({k:None if v is None else round(v,1) for k,v in result['model_relative_image_error_px'].items()}),
        'relative angle '+str(result['relative_angle']['value_deg'])+' '+result['relative_angle']['status'],
        'FOV clip X='+str(obj['widest_row_horizontal_clipped'])+' any='+str(obj['frame_clipped']),
        'next '+result['fov']['next_descent']['status'],
        'DEBUG ONLY - NO CORRECTION / NOT DETECTOR INPUT']
    footer=np.zeros((18*len(lines)+8,w,3),np.uint8)
    for i,text in enumerate(lines):cv2.putText(footer,text,(4,16+18*i),cv2.FONT_HERSHEY_SIMPLEX,.31,(230,230,230),1,cv2.LINE_AA)
    return dict(artifact_role='DEBUG_OVERLAY_ONLY',image_bgr=np.vstack([image,footer]),source_binding=copy.deepcopy(result['binding']))
