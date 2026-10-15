"""One current-frame view proposal, using existing A IK/path/recovery, never pixel-shifted observations."""
import cv2
import numpy as np


def propose_receiver_inner_pad_view(owner, observation, *, actual=None):
    """Adopted noncontact view: this session's clearance pose, then camera-derived receiver roll."""
    import sweepick_full_chain as fc
    before = getattr(owner, 'before_right_pregrasp_q12', None)
    if before is None or getattr(owner, 'before_right_pregrasp_observation', None) is None:
        raise ValueError('same-session measured before-pregrasp tool frame is unavailable')
    current = (owner.p1.rc.ticks_to_q12(owner.mp, actual['ticks']) if actual is not None
               else np.asarray(observation['q12'], float))
    model = owner.p1.model()
    start, _ = fc.tool_pose(model, current, site='right_cube_grasp')
    clear, _ = fc.tool_pose(model, np.asarray(before, float), site='right_cube_grasp')
    camera = np.asarray(owner.scene_look['T_world_camera'], float)[:3, 3]
    axis, jaw = clear[:3, 0], clear[:3, 2]
    desired = np.cross(axis, camera - clear[:3, 3])
    norm = np.linalg.norm(desired)
    if not np.isfinite(norm) or norm <= np.finfo(float).eps:
        raise ValueError('camera/tool geometry does not determine a distinct receiver roll')
    desired /= norm
    angles = [float(np.arctan2(axis @ np.cross(jaw, sign * desired), jaw @ (sign * desired))) for sign in (1, -1)]
    angle = min(angles, key=abs)
    turn = np.eye(4)
    turn[:3, :3] = [[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]]
    return dict(id='CURRENT_RECEIVER_CLEAR_THEN_INNER_PAD_VIEW', source_kind='SENSOR_SIM_CANDIDATE',
        role='MODEL_PRIOR_VIEW_PROPOSAL_NOT_OBSERVATION',
        source_note='existing relative-tool proposal protocol; actual observation retains REAL provenance',
        targets=[dict(side='right', reference_frame='CURRENT_TOOL', reference_from_target_tool=(np.linalg.inv(start) @ clear).tolist()),
                 dict(side='right', reference_frame='CURRENT_TOOL', reference_from_target_tool=turn.tolist())],
        expected_observed_components=['current object approach face', 'current receiving inner-pad plane and approach-side edge'],
        observation_policy='FINAL_TARGET_ONLY', actual_q12_basis=current.tolist(),
        planning_feedback_identity={k: actual.get(k) for k in ('bus_seq', 't_bus_monotonic', 'cycle_start')} if actual is not None else None,
        input_feedback_identity={k: observation.get(k) for k in ('feedback_stamp', 'seq')},
        roll_degrees_from_current_geometry=float(np.degrees(angle)),
        clearance_translation_m=(clear[:3, 3] - start[:3, 3]).tolist(),
        stored_q_used_as_command=False, physical_success=False)


def propose_held_view(owner, observation):
    """Recover visibility from this session; a proposal never supplies a measured held pose."""
    import sweepick_full_chain as fc
    current=np.asarray(observation['q12'],float);model=owner.p1.model()
    prior=getattr(owner,'before_right_pregrasp_observation',None) or {}
    before=prior.get('held_object') or {}
    if before.get('centre_identified') is True and (before.get('orientation_observation') or {}).get('full_rotation_observed') is True:
        old_q=np.asarray(owner.before_right_pregrasp_q12,float)
        start,_=fc.tool_pose(model,current,site='right_cube_grasp')
        previous,_=fc.tool_pose(model,old_q,site='right_cube_grasp')
        delta=np.linalg.inv(start)@previous;side='right';cause='held was measured before right pregrasp and disappeared afterwards'
    else:
        start,_=fc.tool_pose(model,current,site='left_cube_grasp')
        held=observation.get('held_object') or {};search=held.get('search_prior') or {}
        if search.get('role')!='SEARCH_PRIOR' or search.get('pose_model') is None:
            raise ValueError('no current actual-bound held search prior for an observation-only pose')
        pose=np.asarray(search['pose_model'],float);camera=np.asarray(owner.scene_look['T_world_camera'])[:3,3]
        toward=camera-pose[:3,3];toward/=np.linalg.norm(toward)
        normals=np.concatenate([pose[:3,:3].T,-pose[:3,:3].T]);normal=normals[np.argmax(normals@toward)]
        # Preserve the 5-DOF tool approach axis; arbitrary full rotation need not have an A IK solution.
        axis=start[:3,0];a=normal-axis*(normal@axis);b=toward-axis*(toward@axis)
        if min(np.linalg.norm(a),np.linalg.norm(b))<=np.finfo(float).eps:
            raise ValueError('tool roll cannot improve this nominal face view')
        a/=np.linalg.norm(a);b/=np.linalg.norm(b)
        s=float(axis@np.cross(a,b));c=float(a@b)
        if abs(s)<=np.finfo(float).eps and c>0:raise ValueError('current model roll is already face-on; no distinct held view is justified')
        x,y,z=axis;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        turn=np.eye(3)+s*cross+(1-c)*(cross@cross)
        desired=pose.copy();desired[:3,:3]=turn@pose[:3,:3]
        relation=np.linalg.inv(start)@pose
        tool=desired@np.linalg.inv(relation)
        delta=np.linalg.inv(start)@tool;side='left';cause='current held centre/axis missing; model face visibility improved about existing tool roll axis with fixed object centre'
    targets=[dict(side=side,reference_frame='CURRENT_TOOL',reference_from_target_tool=delta.tolist())]
    if side=='left' and 'RIGHT_APPROACH' in owner.done and hasattr(owner,'before_right_pregrasp_q12'):
        # Clear the receiving hand before rotating the held block; both paths still require A checks.
        right_now,_=fc.tool_pose(model,current,site='right_cube_grasp')
        right_before,_=fc.tool_pose(model,np.asarray(owner.before_right_pregrasp_q12,float),site='right_cube_grasp')
        targets.insert(0,dict(side='right',reference_frame='CURRENT_TOOL',
            reference_from_target_tool=(np.linalg.inv(right_now)@right_before).tolist()))
        cause+='; receiving hand cleared to this session\'s observed pre-approach tool pose before held roll'
    return dict(id='V7_current_held_exposure',source_kind='SENSOR_SIM_CANDIDATE',
        targets=targets,
        expected_observed_components=['current held centre','current held face normal and identifiable edge'],
        observation_policy='FINAL_TARGET_ONLY',input_feedback_identity={k:observation.get(k) for k in ('feedback_stamp','seq')},
        role='MODEL_PRIOR_OBSERVATION_PROPOSAL_NOT_MEASUREMENT',cause=cause,
        old_q_used_as_command=False,physical_success=False)


def propose_current_view(owner,observation,frames):
    import mujoco
    from inner_pad_face_contract import inner_face_contract,project_contract
    held=observation.get('held_object') or {}
    if held.get('centre_identified') is not True or (held.get('orientation_observation') or {}).get('full_rotation_observed') is not True:
        raise ValueError('right-only view cannot resolve missing held centre/rotation; preserve paired new-frame deficits for recovery')
    top=frames.get('top') or {};depth=top.get('depth');image=top.get('frame')
    if depth is None or image is None:raise ValueError('no current RGB-D for view proposal')
    K=np.asarray(owner.ep.top.K).reshape(3,3).copy()
    K[0]*=depth.shape[1]/image.shape[1];K[1]*=depth.shape[0]/image.shape[0]
    model=owner.p1.model();data=mujoco.MjData(model)
    owner.p1.sweepick_kin.set_q12(model,data,np.asarray(observation['q12']));mujoco.mj_forward(model,data)
    source=getattr(owner,'face_model_path',None) or owner.contact_plan['face_geometry_source_xml']
    c=inner_face_contract(model,data,1,source)
    T=np.asarray(owner.scene_look['T_world_camera']);p=project_contract(c,model,data,K,T,depth.shape)
    if p['camera_side_signed_m']<=0:raise ValueError('no front-facing model inner-strip view seed')
    edge=np.asarray(p['approach_edges_uv'][0]);n=p['normal_camera'];datum=p['datum_camera_m']
    yy,xx=np.indices(depth.shape);rays=np.stack([(xx-K[0,2])/K[0,0],(yy-K[1,2])/K[1,1],np.ones(depth.shape)],axis=-1)
    denominator=rays@n
    predicted=np.full(depth.shape,np.nan);np.divide(n@datum,denominator,out=predicted,where=denominator!=0)
    background=np.isfinite(predicted)&(predicted>0)&(depth>0)&(depth/1000>predicted)
    # Current farther depth is only a MODEL_PRIOR of complementary view support after a move.
    ends=np.rint(edge).astype(int);lo=ends.min(axis=0);hi=ends.max(axis=0)
    template=np.zeros((hi[1]-lo[1]+1,hi[0]-lo[0]+1),np.float32)
    cv2.line(template,tuple(ends[0]-lo),tuple(ends[1]-lo),1,1)
    if any(a>b for a,b in zip(template.shape,depth.shape)):raise ValueError('projected edge does not fit current image')
    scores=cv2.matchTemplate(background.astype(np.float32),template,cv2.TM_CCORR)
    vy,ux=np.nonzero(np.rint(scores)==template.sum())
    if not len(vy):raise ValueError('current frame has no complementary valid farther-depth support for this model edge')
    shifts=np.column_stack([ux-lo[0],vy-lo[1]])
    nonzero=np.any(shifts!=0,axis=1);shifts=shifts[nonzero]
    if not len(shifts):raise ValueError('no distinct view proposal; current frame cannot be reused as new evidence')
    shift=shifts[np.argmin(np.square(shifts.astype(float)).sum(axis=1))]
    camera_delta=np.array([shift[0]*datum[2]/K[0,0],shift[1]*datum[2]/K[1,1],0.])
    site_R=data.site('right_cube_grasp').xmat.reshape(3,3)
    delta=np.eye(4);delta[:3,3]=site_R.T@T[:3,:3]@camera_delta
    return dict(id='V7_current_pixel_support_view',source_kind='SENSOR_SIM_CANDIDATE',
        targets=[dict(side='right',reference_frame='CURRENT_TOOL',reference_from_target_tool=delta.tolist())],
        expected_observed_components=['current held centre/axis','current inner strip exterior approach edge'],
        observation_policy='FINAL_TARGET_ONLY',input_frame_identity={k:top.get(k) for k in ('n','frame_number','helper_receipt_monotonic_s')},
        input_feedback_identity={k:observation.get(k) for k in ('feedback_stamp','seq')},pixel_shift_proposal=shift.tolist(),
        metric_delta_camera_m=camera_delta.tolist(),role='MODEL_PRIOR_OBSERVATION_PROPOSAL_NOT_MEASUREMENT',
        assumptions=['fixed-image farther support is a visibility hypothesis, not a prediction of future depth',
            'existing A candidate K/TF/CAD and current actual; target regenerated by A IK',
            'new frame/q and unchanged LIVE gates must decide insertion; no image is shifted or filled'],
        physical_success=False)
