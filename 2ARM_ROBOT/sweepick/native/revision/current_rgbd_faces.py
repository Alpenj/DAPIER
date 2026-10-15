"""Current sensor geometry shared by stored replay and the existing LIVE Owner."""
from pathlib import Path
import time
import cv2
import numpy as np


def cad_pixel_correspondence(seed, xyz, K, search, measured, faces):
    """Finite CAD search + integer-mm/pixel-cell representation, never a physical bound."""
    v, u = np.nonzero(seed)
    n, datum = search['normal_camera'], search['datum_camera_m']
    rays = np.column_stack([(u-K[0,2])/K[0,0], (v-K[1,2])/K[1,1], np.ones(len(u))])
    nominal_z = (n@datum)/(rays@n)
    corner_z = []
    for du, dv in ((-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5)):
        corner_rays = rays + [du/K[0,0], dv/K[1,1], 0]
        corner_z.append((n@datum)/(corner_rays@n))
    corner_z = np.asarray(corner_z)
    # These are representation cells of the supplied integer-mm grid, not a sensor accuracy claim.
    lo, hi = corner_z.min(axis=0)-.0005, corner_z.max(axis=0)+.0005
    actual_z = xyz[v,u,2]
    valid = np.isfinite(corner_z).all(axis=0) & (corner_z>0).all(axis=0)
    matches = valid & (actual_z>=lo) & (actual_z<=hi)
    signed = (measured['_points_xyz']-datum)@n
    pv, pu = np.nonzero(measured['_mask'])
    prays = np.column_stack([(pu-K[0,2])/K[0,0],(pv-K[1,2])/K[1,1],np.ones(len(pu))])
    nominal = prays*(np.rint(((n@datum)/(prays@n))*1000)/1000)[:,None]
    return dict(role='MODEL_ASSOCIATION_NOT_PHYSICAL_CERTIFICATE',
        compatible=bool(len(matches) and matches.all()),
        criterion='all current finite CAD-seeded leaf pixels intersect the CAD plane pixel-cell/integer-mm representation interval',
        samples=len(matches), matched_pixels=int(matches.sum()),
        max_representation_excess_mm=float(np.maximum.reduce([lo-actual_z,actual_z-hi,np.zeros(len(u))]).max()*1000),
        pixel_centres='integer u/v in supplied K domain; footprint +/-0.5 pixel, no applied image shift',
        depth_axis='camera Z; ray distance diagnostic is separate',
        quantization_role='stored integer-mm representation only; physical noise/calibration bound UNKNOWN',
        same_plane_BIC_diagnostic=bool(np.isfinite(nominal).all() and faces._components_share_plane(measured['_points_xyz'],nominal)),
        BIC_role='diagnostic only: independent observed RGB components is its original purpose, nominal CAD is deterministic',
        median_signed_plane_residual_mm=float(np.median(signed)*1000),
        residual_is_physical_error_bound=False)


def axis_only_evidence(pad,K,scale,faces):
    """A current RGB line can constrain a plane axis without identifying its exterior boundary."""
    fit=dict(centre=np.asarray(pad['plane_centre']),normal=np.asarray(pad['plane_normal']))
    output=[]
    for edge in pad['rejected_boundary_edges']:
        uv=np.asarray(edge['uv'])/scale
        points,usable,_=faces._plane_points_at_uv(uv,K,np.eye(4),fit)
        vector=points[1]-points[0];length=np.linalg.norm(vector)
        if not usable.all() or length<=np.finfo(float).eps:continue
        output.append(dict(uv=edge['uv'],axis=(vector/length).tolist(),
            classification='ESTIMATED_RGB_RAY_INTERSECTION_WITH_CURRENT_MEASURED_PLANE',
            exterior_boundary_identified=False,original_rejection=edge['reason'],
            usable_for_insertion_ready=False))
    return output


def pad_candidate_groups(accepted):
    groups={name:[pair for pair in accepted if pair[0]['search_candidate']==name]
            for name in ('right_pgripper_pad_1','right_pgripper_pad_2')}
    return groups,[name for name,pairs in groups.items() if len(pairs)>1]


def cross_jaw_patch_overlap(accepted):
    return [(a[0]['search_candidate'],b[0]['search_candidate']) for i,a in enumerate(accepted)
            for b in accepted[i+1:] if a[0]['search_candidate']!=b[0]['search_candidate'] and np.any(a[1]['_mask']&b[1]['_mask'])]


def triangle_distance(points, triangles):
    """Exact finite-triangle distances; batching controls memory, not observation acceptance."""
    a,b,c=triangles.transpose(1,0,2);ab=b-a;ac=c-a;n=np.cross(ab,ac)
    nn=(n*n).sum(1);aa=(ab*ab).sum(1);cc=(ac*ac).sum(1);bc=(ab*ac).sum(1)
    den=aa*cc-bc*bc;output=[]
    for start in range(0,len(points),64):
        p=points[start:start+64,None,:];ap=p-a
        x=(ap*ab).sum(2);y=(ap*ac).sum(2)
        u=np.divide(x*cc-y*bc,den,out=np.full_like(x,np.nan),where=den!=0)
        v=np.divide(y*aa-x*bc,den,out=np.full_like(y,np.nan),where=den!=0)
        edges=[]
        for first,second in ((a,b),(b,c),(c,a)):
            e=second-first;ee=(e*e).sum(1)
            t=np.divide(((p-first)*e).sum(2),ee,out=np.zeros_like(x),where=ee!=0)
            edges.append(((p-first-np.clip(t,0,1)[...,None]*e)**2).sum(2))
        plane=np.divide(((ap*n).sum(2))**2,nn,out=np.full_like(x,np.inf),where=nn!=0)
        d=np.where((u>=0)&(v>=0)&(u+v<=1),plane,np.minimum.reduce(edges))
        output.append(np.sqrt(d.min(1)))
    return np.concatenate(output)


def observed_body_follow(current, previous, camera_from_body, faces):
    """History supports target identity only; current pose/edges are never filled from it."""
    matches=[]
    for old in previous:
        delta=old['camera_from_body']@np.linalg.inv(camera_from_body)
        transformed=current@delta[:3,:3].T+delta[:3,3]
        follows=faces._components_share_plane(transformed,old['points'])
        stationary=faces._components_share_plane(current,old['points'])
        matches.append(dict(previous_frame=old['camera'],previous_feedback=old['feedback'],
            body_follow_same_plane=bool(follows),static_same_plane=bool(stationary),
            previous_target_evidence=old.get('target_evidence'),
            previous_target_eligible=old.get('target_eligible',True),
            classification='CURRENT_MEASURED_PLANES_WITH_ESTIMATED_ACTUAL_FK_TRANSFORM',
            previous_points_used_for_current_pose_or_edges=False))
    supported=[m for m in matches if m['body_follow_same_plane'] and not m['static_same_plane'] and m['previous_target_eligible']]
    frames=[m['previous_frame'] for m in supported]
    identified=any(sum(m['previous_frame']==frame for m in supported)==1
                   and not any(m['previous_frame']==frame and m['static_same_plane'] for m in matches)
                   for frame in frames)
    return dict(supported=identified,comparisons=matches,physical_error_bound=None)


def produce_face_relation(*, depth, image, K_image, T_world_camera, model, data,
                          model_path, held_diagnostic, held_is_current, source_kind,
                          identity, feedback, stage, previous_faces=None, prepared=None,
                          timings=None):
    """No device access, owner mutation, GT, cached pose or readiness injection."""
    import sweepick_261009_offsite_relative_features as faces
    from inner_pad_face_contract import inner_face_contract, project_contract
    from native_surface_candidates import surface_candidates
    missing = dict(status='MISSING',source_kind=source_kind,ready=False,camera=identity,
                   feedback_identity=feedback,stage=stage)
    diag = dict(status='MISSING',stage=stage,camera=identity,feedback_identity=feedback,
                candidates=[],producer_policy='CURRENT_DEPTH_SURFACE_V7', geometry_called=True)
    started=time.perf_counter()
    K=np.asarray(K_image,float).reshape(3,3).copy()
    K[0]*=depth.shape[1]/image.shape[1];K[1]*=depth.shape[0]/image.shape[0]
    v,u=np.indices(depth.shape);z=depth.astype(float)/1000
    xyz=np.stack([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z],axis=-1)
    scale=[image.shape[1]/depth.shape[1],image.shape[0]/depth.shape[0]]
    color=cv2.resize(image,(depth.shape[1],depth.shape[0]),interpolation=cv2.INTER_AREA)
    rgb=faces.observed_rgb_regions(color)
    if timings is not None:timings['pad_image_prepare_s']=time.perf_counter()-started
    feature=held_diagnostic.get('feature') or {}; masks=held_diagnostic.get('_masks',{})
    pieces=(held_diagnostic.get('_feature_masks',{}).get('seed_linked_plane_support'),
            masks.get('search_after_robot'),masks.get('robot_permitted'))
    mask=np.zeros(depth.shape,bool)
    valid_masks=all(isinstance(m,np.ndarray) and m.dtype==bool and m.shape==depth.shape for m in pieces)
    if held_is_current and valid_masks: mask=pieces[0]&pieces[1]&pieces[2]
    object_valid=bool(mask.any() and feature.get('orientation_identified') is True and feature.get('visible_edges')
                      and np.isfinite(xyz[mask]).all() and (z[mask]>0).all())
    obj=(dict(_mask=mask,_points_xyz=xyz[mask],polygon=(np.asarray(feature['polygon'])*scale).tolist(),
              fit_diagnostics=feature['fit_diagnostics']) if object_valid else None)
    diag['held_current_measurement_available']=object_valid
    diag['held_missing_reason']=None if object_valid else (held_diagnostic.get('missing_reason') or
        'current FrameSet/feedback-bound measured held plane/edge unavailable')
    from inner_pad_face_contract import mesh_in_body
    camera_from_world=np.linalg.inv(np.asarray(T_world_camera))
    meshes={}
    for index in (1,2):
        if prepared is None:
            vertices,tri,body,_=mesh_in_body(model,data,f'right_pgripper_jaw_{index}_visual')
            world=vertices@data.xmat[body].reshape(3,3).T+data.xpos[body]
            meshes[index]=(world@camera_from_world[:3,:3].T+camera_from_world[:3,3])[tri]
        else:
            static=prepared['meshes'][index];body=static['body_id'];triangles=static['triangles_body_m']
            world=triangles@data.xmat[body].reshape(3,3).T+data.xpos[body]
            meshes[index]=world@camera_from_world[:3,:3].T+camera_from_world[:3,3]
    accepted=[];current_faces={}
    pad_started=time.perf_counter()
    for index in (1,2):
        name=f'right_pgripper_pad_{index}'
        contract=inner_face_contract(model,data,index,model_path) if prepared is None else prepared['contracts'][index]
        search=project_contract(contract,model,data,K,T_world_camera,depth.shape)
        prior=search['seed']&(depth>0)&~mask
        leaves,splits=surface_candidates(prior,xyz,faces)
        for number,seed in enumerate(leaves):
            d=dict(search_candidate=name,depth_surface_id=number,object=dict(status='OK' if object_valid else 'MISSING'),pad={},
                depth_partition=dict(original_points=int(prior.sum()),leaf_points=int(seed.sum()),leaf_count=len(leaves),splits=splits,
                    grouping_role='CURRENT_DEPTH_FEATURES_UNIDENTIFIED',CAD_used_to_split=False,_masks=dict(surface_candidate_seed=seed)),
                search_prior=dict(role='MODEL_PRIOR_INNER_CONTACT_FACE',source_visual_geom=contract['source_visual_geom'],
                    visual_triangle_ids=contract['visual_triangle_ids'].tolist(),pixels=int(seed.sum()),
                    projected_pixels=int(search['seed'].sum()),
                    valid_depth_before_object_exclusion=int((search['seed']&(depth>0)).sum()),
                    object_support_excluded=int((search['seed']&(depth>0)&mask).sum()),
                    visibility_prior=search['visibility_prior'],normal_camera=search['normal_camera'].tolist(),
                    datum_camera_m=search['datum_camera_m'].tolist(),approach_edge_uv=(search['approach_edges_uv']*scale).tolist(),
                    physical_identity_verified=False,not_an_observed_boundary=True))
            diag['candidates'].append(d)
            pad,reason=faces.observed_face_from_seed(depth,K,color,seed,image_shape=image.shape[:2],rgb_features=rgb,diagnostics=d['pad'])
            if reason is None:
                body_pose=np.eye(4);body_pose[:3,:3]=data.xmat[contract['body_id']].reshape(3,3)
                body_pose[:3,3]=data.xpos[contract['body_id']]
                camera_from_body=camera_from_world@body_pose
                history_record=dict(points=pad['_points_xyz'],camera_from_body=camera_from_body,
                    camera=identity,feedback=feedback)
                current_faces.setdefault(name,[]).append(history_record)
                d['pad']['current_feature']={k:pad[k] for k in ('plane_centre','plane_normal','polygon','ordered_edge','fit_diagnostics','visible_edges','edge_missing_reason','observed_exterior_edges','rejected_boundary_edges','visibility','complete_face_boundary')}
                d['CAD_correspondence']=cad_pixel_correspondence(seed,xyz,K,search,pad,faces)
                d['CAD_correspondence'].update(required_for_model_fixture=source_kind=='SYNTHETIC',
                    REAL_policy='MODEL_PRIOR association diagnostic; unknown physical error never becomes an integer-mm LIVE tolerance')
                d['pad']['axis_only_candidates']=axis_only_evidence(pad,K,scale,faces)
                failed=[]
                if not pad['orientation_identified']: failed.append(pad['edge_missing_reason'])
                if source_kind=='REAL':
                    selected=np.zeros(len(meshes[index]),bool);selected[contract['visual_triangle_ids']]=True
                    measured_seed=xyz[seed]
                    distances=dict(inner=float(np.sqrt(np.mean(triangle_distance(measured_seed,meshes[index][selected])**2))),
                        other_jaw_surface=float(np.sqrt(np.mean(triangle_distance(measured_seed,meshes[index][~selected])**2))),
                        opposite_jaw=float(np.sqrt(np.mean(triangle_distance(measured_seed,meshes[3-index])**2))))
                    follow=observed_body_follow(pad['_points_xyz'],(previous_faces or {}).get(name,[]),camera_from_body,faces)
                    model_consistent=distances['inner']<min(distances['other_jaw_surface'],distances['opposite_jaw'])
                    history_record.update(target_eligible=bool(model_consistent and search['camera_side_signed_m']>0 and not np.any(pad['_mask']&mask)),
                        target_evidence=dict(finite_CAD_distances_m=distances,model_assignment_consistent=model_consistent,
                            nominal_front_facing=bool(search['camera_side_signed_m']>0),held_patch_overlap=bool(np.any(pad['_mask']&mask)),
                            orientation_identified=pad['orientation_identified'],role='MODEL_ASSOCIATION_NOT_PHYSICAL_IDENTITY'))
                    support=bool(follow['supported'] and model_consistent and search['camera_side_signed_m']>0)
                    d['target_association']=dict(supported=support,body_follow=follow,finite_CAD_distances_m=distances,
                        CAD_role='MODEL_ASSOCIATION_CONTRADICTION_ONLY_NOT_PHYSICAL_BOUND',
                        criterion='current plane/edge plus same-session observed body-follow distinct from static alternative and no competing jaw-model assignment',
                        physical_identity_verified=False,physical_error_bound=None)
                    if not support:failed.append('current inner-target association is unsupported or ambiguous; nominal CAD/nearest alone cannot identify the measured patch')
                if source_kind=='SYNTHETIC' and search['camera_side_signed_m']<=0: failed.append('CAD search is back-facing; physical inner identity unverified')
                if source_kind=='SYNTHETIC' and not d['CAD_correspondence']['compatible']: failed.append('current depth leaf is incompatible with finite CAD pixel/depth representation')
                if np.any(pad['_mask']&mask): failed.append('pad and held face overlap the same current measured patch')
                d['failed_requirements']=failed;reason='; '.join(failed) if failed else None
            d.update(status='MISSING' if reason else 'OK',missing_reason=reason)
            if reason is None: accepted.append((d,pad))
    groups,ambiguous=pad_candidate_groups(accepted)
    cross=cross_jaw_patch_overlap(accepted)
    if cross:ambiguous=list(dict.fromkeys(ambiguous+[name for pair in cross for name in pair]))
    diag.update(accepted_surface_count=len(accepted),ambiguous_same_pad=ambiguous,
                ambiguous_cross_jaw_measured_patch=cross,_current_faces=current_faces,
                separately_observed_pads=[name for name,pairs in groups.items() if len(pairs)==1])
    if not object_valid or not accepted or ambiguous:
        reason=(diag['held_missing_reason'] if not object_valid else 'same pad has competing valid surfaces' if ambiguous else
                'no current inner pad plane/edge with model association')
        result=dict(missing,missing_reason=reason,candidate_count=len(diag['candidates']),accepted_surface_count=len(accepted),
                    surface_rejections=[dict(pad=d['search_candidate'],surface=d['depth_surface_id'],points=d['depth_partition']['leaf_points'],reason=d.get('missing_reason')) for d in diag['candidates']])
    else:
        # Distinct pad identities are not ambiguity: reuse the existing pad-1/pad-2 preference.
        d,pad=accepted[0]
        result=faces._relation_from_observed_features(pad,obj,source_kind=source_kind,
            source_image='current depth-separated top FrameSet',timestamps={})
        result.update(camera=identity,feedback_identity=feedback,stage=stage,frame_identity=identity,
            inner_face_identity_verified=False,physical_identity='UNKNOWN',search_pose_role='MODEL_PRIOR',
            relation_scope='current RGB-D plane/edge; CAD model association, no GT/old pose/ready fill',
            producer_policy='CURRENT_DEPTH_SURFACE_V7',selected_depth_surface_id=d['depth_surface_id'],
            selected_pad=d['search_candidate'],separately_observed_pads=diag['separately_observed_pads'],
            target_association=d.get('target_association'),
            approach_edge_semantics_verified=False)
    diag.update(status=result['status'],missing_reason=result.get('missing_reason'))
    if timings is not None:timings['pad_face_production_s']=time.perf_counter()-pad_started
    return result,diag


def current_face_relation(owner,stage,observation,frames):
    import sweepick_full_chain as fc
    import mujoco
    expected={'LIVE_REAL':'REAL','NO_DEVICE_REAL_STORED':'REAL','NO_DEVICE_SENSOR_SIM':'SYNTHETIC'}
    if expected.get(owner.evaluation_mode)!=owner.record_kind:
        raise ValueError('evaluation mode and unchanged observation source do not match')
    top=frames.get('top') or {};identity=fc.camera_identity(top)
    feedback={k:observation.get(k) for k in ('feedback_stamp','seq')}
    K=getattr(getattr(owner.ep,'top',None),'K',None)
    if not fc.camera_valid(top) or top.get('depth') is None or top.get('frame') is None or K is None or (owner.scene_look or {}).get('T_world_camera') is None:
        owner.relative_diagnostic=dict(status='MISSING',geometry_called=False,candidates=[])
        return dict(status='MISSING',source_kind=owner.record_kind,missing_reason='current registered RGB-D/K/TF unavailable',camera=identity,feedback_identity=feedback,stage=stage)
    diagnostic=getattr(owner,'held_diagnostic',{})
    bound=((observation.get('held_object') or {}).get('pose_role')=='CURRENT_OBSERVATION'
           and diagnostic.get('camera')==identity and diagnostic.get('feedback_identity')==feedback)
    prepared=getattr(owner,'face_geometry_prepared',None)
    model=prepared['model'] if prepared is not None else owner.p1.model();data=mujoco.MjData(model)
    fk_started=time.perf_counter()
    owner.p1.sweepick_kin.set_q12(model,data,np.asarray(observation['q12'],float));mujoco.mj_forward(model,data)
    timings={"q_dependent_fk_s":time.perf_counter()-fk_started}
    # The XML must be the source of the compiled model, supplied by the selected launcher/replay.
    model_path=getattr(owner,'face_model_path',None) or (getattr(owner,'contact_plan',None) or {}).get('face_geometry_source_xml')
    if model_path is None:
        raise ValueError('selected model source XML is unavailable for CAD provenance')
    result,owner.relative_diagnostic=produce_face_relation(depth=top['depth'],image=top['frame'],K_image=K,
        T_world_camera=owner.scene_look['T_world_camera'],model=model,data=data,model_path=model_path,
        held_diagnostic=diagnostic,held_is_current=bound,source_kind=owner.record_kind,identity=identity,feedback=feedback,stage=stage,
        previous_faces=getattr(owner,'previous_measured_pad_faces',None),prepared=prepared,timings=timings)
    owner.relative_producer_timings=timings
    current=owner.relative_diagnostic.pop('_current_faces')
    if owner.record_kind=='REAL':
        history=getattr(owner,'previous_measured_pad_faces',{})
        for name,values in current.items():history.setdefault(name,[]).extend(values)
        owner.previous_measured_pad_faces=history
    result['timestamps']={k:top.get(k) for k in ('sdk_color_stamp_s','sdk_depth_stamp_s','helper_receipt_monotonic_s','recv_monotonic_s')}
    return result
