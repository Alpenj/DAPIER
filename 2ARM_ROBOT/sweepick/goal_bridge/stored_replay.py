"""Read saved A RGB-D observations and rerun existing, file-only feature extraction."""
import ast
import copy
from pathlib import Path
import types

import cv2
import numpy as np

from b16_goal_bridge import load_module, read_json, sha256, temporal_alignment


def vision_functions(source):
    source = Path(source)
    faces = load_module('sweepick_261009_offsite_relative_features',
                        source/'sweepick_261009_offsite_relative_features.py')
    wrist = load_module('b17_wrist_geometry', source/'right_wrist_relative_geometry.py')
    # Importing the whole pick module imports the motor executor; these pure functions need only NumPy.
    names = {'_wrist_red_components', '_contour_runs', '_wrist_surface_candidate', 'wrist_object_surfaces'}
    path = source/'sweepick_pick01.py'
    tree = ast.parse(path.read_text())
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    if {n.name for n in nodes} != names:
        raise ValueError('A wrist surface source contract changed')
    module = types.ModuleType('b17_pure_wrist_surfaces')
    module.__dict__['np'] = np
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), module.__dict__)
    return faces, wrist, module


def observation_times(document, clock):
    actual = document['actual_feedback']
    receipt = dict(value=actual.get('t_bus_monotonic'), unit='s', clock=clock,
                   kind='HOST_PACKET_FEEDBACK_RECEIPT')
    joints = {s:dict(value=None, unit='s', clock=clock, kind='PER_ARM_RECEIPT_NOT_RECORDED',
                     measurement_time=None) for s in ('left','right')}
    sensors = []
    for view, camera in document['camera_frames'].items():
        sides = ('left',) if view == 'left_wrist' else ('right',) if view == 'right_wrist' else ('left','right')
        sensors.append(dict(view=view, value=camera.get('recv_monotonic_s'), unit='s', clock=clock,
            kind='HOST_CAMERA_RECEIPT', state_references={s:copy.deepcopy(joints[s]) for s in sides},
            packet_state_reference=copy.deepcopy(receipt),
            native_exposure_known=False, exposure_time=None,
            native_capture=dict(clock='UNKNOWN_DEVICE_TIMESTAMP',
                color_value=camera.get('sdk_color_stamp_s'), depth_value=camera.get('sdk_depth_stamp_s'),unit='s')))
    report = temporal_alignment(dict(joint_state=joints, sensors=sensors))
    from b16_goal_bridge import stamp_difference
    report['packet_receipt'] = receipt
    for row in report['cameras']:
        row['receipt_minus_packet_state_s'] = stamp_difference(row['receipt'],receipt)
    return report


def replay(folder, source, sample_id, clock):
    folder = Path(folder)
    files = ('OBSERVATION.json','top_rgb.png','top_depth_mm.npy','left_wrist_rgb.png',
             'right_wrist_rgb.png','VISION_MASKS.npz','OBSERVATION_DIAGNOSTICS.json')
    provenance = {name:dict(path=str(folder/name), exists=(folder/name).is_file(),
                           sha256=sha256(folder/name) if (folder/name).is_file() else None) for name in files}
    if not all(x['exists'] for x in provenance.values()):
        raise ValueError('saved replay bundle is incomplete; missing files are not synthesized')
    d = read_json(folder/'OBSERVATION.json')
    observation = d['observation']
    if d['source_kind'] != 'REAL':
        raise ValueError('REAL replay requires REAL provenance')
    q = np.asarray(d['q12'],float)
    if q.shape != (12,) or not np.isfinite(q).all() or not np.array_equal(q,observation['q12']):
        raise ValueError('saved actual state is malformed or unbound')
    image = cv2.imread(str(folder/'top_rgb.png'),cv2.IMREAD_COLOR)
    depth = np.load(folder/'top_depth_mm.npy',allow_pickle=False)
    K = np.asarray(d['top_intrinsics'],float).reshape(3,3).copy()
    T = np.asarray(d['T_world_camera'],float)
    K[0] *= depth.shape[1]/image.shape[1]; K[1] *= depth.shape[0]/image.shape[0]
    v,u = np.indices(depth.shape); z = depth.astype(float)/1000
    xyz_camera = np.stack([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z],axis=-1)
    xyz = xyz_camera@T[:3,:3].T+T[:3,3]
    xyz[~(np.isfinite(z)&(z>=.05)&(z<=2.5))] = np.nan
    masks = np.load(folder/'VISION_MASKS.npz',allow_pickle=False)
    faces,wrist,surfaces = vision_functions(source)
    seed = masks['held___masks__search_after_robot']
    allowed = masks['held___masks__robot_permitted']
    feature,reason = faces.observed_face_from_seed(depth,K,image,seed,image_shape=depth.shape,
        xyz_image=xyz,allowed_mask=allowed,minimum_points=50,T_output_camera=T,diagnostics={})
    old = observation.get('held_object',{}).get('observation_diagnostic',{}).get('feature') or {}
    agreement = dict(feature_present_before=bool(old), feature_present_now=feature is not None)
    top = dict(coordinate_frame='assembled dual model world (left base at its model pose)',
        camera_frame='top rectified optical; x right, y down, z forward; metres',
        pixel_frame='top registered depth pixels; scaled saved RGB intrinsics',
        provenance_files=list(files[:3])+['VISION_MASKS.npz'],
        metric_role='measured depth fit transformed through unverified existing A camera chain',
        prior_role='stored A search/robot masks reused; replay is feature-stage verification, not independent target acquisition',
        status='MISSING' if feature is None else 'PARTIAL', reason=reason,
        boundary_identified=False, centre_identified=False, full_rotation_observed=False)
    top['failure_category'] = 'ALGORITHM_FAILED' if feature is None else 'NO_VISIBLE_EDGE'
    top['failure_scope'] = ('existing seeded RGB-D extractor could not establish a face; '
        'this is not proof that no physical edge exists' if feature is None else
        'no farther-depth-certified exterior edge; visible RGB dividers are not a complete physical boundary')
    if feature is not None:
        centre = np.asarray(feature['plane_centre']);normal=np.asarray(feature['plane_normal'])
        top.update(plane_centre_model_m=centre.tolist(),plane_normal_model=normal.tolist(),
            plane_patch_centroid_camera_m=((centre-T[:3,3])@T[:3,:3]).tolist(),
            plane_normal_camera=(normal@T[:3,:3]).tolist(),
            plane_patch_centroid_is_object_centre=False, plane_points=feature['depth_pixels'],
            observed_exterior_edges=feature['observed_exterior_edges'],
            orientation_identified=feature['orientation_identified'],
            complete_face_boundary=feature['complete_face_boundary'],visibility=feature['visibility'],
            occlusion=feature['occlusion'],rejected_boundary_edges=feature['rejected_boundary_edges'],
            fit_diagnostics=feature['fit_diagnostics'])
        if old:
            agreement.update(plane_point_count_equal=feature['depth_pixels']==old['depth_pixels'],
                centre_max_abs_difference_m=float(np.max(np.abs(centre-old['plane_centre']))),
                normal_sign_invariant_max_abs_difference=float(min(np.max(np.abs(normal-old['plane_normal'])),
                    np.max(np.abs(normal+old['plane_normal'])))),
                exterior_edge_count_equal=len(feature['observed_exterior_edges'])==len(old['observed_exterior_edges']))
    wrist_results = {}
    for side in ('left','right'):
        view=side+'_wrist';bgr=cv2.imread(str(folder/(view+'_rgb.png')),cv2.IMREAD_COLOR)
        measured=surfaces.wrist_object_surfaces(bgr)
        old_wrist=observation.get('wrist_rgb_surfaces',{}).get(view,{})
        row=dict(coordinate_frame=f'{view} raw RGB pixels (u right, v down)',unit='pixel',
            provenance_file=view+'_rgb.png',status=measured['status'],
            red_identification_role=measured['colour_role'],
            physical_boundary_identity=False,metric_pose_available=False,
            component_area_px=measured.get('component_area_px'),
            visible_contour_runs=len(measured.get('visible_edges',[])),
            frame_clipped=measured.get('frame_clipped'),
            interior_edge_candidates=len(measured.get('interior_edge_candidates',[])),
            recorded_component_area_equal=(measured.get('component_area_px')==old_wrist.get('component_area_px')),
            camera_state=d['camera_frames'].get(view,{}))
        if measured['visible']:
            uv=np.asarray(measured['contour_points_px'])
            row['observed_2d_boundary_bbox_uv']=np.stack([uv.min(0),uv.max(0)]).tolist()
            vv,uu=np.nonzero(measured['_mask'])
            row['segmented_region_centroid_uv']=[float(uu.mean()),float(vv.mean())]
            row['segmented_region_centroid_is_object_face_centre']=False
            cv2.setRNGSeed(0)
            try:
                points,edges,_,_,diagnostic=wrist.observed_face_graph(bgr,measured['_mask'])
                row.update(graph_junctions_uv=points.tolist(),graph_edges=edges,
                           graph_diagnostic=diagnostic,algorithm_status='RGB_GRAPH_EXTRACTED')
            except (ValueError,cv2.error) as exc:
                row.update(algorithm_status='ALGORITHM_FAILED',algorithm_reason=str(exc))
        row['missing_metric_geometry_reason']='CALIBRATION_UNKNOWN: 2D contours do not establish a metric face centre/orientation or inner-pad identity'
        row['occlusion_evidence']='segmented target meets image border' if row['frame_clipped'] else 'hidden/back surfaces and pad interior not identified by RGB contour'
        row['current_observation_eligible']=d['camera_frames'].get(view,{}).get('fresh') is True
        if row['recorded_component_area_equal'] is False:
            row['recorded_difference_reason']=('saved frame is marked stale; stored live eligibility/result differs from file-only re-extraction'
                if not row['current_observation_eligible'] else 'UNRESOLVED_SOURCE_OR_ALGORITHM_DIFFERENCE')
        wrist_results[view]=row
    pad_results=[]
    diagnostics=read_json(folder/'OBSERVATION_DIAGNOSTICS.json')
    for index,candidate in enumerate(diagnostics.get('relative',{}).get('candidates',[])):
        key=f'relative__candidates__{index}__pad___masks__seed'
        if key not in masks:
            pad_results.append(dict(index=index,status='REPLAY_SEED_NOT_RECORDED'));continue
        pad,why=faces.observed_face_from_seed(depth,K,cv2.resize(image,(depth.shape[1],depth.shape[0]),
            interpolation=cv2.INTER_AREA),masks[key],image_shape=image.shape[:2],diagnostics={})
        original=candidate.get('pad',{}).get('current_feature') or {}
        support_key=f'relative__candidates__{index}__pad___masks__seed_linked_plane_support'
        pad_results.append(dict(index=index,search_candidate=candidate['search_candidate'],
            provenance_files=['VISION_MASKS.npz','top_rgb.png','top_depth_mm.npy','OBSERVATION.json'],
            coordinate_frame='top rectified optical (metres); polygon uses original top RGB pixels',
            status='MISSING' if pad is None else 'PLANE_PATCH_ONLY',reason=why,
            plane_points=None if pad is None else pad['depth_pixels'],
            seed_points=int(np.count_nonzero(masks[key])),
            recorded_plane_support_points=None if support_key not in masks else int(np.count_nonzero(masks[support_key])),
            recorded_plane_support_count_equal=None if pad is None or support_key not in masks else
                pad['depth_pixels']==int(np.count_nonzero(masks[support_key])),
            plane_centre_camera_m=None if pad is None else pad['plane_centre'],
            plane_normal_camera=None if pad is None else pad['plane_normal'],
            actual_inner_pad_identity_verified=False,
            exterior_edges=None if pad is None else pad['observed_exterior_edges'],
            recorded_exterior_edge_count_equal=None if pad is None or not original else
                len(pad['observed_exterior_edges'])==len(original.get('observed_exterior_edges',[])),
            stored_association=copy.deepcopy(candidate.get('target_association')),
            prior_role='CAD-seeded current pixels; CAD alone is not observed physical pad identity'))
    failures=['CALIBRATION_UNKNOWN','TIME_ALIGNMENT_UNKNOWN']
    if feature is None:failures.append('ALGORITHM_FAILED')
    elif not feature['observed_exterior_edges']:failures.append('NO_VISIBLE_EDGE')
    if feature is not None and feature['visibility']=='PARTIAL_OCCLUDED':failures.append('OCCLUDED')
    if any(x.get('algorithm_status')=='ALGORITHM_FAILED' for x in wrist_results.values()):failures.append('ALGORITHM_FAILED')
    return dict(sample_id=sample_id,source_kind='REAL_STORED',stage=d['stage'],sources=provenance,
        saved_intrinsics=d['top_intrinsics'],intrinsics_source='OBSERVATION.json/top_intrinsics',
        registration=d.get('top_registration'),actual_q12=q.tolist(),
        actual_q12_units='10 arm radians + 2 gripper fractions; existing A candidate mapping',
        original_camera_chain_role=d['camera_chain_role'],
        temporal_alignment=observation_times(d,clock),top=top,wrists=wrist_results,pads=pad_results,
        feature_stage_agreement=agreement,actual_pad_object_relation='UNRESOLVED',
        failure_categories=list(dict.fromkeys(failures)),
        failure_interpretation='NO_VISIBLE_EDGE concerns certified top depth exterior edges; wrist 2D contours can be visible. '
            'A failed or partial extractor does not prove physical information is absent. Occlusion identity and metric wrist/pad association remain unverified.',
        final_status='NEEDS_OBSERVATION',executable=False,motor_write_authorized=False)
