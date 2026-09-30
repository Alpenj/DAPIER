"""STATIC14 saved-file mapping/FK and ChArUco PnP; no capture or dispatch."""
import argparse
from datetime import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(os.environ['DAPIER_ALIGNMENT_WORKTREE'])
SIM = ROOT / '2ARM_ROBOT/sim/mobile_dual_so101'
P = Path(os.environ['DAPIER_ALIGNMENT_SUPPORT'])
A = Path(os.environ['DAPIER_ALIGNMENT_STATIC13'])
os.environ['DAPIER_SO101_MJCF'] = os.environ['DAPIER_ALIGNMENT_MJCF']
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(ROOT / '2ARM_ROBOT/research/src'))
import mujoco
from evaluate_single_shot_ik import JOINTS, load_measured_state
from integration_scenes import task_env, portable_model_sha256
from replay_recorded_episode import mapping
from dapier_research.camera_board_transform import camera_from_board_pose


def source(path):
    path = Path(path).resolve(strict=True)
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def write_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def named_arm10_from_readback(readback_path, mapping_path):
    """Reuse calibrated degrees; apply the explicitly selected mapping once."""
    snapshot = json.loads(readback_path.read_text())
    profile_path = P / 'follower-profile.local.json'
    profile = json.loads(profile_path.read_text())
    # Validate the historical sample at its own acquisition end, preserving the
    # existing live freshness gate and the unchanged original timestamps.
    reference = datetime.fromisoformat(snapshot['finished_at']).timestamp()
    calibrated, bindings = {}, {}
    for side in ('left', 'right'):
        calibrated[side], bindings[side] = load_measured_state(
            readback_path, Path(profile['arms'][side]['calibration_path']), side, now_s=reference)
    document = json.loads(mapping_path.read_text())
    signs, offsets = mapping(document)
    degrees = np.concatenate([calibrated[side][:5] for side in ('left', 'right')])
    radians = np.deg2rad(degrees * signs + offsets)
    return radians, dict(mapping=source(mapping_path), profile=source(profile_path),
                         signs=signs.tolist(), zero_offsets_deg=offsets.tolist(),
                         calibrated_arm10_deg=degrees.tolist(), bindings=bindings,
                         gripper_range_0_100={side:float(calibrated[side][5]) for side in calibrated},
                         gripper_canonical=None, physically_verified=False,
                         physical_reference=document.get('physical_reference'))


def diagnostic_fk(model, arm10):
    data = mujoco.MjData(model)
    used, refs, limits = [], [], []
    for k, actuator in enumerate([0, 1, 2, 3, 4, 6, 7, 8, 9, 10]):
        joint = int(model.actuator_trnid[actuator, 0])
        address = int(model.jnt_qposadr[joint])
        lo, hi = max(model.jnt_range[joint, 0], model.actuator_ctrlrange[actuator, 0]), min(model.jnt_range[joint, 1], model.actuator_ctrlrange[actuator, 1])
        if not lo <= arm10[k] <= hi:
            raise ValueError(f'{model.joint(joint).name}: {arm10[k]} outside [{lo},{hi}]; no clipping')
        data.qpos[address] = arm10[k]
        used.append(address)
        refs.append(float(model.qpos0[address]))
        limits.append([float(lo), float(hi)])
    mujoco.mj_forward(model, data)
    result = {}
    for side in ('left', 'right'):
        site = model.site(side + '_cube_grasp')
        parent = int(model.site_bodyid[site.id])
        ancestors = []
        while parent:
            ancestors += [int(model.jnt_qposadr[j]) for j in range(int(model.body_jntadr[parent]), int(model.body_jntadr[parent] + model.body_jntnum[parent]))]
            parent = int(model.body_parentid[parent])
        assert set(ancestors).issubset(used), 'FK must not depend on unknown gripper/block coordinates'
        base = model.body(side + '_base').id
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = data.site_xmat[site.id].reshape(3, 3), data.site_xpos[site.id]
        result[side] = dict(T_model_world_tcp=T.tolist(),
                            xyz_model_base_m=(data.xmat[base].reshape(3, 3).T @ (T[:3, 3]-data.xpos[base])).tolist())
    return dict(tcp=result, qpos_addresses=used, model_joint_reference_rad=refs, scalar_limits_rad=limits)


def pose_record(points, pixels, K, D, rvec, tvec, method):
    T = camera_from_board_pose(rvec, tvec)
    projected = cv2.projectPoints(points, rvec, tvec, K, D)[0].reshape(-1, 2)
    errors = np.linalg.norm(projected - pixels.reshape(-1, 2), axis=1)
    depths = (points.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3])[:, 2]
    return dict(method=method, rvec=np.asarray(rvec).reshape(3).tolist(), tvec_m=np.asarray(tvec).reshape(3).tolist(),
                T_camera_from_board=T.tolist(), T_board_from_camera=np.linalg.inv(T).tolist(),
                reprojection_rms_px=float(np.sqrt(np.mean(errors**2))), reprojection_max_px=float(errors.max()),
                reprojection_errors_px=errors.tolist(), projected_corners_uv=projected.tolist(),
                corner_camera_z_m_range=[float(depths.min()), float(depths.max())], all_corner_depth_positive=bool((depths>0).all()),
                board_axes_in_camera=T[:3, :3].tolist(), rotation_determinant=float(np.linalg.det(T[:3, :3])))


def main(out):
    out.mkdir(parents=True, exist_ok=False)
    snapshot_path = A / 'A-readback.json'
    old_path = P / 'field-startup-20261002-prepared05-02.readback.json'
    model = task_env('desk', grippers='both').model
    model_hash = portable_model_sha256(model)
    comparison = dict(input=source(snapshot_path), previous_readback=source(old_path),
                      timestamp_preserved=True, hardware_access_this_work=False,
                      model_portable_sha256=model_hash, cases={})
    for name, path in [('tabletop_zero_offset', SIM/'tabletop_replay.json'),
                       ('parallel_edge_static_candidate', P/'parallel-edge-mapping-candidate-01.json')]:
        q10, details = named_arm10_from_readback(snapshot_path, path)
        diagnostic12 = q10[:5].tolist()+[None]+q10[5:].tolist()+[None]
        comparison['cases'][name] = dict(**details, arm10_model_rad=q10.tolist(), diagnostic12_rad=diagnostic12,
                                         canonical12_verified=None, fk=diagnostic_fk(model, q10))
    baseline = comparison['cases']['tabletop_zero_offset']
    explicit = comparison['cases']['parallel_edge_static_candidate']
    delta = np.asarray(explicit['arm10_model_rad'])-baseline['arm10_model_rad']
    assert np.allclose(delta, np.deg2rad(explicit['zero_offsets_deg']), rtol=0, atol=1e-14)
    previous_q, previous_bindings = named_arm10_from_readback(old_path, P/'parallel-edge-mapping-candidate-01.json')
    assert abs(previous_q[7]-1.556987128317579)<1e-14
    assert abs(previous_q[2]-1.5615901944766848)<1e-14
    previous_ik = json.loads((P/'field-startup-20261002-prepared05-02.ik.json').read_text())
    comparison.update(previous_explicit_path=dict(source=source(P/'field-startup-20261002-prepared05-02.ik.json'),
                            mapping_record=previous_ik['mapping'], reproduced_old_arm10_rad=previous_q.tolist(), bindings=previous_bindings),
                      explicit_minus_tabletop_rad=delta.tolist(), explicit_minus_tabletop_deg=np.rad2deg(delta).tolist(),
                      tcp_delta_model_world_m={side:(np.asarray(explicit['fk']['tcp'][side]['T_model_world_tcp'])[:3,3]-np.asarray(baseline['fk']['tcp'][side]['T_model_world_tcp'])[:3,3]).tolist() for side in ('left','right')},
                      finding='Documented mapping selection difference: explicit static elbow zero candidate versus generic zero-offset tabletop diagnostics; no duplicate homing offset in either path; physical correctness not established',
                      needed_physical_reference='Quantified upper/lower long-edge relative inclinations at an unchanged pose tied to its raw snapshot; prior qualitative parallel-edge confirmation establishes only a static zero candidate')
    write_json(out/'mapping-comparison.json', comparison)
    saved = json.loads((A/'A-os30a.registered-metadata.json').read_text())
    h = saved['header']
    payloads, arrays = {}, {}
    for name in ('color_transport_npy','source_bgr_npy','source_depth_npy','aligned_drgb_npy','aligned_rgb_npy','aligned_xyz_npy','rectify_log_bin'):
        path = Path(saved['artifact_paths'][name])
        payloads[name] = source(path)
        assert payloads[name]['sha256']==saved['artifact_sha256'][name], name
        if path.suffix=='.npy': arrays[name]=np.load(path, allow_pickle=False)
    bgr = arrays['source_bgr_npy']
    assert bgr.shape==(h['color_height'],h['color_width'],3) and bgr.dtype==np.uint8
    assert np.array_equal(bgr[:,:,0],bgr[:,:,1]) and np.array_equal(bgr[:,:,1],bgr[:,:,2])
    mono = bgr[:,:,0].copy()
    cfg_path=P/'night-runbook/board-geometry-config.json'
    cfg=json.loads(cfg_path.read_text())
    parser_path=Path(cfg['rectification_parser']['path'])
    assert source(parser_path)==cfg['rectification_parser']
    spec=importlib.util.spec_from_file_location('saved_payload_parser',parser_path)
    parser=importlib.util.module_from_spec(spec); spec.loader.exec_module(parser)
    rectify=parser.parse_rectify_log(Path(payloads['rectify_log_bin']['path']).read_bytes())
    assert rectify['rectified_left_extent']==[h['color_width'],h['color_height']] and not rectify['scale_enabled'] and not rectify['crop_enabled']
    K=np.asarray(rectify['projection_matrix_left']).reshape(3,4)[:,:3].copy()
    D=np.zeros(5)
    board_cfg=cfg['board']
    board=cv2.aruco.CharucoBoard(tuple(board_cfg['squares_xy']),board_cfg['square_m'],board_cfg['marker_m'],cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    detector=cv2.aruco.CharucoDetector(board)
    corners,ids,marker_corners,marker_ids=detector.detectBoard(mono)
    camera=dict(frame_source=payloads['source_bgr_npy'], metadata=source(A/'A-os30a.registered-metadata.json'), header=h,
                stream=dict(native_shape=list(bgr.shape),registered_shape=list(arrays['aligned_rgb_npy'].shape),depth_shape=list(arrays['source_depth_npy'].shape),native_channel_order='BGR, exact mono replication; use channel 0',derived_image_transform='none; native pixels unchanged'),
                board_geometry=dict(config=source(cfg_path),definition=board_cfg,dictionary='DICT_4X4_50',legacy_pattern=bool(board.getLegacyPattern()),frame='OpenCV board object coordinates; X along board columns, Y along rows, Z normal',object_corner_coordinates_m=board.getChessboardCorners().tolist()),
                prior_detector_source=source(P/'prepare_board_input.py'), rectification=rectify,parser=source(parser_path),
                K=K.tolist(),D=D.tolist(),KD_status='Hypothesis: source is uncropped rectified-left P1 image, D=0, same route as prior prepare_board_input.py; current RGB rectification independently unverified',
                native_corner_ids=[] if ids is None else ids.reshape(-1).tolist(), detected_marker_ids=[] if marker_ids is None else marker_ids.reshape(-1).tolist(),
                T_world_camera=None, T_world_board=None, previous_T_datum_from_board=cfg['T_datum_from_board'],
                previous_board_world_datum_applicable_now=False, old_camera_pose_copied=False, hypotheses=[],
                acquisition_status='incomplete',cleanup_status='timeout',hardware_access_this_work=False,opencv_version=cv2.__version__)
    if ids is not None and len(ids)>=6:
        points,pixels=board.matchImagePoints(corners,ids)
        camera.update(corner_count=len(ids),object_points_m=points.reshape(-1,3).tolist(),observed_corners_uv=pixels.reshape(-1,2).tolist())
        ok,rvec,tvec=cv2.solvePnP(points,pixels,K,D)
        if not ok: raise RuntimeError('same-frame native solvePnP failed')
        iterative=pose_record(points,pixels,K,D,rvec,tvec,'existing solvePnP ITERATIVE')
        generic=cv2.solvePnPGeneric(points,pixels,K,D,flags=cv2.SOLVEPNP_IPPE)
        camera['hypotheses']=[iterative]+[pose_record(points,pixels,K,D,r,t,'IPPE planar alternative '+str(i)) for i,(r,t) in enumerate(zip(generic[1],generic[2]))]
        camera.update(selected_hypothesis_index=0,estimate_scope='CAMERA_FROM_BOARD_CANDIDATE_UNDER_EXPLICIT_KD_AND_BOARD_GEOMETRY',planar_ambiguity='IPPE alternatives retained; reprojection error alone is not independent metric calibration')
        assert iterative['all_corner_depth_positive']
        # Runnable check of frame direction and solver with exact synthetic
        # projections; actual-image residuals remain measured and unmodified.
        exact_pixels=cv2.projectPoints(points,rvec,tvec,K,D)[0]
        ok_check,r_check,t_check=cv2.solvePnP(points,exact_pixels,K,D)
        assert ok_check and pose_record(points,exact_pixels,K,D,r_check,t_check,'synthetic self-check')['reprojection_max_px']<1e-3
        overlay=bgr.copy()
        cv2.aruco.drawDetectedCornersCharuco(overlay,corners,ids,(0,190,0))
        for observed,projected in zip(pixels.reshape(-1,2),iterative['projected_corners_uv']):
            uv=tuple(np.rint(projected).astype(int))
            cv2.drawMarker(overlay,uv,(0,0,255),cv2.MARKER_CROSS,8,1)
            cv2.line(overlay,tuple(np.rint(observed).astype(int)),uv,(255,0,255),1)
        cv2.drawFrameAxes(overlay,K,D,rvec,tvec,.05,2)
        label=f"STATIC14 native PnP candidate | {len(ids)} corners | RMS {iterative['reprojection_rms_px']:.3f}px | max {iterative['reprojection_max_px']:.3f}px"
        cv2.putText(overlay,label,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,255),2,cv2.LINE_AA)
        assert cv2.imwrite(str(out/'board-reprojection.png'),overlay)
        native_roi=np.zeros(mono.shape,np.uint8)
        cv2.fillConvexPoly(native_roi,cv2.convexHull(np.rint(pixels).astype(np.int32)),1)
        depth=arrays['source_depth_npy']
        roi=cv2.resize(native_roi,(depth.shape[1],depth.shape[0]),interpolation=cv2.INTER_NEAREST).astype(bool)
        camera['depth_grid_board_roi_hypothesis']=dict(method='half-resolution native detected-corner convex hull; raw-depth grid correspondence unverified',candidate_pixels=int(roi.sum()),nonzero_count=int(((depth!=0)&roi).sum()),nonzero_fraction=float((depth[roi]!=0).mean()),metric_validity=False)
    else:
        camera.update(corner_count=0 if ids is None else len(ids),selected_hypothesis_index=None,estimate_scope='NOT_ESTIMATED_INSUFFICIENT_CURRENT_NATIVE_CORNERS')
    write_json(out/'camera-candidate.json',camera)
    assert cv2.imwrite(str(out/'native-mono.png'),mono)
    reuse=dict(source_metadata=source(A/'A-os30a.registered-metadata.json'),payloads=payloads,all_seven_payloads_hash_verified=True,
               arrays={name:dict(shape=list(array.shape),dtype=str(array.dtype),nbytes=array.nbytes) for name,array in arrays.items()},
               header=h,acquisition_status='failed/incomplete',cleanup_status='timeout',normal_helper_exit_verified=False,
               failure_reason=saved['failure_reason'],original_hardware_access=True,hardware_access_this_work=False,
               original_final_close_check=source(A/'A-final-close-check.json'),derived_export=str(out/'native-mono.png'),
               learning_dataset_admission=False,depth_units='unverified ZD-table uint16',depth_nonzero_fraction=float((arrays['source_depth_npy']!=0).mean()),
               depth_roi=camera.get('depth_grid_board_roi_hypothesis'),
               cleanup_finding='payload writes completed before closeStream_begin; no closeStream_end; SDK close wait failure, internal SDK root cause not established; no helper replacement or timeout increase')
    write_json(out/'capture-reuse.json',reuse)
    assert portable_model_sha256(model)==model_hash
    print(json.dumps(dict(output=str(out),right_elbow_tabletop=baseline['arm10_model_rad'][7],right_elbow_explicit=explicit['arm10_model_rad'][7],delta_deg=comparison['explicit_minus_tabletop_deg'][7],corners=camera['corner_count'],pose=None if not camera['hypotheses'] else {k:camera['hypotheses'][0][k] for k in ('reprojection_rms_px','reprojection_max_px','tvec_m','all_corner_depth_positive')},checks='arithmetic/history/raw bindings/scalar limits/FK dependencies/solver synthetic projection PASS'),indent=2))


if __name__=='__main__':
    args=argparse.ArgumentParser(description=__doc__)
    args.add_argument('--output-dir',type=Path,required=True)
    main(args.parse_args().output_dir)
