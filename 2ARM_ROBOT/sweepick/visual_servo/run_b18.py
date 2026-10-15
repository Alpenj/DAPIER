"""Reproduce B18 with saved private inputs and the existing A model; no device runtime."""
import argparse
import copy
import io
import json
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'goal_bridge'))
from b16_goal_bridge import load_module,read_json,sha256,write_json
from run_b17 import audit_side_effects
from visual_servo.features import FEATURE_DEFINITIONS,extract_features,a_image_functions
from visual_servo.image_jacobian import estimate_jacobian
from visual_servo.offline_model import AModelImageLoop
from visual_servo.servo_proposal import propose_servo


def jsonl(path):
    with Path(path).open() as f:
        for line in f:
            if line.strip():yield json.loads(line)


def audit_session(item,geometry,checked):
    root=Path(item['path']);run=item['id'];state=read_json(root/'STATE.json')
    session=read_json(root/'SESSION.json');frames=root/'episode/frames.jsonl';joints=root/'episode/joints.jsonl'
    for path in (root/'STATE.json',root/'SESSION.json',frames,joints):checked[str(path)]=sha256(path)
    scenes=[read_json(root/f'looks/01_pregrasp_{k}/SCENE.json') for k in (0,1)]
    selected={x['n'] for scene in scenes for x in scene['wrist']['frames']}
    frame_records={(row['camera'],row['n']):row for row in jsonl(frames)
                   if row['camera']=='left_wrist' and row['n'] in selected}
    right_records=[row for row in jsonl(frames) if row['camera']=='right_wrist']
    left_joint_records=[row for row in jsonl(joints) if row['side']=='left' and row['kind']=='MEASURED']
    a,_=a_image_functions();features=[];observations=[];scene_evidence=[]
    visual=[r for r in session['trail'] if r.get('step')=='visual' and r.get('height')=='01_pregrasp']
    if len(visual)!=2:raise ValueError('explicit two-look pregrasp pair required; no guessed pairing')
    reference=dict(state['jaw_reference'],camera_key='left_wrist',source_kind='REAL',source_sha256=checked[str(root/'STATE.json')])
    for index,(scene,row) in enumerate(zip(scenes,visual)):
        scene_path=root/f'looks/01_pregrasp_{index}/SCENE.json';checked[str(scene_path)]=sha256(scene_path)
        identity=dict(run_id=run,observation_id=f'01_pregrasp_{index}')
        burst=[];ids=[];brackets=[]
        for saved in scene['wrist']['frames']:
            record=frame_records['left_wrist',saved['n']]
            if record['recv_monotonic_s']!=saved['recv_monotonic_s']:
                raise ValueError('scene/recorded frame timestamp identity differs')
            path=root/'episode'/record['file'];checked[str(path)]=sha256(path)
            image=cv2.imread(str(path),cv2.IMREAD_COLOR)
            if image is None:raise ValueError('missing saved frame; no substitution')
            burst.append(image);ids.append(record)
            earlier=[q for q in left_joint_records if q['t_host_s']<=record['recv_host_s']]
            later=[q for q in left_joint_records if q['t_host_s']>=record['recv_host_s']]
            fields=('t_host_s','ticks','kind','source','velocity','moving')
            brackets.append(dict(image_source=str(path),image_sha256=checked[str(path)],
                frame_identity=record,
                preceding_joint_record=None if not earlier else {k:earlier[-1].get(k) for k in fields},
                following_joint_record=None if not later else {k:later[0].get(k) for k in fields},
                association='same-run HOST_WALL times only; bracketing snapshots are not servo sampling/exposure synchronization'))
        middle=len(burst)//2;top_path=scene_path.parent/'top.png';depth_path=scene_path.parent/'depth_mm.npy'
        for path in (top_path,depth_path):checked[str(path)]=sha256(path)
        preceding_right=[r for r in right_records if r['recv_host_s']<=ids[middle]['recv_host_s']]
        right=preceding_right[-1] if preceding_right else None
        right_path=None if right is None else root/'episode'/right['file']
        if right_path is not None:checked[str(right_path)]=sha256(right_path)
        views={key:dict(source_kind='REAL',observation_identity=copy.deepcopy(identity),
            image_identity=None,image_bgr=None,source_sha256=None,availability='UNAVAILABLE')
               for key in ('top','left_wrist','right_wrist')}
        views['left_wrist'].update(image_identity=ids[middle]['file'],image_bgr=burst[middle],availability='SAVED',
            source_sha256=checked[str(root/'episode'/ids[middle]['file'])],frame_burst_bgr=burst,
            frame_burst_identities=ids,gripper_tick=scene['ticks']['left']['gripper'])
        views['top'].update(image_identity=str(top_path),image_bgr=cv2.imread(str(top_path)),availability='SAVED',
            source_sha256=checked[str(top_path)],depth_source_sha256=checked[str(depth_path)],
            camera_info_source=dict(path=str(scene_path),sha256=checked[str(scene_path)],field='K'),
            spatial_context=dict(object=scene['object'],desk_z_m=scene['desk_z'],
                coordinate_frame='A assembled model world via candidate camera/board chain',
                role='existing partial RGB-D object/worktable estimate; no wrist Y target derived'))
        if right_path is not None:
            views['right_wrist'].update(image_identity=right['file'],image_bgr=cv2.imread(str(right_path)),
                availability='SAVED_CONTEXT_ONLY',source_sha256=checked[str(right_path)],frame_identity=right,
                receipt_difference_s=right['recv_host_s']-ids[middle]['recv_host_s'],
                role='latest preceding recorded right frame; not fused into left control')
        clock=run+':HOST_WALL_UNIX';stamp=dict(value=None,unit='s',clock=clock,measurement_time=None)
        times=dict(joint_state={s:copy.deepcopy(stamp) for s in ('left','right')},
            packet_receipt=dict(value=scene['joint_reads']['joints_host_s'],unit='s',clock=clock),
            sensors=[dict(view='left_wrist',value=ids[middle]['recv_host_s'],unit='s',clock=clock,
                state_references=dict(left=copy.deepcopy(stamp)),native_exposure_known=False,
                native_capture=dict(value=ids[middle]['recv_monotonic_s'],clock=run+':HOST_MONOTONIC',kind='HOST_RECEIPT_NOT_EXPOSURE'))])
        observation=dict(observation_identity=identity,source_kind='REAL',q12=scene['q12'],views=views,reference_times=times)
        obj=scene['wrist']['object'];target_u=state['jaw_reference']['midline_px']+row['lateral_target_mm']/obj['mm_per_px']
        target=dict(values_px=dict(u=target_u,v=None),observed_axes=['u'],camera_key='left_wrist',source_kind='REAL',
            feature_definitions=FEATURE_DEFINITIONS,source_sha256=checked[str(root/'SESSION.json')],
            context=dict(run_id=run,nominal_height='01_pregrasp',reference='recorded A Jaw X plus model-assisted plan offset'),
            role='recorded A X target; hidden Y/metric pose unobserved')
        feature=extract_features(observation,jaw_reference=reference,target=target)
        features.append(feature);observations.append(observation)
        poses=geometry.fk(scene['q12']);tool=poses['left']
        tick_q=geometry.command.ticks_to_q12(geometry.mapping,scene['ticks'])
        scene_evidence.append(dict(scene_path=str(scene_path),scene_sha256=checked[str(scene_path)],
            q12_measured=scene['q12'],actual_ticks=scene['ticks'],q12_from_saved_tick_mapping_max_abs=float(np.max(np.abs(tick_q-scene['q12']))),
            tool_pose_from_measured_q12=tool.tolist(),tool_frame='left_cube_grasp in A assembled-model world; metres',
            saved_tool_fk_max_abs_m=float(np.max(np.abs(tool[:3,3]-scene['tool_left']))),
            joint_read_metadata=scene['joint_reads'],frames=brackets,object=scene['object'],
            recorded_wrist_object=obj,recorded_wrist_offset=scene['wrist']['offset'],
            logged_lateral_target_mm=row['lateral_target_mm'],logged_lateral_error_mm=row['lateral_error_mm'],
            authoritative_error_px=row['lateral_error_mm']/obj['mm_per_px'],
            replay_burst_centres_px=[a.wrist_object(frame).get('centre_px') for frame in burst],
            logged_visual_gate_passed=row['ok']))
    before,after=scene_evidence;axis=np.r_[scenes[0]['jaw_axis_xy'],0.0];basis=np.column_stack([axis,[-axis[1],axis[0],0]])
    delta=(np.asarray(after['tool_pose_from_measured_q12'])[:3,3]-np.asarray(before['tool_pose_from_measured_q12'])[:3,3])*1000
    projected=float(delta@axis);error0=before['authoritative_error_px'];error1=after['authoritative_error_px']
    raw_change=scenes[1]['wrist']['object']['centre_px']-scenes[0]['wrist']['object']['centre_px']
    rotation0=np.asarray(before['tool_pose_from_measured_q12'])[:3,:3];rotation1=np.asarray(after['tool_pose_from_measured_q12'])[:3,:3]
    correction=next(c for c in state['correction'] if c.get('along_jaw_mm') and '01_pregrasp_0' in c.get('basis',''))
    pair=dict(id=run,source_kind='REAL_STORED',before=before,after=after,
        command_source=dict(path=str(root/'STATE.json'),sha256=checked[str(root/'STATE.json')],record=correction),
        commanded_tool_move_mm=correction['along_jaw_mm'],actual_model_FK_projected_tool_move_mm=projected,
        actual_model_FK_translation_xyz_mm=delta.tolist(),translation_plane_coordinates_mm=(delta@basis).tolist(),
        translation_plane_basis_model=basis.tolist(),actual_tool_rotation_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(rotation0.T@rotation1)-1)/2,-1,1)))),
        before_image_error_px=error0,after_image_error_px=error1,
        absolute_error_reduction_fraction=1-abs(error1)/abs(error0),error_sign_before=float(np.sign(error0)),error_sign_after=float(np.sign(error1)),
        raw_object_centre_change_px=raw_change,raw_object_centre_response_px_per_mm=raw_change/projected,
        normalized_error_response_px_per_mm=(error1-error0)/projected,
        target_shift_mm=after['logged_lateral_target_mm']-before['logged_lateral_target_mm'],
        scale_mm_per_px=[s['wrist']['object']['mm_per_px'] for s in scenes],
        recorded_response=visual[1].get('response'),
        comparability='SEPARATE_LOCAL_RESPONSE_WITH_HEIGHT_CAMERA_SCALE_AND_TARGET_CHANGE',
        physical_object_stationarity_verified=False,same_object_evidence='same recorded session and red-development-object appearance; not independent physical identity',
        observed_motion_directions=1,constant_context_REAL_Jacobian_identified=False,
        status='OBSERVED_LOCAL_X_ERROR_REDUCTION' if abs(error1)<abs(error0) else 'NO_OBSERVED_X_ERROR_REDUCTION',
        full_task_success=False,real_convergence='REAL_CONVERGENCE_UNVERIFIED',
        source_clock_note='host receipt and joint snapshots only; native exposure/servo measurement mapping unknown')
    pair['encoded_burst_centre_variability_px']=[float(np.std(x['replay_burst_centres_px'],ddof=1)) for x in scene_evidence]
    pair['variability_role']='three encoded saved frames; not a sensor noise bound or independent calibration residual'
    pair['JPEG_replay_minus_recorded_centre_px']=[x['replay_burst_centres_px'][1]-x['recorded_wrist_object']['centre_px'] for x in scene_evidence]
    return pair,features,observations,basis


def audit_other_contexts(inputs,checked):
    excluded=[]
    for item in inputs['sessions']:
        root=Path(item['path']);before=root/'looks/01_pregrasp_1/SCENE.json';after=root/'looks/01a_above_block_0/SCENE.json'
        if after.is_file():
            checked[str(after)]=sha256(after);a=read_json(before);b=read_json(after)
            excluded.append(dict(run_id=item['id'],before_source=str(before),after_source=str(after),
                source_sha256=[sha256(before),checked[str(after)]],classification='CROSS_HEIGHT_AND_CAMERA_POSE',
                tool_translation_model_m=(np.asarray(b['tool_left'])-a['tool_left']).tolist(),
                frame='A assembled model world from measured q12; not independent physical tool measurement',
                pooled_with_pregrasp_pairs=False))
    root=Path(inputs['excluded_session']);path=root/'STATE.json'
    if path.is_file():
        checked[str(path)]=sha256(path);state=read_json(path)
        excluded.append(dict(run_id='excluded_earlier_run',source=str(path),source_sha256=checked[str(path)],
            recorded_corrections=[dict(along_jaw_mm=c.get('along_jaw_mm'),basis=c.get('basis')) for c in state.get('correction',[])],
            object_updates=state.get('object_updates'),classification='YAW_OR_OBJECT_UPDATE_NOT_TRANSLATION_RESPONSE',
            pooled_with_pregrasp_pairs=False))
    return excluded


def replay_B17_features(root,checked):
    path=Path(root)/'B17_REAL_OBSERVABILITY_REPORT.json';checked[str(path)]=sha256(path)
    records=[]
    for sample in read_json(path)['samples']:
        sources=sample['sources'];document=read_json(sources['OBSERVATION.json']['path'])
        identity=dict(run_id='B17_'+sample['sample_id'].split('_')[1],observation_id=sample['sample_id'])
        views={}
        for key in ('top','left_wrist','right_wrist'):
            source=sources[key+'_rgb.png'];checked[source['path']]=sha256(source['path'])
            if checked[source['path']]!=source['sha256']:raise ValueError('B17 original RGB identity differs')
            views[key]=dict(source_kind='REAL',observation_identity=copy.deepcopy(identity),
                image_identity=dict(sample_id=sample['sample_id'],view=key,recorded=document['camera_frames'][key]),
                image_bgr=cv2.imread(source['path']),source_sha256=source['sha256'],
                availability='SAVED',role='held-stage context; no pregrasp task reference inherited')
        alignment=sample['temporal_alignment']
        observation=dict(observation_identity=identity,source_kind='REAL',q12=sample['actual_q12'],views=views,
            reference_times=dict(joint_state=alignment['joint_state'],sensors=[x['receipt'] for x in alignment['cameras']]))
        feature=extract_features(observation)
        feature['task_reference_scope']='held-stage RGB replay only; pregrasp target/reference not transferred'
        records.append(feature)
    return records


def main():
    p=argparse.ArgumentParser();p.add_argument('--inputs',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    counts,forbidden=audit_side_effects();inputs=read_json(args.inputs);checked={str(args.inputs):sha256(args.inputs)}
    legacy=load_module('b18_private_A_geometry',Path(inputs['b16_root'])/'b16_goal_bridge.py');g=legacy.AOfflineGeometry()
    model_manifest=read_json(Path(inputs['b16_root'])/'SOURCE_INPUT_MANIFEST.json')
    checked[model_manifest['external_model']['source']]=model_manifest['external_model']['sha256']
    pairs=[];all_features=[];all_observations=[]
    for item in inputs['sessions']:
        pair,features,observations,basis=audit_session(item,g,checked)
        pairs.append(pair);all_features.extend(features);all_observations.extend(observations)
    write=lambda name,data:write_json(out/name,legacy.plain(data))
    excluded=audit_other_contexts(inputs,checked)
    write('B18_REAL_1D_CONVERGENCE.json',dict(status='OBSERVED_LOCAL_X_ERROR_REDUCTION',pairs=pairs,excluded_contexts=excluded,
        paired_observations=len(pairs),constant_context_2D_pairs=0,real_convergence='REAL_CONVERGENCE_UNVERIFIED',
        scope='recorded one-step local X error reduction, not independent constant-target/2D convergence or complete task success'))
    proposals=[propose_servo(o,f,f['target'],None) for o,f in zip(all_observations,all_features)]
    B17_features=replay_B17_features(inputs['b17_root'],checked)
    write('B18_2D_FEATURE_EVIDENCE.json',dict(status='Y_REFERENCE_UNOBSERVED',samples=all_features,
        B17_held_stage_replays=B17_features,proposals=proposals,REAL_observed_image_axes=['u'],
        REAL_control_dimension_ceiling=1,REAL_validated_Jacobian_dimension=0,
        unavailable='metric/task Y reference; 2D component centroid is not a full object centre or target',
        side_camera_policy='Top spatial context and right wrist provenance retained; no unverified left-control fusion'))
    fit=estimate_jacobian([[pair['actual_model_FK_projected_tool_move_mm']] for pair in pairs],
        [[pair['raw_object_centre_change_px']] for pair in pairs],feature_axes=['u'],
        motion_basis_model=basis[:,:1],context=dict(status='MIXED_HEIGHT_CAMERA_TARGET'),
        feature_definitions=FEATURE_DEFINITIONS,source_sha256={k:v for k,v in checked.items()},comparable_context=False)
    last_root=Path(inputs['sessions'][-1]['path']);scene=read_json(last_root/'looks/01_pregrasp_1/SCENE.json');state=read_json(last_root/'STATE.json')
    seed_ticks=copy.deepcopy(scene['ticks']);seed_ticks['right'].update(state['plan']['command_seed']['right_gripper']['command_ticks'])
    seed=g.command.ticks_to_q12(g.mapping,seed_ticks)
    loop=AModelImageLoop(g,seed,scene['object']['centre_world'],viewport_hw=(240,320),
        context=dict(kind='MODEL_FIXTURE_ONLY',camera='left_wrist_rgb',object='fixed virtual point'),
        source_sha256=dict(model=model_manifest['external_model']['sha256'],kinematic_source=sha256(ROOT/'native/revision/sweepick_kin.py')))
    initial=loop.jacobian(seed,basis,probe_mm=1.0)
    write('B18_JACOBIAN_IDENTIFIABILITY.json',dict(status='REAL_2D_JACOBIAN_UNIDENTIFIED',REAL=fit,
        real_motion_directions=1,Y_reference='Y_REFERENCE_UNOBSERVED',model_only=initial,
        measured_feedback_unchanged=True,model_seed_role=state['plan']['command_seed']['right_gripper'],
        observed_q12=scene['q12'],model_command_seed_q12=seed.tolist(),REAL_MODEL_fused=False,
        explanation='REAL local responses have height/rotation/scale/target changes; no independent second direction. MODEL camera may have opposite response sign.'))
    import visual_servo.test_visual_servo as tests
    tests.MODEL_FIXTURE=(loop,basis,seed)
    class StoredRealTests(unittest.TestCase):
        def test_original_B17_frames_get_features_without_inherited_task_references(self):
            self.assertEqual(len(B17_features),10)
            self.assertTrue(all('Y_REFERENCE_UNOBSERVED' in f['missing_evidence'] for f in B17_features))
            self.assertTrue(all(f['observed_image_axes']==[] and f['executable'] is False for f in B17_features))
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromModule(tests),
                             unittest.defaultTestLoader.loadTestsFromTestCase(StoredRealTests)])
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    (out/'B18_UNITTEST.log').write_text(stream.getvalue())
    counts['forbidden_modules_loaded']=[n for n in sys.modules if any(n==x or n.startswith(x+'.') for x in forbidden)]
    changed=[p for p,digest in checked.items() if sha256(p)!=digest]
    ok=(result.wasSuccessful() and not result.skipped and not changed and not counts['forbidden_modules_loaded']
        and not counts['forbidden_attempts'] and all(counts[k]==0 for k in
        ('motor_commands_sent','executor_calls','optimizer_updates','mj_step_calls','device_open_attempts')))
    write('B18_OFFLINE_SERVO_REGRESSION.json',dict(status='PASS' if ok else 'FAIL',B18_tests_run=result.testsRun,
        failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),model_cases=tests.MODEL_EVIDENCE,
        side_effect_checks=counts,input_source_changes=changed,model_IK_calls=loop.ik_calls,model_path_checks=loop.path_checks,
        REAL_convergence='REAL_CONVERGENCE_UNVERIFIED',B17_regression='run unchanged goal_bridge/run_b17.py separately',
        implementation_sha256={p.name:sha256(p) for p in Path(__file__).parent.glob('*.py')},
        CPU_environment=dict(python=sys.version.split()[0],numpy=np.__version__,opencv=cv2.__version__,mujoco=g.mj.__version__)))
    write('SOURCE_INPUT_MANIFEST_PRIVATE.json',checked)
    print(dict(status='PASS' if ok else 'FAIL',tests=result.testsRun,pairs=len(pairs),side_effects=counts))
    return 0 if ok else 1


if __name__=='__main__':raise SystemExit(main())
