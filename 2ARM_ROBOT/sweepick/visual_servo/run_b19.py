"""File-only B19 evidence audit and CPU benchmark; never connects to A or a device."""
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
from visual_servo.features import a_image_functions,observation_binding,vision_functions
from visual_servo.a_shadow import left_hold_from_A_record
from visual_servo.image_jacobian import estimate_jacobian
from visual_servo.offline_model import AModelImageLoop
from visual_servo.performance import benchmark,roi_object_feature,scalar_response_comparison
from visual_servo.right_relative import RightRelativeServoProposal
from visual_servo.run_b18 import audit_session


def check(path,checked):
    path=Path(path);checked[str(path)]=sha256(path);return path


def saved_observation(folder,run,checked):
    path=check(folder/'OBSERVATION.json',checked);d=read_json(path)
    if d['source_kind']!='REAL' or not np.array_equal(d['q12'],d['observation']['q12']):
        raise ValueError('stored REAL packet/q12 provenance mismatch')
    identity=dict(run_id=run,observation_id=folder.name);views={};sensors=[]
    clock=run+':HOST_MONOTONIC';unknown=dict(value=None,unit='s',clock=clock,measurement_time=None)
    for key in ('top','left_wrist','right_wrist'):
        image_path=check(folder/d['files'][key]['rgb'],checked)
        image=cv2.imread(str(image_path),cv2.IMREAD_COLOR)
        if image is None:raise ValueError('saved view missing; no substitution')
        camera=d['camera_frames'][key]
        views[key]=dict(image_bgr=image,image_identity=dict(stream=key,n=camera['n'],file=image_path.name),
            source_kind='REAL',source_sha256=checked[str(image_path)],observation_identity=copy.deepcopy(identity),
            recorded_camera=copy.deepcopy(camera))
        sides=('left',) if key=='left_wrist' else ('right',) if key=='right_wrist' else ('left','right')
        sensors.append(dict(view=key,value=camera.get('recv_monotonic_s'),unit='s',clock=clock,
            kind='HOST_CAMERA_RECEIPT_NOT_EXPOSURE',state_references={s:copy.deepcopy(unknown) for s in sides},
            native_exposure_known=False))
    packet=d['actual_feedback'];obs=dict(observation_identity=identity,source_kind='REAL',q12=d['q12'],views=views,
        reference_times=dict(joint_state={s:copy.deepcopy(unknown) for s in ('left','right')},sensors=sensors,
            packet_receipt=dict(value=packet.get('t_bus_monotonic'),unit='s',clock=clock)),
        model_assisted_relative_geometry=copy.deepcopy(d['observation'].get('model_assisted_relative_geometry')))
    return d,obs,path


def rotation_deg(before,after):
    return float(np.degrees(np.arccos(np.clip((np.trace(before[:3,:3].T@after[:3,:3])-1)/2,-1,1))))


def motion_evidence(session,geometry,checked):
    history=check(session/'MOTION_HISTORY.jsonl',checked);rows=[]
    for line in history.read_text().splitlines():
        row=json.loads(line)
        if row.get('kind')!='terminal' or row.get('side')!='right':continue
        start=row.get('actual_start_ticks');end=row.get('last_actual_ticks');goal=row.get('last_applied_goal')
        if not all(isinstance(x,dict) for x in (start,end,goal)):continue
        source=read_json(session/'decision_observations/000005_RECEIVE_CONTACT/OBSERVATION.json')['actual_feedback']['ticks']
        q=[]
        for ticks in (start,end,dict(start,**{k:v for k,v in goal.items() if k!='gripper'})):
            snapshot=copy.deepcopy(source);snapshot['right']=ticks
            q.append(geometry.command.ticks_to_q12(geometry.mapping,snapshot))
        poses=[geometry.fk(value)['right'] for value in q]
        rows.append(dict(name=row['name'],history_record=row,
            actual_candidate_FK_translation_mm=((poses[1][:3,3]-poses[0][:3,3])*1000).tolist(),
            commanded_arm_Goal_candidate_FK_translation_mm=((poses[2][:3,3]-poses[0][:3,3])*1000).tolist(),
            actual_candidate_FK_rotation_deg=rotation_deg(poses[0],poses[1]),
            command_gripper_basis='measured start opening retained solely for arm-target FK; raw gripper Goal field is not reused',
            geometry_role='existing A model/candidate calibration FK; not measured Cartesian ground truth'))
    return rows


def latency_profile(session,checked):
    records=[]
    for path in sorted((session/'decision_observations').glob('*/PRODUCTION_TIMINGS.json')):
        records.append(dict(stage=path.parent.name,source=str(check(path,checked)),timings_s=read_json(path)))
    plan_path=check(session/'RECEIVE_PLANNER_TIMINGS.json',checked)
    internal_path=check(session/'RECEIVE_PLANNER_RESULT.json',checked)
    def unknown(reason):return dict(status='UNKNOWN',duration_s=None,reason=reason)
    stages=dict(camera_acquisition=unknown('native exposure and acquisition start/end are not recorded'),
        camera_receipt=dict(status='RECORDED',duration_s=None,meaning='per-view host receipt timestamps; no exposure mapping'),
        feature_extraction=unknown('PICK feature duration is absent; held/pad producer spans below are combined workloads'),
        feature_error_computation=unknown('no isolated error-computation duration'),
        Jacobian_calculation=unknown('no validated REAL relative Jacobian calculation or timing'),
        IK_planning=dict(status='RECORDED_NESTED_SCOPES',outer=read_json(plan_path),
            planner_internal_s=read_json(internal_path).get('planning_elapsed_s'),
            meaning='nested call/preparation/internal scopes, not additive stages'),
        collision_certification=unknown('certificate trajectory duration is a planned motion duration, not cert CPU time'),
        actuator_command_duration=unknown('no isolated measured command duration; planned trajectory and feedback row spans differ'),
        measured_settling=unknown('settled flags/read counts are recorded without isolated settling start/end'),
        fresh_image_reacquisition=dict(status='PARTIALLY_RECORDED',duration_s=None,
            meaning='native decision frame_wait_s is the three-view fresh-frame wait, not exposure latency or complete PICK look time'))
    return dict(status='RECORDED_SCOPES_AND_UNKNOWN_COMPONENTS',recorded_stages=stages,
        native_observation_production=records,
        event_gap_interpretation='pre/post-image receipt gap includes uninstrumented work; never CPU time or motor duration',
        native_duration_clock='perf_counter/monotonic as verified in frozen A sources',
        physical_time_savings_measured=False)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--inputs',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    counts,forbidden=audit_side_effects();cv2.setNumThreads(1)
    inputs=read_json(args.inputs);checked={str(args.inputs):sha256(args.inputs)}
    session=Path(inputs['session']);a_source=Path(inputs['a_source']);run=session.parent.name
    frozen_sources=read_json(check(inputs['a_source_manifest'],checked))
    for p in a_source.glob('*.py'):
        check(p,checked)
        if checked[str(p)]!=frozen_sources[p.name]['sha256']:raise ValueError('frozen A source SHA mismatch')
    legacy=load_module('b19_private_geometry',Path(inputs['b16_root'])/'b16_goal_bridge.py');g=legacy.AOfflineGeometry()
    model=read_json(Path(inputs['b16_root'])/'SOURCE_INPUT_MANIFEST.json')['external_model'];check(model['source'],checked)
    if checked[model['source']]!=model['sha256']:raise ValueError('existing A model SHA mismatch')
    write=lambda name,data:write_json(out/name,legacy.plain(data))
    real=[];proposals=[];documents=[];observations=[]
    for folder in sorted((session/'decision_observations').iterdir()):
        if not (folder/'OBSERVATION.json').is_file():continue
        d,obs,path=saved_observation(folder,run,checked)
        original=d['observation']
        held=left_hold_from_A_record(obs,original,checked[str(path)])
        proposal=RightRelativeServoProposal(d['q12'],obs,held,{},None,a_source=a_source)
        surface=proposal['feature_evidence']['visible_approach_region']
        mapped=g.command.ticks_to_q12(g.mapping,d['actual_feedback']['ticks'])
        real.append(dict(stage=folder.name,observation_source=str(path),observation_sha256=checked[str(path)],
            source_kind='REAL',frame_identity=d['camera_frames']['right_wrist'],actual_q12=d['q12'],
            tick_mapping_q12_max_abs_difference=float(np.max(np.abs(mapped-d['q12']))),
            feature_evidence=proposal['feature_evidence'],recorded_wrist_surface=original.get('wrist_rgb_surfaces',{}).get('right_wrist'),
            recorded_visible_face_relation=original.get('visible_face_relation'),
            recorded_model_geometry=original.get('model_assisted_relative_geometry'),
            recorded_relative_alignment=original.get('relative_alignment'),object_component_visible=surface['visible']))
        proposals.append(proposal);documents.append(d);observations.append(obs)
    movements=motion_evidence(session,g,checked)
    paired=[]
    for before,after in zip(real,real[1:]):
        p0=g.fk(before['actual_q12'])['right'];p1=g.fk(after['actual_q12'])['right']
        f0=before['feature_evidence'];f1=after['feature_evidence'];c0=f0['object_component_centroid_uv'];c1=f1['object_component_centroid_uv']
        paired.append(dict(before=before['stage'],after=after['stage'],
            actual_candidate_FK_translation_mm=((p1[:3,3]-p0[:3,3])*1000).tolist(),
            actual_candidate_FK_rotation_deg=rotation_deg(p0,p1),
            raw_component_centroid_change_px={a:None if c0[a] is None or c1[a] is None else c1[a]-c0[a] for a in ('u','v')},
            physical_feature_track_verified=False,observed_pad_relative_feature=False,valid_REAL_Jacobian_pair=False,
            exclusion='no physically identified same object/Pad feature; zero excitation, target/pose changes or missing target remain explicit'))
    write('B19_RIGHT_FEATURE_EVIDENCE.json',dict(status='PARTIAL_2D_CURRENT_RGB_EVIDENCE',samples=real,
        optical_evidence=read_json(check(inputs['optical_evidence'],checked)),
        native_source_analysis=read_json(check(inputs['source_analysis'],checked)),
        physically_identified_pad_reference_count=0,relative_task_axes_observed=[],
        actual_A_outcome=read_json(check(session.parent/'RUN_RECORD.json',checked))))
    write('B19_RIGHT_JACOBIAN_IDENTIFIABILITY.json',dict(status='INSUFFICIENT_PAIRED_OBSERVATIONS',
        candidate_consecutive_pairs=paired,valid_relative_pairs=0,independent_relative_directions=0,
        REAL_Jacobian=None,rank=None,condition_number=None,measurement_noise_px=None,
        right_X_response='UNIDENTIFIED_RELATIVE_TO_OBSERVED_PAD',Y_depth_rotation='UNOBSERVED_AS_RELATIVE_TASK_ERRORS',
        actual_and_commanded_motion=movements,REAL_model_response_fused=False))
    write('B19_RIGHT_SERVO_PROPOSAL.json',dict(status='NEEDS_EVIDENCE',proposals=proposals,
        observed_control_dimensions=0,estimated_tool_correction=None,
        executable=False,motor_write_authorized=False))
    pairs=[];left_observations=[];left_features=[]
    for item in inputs['left_sessions']:
        pair,features,obs,_=audit_session(item,g,checked)
        pairs.append(pair);left_features.extend(features);left_observations.extend(obs)
    comparisons=scalar_response_comparison(pairs)
    a,_=a_image_functions(a_source);roi_rows=[];precision=[]
    for obs,features in zip(left_observations,left_features):
        view=obs['views']['left_wrist'];frame_rows=[]
        for image in view['frame_burst_bgr']:
            full=a.wrist_object(image);roi,metadata=roi_object_feature(image,a)
            tips=a.jaw_tips(image);offset=a.wrist_offset(full,features['jaw']['reference'],tips,view['gripper_tick'])
            roi_offset=a.wrist_offset(roi,features['jaw']['reference'],tips,view['gripper_tick'])
            roi_rows.append(dict(observation_identity=obs['observation_identity'],baseline=full,ROI=roi,ROI_metadata=metadata,
                baseline_A_contract_confirmed=offset.get('reference_confirmed_in_this_frame') is True,
                ROI_A_contract_confirmed=roi_offset.get('reference_confirmed_in_this_frame') is True,
                baseline_image_only_reference_confirmed=bool({'left','right'}&set(offset.get('confirmed_by',[]))),
                ROI_image_only_reference_confirmed=bool({'left','right'}&set(roi_offset.get('confirmed_by',[])))))
            frame_rows.append(dict(centre_px=full.get('centre_px'),width_px=full.get('width_px'),tips=tips,offset=offset))
        centres=[r['centre_px'] for r in frame_rows if r['centre_px'] is not None]
        precision.append(dict(observation_identity=obs['observation_identity'],frames=frame_rows,
            encoded_burst_centre_std_px=float(np.std(centres,ddof=1)) if len(centres)>1 else None,
            jaw_reference=features['jaw']['reference'],reference_uncertainty='no independent physical Jaw/TCP error bound',
            pixel_to_TCP_accuracy_converted=False,variation_is_sensor_noise_bound=False))
    comparisons.update(feature_ROI_replay=roi_rows,precision=precision,original_left_pair_diagnostics=pairs,
        visual_reference_loss=dict(samples=len(roi_rows),
            baseline_image_only_unavailable=sum(not r['baseline_image_only_reference_confirmed'] for r in roi_rows),
            ROI_image_only_unavailable=sum(not r['ROI_image_only_reference_confirmed'] for r in roi_rows),
            baseline_A_contract_unconfirmed=sum(not r['baseline_A_contract_confirmed'] for r in roi_rows),
            ROI_A_contract_unconfirmed=sum(not r['ROI_A_contract_confirmed'] for r in roi_rows),
            scope='saved frames only; A confirmation may use unchanged measured opening, separately from image-tip matching'),
        physical_TCP_accuracy=None,physical_time_saved_s=None)
    write('B19_SPEED_ACCURACY_COMPARISON.json',comparisons)
    image=left_observations[-1]['views']['left_wrist']['image_bgr'];target=left_features[-1]['target']['values_px']['u']
    numeric_fit=lambda:estimate_jacobian([[1],[2]],[[2],[4]],feature_axes=['u'],
        motion_basis_model=[[1],[0],[0]],context={},feature_definitions={'u':'TIMING_FIXTURE'},
        source_sha256={},source='TEST_FIXTURE',comparable_context=True)
    cpu=dict(module_initialization=benchmark(lambda:a_image_functions(a_source),repeats=7),
        baseline_object_and_Jaw=benchmark(lambda:(a.wrist_object(image),a.jaw_tips(image),a.jaw_reference(image)),repeats=31),
        ROI_object_and_Jaw=benchmark(lambda:(roi_object_feature(image,a),a.jaw_tips(image),a.jaw_reference(image)),repeats=31),
        feature_error=benchmark(lambda:a.wrist_object(image)['centre_px']-target,repeats=31),
        isolated_error_arithmetic=benchmark(lambda:left_features[-1]['values_px']['u']-target,repeats=101),
        numerical_Jacobian_fixture=benchmark(numeric_fit,repeats=31),
        right_incomplete_shadow_request=benchmark(lambda:RightRelativeServoProposal(
            observations[4]['q12'],observations[4],proposals[4]['reference_provenance']['object_target_association'],{},None,a_source=a_source),repeats=7),
        candidate_FK=benchmark(lambda:g.fk(observations[4]['q12']),repeats=31))
    _,right_geometry,right_functions=vision_functions(a_source)
    right_image=observations[4]['views']['right_wrist']['image_bgr']
    right_surface=right_functions.wrist_object_surfaces(right_image)
    cpu['right_RGB_surface']=benchmark(lambda:right_functions.wrist_object_surfaces(right_image),repeats=31)
    def right_graph():
        cv2.setRNGSeed(0)
        return right_geometry.observed_face_graph(right_image,right_surface['_mask'])
    cpu['right_RGB_face_graph']=benchmark(right_graph,repeats=7)
    per_observation=[]
    for obs in left_observations:
        saved=obs['views']['left_wrist']['image_bgr']
        per_observation.append(dict(observation_identity=obs['observation_identity'],
            baseline_object_and_Jaw=benchmark(lambda:(a.wrist_object(saved),a.jaw_tips(saved),a.jaw_reference(saved)),repeats=31),
            ROI_object_and_Jaw=benchmark(lambda:(roi_object_feature(saved,a),a.jaw_tips(saved),a.jaw_reference(saved)),repeats=31)))
    old=read_json(Path(inputs['b18_root'])/'B18_JACOBIAN_IDENTIFIABILITY.json')
    seed=np.asarray(old['model_command_seed_q12']);basis=np.asarray(old['model_only']['jacobian']['motion_basis_model'])
    point=read_json(Path(inputs['left_sessions'][1]['path'])/'looks/01_pregrasp_1/SCENE.json')['object']['centre_world']
    loop=AModelImageLoop(g,seed,point,viewport_hw=(240,320),context={'kind':'B19_CPU_TIMING_MODEL_FIXTURE'},source_sha256={})
    _,pose=loop.observe(seed);decoded=g.tm.decode_command(seed,g.model)
    def IK():return loop.teacher.plan.solve(loop.base.qpos,decoded,'left',pose[:3,3]+basis[:,0]*.001,
        pose[:3,0],[pose[:3,2]],loop.teacher.cfg)
    goal,info=IK()
    if goal is None:raise ValueError('existing model-only timing fixture IK failed')
    goal[[5,11]]=decoded[[5,11]]
    path_check=lambda:loop.teacher.plan.path_collisions(loop.base.qpos,decoded,goal,
        block_qpos=loop.base.qpos[loop.teacher.cm.block_qpos:loop.teacher.cm.block_qpos+7])
    if path_check() is not None:raise ValueError('existing model-only timing fixture collides')
    cpu['existing_A_pure_IK_model_fixture']=benchmark(IK,repeats=5)
    cpu['existing_A_pure_path_collision_model_fixture']=benchmark(path_check,repeats=5)
    latency=latency_profile(session,checked);latency.update(CPU_benchmarks=cpu,CPU_left_per_observation=per_observation,
        CPU_environment=dict(python=sys.version.split()[0],numpy=np.__version__,opencv=cv2.__version__,
            mujoco=g.mj.__version__,platform=sys.platform,OpenCV_threads=cv2.getNumThreads()),
        CPU_scope='single process, same interpreter and inputs; current CPU timings are separate from historical REAL timings',
        model_timing_fixture='existing B18 seed and model; 1mm numerical IK target only, no hardware write')
    write('B19_LATENCY_PROFILE.json',latency)
    import visual_servo.test_b19 as tests
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(tests))
    (out/'B19_UNITTEST.log').write_text(stream.getvalue())
    changed=[p for p,h in checked.items() if sha256(p)!=h]
    counts['forbidden_modules_loaded']=[n for n in sys.modules if any(n==x or n.startswith(x+'.') for x in forbidden)]
    success=(result.wasSuccessful() and not result.skipped and not changed and not counts['forbidden_attempts']
        and not counts['forbidden_modules_loaded'] and all(counts[k]==0 for k in
        ('motor_commands_sent','executor_calls','optimizer_updates','mj_step_calls','device_open_attempts')))
    write('B19_REGRESSION.json',dict(status='PASS' if success else 'FAIL',B19_tests=result.testsRun,
        failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),side_effect_checks=counts,
        input_source_changes=changed,REAL_samples=len(real),REAL_proposals_with_correction=sum(
            p['estimated_tool_correction'] is not None for p in proposals),
        implementation_sha256={p.name:sha256(p) for p in Path(__file__).parent.glob('*.py')},
        existing_59_tests='run B17 and B18 regressions separately on this checkout',
        response_validation='axis-wise direction products and whole image-error norm; no physical threshold added'))
    write('SOURCE_INPUT_MANIFEST_PRIVATE.json',checked)
    print(dict(status='PASS' if success else 'FAIL',B19_tests=result.testsRun,REAL_samples=len(real),side_effects=counts))
    return 0 if success else 1


if __name__=='__main__':raise SystemExit(main())
