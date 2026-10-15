"""File-only B17 verification with operator-supplied private evidence locations."""
import argparse
import copy
import hashlib
import io
import os
from pathlib import Path
import sys
import unittest
import zipfile

import numpy as np

from b16_goal_bridge import (GoalProposalBridge, b15_times, load_module, read_json,
                             sha256, temporal_alignment, write_json)
from stored_replay import replay


def audit_side_effects():
    os.environ['MUJOCO_GL'] = 'disable'
    counts = dict(motor_commands_sent=0, executor_calls=0, optimizer_updates=0,
                  mj_step_calls=0, device_open_attempts=0, forbidden_attempts=[])
    forbidden = ('serial','scservo_sdk','dynamixel_sdk','rclpy','isaacsim','omni',
                 'sweepick_move_a','sweepick_real_exec','torch')

    def audit(event,args):
        if event == 'open' and args and isinstance(args[0],str) and args[0].startswith(
                ('/dev/tty','/dev/serial','/dev/video','/dev/bus/usb')):
            counts['device_open_attempts'] += 1
            raise RuntimeError('stored evidence verification cannot open robot devices')
        if ((event == 'import' and any(args[0] == x or args[0].startswith(x+'.') for x in forbidden))
                or event in ('subprocess.Popen','socket.connect')):
            counts['forbidden_attempts'].append(event)
            raise RuntimeError('stored evidence verification cannot start an executor or policy')
    sys.addaudithook(audit)
    import mujoco

    def no_step(*args,**kwargs):
        counts['mj_step_calls'] += 1
        raise RuntimeError('only private model FK/IK is in scope')
    mujoco.mj_step = no_step
    return counts,forbidden


def facing_faces(object_pose,camera_pose,half):
    """Model-facing faces only; visibility through grippers is unknown."""
    output=[]
    for axis in range(3):
        for sign in (-1,1):
            normal=object_pose[:3,axis]*sign
            centre=object_pose[:3,3]+normal*half[axis]
            dot=float(normal@(camera_pose[:3,3]-centre))
            output.append(dict(face=f'object_local_{"xyz"[axis]}{sign:+d}',
                facing_dot_m=dot,nominal_front_facing=dot>0,
                coordinate_frame='object axes transformed into assembled dual model world',
                source_role='existing object SEARCH_PRIOR + candidate camera transform',
                occlusion_checked=False,actual_face_identity_verified=False))
    return output


def dataset_readiness(root):
    folder=root/'inputs/dataset'
    names=('R6_TRAINING_MANIFEST.json','R7_TRAINING_MANIFEST.json',
           'R8_TRAINING_MANIFEST.json','R8_SOURCE_JOIN_AUDIT.json')
    manifests={n:read_json(folder/n) for n in names}
    r8=manifests['R8_TRAINING_MANIFEST.json']
    rows=[]
    for i,episode in enumerate(r8['episodes']):
        rows.append(dict(anonymous_episode=f'episode_{i+1:02}',recorded_episode_id=episode['id'],
            recorded_derived_path=episode['path'],recorded_raw_archive=episode.get('raw_archive'),
            split=episode['split'],action_owner=episode.get('action_owner'),
            action_contract=episode.get('action_contract'),shared_parent_group=None,
            grouping_status='UNRESOLVED; setup/condition names are not parent identity'))
    return dict(status='INSUFFICIENT_DATA',split=dict(train=15,val=0,test=0),split_changed=False,
        goal_pose_labels_created=0,optimizer_updates=0,
        sources={n:dict(path=str(folder/n),sha256=sha256(folder/n)) for n in names},
        episode_counts={n:len(manifests[n]['episodes']) for n in names[:3]},episodes=rows,
        source_denominator=copy.deepcopy(r8['source_denominator']),
        source_join_audit=manifests['R8_SOURCE_JOIN_AUDIT.json'],
        teacher_intention='joint/jaw intentions with explicit source joins; not desired object-pose labels',
        actual_state='q_actual and SIM object state are outcomes, not desired GoalPose labels',
        original_lineage_audit='recorded episode/source-run and admission links inspected directly; '
            'no parent grouping inferred from A_HOME, U1/U2, condition_group, or source_home',
        limitations=['shared seed/setup/correction parent identity incomplete',
                     'no independent validation/test split','REAL physical goal/pad identity unverified'])


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--b16-root',type=Path,required=True)
    p.add_argument('--replay-manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();root=args.b16_root.resolve();out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    counts,forbidden=audit_side_effects()
    manifest=read_json(args.replay_manifest)
    original_manifest=read_json(root/'SOURCE_INPUT_MANIFEST.json')
    checked={}
    for rel,entry in original_manifest.items():
        path=Path(entry['source']) if rel=='external_model' else root/rel
        actual=sha256(path)
        if actual != entry['sha256']:raise ValueError('frozen B16 source mismatch: '+rel)
        checked[str(path)]=actual
    source=Path(manifest['a_vision_source'])
    for name in ('sweepick_261009_offsite_relative_features.py','right_wrist_relative_geometry.py','sweepick_pick01.py'):
        checked[str(source/name)]=sha256(source/name)
    legacy=load_module('b17_frozen_b16',root/'b16_goal_bridge.py')
    geometry=legacy.AOfflineGeometry()
    records=read_json(root/'validation_R3/NORMALIZED_OBSERVATION_RECORDS.json')
    b15=read_json(root/'inputs/B15/B15_SHADOW_BRIDGE_REPORT.json')
    for record,prediction in zip(records[:3],b15['prediction_results']):
        record['reference_times']=b15_times(prediction)
    folder=root/'inputs/A_web_20261014'
    request=read_json(folder/'OBSERVATION_MOVE_REQUEST.json')
    observation=read_json(folder/'RECEIVE_CONTACT_OBSERVATION.json')
    planner_input=read_json(folder/'RECEIVE_PLANNER_INPUT.json')
    records[-1]['model_preview_context']=dict(available=True,role='MODEL_PRIOR',
        actual_q12_bound_to_existing_request=True,object_pose_role='SEARCH_PRIOR',
        source_sha256=sha256(folder/'OBSERVATION_MOVE_REQUEST.json'))
    a_preview=geometry.preview_existing_request(request,observation,planner_input)
    def preview(targets):
        if targets != request['candidate']['targets']:raise ValueError('unexpected view target')
        return dict(ik_solution_exists=all(x['ik_solution_exists'] for x in a_preview['targets']),
                    collision_result='CLEAR' if all(x['collision_result']=='CLEAR' for x in a_preview['targets']) else 'NONCLEAR',
                    status=a_preview['status'],existing_A_preview=a_preview)
    proposals=[GoalProposalBridge(geometry.a_fk).propose(r,preview) for r in records]
    write_json(out/'GOAL_PROPOSALS.json',proposals)
    real=[replay(x['folder'],source,x['sample_id'],x['clock']) for x in manifest['samples']]
    checked.update({entry['path']:entry['sha256'] for sample in real for entry in sample['sources'].values()})
    archive=[]
    if manifest.get('archive_oct13'):
        path=Path(manifest['archive_oct13']);checked[str(path)]=sha256(path)
        with zipfile.ZipFile(path) as z:
            for prefix in ('looks/01a_above_block_0/','looks/02_descended/'):
                members={name:dict(archive_member=prefix+name,sha256=hashlib.sha256(z.read(prefix+name)).hexdigest())
                         for name in ('top.png','depth_mm.npy','SCENE.json')}
                archive.append(dict(archive_path=str(path),archive_sha256=checked[str(path)],members=members,
                    final_status='NOT_A_HELD_OBJECT_LABEL',
                    reason='saved pre-grasp frames exist; SCENE.object is null, no held/pad identity is fabricated'))
    write_json(out/'B17_REAL_OBSERVABILITY_REPORT.json',dict(status='NEEDS_OBSERVATION',samples=real,
        early_archive_availability=archive,missing_images_synthesized=0,physical_success=False,
        algorithm_scope='unchanged A feature functions; existing search/robot masks, RGB segmentation and partial-depth fits'))
    temporal=[dict(source=r['source'],alignment=r['temporal_alignment']) for r in proposals]
    write_json(out/'B17_TEMPORAL_ALIGNMENT_REPORT.json',dict(status='DIAGNOSTICS_SEPARATED_FROM_GOAL_GATING',
        samples=temporal,stored_replay=[dict(sample_id=s['sample_id'],alignment=s['temporal_alignment']) for s in real],
        camera_references=dict(left_wrist=['left'],right_wrist=['right'],top=['left','right']),
        thresholds_added=0,normal_async_is_goal_failure=False,earlier_camera_receipt_is_goal_failure=False,
        actual_joint_measurement_stamps='not supplied by these saved host-receipt records',
        exposure_clock_mapping='UNKNOWN; SDK timestamp is not a verified host exposure timestamp'))
    cal=read_json(manifest['wrist_calibration']);checked[manifest['wrist_calibration']]=sha256(manifest['wrist_calibration'])
    object_pose=np.asarray(observation['held_object']['search_prior']['pose_model'])
    contact=read_json(manifest['samples'][-1]['folder']+'/OBSERVATION.json')
    top_pose=np.asarray(contact['T_world_camera'])
    half=np.asarray(planner_input['snapshot']['object']['geometry']['half_extents_m'])
    view_rows=[]
    baseline_camera=geometry.fk(request['plan']['actual_start_q12'])['right']@np.asarray(cal['T_tool_camera'])
    baseline_faces=facing_faces(object_pose,baseline_camera,half)
    for row in a_preview['targets']:
        camera_pose=np.asarray(row['T_model_desired_tool'])@np.asarray(cal['T_tool_camera'])
        view_rows.append(dict(index=row['index'],role='MODEL_VIEW_CANDIDATE',
            right_wrist_model_facing_faces=facing_faces(object_pose,camera_pose,half),
            top_model_facing_faces=facing_faces(object_pose,top_pose,half),
            top_view_change='object stays with unchanged left arm; only right-arm occlusion may change',
            left_wrist_view_change='left tool/object relation unchanged; no new view benefit established',
            actual_post_target_frame=None,actual_information_gain_proven=False,
            physical_pad_surface_exposure_proven=False,executable=False))
        camera_from_object=np.linalg.inv(camera_pose)@object_pose
        view_rows[-1]['model_object_centre_camera_m']=camera_from_object[:3,3].tolist()
        view_rows[-1]['baseline_front_faces']=[x['face'] for x in baseline_faces if x['nominal_front_facing']]
    old=read_json(root/'B16_REAL_GOAL_PREVIEW_REPORT.json')['A_EXISTING_GOAL']
    agreement=[float(np.max(np.abs(np.asarray(new['canonical_waypoints'])-before['canonical_waypoints'])))
               for new,before in zip(a_preview['targets'],old['targets'])]
    view=dict(status='MODEL_VIEW_CANDIDATE',targets=view_rows,A_existing_pure_preview=a_preview,
        B16_waypoint_max_abs_differences=agreement,
        optical_model_source=dict(path=manifest['wrist_calibration'],sha256=checked[manifest['wrist_calibration']],
            status=cal['validation_report']['status'],live_execution_permitted=cal['live_execution_permitted']),
        observed_evidence='saved pre-target RGB-D/2D wrist contours; no measured post-target information gain',
        target_generation='only the two existing A CURRENT_TOOL targets, unchanged',
        measured_goal=False,physical_success=False,executable=False,motor_write_authorized=False)
    if manifest.get('historical_view_outcome'):
        path=manifest['historical_view_outcome'];checked[path]=sha256(path);historical=read_json(path)
        view['recorded_outcome']=dict(source=path,sha256=checked[path],
            observe=historical['observe'],first_error=historical['first_error'],
            extra_view_approval=historical['extra_view_approval'],
            interpretation='historical approval file alone is not execution evidence; recorded native stop preceded approval consumption')
    write_json(out/'B17_VIEW_CANDIDATE_COMPARISON.json',legacy.plain(view))
    readiness=dataset_readiness(root)
    write_json(out/'DATASET_LINEAGE_AUDIT_PRIVATE.json',readiness)
    (out/'B17_DATASET_READINESS.md').write_text(
        '# B17 dataset readiness\n\nINSUFFICIENT_DATA. Train 15 / Val 0 / Test 0 is unchanged.\n\n'
        'The original R6/R7/R8 manifests contain 11/13/15 episodes. Recorded episode, raw archive, '
        'source-run and admission links were inspected directly. Setup labels and correction scope '
        'do not establish shared seed or parent identity. Parent grouping remains unresolved.\n\n'
        'Teacher joint/jaw intentions and their exact source joins are distinct from actual q/object states. '
        'Neither is a typed desired object-pose label. No GoalPose label or split was created.\n\n'
        'Stored REAL contours and partial depth planes do not verify metric object centre, full orientation '
        'or the receiving inner-pad identity. The data cannot establish a goal predictor or physical success. '
        'C10/R13 weights and normalization remain frozen.\n')
    os.environ['B16_PRIVATE_ROOT']=str(root)
    import test_b16_goal_bridge as original_tests
    import test_b17_alignment as time_tests
    original_tests.A_GEOMETRY=geometry
    class RealReplayTests(unittest.TestCase):
        def test_saved_bundles_really_read_and_bound(self):
            self.assertEqual(len(real),len(manifest['samples']))
            self.assertTrue(all(len(s['actual_q12'])==12 and all(f['exists'] for f in s['sources'].values()) for s in real))
        def test_partial_features_do_not_promote_goals(self):
            self.assertTrue(all(s['final_status']=='NEEDS_OBSERVATION' and not s['executable'] for s in real))
            self.assertTrue(all(p['status']=='NEEDS_OBSERVATION' and not p['targets'] for p in proposals))
        def test_A_feature_replay_matches_stored_point_and_edge_counts(self):
            comparable=[s['feature_stage_agreement'] for s in real if s['feature_stage_agreement']['feature_present_before']]
            self.assertTrue(comparable)
            self.assertTrue(all(x.get('plane_point_count_equal') and x.get('exterior_edge_count_equal') for x in comparable))
            pads=[p for s in real for p in s['pads'] if p.get('recorded_plane_support_count_equal') is not None]
            self.assertTrue(pads)
            self.assertTrue(all(p['recorded_plane_support_count_equal'] for p in pads))
        def test_existing_model_preview_preserved_without_physical_promotion(self):
            self.assertEqual(agreement,[0.0,0.0])
            self.assertEqual(proposals[-1]['planning_preview']['collision_result'],'CLEAR')
            self.assertFalse(proposals[-1]['executable'])
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromModule(original_tests),
        unittest.defaultTestLoader.loadTestsFromModule(time_tests),
        unittest.defaultTestLoader.loadTestsFromTestCase(RealReplayTests)])
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    (out/'UNITTEST.log').write_text(stream.getvalue())
    counts['forbidden_modules_loaded']=[n for n in sys.modules if any(n==x or n.startswith(x+'.') for x in forbidden)]
    changed=[path for path,digest in checked.items() if sha256(path)!=digest]
    ok=(result.wasSuccessful() and not result.skipped and not changed and not counts['forbidden_attempts']
        and not counts['forbidden_modules_loaded'] and all(counts[k]==0 for k in
        ('motor_commands_sent','executor_calls','optimizer_updates','mj_step_calls','device_open_attempts')))
    write_json(out/'SOURCE_INPUT_MANIFEST_PRIVATE.json',checked)
    write_json(out/'B17_GOAL_BRIDGE_REGRESSION.json',dict(status='PASS' if ok else 'FAIL',
        tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),
        preserved_B16_cases=17,additional_timing_cases=8,stored_REAL_replay_cases=4,
        source_changes=changed,side_effect_checks=counts,model_waypoint_differences=agreement,
        CPU_environment=dict(python=sys.version.split()[0],numpy=np.__version__,mujoco=geometry.mj.__version__),
        implementation_sha256={p.name:sha256(p) for p in Path(__file__).parent.glob('*.py')}))
    print(dict(status='PASS' if ok else 'FAIL',tests=result.testsRun,replayed=len(real),counts=counts))
    return 0 if ok else 1


if __name__=='__main__':
    raise SystemExit(main())
