"""File-only B20 replay, clipped-feature regression and paired CPU benchmark."""
import argparse
import io
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'goal_bridge'))
from b16_goal_bridge import load_module,read_json,sha256,write_json
from run_b17 import audit_side_effects
from visual_servo.a_shadow import proposal_from_A_record
from visual_servo.features import vision_functions
from visual_servo.performance import benchmark
from visual_servo.right_relative import RightRelativeServoProposal,extract_right_features,_right_functions
from visual_servo.run_b19 import saved_observation


def optical_audit(inputs,check):
    """Re-evaluate saved board points with existing K/D/T; never fit a new calibration."""
    original=read_json(check(inputs['original_calibration']))
    cal=read_json(check(inputs['route_bound_calibration']))
    optical=load_module('b20_saved_optical',check(inputs['optical_source']))
    same=all(np.array_equal(original[k],cal[k]) for k in ('K','D','T_tool_camera'))
    if not same:raise ValueError('route binding changed optical parameters; audit inputs differ')
    records=[read_json(check(p)) for p in sorted(Path(inputs['optical_dataset']).glob('sample_*.json'))]
    optical._validate_records(records)
    for record in records:
        image=check(Path(inputs['optical_dataset'])/record['image_file'])
        if sha256(image)!=record['image_sha256']:raise ValueError('optical sample image/source mismatch')
    K=np.asarray(cal['K'],float);D=np.asarray(cal['D'],float)
    X=optical._rigid('T_tool_camera',cal['T_tool_camera'])
    legacy=load_module('b20_existing_geometry',check(Path(inputs['b16_root'])/'b16_goal_bridge.py'))
    geometry=legacy.AOfflineGeometry()
    manifest=read_json(check(Path(inputs['b16_root'])/'SOURCE_INPUT_MANIFEST.json'))
    model=check(manifest['external_model']['source'])
    if sha256(model)!=manifest['external_model']['sha256']:raise ValueError('existing model SHA changed')
    reference=None;rows=[]
    for record in records:
        camera_board,pnp=optical._target_pose(record,K,D)
        world_tool=optical._rigid('T_base_tool',record['T_base_tool'])
        world_board=world_tool@X@camera_board
        if reference is None:reference=world_board
        fixed=np.linalg.inv(world_tool@X)@reference
        rvec=cv2.Rodrigues(fixed[:3,:3])[0]
        pixels=cv2.projectPoints(np.asarray(record['object_points_m'],float),rvec,fixed[:3,3],K,D)[0].reshape(-1,2)
        fixed_rms=float(np.sqrt(np.mean(np.sum((pixels-np.asarray(record['image_points_px']))**2,axis=1))))
        fk=geometry.fk(record['q'])['right']
        frame=record['frame_stamps'];state=record['q_stamps']
        rows.append(dict(sample_id=record['sample_id'],split=record['split'],PnP_reprojection_rmse_px=pnp,
            fixed_reference_projection_rmse_px=fixed_rms,static_consistency=optical._transform_residual(reference,world_board),
            world_board_translation_m=world_board[:3,3].tolist(),world_translation_delta_m=(world_board[:3,3]-reference[:3,3]).tolist(),
            reference_board_translation_delta_m=(np.linalg.inv(reference)@world_board)[:3,3].tolist(),
            recorded_FK_vs_existing_model_max_abs_difference=float(np.max(np.abs(fk-world_tool))),
            q_mapping_verified=record['q_mapping'].get('verified'),model_verified=record['geometry'].get('verified'),
            receipt_minus_feedback_stamp_s=frame['recv_monotonic_s']-state['feedback_stamp'],
            exposure_clock=frame.get('exposure_clock','UNKNOWN'),q_measurement_time=None))
    return dict(status='FIELD_VALIDATION_REQUIRED',source_kind='REAL',rows=rows,
        original_calibration_sha256=sha256(inputs['original_calibration']),route_bound_sha256=sha256(inputs['route_bound_calibration']),
        route_rebind_unchanged_K_D_T=same,route_rebind_provenance=cal.get('route_rebind_provenance'),
        reference='sample_0000_train reconstructed world-board pose; not independent metric ground truth',
        conventions=dict(T_tool_camera='camera coordinates to right_cube_grasp tool coordinates',
            T_world_camera='T_base_tool @ T_tool_camera',T_world_board='T_base_tool @ T_tool_camera @ T_camera_board',
            projection='inv(T_world_camera) then existing raw K/D',optical_axes='x right, y down, z forward',base_frame='A_MODEL_WORLD'),
        camera_origin_in_tool_m=X[:3,3].tolist(),camera_optical_Z_in_tool=X[:3,2].tolist(),
        stored_validation=cal['validation_report'],calibration_solver_calls=0,saved_point_PnP_calls=len(records),
        new_capture_count=0,calibration_changed=False,physical_TCP_error_bound_m=None,live_execution_permitted=False,
        conclusion='per-frame PnP fits do not establish fixed-world consistency; current q/FK, optical bias, mounting and timing cannot be isolated from saved data',
        physical_mapping_verified=False,physical_pad_identity_verified=False)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--inputs',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    counts,forbidden=audit_side_effects();cv2.setNumThreads(1)
    checked={};inputs=read_json(args.inputs)
    def check(path):
        path=Path(path);checked[str(path)]=sha256(path);return path
    check(args.inputs)
    for path in Path(__file__).parent.glob('*.py'):check(path)
    original=load_module('visual_servo.b19_original_right',check(inputs['b19_right_source']))
    samples=[];records=[];outcomes=[]
    for item in inputs['sessions']:
        session=Path(item['session']);source=Path(item['a_source'])
        for path in source.glob('*.py'):check(path)
        outcomes.append(dict(run=session.parent.name,record=read_json(check(session.parent/'RUN_RECORD.json'))))
        for folder in sorted((session/'decision_observations').iterdir()):
            if not (folder/'OBSERVATION.json').is_file():continue
            document,observation,path=saved_observation(folder,session.parent.name,checked)
            kwargs=dict(record_identity=observation['observation_identity'],record_sha256=checked[str(path)],
                image_sha256={k:v['source_sha256'] for k,v in observation['views'].items()},a_source=source)
            images={k:v['image_bgr'] for k,v in observation['views'].items()}
            proposal=proposal_from_A_record(document,images,**kwargs)
            held=proposal['reference_provenance']['object_target_association']
            detailed=RightRelativeServoProposal(document['q12'],observation,held,{},None,
                a_source=source,include_geometry=True)
            same=all(proposal[k]==detailed[k] for k in
                ('relative_feature_error_px','observed_control_dimensions','estimated_tool_correction'))
            feature=proposal['feature_evidence']
            samples.append(dict(observation_source=str(path),record_sha256=checked[str(path)],
                identity=observation['observation_identity'],proposal=proposal,
                detailed_graph= detailed['feature_evidence']['visible_approach_region']['graph'],
                fast_detailed_control_equal=same,frame_clipped=feature['visibility']['frame_clipped'],
                raw_visible=feature['visible_approach_region']['visible']))
            records.append((document,observation,held,images,kwargs,source))
    clipped=[s for s in samples if s['frame_clipped']]
    optical=optical_audit(inputs,check)
    write_json(out/'B20_OPTICAL_HOLDOUT_AUDIT.json',optical)
    class StoredRealNegativeTests(unittest.TestCase):
        def test_saved_holdout_values_reproduce_without_refitting_calibration(self):
            prior={r['sample_id']:r for r in optical['stored_validation']['holdout']['samples']}
            for row in optical['rows']:
                if row['split']!='holdout':continue
                old=prior[row['sample_id']]
                np.testing.assert_allclose(row['PnP_reprojection_rmse_px'],old['reprojection_rmse_px'])
                for key,value in row['static_consistency'].items():
                    np.testing.assert_allclose(value,old['target_static_consistency'][key])
            self.assertEqual(optical['calibration_solver_calls'],0)
            self.assertEqual(optical['status'],'FIELD_VALIDATION_REQUIRED')
        def test_actual_clipped_frames_and_all_missing_references_cannot_correct(self):
            self.assertTrue(clipped,'REAL clipped negative fixtures must exist')
            self.assertTrue(all(s['fast_detailed_control_equal'] for s in samples))
            for s in samples:
                p=s['proposal'];self.assertEqual(p['observed_control_dimensions'],0)
                self.assertIsNone(p['estimated_tool_correction']);self.assertFalse(p['executable'])
                self.assertFalse(p['motor_write_authorized'])
            for s in clipped:
                self.assertIn('RIGHT_COMPONENT_CENTROID_FRAME_CLIPPED',s['proposal']['missing_evidence'])
                self.assertFalse(s['proposal']['feature_evidence']['feature_validity']['centroid_usable_for_control'])
    import visual_servo.test_b19 as old_tests
    import visual_servo.test_b20 as new_tests
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromModule(m) for m in (old_tests,new_tests)]+
        [unittest.defaultTestLoader.loadTestsFromTestCase(StoredRealNegativeTests)])
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    (out/'B20_UNITTEST.log').write_text(stream.getvalue())
    timings=[]
    # One old RECEIVE frame and one new candidate frame retain the exact same pixels in both paths.
    selected=[i for i,s in enumerate(samples) if s['identity']['observation_id'] in
        ('000005_RECEIVE_CONTACT','000023_OBSERVATION_CANDIDATE')]
    for i in selected:
        document,observation,held,images,kwargs,source=records[i]
        common=lambda:RightRelativeServoProposal(document['q12'],observation,held,{},None,a_source=source)
        def detailed():
            cv2.setRNGSeed(0)
            return RightRelativeServoProposal(document['q12'],observation,held,{},None,
                a_source=source,include_geometry=True)
        def baseline():
            cv2.setRNGSeed(0)
            return original.RightRelativeServoProposal(document['q12'],observation,held,{},None,a_source=source)
        def initialize():
            _right_functions.cache_clear();return common()
        init=benchmark(initialize,repeats=3);common()
        _,geometry,functions=vision_functions(source)
        image=images['right_wrist'];surface=functions.wrist_object_surfaces(image)
        def graph():
            cv2.setRNGSeed(0);return geometry.observed_face_graph(image,surface['_mask'])
        timings.append(dict(identity=samples[i]['identity'],image_array_sha256=
            samples[i]['proposal']['feature_evidence']['source_sha256']['image_array'],
            source_initialization_and_first_feature=init,
            B19_original_full_shadow=benchmark(baseline,repeats=9),
            cached_right_feature_fast=benchmark(lambda:extract_right_features(observation,held,{},a_source=source),repeats=31),
            cached_feature_and_servo_fast=benchmark(common,repeats=31),
            cached_feature_and_servo_detailed=benchmark(detailed,repeats=9),
            full_A_record_adapter_fast=benchmark(lambda:proposal_from_A_record(document,images,**kwargs),repeats=31),
            bare_RGB_surface=benchmark(lambda:functions.wrist_object_surfaces(image),repeats=31),
            bare_face_graph=benchmark(graph,repeats=9)))
    write_json(out/'B20_FAST_PATH_BENCHMARK.json',dict(status='MEASURED_CPU_ONLY',timings=timings,
        environment=dict(python=sys.version.split()[0],numpy=np.__version__,opencv=cv2.__version__,
            platform=sys.platform,OpenCV_threads=cv2.getNumThreads()),
        timing_scope='single process; identical input arrays and frozen A source per comparison; function wall time',
        geometry_RNG='OpenCV RNG reset to zero for detailed/graph/baseline CPU fixtures only',
        full_computation='full_A_record_adapter_fast includes record binding, feature extraction and servo proposal',
        stages_are_additive=False,camera_acquisition_s=None,actuator_command_s=None,settling_s=None,
        physical_cycle_time_saved_s=None,A_gains_or_motor_parameters_changed=False))
    changed=[p for p,h in checked.items() if sha256(p)!=h]
    counts['forbidden_modules_loaded']=[n for n in sys.modules if any(n==x or n.startswith(x+'.') for x in forbidden)]
    ok=result.wasSuccessful() and not result.skipped and not changed and not counts['forbidden_attempts'] and not counts['forbidden_modules_loaded']
    ok=ok and all(counts[k]==0 for k in ('motor_commands_sent','executor_calls','optimizer_updates','mj_step_calls','device_open_attempts'))
    write_json(out/'B20_FEATURE_VALIDITY_REGRESSION.json',dict(status='PASS' if ok else 'FAIL',
        tests_run=result.testsRun,B19_tests_preserved=23,B20_unit_tests=6,stored_REAL_evidence_tests=2,
        failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),
        REAL_samples=len(samples),REAL_clipped_negative_samples=len(clipped),samples=samples,
        native_outcomes=outcomes,side_effect_checks=counts,input_source_changes=changed,
        implementation_sha256={p.name:sha256(p) for p in Path(__file__).parent.glob('*.py')},
        new_numeric_physical_limits=0,REAL_Jacobian=None,REAL_X_sign_verified=False,REAL_success=False))
    write_json(out/'SOURCE_INPUT_MANIFEST_PRIVATE.json',checked)
    print(dict(status='PASS' if ok else 'FAIL',tests=result.testsRun,REAL_samples=len(samples),
        clipped=len(clipped),side_effects=counts))
    return 0 if ok else 1


if __name__=='__main__':raise SystemExit(main())
