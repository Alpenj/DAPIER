"""Pinned offline RGB replay; debug outputs never enter the original Episode."""
import argparse
import copy
import io
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'goal_bridge'))
from b16_goal_bridge import load_module,sha256
from run_b17 import audit_side_effects
from visual_servo.wrist_overlay import frame_binding,measure_wrist,render_wrist


def debug_output_path(output, original_runs):
    output=Path(output).resolve()
    if any(output.is_relative_to(Path(p).resolve()) for p in original_runs):
        raise ValueError('DEBUG_OUTPUT_MUST_STAY_OUTSIDE_ORIGINAL_RUNS')
    return output


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--inputs',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();inputs=json.loads(args.inputs.read_text())
    args.output=debug_output_path(args.output,[s['root'] for s in inputs['sessions']]);args.output.mkdir(parents=True,exist_ok=False)
    counts,forbidden=audit_side_effects();cv2.setNumThreads(1)
    import mujoco
    checked={};rows=[];panels={}
    def check(path):
        path=Path(path);checked[str(path)]=sha256(path);return path
    def read(path):return json.loads(check(path).read_text())
    check(args.inputs)
    for p,digest in inputs['source_manifest'].items():
        if sha256(check(p))!=digest:raise ValueError('PINNED_A_SOURCE_CHANGED')
    for p in (ROOT/'visual_servo').glob('*.py'):check(p)
    model=mujoco.MjModel.from_binary_path(str(check(inputs['model'])))
    source_xml=check(inputs['source_model_xml']);xml=ET.parse(source_xml).getroot()
    meshdir=source_xml.parent/xml.find('compiler').get('meshdir','')
    for name in ('right_pgripper_jaw_1','right_pgripper_jaw_2','right_pgripper_pad_1','right_pgripper_pad_2'):
        check(meshdir/xml.find("asset/mesh[@name='"+name+"']").get('file'))
    first=Path(inputs['sessions'][0]['source'])
    kin=load_module('overlay_existing_kin',first/'sweepick_kin.py')
    command=load_module('overlay_existing_command_math',first/'sweepick_real_command.py')
    face=load_module('overlay_existing_pad_contract',first/'inner_pad_face_contract.py')
    from visual_servo.features import a_image_functions
    from visual_servo.right_relative import _right_functions
    source_hashes=tuple(sha256(first/n) for n in ('sweepick_pick01.py','right_wrist_relative_geometry.py','sweepick_261009_offsite_relative_features.py'))
    _,geometry,_=_right_functions(first,source_hashes)
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    contracts={i:face.inner_face_contract(model,data,i,source_xml) for i in (1,2)}
    optical=read(inputs['right_optical'])
    def mapping(source):
        for n in ('CANDIDATE_Q12_CONFIG.json','ASSEMBLED_ZERO_AND_PGRIPPER.json','dapier_dual_follower_left.json','dapier_dual_follower_right.json'):check(source/n)
        return command.load_mapping(source/'CANDIDATE_Q12_CONFIG.json',source,source/'ASSEMBLED_ZERO_AND_PGRIPPER.json')
    def q6(mp,side,ticks):return [command.tick_to_model(n,mp[side][n],ticks[n]) for n in command.JOINTS]
    def pose(side,q):
        d=mujoco.MjData(model);kin.set_arm_q(model,d,side,q);mujoco.mj_forward(model,d)
        t=np.eye(4);site=model.site(side+'_cube_grasp').id
        t[:3,:3]=d.site_xmat[site].reshape(3,3);t[:3,3]=d.site_xpos[site]
        return d,t
    def camera(side,d,t,hw):
        if side=='right':return np.asarray(optical['K']),np.asarray(optical['D']),t@np.asarray(optical['T_tool_camera'])
        c=model.camera('left_wrist_rgb').id;twc=np.eye(4)
        twc[:3,:3]=d.cam_xmat[c].reshape(3,3)@np.diag([1,-1,-1]);twc[:3,3]=d.cam_xpos[c]
        h,w=hw;f=h/(2*np.tan(np.deg2rad(model.cam_fovy[c])/2))
        return np.array([[f,0,w/2],[0,f,h/2],[0,0,1]]),np.zeros(5),twc
    def model_reference(frame,side,q):
        d,t=pose(side,q);K,D,twc=camera(side,d,t,frame['image_bgr'].shape[:2]);missing=[]
        def project(points):
            try:return geometry.project_points(points,K,D,twc).tolist()
            except ValueError as exc:missing.append(str(exc));return None
        centre=project([t[:3,3]]);centres=[d.site_xpos[model.site(side+'_pgripper_pad_'+str(i)+'_inner').id] for i in (1,2)]
        closing=project(centres);boundaries=[]
        if side=='right':
            projected=geometry.pad_projection(model,d,contracts,K,D,twc)
            for pad in projected:
                for edge in pad.get('pixels') or []:boundaries.append(edge)
        ref=dict(binding=frame_binding(frame),role='MODEL_PRIOR',centre_uv=centre[0] if centre else None,
            closing_line_uv=closing,pad_boundaries_uv=boundaries,
            source=dict(model_sha256=checked[inputs['model']],camera='existing measured K/D/T with unresolved field consistency' if side=='right' else 'existing MJCF camera seat and 75-degree virtual FOV; not REAL intrinsics',
                        actual_q_role='candidate mapping of recorded arm sample, exposure alignment unverified',missing=missing))
        return ref,K,D,twc
    def frame(path,identity,side='left'):
        path=check(path);image=cv2.imread(str(path),cv2.IMREAD_COLOR)
        if image is None:raise ValueError('RAW_RGB_DECODE_FAILED')
        return dict(image_bgr=image,source_kind='REAL',camera_key=side+'_wrist',artifact_role='RAW_RGB',source_sha256=checked[str(path)],image_identity=identity)
    def keep(sample,f,r,metadata):
        raw_before=f['image_bgr'].copy();vis=render_wrist(f,r)
        np.testing.assert_array_equal(raw_before,f['image_bgr'])
        panels[sample]=vis['image_bgr'];rows.append(dict(sample=sample,metadata=metadata,features=r))
    for item in inputs['sessions']:
        root=Path(item['root']);session=root/'session';source=Path(item['source']);mp=mapping(source)
        state=read(session/'STATE.json');base_ref=copy.deepcopy(state['jaw_reference'])
        initial=frame(session/'looks/00_observe/left_wrist.jpg',item['run']+':REFERENCE')
        a,_=a_image_functions(source);detected=a.jaw_reference(initial['image_bgr'])
        # JPEG replay may change edge pixels: retain recorded reference and disclose the comparison.
        base_ref.update(camera_key='left_wrist',source_kind='REAL',source_sha256=checked[str(session/'STATE.json')],reference_shape_hw=list(initial['image_bgr'].shape[:2]))
        evidence=state.get('evidence') or []
        for index,look in enumerate(item['looks']):
            scene_path=session/'looks'/look/'SCENE.json';scene=read(scene_path);p=scene_path.parent/'left_wrist.jpg'
            sample=item['run']+':'+look;f=frame(p,sample);binding=frame_binding(f)
            context=dict(binding=binding,scene=scene,plan=state['plan'],gripper_tick=scene['ticks']['left']['gripper'])
            ref,K,D,twc=model_reference(f,'left',scene['q12'][:6]);future=None
            if index==0:
                next_ticks=dict(scene['ticks']['left']);next_ticks.update(state['plan']['ticks']['descend'])
                dnext,tnext=pose('left',q6(mp,'left',next_ticks))
                _,_,twcnext=camera('left',dnext,tnext,f['image_bgr'].shape[:2])
                obj=scene.get('object') or {};geom=obj.get('geometry') or {};centre=obj.get('centre_world');half=geom.get('half_extents_m')
                if centre is not None and half is not None:
                    plane=np.asarray(centre,float)+[0,0,half[2]]
                    future=dict(binding=binding,role='MODEL_PRIOR',K=K.tolist(),D=D.tolist(),T_world_camera=twc.tolist(),T_world_camera_next=twcnext.tolist(),
                        plane_centre=plane.tolist(),plane_normal=[0,0,1],input_role='RECORDED_PLANNED_DESCEND_ENDPOINT_NOT_ACTUAL',source=dict(scene_sha256=checked[str(scene_path)],state_sha256=checked[str(session/'STATE.json')],object_geometry_role=geom.get('classification'),camera_role='unverified existing model optics'))
            r=measure_wrist(f,side='left',jaw_reference=base_ref,context=context,model_projection=ref,next_descent=future,a_source=source)
            lift=[e for e in evidence if e.get('what')=='held_after_lift']
            if item['group']!='FAILURE':
                assert any(e.get('label')=='HELD_CONFIRMED' and e.get('ok') is True and (e.get('jaw_during_lift') or {}).get('loaded_throughout') is True for e in lift)
            keep(sample,f,r,dict(group=item['group'],raw_path=str(p),scene_path=str(scene_path),scene_sha256=checked[str(scene_path)],
                state_sha256=checked[str(session/'STATE.json')],recorded_wrist=scene.get('wrist'),reference_replay=detected,
                held_after_lift=lift,measurement_vs_receipt='scene joint reads and wrist receipt not simultaneous exposure; no interpolation'))
    archived=read(inputs['right_archive_manifest'])
    for candidate in ('archive_candidate_005','archive_candidate_037'):
        row=next(r for r in archived if r['candidate_id']==candidate);original=Path(row['source'])
        if sha256(check(original))!=row['source_sha256']:raise ValueError('ARCHIVE_RGB_SHA_MISMATCH')
        sample=frame(original,candidate,'right');run=next(s for s in inputs['sessions'] if s['run']==row['run']);source=Path(run['source']);mp=mapping(source)
        actual=row['right_actual_at_stage_row'];log=read(actual['source'])
        if checked[actual['source']]!=actual['source_sha256'] or log['side']!='right' or log['rows'][actual['row_index']]['present']!=actual['present']:raise ValueError('ARCHIVE_ACTUAL_ARM_SOURCE_MISMATCH')
        ref,_,_,_=model_reference(sample,'right',q6(mp,'right',actual['present']))
        r=measure_wrist(sample,side='right',model_projection=ref,a_source=source)
        keep(candidate,sample,r,dict(group='UNVERIFIED_REFERENCE_CANDIDATE',raw_path=str(original),right_actual_stage_sha256=actual['source_sha256'],row_index=actual['row_index'],q_scope='RIGHT_ARM_ONLY_RECORDED_ACTUAL_ROW; no left q fabricated',exposure_time='UNKNOWN',next_descent='UNKNOWN_NO_RECORDED_DESCENT_TARGET'))
    from visual_servo import test_wrist_overlay,test_visual_servo,test_b19,test_b20
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromModule(m) for m in (test_wrist_overlay,test_visual_servo,test_b19,test_b20))
    stream=io.StringIO();tests=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    def title(panel,text):
        panel=cv2.resize(panel,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST)
        head=np.zeros((35,panel.shape[1],3),np.uint8);cv2.putText(head,text,(8,24),cv2.FONT_HERSHEY_SIMPLEX,.55,(240,240,240),1,cv2.LINE_AA)
        return np.vstack([head,panel])
    left_grid=[]
    for index,item in enumerate(inputs['sessions']):
        label='FAILURE' if index==0 else 'LEFT SUCCESS '+str(index)
        name=item['run'];left_grid.append(np.hstack([title(panels[name+':'+look],label+' '+('BEFORE FINAL DESCENT' if i==0 else 'AFTER DESCENT')) for i,look in enumerate(item['looks'])]))
    left_image=np.vstack(left_grid)
    representative=inputs['sessions'][1];key=representative['run']+':'+representative['looks'][0]
    dual=np.hstack([title(panels[key],'LEFT successful pick: before descent'),title(panels['archive_candidate_005'],'RIGHT archive 005'),title(panels['archive_candidate_037'],'RIGHT archive 037')])
    header=np.zeros((36,dual.shape[1],3),np.uint8);cv2.putText(header,'DIFFERENT RECORDED OBSERVATIONS / DEBUG ONLY',(8,26),cv2.FONT_HERSHEY_SIMPLEX,.7,(240,240,240),1,cv2.LINE_AA);dual=np.vstack([header,dual])
    outputs={}
    for name,image in (('LEFT_WRIST_DESCENT_COMPARISON.png',left_image),('DUAL_WRIST_REFERENCE_OVERLAY.png',dual)):
        p=args.output/name
        if not cv2.imwrite(str(p),image):raise OSError('DEBUG_OVERLAY_WRITE_FAILED')
        outputs[name]=dict(sha256=sha256(p),artifact_role='DEBUG_OVERLAY_ONLY',used_for_detection=False)
    changes=[p for p,d in checked.items() if sha256(p)!=d]
    loaded=[n for n in sys.modules if any(n==f or n.startswith(f+'.') for f in forbidden)]
    status='PASS' if tests.wasSuccessful() and not changes and not loaded and not counts['forbidden_attempts'] else 'FAIL'
    forecast_checks=[]
    for item in inputs['sessions']:
        pair=[next(r['features'] for r in rows if r['sample']==item['run']+':'+look) for look in item['looks']]
        forecast=pair[0]['fov']['next_descent'];clip=pair[1]['object']['widest_row_horizontal_clipped']
        forecast_checks.append(dict(run=item['run'],nominal_top_envelope_status=forecast['status'],after_actual_horizontal_clip=clip,
            assessment='MODEL_ENVELOPE_DID_NOT_FLAG_ACTUAL_X_CLIP' if clip and forecast.get('outside_fraction')==0 else 'NO_X_CLIP_AFTER_NOT_A_PHYSICAL_FORECAST_VALIDATION',valid_for_control=False))
    summary=dict(schema='sweepick.dual-wrist-overlay-validation/1',status=status,
        validation_scope='OFFLINE_IMPLEMENTATION_AND_SOURCE_CHECKS; physical control/forecast validation not established',
        tests=dict(run=tests.testsRun,failures=len(tests.failures),errors=len(tests.errors),skipped=len(tests.skipped)),
        frames=[dict(sample=row['sample'],side=row['features']['side'],image_error_px=row['features']['image_error_px'],
            raw_far_edge_slope_deg=row['features']['object']['raw_far_edge_slope_deg'],relative_angle=row['features']['relative_angle'],
            frame_clipped=row['features']['object']['frame_clipped'],horizontal_clipped=row['features']['object']['widest_row_horizontal_clipped'],next_FOV_status=row['features']['fov']['next_descent']['status']) for row in rows],
        usable_for_A=['left confirmed X-only reference error when not horizontally clipped; component may be bottom-clipped','observed visible contour/component centroid, with visibility caveats'],
        diagnostics_only=['MODEL_PRIOR grasp centre/closing/pad projection','legacy raw edge slope without correspondence','nominal model next-descent FOV forecast'],
        not_available=['observed 2D grasp centre','right observed Pad-relative error','REAL 3D yaw','verified relative angles in current archives'],
        next_descent_retrospective_checks=forecast_checks,
        model_next_descent_FOV_validation='DID_NOT_FLAG_SPECIFIED_FAILURE_X_CLIP; DIAGNOSTIC_ONLY',
        A_feature_contract=dict(
            left_X=dict(field='image_error_px.u',definition='A widest-row object midpoint minus confirmed saved jaw midline',unit='pixel',
                eligibility='own raw frame/reference binding; legacy wrist_offset readable/current-confirmed; no horizontal clipping',v_reference=None),
            observed_contour=dict(field='object.visible_edges',meaning='visible red-region boundary, not a certified physical cuboid edge',
                clipped_boundary='object.clipped_boundary',partial_centroid='object.component_centroid_uv'),
            image_angle=dict(field='relative_angle.value_deg',meaning='image-plane angle only; configured image target and MODEL_PRIOR direction have separate provenance; neither certifies observed gripper direction',required='own bound unoccluded corresponding straight observed edge plus valid reference',
                current_REAL_frames='UNKNOWN',physical_3D_yaw=None),
            models=dict(fields=['gripper.model_grasp_centre_uv','gripper.model_pad_boundaries_uv','fov.next_descent'],role='MODEL_PRIOR_DIAGNOSTIC_ONLY',use_for_control=False),
            input_output=dict(raw_role='RAW_RGB',output_role='DEBUG_OVERLAY_ONLY',overlays_as_detector_input=False)),
        skipped_test_details=[dict(test=str(t),reason=reason) for t,reason in tests.skipped],
        output_images=outputs,side_effect_checks=dict(counts,forbidden_modules_loaded=loaded),original_source_changes=changes,
        Correction=None,executable=False,motor_write_authorized=False,
        code_sha256={n:sha256(ROOT/'visual_servo'/n) for n in ('wrist_overlay.py','test_wrist_overlay.py','run_wrist_overlay.py')})
    private=dict(summary,feature_records=rows,source_manifest=checked,unittest_log=stream.getvalue())
    (args.output/'execution.private.json').write_text(json.dumps(private,indent=2,allow_nan=False)+'\n')
    (args.output/'DUAL_WRIST_OVERLAY_VALIDATION.json').write_text(json.dumps(private,indent=2,allow_nan=False)+'\n')
    (args.output/'public_summary.private.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(status=status,tests=summary['tests'],frames=summary['frames'],side_effect_checks=summary['side_effect_checks']),indent=2))
    return 0 if status=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
