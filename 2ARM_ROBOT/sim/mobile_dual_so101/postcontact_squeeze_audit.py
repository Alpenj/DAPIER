#!/usr/bin/env python3
"""One saved-state squeeze-budget comparison; never changes task policy."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import types
import mujoco
import numpy as np
from controlled_contact_audit import (MODE, PADS, array_hashes, contact_metrics,
                                      native_contacts, options)
from close_contact_diagnostic import restore
from dynamic_preflight import integration_state
from pgripper import jaw_gap_m
from waypoint_block_teacher import WaypointBlockTeacher


def closing_budget(first, last):
    # A phase transition holds ctrl; only a decreasing opening reference adds squeeze.
    return float(first['command_rad'] - last['command_rad'])


def matched_target(a0_first, a0_last, a2_first, a2_last, ctrlrange):
    budget = closing_budget(a0_first, a0_last)
    if budget <= closing_budget(a2_first, a2_last):
        raise ValueError('A2 does not have a smaller commanded post-contact budget')
    target = a2_first['command_rad'] - budget
    if not np.isfinite(target) or not ctrlrange[0] <= target <= ctrlrange[1]:
        raise ValueError('matched command violates original ctrlrange')
    return float(target)


def audit(a0_path, a2_path, model_path):
    a0 = json.loads(a0_path.read_text())['rows']
    a2 = [json.loads(line) for line in a2_path.open()]
    model = mujoco.MjModel.from_binary_path(str(model_path))
    j = int(model.actuator_trnid[5, 0]); qa = int(model.jnt_qposadr[j]); va = int(model.jnt_dofadr[j])
    assert model.actuator_gaintype[5] == mujoco.mjtGain.mjGAIN_FIXED
    assert model.actuator_biastype[5] == mujoco.mjtBias.mjBIAS_AFFINE
    assert model.actuator_dyntype[5] == mujoco.mjtDyn.mjDYN_NONE
    np.testing.assert_array_equal(model.actuator_gear[5], [1, 0, 0, 0, 0, 0])
    def normalized(row, bench):
        if bench:
            return dict(time_s=row['force_evaluation_time_s'], phase=row['phase'],
                command_rad=row['ctrl'][0], measured_rad=row['motor_qpos_rad'],
                opening_m=row['jaw_plane_gap_evaluation_m'], force_Nm=row['actuator_force_Nm'][0],
                normal_N=[row['Fn_N'][p] for p in PADS], table_N=row['support_Fn_N'],
                force_source='recorded pre-integration actuator_force; motor q is post-integration')
        q, v = row['raw_qpos'][qa], row['raw_qvel'][va]
        b = model.actuator_biasprm[5]
        force = model.actuator_gainprm[5, 0]*row['ctrl'][5] + b[0] + b[1]*q + b[2]*v
        force = np.clip(force, *model.actuator_forcerange[5]) if model.actuator_forcelimited[5] else force
        return dict(time_s=row['time_s'], phase=row['phase'], command_rad=row['ctrl'][5],
            measured_rad=q, opening_m=row['jaw_opening_m'], force_Nm=float(force),
            normal_N=[row['normalized']['pads'][p]['summed_normal_force_N'] for p in PADS],
            table_N=sum(max(0, c['wrench_contact'][0]) for c in row['contacts'] if 'table' in c['names']),
            force_source='analytic compiled fixed-gain/affine-bias law on recorded post-forward state')
    result = dict(mode=MODE, task_success=False, cases={}, sources={
        str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (a0_path,a2_path,model_path)})
    for name, rows, bench in [('A0',a0,True),('A2',a2,False)]:
        records = [normalized(r,bench) for r in rows if r['phase'] in ('CLOSE','GRASP_CONFIRM')]
        onset = next(i for i,r in enumerate(records) if min(r['normal_N']) > 0)
        tail = records[onset:]; first = tail[0]; end = next(r for r in reversed(tail) if r['phase']=='CLOSE')
        confirm = [r for r in tail if r['phase']=='GRASP_CONFIRM']
        result['cases'][name] = dict(first_bilateral=first, close_end=end, confirm_end=tail[-1],
            command_budget_rad=closing_budget(first,tail[-1]),
            measured_closing_rad=first['measured_rad']-tail[-1]['measured_rad'],
            opening_reduction_m=first['opening_m']-tail[-1]['opening_m'],
            post_bilateral_duration_s=tail[-1]['time_s']-first['time_s'],
            confirm_command_unchanged=all(r['command_rad']==end['command_rad'] for r in confirm),
            max_abs_actuator_force_Nm=max(abs(r['force_Nm']) for r in tail), post_bilateral=tail)
    result['actuator_contract'] = dict(gain=model.actuator_gainprm[5].tolist(),
        bias=model.actuator_biasprm[5].tolist(),forcerange=model.actuator_forcerange[5].tolist(),
        force_limited=bool(model.actuator_forcelimited[5]),ctrlrange=model.actuator_ctrlrange[5].tolist())
    a,b = (result['cases'][k] for k in ('A0','A2'))
    result['matched_target_rad'] = matched_target(a['first_bilateral'],a['confirm_end'],
        b['first_bilateral'],b['confirm_end'],model.actuator_ctrlrange[5])
    result['less_actual_squeeze'] = (b['measured_closing_rad'] < a['measured_closing_rad'] and
                                    b['opening_reduction_m'] < a['opening_reduction_m'])
    return result, a2


def lift_outcome(rows, success):
    lift = [r for r in rows if r['phase'].startswith('LIFT') or r['phase']=='HOLD']
    detached = next((i for i,r in enumerate(lift) if r['table_contact_count']==0 and r['table_force_N']==0), None)
    lost = next((i for i,r in enumerate(lift) if min(r['finger_forces_N'])<=0), None)
    if success and lift and lift[-1]['table_contact_count']==0 and lift[-1]['table_force_N']==0 and min(lift[-1]['finger_forces_N'])>0 and lift[-1]['lift_supported'] and lift[-1]['continuous_hold_s']>=3.0:
        return 'A'
    if lost is not None and (detached is None or lost < detached):
        return 'B'
    if detached is None:
        return 'C'
    if any(r['phase']=='HOLD' and (min(r['finger_forces_N'])<=0 or not r['lift_supported']) for r in lift):
        return 'D'
    return 'UNCLASSIFIED_PARTIAL_LIFT'



def lift_summary(case):
    rows=case['rows'];lift=[r for r in rows if r['phase'].startswith('LIFT') or r['phase']=='HOLD']
    dt=case['options']['timestep'];start=lift[0]['time_s']-dt if lift else None
    def event(predicate):
        for i,r in enumerate(lift,1):
            if predicate(r):
                return dict(lift_step=i,time_s=r['time_s'],elapsed_s=r['time_s']-start,
                            force_N=r['finger_forces_N'],table_count=r['table_contact_count'],
                            table_force_N=r['table_force_N'],block_bottom_lift_m=r['block_bottom_lift_m'])
        return None
    det=event(lambda r:r['table_contact_count']==0 and r['table_force_N']==0)
    tail=[r for r in lift if det is not None and r['time_s']>=det['time_s']]
    last_table=max((i for i,r in enumerate(lift) if r['table_contact_count']>0 or r['table_force_N']>0),default=-1)
    terminal_free=lift[last_table+1:]
    # First detachment can be followed by recontact; count only the terminal free interval.
    return dict(classification=lift_outcome(rows,case['lift_success']),failure=case['failure'],
        mode=MODE,hardware_execution=False,live_task_success=False,copied_lift_success=case['lift_success'],
        matched_reproduction_exact=case['matched_reproduction_exact'],model_unchanged=case['model_unchanged'],
        source_hashes=case['source_hashes'],noslip=case['options']['noslip_iterations'],timestep_s=dt,
        phase_steps={p:sum(r['phase']==p for r in rows) for p in dict.fromkeys(r['phase'] for r in rows)},
        confirm_end=next(r for r in reversed(rows) if r['phase']=='GRASP_CONFIRM'),
        first_table_detachment=det,first_force_loss=event(lambda r:min(r['finger_forces_N'])<=0),
        first_geometric_contact_loss=event(lambda r:any(v['contact_count']==0 for v in r['normalized']['pads'].values())),
        table_recontact_after_detachment=any(r['table_contact_count']>0 or r['table_force_N']>0 for r in tail),
        first_table_recontact=event(lambda r:det is not None and r['time_s']>det['time_s'] and (r['table_contact_count']>0 or r['table_force_N']>0)),
        continuous_support_free_start=event(lambda r:bool(terminal_free) and r['time_s']==terminal_free[0]['time_s']),
        observed_support_free_s=terminal_free[-1]['time_s']-terminal_free[0]['time_s'] if terminal_free else 0,
        minimum_forces_after_detachment_N=np.min([r['finger_forces_N'] for r in tail],axis=0).tolist() if tail else None,
        gripper_command_constant_during_lift=len({r['ctrl'][5] for r in lift})==1,
        maximum_bottom_lift_m=max(r['block_bottom_lift_m'] for r in rows),
        maximum_penetration_m=max(r['max_penetration_m'] for r in rows),
        maximum_abs_gripper_actuator_force_Nm=max(abs(r['actuator_force_Nm']) for r in rows),
        any_actuator_saturation=any(any(r['actuator_saturated']) for r in rows),
        maximum_warning_counts=np.max([r['warning_counts'] for r in rows],axis=0).tolist(),
        waypoint_gates=case['waypoint_gates'],final=rows[-1])


def experiment(report, baseline, source_path, continuation_path, donor_path, model_path, output,
               *, lift_reference=None):
    if (output/'experiment.json').exists():
        raise FileExistsError('Preserve completed or partial A/B evidence')
    if not report['less_actual_squeeze']:
        raise ValueError('No evidence authorizing an extra-squeeze experiment')
    source,cont,donor = [json.loads(p.read_text()) for p in (source_path,continuation_path,donor_path)]
    state = np.array(cont['close_stages'][-1]['terminal_state'])
    candidate = source['candidate_kinematic']; cases = {}
    for name in (('matched_budget',) if lift_reference is not None else ('baseline','matched_budget')):
        t = WaypointBlockTeacher(donor['candidate']); t.env.physics_observer=None; t.env.reset(seed=0)
        m=mujoco.MjModel.from_binary_path(str(model_path)); d=mujoco.MjData(m)
        assert m.opt.noslip_iterations == 0
        before_arrays,before_options=array_hashes(m),options(m)
        t.m=m;t.d=d;t.env.model=m;t.env.data=d;restore(m,d,state)
        np.testing.assert_array_equal(integration_state(m,d),state)
        last_close=next(r for r in reversed(baseline) if r['phase']=='CLOSE')
        for field,value in [('qpos',last_close['raw_qpos']),('qvel',last_close['raw_qvel']),('ctrl',last_close['ctrl'])]:
            np.testing.assert_array_equal(getattr(d,field),value)
        t.site=m.site('left_cube_grasp').id;t.approach=np.array(candidate['target_TCP'])[:3,0]
        t.closing=np.array(candidate['target_TCP'])[:3,2];t.waypoint_xyz=np.array(candidate['target_TCP'])[:3,3]
        t.env.settle_info=donor['settle'];t.env._block_hold_s=0
        t.close_arm_reference=d.ctrl.copy();t.gripper_hold_reference=(d.ctrl[5],d.ctrl[11])
        t.close_start_tcp=d.site_xpos[t.site].copy();t.close_contact_origin=None
        t.execution_mode=MODE;rows=[];failure=None;reproduced=None;confirm_state=None;success=False
        def record(self):
            contacts=native_contacts(m,d);metrics=contact_metrics(contacts,d.xipos[t.block_body],d.xmat[t.block_body])
            row=dict(time_s=float(d.time),phase=t.phase,ctrl=d.ctrl.tolist(),raw_qpos=d.qpos.tolist(),
                raw_qvel=d.qvel.tolist(),jaw_opening_m=jaw_gap_m(m,d,'left'),
                actuator_force_Nm=float(d.actuator_force[5]),normalized=metrics,contacts=contacts,
                tcp_error_m=float(np.linalg.norm(d.site_xpos[t.site]-t.waypoint_xyz)),
                warning_counts=d.warning.number.tolist())
            if lift_reference is not None:
                task=t.env.metrics();velocity=np.zeros(6)
                mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,t.block_body,velocity,0)
                tables=[c for c in contacts if 'table' in c['names']]
                forces=[metrics['pads'][p]['summed_normal_force_N'] for p in PADS]
                limits=m.actuator_forcerange
                saturated=m.actuator_forcelimited & ((d.actuator_force<=limits[:,0]) | (d.actuator_force>=limits[:,1]))
                row.update(table_contact_count=len(tables),
                    table_force_N=sum(max(0.,c['wrench_contact'][0]) for c in tables),
                    finger_forces_N=forces,block_z_m=float(d.xipos[t.block_body,2]),
                    block_vz_m_s=float(velocity[5]),
                    block_bottom_lift_m=float(task['block_bottom_height_m']-t.env.settle_info['block_bottom_height_m']),
                    lift_supported=bool(task['lift_supported']),continuous_hold_s=float(task['continuous_hold_s']),
                    actuator_forces_Nm=d.actuator_force.tolist(),actuator_saturated=saturated.astype(bool).tolist(),
                    jaw_q_rad=float(d.qpos[5]),jaw_qvel_rad_s=float(d.qvel[5]),
                    max_penetration_m=max([0.]+[-c['distance_m'] for c in contacts]))
            rows.append(row)
            if len(rows)%25==0 or (lift_reference is not None and t.phase.startswith('LIFT') and len(rows)>1 and rows[-2]['phase']!=t.phase):
                if lift_reference is not None:
                    print('LIFT_TELEMETRY',t.phase,'t',row['time_s'],'Fn',row['finger_forces_N'],
                          'table',row['table_contact_count'],row['table_force_N'],
                          'lift_mm',row['block_bottom_lift_m']*1000,'vz_mm_s',row['block_vz_m_s']*1000,flush=True)
            if len(rows)%50==0:
                print(MODE,name,'step',len(rows),'Fn',[metrics['pads'][p]['summed_normal_force_N'] for p in PADS],
                      'jaw',d.qpos[5],'ctrl',d.ctrl[5],flush=True)
        t.record_step=types.MethodType(record,t)
        print(MODE,name,'START same CLOSE48 full-state',float(d.time),flush=True)
        try:
            if name=='matched_budget':
                t.phase='CLOSE';t.env.collision_phase='CLOSE';target=d.ctrl.copy()
                target[5]=report['matched_target_rad'];t.move(target,duration=.1)
            t.phase='GRASP_CONFIRM';t.env.collision_phase='GRASP_CONFIRM'
            for _ in range(50):
                t.env.apply_action(tuple(d.ctrl),physics_steps=1);t.record_step();checked=t.inspect_runtime()
                if not all(v>0 for v in checked['finger_force_N'].values()):
                    raise ValueError('bilateral contact lost during confirmation')
            if lift_reference is not None:
                previous=lift_reference['cases']['matched_budget']
                np.testing.assert_array_equal(integration_state(m,d),previous['final_state'])
                assert len(rows)==len(previous['rows'])
                for row,old in zip(rows,previous['rows']):
                    for key in ('raw_qpos','raw_qvel','ctrl','contacts'):
                        assert row[key]==old[key]
                reproduced=True;confirm_state=integration_state(m,d).tolist()
                print(MODE,'MATCHED CLOSE + CONFIRM exact reproduction PASS',flush=True)
                # Reuse the standard teacher's solve/move path and retained gripper command.
                # No kinematic state placement or force-gate bypass is allowed during lift.
                t.close_arm_reference=None;grasp=t.waypoint_xyz.copy()
                for phase,height in t.lift_targets:
                    t.transition(phase)
                    t.move(t.solve(grasp+np.array([0.,0.,height])),duration=.5)
                t.transition('HOLD');target=tuple(d.ctrl);hold_start=float(d.time)
                for _ in range(math.ceil(3.2/m.opt.timestep)):
                    t.env.apply_action(target,physics_steps=1);t.record_step();checked=t.inspect_runtime()
                    if checked['metrics']['success'] and d.time-hold_start>=3.0:
                        success=True;break
                if not success:
                    raise ValueError('continuous 3-second supported lift success condition not reached')
        except (ValueError,RuntimeError) as e:
            failure=str(e)
        assert before_arrays==array_hashes(m) and before_options==options(m)
        if name=='baseline':
            expected=[r for r in baseline if r['phase']=='GRASP_CONFIRM']
            assert len(rows)==len(expected)
            for row,old in zip(rows,expected):
                for key in ('raw_qpos','raw_qvel','ctrl'):
                    np.testing.assert_array_equal(row[key],old[key])
                assert row['contacts']==old['contacts']
        cases[name]=dict(initial_state=state.tolist(),final_state=integration_state(m,d).tolist(),
            failure=failure,rows=rows,model_unchanged=True,options=before_options,
            waypoint_gates=t.report.get('waypoint_gates', []),plans=t.report.get('plans', []),
            lift_attempted=lift_reference is not None,matched_reproduction_exact=reproduced,
            confirm_final_state=confirm_state,lift_success=success,
            classification=lift_outcome(rows,success) if lift_reference is not None else None,
            source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in (source_path,continuation_path,donor_path,model_path,Path(__file__))})
        (output/'experiment.json').write_text(json.dumps(dict(mode=MODE,task_success=False,cases=cases),indent=2))
        if lift_reference is not None:
            (output/'lift-summary.json').write_text(json.dumps(lift_summary(cases[name]),indent=2))
        print(name,'DONE',failure,'steps',len(rows),flush=True)
    return cases


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('a0','a2','model','source','continuation','donor','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--run-one-ab',action='store_true')
    p.add_argument('--lift-from',type=Path,help='Previous squeeze evidence directory; reproduce candidate once then standard lift')
    a=p.parse_args()
    if a.lift_from and a.run_one_ab:p.error('choose original A/B or matched-lift continuation, not both')
    a.output.mkdir(parents=True,exist_ok=False)
    if a.lift_from:
        report=json.loads((a.lift_from/'audit.json').read_text())
        baseline=[json.loads(line) for line in a.a2.open()]
        for path in (a.a2,a.model):
            assert hashlib.sha256(path.read_bytes()).hexdigest()==report['sources'][str(path)]
    else:
        report,baseline=audit(a.a0,a.a2,a.model)
    (a.output/'audit.json').write_text(json.dumps(report,indent=2))
    if a.lift_from:
        print('REPRODUCE cached matched budget',report['matched_target_rad'],'then standard LIFT',flush=True)
    else:
        for name,c in report['cases'].items():
            print(name,{k:v for k,v in c.items() if k!='post_bilateral'},flush=True)
    if a.lift_from:
        experiment(report,baseline,a.source,a.continuation,a.donor,a.model,a.output,
                   lift_reference=json.loads((a.lift_from/'experiment.json').read_text()))
    elif a.run_one_ab:
        experiment(report,baseline,a.source,a.continuation,a.donor,a.model,a.output)

if __name__=='__main__':main()
