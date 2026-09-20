#!/usr/bin/env python3
"""One saved-state squeeze-budget comparison; never changes task policy."""
import argparse
import hashlib
import json
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


def experiment(report, baseline, source_path, continuation_path, donor_path, model_path, output):
    if (output/'experiment.json').exists():
        raise FileExistsError('Preserve completed or partial A/B evidence')
    if not report['less_actual_squeeze']:
        raise ValueError('No evidence authorizing an extra-squeeze experiment')
    source,cont,donor = [json.loads(p.read_text()) for p in (source_path,continuation_path,donor_path)]
    state = np.array(cont['close_stages'][-1]['terminal_state'])
    candidate = source['candidate_kinematic']; cases = {}
    for name in ('baseline','matched_budget'):
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
        t.execution_mode=MODE;rows=[];failure=None
        def record(self):
            contacts=native_contacts(m,d);metrics=contact_metrics(contacts,d.xipos[t.block_body],d.xmat[t.block_body])
            row=dict(time_s=float(d.time),phase=t.phase,ctrl=d.ctrl.tolist(),raw_qpos=d.qpos.tolist(),
                raw_qvel=d.qvel.tolist(),jaw_opening_m=jaw_gap_m(m,d,'left'),
                actuator_force_Nm=float(d.actuator_force[5]),normalized=metrics,contacts=contacts,
                tcp_error_m=float(np.linalg.norm(d.site_xpos[t.site]-t.waypoint_xyz)),
                warning_counts=d.warning.number.tolist())
            rows.append(row)
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
            waypoint_gates=t.report.get('waypoint_gates', []),
            source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in (source_path,continuation_path,donor_path,model_path,Path(__file__))})
        (output/'experiment.json').write_text(json.dumps(dict(mode=MODE,task_success=False,cases=cases),indent=2))
        print(name,'DONE',failure,'steps',len(rows),flush=True)
    return cases


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('a0','a2','model','source','continuation','donor','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--run-one-ab',action='store_true');a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    report,baseline=audit(a.a0,a.a2,a.model)
    (a.output/'audit.json').write_text(json.dumps(report,indent=2))
    for name,c in report['cases'].items():
        print(name,{k:v for k,v in c.items() if k!='post_bilateral'},flush=True)
    if a.run_one_ab:
        experiment(report,baseline,a.source,a.continuation,a.donor,a.model,a.output)

if __name__=='__main__':main()
