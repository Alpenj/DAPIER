#!/usr/bin/env python3
"""One identical-state NoSlip0/5 airborne HOLD comparison; diagnostic only."""
import argparse
import hashlib
import json
from pathlib import Path
import types
from unittest.mock import patch
import mujoco
import numpy as np
from close_contact_diagnostic import restore
from controlled_contact_audit import MODE, PADS, array_hashes, contact_metrics, native_contacts, options
from dynamic_preflight import integration_state
from pgripper import jaw_gap_m
from waypoint_block_teacher import WaypointBlockTeacher


def elliptic_utilization(contact):
    """Full condim cone norm, including torsional/rolling components when present."""
    force=np.asarray(contact['wrench_contact'],float)
    mu=np.asarray(contact['friction'],float)[:contact['condim']-1]
    tangent=force[1:contact['condim']]
    if not np.isfinite(force).all() or not np.isfinite(mu).all() or np.any(mu<0):
        raise ValueError('invalid contact force/friction')
    if force[0]<=0:return None
    if np.any((mu==0)&(tangent!=0)):raise ValueError('force on a zero-friction axis')
    ratio=np.divide(tangent,force[0]*mu,out=np.zeros_like(tangent),where=mu>0)
    return float(np.linalg.norm(ratio))


def force_sample(t):
    m,d=t.m,t.d;contacts=native_contacts(m,d)
    n=contact_metrics(contacts,d.xipos[t.block_body],d.xmat[t.block_body])
    table=[c for c in contacts if 'table' in c['names']]
    util=[elliptic_utilization(c) for c in contacts if any(p in c['names'] for p in PADS)]
    return dict(contacts=contacts,normalized=n,
        forces_N=[n['pads'][p]['summed_normal_force_N'] for p in PADS],
        table_count=len(table),table_force_N=sum(max(0.,c['wrench_contact'][0]) for c in table),
        finger_vertical_resultant_N=n['F_net_world_N'][2],
        net_vertical_with_gravity_N=n['F_net_world_N'][2]+float(m.body_mass[t.block_body]*m.opt.gravity[2]),
        friction_utilization=util)


def teacher(model_path,donor,source,state,noslip,phase,xyz,hold_s):
    t=WaypointBlockTeacher(donor['candidate']);t.env.physics_observer=None;t.env.reset(seed=0)
    m=mujoco.MjModel.from_binary_path(str(model_path));m.opt.noslip_iterations=noslip;d=mujoco.MjData(m)
    t.m=m;t.d=d;t.env.model=m;t.env.data=d;restore(m,d,state)
    np.testing.assert_array_equal(integration_state(m,d),state)
    t.site=m.site('left_cube_grasp').id;t.env.settle_info=donor['settle'];t.env._block_hold_s=hold_s
    t.transition(phase);t.execution_mode=MODE;t.waypoint_xyz=np.asarray(xyz)
    T=np.asarray(source['candidate_kinematic']['target_TCP'])
    t.approach=T[:3,0];t.closing=T[:3,2]
    t.gripper_hold_reference=(d.ctrl[5],d.ctrl[11]);t.close_arm_reference=None
    return t


def checkpoint(a,prior,donor,source):
    # Recorded qpos/qvel alone omit warmstart. Replay the saved LIFT15 full state
    # and require every existing LIFT30 sample to match before saving the new checkpoint.
    plan=next(p for p in prior['plans'] if p['phase']=='LIFT_30MM' and 'ik' in p)
    t=teacher(a.model,donor,source,prior['lift15_pass_state'],0,'LIFT_30MM',plan['xyz'],0.)
    old=[r for r in prior['rows'] if r['phase']=='LIFT_30MM'];index=0
    def record(self):
        nonlocal index
        r=old[index]
        for field,key in [('qpos','raw_qpos'),('qvel','raw_qvel'),('ctrl','ctrl')]:
            np.testing.assert_array_equal(getattr(t.d,field),r[key])
        np.testing.assert_array_equal(force_sample(t)['forces_N'],r['finger_forces_N'])
        assert t.d.time==r['time_s']
        index+=1
        if index%100==0:print('CHECKPOINT REPLAY',index,'/',len(old),'bitwise trace PASS',flush=True)
    t.record_step=types.MethodType(record,t)
    t.move(plan['ik']['action_rad'],duration=.5);assert index==len(old)
    t.transition('HOLD')
    result=dict(state=integration_state(t.m,t.d).tolist(),state_spec='mjSTATE_INTEGRATION',
        prior_hold_counter_s=t.env._block_hold_s,target_xyz_m=plan['xyz'],ctrl=t.d.ctrl.tolist(),
        time_s=float(t.d.time),replayed_steps=index,prior_trace_bitwise_match=True,
        model_arrays=array_hashes(t.m),model_options=options(t.m))
    result['state_sha256']=hashlib.sha256(np.asarray(result['state']).tobytes()).hexdigest()
    (a.output/'hold-start.json').write_text(json.dumps(result,indent=2))
    print('EXACT HOLD START SAVED',result['time_s'],result['state_sha256'],flush=True)
    return result


def run(a):
    a.output.mkdir(parents=True,exist_ok=False)
    prior=json.loads(a.experiment.read_text())['cases']['refined']
    donor=json.loads(a.donor.read_text());source=json.loads(a.source.read_text())
    start=checkpoint(a,prior,donor,source)
    expected=[r for r in prior['rows'] if r['phase']=='HOLD']
    result=dict(mode=MODE,live_task_success=False,hardware_execution=False,
        mujoco_version=mujoco.__version__,checkpoint_state_sha256=start['state_sha256'],
        sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (a.experiment,a.model,a.donor,a.source,Path(__file__))},cases={})
    for noslip in (0,5):
        t=teacher(a.model,donor,source,start['state'],noslip,'HOLD',start['target_xyz_m'],start['prior_hold_counter_s'])
        m,d=t.m,t.d;before=options(m);arrays=array_hashes(m)
        assert arrays==start['model_arrays']
        assert {k:v for k,v in before.items() if k!='noslip_iterations'}=={k:v for k,v in start['model_options'].items() if k!='noslip_iterations'}
        assert m.opt.cone==mujoco.mjtCone.mjCONE_ELLIPTIC
        assert m.opt.integrator==mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        joint=int(m.body_jntadr[t.block_body]);dof=int(m.jnt_dofadr[joint])
        assert m.jnt_type[joint]==mujoco.mjtJoint.mjJNT_FREE
        np.testing.assert_array_equal(m.body_ipos[t.block_body],np.zeros(3))
        assert np.all(m.dof_damping[dof:dof+6]==0)
        command=tuple(start['ctrl']);rows=[];failure=None;applied=None
        initial=dict(time_s=float(d.time),block_com_m=d.xipos[t.block_body].tolist(),
            block_quat=d.xquat[t.block_body].tolist(),bottom_m=float(t.env.metrics()['block_bottom_height_m']),
            vz_m_s=float(d.qvel[dof+2]),contact=force_sample(t),qpos=d.qpos.tolist(),qvel=d.qvel.tolist(),ctrl=d.ctrl.tolist())
        original_step=mujoco.mj_step
        def step_and_capture(model,data):
            nonlocal applied
            time_before=float(data.time);vz_before=float(data.qvel[dof+2])
            original_step(model,data)
            # Capture the solver cache before the teacher's existing mj_forward;
            # applied forces and post-forward gate forces describe different evaluation times.
            applied=force_sample(t)
            applied.update(force_time_s=time_before,integrated_time_s=float(data.time),
                acceleration_from_dv_m_s2=float((data.qvel[dof+2]-vz_before)/m.opt.timestep))
        print(MODE,'NoSlip',noslip,'HOLD 3 s START',float(d.time),flush=True)
        np.testing.assert_array_equal(integration_state(m,d),start['state'])
        with (a.output/f'noslip{noslip}.jsonl').open('w') as stream,patch.object(mujoco,'mj_step',step_and_capture):
            try:
                for i in range(round(3./m.opt.timestep)):
                    t.env.apply_action(command,physics_steps=1)
                    gate=force_sample(t);metrics=t.env.metrics();limits=m.actuator_forcerange
                    row=dict(step=i+1,time_s=float(d.time),elapsed_s=float(d.time-start['time_s']),
                        ctrl=d.ctrl.tolist(),raw_qpos=d.qpos.tolist(),raw_qvel=d.qvel.tolist(),
                        block_com_m=d.xipos[t.block_body].tolist(),block_quat=d.xquat[t.block_body].tolist(),
                        block_z_m=float(d.xipos[t.block_body,2]),block_vz_m_s=float(d.qvel[dof+2]),
                        bottom_m=float(metrics['block_bottom_height_m']),jaw_opening_m=jaw_gap_m(m,d,'left'),
                        applied=applied,gate=gate,continuous_hold_s=float(metrics['continuous_hold_s']),
                        lift_supported=bool(metrics['lift_supported']),success=bool(metrics['success']),
                        tcp_error_m=float(np.linalg.norm(d.site_xpos[t.site]-t.waypoint_xyz)),
                        actuator_force_Nm=d.actuator_force.tolist(),
                        saturation=(m.actuator_forcelimited & ((d.actuator_force<=limits[:,0])|(d.actuator_force>=limits[:,1]))).astype(bool).tolist(),
                        penetration_m=max([0.]+[-float(c.dist) for c in d.contact]),warnings=d.warning.number.tolist())
                    rows.append(row);stream.write(json.dumps(row)+'\n')
                    np.testing.assert_array_equal(d.ctrl,command)
                    if noslip==0:
                        for k in ('raw_qpos','raw_qvel','ctrl'):np.testing.assert_array_equal(row[k],expected[i][k])
                        np.testing.assert_array_equal(gate['forces_N'],expected[i]['finger_forces_N'])
                    t.inspect_runtime()
                    if (i+1)%100==0:
                        print('HOLD NoSlip',noslip,'step',i+1,'Fn',gate['forces_N'],
                              'bottom mm',row['bottom_m']*1000,'vz mm/s',row['block_vz_m_s']*1000,
                              'table',gate['table_count'],'Fz',gate['finger_vertical_resultant_N'],flush=True)
            except (ValueError,RuntimeError) as e:failure=str(e)
        assert arrays==array_hashes(m) and before==options(m)
        result['cases'][str(noslip)]=dict(initial=initial,initial_state_exact=True,
            options=before,model_arrays_unchanged=True,only_noslip_changed=True,
            rows_file=f'noslip{noslip}.jsonl',steps=len(rows),failure=failure,
            prior_hold_bitwise_match=noslip==0 and len(rows)==1500,
            final_state=integration_state(m,d).tolist(),mass_kg=float(m.body_mass[t.block_body]),
            gravity_m_s2=m.opt.gravity.tolist(),
            copied_hold_pass=(failure is None and len(rows)==1500 and all(r['lift_supported'] and r['gate']['table_count']==0 and r['gate']['table_force_N']==0 for r in rows) and rows[-1]['success']))
        (a.output/'ab.json').write_text(json.dumps(result,indent=2))
        print('HOLD NoSlip',noslip,'DONE',failure,'copied HOLD PASS',result['cases'][str(noslip)]['copied_hold_pass'],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('experiment','model','source','donor','output'):p.add_argument('--'+name,type=Path,required=True)
    run(p.parse_args())
