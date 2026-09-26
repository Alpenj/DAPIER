#!/usr/bin/env python3
"""One copied endpoint A/B; refine planning once without changing acceptance."""
import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path
import types
import mujoco
import numpy as np
from close_contact_diagnostic import restore
from collision_guard import check_bimanual_path, task_clearance_status
from controlled_contact_audit import MODE, PADS, array_hashes, contact_metrics, native_contacts, options
from dynamic_preflight import integration_state
from lift_endpoint_audit import error_components
from mobile_dual_so101 import actuator_targets_from_qpos, apply_control_as_pose
from pgripper import jaw_gap_m
from physics_ik import solve_bimanual_position_ik, axis_direction_task
from shoe_task import measured_joint_state
from waypoint_block_teacher import WaypointBlockTeacher


def refine_once(m, d, site, q, xyz, axis):
    p=mujoco.MjData(m);p.qpos[:]=d.qpos;apply_control_as_pose(m,p,q)
    jp=np.zeros((3,m.nv));jw=np.zeros((3,m.nv));mujoco.mj_jacSite(m,p,jp,jw,site)
    dofs=m.jnt_dofadr[m.actuator_trnid[:5,0]]
    ja,ea=axis_direction_task(p.site_xmat[site].reshape(3,3)[:,0],axis,jw[:,dofs])
    defaults=inspect.signature(solve_bimanual_position_ik).parameters
    settings={k:defaults[k].default for k in ('damping','tool_axis_weight_m','max_joint_step_rad')}
    w=settings['tool_axis_weight_m'];J=np.vstack((jp[:,dofs],w*ja))
    e=np.concatenate((xyz-p.site_xpos[site],w*ea))
    # Existing IK stops on acceptance, not the local residual minimum.
    # Continue its position+axis DLS update once; no target offset or gate change.
    delta=J.T@np.linalg.solve(J@J.T+settings['damping']**2*np.eye(len(e)),e)
    out=np.asarray(q).copy();limit=settings['max_joint_step_rad']
    out[:5]=np.clip(out[:5]+np.clip(delta,-limit,limit),m.actuator_ctrlrange[:5,0],m.actuator_ctrlrange[:5,1])
    return out,settings


def plan_report(t,q):
    p=mujoco.MjData(t.m);p.qpos[:]=t.d.qpos;apply_control_as_pose(t.m,p,q)
    measured_joint_state(t.m,p,integration_desk=True)
    R=p.site_xmat[t.site].reshape(3,3);err=p.site_xpos[t.site]-t.waypoint_xyz
    approach=float(np.arccos(np.clip(R[:,0]@t.approach,-1,1)))
    closing=float(np.arccos(np.clip(R[:,2]@t.closing,-1,1)))
    guard=check_bimanual_path(t.m,actuator_targets_from_qpos(t.m,t.d.qpos),q,
        task_phase=t.env.collision_phase,reference_data=t.d)
    status=task_clearance_status(t.m,p,t.env.collision_phase)
    margins=[min(float(x-lo),float(hi-x)) for x,(lo,hi) in zip(q,t.m.jnt_range[t.m.actuator_trnid[:,0]])]
    if not (np.linalg.norm(err)<=.0005 and approach<=math.radians(2) and closing<=math.radians(15)
            and min(margins)>=0 and guard.safe and status['safe']):
        raise ValueError('unchanged endpoint / joint / collision criteria rejected candidate')
    return dict(q=list(q),xyz=p.site_xpos[t.site].tolist(),position_vector_m=err.tolist(),
        position_error_m=float(np.linalg.norm(err)),approach_error_rad=approach,closing_error_rad=closing,
        joint_margins_rad=margins,guard=guard.as_report(),task_policy=status)


def run(a):
    a.output.mkdir(parents=True,exist_ok=False)
    original=json.loads(a.experiment.read_text())['cases']['matched_budget']
    donor=json.loads(a.donor.read_text());source=json.loads(a.source.read_text())
    oldplan=next(x for x in reversed(original['plans']) if x['phase']=='LIFT_15MM' and 'ik' in x)
    state=np.asarray(original['final_state']);cases={}
    evidence=dict(mode=MODE,live_task_success=False,hardware_execution=False,
        initial_source='same saved failed LIFT15 endpoint, not a new HOME-to-LIFT rollout',
        initial_state=state.tolist(),original_ik=oldplan['ik'],cases=cases,
        sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (a.model,a.experiment,a.source,a.donor,Path(__file__))})
    for name in ('original','refined'):
        t=WaypointBlockTeacher(donor['candidate']);t.env.physics_observer=None;t.env.reset(seed=0)
        m=mujoco.MjModel.from_binary_path(str(a.model));d=mujoco.MjData(m)
        before=(array_hashes(m),options(m));assert m.opt.noslip_iterations==0
        t.m=m;t.d=d;t.env.model=m;t.env.data=d;restore(m,d,state)
        np.testing.assert_array_equal(integration_state(m,d),state)
        t.site=m.site('left_cube_grasp').id;t.env.settle_info=donor['settle'];t.env._block_hold_s=0
        t.transition('LIFT_15MM');t.execution_mode=MODE
        T=np.asarray(source['candidate_kinematic']['target_TCP'])
        t.approach=T[:3,0];t.closing=T[:3,2];t.waypoint_xyz=np.asarray(oldplan['xyz'])
        t.gripper_hold_reference=(d.ctrl[5],d.ctrl[11]);t.close_arm_reference=None
        oldq=np.asarray(oldplan['ik']['action_rad']);q=oldq.copy();settings=None
        if name=='refined':q,settings=refine_once(m,d,t.site,oldq,t.waypoint_xyz,t.approach)
        np.testing.assert_array_equal(q[5:],oldq[5:])
        plan=plan_report(t,q)
        if name=='refined' and plan['position_error_m']>=cases['original']['plan']['position_error_m']:
            raise ValueError('one-step refinement did not reduce position residual')
        np.testing.assert_array_equal(integration_state(m,d),state)
        rows=[];failure=None;success=False;endpoint_state=None
        planned_xyz=np.asarray(plan['xyz']);start=float(d.time)
        def record(self):
            contacts=native_contacts(m,d);n=contact_metrics(contacts,d.xipos[t.block_body],d.xmat[t.block_body])
            metrics=t.env.metrics();v=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,t.block_body,v,0)
            tables=[c for c in contacts if 'table' in c['names']];R=d.site_xmat[t.site].reshape(3,3)
            limits=m.actuator_forcerange
            row=dict(time_s=float(d.time),elapsed_s=float(d.time-start),phase=t.phase,
                target_xyz_m=t.waypoint_xyz.tolist(),errors=error_components(t.waypoint_xyz,planned_xyz,d.site_xpos[t.site]),
                approach_error_rad=float(np.arccos(np.clip(R[:,0]@t.approach,-1,1))),
                closing_error_rad=float(np.arccos(np.clip(R[:,2]@t.closing,-1,1))),
                ctrl=d.ctrl.tolist(),raw_qpos=d.qpos.tolist(),raw_qvel=d.qvel.tolist(),
                measured_q=list(actuator_targets_from_qpos(m,d.qpos)),tcp_xyz=d.site_xpos[t.site].tolist(),
                finger_forces_N=[n['pads'][p]['summed_normal_force_N'] for p in PADS],
                table_count=len(tables),table_force_N=sum(max(0.,c['wrench_contact'][0]) for c in tables),
                block_z_m=float(d.xipos[t.block_body,2]),block_vz_m_s=float(v[5]),
                bottom_lift_m=float(metrics['block_bottom_height_m']-t.env.settle_info['block_bottom_height_m']),
                continuous_hold_s=float(metrics['continuous_hold_s']),lift_supported=bool(metrics['lift_supported']),
                jaw_opening_m=jaw_gap_m(m,d,'left'),actuator_force_Nm=d.actuator_force.tolist(),
                saturation=(m.actuator_forcelimited & ((d.actuator_force<=limits[:,0])|(d.actuator_force>=limits[:,1]))).astype(bool).tolist(),
                penetration_m=max([0.]+[-float(c.dist) for c in d.contact]),warnings=d.warning.number.tolist())
            rows.append(row)
            if len(rows)%50==0:
                print(MODE,name,t.phase,'step',len(rows),'TCP mm',row['errors']['total_m']*1000,
                      'Fn',row['finger_forces_N'],'table',row['table_count'],'lift mm',row['bottom_lift_m']*1000,flush=True)
        t.record_step=types.MethodType(record,t)
        print(MODE,name,'planned mm',plan['position_error_m']*1000,'approach deg',math.degrees(plan['approach_error_rad']),flush=True)
        t.record_step()
        try:
            t.move(q,duration=.5)
            if min(rows[-1]['finger_forces_N'])<=0 or rows[-1]['table_count'] or rows[-1]['table_force_N']:
                raise ValueError('LIFT15 continuation requires bilateral table-free support')
            endpoint_state=integration_state(m,d).tolist()
            if name=='refined':
                # Keep the standard next target/gates; LIFT30 is not another refinement search.
                phase,height=t.lift_targets[-1];t.transition(phase)
                nextq=t.solve(T[:3,3]+np.array([0.,0.,height]));nextplan=plan_report(t,nextq)
                planned_xyz=np.asarray(nextplan['xyz'])
                t.move(nextq,duration=.5)
                t.transition('HOLD');command=tuple(d.ctrl);hold_start=float(d.time)
                for _ in range(math.ceil(3.2/m.opt.timestep)):
                    t.env.apply_action(command,physics_steps=1);t.record_step();checked=t.inspect_runtime()
                    if checked['metrics']['success'] and d.time-hold_start>=3.0:
                        success=True;break
                if not success:raise ValueError('continuous 3-second supported lift not reached')
        except (ValueError,RuntimeError) as e:failure=str(e)
        assert before==(array_hashes(m),options(m))
        cases[name]=dict(plan=plan,refinement_settings=settings,failure=failure,rows=rows,
            final_state=integration_state(m,d).tolist(),lift15_pass_state=endpoint_state,
            waypoint_gates=t.report.get('waypoint_gates',[]),plans=t.report['plans'],
            model_unchanged=True,options=options(m),copied_success=success)
        (a.output/'ab.json').write_text(json.dumps(evidence,indent=2))
        print(name,'DONE',t.phase,'failure',failure,'copied success',success,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('experiment','model','source','donor','output'):p.add_argument('--'+k,type=Path,required=True)
    run(p.parse_args())
