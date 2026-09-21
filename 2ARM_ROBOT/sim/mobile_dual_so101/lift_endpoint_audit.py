#!/usr/bin/env python3
"""Bounded same-command continuation of a saved LIFT15 endpoint; diagnostic only."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import mujoco
import numpy as np
from close_contact_diagnostic import restore
from controlled_contact_audit import MODE, PADS, array_hashes, contact_metrics, native_contacts, options
from dynamic_preflight import integration_state
from mobile_dual_so101 import actuator_targets_from_qpos, apply_control_as_pose
from pgripper import jaw_gap_m
from waypoint_block_teacher import WaypointBlockTeacher


def error_components(target, planned, measured):
    # Residuals add as vectors; adding their norms would misattribute tracking error.
    p, e = np.asarray(planned)-target, np.asarray(measured)-planned
    return dict(planned_vector_m=p.tolist(), tracking_vector_m=e.tolist(),
                total_vector_m=(p+e).tolist(), planned_m=float(np.linalg.norm(p)),
                tracking_m=float(np.linalg.norm(e)), total_m=float(np.linalg.norm(p+e)))


def run(a):
    a.output.mkdir(parents=True, exist_ok=False)
    case=json.loads(a.experiment.read_text())['cases']['matched_budget']
    donor=json.loads(a.donor.read_text()); source=json.loads(a.source.read_text())
    plan=next(p for p in reversed(case['plans']) if p['phase']=='LIFT_15MM' and 'ik' in p)
    assert case['failure']=='measured waypoint position tolerance failed'
    t=WaypointBlockTeacher(donor['candidate']);t.env.physics_observer=None;t.env.reset(seed=0)
    m=mujoco.MjModel.from_binary_path(str(a.model));d=mujoco.MjData(m)
    before_arrays,before_options=array_hashes(m),options(m)
    assert m.opt.noslip_iterations==0
    t.m=m;t.d=d;t.env.model=m;t.env.data=d
    state=np.asarray(case['final_state']);restore(m,d,state)
    np.testing.assert_array_equal(integration_state(m,d),state)
    t.site=m.site('left_cube_grasp').id;t.phase='LIFT_15MM';t.env.collision_phase=t.phase
    t.env.settle_info=donor['settle'];t.env._block_hold_s=0
    t.waypoint_xyz=np.asarray(plan['xyz']);target=t.waypoint_xyz
    axes=np.asarray(source['candidate_kinematic']['target_TCP'])[:3,:3]
    t.approach=axes[:,0];t.closing=axes[:,2]
    command=d.ctrl.copy();qtarget=np.asarray(plan['ik']['action_rad'])
    preview=mujoco.MjData(m);preview.qpos[:]=d.qpos
    apply_control_as_pose(m,preview,qtarget)
    planned=preview.site_xpos[t.site].copy();planned_R=preview.site_xmat[t.site].reshape(3,3).copy()
    apply_control_as_pose(m,preview,command)
    command_xyz=preview.site_xpos[t.site].copy()
    jac=np.zeros((3,m.nv));mujoco.mj_jacSite(m,preview,jac,None,t.site)
    qa=m.jnt_qposadr[m.actuator_trnid[:,0]];va=m.jnt_dofadr[m.actuator_trnid[:,0]]
    start=float(d.time)
    def sample(data):
        c=native_contacts(m,data);n=contact_metrics(c,data.xipos[t.block_body],data.xmat[t.block_body])
        table=[v for v in c if 'table' in v['names']]
        rotation=data.site_xmat[t.site].reshape(3,3)
        vel=np.zeros(6);mujoco.mj_objectVelocity(m,data,mujoco.mjtObj.mjOBJ_BODY,t.block_body,vel,0)
        bottom=data.geom_xpos[t.block,2]-abs(data.geom_xmat[t.block].reshape(3,3)[2])@m.geom_size[t.block]
        limits=m.actuator_forcerange
        error=error_components(target,planned,data.site_xpos[t.site])
        return dict(time_s=float(data.time),elapsed_s=float(data.time-start),errors=error,
            tcp_xyz_m=data.site_xpos[t.site].tolist(),tcp_rotation=rotation.tolist(),
            approach_error_rad=float(np.arccos(np.clip(rotation[:,0]@t.approach,-1,1))),
            closing_error_rad=float(np.arccos(np.clip(rotation[:,2]@t.closing,-1,1))),
            ctrl=data.ctrl.tolist(),measured_q=list(actuator_targets_from_qpos(m,data.qpos)),
            raw_qpos=data.qpos.tolist(),raw_qvel=data.qvel.tolist(),joint_qvel=data.qvel[va].tolist(),
            tracking_q_rad=(data.qpos[qa]-data.ctrl).tolist(),
            actuator_force_Nm=data.actuator_force.tolist(),
            saturation=(m.actuator_forcelimited & ((data.actuator_force<=limits[:,0])|(data.actuator_force>=limits[:,1]))).astype(bool).tolist(),
            jaw_opening_m=jaw_gap_m(m,data,'left'),finger_force_N=[n['pads'][p]['summed_normal_force_N'] for p in PADS],
            table_count=len(table),table_force_N=sum(max(0.,c['wrench_contact'][0]) for c in table),
            bottom_lift_m=float(bottom-t.env.settle_info['block_bottom_height_m']),block_vz_m_s=float(vel[5]),
            penetration_m=max([0.]+[-float(c.dist) for c in data.contact]),warnings=data.warning.number.tolist())
    # FK writes are isolated to preview data; the physical continuation keeps the saved full state.
    history=[]
    for row in case['rows']:
        if row['phase']=='LIFT_15MM' and row['time_s']>=start-.100:
            preview.qpos[:]=row['raw_qpos'];preview.qvel[:]=row['raw_qvel'];preview.ctrl[:]=row['ctrl'];preview.time=row['time_s']
            mujoco.mj_forward(m,preview);history.append(sample(preview))
    rows=[sample(d)];failure=None
    np.testing.assert_array_equal(integration_state(m,d),state)
    print(MODE,'same terminal command; bounded 0.5 s observation',flush=True)
    try:
        for step in range(round(.5/m.opt.timestep)):
            try:
                t.env.apply_action(tuple(command),physics_steps=1)
                t.inspect_runtime()
            finally:
                rows.append(sample(d))
            np.testing.assert_array_equal(d.ctrl,command)
            if (step+1)%25==0:
                r=rows[-1];print('LIFT15 SAME-COMMAND',step+1,'TCP mm',r['errors']['total_m']*1000,'Fn',r['finger_force_N'],'table',r['table_count'],'lift mm',r['bottom_lift_m']*1000,flush=True)
    except (ValueError,RuntimeError) as e:failure=str(e)
    unchanged=before_arrays==array_hashes(m) and before_options==options(m)
    assert unchanged
    first=next((r['elapsed_s'] for r in rows if r['errors']['total_m']<=.0005 and r['approach_error_rad']<=math.radians(2) and r['closing_error_rad']<=math.radians(15)),None)
    report=dict(mode=MODE,task_success=False,phase='LIFT_15MM',continued_to_30mm=False,
        duration_s=float(d.time-start),failure=failure,model_unchanged=unchanged,options=before_options,
        target_xyz_m=target.tolist(),planned_xyz_m=planned.tolist(),command_xyz_m=command_xyz.tolist(),
        planned_rotation=planned_R.tolist(),target_q=qtarget.tolist(),command_q=command.tolist(),
        site_body=m.body(int(m.site_bodyid[t.site])).name,
        joint_names=[m.joint(int(j)).name for j in m.actuator_trnid[:,0]],
        joint_tcp_linear_contributions_m=(jac[:,va]*(np.asarray(rows[0]['measured_q'])-command)).T.tolist(),
        first_gate_pass_s=first,history_last_100ms=history,rows=rows,
        initial_state=state.tolist(),final_state=integration_state(m,d).tolist(),
        sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (a.model,a.experiment,a.source,a.donor,Path(__file__))})
    (a.output/'endpoint-hold.json').write_text(json.dumps(report,indent=2))
    print('DONE','first gate PASS',first,'failure',failure,'initial/final TCP mm',rows[0]['errors']['total_m']*1000,rows[-1]['errors']['total_m']*1000,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('experiment','model','source','donor','output'):p.add_argument('--'+name,type=Path,required=True)
    run(p.parse_args())
