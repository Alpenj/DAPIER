"""Bounded copied-state contact evidence; never authorizes a live grasp."""
import argparse
import copy
import json
from pathlib import Path
import numpy as np
import mujoco
from close_contact_diagnostic import restore, MODE
from dynamic_preflight import integration_state, record_step, contact_telemetry
from mobile_dual_so101 import actuator_targets_from_qpos
from physics_ik import plan_septic_joint_trajectory
from waypoint_block_teacher import WaypointBlockTeacher
from integration_scenes import same_audited_desk_model


def point_velocity(t, body, point):
    jac=np.zeros((3,t.m.nv))
    mujoco.mj_jac(t.m,t.d,jac,None,np.asarray(point),int(body))
    return jac @ t.d.qvel


def contact_chronology(rows):
    first=lambda predicate:next((r['step'] for r in rows if predicate(r)),None)
    # Geometry contact and positive supporting force are different observations.
    # None is right-censored by this window, not proof of future stability.
    return dict(observed_steps=len(rows),
        table_support_loss=first(lambda r:r['block_table_normal_force_N']<=0),
        first_finger_support_loss=first(lambda r:not all(v>0 for v in r['finger_force_N'].values())),
        both_finger_support_loss=first(lambda r:not any(v>0 for v in r['finger_force_N'].values())),
        both_geometric_contacts_lost=first(lambda r:not any(r['finger_contacts'].values())))



def block_load_evidence(t, contacts):
    """Diagnostic wrench balance and optimistic friction bound, never a grasp gate."""
    up=-t.m.opt.gravity/np.linalg.norm(t.m.opt.gravity)
    rotation=t.d.xmat[t.block_body].reshape(3,3)
    com=t.d.xipos[t.block_body]
    finger_ids=set(t.fingers.values())
    force=np.zeros(3);torque=np.zeros(3);upper=0.;points=[]
    for c in contacts:
        g1,g2=c['geom_ids']
        if t.block not in (g1,g2) or not finger_ids.intersection((g1,g2)):continue
        frame=np.asarray(c['contact_frame_world']).reshape(3,3)
        wrench=np.asarray(c['contact_frame_wrench'])
        # MuJoCo's contact wrench acts on geom2; reverse it for a block in geom1.
        sign=1. if g2==t.block else -1.
        f=sign*frame.T@wrench[:3];moment=sign*frame.T@wrench[3:]
        lever=np.asarray(c['position_m'])-com
        force+=f;torque+=np.cross(lever,f)+moment
        normal=sign*frame[0]
        # This elliptic-cone upper bound ignores torque equilibrium and its shared
        # torsional budget. Exceeding weight is necessary, never sufficient for lift.
        bound=None
        if t.m.opt.cone==mujoco.mjtCone.mjCONE_ELLIPTIC and c['contact_dimension']>=3:
            bound=max(0.,wrench[0])*(up@normal+np.linalg.norm(np.asarray(c['friction_coefficients'][:2])*(frame[1:]@up)))
            upper+=bound
        else:upper=float('nan')
        points.append(dict(pad=t.m.geom(g2 if g1==t.block else g1).name,
            block_com_local_m=(rotation.T@lever).tolist(),
            force_on_block_world_N=f.tolist(),force_on_block_local_N=(rotation.T@f).tolist(),
            normal_on_block_world=normal.tolist(),vertical_upper_bound_N=bound))
    return dict(finger_contacts=points,finger_force_world_N=force.tolist(),
        finger_torque_about_com_world_Nm=torque.tolist(),
        finger_force_block_N=(rotation.T@force).tolist(),
        finger_torque_about_com_block_Nm=(rotation.T@torque).tolist(),
        finger_vertical_force_N=float(force@up),
        optimistic_vertical_upper_bound_N=float(upper),
        block_weight_N=float(t.m.body_mass[t.block_body]*np.linalg.norm(t.m.opt.gravity)),
        is_load_ready_gate=False)


def observe(t, previous, reference, origin, step, case):
    record_step(t)
    row=t.step_telemetry[-1]
    contact_telemetry(t,row,origin,reference,previous,np.asarray(row['measured_actuator_q']),case)
    for i,c in enumerate(t.d.contact):
        wrench=np.zeros(6);mujoco.mj_contactForce(t.m,t.d,i,wrench)
        row['contacts'][i]['contact_frame_wrench']=wrench.tolist()
        row['contacts'][i]['contact_frame_world']=c.frame.tolist()
        row['contacts'][i]['tangential_force_N']=wrench[1:3].tolist()
        row['contacts'][i]['friction_coefficients']=c.friction.tolist()
        row['contacts'][i]['contact_dimension']=int(c.dim)
        v1=point_velocity(t,t.m.geom_bodyid[c.geom1],c.pos)
        v2=point_velocity(t,t.m.geom_bodyid[c.geom2],c.pos)
        relative=c.frame.reshape(3,3) @ (v2-v1)
        row['contacts'][i].update(relative_velocity_contact_frame_m_s=relative.tolist(),
            relative_tangent_speed_m_s=float(np.linalg.norm(relative[1:])))
    row['load_evidence']=block_load_evidence(t,row['contacts'])
    row['tcp_linear_velocity_world_m_s']=point_velocity(t,t.m.site_bodyid[t.site],t.d.site_xpos[t.site]).tolist()
    table_top=t.d.geom_xpos[t.floor,2]+np.abs(t.d.geom_xmat[t.floor].reshape(3,3)[2]) @ t.m.geom_size[t.floor]
    row['block_bottom_table_gap_m']=float(t.env.metrics()['block_bottom_height_m']-table_top)
    faces=np.array([t.d.site(f'left_pgripper_pad_{i}_inner').xpos for i in (1,2)])
    closing_center=faces.mean(axis=0)
    forces=list(row['finger_force_N'].values())
    row.update(jaw_inner_face_centers_world_m=faces.tolist(),
        closing_center_world_m=closing_center.tolist(),
        block_center_minus_closing_center_m=(t.d.xpos[t.block_body]-closing_center).tolist(),
        finger_normal_force_balance_N=float(forces[0]-forces[1]))
    table=[c for c in row['contacts'] if set(c['geom_ids'])=={t.block,t.floor}]
    row.update(mode=MODE,case=case,step=step,
        command_delta_rad=(t.d.ctrl-previous).tolist(),
        maximum_command_delta_rad=float(np.max(np.abs(t.d.ctrl-previous))),
        gripper_command_delta_from_close_rad=float(t.d.ctrl[5]-reference[5]),
        tcp_world_rotation=t.d.site_xmat[t.site].reshape(3,3).tolist(),
        command_tcp_world_rotation=t.command_preview.site_xmat[t.site].reshape(3,3).tolist(),
        arm_reference_source=('GRASP_CONFIRM terminal ctrl' if case in ('hold','command_continuous_lift','extra_close_continuous_lift') else 'measured arm trajectory start' if case=='baseline_lift' else 'CLOSE terminal ctrl'),
        gripper_reference_source='CLOSE terminal ctrl, unchanged',
        tcp_vertical_displacement_m=float(t.d.site_xpos[t.site,2]-origin['tcp'][2]),
        block_table_contact_count=len(table),block_table_normal_force_N=sum(c['normal_force_N'] for c in table),
        block_lift_m=float(t.env.metrics()['block_bottom_height_m']-t.env.settle_info['block_bottom_height_m']),
        control_order=['ctrl assignment','mj_step','mj_forward integrated state','geometry/state gates','contact force observation','teacher contact gate'],
        gate_failure=None)
    try:t.inspect_runtime()
    except (ValueError,RuntimeError) as error:
        row['gate_failure']=str(error)
        # Contact loss is the measured outcome of this bounded diagnostic, not a
        # task exemption. Every other safety failure still stops the copied case.
        if str(error)!='bilateral finger contact lost during lift/hold':raise
    print(json.dumps({k:row[k] for k in ('mode','case','step','phase','time_s','finger_force_N','executed_tcp_error_m','block_lift_m','block_table_contact_count','gate_failure')}),flush=True)
    return row


def run(saved, steps=50):
    t=WaypointBlockTeacher(saved['candidate']);t.env.reset(seed=0)
    identity='portable_model_sha256' if 'portable_model_sha256' in saved['provenance'] else 'model_sha256'
    if not same_audited_desk_model(t.report['provenance'][identity],saved['provenance'][identity]):
        raise ValueError('diagnostic model differs from actual failure model')
    t.env.settle_info=saved['settle'];t.gripper_hold_reference=(saved['task_open']['reference_rad'],)*2
    t.env.physics_observer=None;t.closing=np.array([1.,0,0])
    t.waypoint_xyz=np.array(saved['teacher_input']['grasp_xyz'])
    t.execution_mode=MODE
    restore(t.m,t.d,saved['close_preflights'][-1]['final_state'])
    reference=t.d.ctrl.copy();t.phase='GRASP_CONFIRM';t.env.collision_phase='GRASP_CONFIRM'
    origin=dict(position=t.d.xpos[t.block_body].copy(),quaternion=t.d.xquat[t.block_body].copy(),tcp=t.d.site_xpos[t.site].copy())
    result=dict(mode=MODE,live_success=False,provenance=t.report['provenance'],candidate=saved['candidate'],settle=saved['settle'],
                task_open=saved['task_open'],grasp_xyz=t.waypoint_xyz.tolist(),states={'A_close':integration_state(t.m,t.d).tolist()},cases={})
    result['close_terminal_command']=reference.tolist()
    confirmation=[]
    for i in range(50):
        previous=t.d.ctrl.copy();t.env.apply_action(tuple(previous),physics_steps=1)
        confirmation.append(observe(t,previous,reference,origin,i+1,'reconstructed_confirmation'))
    result['confirmation']=confirmation
    result['states']['B_confirm']=integration_state(t.m,t.d).tolist()
    result['confirm_terminal_command']=t.d.ctrl.tolist()
    live_b=[r for r in saved['execution_telemetry'] if r['phase']=='GRASP_CONFIRM'][-1]
    result['confirmation_replay_max_qpos_difference']=float(np.max(np.abs(t.d.qpos-live_b['raw_qpos'])))
    result['grasp_quality']=confirmation[-1]
    result['grasp_quality']['block_center_minus_tcp_m']=(t.d.xpos[t.block_body]-t.d.site_xpos[t.site]).tolist()
    before=integration_state(t.m,t.d)
    target=np.asarray(saved['plans'][-1]['target_q'])
    result['lift_target']=target.tolist()
    result['chronology']={}
    for case in ('hold','baseline_lift','command_continuous_lift'):
        clone=copy.copy(t);clone.env=copy.copy(t.env);clone.d=copy.copy(t.d);clone.env.data=clone.d
        clone.step_telemetry=[];clone.command_preview=None;clone.last_telemetry_print=None
        clone.phase='HOLD' if case=='hold' else 'LIFT_5MM';clone.env.collision_phase=clone.phase
        clone.waypoint_xyz=t.waypoint_xyz.copy()+(np.zeros(3) if case=='hold' else [0,0,.005])
        current=np.asarray(actuator_targets_from_qpos(clone.m,clone.d.qpos));start=current.copy()
        start[[5,11]]=clone.d.ctrl[[5,11]]
        if case=='command_continuous_lift':start=clone.d.ctrl.copy()
        trajectory=plan_septic_joint_trajectory(clone.m,start,target,minimum_duration_s=.5)
        rows=[];result['cases'][case]=rows
        for i in range(steps):
            previous=clone.d.ctrl.copy()
            command=reference if case=='hold' else trajectory.sample((i+1)*clone.m.opt.timestep)[0]
            clone.env.apply_action(command,physics_steps=1)
            row=observe(clone,previous,reference,origin,i+1,case);rows.append(row)
            derivatives=([np.zeros(clone.m.nu)]*3 if case=='hold' else
                         trajectory.sample((i+1)*clone.m.opt.timestep)[1:])
            row.update(command_velocity_rad_s=derivatives[0].tolist(),
                command_acceleration_rad_s2=derivatives[1].tolist(),
                trajectory_duration_s=trajectory.duration_s)
            if case=='baseline_lift' and i==0:
                result['states']['C_first_lift']=integration_state(clone.m,clone.d).tolist()
                result['first_lift_replay_max_qpos_difference']=float(np.max(np.abs(clone.d.qpos-saved['execution_telemetry'][-1]['raw_qpos'])))
        np.testing.assert_array_equal(before,integration_state(t.m,t.d))
        result['chronology'][case]=contact_chronology(rows)
    return result


def next_close_diagnostic(saved, evidence):
    """One existing CLOSE increment from saved confirmation; stop at first blocker."""
    import math
    from dynamic_preflight import full_state_preflight
    from collision_guard import check_bimanual_path
    t=WaypointBlockTeacher(saved['candidate']);t.env.reset(seed=0)
    identity='portable_model_sha256'
    if not same_audited_desk_model(t.report['provenance'][identity],evidence['provenance'][identity]):
        raise ValueError('diagnostic model differs from saved confirmation')
    t.env.settle_info=saved['settle']
    t.gripper_hold_reference=(saved['task_open']['reference_rad'],)*2
    t.closing=np.array([1.,0,0]);t.waypoint_xyz=np.array(saved['teacher_input']['grasp_xyz'])
    t.env.physics_observer=None;t.execution_mode=MODE
    restore(t.m,t.d,evidence['states']['B_confirm'])
    before=integration_state(t.m,t.d)
    t.close_arm_reference=t.d.ctrl.copy();t.close_start_tcp=t.d.site_xpos[t.site].copy()
    t.close_contact_origin=None;t.close_previous_command=t.d.ctrl.copy()
    t.close_previous_measured=np.asarray(actuator_targets_from_qpos(t.m,t.d.qpos))
    opened=saved['task_open']['reference_rad'];closed=float(t.m.actuator_ctrlrange[5,0])
    schedule=np.linspace(opened,closed,max(2,math.ceil((opened-closed)/.01)+1))[1:]
    index=int(np.argmin(abs(schedule-t.d.ctrl[5])))
    if abs(schedule[index]-t.d.ctrl[5])>1e-12 or index+1>=len(schedule):
        raise ValueError('saved command is not an interior CLOSE schedule sample')
    target=t.d.ctrl.copy();target[5]=schedule[index+1]
    preflight=full_state_preflight(t,[dict(eligible=True,phase='CLOSE',
        xyz=t.waypoint_xyz.tolist(),ik=dict(action_rad=target.tolist()),duration_s=.1)])
    np.testing.assert_array_equal(before,integration_state(t.m,t.d))
    result=dict(mode=MODE,live_success=False,candidate=saved['candidate'],provenance=t.report['provenance'],
        candidate_close_stage=index+2,close_preflight=preflight,cases={},failure=preflight['failure'])
    if not preflight['passed']:return result
    restore(t.m,t.d,preflight['final_state'])
    t.close_arm_reference=None
    origin=dict(position=t.d.xpos[t.block_body].copy(),quaternion=t.d.xquat[t.block_body].copy(),tcp=t.d.site_xpos[t.site].copy())
    reference=t.d.ctrl.copy()
    try:
        t.phase='GRASP_CONFIRM';t.env.collision_phase=t.phase
        rows=[];result['cases']['extra_close_confirm']=rows
        for i in range(50):
            previous=t.d.ctrl.copy();t.env.apply_action(tuple(reference),physics_steps=1)
            row=observe(t,previous,reference,origin,i+1,'extra_close_confirm');rows.append(row)
            if not all(v>0 for v in row['finger_force_N'].values()):
                raise ValueError('confirmation lost bilateral force')
        t.phase='LIFT_5MM';t.env.collision_phase=t.phase
        target=t.solve(t.waypoint_xyz+np.array([0.,0.,.005]))
        guard=check_bimanual_path(t.m,actuator_targets_from_qpos(t.m,t.d.qpos),target,
            task_phase=t.phase,reference_data=t.d,max_joint_step_rad=.025)
        if not guard.safe:raise ValueError('candidate lift path failed: '+str(guard))
        trajectory=plan_septic_joint_trajectory(t.m,t.d.ctrl.copy(),target,minimum_duration_s=.5)
        rows=[];result['cases']['extra_close_continuous_lift']=rows
        for i in range(math.ceil(trajectory.duration_s/t.m.opt.timestep)):
            previous=t.d.ctrl.copy()
            t.env.apply_action(trajectory.sample((i+1)*t.m.opt.timestep)[0],physics_steps=1)
            row=observe(t,previous,reference,origin,i+1,'extra_close_continuous_lift');rows.append(row)
            if row['gate_failure']:raise ValueError(row['gate_failure'])
        # Reuse the existing endpoint gates; this branch is not live authorization.
        axis=t.d.site_xmat[t.site].reshape(3,3)
        if np.linalg.norm(t.d.site_xpos[t.site]-t.waypoint_xyz)>.0005:
            raise ValueError('candidate lift endpoint position failed')
        if np.arccos(np.clip(axis[:,0]@t.approach,-1,1))>math.radians(2) or np.arccos(np.clip(axis[:,2]@t.closing,-1,1))>math.radians(15):
            raise ValueError('candidate lift endpoint orientation failed')
    except (ValueError,RuntimeError) as error:
        result['failure']=dict(phase=t.phase,reason=str(error),step=i+1)
    result['final_state']=integration_state(t.m,t.d).tolist()
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--report',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--next-close',action='store_true',help='diagnose one existing CLOSE increment, never live')
    args=parser.parse_args()
    if Path(args.output).exists():raise FileExistsError(args.output)
    saved=json.loads(Path(args.report).read_text())
    result=run(saved)
    if args.next_close:result=next_close_diagnostic(saved,result)
    Path(args.output).write_text(json.dumps(result,indent=2))
