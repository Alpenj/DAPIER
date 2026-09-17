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


def observe(t, previous, reference, origin, step, case):
    record_step(t)
    row=t.step_telemetry[-1]
    contact_telemetry(t,row,origin,reference,previous,np.asarray(row['measured_actuator_q']),case)
    for i,c in enumerate(t.d.contact):
        wrench=np.zeros(6);mujoco.mj_contactForce(t.m,t.d,i,wrench)
        row['contacts'][i]['contact_frame_wrench']=wrench.tolist()
        row['contacts'][i]['tangential_force_N']=wrench[1:3].tolist()
        row['contacts'][i]['friction_coefficients']=c.friction.tolist()
        row['contacts'][i]['contact_dimension']=int(c.dim)
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
        arm_reference_source=('GRASP_CONFIRM terminal ctrl' if case in ('hold','command_continuous_lift') else 'measured arm trajectory start' if case=='baseline_lift' else 'CLOSE terminal ctrl'),
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
    if t.report['provenance'][identity]!=saved['provenance'][identity]:
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
            if case=='baseline_lift' and i==0:
                result['states']['C_first_lift']=integration_state(clone.m,clone.d).tolist()
                result['first_lift_replay_max_qpos_difference']=float(np.max(np.abs(clone.d.qpos-saved['execution_telemetry'][-1]['raw_qpos'])))
        np.testing.assert_array_equal(before,integration_state(t.m,t.d))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--report',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if Path(args.output).exists():raise FileExistsError(args.output)
    result=run(json.loads(Path(args.report).read_text()))
    Path(args.output).write_text(json.dumps(result,indent=2))
