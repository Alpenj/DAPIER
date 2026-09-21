#!/usr/bin/env python3
"""One continuous copied CLOSE-to-HOLD path; fixed NoSlip0/impratio100."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import types
from unittest.mock import patch
import mujoco
import numpy as np
from airborne_hold_audit import force_sample
from close_contact_diagnostic import restore
from controlled_contact_audit import MODE, array_hashes, options
from dynamic_preflight import integration_state
from lift_refinement_audit import plan_report
from pgripper import jaw_gap_m
from waypoint_block_teacher import WaypointBlockTeacher


def outcome(close_pass, lift_pass, hold_pass):
    if not close_pass:return 'B'
    if not lift_pass:return 'INCOMPLETE_LIFT'
    return 'A' if hold_pass else 'C'


def frozen_commands(source, squeeze, matched, refined, opened, closed):
    schedule=np.linspace(opened,closed,max(2,math.ceil((opened-closed)/.01)+1))[1:48+1]
    assert len(schedule)==48
    target=float(squeeze['matched_target_rad'])
    assert closed<=target<schedule[-1]
    first=next(p for p in matched['plans'] if p['phase']=='LIFT_5MM' and 'ik' in p)
    fifteen=next(p for p in matched['plans'] if p['phase']=='LIFT_15MM' and 'ik' in p)
    thirty=next(p for p in refined['plans'] if p['phase']=='LIFT_30MM' and 'ik' in p)
    # Keep the verified squeeze and endpoint references; this tests contact settings,
    # not a new IK solution or a phase-specific target compensation.
    lifts=[dict(phase='LIFT_5MM',xyz=first['xyz'],q=first['ik']['action_rad']),
           dict(phase='LIFT_15MM',xyz=fifteen['xyz'],q=refined['plan']['q']),
           dict(phase='LIFT_30MM',xyz=thirty['xyz'],q=thirty['ik']['action_rad'])]
    assert all(p['q'][5]==target for p in lifts)
    return schedule.tolist()+[target],lifts


def run(a):
    a.output.mkdir(parents=True,exist_ok=False)
    source=json.loads(a.source.read_text());donor=json.loads(a.donor.read_text())
    squeeze=json.loads(a.squeeze.read_text())
    matched=json.loads(a.matched.read_text())['cases']['matched_budget']
    refined=json.loads(a.refined.read_text())['cases']['refined']
    t=WaypointBlockTeacher(donor['candidate']);t.env.physics_observer=None;t.env.reset(seed=0)
    m=mujoco.MjModel.from_binary_path(str(a.model));original_options=options(m);arrays=array_hashes(m)
    assert m.opt.noslip_iterations==0 and m.opt.impratio==1
    m.opt.impratio=100;fixed_options=options(m);d=mujoco.MjData(m)
    state=np.asarray(source['approach_terminal_state'])
    t.m=m;t.d=d;t.env.model=m;t.env.data=d;restore(m,d,state)
    np.testing.assert_array_equal(integration_state(m,d),state)
    t.site=m.site('left_cube_grasp').id;t.env.settle_info=donor['settle'];t.env._block_hold_s=0.
    T=np.asarray(source['candidate_kinematic']['target_TCP'])
    t.approach=T[:3,0];t.closing=T[:3,2];t.waypoint_xyz=T[:3,3]
    reference=d.ctrl.copy();t.close_arm_reference=reference.copy()
    t.gripper_hold_reference=(reference[5],reference[11]);t.close_start_tcp=d.site_xpos[t.site].copy()
    t.close_contact_origin=None;t.execution_mode=MODE;t.transition('CLOSE')
    closes,lifts=frozen_commands(source,squeeze,matched,refined,reference[5],m.actuator_ctrlrange[5,0])
    result=dict(mode=MODE,live_task_success=False,hardware_execution=False,runtime_adopted=False,
        mujoco_version=mujoco.__version__,initial_state=state.tolist(),state_spec='mjSTATE_INTEGRATION',
        initial_state_sha256=hashlib.sha256(state.tobytes()).hexdigest(),
        source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                      (a.source,a.donor,a.squeeze,a.matched,a.refined,a.model,Path(__file__))},
        original_options=original_options,fixed_options=fixed_options,model_arrays=arrays,
        close_commands_rad=closes,lift_commands=lifts,stages=[],phase_summaries=[],failure=None)
    joint=int(m.body_jntadr[t.block_body]);dof=int(m.jnt_dofadr[joint])
    qa=int(m.jnt_qposadr[m.actuator_trnid[5,0]]);va=int(m.jnt_dofadr[m.actuator_trnid[5,0]])
    rows=[];phase_rows=[];stage=0;step_count=0;applied=None
    start=float(d.time);last=(d.qpos.copy(),d.qvel.copy(),d.act.copy(),float(d.time))
    original_step=mujoco.mj_step
    def step(model,data):
        nonlocal step_count,last,applied
        assert model is m and data is d
        # A copied path may restore once at initialization, never jump at a phase boundary.
        for current,previous in zip((d.qpos,d.qvel,d.act,d.time),last):
            np.testing.assert_array_equal(current,previous)
        assert m.opt.noslip_iterations==0 and m.opt.impratio==100
        original_step(model,data);step_count+=1
        last=(d.qpos.copy(),d.qvel.copy(),d.act.copy(),float(d.time))
        applied=force_sample(t)
    def record(self):
        assert len(rows)<step_count
        metrics=t.env.metrics();gate=force_sample(t);limits=m.actuator_forcerange
        row=dict(step=step_count,phase=t.phase,stage=stage,time_s=float(d.time),elapsed_s=float(d.time-start),
            ctrl=d.ctrl.tolist(),raw_qpos=d.qpos.tolist(),raw_qvel=d.qvel.tolist(),
            tcp_xyz=d.site_xpos[t.site].tolist(),target_xyz=t.waypoint_xyz.tolist(),
            tcp_error_m=float(np.linalg.norm(d.site_xpos[t.site]-t.waypoint_xyz)),
            block_z_m=float(d.xipos[t.block_body,2]),block_vz_m_s=float(d.qvel[dof+2]),
            bottom_m=float(metrics['block_bottom_height_m']),block_quat=d.xquat[t.block_body].tolist(),
            jaw_q_rad=float(d.qpos[qa]),jaw_qvel_rad_s=float(d.qvel[va]),jaw_opening_m=jaw_gap_m(m,d,'left'),
            applied=applied,gate=gate,continuous_hold_s=float(metrics['continuous_hold_s']),
            lift_supported=bool(metrics['lift_supported']),success=bool(metrics['success']),
            penetration_m=max([0.]+[-float(c.dist) for c in d.contact]),warnings=d.warning.number.tolist(),
            actuator_force_Nm=d.actuator_force.tolist(),
            saturation=(m.actuator_forcelimited & ((d.actuator_force<=limits[:,0])|(d.actuator_force>=limits[:,1]))).astype(bool).tolist())
        rows.append(row);phase_rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n')
        if step_count%100==0:
            print(MODE,t.phase,'stage',stage,'step',step_count,'Fn',gate['forces_N'],
                  'table',gate['table_count'],gate['table_force_N'],'bottom mm',row['bottom_m']*1000,
                  'vz mm/s',row['block_vz_m_s']*1000,'TCP mm',row['tcp_error_m']*1000,flush=True)
    t.record_step=types.MethodType(record,t)
    def summarize_phase():
        if not phase_rows:return
        r=phase_rows[-1]
        result['phase_summaries'].append(dict(phase=r['phase'],steps=len(phase_rows),
            time_s=r['time_s'],finger_forces_N=r['gate']['forces_N'],
            table_count=r['gate']['table_count'],table_force_N=r['gate']['table_force_N'],
            bottom_m=r['bottom_m'],vz_m_s=r['block_vz_m_s'],tcp_error_m=r['tcp_error_m']))
        phase_rows.clear()
    close_pass=lift_pass=hold_pass=False
    print(MODE,'FIXED NoSlip0 / impratio100 START',start,'48 CLOSE + matched squeeze; no saved-state jumps',flush=True)
    with (a.output/'path.jsonl').open('w') as stream,patch.object(mujoco,'mj_step',step):
        try:
            for stage,opening in enumerate(closes,1):
                target=reference.copy();target[5]=opening
                t.move(target,duration=.1)
                result['stages'].append(dict(stage=stage,time_s=float(d.time),ctrl=d.ctrl.tolist(),
                    forces_N=rows[-1]['gate']['forces_N']))
            if min(rows[-1]['gate']['forces_N'])<=0:raise ValueError('CLOSE did not form bilateral positive contact')
            summarize_phase();t.close_arm_reference=None;t.transition('GRASP_CONFIRM')
            command=tuple(d.ctrl)
            for _ in range(50):
                t.env.apply_action(command,physics_steps=1);t.record_step();checked=t.inspect_runtime()
                if not all(v>0 for v in checked['finger_force_N'].values()):
                    raise ValueError('bilateral contact lost during confirmation')
            close_pass=True;summarize_phase()
            for plan in lifts:
                t.transition(plan['phase']);t.waypoint_xyz=np.asarray(plan['xyz'])
                result.setdefault('endpoint_plans',[]).append(dict(phase=t.phase,**plan_report(t,plan['q'])))
                t.move(plan['q'],duration=.5);summarize_phase()
            lift_pass=bool(rows[-1]['lift_supported'] and rows[-1]['gate']['table_count']==0
                           and rows[-1]['gate']['table_force_N']==0)
            if not lift_pass:raise ValueError('LIFT30 not bilateral/table-free above unchanged 30 mm gate')
            t.transition('HOLD');command=tuple(d.ctrl);hold_start=len(rows)
            for _ in range(round(3./m.opt.timestep)):
                t.env.apply_action(command,physics_steps=1);t.record_step();t.inspect_runtime()
                np.testing.assert_array_equal(d.ctrl,command)
            hold_rows=rows[hold_start:]
            hold_pass=bool(len(hold_rows)==1500 and all(r['lift_supported'] and
                r['gate']['table_count']==0 and r['gate']['table_force_N']==0 for r in hold_rows)
                and hold_rows[-1]['success'])
            if not hold_pass:raise ValueError('continuous 3-second supported HOLD not reached')
        except (ValueError,RuntimeError) as e:
            result['failure']=dict(phase=t.phase,stage=stage,time_s=float(d.time),reason=str(e))
        finally:
            summarize_phase()
    assert len(rows)==step_count and arrays==array_hashes(m) and fixed_options==options(m)
    result.update(classification=outcome(close_pass,lift_pass,hold_pass),close_confirm_pass=close_pass,
        lift_pass=lift_pass,copied_hold_pass=hold_pass,physics_steps=step_count,
        continuous_state_verified=True,options_constant_every_step=True,model_arrays_unchanged=True,
        final_state=integration_state(m,d).tolist(),final_phase=t.phase,
        waypoint_gates=t.report.get('waypoint_gates',[]),plans=t.report['plans'])
    (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print('DONE',result['classification'],t.phase,result['failure'],'copied HOLD',hold_pass,flush=True)


def replay(directory,model_path):
    import mujoco.viewer
    result=json.loads((directory/'result.json').read_text())
    rows=[json.loads(line) for line in (directory/'path.jsonl').read_text().splitlines()]
    m=mujoco.MjModel.from_binary_path(str(model_path));m.opt.impratio=100
    assert m.opt.noslip_iterations==0
    d=mujoco.MjData(m)
    with mujoco.viewer.launch_passive(m,d,show_left_ui=False,show_right_ui=False) as v:
        v.cam.lookat[:]=[.2,0,.04];v.cam.distance=.38;v.cam.azimuth=115;v.cam.elevation=-15
        start=time.monotonic();duration=rows[-1]['elapsed_s'];index=0
        while v.is_running():
            elapsed=(time.monotonic()-start)%(duration+3.)
            if elapsed<rows[index]['elapsed_s']:index=0
            while index+1<len(rows) and rows[index+1]['elapsed_s']<=elapsed:index+=1
            r=rows[index]
            with v.lock():
                d.qpos[:]=r['raw_qpos'];d.qvel[:]=r['raw_qvel'];d.ctrl[:]=r['ctrl'];d.time=r['time_s']
                mujoco.mj_forward(m,d)
            lines=[result.get("mode",MODE),'RECORDED ctrl + mj_step / DISPLAY ONLY','NoSlip=0 / impratio=100 throughout',
                f"{r['phase']} stage {r['stage']} | SIM {r['time_s']:.3f} s",
                f"Finger normal N: {np.round(r['gate']['forces_N'],4)}",
                f"Table contacts {r['gate']['table_count']} / N {r['gate']['table_force_N']:.5f}",
                f"Bottom {r['bottom_m']*1000:.3f} mm | vz {r['block_vz_m_s']*1000:.4f} mm/s",
                f"TCP error {r['tcp_error_m']*1000:.4f} mm | HOLD {r['continuous_hold_s']:.3f} s",
                f"Result {result['classification']} | "+str(result['failure'])]
            v.set_texts([(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,'\n'.join(lines),'')])
            v.sync();time.sleep(.025)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--replay',type=Path);p.add_argument('--model',type=Path,required=True)
    for name in ('source','donor','squeeze','matched','refined','output'):p.add_argument('--'+name,type=Path)
    a=p.parse_args()
    if a.replay:replay(a.replay,a.model)
    elif all(getattr(a,k) for k in ('source','donor','squeeze','matched','refined','output')):run(a)
    else:p.error('run requires source/donor/squeeze/matched/refined/output')
