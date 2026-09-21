#!/usr/bin/env python3
"""Opt-in HOME-to-HOLD SIM candidate; no checkpoint restore or solver switching."""
import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from unittest.mock import patch
import mujoco
import numpy as np
from airborne_hold_audit import force_sample
from controlled_contact_audit import array_hashes, options
from global_impratio_audit import replay
from lift_refinement_audit import plan_report
from pgripper import jaw_gap_m
from waypoint_block_teacher import WaypointBlockTeacher
from dynamic_preflight import full_state_preflight
from collision_guard import task_clearance_status

MODE='SIM PHYSICS / LIVE TASK STATE / RUNTIME CANDIDATE'


class RuntimeCandidateTeacher(WaypointBlockTeacher):
    def __init__(self, candidate, staging, config, *, connection_only=False):
        candidate=dict(axis=np.asarray(config['target_TCP'])[:3,0].tolist(),
            label=config['label'],best=dict(pregrasp=dict(q=config['pregrasp_q'])))
        super().__init__(candidate,staging_reference=staging)
        self.config=config
        self.connection_only=connection_only
        assert config['noslip_iterations']==0 and config['impratio']==100
        self.m.opt.noslip_iterations=0;self.m.opt.impratio=100
        # A2's successful copied formation used a different target from the tilted
        # HOME teacher. Carry its command geometry explicitly, never its saved state.
        self.approach=np.asarray(config['target_TCP'])[:3,0]
        self.closing=np.asarray(config['target_TCP'])[:3,2]
        self.execution_mode=MODE
        self.report['runtime_candidate']=config
        self.report['live_task_success']=False
        self.env.physics_observer=None
        a,b=(self.m.site(name).id for name in ('left_cube_grasp','left_gripperframe'))
        if self.m.site_bodyid[a]!=self.m.site_bodyid[b]:
            raise ValueError('A2 and teacher TCP parents differ')
        np.testing.assert_array_equal(self.m.site_pos[a],self.m.site_pos[b])
        np.testing.assert_array_equal(self.m.site_quat[a],self.m.site_quat[b])

    def record_step(self):
        if getattr(self,'live_data',None) is self.d:
            self.live_recorder(self)
        else:
            super().record_step()  # Existing copied preflight keeps its own telemetry.

    def configure_task_open(self):
        self.gripper_hold_reference=tuple(self.config['task_open_rad'])
        self.report['task_open']=dict(reference_rad=self.gripper_hold_reference,
            source='unchanged A2 mapped grasp task-open reference')
        for a,q in zip((5,11),self.gripper_hold_reference):
            if not self.m.actuator_ctrlrange[a,0]<=q<=self.m.actuator_ctrlrange[a,1]:
                raise ValueError('existing task-open command outside ctrlrange')

    def staging_plan(self,pregrasp,grasp,*,require_dynamic=False):
        self.approach=np.asarray(self.config['target_TCP'])[:3,0]
        self.closing=np.asarray(self.config['target_TCP'])[:3,2]
        grasp=np.asarray(self.config['target_TCP'])[:3,3]
        pregrasp=np.asarray(self.config['pregrasp_xyz_m'])
        self.report['teacher_input'].update(grasp_xyz=grasp.tolist(),pregrasp_xyz=pregrasp.tolist(),
            approach=self.approach.tolist(),closing=self.closing.tolist(),
            target_source='A2 mapped command geometry; no checkpoint state')
        return super().staging_plan(pregrasp,grasp,require_dynamic=require_dynamic)

    def finish_task(self,grasp):
        self.report['connection_pass']=True
        if self.connection_only:
            self.transition('A2_CONNECTION_READY')
            return  # Explicit bounded run stops before CLOSE; never task success.
        grasp=np.asarray(self.config['target_TCP'])[:3,3];self.waypoint_xyz=grasp
        self.close_arm_reference=self.d.ctrl.copy()
        self.close_start_tcp=self.d.site_xpos[self.site].copy();self.close_contact_origin=None
        self.transition('CLOSE')
        for stage,opening in enumerate(self.config['close_commands_rad'],1):
            self.stage=stage;target=self.close_arm_reference.copy();target[5]=opening
            segment=dict(eligible=True,phase='CLOSE',xyz=grasp.tolist(),
                         ik=dict(action_rad=target.tolist()),duration_s=.1)
            preflight=full_state_preflight(self,[segment])
            self.report.setdefault('close_preflights',[]).append(preflight)
            if not preflight['passed']:raise ValueError('CLOSE preflight failed: '+str(preflight['failure']))
            self.move(target,duration=.1)
        self.close_arm_reference=None;self.transition('GRASP_CONFIRM')
        for _ in range(50):
            self.env.apply_action(tuple(self.d.ctrl),physics_steps=1);self.record_step()
            if not all(v>0 for v in self.inspect_runtime()['finger_force_N'].values()):
                raise ValueError('bilateral contact did not persist through confirmation')
        for plan in self.config['lift_commands']:
            self.transition(plan['phase']);self.waypoint_xyz=np.asarray(plan['xyz'])
            self.report.setdefault('frozen_lift_plans',[]).append(dict(phase=self.phase,**plan_report(self,plan['q'])))
            self.move(plan['q'],duration=.5)
        self.transition('HOLD');command=tuple(self.d.ctrl);held=[]
        for _ in range(round(3./self.m.opt.timestep)):
            self.env.apply_action(command,physics_steps=1);self.record_step()
            checked=self.inspect_runtime();held.append(bool(checked['metrics']['lift_supported']))
        if not (all(held) and self.env.metrics()['success']):
            raise ValueError('continuous 3-second supported HOLD not reached')
        self.report['success']=True;self.report['live_task_success']=True;self.transition('SUCCESS')


def connection_evidence(report, rows):
    """Compare equal scopes; endpoint equality alone is not trajectory equality."""
    preflight=report.get('staging_search',{}).get('selected',{}).get('dynamic_preflight')
    if not preflight:
        return None
    scope=preflight['scope']
    live=[r for r in rows if r['phase'] in scope]
    copied=preflight['telemetry']
    fields=(('time_s','time_s'),('raw_qpos','raw_qpos'),('raw_qvel','raw_qvel'),('ctrl','target_q'))
    mismatches=[i for i,(a,b) in enumerate(zip(live,copied))
        if a['phase']!=b['phase'] or any(not np.array_equal(a[x],b[y]) for x,y in fields)]
    minimum=lambda values: min(values,default=None)
    equal=bool(live and len(live)==len(copied) and not mismatches)
    endpoint=report.get('preflight_live_comparison',{})
    verified=bool(preflight.get('passed') and equal and all(r['general_policy_safe'] for r in live)
        and endpoint.get('scope')==scope and endpoint.get('bitwise_equal'))
    return dict(verified=verified,scope=scope,comparison_fields=[x for x,y in fields],
        live_samples=len(live),copied_samples=len(copied),
        samplewise_equal=equal,
        first_mismatch_index=mismatches[0] if mismatches else None,
        live_minimum_general_clearance_m=minimum(r['general_clearance_m'] for r in live),
        copied_minimum_general_clearance_m=minimum(r['measured_clearance_m'] for r in copied),
        live_all_policy_safe=bool(live and all(r['general_policy_safe'] for r in live)))


def run(a, *, teacher_class=RuntimeCandidateTeacher):
    a.output.mkdir(parents=True,exist_ok=False)
    config=json.loads(a.config.read_text());staging=json.loads(a.staging.read_text())
    donor=json.loads(a.donor.read_text())
    # Snapshot exact disk sources and inputs before execution; never reuse output directories.
    root=Path(__file__).resolve().parent
    sources=sorted(set(root.glob('*.py')) | set(root.glob('*.json')) | set((root/'config').glob('*.json')))
    source_hashes={}
    for path in sources:
        raw=path.read_bytes();relative=path.relative_to(root)
        destination=a.output/'source'/relative;destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes(raw);source_hashes[str(relative)]=hashlib.sha256(raw).hexdigest()
    for name,path in (('config',a.config),('staging',a.staging),('donor',a.donor)):
        (a.output/(name+'-input.json')).write_bytes(path.read_bytes())
    provenance=dict(python=sys.version,executable=sys.executable,mujoco=mujoco.__version__,
        numpy=np.__version__,entrypoint=str(Path(sys.argv[0]).resolve()),argv=sys.argv,
        teacher_class=teacher_class.__name__,source_sha256=source_hashes,
        collision_guard_file=sys.modules['collision_guard'].__file__,
        mj_geomDistance_doc=mujoco.mj_geomDistance.__doc__,
        asset_path=os.environ.get('DAPIER_SO101_MJCF'))
    for name,args in (('head',['rev-parse','HEAD']),('status',['status','--short']),('diff',['diff','HEAD'])):
        provenance[name]=subprocess.check_output(['git',*args],cwd=root,text=True)
    (a.output/'run-provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    t=teacher_class(donor['candidate'],staging,config,connection_only=a.connection_only)
    m,d=t.m,t.d;fixed=options(m);arrays=array_hashes(m)
    mujoco.mj_saveModel(m,str(a.output/'runtime-model.mjb'),None)
    t.stage=0;rows=[];counts=dict(live=0,preflight=0);last=None
    original=mujoco.mj_step
    def step(model,data):
        nonlocal last
        if model is m:
            assert model.opt.noslip_iterations==0 and model.opt.impratio==100
        if data is d:
            if last is not None:
                for current,previous in zip((d.qpos,d.qvel,d.act,d.time),last):
                    np.testing.assert_array_equal(current,previous)
            original(model,data);counts['live']+=1
            last=(d.qpos.copy(),d.qvel.copy(),d.act.copy(),float(d.time))
        else:
            original(model,data);counts['preflight']+=1
    def record(self):
        if self.d is not d:return  # Existing preflight observer is separate from live evidence.
        if rows and rows[-1]['time_s']==float(d.time):return
        f=force_sample(self);metrics=self.env.metrics();limits=m.actuator_forcerange
        clearance=task_clearance_status(m,d,self.env.collision_phase)
        joint=int(m.body_jntadr[self.block_body]);dof=int(m.jnt_dofadr[joint])
        row=dict(step=counts['live'],phase=self.phase,stage=self.stage,time_s=float(d.time),elapsed_s=float(d.time),
            raw_qpos=d.qpos.tolist(),raw_qvel=d.qvel.tolist(),ctrl=d.ctrl.tolist(),gate=f,
            general_clearance_m=clearance['general_clearance_m'],
            general_provable_clearance_m=clearance['general_provable_clearance_m'],
            general_evidence_status=clearance['general_evidence_status'],
            general_policy_safe=clearance['safe'],closest_general_pair=clearance['closest_general_pair'],
            tcp_xyz=d.site_xpos[self.site].tolist(),
            tcp_error_m=0. if self.waypoint_xyz is None else float(np.linalg.norm(d.site_xpos[self.site]-self.waypoint_xyz)),
            target_xyz=None if self.waypoint_xyz is None else self.waypoint_xyz.tolist(),
            bottom_m=float(metrics['block_bottom_height_m']),block_z_m=float(d.xipos[self.block_body,2]),
            block_vz_m_s=float(d.qvel[dof+2]),continuous_hold_s=float(metrics['continuous_hold_s']),
            lift_supported=bool(metrics['lift_supported']),success=bool(metrics['success']),
            attachment_active=bool(metrics['attachment_active']),
            jaw_opening_m=jaw_gap_m(m,d,'left'),penetration_m=max([0.]+[-float(c.dist) for c in d.contact]),
            warnings=d.warning.number.tolist(),actuator_force_Nm=d.actuator_force.tolist(),
            saturation=(m.actuator_forcelimited & ((d.actuator_force<=limits[:,0])|(d.actuator_force>=limits[:,1]))).astype(bool).tolist())
        rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n')
        if counts['live']%100==0:print(MODE,self.phase,'step',counts['live'],'Fn',f['forces_N'],
            'table',f['table_count'],'bottom mm',row['bottom_m']*1000,'TCP mm',row['tcp_error_m']*1000,flush=True)
    t.live_data=d;t.live_recorder=record
    t.env.physics_observer=t.record_step
    t.observer=lambda phase:print(MODE,'PHASE',phase,'time',float(d.time),flush=True)
    t.planning_observer=lambda r:print('KINEMATIC CONNECTION / NOT PHYSICS',r['phase'],
        'eligible',r['eligible'],'position mm',r['position_error_m']*1000,
        'approach deg',np.rad2deg(r['approach_error_rad']),flush=True)
    # Normal env.reset is the only initialization. Planning and preflight have
    # private MjData and cannot replace the live state across phase boundaries.
    with (a.output/'path.jsonl').open('w') as stream,patch.object(mujoco,'mj_step',step):
        report=t.run()
    assert arrays==array_hashes(m) and fixed==options(m)
    if any(hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest for name,digest in source_hashes.items()):
        raise RuntimeError('source changed during execution; run is not certified')
    report['connection_evidence']=connection_evidence(report,rows)
    report['connection_verified']=bool(report['connection_evidence'] and report['connection_evidence']['verified'])
    report['verification_passed']=bool(report['connection_verified'] and not report.get('failure')
        and (report['success'] or (a.connection_only and report.get('connection_pass'))))
    report.update(mode=MODE,hardware_execution=False,runtime_default_changed=False,
        initial_source='normal scene HOME and env.reset, no saved-state input',
        live_physics_steps=counts['live'],copied_preflight_steps=counts['preflight'],
        live_state_continuity_verified=True,configuration_unchanged=True,options=fixed,
        final_state_qpos=d.qpos.tolist(),final_state_qvel=d.qvel.tolist(),
        candidate_source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in (a.config,a.staging,a.donor,Path(__file__))})
    (a.output/'teacher.json').write_text(json.dumps(report,indent=2)+'\n')
    (a.output/'result.json').write_text(json.dumps(dict(mode=MODE,failure=report.get('failure'),
        classification=('FIRST GATE FAILURE' if report.get('failure') else 'CONNECTION VERIFICATION FAILED' if report.get('connection_pass') and not report['connection_verified'] else
            'CENTER SUCCESS' if report['success'] else
            'A2 CONNECTION PASS / TASK NOT RUN' if report.get('connection_pass') else 'FIRST GATE FAILURE'),
        live_task_success=report['success'],connection_pass=report.get('connection_pass',False),
        connection_evidence=report['connection_evidence'],connection_verified=report['connection_verified'],
        verification_passed=report['verification_passed'],physics_steps=counts['live']),indent=2)+'\n')
    print('LIVE FINISHED',report['final_phase'],report.get('failure'),'CENTER SUCCESS',report['success'],flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--replay',type=Path);p.add_argument('--model',type=Path)
    p.add_argument('--connection-only',action='store_true',help='stop after A2 approach, before CLOSE')
    for k in ('config','staging','donor','output'):p.add_argument('--'+k,type=Path)
    a=p.parse_args()
    if a.replay:
        if not a.model:p.error('replay requires --model')
        replay(a.replay,a.model)
    elif all(getattr(a,k) for k in ('config','staging','donor','output')):
        raise SystemExit(0 if run(a)['verification_passed'] else 1)
    else:p.error('runtime requires config/staging/donor/output')
