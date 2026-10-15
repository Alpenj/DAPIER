"""One held-out student-only SIM attempt; GT is reset/checker-only, never policy input."""
import argparse
import json
import time
import copy
from pathlib import Path
import numpy as np
import torch
import mujoco
from dapier_act_policy import ACTPolicy, ACTTemporalEnsembler, BoundedGripperACTConfig, MeasuredRelativeArmACTConfig, MEASURED_Q_KEY, ARM_AXES
from pgripper_learning import RGB_KEYS
from . import core
from .r2_data import validate_index,sha,policy_observation
from .recorder import EpisodeRecorder,_json
from .reset_sampler import make_task_episode
from .sensor import SimulatedCameraSensor
from .task_success import ContactModel,evaluate_states


class Observations(SimulatedCameraSensor):
    def observe(self,m,d,c):
        self.current=super().observe(m,d,c)
        return self.current


ARM_BOUND_POLICIES=('reject','project')


def project_arm_command(command,model):
    """Opt-in SIM execution transform (arm_bound_policy='project'): a finite arm target outside the interval the decoder
    enforces (model.actuator_ctrlrange, compared in float64) becomes exactly that bound. Arm values inside it, both
    grippers, and a non-finite or malformed vector are returned unchanged; the decoder still rejects the last. No margin."""
    raw=np.asarray(command)
    if raw.shape!=(12,) or not np.isfinite(raw).all():return command,None
    arm=list(ARM_AXES);lo,hi=model.actuator_ctrlrange[arm,0],model.actuator_ctrlrange[arm,1]
    q=raw.astype(float)
    mask=(q[arm]<lo)|(q[arm]>hi)
    info=dict(q_raw=q.tolist(),projection_mask=mask.tolist(),q_applied_minus_raw=[0.]*12)
    if not mask.any():return command,info  # inside the interval: the very same command object goes on
    applied=q.copy();applied[arm]=np.minimum(hi,np.maximum(lo,q[arm]))
    info['q_applied_minus_raw']=(applied-q).tolist()
    return applied,info


class Student:
    def __init__(self,policy,stats,sensor,execution_prefix=1,temporal_ensemble_coeff=None,arm_bound_policy='reject'):
        if type(execution_prefix) is not int or not 1 <= execution_prefix <= policy.config.chunk_size:
            raise ValueError('execution prefix outside predicted chunk')
        if policy.config.temporal_ensemble_coeff is not None:
            raise ValueError('prefix experiment requires no temporal ensemble')
        self.policy,self.stats,self.sensor=policy,stats,sensor;self.failure=None;self.calls=0
        self.first_prediction=None;self.trace=[];self.previous=None
        # Absolute actions remain anchored to the generation observation; never rebase on feedback.
        self.execution_prefix=execution_prefix;self.chunk=None;self.chunk_index=0;self.chunks=[]
        # Optional execution-time aggregation of an unchanged checkpoint; None leaves the path above as it was.
        self.ensemble_coeff=temporal_ensemble_coeff;self.ensembler=None;self.step=0
        if temporal_ensemble_coeff is not None:
            if execution_prefix!=1:raise ValueError('temporal ensemble re-infers at every control step: prefix must be 1')
            if type(temporal_ensemble_coeff) is not float or not np.isfinite(temporal_ensemble_coeff):
                raise ValueError('finite float ensemble coefficient required')
            self.ensembler=ACTTemporalEnsembler(temporal_ensemble_coeff,policy.config.chunk_size,dtype=torch.float64)
        # Optional arm target projection onto the decoder interval (SIM candidate); 'reject' leaves the path above as it was.
        if arm_bound_policy not in ARM_BOUND_POLICIES:raise ValueError('arm bound policy must be reject or project')
        self.arm_bound_policy=arm_bound_policy

    def discard_chunk(self):
        self.chunk=None
        if self.ensembler is not None:self.ensembler.reset()  # abort/reset: no prediction is carried anywhere else

    def combine(self):
        """Blend only the predictions that target THIS control step, in canonical units (arm rad,
        gripper opening fraction): this step's chunk[0], the previous chunk[1], ... at most chunk_size,
        and only those that exist. Membership is positional; a value of 0 is an ordinary prediction."""
        n=min(len(self.chunks),self.policy.config.chunk_size)
        if len(self.chunks)!=self.step+1:raise RuntimeError('temporal ensemble needs exactly one chunk per control step')
        blended=self.ensembler.update(torch.from_numpy(np.asarray(self.chunk,dtype=np.float64))[None])[0].numpy().copy()
        # Oldest first, as in the reference: generation step-n+1 contributes its index n-1, ..., this step's chunk index 0.
        members=np.stack([np.asarray(self.chunks[i-n]['actions'][n-1-i],dtype=np.float64) for i in range(n)])
        weights=np.exp(-self.ensemble_coeff*np.arange(n));weights/=weights.sum()
        info=dict(target_step=self.step,count=n,generation_ids=[self.chunks[i-n]['id'] for i in range(n)],
            chunk_indices=list(range(n-1,-1,-1)),weights=weights.tolist())
        if not (np.isfinite(members).all() and np.isfinite(blended).all()):return blended,info  # the decoder rejects it
        mismatch=float(np.abs(weights@members-blended).max())
        if mismatch>1e-9:raise RuntimeError('temporal ensemble is not aligned to the current control step')
        # A convex combination cannot leave the span of its members; this removes float rounding only.
        command=np.minimum(np.maximum(blended,members.min(0)),members.max(0))
        info.update(direct_formula_max_abs_diff=mismatch,span_rounding_adjustment_max_abs=float(np.abs(command-blended).max()))
        return command,info

    def next_command(self,context):
        if self.failure is not None or context.get('terminal_failure') is not None:
            self.discard_chunk()
            raise RuntimeError('student already aborted')
        q=core.encode_state(context['model'],context['data']).astype(np.float32)
        batch={k:v[None] for k,v in policy_observation(q,{side:self.sensor.current[side+'_wrist_rgb']['rgb'] for side in ('left','right')},self.stats,device='cuda').items()}
        if isinstance(self.policy.config,MeasuredRelativeArmACTConfig):
            # Raw measured state of THIS observation: the one anchor of all 50 arm targets of the chunk it generates.
            batch[MEASURED_Q_KEY]=torch.tensor(q,device='cuda')[None]
        elapsed=0.
        if self.chunk is None or self.chunk_index == self.execution_prefix:
            start=time.perf_counter();raw_head=[]
            hook=self.policy.model.action_head.register_forward_hook(lambda module,args,out:raw_head.append(out.detach()))
            try:
                with torch.inference_mode():
                    if isinstance(self.policy.config, BoundedGripperACTConfig):
                        self.chunk=self.policy.predict_canonical_chunk(batch)[0].cpu().numpy().copy()
                    else:
                        normalized=self.policy.predict_action_chunk(batch)[0].cpu().numpy()
                        self.chunk=normalized*self.stats['action']['std']+self.stats['action']['mean']
            finally:
                hook.remove()
            elapsed=time.perf_counter()-start;self.calls+=1;self.chunk_index=0
            self.chunk_time=float(context['data'].time)
            self.raw_gripper_chunk=raw_head[0][0,:,[5,11]].cpu().numpy() if raw_head else None
            self.chunks.append(dict(id=self.calls,time=self.chunk_time,anchor=q.copy(),actions=self.chunk.copy()))
        k=self.chunk_index;command=self.chunk[k].copy();self.chunk_index+=1
        ensemble=None
        if self.ensembler is not None:
            raw_k0=command;command,ensemble=self.combine()
        out=(command-self.stats['action']['mean'])/self.stats['action']['std']  # trace only
        if self.first_prediction is None:self.first_prediction=command.tolist()
        projection=None
        if self.arm_bound_policy=='project':command,projection=project_arm_command(command,context['model'])  # raw kept in the trace
        ctrl=None
        try:ctrl=core.decode_command(command,context['model'])
        except ValueError as e:
            self.failure=dict(reason='student_action_rejected',detail=str(e),command=command.tolist(),time=float(context['data'].time))
            self.discard_chunk()
        self.trace.append(dict(time=float(context['data'].time),q_measured=q.tolist(),
            normalized_state=batch['observation.state'][0].cpu().tolist(),normalized_action=out.tolist(),
            raw_gripper_head=(self.raw_gripper_chunk[k].tolist() if self.raw_gripper_chunk is not None else None),
            raw_gripper_head_semantics=('sigmoid logits' if isinstance(self.policy.config,BoundedGripperACTConfig) else 'normalized linear regression, NOT logits'),
            q_cmd=command.tolist(),ctrl_validated=ctrl.tolist() if ctrl is not None else None,
            command_minus_measured=(command-q).tolist(),command_change=None if self.previous is None else (command-self.previous).tolist(),
            inference_wall_s=elapsed,selected_chunk_index=k,chunk_generation_id=self.calls,
            chunk_generation_time=self.chunk_time,target_time=self.chunk_time+k*context.get('control_dt',.02)))
        if ensemble is not None:
            self.trace[-1].update(execution_aggregation='temporal_ensemble',control_step=self.step,raw_k0_q_cmd=raw_k0.tolist(),
                ensemble_minus_raw_k0=(command-raw_k0).tolist(),ensemble=ensemble)
        if self.arm_bound_policy=='project':  # q_cmd above is the applied (dispatched) command; q_raw is the policy's own output
            self.trace[-1].update(arm_bound_policy='project',**(projection or dict(projection_skipped='non-finite or malformed vector; the decoder decides')))
        self.step+=1
        self.previous=command.copy()
        return command,dict(policy='ACT student only',inference=self.calls,input_keys=list(batch))


def run(root,training,output,*,reset=None,group="development_regression",dataset=None,checkpoint="final",execution_prefix=1,temporal_ensemble_coeff=None,arm_bound_policy='reject'):
    torch.set_num_threads(4);torch.manual_seed(4100)
    root=Path(root);training=Path(training);output=Path(output)
    dataset=Path(dataset) if dataset else root/'datasets/r2-learning'
    index=json.loads((dataset/'manifest.json').read_text());validate_index(dataset)
    if reset is not None:
        q0, block, seed = reset['q0'], reset['block_xy_yaw'], reset['seed']
    else:
        row = next(e for e in index['episodes'] if e['split'] == 'test')
        meta = json.loads((Path(row['path']) / 'metadata.json').read_text())
        q0 = meta['requested_initial_state']['q']
        block = meta.get('parent_reset', meta.get('reset', {}))['block_xy_yaw']
        seed = meta['seed']
    ctx=make_task_episode(seed,q0,block);m=ctx['model'];cm=ContactModel(m)
    ctx['substep_check']=cm.substep_failure
    policy=ACTPolicy.from_pretrained(training/checkpoint,local_files_only=True,strict=True).cuda().eval()
    with np.load(training/'normalization.npz',allow_pickle=False) as a:
        stats={k:{s:a[k+'_'+s].copy() for s in ('mean','std')} for k in ('state','action')}
    policy.check_normalization(stats)
    if policy.config.chunk_size!=50:raise ValueError('unexpected prediction horizon')
    policy.reset()
    sensor=Observations(width=160,height=120,camera_ids=('left_wrist_rgb','right_wrist_rgb'),depth_camera_ids=());actor=Student(policy,stats,sensor,execution_prefix,temporal_ensemble_coeff,arm_bound_policy)
    training_record=json.loads((training/'run.json').read_text())
    training_groups=[index['episodes'][i]['parent_physics_group'] for i in training_record['train_episodes']]
    metadata=dict(diagnostic_train_parent=training_record.get('diagnostic_train_parent'),contract_revision=core.REVISION,observation_revision='os30a-mono-v1',evaluation_group=group,seed=seed,control_dt=ctx['control_dt'],
        classification='student_sim_evaluation',task_success=False,hardware_access=False,
        object_attachment=False,external_forces=False,runtime_qpos_writes=0,
        requested_initial_state=ctx['reset']['requested_q0'],actual_initial_state=ctx['initial'],
        reset=ctx['reset'],compiled_model_sha256=ctx['compiled_model_sha256'],
        checkpoint=str(training/checkpoint),checkpoint_sha256={p.name:sha(p) for p in (training/checkpoint).iterdir() if p.is_file()},
        source_parent_group=f'seed-{seed}',training_groups=training_groups,
        validation_groups=[e['parent_physics_group'] for e in index['episodes'] if e['split']=='val'],
        control_dt_seconds=.02,action_chunk=50,executed_prefix=execution_prefix,reobserve_every_step=True,
        policy_inputs=['current measured canonical q_state','raw left/right wrist RGB'],
        action_filter='finite/command-range rejection only; no teacher correction or clipping')
    ensemble_config=None if temporal_ensemble_coeff is None else dict(coeff=temporal_ensemble_coeff,
        weights='w_i=exp(-coeff*i), i=0 is the oldest prediction for the control step; normalized to sum 1',
        members='chunk[t][0], chunk[t-1][1], ...: existing predictions for the current control step only, at most chunk_size',
        units='canonical after the policy output conversion: arm rad, gripper opening fraction; float64',
        checks='decoder finite/range rejection and substep fault unchanged')
    if ensemble_config is not None:metadata.update(execution_aggregation='temporal_ensemble',temporal_ensemble=ensemble_config)
    projection_config=None if arm_bound_policy=='reject' else dict(arm_bound_policy='project',arm_axes=list(ARM_AXES),
        arm_names=[m.actuator(i).name for i in ARM_AXES],lower=m.actuator_ctrlrange[list(ARM_AXES),0].tolist(),upper=m.actuator_ctrlrange[list(ARM_AXES),1].tolist(),
        bounds_source='model.actuator_ctrlrange of this compiled model: the arrays core.decode_command compares the arm channels with',
        rule='q_applied[j]=min(upper[j],max(lower[j],q_raw[j])) in float64 for the 10 arm axes only; grippers unchanged; no margin',
        unchanged='non-finite / shape / unit / gripper-range rejection, physics, contact and state checks, C25 indices and anchors',
        trace='q_cmd and the episode commands are the applied command; q_raw, projection_mask, q_applied_minus_raw per control step')
    if projection_config is not None:
        metadata.update(arm_bound_policy='project',arm_projection=projection_config,
            action_filter='finite/shape/unit/gripper-range rejection unchanged; finite arm targets outside the decoder interval are projected onto it (opt-in SIM transform); no teacher correction')
    relative_arm=isinstance(policy.config,MeasuredRelativeArmACTConfig)
    if relative_arm:metadata.update(arm_action_representation='measured_relative: arm command = measured state of the generation observation + predicted displacement; the decoder receives the absolute command')
    rec=EpisodeRecorder(output,metadata)
    try:
        geometry_trace=[]
        for _ in range(2000):
            d=ctx['data']
            geometry_trace.append(dict(time=float(d.time),
                tcp_block_distance_m={side:float(np.linalg.norm(d.site_xpos[cm.site[side]]-d.xpos[cm.block_body])) for side in ('left','right')},
                measured_gripper=core.encode_state(m,d)[[5,11]].tolist()))
            before_time=float(ctx['data'].time);before_count=len(rec.commands)
            progressed=core.rollout(ctx,actor,sensor,rec,1)
            if actor.trace:
                actor.trace[-1].update(ctrl_applied=ctx['data'].ctrl.tolist() if ctx['data'].time>before_time else None,
                    complete_transition=len(rec.commands)>before_count,physics_end_time=float(ctx['data'].time))
            if not progressed:
                actor.discard_chunk()
                break
        states=rec.states or [ctx['initial']]
        check=evaluate_states(m,np.stack([s['integration_state'] for s in states]),np.stack([s['q_state'] for s in states]),control_dt=.02)
        scoring_rows=check.pop('rows')
        (output/'scoring_trace.json').write_text(json.dumps(dict(geometry=geometry_trace,contacts=scoring_rows),default=_json))
        fault=ctx.get('terminal_failure')
        check=include_terminal(check,fault)
        diagnostics={side:dict(initial_distance_m=geometry_trace[0]['tcp_block_distance_m'][side],
            minimum_distance_m=min(r['tcp_block_distance_m'][side] for r in geometry_trace),
            final_distance_m=geometry_trace[-1]['tcp_block_distance_m'][side]) for side in ('left','right')}
        report=dict(tcp_block_diagnostic=diagnostics,attempts=1,full_handover=int(check['handover_success']),seed=seed,evaluation_group=group,
            attempted=1,executed=1,stopped=int(fault is not None),completed=int(fault is None),
            executed_scope='policy evaluated after accepted reset; not a task success',
            physics_started=bool(ctx['data'].time>ctx['initial']['sim_time']),
            command_dispatches=sum(t['ctrl_applied'] is not None for t in actor.trace),
            reset_rejections=0,reset_kind=('explicit preregistered sampled q/block' if group=='unseen' else 'explicit '+group+' parent initial q/block'),
            stage=check['stage'],checker=check,inference_calls=actor.calls,completed_transitions=len(rec.commands),
            inference_wall_s=sum(t['inference_wall_s'] for t in actor.trace),actual_sim_time=float(ctx['data'].time),first_predicted_canonical=actor.first_prediction,
            abort=fault,checkpoint_reload='PASS strict=True local_files_only=True',
            inference_shape=[1,12],gripper_unit='opening_fraction',arm_unit='rad',
            teacher_behavior_calls=0,hardware_access=False,source_sha256=core.source_identity(),
            source_identity_skips=dict(core.SOURCE_IDENTITY_SKIPS))
        if ensemble_config is not None:
            combined=[t for t in actor.trace if 'ensemble' in t]
            report['temporal_ensemble']=dict(ensemble_config,control_steps=len(combined),
                max_members=max((t['ensemble']['count'] for t in combined),default=0),
                max_abs_change_from_raw_k0=max((max(abs(x) for x in t['ensemble_minus_raw_k0']) for t in combined),default=0.),
                span_rounding_adjustment_max_abs=max((t['ensemble'].get('span_rounding_adjustment_max_abs',0.) for t in combined),default=0.))
        if projection_config is not None:
            rows=[t for t in actor.trace if 'projection_mask' in t]
            masks=np.array([t['projection_mask'] for t in rows],dtype=bool).reshape(-1,len(ARM_AXES))
            dispatched=[t for t in rows if t.get('ctrl_applied') is not None]
            report['arm_projection']=dict(projection_config,control_steps_seen=len(rows),
                control_steps_projected=int(masks.any(1).sum()),dispatched_steps_projected=sum(any(t['projection_mask']) for t in dispatched),
                projected_steps_per_arm_axis=dict(zip(projection_config['arm_names'],masks.sum(0).astype(int).tolist())),
                max_abs_change=max((max(abs(x) for x in t['q_applied_minus_raw']) for t in rows),default=0.),
                first_projected_time=next((t['time'] for t in rows if any(t['projection_mask'])),None))
        if rec.commands:rec.finish(dict(task_success=bool(check['handover_success']),evaluation=report,terminal_state=ctx.get('terminal_state')))
        np.savez_compressed(output/'predicted_chunks.npz',generation_id=[c['id'] for c in actor.chunks],time=[c['time'] for c in actor.chunks],anchor=np.stack([c['anchor'] for c in actor.chunks]),q_cmd=np.stack([c['actions'] for c in actor.chunks]))
        (output/'execution_config.json').write_text(json.dumps(dict(prefix=execution_prefix,checkpoint_config_unchanged=True,control_dt=.02,physics_dt=.002,temporal_ensemble=ensemble_config,**({'arm_action_representation':'measured_relative'} if relative_arm else {}),**({'arm_projection':projection_config} if projection_config is not None else {})),indent=2))
        (output/'action_trace.json').write_text(json.dumps(actor.trace,indent=2))
        (output/'evaluation.json').write_text(json.dumps(report,default=_json,indent=2))
        print(json.dumps(report,default=_json,indent=2));return report
    finally:sensor.close()


def include_terminal(check,fault):
    check=copy.deepcopy(check)
    check['completed_transition_worst']=dict(check['worst'])
    count=max(check.get('failure_count',0),len(check['failures']))
    if fault:
        check['valid']=False;check['handover_success']=False;check['pick_lift_success']=False
        if fault not in check['failures']:
            check['failures'].append(fault);count+=1
        check['worst']['penetration_m']=max(check['worst']['penetration_m'],fault.get('max_penetration_m',0.))
    check['failure_count']=count
    check['worst_scope']='completed transitions plus terminal fault; abort duplicate is not another event'
    return check


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--training',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--reset-json',type=Path);p.add_argument('--group',default='development_regression');p.add_argument('--dataset',type=Path);p.add_argument('--checkpoint',default='final');p.add_argument('--execution-prefix',type=int,default=1)
    a=p.parse_args();run(a.root,a.training,a.output,reset=json.loads(a.reset_json.read_text()) if a.reset_json else None,group=a.group,dataset=a.dataset,checkpoint=a.checkpoint,execution_prefix=a.execution_prefix)
