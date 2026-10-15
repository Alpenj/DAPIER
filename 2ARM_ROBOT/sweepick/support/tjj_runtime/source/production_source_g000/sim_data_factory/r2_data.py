"""R1 -> R2 derived records by original-control physics replay, never relabel in place."""
import argparse
import gc
import hashlib
import json
import time
from pathlib import Path
import mujoco
import numpy as np
from . import core
from .recorder import EpisodeRecorder, load_episode, _json
from .task_success import ContactModel, evaluate_states
from .sensor import SimulatedCameraSensor
from pgripper import MOTOR_MAX_RAD


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def convert_commands(commands):
    q=np.asarray(commands,dtype=float).copy();q[..., [5,11]]=1-q[..., [5,11]]
    return q


def replay(parent, root, *, render=True, camera_ids=('left_wrist_rgb','right_wrist_rgb','os30a_UNVERIFIED')):
    parent=Path(parent);root=Path(root);started=time.monotonic()
    meta,a,_,_=load_episode(parent)
    if meta['contract_revision']!='sim-data-r1':raise ValueError('expected original r1 parent')
    parent_hash={name:sha(parent/name) for name in ('episode.npz','metadata.json','diagnostics.json')}
    initial=dict(q=convert_commands(a['q_state'][0]).tolist(),names=core.ACTION_NAMES,units=core.UNITS)
    ctx=core.make_episode(meta['seed'],initial,meta['scene_config']);m,d=ctx['model'],ctx['data']
    if ctx['compiled_model_sha256']!=meta['compiled_model_sha256']:raise ValueError('parent model mismatch')
    if (meta['mujoco_version']!=mujoco.__version__ or meta['control_dt']!=ctx['control_dt']
            or meta['physics_steps']!=ctx['physics_steps']):raise ValueError('parent engine/clock mismatch')
    cmds=convert_commands(a['q_cmd']);decoded=np.stack([core.decode_command(q,m) for q in cmds])
    ctrl_error=float(np.max(abs(decoded-a['ctrl_applied'])))
    if ctrl_error>1e-12:raise ValueError('canonical conversion changed physical ctrl')
    out=root/'datasets/episodes'/('r2-derived-'+parent.name)
    recorder=EpisodeRecorder(out,dict(contract_revision=core.REVISION,observation_revision='os30a-mono-v1',seed=meta['seed'],
        classification='physics_replay_unchecked',control_dt=ctx['control_dt'],physics_dt=float(m.opt.timestep),
        physics_steps=ctx['physics_steps'],mujoco_version=mujoco.__version__,
        compiled_model_sha256=ctx['compiled_model_sha256'],scene_config=ctx['scene_config'],
        requested_initial_state=initial,parent_episode=str(parent.resolve()),parent_sha256=parent_hash,
        parent_contract_revision='sim-data-r1',parent_reset=meta.get('reset'),
        parent_physics_group=f"seed-{meta['seed']}",derivation='original ctrl mj_step replay; sensor on precommand state copy',
        contract_sha256=sha(root/'integration_contract.md'),joint_names=core.ACTION_NAMES,units=core.UNITS,
        physics_backend='mujoco_mj_step',task_scope='left_pick_right_handover_same_block',
        hardware_access=False,hardware_qualification='NOT_EVALUATED',object_attachment=False,
        external_forces=False,runtime_qpos_writes=0,student_allowlist=['q_state','cameras.rgb']))
    cm=ContactModel(m);spec=mujoco.mjtState.mjSTATE_INTEGRATION
    mujoco.mj_setState(m,d,a['integration_state'][0],spec)
    render_data=mujoco.MjData(m);sensor=SimulatedCameraSensor(width=160,height=120,camera_ids=camera_ids,depth_camera_ids=tuple(c for c in camera_ids if c=='os30a_UNVERIFIED')) if render else None
    max_error={key:0. for key in ('qpos','sim_qvel','integration_state','sim_time')}
    fault=None;checked_substeps=0
    try:
        for i,ctrl in enumerate(a['ctrl_applied']):
            obs=core.snapshot(ctx)
            # Rendering/forward happens on a copy; original solver warmstart is untouched.
            mujoco.mj_setState(m,render_data,obs['integration_state'],spec);mujoco.mj_forward(m,render_data)
            cameras=sensor.observe(m,render_data,ctx) if sensor else {}
            d.ctrl[:]=ctrl
            for substep in range(ctx['physics_steps']):
                mujoco.mj_step(m,d);checked_substeps+=1
                fault=cm.substep_failure(d)
                if fault:break
            if fault:
                fault.update(control_index=i,substep=substep);break
            mujoco.mj_forward(m,d);nxt=core.snapshot(ctx)
            for key in max_error:
                err=float(np.max(abs(nxt[key]-a[key][i+1])));max_error[key]=max(max_error[key],err)
                if err>1e-12:raise ValueError(f'replay mismatch {key} step {i}: {err}')
            recorder.append(obs,cmds[i],nxt,{'source_control_index':i},ctrl,cameras)
            if i%400==0:print(json.dumps({'parent':parent.name,'step':i,'total':len(cmds)}),flush=True)
        if fault:
            report=dict(parent_episode=str(parent),parent_sha256=parent_hash,valid=False,
                failure=fault,checked_substeps=checked_substeps,hardware_access=False)
            (out/'failure.json').write_text(json.dumps(report,indent=2))
            return out,report
        states=np.stack([s['integration_state'] for s in recorder.states]);qs=np.stack([s['q_state'] for s in recorder.states])
        check=evaluate_states(m,states,qs,control_dt=ctx['control_dt']);rows=check.pop('rows')
        final=dict(task_success=check['handover_success'],task_check=check,substep_checks=True,
            substeps_checked=checked_substeps,replay_max_error=max_error,ctrl_equivalence_max_error=ctrl_error,
            classification='physics_demonstration' if check['handover_success'] else 'physics_failed_attempt',
            actual_initial_state=recorder.states[0],source_sha256=core.source_identity(),
            source_identity_skips=dict(core.SOURCE_IDENTITY_SKIPS),conversion_source_sha256=sha(__file__),
            elapsed_s=time.monotonic()-started)
        recorder.finish(final);(out/'task_check_rows.json').write_text(json.dumps(rows))
        loaded,b,c,_=load_episode(out)
        assert len(b['q_state'])==len(cmds)+1 and len(b['q_cmd'])==len(cmds)
        assert set(c)==(set(camera_ids) if render else set())
        if parent_hash!={name:sha(parent/name) for name in parent_hash}:raise ValueError('parent modified')
        return out,dict(episode=str(out),valid=check['valid'],handover_success=check['handover_success'],
            parent_physics_group=f"seed-{meta['seed']}",transitions=len(cmds),camera_ids=list(c),
            replay_max_error=max_error,ctrl_equivalence_max_error=ctrl_error,elapsed_s=final['elapsed_s'])
    finally:
        if sensor:sensor.close()
        del recorder
        gc.collect()


def admit_index(paths, splits, output):
    """Index admission is separate from raw recorder completion (training_eligible=False)."""
    rows=[];groups=set()
    for path,split in zip(paths,splits,strict=True):
        meta,a,c,_=load_episode(path)
        if (meta['contract_revision']!=core.REVISION or not meta.get('task_success')
            or not meta['task_check']['valid'] or not meta.get('substep_checks')
            or meta['classification']!='physics_demonstration'
            or meta.get('terminal_failure') or meta.get('object_attachment') or meta.get('external_forces')
            or meta.get('hardware_access') or not {'left_wrist_rgb','right_wrist_rgb'}<=set(c)):
            raise ValueError('episode not eligible for r2 successful sensor dataset: '+str(path))
        group=meta.get('parent_physics_group',f"seed-{meta['seed']}")
        if group in groups:raise ValueError('duplicate physics parent/seed group')
        groups.add(group)
        rows.append(dict(path=str(Path(path).resolve()),parent_physics_group=group,split=split,
            transitions=len(a['q_cmd']),training_eligible=True,
            file_sha256={str(p.relative_to(path)):sha(p) for p in [Path(path)/'metadata.json',Path(path)/'episode.npz',*sorted((Path(path)/'cameras').glob('*.npz'))]}))
    result=dict(contract_revision=core.REVISION,control_dt=.02,episodes=rows,
        student_input=['current q_state','left_wrist_rgb','right_wrist_rgb'],hardware_access=False,
        admission='independent task checker + all physics substeps + replay equivalence where derived')
    with Path(output).open('x') as f:json.dump(result,f,indent=2)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--parent',type=Path,required=True);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--no-render',action='store_true');args=p.parse_args()
    path,result=replay(args.parent,args.root,render=not args.no_render);print(json.dumps(result,indent=2))


def validate_index(root):
    index=json.loads((Path(root)/'manifest.json').read_text());groups=set();counts={}
    if index['contract_revision']!=core.REVISION or index['control_dt']!=.02:raise ValueError('r2 clock/revision mismatch')
    for row in index['episodes']:
        p=Path(row['path']);meta=json.loads((p/'metadata.json').read_text())
        if (not row['training_eligible'] or meta['contract_revision']!=core.REVISION
            or not meta['task_success'] or not meta['task_check']['valid'] or not meta['substep_checks']):
            raise ValueError('unvalidated demonstration')
        if row['parent_physics_group'] in groups:raise ValueError('parent group split leakage')
        groups.add(row['parent_physics_group'])
        for name,digest in row['file_sha256'].items():
            if sha(p/name)!=digest:raise ValueError('dataset file identity mismatch: '+str(p/name))
        counts[row['split']]=counts.get(row['split'],0)+1
    if any(not counts.get(s) for s in ('train','val','test')):raise ValueError('empty parent-group split')
    return dict(parent_groups=len(groups),split_counts=counts,contract_revision=core.REVISION,control_dt=.02)


class R2Demonstrations:
    """NPZ camera adapter for the existing pgripper_learning ACT train/validate loop."""
    def __init__(self,root,ids,stats=None):
        index=json.loads((Path(root)/'manifest.json').read_text());self.episodes={}
        for i in ids:
            meta,a,c,_=load_episode(index['episodes'][i]['path'])
            if meta['contract_revision']!=core.REVISION or not index['episodes'][i]['training_eligible']:
                raise ValueError('r2 admitted data required')
            self.episodes[i]=dict(state=a['q_state'][:-1].astype(np.float32),action=a['q_cmd'].astype(np.float32),
                rgb={s:c[f'{s}_wrist_rgb']['rgb'] for s in ('left','right')})
        self.indices=[(i,t) for i,e in self.episodes.items() for t in range(len(e['action']))]
        self.stats=stats or {k:dict(mean=np.concatenate([e[k] for e in self.episodes.values()]).mean(0),
            std=np.maximum(np.concatenate([e[k] for e in self.episodes.values()]).std(0),.01)) for k in ('state','action')}
        shapes={tuple(e['rgb'][s].shape[1:]) for e in self.episodes.values() for s in ('left','right')}
        if len(shapes)!=1:raise ValueError('mixed image resolution')
        h,w,c=shapes.pop();self.image_shape=(c,h,w)

    def __len__(self):return len(self.indices)

    def __getitem__(self,index):
        import torch
        from pgripper_learning import CHUNK,RGB_KEYS
        i,t=self.indices[index];ep=self.episodes[i];steps=np.arange(t,t+CHUNK)
        action=ep['action'][np.minimum(steps,len(ep['action'])-1)]
        batch=policy_observation(ep['state'][t],{side:ep['rgb'][side][t] for side in ('left','right')},self.stats)
        batch.update(action=torch.tensor((action-self.stats['action']['mean'])/self.stats['action']['std']),
                     action_is_pad=torch.tensor(steps>=len(ep['action'])))
        return batch


def policy_observation(q, wrist_rgb, stats, *, device=None):
    """Same serving/training transform; q is measured canonical state, never a delta."""
    import torch
    from pgripper_learning import RGB_KEYS
    q=np.asarray(q,dtype=np.float32)
    if q.shape!=(12,) or not np.isfinite(q).all():raise ValueError('finite canonical12-state required')
    batch={'observation.state':torch.tensor((q-stats['state']['mean'])/stats['state']['std'],device=device)}
    for side,key in zip(('left','right'),RGB_KEYS):
        image=np.asarray(wrist_rgb[side])
        if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:raise ValueError('raw RGB uint8 required')
        rgb=torch.tensor(image,device=device).permute(2,0,1).float()/255
        batch[key]=(rgb-torch.tensor([.485,.456,.406],device=device)[:,None,None])/torch.tensor([.229,.224,.225],device=device)[:,None,None]
    return batch
