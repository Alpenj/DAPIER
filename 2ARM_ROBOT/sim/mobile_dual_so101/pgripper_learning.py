#!/usr/bin/env python3
"""SIM-only parallel demonstrations and DAPIER-owned two-wrist ACT training.

Private JPEG/NPZ staging deliberately stays separate from the real-robot dataset.
This is a fixed-fixture initial-arm-perturbation study, not object generalization.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import multiprocessing
import os
from pathlib import Path
import time

import mujoco
import numpy as np
from PIL import Image

from mobile_dual_so101 import ACTION_NAMES, actuator_targets_from_qpos
from pgripper_handover import run, contract_vector
from pgripper_execution import COMMAND_DT_S
from pgripper_dataset import FPS, UNITS, file_sha256, validate_dataset

RGB_KEYS = ('observation.images.left_wrist', 'observation.images.right_wrist')
CHUNK = 50


def collect_episode(options):
    output, episode, seed, *extra = options
    settings = extra[0] if extra else {}
    command_dt_s = settings.get('command_dt_s', COMMAND_DT_S)
    directory = Path(output) / f'episode-{episode:03d}'
    directory.mkdir(exist_ok=False)
    renderer = None
    rows = {key: [] for key in ('state', 'action', 'action_sent', 'time', 'phase', 'physics', 'block')}
    control_ranges = None
    dense_ctrl, dense_sent_ctrl = [], []
    def observe(model, data, target, sent, phase):
        nonlocal renderer, control_ranges
        control_ranges = model.actuator_ctrlrange.copy()
        dense_ctrl.append(target.copy())
        dense_sent_ctrl.append(sent.copy())
        step = len(dense_ctrl) - 1
        interval = round(1 / FPS / command_dt_s)
        if not np.isclose(interval * command_dt_s, 1 / FPS):
            raise ValueError('physics/control rates must divide exactly')
        if step % interval:
            return
        if renderer is None:
            renderer = mujoco.Renderer(model, height=240, width=320)
        index = len(rows['time'])
        for side in ('left', 'right'):
            renderer.update_scene(data, camera=f'{side}_wrist_rgb')
            Image.fromarray(renderer.render()).save(directory / f'{side}-{index:05d}.jpg', quality=92)
        state = np.empty(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_getState(model, data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
        values = (contract_vector(model, actuator_targets_from_qpos(model, data.qpos)),
                  contract_vector(model, target), contract_vector(model, sent), float(data.time), phase, state,
                  data.body('red_block').xpos.copy())
        for key, value in zip(rows, values):
            rows[key].append(value)
    args = argparse.Namespace(output=directory / 'expert', model=None, donor='left',
        viewer=False, render=False, handover_only=False, disable_recipient_contact=False)
    rng = np.random.default_rng(seed)
    offset = rng.uniform(-np.deg2rad(.5), np.deg2rad(.5), 12)
    offset[[5, 11]] = 0
    code, failure = 1, None
    try:
        code = run(args, command_observer=observe, initial_arm_offset=offset,
            policy_target_hz=settings.get('policy_target_hz'),
            physics_substeps=settings.get('physics_substeps', 1), command_dt_s=command_dt_s,
            initial_state=settings.get('initial_state'))
    except (ValueError, RuntimeError) as error:
        failure = str(error)
    finally:
        if renderer is not None:
            renderer.close()
        np.savez_compressed(directory / 'episode.npz', **{k: np.asarray(v) for k, v in rows.items()},
            dense_ctrl=np.asarray(dense_ctrl), dense_sent_ctrl=np.asarray(dense_sent_ctrl),
            seed=seed, initial_arm_offset=offset, command_dt_s=command_dt_s,
            **({'control_ranges': control_ranges} if control_ranges is not None else {}))
    report_path = directory / 'expert' / 'report.json'
    report = json.loads(report_path.read_text()) if report_path.exists() else {}
    result = {'episode': episode, 'seed': seed, 'frames': len(rows['time']), 'pid': os.getpid(),
        'source_task_success': code == 0, 'canonical_full_task_success': report.get('canonical_full_task_success') is True,
        'task_success': code == 0 and report.get('canonical_full_task_success') is True,
        'failure': failure or report.get('failure'), 'execution': report.get('execution'),
        'episode_sha256': file_sha256(directory / 'episode.npz'),
        'action_source': 'requested_target', 'hold_events': report.get('hold_events', [])}
    (directory / 'collection.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def collect(args):
    if not 4 <= args.episodes <= 64 or not 1 <= args.workers <= 8:
        raise ValueError('expected 4..64 episodes and 1..8 workers')
    args.output.mkdir(parents=True, exist_ok=False)
    split = max(1, args.episodes * 3 // 4)
    plan = {'train': list(range(split)), 'holdout': list(range(split, args.episodes))}
    (args.output / 'split-plan.json').write_text(json.dumps(plan, indent=2) + '\n')
    settings = {key: getattr(args, key, default) for key, default in
        (('policy_target_hz', None), ('physics_substeps', 1), ('command_dt_s', COMMAND_DT_S))}
    options = [(str(args.output), i, args.seed + i, settings) for i in range(args.episodes)]
    with ProcessPoolExecutor(args.workers, mp_context=multiprocessing.get_context('spawn')) as pool:
        episodes = list(pool.map(collect_episode, options))
    finalize(args, episodes)
    print(json.dumps(validate_dataset(args.output)), flush=True)


def finalize(args, episodes=None):
    if (args.output / 'manifest.json').exists():
        raise ValueError('manifest already exists')
    if episodes is None:
        episodes = []
        for path in sorted(args.output.glob('episode-*/expert/report.json')):
            report = json.loads(path.read_text())
            episode = int(path.parents[1].name.split('-')[1])
            archive = path.parents[1] / 'episode.npz'
            if report['task_success'] and not archive.exists():
                raise ValueError('successful episode missing numeric archive')
            with np.load(archive) if archive.exists() else __import__('contextlib').nullcontext(None) as data:
                episodes.append({'episode': episode, 'task_success': report['task_success'] and report.get('canonical_full_task_success') is True,
                    'canonical_full_task_success': report.get('canonical_full_task_success') is True,
                    'episode_sha256': file_sha256(archive) if archive.exists() else None,
                    'frames': len(data['state']) if data is not None else 0, 'pid': None})
    accepted = [e['episode'] for e in episodes
                if e['task_success'] is True and e.get('canonical_full_task_success') is True]
    if len(accepted) < 4:
        raise ValueError('need at least four complete successful demonstrations')
    split = max(1, len(accepted) * 3 // 4)
    plan_path = args.output / 'split-plan.json'
    plan = json.loads(plan_path.read_text()) if plan_path.exists() else {'train': accepted[:split], 'holdout': accepted[split:]}
    if set(plan['train']) & set(plan['holdout']) or not set(accepted) <= set(plan['train'] + plan['holdout']):
        raise ValueError('invalid fixed split plan')
    manifest = {'kind': 'SIM fixed-scene PGripper expert', 'fps': FPS, 'rgb_shape': [240, 320, 3],
        'state_action_units': UNITS, 'action_names': list(ACTION_NAMES),
        'action_source': 'requested simulator ctrl before mj_step, not next-state labels',
        'randomization': 'initial arm positions only +/-0.5 degree; fixed cube and fixture',
        'train': [i for i in plan['train'] if i in accepted], 'holdout': [i for i in plan['holdout'] if i in accepted],
        'episodes': episodes, 'hardware_execution': False}
    if not manifest['train'] or not manifest['holdout']:
        raise ValueError('successful demonstrations must remain in both fixed splits')
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest), flush=True)


class Demonstrations:
    def __init__(self, root, ids, stats=None):
        self.root = Path(root)
        self.episodes = {i: dict(np.load(self.root / f'episode-{i:03d}' / 'episode.npz')) for i in ids}
        self.indices = [(i, t) for i, ep in self.episodes.items() for t in range(len(ep['state']))]
        self.stats = stats or {key: {'mean': np.concatenate([e[key] for e in self.episodes.values()]).mean(0),
            'std': np.maximum(np.concatenate([e[key] for e in self.episodes.values()]).std(0), .01)}
            for key in ('state', 'action')}

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, index):
        import torch
        episode, t = self.indices[index]
        ep = self.episodes[episode]
        steps = np.arange(t, t + CHUNK)
        action = ep['action'][np.minimum(steps, len(ep['action']) - 1)]
        batch = {'observation.state': torch.tensor((ep['state'][t] - self.stats['state']['mean']) / self.stats['state']['std']),
            'action': torch.tensor((action - self.stats['action']['mean']) / self.stats['action']['std']),
            'action_is_pad': torch.tensor(steps >= len(ep['action']))}
        if 'phase' in ep:
            indices = np.minimum(steps, len(ep['action']) - 1)
            # Evaluation metadata only: the last old-phase and first new-phase frames.
            phase = ep['phase']
            batch['action_near_phase_transition'] = torch.tensor(
                (phase[indices] != phase[np.maximum(indices - 1, 0)])
                | (phase[indices] != phase[np.minimum(indices + 1, len(phase) - 1)]))
        for side, key in zip(('left', 'right'), RGB_KEYS):
            with Image.open(self.root / f'episode-{episode:03d}' / f'{side}-{t:05d}.jpg') as im:
                rgb = torch.from_numpy(np.asarray(im).copy()).permute(2, 0, 1).float() / 255
            batch[key] = (rgb - torch.tensor([.485, .456, .406])[:, None, None]) / torch.tensor([.229, .224, .225])[:, None, None]
        return batch


def validate(policy, loader, *, device, execution_prefix=1):
    """Aggregate normalized errors over valid scalar elements; never feed labels to ACT."""
    import torch
    if type(execution_prefix) is not int or execution_prefix < 1:
        raise ValueError('execution_prefix must be a positive integer')
    totals = {key: [0.0, 0] for key in ('full_chunk', 'first_action', 'execution_prefix',
                                      'gripper', 'phase_transition')}
    modes = [(module, module.training) for module in policy.modules()]
    policy.eval()
    try:
        with torch.inference_mode():
            for batch in loader:
                batch = {k: v.to(device) for k, v in batch.items()}
                prediction = policy.predict_action_chunk({k: v for k, v in batch.items()
                                                          if k.startswith('observation.')})
                error = (prediction - batch['action']).abs()
                valid = ~batch['action_is_pad']
                if execution_prefix > error.shape[1]:
                    raise ValueError('execution_prefix exceeds action chunk')
                if not torch.isfinite(error[valid]).all():
                    raise ValueError('non-finite validation error')
                transition = batch.get('action_near_phase_transition', torch.zeros_like(valid))
                values = {
                    'full_chunk': error[valid],
                    'first_action': error[:, :1][valid[:, :1]],
                    'execution_prefix': error[:, :execution_prefix][valid[:, :execution_prefix]],
                    'gripper': error[:, :, [5, 11]][valid],
                    'phase_transition': error[valid & transition],
                }
                for key, value in values.items():
                    totals[key][0] += value.double().sum().item()
                    totals[key][1] += value.numel()
    finally:
        for module, training in modes:
            module.training = training
    if not totals['full_chunk'][1]:
        raise ValueError('validation has no valid action elements')
    return {**{f'{key}_normalized_l1': total / count if count else None
               for key, (total, count) in totals.items()},
            'valid_elements': {key: count for key, (_, count) in totals.items()},
            'execution_prefix': execution_prefix,
            'phase_transition_window': 'last old-phase and first new-phase frames'}


def train(args):
    import torch
    from torch.utils.data import DataLoader
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig
    from dapier_act_policy import ACTPolicy
    if not .01 <= args.minutes <= 120:
        raise ValueError('training budget must be .01..120 minutes')
    max_steps = getattr(args, 'max_steps', None)
    if max_steps is not None and (type(max_steps) is not int or not 1 <= max_steps <= 2000):
        raise ValueError('max_steps must be an integer in 1..2000')
    torch.set_num_threads(4)
    torch.manual_seed(args.seed)
    if not torch.cuda.is_available():
        raise RuntimeError('this extended ACT run requires CUDA')
    manifest = json.loads((args.dataset / 'manifest.json').read_text())
    if manifest['fps'] != FPS or set(manifest['train']) & set(manifest['holdout']):
        raise ValueError('invalid SIM dataset split')
    dataset_validation = validate_dataset(args.dataset)
    resume = getattr(args, 'resume', None)
    stats = None
    if resume is not None:
        with np.load(resume.parent / 'normalization.npz') as saved:
            stats = {k: {s: saved[f'{k}_{s}'].copy() for s in ('mean', 'std')} for k in ('state', 'action')}
        if any(v.shape != (12,) or not np.isfinite(v).all() or (s == 'std' and (v <= 0).any())
               for entry in stats.values() for s, v in entry.items()):
            raise ValueError('invalid warm-start normalization')
    training = Demonstrations(args.dataset, manifest['train'], stats)
    holdout = Demonstrations(args.dataset, manifest['holdout'], training.stats)
    args.output.mkdir(parents=True, exist_ok=False)
    np.savez(args.output / 'normalization.npz', **{f'{k}_{s}': v for k, entry in training.stats.items() for s, v in entry.items()})
    config = ACTConfig(input_features={'observation.state': PolicyFeature(FeatureType.STATE, (12,)),
        **{key: PolicyFeature(FeatureType.VISUAL, (3, 240, 320)) for key in RGB_KEYS}},
        output_features={'action': PolicyFeature(FeatureType.ACTION, (12,))},
        chunk_size=CHUNK, n_action_steps=1, dim_model=256, dim_feedforward=1024,
        n_encoder_layers=4, n_vae_encoder_layers=4, device='cuda', push_to_hub=False,
        optimizer_lr=1e-4, optimizer_lr_backbone=1e-5)
    if resume is None:
        policy = ACTPolicy(config).cuda()
    else:
        policy = ACTPolicy.from_pretrained(resume, local_files_only=True, strict=True).cuda()
        if (policy.config.chunk_size != CHUNK or policy.config.input_features != config.input_features
                or policy.config.output_features != config.output_features):
            raise ValueError('warm-start checkpoint does not match PGripper ACT contract')
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=1e-4, weight_decay=1e-4)
    loader = DataLoader(training, batch_size=8, shuffle=True, num_workers=2, pin_memory=True)
    # Fixed, episode-disjoint validation subset; labels never enter the policy input.
    validation = torch.utils.data.Subset(holdout, np.linspace(0, len(holdout) - 1, min(128, len(holdout)), dtype=int))
    validation_loader = DataLoader(validation, batch_size=8, num_workers=0)
    initial_metrics = validate(policy, validation_loader, device='cuda',
                               execution_prefix=policy.config.n_action_steps)
    initial = initial_metrics['full_chunk_normalized_l1']
    policy.save_pretrained(args.output / 'initial')
    started, step, last_log = time.monotonic(), 0, time.monotonic()
    record = {'kind': 'DAPIER ACT (LeRobot 0.6.0 architecture), two wrist RGB + measured state, SIM ONLY', 'train_episodes': manifest['train'],
        'holdout_episodes': manifest['holdout'], 'initial_holdout_normalized_l1': initial,
        'initial_holdout_metrics': initial_metrics, 'max_steps_requested': max_steps,
        'normalization': 'normalization.npz plus ImageNet RGB mean/std; train episodes only',
        'task_success': None, 'minutes_requested': args.minutes, 'seed': args.seed,
        'warm_start': str(resume) if resume else None,
        'optimizer_restarted': resume is not None,
        'normalization_source': str(resume.parent / 'normalization.npz') if resume else 'current training episodes',
        'dataset': str(args.dataset.resolve())}
    record['dataset_validation'] = dataset_validation
    (args.output / 'run.json').write_text(json.dumps(record, indent=2) + '\n')
    with (args.output / 'metrics.jsonl').open('x') as log:
        while time.monotonic() - started < args.minutes * 60 and (max_steps is None or step < max_steps):
            for batch in loader:
                if time.monotonic() - started >= args.minutes * 60 or (max_steps is not None and step >= max_steps):
                    break
                policy.train()
                if not policy.training or not torch.is_grad_enabled() or torch.is_inference_mode_enabled():
                    raise RuntimeError('ACT training requires train mode and enabled gradients')
                batch = {k: v.cuda(non_blocking=True) for k, v in batch.items()}
                loss, metrics = policy(batch)
                if not torch.isfinite(loss):
                    raise RuntimeError('non-finite ACT loss')
                if not loss.requires_grad:
                    raise RuntimeError('ACT training loss has no gradient')
                optimizer.zero_grad(set_to_none=True)
                loss.backward()
                torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
                optimizer.step()
                step += 1
                if step == 1 or time.monotonic() - last_log > 60:
                    row = dict(step=step, elapsed_s=time.monotonic() - started, loss=float(loss), **metrics)
                    log.write(json.dumps(row) + '\n'); log.flush()
                    print(json.dumps(row), flush=True)
                    last_log = time.monotonic()
                if step % 1000 == 0:
                    policy.save_pretrained(args.output / f'step-{step:06d}')
                if time.monotonic() - started >= args.minutes * 60 or (max_steps is not None and step >= max_steps):
                    break
    final_metrics = validate(policy, validation_loader, device='cuda',
                             execution_prefix=policy.config.n_action_steps)
    final = final_metrics['full_chunk_normalized_l1']
    policy.save_pretrained(args.output / 'final')
    torch.save({'optimizer': optimizer.state_dict(), 'step': step, 'rng': torch.get_rng_state()}, args.output / 'training_state.pt')
    record.update(steps=step, training_seconds=time.monotonic() - started,
                  final_holdout_metrics=final_metrics,
                  final_holdout_normalized_l1=final, heldout_imitation_loss_improved=final < initial)
    (args.output / 'run.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('collect', 'finalize', 'train'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--dataset', type=Path)
    parser.add_argument('--resume', type=Path, help='warm-start weights with their original normalization; fresh optimizer')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--episodes', type=int, default=8)
    parser.add_argument('--minutes', type=float, default=60)
    parser.add_argument('--max-steps', type=int, help='optional optimizer update cap, 1..2000; minutes also applies')
    parser.add_argument('--seed', type=int, default=4100)
    parser.add_argument('--policy-target-hz', type=float, default=None)
    parser.add_argument('--physics-substeps', type=int, choices=(1, 2, 4), default=1)
    parser.add_argument('--command-dt-s', type=float, default=COMMAND_DT_S)
    args = parser.parse_args()
    {'collect': collect, 'finalize': finalize, 'train': train}[args.mode](args)
