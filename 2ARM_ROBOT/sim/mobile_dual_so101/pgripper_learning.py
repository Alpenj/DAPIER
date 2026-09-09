#!/usr/bin/env python3
"""SIM-only parallel demonstrations and two-wrist LeRobot ACT training.

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

from mobile_dual_so101 import actuator_targets_from_qpos
from pgripper_handover import run

RGB_KEYS = ('observation.images.left_wrist', 'observation.images.right_wrist')
FPS = 25
CHUNK = 50


def contract_vector(model, value, inverse=False):
    result = np.asarray(value, dtype=np.float64).copy()
    if result.shape != (12,) or not np.isfinite(result).all():
        raise ValueError('expected finite left-six/right-six vector')
    lo, hi = model.actuator_ctrlrange[[5, 11]].T
    result[[5, 11]] = result[[5, 11]] * (hi - lo) + lo if inverse else (result[[5, 11]] - lo) / (hi - lo)
    return result.astype(np.float32)


def collect_episode(options):
    output, episode, seed = options
    directory = Path(output) / f'episode-{episode:03d}'
    directory.mkdir(exist_ok=False)
    renderer = None
    rows = {key: [] for key in ('state', 'action', 'time', 'phase', 'physics', 'block')}
    dense_ctrl = []
    def observe(model, data, target, phase):
        nonlocal renderer
        dense_ctrl.append(target.copy())
        step = len(dense_ctrl) - 1
        interval = round(1 / FPS / model.opt.timestep)
        if not np.isclose(interval * model.opt.timestep, 1 / FPS):
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
                  contract_vector(model, target), float(data.time), phase, state,
                  data.body('red_block').xpos.copy())
        for key, value in zip(rows, values):
            rows[key].append(value)
    args = argparse.Namespace(output=directory / 'expert', model=None, donor='left',
        viewer=False, render=False, handover_only=False, disable_recipient_contact=False)
    rng = np.random.default_rng(seed)
    offset = rng.uniform(-np.deg2rad(.5), np.deg2rad(.5), 12)
    offset[[5, 11]] = 0
    try:
        code = run(args, observer=observe, initial_arm_offset=offset)
    finally:
        if renderer is not None:
            renderer.close()
    np.savez_compressed(directory / 'episode.npz', **{k: np.asarray(v) for k, v in rows.items()},
                        dense_ctrl=np.asarray(dense_ctrl), seed=seed, initial_arm_offset=offset)
    return {'episode': episode, 'seed': seed, 'frames': len(rows['time']), 'pid': os.getpid(), 'task_success': code == 0}


def collect(args):
    if not 2 <= args.episodes <= 64 or not 1 <= args.workers <= 8:
        raise ValueError('expected 2..64 episodes and 1..8 workers')
    args.output.mkdir(parents=True, exist_ok=False)
    options = [(str(args.output), i, args.seed + i) for i in range(args.episodes)]
    with ProcessPoolExecutor(args.workers, mp_context=multiprocessing.get_context('spawn')) as pool:
        episodes = list(pool.map(collect_episode, options))
    finalize(args, episodes)


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
                episodes.append({'episode': episode, 'task_success': report['task_success'],
                    'frames': len(data['state']) if data is not None else 0, 'pid': None})
    accepted = [e['episode'] for e in episodes if e['task_success']]
    if len(accepted) < 4:
        raise ValueError('need at least four complete successful demonstrations')
    split = max(1, len(accepted) * 3 // 4)
    manifest = {'kind': 'SIM fixed-scene PGripper expert', 'fps': FPS, 'rgb_shape': [240, 320, 3],
        'state_action_units': 'left6/right6; arm radians; gripper 0..1',
        'action_source': 'requested simulator ctrl before mj_step, not next-state labels',
        'randomization': 'initial arm positions only +/-0.5 degree; fixed cube and fixture',
        'train': accepted[:split], 'holdout': accepted[split:],
        'episodes': episodes, 'hardware_execution': False}
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
        for side, key in zip(('left', 'right'), RGB_KEYS):
            with Image.open(self.root / f'episode-{episode:03d}' / f'{side}-{t:05d}.jpg') as im:
                rgb = torch.from_numpy(np.asarray(im).copy()).permute(2, 0, 1).float() / 255
            batch[key] = (rgb - torch.tensor([.485, .456, .406])[:, None, None]) / torch.tensor([.229, .224, .225])[:, None, None]
        return batch


def train(args):
    import torch
    from torch.utils.data import DataLoader
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    if not .01 <= args.minutes <= 120:
        raise ValueError('training budget must be .01..120 minutes')
    torch.set_num_threads(4)
    torch.manual_seed(args.seed)
    if not torch.cuda.is_available():
        raise RuntimeError('this extended ACT run requires CUDA')
    manifest = json.loads((args.dataset / 'manifest.json').read_text())
    if manifest['fps'] != FPS or set(manifest['train']) & set(manifest['holdout']):
        raise ValueError('invalid SIM dataset split')
    training = Demonstrations(args.dataset, manifest['train'])
    holdout = Demonstrations(args.dataset, manifest['holdout'], training.stats)
    args.output.mkdir(parents=True, exist_ok=False)
    np.savez(args.output / 'normalization.npz', **{f'{k}_{s}': v for k, entry in training.stats.items() for s, v in entry.items()})
    config = ACTConfig(input_features={'observation.state': PolicyFeature(FeatureType.STATE, (12,)),
        **{key: PolicyFeature(FeatureType.VISUAL, (3, 240, 320)) for key in RGB_KEYS}},
        output_features={'action': PolicyFeature(FeatureType.ACTION, (12,))},
        chunk_size=CHUNK, n_action_steps=1, dim_model=256, dim_feedforward=1024,
        n_encoder_layers=4, n_vae_encoder_layers=4, device='cuda', push_to_hub=False,
        optimizer_lr=1e-4, optimizer_lr_backbone=1e-5)
    policy = ACTPolicy(config).cuda()
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=1e-4, weight_decay=1e-4)
    loader = DataLoader(training, batch_size=8, shuffle=True, num_workers=2, pin_memory=True)
    # Fixed, episode-disjoint validation subset; labels never enter the policy input.
    validation = torch.utils.data.Subset(holdout, np.linspace(0, len(holdout) - 1, min(128, len(holdout)), dtype=int))
    validation_loader = DataLoader(validation, batch_size=8, num_workers=0)
    def validate():
        errors = []
        with torch.inference_mode():
            for batch in validation_loader:
                batch = {k: v.cuda() for k, v in batch.items()}
                inputs = {k: v for k, v in batch.items() if k.startswith('observation.')}
                prediction = policy.predict_action_chunk(inputs)
                valid = ~batch['action_is_pad']
                errors.append(float((prediction - batch['action']).abs()[valid].mean()))
        return float(np.mean(errors))
    initial = validate()
    policy.save_pretrained(args.output / 'initial')
    started, step, last_log = time.monotonic(), 0, time.monotonic()
    record = {'kind': 'LeRobot ACT, two wrist RGB + measured state, SIM ONLY', 'train_episodes': manifest['train'],
        'holdout_episodes': manifest['holdout'], 'initial_holdout_normalized_l1': initial,
        'normalization': 'normalization.npz plus ImageNet RGB mean/std; train episodes only',
        'task_success': None, 'minutes_requested': args.minutes, 'seed': args.seed}
    (args.output / 'run.json').write_text(json.dumps(record, indent=2) + '\n')
    with (args.output / 'metrics.jsonl').open('x') as log:
        while time.monotonic() - started < args.minutes * 60:
            for batch in loader:
                policy.train()
                batch = {k: v.cuda(non_blocking=True) for k, v in batch.items()}
                loss, metrics = policy(batch)
                if not torch.isfinite(loss):
                    raise RuntimeError('non-finite ACT loss')
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
                if time.monotonic() - started >= args.minutes * 60:
                    break
    final = validate()
    policy.save_pretrained(args.output / 'final')
    torch.save({'optimizer': optimizer.state_dict(), 'step': step, 'rng': torch.get_rng_state()}, args.output / 'training_state.pt')
    record.update(steps=step, training_seconds=time.monotonic() - started,
                  final_holdout_normalized_l1=final, heldout_imitation_loss_improved=final < initial)
    (args.output / 'run.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('collect', 'finalize', 'train'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--dataset', type=Path)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--episodes', type=int, default=8)
    parser.add_argument('--minutes', type=float, default=60)
    parser.add_argument('--seed', type=int, default=4100)
    args = parser.parse_args()
    {'collect': collect, 'finalize': finalize, 'train': train}[args.mode](args)
