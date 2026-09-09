#!/usr/bin/env python3
"""Report-only, SIM-only whole-task evaluation of frozen ACT or reference-residual PPO."""
import argparse
import json
from pathlib import Path
import time

import mujoco
import numpy as np
from PIL import Image

from collision_guard import _collision_geoms_for_arm, _same_arm_geom_pairs
from mobile_dual_so101 import apply_control_as_pose, actuator_targets_from_qpos, resolve_so101_model
from pgripper import home_action
from pgripper_handover import run as expert_run, target_pad_forces, table_support_force
from pgripper_learning import FPS, RGB_KEYS, contract_vector
from replay_recorded_episode import build_tabletop_spec, inspect_contacts, physics_settings, sha256


class TaskProgress:
    """Verifier only: progression is established by physics, never policy/expert labels."""
    def __init__(self):
        self.events = {}
        self.left_hold = self.right_hold = self.recipient_hold = self.support_hold = self.placed_hold = 0.
        self.failure = None

    def update(self, dt, timestamp, forces, position, speed, support):
        values = np.r_[dt, timestamp, np.asarray(forces).ravel(), position, speed, support]
        if not np.isfinite(values).all() or dt <= 0:
            raise ValueError('invalid verifier evidence')
        left, right = np.asarray(forces)
        self.left_hold = self.left_hold + dt if left.min() >= 1 else 0.
        if self.left_hold >= .33 and 'left_grasp' not in self.events:
            self.events['left_grasp'] = timestamp
        if 'left_grasp' in self.events and left.min() >= .3 and position[2] > .06:
            self.events.setdefault('left_lift', timestamp)
        self.right_hold = self.right_hold + dt if 'left_lift' in self.events and right.min() >= 1 else 0.
        if self.right_hold >= .33:
            self.events.setdefault('right_grasp', timestamp)
        holding = ('right_grasp' in self.events and left.max() < .01 and right.min() >= .3 and position[2] > .06)
        self.recipient_hold = self.recipient_hold + dt if holding else 0.
        if self.recipient_hold >= 3 - 1e-9:
            self.events.setdefault('handover', timestamp)
        if 'left_lift' in self.events and 'right_grasp' not in self.events and left.min() < .3 and support < .05:
            self.failure = 'donor lost the object before verified recipient grip'
        if ('right_grasp' in self.events and 'table_support' not in self.events
                and left.max() < .01 and right.min() < .3 and support < .05):
            self.failure = 'recipient lost the object before table support'
        self.support_hold = self.support_hold + dt if 'handover' in self.events and support >= .05 else 0.
        if self.support_hold >= .2:
            self.events.setdefault('table_support', timestamp)
        placed = ('table_support' in self.events and np.asarray(forces).max() < .01 and support >= .1
            and .019 <= position[2] <= .022 and speed <= .01 and np.linalg.norm(np.asarray(position)[:2] - [.22, -.08]) <= .003)
        self.placed_hold = self.placed_hold + dt if placed else 0.
        if self.placed_hold >= 3 - 1e-9:
            self.events.setdefault('placed', timestamp)
        return 'placed' in self.events and self.failure is None


def evidence(model, data, pairs):
    dof = int(model.joint('red_block_free').dofadr[0])
    return (target_pad_forces(model, data, pairs), data.body('red_block').xpos.copy(),
            float(np.linalg.norm(data.qvel[dof:dof + 3])), table_support_force(model, data))


def contact_pairs(model):
    return {frozenset((model.geom('red_block_geom').id, model.geom(f'{side}_pgripper_pad_{finger}').id)): i * 2 + finger - 1
        for i, side in enumerate(('left', 'right')) for finger in (1, 2)}


def act_observation(state, images, stats, device):
    import torch
    if np.shape(state) != (12,) or not np.isfinite(state).all() or set(images) != set(RGB_KEYS):
        raise ValueError('ACT must receive measured state and exactly two wrist images')
    result = {'observation.state': torch.tensor((state - stats['state_mean']) / stats['state_std'], device=device)[None]}
    for key, rgb in images.items():
        if rgb.shape != (240, 320, 3) or rgb.dtype != np.uint8:
            raise ValueError('invalid wrist image')
        im = torch.from_numpy(rgb.copy()).permute(2, 0, 1).float().to(device) / 255
        result[key] = ((im - torch.tensor([.485, .456, .406], device=device)[:, None, None])
            / torch.tensor([.229, .224, .225], device=device)[:, None, None])[None]
    return result


def calibrate(args):
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for disabled in (False, True):
        progress = TaskProgress()
        last_time = 0.
        def observe(model, data, target, phase):
            nonlocal last_time
            if data.time > last_time:
                progress.update(data.time - last_time, float(data.time), *evidence(model, data, contact_pairs(model)))
                last_time = float(data.time)
        run_args = argparse.Namespace(model=None, output=args.output / ('no-recipient-contact' if disabled else 'expert'),
            donor='left', viewer=False, render=False, handover_only=False, disable_recipient_contact=disabled)
        code = expert_run(run_args, observer=observe)
        accepted = 'placed' in progress.events and progress.failure is None
        if accepted != (not disabled) or (code == 0) != (not disabled):
            raise RuntimeError(f'verifier calibration mismatch: {progress.events}, {progress.failure}')
        results.append({'disabled_recipient_contact': disabled, 'task_success': accepted, 'events': progress.events})
    (args.output / 'calibration.json').write_text(json.dumps(results, indent=2) + '\n')


def evaluate(args):
    import torch
    torch.set_num_threads(2)
    if not 1 <= args.episodes <= 8 or not 1 <= args.seconds <= 120:
        raise ValueError('bounded evaluation: 1..8 episodes, 1..120 simulation seconds')
    for protected in (args.checkpoint, args.dataset):
        if protected and (args.output.resolve() == protected.resolve() or protected.resolve() in args.output.resolve().parents):
            raise ValueError('output must be outside model and dataset')
    args.output.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__), Path(__file__).with_name('pgripper_learning.py'), Path(__file__).with_name('pgripper.py'),
        Path(__file__).with_name('tabletop_replay.json'), Path(__file__).with_name('pgripper_handover.py')]
    policy, stats, reference = None, None, None
    if args.mode == 'act':
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.configs.policies import PreTrainedConfig
        config = PreTrainedConfig.from_pretrained(args.checkpoint, local_files_only=True)
        if (set(config.input_features) != {'observation.state', *RGB_KEYS} or config.n_action_steps != 1
                or list(config.output_features['action'].shape) != [12]):
            raise ValueError('wrong ACT checkpoint contract')
        config.device = 'cuda'
        policy = ACTPolicy.from_pretrained(args.checkpoint, config=config, local_files_only=True).eval().cuda()
        stats_path = args.checkpoint.parent / 'normalization.npz'
        stats = dict(np.load(stats_path))
        for key in ('state_mean', 'state_std', 'action_mean', 'action_std'):
            if stats[key].shape != (12,) or not np.isfinite(stats[key]).all() or (key.endswith('std') and np.any(stats[key] <= 0)):
                raise ValueError('invalid training normalization')
        sources += [args.checkpoint / 'model.safetensors', args.checkpoint / 'config.json', stats_path]
    elif args.mode == 'ppo':
        from stable_baselines3 import PPO
        policy = PPO.load(args.checkpoint, device='cpu')
        sources.append(args.checkpoint)
    hashes = {str(p.resolve()): sha256(p) for p in sources}
    reports = []
    for trial in range(args.episodes):
        output = args.output / f'trial-{trial:02d}'
        output.mkdir()
        profile = json.loads(Path(__file__).with_name('tabletop_replay.json').read_text())
        model = build_tabletop_spec(resolve_so101_model(None), profile, grippers='both').compile()
        model.opt.impratio = 10
        data = mujoco.MjData(model)
        if args.mode in ('ppo', 'reference'):
            manifest = json.loads((args.dataset / 'manifest.json').read_text())
            episode = manifest['holdout'][trial % len(manifest['holdout'])]
            reference_path = args.dataset / f'episode-{episode:03d}' / 'episode.npz'
            reference = dict(np.load(reference_path))
            hashes[str(reference_path.resolve())] = sha256(reference_path)
            # Full task starts at frame zero, never a successful mid-task snapshot.
            mujoco.mj_setState(model, data, reference['physics'][0], mujoco.mjtState.mjSTATE_INTEGRATION)
            mujoco.mj_forward(model, data)
        else:
            command = np.asarray(home_action(model, np.tile(np.deg2rad([0, -35, 55, 35, 0, 0]), 2)))
            rng = np.random.default_rng(args.seed + trial)
            offset = rng.uniform(-np.deg2rad(.5), np.deg2rad(.5), 12) if trial else np.zeros(12)
            offset[[5, 11]] = 0
            apply_control_as_pose(model, data, command + offset)
            policy.reset()
        initial_time = float(data.time)
        initial_qpos = data.qpos.copy()
        renderer = mujoco.Renderer(model, width=320, height=240)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [.19, 0, .14]
        camera.distance, camera.azimuth, camera.elevation = .85, 135, -30
        def save_image(label):
            renderer.update_scene(data, camera=camera)
            Image.fromarray(renderer.render()).save(output / f'{label}.png')
        pairs = contact_pairs(model)
        forbidden = {frozenset(p) for side in ('left', 'right')
            for p in _same_arm_geom_pairs(model, _collision_geoms_for_arm(model, side))}
        sides = [(model.body(int(b)).name or '').split('_')[0] for b in model.geom_bodyid]
        table = model.geom('table').id
        forbidden.update(frozenset((a, b)) for a in range(model.ngeom) for b in range(a)
            if {sides[a], sides[b]} == {'left', 'right'} or
            (table in (a, b) and (sides[a] in ('left', 'right') or sides[b] in ('left', 'right'))))
        progress, deepest = TaskProgress(), {'depth_m': 0.}
        trace, failure, queries, saved_events = [], None, 0, set()
        dt = float(model.opt.timestep)
        substeps = round(1 / FPS / dt)
        if not np.isclose(substeps * dt, 1 / FPS):
            raise ValueError('invalid evaluation control clock')
        started = time.monotonic()
        save_image('initial')
        try:
            for step in range(int(args.seconds * FPS)):
                captured_time = float(data.time)
                state = contract_vector(model, actuator_targets_from_qpos(model, data.qpos))
                if args.mode == 'act':
                    images = {}
                    for side, key in zip(('left', 'right'), RGB_KEYS):
                        renderer.update_scene(data, camera=f'{side}_wrist_rgb')
                        images[key] = renderer.render().copy()
                    batch = act_observation(state, images, stats, 'cuda')
                    with torch.inference_mode():
                        raw = policy.select_action(batch)[0].cpu().numpy() * stats['action_std'] + stats['action_mean']
                    queries += 1
                else:
                    index = min(step, len(reference['action']) - 1)
                    nominal = reference['action'][index]
                    observation = np.r_[state, nominal, state - nominal, index / len(reference['state'])].astype(np.float32)
                    residual = np.zeros(12) if policy is None else policy.predict(observation, deterministic=True)[0]
                    if np.shape(residual) != (12,) or not np.isfinite(residual).all() or np.max(np.abs(residual)) > 1.00001:
                        raise ValueError('invalid PPO action')
                    queries += policy is not None
                if data.time != captured_time:
                    raise RuntimeError('state advanced during inference')
                for substep in range(substeps):
                    if args.mode != 'act':
                        index = min(step * substeps + substep, len(reference['dense_ctrl']) - 1)
                        scale = np.full(12, .02); scale[[5, 11]] = .004
                        raw = contract_vector(model, reference['dense_ctrl'][index]) + residual * scale
                    target = contract_vector(model, raw, inverse=True)
                    target = np.clip(target, *model.actuator_ctrlrange.T)
                    target = data.ctrl + np.clip(target - data.ctrl, -.6 * dt, .6 * dt)
                    data.ctrl[:] = target
                    mujoco.mj_step(model, data)
                    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
                        raise ValueError('invalid physics or MuJoCo warning')
                    inspect_contacts(model, data, deepest, step)
                    if deepest['depth_m'] > .001 or any(frozenset((c.geom1, c.geom2)) in forbidden for c in data.contact):
                        raise ValueError('penetration or forbidden arm collision')
                    forces, position, speed, support = evidence(model, data, pairs)
                    if forces.max() > 10:
                        raise ValueError('virtual pad overload')
                    progress.update(dt, float(data.time), forces, position, speed, support)
                    if progress.failure:
                        raise ValueError(progress.failure)
                trace.append({'time_s': float(data.time), 'observation_time_s': captured_time, 'measured_state': state.tolist(),
                    'raw_action': raw.tolist(), 'sent_ctrl': data.ctrl.tolist(), 'forces_N': forces.tolist(),
                    'block_position_m': position.tolist(), 'support_N': support, 'block_speed_m_s': speed})
                for event in set(progress.events) - saved_events:
                    save_image(event); saved_events.add(event)
                if 'placed' in progress.events:
                    break
                if step % 500 == 0:
                    print(json.dumps({'mode': args.mode, 'trial': trial, 'simulation_s': data.time, 'events': progress.events}), flush=True)
            else:
                failure = 'full-task timeout'
        except ValueError as error:
            failure = str(error)
        finally:
            save_image('final')
            renderer.close()
        report = {'mode': args.mode, 'trial': trial, 'seed': args.seed + trial, 'task_success': failure is None and 'placed' in progress.events,
            'failure': failure, 'events': progress.events, 'simulation_seconds': float(data.time),
            'wall_seconds': time.monotonic() - started, 'policy_queries': int(queries), 'deepest_contact': deepest,
            'physics': physics_settings(model), 'initial_time': initial_time, 'initial_qpos': initial_qpos.tolist(),
            'final_block_position_m': data.body('red_block').xpos.tolist(), 'warning_counts': [int(w.number) for w in data.warning],
            'reference_actions_used': args.mode != 'act', 'object_truth_in_policy': False,
            'runtime_qpos_writes_after_reset': 0, 'hardware_execution': False, 'physical_policy_isolation_proven': False,
            'recommendation': 'report-only', 'actor_interface': 'tensor observations only; no file/tool actions'}
        (output / 'trace.json').write_text(json.dumps(trace, allow_nan=False) + '\n')
        (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        reports.append(report)
        print(json.dumps(report), flush=True)
    summary = {'mode': args.mode, 'episodes': reports, 'successes': sum(r['task_success'] for r in reports),
        'trials': len(reports), 'input_sha256': hashes,
        'inputs_unchanged': all(sha256(Path(p)) == h for p, h in hashes.items()), 'report_only': True,
        'limitations': ['Fixed fixture; not calibrated sim-to-real.', 'PPO uses expert reference, not ACT.',
            'In-process tensor policy; OS-level isolation from verifier files is not established.']}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('act', 'ppo', 'reference', 'calibrate'))
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--dataset', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--episodes', type=int, default=2)
    parser.add_argument('--seconds', type=float, default=110)
    parser.add_argument('--seed', type=int, default=12000)
    args = parser.parse_args()
    (calibrate if args.mode == 'calibrate' else evaluate)(args)
