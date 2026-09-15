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
from pgripper_handover import run as expert_run, target_pad_forces, table_support_force, PolicyTargetExecutor, finite_list
from pgripper_learning import FPS, RGB_KEYS, contract_vector
from pgripper_execution import (COMMAND_DT_S, TaskProgress, evidence, contact_pairs,
    configure_physics, step_physics, execution_metadata)
from replay_recorded_episode import build_tabletop_spec, inspect_contacts, physics_settings, sha256


class LeftPickHoldProgress(TaskProgress):
    """Separate partial-task verifier; never counts as full handover success."""
    def __init__(self):
        super().__init__()
        self.stable_hold = 0.

    def update(self, dt, timestamp, forces, position, speed, support):
        super().update(dt, timestamp, forces, position, speed, support)
        left, right = np.asarray(forces)
        if right.max() >= .01:
            self.failure = 'right arm contacted object in left-only task'
        stable = ('left_lift' in self.events and left.min() >= .3 and right.max() < .01
                  and position[2] > .06 and support < .05 and speed <= .01)
        self.stable_hold = self.stable_hold + dt if stable else 0.
        if self.stable_hold >= 3 - 1e-9 and self.failure is None:
            self.events.setdefault('left_held', timestamp)
        return 'left_held' in self.events and self.failure is None


def reference_ctrl(dense, step, substep, substeps, clock):
    """Same saved commands: dense playback or a held first/end 25Hz target."""
    if clock not in ('dense', 'first', 'end') or substeps <= 0 or len(dense) < substeps:
        raise ValueError('invalid reference clock or incomplete reference')
    if clock == 'dense':
        index = min(step * substeps + substep, len(dense) - 1)
    else:
        # Match ACT staging: discard a partial final interval, then hold the last full target.
        index = min(step, len(dense) // substeps - 1) * substeps
        index += substeps - 1 if clock == 'end' else 0
    return dense[index]


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


def restore_frame_zero(model, data, state, policy=None):
    state = np.asarray(state, dtype=float)
    if (state.shape != (mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION),)
            or not np.isfinite(state).all() or state[0] != 0):
        raise ValueError('initial state must be a finite frame-zero integration state')
    mujoco.mj_setState(model, data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(model, data)
    if policy is not None:
        policy.reset()


def frame_evidence(model, data, pairs, raw, projected, state, captured_time, step, substep):
    forces, position, speed, support = evidence(model, data, pairs)
    return {'frame': step, 'command_in_frame': substep, 'time_s': float(data.time),
        'observation_time_s': captured_time, 'measured_state': finite_list(state),
        'current_measured_state': finite_list(contract_vector(model, actuator_targets_from_qpos(model, data.qpos))),
        'raw_action': finite_list(raw), 'projected_target': finite_list(projected) if projected is not None else None,
        'sent_ctrl': finite_list(data.ctrl), 'forces_N': finite_list(forces),
        'block_position_m': finite_list(position), 'support_N': support, 'block_speed_m_s': speed}


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
    command_dt = getattr(args, 'command_dt_s', COMMAND_DT_S)
    physics_substeps = getattr(args, 'physics_substeps', 1)
    range_policy = getattr(args, 'range_policy', 'project')
    initial_episode = getattr(args, 'initial_episode', None)
    full_reference = getattr(args, 'reference_full_sequence', False)
    task = getattr(args, 'task', 'full')
    left_only = task == 'left-pick-hold'
    block_offset = np.asarray(getattr(args, 'block_offset', (0., 0.)), dtype=float)
    if (task not in ('full', 'left-pick-hold') or block_offset.shape != (2,)
            or not np.isfinite(block_offset).all() or np.max(np.abs(block_offset)) > .02
            or (not left_only and np.any(block_offset)) or (left_only and (full_reference or args.mode == 'ppo'))):
        raise ValueError('invalid bounded left-only evaluation settings')
    success_event = 'left_held' if left_only else 'placed'
    if full_reference and args.mode != 'reference':
        raise ValueError('full reference sequence is a reference-only diagnostic')
    if not 1 <= args.episodes <= 40 or not 1 <= args.seconds <= 120:
        raise ValueError('bounded evaluation: 1..40 episodes, 1..120 simulation seconds')
    if args.reference_clock != 'dense' and args.mode != 'reference':
        raise ValueError('reference-clock experiments require the reference baseline, not a learned policy')
    for protected in (args.checkpoint, args.dataset):
        if protected and (args.output.resolve() == protected.resolve() or protected.resolve() in args.output.resolve().parents):
            raise ValueError('output must be outside model and dataset')
    args.output.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__), Path(__file__).with_name('pgripper_learning.py'), Path(__file__).with_name('pgripper.py'),
        Path(__file__).with_name('tabletop_replay.json'), Path(__file__).with_name('pgripper_handover.py'),
        Path(__file__).with_name('pgripper_execution.py')]
    policy, stats, reference = None, None, None
    if args.mode == 'act':
        from dapier_act_policy import ACTPolicy
        from lerobot.configs.policies import PreTrainedConfig
        config = PreTrainedConfig.from_pretrained(args.checkpoint, local_files_only=True)
        if (set(config.input_features) != {'observation.state', *RGB_KEYS} or config.n_action_steps != 1
                or list(config.output_features['action'].shape) != [12]):
            raise ValueError('wrong ACT checkpoint contract')
        if not 1 <= args.action_steps <= config.chunk_size or config.temporal_ensemble_coeff is not None:
            raise ValueError('action_steps must fit the trained chunk; temporal ensembling is a separate experiment')
        config.n_action_steps = args.action_steps
        config.device = 'cuda'
        policy = ACTPolicy.from_pretrained(
            args.checkpoint, config=config, local_files_only=True, strict=True
        ).eval().cuda()
        stats_path = args.checkpoint.parent / 'normalization.npz'
        stats = dict(np.load(stats_path))
        for key in ('state_mean', 'state_std', 'action_mean', 'action_std'):
            if stats[key].shape != (12,) or not np.isfinite(stats[key]).all() or (key.endswith('std') and np.any(stats[key] <= 0)):
                raise ValueError('invalid training normalization')
        sources += [Path(__file__).with_name('dapier_act_policy.py'),
                    args.checkpoint / 'model.safetensors', args.checkpoint / 'config.json', stats_path]
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
        configure_physics(model, physics_substeps, command_dt)
        model.opt.impratio = 10
        data = mujoco.MjData(model)
        if args.mode in ('ppo', 'reference'):
            if initial_episode is not None:
                reference_path = initial_episode
            else:
                manifest = json.loads((args.dataset / 'manifest.json').read_text())
                episode = manifest['holdout'][trial % len(manifest['holdout'])]
                reference_path = args.dataset / f'episode-{episode:03d}' / 'episode.npz'
            reference = dict(np.load(reference_path))
            dense = reference['dense_ctrl']
            if dense.ndim != 2 or dense.shape[1] != 12 or not np.isfinite(dense).all():
                raise ValueError('invalid reference commands')
            hashes[str(reference_path.resolve())] = sha256(reference_path)
            # Full task starts at frame zero, never a successful mid-task snapshot.
            restore_frame_zero(model, data, reference['physics'][0])
        else:
            command = np.asarray(home_action(model, np.tile(np.deg2rad([0, -35, 55, 35, 0, 0]), 2)))
            rng = np.random.default_rng(args.seed + trial)
            offset = rng.uniform(-np.deg2rad(.5), np.deg2rad(.5), 12) if trial else np.zeros(12)
            offset[[5, 11]] = 0
            apply_control_as_pose(model, data, command + offset)
            policy.reset()
            if initial_episode is not None:
                with np.load(initial_episode) as saved:
                    restore_frame_zero(model, data, saved['physics'][0], policy)
                hashes[str(initial_episode.resolve())] = sha256(initial_episode)
        if left_only:
            address = int(model.joint('red_block_free').qposadr[0])
            data.qpos[address:address + 2] += block_offset
            mujoco.mj_forward(model, data)
        fixed_right = contract_vector(model, actuator_targets_from_qpos(model, data.qpos))[6:].copy()
        maximum_right_drift = 0.
        initial_time = float(data.time)
        initial_qpos = data.qpos.copy()
        initial_integration = np.empty(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_getState(model, data, initial_integration, mujoco.mjtState.mjSTATE_INTEGRATION)
        np.savez_compressed(output / 'initial-state.npz', integration=initial_integration)
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
        progress = LeftPickHoldProgress() if left_only else TaskProgress()
        deepest = {'depth_m': 0.}
        trace, failure, queries, network_queries, saved_events = [], None, 0, 0, set()
        dt = command_dt
        substeps = round(1 / FPS / dt)
        if not np.isclose(substeps * dt, 1 / FPS):
            raise ValueError('invalid evaluation control clock')
        target_hz = 1 / dt if reference is not None and args.reference_clock == 'dense' else FPS
        executor = PolicyTargetExecutor(model, target_hz, command_dt_s=dt, range_policy=range_policy)
        substep_peaks, inference_seconds = {}, []
        raw, state, projected, captured_time, step, substep = np.zeros(12), np.zeros(12), None, float(data.time), 0, -1
        started = time.monotonic()
        save_image('initial')
        try:
            frame_count = int(np.ceil(len(reference['dense_ctrl']) / substeps)) if full_reference else int(args.seconds * FPS)
            if frame_count / FPS > args.seconds:
                raise ValueError('reference sequence exceeds evaluation time bound')
            for step in range(frame_count):
                substep, projected = -1, None
                captured_time = float(data.time)
                state = contract_vector(model, actuator_targets_from_qpos(model, data.qpos))
                if args.mode == 'act':
                    images = {}
                    for side, key in zip(('left', 'right'), RGB_KEYS):
                        renderer.update_scene(data, camera=f'{side}_wrist_rgb')
                        images[key] = renderer.render().copy()
                    batch = act_observation(state, images, stats, 'cuda')
                    network_queries += len(policy._action_queue) == 0
                    inference_started = time.monotonic()
                    with torch.inference_mode():
                        raw = policy.select_action(batch)[0].cpu().numpy() * stats['action_std'] + stats['action_mean']
                    inference_seconds.append(time.monotonic() - inference_started)
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
                command_count = min(substeps, len(reference['dense_ctrl']) - step * substeps) if full_reference else substeps
                for substep in range(command_count):
                    if args.mode != 'act':
                        scale = np.full(12, .02); scale[[5, 11]] = .004
                        raw = contract_vector(model, reference_ctrl(reference['dense_ctrl'], step, substep,
                            substeps, args.reference_clock)) + residual * scale
                    requested = raw.copy()
                    if left_only:
                        requested[6:] = fixed_right
                    target = executor.step(data.ctrl, requested)
                    projected = contract_vector(model, executor.target)
                    data.ctrl[:] = target
                    step_physics(model, data, physics_substeps, command_dt_s=dt, peaks=substep_peaks, pairs=pairs)
                    if left_only:
                        right = contract_vector(model, actuator_targets_from_qpos(model, data.qpos))[6:]
                        maximum_right_drift = max(maximum_right_drift, float(np.max(np.abs(right - fixed_right))))
                        if maximum_right_drift > .02:
                            raise ValueError('fixed right arm drift exceeded .02 rad/normalized gripper')
                    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
                        raise ValueError('invalid physics or MuJoCo warning')
                    inspect_contacts(model, data, deepest, step)
                    if deepest['depth_m'] > .001 or any(frozenset((c.geom1, c.geom2)) in forbidden for c in data.contact):
                        raise ValueError('penetration or forbidden arm collision')
                    forces, position, speed, support = evidence(model, data, pairs)
                    if substep_peaks.get('maximum_pad_force_N', 0) > 10 or substep_peaks.get('maximum_penetration_m', 0) > .001:
                        raise ValueError('substep force or penetration limit exceeded')
                    progress.update(dt, float(data.time), forces, position, speed, support)
                    if progress.failure:
                        raise ValueError(progress.failure)
                trace.append(frame_evidence(model, data, pairs, raw, projected, state, captured_time, step, substep))
                for event in set(progress.events) - saved_events:
                    save_image(event); saved_events.add(event)
                if success_event in progress.events and not full_reference:
                    break
                if step % 500 == 0:
                    print(json.dumps({'mode': args.mode, 'trial': trial, 'simulation_s': data.time, 'events': progress.events}), flush=True)
            else:
                failure = None if success_event in progress.events else ('left-pick-hold timeout' if left_only else 'full-task timeout')
        except ValueError as error:
            failure = str(error)
            try:
                failed = frame_evidence(model, data, pairs, raw, projected, state, captured_time, step, substep)
            except ValueError:
                failed = {'frame': step, 'command_in_frame': substep, 'raw_action': finite_list(raw),
                    'sent_ctrl': finite_list(data.ctrl), 'qpos': finite_list(data.qpos), 'qvel': finite_list(data.qvel)}
            failed['failure'] = failure
            trace.append(failed)
        finally:
            save_image('final')
            renderer.close()
        final_integration = np.empty_like(initial_integration)
        mujoco.mj_getState(model, data, final_integration, mujoco.mjtState.mjSTATE_INTEGRATION)
        np.savez_compressed(output / 'final-state.npz', integration=final_integration,
            qpos=data.qpos, qvel=data.qvel, ctrl=data.ctrl, qacc_warmstart=data.qacc_warmstart)
        report = {'mode': args.mode, 'trial': trial, 'seed': args.seed + trial,
            'task': task, 'task_success': failure is None and success_event in progress.events,
            'full_task_success': failure is None and 'placed' in progress.events,
            'right_arm_fixed': left_only, 'maximum_right_drift': maximum_right_drift if left_only else None,
            'block_offset_m': block_offset.tolist(),
            'policy_action_override': 'right six axes fixed at initial measured pose' if left_only else None,
            'range_checked_action': 'phase-masked proposal' if left_only else 'raw policy proposal',
            'failure': failure, 'events': progress.events, 'simulation_seconds': float(data.time),
            'wall_seconds': time.monotonic() - started, 'policy_queries': int(queries), 'deepest_contact': deepest,
            'act_action_steps': args.action_steps if args.mode == 'act' else None,
            'act_network_queries': int(network_queries),
            'inference_seconds': inference_seconds,
            'range_policy': range_policy, 'range_events': executor.range_events,
            'raw_output_valid': not executor.range_events and all(np.isfinite(r.get('raw_action', [])).all() if None not in r.get('raw_action', []) else False for r in trace),
            'range_violation_count': len(executor.range_events),
            'saturated_axes': sorted({axis for event in executor.range_events for axis in event['axes']}),
            'maximum_range_excess': max((event['maximum_excess'] for event in executor.range_events), default=0.),
            'execution': execution_metadata(model, physics_substeps, target_hz, dt), 'substep_peaks': substep_peaks,
            'reference_clock': args.reference_clock if reference is not None else None,
            'target_update_hz': 1 / dt if reference is not None and args.reference_clock == 'dense' else FPS,
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
    parser.add_argument('--reference-clock', choices=('dense', 'first', 'end'), default='dense',
        help='reference baseline only: 500Hz dense commands or 25Hz interval-first/interval-end hold')
    parser.add_argument('--action-steps', type=int, default=1,
        help='explicit ACT execution experiment: how many predicted actions before replanning; checkpoint unchanged')
    parser.add_argument('--initial-episode', type=Path, help='same finite frame-zero physics[0] for ACT and reference')
    parser.add_argument('--physics-substeps', type=int, choices=(1, 2, 4), default=1)
    parser.add_argument('--command-dt-s', type=float, default=COMMAND_DT_S)
    parser.add_argument('--range-policy', choices=('project', 'reject'), default='project')
    parser.add_argument('--reference-full-sequence', action='store_true', help='continue saved reference through its final command for equal-time state comparison')
    parser.add_argument('--task', choices=('full', 'left-pick-hold'), default='full')
    parser.add_argument('--block-offset', type=float, nargs=2, default=(0., 0.), metavar=('X', 'Y'),
        help='left-only reset-time block XY offset in metres; each bounded to +/- .02')
    args = parser.parse_args()
    (calibrate if args.mode == 'calibrate' else evaluate)(args)
