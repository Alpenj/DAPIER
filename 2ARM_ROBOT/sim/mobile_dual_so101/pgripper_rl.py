#!/usr/bin/env python3
"""SIM-only PPO residual trajectory/contact curriculum in independent MuJoCo processes.

The base commands are an explicit expert reference, NOT ACT inference. Segment
success is not full pickup/handover/place success. Object truth is reward-only.
"""
import argparse
from functools import partial
import json
import os
from pathlib import Path
import time

import gymnasium as gym
import mujoco
import numpy as np

from collision_guard import _collision_geoms_for_arm, _same_arm_geom_pairs
from mobile_dual_so101 import actuator_targets_from_qpos, resolve_so101_model
from pgripper_handover import target_pad_forces
from pgripper_learning import FPS, contract_vector
from replay_recorded_episode import build_tabletop_spec


class HandoverCurriculum(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, dataset, episode_ids, horizon=100):
        super().__init__()
        self.references = [dict(np.load(Path(dataset) / f'episode-{i:03d}' / 'episode.npz')) for i in episode_ids]
        if not self.references or not 25 <= horizon <= 2500:
            raise ValueError('reference episodes and a bounded horizon are required')
        profile = json.loads(Path(__file__).with_name('tabletop_replay.json').read_text())
        self.model = build_tabletop_spec(resolve_so101_model(None), profile, grippers='both').compile()
        self.model.opt.impratio = 10
        self.data = mujoco.MjData(self.model)
        self.substeps = round(1 / FPS / self.model.opt.timestep)
        if not np.isclose(self.substeps * self.model.opt.timestep, 1 / FPS):
            raise ValueError('control clock must divide the physics timestep')
        self.horizon = horizon
        self.damping = self.model.dof_damping.copy()
        self.action_space = gym.spaces.Box(-1, 1, (12,), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (37,), np.float32)
        self.scale = np.full(12, .02)
        self.scale[[5, 11]] = .004
        self.pairs = {frozenset((self.model.geom('red_block_geom').id,
            self.model.geom(f'{side}_pgripper_pad_{finger}').id)): i * 2 + finger - 1
            for i, side in enumerate(('left', 'right')) for finger in (1, 2)}
        sides = [(self.model.body(int(b)).name or '').split('_')[0] for b in self.model.geom_bodyid]
        self.forbidden = {frozenset(p) for side in ('left', 'right')
            for p in _same_arm_geom_pairs(self.model, _collision_geoms_for_arm(self.model, side))}
        table = self.model.geom('table').id
        self.forbidden.update(frozenset((a, b)) for a in range(self.model.ngeom) for b in range(a)
            if {sides[a], sides[b]} == {'left', 'right'} or
            (table in (a, b) and (sides[a] in ('left', 'right') or sides[b] in ('left', 'right'))))

    def observation(self):
        state = contract_vector(self.model, actuator_targets_from_qpos(self.model, self.data.qpos))
        target = self.ref['action'][self.index]
        observation = np.r_[state, target, state - target, self.index / len(self.ref['state'])].astype(np.float32)
        if not np.isfinite(observation).all():
            raise ValueError('non-finite measured observation')
        return observation

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.ref = self.references[int(self.np_random.integers(len(self.references)))]
        self.start = int(self.np_random.integers(max(1, len(self.ref['state']) - self.horizon - 1)))
        self.index = self.start
        self.end = min(self.start + self.horizon, len(self.ref['state']) - 1)
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_setState(self.model, self.data, self.ref['physics'][self.index], mujoco.mjtState.mjSTATE_INTEGRATION)
        self.model.dof_damping[:] = self.damping * self.np_random.uniform(.9, 1.1)
        mujoco.mj_forward(self.model, self.data)
        self.previous = contract_vector(self.model, self.data.ctrl)
        self.max_object_error = 0.
        self.tracking_sum = 0.
        return self.observation(), {'pid': os.getpid(), 'start_frame': self.start}

    def step(self, action):
        action = np.asarray(action, dtype=np.float64)
        if action.shape != (12,) or not np.isfinite(action).all() or np.max(np.abs(action)) > 1.00001:
            raise ValueError('PPO residual must be finite and within [-1, 1]')
        failure = None
        for substep in range(self.substeps):
            target = contract_vector(self.model, self.ref['dense_ctrl'][self.index * self.substeps + substep])
            target += action * self.scale
            sim_target = contract_vector(self.model, target, inverse=True)
            sim_target = np.clip(sim_target, *self.model.actuator_ctrlrange.T)
            # Every residual command is slew-limited, including a new policy sample.
            rates = np.full(12, .6)
            rates[[5, 11]] = .6
            sim_target = self.data.ctrl + np.clip(sim_target - self.data.ctrl,
                -rates * self.model.opt.timestep, rates * self.model.opt.timestep)
            self.data.ctrl[:] = sim_target
            mujoco.mj_step(self.model, self.data)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all() or any(w.number for w in self.data.warning):
                failure = 'invalid physics'; break
            if any(c.dist < -.001 or frozenset((c.geom1, c.geom2)) in self.forbidden for c in self.data.contact):
                failure = 'penetration or forbidden arm collision'; break
            if target_pad_forces(self.model, self.data, self.pairs).max() > 10:
                failure = 'pad overload'; break
        self.index += 1
        state = contract_vector(self.model, actuator_targets_from_qpos(self.model, self.data.qpos))
        error = float(np.mean((state - self.ref['state'][self.index]) ** 2))
        object_error = float(np.linalg.norm(self.data.body('red_block').xpos - self.ref['block'][self.index]))
        self.max_object_error = max(self.max_object_error, object_error)
        self.tracking_sum += error
        forces = target_pad_forces(self.model, self.data, self.pairs)
        phase = str(self.ref['phase'][self.index])
        if phase in ('donor lift', 'donor present', 'recipient orient away', 'recipient approach', 'recipient insert', 'recipient close') and forces[0].min() < .3:
            failure = failure or 'donor grip lost'
        if phase in ('donor release', 'donor retreat', 'recipient carry', 'place approach', 'place seat', 'place support confirm') and forces[1].min() < .3:
            failure = failure or 'recipient grip lost'
        if object_error > .03:
            failure = failure or 'object escaped reference corridor'
        done = failure is not None
        truncated = self.index >= self.end and not done
        segment_success = truncated and self.max_object_error < .015 and self.tracking_sum / (self.index - self.start) < .0025
        reward = float(np.exp(-100 * error) + np.exp(-5000 * object_error ** 2) - .01 * np.mean(action ** 2))
        reward += -20 if done else (20 if segment_success else (-10 if truncated else 0))
        info = {'is_success': bool(segment_success), 'task_success': None, 'failure': failure,
            'tracking_mse': error, 'object_error_m': object_error, 'phase': phase, 'pid': os.getpid()}
        if done and failure == 'invalid physics':
            observation = np.zeros(37, dtype=np.float32)
        else:
            observation = self.observation()
        return observation, reward, done, truncated, info


def evaluate(policy, dataset, ids, seeds=range(9000, 9016)):
    env = HandoverCurriculum(dataset, ids)
    reports = []
    try:
        for seed in seeds:
            obs, _ = env.reset(seed=seed)
            total, errors = 0., []
            while True:
                action = np.zeros(12, np.float32) if policy is None else policy.predict(obs, deterministic=True)[0]
                obs, reward, done, truncated, info = env.step(action)
                total += reward
                errors.append(info['tracking_mse'])
                if done or truncated:
                    reports.append(dict(info, seed=seed, reward=total, mean_tracking_mse=float(np.mean(errors))))
                    break
    finally:
        env.close()
    return {'segment_success_rate': float(np.mean([r['is_success'] for r in reports])),
        'mean_reward': float(np.mean([r['reward'] for r in reports])), 'task_success': None, 'episodes': reports}


def train(args):
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import SubprocVecEnv
    from stable_baselines3.common.logger import configure
    if not 2 <= args.workers <= 8 or not .01 <= args.minutes <= 120:
        raise ValueError('expected 2..8 environments and .01..120 minutes')
    torch.set_num_threads(2)
    manifest = json.loads((args.dataset / 'manifest.json').read_text())
    if set(manifest['train']) & set(manifest['holdout']):
        raise ValueError('train/holdout overlap')
    args.output.mkdir(parents=True, exist_ok=False)
    env = SubprocVecEnv([partial(make_environment, str(args.dataset), manifest['train']) for _ in range(args.workers)], start_method='spawn')
    model = PPO('MlpPolicy', env, device='cpu', seed=args.seed, n_steps=256, batch_size=256,
        n_epochs=5, learning_rate=3e-4, target_kl=.02, policy_kwargs={'net_arch': [128, 128]}, verbose=1)
    model.set_logger(configure(str(args.output), ['stdout', 'csv']))
    model.save(args.output / 'initial')
    baseline = evaluate(None, args.dataset, manifest['holdout'])
    initial = evaluate(model, args.dataset, manifest['holdout'])
    started = time.monotonic()
    record = {'kind': 'PPO residual expert-reference contact/trajectory curriculum',
        'base_controller': 'expert reference, NOT ACT', 'workers': args.workers,
        'worker_pids': [p.pid for p in env.processes], 'physics_device': 'CPU', 'learner_device': 'CPU',
        'minutes_requested': args.minutes, 'seed': args.seed, 'hardware_execution': False,
        'actor_inputs': 'measured joints, nominal reference joint command, tracking delta, trajectory fraction',
        'privileged_reward_only': 'simulator block position and contact forces',
        'reward': 'joint/object tracking shaping; +20 segment success; -20 failure; -10 unsuccessful horizon',
        'randomization': 'episode snapshot start; damping 0.9..1.1; fixed object fixture',
        'full_task_success': None, 'baseline': baseline, 'initial': initial}
    (args.output / 'run.json').write_text(json.dumps(record, indent=2) + '\n')
    class Budget(BaseCallback):
        def _on_step(self):
            if self.num_timesteps % (args.workers * 10000) == 0:
                self.model.save(args.output / f'step-{self.num_timesteps:09d}')
            return time.monotonic() - started < args.minutes * 60
    try:
        model.learn(total_timesteps=100_000_000, callback=Budget(), log_interval=4)
        model.save(args.output / 'final')
        record.update(training_seconds=time.monotonic() - started, timesteps=model.num_timesteps,
            updates=model._n_updates, final=evaluate(model, args.dataset, manifest['holdout']))
        (args.output / 'run.json').write_text(json.dumps(record, indent=2) + '\n')
        print(json.dumps({k: v for k, v in record.items() if k not in ('baseline', 'initial', 'final')}), flush=True)
    finally:
        env.close()


def make_environment(dataset, ids):
    from stable_baselines3.common.monitor import Monitor
    return Monitor(HandoverCurriculum(dataset, ids))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--minutes', type=float, default=60)
    parser.add_argument('--seed', type=int, default=6100)
    train(parser.parse_args())
