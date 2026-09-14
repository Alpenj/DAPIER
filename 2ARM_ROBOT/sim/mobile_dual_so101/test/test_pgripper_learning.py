"""Hardware-free dataset boundary and PPO action/reward checks."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mobile_dual_so101 import apply_control_as_pose, actuator_targets_from_qpos, resolve_so101_model
from pgripper import home_action
from pgripper_learning import CHUNK, Demonstrations, contract_vector, finalize
from replay_recorded_episode import build_tabletop_spec


class LearningContractTest(unittest.TestCase):
    def test_units_dataset_padding_and_episode_split(self):
        model = SimpleNamespace(actuator_ctrlrange=np.tile([0., 2.2028], (12, 1)))
        target = np.linspace(0, 1, 12)
        np.testing.assert_allclose(contract_vector(model, contract_vector(model, target), inverse=True), target, atol=1e-6)
        for bad in ([0] * 11, [np.nan] * 12):
            with self.assertRaises(ValueError):
                contract_vector(model, bad)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for i in range(4):
                directory = root / f'episode-{i:03d}'
                directory.mkdir()
                np.savez(directory / 'episode.npz', state=np.full((2, 12), i, np.float32),
                    action=np.full((2, 12), i + .5, np.float32))
                for side in ('left', 'right'):
                    for t in range(2):
                        Image.fromarray(np.zeros((240, 320, 3), np.uint8)).save(directory / f'{side}-{t:05d}.jpg')
            train = Demonstrations(root, [0, 1])
            held = Demonstrations(root, [3], train.stats)
            self.assertIs(held.stats, train.stats)
            self.assertEqual(float(train.stats['state']['mean'][0]), .5)
            batch = train[1]
            self.assertEqual(tuple(batch['action'].shape), (CHUNK, 12))
            self.assertEqual(int(batch['action_is_pad'].sum()), CHUNK - 1)
            self.assertEqual(set(batch), {'observation.state', 'action', 'action_is_pad',
                'observation.images.left_wrist', 'observation.images.right_wrist'})
            episodes = [{'episode': i, 'task_success': True, 'canonical_full_task_success': True} for i in range(4)]
            episodes.append({'episode': 9, 'task_success': False})
            finalize(SimpleNamespace(output=root), episodes)
            manifest = json.loads((root / 'manifest.json').read_text())
            self.assertEqual(manifest['train'], [0, 1, 2])
            self.assertEqual(manifest['holdout'], [3])
            with self.assertRaises(ValueError):
                finalize(SimpleNamespace(output=root), episodes)

    def test_ppo_sim_transition_and_rejection(self):
        from pgripper_rl import HandoverCurriculum
        from stable_baselines3.common.env_checker import check_env
        profile = json.loads(Path(__file__).resolve().parents[1].joinpath('tabletop_replay.json').read_text())
        model = build_tabletop_spec(resolve_so101_model(), profile, grippers='both').compile()
        model.opt.impratio = 10
        data = mujoco.MjData(model)
        command = np.asarray(home_action(model, np.tile(np.deg2rad([0, -35, 55, 35, 0, 0]), 2)))
        apply_control_as_pose(model, data, command)
        state = contract_vector(model, actuator_targets_from_qpos(model, data.qpos))
        physics = np.empty(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_getState(model, data, physics, mujoco.mjtState.mjSTATE_INTEGRATION)
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / 'episode-000'
            directory.mkdir()
            # Constant home fixture, not a claimed successful handover demonstration.
            np.savez(directory / 'episode.npz', state=np.tile(state, (120, 1)),
                action=np.tile(contract_vector(model, command), (120, 1)),
                physics=np.tile(physics, (120, 1)), dense_ctrl=np.tile(command, (2400, 1)),
                block=np.tile(data.body('red_block').xpos, (120, 1)), phase=np.full(120, 'settle'))
            env = HandoverCurriculum(tmp, [0], horizon=25)
            check_env(env)
            obs, _ = env.reset(seed=42)
            start = env.data.time
            next_obs, reward, done, truncated, info = env.step(np.zeros(12, np.float32))
            self.assertGreater(env.data.time, start)
            self.assertEqual(next_obs.shape, (37,))
            self.assertTrue(np.isfinite(reward))
            self.assertIsNone(info['task_success'])
            self.assertFalse(info['is_success'])
            for bad in (np.full(12, np.nan), np.full(12, 2.), np.zeros(11)):
                before = env.data.time
                with self.assertRaises(ValueError):
                    env.step(bad)
                self.assertEqual(env.data.time, before)
            env.close()


if __name__ == '__main__':
    unittest.main()
