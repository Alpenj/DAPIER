"""CPU-only validation regressions; no rollout, device access, or training run."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
from PIL import Image
import torch
from torch import nn
from torch.utils.data import DataLoader

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pgripper_learning import Demonstrations, train, validate


class ZeroPolicy(nn.Module):
    def __init__(self, chunk=4):
        super().__init__()
        self.chunk = chunk
        self.bn = nn.BatchNorm1d(12)

    def predict_action_chunk(self, inputs):
        assert set(inputs) == {'observation.state'}
        assert not self.training and torch.is_inference_mode_enabled()
        return self.bn(inputs['observation.state'])[:, None].expand(-1, self.chunk, -1)


class ValidationTest(unittest.TestCase):
    def rows(self, padding=0):
        rows = []
        for i, (values, length) in enumerate((([1, 9, 9, 9], 1), ([2, 4, 8, 9], 3), ([3, 6, 9, 12], 4))):
            action = torch.tensor(values + [float('nan')] * padding, dtype=torch.float64)[:, None].repeat(1, 12)
            action[:, [5, 11]] *= 2
            transition = torch.zeros(4 + padding, dtype=torch.bool)
            if i == 1:
                transition[1] = True
            rows.append({'observation.state': torch.zeros(12),
                         'action': action, 'action_is_pad': torch.arange(4 + padding) >= length,
                         'action_near_phase_transition': transition})
        return rows

    def test_global_l1_and_breakdowns_ignore_batch_partition_and_padding(self):
        for padding in (0, 3):
            for batch_size in (1, 2, 3):
                with self.subTest(padding=padding, batch_size=batch_size):
                    result = validate(ZeroPolicy(4 + padding), DataLoader(self.rows(padding), batch_size=batch_size),
                                      device='cpu', execution_prefix=2)
                    self.assertAlmostEqual(result['full_chunk_normalized_l1'], 45 * 14 / 96)
                    self.assertAlmostEqual(result['first_action_normalized_l1'], 6 * 14 / 36)
                    self.assertAlmostEqual(result['execution_prefix_normalized_l1'], 16 * 14 / 60)
                    self.assertAlmostEqual(result['gripper_normalized_l1'], 90 / 8)
                    self.assertAlmostEqual(result['phase_transition_normalized_l1'], 4 * 14 / 12)
                    self.assertEqual(result['valid_elements'],
                                     dict(full_chunk=96, first_action=36, execution_prefix=60,
                                          gripper=16, phase_transition=12))

    def test_validation_preserves_modes_parameters_buffers_and_restores_on_error(self):
        policy = ZeroPolicy()
        policy.train()
        policy.bn.eval()  # A deliberately mixed mode must also survive validation.
        before = {key: value.clone() for key, value in policy.state_dict().items()}
        validate(policy, DataLoader(self.rows(), batch_size=2), device='cpu')
        self.assertTrue(policy.training)
        self.assertFalse(policy.bn.training)
        for key, value in policy.state_dict().items():
            self.assertTrue(torch.equal(value, before[key]), key)
        policy.eval()
        rows = self.rows()
        rows[0]['action'][0, 0] = float('nan')
        with self.assertRaisesRegex(ValueError, 'non-finite'):
            validate(policy, DataLoader(rows), device='cpu')
        self.assertFalse(policy.training)
        self.assertFalse(policy.bn.training)
        self.assertTrue(torch.is_grad_enabled())
        policy.train()
        loss = policy.bn(torch.randn(2, 12)).square().mean()
        loss.backward()
        self.assertIsNotNone(policy.bn.weight.grad)

    def test_installed_act_predictor_and_following_gradient_are_compatible(self):
        from lerobot.configs.types import FeatureType, PolicyFeature
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        # Tiny synthetic env-state feature avoids downloading a vision backbone.
        config = ACTConfig(
            input_features={'observation.state': PolicyFeature(FeatureType.STATE, (12,)),
                            'observation.environment_state': PolicyFeature(FeatureType.ENV, (2,))},
            output_features={'action': PolicyFeature(FeatureType.ACTION, (12,))},
            chunk_size=4, n_action_steps=2, dim_model=16, n_heads=2, dim_feedforward=32,
            n_encoder_layers=1, n_decoder_layers=1, n_vae_encoder_layers=1, latent_dim=4,
            pretrained_backbone_weights=None, device='cpu', push_to_hub=False)
        policy = ACTPolicy(config)
        rows = self.rows()
        for row in rows:
            row['action'] = row['action'].float()
            row['observation.environment_state'] = torch.zeros(2)
        loader = DataLoader(rows, batch_size=3)
        policy.train()
        before = {key: value.clone() for key, value in policy.state_dict().items()}
        validate(policy, loader, device='cpu', execution_prefix=2)
        self.assertTrue(policy.training)
        for key, value in policy.state_dict().items():
            self.assertTrue(torch.equal(value, before[key]), key)
        batch = next(iter(loader))
        loss, _ = policy(batch)
        self.assertTrue(loss.requires_grad)
        loss.backward()
        self.assertTrue(any(p.grad is not None and torch.isfinite(p.grad).all() for p in policy.parameters()))

    def test_phase_metadata_and_original_normalization_stay_separate_from_observations(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            episode = root / 'episode-000'
            episode.mkdir()
            np.savez(episode / 'episode.npz', state=np.ones((4, 12), np.float32),
                     action=np.ones((4, 12), np.float32), phase=np.array(['a', 'a', 'b', 'b']))
            for side in ('left', 'right'):
                Image.fromarray(np.zeros((8, 8, 3), np.uint8)).save(episode / f'{side}-00000.jpg')
            stats = {key: {'mean': np.full(12, 5), 'std': np.full(12, 2)} for key in ('state', 'action')}
            dataset = Demonstrations(root, [0], stats)
            self.assertIs(dataset.stats, stats)
            row = dataset[0]
            self.assertEqual(row['action_near_phase_transition'][:4].tolist(), [False, True, True, False])
            self.assertTrue(torch.all(row['action'] == -2))
            self.assertTrue(torch.all(row['observation.state'] == -2))
            self.assertFalse(any('phase' in k for k in row if k.startswith('observation.')))

    def test_invalid_update_caps_rejected_before_cuda_or_dataset_access(self):
        for cap in (0, -1, 2001, 1.5, True):
            with self.subTest(cap=cap):
                with self.assertRaisesRegex(ValueError, 'max_steps'):
                    train(SimpleNamespace(minutes=1, max_steps=cap))


if __name__ == '__main__':
    torch.set_num_threads(1)
    unittest.main()
