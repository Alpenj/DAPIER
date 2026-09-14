"""CPU-only compatibility checks for the repo-owned PGripper ACT model."""
from pathlib import Path
import sys
import tempfile
import unittest

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dapier_act_policy import ACTPolicy as DAPIERACTPolicy
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy as LeRobotACTPolicy


class DAPIERACTPolicyCompatibilityTest(unittest.TestCase):
    def test_rgb_inference_and_checkpoint_match_lerobot_0_6_0(self):
        config = ACTConfig(
            input_features={
                "observation.state": PolicyFeature(FeatureType.STATE, (12,)),
                "observation.images.left_wrist": PolicyFeature(FeatureType.VISUAL, (3, 32, 32)),
                "observation.images.right_wrist": PolicyFeature(FeatureType.VISUAL, (3, 32, 32)),
            },
            output_features={"action": PolicyFeature(FeatureType.ACTION, (12,))},
            chunk_size=4,
            n_action_steps=2,
            dim_model=16,
            n_heads=2,
            dim_feedforward=32,
            n_encoder_layers=1,
            n_decoder_layers=1,
            n_vae_encoder_layers=1,
            latent_dim=4,
            pretrained_backbone_weights=None,
            device="cpu",
            push_to_hub=False,
        )
        torch.manual_seed(7)
        reference = LeRobotACTPolicy(config).eval()
        policy = DAPIERACTPolicy(config).eval()
        policy.load_state_dict(reference.state_dict())
        batch = {
            "observation.state": torch.randn(1, 12),
            "observation.images.left_wrist": torch.randn(1, 3, 32, 32),
            "observation.images.right_wrist": torch.randn(1, 3, 32, 32),
        }

        expected = reference.predict_action_chunk(batch)
        actual = policy.predict_action_chunk(batch)
        self.assertTrue(torch.equal(actual, expected))
        self.assertEqual(actual.shape, (1, 4, 12))

        with tempfile.TemporaryDirectory() as tmp:
            policy.save_pretrained(tmp)
            restored = DAPIERACTPolicy.from_pretrained(
                tmp, local_files_only=True, strict=True
            ).eval()
            self.assertTrue(torch.equal(restored.predict_action_chunk(batch), actual))


if __name__ == "__main__":
    torch.set_num_threads(1)
    unittest.main()
