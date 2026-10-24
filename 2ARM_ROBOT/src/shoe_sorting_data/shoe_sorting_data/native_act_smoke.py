"""Optional LeRobot Dataset v3 round-trip and ACT input-contract smoke gate.

All LeRobot and Torch imports stay inside the executable path so the ROS 2
recorder environment remains independent from the ML environment.
"""

from __future__ import annotations

from datetime import datetime, timezone
import importlib.metadata
import importlib.util
import json
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any, Mapping
from urllib.parse import unquote, urlparse


NATIVE_ACT_SMOKE_SCHEMA_VERSION = "dapier.native-act-smoke.v0.1"
NATIVE_ACT_REQUIRED_MODULES = ("lerobot", "torch", "torchvision", "numpy", "datasets", "pyarrow")
EXPECTED_TAIL_MASKS = (
    (False, False, False),
    (False, False, True),
    (False, True, True),
)


class NativeActSmokeError(RuntimeError):
    pass


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def native_act_dependency_status() -> dict[str, Any]:
    modules = {name: importlib.util.find_spec(name) is not None for name in NATIVE_ACT_REQUIRED_MODULES}
    missing = [name for name, available in modules.items() if not available]
    return {
        "available": not missing,
        "modules": modules,
        "missing": missing,
        "base_recorder_affected": False,
    }


def _lerobot_provenance() -> dict[str, Any]:
    distribution = importlib.metadata.distribution("lerobot")
    direct_url_text = distribution.read_text("direct_url.json")
    direct_url = json.loads(direct_url_text) if direct_url_text else None
    provenance: dict[str, Any] = {
        "version": distribution.version,
        "direct_url": direct_url,
        "git_commit": None,
    }
    if isinstance(direct_url, dict):
        url = direct_url.get("url")
        if isinstance(url, str) and url.startswith("file:"):
            parsed = urlparse(url)
            decoded_path = unquote(parsed.path)
            if sys.platform == "win32" and len(decoded_path) >= 3 and decoded_path[0] == "/" and decoded_path[2] == ":":
                decoded_path = decoded_path[1:]
            local_path = Path(decoded_path)
            try:
                result = subprocess.run(
                    ["git", "-C", str(local_path), "rev-parse", "HEAD"],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                provenance["git_commit"] = result.stdout.strip()
            except (OSError, subprocess.SubprocessError):
                pass
    return provenance


def _tensor_shape(value: Any) -> list[int]:
    return list(value.shape)


def run_native_act_smoke(
    dataset_root: str | Path,
    *,
    repo_id: str,
    chunk_size: int = 3,
) -> dict[str, Any]:
    """Verify native v3 reopen, temporal windows, DataLoader, and ACT one-forward.

    The 2 episode x 3 frame fixture uses chunk_size=3 only to exercise full and
    tail-padded windows. It is not a training or rollout hyperparameter claim.
    """

    root = Path(dataset_root).resolve()
    if not root.is_dir():
        raise ValueError(f"native dataset root does not exist: {root}")
    if chunk_size != 3:
        raise ValueError("the Stage 3 fixture contract requires chunk_size=3")
    dependency_status = native_act_dependency_status()
    receipt_path = root / "dapier_act_roundtrip_receipt.json"
    if not dependency_status["available"]:
        receipt = {
            "schema_version": NATIVE_ACT_SMOKE_SCHEMA_VERSION,
            "created_at_utc": _utc_now(),
            "status": "SKIP",
            "reason": "optional native ACT dependencies are missing",
            "dependency_status": dependency_status,
        }
        _write_json(receipt_path, receipt)
        return receipt

    try:
        import torch
        from torch.utils.data import DataLoader

        from lerobot.configs.types import FeatureType
        from lerobot.datasets.factory import resolve_delta_timestamps
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.utils.constants import ACTION, OBS_STATE
        from lerobot.utils.feature_utils import dataset_to_policy_features

        base_dataset = LeRobotDataset(repo_id, root=root)
        if len(base_dataset) != 6 or base_dataset.num_episodes != 2:
            raise ValueError(
                f"Stage 3 fixture must be exactly 2 episodes x 3 frames; got "
                f"episodes={base_dataset.num_episodes}, frames={len(base_dataset)}"
            )
        policy_features = dataset_to_policy_features(base_dataset.meta.features)
        rgb_key = "observation.images.workspace_rgb"
        depth_key = "observation.images.workspace_depth"
        for key in (OBS_STATE, ACTION, rgb_key, depth_key):
            if key not in policy_features:
                raise ValueError(f"native dataset feature is missing: {key}")
        if policy_features[OBS_STATE].shape != (12,) or policy_features[ACTION].shape != (12,):
            raise ValueError("ACT smoke requires 12-dimensional observation.state and action")
        if policy_features[rgb_key].shape[0] != 3:
            raise ValueError("ACT RGB baseline requires a 3-channel workspace_rgb feature")
        if policy_features[depth_key].shape[0] != 1:
            raise ValueError("native depth contract requires a 1-channel workspace_depth feature")
        if min(policy_features[rgb_key].shape[1:]) < 64:
            raise ValueError("ACT forward smoke requires RGB fixture dimensions of at least 64x64")

        config = ACTConfig(
            input_features={
                OBS_STATE: policy_features[OBS_STATE],
                rgb_key: policy_features[rgb_key],
            },
            output_features={ACTION: policy_features[ACTION]},
            chunk_size=chunk_size,
            n_action_steps=1,
            device="cpu",
            use_amp=False,
            push_to_hub=False,
            pretrained_backbone_weights=None,
            dim_model=64,
            n_heads=4,
            dim_feedforward=128,
            n_encoder_layers=1,
            n_decoder_layers=1,
            use_vae=True,
            latent_dim=8,
            n_vae_encoder_layers=1,
            dropout=0.0,
        )
        delta_timestamps = resolve_delta_timestamps(config, base_dataset.meta)
        expected_delta = [index / base_dataset.fps for index in range(chunk_size)]
        if delta_timestamps is None or delta_timestamps.get(ACTION) != expected_delta:
            raise ValueError(
                f"official ACT delta timestamps differ from export FPS: "
                f"expected={expected_delta}, actual={None if delta_timestamps is None else delta_timestamps.get(ACTION)}"
            )

        dataset = LeRobotDataset(repo_id, root=root, delta_timestamps=delta_timestamps)
        observed_masks: list[list[bool]] = []
        for episode_start in (0, 3):
            for frame_offset, expected_mask in enumerate(EXPECTED_TAIL_MASKS):
                sample = dataset[episode_start + frame_offset]
                observed = tuple(bool(value) for value in sample["action_is_pad"].tolist())
                if observed != expected_mask:
                    raise ValueError(
                        f"wrong action_is_pad at dataset index {episode_start + frame_offset}: "
                        f"expected={expected_mask}, actual={observed}"
                    )
                observed_masks.append(list(observed))

        episode0_tail = dataset[2]
        episode1_first = base_dataset[3][ACTION]
        if not torch.equal(episode0_tail[ACTION][0], base_dataset[2][ACTION]):
            raise ValueError("episode 0 tail lost its current action")
        if torch.equal(episode0_tail[ACTION][1], episode1_first) or torch.equal(
            episode0_tail[ACTION][2], episode1_first
        ):
            raise ValueError("action window leaked across the episode 0 to 1 boundary")

        loader = DataLoader(dataset, batch_size=1, shuffle=False, num_workers=0)
        batch = next(iter(loader))
        if tuple(batch[ACTION].shape) != (1, chunk_size, 12):
            raise ValueError(f"wrong DataLoader action shape: {tuple(batch[ACTION].shape)}")
        if tuple(batch["action_is_pad"].shape) != (1, chunk_size):
            raise ValueError(f"wrong DataLoader action_is_pad shape: {tuple(batch['action_is_pad'].shape)}")

        torch.manual_seed(0)
        policy = ACTPolicy(config)
        policy.train()
        with torch.no_grad():
            loss, loss_dict = policy(batch)
        if not torch.isfinite(loss):
            raise ValueError("ACT one-forward produced a non-finite loss")

        receipt = {
            "schema_version": NATIVE_ACT_SMOKE_SCHEMA_VERSION,
            "created_at_utc": _utc_now(),
            "status": "PASS",
            "scope": "writer-reopen-dataloader-act-one-forward; not training or task success",
            "dataset": {
                "root": str(root),
                "repo_id": repo_id,
                "fps": base_dataset.fps,
                "episode_count": base_dataset.num_episodes,
                "frame_count": len(base_dataset),
                "state_dim": 12,
                "action_dim": 12,
                "rgb_shape_chw": list(policy_features[rgb_key].shape),
                "depth_shape_chw": list(policy_features[depth_key].shape),
            },
            "temporal_contract": {
                "chunk_size": chunk_size,
                "n_action_steps": 1,
                "not_for_training_hyperparameter": True,
                "delta_timestamps_seconds": expected_delta,
                "tail_masks": observed_masks,
                "cross_episode_no_leak": True,
            },
            "dataloader": {
                "batch_size": 1,
                "num_workers": 0,
                "action_shape": _tensor_shape(batch[ACTION]),
                "action_is_pad_shape": _tensor_shape(batch["action_is_pad"]),
            },
            "act_forward": {
                "device": "cpu",
                "rgb_only_baseline": True,
                "depth_exclusion_reason": "official ACT ResNet input is 3-channel; native depth remains a verified auxiliary feature",
                "finite_loss": True,
                "model_init_seed": 0,
                "loss": float(loss.item()),
                "loss_dict": loss_dict,
            },
            "runtime": {
                "python": sys.version.split()[0],
                "platform": platform.platform(),
                "torch": torch.__version__,
                "cuda_available": torch.cuda.is_available(),
                "lerobot": _lerobot_provenance(),
            },
            "dependency_status": dependency_status,
        }
        _write_json(receipt_path, receipt)
        encoder_receipt_path = root / "dapier_encoder_receipt.json"
        if encoder_receipt_path.is_file():
            encoder_receipt = json.loads(encoder_receipt_path.read_text(encoding="utf-8"))
            encoder_receipt["round_trip"] = {
                "status": "PASS",
                "receipt": receipt_path.name,
                "schema_version": NATIVE_ACT_SMOKE_SCHEMA_VERSION,
            }
            _write_json(encoder_receipt_path, encoder_receipt)
        return receipt
    except Exception as error:
        receipt = {
            "schema_version": NATIVE_ACT_SMOKE_SCHEMA_VERSION,
            "created_at_utc": _utc_now(),
            "status": "FAIL",
            "error_type": type(error).__name__,
            "error": str(error),
            "dependency_status": dependency_status,
        }
        _write_json(receipt_path, receipt)
        raise NativeActSmokeError(f"native ACT smoke failed: {error}") from error
