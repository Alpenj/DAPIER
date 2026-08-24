"""Small, LeRobot-independent ACT runtime for finalized DAPIER episodes.

PyTorch and NumPy are optional and imported only when this runtime is used.
The implementation owns dataset windows, RGB-D decoding, CVAE action-chunk
training, checkpoints, and inference.  It does not claim compatibility with
official ACT checkpoints.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import argparse
import importlib.util
import json
from pathlib import Path
import random
import sys
from typing import Any, Mapping, Sequence

from shoe_sorting_data.act_interchange import POLICY_STREAM_ORDER
from shoe_sorting_data.camera_payload import read_camera_payload
from shoe_sorting_data.contract import load_manifest
from shoe_sorting_data.quality import validate_episode
from shoe_sorting_data.synthetic import generate_episode


RUNTIME_SCHEMA_VERSION = "dapier.native-act-runtime.v0.1"


def dependency_status() -> dict[str, Any]:
    modules = {name: importlib.util.find_spec(name) is not None for name in ("torch", "numpy")}
    missing = [name for name, available in modules.items() if not available]
    return {
        "available": not missing,
        "modules": modules,
        "missing": missing,
        "lerobot_required": False,
    }


def _require_ml():
    status = dependency_status()
    if not status["available"]:
        raise RuntimeError("DAPIER-native ACT requires the optional ML environment: " + ", ".join(status["missing"]))
    import numpy as np
    import torch

    return np, torch


def _flatten(sample: Mapping[str, Any], group_name: str) -> list[float]:
    group = sample[group_name]
    return [float(value) for stream in POLICY_STREAM_ORDER for value in group[stream]]


def _load_rgbd(
    episode_dir: Path,
    sample: Mapping[str, Any],
    *,
    depth_scale: float,
    max_depth_m: float,
    include_depth: bool,
):
    np, torch = _require_ml()
    cameras = sample["cameras"]
    rgb = read_camera_payload(episode_dir, "workspace_rgb", cameras["workspace_rgb"]["payload"])
    depth = read_camera_payload(episode_dir, "workspace_depth", cameras["workspace_depth"]["payload"])

    rgb_channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "8uc1": 1}
    encoding = rgb.encoding.lower()
    channels = rgb_channels[encoding]
    rgb_rows = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.step)[:, : rgb.width * channels]
    rgb_array = rgb_rows.reshape(rgb.height, rgb.width, channels)
    if encoding in {"bgr8", "bgra8"}:
        rgb_array = rgb_array[..., [2, 1, 0]]
    elif channels == 4:
        rgb_array = rgb_array[..., :3]
    elif channels == 1:
        rgb_array = np.repeat(rgb_array, 3, axis=2)

    depth_encoding = depth.encoding.lower()
    if depth_encoding == "32fc1":
        dtype = np.dtype(">f4" if depth.is_bigendian else "<f4")
        scale = 1.0
    elif depth_encoding == "16sc1":
        dtype = np.dtype(">i2" if depth.is_bigendian else "<i2")
        scale = depth_scale
    else:
        dtype = np.dtype(">u2" if depth.is_bigendian else "<u2")
        scale = depth_scale
    depth_rows = np.frombuffer(depth.data, dtype=dtype).reshape(depth.height, depth.step // dtype.itemsize)
    depth_array = depth_rows[:, : depth.width].astype(np.float32) / scale
    depth_array = np.clip(depth_array, 0.0, max_depth_m) / max_depth_m

    if (rgb.height, rgb.width) != (depth.height, depth.width):
        raise ValueError("workspace RGB and depth payload dimensions must match")
    rgb_tensor = torch.from_numpy(rgb_array.copy()).permute(2, 0, 1).float() / 255.0
    depth_tensor = torch.from_numpy(depth_array.copy()).unsqueeze(0)
    return torch.cat((rgb_tensor, depth_tensor), dim=0) if include_depth else rgb_tensor


class DAPIERACTDataset:
    """Accepted raw episodes exposed as independent ACT temporal windows."""

    def __init__(
        self,
        root: str | Path,
        *,
        chunk_size: int,
        split: str = "train",
        depth_scale: float = 1000.0,
        max_depth_m: float = 5.0,
        include_depth: bool = True,
    ) -> None:
        if chunk_size <= 0 or depth_scale <= 0 or max_depth_m <= 0:
            raise ValueError("chunk_size, depth_scale, and max_depth_m must be positive")
        self.chunk_size = chunk_size
        self.depth_scale = depth_scale
        self.max_depth_m = max_depth_m
        self.include_depth = include_depth
        self.episodes: list[dict[str, Any]] = []
        self.index: list[tuple[int, int]] = []
        manifests = sorted(Path(root).resolve().rglob("episode_manifest.json"))
        if not manifests:
            raise ValueError(f"no episode_manifest.json files found below: {root}")
        for manifest_path in manifests:
            manifest = load_manifest(manifest_path)
            if manifest["provenance"]["source_split"] != split:
                continue
            report = validate_episode(manifest_path)
            if not report.usable:
                raise ValueError(f"episode is not quality accepted: {manifest_path}")
            sample_path = manifest_path.parent / manifest["recording"]["sample_file"]
            samples = [json.loads(line) for line in sample_path.read_text(encoding="utf-8").splitlines()]
            episode_index = len(self.episodes)
            self.episodes.append(
                {
                    "episode_id": manifest["episode_id"],
                    "directory": manifest_path.parent,
                    "samples": samples,
                }
            )
            self.index.extend((episode_index, frame_index) for frame_index in range(len(samples)))
        if not self.index:
            raise ValueError(f"no accepted {split!r} samples found below: {root}")
        first = self.episodes[0]["samples"][0]
        self.state_dim = len(_flatten(first, "state"))
        self.action_dim = len(_flatten(first, "action"))
        first_image = _load_rgbd(
            self.episodes[0]["directory"],
            first,
            depth_scale=self.depth_scale,
            max_depth_m=self.max_depth_m,
            include_depth=self.include_depth,
        )
        self.image_shape = tuple(first_image.shape)
        for episode in self.episodes:
            for sample in episode["samples"]:
                if len(_flatten(sample, "state")) != self.state_dim or len(_flatten(sample, "action")) != self.action_dim:
                    raise ValueError("state/action dimensions changed across episodes")

    def __len__(self) -> int:
        return len(self.index)

    def __getitem__(self, item: int) -> dict[str, Any]:
        _, torch = _require_ml()
        episode_index, frame_index = self.index[item]
        episode = self.episodes[episode_index]
        samples = episode["samples"]
        sample = samples[frame_index]
        final_action = _flatten(samples[-1], "action")
        actions: list[list[float]] = []
        action_is_pad: list[bool] = []
        for offset in range(self.chunk_size):
            target_index = frame_index + offset
            padded = target_index >= len(samples)
            actions.append(final_action if padded else _flatten(samples[target_index], "action"))
            action_is_pad.append(padded)
        image = _load_rgbd(
            episode["directory"],
            sample,
            depth_scale=self.depth_scale,
            max_depth_m=self.max_depth_m,
            include_depth=self.include_depth,
        )
        if tuple(image.shape) != self.image_shape:
            raise ValueError("camera image shape changed across the dataset")
        return {
            "observation.state": torch.tensor(_flatten(sample, "state"), dtype=torch.float32),
            "observation.rgbd": image,
            "action": torch.tensor(actions, dtype=torch.float32),
            "action_is_pad": torch.tensor(action_is_pad, dtype=torch.bool),
            "episode_id": episode["episode_id"],
            "frame_index": frame_index,
        }

    def normalization(self) -> dict[str, Any]:
        np, torch = _require_ml()
        states = np.asarray(
            [_flatten(episode["samples"][i], "state") for episode in self.episodes for i in range(len(episode["samples"]))],
            dtype=np.float32,
        )
        actions = np.asarray(
            [_flatten(episode["samples"][i], "action") for episode in self.episodes for i in range(len(episode["samples"]))],
            dtype=np.float32,
        )
        return {
            "state_mean": torch.from_numpy(states.mean(axis=0)),
            "state_std": torch.from_numpy(np.maximum(states.std(axis=0), 1e-6)),
            "action_mean": torch.from_numpy(actions.mean(axis=0)),
            "action_std": torch.from_numpy(np.maximum(actions.std(axis=0), 1e-6)),
        }


@dataclass(frozen=True)
class DAPIERACTConfig:
    state_dim: int
    action_dim: int
    chunk_size: int = 16
    image_channels: int = 4
    hidden_dim: int = 128
    latent_dim: int = 16
    n_heads: int = 4
    n_decoder_layers: int = 2
    dropout: float = 0.0
    kl_weight: float = 10.0
    depth_scale: float = 1000.0
    max_depth_m: float = 5.0
    include_depth: bool = True

    def __post_init__(self) -> None:
        values = (self.state_dim, self.action_dim, self.chunk_size, self.image_channels, self.hidden_dim, self.latent_dim)
        if any(value <= 0 for value in values) or self.depth_scale <= 0 or self.max_depth_m <= 0:
            raise ValueError("ACT dimensions must be positive")
        if self.hidden_dim % self.n_heads:
            raise ValueError("hidden_dim must be divisible by n_heads")


def build_model(config: DAPIERACTConfig, normalization: Mapping[str, Any] | None = None):
    """Build the local ACT-like CVAE policy without importing LeRobot."""

    _, torch = _require_ml()
    nn = torch.nn

    class DAPIERNativeACT(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.config = config
            self.image_encoder = nn.Sequential(
                nn.Conv2d(config.image_channels, 16, 5, stride=2, padding=2),
                nn.ReLU(),
                nn.Conv2d(16, 32, 3, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(32, config.hidden_dim, 3, stride=2, padding=1),
                nn.ReLU(),
                nn.AdaptiveAvgPool2d(1),
                nn.Flatten(),
            )
            self.state_projection = nn.Linear(config.state_dim, config.hidden_dim)
            self.image_projection = nn.Linear(config.hidden_dim, config.hidden_dim)
            self.latent_projection = nn.Linear(config.latent_dim, config.hidden_dim)
            self.posterior = nn.Sequential(
                nn.Linear(config.state_dim + config.chunk_size * config.action_dim, config.hidden_dim),
                nn.ReLU(),
                nn.Linear(config.hidden_dim, config.latent_dim * 2),
            )
            layer = nn.TransformerDecoderLayer(
                d_model=config.hidden_dim,
                nhead=config.n_heads,
                dim_feedforward=config.hidden_dim * 4,
                dropout=config.dropout,
                batch_first=True,
                norm_first=True,
            )
            self.decoder = nn.TransformerDecoder(layer, num_layers=config.n_decoder_layers)
            self.queries = nn.Embedding(config.chunk_size, config.hidden_dim)
            self.action_head = nn.Linear(config.hidden_dim, config.action_dim)
            defaults = {
                "state_mean": torch.zeros(config.state_dim),
                "state_std": torch.ones(config.state_dim),
                "action_mean": torch.zeros(config.action_dim),
                "action_std": torch.ones(config.action_dim),
            }
            for name, default in defaults.items():
                self.register_buffer(name, (normalization or {}).get(name, default).float().clone())

        def forward(self, batch: Mapping[str, Any]) -> dict[str, Any]:
            state = (batch["observation.state"] - self.state_mean) / self.state_std
            target = (batch["action"] - self.action_mean) / self.action_std
            pad = batch["action_is_pad"]
            posterior_actions = target.masked_fill(pad.unsqueeze(-1), 0.0)
            mu, logvar = self.posterior(torch.cat((state, posterior_actions.flatten(1)), dim=1)).chunk(2, dim=1)
            latent = mu + torch.randn_like(mu) * torch.exp(0.5 * logvar) if self.training else mu
            predicted = self._decode(state, batch["observation.rgbd"], latent)
            valid = (~pad).unsqueeze(-1).expand_as(predicted)
            l1 = (predicted - target).abs().masked_select(valid).mean()
            kl = -0.5 * (1.0 + logvar - mu.square() - logvar.exp()).sum(dim=1).mean()
            return {"loss": l1 + config.kl_weight * kl, "l1": l1, "kl": kl, "action_normalized": predicted}

        def _decode(self, state: Any, image: Any, latent: Any):
            memory = torch.stack(
                (
                    self.state_projection(state),
                    self.image_projection(self.image_encoder(image)),
                    self.latent_projection(latent),
                ),
                dim=1,
            )
            queries = self.queries.weight.unsqueeze(0).expand(state.shape[0], -1, -1)
            return self.action_head(self.decoder(queries, memory))

        def predict(self, state: Any, image: Any):
            normalized_state = (state - self.state_mean) / self.state_std
            latent = torch.zeros(state.shape[0], config.latent_dim, device=state.device, dtype=state.dtype)
            normalized_action = self._decode(normalized_state, image, latent)
            return normalized_action * self.action_std + self.action_mean

    return DAPIERNativeACT()


class ActionChunkQueue:
    """Execute a bounded prefix and discard stale chunks on reset."""

    def __init__(self, n_action_steps: int) -> None:
        if n_action_steps <= 0:
            raise ValueError("n_action_steps must be positive")
        self.n_action_steps = n_action_steps
        self._pending: list[Any] = []

    def load(self, chunk: Sequence[Any]) -> None:
        if not chunk:
            raise ValueError("action chunk must not be empty")
        self._pending = list(chunk[: self.n_action_steps])

    def pop(self) -> Any:
        if not self._pending:
            raise RuntimeError("no pending action; request a fresh policy chunk")
        return self._pending.pop(0)

    def reset(self) -> None:
        self._pending.clear()

    def __len__(self) -> int:
        return len(self._pending)


def save_checkpoint(path: str | Path, model: Any, optimizer: Any, *, config: DAPIERACTConfig, step: int) -> Path:
    _, torch = _require_ml()
    destination = Path(path)
    if destination.exists():
        raise ValueError(f"checkpoint already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "schema_version": RUNTIME_SCHEMA_VERSION,
            "config": asdict(config),
            "step": int(step),
            "model_state": model.state_dict(),
            "optimizer_state": optimizer.state_dict(),
        },
        destination,
    )
    return destination


def load_checkpoint(path: str | Path, *, device: str = "cpu", with_optimizer: bool = False):
    _, torch = _require_ml()
    try:
        payload = torch.load(Path(path), map_location=device, weights_only=True)
    except TypeError:  # PyTorch < 2.0 compatibility on the education PC.
        payload = torch.load(Path(path), map_location=device)
    if payload.get("schema_version") != RUNTIME_SCHEMA_VERSION:
        raise ValueError("unsupported DAPIER-native ACT checkpoint")
    config = DAPIERACTConfig(**payload["config"])
    model = build_model(config).to(device)
    model.load_state_dict(payload["model_state"])
    optimizer = None
    if with_optimizer:
        optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
        optimizer.load_state_dict(payload["optimizer_state"])
    return model, optimizer, config, int(payload["step"])


def train(
    dataset_root: str | Path,
    checkpoint_path: str | Path,
    *,
    chunk_size: int = 16,
    batch_size: int = 8,
    max_steps: int = 100,
    learning_rate: float = 1e-4,
    device: str = "cpu",
    seed: int = 0,
    depth_scale: float = 1000.0,
    max_depth_m: float = 5.0,
    include_depth: bool = True,
) -> dict[str, Any]:
    if max_steps <= 0 or batch_size <= 0 or learning_rate <= 0:
        raise ValueError("max_steps, batch_size, and learning_rate must be positive")
    _, torch = _require_ml()
    lerobot_loaded_before = "lerobot" in sys.modules
    random.seed(seed)
    torch.manual_seed(seed)
    dataset = DAPIERACTDataset(
        dataset_root,
        chunk_size=chunk_size,
        depth_scale=depth_scale,
        max_depth_m=max_depth_m,
        include_depth=include_depth,
    )
    config = DAPIERACTConfig(
        state_dim=dataset.state_dim,
        action_dim=dataset.action_dim,
        chunk_size=chunk_size,
        image_channels=dataset.image_shape[0],
        depth_scale=depth_scale,
        max_depth_m=max_depth_m,
        include_depth=include_depth,
    )
    model = build_model(config, dataset.normalization()).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=learning_rate)
    loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=0)
    losses: list[float] = []
    iterator = iter(loader)
    model.train()
    for _ in range(max_steps):
        try:
            batch = next(iterator)
        except StopIteration:
            iterator = iter(loader)
            batch = next(iterator)
        batch = {key: value.to(device) if hasattr(value, "to") else value for key, value in batch.items()}
        optimizer.zero_grad(set_to_none=True)
        result = model(batch)
        result["loss"].backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        losses.append(float(result["loss"].detach().cpu()))
    save_checkpoint(checkpoint_path, model, optimizer, config=config, step=max_steps)
    model.eval()
    restored, _, _, restored_step = load_checkpoint(checkpoint_path, device=device)
    restored.eval()
    with torch.no_grad():
        expected = model.predict(batch["observation.state"], batch["observation.rgbd"])
        actual = restored.predict(batch["observation.state"], batch["observation.rgbd"])
    checkpoint_roundtrip = restored_step == max_steps and torch.equal(expected, actual)
    if not checkpoint_roundtrip:
        raise RuntimeError("checkpoint round-trip changed deterministic inference")
    return {
        "status": "PASS",
        "lerobot_loaded_by_runtime": not lerobot_loaded_before and "lerobot" in sys.modules,
        "samples": len(dataset),
        "steps": max_steps,
        "first_loss": losses[0],
        "final_loss": losses[-1],
        "checkpoint_roundtrip": checkpoint_roundtrip,
        "checkpoint": str(Path(checkpoint_path).resolve()),
    }


def infer(
    dataset_root: str | Path,
    checkpoint_path: str | Path,
    *,
    item: int = 0,
    device: str = "cpu",
    split: str = "train",
) -> dict[str, Any]:
    _, torch = _require_ml()
    model, _, config, step = load_checkpoint(checkpoint_path, device=device)
    dataset = DAPIERACTDataset(
        dataset_root,
        chunk_size=config.chunk_size,
        split=split,
        depth_scale=config.depth_scale,
        max_depth_m=config.max_depth_m,
        include_depth=config.include_depth,
    )
    if dataset.state_dim != config.state_dim or dataset.action_dim != config.action_dim:
        raise ValueError("checkpoint and dataset state/action dimensions differ")
    sample = dataset[item]
    state = sample["observation.state"].unsqueeze(0).to(device)
    image = sample["observation.rgbd"].unsqueeze(0).to(device)
    model.eval()
    with torch.no_grad():
        action_chunk = model.predict(state, image)[0].cpu().tolist()
    return {
        "status": "PASS",
        "checkpoint_step": step,
        "episode_id": sample["episode_id"],
        "frame_index": sample["frame_index"],
        "action_chunk": action_chunk,
        "control_authorized": False,
    }


def run_smoke(output_root: str | Path) -> dict[str, Any]:
    _, torch = _require_ml()
    root = Path(output_root).resolve()
    if root.exists() and any(root.iterdir()):
        raise ValueError(f"smoke output must be absent or empty: {root}")
    root.mkdir(parents=True, exist_ok=True)
    raw = root / "raw"
    for index in range(2):
        generate_episode(
            raw / f"episode_{index + 1:06d}",
            sample_count=3,
            seed=700 + index,
            include_camera_payload=True,
            camera_width=32,
            camera_height=32,
        )
    dataset = DAPIERACTDataset(raw, chunk_size=3)
    rgb_only_dataset = DAPIERACTDataset(raw, chunk_size=3, include_depth=False)
    if dataset.image_shape[0] != 4 or rgb_only_dataset.image_shape[0] != 3:
        raise AssertionError("RGB-D/RGB-only channel selection is invalid")
    masks = [dataset[index]["action_is_pad"].tolist() for index in range(len(dataset))]
    expected = [[False, False, False], [False, False, True], [False, True, True]] * 2
    if masks != expected:
        raise AssertionError(f"wrong tail padding masks: {masks}")
    checkpoint = root / "dapier_native_act_smoke.pt"
    train_report = train(raw, checkpoint, chunk_size=3, batch_size=2, max_steps=1, seed=0)
    batch = next(iter(torch.utils.data.DataLoader(dataset, batch_size=1, shuffle=False, num_workers=0)))
    model, _, config, step = load_checkpoint(checkpoint)
    model.eval()
    with torch.no_grad():
        predicted = model.predict(batch["observation.state"], batch["observation.rgbd"])
    if tuple(predicted.shape) != (1, 3, dataset.action_dim) or not torch.isfinite(predicted).all():
        raise AssertionError("checkpoint inference produced an invalid action chunk")
    queue = ActionChunkQueue(n_action_steps=2)
    queue.load(predicted[0].tolist())
    queue.pop()
    queue.reset()
    try:
        queue.pop()
    except RuntimeError:
        stale_chunk_blocked = True
    else:
        stale_chunk_blocked = False
    if not stale_chunk_blocked:
        raise AssertionError("action queue reset did not discard stale actions")
    receipt = {
        "schema_version": RUNTIME_SCHEMA_VERSION,
        "created_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "status": "PASS",
        "scope": "independent dataset-window, RGB-D decode, one optimizer step, checkpoint reload, inference, queue reset",
        "official_act_checkpoint_compatible": False,
        "lerobot_required": False,
        "lerobot_loaded_by_runtime": train_report["lerobot_loaded_by_runtime"],
        "checkpoint_roundtrip": train_report["checkpoint_roundtrip"],
        "dataset": {
            "episodes": 2,
            "frames": len(dataset),
            "state_dim": dataset.state_dim,
            "action_dim": dataset.action_dim,
            "rgbd_channels": dataset.image_shape[0],
            "rgb_only_channels": rgb_only_dataset.image_shape[0],
        },
        "tail_masks": masks,
        "checkpoint": {"path": checkpoint.name, "step": step, "chunk_size": config.chunk_size},
        "inference_shape": list(predicted.shape),
        "stale_chunk_blocked": stale_chunk_blocked,
    }
    (root / "dapier_native_act_smoke_receipt.json").write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="LeRobot-independent DAPIER ACT runtime")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("status")
    smoke_parser = subparsers.add_parser("smoke")
    smoke_parser.add_argument("--output", required=True)
    train_parser = subparsers.add_parser("train")
    train_parser.add_argument("--root", required=True)
    train_parser.add_argument("--checkpoint", required=True)
    train_parser.add_argument("--chunk-size", type=int, default=16)
    train_parser.add_argument("--batch-size", type=int, default=8)
    train_parser.add_argument("--max-steps", type=int, default=100)
    train_parser.add_argument("--device", default="cpu")
    train_parser.add_argument("--depth-scale", type=float, default=1000.0)
    train_parser.add_argument("--max-depth-m", type=float, default=5.0)
    train_parser.add_argument("--rgb-only", action="store_true")
    infer_parser = subparsers.add_parser("infer")
    infer_parser.add_argument("--root", required=True)
    infer_parser.add_argument("--checkpoint", required=True)
    infer_parser.add_argument("--item", type=int, default=0)
    infer_parser.add_argument("--device", default="cpu")
    infer_parser.add_argument("--split", choices=("train", "validation", "test"), default="train")
    rollout_parser = subparsers.add_parser("rollout-smoke")
    rollout_parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    if args.command == "status":
        result = dependency_status()
    elif args.command == "smoke":
        result = run_smoke(args.output)
    elif args.command == "train":
        result = train(
            args.root,
            args.checkpoint,
            chunk_size=args.chunk_size,
            batch_size=args.batch_size,
            max_steps=args.max_steps,
            device=args.device,
            depth_scale=args.depth_scale,
            max_depth_m=args.max_depth_m,
            include_depth=not args.rgb_only,
        )
    elif args.command == "rollout-smoke":
        from shoe_sorting_data.native_act_rollout import run_native_act_rollout_smoke

        result = run_native_act_rollout_smoke(args.output)
    else:
        result = infer(args.root, args.checkpoint, item=args.item, device=args.device, split=args.split)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
