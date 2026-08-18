"""Shared model, data, plotting, and evaluation code for Projects 1-5.

Each assignment has its own executable file. This module only keeps the shared
contract needed to make the five independently runnable exercises comparable.
"""

from __future__ import annotations

import json
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import torch
import torch.nn.functional as F
from sklearn.metrics import classification_report, confusion_matrix
from torch import nn
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from torchvision import datasets, transforms

CLASSES = (
    "plane",
    "car",
    "bird",
    "cat",
    "deer",
    "dog",
    "frog",
    "horse",
    "ship",
    "truck",
)
MEAN = (0.5, 0.5, 0.5)
STD = (0.5, 0.5, 0.5)


@dataclass(frozen=True)
class RunConfig:
    """One controlled ablation condition."""

    name: str
    use_bn: bool
    use_dropout: bool


RUN_CONFIGS = (
    RunConfig("baseline_bn_dropout", use_bn=True, use_dropout=True),
    RunConfig("no_bn", use_bn=False, use_dropout=True),
    RunConfig("no_dropout", use_bn=True, use_dropout=False),
)


def resolve_device(requested: str) -> torch.device:
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    device = torch.device(requested)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested, but torch.cuda.is_available() is false")
    return device


def set_seed(seed: int) -> None:
    """Reset every RNG used by this workbook before each fair comparison."""

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.use_deterministic_algorithms(True, warn_only=True)


def seed_worker(_worker_id: int) -> None:
    worker_seed = torch.initial_seed() % (2**32)
    np.random.seed(worker_seed)
    random.seed(worker_seed)


def train_transform() -> transforms.Compose:
    return transforms.Compose(
        [
            transforms.RandomCrop(32, padding=4),
            transforms.RandomHorizontalFlip(),
            transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2),
            transforms.ToTensor(),
            transforms.Normalize(MEAN, STD),
        ]
    )


def test_transform() -> transforms.Compose:
    return transforms.Compose(
        [
            transforms.ToTensor(),
            transforms.Normalize(MEAN, STD),
        ]
    )


def make_loaders(
    data_dir: Path,
    batch_size: int,
    num_workers: int,
    seed: int,
    *,
    download: bool = True,
) -> tuple[DataLoader, DataLoader]:
    """Create fresh, identically seeded loaders for every ablation run."""

    data_dir.mkdir(parents=True, exist_ok=True)
    train_set = datasets.CIFAR10(
        root=data_dir,
        train=True,
        download=download,
        transform=train_transform(),
    )
    test_set = datasets.CIFAR10(
        root=data_dir,
        train=False,
        download=download,
        transform=test_transform(),
    )
    generator = torch.Generator().manual_seed(seed)
    common = {
        "batch_size": batch_size,
        "num_workers": num_workers,
        "worker_init_fn": seed_worker if num_workers else None,
        "generator": generator,
        "pin_memory": torch.cuda.is_available(),
        "persistent_workers": num_workers > 0,
    }
    train_loader = DataLoader(train_set, shuffle=True, **common)

    test_generator = torch.Generator().manual_seed(seed)
    test_common = dict(common)
    test_common["generator"] = test_generator
    test_loader = DataLoader(test_set, shuffle=False, **test_common)
    return train_loader, test_loader


class CIFARNet(nn.Module):
    """Four-convolution CIFAR-10 CNN with switchable BN and Dropout."""

    def __init__(self, use_bn: bool = True, use_dropout: bool = True) -> None:
        super().__init__()
        self.use_bn = use_bn
        self.use_dropout = use_dropout

        self.conv1 = nn.Conv2d(3, 32, 3, padding=1)
        self.bn1 = nn.BatchNorm2d(32) if use_bn else nn.Identity()
        self.relu1 = nn.ReLU()
        self.conv2 = nn.Conv2d(32, 32, 3, padding=1)
        self.bn2 = nn.BatchNorm2d(32) if use_bn else nn.Identity()
        self.relu2 = nn.ReLU()

        self.conv3 = nn.Conv2d(32, 64, 3, padding=1)
        self.bn3 = nn.BatchNorm2d(64) if use_bn else nn.Identity()
        self.relu3 = nn.ReLU()
        self.conv4 = nn.Conv2d(64, 64, 3, padding=1)
        self.bn4 = nn.BatchNorm2d(64) if use_bn else nn.Identity()
        self.relu4 = nn.ReLU()

        self.pool = nn.MaxPool2d(2, 2)
        self.dropout_conv = nn.Dropout(0.3) if use_dropout else nn.Identity()
        self.fc1 = nn.Linear(64 * 8 * 8, 512)
        self.bn5 = nn.BatchNorm1d(512) if use_bn else nn.Identity()
        self.relu5 = nn.ReLU()
        self.dropout_fc = nn.Dropout(0.5) if use_dropout else nn.Identity()
        self.fc2 = nn.Linear(512, 10)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.relu1(self.bn1(self.conv1(x)))
        x = self.relu2(self.bn2(self.conv2(x)))
        x = self.dropout_conv(self.pool(x))

        x = self.relu3(self.bn3(self.conv3(x)))
        x = self.relu4(self.bn4(self.conv4(x)))
        x = self.dropout_conv(self.pool(x))

        x = torch.flatten(x, 1)
        x = self.relu5(self.bn5(self.fc1(x)))
        x = self.dropout_fc(x)
        return self.fc2(x)


def denormalize(tensor: torch.Tensor) -> torch.Tensor:
    mean = torch.tensor(MEAN, dtype=tensor.dtype, device=tensor.device).view(-1, 1, 1)
    std = torch.tensor(STD, dtype=tensor.dtype, device=tensor.device).view(-1, 1, 1)
    return tensor * std + mean


def tensor_stats(tensor: torch.Tensor) -> dict[str, float | int]:
    return {
        "min": float(tensor.min()),
        "max": float(tensor.max()),
        "mean": float(tensor.mean()),
        "std": float(tensor.std()),
        "values_outside_display_range": int(((tensor < 0) | (tensor > 1)).sum()),
    }


def prepare_output_dirs(output_dir: Path) -> dict[str, Path]:
    paths = {
        "figures": output_dir / "figures",
        "metrics": output_dir / "metrics",
        "reports": output_dir / "reports",
        "checkpoints": output_dir / "checkpoints",
        "tensorboard": output_dir / "tensorboard",
    }
    for path in paths.values():
        path.mkdir(parents=True, exist_ok=True)
    return paths


def public_path(path: Path) -> str:
    """Prefer a repository-relative artifact path in shareable reports."""

    project_dir = Path(__file__).resolve().parent
    try:
        return path.resolve().relative_to(project_dir).as_posix()
    except ValueError:
        return path.name


def plot_image_batch(
    images: torch.Tensor,
    labels: torch.Tensor,
    output_path: Path,
    title: str,
) -> None:
    figure, axes = plt.subplots(2, 4, figsize=(11, 6))
    for axis, image, label in zip(axes.flat, images[:8], labels[:8], strict=True):
        visible = denormalize(image).clamp(0, 1).permute(1, 2, 0).cpu().numpy()
        axis.imshow(visible)
        axis.set_title(CLASSES[int(label)])
        axis.axis("off")
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def run_p1(
    data_dir: Path,
    paths: dict[str, Path],
    batch_size: int,
    num_workers: int,
    seed: int,
) -> dict[str, Any]:
    """Project 1: visualize augmentation, normalization, and test leakage boundary."""

    set_seed(seed)
    train_loader, test_loader = make_loaders(data_dir, batch_size, num_workers, seed)
    train_images, train_labels = next(iter(train_loader))
    test_images, test_labels = next(iter(test_loader))

    plot_image_batch(
        train_images,
        train_labels,
        paths["figures"] / "p1_train_augmented_grid.png",
        "P1 Train: crop + flip + color jitter",
    )
    plot_image_batch(
        test_images,
        test_labels,
        paths["figures"] / "p1_test_clean_grid.png",
        "P1 Test: augmentation off",
    )

    figure, axes = plt.subplots(2, 8, figsize=(16, 5))
    for column in range(8):
        for row, images, labels, row_name in (
            (0, train_images, train_labels, "train/aug"),
            (1, test_images, test_labels, "test/clean"),
        ):
            visible = denormalize(images[column]).clamp(0, 1).permute(1, 2, 0).numpy()
            axes[row, column].imshow(visible)
            axes[row, column].set_title(f"{row_name}\n{CLASSES[int(labels[column])]}")
            axes[row, column].axis("off")
    figure.suptitle("P1 Augmentation boundary: train ON, test OFF")
    figure.tight_layout()
    figure.savefig(paths["figures"] / "p1_train_vs_test.png", dpi=160)
    plt.close(figure)

    sample = train_images[0]
    figure, axes = plt.subplots(1, 2, figsize=(8, 4))
    axes[0].imshow(sample.permute(1, 2, 0).numpy())
    axes[0].set_title("Normalized tensor shown directly")
    axes[1].imshow(denormalize(sample).clamp(0, 1).permute(1, 2, 0).numpy())
    axes[1].set_title("After x * 0.5 + 0.5")
    for axis in axes:
        axis.axis("off")
    figure.tight_layout()
    figure.savefig(paths["figures"] / "p1_normalized_vs_denormalized.png", dpi=160)
    plt.close(figure)

    report = {
        "train_normalized": tensor_stats(train_images),
        "test_normalized": tensor_stats(test_images),
        "train_denormalized": tensor_stats(denormalize(train_images)),
        "test_denormalized": tensor_stats(denormalize(test_images)),
        "observation": (
            "Normalize(0.5, 0.5) maps [0,1] pixels to approximately [-1,1]. "
            "Only the train pipeline contains stochastic crop, flip, and color jitter."
        ),
    }
    report_path = paths["reports"] / "p1_batch_statistics.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"P1 complete: {report_path}")
    return report


def train_one_epoch(
    model: nn.Module,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    criterion: nn.Module,
    device: torch.device,
) -> tuple[float, float, dict[str, float]]:
    model.train()
    loss_sum = 0.0
    correct = 0
    sample_count = 0
    gradient_norms = {"conv1": 0.0, "conv4": 0.0}

    for images, labels in loader:
        images = images.to(device, non_blocking=True)
        labels = labels.to(device, non_blocking=True)
        optimizer.zero_grad(set_to_none=True)
        logits = model(images)
        loss = criterion(logits, labels)
        loss.backward()

        for name in gradient_norms:
            layer = getattr(model, name)
            if layer.weight.grad is not None:
                gradient_norms[name] = float(layer.weight.grad.norm().detach().cpu())

        optimizer.step()
        batch_size = labels.size(0)
        loss_sum += float(loss.detach()) * batch_size
        correct += int((logits.argmax(dim=1) == labels).sum())
        sample_count += batch_size

    return loss_sum / sample_count, correct / sample_count, gradient_norms


@torch.inference_mode()
def evaluate(
    model: nn.Module,
    loader: DataLoader,
    criterion: nn.Module,
    device: torch.device,
) -> tuple[float, float]:
    model.eval()
    loss_sum = 0.0
    correct = 0
    sample_count = 0
    for images, labels in loader:
        images = images.to(device, non_blocking=True)
        labels = labels.to(device, non_blocking=True)
        logits = model(images)
        loss = criterion(logits, labels)
        batch_size = labels.size(0)
        loss_sum += float(loss) * batch_size
        correct += int((logits.argmax(dim=1) == labels).sum())
        sample_count += batch_size
    return loss_sum / sample_count, correct / sample_count


def find_overfit_onset(metrics: pd.DataFrame) -> int | None:
    """Return the first epoch where test loss rises twice while train loss falls."""

    for index in range(2, len(metrics)):
        test_rising = (
            metrics.loc[index, "test_loss"] > metrics.loc[index - 1, "test_loss"]
            and metrics.loc[index - 1, "test_loss"]
            > metrics.loc[index - 2, "test_loss"]
        )
        train_falling = (
            metrics.loc[index, "train_loss"]
            < metrics.loc[index - 1, "train_loss"]
            < metrics.loc[index - 2, "train_loss"]
        )
        if test_rising and train_falling:
            return int(metrics.loc[index - 1, "epoch"])
    return None


def train_one_run(
    config: RunConfig,
    data_dir: Path,
    paths: dict[str, Path],
    epochs: int,
    batch_size: int,
    num_workers: int,
    seed: int,
    device: torch.device,
) -> dict[str, Any]:
    """Train one ablation condition and emit CSV, checkpoint, and TensorBoard data."""

    set_seed(seed)
    train_loader, test_loader = make_loaders(data_dir, batch_size, num_workers, seed)
    model = CIFARNet(config.use_bn, config.use_dropout).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    writer = SummaryWriter(log_dir=str(paths["tensorboard"] / config.name))

    if config.name == "baseline_bn_dropout":
        was_training = model.training
        model.eval()
        writer.add_graph(model, torch.zeros(2, 3, 32, 32, device=device))
        model.train(was_training)

    rows: list[dict[str, float | int]] = []
    best_test_loss = float("inf")
    best_epoch = 0
    checkpoint_path = paths["checkpoints"] / f"{config.name}_best.pt"

    for epoch in range(1, epochs + 1):
        train_loss, train_acc, gradient_norms = train_one_epoch(
            model, train_loader, optimizer, criterion, device
        )
        test_loss, test_acc = evaluate(model, test_loader, criterion, device)
        row = {
            "epoch": epoch,
            "train_loss": train_loss,
            "train_acc": train_acc,
            "test_loss": test_loss,
            "test_acc": test_acc,
            "lr": optimizer.param_groups[0]["lr"],
            "grad_norm_conv1": gradient_norms["conv1"],
            "grad_norm_conv4": gradient_norms["conv4"],
        }
        rows.append(row)

        writer.add_scalar("loss/train", train_loss, epoch)
        writer.add_scalar("loss/test", test_loss, epoch)
        writer.add_scalar("accuracy/train", train_acc, epoch)
        writer.add_scalar("accuracy/test", test_acc, epoch)
        writer.add_scalar("lr", row["lr"], epoch)
        writer.add_scalar("gradients/conv1_norm", gradient_norms["conv1"], epoch)
        writer.add_scalar("gradients/conv4_norm", gradient_norms["conv4"], epoch)
        if epoch == 1 or epoch % 5 == 0:
            writer.add_histogram("weights/conv1", model.conv1.weight, epoch)
            writer.add_histogram("weights/conv4", model.conv4.weight, epoch)

        if test_loss < best_test_loss:
            best_test_loss = test_loss
            best_epoch = epoch
            torch.save(
                {
                    "model_state": model.state_dict(),
                    "config": asdict(config),
                    "epoch": epoch,
                    "test_loss": test_loss,
                    "test_acc": test_acc,
                },
                checkpoint_path,
            )

        print(
            f"{config.name:22s} epoch {epoch:02d}/{epochs} "
            f"train={train_loss:.4f}/{train_acc:.3f} "
            f"test={test_loss:.4f}/{test_acc:.3f}"
        )

    writer.close()
    metrics = pd.DataFrame(rows)
    metrics_path = paths["metrics"] / f"p3_{config.name}.csv"
    metrics.to_csv(metrics_path, index=False)
    best_row = metrics.loc[metrics["test_loss"].idxmin()]
    return {
        "name": config.name,
        "use_bn": config.use_bn,
        "use_dropout": config.use_dropout,
        "best_epoch": best_epoch,
        "best_test_loss": best_test_loss,
        "best_test_acc": float(best_row["test_acc"]),
        "final_train_acc": float(metrics.iloc[-1]["train_acc"]),
        "final_test_acc": float(metrics.iloc[-1]["test_acc"]),
        "overfit_onset_epoch": find_overfit_onset(metrics),
        "metrics_csv": public_path(metrics_path),
        "checkpoint": public_path(checkpoint_path),
    }


def plot_ablation(paths: dict[str, Path], summaries: list[dict[str, Any]]) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(14, 10))
    for summary in summaries:
        metrics = pd.read_csv(paths["metrics"] / f"p3_{summary['name']}.csv")
        label = summary["name"]
        axes[0, 0].plot(metrics["epoch"], metrics["train_loss"], label=label)
        axes[0, 1].plot(metrics["epoch"], metrics["test_loss"], label=label)
        axes[1, 0].plot(metrics["epoch"], metrics["test_acc"], label=label)
        gap = metrics["train_acc"] - metrics["test_acc"]
        axes[1, 1].plot(metrics["epoch"], gap, label=label)
        axes[0, 1].axvline(summary["best_epoch"], alpha=0.25, linestyle="--")

    titles = (
        "Train loss",
        "Test loss (dashed: each minimum)",
        "Test accuracy",
        "Generalization gap: train acc - test acc",
    )
    for axis, title in zip(axes.flat, titles, strict=True):
        axis.set_title(title)
        axis.set_xlabel("epoch")
        axis.grid(alpha=0.25)
        axis.legend()
    figure.tight_layout()
    figure.savefig(paths["figures"] / "p3_ablation_curves.png", dpi=170)
    plt.close(figure)


def run_p3_p4(
    data_dir: Path,
    paths: dict[str, Path],
    epochs: int,
    batch_size: int,
    num_workers: int,
    seed: int,
    device: torch.device,
) -> list[dict[str, Any]]:
    summaries = [
        train_one_run(
            config,
            data_dir,
            paths,
            epochs,
            batch_size,
            num_workers,
            seed,
            device,
        )
        for config in RUN_CONFIGS
    ]
    plot_ablation(paths, summaries)
    summary_path = paths["reports"] / "p3_p4_ablation_summary.json"
    summary_path.write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    tensorboard_manifest = {
        "command": f"tensorboard --logdir={paths['tensorboard']} --port=6006",
        "runs": [summary["name"] for summary in summaries],
        "tracked": [
            "loss/train",
            "loss/test",
            "accuracy/train",
            "accuracy/test",
            "lr",
            "gradients/conv1_norm",
            "gradients/conv4_norm",
            "weights/conv1",
            "weights/conv4",
        ],
    }
    (paths["reports"] / "p4_tensorboard_manifest.json").write_text(
        json.dumps(tensorboard_manifest, indent=2), encoding="utf-8"
    )
    print(f"P3/P4 complete: {summary_path}")
    return summaries


def load_baseline(paths: dict[str, Path], device: torch.device) -> CIFARNet:
    checkpoint_path = paths["checkpoints"] / "baseline_bn_dropout_best.pt"
    if not checkpoint_path.is_file():
        raise FileNotFoundError(
            f"Missing {checkpoint_path}. Run the 'train' command before P2 or P5."
        )
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=True)
    model = CIFARNet(use_bn=True, use_dropout=True).to(device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()
    return model


def normalize_channel(channel: torch.Tensor) -> torch.Tensor:
    channel = channel.float()
    minimum = channel.min()
    maximum = channel.max()
    if float(maximum - minimum) < 1e-12:
        return torch.zeros_like(channel)
    return (channel - minimum) / (maximum - minimum)


def plot_feature_grid(
    activation: torch.Tensor,
    output_path: Path,
    title: str,
    max_channels: int = 32,
) -> None:
    channel_count = min(max_channels, activation.shape[1])
    rows = 4
    columns = 8
    figure, axes = plt.subplots(rows, columns, figsize=(13, 7))
    for index, axis in enumerate(axes.flat):
        if index < channel_count:
            channel = normalize_channel(activation[0, index]).cpu().numpy()
            axis.imshow(channel, cmap="gray", vmin=0, vmax=1)
            axis.set_title(f"ch {index}", fontsize=8)
        axis.axis("off")
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(output_path, dpi=170)
    plt.close(figure)


def run_p2(
    data_dir: Path,
    paths: dict[str, Path],
    batch_size: int,
    num_workers: int,
    seed: int,
    device: torch.device,
) -> dict[str, Any]:
    """Project 2: capture post-ReLU activations using removable forward hooks."""

    set_seed(seed)
    _, test_loader = make_loaders(
        data_dir, batch_size, num_workers, seed, download=False
    )
    model = load_baseline(paths, device)
    image, label = test_loader.dataset[0]
    activations: dict[str, torch.Tensor] = {}

    def get_hook(name: str):
        def hook(
            _module: nn.Module, _inputs: tuple[torch.Tensor, ...], output: torch.Tensor
        ):
            activations[name] = output.detach().cpu()

        return hook

    handles = []
    for index in range(1, 5):
        layer = getattr(model, f"relu{index}")
        handles.append(layer.register_forward_hook(get_hook(f"conv{index}")))

    try:
        with torch.inference_mode():
            logits = model(image.unsqueeze(0).to(device))
            prediction = int(logits.argmax(dim=1))
    finally:
        for handle in handles:
            handle.remove()

    report: dict[str, Any] = {
        "true_label": CLASSES[label],
        "predicted_label": CLASSES[prediction],
        "layers": {},
        "hook_count_after_remove": sum(
            len(module._forward_hooks) for module in model.modules()
        ),
    }
    for name, activation in activations.items():
        zero_fraction = float((activation <= 1e-8).float().mean())
        report["layers"][name] = {
            "shape": list(activation.shape),
            "zero_fraction_after_relu": zero_fraction,
            "mean": float(activation.mean()),
            "std": float(activation.std()),
        }
        plot_feature_grid(
            activation,
            paths["figures"] / f"p2_{name}_feature_maps.png",
            f"P2 {name} post-ReLU | shape={tuple(activation.shape)}",
        )

    figure, axis = plt.subplots(figsize=(4, 4))
    axis.imshow(denormalize(image).clamp(0, 1).permute(1, 2, 0).numpy())
    axis.set_title(f"true={CLASSES[label]}, pred={CLASSES[prediction]}")
    axis.axis("off")
    figure.tight_layout()
    figure.savefig(paths["figures"] / "p2_source_image.png", dpi=170)
    plt.close(figure)

    report_path = paths["reports"] / "p2_feature_map_summary.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"P2 complete: {report_path}")
    return report


def distinct_confident_errors(
    true_labels: np.ndarray,
    predictions: np.ndarray,
    confidences: np.ndarray,
    count: int = 5,
) -> list[int]:
    wrong = np.flatnonzero(true_labels != predictions)
    ranked = wrong[np.argsort(confidences[wrong])[::-1]]
    selected: list[int] = []
    seen_pairs: set[tuple[int, int]] = set()
    for index in ranked:
        pair = (int(true_labels[index]), int(predictions[index]))
        if pair in seen_pairs:
            continue
        selected.append(int(index))
        seen_pairs.add(pair)
        if len(selected) == count:
            break
    return selected


@torch.inference_mode()
def run_p5(
    data_dir: Path,
    paths: dict[str, Path],
    batch_size: int,
    num_workers: int,
    seed: int,
    device: torch.device,
) -> dict[str, Any]:
    """Project 5: confusion matrix and diverse high-confidence errors."""

    set_seed(seed)
    _, test_loader = make_loaders(
        data_dir, batch_size, num_workers, seed, download=False
    )
    model = load_baseline(paths, device)
    true_chunks: list[torch.Tensor] = []
    prediction_chunks: list[torch.Tensor] = []
    probability_chunks: list[torch.Tensor] = []
    image_chunks: list[torch.Tensor] = []

    for images, labels in test_loader:
        logits = model(images.to(device, non_blocking=True))
        probabilities = F.softmax(logits, dim=1).cpu()
        true_chunks.append(labels)
        prediction_chunks.append(probabilities.argmax(dim=1))
        probability_chunks.append(probabilities)
        image_chunks.append(images)

    true_labels = torch.cat(true_chunks).numpy()
    predictions = torch.cat(prediction_chunks).numpy()
    probabilities = torch.cat(probability_chunks).numpy()
    images = torch.cat(image_chunks)
    confidences = probabilities.max(axis=1)

    matrix = confusion_matrix(true_labels, predictions, labels=np.arange(10))
    normalized = matrix / matrix.sum(axis=1, keepdims=True)
    figure, axis = plt.subplots(figsize=(10, 8))
    sns.heatmap(
        normalized,
        annot=True,
        fmt=".2f",
        xticklabels=CLASSES,
        yticklabels=CLASSES,
        cmap="Blues",
        vmin=0,
        vmax=1,
        ax=axis,
    )
    axis.set_xlabel("Predicted")
    axis.set_ylabel("True")
    axis.set_title("P5 Row-normalized confusion matrix")

    off_diagonal = normalized.copy()
    np.fill_diagonal(off_diagonal, -1)
    top_cells = np.dstack(
        np.unravel_index(np.argsort(off_diagonal.ravel())[::-1][:5], off_diagonal.shape)
    )[0]
    for row, column in top_cells:
        axis.add_patch(
            plt.Rectangle((column, row), 1, 1, fill=False, edgecolor="red", lw=2)
        )
    figure.tight_layout()
    figure.savefig(paths["figures"] / "p5_confusion_matrix.png", dpi=180)
    plt.close(figure)

    selected = distinct_confident_errors(true_labels, predictions, confidences, count=5)
    figure, axes = plt.subplots(len(selected), 2, figsize=(11, 4 * len(selected)))
    error_records = []
    for row, index in enumerate(selected):
        visible = denormalize(images[index]).clamp(0, 1).permute(1, 2, 0).numpy()
        true_index = int(true_labels[index])
        predicted_index = int(predictions[index])
        axes[row, 0].imshow(visible)
        axes[row, 0].set_title(
            f"true={CLASSES[true_index]} → pred={CLASSES[predicted_index]}\n"
            f"confidence={confidences[index]:.3f}"
        )
        axes[row, 0].axis("off")

        top5 = np.argsort(probabilities[index])[::-1][:5]
        axes[row, 1].barh(
            [CLASSES[class_index] for class_index in top5][::-1],
            probabilities[index, top5][::-1],
        )
        axes[row, 1].set_xlim(0, 1)
        axes[row, 1].set_xlabel("softmax probability")
        error_records.append(
            {
                "dataset_index": index,
                "true": CLASSES[true_index],
                "predicted": CLASSES[predicted_index],
                "confidence": float(confidences[index]),
                "top5": [
                    {
                        "class": CLASSES[class_index],
                        "probability": float(probabilities[index, class_index]),
                    }
                    for class_index in top5
                ],
            }
        )
    figure.tight_layout()
    figure.savefig(paths["figures"] / "p5_top5_diverse_confident_errors.png", dpi=170)
    plt.close(figure)

    report = classification_report(
        true_labels,
        predictions,
        labels=np.arange(10),
        target_names=CLASSES,
        output_dict=True,
        zero_division=0,
    )
    pd.DataFrame(report).transpose().to_csv(
        paths["metrics"] / "p5_classification_report.csv"
    )
    pd.DataFrame(matrix, index=CLASSES, columns=CLASSES).to_csv(
        paths["metrics"] / "p5_confusion_counts.csv"
    )
    pd.DataFrame(normalized, index=CLASSES, columns=CLASSES).to_csv(
        paths["metrics"] / "p5_confusion_normalized.csv"
    )

    weakest_index = int(np.argmin(np.diag(normalized)))
    confusions = [
        {
            "true": CLASSES[int(row)],
            "predicted": CLASSES[int(column)],
            "rate": float(normalized[row, column]),
            "count": int(matrix[row, column]),
        }
        for row, column in top_cells
    ]
    diagnosis = {
        "accuracy": float((true_labels == predictions).mean()),
        "weakest_recall_class": CLASSES[weakest_index],
        "weakest_recall": float(normalized[weakest_index, weakest_index]),
        "top_off_diagonal_confusions": confusions,
        "diverse_confident_errors": error_records,
    }
    diagnosis_path = paths["reports"] / "p5_diagnosis.json"
    diagnosis_path.write_text(json.dumps(diagnosis, indent=2), encoding="utf-8")
    print(f"P5 complete: {diagnosis_path}")
    return diagnosis
