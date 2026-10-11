"""작은 3-class 이미지 데이터셋을 위한 안정화 MiniVGG 개인화 학습."""

# ruff: noqa: I001 -- conda 재실행이 PyTorch import보다 먼저여야 한다.

import argparse
import json
import os
import random
import sys
from pathlib import Path


CONDA_PYTHON = Path.home() / "miniconda3/envs/lerobot-vision/bin/python"
if CONDA_PYTHON.exists() and Path(sys.executable).resolve() != CONDA_PYTHON.resolve():
    os.execv(str(CONDA_PYTHON), [str(CONDA_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])

import matplotlib.pyplot as plt
import numpy as np
import torch
from torch import nn, optim
from torch.nn.utils import clip_grad_norm_
from torch.utils.data import DataLoader, Subset
from torchsummary import summary
from torchvision import datasets

from minivgg_common import (
    IMAGE_SIZE,
    MODEL_VERSION,
    NORMALIZE_MEAN,
    NORMALIZE_STD,
    MiniVGGNet,
    build_eval_transform,
    build_train_transform,
)


SCRIPT_DIR = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=SCRIPT_DIR / "images")
    parser.add_argument("--output", type=Path, default=SCRIPT_DIR / "miniVGGnet_stable.pth")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--val-ratio", type=float, default=0.20)
    parser.add_argument("--split-seed", type=int, default=2026)
    parser.add_argument(
        "--seeds",
        type=int,
        nargs="+",
        default=[42],
        help="학습 seed 목록. 안정성 비교: --seeds 42 43 44",
    )
    parser.add_argument("--patience", type=int, default=8)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.use_deterministic_algorithms(True, warn_only=True)


def stratified_indices(targets: list[int], val_ratio: float, seed: int) -> tuple[list[int], list[int]]:
    if not 0.0 < val_ratio < 0.5:
        raise ValueError("--val-ratio는 0보다 크고 0.5보다 작아야 합니다.")

    rng = random.Random(seed)
    by_class: dict[int, list[int]] = {}
    for index, target in enumerate(targets):
        by_class.setdefault(target, []).append(index)

    train_indices: list[int] = []
    val_indices: list[int] = []
    for class_indices in by_class.values():
        rng.shuffle(class_indices)
        val_count = max(1, round(len(class_indices) * val_ratio))
        val_indices.extend(class_indices[:val_count])
        train_indices.extend(class_indices[val_count:])

    rng.shuffle(train_indices)
    rng.shuffle(val_indices)
    return train_indices, val_indices


def make_loaders(
    data_path: Path,
    batch_size: int,
    val_ratio: float,
    split_seed: int,
    train_seed: int,
) -> tuple[DataLoader, DataLoader, list[str], torch.Tensor]:
    index_dataset = datasets.ImageFolder(data_path)
    train_indices, val_indices = stratified_indices(index_dataset.targets, val_ratio, split_seed)

    train_dataset = datasets.ImageFolder(data_path, transform=build_train_transform())
    val_dataset = datasets.ImageFolder(data_path, transform=build_eval_transform())
    train_subset = Subset(train_dataset, train_indices)
    val_subset = Subset(val_dataset, val_indices)

    generator = torch.Generator().manual_seed(train_seed)
    train_loader = DataLoader(
        train_subset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=0,
        generator=generator,
        pin_memory=torch.cuda.is_available(),
    )
    val_loader = DataLoader(
        val_subset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=0,
        pin_memory=torch.cuda.is_available(),
    )

    class_counts = torch.bincount(
        torch.tensor([index_dataset.targets[index] for index in train_indices]),
        minlength=len(index_dataset.classes),
    ).float()
    class_weights = class_counts.sum() / (len(class_counts) * class_counts)
    print(
        f"split: train={len(train_subset)} validation={len(val_subset)} "
        f"classes={dict(zip(index_dataset.classes, class_counts.int().tolist(), strict=True))}"
    )
    return train_loader, val_loader, index_dataset.classes, class_weights


def evaluate(
    model: nn.Module,
    loader: DataLoader,
    criterion: nn.Module,
    device: torch.device,
    num_classes: int,
) -> dict[str, float | list[float]]:
    model.eval()
    loss_sum = 0.0
    correct = 0
    total = 0
    class_correct = torch.zeros(num_classes, dtype=torch.long)
    class_total = torch.zeros(num_classes, dtype=torch.long)

    with torch.inference_mode():
        for images, labels in loader:
            images = images.to(device, non_blocking=True)
            labels = labels.to(device, non_blocking=True)
            outputs = model(images)
            loss_sum += criterion(outputs, labels).item() * labels.size(0)
            predictions = outputs.argmax(dim=1)
            correct += predictions.eq(labels).sum().item()
            total += labels.size(0)
            for class_index in range(num_classes):
                mask = labels == class_index
                class_total[class_index] += mask.sum().cpu()
                class_correct[class_index] += predictions[mask].eq(labels[mask]).sum().cpu()

    recalls = (class_correct.float() / class_total.clamp_min(1)).tolist()
    return {
        "loss": loss_sum / total,
        "accuracy": correct / total,
        "macro_recall": float(np.mean(recalls)),
        "class_recall": recalls,
    }


def train_one_seed(args: argparse.Namespace, seed: int, device: torch.device) -> dict:
    set_seed(seed)
    train_loader, val_loader, class_names, class_weights = make_loaders(
        args.data,
        args.batch_size,
        args.val_ratio,
        args.split_seed,
        seed,
    )
    model = MiniVGGNet(num_classes=len(class_names)).to(device)
    criterion = nn.CrossEntropyLoss(weight=class_weights.to(device), label_smoothing=0.05)
    optimizer = optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer,
        mode="min",
        factor=0.5,
        patience=3,
        min_lr=1e-6,
    )

    history = {"train_loss": [], "train_acc": [], "val_loss": [], "val_acc": [], "lr": []}
    best_loss = float("inf")
    best_epoch = 0
    best_state = None
    best_metrics = None
    epochs_without_improvement = 0

    print(f"\nseed={seed} device={device}")
    for epoch in range(1, args.epochs + 1):
        model.train()
        loss_sum = 0.0
        correct = 0
        total = 0

        for images, labels in train_loader:
            images = images.to(device, non_blocking=True)
            labels = labels.to(device, non_blocking=True)
            optimizer.zero_grad(set_to_none=True)
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            loss_sum += loss.item() * labels.size(0)
            correct += outputs.argmax(dim=1).eq(labels).sum().item()
            total += labels.size(0)

        train_loss = loss_sum / total
        train_acc = correct / total
        val_metrics = evaluate(model, val_loader, criterion, device, len(class_names))
        scheduler.step(float(val_metrics["loss"]))
        current_lr = optimizer.param_groups[0]["lr"]

        history["train_loss"].append(train_loss)
        history["train_acc"].append(train_acc)
        history["val_loss"].append(val_metrics["loss"])
        history["val_acc"].append(val_metrics["accuracy"])
        history["lr"].append(current_lr)

        improved = float(val_metrics["loss"]) < best_loss - 1e-4
        if improved:
            best_loss = float(val_metrics["loss"])
            best_epoch = epoch
            best_metrics = val_metrics
            best_state = {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}
            epochs_without_improvement = 0
        else:
            epochs_without_improvement += 1

        print(
            f"epoch={epoch:02d} train_loss={train_loss:.4f} train_acc={train_acc * 100:5.1f}% "
            f"val_loss={float(val_metrics['loss']):.4f} val_acc={float(val_metrics['accuracy']) * 100:5.1f}% "
            f"macro_recall={float(val_metrics['macro_recall']) * 100:5.1f}% lr={current_lr:.1e}"
        )
        if epochs_without_improvement >= args.patience:
            print(f"early stopping: {args.patience} epochs 동안 validation loss 개선 없음")
            break

    assert best_state is not None and best_metrics is not None
    return {
        "seed": seed,
        "best_epoch": best_epoch,
        "best_val_loss": best_loss,
        "best_val_accuracy": best_metrics["accuracy"],
        "best_macro_recall": best_metrics["macro_recall"],
        "best_class_recall": best_metrics["class_recall"],
        "class_names": class_names,
        "state_dict": best_state,
        "history": history,
    }


def save_plot(run: dict, path: Path, show: bool) -> None:
    history = run["history"]
    epochs = range(1, len(history["train_loss"]) + 1)
    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(epochs, history["train_loss"], label="train")
    axes[0].plot(epochs, history["val_loss"], label="validation")
    axes[0].axvline(run["best_epoch"], color="gray", linestyle="--", label="best")
    axes[0].set_title("Loss")
    axes[0].legend()
    axes[1].plot(epochs, np.array(history["train_acc"]) * 100, label="train")
    axes[1].plot(epochs, np.array(history["val_acc"]) * 100, label="validation")
    axes[1].axvline(run["best_epoch"], color="gray", linestyle="--", label="best")
    axes[1].set_title("Accuracy (%)")
    axes[1].legend()
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    print(f"학습 곡선 저장: {path}")
    if show:
        plt.show()
    plt.close(figure)


def main() -> None:
    args = parse_args()
    if not args.data.is_dir():
        raise FileNotFoundError(f"이미지 폴더를 찾을 수 없습니다: {args.data}")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    preview_model = MiniVGGNet(num_classes=len(datasets.ImageFolder(args.data).classes)).to(device)
    summary(preview_model, input_size=(3, IMAGE_SIZE, IMAGE_SIZE), device=device.type)
    del preview_model

    runs = [train_one_seed(args, seed, device) for seed in args.seeds]
    selected = min(runs, key=lambda run: run["best_val_loss"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    checkpoint = {
        "model_version": MODEL_VERSION,
        "state_dict": selected["state_dict"],
        "class_names": selected["class_names"],
        "image_size": IMAGE_SIZE,
        "normalize_mean": NORMALIZE_MEAN,
        "normalize_std": NORMALIZE_STD,
        "seed": selected["seed"],
        "split_seed": args.split_seed,
        "best_epoch": selected["best_epoch"],
        "best_val_loss": selected["best_val_loss"],
        "best_val_accuracy": selected["best_val_accuracy"],
        "best_macro_recall": selected["best_macro_recall"],
        "best_class_recall": selected["best_class_recall"],
    }
    torch.save(checkpoint, args.output)

    public_runs = [{key: value for key, value in run.items() if key != "state_dict"} for run in runs]
    report_path = args.output.with_suffix(".json")
    report_path.write_text(json.dumps(public_runs, indent=2), encoding="utf-8")
    plot_path = args.output.with_name(f"{args.output.stem}_training.png")
    save_plot(selected, plot_path, show=not args.no_show)

    accuracies = np.array([run["best_val_accuracy"] for run in runs]) * 100
    print(
        f"\nbest checkpoint: {args.output}\n"
        f"selected seed={selected['seed']} epoch={selected['best_epoch']} "
        f"val_acc={selected['best_val_accuracy'] * 100:.2f}% "
        f"macro_recall={selected['best_macro_recall'] * 100:.2f}%\n"
        f"seed stability: mean={accuracies.mean():.2f}% std={accuracies.std(ddof=0):.2f}% "
        f"runs={len(accuracies)}"
    )


if __name__ == "__main__":
    main()
