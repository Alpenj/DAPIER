#!/usr/bin/env python3
"""문제 1: 내 웹캠으로 가위·바위·보·없음을 학습하고 비교한다.

2026-09-09 실제 Test Accuracy: MLP 60.00%, CNN 74.17%,
ResNet18 동결 60.83%, 전체 파인튜닝 79.17% (120장, 모델별 10 Epoch).
상세 Accuracy/Loss/Latency/FPS는 실행별 RESULTS.md 및 report.html에 기록된다.
합성 smoke 결과는 구현 검증 전용이며 제출 성능으로 사용하지 않는다.

serve: 촬영·학습·게임 브라우저 UI, train: 같은 파이프라인의 CLI 실행.
촬영 흐름은 기존 resnet_dataset_studio_step1의 getUserMedia/로컬 JPEG 저장
패턴을 재사용한다. 촬영과 게임에서 같은 중앙 70% ROI를 사용한다.
"""

from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import hashlib
import html
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import random
import re
import signal
import statistics
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, urlsplit
import uuid

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, UnidentifiedImageError
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset
from torchvision import models, transforms

ROOT = Path(__file__).resolve().parent
CLASSES = ("scissors", "rock", "paper", "none")
KOREAN = dict(zip(CLASSES, ("가위", "바위", "보", "없음")))
MODEL_NAMES = ("mlp", "cnn", "resnet_frozen", "resnet_finetune")
FRAMES_PER_CLASS = 30
IMAGE_SIZE = 128
MEAN, STD = (0.485, 0.456, 0.406), (0.229, 0.224, 0.225)
ID_PATTERN = re.compile(r"^[a-f0-9]{32}$")
MAX_IMAGE_BYTES = 2_000_000


def write_json(path: Path, value) -> None:
    """중간에 프로세스가 끝나도 기존 JSON을 손상시키지 않는다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    tmp.replace(path)


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def sessions(data: Path) -> list[dict]:
    records = []
    for path in sorted((data / "sessions").glob("*")):
        if not path.is_dir() or not ID_PATTERN.fullmatch(path.name):
            continue
        counts = {c: len(list((path / c).glob("*.jpg"))) for c in CLASSES}
        meta = read_json(path / "session.json")
        records.append({"id": path.name, "counts": counts,
                        "complete": all(n == FRAMES_PER_CLASS for n in counts.values()),
                        "created": meta["created"]})
    return sorted(records, key=lambda x: (x["created"], x["id"]))


def make_manifest(data: Path, seed: int = 42) -> dict:
    """촬영 회차를 통째로 분리한다. 같은 회차의 연속 프레임은 섞지 않는다.

    모든 회차가 클래스별 30장이므로 10회 단위의 7:2:1 회차 분할은
    이미지 개수도 정확히 7:2:1이다. 테스트 회차는 모델 선택에 쓰지 않는다.
    """
    records = sessions(data)
    if len(records) < 10 or len(records) % 10 or not all(s["complete"] for s in records):
        raise ValueError("4종류 × 30장 촬영을 회차마다 완료해야 합니다. 완료 회차는 10회 단위로 필요합니다.")
    shuffled = [s["id"] for s in records]
    random.Random(seed).shuffle(shuffled)
    n = len(shuffled) // 10
    groups = {"train": shuffled[:7*n], "val": shuffled[7*n:9*n], "test": shuffled[9*n:]}
    result = {"seed": seed, "classes": list(CLASSES), "session_groups": groups, "splits": {}}
    seen_hashes = {}
    for split, ids in groups.items():
        rows = []
        for session in ids:
            for label, name in enumerate(CLASSES):
                for path in sorted((data / "sessions" / session / name).glob("*.jpg")):
                    if not path.resolve().is_relative_to(data.resolve()):
                        raise ValueError("데이터 폴더 밖의 이미지 경로입니다.")
                    digest = hashlib.sha256(path.read_bytes()).hexdigest()
                    if digest in seen_hashes and seen_hashes[digest] != split:
                        raise ValueError("서로 다른 분할에 동일 파일 내용이 있습니다. 복사된 사진을 점검하세요.")
                    seen_hashes[digest] = split
                    rows.append({"path": path.relative_to(data).as_posix(), "label": label,
                                 "session": session, "sha256": digest})
        result["splits"][split] = rows
    fingerprint = json.dumps(result, sort_keys=True).encode()
    result["fingerprint"] = hashlib.sha256(fingerprint).hexdigest()
    return result


def image_transform(training: bool, size: int = IMAGE_SIZE):
    # 동일 128px 입력으로 네 모델을 비교한다. ImageNet의 표준 224px보다 작게
    # 사용하는 연산량 절충을 보고서에 명시한다. 색상·방향 증강은 손 패 라벨을 유지한다.
    ops = ([transforms.Resize((size + 16, size + 16)), transforms.RandomCrop(size),
            transforms.RandomHorizontalFlip(), transforms.ColorJitter(.15, .15, .15)]
           if training else [transforms.Resize((size, size))])
    return transforms.Compose(ops + [transforms.ToTensor(), transforms.Normalize(MEAN, STD)])


class HandDataset(Dataset):
    """시험 1-1: 촬영 manifest와 분할별 transform을 연결하는 Custom Dataset."""
    def __init__(self, data: Path, rows: list[dict], training=False, size=IMAGE_SIZE):
        self.data, self.rows = data.resolve(), rows
        self.transform = image_transform(training, size)

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        row = self.rows[index]
        path = (self.data / row["path"]).resolve()
        if not path.is_relative_to(self.data) or row["label"] not in range(len(CLASSES)):
            raise ValueError("잘못된 데이터 경로 또는 라벨입니다.")
        with Image.open(path) as image:
            tensor = self.transform(image.convert("RGB"))
        return tensor, row["label"]


class MLP(nn.Module):
    def __init__(self, size=IMAGE_SIZE):
        super().__init__()
        self.layers = nn.Sequential(nn.Flatten(), nn.Linear(3*size*size, 256), nn.ReLU(),
                                    nn.Dropout(.2), nn.Linear(256, 128), nn.ReLU(), nn.Linear(128, 4))

    def forward(self, x):
        return self.layers(x)


class CNN(nn.Module):
    def __init__(self):
        super().__init__()
        # 합성곱 3개: 공간 특징을 단계적으로 추출한다. Adaptive pooling은
        # 시험용 작은 입력에서도 같은 분류층으로 출력 크기를 유지한다.
        self.layers = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1), nn.BatchNorm2d(16), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.BatchNorm2d(32), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(), nn.MaxPool2d(2),
            nn.AdaptiveAvgPool2d((4, 4)), nn.Flatten(), nn.Linear(64*4*4, 128),
            nn.ReLU(), nn.Dropout(.2), nn.Linear(128, 4))

    def forward(self, x):
        return self.layers(x)


def build_model(name, *, pretrained=True, size=IMAGE_SIZE):
    if name == "mlp":
        return MLP(size)
    if name == "cnn":
        return CNN()
    if name not in MODEL_NAMES:
        raise ValueError("알 수 없는 모델입니다.")
    weights = models.ResNet18_Weights.DEFAULT if pretrained else None
    model = models.resnet18(weights=weights)
    model.fc = nn.Linear(model.fc.in_features, 4)
    if name == "resnet_frozen":
        for p in model.parameters():
            p.requires_grad_(False)
        for p in model.fc.parameters():
            p.requires_grad_(True)
    return model


def train_mode(model, name):
    model.train()
    if name == "resnet_frozen":
        # requires_grad=False만으로는 BatchNorm running statistics가 고정되지 않는다.
        model.eval()
        model.fc.train()


def make_optimizer(model, name):
    if name == "resnet_finetune":
        # 기존 특징은 작은 LR로 보존하며 새 분류층은 더 빠르게 적응시킨다.
        groups = [{"params": [p for n, p in model.named_parameters() if not n.startswith("fc.")], "lr": 1e-4},
                  {"params": model.fc.parameters(), "lr": 1e-3}]
    else:
        groups = [{"params": [p for p in model.parameters() if p.requires_grad], "lr": 1e-3}]
    return torch.optim.Adam(groups, weight_decay=1e-4)


def epoch_pass(model, loader, device, optimizer=None, name=""):
    if optimizer is None:
        model.eval()
    else:
        train_mode(model, name)
    loss_sum, correct, total = 0., 0, 0
    confusion = torch.zeros(4, 4, dtype=torch.int64)
    with torch.set_grad_enabled(optimizer is not None):
        for x, y in loader:
            x, y = x.to(device), y.to(device)
            if optimizer is not None:
                optimizer.zero_grad(set_to_none=True)
            logits = model(x)
            loss = nn.functional.cross_entropy(logits, y)
            if not torch.isfinite(loss):
                raise ValueError("유한하지 않은 loss가 발생했습니다.")
            if optimizer is not None:
                loss.backward()
                optimizer.step()
            predicted = logits.detach().argmax(1)
            loss_sum += loss.item()*len(y)
            correct += (predicted == y).sum().item()
            total += len(y)
            confusion += torch.bincount((y*4+predicted).cpu(), minlength=16).reshape(4, 4)
    if not total:
        raise ValueError("빈 데이터 분할입니다.")
    return {"loss": loss_sum/total, "accuracy": correct/total, "confusion": confusion.tolist()}


def sync(device):
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def benchmark(model, image, device, size, repeats=50):
    """Batch 1 순수 forward와 전처리 포함 시간을 분리한다. 카메라·HTTP는 제외."""
    transform = image_transform(False, size)
    x = transform(image).unsqueeze(0).to(device)
    model.eval()
    forward, pipeline = [], []
    with torch.inference_mode():
        for _ in range(10):
            model(x)
        sync(device)
        for _ in range(repeats):
            start = time.perf_counter()
            model(x)
            sync(device)
            forward.append((time.perf_counter()-start)*1000)
        for _ in range(repeats):
            start = time.perf_counter()
            value = transform(image).unsqueeze(0).to(device)
            model(value).softmax(1).cpu()
            sync(device)
            pipeline.append((time.perf_counter()-start)*1000)
    ms = statistics.mean(forward)
    return {"latency_ms": ms, "fps": 1000/ms, "latency_p95_ms": float(np.percentile(forward, 95)),
            "pipeline_ms": statistics.mean(pipeline), "benchmark_repeats": repeats, "benchmark_batch": 1}


def save_curves(run: Path, histories):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    for name, rows in histories.items():
        for ax, metric in zip(axes, ("loss", "accuracy")):
            line, = ax.plot([r["epoch"] for r in rows], [r["val_"+metric] for r in rows], label=name+" val")
            ax.plot([r["epoch"] for r in rows], [r["train_"+metric] for r in rows],
                    "--", color=line.get_color(), alpha=.5, label=name+" train")
            ax.set(xlabel="Epoch", ylabel=metric.title())
            ax.grid(alpha=.2)
    axes[1].set_ylim(0, 1.03)
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(run/"learning_curves.png", dpi=150)
    plt.close(fig)


def save_confusion(run, name, values):
    fig, ax = plt.subplots(figsize=(5, 4))
    matrix = np.array(values)
    ax.imshow(matrix, cmap="Blues")
    for (i, j), value in np.ndenumerate(matrix):
        ax.text(j, i, str(value), ha="center", va="center",
                color="white" if value > matrix.max()/2 else "black")
    ax.set(xticks=range(4), yticks=range(4), xticklabels=CLASSES, yticklabels=CLASSES,
           xlabel="Predicted", ylabel="True", title=name)
    fig.tight_layout()
    fig.savefig(run/f"{name}_confusion.png", dpi=130)
    plt.close(fig)


def save_examples(run, data, manifest, model, device, size):
    rows = [next(r for r in manifest["splits"]["train"] if r["label"] == label) for label in range(4)]
    fig, axes = plt.subplots(2, 4, figsize=(10, 5))
    for i, row in enumerate(rows):
        with Image.open(data/row["path"]) as source:
            source = source.convert("RGB")
            axes[0, i].imshow(source)
            augmented = image_transform(True, size)(source).permute(1, 2, 0).numpy()
            axes[1, i].imshow(np.clip(augmented*np.array(STD)+np.array(MEAN), 0, 1))
        axes[0, i].set_title(CLASSES[i]+" original")
        axes[1, i].set_title("augmented")
    for ax in axes.flat:
        ax.axis("off")
    fig.tight_layout(); fig.savefig(run/"augmentation.png", dpi=130); plt.close(fig)
    chosen = [next(r for r in manifest["splits"]["test"] if r["label"] == label) for label in range(4)]
    fig, axes = plt.subplots(1, 4, figsize=(12, 3))
    model.eval()
    with torch.inference_mode():
        for ax, row in zip(axes, chosen):
            with Image.open(data/row["path"]) as source:
                source = source.convert("RGB")
                p = model(image_transform(False, size)(source).unsqueeze(0).to(device)).softmax(1)[0].cpu()
                ax.imshow(source)
            ax.set_title(f"true {CLASSES[row['label']]}\npred {CLASSES[p.argmax().item()]} ({p.max():.1%})")
            ax.axis("off")
    fig.tight_layout(); fig.savefig(run/"sample_predictions.png", dpi=130); plt.close(fig)


def save_report(run, meta, metrics, manifest, complete=False):
    status = "완료" if complete else "진행 중"
    if meta["smoke"]:
        status = "합성 SMOKE — 제출 성능 아님 / " + status
    headers = "| 모델 | Val Accuracy | Test Accuracy | Test Loss | ms/장 | FPS |\n|---|---:|---:|---:|---:|---:|\n"
    rows = "".join(f"| {m['model']} | {m['val_accuracy']:.4f} | {m['test_accuracy']:.4f} | {m['test_loss']:.4f} | {m['latency_ms']:.3f} | {m['fps']:.1f} |\n" for m in metrics)
    counts = {k: len(v) for k, v in manifest["splits"].items()}
    text = (f"# 문제 1 실습 결과 — {status}\n\nrecord_id: DAPIER-2026-09-09-rps-exam\n\n"
            + headers + rows + f"\n나는 웹캠의 가위·바위·보·없음을 네 클래스로 분류하는 실험을 진행한다.\n"
            f"- 데이터: 촬영 회차 단위 7:2:1. 이미지 개수 {counts}.\n"
            f"- seed={meta['seed']}, Epoch={meta['epochs']}, Batch={meta['batch_size']}, 입력={meta['image_size']}px.\n"
            f"- 장치: {meta['device_name']}, PyTorch {meta['torch_version']}.\n"
            "- 네 모델 모두 같은 분할·입력 크기·ImageNet 정규화를 사용한다. ResNet 표준 224px보다 작은 입력은 연산량 절충이다.\n"
            "- MLP/CNN/동결 ResNet Adam LR=0.001. 파인튜닝 백본 LR=0.0001, fc LR=0.001. weight_decay=0.0001.\n"
            "- 학습에만 RandomCrop·RandomHorizontalFlip·ColorJitter를 적용한다. 검증 Accuracy(동률이면 낮은 Loss)로 가중치를 고른다.\n"
            "- 테스트 지표는 각 모델의 가중치를 선택한 뒤 계산한다. 게임 기본 모델도 검증 Accuracy로 선택한다.\n"
            "- 속도: batch 1, 준비 추론 10회 뒤 반복 측정, GPU 동기화. 표는 전처리를 제외한 forward 평균이며 FPS=1000/ms이다.\n"
            "- metrics.json의 pipeline_ms는 전처리·장치 전송·forward·확률 반환을 포함한다. 둘 다 카메라·브라우저·HTTP 시간은 제외한다.\n"
            "- PC 대응 규칙이 이겼다는 사실은 인식 정확도의 증거가 아니다. none 오인식은 혼동행렬로 확인한다.\n"
            "- 아직 별도 사용자·장소에서의 일반화와 실제 카메라 게임 정확도는 별도로 검증해야 한다. 증강의 개선 효과도 비교 실험 전에는 단정하지 않는다.\n")
    if complete and metrics:
        best = max(metrics, key=lambda m: (m["val_accuracy"], -m["val_loss"]))
        text += f"\n검증 기준으로 선택한 게임 모델은 **{best['model']}**이다.\n"
    (run/"RESULTS.md").write_text(text, encoding="utf-8")
    images = ""
    for name in ["learning_curves.png", "augmentation.png", "sample_predictions.png"] + [f"{m['model']}_confusion.png" for m in metrics]:
        path = run/name
        if path.exists():
            encoded = base64.b64encode(path.read_bytes()).decode()
            images += f'<figure><img alt="{name}" src="data:image/png;base64,{encoded}"><figcaption>{name}</figcaption></figure>'
    table_rows = "".join(f"<tr><th>{m['model']}</th><td>{m['val_accuracy']:.2%}</td><td>{m['test_accuracy']:.2%}</td><td>{m['test_loss']:.4f}</td><td>{m['latency_ms']:.3f}</td><td>{m['fps']:.1f}</td></tr>" for m in metrics)
    document = f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>문제 1 실제 실험 결과</title>
<style>body{{max-width:1100px;margin:40px auto;padding:0 24px;font:18px/1.8 system-ui;color:#182c40}}table{{border-collapse:collapse;width:100%}}th,td{{border-bottom:1px solid #ccd5e0;padding:10px;text-align:left}}img{{max-width:100%;height:auto}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;font:15px/1.8 system-ui;background:#f2f5f8;padding:20px}}figure{{margin:28px 0}}@media print{{body{{font-size:11pt;margin:0}}figure{{break-inside:avoid}}}}</style>
<h1>문제 1 · 실제 실험 결과</h1><p>{html.escape(status)}</p><div style="overflow:auto"><table><tr><th>모델</th><th>Val Accuracy</th><th>Test Accuracy</th><th>Test Loss</th><th>ms/장</th><th>FPS</th></tr>{table_rows}</table></div>{images}<h2>실행 조건과 해석</h2><pre>{html.escape(text)}</pre></html>'''
    (run/"report.html").write_text(document, encoding="utf-8")


def train(args):
    if not 10 <= args.epochs <= 100 and not (args.smoke and 1 <= args.epochs <= 100):
        raise ValueError("제출용 학습은 10~100 Epoch가 필요합니다. 작은 검증에만 --smoke를 사용하세요.")
    if not 1 <= args.batch_size <= 128:
        raise ValueError("Batch size는 1~128이어야 합니다.")
    torch.set_num_threads(4)
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    run = args.run.resolve()
    run.mkdir(parents=True, exist_ok=True)
    if (run/"meta.json").exists():
        raise ValueError("이미 실행 기록이 있는 폴더입니다. 새 run 경로를 지정하세요.")
    manifest = make_manifest(args.data, args.seed)
    if args.smoke:
        # ponytail: tiny synthetic smoke validates flow only; use full per-session data for real evaluation.
        for split, rows in manifest["splits"].items():
            manifest["splits"][split] = [r for label in range(4) for r in [next(x for x in rows if x["label"] == label)]]
    write_json(run/"manifest.json", manifest)
    size = 32 if args.smoke else IMAGE_SIZE
    meta = {"seed": args.seed, "epochs": args.epochs, "batch_size": args.batch_size,
            "image_size": size, "classes": list(CLASSES), "smoke": args.smoke, "complete": False,
            "device_name": torch.cuda.get_device_name(0) if device.type == "cuda" else "CPU",
            "torch_version": str(torch.__version__), "fingerprint": manifest["fingerprint"],
            "created": datetime.now(timezone.utc).isoformat()}
    write_json(run/"meta.json", meta)
    metrics, histories = [], {}
    for name in MODEL_NAMES:
        # seed와 DataLoader generator를 모델마다 재설정해 증강·순서의 조건을 맞춘다.
        random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
        model = build_model(name, pretrained=not args.smoke, size=size).to(device)
        optimizer = make_optimizer(model, name)
        data = {s: HandDataset(args.data, rows, training=s == "train", size=size) for s, rows in manifest["splits"].items()}
        loaders = {s: DataLoader(ds, batch_size=args.batch_size, shuffle=s == "train", num_workers=0,
                                generator=torch.Generator().manual_seed(args.seed)) for s, ds in data.items()}
        history, best_key, best_epoch = [], (-1., -math.inf), 0
        trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
        total_params = sum(p.numel() for p in model.parameters())
        print(f"{name}: {total_params:,} parameters ({trainable:,} trainable)", flush=True)
        for epoch in range(1, args.epochs+1):
            start = time.perf_counter()
            t = epoch_pass(model, loaders["train"], device, optimizer, name)
            v = epoch_pass(model, loaders["val"], device)
            row = {"epoch": epoch, "train_loss": t["loss"], "train_accuracy": t["accuracy"],
                   "val_loss": v["loss"], "val_accuracy": v["accuracy"], "seconds": time.perf_counter()-start}
            history.append(row)
            write_json(run/f"{name}_history.json", history)
            print(f"{name} {epoch:02d}/{args.epochs} train loss={t['loss']:.4f} acc={t['accuracy']:.3f} | val loss={v['loss']:.4f} acc={v['accuracy']:.3f}", flush=True)
            key = (v["accuracy"], -v["loss"])
            if key > best_key:
                best_key, best_epoch = key, epoch
                checkpoint = {"model": name, "classes": list(CLASSES), "image_size": size,
                              "state_dict": {k: p.detach().cpu() for k, p in model.state_dict().items()},
                              "val_accuracy": v["accuracy"], "epoch": epoch, "smoke": args.smoke}
                torch.save(checkpoint, run/f"{name}.pt.tmp")
                (run/f"{name}.pt.tmp").replace(run/f"{name}.pt")
        checkpoint = torch.load(run/f"{name}.pt", map_location="cpu", weights_only=True)
        model.load_state_dict(checkpoint["state_dict"])
        test = epoch_pass(model, loaders["test"], device)
        with Image.open(args.data/manifest["splits"]["test"][0]["path"]) as source:
            timing = benchmark(model, source.convert("RGB"), device, size, repeats=3 if args.smoke else 50)
        metric = {"model": name, "val_accuracy": best_key[0], "val_loss": -best_key[1],
                  "best_epoch": best_epoch, "test_accuracy": test["accuracy"], "test_loss": test["loss"],
                  "confusion": test["confusion"], "trainable_parameters": trainable,
                  "total_parameters": total_params, **timing}
        metrics.append(metric); histories[name] = history
        write_json(run/"metrics.json", metrics)
        save_curves(run, histories)
        save_confusion(run, name, test["confusion"])
        save_report(run, meta, metrics, manifest)
        print(f"{name} test accuracy={test['accuracy']:.4f}, inference={timing['latency_ms']:.3f} ms", flush=True)
        del model, optimizer, checkpoint
        if device.type == "cuda":
            torch.cuda.empty_cache()
    best = max(metrics, key=lambda m: (m["val_accuracy"], -m["val_loss"]))
    model, _ = load_checkpoint(run/f"{best['model']}.pt", device, allow_smoke=args.smoke)
    save_examples(run, args.data, manifest, model, device, size)
    meta["complete"] = True
    meta["selected_model"] = best["model"]
    write_json(run/"meta.json", meta)
    save_report(run, meta, metrics, manifest, complete=True)
    print(f"완료: {run/'report.html'}", flush=True)


def load_checkpoint(path, device, allow_smoke=False):
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    if checkpoint.get("classes") != list(CLASSES) or checkpoint.get("model") not in MODEL_NAMES:
        raise ValueError("체크포인트 클래스·모델이 이 게임과 다릅니다.")
    if checkpoint.get("smoke") and not allow_smoke:
        raise ValueError("합성 smoke 가중치는 실제 게임에 사용할 수 없습니다.")
    if checkpoint.get("image_size") not in (32, IMAGE_SIZE):
        raise ValueError("체크포인트 이미지 크기가 잘못되었습니다.")
    model = build_model(checkpoint["model"], pretrained=False, size=checkpoint["image_size"])
    model.load_state_dict(checkpoint["state_dict"])
    return model.to(device).eval(), checkpoint


def decode_image(raw):
    if not 1 <= len(raw) <= MAX_IMAGE_BYTES:
        raise ValueError("이미지 크기가 허용 범위를 벗어났습니다.")
    try:
        with Image.open(io.BytesIO(raw)) as image:
            if image.format != "JPEG" or image.size != (224, 224):
                raise ValueError("224×224 JPEG 이미지가 필요합니다.")
            image.load()
            return image.convert("RGB")
    except (UnidentifiedImageError, OSError) as error:
        raise ValueError("JPEG 이미지를 읽을 수 없습니다.") from error


class Studio(ThreadingHTTPServer):
    """로컬 브라우저용 작은 서버. 파일·모델 변경은 하나의 lock으로 직렬화한다."""
    daemon_threads = True

    def __init__(self, address, data, runs):
        super().__init__(address, Handler)
        self.data, self.runs = data.resolve(), runs.resolve()
        self.data.mkdir(parents=True, exist_ok=True); self.runs.mkdir(parents=True, exist_ok=True)
        # ponytail: one-user local studio uses a global lock; split inference/storage locks if multiple clients are needed.
        self.lock = threading.Lock()
        self.process = None
        self.run_id = None
        self.cached_key = self.cached_model = self.cached_transform = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    def running(self):
        return self.process is not None and self.process.poll() is None

    def status(self):
        ss = sessions(self.data)
        completed = sum(s["complete"] for s in ss)
        runs, available = [], []
        for run in sorted(self.runs.glob("*"), key=lambda p: p.stat().st_mtime, reverse=True):
            if not ID_PATTERN.fullmatch(run.name) or not (run/"meta.json").exists():
                continue
            meta = read_json(run/"meta.json")
            if meta.get("smoke"):
                continue
            metrics = read_json(run/"metrics.json") if (run/"metrics.json").exists() else []
            runs.append({"id": run.name, "complete": meta.get("complete", False), "metrics": metrics,
                         "report_url": f"/results/{run.name}/report.html" if (run/"report.html").exists() else None})
            if meta.get("complete"):
                available.extend({"run": run.name, **m} for m in metrics)
        tail, error = "", None
        if self.run_id:
            log = self.runs/self.run_id/"train.log"
            if log.exists():
                tail = "\n".join(log.read_text(encoding="utf-8", errors="replace").splitlines()[-18:])
            if self.process is not None and self.process.poll() not in (None, 0):
                error = "학습이 중단되었습니다. 아래 실행 로그를 확인하세요."
        return {"classes": list(CLASSES), "frames_per_class": FRAMES_PER_CLASS, "sessions": ss,
                "total_counts": {c: sum(s["counts"][c] for s in ss) for c in CLASSES},
                "complete_sessions": completed,
                "ready_to_train": completed >= 10 and completed % 10 == 0 and len(ss) == completed,
                "training": {"running": self.running(), "run_id": self.run_id, "log_tail": tail, "error": error},
                "runs": runs, "models": available}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass  # 카메라 프레임마다 터미널에 로그를 반복하지 않는다.

    def reply(self, status, payload, content_type="application/json; charset=utf-8"):
        raw = json.dumps(payload, ensure_ascii=False).encode() if isinstance(payload, dict) else payload
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Cross-Origin-Resource-Policy", "same-origin")
        self.end_headers()
        try:
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def local_request(self):
        # localhost binding alone does not prevent DNS rebinding or cross-origin writes.
        port = self.server.server_port
        hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
        host = self.headers.get("Host", "")
        if host not in hosts:
            raise ValueError("로컬 주소로 접속하세요.")
        origin = self.headers.get("Origin")
        if origin and origin != "http://"+host:
            raise ValueError("다른 웹사이트에서의 요청은 허용하지 않습니다.")
        if self.headers.get("Sec-Fetch-Site") == "cross-site":
            raise ValueError("교차 사이트 요청은 허용하지 않습니다.")

    def body(self, limit=MAX_IMAGE_BYTES):
        length = int(self.headers.get("Content-Length", "0"))
        if not 0 < length <= limit:
            raise ValueError("요청 크기가 허용 범위를 벗어났습니다.")
        self.connection.settimeout(10)
        raw = self.rfile.read(length)
        if len(raw) != length:
            raise ValueError("요청이 완전히 수신되지 않았습니다.")
        return raw

    def do_GET(self):
        try:
            self.local_request()
            path = urlsplit(self.path).path
            if path in ("/", "/studio.html"):
                self.reply(200, (ROOT/"studio.html").read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/status":
                with self.server.lock:
                    self.reply(200, self.server.status())
            elif re.fullmatch(r"/results/[a-f0-9]{32}/[a-zA-Z0-9_.]+", path):
                _, _, run, name = path.split("/")
                file = self.server.runs/run/name
                types = {".html": "text/html; charset=utf-8", ".png": "image/png", ".md": "text/plain; charset=utf-8"}
                if file.suffix not in types or not file.is_file():
                    self.reply(404, {"error": "결과 파일이 없습니다."})
                else:
                    self.reply(200, file.read_bytes(), types[file.suffix])
            else:
                self.reply(404, {"error": "페이지가 없습니다."})
        except ValueError as e:
            self.reply(400, {"error": str(e)})
        except OSError as e:
            self.reply(500, {"error": f"파일 읽기 실패: {e}"})

    def do_POST(self):
        try:
            self.local_request()
            parsed = urlsplit(self.path)
            params = parse_qs(parsed.query)
            raw = self.body()
            with self.server.lock:
                if parsed.path == "/api/session":
                    self.new_session()
                elif parsed.path == "/api/capture":
                    self.capture(params, raw)
                elif parsed.path == "/api/train":
                    payload = json.loads(raw)
                    if not isinstance(payload, dict):
                        raise ValueError("JSON 객체가 필요합니다.")
                    self.start_training(payload)
                elif parsed.path == "/api/predict":
                    self.predict(params, raw)
                else:
                    self.reply(404, {"error": "API가 없습니다."})
        except (ValueError, TypeError, KeyError) as e:
            self.reply(400, {"error": str(e)})
        except (OSError, RuntimeError) as e:
            self.reply(500, {"error": f"실행 실패: {e}"})

    def require_idle(self):
        if self.server.running():
            raise ValueError("학습 중에는 촬영·게임을 멈춥니다. 학습 완료 후 다시 실행하세요.")

    def new_session(self):
        self.require_idle()
        ss = sessions(self.server.data)
        if any(not s["complete"] for s in ss):
            raise ValueError("진행 중인 촬영 회차를 먼저 완료하세요.")
        identifier = uuid.uuid4().hex
        path = self.server.data/"sessions"/identifier
        path.mkdir(parents=True)
        write_json(path/"session.json", {"created": datetime.now(timezone.utc).isoformat(),
                                      "classes": list(CLASSES), "frames_per_class": FRAMES_PER_CLASS,
                                      "roi": "central 70% of min dimension, mirrored", "image_size": 224})
        self.reply(201, {"session": sessions(self.server.data)[-1]})

    def capture(self, params, raw):
        self.require_idle()
        sid = params.get("session", [""])[0]
        label = params.get("label", [""])[0]
        if not ID_PATTERN.fullmatch(sid) or label not in CLASSES:
            raise ValueError("올바른 촬영 회차와 라벨이 필요합니다.")
        ss = sessions(self.server.data)
        if not ss or ss[-1]["id"] != sid:
            raise ValueError("현재 촬영 회차만 저장할 수 있습니다.")
        count = ss[-1]["counts"][label]
        if count >= FRAMES_PER_CLASS:
            raise ValueError("이 종류는 이번 회차의 촬영이 완료되었습니다.")
        image = decode_image(raw)
        target = self.server.data/"sessions"/sid/label/f"{count+1:04d}.jpg"
        target.parent.mkdir(exist_ok=True)
        # EXIF 등 부가 메타데이터를 제거하고 학습에 쓰는 픽셀만 저장한다.
        temporary = target.with_suffix(".tmp")
        try:
            # 이전 저장 실패의 임시 파일은 재사용한다. 완료된 JPEG는 덮어쓰지 않는다.
            with temporary.open("wb") as out:
                image.save(out, format="JPEG", quality=95)
            temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)
        self.reply(201, {"count": count+1})

    def start_training(self, payload):
        self.require_idle()
        epochs = payload.get("epochs", 10)
        if type(epochs) is not int or not 10 <= epochs <= 100:
            raise ValueError("학습은 10~100 Epoch 정수로 지정하세요.")
        make_manifest(self.server.data)  # 실제 파일 중복까지 확인한 뒤 학습 프로세스를 시작한다.
        sid = uuid.uuid4().hex
        run = self.server.runs/sid
        run.mkdir()
        self.server.cached_key = self.server.cached_model = self.server.cached_transform = None
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        command = [sys.executable, "-u", str(Path(__file__).resolve()), "train", "--data", str(self.server.data),
                   "--run", str(run), "--epochs", str(epochs)]
        with (run/"train.log").open("w", encoding="utf-8") as log:
            self.server.process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        self.server.run_id = sid
        self.reply(202, {"run_id": sid})

    def predict(self, params, raw):
        self.require_idle()
        run, name = params.get("run", [""])[0], params.get("model", [""])[0]
        if not ID_PATTERN.fullmatch(run) or name not in MODEL_NAMES:
            raise ValueError("완료된 학습 모델을 선택하세요.")
        meta = read_json(self.server.runs/run/"meta.json")
        if not meta.get("complete") or meta.get("smoke"):
            raise ValueError("실제 데이터로 학습을 완료한 모델이 필요합니다.")
        image = decode_image(raw)
        if self.server.cached_key != (run, name):
            model, checkpoint = load_checkpoint(self.server.runs/run/f"{name}.pt", self.server.device)
            self.server.cached_model = model
            self.server.cached_transform = image_transform(False, checkpoint["image_size"])
            self.server.cached_key = (run, name)
        x = self.server.cached_transform(image).unsqueeze(0).to(self.server.device)
        sync(self.server.device)
        start = time.perf_counter()
        with torch.inference_mode():
            logits = self.server.cached_model(x)
            sync(self.server.device)
            latency = (time.perf_counter()-start)*1000
            probabilities = logits.softmax(1)[0].cpu().tolist()
        index = max(range(4), key=lambda i: probabilities[i])
        self.reply(200, {"label": CLASSES[index], "confidence": probabilities[index],
                         "probabilities": probabilities, "latency_ms": latency})


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    serve_parser = sub.add_parser("serve", help="촬영·학습·게임 웹 화면")
    serve_parser.add_argument("--port", type=int, default=8766)
    serve_parser.add_argument("--data", type=Path, default=ROOT/"data")
    serve_parser.add_argument("--runs", type=Path, default=ROOT/"runs")
    trainer = sub.add_parser("train", help="네 모델 순차 학습·평가")
    trainer.add_argument("--data", type=Path, default=ROOT/"data")
    trainer.add_argument("--run", type=Path, required=True)
    trainer.add_argument("--epochs", type=int, default=10)
    trainer.add_argument("--batch-size", type=int, default=32)
    trainer.add_argument("--seed", type=int, default=42)
    trainer.add_argument("--smoke", action="store_true", help="합성 검증용, 게임·제출 성능으로 사용 금지")
    args = parser.parse_args()
    if args.command == "train":
        train(args)
        return
    torch.set_num_threads(4)
    server = Studio(("127.0.0.1", args.port), args.data, args.runs)
    def stop_server(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_server)
    print(f"문제 1 스튜디오: http://127.0.0.1:{server.server_port}", flush=True)
    print("카메라는 브라우저의 ‘카메라 시작’을 눌렀을 때만 사용합니다. 종료: Ctrl+C", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if server.running():
            server.process.terminate()
            try:
                server.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                server.process.kill(); server.process.wait()
        server.server_close()


if __name__ == "__main__":
    main()
