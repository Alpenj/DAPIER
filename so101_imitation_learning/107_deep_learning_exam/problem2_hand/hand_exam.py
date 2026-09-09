#!/usr/bin/env python3
"""문제 2: 웹캠 가위바위보에 반응하는 10관절 가상 손의 미래 행동 예측.

2026-09-09 실제 Test MSE(rad²): RNN 0.001381, LSTM 0.001847,
Transformer 0.001447, ACT(z=0) 0.010636; ViT 보조 Accuracy 75.83%.
30 Epoch, mixed-open-start-v2 교사. 상세 결과는 RESULTS.md 참고.
문제 1의 촬영 이미지는 재사용하고, 관절 상태·행동은 기구학 교사로 생성한다.
이 코드는 물리 접촉 시뮬레이션이나 실물 로봇 구동을 수행하지 않는다.
"""
from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import html
import math
from pathlib import Path
import random
import signal
import subprocess
import sys
import time
from urllib.parse import parse_qs, urlsplit
import uuid

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent/"problem1_rps"))
import rps_exam as vision
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader

CLASSES = vision.CLASSES
JOINTS, MAX_RAD, CHUNK, HISTORY, EPISODE_STEPS = 10, 1.4, 8, 32, 72
LAG = .35  # 10Hz에서 목표 명령으로 접근하는 1차 기구학 모델. UI도 같은 계수를 쓴다.
SEQUENCE_MODELS = ("rnn", "lstm", "transformer")


def counter_pose(label):
    """관측 라벨의 반대 패를 목표로 한다. none의 목표는 호출자가 현재 자세로 둔다."""
    shapes = {
        "rock": [1.05, .95]*5,
        "paper": [.05, .05]*5,
        "scissors": [1.05, .95, .05, .05, .05, .05, 1.05, .95, 1.05, .95],
    }
    counter = {"scissors": "rock", "rock": "paper", "paper": "scissors"}
    return None if label == "none" else np.array(shapes[counter[label]], dtype=np.float32)


def make_episodes(rows, seed):
    """같은 이미지에 가상 손의 시작 자세·속도·중간 경로를 짝지어 학습 데이터를 만든다.

    a[t]는 q[t]에서 전송할 관절 목표, q[t+1]=q[t]+0.35*(a[t]-q[t])다.
    관절 순서는 엄지/검지/중지/약지/소지 각각 근위·원위, 단위는 rad다.
    이미지의 손은 상대 손이며 q는 내가 제어할 가상 손이다.
    """
    rng = np.random.default_rng(seed)
    episodes = []
    for index, row in enumerate(rows):
        start = rng.uniform(.05, 1.1, JOINTS).astype(np.float32)
        # UI의 전관절 0 시작도 학습에 포함한다. 나머지는 굽힌 상태를 다룬다.
        if index % 2 == 0:
            start.fill(0)
        target = counter_pose(CLASSES[row["label"]])
        duration = int(rng.integers(36, 65))
        phase = np.clip((np.arange(EPISODE_STEPS)+1)/duration, 0, 1).astype(np.float32)
        smooth = phase*phase*(3-2*phase)
        if target is None:
            actions = np.repeat(start[None], EPISODE_STEPS, axis=0)
        else:
            style = rng.uniform(-.15, .15, JOINTS).astype(np.float32)
            actions = start + smooth[:, None]*(target-start) + np.sin(np.pi*phase)[:, None]*style
            actions = np.clip(actions, 0, MAX_RAD).astype(np.float32)
        states = np.empty((EPISODE_STEPS+1, JOINTS), dtype=np.float32)
        states[0] = start
        for t, action in enumerate(actions):
            states[t+1] = states[t] + LAG*(action-states[t])
        # 장기기억 보조 과제: 초반 cue를 본 뒤 공통 중립 상태에서 다음 값을 예측.
        # 실제 촬영 관절 데이터가 아닌 통제된 수치 실험으로 결과를 따로 보고한다.
        cue = rng.uniform(0, MAX_RAD, JOINTS).astype(np.float32)
        memory = np.full((HISTORY, JOINTS), MAX_RAD/2, dtype=np.float32)
        memory[:4] = cue
        memory_target = memory[-1]+.2*(cue-memory[-1])
        episodes.append({"image": row["path"], "label": row["label"], "session": row["session"],
                         "states": states, "actions": actions, "memory": memory,
                         "memory_target": memory_target})
    return episodes


class SequenceDataset(Dataset):
    def __init__(self, episodes):
        self.episodes = episodes
        self.samples = [(i, t) for i in range(len(episodes)) for t in (31, 39, 47, 55, 63, -1)]

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        episode_id, t = self.samples[index]
        e = self.episodes[episode_id]
        if t == -1:
            return torch.from_numpy(e["memory"]), torch.from_numpy(e["memory_target"]), True
        return torch.from_numpy(e["states"][t-HISTORY+1:t+1]), torch.from_numpy(e["states"][t+1]), False


class ActionDataset(Dataset):
    def __init__(self, data, episodes, training=False):
        self.data, self.episodes = data.resolve(), episodes
        self.transform = vision.image_transform(training, 64)
        self.samples = [(i, t) for i in range(len(episodes)) for t in (0, 12, 24, 40, 56)]

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        i, t = self.samples[index]
        e = self.episodes[i]
        path = (self.data/e["image"]).resolve()
        if not path.is_relative_to(self.data):
            raise ValueError("데이터 폴더 밖의 이미지입니다.")
        with Image.open(path) as image:
            image = self.transform(image.convert("RGB"))
        return image, torch.from_numpy(e["states"][t]), torch.from_numpy(e["actions"][t:t+CHUNK]), e["label"]


class PositionEncoding(nn.Module):
    def __init__(self, dimension=64, max_length=128):
        super().__init__()
        position = torch.arange(max_length).unsqueeze(1)
        frequency = torch.exp(torch.arange(0, dimension, 2)*(-math.log(10000.)/dimension))
        values = torch.zeros(max_length, dimension)
        values[:, 0::2], values[:, 1::2] = torch.sin(position*frequency), torch.cos(position*frequency)
        self.register_buffer("values", values.unsqueeze(0))

    def forward(self, x, offset=0):
        return x+self.values[:, offset:offset+x.shape[1]]


class StatePredictor(nn.Module):
    """2-1/2-2 공통 입출력. Transformer는 encoder와 decoder를 모두 사용한다."""
    def __init__(self, name, dimension=64):
        super().__init__()
        if name not in SEQUENCE_MODELS:
            raise ValueError("알 수 없는 시퀀스 모델입니다.")
        self.name = name
        if name == "transformer":
            self.embed = nn.Linear(JOINTS, dimension)
            self.position = PositionEncoding(dimension)
            self.query = nn.Parameter(torch.randn(1, 1, dimension)*.02)
            self.network = nn.Transformer(d_model=dimension, nhead=4, num_encoder_layers=2,
                                          num_decoder_layers=2, dim_feedforward=128, dropout=.1, batch_first=True)
        else:
            recurrent = nn.RNN if name == "rnn" else nn.LSTM
            self.network = recurrent(JOINTS, dimension, num_layers=2, dropout=.1, batch_first=True)
        self.output = nn.Linear(dimension, JOINTS)

    def forward(self, history):
        if self.name == "transformer":
            source = self.position(self.embed(history/MAX_RAD))
            query = self.position(self.query.expand(len(history), -1, -1), offset=history.shape[1])
            feature = self.network(source, query)[:, 0]
        else:
            feature = self.network(history/MAX_RAD)[0][:, -1]
        return self.output(feature).sigmoid()*MAX_RAD


class PatchEmbedding(nn.Module):
    def __init__(self, patch=8, dimension=64):
        super().__init__()
        self.patch = patch
        self.projection = nn.Linear(3*patch*patch, dimension)

    def forward(self, image):
        b, c, h, w = image.shape
        p = self.patch
        if h % p or w % p:
            raise ValueError("이미지 크기는 패치 크기의 배수여야 합니다.")
        # [B,C,H,W] → [B,패치 수,C*p*p] → [B,패치 수,D]. Conv 대용 없이 Linear를 직접 사용.
        patches = image.unfold(2, p, p).unfold(3, p, p).permute(0, 2, 3, 1, 4, 5)
        return self.projection(patches.reshape(b, (h//p)*(w//p), c*p*p))


class TinyViT(nn.Module):
    def __init__(self, dimension=64):
        super().__init__()
        self.patches = PatchEmbedding(8, dimension)
        self.cls = nn.Parameter(torch.zeros(1, 1, dimension))
        self.position = nn.Parameter(torch.randn(1, 65, dimension)*.02)
        layer = nn.TransformerEncoderLayer(dimension, 4, 128, dropout=.1, batch_first=True)
        self.encoder = nn.TransformerEncoder(layer, 2)
        self.classifier = nn.Linear(dimension, 4)

    def forward(self, image):
        tokens = self.patches(image)
        tokens = torch.cat((self.cls.expand(len(image), -1, -1), tokens), dim=1)
        features = self.encoder(tokens+self.position)
        return features, self.classifier(features[:, 0])


class ActionCVAE(nn.Module):
    """2-4: ViT + state + z로 K개 액션을 예측하는 작은 ACT 구성.

    학습에서만 posterior가 정답 액션을 본다. 평가/게임은 정답 없이 z=0 또는
    사전분포 N(0,I) 표본을 쓴다. 원 ACT의 CNN 백본 대신 직접 만든 ViT를 사용한다.
    """
    def __init__(self, dimension=64, latent=8):
        super().__init__()
        self.latent = latent
        self.vision = TinyViT(dimension)
        self.state = nn.Linear(JOINTS, dimension)
        self.action = nn.Linear(JOINTS, dimension)
        self.posterior_cls = nn.Parameter(torch.zeros(1, 1, dimension))
        self.posterior_position = PositionEncoding(dimension)
        self.posterior = nn.TransformerEncoder(nn.TransformerEncoderLayer(dimension, 4, 128, batch_first=True), 1)
        self.mu, self.logvar = nn.Linear(dimension, latent), nn.Linear(dimension, latent)
        self.z_embed = nn.Linear(latent, dimension)
        self.queries = nn.Parameter(torch.randn(1, CHUNK, dimension)*.02)
        layer = nn.TransformerDecoderLayer(dimension, 4, 128, dropout=.1, batch_first=True)
        self.decoder = nn.TransformerDecoder(layer, 2)
        self.output = nn.Linear(dimension, JOINTS)

    def forward(self, image, joints, actions=None, sample=False):
        features, classification = self.vision(image)
        state = self.state(joints/MAX_RAD).unsqueeze(1)
        mu = logvar = None
        if actions is not None:
            tokens = torch.cat((self.posterior_cls.expand(len(image), -1, -1), state,
                                self.action(actions/MAX_RAD)), dim=1)
            posterior = self.posterior(self.posterior_position(tokens))[:, 0]
            mu, logvar = self.mu(posterior), self.logvar(posterior).clamp(-8, 6)
            z = mu+(logvar*.5).exp()*torch.randn_like(mu)
        else:
            z = torch.randn(len(image), self.latent, device=image.device) if sample else torch.zeros(len(image), self.latent, device=image.device)
        memory = torch.cat((features, state, self.z_embed(z).unsqueeze(1)), dim=1)
        decoded = self.decoder(self.queries.expand(len(image), -1, -1), memory)
        return self.output(decoded).sigmoid()*MAX_RAD, mu, logvar, classification


def sequence_pass(model, loader, device, optimizer=None, trim=None):
    model.train(optimizer is not None)
    totals = {"regular": [0., 0], "memory": [0., 0]}
    loss_sum, count = 0., 0
    with torch.set_grad_enabled(optimizer is not None):
        for x, y, memory in loader:
            x, y = x.to(device), y.to(device)
            if trim:
                x = x[:, -trim:]
            predicted = model(x)
            errors = (predicted-y).square().mean(1)
            loss = errors.mean()
            if not torch.isfinite(loss):
                raise ValueError("시퀀스 loss가 유한하지 않습니다.")
            if optimizer is not None:
                optimizer.zero_grad(set_to_none=True); loss.backward(); optimizer.step()
            loss_sum += loss.item()*len(y); count += len(y)
            errors = errors.detach().cpu()
            for name, mask in (("memory", memory), ("regular", ~memory)):
                totals[name][0] += errors[mask].sum().item()
                totals[name][1] += mask.sum().item()
    return {"loss": loss_sum/count, "mse": totals["regular"][0]/totals["regular"][1],
            "memory_mse": totals["memory"][0]/totals["memory"][1]}


def action_pass(model, loader, device, optimizer=None, sample=False):
    model.train(optimizer is not None)
    count, squared, kl_sum, ce_sum, correct = 0, 0., 0., 0., 0
    with torch.set_grad_enabled(optimizer is not None):
        for image, q, target, label in loader:
            image, q, target, label = image.to(device), q.to(device), target.to(device), label.to(device)
            predicted, mu, logvar, logits = model(image, q, actions=target if optimizer is not None else None, sample=sample)
            reconstruction = nn.functional.mse_loss(predicted, target)
            kl = -.5*(1+logvar-mu.square()-logvar.exp()).mean() if mu is not None else reconstruction.new_zeros(())
            classification = nn.functional.cross_entropy(logits, label)
            # 시각 특징이 손 모양을 구별하도록 보조 분류 손실을 더한다. 행동 정답은 별도의 연속 관절값이다.
            loss = reconstruction+.001*kl+.2*classification
            if not torch.isfinite(loss):
                raise ValueError("ACT loss가 유한하지 않습니다.")
            if optimizer is not None:
                optimizer.zero_grad(set_to_none=True); loss.backward(); optimizer.step()
            n = len(q); count += n; squared += reconstruction.item()*n
            kl_sum += kl.item()*n; ce_sum += classification.item()*n
            correct += (logits.argmax(1) == label).sum().item()
    return {"mse": squared/count, "kl": kl_sum/count, "classification_loss": ce_sum/count,
            "vision_accuracy": correct/count}


def cpu_weights(model):
    return {k: p.detach().cpu() for k, p in model.state_dict().items()}


def save_checkpoint(run, name, model, meta):
    torch.save({"state_dict": cpu_weights(model), "smoke": meta["smoke"],
                "classes": list(CLASSES), "joints": JOINTS, "chunk": CHUNK, "max_rad": MAX_RAD}, run/f"{name}.pt.tmp")
    (run/f"{name}.pt.tmp").replace(run/f"{name}.pt")


def draw_plots(run, histories, sequence_metrics, act_history):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    for name, rows in histories.items():
        axes[0].plot([r["epoch"] for r in rows], [r["train_loss"] for r in rows], "--", label=name+" train")
        axes[0].plot([r["epoch"] for r in rows], [r["val_loss"] for r in rows], label=name+" val")
    axes[0].set(xlabel="Epoch", ylabel="MSE (rad²)", title="Sequence regression (regular + memory)")
    if histories:
        axes[0].legend(fontsize=7)
    if act_history:
        for key in ("train_mse", "val_mse", "train_kl"):
            axes[1].plot([r["epoch"] for r in act_history], [r[key] for r in act_history], label=key)
        axes[1].legend(fontsize=7)
    axes[1].set(xlabel="Epoch", title="ACT reconstruction MSE / KL (different scales)")
    for ax in axes:
        ax.grid(alpha=.2)
    fig.tight_layout(); fig.savefig(run/"learning_curves.png", dpi=140); plt.close(fig)
    if sequence_metrics:
        fig, ax = plt.subplots(figsize=(7, 4))
        x = np.arange(len(sequence_metrics))
        ax.bar(x-.18, [r["memory_long_mse"] for r in sequence_metrics], .36, label="full 32 states")
        ax.bar(x+.18, [r["memory_short_mse"] for r in sequence_metrics], .36, label="last 8 states only")
        ax.set(xticks=x, xticklabels=[r["model"] for r in sequence_metrics], ylabel="MSE (rad²)", title="Controlled delayed-cue diagnostic")
        ax.legend(); fig.tight_layout(); fig.savefig(run/"memory_comparison.png", dpi=140); plt.close(fig)


def save_action_plots(run, model, data, episodes, device):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for label, ax in enumerate(axes.flat):
        e = next(e for e in episodes if e["label"] == label)
        with Image.open(data/e["image"]) as image:
            x = vision.image_transform(False, 64)(image.convert("RGB")).unsqueeze(0).to(device)
        q = torch.from_numpy(e["states"][0]).unsqueeze(0).to(device)
        with torch.inference_mode():
            predicted = model(x, q)[0][0].cpu().numpy()
        for j in (0, 2, 4, 6, 8):
            line, = ax.plot(e["actions"][:CHUNK, j], label=f"joint {j} teacher")
            ax.plot(predicted[:, j], "--", color=line.get_color(), label=f"joint {j} predicted")
        ax.set(title=f"Opponent: {CLASSES[label]}", xlabel="Future step", ylabel="Command (rad)", ylim=(0, MAX_RAD))
        ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=6, ncol=2)
    fig.tight_layout(); fig.savefig(run/"action_chunks.png", dpi=140); plt.close(fig)
    with Image.open(data/episodes[0]["image"]) as source:
        pixels = source.convert("RGB").resize((64, 64))
    fig, ax = plt.subplots(figsize=(4, 4)); ax.imshow(pixels)
    for pos in np.arange(7.5, 64, 8):
        ax.axhline(pos, color="white", linewidth=.8); ax.axvline(pos, color="white", linewidth=.8)
    ax.set_title("64×64 image → 64 patches (8×8) + CLS"); ax.axis("off")
    fig.tight_layout(); fig.savefig(run/"vit_patches.png", dpi=140); plt.close(fig)


def report(run, meta, seq, act, complete=False):
    status = "완료" if complete else "진행 중"
    if meta["smoke"]:
        status = "합성 SMOKE — 제출 성능 아님 / "+status
    rows = "".join(f"| {m['model']} | {m['mse']:.6f} | {m['memory_long_mse']:.6f} | {m['memory_short_mse']:.6f} |\n" for m in seq)
    text = (f"# 문제 2 결과 — {status}\n\nrecord_id: DAPIER-2026-09-09-rps-virtual-hand\n\n"
            "| 모델 | 다음 상태 MSE (rad²) | 기억 과제 32시점 MSE | 마지막 8시점 MSE |\n|---|---:|---:|---:|\n"+rows)
    if act:
        text += f"\nACT prior z=0 chunk MSE: **{act['chunk_mse']:.6f} rad²**\n\nACT prior 샘플 chunk MSE: **{act['prior_sample_mse']:.6f} rad²**\n\nViT 보조 이미지 분류 Accuracy: **{act['vision_accuracy']:.2%}**\n"
    text += (f"\n나는 문제 1의 웹캠 이미지를 재사용하고 10관절 가상 손의 교사 궤적을 생성하여 실습한다.\n"
             f"- seed={meta['seed']}, Epoch={meta['epochs']}, batch={meta['batch_size']}, 장치={meta['device_name']}.\n"
             f"- 교사 버전: {meta.get('teacher_version', 'random-start-v1')}. 버전이 다른 MSE는 평가 궤적도 다르므로 직접 비교하지 않는다.\n"
             "- 상대 이미지의 패에 이기는 손 모양을 목표로 한다. 없음이면 현재 자세를 유지한다.\n"
             "- 단위 rad, 관절 범위 0~1.4. 엄지/검지/중지/약지/소지 각 2개 관절.\n"
             "- 10Hz 기구학 상태 갱신 q[t+1]=q[t]+0.35*(a[t]-q[t]). 물리 접촉이나 실물 검증은 수행하지 않는다.\n"
             "- 촬영 회차 단위의 문제 1 분할을 유지한다. 생성한 궤적의 이미지도 같은 분할에만 속한다.\n"
             "- RNN/LSTM/Transformer는 32개 과거 상태로 다음 상태를 예측한다. 미래 정답은 입력하지 않는다.\n"
             "- 장기기억 보조 실험은 초반 4개 상태에만 cue를 주고 뒤를 공통 중립값으로 둔 통제된 합성 수치 과제다. 일반 움직임 MSE와 분리한다.\n"
             "- 마지막 8시점 평가는 동일 모델의 입력을 잘라낸 진단이다. 별도로 짧은 입력에 최적화한 모델 비교가 아니므로 입력 분포 변화도 결과에 영향을 준다.\n"
             "- ViT는 64×64 이미지, 8×8 패치, Linear Projection, CLS·학습 가능한 위치 임베딩, 2층 Encoder를 사용한다.\n"
             "- ACT는 ViT 특징·현재 관절값·z로 미래 8개 목표 명령을 출력한다. posterior는 학습에만 정답 청크를 사용한다.\n"
             "- 학습 손실은 재구성 MSE + 0.001 KL + 0.2 이미지 보조 분류 CE. 원 ACT와 달리 직접 만든 ViT와 보조 분류기를 사용한다.\n"
             "- 검증 MSE로 가중치를 선택한다. 평가는 정답 액션 없이 z=0, 고정 seed의 prior 샘플링을 각각 수행한다.\n"
             "- 학습률 Adam 0.001(시퀀스), 0.0003(ACT), weight decay 0.0001. 이 설정의 우월성은 대조 실험 없이 주장하지 않는다.\n"
             "- MSE는 오프라인 예측 오차다. 이 수치만으로 게임 승률, 폐루프 수렴, 실제 손 구동 성공을 주장하지 않는다.\n")
    (run/"RESULTS.md").write_text(text, encoding="utf-8")
    figures = ""
    for name in ("learning_curves.png", "memory_comparison.png", "action_chunks.png", "vit_patches.png"):
        file = run/name
        if file.exists():
            figures += f'<figure><img alt="{name}" src="data:image/png;base64,{base64.b64encode(file.read_bytes()).decode()}"><figcaption>{name}</figcaption></figure>'
    document = f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>문제 2 가상 로봇손 결과</title>
<style>body{{max-width:1100px;margin:35px auto;padding:0 24px;font:18px/1.8 system-ui;color:#1d3045}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;background:#f0f4fa;padding:20px;font:16px/1.8 system-ui}}img{{max-width:100%;height:auto}}figure{{margin:24px 0}}@media print{{figure{{break-inside:avoid}}body{{font-size:11pt}}}}</style>
<h1>문제 2 · 가상 로봇손의 행동 예측</h1><p>{html.escape(status)}</p><pre>{html.escape(text)}</pre>{figures}</html>'''
    (run/"report.html").write_text(document, encoding="utf-8")


def train(args):
    if not 1 <= args.epochs <= 100 or not 1 <= args.batch_size <= 128:
        raise ValueError("Epoch 1~100, batch 1~128 범위여야 합니다.")
    torch.set_num_threads(4)
    torch.manual_seed(args.seed); random.seed(args.seed); np.random.seed(args.seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    manifest = vision.make_manifest(args.data, args.seed)
    if args.smoke:
        for split, rows in manifest["splits"].items():
            manifest["splits"][split] = [next(r for r in rows if r["label"] == label) for label in range(4)]
    run = args.run.resolve(); run.mkdir(parents=True, exist_ok=True)
    if (run/"meta.json").exists():
        raise ValueError("기록이 있는 실행 폴더입니다. 새 경로를 지정하세요.")
    meta = {"smoke": args.smoke, "complete": False, "seed": args.seed, "epochs": args.epochs,
            "teacher_version": "mixed-open-start-v2",
            "batch_size": args.batch_size, "created": datetime.now(timezone.utc).isoformat(),
            "device_name": torch.cuda.get_device_name(0) if device.type == "cuda" else "CPU",
            "torch_version": str(torch.__version__), "source_fingerprint": manifest["fingerprint"]}
    vision.write_json(run/"meta.json", meta); vision.write_json(run/"source_manifest.json", manifest)
    episodes = {split: make_episodes(rows, args.seed+i) for i, (split, rows) in enumerate(manifest["splits"].items())}
    histories, seq_metrics, act_history = {}, [], []
    for name in SEQUENCE_MODELS:
        torch.manual_seed(args.seed)
        model = StatePredictor(name).to(device)
        optimizer = torch.optim.Adam(model.parameters(), lr=.001, weight_decay=.0001)
        loaders = {s: DataLoader(SequenceDataset(es), batch_size=args.batch_size, shuffle=s == "train", num_workers=0,
                                generator=torch.Generator().manual_seed(args.seed)) for s, es in episodes.items()}
        history, best = [], math.inf
        for epoch in range(1, args.epochs+1):
            t = sequence_pass(model, loaders["train"], device, optimizer)
            v = sequence_pass(model, loaders["val"], device)
            history.append({"epoch": epoch, "train_loss": t["loss"], "val_loss": v["loss"]})
            vision.write_json(run/f"{name}_history.json", history)
            print(f"{name} {epoch}/{args.epochs}: train MSE={t['loss']:.6f}, val MSE={v['loss']:.6f}", flush=True)
            if v["loss"] < best:
                best = v["loss"]; save_checkpoint(run, name, model, meta)
        checkpoint = torch.load(run/f"{name}.pt", weights_only=True, map_location="cpu")
        model.load_state_dict(checkpoint["state_dict"])
        long = sequence_pass(model, loaders["test"], device)
        short = sequence_pass(model, loaders["test"], device, trim=8)
        seq_metrics.append({"model": name, "mse": long["mse"], "memory_long_mse": long["memory_mse"], "memory_short_mse": short["memory_mse"]})
        histories[name] = history
        vision.write_json(run/"sequence_metrics.json", seq_metrics)
        draw_plots(run, histories, seq_metrics, act_history)
        report(run, meta, seq_metrics, None)
        del model, optimizer, checkpoint
        if device.type == "cuda":
            torch.cuda.empty_cache()
    torch.manual_seed(args.seed)
    model = ActionCVAE().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=.0003, weight_decay=.0001)
    loaders = {s: DataLoader(ActionDataset(args.data, es, training=s == "train"), batch_size=args.batch_size,
                            shuffle=s == "train", num_workers=0, generator=torch.Generator().manual_seed(args.seed)) for s, es in episodes.items()}
    best = math.inf
    for epoch in range(1, args.epochs+1):
        t = action_pass(model, loaders["train"], device, optimizer)
        v = action_pass(model, loaders["val"], device)
        act_history.append({"epoch": epoch, "train_mse": t["mse"], "train_kl": t["kl"],
                            "train_classification_loss": t["classification_loss"], "val_mse": v["mse"], "val_vision_accuracy": v["vision_accuracy"]})
        vision.write_json(run/"act_history.json", act_history)
        print(f"ACT {epoch}/{args.epochs}: train MSE={t['mse']:.6f}, KL={t['kl']:.4f} | prior val MSE={v['mse']:.6f}, image acc={v['vision_accuracy']:.3f}", flush=True)
        if v["mse"] < best:
            best = v["mse"]; save_checkpoint(run, "act", model, meta)
    model.load_state_dict(torch.load(run/"act.pt", weights_only=True, map_location="cpu")["state_dict"])
    zero = action_pass(model, loaders["test"], device)
    torch.manual_seed(args.seed)
    sampled = action_pass(model, loaders["test"], device, sample=True)
    act_metrics = {"chunk_mse": zero["mse"], "prior_sample_mse": sampled["mse"], "vision_accuracy": zero["vision_accuracy"], "val_mse": best}
    vision.write_json(run/"act_metrics.json", act_metrics)
    draw_plots(run, histories, seq_metrics, act_history)
    model.eval(); save_action_plots(run, model, args.data, episodes["test"], device)
    meta["complete"] = True; vision.write_json(run/"meta.json", meta)
    report(run, meta, seq_metrics, act_metrics, complete=True)
    print(f"완료: {run/'report.html'}", flush=True)


class HandStudio(vision.Studio):
    """문제 1과 같은 서버·프로세스 슬롯을 사용해 무거운 학습을 동시에 시작하지 않는다."""
    def __init__(self, address, data, runs, hand_runs):
        super().__init__(address, data, runs)
        self.RequestHandlerClass = HandHandler
        self.hand_runs = hand_runs.resolve(); self.hand_runs.mkdir(parents=True, exist_ok=True)
        self.hand_model = self.hand_run = None

    def hand_status(self):
        ss = vision.sessions(self.data)
        completed = sum(s["complete"] for s in ss)
        runs, available = [], []
        for run in sorted(self.hand_runs.glob("*"), key=lambda p: p.stat().st_mtime, reverse=True):
            if not vision.ID_PATTERN.fullmatch(run.name) or not (run/"meta.json").exists():
                continue
            meta = vision.read_json(run/"meta.json")
            if meta.get("smoke"):
                continue
            metrics = vision.read_json(run/"sequence_metrics.json") if (run/"sequence_metrics.json").exists() else []
            act = vision.read_json(run/"act_metrics.json") if (run/"act_metrics.json").exists() else None
            runs.append({"id": run.name, "complete": meta["complete"], "sequence_metrics": metrics, "act_metrics": act,
                         "report_url": f"/hand-results/{run.name}/report.html" if (run/"report.html").exists() else None})
            if meta["complete"]:
                available.append({"run": run.name, "teacher_version": meta.get("teacher_version", "random-start-v1")})
        tail, error = "", None
        if self.run_id:
            log = self.hand_runs/self.run_id/"train.log"
            if not log.exists():
                log = self.runs/self.run_id/"train.log"
            if log.exists():
                tail = "\n".join(log.read_text(encoding="utf-8", errors="replace").splitlines()[-18:])
            if self.process is not None and self.process.poll() not in (None, 0):
                error = "학습이 중단되었습니다. 실행 로그를 확인하세요."
        return {"ready_to_train": completed >= 10 and completed % 10 == 0 and completed == len(ss), "source_sessions": completed,
                "training": {"running": self.running(), "log_tail": tail, "error": error}, "runs": runs, "models": available}


class HandHandler(vision.Handler):
    def do_GET(self):
        path = urlsplit(self.path).path
        if path not in ("/hand", "/hand.html", "/api/hand/status") and not path.startswith("/hand-results/"):
            return super().do_GET()
        try:
            self.local_request()
            if path in ("/hand", "/hand.html"):
                self.reply(200, (ROOT/"hand.html").read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/hand/status":
                with self.server.lock:
                    self.reply(200, self.server.hand_status())
            else:
                pieces = path.split("/")
                if len(pieces) != 4 or not vision.ID_PATTERN.fullmatch(pieces[2]) or pieces[3] != "report.html":
                    raise ValueError("잘못된 결과 경로입니다.")
                self.reply(200, (self.server.hand_runs/pieces[2]/pieces[3]).read_bytes(), "text/html; charset=utf-8")
        except ValueError as e:
            self.reply(400, {"error": str(e)})
        except OSError as e:
            self.reply(404, {"error": f"파일을 열 수 없습니다: {e}"})

    def do_POST(self):
        parsed = urlsplit(self.path)
        if not parsed.path.startswith("/api/hand/"):
            return super().do_POST()
        try:
            self.local_request()
            payload = vision.json.loads(self.body(limit=3_000_000))
            if not isinstance(payload, dict):
                raise ValueError("JSON 객체가 필요합니다.")
            with self.server.lock:
                self.require_idle()
                if parsed.path == "/api/hand/train":
                    self.start_hand_training(payload)
                elif parsed.path == "/api/hand/predict":
                    self.predict_hand(parse_qs(parsed.query), payload)
                else:
                    self.reply(404, {"error": "API가 없습니다."})
        except (ValueError, TypeError, KeyError) as e:
            self.reply(400, {"error": str(e)})
        except (OSError, RuntimeError) as e:
            self.reply(500, {"error": f"실행 실패: {e}"})

    def start_hand_training(self, payload):
        epochs = payload.get("epochs", 10)
        if type(epochs) is not int or not 1 <= epochs <= 100:
            raise ValueError("Epoch는 1~100 정수로 지정하세요.")
        vision.make_manifest(self.server.data)
        identifier = uuid.uuid4().hex
        run = self.server.hand_runs/identifier; run.mkdir()
        self.server.hand_model = self.server.hand_run = None
        self.server.cached_key = self.server.cached_model = self.server.cached_transform = None
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        command = [sys.executable, "-u", str(Path(__file__).resolve()), "train", "--data", str(self.server.data), "--run", str(run), "--epochs", str(epochs)]
        with (run/"train.log").open("w", encoding="utf-8") as log:
            self.server.process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        self.server.run_id = identifier
        self.reply(202, {"run_id": identifier})

    def predict_hand(self, params, payload):
        run = params.get("run", [""])[0]
        if not vision.ID_PATTERN.fullmatch(run):
            raise ValueError("학습을 완료한 실행을 선택하세요.")
        meta = vision.read_json(self.server.hand_runs/run/"meta.json")
        if not meta.get("complete") or meta.get("smoke"):
            raise ValueError("실제 사진을 사용해 학습을 완료한 모델이 필요합니다.")
        values = payload.get("joints")
        if not isinstance(values, list) or len(values) != JOINTS or any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= MAX_RAD for v in values):
            raise ValueError("현재 관절값은 0~1.4 rad 범위의 유한한 숫자 10개여야 합니다.")
        sample = payload.get("sample", False)
        if type(sample) is not bool:
            raise ValueError("sample은 boolean이어야 합니다.")
        encoded = payload.get("image", "")
        if not isinstance(encoded, str):
            raise ValueError("이미지 문자열이 필요합니다.")
        if encoded.startswith("data:image/jpeg;base64,"):
            encoded = encoded.split(",", 1)[1]
        image = vision.decode_image(base64.b64decode(encoded, validate=True))
        if self.server.hand_run != run:
            checkpoint = torch.load(self.server.hand_runs/run/"act.pt", map_location="cpu", weights_only=True)
            if (checkpoint.get("smoke") or checkpoint.get("classes") != list(CLASSES)
                    or checkpoint.get("chunk") != CHUNK or checkpoint.get("joints") != JOINTS
                    or checkpoint.get("max_rad") != MAX_RAD):
                raise ValueError("이 가상 손과 호환되지 않는 체크포인트입니다.")
            model = ActionCVAE(); model.load_state_dict(checkpoint["state_dict"])
            self.server.hand_model = model.to(self.server.device).eval(); self.server.hand_run = run
        x = vision.image_transform(False, 64)(image).unsqueeze(0).to(self.server.device)
        joints = torch.tensor([values], dtype=torch.float32, device=self.server.device)
        vision.sync(self.server.device); start = time.perf_counter()
        with torch.inference_mode():
            actions, _, _, logits = self.server.hand_model(x, joints, sample=sample)
            vision.sync(self.server.device); latency = (time.perf_counter()-start)*1000
            probabilities = logits.softmax(1)[0].cpu().tolist()
            action_values = actions[0].cpu().tolist()
        index = max(range(4), key=lambda i: probabilities[i])
        self.reply(200, {"actions": action_values, "label": CLASSES[index], "confidence": probabilities[index],
                         "probabilities": probabilities, "latency_ms": latency, "z_mode": "sample" if sample else "zero"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    t = sub.add_parser("train")
    t.add_argument("--data", type=Path, default=vision.ROOT/"data")
    t.add_argument("--run", type=Path, required=True)
    t.add_argument("--epochs", type=int, default=10)
    t.add_argument("--batch-size", type=int, default=64)
    t.add_argument("--seed", type=int, default=42)
    t.add_argument("--smoke", action="store_true")
    s = sub.add_parser("serve", help="문제 1과 문제 2를 함께 제공")
    s.add_argument("--port", type=int, default=8766)
    s.add_argument("--data", type=Path, default=vision.ROOT/"data")
    s.add_argument("--vision-runs", type=Path, default=vision.ROOT/"runs")
    s.add_argument("--runs", type=Path, default=ROOT/"runs")
    args = parser.parse_args()
    if args.command == "train":
        train(args); return
    torch.set_num_threads(4)
    server = HandStudio(("127.0.0.1", args.port), args.data, args.vision_runs, args.runs)
    def stop_server(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_server)
    print(f"문제 1 http://127.0.0.1:{args.port}/ | 문제 2 http://127.0.0.1:{args.port}/hand", flush=True)
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
