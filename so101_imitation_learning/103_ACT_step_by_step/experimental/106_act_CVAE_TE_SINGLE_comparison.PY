"""
CVAE Inference with TE-style Execution Comparison
==================================================
이전 TE 시각화(act_viz_TE.py)와 동일한 환경에서
CVAE 모델(cvae_policy.pt)을 사용하여 3가지 실행 방식을 비교:

  1. Single-Shot       : z 1회 샘플링 → 20 waypoint 전부 실행
  2. Receding Horizon  : k=5 스텝마다 z 재샘플링 → 앞 5개만 실행
  3. Temporal Ensemble : 매 스텝 z 재샘플링 → 지수 가중 평균

CVAE만의 특징:
  - 같은 obs에서도 z가 다르면 다른 chunk가 나옴 (stochastic)
  - Receding Horizon에서 매번 다른 z → 경로가 흔들림
  - Temporal Ensemble이 가중 평균으로 흔들림을 smoothing

추가 패널:
  - 4번째 패널: 같은 obs에서 5개 z를 샘플링한 다양한 경로 오버레이
    → CVAE의 multimodal 특성 시각화
"""

import numpy as np
import torch
import torch.nn as nn
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D   # noqa
import warnings
warnings.filterwarnings("ignore")

# ─────────────────────────────────────────
# 설정 (TE 코드와 완전 동일)
# ─────────────────────────────────────────
DEVICE        = "cuda" if torch.cuda.is_available() else "cpu"
CHUNK_SIZE    = 20
K_RH          = 5
TE_WINDOW     = 8
TE_LAMBDA     = 0.15
WORKSPACE_MIN = np.array([0.0, -0.3, 0.0])
WORKSPACE_MAX = np.array([0.5,  0.3, 0.4])

# CVAE 모델 경로 (MLP가 아닌 CVAE)
LATENT_DIM    = 32
MODEL_PATH    = "./cvae_policy.pt"
SAVE_GIF      = "./viz_CVAE_TE_comparison.gif"
SAVE_PNG      = "./viz_CVAE_TE_final_frame.png"


# ─────────────────────────────────────────
# CVAE 모델 정의 (act_cvae_full.py와 동일)
# ─────────────────────────────────────────
class Encoder(nn.Module):
    def __init__(self, obs_dim=6, chunk_dim=60, latent_dim=LATENT_DIM, hidden=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim + chunk_dim, hidden), nn.LayerNorm(hidden), nn.ReLU(),
            nn.Linear(hidden, hidden),              nn.LayerNorm(hidden), nn.ReLU(),
        )
        self.fc_mu = nn.Linear(hidden, latent_dim)
        self.fc_lv = nn.Linear(hidden, latent_dim)

    def forward(self, obs, chunk):
        h = self.net(torch.cat([obs, chunk], dim=-1))
        return self.fc_mu(h), self.fc_lv(h)


class Decoder(nn.Module):
    def __init__(self, obs_dim=6, latent_dim=LATENT_DIM, chunk_dim=60, hidden=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim + latent_dim, hidden), nn.LayerNorm(hidden), nn.ReLU(),
            nn.Linear(hidden, hidden),               nn.LayerNorm(hidden), nn.ReLU(),
            nn.Linear(hidden, hidden),               nn.LayerNorm(hidden), nn.ReLU(),
            nn.Linear(hidden, chunk_dim),
        )

    def forward(self, obs, z):
        return self.net(torch.cat([obs, z], dim=-1))


class CVAE(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = Encoder()
        self.decoder = Decoder()

    def reparametrize(self, mu, log_var):
        std = torch.exp(0.5 * log_var)
        eps = torch.randn_like(std)
        return mu + eps * std

    def forward(self, obs, chunk):
        mu, lv = self.encoder(obs, chunk)
        z      = self.reparametrize(mu, lv)
        pred   = self.decoder(obs, z)
        return pred, mu, lv

    # ── 핵심 추론 메서드 ──────────────────────────────────────
    @torch.no_grad()
    def predict(self, start: np.ndarray, target: np.ndarray) -> np.ndarray:
        """
        Single-shot 추론: z를 한 번 샘플링하여 20개 waypoint 생성.
        MLP의 predict와 동일한 인터페이스이지만, 호출할 때마다
        다른 z → 다른 chunk가 나옴 (stochastic).
        """
        self.eval()
        obs = torch.tensor(
            np.concatenate([start, target]), dtype=torch.float32
        ).unsqueeze(0).to(DEVICE)
        z = torch.randn(1, LATENT_DIM).to(DEVICE)          # ← CVAE 핵심: z 샘플링
        pred = self.decoder(obs, z)
        return pred.squeeze(0).cpu().numpy().reshape(CHUNK_SIZE, 3)

    @torch.no_grad()
    def predict_deterministic(self, start: np.ndarray, target: np.ndarray) -> np.ndarray:
        """
        z=0 (평균)으로 추론 — 결정적(deterministic) 경로.
        CVAE의 "평균 행동"을 보고 싶을 때 사용.
        """
        self.eval()
        obs = torch.tensor(
            np.concatenate([start, target]), dtype=torch.float32
        ).unsqueeze(0).to(DEVICE)
        z = torch.zeros(1, LATENT_DIM).to(DEVICE)          # z=0 → 평균 경로
        pred = self.decoder(obs, z)
        return pred.squeeze(0).cpu().numpy().reshape(CHUNK_SIZE, 3)

    @torch.no_grad()
    def sample_multiple(self, start: np.ndarray, target: np.ndarray,
                        n: int = 5) -> np.ndarray:
        """
        같은 obs에서 n개의 서로 다른 z를 샘플링 → n개의 다양한 경로.
        반환: (n, CHUNK_SIZE, 3)
        """
        self.eval()
        obs = torch.tensor(
            np.concatenate([start, target]), dtype=torch.float32
        ).unsqueeze(0).expand(n, -1).to(DEVICE)
        z = torch.randn(n, LATENT_DIM).to(DEVICE)
        pred = self.decoder(obs, z)
        return pred.cpu().numpy().reshape(n, CHUNK_SIZE, 3)


def load_model(path):
    ckpt  = torch.load(path, map_location=DEVICE, weights_only=False)
    model = CVAE()
    model.load_state_dict(ckpt["state_dict"])
    return model.to(DEVICE).eval()


# ─────────────────────────────────────────
# GT 궤적 (TE 코드와 동일: 고정 곡률 0.15)
# ─────────────────────────────────────────
def make_gt(start, target, n=CHUNK_SIZE, curv=0.15):
    t      = np.linspace(0, 1, n).reshape(-1, 1)
    linear = start + t * (target - start)
    d      = target - start
    perp   = np.cross(d, [0,0,1]) if abs(d[2]) < 1e-6 else np.cross(d, [1,0,0])
    nm     = np.linalg.norm(perp)
    perp   = perp / nm if nm > 1e-6 else np.array([0,0,1])
    return linear + curv * 4 * t * (1-t) * perp.reshape(1,3)


# ─────────────────────────────────────────
# 시뮬레이션 (TE 코드 구조 그대로, 모델만 CVAE)
# ─────────────────────────────────────────
def simulate_single_shot(model, start, target):
    """
    CVAE Single-Shot: z 1회 샘플링 → 20개 waypoint 전부 실행.
    호출마다 다른 경로가 나옴 (z가 stochastic).
    """
    chunk = model.predict(start, target)       # z ~ N(0,I) 1회
    steps = list(chunk)
    return np.array(steps), 1, [chunk]*len(steps)


def simulate_receding_horizon(model, start, target, k=K_RH):
    """
    CVAE Receding Horizon: k스텝마다 z 재샘플링.
    매번 다른 z → chunk가 불연속적으로 변함.
    """
    steps, replan_idx, active_chunks = [], [], []
    n_infer, current_pos = 0, start.copy()
    MAX = CHUNK_SIZE * 4

    while len(steps) < MAX:
        replan_idx.append(len(steps))
        chunk   = model.predict(current_pos, target)   # 새 z 샘플링
        n_infer += 1
        for i in range(k):
            if len(steps) >= MAX: break
            wp = chunk[i]
            steps.append(wp.copy())
            active_chunks.append(chunk.copy())
            current_pos = wp.copy()
            if np.linalg.norm(current_pos - target) < 0.03:
                return np.array(steps), n_infer, replan_idx, active_chunks
    return np.array(steps), n_infer, replan_idx, active_chunks


def simulate_temporal_ensemble(model, start, target,
                                window=TE_WINDOW, lam=TE_LAMBDA):
    """
    CVAE Temporal Ensemble: 매 스텝 z 재샘플링 + 지수 가중 평균.
    CVAE에서 TE가 특히 의미있는 이유:
      - 매 스텝 다른 z → 각 chunk가 서로 다른 경로 제안
      - TE가중 평균이 이 stochastic 제안들을 smoothing
    """
    steps          = []
    chunk_buffer   = []
    weight_history = []
    n_infer        = 0
    current_pos    = start.copy()
    MAX            = CHUNK_SIZE * 4

    while len(steps) < MAX:
        # 매 스텝 새 z 샘플링
        new_chunk   = model.predict(current_pos, target)
        n_infer    += 1

        chunk_buffer.insert(0, new_chunk)
        if len(chunk_buffer) > window:
            chunk_buffer.pop()

        n_buf   = len(chunk_buffer)
        raw_w   = np.array([np.exp(-i * lam) for i in range(n_buf)])
        weights = raw_w / raw_w.sum()

        weighted_wp = np.zeros(3)
        for i, (ch, w) in enumerate(zip(chunk_buffer, weights)):
            if i < len(ch):
                weighted_wp += w * ch[i]

        weight_history.append(weights.copy())
        steps.append(weighted_wp.copy())
        current_pos = weighted_wp.copy()

        if np.linalg.norm(current_pos - target) < 0.03:
            break

    return np.array(steps), n_infer, chunk_buffer, weight_history


# ─────────────────────────────────────────
# 누적 RMSE
# ─────────────────────────────────────────
def cumulative_rmse(pos_arr, ref_arr):
    rmse = []
    for i in range(1, len(pos_arr)+1):
        t_idx = np.linspace(0, len(ref_arr)-1, i).astype(int)
        diff  = pos_arr[:i] - ref_arr[t_idx]
        rmse.append(float(np.sqrt(np.mean(diff**2))))
    return rmse


# ─────────────────────────────────────────
# 메인
# ─────────────────────────────────────────
def main():
    print("CVAE 모델 로드 중...")
    model = load_model(MODEL_PATH)
    print(f"  모델: {MODEL_PATH}")

    # TE 코드와 동일한 시드/시나리오
    rng    = np.random.default_rng(7)
    start  = rng.uniform(WORKSPACE_MIN + 0.05, WORKSPACE_MAX - 0.05)
    target = rng.uniform(WORKSPACE_MIN + 0.05, WORKSPACE_MAX - 0.05)
    while np.linalg.norm(target - start) < 0.25:
        target = rng.uniform(WORKSPACE_MIN + 0.05, WORKSPACE_MAX - 0.05)

    print(f"  시작점: {start.round(3)}")
    print(f"  목표점: {target.round(3)}")
    gt = make_gt(start, target)

    # ── CVAE 특유: 같은 obs에서 5개 경로 샘플링 ──────────────
    print("\nCVAE multimodal 샘플링 (같은 start→target, 5개 z)...")
    multi_trajs = model.sample_multiple(start, target, n=5)
    for i, tr in enumerate(multi_trajs):
        rmse = np.sqrt(np.mean((tr - gt)**2))
        print(f"  z샘플 {i+1}: RMSE={rmse*100:.1f}cm, "
              f"final={tr[-1].round(3)}")

    # ── z=0 (결정적 평균 경로) ────────────────────────────────
    det_traj = model.predict_deterministic(start, target)
    det_rmse = np.sqrt(np.mean((det_traj - gt)**2))
    print(f"\nz=0 (평균 경로): RMSE={det_rmse*100:.1f}cm")

    # ── 시뮬레이션 ──────────────────────────────────────────
    print("\nSingle-Shot 시뮬레이션 (CVAE)...")
    ss_pos, ss_ninf, _            = simulate_single_shot(model, start, target)

    print("Receding Horizon 시뮬레이션 (CVAE)...")
    rh_pos, rh_ninf, rh_replan, _ = simulate_receding_horizon(model, start, target)

    print("Temporal Ensemble 시뮬레이션 (CVAE)...")
    te_pos, te_ninf, _, te_weights = simulate_temporal_ensemble(model, start, target)

    print(f"\n  Single-Shot      : 추론 {ss_ninf:2d}회, {len(ss_pos):2d}스텝")
    print(f"  Receding(k={K_RH})    : 추론 {rh_ninf:2d}회, {len(rh_pos):2d}스텝")
    print(f"  Temporal Ensemble: 추론 {te_ninf:2d}회, {len(te_pos):2d}스텝")

    ss_rmse = cumulative_rmse(ss_pos, gt)
    rh_rmse = cumulative_rmse(rh_pos, gt)
    te_rmse = cumulative_rmse(te_pos, gt)

    final_ss = ss_rmse[-1]
    final_rh = rh_rmse[-1]
    final_te = te_rmse[-1]
    print(f"\n  최종 누적 RMSE:")
    print(f"    Single-Shot      : {final_ss:.4f} m")
    print(f"    Receding Horizon : {final_rh:.4f} m")
    print(f"    Temporal Ensemble: {final_te:.4f} m")

    # ─────────────────────────────────────────
    # 레이아웃 (TE 코드 구조 + CVAE multimodal 패널 추가)
    # ─────────────────────────────────────────
    fig = plt.figure(figsize=(18, 9), facecolor="#0d0d0d")
    fig.suptitle(
        "CVAE Inference: Single-Shot  vs  Receding Horizon  vs  Temporal Ensemble\n"
        "(stochastic z → 매 추론마다 다른 chunk)",
        color="white", fontsize=12, fontweight="bold", y=0.97
    )

    gs = gridspec.GridSpec(
        2, 5,
        left=0.02, right=0.98, top=0.88, bottom=0.08,
        wspace=0.25, hspace=0.40,
        width_ratios=[2, 2, 2, 2, 1.4]
    )

    # ── 3D 축 공통 설정 ────────────────────────────────────
    def make_ax(subplot_spec, title, color):
        ax = fig.add_subplot(subplot_spec, projection='3d')
        ax.set_facecolor("#111111")
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor("#2a2a2a")
        ax.set_xlim(WORKSPACE_MIN[0], WORKSPACE_MAX[0])
        ax.set_ylim(WORKSPACE_MIN[1], WORKSPACE_MAX[1])
        ax.set_zlim(WORKSPACE_MIN[2], WORKSPACE_MAX[2])
        for axis, label in zip([ax.set_xlabel, ax.set_ylabel, ax.set_zlabel],
                               ["x","y","z"]):
            axis(label, color="#555", labelpad=2, fontsize=6)
        ax.tick_params(colors="#333", labelsize=5)
        ax.set_title(title, color=color, fontsize=8, pad=4, fontweight="bold")
        ax.plot(gt[:,0], gt[:,1], gt[:,2],
                '--', color="#444", lw=1.0, alpha=0.5)
        ax.scatter(*start,  color="#44ff44", s=60,  marker="s", zorder=5)
        ax.scatter(*target, color="#ff4444", s=100, marker="*", zorder=5)
        return ax

    ax_ss  = make_ax(gs[:, 0], f"Single-Shot\n(추론 {ss_ninf}회, z 1회)", "#00ccff")
    ax_rh  = make_ax(gs[:, 1], f"Receding(k={K_RH})\n(추론 {rh_ninf}회)", "#ff9944")
    ax_te  = make_ax(gs[:, 2], f"Temp. Ensemble\n(추론 {te_ninf}회)", "#cc88ff")
    ax_mul = make_ax(gs[:, 3], "Multimodal\n(5 z samples)", "#f4a261")

    # ── multimodal 패널: 5개 경로 미리 그리기 ────────────────
    cmap_multi = ['#2dc5a2','#61afef','#e5c07b','#c678dd','#e06c75']
    for i, tr in enumerate(multi_trajs):
        ax_mul.plot(tr[:,0], tr[:,1], tr[:,2],
                     ':', color=cmap_multi[i], lw=0.8, alpha=0.4)
        ax_mul.scatter(tr[-1,0], tr[-1,1], tr[-1,2],
                        color=cmap_multi[i], s=15, zorder=4)
    # z=0 결정적 경로
    ax_mul.plot(det_traj[:,0], det_traj[:,1], det_traj[:,2],
                '--', color='white', lw=1.5, alpha=0.6, label='z=0 (mean)')

    # ── 오른쪽 패널 ─────────────────────────────────────────
    ax_rmse = fig.add_subplot(gs[0, 4])
    ax_rmse.set_facecolor("#111111")
    ax_rmse.set_title("누적 RMSE", color="white", fontsize=8)
    ax_rmse.set_xlabel("스텝", color="#888", fontsize=7)
    ax_rmse.set_ylabel("RMSE (m)", color="#888", fontsize=7)
    ax_rmse.tick_params(colors="#555", labelsize=6)
    ax_rmse.spines[:].set_edgecolor("#333")
    max_step = max(len(ss_rmse), len(rh_rmse), len(te_rmse))
    max_val  = max(max(ss_rmse), max(rh_rmse), max(te_rmse))
    ax_rmse.set_xlim(0, max_step+1)
    ax_rmse.set_ylim(0, max_val * 1.15)

    ax_rmse.plot(range(1, len(ss_rmse)+1), ss_rmse,
                 color="#00ccff", lw=0.5, alpha=0.12)
    ax_rmse.plot(range(1, len(rh_rmse)+1), rh_rmse,
                 color="#ff9944", lw=0.5, alpha=0.12)
    ax_rmse.plot(range(1, len(te_rmse)+1), te_rmse,
                 color="#cc88ff", lw=0.5, alpha=0.12)

    line_ss_r, = ax_rmse.plot([], [], color="#00ccff", lw=1.5,
                               label=f"SS ({ss_ninf}회)")
    line_rh_r, = ax_rmse.plot([], [], color="#ff9944", lw=1.5,
                               label=f"RH ({rh_ninf}회)")
    line_te_r, = ax_rmse.plot([], [], color="#cc88ff", lw=2.0,
                               label=f"TE ({te_ninf}회)")
    ax_rmse.legend(fontsize=6, facecolor="#1a1a1a",
                   edgecolor="#444", labelcolor="white")

    for ri in rh_replan[1:]:
        if ri < len(rh_rmse):
            ax_rmse.axvline(x=ri, color="#ff9944", lw=0.3, alpha=0.2, ls=":")

    # TE 가중치 바
    ax_w = fig.add_subplot(gs[1, 4])
    ax_w.set_facecolor("#111111")
    ax_w.set_title(f"TE 가중치 (W={TE_WINDOW})", color="white", fontsize=8)
    ax_w.set_xlabel("chunk idx (0=최신)", color="#888", fontsize=6)
    ax_w.set_ylabel("가중치", color="#888", fontsize=6)
    ax_w.tick_params(colors="#555", labelsize=6)
    ax_w.spines[:].set_edgecolor("#333")
    ax_w.set_xlim(-0.5, TE_WINDOW - 0.5)
    ax_w.set_ylim(0, 1.05)

    ref_w_raw = np.array([np.exp(-i * TE_LAMBDA) for i in range(TE_WINDOW)])
    ref_w     = ref_w_raw / ref_w_raw.sum()
    ax_w.bar(range(TE_WINDOW), ref_w, color="#555", alpha=0.25, width=0.5)
    bars_w = ax_w.bar(range(TE_WINDOW), [0]*TE_WINDOW,
                      color="#cc88ff", alpha=0.85, width=0.5)
    w_text = ax_w.text(TE_WINDOW/2, 0.85, "",
                       ha="center", fontsize=7, color="#cc88ff")

    # ── 이동 요소 ───────────────────────────────────────────
    def make_elements(ax, color):
        traj, = ax.plot([], [], [], '-',  color=color, lw=2.0, alpha=0.9)
        grip, = ax.plot([], [], [], 'o',  color="white", markersize=7,
                        markeredgecolor=color, markeredgewidth=2, zorder=10)
        info  = ax.text2D(0.02, 0.97, "", transform=ax.transAxes,
                          color="white", fontsize=6, va="top",
                          bbox=dict(boxstyle="round,pad=0.2",
                                    facecolor="#000000bb", edgecolor="#444"))
        return traj, grip, info

    ss_traj, ss_grip, ss_info = make_elements(ax_ss, "#00ccff")
    rh_traj, rh_grip, rh_info = make_elements(ax_rh, "#ff9944")
    te_traj, te_grip, te_info = make_elements(ax_te, "#cc88ff")

    # multimodal 패널용 이동 요소
    mul_trajs_anim = []
    mul_grips_anim = []
    for i in range(5):
        tr, = ax_mul.plot([], [], [], '-', color=cmap_multi[i], lw=1.5, alpha=0.7)
        gr, = ax_mul.plot([], [], [], 'o', color=cmap_multi[i], markersize=5,
                          zorder=8)
        mul_trajs_anim.append(tr)
        mul_grips_anim.append(gr)
    mul_info = ax_mul.text2D(0.02, 0.97, "", transform=ax_mul.transAxes,
                              color="white", fontsize=6, va="top",
                              bbox=dict(boxstyle="round,pad=0.2",
                                        facecolor="#000000bb", edgecolor="#444"))

    rh_replan_txt = ax_rh.text2D(0.50, 0.88, "", transform=ax_rh.transAxes,
                                  color="#ff9944", fontsize=8, ha="center",
                                  fontweight="bold")
    te_avg_txt = ax_te.text2D(0.50, 0.88, "", transform=ax_te.transAxes,
                               color="#cc88ff", fontsize=8, ha="center",
                               fontweight="bold")

    edu = (
        "💡 CVAE 추론: z~N(0,I)를 샘플링 → 같은 obs라도 매번 다른 chunk  |  "
        "Single-Shot: z 1회  |  RH: k=5마다 z 재샘플  |  "
        f"TE: 매 스텝 z 재샘플 + W={TE_WINDOW}개 가중평균  |  "
        "λ=0.15"
    )
    fig.text(0.5, 0.02, edu, ha="center", va="bottom", fontsize=7.5,
             color="#aaaaaa",
             bbox=dict(boxstyle="round,pad=0.3",
                       facecolor="#1a1a1a", edgecolor="#444"))

    # ─────────────────────────────────────────
    # 애니메이션
    # ─────────────────────────────────────────
    PAUSE    = 6
    n_ss     = len(ss_pos)
    n_rh     = len(rh_pos)
    n_te     = len(te_pos)
    n_common = max(n_ss, n_rh, n_te)
    total    = PAUSE + n_common + PAUSE

    def update(frame):
        fi = max(0, min(frame - PAUSE, n_common - 1))

        si = min(fi, n_ss - 1)
        ri = min(fi, n_rh - 1)
        ti = min(fi, n_te - 1)

        # Single-Shot
        ss_traj.set_data(ss_pos[:si+1,0], ss_pos[:si+1,1])
        ss_traj.set_3d_properties(ss_pos[:si+1,2])
        ss_grip.set_data([ss_pos[si,0]], [ss_pos[si,1]])
        ss_grip.set_3d_properties([ss_pos[si,2]])
        ss_info.set_text(
            f"Step {si+1:2d}/{n_ss}\n"
            f"dist: {np.linalg.norm(ss_pos[si]-target):.3f}m\n"
            f"z샘플: 1회 고정"
        )

        # Receding Horizon
        rh_traj.set_data(rh_pos[:ri+1,0], rh_pos[:ri+1,1])
        rh_traj.set_3d_properties(rh_pos[:ri+1,2])
        rh_grip.set_data([rh_pos[ri,0]], [rh_pos[ri,1]])
        rh_grip.set_3d_properties([rh_pos[ri,2]])
        infer_rh = sum(1 for r in rh_replan if r <= ri)
        rh_info.set_text(
            f"Step {ri+1:2d}/{n_rh}\n"
            f"dist: {np.linalg.norm(rh_pos[ri]-target):.3f}m\n"
            f"z샘플: {infer_rh}회"
        )
        rh_replan_txt.set_text("🔄 z 재샘플!" if (ri in rh_replan and fi > 0) else "")

        # Temporal Ensemble
        te_traj.set_data(te_pos[:ti+1,0], te_pos[:ti+1,1])
        te_traj.set_3d_properties(te_pos[:ti+1,2])
        te_grip.set_data([te_pos[ti,0]], [te_pos[ti,1]])
        te_grip.set_3d_properties([te_pos[ti,2]])
        te_info.set_text(
            f"Step {ti+1:2d}/{n_te}\n"
            f"dist: {np.linalg.norm(te_pos[ti]-target):.3f}m\n"
            f"z샘플: {ti+1}회"
        )
        te_avg_txt.set_text("⚖ 가중평균" if fi > 0 else "")

        # Multimodal: 5개 경로 동시 진행
        for i in range(5):
            mi = min(fi, CHUNK_SIZE - 1)
            tr = multi_trajs[i]
            mul_trajs_anim[i].set_data(tr[:mi+1,0], tr[:mi+1,1])
            mul_trajs_anim[i].set_3d_properties(tr[:mi+1,2])
            mul_grips_anim[i].set_data([tr[mi,0]], [tr[mi,1]])
            mul_grips_anim[i].set_3d_properties([tr[mi,2]])
        mul_info.set_text(
            f"5개 z 샘플\n"
            f"Step {min(fi,CHUNK_SIZE-1)+1}/{CHUNK_SIZE}\n"
            f"같은 obs, 다른 경로"
        )

        # 가중치 바
        if ti < len(te_weights):
            w = te_weights[ti]
            n_w = len(w)
            for bi, bar in enumerate(bars_w):
                bar.set_height(w[bi] if bi < n_w else 0)
            w_text.set_text(f"w₀={w[0]:.3f}")

        # RMSE
        line_ss_r.set_data(range(1, si+2), ss_rmse[:si+1])
        line_rh_r.set_data(range(1, ri+2), rh_rmse[:ri+1])
        line_te_r.set_data(range(1, ti+2), te_rmse[:ti+1])

        return (ss_traj, ss_grip, rh_traj, rh_grip, te_traj, te_grip,
                line_ss_r, line_rh_r, line_te_r,
                rh_replan_txt, te_avg_txt, ss_info, rh_info, te_info,
                *bars_w, *mul_trajs_anim, *mul_grips_anim, mul_info)

    anim = FuncAnimation(fig, update, frames=total,
                         interval=170, blit=False, repeat=True)

    print("\nGIF 저장 중...")
    anim.save(SAVE_GIF, writer="pillow", fps=7, dpi=100)
    print(f"[saved] {SAVE_GIF}")

    update(PAUSE + n_common - 1)
    plt.savefig(SAVE_PNG, dpi=140, facecolor=fig.get_facecolor())
    print(f"[saved] {SAVE_PNG}")

    # ── 최종 수치 요약 ──────────────────────────────────────
    print("\n" + "="*60)
    print("  CVAE 추론 결과 요약")
    print("="*60)
    print(f"  방식               추론횟수  스텝수  최종RMSE")
    print(f"  Single-Shot        {ss_ninf:5d}회  {n_ss:4d}개  {final_ss:.4f} m")
    print(f"  Receding(k={K_RH})      {rh_ninf:5d}회  {n_rh:4d}개  {final_rh:.4f} m")
    print(f"  Temporal Ensemble  {te_ninf:5d}회  {n_te:4d}개  {final_te:.4f} m")
    print(f"  z=0 (결정적 평균)      1회  {CHUNK_SIZE:4d}개  {det_rmse:.4f} m")
    print("="*60)

    # ── MLP vs CVAE 비교 표 출력 ─────────────────────────────
    print("\n  [참고] 이전 MLP TE 결과와 비교")
    print("  ─────────────────────────────────────")
    print("  모델     | SS RMSE  | RH RMSE  | TE RMSE")
    print(f"  MLP      |  0.0076  |  0.1428  |  0.1455  (이전 결과)")
    print(f"  CVAE     |  {final_ss:.4f}  |  {final_rh:.4f}  |  {final_te:.4f}  (이번 결과)")
    print("  ─────────────────────────────────────")
    print("  ※ CVAE는 stochastic하므로 RMSE가 MLP보다 높음.")
    print("     대신 같은 obs에서 다양한 경로 생성 가능.")


if __name__ == "__main__":
    main()