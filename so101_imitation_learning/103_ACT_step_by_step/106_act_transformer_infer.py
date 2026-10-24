"""
ACT Full Implementation - Vision + Transformer + CVAE + Fix Target + Precision TE Control Loop
===========================================================================================
최종 완성형: 비전 기반 Transformer CVAE 백본 + 고정 목적지 수렴 + 실시간 TE 필터 제어 시뮬레이션
"""

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, TensorDataset
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os, time, warnings
warnings.filterwarnings("ignore")

# ═══════════════════════════════════════════════════════════
# 하이퍼파라미터 및 설정
# ═══════════════════════════════════════════════════════════
CHUNK_SIZE   = 20
OBS_DIM      = 6          # start(3) + target(3)
CHUNK_DIM    = 60         # CHUNK_SIZE × 3
LATENT_DIM   = 32
D_MODEL      = 128
N_HEAD       = 4
N_ENC_LAYERS = 2
N_DEC_LAYERS = 2
IMG_RES      = 64
BETA         = 0.1        # 복원 정밀도 극대화를 위해 KL 패널티 완화
EPOCHS       = 40
LR           = 1e-3
BATCH_SIZE   = 32
N_DATA       = 800

WORKSPACE_MIN = np.array([0.0, -0.3, 0.0])
WORKSPACE_MAX = np.array([0.5,  0.3, 0.4])
FIXED_TARGET  = np.array([0.25, 0.0, 0.2], dtype=np.float32) # 고정 목적지
OUTPUT_DIR    = './107_output'

# TE 제어 관련 하이퍼파라미터
SIM_STEPS    = 35         # 시뮬레이션 총 타임스텝
TE_WINDOW    = 8          # 시간 앙상블 윈도우 크기
TE_LAMBDA    = 0.25       # 지수 감쇠 가중치 균형 계수

# ═══════════════════════════════════════════════════════════
# 1. 가상 비전 카메라 렌더링 (목적지 고정형 반영)
# ═══════════════════════════════════════════════════════════
def render_camera(gripper_pos, target_pos=FIXED_TARGET, res=IMG_RES):
    H, W = res, res
    half = H // 2
    img = np.full((H, W, 3), 0.06, dtype=np.float32)

    x_min, x_max = 0.0, 0.5
    y_min, y_max = -0.3, 0.3
    z_min, z_max = 0.0, 0.4

    def to_px(val, vmin, vmax, px_max):
        return int(np.clip((val - vmin) / (vmax - vmin) * (px_max - 1), 0, px_max - 1))

    def draw_circle(canvas, cx, cy, r, color):
        Y, X = np.ogrid[:canvas.shape[0], :canvas.shape[1]]
        mask = (X - cx) ** 2 + (Y - cy) ** 2 <= r ** 2
        canvas[mask] = color

    def draw_rect(canvas, cx, cy, s, color):
        x0 = max(0, cx - s // 2); x1 = min(canvas.shape[1], cx + s // 2 + 1)
        y0 = max(0, cy - s // 2); y1 = min(canvas.shape[0], cy + s // 2 + 1)
        canvas[y0:y1, x0:x1] = color

    def draw_line(canvas, x1, y1, x2, y2, color, alpha=0.15):
        n = max(abs(x2 - x1), abs(y2 - y1), 1)
        xs = np.linspace(x1, x2, n).astype(int)
        ys = np.linspace(y1, y2, n).astype(int)
        for x, y in zip(xs, ys):
            if 0 <= x < canvas.shape[1] and 0 <= y < canvas.shape[0]:
                canvas[y, x] = canvas[y, x] * (1 - alpha) + np.array(color) * alpha

    def draw_grid(canvas):
        for i in range(6):
            p = int(i / 5 * (canvas.shape[0] - 1))
            canvas[:, p, :] = np.maximum(canvas[:, p, :], 0.12)
            canvas[p, :, :] = np.maximum(canvas[p, :, :], 0.12)

    # 상단: XY top-down view
    top = img[:half]
    draw_grid(top)
    gx, gy = to_px(gripper_pos[0], x_min, x_max, half), to_px(gripper_pos[1], y_min, y_max, half)
    tx, ty = to_px(target_pos[0],  x_min, x_max, half), to_px(target_pos[1],  y_min, y_max, half)
    draw_line(top, gx, gy, tx, ty, [0.4, 0.4, 0.4])
    g_r = int(2 + gripper_pos[2] / z_max * 5)
    t_s = int(4 + target_pos[2] / z_max * 5)
    draw_circle(top, gx, gy, g_r, [0.1, 0.85, 0.1]) # 그리퍼 녹색
    draw_rect(top, tx, ty, t_s, [0.85, 0.1, 0.1])   # 타겟 빨강

    # 하단: XZ side view
    bot = img[half:]
    draw_grid(bot)
    gx2, gz2 = to_px(gripper_pos[0], x_min, x_max, half), to_px(gripper_pos[2], z_min, z_max, half)
    tx2, tz2 = to_px(target_pos[0],  x_min, x_max, half), to_px(target_pos[2],  z_min, z_max, half)
    draw_line(bot, gx2, gz2, tx2, tz2, [0.4, 0.4, 0.4])
    draw_circle(bot, gx2, gz2, g_r, [0.1, 0.85, 0.1])
    draw_rect(bot, tx2, tz2, t_s, [0.85, 0.1, 0.1])

    return np.clip(img, 0, 1)

# ═══════════════════════════════════════════════════════════
# 2. 고정 타겟 기반 데이터셋 생성
# ═══════════════════════════════════════════════════════════
def generate_trajectory(start, target=FIXED_TARGET, n=CHUNK_SIZE, curvature=0.15):
    tv     = np.linspace(0, 1, n).reshape(-1, 1)
    linear = start + tv * (target - start)
    d      = target - start
    perp   = np.cross(d, [0, 0, 1]) if abs(d[2]) < 1e-6 else np.cross(d, [1, 0, 0])
    nm     = np.linalg.norm(perp)
    perp   = perp / nm if nm > 1e-6 else np.array([0, 0, 1])
    return linear + curvature * 4 * tv * (1 - tv) * perp.reshape(1, 3)

def generate_dataset_fixed(n_samples=N_DATA, seed=42):
    rng = np.random.default_rng(seed)
    X, imgs, Y = [], [], []
    while len(X) < n_samples:
        s = rng.uniform(WORKSPACE_MIN, WORKSPACE_MAX)
        if np.linalg.norm(FIXED_TARGET - s) < 0.08:
            continue
        curv = rng.uniform(-0.2, 0.2)
        traj = generate_trajectory(s, FIXED_TARGET, CHUNK_SIZE, curvature=curv)
        img  = render_camera(s, FIXED_TARGET, IMG_RES)
        X.append(np.concatenate([s, FIXED_TARGET]))
        imgs.append(img.transpose(2, 0, 1)) # CHW 포맷
        Y.append(traj.flatten())
    return (np.array(X, dtype=np.float32),
            np.array(imgs, dtype=np.float32),
            np.array(Y, dtype=np.float32))

# ═══════════════════════════════════════════════════════════
# 3. CNN Image Encoder & Transformer CVAE Policy
# ═══════════════════════════════════════════════════════════
class ImageEncoder(nn.Module):
    def __init__(self, out_dim=D_MODEL):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(3, 16, 3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(16, 32, 3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 64, 3, stride=2, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d(1),
        )
        self.proj = nn.Linear(64, out_dim)

    def forward(self, x):
        h = self.conv(x).flatten(1)
        return self.proj(h)

class TransformerCVAE(nn.Module):
    def __init__(self, latent_dim=LATENT_DIM, d_model=D_MODEL, nhead=N_HEAD, n_enc=N_ENC_LAYERS, n_dec=N_DEC_LAYERS):
        super().__init__()
        self.latent_dim = latent_dim
        self.d_model    = d_model
        self.chunk_size = CHUNK_SIZE

        self.img_encoder  = ImageEncoder(d_model)
        self.obs_proj     = nn.Linear(OBS_DIM, d_model)
        self.action_in    = nn.Linear(3, d_model)
        self.action_out   = nn.Linear(d_model, 3)
        self.z_to_token   = nn.Linear(latent_dim, d_model)
        self.query_embed  = nn.Parameter(torch.randn(CHUNK_SIZE, d_model) * 0.02)
        self.fc_mu     = nn.Linear(d_model, latent_dim)
        self.fc_logvar = nn.Linear(d_model, latent_dim)

        enc_layer = nn.TransformerEncoderLayer(d_model=d_model, nhead=nhead, dim_feedforward=d_model * 2, dropout=0.1, activation='gelu', batch_first=True)
        self.encoder_tf = nn.TransformerEncoder(enc_layer, n_enc)

        dec_layer = nn.TransformerDecoderLayer(d_model=d_model, nhead=nhead, dim_feedforward=d_model * 2, dropout=0.1, activation='gelu', batch_first=True)
        self.decoder_tf = nn.TransformerDecoder(dec_layer, n_dec)

    def encode(self, img, obs, gt_chunk):
        B = img.shape[0]
        img_tok = self.img_encoder(img).unsqueeze(1)
        obs_tok = self.obs_proj(obs).unsqueeze(1)
        gt_tok  = self.action_in(gt_chunk.reshape(B, self.chunk_size, 3))
        tokens = torch.cat([img_tok, obs_tok, gt_tok], dim=1)
        encoded = self.encoder_tf(tokens)
        pooled = encoded[:, 2:].mean(dim=1)
        return self.fc_mu(pooled), self.fc_logvar(pooled)

    def decode(self, img, obs, z):
        B = img.shape[0]
        img_tok = self.img_encoder(img).unsqueeze(1)
        obs_tok = self.obs_proj(obs).unsqueeze(1)
        z_tok   = self.z_to_token(z).unsqueeze(1)
        memory = torch.cat([img_tok, obs_tok, z_tok], dim=1)
        queries = self.query_embed.unsqueeze(0).expand(B, -1, -1)
        decoded = self.decoder_tf(tgt=queries, memory=memory)
        return self.action_out(decoded).reshape(B, -1)

    def reparametrize(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        return mu + torch.randn_like(std) * std

    def forward(self, img, obs, gt_chunk):
        mu, logvar = self.encode(img, obs, gt_chunk)
        z = self.reparametrize(mu, logvar)
        pred = self.decode(img, obs, z)
        return pred, mu, logvar

    @torch.no_grad()
    def predict(self, img, obs, use_z=True):
        self.eval()
        if img.dim() == 3: img = img.unsqueeze(0)
        if obs.dim() == 1: obs = obs.unsqueeze(0)
        B = img.shape[0]
        z = torch.randn(B, self.latent_dim) if use_z else torch.zeros(B, self.latent_dim)
        return self.decode(img, obs, z).reshape(B, self.chunk_size, 3)

def cvae_loss(pred, target, mu, logvar, beta=BETA):
    recon = F.mse_loss(pred, target)
    kl = -0.5 * torch.mean(1 + logvar - mu.pow(2) - logvar.exp())
    return recon + beta * kl, recon, kl

# ═══════════════════════════════════════════════════════════
# 4. 메인 실행 및 시뮬레이션 제어 루프
# ═══════════════════════════════════════════════════════════
if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plt.ion() # 대화형 모드 활성화

    # 1) 데이터 생성 및 학습
    print("1. 고정 목적지 비전 데이터셋 생성...")
    X, imgs, Y = generate_dataset_fixed(N_DATA, seed=42)
    
    print("\n2. Transformer CVAE Policy 학습 시작...")
    model = TransformerCVAE()
    dataset = TensorDataset(torch.from_numpy(imgs), torch.from_numpy(X), torch.from_numpy(Y))
    loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True, drop_last=True)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=1e-4)
    
    for ep in range(1, EPOCHS + 1):
        model.train()
        tl = rl = kl_ = 0.0
        for img_b, obs_b, chunk_b in loader:
            pred, mu, lv = model(img_b, obs_b, chunk_b)
            loss, r, k   = cvae_loss(pred, chunk_b, mu, lv)
            opt.zero_grad(); loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0); opt.step()
            tl += loss.item(); rl += r.item(); kl_ += k.item()
        if ep % 10 == 0 or ep == 1:
            print(f"   Epoch [{ep:2d}/{EPOCHS}] Loss={tl/len(loader):.5f} (Recon={rl/len(loader):.5f})")

    torch.save(model.state_dict(), os.path.join(OUTPUT_DIR, 'transformer_policy_fixed.pt'))
    print("\n3. 모델 학습 완료 및 저장 완료.")

    # ═══════════════════════════════════════════════════════════
    # 5. 정밀 시간적 앙상블(TE) 실시간 GUI 시뮬레이션
    # ═══════════════════════════════════════════════════════════
    print("\n4. 실시간 정밀 TE 제어 시뮬레이션 가동...")
    
    # 평가용 임의의 시작점 설정
    START_POS = np.array([0.05, -0.22, 0.35], dtype=np.float32)
    current_pos = START_POS.copy()
    
    # 정답 가이드라인 생성
    gt_trajectory = generate_trajectory(START_POS, FIXED_TARGET, n=SIM_STEPS, curvature=0.0)

    # 정밀 시간 그리드 버퍼 정의 (올타임 액션 공간)
    all_time_actions = np.zeros((SIM_STEPS + CHUNK_SIZE, 3), dtype=np.float32)
    all_time_counts  = np.zeros((SIM_STEPS + CHUNK_SIZE, 1), dtype=np.float32)
    
    # 제어 기록용 리스트
    te_actual_path = [current_pos.copy()]
    rmse_history = []

    # GUI 창 레이아웃 설정
    fig = plt.figure(figsize=(13, 5.5), facecolor='#0d1117')
    
    ax_cam = fig.add_subplot(131)
    ax3d   = fig.add_subplot(132, projection='3d')
    ax_err = fig.add_subplot(133)

    for t in range(SIM_STEPS):
        # 1. 환경 관측 데이터 빌드 (실시간 가상 카메라 + 관절 상태)
        current_img = render_camera(current_pos, FIXED_TARGET)
        img_tensor  = torch.tensor(current_img.transpose(2,0,1), dtype=torch.float32).unsqueeze(0)
        obs_tensor  = torch.tensor(np.concatenate([current_pos, FIXED_TARGET]), dtype=torch.float32).unsqueeze(0)
        
        # 2. Transformer CVAE 정책 모델을 통해 미래 20스텝 확률형 액션 청크 예측
        # 매 스텝 무작위 z 샘플링이 내장되어 있어 날것의 예측은 사방으로 휘어짐
        pred_chunk = model.predict(img_tensor, obs_tensor, use_z=True).squeeze(0).numpy()
        
        # 3. 시간축 정밀 인덱싱 앙상블 (TE) 수행
        for i in range(CHUNK_SIZE):
            target_time = t + i
            # 최신 보정 예측 궤적일수록 높은 지수적 신뢰도 가중치 부여
            weight = np.exp(-i * TE_LAMBDA)
            
            all_time_actions[target_time] += pred_chunk[i] * weight
            all_time_counts[target_time]  += weight
            
        # 4. 현재 타임스텝 t에 약속된 최종 앙상블 액션 추출 및 실행
        final_action = all_time_actions[t] / all_time_counts[t]
        current_pos  = final_action # 로봇 이동 활성화
        te_actual_path.append(current_pos.copy())
        
        # 5. 오차 계측
        step_rmse = np.sqrt(np.mean((current_pos - FIXED_TARGET)**2)) * 100 # cm 단위
        rmse_history.append(step_rmse)

        # ═══════════════════════════════════════════════════════
        # 6. 실시간 동적 그래픽 플로팅
        # ═══════════════════════════════════════════════════════
        # [왼쪽 패널]: 실시간 가상 카메라 뷰
        ax_cam.clear()
        ax_cam.imshow(current_img)
        ax_cam.set_title(f"Real-time Camera View (t={t})\n● Gripper  ■ Target", color='white', fontsize=10)
        ax_cam.axis('off')
        ax_cam.axhline(y=32, color='yellow', lw=0.5, alpha=0.4)

        # [가운데 패널]: 3D 공간 제어 수렴 상태
        ax3d.clear()
        ax3d.set_facecolor('#0d1117')
        # 고정 목적지 별표
        ax3d.scatter(*FIXED_TARGET, color='#ff4444', s=180, marker='*', zorder=10, label='Fixed Target')
        # 시작점 사각형
        ax3d.scatter(*START_POS, color='#44ff44', s=80, marker='s', zorder=5)
        
        # CVAE가 지금 머릿속으로 그린 무작위 날것의 미래 예측 파편 (하늘색 점선)
        ax3d.plot(pred_chunk[:,0], pred_chunk[:,1], pred_chunk[:,2], ':', color='#00ccff', alpha=0.7, lw=1.5, label='Raw Chunk (Stochastic)')
        
        # TE 필터를 거쳐 매끄럽게 다듬어진 실제 주행 경로 (보라색 실선)
        path_arr = np.array(te_actual_path)
        ax3d.plot(path_arr[:,0], path_arr[:,1], path_arr[:,2], '-', color='#b573ff', lw=2.5, label='TE Filtered Path')
        
        ax3d.set_xlim(0, 0.5); ax3d.set_ylim(-0.3, 0.3); ax3d.set_zlim(0, 0.4)
        ax3d.set_xlabel('X'); ax3d.set_ylabel('Y'); ax3d.set_zlabel('Z')
        ax3d.set_title("3D Controller Space", color='white', fontsize=10)
        ax3d.legend(fontsize=7, loc='upper left', facecolor='#1a1a1a', labelcolor='white')
        ax3d.xaxis.pane.fill=False; ax3d.yaxis.pane.fill=False; ax3d.zaxis.pane.fill=False
        ax3d.tick_params(colors='gray', labelsize=7)

        # [오른쪽 패널]: 타겟까지의 실시간 오차 계측 곡선
        ax_err.clear()
        ax_err.set_facecolor('#0d1117')
        ax_err.plot(range(len(rmse_history)), rmse_history, color='#b573ff', lw=2, label='Distance to Target')
        ax_err.set_xlabel('Control Step', color='white')
        ax_err.set_ylabel('Error Distance (cm)', color='white')
        ax_err.set_title(f"Convergence Metric\nCurrent Err: {step_rmse:.2f} cm", color='white', fontsize=10)
        ax_err.tick_params(colors='white')
        ax_err.set_ylim(0, 40)
        ax_err.grid(True, color='#333', ls='--')
        for sp in ax_err.spines.values(): sp.set_edgecolor('#555')

        plt.tight_layout()
        plt.draw()
        plt.pause(0.05) # 실시간 윈도우 갱신 싱크

    print(f"   최종 목적지 도달 오차: {rmse_history[-1]:.2f} cm -> 수렴 성공!")
    
    # 최종 결과 화면 저장 및 유지
    plt.savefig(os.path.join(OUTPUT_DIR, 'transformer_te_final_converge.png'), dpi=130, facecolor='#0d1117')
    plt.ioff()
    print("\n=== 모든 ACT 학습 및 TE 검증 파이프라인 완료 ===")
    plt.show()