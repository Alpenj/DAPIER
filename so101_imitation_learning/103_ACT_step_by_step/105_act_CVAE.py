import os
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, TensorDataset
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

# ─────────────────────────────────────────
# 1. 설정 및 하이퍼파라미터 (목적지 고정형)
# ─────────────────────────────────────────
DEVICE        = "cuda" if torch.cuda.is_available() else "cpu"
OUTPUT_DIR    = "./105_output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

CHUNK_SIZE    = 20      # Action Chunk 길이 (미래 예측 스텝 수)
OBS_DIM       = 6       # 입력 차원: start(3) + target(3)
CHUNK_DIM     = 60      # 출력 차원: CHUNK_SIZE * 3 (20 * 3D)
LATENT_DIM    = 32      # CVAE 잠재 변수 z 차원
HIDDEN_DIM    = 256
BETA          = 0.05     # KL Loss 가중치
EPOCHS        = 300     # 빠른 교육용 학습을 위해 100 에폭 설정
LR            = 1e-3
BATCH_SIZE    = 128
N_DATA        = 2500

# 제어 및 TE 하이퍼파라미터
TE_WINDOW     = 8       # 앙상블할 궤적 버퍼 크기
TE_LAMBDA     = 0.15    # 지수 감쇠 가중치 계수
MAX_STEPS     = 60      # 로봇 최대 구동 스텝 수

WORKSPACE_MIN = np.array([0.0, -0.3, 0.0])
WORKSPACE_MAX = np.array([0.5,  0.3, 0.4])

# ★ 핵심 요구사항: 목적지(Target)를 공간 중앙의 고정 좌표로 지정
FIXED_TARGET  = np.array([0.25, 0.0, 0.2], dtype=np.float32)

# ─────────────────────────────────────────
# 2. 데이터셋 생성 (고정 목적지 기반)
# ─────────────────────────────────────────
def generate_trajectory(start, target, n=CHUNK_SIZE, curvature=0.15):
    t_vec  = np.linspace(0, 1, n).reshape(-1, 1)
    linear = start + t_vec * (target - start)
    d      = target - start
    perp   = np.cross(d, [0, 0, 1]) if abs(d[2]) < 1e-6 else np.cross(d, [1, 0, 0])
    nm     = np.linalg.norm(perp)
    perp   = perp / nm if nm > 1e-6 else np.array([0, 0, 1])
    return linear + curvature * 4 * t_vec * (1 - t_vec) * perp.reshape(1, 3)

def generate_dataset_fixed(n_samples=N_DATA, seed=42):
    rng = np.random.default_rng(seed)
    X, Y = [], []
    while len(X) < n_samples:
        s = rng.uniform(WORKSPACE_MIN + 0.02, WORKSPACE_MAX - 0.02)
        if np.linalg.norm(FIXED_TARGET - s) < 0.06:
            continue
        curv = rng.uniform(-0.1, 0.1)
        traj = generate_trajectory(s, FIXED_TARGET, CHUNK_SIZE, curvature=curv)
        X.append(np.concatenate([s, FIXED_TARGET]))
        Y.append(traj.flatten())
    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.float32)

# ─────────────────────────────────────────
# 3. 모델 정의 (MLP 내장형 CVAE)
# ─────────────────────────────────────────
class Encoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(OBS_DIM + CHUNK_DIM, HIDDEN_DIM), nn.LayerNorm(HIDDEN_DIM), nn.ReLU(),
            nn.Linear(HIDDEN_DIM, HIDDEN_DIM),          nn.LayerNorm(HIDDEN_DIM), nn.ReLU(),
        )
        self.fc_mu = nn.Linear(HIDDEN_DIM, LATENT_DIM)
        self.fc_lv = nn.Linear(HIDDEN_DIM, LATENT_DIM)

    def forward(self, obs, chunk):
        h = self.net(torch.cat([obs, chunk], dim=-1))
        return self.fc_mu(h), self.fc_lv(h)

class Decoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(OBS_DIM + LATENT_DIM, HIDDEN_DIM), nn.LayerNorm(HIDDEN_DIM), nn.ReLU(),
            nn.Linear(HIDDEN_DIM, HIDDEN_DIM),           nn.LayerNorm(HIDDEN_DIM), nn.ReLU(),
            nn.Linear(HIDDEN_DIM, HIDDEN_DIM),           nn.LayerNorm(HIDDEN_DIM), nn.ReLU(),
            nn.Linear(HIDDEN_DIM, CHUNK_DIM),
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

    @torch.no_grad()
    def predict(self, start: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Inference: z ~ N(0, I) 확률 샘플링 기반 단발성 20스텝 예측"""
        self.eval()
        obs = torch.tensor(np.concatenate([start, target]), dtype=torch.float32).unsqueeze(0).to(DEVICE)
        z   = torch.randn(1, LATENT_DIM).to(DEVICE)
        pred = self.decoder(obs, z)
        return pred.squeeze(0).cpu().numpy().reshape(CHUNK_SIZE, 3)

def cvae_loss(pred, target, mu, log_var, beta=BETA):
    recon = F.mse_loss(pred, target)
    kl    = -0.5 * torch.mean(1 + log_var - mu.pow(2) - log_var.exp())
    return recon + beta * kl, recon, kl

# ─────────────────────────────────────────
# 4. 메인 실행부 (학습 및 실시간 가시화)
# ─────────────────────────────────────────
def main():
    # 1) 데이터셋 준비
    print("고정 목적지 기반 데이터셋 생성 중...")
    X, Y = generate_dataset_fixed(N_DATA)
    dataset = TensorDataset(torch.from_numpy(X), torch.from_numpy(Y))
    loader  = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True, drop_last=True)

    # 2) 모델 학습
    print("\n[MLP 내장 CVAE 모델 학습 시작]")
    model = CVAE().to(DEVICE)
    opt   = torch.optim.Adam(model.parameters(), lr=LR, weight_decay=1e-4)
    
    for ep in range(1, EPOCHS + 1):
        model.train()
        tl = rl = kl_ = 0.0
        for obs_b, chunk_b in loader:
            obs_b, chunk_b = obs_b.to(DEVICE), chunk_b.to(DEVICE)
            pred, mu, lv = model(obs_b, chunk_b)
            loss, r, k   = cvae_loss(pred, chunk_b, mu, lv)
            opt.zero_grad()
            loss.backward()
            opt.step()
            tl += loss.item(); rl += r.item(); kl_ += k.item()
        if ep % 20 == 0 or ep == 1:
            print(f"  Epoch [{ep:3d}/{EPOCHS}] Loss={tl/len(loader):.5f} (Recon={rl/len(loader):.5f}, KL={kl_/len(loader):.5f})")

    # 3) 모델 저장 (요구사항: ./105_output 폴더 저장)
    ckpt_path = os.path.join(OUTPUT_DIR, 'cvae_policy_te.pt')
    torch.save({'state_dict': model.state_dict()}, ckpt_path)
    print(f"\n✔ 모델 가중치 저장 완료: {ckpt_path}")

    # 4) 실시간 윈도우 시뮬레이션 환경 구축
    print("\n[실시간 제어 시뮬레이션 창을 엽니다...]")
    plt.ion() # 대화형 모드(Interactive Mode) 활성화
    fig = plt.figure(figsize=(14, 7), facecolor="#111111")
    fig.suptitle("ACT Lite: CVAE Stochastic Inference + Temporal Ensembling (TE) Filter Loop", color="white", fontsize=12, fontweight="bold")
    
    gs = gridspec.GridSpec(2, 3, width_ratios=[2, 1, 1], wspace=0.3, hspace=0.4)
    
    # 패널 1: 3D 메인 구동 공간
    ax_3d = fig.add_subplot(gs[:, 0], projection='3d')
    ax_3d.set_facecolor("#151515")
    ax_3d.set_xlim(WORKSPACE_MIN[0], WORKSPACE_MAX[0])
    ax_3d.set_ylim(WORKSPACE_MIN[1], WORKSPACE_MAX[1])
    ax_3d.set_zlim(WORKSPACE_MIN[2], WORKSPACE_MAX[2])
    ax_3d.set_title("3D Robot Workspace (k=1 Re-plan)", color="white", fontsize=10)
    
    # 패널 2: 목표까지의 남은 거리 추이
    ax_dist = fig.add_subplot(gs[0, 1])
    ax_dist.set_facecolor("#151515")
    ax_dist.set_title("Distance to Target", color="white", fontsize=9)
    ax_dist.set_xlim(0, MAX_STEPS)
    ax_dist.set_ylim(0, 0.5)
    ax_dist.grid(True, color="#333")
    ax_dist.tick_params(colors="gray")
    
    # 패널 3: TE 현재 시점 가중치 분포
    ax_w = fig.add_subplot(gs[1, 1])
    ax_w.set_facecolor("#151515")
    ax_w.set_title(f"TE Buffer Weights (W={TE_WINDOW})", color="white", fontsize=9)
    ax_w.set_xlim(-0.5, TE_WINDOW - 0.5)
    ax_w.set_ylim(0, 1.0)
    ax_w.tick_params(colors="gray")

    # 고정 목적지 마커 그리기
    ax_3d.scatter(*FIXED_TARGET, color="#ff4444", s=120, marker="*", zorder=10, label="Fixed Target")
    
    # 테스트용 무작위 시작점(Start) 1개 선정
    rng = np.random.default_rng()
    current_pos = rng.uniform(WORKSPACE_MIN + 0.05, WORKSPACE_MAX - 0.05)
    start_pos   = current_pos.copy()
    ax_3d.scatter(*start_pos, color="#44ff44", s=60, marker="s", label="Start")
    
    # 실시간 업데이트용 라인/플롯 요소 객체 정의
    te_traj_line, = ax_3d.plot([], [], [], color="#cc88ff", lw=2.5, label="TE Actual Path")
    robot_grip,   = ax_3d.plot([], [], [], 'o', color="white", markersize=8, markeredgecolor="#cc88ff", markeredgewidth=2, zorder=15)
    raw_chunk_lines = [] # 매 순간 샘플링되는 흔들리는 원본 청크들을 담을 리스트

    executed_path = [start_pos.copy()]
    distances     = []
    te_buffer     = []

    # 5) 실시간 시뮬레이션 제어 루프
    for step in range(MAX_STEPS):
        # [TE 단계 1] k=1 주기로 현재 위치에서 CVAE 신규 예측 (매 스텝 새로운 z 무작위 샘플링)
        # CVAE 특성상 같은 위치라도 매 스텝 예측되는 경로 스타일(물길)이 다릅니다.
        pred_chunk = model.predict(current_pos, FIXED_TARGET)
        
        # [TE 단계 2] 시간적 앙상블 버퍼에 신규 궤적 삽입 (가장 최신이 index 0)
        te_buffer.insert(0, pred_chunk)
        if len(te_buffer) > TE_WINDOW:
            te_buffer.pop()
            
        # [TE 단계 3] 지수 감쇠 가중치 계산 (최신 예측에 높은 가중치 부여)
        n_buf   = len(te_buffer)
        raw_w   = np.array([np.exp(-i * TE_LAMBDA) for i in range(n_buf)])
        weights = raw_w / raw_w.sum()
        
        # [TE 단계 4] 겹치는 타임슬롯 가중 평균을 통한 최종 1스텝 명령 산출 (스무딩 필터링)
        final_action = np.zeros(3)
        for i, (ch, w) in enumerate(zip(te_buffer, weights)):
            final_action += w * ch[i] # i스텝 전 청크의 i번째 경유지
            
        # 로봇 하드웨어 구동 및 위치 갱신
        current_pos = final_action.copy()
        executed_path.append(current_pos.copy())
        
        # 오차 계산
        dist = np.linalg.norm(FIXED_TARGET - current_pos)
        distances.append(dist)
        
        # ─── 실시간 Matplotlib 윈도우 그래픽 업데이트 ───
        # 기존 잔상 제거 (이전 스텝에서 그렸던 CVAE 원본 궤적 파편 지우기)
        for line in raw_chunk_lines:
            line.remove()
        raw_chunk_lines.clear()
        
        # 1. 버퍼에 쌓여서 앙상블 대기 중인 모든 CVAE 확률적 청크(궤적)들을 화면에 투명하게 표시
        # CVAE가 매 스텝 얼마나 흔들리는 길을 제안하고 있는지 눈으로 확인하는 파트입니다.
        for idx, ch in enumerate(te_buffer):
            alpha = float(weights[idx]) * 0.8
            # 로봇의 과거 궤적 시점부터 앞으로 뻗어나가는 상대적 흐름이므로 편의상 현재 가중치 기반 시각화
            l, = ax_3d.plot(ch[:, 0], ch[:, 1], ch[:, 2], ':', color="#61afef", alpha=max(0.1, alpha))
            raw_chunk_lines.append(l)
            
        # 2. TE 가중 평균 필터를 거쳐 매끄럽게 완성되어가는 실제 로봇 주행 경로 업데이트
        path_arr = np.array(executed_path)
        te_traj_line.set_data(path_arr[:, 0], path_arr[:, 1])
        te_traj_line.set_3d_properties(path_arr[:, 2])
        
        # 3. 실시간 그리퍼 위치 마킹
        robot_grip.set_data([current_pos[0]], [current_pos[1]])
        robot_grip.set_3d_properties([current_pos[2]])
        
        # 4. 오른쪽 상단 패널: 거리 수렴 차트 업데이트
        ax_dist.plot(range(len(distances)), distances, color="#ff9944", lw=1.5)
        
        # 5. 오른쪽 하단 패널: TE 가중치 분배 바 차트 업데이트
        ax_w.cla()
        ax_w.set_facecolor("#151515")
        ax_w.set_title(f"TE Buffer Weights (W={TE_WINDOW})", color="white", fontsize=9)
        ax_w.bar(range(n_buf), weights, color="#cc88ff", alpha=0.85, width=0.6)
        ax_w.set_xlim(-0.5, TE_WINDOW - 0.5)
        ax_w.set_ylim(0, 1.0)
        ax_w.tick_params(colors="gray")
        
        # 인터랙티브 윈도우 플롯 강제 리프레시 및 지연 (로봇 속도 모사)
        plt.draw()
        plt.pause(0.08)
        
        # 목적지 안착 조건 도달 시 루프 종료
        if dist < 0.015:
            print(f"🎯 목적지에 부드럽게 안착했습니다! (최종 스텝: {step+1})")
            break
            
    # 시뮬레이션 종료 후 윈도우 유지
    plt.ioff()
    ax_3d.legend(loc="upper left", fontsize=8)
    print("\n[시뮬레이션 완료] Matplotlib 창을 닫으면 프로그램이 종료됩니다.")
    plt.show()

if __name__ == '__main__':
    main()