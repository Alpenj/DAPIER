import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa
import os, warnings
warnings.filterwarnings("ignore")

# ─────────────────────────────────────────────
# 0. 디바이스 및 환경 설정
# ─────────────────────────────────────────────
DEVICE     = "cuda" if torch.cuda.is_available() else "cpu"
CHUNK_SIZE = 20
WORKSPACE_MIN = np.array([0.0, -0.3, 0.0])
WORKSPACE_MAX = np.array([0.5,  0.3, 0.4])

# ★ 핵심 변화: 목적지(Target)를 현실적인 물체 위치로 딱 고정합니다!
FIXED_TARGET = np.array([0.25, 0.0, 0.2]) 

print(f"Device: {DEVICE}")
print(f"Fixed Target State: {FIXED_TARGET}")

# ─────────────────────────────────────────────
# 1. 궤적 데이터 생성 (목적지 고정형)
# ─────────────────────────────────────────────
def generate_trajectory(start, target, n_steps=CHUNK_SIZE, curvature=0.15):
    t = np.linspace(0, 1, n_steps).reshape(-1, 1)
    linear = start + t * (target - start)
    direction = target - start
    if abs(direction[2]) < 1e-6:
        perp = np.cross(direction, np.array([0, 0, 1]))
    else:
        perp = np.cross(direction, np.array([1, 0, 0]))
    norm = np.linalg.norm(perp)
    perp = perp / norm if norm > 1e-6 else np.array([0, 0, 1])
    offset = curvature * 4 * t * (1 - t) * perp.reshape(1, 3)
    return linear + offset  # (n_steps, 3)


def generate_dataset_fixed_target(n_samples=2000, seed=42):
    """
    시작점(s)은 랜덤하게 생성하되, 목적지(t)는 고정된 FIXED_TARGET을 사용합니다.
    """
    rng = np.random.default_rng(seed)
    X, Y = [], []
    for _ in range(n_samples):
        s = rng.uniform(WORKSPACE_MIN, WORKSPACE_MAX)
        t = FIXED_TARGET  # 목적지 고정
        
        # 시작점과 목적지가 너무 가까운 노이즈 데이터는 제외
        if np.linalg.norm(t - s) < 0.05:
            continue
            
        # 곡률을 다양하게 주어 풍부한 전문가 데모 생성
        traj = generate_trajectory(s, t, CHUNK_SIZE,
                                   curvature=rng.uniform(0.05, 0.25))
        
        # 입력 X: [시작점_xyz (3차원) | 고정된_목적지_xyz (3차원)] = 6차원
        X.append(np.concatenate([s, t]))
        # 출력 Y: 20스텝의 3D 좌표를 일렬로 펼친 60차원 벡터 (Action Chunk)
        Y.append(traj.flatten())
        
    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.float32)


# ─────────────────────────────────────────────
# 2. PyTorch 모델 정의
# ─────────────────────────────────────────────
class ActionChunkingPolicy(nn.Module):
    def __init__(self,
                 obs_dim   : int = 6,
                 chunk_dim : int = CHUNK_SIZE * 3,
                 hidden    : int = 256,
                 n_layers  : int = 3,
                 dropout   : float = 0.1):
        super().__init__()

        # ── MLP backbone ──────────────────────────────────────────
        layers = []
        in_dim = obs_dim
        for _ in range(n_layers):
            layers += [
                nn.Linear(in_dim, hidden),
                nn.LayerNorm(hidden),   # BatchNorm 대신 LayerNorm (Transformer 친화적)
                nn.ReLU(),
                nn.Dropout(dropout),
            ]
            in_dim = hidden
        layers.append(nn.Linear(hidden, chunk_dim))  # 출력층 (60차원)
        self.net = nn.Sequential(*layers)

        # ── 가중치 초기화 (He init) ───────────────────────────────
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
                nn.init.zeros_(m.bias)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)

    # ── 인퍼런스 헬퍼 (numpy in / numpy out) ─────────────────────
    @torch.no_grad()
    def predict(self, start: np.ndarray, target: np.ndarray) -> np.ndarray:
        obs = np.concatenate([start, target]).astype(np.float32)
        obs_t = torch.from_numpy(obs).unsqueeze(0).to(DEVICE)  # (1, 6)
        chunk = self(obs_t).squeeze(0).cpu().numpy()            # (60,)
        return chunk.reshape(CHUNK_SIZE, 3)


# ─────────────────────────────────────────────
# 3. 학습 루프
# ─────────────────────────────────────────────
def train(model, X, Y, epochs=200, batch_size=64, lr=1e-3):
    ds     = TensorDataset(torch.from_numpy(X), torch.from_numpy(Y))
    loader = DataLoader(ds, batch_size=batch_size, shuffle=True)

    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    criterion = nn.MSELoss()

    model.to(DEVICE)
    model.train()
    loss_history = []

    for epoch in range(1, epochs + 1):
        epoch_loss = 0.0
        for x_batch, y_batch in loader:
            x_batch = x_batch.to(DEVICE)
            y_batch = y_batch.to(DEVICE)

            pred = model(x_batch)           
            loss = criterion(pred, y_batch)

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            epoch_loss += loss.item()

        epoch_loss /= len(loader)
        loss_history.append(epoch_loss)
        scheduler.step()

        if epoch % 20 == 0 or epoch == 1:
            print(f"  Epoch [{epoch:3d}/{epochs}]  "
                  f"MSE Loss = {epoch_loss:.6f}  "
                  f"LR = {scheduler.get_last_lr()[0]:.6f}")

    return loss_history


# ─────────────────────────────────────────────
# 4. 모델 저장 / 불러오기 
# ─────────────────────────────────────────────
def save_model(model, path="./101_output/act_policy_fixed.pt"):
    torch.save({
        "state_dict": model.state_dict(),
        "config": {
            "obs_dim"   : 6,
            "chunk_dim" : CHUNK_SIZE * 3,
            "hidden"    : 256,
            "n_layers"  : 3,
            "dropout"   : 0.1,
        }
    }, path)
    print(f"[saved] {path}")


def load_model(path="./101_output/act_policy_fixed.pt") -> ActionChunkingPolicy:
    ckpt   = torch.load(path, map_location=DEVICE)
    cfg    = ckpt["config"]
    model  = ActionChunkingPolicy(**cfg)
    model.load_state_dict(ckpt["state_dict"])
    model.to(DEVICE).eval()
    print(f"[loaded] {path}")
    return model


# ─────────────────────────────────────────────
# 5. 개선된 '수렴형' 시각화 함수들
# ─────────────────────────────────────────────
def plot_loss_curve(loss_history):
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(loss_history, color='forestgreen', linewidth=1.8)
    ax.set_xlabel("Epoch");  ax.set_ylabel("MSE Loss")
    ax.set_title("Training Loss Curve (Fixed Target Scenario)")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.4)
    plt.tight_layout()
    plt.savefig("./101_output/pt_loss_fixed.png", dpi=150)
    plt.show()


def plot_converging_trajectories(model, n_show=5, seed=77):
    """
    다양한 시작점(Start)에서 하나의 고정된 목적지(FIXED_TARGET)로
    모여드는 Ground Truth와 예측값(Action Chunk)들을 한눈에 보여주는 정적 시각화
    """
    rng = np.random.default_rng(seed)
    fig = plt.figure(figsize=(15, 6))

    # 왼쪽 서브플롯: Ground Truth (원래 전문가가 생성한 데이터)
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.scatter(*FIXED_TARGET, color='red', s=180, marker='*', zorder=10, label='Fixed Target (Object)')
    
    # 오른쪽 서브플롯: Predicted Trajectories (모델이 오직 시작점만 보고 예측한 결과)
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    ax2.scatter(*FIXED_TARGET, color='red', s=180, marker='*', zorder=10, label='Fixed Target (Object)')

    for i in range(n_show):
        start = rng.uniform(WORKSPACE_MIN, WORKSPACE_MAX)
        
        # Ground Truth 데이터셋 생성 및 플로팅
        gt_traj = generate_trajectory(start, FIXED_TARGET, CHUNK_SIZE, curvature=0.15)
        ax1.plot(gt_traj[:, 0], gt_traj[:, 1], gt_traj[:, 2], 'g-o', markersize=3, alpha=0.6)
        ax1.scatter(*start, color='blue', s=50, marker='s')

        # 학습된 모델의 예측(Action Chunk) 및 플로팅
        pred_traj = model.predict(start, FIXED_TARGET)
        ax2.plot(pred_traj[:, 0], pred_traj[:, 1], pred_traj[:, 2], 'b-o', markersize=3, alpha=0.6)
        ax2.scatter(*start, color='blue', s=50, marker='s')

    # 그래프 외형 세부 조정
    for ax, title in [(ax1, "Ground Truth Trajectories (Expert)"), 
                      (ax2, "Model Predicted Trajectories (Action Chunk)")]:
        ax.set_xlim(WORKSPACE_MIN[0], WORKSPACE_MAX[0])
        ax.set_ylim(WORKSPACE_MIN[1], WORKSPACE_MAX[1])
        ax.set_zlim(WORKSPACE_MIN[2], WORKSPACE_MAX[2])
        ax.set_title(title, fontsize=11, fontweight='bold')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        # 범례 임시 구성용 (중복 방지)
        ax.scatter([], [], [], color='blue', marker='s', label='Random Starts')
        ax.legend(loc='upper left', fontsize=8)

    plt.suptitle("ACT Generalization Test: Converging to a Fixed Target", fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout()
    plt.savefig("./101_output/pt_static_fixed.png", dpi=150)
    plt.show()
    print("[saved] pt_static_fixed.png")


def animate_converging_grippers(model, n_grippers=4, seed=12):
    """
    여러 개의 그리퍼(빨간 구체)가 서로 다른 임의의 위치에서 출발하여,
    모델이 설계한 액션 청크(파란 선)를 따라 동시에 목표 지점으로 수렴해 가는 동적 시뮬레이션
    """
    rng = np.random.default_rng(seed)
    starts = [rng.uniform(WORKSPACE_MIN, WORKSPACE_MAX) for _ in range(n_grippers)]
    
    # 모델에 예측 수행
    trajs = [model.predict(s, FIXED_TARGET) for s in starts]

    fig = plt.figure(figsize=(9, 7))
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_xlim(WORKSPACE_MIN[0], WORKSPACE_MAX[0])
    ax.set_ylim(WORKSPACE_MIN[1], WORKSPACE_MAX[1])
    ax.set_zlim(WORKSPACE_MIN[2], WORKSPACE_MAX[2])
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title("Simultaneous Action Chunk Playback\n(Converging to Target)", fontweight='bold', pad=10)

    # 타겟(종이컵) 위치 고정 표시
    ax.scatter(*FIXED_TARGET, color='red', s=200, marker='*', zorder=15, label='Fixed Target (Object)')

    # 시작 마커들 표시
    for s in starts:
        ax.scatter(*s, color='green', s=60, marker='s', alpha=0.5)
    ax.scatter([], [], [], color='green', marker='s', label='Random Starts') # 범례용

    # 애니메이션으로 업데이트될 라인들과 그리퍼(구체들) 초기화
    lines    = [ax.plot([], [], [], 'b-', alpha=0.5, lw=2)[0] for _ in range(n_grippers)]
    grippers = [ax.plot([], [], [], 'ro', markersize=9, zorder=10)[0] for _ in range(n_grippers)]
    
    # 범례 추가용
    lines[0].set_label('Predicted Action Chunk (Future Plan)')
    grippers[0].set_label('Gripper Model Execution')
    ax.legend(loc='upper left', fontsize=9)

    def init():
        for line, gripper in zip(lines, grippers):
            line.set_data([], []); line.set_3d_properties([])
            gripper.set_data([], []); gripper.set_3d_properties([])
        return lines + grippers

    def update(frame):
        idx = min(frame, CHUNK_SIZE - 1)
        for i in range(n_grippers):
            # 과거부터 현재(idx)까지의 예측 경로 그리기
            lines[i].set_data(trajs[i][:idx+1, 0], trajs[i][:idx+1, 1])
            lines[i].set_3d_properties(trajs[i][:idx+1, 2])
            # 실제 제어부가 실행 중인 실시간 그리퍼 위치 업데이트
            grippers[i].set_data([trajs[i][idx, 0]], [trajs[i][idx, 1]])
            grippers[i].set_3d_properties([trajs[i][idx, 2]])
        return lines + grippers

    anim = FuncAnimation(fig, update, frames=CHUNK_SIZE + 7,
                         init_func=init, blit=False, interval=150, repeat=True)
    anim.save("./101_output/pt_animation_fixed.gif", writer='pillow', fps=7)
    plt.show()
    print("[saved] pt_animation_fixed.gif")


# ─────────────────────────────────────────────
# 6. 메인 실행부
# ─────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 60)
    print("  ACT Lite — PyTorch MLP 버전 (목적지 고정 수렴형)")
    print("=" * 60)

    # 1. 데이터셋 생성
    print("\n[1/5] 데이터 생성 (목적지: FIXED_TARGET)...")
    X, Y = generate_dataset_fixed_target(n_samples=2000)
    X_te, Y_te = generate_dataset_fixed_target(n_samples=300, seed=999)
    print(f"  Train 데이터셋 크기: X={X.shape}, Y={Y.shape}")

    # 2. 모델 인스턴스화
    print("\n[2/5] 모델 생성...")
    policy = ActionChunkingPolicy(obs_dim=6, chunk_dim=CHUNK_SIZE*3,
                                  hidden=256, n_layers=3, dropout=0.1)
    n_params = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    print(f"  파라미터 수: {n_params:,} ({n_params/1e3:.1f}K)")

    # 3. 모델 학습
    print("\n[3/5] 학습 시작...")
    history = train(policy, X, Y, epochs=200, batch_size=64, lr=1e-3)

    # 4. 테스트 셋 평가 (RMSE)
    policy.eval()
    with torch.no_grad():
        X_te_t = torch.from_numpy(X_te).to(DEVICE)
        Y_pred = policy(X_te_t).cpu().numpy()
    rmse = float(np.sqrt(np.mean((Y_te - Y_pred) ** 2)))
    print(f"\n  테스트 RMSE 오차: {rmse:.6f} m  ({rmse*1000:.2f} mm)")

    # 5. 모델 저장 및 복원 테스트
    print("\n[4/5] 모델 저장 및 불러오기...")
    save_model(policy, "./101_output/act_policy_fixed.pt")
    policy_loaded = load_model("./101_output/act_policy_fixed.pt")

    # 6. 정적/동적 시각화 출력
    print("\n[5/5] 수렴형 시각화 자료 출력...")
    plot_loss_curve(history)
    plot_converging_trajectories(policy_loaded, n_show=5)
    animate_converging_grippers(policy_loaded, n_grippers=5)

    print("\n모든 설명용 데이터셋 구축 및 시각화 준비가 완료되었습니다!")