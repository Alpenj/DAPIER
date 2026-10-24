import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader
from torchvision import transforms
from torchvision.models import resnet18, ResNet18_Weights
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# --- 1. ACT 핵심 모델 구성 ---
class ACTPolicy(nn.Module):
    def __init__(self, action_dim=6, state_dim=6, chunk_size=50, hidden_dim=512, nheads=8):
        super().__init__()
        self.chunk_size = chunk_size
        self.action_dim = action_dim

        # A. 시각 백본 (ResNet18)
        self.backbone = resnet18(weights=ResNet18_Weights.DEFAULT)
        self.backbone.fc = nn.Identity()
        self.normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])

        # B. State Encoder (현재 상태 6차원 -> 512차원 확장)
        self.state_encoder = nn.Linear(state_dim, hidden_dim)

        # C. CVAE Encoder (이미지 특징 512 + 액션 뭉치 300 = 812 입력)
        self.cvae_encoder = nn.Linear(hidden_dim + (action_dim * chunk_size), hidden_dim * 2)
        
        # D. Transformer (핵심 추론 엔진)
        self.transformer = nn.Transformer(
            d_model=hidden_dim, nhead=nheads, 
            num_encoder_layers=4, num_decoder_layers=4, batch_first=True
        )

        # E. Action Head (미래 궤적 출력)
        self.action_head = nn.Linear(hidden_dim, action_dim * chunk_size)

    def reparameterize(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return mu + eps * std

    def forward(self, image, state, actions=None):
        batch_size = image.shape[0]
        image = self.normalize(image)
        img_feat = self.backbone(image) # (B, 512)
        
        # State를 512차원으로 투영
        state_feat = self.state_encoder(state) # (B, 512)

        latent_loss = torch.tensor(0.0).to(image.device)
        if actions is not None:
            # [차원 맞춤] 만약 데이터로더에서 1개 액션만 왔다면 50개로 확장
            if actions.ndim == 2: # (B, 6)
                actions = actions.unsqueeze(1).repeat(1, self.chunk_size, 1)
            
            flattened_actions = actions.reshape(batch_size, -1) # (B, 300)
            combined = torch.cat([img_feat, flattened_actions], dim=1) # (B, 812)
            
            h = F.relu(self.cvae_encoder(combined))
            mu, logvar = torch.chunk(h, 2, dim=1)
            z = self.reparameterize(mu, logvar) # (B, 512)
            latent_loss = -0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp())
        else:
            z = torch.zeros(batch_size, 512).to(image.device)

        # Transformer 입력 준비: [Style(z), Image, State] 모두 (B, 512)
        # stack 결과: (B, 3, 512)
        src = torch.stack([z, img_feat, state_feat], dim=1) 
        
        # Transformer 통과
        out = self.transformer.encoder(src)
        # Style 토큰(0번 인덱스) 결과물을 사용하여 액션 예측
        pred_actions = self.action_head(out[:, 0, :]) 
        
        return pred_actions.view(batch_size, self.chunk_size, self.action_dim), latent_loss

# --- 2. 데이터 로더 (LeRobot 0.4.4 대응) ---
def get_dataloader(repo_id, chunk_size, batch_size):
    # video_backend를 pyav로 설정하여 torchcodec 에러 회피
    dataset = LeRobotDataset(repo_id=repo_id, video_backend="pyav")
    
    # 미래 액션 데이터를 가져오기 위한 델타 타임스텝 설정
    dataset.delta_timesteps = {
        "action": list(range(chunk_size))
    }
    
    return DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=0)

# --- 3. 트레이닝 루프 ---
def train_act(repo_id="my_robot_task"):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    chunk_size = 50 
    action_dim = 6
    print(device)
    
    model = ACTPolicy(action_dim=action_dim, state_dim=6, chunk_size=chunk_size).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
    
    try:
        loader = get_dataloader(repo_id, chunk_size, 64)
    except Exception as e:
        print(f"데이터셋 로드 실패: {e}")
        return

    model.train()
    print("🚀 학습 시작...")
    
    for epoch in range(100):
        total_loss = 0
        for batch in loader:
            # 데이터 전처리 및 GPU 이동
            images = batch["observation.image"].to(device).float() / 255.0
            states = batch["observation.state"].to(device).float()
            actions_target = batch["action"].to(device).float() 

            # [핵심] 만약 데이터셋에서 미래 액션을 안 줬을 경우를 대비한 강제 차원 맞춤
            if actions_target.ndim == 2:
                actions_target = actions_target.unsqueeze(1).repeat(1, chunk_size, 1)

            # 순전파
            pred_actions, kl_loss = model(images, states, actions_target)
            
            # MSE Loss (예측 궤적 vs 실제 궤적)
            mse_loss = F.mse_loss(pred_actions, actions_target)
            
            # 전체 손실 (KL 가중치 0.01 적용)
            loss = mse_loss + (0.01 * kl_loss / images.shape[0])

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        print(f"Epoch [{epoch+1}/100] - Loss: {total_loss/len(loader):.6f}")

    torch.save(model.state_dict(), "act_robot_model.pth")
    print("✅ ACT 모델 저장 완료: act_robot_model.pth")

if __name__ == "__main__":
    # 본인의 실제 repo_id로 변경하여 실행하세요
    train_act(repo_id="my_robot_task")