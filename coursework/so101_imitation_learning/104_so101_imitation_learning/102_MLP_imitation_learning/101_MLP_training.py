import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torchvision import transforms
from torchvision.models import resnet18, ResNet18_Weights
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# 1. 모델 정의
class RobotPolicy(nn.Module):
    def __init__(self, action_dim=6):
        super().__init__()
        # 전이 학습: ImageNet으로 선학습된 ResNet18 사용
        self.visual_backbone = resnet18(weights=ResNet18_Weights.DEFAULT)
        self.visual_backbone.fc = nn.Identity() 
        
        # [특징 추출(512) + 현재 상태(6)] -> [예측 행동(6)]
        self.policy_head = nn.Sequential(
            nn.Linear(512 + 6, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim)
        )

        # ResNet 표준 정규화 (전이 학습 성능 최적화)
        self.normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], 
            std=[0.229, 0.224, 0.225]
        )

    def forward(self, image, state):
        # 1. 이미지 정규화 적용
        image = self.normalize(image)
        # 2. 특징 추출
        img_features = self.visual_backbone(image) 
        # 3. 데이터 결합 및 정책 결정
        combined = torch.cat([img_features, state], dim=1) 
        return self.policy_head(combined)

# 2. 트레이닝 루프
def train(repo_id="my_robot_task"):
    # RTX 5060 Ti 활용 설정
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"🚀 학습 장치: {device} ({torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'})")

    try:
        # 데이터셋 로드 (수집 시 0~1로 저장된 상태)
        dataset = LeRobotDataset(repo_id=repo_id, video_backend="pyav")
    except Exception as e:
        print(f"❌ 데이터셋 로드 실패: {e}")
        return

    train_loader = DataLoader(
        dataset, 
        batch_size=64, 
        shuffle=True, 
        num_workers=4,
        pin_memory=True
    )

    model = RobotPolicy().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
    # MSELoss를 사용하여 수렴 속도 향상
    criterion = nn.MSELoss()

    model.train()
    print("--- 정규화 기반 트레이닝 시작 ---")
    
    for epoch in range(50):
        total_loss = 0
        for batch in train_loader:
            # [수정] 이미 데이터 수집 시 0~1로 정규화되었으므로 추가 변환 없이 float화만 진행
            images = batch["observation.image"].to(device, non_blocking=True).float() / 255.0
            states = batch["observation.state"].to(device, non_blocking=True).float()
            actions_target = batch["action"].to(device, non_blocking=True).float()

            # 순전파
            actions_pred = model(images, states)
            loss = criterion(actions_pred, actions_target)

            # 역전파
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item()
        
        avg_loss = total_loss / len(train_loader)
        # 이제 Loss가 3000대가 아닌 0.00xxx 단위로 출력될 것입니다.
        print(f"Epoch [{epoch+1}/50] - Loss: {avg_loss:.8f}")

    # 4. 모델 저장
    torch.save(model.state_dict(), "robot_model_final.pth")
    print("✅ 정규화 모델 학습 완료: 'robot_model_final.pth'")

if __name__ == "__main__":
    train()