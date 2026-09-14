import cv2
import torch
import torch.nn.functional as F
from torchvision.models import resnet18, ResNet18_Weights
from torchvision import transforms
import torch.nn as nn
from motor_control import MiniFeetechDriver # 드라이버 임포트
import json

class ACTPolicy(nn.Module):
    def __init__(self, action_dim=6, state_dim=6, chunk_size=50, hidden_dim=512, nheads=8):
        super().__init__()
        self.chunk_size = chunk_size
        self.action_dim = action_dim
        self.hidden_dim = hidden_dim # 추론 시 z 생성을 위해 추가

        self.backbone = resnet18(weights=ResNet18_Weights.DEFAULT)
        self.backbone.fc = nn.Identity()
        self.normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        
        self.state_encoder = nn.Linear(state_dim, hidden_dim)
        self.cvae_encoder = nn.Linear(hidden_dim + (action_dim * chunk_size), hidden_dim * 2)

        self.transformer = nn.Transformer(
                    d_model=hidden_dim, nhead=nheads, 
                    num_encoder_layers=4, num_decoder_layers=4, batch_first=True
                )
        self.action_head = nn.Linear(hidden_dim, action_dim * chunk_size)
    
    def forward(self, image, state, actions=None):
        batch_size = image.shape[0]
        image = self.normalize(image)
        img_feat = self.backbone(image)
        state_feat = self.state_encoder(state)

        if actions is not None: # 학습 모드
            flattened_actions = actions.reshape(batch_size, -1)
            combined = torch.cat([img_feat, flattened_actions], dim=1)
            h = F.relu(self.cvae_encoder(combined))
            mu, logvar = torch.chunk(h, 2, dim=1)
            std = torch.exp(0.5 * logvar)
            eps = torch.randn_like(std)
            z = mu + eps * std
            latent_loss = -0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp())
        else: # 추론 모드 (z를 0으로 설정)
            z = torch.zeros(batch_size, self.hidden_dim).to(image.device)
            latent_loss = None

        src = torch.stack([z, img_feat, state_feat], dim=1) 
        out = self.transformer.encoder(src)
        pred_actions = self.action_head(out[:, 0, :]) 
        return pred_actions.view(batch_size, self.chunk_size, self.action_dim), latent_loss


def load_model(checkpoint_path, device):
    # 이제 ACTPolicy가 정의되어 있으므로 에러가 나지 않습니다.
    model = ACTPolicy(action_dim=6, state_dim=6, chunk_size=50).to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()
    return model

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = load_model("act_robot_model.pth", device)
driver = MiniFeetechDriver(port="/dev/ttyUSB1")

f_ids = [1, 2, 3, 4, 5, 6] # 실제 ID에 맞게 수정

# 1. 캘리브레이션 데이터 로드 (추론 코드 상단에 추가)
with open("full_arm_calibration_follower.json", "r") as f:
    follower_cfg = json.load(f)
joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

cap = cv2.VideoCapture(0)
print("로봇 추론 시작... 'q'를 누르면 종료합니다.")
try:
    with torch.no_grad(): # 기울기 계산 비활성화 (메모리 절약)
        while True:
            ret, frame = cap.read()
            if ret:
                cv2.imshow('win', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("test")
                    break
            # 1. 입력 데이터 준비 (수집 시와 동일한 전처리)
            img_resized = cv2.resize(frame, (224, 224))
            img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
            img_torch = torch.from_numpy(img_rgb).permute(2, 0, 1).float() / 255.0
            img_torch = img_torch.unsqueeze(0).to(device) # (1, 3, 224, 224)

            current_states = []
            for name in joint_names:
                cfg = follower_cfg[name]
                pos = driver.get_position(cfg["id"])
                # 수집 시와 동일한 공식: (현재 - 최소) / (최대 - 최소)
                ratio = (pos - cfg["range_min"]) / (cfg["range_max"] - cfg["range_min"])
                current_states.append(max(0.0, min(1.0, ratio)))

            state_torch = torch.tensor(current_states).float().unsqueeze(0).to(device)

            # 2. 모델 예측 (actions 인자 없이 호출 -> 내부에서 z=0 처리)
            # pred_actions shape: (1, 50, 6)
            pred_actions, _ = model(img_torch, state_torch)

            # 3. 액션 실행 (Action Chunking의 이점 활용)
            # 50개의 미래 동작 중 처음 1~10개 정도만 실행하거나, 
            # 가장 부드러운 첫 번째 액션만 실행 (Temporal Ensembling 미적용 시)
            target_action = pred_actions[0, 0, :].cpu().numpy() # 첫 번째 시점 액션 선택

            # 4. 로봇에게 명령 전달 (정규화 해제)
            raw_goals = []
            for i, name in enumerate(joint_names):
                cfg = follower_cfg[name]
                # 수집 시와 동일한 복원 공식: (비율 * 범위) + 최소
                val = target_action[i]
                actual_pos = int(val * (cfg["range_max"] - cfg["range_min"]) + cfg["range_min"])
                raw_goals.append(actual_pos)

            print(f"Pred Goals: {raw_goals}")
            driver.sync_write_position([follower_cfg[n]["id"] for n in joint_names], raw_goals)

except:
     # 1. 모든 모터의 토크 해제 (드라이버가 살아있을 때만 실행)
    if 'driver' in locals() and driver is not None:
        try:
            for f_id in f_ids:
                driver.set_torque(f_id, False)
            print("✅ 모든 모터 토크 해제 완료")
        except Exception as e:
            print(f"⚠️ 토크 해제 중 오류 발생: {e}")

finally:
    cap.release()
    cv2.destroyAllWindows()
    # 1. 모든 모터의 토크 해제 (드라이버가 살아있을 때만 실행)
    if 'driver' in locals() and driver is not None:
        try:
            for f_id in f_ids:
                driver.set_torque(f_id, False)
            print("✅ 모든 모터 토크 해제 완료")
        except Exception as e:
            print(f"⚠️ 토크 해제 중 오류 발생: {e}")

     
