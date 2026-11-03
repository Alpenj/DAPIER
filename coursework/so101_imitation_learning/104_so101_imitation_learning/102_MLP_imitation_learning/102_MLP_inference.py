import torch
import torch.nn as nn
import cv2
import json
import time
import numpy as np
from torchvision import models
from torchvision.models import resnet18, ResNet18_Weights
from motor_control import MiniFeetechDriver
from torchvision import transforms # 상단에 추가 필요

# 1. 학습할 때 사용한 모델 구조와 동일해야 합니다.
class RobotPolicy(nn.Module):
    def __init__(self, action_dim=6):
        super().__init__()
        self.visual_backbone = resnet18(weights=ResNet18_Weights.DEFAULT)
        self.visual_backbone.fc = nn.Identity()
        
        self.policy_head = nn.Sequential(
            nn.Linear(512 + 6, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim)
        )

    def forward(self, image, state):
        img_features = self.visual_backbone(image)
        combined = torch.cat([img_features, state], dim=1)
        return self.policy_head(combined)

class RobotInference:
    def __init__(self, follower_port, model_path="robot_model_final.pth"):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"🚀 추론 엔진 시작: {self.device}")

        # 2. 하드웨어 설정 (팔로워만 제어)
        self.follower = MiniFeetechDriver(port=follower_port)
        with open("full_arm_calibration_follower.json", "r") as f:
            self.follower_cfg = json.load(f)
        
        self.joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
        self.f_ids = [self.follower_cfg[name]["id"] for name in self.joint_names]

        # 3. 모델 로드
        self.model = RobotPolicy(action_dim=6).to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval() # 추론 모드 (Dropout, Batchnorm 고정)
        print(f"✅ 모델 로드 완료: {model_path}")

        self.img_transform = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], 
            std=[0.229, 0.224, 0.225]
        )

    def run(self):
        cap = cv2.VideoCapture(0)
        
        # 팔로워 토크 ON
        for f_id in self.f_ids:
            self.follower.set_torque(f_id, True)

        print("\n[작동 중] 'q'를 누르면 종료합니다.")
        
        try:
            with torch.no_grad(): # 추론 시 gradient 계산 불필요 (속도/메모리 최적화)
                while True:
                    ret, frame = cap.read()
                    if not ret: break

                    # A. 입력 데이터 준비 (Pre-processing)
                    #img_resized = cv2.resize(frame, (224, 224))
                    #img_tensor = torch.from_numpy(img_resized).permute(2, 0, 1).float().unsqueeze(0).to(self.device) / 255.0
                    img_resized = cv2.resize(frame, (224, 224))
                    img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB) # RGB 변환 필수
                    img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).float().to(self.device) / 255.0
                    img_tensor = self.img_transform(img_tensor).unsqueeze(0) # 정규화 적용 및 배치 차원 추가

                    # --- A-2. 현재 상태 읽기 및 정규화 (0~1) ---
                    norm_states = []
                    read_success = True
                    for name in self.joint_names:
                        f_cfg = self.follower_cfg[name]
                        pos = self.follower.get_position(f_cfg["id"])
                        if pos is None:
                            read_success = False
                            break
                        # 현재 위치를 0~1 비율로 변환
                        ratio = (pos - f_cfg["range_min"]) / (f_cfg["range_max"] - f_cfg["range_min"])
                        norm_states.append(max(0.0, min(1.0, ratio)))
                    
                    if not read_success: continue
                    state_tensor = torch.tensor([norm_states], dtype=torch.float32).to(self.device)

                    # --- B. 모델 추론 ---
                    action_pred = self.model(img_tensor, state_tensor) # 결과값은 0~1 사이의 비율
                    pred_ratios = action_pred.squeeze().cpu().numpy()

                    # --- C. 역정규화 (Denormalization: 비율 -> 실제 모터값) ---
                    final_goals = []
                    for i, name in enumerate(self.joint_names):
                        f_cfg = self.follower_cfg[name]
                        # 0~1 비율을 다시 실제 범위(예: 500~2500)로 환산
                        actual_pos = int(pred_ratios[i] * (f_cfg["range_max"] - f_cfg["range_min"]) + f_cfg["range_min"])
                        # 안전을 위한 클램핑
                        actual_pos = max(f_cfg["range_min"], min(f_cfg["range_max"], actual_pos))
                        final_goals.append(actual_pos)

                    # --- D. 하드웨어 제어 ---
                    self.follower.sync_write_position(self.f_ids, final_goals)

                    # 시각화
                    cv2.putText(frame, f"AI Status: Running", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.imshow("Robot AI Inference", frame)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        finally:
            cap.release()
            cv2.destroyAllWindows()
            for f_id in self.f_ids:
                self.follower.set_torque(f_id, False)
            print("종료되었습니다.")

if __name__ == "__main__":
    # 포트 번호 확인 필요
    inference = RobotInference(follower_port="/dev/ttyUSB1")
    inference.run()