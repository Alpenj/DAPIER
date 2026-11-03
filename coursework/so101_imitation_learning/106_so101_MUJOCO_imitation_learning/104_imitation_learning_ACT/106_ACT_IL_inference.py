import cv2
import torch
import numpy as np
import mujoco as mj
from mujoco.glfw import glfw
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18, ResNet18_Weights
from torchvision import transforms
from motor_control import MiniFeetechDriver
import json
import os

# --- 1. 모델 구조 정의 (학습 시와 동일해야 함) ---
class ACTPolicy(nn.Module):
    def __init__(self, action_dim=6, state_dim=6, chunk_size=50, hidden_dim=512, nheads=8):
        super().__init__()
        self.chunk_size = chunk_size
        self.action_dim = action_dim
        self.hidden_dim = hidden_dim

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

        # 추론 모드: 고정된 노이즈 z=0 사용
        z = torch.zeros(batch_size, self.hidden_dim).to(image.device)
        
        src = torch.stack([z, img_feat, state_feat], dim=1) 
        out = self.transformer.encoder(src)
        pred_actions = self.action_head(out[:, 0, :]) 
        return pred_actions.view(batch_size, self.chunk_size, self.action_dim)

# --- 2. MuJoCo 보조 함수 ---
def get_image(model, data, scene, ctx):
    rect = mj.MjrRect(0, 0, 640, 480)
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    mj.mjr_render(rect, scene, ctx)
    mj.mjr_readPixels(rgb, None, rect, ctx)
    rgb = np.flipud(rgb)
    return cv2.resize(rgb, (224, 224))

# --- 3. 초기화 및 로드 ---
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
checkpoint_path = "act_robot_model.pth"

model_act = ACTPolicy().to(device)
model_act.load_state_dict(torch.load(checkpoint_path, map_location=device))
model_act.eval()

# MuJoCo 로드
xml_path = "lift_cube_calibration.xml"
mj_model = mj.MjModel.from_xml_path(xml_path)
mj_data = mj.MjData(mj_model)

# Viewer 초기화
glfw.init()
window = glfw.create_window(640, 480, "ACT MuJoCo Inference", None, None)
glfw.make_context_current(window)
scene = mj.MjvScene(mj_model, maxgeom=1000)
cam = mj.MjvCamera()
cam.lookat[:] = [0.35, 0.0, 0.15] 
cam.distance = 0.6
cam.azimuth = 180
cam.elevation = -30
ctx = mj.MjrContext(mj_model, mj.mjtFontScale.mjFONTSCALE_150.value)

# 리더 설정 (상태 입력을 위해 필요할 수 있음, 또는 시뮬레이터 qpos 사용)
with open("full_arm_calibration_leader.json", "r") as f:
    leader_cfg = json.load(f)
joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# --- 4. 메인 루프 ---
print("🚀 MuJoCo 추론 시작...")
try:
    with torch.no_grad():
        while not glfw.window_should_close(window):
            time_prev = mj_data.time
            
            # 1. 입력 관측값 준비 (시뮬레이션 내부 정보 사용)
            img_rgb = get_image(mj_model, mj_data, scene, ctx)
            img_torch = torch.from_numpy(img_rgb).permute(2, 0, 1).float() / 255.0
            img_torch = img_torch.unsqueeze(0).to(device)

            # 현재 관절 상태 정규화 (0~1)
            current_states = []
            for name in joint_names:
                mj_idx = mj.mj_name2id(mj_model, mj.mjtObj.mjOBJ_JOINT, name)
                mj_min, mj_max = mj_model.jnt_range[mj_idx]
                curr_qpos = mj_data.qpos[mj_idx]
                # 물리 범위를 0~1 비율로 변환
                ratio = (curr_qpos - mj_min) / (mj_max - mj_min)
                current_states.append(max(0.0, min(1.0, ratio)))
            
            state_torch = torch.tensor(current_states).float().unsqueeze(0).to(device)

            # 2. ACT 모델 예측
            pred_actions = model_act(img_torch, state_torch) # (1, 50, 6)
            
            # 3. 액션 실행 (첫 번째 액션 선택)
            target_norm_action = pred_actions[0, 0, :].cpu().numpy()

            # 4. 시뮬레이터 제어 주입 (PD Control)
            for i, name in enumerate(joint_names):
                mj_idx = mj.mj_name2id(mj_model, mj.mjtObj.mjOBJ_JOINT, name)
                mj_min, mj_max = mj_model.jnt_range[mj_idx]
                
                # 0~1 정규화 값을 다시 물리 라디안 값으로 복원
                target_rad = target_norm_action[i] * (mj_max - mj_min) + mj_min
                
                # 시뮬레이터의 액추에이터에 토크 주입
                # 실제 데이터 수집 시 사용했던 KP, KD 값과 일치해야 함
                mj_data.ctrl[i] = 100.0 * (target_rad - mj_data.qpos[i]) - 2.0 * mj_data.qvel[i]

            # 5. 시뮬레이션 스텝 및 렌더링
            while (mj_data.time - time_prev) < (1.0/60.0):
                mj.mj_step(mj_model, mj_data)

            mj.mjv_updateScene(mj_model, mj_data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
            glfw.swap_buffers(window)
            glfw.poll_events()

finally:
    glfw.terminate()
    print("✅ 추론 종료")