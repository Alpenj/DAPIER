import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time

# 1. 모델 로딩 및 초기화 (기존 코드 동일)
xml_path = os.path.join(os.path.dirname(__file__), "scene.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# 액추에이터 개수 확인
n_actuators = model.nu

def get_trajectory(start_q, end_q, steps, noise_level=0.02):
    """
    시작 지점에서 끝 지점까지의 직선 보간 궤적 생성 (노이즈 포함)
    """
    traj = np.linspace(start_q, end_q, steps)
    # 궤적에 약간의 랜덤 노이즈 추가 (데이터 다양성 확보)
    noise = np.random.normal(0, noise_level, traj.shape)
    return traj + noise

# 시퀀스 정의 (각 관절의 목표 각도 리스트)
# 예: [관절1, 관절2, ..., 관절N]
pose_ready = np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0])
pose_target_1 = np.array([0.5, -0.8, 1.0, 0.2, 0.5, 0.0])
pose_target_2 = np.array([-0.5, -0.4, 0.3, -0.2, -0.5, 0.0])

sequence = [pose_ready, pose_target_1, pose_target_2, pose_ready]

# GLFW 및 렌더링 설정 생략 (기존 코드와 동일)
glfw.init()
window = glfw.create_window(1280, 720, "SO-101 Trajectory Control", None, None)
glfw.make_context_current(window)
scene = mj.MjvScene(model, maxgeom=1000)
cam = mj.MjvCamera()
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# 메인 루프 변수
step_idx = 0
target_idx = 0
current_traj = get_trajectory(data.qpos[:n_actuators], sequence[0], 100)

while not glfw.window_should_close(window):
    # --- 시퀀스 제어 로직 ---
    if step_idx < len(current_traj):
        # 현재 궤적의 목표값을 액추에이터에 입력
        data.ctrl[:n_actuators] = current_traj[step_idx]
        step_idx += 1
    else:
        # 다음 목표 포즈로 변경
        target_idx = (target_idx + 1) % len(sequence)
        start_pose = data.qpos[:n_actuators].copy()
        end_pose = sequence[target_idx]
        
        # 새로운 랜덤 궤적 생성 (매번 조금씩 다름)
        current_traj = get_trajectory(start_pose, end_pose, 120, noise_level=0.05)
        step_idx = 0
        print(f"Moving to Target {target_idx}")

    # 물리 시뮬레이션 진행
    mj.mj_step(model, data)

    # --- 렌더링 ---
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL, scene)
    mj.mjr_render(viewport, scene, ctx)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()