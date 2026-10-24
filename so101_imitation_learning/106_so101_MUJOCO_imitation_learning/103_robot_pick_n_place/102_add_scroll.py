import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

# --- 콜백 함수 정의 ---
def scroll_callback(window, xoffset, yoffset):
    # yoffset: 휠을 위로 굴리면 양수(줌 인), 아래로 굴리면 음수(줌 아웃)
    # 카메라의 distance 값을 조절 (최소 거리 0.2 등으로 제한 권장)
    cam.distance -= yoffset * 0.05
    if cam.distance < 0.1: 
        cam.distance = 0.1

# 1. 모델 로딩 및 초기화
xml_path = os.path.join(os.path.dirname(__file__), "scene.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

n_actuators = model.nu

def get_trajectory(start_q, end_q, steps, noise_level=0.02):
    traj = np.linspace(start_q, end_q, steps)
    noise = np.random.normal(0, noise_level, traj.shape)
    return traj + noise

# 시퀀스 (그리퍼 인덱스가 포함되어 있다면 값을 추가해야 함)
pose_ready = np.zeros(n_actuators)
pose_target_1 = np.array([0.5, -0.8, 1.0, 0.2, 0.5, 0.0] + [0.0]*(n_actuators-6))
sequence = [pose_ready, pose_target_1]

# GLFW 초기화
glfw.init()
window = glfw.create_window(1280, 720, "SO-101 Zoom Control", None, None)
glfw.make_context_current(window)

# --- 뷰어 구성 요소 및 카메라 초기설정 ---
scene = mj.MjvScene(model, maxgeom=1000)
cam = mj.MjvCamera()
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# 카메라 초기 위치
cam.lookat[:] = [0.3, 0, 0.2] # 탁자 근처를 바라보게 설정
cam.distance = 1.0
cam.elevation = -30
cam.azimuth = 90

# 마우스 콜백 등록
glfw.set_scroll_callback(window, scroll_callback)

step_idx = 0
target_idx = 0
current_traj = get_trajectory(data.qpos[:n_actuators], sequence[0], 100)

while not glfw.window_should_close(window):
    if step_idx < len(current_traj):
        data.ctrl[:n_actuators] = current_traj[step_idx]
        step_idx += 1
    else:
        target_idx = (target_idx + 1) % len(sequence)
        start_pose = data.qpos[:n_actuators].copy()
        end_pose = sequence[target_idx]
        current_traj = get_trajectory(start_pose, end_pose, 120, noise_level=0.05)
        step_idx = 0

    mj.mj_step(model, data)

    # 렌더링
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL, scene)
    mj.mjr_render(viewport, scene, ctx)
    
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()