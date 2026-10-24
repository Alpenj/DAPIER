import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

# 1. 모델 로딩
xml_path = os.path.join(os.path.dirname(__file__), "scene.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# --- 제어 설정 ---
# 목표 포즈 직접 입력 (로봇의 자유도에 맞춰 리스트 길이를 조절하세요)
pose_A = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
pose_B = np.array([0.0, 0.5, 0.0, 0.0, 0.5, 0.5])

kp = 500.0  # 위치 이득
kd = 50.0   # 속도 이득 (댐핑)
switch_interval = 2.0  # 2초마다 포즈 변경

# GLFW 초기화 및 윈도우 설정 (기존과 동일)
glfw.init()
window = glfw.create_window(800, 600, "Two-Pose Control", None, None)
glfw.make_context_current(window)

# 시각화 객체 생성
scene = mj.MjvScene(model, maxgeom=1000)
cam = mj.MjvCamera()
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# --- [추가] 초기 카메라 위치 설정 ---
cam.lookat[:] = [0.0, 0.0, 0.1] # 카메라가 바라보는 목표 지점 (x, y, z) 
cam.distance = 1.2            # 물체로부터의 거리 
cam.azimuth = 135.0             # 좌우 회전 각도 (도 단위) 
cam.elevation = -20.0          # 위아래 각도 (도 단위)

# 루프
while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev) < (1.0/60.0):
        # [핵심] 시간에 따라 목표 포즈 결정 (나머지 연산 이용)
        if (data.time // switch_interval) % 2 == 0:
            target_qpos = pose_A
        else:
            target_qpos = pose_B

        # PD 제어 계산 [cite: 67, 69]
        # data.ctrl은 로봇의 액추에이터(모터)에 전달되는 힘입니다[cite: 57, 58].
        position_error = target_qpos - data.qpos[:model.nu]
        velocity_error = 0 - data.qvel[:model.nu]
        
        data.ctrl[:model.nu] = (kp * position_error) + (kd * velocity_error)

        # 물리 시뮬레이션 한 스텝 진행 [cite: 23]
        mj.mj_step(model, data)

    # 렌더링 및 화면 업데이트 [cite: 24, 25]
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL, scene)
    mj.mjr_render(viewport, scene, ctx)

    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()