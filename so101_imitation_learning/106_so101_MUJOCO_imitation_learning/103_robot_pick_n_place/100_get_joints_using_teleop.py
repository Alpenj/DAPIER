import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time

# --- [설정] 모델 로딩 ---
xml_path = os.path.join(os.path.dirname(__file__), "scene.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
n_actuators = model.nu

# --- [기능] 데이터 로깅용 변수 및 함수 ---
recorded_poses = []

def keyboard_callback(window, key, scancode, action, mods):
    """S 키를 누르면 현재 관절 상태를 출력하고 저장"""
    if action == glfw.PRESS:
        if key == glfw.KEY_S:
            # 현재 6개 액추에이터의 실제 위치(qpos)를 추출
            current_pose = data.qpos[:n_actuators].copy()
            recorded_poses.append(current_pose)
            
            # 출력 (복사해서 코드에 바로 쓸 수 있는 형태)
            pose_str = ", ".join([f"{val:.4f}" for val in current_pose])
            print(f"pose_step_{len(recorded_poses)} = np.array([{pose_str}])")
            
        elif key == glfw.KEY_ENTER:
            print("-" * 30)
            print("전체 시퀀스 리스트:")
            print(f"sequence = [\n    " + ",\n    ".join([f"np.array([{', '.join([f'{v:.4f}' for v in p])}])" for p in recorded_poses]) + "\n]")
            print("-" * 30)

# --- [설정] GLFW 및 카메라 ---
glfw.init()
window = glfw.create_window(1280, 720, "Capture Poses (Press 'S' to Save)", None, None)
glfw.make_context_current(window)
glfw.set_key_callback(window, keyboard_callback) # 키보드 콜백 등록 [cite: 33]

scene = mj.MjvScene(model, maxgeom=1000)
cam = mj.MjvCamera()
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# --- 메인 루프 ---
print("\n[작업 시작] 리더 로봇을 움직여 위치를 잡은 후 'S' 키를 눌러 포즈를 저장하세요.")
print("순서: Ready -> Approach -> Pick -> Grasp -> Lift -> Place -> Release")

while not glfw.window_should_close(window):
    # 텔레오퍼레이션 로직 (리더 하드웨어의 값을 data.ctrl에 주입하는 부분) [cite: 101, 154]
    # (여기에 기존 텔레오퍼레이션 통신 코드를 넣으시면 됩니다)
    
    mj.mj_step(model, data) # 물리 연산 [cite: 66, 105]

    # 렌더링 [cite: 32, 197]
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL, scene)
    mj.mjr_render(viewport, scene, ctx)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()