import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import json
import time
import cv2 # [추가] 제2 카메라 시각화용
from motor_control import MiniFeetechDriver

# --- [1] 데이터 변환 및 매핑 설정 ---
def clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def raw_to_norm(raw, cal):
    mn, mx = cal["range_min"], cal["range_max"]
    raw_b = clip(raw, mn, mx)
    norm = (((raw_b - mn) / (mx - mn)) * 200.0) - 100.0
    if cal.get("drive_mode", 0): norm = -norm
    return norm

JOINT_MAPPING = [
    {"name": "shoulder_pan",   "id": 1},
    {"name": "shoulder_lift",  "id": 2},
    {"name": "elbow_flex",     "id": 3},
    {"name": "wrist_flex",     "id": 4},
    {"name": "wrist_roll",     "id": 5},
    {"name": "gripper",        "id": 6}
]

# --- [2] MuJoCo 설정 및 초기화 ---
xml_path = "lift_cube_calibration.xml"
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# [추가] 손목 카메라 설정
wrist_cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, 'wrist_camera_sensor')
cam_width, cam_height = 320, 240 # PiP 및 OpenCV 창 해상도

glfw.init()
window = glfw.create_window(1280, 720, "ACT Teleop: Dual Camera Mode", None, None)
glfw.make_context_current(window)

scene = mj.MjvScene(model, maxgeom=1000)
cam = mj.MjvCamera()
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# [추가] 오프스크린 렌더링용 전용 Scene (독립적 시점 관리용)
off_scene = mj.MjvScene(model, maxgeom=1000)

cam.lookat[:] = [0.3, 0, 0.2]
cam.distance = 0.5

# --- [3] 하드웨어(리더) 설정 ---
LEADER_PORT = "/dev/ttyUSB0" 
LEADER_JSON = "full_arm_calibration_leader.json"

with open(LEADER_JSON, "r", encoding="utf-8") as f:
    leader_cal_all = json.load(f)

leader_driver = MiniFeetechDriver(port=LEADER_PORT)

for j_info in JOINT_MAPPING:
    leader_driver.set_torque(j_info["id"], False)

kp, kd = 80, 5 #150.0, 10.0

# --- [추가] 카메라 캡처 및 PiP 렌더링 함수 ---
def process_wrist_camera(main_ctx):
    """손목 카메라를 렌더링하여 PiP로 표시하고 이미지를 반환"""
    # 1. 손목 카메라 시점 설정
    off_cam = mj.MjvCamera()
    off_cam.type = mj.mjtCamera.mjCAMERA_FIXED
    off_cam.fixedcamid = wrist_cam_id
    
    # 2. PiP 영역 설정 (왼쪽 하단 0,0 좌표에 320x240)
    pip_rect = mj.MjrRect(0, 0, cam_width, cam_height)
    
    # 3. 동일한 컨텍스트(main_ctx)를 사용하여 메인 버퍼 위에 덧그리기
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, off_cam, mj.mjtCatBit.mjCAT_ALL.value, off_scene)
    mj.mjr_render(pip_rect, off_scene, main_ctx)
    
    # 4. 데이터 캡처 (배열로 읽기)
    rgb = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
    mj.mjr_readPixels(rgb, None, pip_rect, main_ctx)
    return np.flipud(rgb)

print("\n[Teleop Start] 6축 동기화 및 듀얼 카메라 가동 중...")

# --- [4] 메인 루프 ---
while not glfw.window_should_close(window):
    time_prev = data.time

    # 1. 하드웨어 제어 로직 (기존과 동일)
    for i, j_info in enumerate(JOINT_MAPPING):
        j_name = j_info["name"]
        m_id = j_info["id"]
        raw_leader = leader_driver.get_position(m_id)
        
        if raw_leader is not None:
            cal = leader_cal_all[j_name]
            norm = raw_to_norm(raw_leader, cal)
            mj_jnt_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, j_name)
            mj_min, mj_max = model.jnt_range[mj_jnt_id]
            target_rad = ((norm + 100.0) / 200.0) * (mj_max - mj_min) + mj_min

            qpos_idx = model.jnt_qposadr[mj_jnt_id]
            qvel_idx = model.jnt_dofadr[mj_jnt_id]
            qpos_curr = data.qpos[qpos_idx]
            qvel_curr = data.qvel[qvel_idx]
            
            if j_name == "gripper":
                torque = 200.0 * (target_rad - qpos_curr) - 3.0 * qvel_curr
            else:
                torque = kp * (target_rad - qpos_curr) - kd * qvel_curr
            data.ctrl[i] = torque

    # 2. 물리 시뮬레이션
    while (data.time - time_prev) < (1.0/60.0):
        mj.mj_step(model, data)

    # 3. 메인 화면 렌더링 (전체 뷰)
    viewport = mj.MjrRect(0, 0, *glfw.get_framebuffer_size(window))
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, ctx)
    
    # 4. [추가] 제2 카메라 처리 (PiP 생성 및 데이터 획득)
    wrist_img_rgb = process_wrist_camera(ctx)
    
    # 5. [추가] OpenCV 창 업데이트
    wrist_img_bgr = cv2.cvtColor(wrist_img_rgb, cv2.COLOR_RGB2BGR)
    cv2.imshow("Wrist Camera (Robot Eye)", wrist_img_bgr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    # 6. 최종 버퍼 교체 (메인 뷰 + PiP가 합쳐진 상태)
    glfw.swap_buffers(window)
    glfw.poll_events()

cv2.destroyAllWindows()
glfw.terminate()