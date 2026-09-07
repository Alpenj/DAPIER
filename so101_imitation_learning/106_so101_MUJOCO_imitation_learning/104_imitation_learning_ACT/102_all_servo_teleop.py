import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import json
import time
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

# SO-101 리더 ID와 MuJoCo 조인트 이름 매핑 (사용자 환경에 맞게 순서 조정)
# 리더의 각 부위별 이름을 키로 사용합니다.
JOINT_MAPPING = [
    {"name": "shoulder_pan",   "id": 1},
    {"name": "shoulder_lift",  # shoulder_pan
     "id": 2},
    {"name": "elbow_flex",     "id": 3},
    {"name": "wrist_flex",     "id": 4},
    {"name": "wrist_roll",     "id": 5},
    {"name": "gripper",        "id": 6}
]

# --- [2] MuJoCo 설정 및 초기화 ---
xml_path = "lift_cube_calibration.xml"
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# Viewer 설정
glfw.init()
window = glfw.create_window(1280, 720, "ACT Full Teleop: 6-DOF Sync", None, None)
glfw.make_context_current(window)
scene = mj.MjvScene(model, maxgeom=1000)
cam = mj.MjvCamera()
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# 카메라 위치 (로봇과 탁자가 잘 보이게)
cam.lookat[:] = [0.35, 0.0, 0.05] 
cam.distance = 0.65
cam.azimuth = 180
cam.elevation = -40

# --- [3] 하드웨어(리더) 설정 ---
LEADER_PORT = "/dev/ttyUSB0" 
LEADER_JSON = "full_arm_calibration_leader.json" # 모든 관절 데이터가 포함된 JSON

with open(LEADER_JSON, "r", encoding="utf-8") as f:
    leader_cal_all = json.load(f)

leader_driver = MiniFeetechDriver(port=LEADER_PORT)

# 모든 리더 모터 토크 OFF 및 초기 설정 확인
for j_info in JOINT_MAPPING:
    m_id = j_info["id"]
    leader_driver.set_torque(m_id, False)

# PD 제어 게인 (모든 관절 공통 적용, 필요시 관절별 차등 가능)
kp, kd = 500.0, 30.0

print("\n[Teleop Start] 6축 전체 동기화 중... 리더를 움직이세요.")

# --- [4] 메인 루프 ---
while not glfw.window_should_close(window):
    time_prev = data.time

    # 6개 관절에 대해 반복 처리
    for i, j_info in enumerate(JOINT_MAPPING):
        j_name = j_info["name"]
        m_id = j_info["id"]
        
        # 1. 리더 데이터 읽기
        raw_leader = leader_driver.get_position(m_id)
        
        if raw_leader is not None:
            # 2. Raw -> Norm (-100 ~ 100)
            cal = leader_cal_all[j_name]
            norm = raw_to_norm(raw_leader, cal)
            
            # 3. MuJoCo 조인트 범위 읽기 및 매핑
            # XML에 설정된 해당 조인트의 물리적 한계를 가져옴
            mj_jnt_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, j_name)
            mj_min, mj_max = model.jnt_range[mj_jnt_id]
            
            # Norm -> Radian 변환 (1:1 매핑)
            target_rad = ((norm + 100.0) / 200.0) * (mj_max - mj_min) + mj_min

            # 4. PD 제어 (토크 주입)
            # qpos와 ctrl 인덱스는 보통 XML의 순서를 따릅니다 (0~5)
            qpos_idx = model.jnt_qposadr[mj_jnt_id]
            qvel_idx = model.jnt_dofadr[mj_jnt_id]
        
            qpos_curr = data.qpos[qpos_idx]
            qvel_curr = data.qvel[qvel_idx]
            if j_name == "gripper":
                # 그리퍼는 블록을 꽉 쥐어야 하므로 매우 높은 Kp(1000)를 사용하여 목표 각도까지 강하게 밀어붙입니다.
                torque = 1000.0 * (target_rad - qpos_curr) - 5.0 * qvel_curr
            else:
                # 나머지 관절은 기존 설정대로 부드럽게 움직입니다.
                torque = kp * (target_rad - qpos_curr) - kd * qvel_curr
            
            torque = kp * (target_rad - qpos_curr) + kd * (0.0 - qvel_curr)
            data.ctrl[i] = torque

    # 5. 시뮬레이션 물리 스텝
    while (data.time - time_prev) < (1.0/60.0):
        mj.mj_step(model, data)

    # 6. 렌더링
    viewport = mj.MjrRect(0, 0, *glfw.get_framebuffer_size(window))
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, ctx)
    
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()