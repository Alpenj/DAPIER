import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import json
import time
from motor_control import MiniFeetechDriver # 제시해주신 드라이버 파일

# --- [1] 데이터 변환 함수 (제시해주신 로직 활용) ---
def clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def raw_to_norm(raw, cal):
    """리더 하드웨어 RAW 값을 -100 ~ 100 사이로 정규화"""
    mn, mx = cal["hw_range_min"], cal["hw_range_max"]
    raw_b = clip(raw, mn, mx)
    norm = (((raw_b - mn) / (mx - mn)) * 200.0) - 100.0
    if cal.get("drive_mode", 0): norm = -norm
    return norm

def norm_to_radian(norm, rad_range=(-2.9, 2.9)):
    """정규화된 값(-100~100)을 MuJoCo 라디안 범위로 변환"""
    # -100 -> rad_range[0], 100 -> rad_range[1]
    return ((norm + 100.0) / 200.0) * (rad_range[1] - rad_range[0]) + rad_range[0]

# --- [2] MuJoCo 설정 및 초기화 ---
xml_path = "lift_cube_calibration.xml"
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# Viewer 초기화
glfw.init()
window = glfw.create_window(1200, 900, "ACT Teleop: HW Leader -> MJ Follower", None, None)
glfw.make_context_current(window)
scene = mj.MjvScene(model, maxgeom=1000)
cam = mj.MjvCamera()
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# 카메라 초기 위치 (블록 쪽을 바라보게)
cam.lookat[:] = [0.3, 0, 0.2]
cam.distance = 1.2

# --- [3] 하드웨어(리더) 설정 ---
LEADER_PORT = "/dev/ttyUSB0" # 리더암 포트 (상황에 맞게 수정)
#LEADER_JSON = "shoulder_pan_calibration_leader.json"
LEADER_JSON = "hybrid_calibration.json"

# 캘리브레이션 로드
with open(LEADER_JSON, "r", encoding="utf-8") as f:
    leader_cal_all = json.load(f)
    print(leader_cal_all)
    # 현재는 1개(shoulder_pan)만 있다고 가정, 실제로는 6개 루프 필요
    m1_cal = leader_cal_all["shoulder_pan"]
    LEADER_MOTOR_ID = 1

leader_driver = MiniFeetechDriver(port=LEADER_PORT)
leader_driver.set_torque(m1_cal["id"], False) # 리더는 토크 OFF

# PD Gain (시뮬레이션 추종 성능용)
kp, kd = 100.0, 5.0

print("\n[Teleop Start] 리더 암을 움직여 시뮬레이션을 조종하세요.")

# --- [4] 메인 루프 ---
while not glfw.window_should_close(window):
    time_prev = data.time

    # 1. 리더(HW) 데이터 읽기
    raw_leader = leader_driver.get_position(LEADER_MOTOR_ID)

    if raw_leader is not None:
        # 2. MuJoCo에서 해당 조인트의 실제 범위를 자동으로 가져옴
        jnt_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "shoulder_pan")
        mj_min, mj_max = model.jnt_range[jnt_id]  # XML에 설정된 -1.91, 1.91을 가져옴

        # 3. HW Raw 값을 0.0 ~ 1.0 비율로 변환
        hw_min, hw_max = m1_cal["hw_range_min"], m1_cal["hw_range_max"]
        raw_clipped = np.clip(raw_leader, hw_min, hw_max)
        ratio = (raw_clipped - hw_min) / (hw_max - hw_min)
        
        # 만약 방향이 반대라면 비율을 뒤집음
        if m1_cal.get("drive_mode", 0):
            ratio = 1.0 - ratio

        # 4. 비율을 MuJoCo의 실제 라디안 범위로 변환 (Direct Mapping)
        target_rad = ratio * (mj_max - mj_min) + mj_min

        # 5. 제어 주입
        qpos_curr = data.qpos[model.jnt_qposadr[jnt_id]]
        qvel_curr = data.qvel[model.jnt_dofadr[jnt_id]]
        
        torque = kp * (target_rad - qpos_curr) - kd * qvel_curr
        data.ctrl[0] = torque
    
    '''if raw_leader is not None:
        # 2. Raw -> Norm -> Radian 변환
        norm = raw_to_norm(raw_leader, m1_cal)
        target_rad = norm_to_radian(norm, rad_range=(-3.0, 3.0)) # 관절 범위에 맞게 조정

        # 3. MuJoCo 액추에이터에 토크 주입 (PD Control)
        # 0번 액추에이터가 shoulder_pan이라고 가정
        qpos_curr = data.qpos[0]
        qvel_curr = data.qvel[0]
        
        torque = kp * (target_rad - qpos_curr) + kd * (0.0 - qvel_curr)
        data.ctrl[0] = torque
        
        print(f"Leader Raw: {raw_leader:4d} | Target Rad: {target_rad:.3f}", end='\r')
    '''
    # 4. 시뮬레이션 진행
    while (data.time - time_prev) < (1.0/60.0):
        mj.mj_step(model, data)

    # 5. 렌더링
    viewport = mj.MjrRect(0, 0, *glfw.get_framebuffer_size(window))
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, ctx)
    
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()