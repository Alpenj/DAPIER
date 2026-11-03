import mujoco as mj
from mujoco.glfw import glfw
import os
import numpy as np  # 수학 연산을 위한 numpy 추가

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# 키보드 제어 상태
key_state = {
    'w': False,
    'a': False,
    's': False,
    'd': False,
}

# 키 입력 콜백 함수
def key_callback(window, key, scancode, action, mods):
    global key_state

    if action == glfw.PRESS:
        if key == glfw.KEY_W:
            key_state['w'] = True
        elif key == glfw.KEY_A:
            key_state['a'] = True
        elif key == glfw.KEY_S:
            key_state['s'] = True
        elif key == glfw.KEY_D:
            key_state['d'] = True

    elif action == glfw.RELEASE:
        if key == glfw.KEY_W:
            key_state['w'] = False
        elif key == glfw.KEY_A:
            key_state['a'] = False
        elif key == glfw.KEY_S:
            key_state['s'] = False
        elif key == glfw.KEY_D:
            key_state['d'] = False

# mouse button callback 
def mouse_button(window, button, act, mods):
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
    glfw.get_cursor_pos(window)

# mouse move callback 
def mouse_move(window, xpos, ypos):
    global lastx, lasty, button_left, button_middle, button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    if (not button_left) and (not button_middle) and (not button_right):
        return

    width, height = glfw.get_window_size(window)
    PRESS_LEFT_SHIFT = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    if button_right:
        action = mj.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        action = mj.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height, dy/height, scene, cam)

# scroll callback
def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 * yoffset, scene, cam)   

# [수정] 업로드 해주신 올바른 XML 파일명으로 변경
xml_name = "scene.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + '/' + xml_name)

model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model) 
cam = mj.MjvCamera()
opt = mj.MjvOption() 

# GLFW 초기화
glfw.init()
window = glfw.create_window(800, 600, "mj Custom Viewer - IMU Visualization", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)

scene = mj.MjvScene(model, maxgeom=1000)
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# 콜백 등록
glfw.set_key_callback(window, key_callback)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

cam.lookat[:] = [0, 0, 0]
cam.distance = 2.0
cam.elevation = -20
cam.azimuth = -130

width, height = glfw.get_framebuffer_size(window)
viewport = mj.MjrRect(0, 0, width, height)

# --- [추가] IMU 자세 추정을 위한 변수 초기화 ---
yaw = 0.0
roll_deg, pitch_deg, yaw_deg = 0.0, 0.0, 0.0
dt = model.opt.timestep  # XML에 정의된 0.001초 timestep

# 루프
while not glfw.window_should_close(window):
    mj.mj_step(model, data)
    
    # 휠 제어 로직
    velocity = 40.0
    left_vel = 0.0
    right_vel = 0.0
    left_handycap = 0.7

    if key_state['w']:
        left_vel += velocity
        right_vel += velocity
    if key_state['s']:
        left_vel -= velocity
        right_vel -= velocity
    if key_state['a']:
        left_vel -= velocity
        right_vel += velocity
    if key_state['d']:
        left_vel += velocity
        right_vel -= velocity

    try:
        left_id = model.actuator(name='left-velocity-servo')
        right_id = model.actuator(name='right-velocity-servo')
        data.ctrl[left_id.id] = left_vel * left_handycap
        data.ctrl[right_id.id] = right_vel
    except Exception as e:
        print("Actuator 이름 오류:", e)

    # --- [추가/수정] IMU 센서 데이터를 이용한 Pitch, Roll, Yaw 계산 ---
    try:
        acc_id = model.sensor(name='imu_acc').id
        gyro_id = model.sensor(name='imu_gyro').id
       
        acc_data = data.sensordata[acc_id:acc_id+3]     # ax, ay, az
        gyro_data = data.sensordata[gyro_id:gyro_id+3]   # wx, wy, wz

        # 1. 가속도계를 사용한 Roll, Pitch 계산
        roll = np.arctan2(acc_data[1], acc_data[2])
        pitch = np.arctan2(-acc_data[0], np.sqrt(acc_data[1]**2 + acc_data[2]**2))
        
        # 2. 자이로스코프 Z축 각속도를 누적 적분하여 Yaw 계산
        yaw += gyro_data[2] * dt

        # 라디안 단위를 도(degree) 단위로 변환
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

    except Exception as e:
        print("IMU 센서 읽기 오류:", e)
    
    # 카메라에서 장면 생성
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL, scene)

    # 렌더링
    mj.mjr_render(viewport, scene, ctx)

    # --- [추가] 시뮬레이션 화면 좌측 상단에 실시간 오버레이 시각화 ---
    overlay_text = f"Roll: {roll_deg:6.1f} deg | Pitch: {pitch_deg:6.1f} deg | Yaw: {yaw_deg:6.1f} deg"
    mj.mjr_overlay(
        mj.mjtFontScale.mjFONTSCALE_150.value, 
        mj.mjtGridPos.mjGRID_TOPLEFT, 
        viewport, 
        overlay_text, 
        "", 
        ctx
    )

    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.destroy_window(window)
glfw.terminate()