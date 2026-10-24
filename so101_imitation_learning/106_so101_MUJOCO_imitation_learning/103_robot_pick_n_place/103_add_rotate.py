import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

# 전역 변수 설정
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0
hip = 0.5
knee = 0.1

# PD gains
kp = 80.0
kd = 2.0

# --- Callback Functions ---
def keyboard(window, key, scancode, act, mods):
    global knee
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
    elif (act == glfw.PRESS and key == glfw.KEY_W):
        knee += 0.1
        if knee > 1.5: knee = 1.7
    elif (act == glfw.PRESS and key == glfw.KEY_S):
        knee -= 0.1
        if knee < -0.2: knee = 0.0

def mouse_button(window, button, act, mods):
    global button_left, button_middle, button_right
    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    global lastx, lasty
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    if not (button_left or button_middle or button_right):
        return

    width, height = glfw.get_window_size(window)
    mod_shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or
                 glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)

    # 수정 포인트: mj.mjtMouse 상수의 올바른 참조와 .value 사용
    if button_right:
        action = mj.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        action = mj.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    # mjv_moveCamera는 두 번째 인자로 정수형(action.value)을 받습니다.
    mj.mjv_moveCamera(model, action.value, dx/height, dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM.value
    mj.mjv_moveCamera(model, action, 0.0, -0.05 * yoffset, scene, cam)

# --- Initialize MuJoCo ---
xml_path = "scene.xml"
abspath = os.path.join(os.path.dirname(__file__), xml_path)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)

# --- Viewer Setup ---
glfw.init()
window = glfw.create_window(1200, 900, "Unitree Stand Control", None, None)
glfw.make_context_current(window)

cam = mj.MjvCamera()
mj.mjv_defaultCamera(cam)
opt = mj.MjvOption()
mj.mjv_defaultOption(opt)

scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# --- Main Loop ---
while not glfw.window_should_close(window):
    time_prev = data.time

    # Desired Joint Positions
    desired_positions = np.array([
        hip, knee+0.1,  # Back Right
        hip, knee+0.1,  # Back Left
        hip, knee,      # Front Right
        hip, knee       # Front Left
    ])

    # PD Control
    # sensor 데이터 개수가 actuator 개수와 맞는지 확인이 필요합니다.
   # --- PD CONTROL: compute torques ---
    for i in range(model.nu):
        # data.sensordata 대신 실제 관절 상태인 qpos, qvel을 직접 참조
        # 로봇의 관절 각도는 qpos[0:n_actuators], 속도는 qvel[0:n_actuators]에 있습니다.
        qpos = data.qpos[i]      # 현재 관절 각도
        qvel = data.qvel[i]      # 현재 관절 속도
        
        torque = kp * (desired_positions[i] - qpos) + kd * (0.0 - qvel)
        data.ctrl[i] = torque

    # Simulation Step
    while (data.time - time_prev) < (1.0/60.0):
        mj.mj_step(model, data)

    # Render
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()