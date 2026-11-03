import mujoco as mj
from mujoco.glfw import glfw
import os

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# mouse button callback 
def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

# mouse move callback 
def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

# scroll callback
def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)   

# MJCF 모델 로딩
xml_name = "scene.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname+'/'+xml_name)
#print(dirname)
#print(abspath)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model) 

# GLFW 초기화
glfw.init()
# 윈도우 생성
window = glfw.create_window(640, 480, "mj Custom Viewer", None, None)
glfw.make_context_current(window)
# mouse callback
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)


# 뷰어 구성 요소 초기화
scene = mj.MjvScene(model, maxgeom=1000)
cam = mj.MjvCamera()
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# 카메라 위치 설정 (optional)
cam.lookat[:] = [0, 0, 0]
cam.distance = 1.5
cam.elevation = -10
cam.azimuth = 90

# 화면 버퍼 크기 조회
width, height = glfw.get_framebuffer_size(window)
viewport = mj.MjrRect(0, 0, width, height)

# 루프
while not glfw.window_should_close(window):
    mj.mj_step(model, data)

    # 카메라에서 장면 생성
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL, scene)

    # 렌더링
    mj.mjr_render(viewport, scene, ctx)

    glfw.swap_buffers(window)
    glfw.poll_events()

# 정리
glfw.destroy_window(window)
glfw.terminate()