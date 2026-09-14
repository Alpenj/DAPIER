import mujoco as mj
from mujoco.glfw import glfw
import os
import time

# MJCF 모델 로딩
xml_name = "2wheel_robot.xml"
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
if not window:
    glfw.terminate()
    raise Exception("GLFW 윈도우 생성 실패")

glfw.make_context_current(window)

# 뷰어 구성 요소 초기화
scene = mj.MjvScene(model, maxgeom=1000)
cam = mj.MjvCamera()
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# 카메라 위치 설정 (optional)
cam.lookat[:] = [0, 0, 0]
cam.distance = 2.0
cam.elevation = -20
cam.azimuth = 90

# 화면 버퍼 크기 조회
width, height = glfw.get_framebuffer_size(window)
viewport = mj.MjrRect(0, 0, width, height)

# 루프
while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev) < (1.0/60.0):
        mj.mj_step(model, data)
    # 카메라에서 장면 생성
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL, scene)

    # 렌더링
    mj.mjr_render(viewport, scene, ctx)

    glfw.swap_buffers(window)
    glfw.poll_events()

# 정리
glfw.destroy_window(window)
glfw.terminate()
