import mujoco
import mujoco.viewer
from mujoco.glfw import glfw

# GLFW 초기화
if not glfw.init():
    raise Exception("GLFW 초기화 실패")

# 윈도우 생성 (64x480)
window = glfw.create_window(640, 480, "MuJoCo GLFW Window", None, None)

if not window:
    glfw.terminate()
    raise Exception("GLFW 윈도우 생성 실패")

# OpenGL 컨텍스트 설정
glfw.make_context_current(window)

# 루프: ESC 누를 때까지
while not glfw.window_should_close(window):
    glfw.poll_events()
    glfw.swap_buffers(window)  # 화면 업데이트

# 정리
glfw.destroy_window(window)
glfw.terminate()
