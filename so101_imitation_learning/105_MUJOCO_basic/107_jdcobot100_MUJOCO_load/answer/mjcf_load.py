import mujoco
import glfw
import sys

# 1. GLFW 초기화 및 창 생성
if not glfw.init():
    sys.exit("GLFW를 초기화할 수 없습니다.")

# 창 크기 및 이름 설정
window = glfw.create_window(800, 480, "MuJoCo GLFW Viewer", None, None)
if not window:
    glfw.terminate()
    sys.exit("GLFW 창을 생성할 수 없습니다.")

glfw.make_context_current(window)
glfw.swap_interval(1) # VSync 활성화 (60fps 제한)

# 2. MuJoCo 모델 및 데이터 로드
# 'robot.xml' 경로를 실제 파일 경로로 수정하세요.
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# 3. 시각화 및 카메라 구조체 초기화
cam = mujoco.MjvCamera()           # 추적할 카메라 설정
opt = mujoco.MjvOption()           # 시각화 옵션 (컨텍스트 내부 요소 온/오프)
scene = mujoco.MjvScene(model, maxgeom=1000) # 3D 오브젝트들을 담을 씬
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value) # GPU 렌더링 컨텍스트



# 기본 카메라 시점 자동 설정
mujoco.mjv_defaultCamera(cam)
mujoco.mjv_defaultOption(opt)

cam.lookat[:] = [0, 0, 0]
cam.distance = 0.7
cam.elevation = -25
cam.azimuth = 90

# 4. 렌더링 및 시뮬레이션 루프
while not glfw.window_should_close(window):
    # 실제 물리 시뮬레이션 한 스텝 진행
    mujoco.mj_step(model, data)

    # 현재 창 크기 가져오기 (창 크기가 조절되어도 대응 가능하도록)
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)

    # 3D 씬 업데이트 및 GPU 컨텍스트에 렌더링
    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(viewport, scene, context)

    # 버퍼 교체 및 이벤트 처리
    glfw.swap_buffers(window)
    glfw.poll_events()

# 5. 종료 처리 (메모리 해제)
glfw.terminate()