import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import cv2  # OpenCV 추가

# 1. 모델 로딩 및 초기화
# scene.xml이 있는 경로를 정확히 지정하세요.
xml_path = os.path.join(os.path.dirname(__file__), "lift_cube_calibration.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# --- [추가] 카메라 및 센서 설정 ---
# XML에서 정의한 카메라 이름을 찾습니다.
wrist_cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, 'wrist_camera_sensor')
cam_width, cam_height = 320, 240  # 보조 창 해상도

n_actuators = model.nu

def get_trajectory(start_q, end_q, steps, noise_level=0.02):
    traj = np.linspace(start_q, end_q, steps)
    noise = np.random.normal(0, noise_level, traj.shape)
    return traj + noise

# 시퀀스 정의
pose_ready = np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0])
pose_target_1 = np.array([0.5, -0.8, 1.0, 0.2, 0.5, 0.0])
pose_target_2 = np.array([-0.5, -0.4, 0.3, -0.2, -0.5, 0.0])
sequence = [pose_ready, pose_target_1, pose_target_2, pose_ready]

# GLFW 설정
glfw.init()
window = glfw.create_window(1280, 720, "SO-101 Main View", None, None)
glfw.make_context_current(window)

# 렌더링 컨텍스트 초기화
scene = mj.MjvScene(model, maxgeom=1000)
cam = mj.MjvCamera()  # 메인 창용 자유 시점 카메라
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# 오프스크린용 장면 및 뷰포트 설정
off_scene = mj.MjvScene(model, maxgeom=1000)
off_viewport = mj.MjrRect(0, 0, cam_width, cam_height)

def capture_wrist_camera():
    """메모리 상에서 손목 카메라 영상을 렌더링하고 배열로 반환"""
    off_cam = mj.MjvCamera()
    off_cam.type = mj.mjtCamera.mjCAMERA_FIXED
    off_cam.fixedcamid = wrist_cam_id
    
    # 해당 시점으로 장면 업데이트
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, off_cam, mj.mjtCatBit.mjCAT_ALL, off_scene)
    mj.mjr_render(mj.MjrRect(0, 0, cam_width, cam_height), off_scene, ctx)
    
    # 픽셀 읽기
    rgb = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
    mj.mjr_readPixels(rgb, None, mj.MjrRect(0, 0, cam_width, cam_height), ctx)
    return np.flipud(rgb)

# 메인 루프 변수
step_idx = 0
target_idx = 0
current_traj = get_trajectory(data.qpos[:n_actuators], sequence[0], 100)

while not glfw.window_should_close(window):
    # --- 시퀀스 제어 로직 ---
    if step_idx < len(current_traj):
        data.ctrl[:n_actuators] = current_traj[step_idx]
        step_idx += 1
    else:
        target_idx = (target_idx + 1) % len(sequence)
        start_pose = data.qpos[:n_actuators].copy()
        end_pose = sequence[target_idx]
        current_traj = get_trajectory(start_pose, end_pose, 120, noise_level=0.05)
        step_idx = 0

    # 물리 시뮬레이션
    mj.mj_step(model, data)

    # --- [수정] 1. 메인 화면 렌더링 ---
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    main_viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL, scene)
    mj.mjr_render(main_viewport, scene, ctx)

    # --- [수정] 2. 제 2 카메라(OpenCV 창) 처리 ---
    wrist_rgb = capture_wrist_camera()
    # OpenCV는 BGR 형식을 사용하므로 RGB를 BGR로 변환
    #wrist_bgr = cv2.cvtColor(wrist_rgb, cv2.COLOR_RGB2BGR)
    
    # 별도 윈도우에 이미지 출력
    #cv2.imshow("Wrist Camera View (Robot Eye)", wrist_bgr)
    
    # OpenCV 이벤트를 처리하기 위해 waitKey 호출 (1ms 대기)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break

    glfw.swap_buffers(window)
    glfw.poll_events()

# 종료 처리
cv2.destroyAllWindows()
glfw.terminate()