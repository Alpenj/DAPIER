import mujoco
import glfw
import time
import numpy as np

# GLFW 윈도우 크기 설정
WIDTH, HEIGHT = 1200, 900

def main():
    # 1. GLFW 초기화 및 창 생성
    if not glfw.init():
        return

    window = glfw.create_window(WIDTH, HEIGHT, "MuJoCO GLFW - Robot Pose Hold", None, None)
    if not window:
        glfw.terminate()
        return

    glfw.make_context_current(window)
    glfw.swap_interval(1) # V-Sync 활성화 (화면 주사율에 맞춤)

    # 2. MuJoCO 모델 및 데이터 로드 (XML 내부 파라미터 적용 상태)
    model = mujoco.MjModel.from_xml_path('scene.xml')
    data = mujoco.MjData(model)

    # 3. MuJoCO 시각화 및 카메라/옵션 객체 생성
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

    # 기본 카메라 시점 설정 (로봇이 잘 보이도록 약간 뒤/위로 조정)
    mujoco.mjv_defaultCamera(cam)
    cam.distance = 1.5
    cam.elevation = -20
    cam.azimuth = 135

    # 4. 시뮬레이션을 한 스텝 진행하여 로봇의 현재 초기 각도(정지 상태 자세) 획득
    mujoco.mj_step(model, data)
    
    # 0~6번 인덱스는 Freejoint(공중 베이스 위치)이므로, 7번 이후의 실제 8개 다리 관절 각도 추출
    initial_joint_angles = np.copy(data.qpos[7:])
    print(f"로봇 고정 관절 각도(rad): {initial_joint_angles}")

    # 실시간 동기화를 위한 시간 설정
    last_time = time.time()

    # 5. 메인 GLFW 렌더링 및 제어 루프
    while not glfw.window_should_close(window):
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # [제어 핵심] 초기 정지 각도를 모든 모터(actuator) 입력에 지속적으로 주입
        # XML에 세팅된 HighTorqueServo 물리 상수(kp, kv)에 의해 이 위치를 굳건히 버팁니다.
        data.ctrl[:] = initial_joint_angles

        # 시뮬레이션 타임스텝에 맞춰 물리 연산 진행
        # (루프 주기 dt 동안 시뮬레이션의 timestep만큼 쪼개어 연산 수행)
        sim_steps = int(np.round(dt / model.opt.timestep))
        for _ in range(max(1, sim_steps)):
            mujoco.mj_step(model, data)

        # GLFW 창 크기 업데이트 (창 크기 조절 대응)
        width, height = glfw.get_framebuffer_size(window)
        viewport = mujoco.MjrRect(0, 0, width, height)

        # MuJoCO 씬(Scene) 업데이트 및 렌더링
        mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_render(viewport, scene, context)

        # 버퍼 교체 및 이벤트 처리 (화면 갱신)
        glfw.swap_buffers(window)
        glfw.poll_events()

    # 6. 종료 처리 및 자원 해제
    glfw.terminate()

if __name__ == "__main__":
    main()