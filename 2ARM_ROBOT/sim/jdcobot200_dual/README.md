# JDcobot200 양팔 MuJoCo 기준 모델

> **Legacy reference:** 현재 실물은 SO-101 두 팔이다. 이 모델을 실물 명령·관절 제한·카메라
> 역할의 근거로 사용하지 않는다. 현재 정본은 `2ARM_ROBOT/config/hardware_roles.json`과
> `sim/mobile_dual_so101`이다.

이 폴더는 강사 저장소의 JDcobot200 단일 팔 모델을 출발점으로, DAPIER의 양팔 제어 의미를
확인하기 위한 시뮬레이션 전용 작업 공간이다. ROS 2 publisher, USB serial, Dynamixel 명령 경로는
포함하지 않는다.

## 현재 확인한 것

- 팔 한 대는 5개 팔 관절과 1개 그리퍼 actuator를 사용한다.
- 양팔 action 순서는 `left_*` 6개 다음 `right_*` 6개인 총 12차원이다.
- 그리퍼는 손가락 관절 2개를 equality constraint로 묶고 actuator 1개로 제어한다.
- `dual_model.py`는 원본 단일 팔을 두 번 읽어 `left_`/`right_` prefix를 붙인다.
- 기본 장착 간격 `0.36 m`는 컴파일 검증용 임시값이다. TurtleBot3 상판 실측값이 아니다.

원본은 그대로 두고 DAPIER 조합 단계에서 팔당 coarse primitive collision 6개를 덧붙인다.

viewer와 smoke reset은 actuator target을 현재 qpos0로 맞춰 원본 초기 그리퍼 자세를 정지 유지한다.

12차원 action 순서는 다음과 같다.

```text
left_base
left_shoulder
left_elbow
left_wrist_pitch
left_wrist_roll
left_gripper_motor
right_base
right_shoulder
right_elbow
right_wrist_pitch
right_wrist_roll
right_gripper_motor
```

## 실행

현재 교육 PC에 이미 준비된 MuJoCo 환경을 사용한다. 이 폴더 때문에 패키지를 새로 설치하지 않는다.

```bash
cd ~/DAPIER/2ARM_ROBOT/sim/jdcobot200_dual
~/DAPIER/so101_imitation_learning/.venv/bin/python dual_model.py --smoke-steps 1000
```

화면에서 확인할 때만 `--viewer`를 추가한다.

```bash
~/DAPIER/so101_imitation_learning/.venv/bin/python dual_model.py --viewer
```

테스트:

```bash
~/DAPIER/so101_imitation_learning/.venv/bin/python -m unittest discover -s test -v
```

## MuJoCo actuator slider → 실물 양팔 teleop

`shoe_mujoco_teleop`은 viewer의 12개 actuator slider를 좌우 팔의 motor ID 1~6에
각각 연결한다. 양팔은 같은 STS3215 구성과 speed 120, acceleration 40, runtime torque
limit 1000(100%) 프로파일을 쓴다. 실행 시작 시 실물 자세를 MuJoCo 기준 자세로 저장하고 이후
slider 변화량을 상대 tick으로 전송한다. 따라서 아직 실측하지 않은 절대 offset을
가정하지 않는다.

hardware preflight 직후에는 선택한 팔의 ID 1~6 모두에 현재 위치 hold를 먼저 전송한다.
slider로 움직이지 않은 관절도 torque ON 상태로 중력 하중을 버티며, viewer 종료 시에는
ID 1~6 전체를 torque OFF한다.

기본 실행은 simulation-only이며 serial을 열거나 패킷을 보내지 않는다.
launcher가 실행되는 동안에는 systemd `idle`과 GNOME `idle/suspend` inhibitor를 함께
유지해 자동 화면 절전으로 USB serial이 reset되지 않게 한다. viewer가 끝나면 inhibitor도
자동 해제된다. 사용자가 직접 누르는 suspend/power 동작은 막지 않는다.

```bash
bash scripts/run_mujoco_hardware_teleop.sh
```

실물 연결은 `--hardware-dispatch-authorized`와 정확한 확인 문자열을 함께 전달해야 한다.
좌우 중 한 팔만 지정할 수도 있고 두 팔을 모두 지정할 수도 있다.

```bash
bash scripts/run_mujoco_hardware_teleop.sh \
  --hardware-dispatch-authorized \
  --left-port /dev/serial/by-id/LEFT_CONTROLLER \
  --left-expected-serial LEFT_SERIAL \
  --right-port /dev/serial/by-id/RIGHT_CONTROLLER \
  --right-expected-serial RIGHT_SERIAL \
  --confirm AUTHORIZE_MUJOCO_JDCOBOT_TELEOP
```

viewer를 닫거나 `Ctrl-C`를 누르거나 telemetry interlock이 동작하면 양팔 ID 1~6에
torque off를 보내고 다시 읽는다. 각 motor의 EEPROM position limit 전체를 허용하고,
slider가 그 범위를 넘으면 해당 목표만 저장된 `0` 또는 `4095` 끝점으로 포화한다.
11~13V 밖의 전압, 45°C 초과, 650mA 초과, load 35% 초과, hardware error는 dispatch를
중단한다. 좌우 물리 장착에 따른 절대 부호와 offset은 별도 실측 전까지 보정하지 않는다.

## 아직 확정하지 않은 것

- TurtleBot3 상판 기준 좌우 팔의 `xyz/rpy`
- 장착 브래킷과 카메라 좌표
- 링크별 질량, 무게중심, 관성
- primitive collision과 신발 접촉 파라미터
- 실기체별 encoder zero, 회전 부호, soft/hard limit
- actuator gain, damping, friction, latency, current/torque limit

원본 MJCF는 대부분의 mesh collision을 꺼 둔 상태다. 현재 코드는 링크와 그리퍼에 실측 전 coarse
primitive collision 6개를 팔마다 추가한다. --no-primitive-collisions 옵션으로 이 overlay를 끌 수 있다.
primitive 치수 역시 아직 실측값이 아니므로 현재 모델은 형상·관절·action contract와 초기 접촉
파이프라인 검증용이다. 신발을 집는 학습이나 sim-to-real 동역학 모델로 바로 사용하면 안 된다.

## 다음 실측

1. TurtleBot3 상판 기준으로 두 팔 베이스 중심의 좌표와 방향을 측정한다.
2. 각 링크와 브래킷의 질량 및 무게중심을 기록한다.
3. 저속 미세동작으로 관절 ID, 부호, encoder zero와 안전 limit을 확인한다.
4. primitive collision부터 추가해 양팔 self-collision과 상판 간섭을 확인한다.
5. 실기체 step response와 비교해 관절별 gain, damping, friction, latency를 맞춘다.

원본과 사용 허가 기록은 [THIRD_PARTY_NOTICE.md](THIRD_PARTY_NOTICE.md)에 남긴다.
