# Waffle Pi + SO-101 양팔 MuJoCo 모델

JDcobot200 코드를 삭제하지 않고 별도 경로에 만든 SO-101 양팔 기준 모델이다. 팔 하나는
`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`
6개 actuator를 가지며, 좌우를 합친 action은 12차원이다.

이 코드는 MuJoCo 안에서만 동작한다. serial port를 열지 않으며 ROS 2 publisher, 모터
command, hardware dispatch 경로가 없다.

실물 계획에 맞춰 TurtleBot3 LiDAR는 기본적으로 제거한다. 원본 형상과 비교할 때만
`--include-lidar`를 사용한다.

원본 Waffle Pi의 작은 `camera_link`도 제거했다. 전면 중앙에는 AADJA1300GX가
Orbbec Astra 계열로 관측된 사실을 바탕으로 Astra 공식 보수 외형
(165 x 48 x 40 mm), 질량 310 g을 둔다. 정확한 외형과 optical origin을 실측하지
않았으므로 현재 카메라 형상은 확정 CAD가 아니다. 광학 중심은 base 기준
`(0.120, 0, 0.200) m`, 아래쪽 10도로 배치했다.

## 선택형 중앙 STEP 지지대 layout

기존 `printed-torso`를 기본값으로 보존하고, 조원이 분할한 `assem_base.step` 기반 중앙
지지대는 호환성을 위해 기존 CLI 이름인 `--mount-layout tower`로 선택한다. MuJoCo
visual은 조원이 제공한 하부·상부 STL을 수정 없이 사용한다. upper 좌우 소켓은 stock
SO-101의 큰 베이스 출력물 `base_so101_v2`만 대신한다. `base_motor_holder`, Waveshare
plate, base servo와 shoulder 이후 관절 체인은 원본 조립 상태로 큰 홀 축에 끼운다.
따라서 upper와 `base_so101_v2`만 중복 렌더링하지 않는다. 단순 box는 보이지 않는 collision proxy에만
사용한다. Waffle
좌표 기준과 추출 과정은
[`WAFFLE_COORDINATE_REFERENCE.md`](WAFFLE_COORDINATE_REFERENCE.md)에 기록했다.
현재 기준은 다음과 같다.

- 축: `base_link` 기준 +X 전방, +Y 좌측, +Z 위
- STEP 바닥면: 공식 Waffle 물리 상판 Z=0.0915 m에 직접 접촉
- 하부 STL: `160 x 180 x 170 mm`, assembly Z=`0~170 mm`
- 원본 상부 STL: `110.963 x 254 x 156 mm`, assembly Z=`160~316 mm`
- 상부 STL: `110.963 x 254 x 156 mm`, assembly Z=`160~316 mm`
- 상·하부 결합 중첩: `10 mm`
- upper socket 큰 홀 중심 datum: base_link local `(-64, +/-127, 394.050896) mm`
- 사진의 SO-101 teardrop hole: `base_so101_v2.stl`의 반지름 8.5 mm 홀 중심축
- SO-101 arm frame: `(-64, +/-93.4, 387.686186) mm`; mesh/XML/holder 회전을
  역산해 teardrop hole 중심을 holder 최상단에 맞춤
- depth camera body center: 전용 mast 위 `(-64, 0, 550) mm`
- camera down tilt: 27도
- 중심 광선의 바닥 교차점: 로봇 전방 약 1.035 m
- 수직 FOV의 바닥 교차 범위: 약 0.421~7.362 m
- 중앙 지지대 가정 질량: 1.50 kg; camera 가정 질량은 별도 0.31 kg

    ~/DAPIER/so101_imitation_learning/.venv/bin/python \
      mobile_dual_so101.py --mount-layout tower \
      --arm-mount-height-m 0.387686186 --smoke-steps 1000

`--arm-mount-x-m`을 생략하면 tower에는 -0.064 m, 기존 printed torso에는 +0.020 m가
각각 적용된다. 팔 frame 간격은 186.8 mm이고 upper socket의 큰 홀 간격은 254 mm다.
tower layout에서는 stock SO-101의 `base_so101_v2`만 제외한다. base motor holder,
Waveshare mounting plate, base servo, `shoulder_pan`과 이후 관절 체인은 그대로 유지한다.
MuJoCo 뷰어의 Reset도 기록된 home action으로 돌아가도록 qpos와 position target을 함께 복원한다.
카메라는 팔 위치를 바꾸지 않고
중앙의 24 x 30 mm 전용 mast와 50 x 60 x 6 mm 경사 interface plate 위 Z=550 mm로
올린다. 실제 카메라 모델이 확인되기 전까지 중앙 체결 위치는 측정 필요 datum이며,
최종 볼트 규격·hole pattern·optical origin과 STEP 실제 재료/질량은 확정값이 아니다.

현재 home 자세의 camera-arm 최소 간격은 약 136 mm다. 하지만 전체 joint range의
무작위 10,000자세에서는 18개가 30 mm clearance를 위반했으므로 unrestricted motion은
허용하지 않는다. camera, mast와 plate는 collision guard의 keep-out 대상으로 유지한다.

2026-08-28에 official Waffle mesh와 provisional 질량으로 4,096 endpoint corner 및
무작위 1,500자세를 계산했다. home 자세 COM의 휠-캐스터 지지다각형 여유는 약
+32 mm였지만, 전체 관절 범위에는 무부하에서도 약 -13 mm의 전도 자세가 남았다.
따라서 이 결과는 지지대 안전 인증이 아니다. 주행 중에는 낮은 transport pose를 쓰고,
작업 pose 허용영역과 base 정지 interlock은 별도로 제한해야 한다.

제작용 CAD/STL/G-code에 필요한 실측값과 출력 순서는
[`TOWER_FABRICATION.md`](TOWER_FABRICATION.md)에 분리했다.

신발 한 짝부터 한 켤레 양팔 정리, 이동·다품종·실패복구까지 전 과정을 순서대로
경험하는 회의안은
[`PROJECT_DIRECTION_ABC_KO.md`](PROJECT_DIRECTION_ABC_KO.md)에 정리했다.

## physics-executed dual-arm IK

[`physics_ik.py`](physics_ik.py)는 pose editor와 실행 경로를 분리한다. DLS IK는 별도
planning `MjData`에서만 qpos를 갱신한다. 실행은 시작 자세 초기화 1회 후 7차
(septic) trajectory의 actuator target만 `data.ctrl`에 넣고 `mj_step`으로 질량, 관성,
중력, damping, actuator force와 접촉을 계산한다. trajectory는 시작/끝의 속도,
가속도와 jerk가 모두 0이며 설정한 target limit에 맞춰 duration을 늘린다.

실행 보고서는 target과 actual의 velocity/acceleration/jerk를 따로 기록하고, actual
limit, torque saturation, 지지다각형, 금지 접촉과 finite state를 모두 통과해야만
`simulation_motion_accepted=true`로 둔다. 이 값도 실물 실행 승인은 아니다.

2026-08-28 provisional actuator 모델에서 양 gripper를 전방/상향 10 mm 움직인 결과는
다음과 같았다.

- IK residual: left 0.307 mm, right 0.482 mm
- runtime qpos write: 0회, physics step: 1,203
- final joint tracking error: 0.000101 rad
- actuator force limit 사용률: 최대 9.63%, 금지 접촉 0회
- 최소 지지다각형 여유: 31.6 mm
- target 최대값: 0.300 rad/s, 0.514 rad/s², 1.79 rad/s³
- pre-settle 후 actual 최대값: 0.300 rad/s, 0.536 rad/s², 52.5 rad/s³

따라서 현재 결과는 **jerk limit FAIL**이며 accepted가 아니다. 느린 target만으로
감추지 않고, 실제 motor/gear friction, backlash, servo update/latency와 controller
response를 식별한 뒤 actuator 모델과 limit를 다시 맞춰야 한다. 2 ms simulation
qacc 차분으로 계산한 raw jerk도 실물 센서 bandwidth와 함께 재정의해야 한다.

    DAPIER_SO101_MJCF=/absolute/path/to/so101_new_calib.xml \
      ~/DAPIER/so101_imitation_learning/.venv/bin/python -m unittest \
      discover -s test -p 'test_physics_ik.py' -v

## 외부 모델 자산

약 16 MB인 SO-101 원본 XML/STL은 이 폴더에 중복 복사하지 않는다. 기본값은 기존
LeRobot 작업공간의 다음 모델을 사용한다.

    ~/DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/
      so101_mujoco/assets/so101_new_calib.xml

다른 checkout에서는 XML과 인접한 `assets/*.stl`을 준비하고 환경변수로 지정한다.

    export DAPIER_SO101_MJCF=/absolute/path/to/so101_new_calib.xml

원본 자산 출처와 해시는
`~/DAPIER/so101/integrations/lerobot_v0_6_so101_mujoco/`에 기록되어 있다. 로더는
파일 존재뿐 아니라 6관절·6 actuator 이름 계약도 검사하고 다르면 중단한다.
실행 보고서는 사용한 XML의 실제 SHA-256과 기록된 upstream SHA-256의 일치 여부를
함께 출력한다. 현재 로컬 source XML은 shoulder 범위 등 실험 수정 이력이 있어 기록된
원본 해시와 일치하지 않으며, 이 코드는 해당 파일을 자동 수정하지 않는다.

### MuJoCo가 없는 PC와 CI에서 검증

집 PC에서는 MuJoCo GUI 없이 코드만 작성해도 된다. Pull Request가 열리면 별도
`MuJoCo simulation tests` workflow가 Apache-2.0 SO-101 원본 자산을 기록된 commit으로
다운로드하고 14개 파일의 SHA-256을 확인한다. 이어서 전체 simulation-only test와
1,000-step smoke test를 실행하고 tower 조립 상태의 정면·좌우 사선·상단·depth-camera
PNG 및 provenance JSON을 Actions artifact로 남긴다. serial, ROS publisher, motor 및
실물 장치에는 접근하지 않는다.

로컬에서 같은 검증을 재현하려면 Python 3.12 환경에서 다음을 실행한다.

    cd ~/DAPIER
    python3 -m pip install --only-binary=:all: --require-hashes \
      -r requirements-mujoco.txt
    export DAPIER_SO101_MJCF="$(scripts/setup-mujoco-sim \
      --dest /tmp/dapier-so101-assets)"
    scripts/verify-mujoco-headless \
      --artifact-dir /tmp/dapier-mujoco-artifacts

artifact 경로는 기존 PNG/JSON이 없는 새 디렉터리를 사용한다. 스크립트는 결과를
덮어쓰지 않으며, 공식 XML 해시가 다르면 테스트 전에 중단한다. GUI 자세 확인과 수동
조작은 MuJoCo가 설치된 개발 PC에서 기존 `--viewer`/`--pose-editor`로 수행한다.
CI 성공은 Linux headless simulation 결과일 뿐 실물 안전·제작 적합성·sim-to-real
성공을 의미하지 않는다.

## 사람형 어깨 배치

두 SO-101 holder를 Waffle Pi 좌우에서 모두 pitch +90도로 세운다. 그 상태에서
holder의 로컬 X축을 기준으로 왼팔은 -90도, 오른팔은 +90도 twist해 양팔이 몸통
중앙이 아니라 각자의 바깥쪽으로 뻗는 사람의 왼팔·오른팔 형상으로 만든다. 같은 팔 두
개처럼 보이지 않게 좌우를 대칭으로 배치한다. shoulder-pan에 가짜 90도
offset을 넣지 않으며 gripperframe의 접근축은 계속 world -Z를 향한다.

SO-101은 짧으므로 예전에 JDcobot 화면 확인에 쓴 0.55 m 높이를 재사용하면 바닥의
신발에 닿지 않는다. 실제 브래킷을 측정하기 전에는 높이를 확정할 수 없으므로 CLI에서
`--arm-mount-height-m`을 필수로 받는다. 현재 모델에서는 0.30 m일 때 screenshot
pose의 gripperframe이 바닥 위 약 59 mm에 있고 팔의 추가 충돌이 없어 provisional
시각화 기준값으로 사용한다.

## 실행

    cd ~/DAPIER/2ARM_ROBOT/sim/mobile_dual_so101
    ~/DAPIER/so101_imitation_learning/.venv/bin/python \
      mobile_dual_so101.py --arm-mount-height-m 0.30 --smoke-steps 1000

화면 확인:

    ~/DAPIER/so101_imitation_learning/.venv/bin/python \
      mobile_dual_so101.py --arm-mount-height-m 0.30 --viewer

MuJoCo `Control` 슬라이더로 양팔 자세를 직접 잡으려면:

    ~/DAPIER/so101_imitation_learning/.venv/bin/python \
      mobile_dual_so101.py --arm-mount-height-m 0.30 --pose-editor

`--save-pose /tmp/new-pose.json`을 함께 주면 기존 파일은 덮어쓰지 않고 새 JSON에만
저장한다. 이 값은 자동으로 실물에 전송되지 않는다.

### simulation-only Control panel + keyboard teleop

`--pose-editor`는 설계 자세를 빠르게 확인하려고 qpos와 control target을 함께 맞추는
kinematic 도구다. 반면 `--teleop`은 Control panel이나 키 입력으로 actuator target만 바꾸고
`mj_step()`이 질량, 관성, 중력, 마찰, damping과 actuator force를 계산하게 한다. 나는
두 경로를 구분해 자세 편집 결과를 물리 실행 결과로 잘못 해석하지 않도록 한다.

현재 중앙 STEP 지지대와 전용 depth-camera mast를 사용하는 model은 main 실행 파일에서
다음처럼 연다.

    DAPIER_SO101_MJCF=/absolute/path/to/so101_new_calib.xml \
      ~/DAPIER/so101_imitation_learning/.venv/bin/python \
      mobile_dual_so101.py --mount-layout tower \
      --arm-mount-height-m 0.387686186 --smoke-steps 0 --teleop

오른쪽 `Control` section을 펼쳐 12개 joint slider로 양팔 자세를 직접 조절하는 방식을
주 조작으로 권장한다. slider 변경은 현재 안전 target에서 새 target까지 충돌 검사를
통과한 뒤 즉시 `data.ctrl`에 채택되며 qpos를 직접 쓰지 않는다. keyboard는 panel에서
잡은 자세를 기준으로 특정 축만 보정하는 선택 입력이다.

선택적인 keyboard 입력은 다음과 같다.

- `←` / `→`: 왼팔·오른팔 선택
- 숫자열 또는 keypad `1`~`6`: shoulder pan, shoulder lift, elbow flex,
  wrist flex, wrist roll, gripper 선택
- `↑` / `↓`: 선택 관절 목표를 증감. 누르고 있으면 viewer의 key-repeat마다 계속 변함
- `O` / `C`: 선택한 팔 gripper 열기·닫기
- `H`: 기록된 양팔 home target 요청
- `Space`: simulation stop 또는 현재 pose hold 상태에서 재개
- `Esc`: viewer 닫기

`L/R`와 `+/-` 별칭은 키 역할이 겹쳐 보이는 문제를 피하려고 제거했다. keyboard
입력은 repeat마다 5도이고 입력 대기는 0 ms다. keyboard 누적 목표는 `data.ctrl`에
즉시 점프시키지 않고 50 Hz에서 최대 45 deg/s, 180 deg/s²의 가속·감속 ramp로
추종한다. 다음 옵션으로 체감 속도를 바꿀 수 있다.

    --teleop-step-deg 5 \
    --teleop-min-key-interval-ms 0 \
    --teleop-max-speed-deg-s 45 \
    --teleop-accel-deg-s2 180

MuJoCo actuator의 공식 관절 범위와 collision guard의 30 mm protected clearance는
해제하지 않는다. panel 목표 경로와 keyboard의 실제 스무딩 제어 경로를 최대 2도
간격으로 각각 검사해 양팔, camera, 중앙 지지대와 TurtleBot 본체 간 간섭을 거부한다.
stop 진입 순간 simulator qpos를 한 번 latch하고
그 target을 유지하며, stop 중 drift를 새 target으로 계속 따라가지 않는다. 이 기능은
serial, ROS, LeRobot hardware API를 import하지 않고 `data.ctrl`만 갱신한다. 따라서
MuJoCo teleop이 실물 SO-101을 움직이지 않는다.

최종 목표인 MuJoCo 병렬 학습에서 이 teleop은 입력 계약을 확인하는 첫 단계다. 다음
단계에서는 RGB-D, joint state, accepted action, monotonic timestamp와 safety rejection을
동일 episode schema로 기록한다. GUI가 없는 병렬 rollout worker는 같은 12차원 action
순서와 guard를 재사용하고, ACT의 Receding Horizon·Temporal Ensembling을 먼저 비교한다.
학습 checkpoint는 simulation success, collision, tracking error, latency와 jerk gate를
통과한 뒤에만 sim-to-real 후보가 된다. 실물 적용은 좌우 독립 calibration, measured
state freshness, velocity/effort limit, watchdog, E-stop과 같은 대화의 명시적 현장 승인
뒤에 별도 bridge가 수행한다. SIM 성공은 HW 성공 근거가 아니다.

## 실물 좌우 매핑

`/dev/ttyACM0` 같은 번호는 재연결 순서에 따라 바뀌므로 사용하지 않는다. 좌우 팔을
하나씩 식별한 뒤 `/dev/serial/by-id/...` 고유 경로를 각각 기록해야 한다. 같은
SO-101 모델이어도 두 팔의 encoder zero/range calibration은 독립적으로 유지한다.

현재 단계에서 확정하지 않은 항목은 실제 카메라 mount/optical datum, STEP 재료별
질량·관성, 좌우 USB ID 매핑, 각 팔의 calibration, 실물 torque/velocity, 신발 파지
trajectory와 sim-to-real이다.

TurtleBot3 위에 실제로 고정하는 구조 초안과 실측 체크리스트는
[`MOUNTING_CONCEPT.md`](MOUNTING_CONCEPT.md)에 기록했다. 완성된 SO-101 전체를
90도로 돌리면 원래의 바닥 체결면도 수직이 된다. 조원 STEP의 큰 원형 홀 중심과
사진의 `base_so101_v2` teardrop hole 중심을 결합 datum으로 사용하고, 중앙 지지대의 바닥면을 Waffle 상판
local Z=91.5 mm에 둔다. 하부 STL 바닥의 네 홀 `(X,Y)=(+/-77,+/-87) mm`는 공식 Waffle
M3 후보와 일치하지 않으므로 별도 adapter plate/coupon이 필요하다. 외곽
outrigger/caster를 추가하는 안은 주행성과 제작성이 나빠 채택하지 않았다. 현재
형상은 아직 제작 도면이 아니다.

## 양팔 충돌 가드와 구조 검증

목표 자세만 검사하지 않고 현재 자세에서 목표까지 기본 2도 간격으로 보간해 좌우 팔,
전면 bare camera, 그리퍼-TurtleBot 본체와 중앙 STEP base/column의 최소
거리를 검사한다. shoulder와 중앙 기둥의 결합부만 의도된 interface로 제외한다.
30 mm 미만이면 fail-closed로
거부하며 이 명령에는 하드웨어 전송 경로가 없다.

    ~/DAPIER/so101_imitation_learning/.venv/bin/python collision_guard.py

정역학 표본 검증은 4,096개 관절 끝점 조합과 한 팔/반대 팔/양팔 비대칭 무작위 자세,
0.5 kg nominal 및 1.0 kg proof payload를 계산한다.

    ~/DAPIER/so101_imitation_learning/.venv/bin/python design_validation.py \
      --random-samples 10000 --collision-samples 2000

연속 관절공간의 모든 실수를 완전 열거한 결과가 아니며, 실물 질량·관성·브레이크 거리와
출력물 강성이 측정되기 전에는 제조 안전 인증이나 실물 제어 승인으로 사용하지 않는다.

## LLM 상위 작업 감독기

[`llm_task_supervisor.py`](llm_task_supervisor.py)는 특정 LLM API에 연결하기 전의
provider-neutral 제어 경계다. LLM 출력은 탐색·접근·집기·handoff·양팔 정렬·놓기·복구·
중단의 typed JSON skill만 허용한다. 관측 sequence/age, base 정지, safety gate,
재시도 예산과 simulation-only 상태를 확인하며 raw joint/torque/velocity·serial·hardware
인자는 거부한다.

이 모듈은 proposal을 검증할 뿐 action을 실행하거나 hardware를 승인하지 않는다. 실제
구조는 LLM supervisor → deterministic task executor → ACT/IK/navigation → 독립 safety
gate 순서이며, safety gate가 항상 최종 거부권을 가진다.

    python -m unittest discover -s test -p 'test_llm_task_supervisor.py' -v

## 상태 기반 신발 task

SO-101 관절명과 gripper frame을 사용하는 별도 21차원 ground-truth observation
환경이다. 기본값은 현재 clearance upper를 쓰는 `tower` 모델이며 12차원 양팔
action을 받는다.

    ~/DAPIER/so101_imitation_learning/.venv/bin/python shoe_task.py \
      --smoke-steps 200

현재 기본 floor shoe `(0.26, 0, 0.015) m`는 가장 가까운 shoulder에서 약 0.497 m로,
0.40 m 거리 envelope 밖이다. 따라서 상태 기반 imitation 데이터 계약은 실행 가능하지만
이 위치의 바닥 신발 집기는 현재 기구 배치로 학습 가능한 task라고 판정하지 않는다.
지지대/팔 높이를 낮추거나, 충돌 없는 가까운 작업면을 별도 설계해야 한다.

## 병렬 MuJoCo rollout

[`parallel_shoe_rollout.py`](parallel_shoe_rollout.py)는 각 worker가 독립 `MjModel`과
`MjData`를 갖는 spawn process 병렬 환경이다. 2026-08-28 로컬 benchmark에서 동일한
4,000 transition을 1 worker는 577.3 transition/s, 4 workers는 910.5 transition/s로
처리해 약 1.58배 처리량을 확인했다. 짧은 400-transition 시험은 process/model 시작
비용 때문에 4 workers가 오히려 느렸으므로 긴 rollout batch에만 병렬화를 사용한다.

    ~/DAPIER/so101_imitation_learning/.venv/bin/python \
      parallel_shoe_rollout.py --workers 4 --episodes 8 --steps 500

이 결과가 보장하는 것은 상태 기반 simulation rollout 병렬화다. image ACT 학습까지
닫으려면 RGB/depth capture, episode writer, 12차원 action normalization, train/validation
split과 checkpoint adapter가 추가로 필요하다. primitive 신발, stationary base,
domain randomization과 실물 실행도 아직 검증하지 않았다.
