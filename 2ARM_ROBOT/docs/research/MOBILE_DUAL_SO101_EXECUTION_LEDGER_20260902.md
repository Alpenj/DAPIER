# 이동형 양팔 SO-101 박스·신발 미션 실행 원장

- `record_id`: `DAPIER-2026-09-02-mobile-dual-shoe-execution`
- 시작일: 2026-09-02 (Asia/Seoul)
- 후속 브랜치: `pro/mujoco-shoe-followup-01`
- 고정 기준: PR [#40](https://github.com/Alpenj/DAPIER/pull/40) head `fecc0c7`
- 실행 범위: MuJoCo-only, offline data, hardware-free test

## 내가 끝내려는 결과

나는 visual SLAM 도착 신호부터 오른팔 박스 열기·유지, 왼팔 신발 proxy 파지·꺼내기,
운반·복귀·내려놓기까지의 양팔 미션을 재현 가능한 simulation과 데이터 흐름으로 완성한다.
일정의 날짜를 기다리지 않고 각 단계의 완료 조건을 통과하면 바로 다음 단계로 넘어간다.

## 완료 기준

1. 전체 hardware-free/MuJoCo test가 통과한다.
2. 오른팔은 뚜껑을 열어 유지하고, 왼팔은 동적 contact를 만든 뒤 신발 proxy를 박스 밖으로 꺼낸다.
3. 허용 contact와 금지 collision을 geom pair로 구분하고 금지 collision을 0으로 유지한다.
4. 같은 seed에서 state transition과 failure code를 재현한다.
5. 정상·no-contact·collision·stale-camera run을 export하고 같은 명령으로 replay·plot을 만든다.
6. mass, friction, pose, camera, network, motor, FSR 변화에서 성공률과 recovery를 비교한다.
7. SLAM 도착·정지 계약부터 복귀·내려놓기까지 headless E2E를 검증한다.
8. 실기 측정값을 받으면 simulation parameter와 gap report를 갱신한다.
9. 최종 수치와 그림을 원본 run ID까지 역추적할 수 있게 보관한다.

## 작업 권한과 경계

- PR #40의 브랜치와 head는 Hermes 검토가 끝날 때까지 변경하지 않는다.
- 후속 코드는 이 worktree와 브랜치에서만 수정한다.
- simulation entrypoint는 ROS, serial, motor SDK를 import하거나 장치를 열지 않는다.
- 실제 motor, torque, EEPROM, publisher, read-only hardware snapshot은 별도 승인 없이 실행하지 않는다.
- 테스트 성공을 실기 성공으로 표현하지 않는다.
- GitHub에는 직접 실행한 명령과 결과만 남긴다.
- 눈으로 확인해야 하는 MuJoCo 단계는 headless 선행 검증 후 사용자에게 관찰 항목을 먼저 알리고 viewer를 연다.
- viewer에서 양팔 자세·접촉·충돌·trajectory를 함께 확인하기 전에는 시각 검증 완료로 기록하지 않는다.

## 실행 그래프

Topology: pipeline

Integration owner와 final verifier는 이 기록을 작성하는 내가 맡는다. 앞 단계의 계약이 바뀌면 영향받는
뒤 단계만 다시 검증한다.

| ID | 목표 | 선행 조건 | 출력 | 완료 검사 | 상태 |
|---|---|---|---|---|---|
| G0 | PR #40 고정과 후속 worktree 격리 | 없음 | 별도 branch/worktree, 이 원장 | PR head 불변, clean baseline | 완료 |
| P1 | camera contract와 home clearance를 정렬 | G0 | 원인·수정·회귀 test | 전체 suite 통과 | 완료 |
| P2 | 왼팔 동적 contact·파지·꺼내기와 금지 충돌 제거 | P1 | trajectory, contact/collision assertion | 양지 contact·weld 없는 lift·collision 0 | 진행 중 |
| P3 | 양팔 전체 sequence, export, replay, 시각화 | P2 | 정상·실패 run artifact | 동일 seed 재현 | 예정 |
| T1 | 물리·센서·지연·모터·FSR tuning | P3 | sweep matrix와 parameter revision | baseline 대비 강건성 표 | 예정 |
| I1 | SLAM 도착·복귀를 포함한 E2E integration | T1 | 통합 state trace와 recovery | 반복 E2E acceptance | 예정 |
| F1 | 실측 기반 sim-real feedback와 RC 회귀 | I1 | gap report와 RC evidence | 알려진 blocker 명시 | 예정 |
| R1 | 재현 문서·영상·발표 artifact 동결 | F1 | 최종 evidence bundle | 새 환경 재현·리허설 | 예정 |

## 기준점에서 직접 확인된 상태

- PR #40의 원격 head는 `fecc0c7`이고 두 GitHub check가 통과했다.
- 기존 로컬 기록은 box mission 5/5, grasp planner 4/4, 전체 149/151이다.
- 남은 두 항목은 camera 수 기대값 3과 모델 5의 불일치, home-pose camera clearance
  0.1075 m와 gate 0.13 m의 불일치다.
- 물리 데모에서는 오른팔 뚜껑 열림을 확인했지만 왼 gripper–shoe 동적 contact는 0이었고,
  lid–left wrist 비의도 충돌이 남아 있다.
- 이 값들은 후속 브랜치에서 다시 실행하기 전까지 기준점 기록이며 새 검증 결과가 아니다.

## 단계별 학습 기록 형식

각 단계를 마칠 때 아래 항목을 이 파일에 이어 쓴다.

1. 오늘 확인하려던 가설
2. 변경한 코드와 이유
3. 실행 환경과 명령
4. 통과·실패 수치와 artifact 경로
5. 실패 원인과 수정 과정
6. simulation에서 확인한 범위
7. 아직 실기에서 확인하지 못한 범위
8. 다음 단계의 첫 검증

## G0 · 검토 브랜치 격리

### 직접 한 일

PR #40의 head `fecc0c7`에서 `pro/mujoco-shoe-followup-01` 브랜치와
`DAPIER-mujoco-shoe-followup-01` worktree를 만들었다. PR #40 worktree의 미커밋 문서와
스크린샷은 옮기거나 수정하지 않았다.

### 확인한 결과

- 후속 worktree HEAD: `fecc0c7`
- PR #40 원격 head: `fecc0c7`
- PR #40 상태: Draft/Open, merge state clean
- GitHub checks: `hardware-free-tests`, `mujoco-headless-tests` 성공

### 다음에 확인할 것

P1에서 전체 suite를 같은 SO-101 MJCF 입력으로 다시 실행하고, 두 실패가 실제 요구 변경인지
잘못된 고정값인지 코드·모델 근거로 분리한다.

## P1 · camera contract와 home clearance 정렬

### 확인하려던 가설

MuJoCo 모델의 카메라 5개는 실제 센서 요구가 늘어난 결과가 아니라 원본 wrist camera와 후속
provisional gripper camera가 중복된 결과라고 예상했다. camera clearance 0.13 m 실패는
실제 SO-101 base assembly를 복원한 뒤 남은 이전 geometry 회귀값인지 확인했다.

### 직접 확인한 원인

모델 camera 이름을 출력해 보니 다음 5개가 있었다.

- `front_depth_camera`
- `left_wrist_cam`, `right_wrist_cam`
- `left_gripper_camera`, `right_gripper_camera`

고정 SO-101 MJCF의 `wrist_cam`은 실제 InnoMaker 130도 카메라 기준으로 보정돼 있었다.
mission adapter는 이 카메라를 두고 65도 provisional camera를 양쪽에 하나씩 더 만들었다.
따라서 실물 입력 계약인 전방 RGB-D 1개와 양쪽 gripper RGB 2개보다 MuJoCo object가 두 개 많았다.

camera clearance의 최단 pair도 출력했다. `depth_camera_collision`과 복원된 왼쪽
`waveshare_mounting_plate_so101_v2` 사이가 0.1075428816 m였다. 0.13 m assertion은
Waveshare plate를 제거했던 이전 모델에서 추가됐고, 이후 `1a32622`에서 실제 base assembly를
복원할 때 갱신되지 않았다.

### 변경한 코드와 이유

- 원본 `left/right_wrist_cam`을 `left/right_gripper_camera` 역할로 이름을 바꿔 재사용했다.
- 65도 provisional camera와 실제 optical frame처럼 보이던 임시 site를 제거했다.
- camera payload의 `optical_frame`은 실제 MuJoCo camera object 이름을 사용한다.
- 테스트는 camera object 3개, 보정 FOV 70.5도, 이전 wrist 이름 부재를 확인한다.
- 복원된 mounting plate를 포함한 camera 회귀 여유를 0.10 m로 명시했다.
  전체 protected path의 operational gate 0.03 m는 변경하지 않았다.

### 실행 검증

집중 검증:

```bash
python -m unittest -v \
  test_mujoco_mission_adapters.MuJoCoMissionAdapterTest.test_mobile_model_is_separate_and_has_three_cameras \
  test_mujoco_mission_adapters.MuJoCoMissionAdapterTest.test_three_camera_payloads_are_rendered_and_synchronized \
  test_collision_guard.CollisionGuardTest.test_step_support_home_protects_base_column_and_bare_camera
```

- 결과: 3/3 통과
- RGB-D와 양쪽 RGB frame payload 렌더·동기화 포함

전체 검증:

```bash
python -m unittest discover -s 2ARM_ROBOT/sim/mobile_dual_so101/test -v
```

- 결과: 151/151 통과, 17.878 s
- `hardware_execution=false`

### 배운 점

센서 role 수와 simulator camera object 수를 같다고 가정하면 중복 장착을 놓칠 수 있다. 외부 MJCF를
합성할 때는 source camera의 보정 provenance를 먼저 확인하고 새 camera를 추가해야 한다. geometry
회귀 기준도 mesh를 제거하거나 복원한 commit과 함께 갱신해야 하며, collision gate를 단순히 낮추는
방식과 의도된 설계 envelope를 갱신하는 방식을 구분해야 한다.

### 아직 확인하지 못한 것

- 실제 양쪽 USB RGB camera의 serial, resolution, FOV, intrinsics/extrinsics
- 실제 Waveshare plate와 depth camera 사이 거리
- MuJoCo camera 영상과 실기 영상의 reprojection 차이

### 다음에 확인할 것

P2에서 오른팔이 뚜껑을 유지하는 동안 왼팔의 approach·contact·grasp·extract를 동적 물리로
검증한다. headless 사전 검증 뒤 사용자에게 관찰 항목을 알리고 MuJoCo viewer를 연다.

## P2 · 오른팔 뚜껑 개방과 왼팔 파지 판정 수정 · 진행 중

### 확인하려던 가설

기존 실패는 IK 전체가 틀린 것이 아니라 왼 gripper frame과 실제 finger collision pad의 오프셋,
박스 안에서 한 번에 인출하려던 경로, 뚜껑 파지 후 중복된 접촉 제약 때문이라고 예상했다. 정적 IK
수렴이 아니라 실제 MuJoCo contact, 회전된 물체의 최저점, 최종 접촉, actuator 추종을 함께 판정했다.

### 실패 재현과 원인

수정 전 headless run은 오른팔 날개 접촉 2건과 뚜껑 101.01도 유지까지 진행했지만 왼팔–신발
접촉이 0건이었다. 왼 gripperframe은 목표 근처에 있었지만 static finger pad가 신발 상면보다 약
7 mm 높았다. contact 목표 z를 0.090 m에서 0.082 m로 내리자 접촉점 2개가 생겼다.
하지만 두 접촉점은 서로 다른 손가락이 아니라 모두 고정측 left_gripper mesh에 있었다.

처음에는 중심 z에서 고정된 신발 반높이만 빼서 박스 밖으로 나왔다고 판정했다. 직육면체가 기울면
이 계산은 틀린다. geom 회전행렬의 절댓값과 half-size로 world AABB를 계산해 가장 낮은 모서리와
박스 상단의 실제 간격을 사용하도록 바꿨다.

오른팔은 임의 gripperframe 목표 하나로 뚜껑을 당기면서 약 0.97 rad 추종 오차와 actuator
포화를 만들었다. 접촉 검출용 날개–그리퍼 collision과 접촉 뒤 활성화한 equality가 같은 면을
동시에 구속한 것도 원인이었다. 접촉을 확인한 뒤 해당 날개의 collision을 equality로 넘기고,
힌지 30도·60도·95도의 실제 contact-site 원호를 순서대로 따라가도록 변경했다.

왼팔은 낮은 위치에서 곧바로 높은 목표로 이동하면 wrist–lid 충돌이 났다. 다음 waypoint로
수직 여유를 먼저 만든 뒤 박스 밖으로 이동했다.

1. (0.20, 0.10, 0.14) m
2. (0.20, 0.12, 0.20) m
3. (0.20, 0.12, 0.26) m
4. (0.18, 0.16, 0.32) m
5. extract (0.16, 0.22, 0.32) m

### 사람이 확인한 배치 결정

초기 배치는 힌지가 로봇 가까운 쪽이라 열린 뚜껑이 양팔 작업공간을 가렸다. 사용자와 viewer에서
확인한 뒤 박스를 -90도로 돌려 힌지를 몸에서 먼 쪽에 놓고, 오른팔 바로 앞에 오는
box_lid_left_dust_flap을 들어 올리도록 단순화했다. 이 방향의 새 viewer에서 사용자가 의도한
개방 방향임을 직접 확인했다. 시각 확인 전에는 완료로 기록하지 않았다.

### 변경한 코드와 이유

- position IK에 선택적 site 이름을 받아 gripper 중심 대신 실제 lid contact site를 풀 수 있게 했다.
- 오른팔 개방을 contact-site 기준 30도·60도·95도 waypoint로 나눴다.
- contact가 확인된 날개 collision은 equality handoff 뒤 비활성화해 중복 구속을 제거했다.
- 왼 contact 높이를 조정했지만 현재는 고정측만 닿으므로 lift·extract를 실행하지 않는다.
- contact 직후 shoe를 붙이던 weld equality를 scene과 실행 코드에서 제거했다.
- shoe contact를 고정측과 이동측으로 나눠 둘 다 닿지 않으면 실패하도록 했다.
- 회전된 shoe geom의 world AABB 최저점을 clearance에 사용했다.
- 최종 shoe–box/lid contact 수와 final joint tracking error를 acceptance에 포함했다.
- actuator peak force ratio와 관절별 saturation fraction을 report에 추가했다.
- viewer 모드는 10 ms timestep에 맞춰 재생해 동작을 눈으로 따라갈 수 있게 했다.

### 실행 검증

Headless 명령: python 2ARM_ROBOT/sim/mobile_dual_so101/box_shoe_physics_demo.py

- 오른팔 날개 contact: 2
- 왼팔 고정측 shoe contact: 2
- 왼팔 이동측 shoe contact: 0
- 최종 뚜껑 각도: 94.9135도
- shoe grasp weld: 없음
- 회전 반영 shoe bottom clearance: -0.10434 m
- 최종 shoe–box/lid contact: 4
- 금지 arm–box/lid collision: 0
- final tracking error: 0.01838 rad
- runtime arm qpos write: 0
- 결과: left_bilateral_contact_failed, success=false, hardware_execution=false

전체 명령: python -m unittest discover -s 2ARM_ROBOT/sim/mobile_dual_so101/test -v

- 이전 전체 결과는 152/152 통과였지만 잘못된 grasp 성공 조건을 포함했으므로 현재 기준으로
  재검증하기 전 완료 근거로 사용하지 않는다.
- 새 회귀 테스트는 같은 고정측 접촉점 두 개가 생겨도 양지 파지나 성공으로 판정하지 않는지 확인한다.

MuJoCo viewer는 headless 성공 뒤 관찰 항목을 먼저 공유하고 열었다. 사용자가 뚜껑이 몸에서
멀어지는 방향으로 열리고 오른팔 앞 날개를 드는 배치가 맞다고 확인했다. 이 확인은 박스 방향과
뚜껑 개방에만 해당하며 신발 파지 성공 확인은 아니다.

### 배운 점

position IK residual이나 contact point 개수만으로 파지 성공을 판단할 수 없다. 고정측과 이동측의
접촉을 분리하고, attachment constraint 없이 그리퍼를 닫아 마찰로 들어 올린 뒤 상대 slip까지
확인해야 한다. 또한 한 개의 큰 Cartesian 목표보다 충돌 의미가 분명한 짧은 waypoint가
디버깅과 시각 검증에 유리했다.

전체 pose 보존 IK 초안도 시험했지만 5-DoF SO-101에 6-DoF 자세를 과구속하고 현재 파지에서는
도달성이 나빠 채택하지 않고 제거했다. 이번 단계에는 contact-site 위치 IK가 더 작은 해법이었다.

### 아직 확인하지 못한 것

- 양지 contact와 friction-only lift는 아직 성공하지 않았다.
- actuator는 peak force ratio 1.0에 도달했다.
- 전방 RGB-D와 gripper RGB 관측을 grasp target에 연결하지 않았다.
- 실제 판지 날개 변형, servo current/temperature, backlash, FSR 접촉값은 미검증이다.

### 다음에 확인할 것

P2에서 기존 ShoePoseEstimate와 camera contract를 재사용해 RGB-D 관측 pose를 grasp target으로
연결한다. 양지 contact와 friction-only lift가 headless에서 통과한 뒤 사용자에게 관찰 항목을
먼저 알리고 viewer 검증을 진행한다. 그 전에는 P3로 넘어가지 않는다.

## HW 준비 · 장치 역할 고정과 Astra S Color 진단 · Color 복구 완료 · 동시 부하 검증 대기

### 직접 확인한 연결

- 외장 허브 물리 1번: 왼쪽 wrist RGB
- 외장 허브 물리 2번: 오른쪽 wrist RGB
- 외장 허브 물리 3번: 왼팔 SO-101 controller
- 외장 허브 물리 4번: 오른팔 SO-101 controller
- 작업공간 RGB-D: Orbbec Astra S (`2bc5:0402`)

현재 노트북에서는 `/dev/dapier/left_arm`, `right_arm`, `left_wrist_rgb`,
`right_wrist_rgb`, `workspace_rgbd` 별칭으로 접근한다. 팔 controller는 장치 serial로,
serial을 제공하지 않는 동일 모델 wrist camera 두 대는 외장 허브 downstream path로 역할을
고정했다. 따라서 `/dev/ttyACM*`, `/dev/video*` 번호가 바뀌거나 허브 전체를 다른 host USB
포트에 연결해도 역할을 유지한다. 단, 동일 wrist camera 두 개의 개별 허브 플러그를 서로
바꾸면 passive udev만으로 좌우를 식별할 수 없으므로 시작 점검에서 경고해야 한다.

실제 controller serial은 공개 저장소에 기록하지 않고 개인 규칙 파일에만 보관한다.
`scripts/install_hardware_aliases.sh`는 그 파일을 대상 PC 또는 Raspberry Pi 4의 udev에
설치하고, 연결되지 않은 장치는 실패 대신 경고로 표시한다.

### 권장 USB 배선

- 전원형 USB 3.x 허브의 고정 1~4번 포트에 좌/우 wrist RGB와 좌/우 SO-101 controller를 함께 연결한다.
- Astra S는 가능하면 호스트의 다른 포트 또는 별도 전원형 허브로 분리한다.
- 허브 수가 아니라 `lsusb -t`의 root hub가 실제 대역폭 경계다. 같은 `480M` root hub 아래면
  물리 허브를 나눠도 카메라 대역폭은 계속 공유하므로 전체 연결 뒤 동시 FPS/drop/reset을 측정한다.

### Astra S에서 직접 재현한 결과

- Orbbec Viewer/SDK v1.10.37: Depth 1프레임 수신 성공
- 같은 SDK의 Color: `OB_SENSOR_COLOR Match openni video mode failed`
- 공식 OpenNI2 2.3.0.86 beta6: `device.open()` 단계에서 `USB transfer timeout`
- beta6의 `UsbInterface`를 BULK(2)와 ISO(1)로 각각 시험했지만 같은 timeout
- USB 장치 권한은 `0666`이고 `usbcore.usbfs_memory_mb=128`에서도 결과가 같아 권한과
  usbfs buffer를 원인에서 제외
- 공식 `OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux.tar.gz`의 x64 redist:
  배포물 SHA-256: `05dda4507620e91408249b1c139b0ba25d4ed3cb8a402905a10fda719dbaaf44`
  같은 Astra와 같은 Bus 001에서 Depth 및 Color stream open 성공
- `ldd`로 해당 redist의 `libOpenNI2.so`가 실제 로드됨을 확인
- 공식 `ColorReaderPoll`을 이 redist에 링크해 약 33.8 ms 간격, 약 29.6 FPS의 연속 RGB888
  프레임과 변화하는 RGB 값을 확인
- 같은 redist의 `NiViewer` GUI 실행 성공
- 양쪽 wrist RGB: 320x240, YUYV, 15 FPS 화면 확인

원인은 Astra 하드웨어나 현재 USB 포트 자체가 아니라 OpenNI 런타임/driver 조합이었다.
현재 Ubuntu에서는 공식 ROS2 OpenNI v1.0.2 redist를 사용하고, 최신 Orbbec SDK와 beta6
OpenNI 조합을 이 장치의 실행 경로에서 제외한다. firmware는 변경하지 않았다.
같은 공식 tar에는 `arm`과 `arm64` redist도 포함되지만, 이번 실기 증거는 x64 노트북에만 해당한다.
Raspberry Pi 4에서는 OS 아키텍처에 맞는 redist로 별도 FPS·온도·USB 동시 부하 검증을 수행한다.

### USB root hub와 동시 스트림 판단

현재 외장 허브와 Astra는 모두 Bus 001의 같은 480M root hub를 공유한다. 두 wrist RGB의
320x240 YUYV 15 FPS payload는 합계 약 36.9 Mbit/s다. Astra를 640x480 30 FPS,
16-bit depth와 16-bit on-wire color로 가정하면 약 294.9 Mbit/s가 추가된다. 합계 약
331.8 Mbit/s는 프로토콜 overhead와 예약 대역폭을 제외한 USB 2.0 안정 실효 범위에 가까우므로
Color+Depth+wrist 두 대를 동시에 켤 때 frame drop, stream start 실패, USB reset 가능성이 있다.

실제로 Astra 연결 직후 같은 root 아래 wrist camera 한 대가 reset되고 다른 한 대가 재열거된
kernel 기록도 있었다. 이는 streaming 대역폭 초과의 직접 증거는 아니지만 전원 또는 bus transient
위험의 증거다. 반면 다른 영상 스트림을 끈 상태에서 Astra Color가 같은 Bus 001에서 29.6 FPS로
열렸으므로 기존 Color open 실패를 대역폭 문제로 판정하지 않는다.

최종 실기 배선은 Astra를 `lsusb -t`에서 Bus 003, 005 또는 007처럼 별도의 480M root 아래로
분리한다. 단순히 다른 허브를 쓰는 것으로 충분하지 않고 root hub가 실제로 달라야 한다. 분리 뒤
Color+Depth+wrist 두 대를 동시에 실행해 FPS, frame drop, kernel reset을 측정한 결과를 최종
commissioning 근거로 남긴다. 팔 controller 두 개의 serial 통신량은 영상에 비해 작아서 대역폭
병목의 주원인이 아니다.

### 고정한 실행 경로

- 로컬 런타임: `~/.local/opt/orbbec-openni2-ros2-v1.0.2`
- 장치 별칭: `/dev/dapier/workspace_rgbd`
- 준비 확인: `2ARM_ROBOT/scripts/run_astra_openni2_color check`
- Color 원시 프레임: `2ARM_ROBOT/scripts/run_astra_openni2_color poll`
- Color/Depth GUI: `2ARM_ROBOT/scripts/run_astra_openni2_color viewer`

실행 스크립트는 `OPENNI2_REDIST`와 `LD_LIBRARY_PATH`를 검증된 redist로 고정한다.
Ubuntu 24.04의 FreeGLUT SONAME 차이는 설치된 `libglut.so.3.12`를 로컬 `compat`
경로에서만 연결해 해결했으며 시스템 라이브러리나 firmware는 수정하지 않았다.
