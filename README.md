# DAPIER

DAPIER는 Physical AI와 로봇 제어를 **시뮬레이션 → 계약 검증 → 데이터 수집 →
정책 평가 → 실물 검증** 순서로 연결하는 개인 학습·실험 저장소입니다.

현재 주력 목표는 SO-101 한 팔로 카드 한 장을 안정적으로 다루는 기준선을 만든
뒤, 두 번째 팔을 추가해 양팔 카지노 딜러로 확장하는 것입니다. 이 과정에서
ROS 2 Jazzy, MuJoCo, LeRobot, imitation learning/VLA, TurtleBot3 SLAM·Nav2,
Arduino 서보 제어를 각각 독립된 실험 단위로 다룹니다.

> **2026-08-11 기준:** 소프트웨어·모의 ROS·MuJoCo 증거는 축적되어 있지만,
> SO-101 실제 모터 제어, 실물 wrist camera 정렬, 실제 카드 pick/place와 양팔
> 동작은 아직 통과한 결과가 아닙니다.

## 전체 작업 흐름

```text
CardBench / task contract
          │
          ├── MuJoCo G0: 환경·관절·단위·frame 계약
          │        │
          │        └── G1: scripted pick-and-lift
          │                 │
          │                 ├── IK expert episode
          │                 └── wrist-only VLA
          │                          └── action smoothing + trace 분석
          │
          ├── ROS 2 core / safe teleop contract
          │        └── read-only hardware gate → 제한된 실물 검증
          │
          └── casino planner / one-card baseline
                   └── 한 팔 실물 skill → 두 팔 역할 분할
```

핵심 원칙은 모델 성능보다 먼저 관절 순서, 단위, calibration, stale frame 거부,
명령과 측정값의 구분, 실험 seed와 revision을 고정하는 것입니다.

## 저장소 지도

| 경로 | 역할 | 현재 확인·구현 범위 |
|---|---|---|
| [`so101/`](so101/README.md) | SO-101 정본 인덱스, integration, hardware tool, 실험 기록 | 흩어진 SO-101 작업의 소유 경계와 기록 위치 정리 |
| [`dapier_sim_first/`](dapier_sim_first/README.md) | ROS 2·실기체 없이 실행하는 sim-first Gate | G0 계약 PASS, G1 scripted pick-and-lift 1 episode, offline digital-twin evaluator |
| [`so101/integrations/lerobot_v0_6_so101_mujoco/`](so101/integrations/lerobot_v0_6_so101_mujoco/README.md) | LeRobot v0.6.0용 SO-101 MuJoCo overlay 보존본 | IK episode, wrist-only SmolVLA 학습·평가, camera/control routing, action smoothing과 trace 분석 |
| [`so101_ros2/`](so101_ros2/README.md) | 직접 구성한 SO-101 ROS 2 Jazzy stack | 관절 계약·calibration·제한 코어와 합성 `JointState` 기반 안전 teleop 검증 |
| [`casino_dealer/`](casino_dealer/README.md) | CardBench 계약, 블랙잭 planner, episode manifest | 1~7명 딜 순서, one-card 기구학 baseline, 장비 없는 단위 테스트 |
| [`turtlebot3_ws/`](turtlebot3_ws/README.md) | TurtleBot3 SLAM·Nav2 학습 workspace | Humble/Gazebo Classic 교재를 Jazzy/gz-sim Harmonic으로 이식해 지도 저장과 Nav2 목표 주행 검증 |
| [`jdcobot100_sim/`](jdcobot100_sim/README.md) | 4축 로봇암 시뮬레이션 전용 ROS 2 패키지 | RViz, Gazebo, `ros2_control` 연결 학습 |
| [`ros_arm/`](ros_arm/README.md) | Arduino Uno + SG90/MG90 4축 실물 제어 패키지 | `JointState`→USB serial 변환, sequence GUI, Arduino firmware와 안전 범위 |
| [`onshape/jdcobot100/`](onshape/jdcobot100/) | jdcobot100 CAD·mesh·MJCF 기준 자산 | ROS 2 visual과 MuJoCo reference model |
| [`project-planning/`](project-planning/) | 설계 판단과 Gate 계획 | LeRobot/ROS 2 분해, sim-to-real foundation, RCS 개념 채택 기록 |
| [`docs/`](docs/) | 사람이 따라 하는 실행 문서 | SO-101 카지노 딜러 calibration·수집·검수·평가 runbook |

## 검증 스냅샷

아래 숫자는 각 문서에 적힌 환경과 seed에서 얻은 **범위가 제한된 결과**입니다.
시뮬레이션 성공을 실물 성공률로 해석하지 않습니다.

| 작업 | 기록된 결과 | 해석 범위 |
|---|---:|---|
| SO-101 G0 contract Gate | PASS | 6축 순서, degree/radian·gripper 변환, calibration identity, stale/invalid frame 거부 |
| SO-101 G1 scripted pick | 1/1 episode, 300/300 frame | 조정된 MuJoCo task에서 최대 lift 47.15 mm; human demo나 learned policy가 아님 |
| Casino one-card baseline | 100/100 episode | Cartesian tool point와 vacuum attachment를 계산한 결정론적 기구학 state machine; physics·실물 아님 |
| `casino_dealer` test | 20/20 PASS | contract, planner, manifest와 baseline 코드 |
| SO-101 ROS 2 core | GTest 7개 PASS | serial·motor 없이 core 계산과 합성 message 기반 teleop 경계 |
| TurtleBot3 SLAM/Nav2 | 지도 저장 및 목표 전송 3방식 성공 | Gazebo Harmonic simulation에서 topic, action, SimpleCommander 경로 확인 |
| wrist-only VLA v2 | 서로 다른 두 held-out 세트 각각 14/20 | 70% 수준의 sim 결과이며 안정적 80% 일반화 기준은 미충족 |

## 2026-08-11 최근 변경

### VLA action smoothing

정책 성공 여부와 action chunk 경계의 제어 떨림을 분리해 분석했습니다.
VLA route에 다음 항목을 적용하고 generic environment의 기본값은 꺼 두었습니다.

- 25-action chunk의 첫 3 frame blend
- IK teacher action delta에서 정한 관절별 slew limit
- 1 percentage point gripper deadband
- raw/applied action, filter 결정, radian command, 동기식 MuJoCo readback JSONL 기록

같은 seed 5개의 A/B에서 baseline과 smoothing은 모두 `4/5`로 성공 수가 같았습니다.
대신 shoulder-pan chunk-boundary 최대 target jump는 `9.405° → 1.750°`, gripper는
`12.887 → 4.282` percentage point로 줄었습니다. 새 seed 20개는 `16/20`이었지만,
이전의 서로 다른 held-out 20-episode 결과가 각각 `14/20`이므로 안정적인 80%
일반화 성공으로 올리지 않습니다.

재현 조건과 물리 장비 Gate는
[`SO-101 wrist-only VLA action smoothing and RCS trace`](so101/records/2026-08-11-so101-vla-action-smoothing.md)에
기록했습니다.

### Robot Control Stack 개념 적용

외부 Robot Control Stack의 source·asset이나 runtime을 복사하지 않고 다음 개념만
DAPIER 계약에 독립 구현했습니다.

- synchronous step과 post-action readback
- absolute target과 `async_control=false` manifest
- command/simulation/physical trace의 timestamp·joint-order 검증
- joint별 MAE, RMSE, p95, endpoint error, delay gap과 timestamp skew
- 승인된 실측 threshold가 없을 때 PASS를 만들지 않는 `MEASURED` 결과

자세한 판단과 라이선스 경계는
[`Robot Control Stack 개념 채택 기록`](project-planning/2026-08-11-robot-control-stack-concept-adoption.md)에
남겼습니다.

## 빠른 시작

이 저장소는 하나의 단일 실행 프로그램이 아니라 여러 독립 실험을 모은
저장소입니다. 전체를 한 번에 `colcon build`하거나 하나의 Python 환경에 모두
설치하지 않습니다.

```bash
git clone https://github.com/Alpenj/DAPIER.git
cd DAPIER
```

### 장비 없이 바로 확인할 Python 경로

```bash
cd ~/DAPIER/casino_dealer
python3 -m unittest discover -s test -v
python3 -m casino_dealer.cli --players 3

cd ~/DAPIER
python3 -m unittest discover -s dapier_sim_first/test -v
```

`dapier_sim_first`의 단위 테스트는 외부 장비 없이 실행할 수 있지만, 실제 G1
재현에는 문서에 고정된 MuJoCo model, calibration 파일과 별도 환경이 필요합니다.

### ROS 2 경로

Ubuntu 24.04와 ROS 2 Jazzy를 기준으로 하며 각 패키지를 별도 workspace에서
빌드합니다.

- 이동로봇 SLAM·Nav2: [`turtlebot3_ws/README.md`](turtlebot3_ws/README.md)
- jdcobot100 시뮬레이션: [`jdcobot100_sim/README.md`](jdcobot100_sim/README.md)
- Arduino 4축 로봇암: [`ros_arm/README.md`](ros_arm/README.md)
- SO-101 mock 안전 teleop: [`so101_ros2/README.md`](so101_ros2/README.md)

## SO-101을 처음 읽는 순서

1. [`so101/README.md`](so101/README.md)에서 정본과 외부 checkout의 경계를 확인합니다.
2. [`SO-101 sim-to-real foundation`](project-planning/2026-08-07-so101-sim-to-real-foundation.md)에서 Gate와 중단 조건을 읽습니다.
3. [`dapier_sim_first/README.md`](dapier_sim_first/README.md)에서 G0·G1 증거를 확인합니다.
4. [`LeRobot v0.6.0 integration README`](so101/integrations/lerobot_v0_6_so101_mujoco/README.md)에서 IK/VLA 실험을 확인합니다.
5. 실물 준비 단계에서는 [`SO-101 카지노 딜러 runbook`](docs/SO101_CASINO_DEALER_RUNBOOK_KO.md)을 순서대로 실행합니다.

## 기록과 판정 원칙

- **명령값과 측정값을 구분합니다.** 마지막 command를 measured joint state로 기록하지 않습니다.
- **SIM, MOCK, HW를 섞지 않습니다.** MuJoCo PASS나 합성 ROS message 통과는 실물 성공이 아닙니다.
- **실험 조건을 남깁니다.** seed, revision, contract hash, camera profile, action horizon과 평가 기준을 결과와 함께 기록합니다.
- **생성물을 소스와 분리합니다.** `.venv`, `build/`, `install/`, `log/`, 원시 dataset, 영상과 calibration 개인 파일은 기본적으로 Git에 넣지 않습니다.
- **외부 코드와 자산의 경계를 유지합니다.** overlay, patch, provenance와 upstream license를 integration별로 분리합니다.
- AI는 반복 코드·문서 초안과 로그 해석을 보조하지만, 실험 방향·하드웨어 안전 판정·결과 승인 여부는 실제 실행 기록을 기준으로 결정합니다.

## 현재 물리 장비 Gate

기록된 SO-101 hardware gate는 아직 `blocked`입니다. 선택한 wrist camera profile이
`physical_alignment=false`이고, 당시 inventory에서 SO-101 serial 장치를 확인하지
못했기 때문입니다. 이 상태에서 motor command나 physical rollout을 성공으로
표시하지 않습니다.

실물 단계는 다음 순서를 통과한 뒤 진행합니다.

1. USB/serial과 motor ID 1~6의 읽기 전용 inventory
2. 실제 wrist camera extrinsic·회전·FOV 확인
3. 현재 위치로 command seed 후 한 관절 저속 이동
4. torque OFF와 비상 정지 경로 확인
5. command·simulation·physical readback 동기 trace 수집
6. 반복 측정으로 physical threshold 승인
7. 한 장 pick/place episode 수집과 사람이 직접 성공·실패 검수
8. 한 팔 기준선이 안정된 뒤 두 번째 팔의 deck 고정 역할 추가

## 프로젝트 범위

이 저장소는 완성된 상용 로봇 제품이나 실물 성능을 보증하는 배포판이 아닙니다.
학습 과정에서 실패 원인, 환경 차이, 검증 범위와 다음 Gate를 재현 가능한 형태로
남기는 것이 목적입니다. 상세 실행법과 주의사항은 각 하위 디렉터리 README를
기준으로 합니다.
