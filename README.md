# DAPIER — Physical AI 학습·실험 기록

이 저장소는 수업과 개인 실습에서 직접 실행한 로봇 코드, 실패 원인과 검증
기록을 한곳에 모은 작업 공간이다.

최종 목표는 SO-101 두 대를 이용한 카드 딜러지만, 현재는 **한 팔
pick/place → 데이터와 정책 검증 → 실물 calibration** 순서로 진행하고 있다.
시뮬레이션에서 성공한 것, 합성 ROS message로 확인한 것, 실제 장비에서 확인한
것을 같은 결과로 취급하지 않는다.

## 지금 상태

저장소에 남아 있는 최신 실물 기록은 2026-08-20이다.

| 영역 | 현재 판단 |
|---|---|
| SO-101 sim-first | 6축 순서·단위·frame 계약을 검사하는 G0와 조정된 MuJoCo task의 G1 scripted pick-and-lift를 통과했다. |
| SO-101 policy | 기존 wrist-only v2 checkpoint를 유지한다. corrected contact physics에서 unseen seed `2100..2119` 결과는 `11/20`이며 80% release gate는 닫혀 있다. |
| SO-101 실물 | leader/follower serial board와 각 motor ID 1~6 응답은 확인했다. 첫 follower calibration은 실패했고 teleoperation은 아직 금지 상태다. |
| SO-101 ROS 2 | 관절 계약, calibration 변환, 제한 계산과 합성 `JointState` 기반 safe teleop까지만 검증했다. 실제 motor driver는 미구현이다. |
| 카지노 딜러 | blackjack planner, episode manifest와 one-card 기구학 baseline이 있다. 실제 카드·흡착·양팔 동작은 아직 없다. |
| 이동로봇 | TurtleBot3 SLAM·Nav2는 Jazzy/gz-sim에서 end-to-end로 확인했다. 커스텀 `ros_dd_ws`는 현재 world와 저장 map이 어긋나 Nav2 재검증이 필요하다. |
| 이동형 양팔 신발 정리 | JDcobot200 두 팔의 STS3215 12개를 읽기 전용으로 확인했고 TurtleBot3 stationary baseline과 들린 바퀴 속도 응답을 기록했다. 팔 동작·RGB-D stream·실제 신발 집기는 아직 미검증이다. |
| 4축 로봇암 | RViz/Gazebo 시뮬레이션과 Arduino Uno 서보 제어를 별도 패키지로 보관한다. SO-101 결과와 섞지 않는다. |

## 저장소 구성

### SO-101 주 작업

| 경로 | 내용 |
|---|---|
| [`so101/`](so101/README.md) | SO-101 코드·integration·실험 기록·hardware tool을 찾는 최신 인덱스 |
| [`dapier_sim_first/`](dapier_sim_first/README.md) | ROS 2와 실기체 없이 실행하는 G0/G1 Gate와 offline digital-twin evaluator |
| [`so101/integrations/lerobot_v0_6_so101_mujoco/`](so101/integrations/lerobot_v0_6_so101_mujoco/README.md) | LeRobot v0.6.0용 SO-101 MuJoCo overlay, IK/VLA 실험과 action trace 보존본 |
| [`so101_ros2/`](so101_ros2/README.md) | DAPIER가 직접 구성한 ROS 2 Jazzy core와 mock safe teleop |
| [`so101/hardware_tools/`](so101/hardware_tools/README.md) | read-only inventory, 안전 calibration wrapper와 STS3215 감사·복구 도구 |
| [`casino_dealer/`](casino_dealer/README.md) | CardBench 계약, blackjack planner, episode manifest와 one-card 기구학 baseline |
| [`docs/SO101_CASINO_DEALER_RUNBOOK_KO.md`](docs/SO101_CASINO_DEALER_RUNBOOK_KO.md) | calibration부터 episode 검수와 policy 평가까지 사람이 따라 하는 실행 순서 |

### 이동로봇과 기초 제어 실습

| 경로 | 내용 | 현재 범위 |
|---|---|---|
| [`turtlebot3_ws/`](turtlebot3_ws/README.md) | TurtleBot3 SLAM·Nav2 | Humble/Gazebo Classic 교재를 Jazzy/gz-sim Harmonic으로 옮겨 지도 저장과 목표 전송을 확인 |
| [`2ARM_ROBOT/`](2ARM_ROBOT/README.md) | JDcobot200 양팔 + TurtleBot3 Waffle Pi 신발 정리 | 5축+그리퍼 episode 계약, 비식별 실측 evidence, 전력·URDF·sim-to-real 계획과 ROS 2 Jazzy 검증 |
| [`ros_dd_ws/`](ros_dd_ws/README.md) | 커스텀 differential-drive robot | 7개 package build, gz-sim, TF와 Cartographer 학습 기록. `hexa` scale 원복 뒤 기존 map/Nav2 결과는 재검증 필요 |
| [`jdcobot100_sim/`](jdcobot100_sim/README.md) | jdcobot100 시뮬레이션 | RViz, Gazebo, `ros2_control` 연결 학습 |
| [`ros_arm/`](ros_arm/README.md) | Arduino Uno + SG90/MG90 4축 제어 | `JointState`→USB serial, sequence GUI, firmware와 안전 범위 |
| [`onshape/jdcobot100/`](onshape/jdcobot100/) | CAD·mesh·MJCF 자산 | jdcobot100 visual과 MuJoCo reference model |

설계 판단과 중단 조건은 [`project-planning/`](project-planning/)에 남긴다.
원시 dataset이나 실행 영상 대신 사람이 다시 확인할 수 있는 revision, contract,
metric과 hash를 기록한다.

## SO-101 결과를 해석하는 기준

과거 README에는 기존 contact model에서 얻은 held-out `14/20` 두 세트를 대표
성능처럼 적어 두었다. 이후 성공 replay에서 finger pad가 cube 안으로 최대 약
8.3 mm 들어가는 문제가 측정됐으므로 그 숫자를 현재 release 판단에 사용하지
않는다.

visible pad/cube contact, bilateral contact 기록과 1 mm penetration gate를 추가한
뒤 기존 v2 policy를 다시 평가한 결과가 `11/20`이다. corrected IK teacher는 초기
30 episode와 후속 100 episode 수집 gate를 통과했지만, 새 student와 mixed
rehearsal checkpoint들은 기준선보다 나빠 선택하지 않았다. 따라서 현재 선택은
**기존 v2 policy + corrected contact model**이며, simulation release gate도 실물
준비 gate도 통과하지 않았다.

최근 추가한 기능도 같은 기준으로 구분한다.

- action smoothing은 chunk 경계의 target jump를 줄였지만 같은-seed 성공 수를
  늘린 결과는 아니다.
- human intervention은 policy/human 교정 evidence를 남기지만 아직 native
  LeRobot training dataset은 아니다.
- parallel rollout은 현재 GPU에서 더 빠르지 않았고, 최종 성능 평가보다 실패
  trace 수집에 사용한다.

세부 근거와 선택하지 않은 checkpoint는 [`SO-101 작업 허브`](so101/README.md)에서
실험 흐름별로 확인할 수 있다.

## 실물 준비 상태

이전에는 SO-101 serial 장치를 찾지 못해 hardware gate가 막혀 있었다. 최신 기록에서는
두 USB serial board가 안정적인 by-id 경로로 보이고, 양쪽 bus의 motor ID 1~6이
응답하는 것까지 확인했다.

다만 첫 follower calibration은 중앙 자세 뒤 range recorder가 즉시 끝나
`MIN=POS=MAX=2047`로 남았고 저장이 거부됐다. homing offset은 servo에 기록됐기
때문에 calibration을 다시 끝내기 전에는 teleoperation을 하지 않는다.

현재 다음 순서가 남아 있다.

1. follower와 leader calibration을 각각 완료하고 read-only `inspect`로 교차 확인
2. calibration JSON schema, motor ID와 normalized pose 비교
3. emergency stop과 current limit 준비 후 한 관절씩 저속 방향 확인
4. wrist camera extrinsic, image rotation과 FOV 측정
5. 실제 한 장 pick/place demonstration 수집과 사람의 성공·실패 검수
6. 짧은 replay와 policy rollout을 거친 뒤 두 번째 팔 추가

## 빠른 확인

저장소 전체를 하나의 Python 환경이나 colcon workspace에서 한 번에 빌드하지
않는다. 각 실험의 README에 적힌 환경을 사용한다.

```bash
git clone https://github.com/Alpenj/DAPIER.git
cd DAPIER
```

장비 없이 바로 실행할 수 있는 대표 경로:

```bash
cd ~/DAPIER/casino_dealer
python3 -m unittest discover -s test -v
python3 -m casino_dealer.cli --players 3

cd ~/DAPIER
python3 -m unittest discover -s dapier_sim_first/test -v
```

`dapier_sim_first` 단위 테스트는 장비 없이 실행할 수 있지만, G1 전체 재현에는
문서에 고정된 MuJoCo model, calibration 파일과 별도 환경이 필요하다.

ROS 2 실습은 Ubuntu 24.04와 ROS 2 Jazzy 기준이다.

- SO-101 mock safe teleop: [`so101_ros2/README.md`](so101_ros2/README.md)
- TurtleBot3 SLAM·Nav2: [`turtlebot3_ws/README.md`](turtlebot3_ws/README.md)
- 이동형 양팔 신발 정리: [`2ARM_ROBOT/README.md`](2ARM_ROBOT/README.md)
- 커스텀 차동구동 로봇: [`ros_dd_ws/README.md`](ros_dd_ws/README.md)
- 4축 시뮬레이션: [`jdcobot100_sim/README.md`](jdcobot100_sim/README.md)
- Arduino 4축 실물 제어: [`ros_arm/README.md`](ros_arm/README.md)

## SO-101을 처음 읽는 순서

1. [`so101/README.md`](so101/README.md)에서 현재 선택한 결과와 코드 위치를 확인한다.
2. [`sim-to-real foundation`](project-planning/2026-08-07-so101-sim-to-real-foundation.md)에서 Gate와 중단 조건을 읽는다.
3. [`dapier_sim_first/README.md`](dapier_sim_first/README.md)에서 G0/G1 근거를 확인한다.
4. [`LeRobot integration README`](so101/integrations/lerobot_v0_6_so101_mujoco/README.md)에서 IK/VLA 구현과 재현법을 확인한다.
5. 실물 작업은 [`카지노 딜러 runbook`](docs/SO101_CASINO_DEALER_RUNBOOK_KO.md)과 [`hardware tools`](so101/hardware_tools/README.md)를 함께 본다.

## 기록 원칙

- command와 measured state를 구분한다. 마지막 명령을 측정값으로 기록하지 않는다.
- SIM, MOCK, HW 결과를 분리한다. MuJoCo나 합성 message 통과는 실물 성공이 아니다.
- seed, revision, contract hash, camera profile, action horizon과 판정 기준을 결과와 함께 남긴다.
- `.venv`, `build/install/log`, 원시 Dataset v3, 영상, serial ID와 calibration JSON은 Git에서 제외한다.
- 외부 코드와 자산은 overlay, patch, provenance와 license 경계를 명시한다.

AI는 반복 코드와 테스트 초안, 명령 정리, 로그 비교에 보조적으로 사용했다.
README의 상태는 생성된 설명이 아니라 커밋된 코드와 저장된 record, 직접 실행한
결과를 기준으로 갱신한다.
