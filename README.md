# DAPIER · Physical AI 과정의 미니 프로젝트와 실습 기록

동국대학교 DAPIER 1기 Physical AI 과정(2026.07.06–11.04, 656시간)에서 한 개인 실험, ROS 2 실습, 교육 학습 코드를 모은 저장소다.
최종 팀 프로젝트 **SWEEPICK**은 별도 저장소로 옮겼다 → **[Alpenj/sweepick](https://github.com/Alpenj/sweepick)**

| 폴더 | 내용 |
|---|---|
| [mini-projects](mini-projects/README.md) | SWEEPICK과 별개로 직접 설계·실험한 작은 프로젝트 |
| [ros2-practice](ros2-practice/README.md) | 교재·수업 예제를 직접 실행하며 오류를 진단한 ROS 2 실습 |
| [coursework](coursework/README.md) | 교육용 원본을 따라 읽고 수정한 학습 코드 |
| [docs](docs/) | 교육·실습 안내, 하드웨어 안전 절차, 기획 기록 |

## 미니 프로젝트

| 프로젝트 | 한 일 | 폴더 |
|---|---|---|
| deepThinkCar 후보 평가·정지 조건 | 카메라·차선 상실 정지와 watchdog. 같은 분할에서 조향 MAE 14.93°→4.93°를 확인했지만 평가 범위가 좁아 실차 적용은 보류 | [deepThinkCar_mini](mini-projects/deepThinkCar_mini/README.md) · [PR](https://github.com/JD-edu/deepThinkCar_mini/pull/1) |
| CardBench | 양팔 카지노 딜러 아이디어를 위해 게임 순서와 관측·행동 형식을 먼저 코드로 고정한 소프트웨어 실험 | [casino_dealer](mini-projects/casino_dealer/README.md) |
| SO-101 sim-first G0–G1 | 한 팔 SO-101의 SIM 환경 점검과 scripted pick-and-lift 기록 | [dapier_sim_first](mini-projects/dapier_sim_first/README.md) |

## ROS 2 실습

| 실습 | 확인한 내용 | 폴더 |
|---|---|---|
| ROS 2 차동구동 로봇 | URDF·물리 장면·TF·SLAM·Nav2를 한 흐름으로 확인한 Jazzy 워크스페이스 | [ros_dd_ws](ros2-practice/ros_dd_ws/README.md) |
| TurtleBot3 SLAM·Nav2 | Humble·Gazebo Classic 예제를 Jazzy·Gazebo Harmonic으로 옮기며 메시지·플러그인 차이 진단 | [turtlebot3_ws](ros2-practice/turtlebot3_ws/README.md) |
| ROS 2 + Arduino 4축 로봇암 | ROS 2 JointState를 Arduino 시리얼 명령과 servo 각도로 연결 | [ros_arm](ros2-practice/ros_arm/README.md) |
| jdcobot100 시뮬레이션 | 실제 장비 연결 전 관절·모델·컨트롤러·launch 흐름 확인, Onshape 모델 내보내기 | [jdcobot100_sim](ros2-practice/jdcobot100_sim/README.md) |

## 교육 학습 코드

| 위치 | 내용 |
|---|---|
| [so101_imitation_learning](coursework/so101_imitation_learning/README.md) | PyTorch → Transformer → ACT → SO-101·MuJoCo를 이어 읽는 학습용 포크 |
| [so101](coursework/so101/README.md) | SWEEPICK 이전 한 팔 학습·접촉 모델·장치 연결 초기 기록 |
| [puppy](coursework/puppy/mujoco/README.md) | Puppy 4족 로봇 URDF → MuJoCo 변환 실습 |

## 기록을 읽을 때

- **SIM / MOCK / HW**를 구분한다. 테스트 통과, 예측오차와 실제 작업 완료는 서로 다른 결과다.
- 교육 예제·외부 코드·자산의 출처는 각 폴더에 보존하고, 직접 구현·실험한 부분은 각 README에서 따로 적는다. 이전 학습 포크를 옮긴 내역은 [통합 기록](docs/FORK_MIGRATION_20260907_KO.md)에 있다.
- 실물 보정·제어 전에는 [하드웨어 안전 절차](docs/HARDWARE_SAFETY_KO.md)를 확인한다. 강의 내용과 프로젝트 적용의 연결은 [교육·실습 안내](docs/LEARNING_GUIDE_KO.md)에 있다.

2026-11-03에 폴더를 위 구성으로 정리했다. 옮기기 전 경로의 이력은 [정리 전 상태(2399bfc)](https://github.com/Alpenj/DAPIER/tree/2399bfc531074f29e8c80905b6082b90f99cbd60)에서 볼 수 있다.
[`2ARM_ROBOT`](2ARM_ROBOT/README.md)에는 이미 공유된 링크를 위한 이동 안내만 남겼다.
