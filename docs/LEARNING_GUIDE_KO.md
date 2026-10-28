# DAPIER 교육·실습 안내

동국대학교 DAPIER 1기 Physical AI 과정에서 언어·제어 기초를 익히고, 카메라와 로봇의 상태를 학습·실행 데이터로 연결하는 방법을 공부하고 있다. 교육 기간은 **2026.07.06–11.04**, 현재 수강 중이며 총 편성은 **656시간**이다.

예제 코드를 읽거나 수정한 경험, SIM/MOCK에서 확인한 결과, 실물 장치에서 확인한 기록을 나눠 정리했다. 원본 교육 코드의 기능과 개인 구현 성과도 구분한다.

## 읽는 순서

| 단계 | 배운 내용 | 시작할 코드·문서 | 프로젝트와의 연결 |
|---|---|---|---|
| 1. 언어와 장치 | Python·C++·Linux, Arduino Serial/UART·PWM 코드 실습 | [4축 로봇암](../ros_arm/README.md) · [시리얼 브리지](../ros_arm/ros_arm/ros_arm_bridge.py) · [Arduino 코드](../ros_arm/arduino/ros_control/ros_control.ino) | 관절 이름·rad/degree·명령 문자열의 관계를 읽는다 |
| 2. 모델과 ROS 2 | URDF·TF·RViz, 노드·토픽·launch | [JDcobot 시뮬레이션](../jdcobot100_sim/README.md) · [SO-101 ROS 2](../so101_ros2/README.md) | C++ 관절 변환·한계와 Python 연구 제안을 구분한다 |
| 3. 이동과 좌표계 | Gazebo Harmonic, SLAM·Nav2·AMCL·프레임 | [TurtleBot3](../turtlebot3_ws/README.md) · [차동구동 로봇](../ros_dd_ws/README.md) | Jazzy 환경 이식, 지도·localization·이동 모듈 이해 |
| 4. 영상과 학습 데이터 | OpenCV·카메라 영상·조향 라벨·PC 학습 | [deepThinkCar-mini](../deepThinkCar_mini/README.md) | 수집·학습·검증 데이터와 실제 동작 결과를 구분한다 |
| 5. 정책 학습 | PyTorch·Transformer·ACT·추론 청크 | [모방학습 교육 코드](../so101_imitation_learning/README.md) · [ACT 단계별 예제](../so101_imitation_learning/103_ACT_step_by_step/) | ACT 공동 학습에서 입력·출력·청크 실행을 비교한다 |
| 6. 시뮬레이션과 연동 | MuJoCo·SO-101, 데이터·접촉 조건 | [한 팔 실험 이력](../so101/README.md) · [SWEEPICK 모델](../2ARM_ROBOT/sim/mobile_dual_so101/README.md) | 관측·state/action·단위·시각과 실제 장치 조건 대조 |

표는 학습 흐름이며 모든 폴더를 순서대로 설치해야 한다는 뜻은 아니다. 각 모듈의 환경이 다르므로 읽으려는 폴더의 README에서 버전과 실행 범위를 확인한다.

## 교육에서 SWEEPICK으로

- Arduino Serial/UART 송수신·PWM 제어는 **교육 코드 실습**으로 적었다. SO-101/STS3215 상태 조회는 [저장된 read-only 기록](../2ARM_ROBOT/docs/evidence/dual_arm_read_only_summary_20260820.json)과 함께 확인한다.
- [C++ 관절 코어](../so101_ros2/dapier_so101_core/src/joint_model.cpp)에서 변환·한계를 다루고, [Python 연구 계층](../2ARM_ROBOT/research/README.md)에서는 관측 출처·데이터·정책 제안을 다룬다. 이 스택의 미구현 하드웨어 드라이버를 실물 완료로 소개하지 않는다.
- ACT는 팀원과 공동 개발·검증했다. 나는 모방학습·시뮬레이션·Sim-to-Real·데이터 검증을 주로 담당했고, Visual SLAM은 협업으로 참여했다. [CUSTOM ACT V1](../2ARM_ROBOT/docs/CUSTOM_ACT_V1_KO.md)에서 학습 조건과 SIM 평가를 읽을 수 있다.
- Isaac Sim은 환경·장면·센서 기초를 학습한 범위다. 여기서 근거로 연결하는 MuJoCo·RoboTwin 프로젝트 실행과 구분한다.

## 다른 실험과 원본 자료

| 폴더 | 범위 |
|---|---|
| [dapier_sim_first](../dapier_sim_first/README.md) | 한 팔 SIM의 관측·접촉·성공 판정과 실패 조사 |
| [casino_dealer](../casino_dealer/README.md) | 카드 딜러의 초기 기구학·계획 실험 |
| [onshape/jdcobot100](../onshape/jdcobot100/README.md) | 로봇암 형상과 모델 변환 자료 |
| [puppy MuJoCo](../puppy/mujoco/README.md) | 별도 MuJoCo 학습 자료 |
| [학습 포크 통합 기록](FORK_MIGRATION_20260907_KO.md) | 교육용 원본과 DAPIER로 옮긴 경위 |

## 실행 전에

코드 탐색은 GitHub에서 할 수 있다. 장비 없이 연구 계층을 확인하려면 [저장소 첫 화면의 Python 테스트](../README.md#장비-없이-먼저-확인하기)를 따른다. ROS 2·Gazebo·MuJoCo의 설치·실행은 각 README의 환경을 사용한다.

과거 문서의 장치 이름·보정값·성능은 그 시점의 기록이다. 실제 로봇 접속·보정·제어는 [하드웨어 안전 절차](HARDWARE_SAFETY_KO.md)에 따라 현장에서 확인한다. 공개 저장소에는 녹음·원시 개인 데이터·장치 식별자·개인 보정 파일을 싣지 않는다.

정리 기준일: 2026-10-28. 기존 교육·실험 기록과 공개 코드를 대조한 안내이며 수료 또는 새 실물 실행 결과를 추가한 것은 아니다.
