# DAPIER SO-101 ROS 2 Stack

SO-101의 실행 계층을 직접 이해하고 구성하기 위한 ROS 2 Jazzy 프로젝트다.
LeRobot의 원클릭 실행 명령을 로봇 런타임으로 사용하지 않고, 관절 계약,
안전 제한, 텔레옵, 하드웨어, 데이터 기록의 경계를 단계별로 구현한다.

현재 이 디렉터리의 어떤 패키지도 LeRobot을 import하거나 실행하지 않는다.
LeRobot은 추후 데이터셋 변환과 정책 학습을 비교하는 오프라인 도구로만
연결할 예정이다.

## 현재 상태

| 모듈 | 상태 | 실제로 확인한 범위 |
|---|---|---|
| dapier_so101_core | 구현·테스트 완료 | 관절 순서, YAML 검증, calibration 변환, 위치·속도 제한 |
| dapier_so101_teleop | 모의 통합 검증 완료 | 명시적 enable, state freshness, 시작 자세 차이, trajectory 발행 |
| dapier_so101_hardware | 미구현 | 교체 장비에서 읽기 전용 통신부터 시작 예정 |
| dapier_so101_data | 미구현 | rosbag2/MCAP episode 기록 예정 |
| dapier_so101_policy | 미구현 | 학습이 끝난 정책을 ROS 명령으로 연결할 예정 |

중요: 실제 모터 통신, torque 제어, 실제 리더-팔로워 동작은 아직 검증하지
않았다. 예제 calibration 파일은 의도적으로 verified: false이며 하드웨어에서
사용하면 안 된다.

## 구조

~~~text
SO-101 motor bus
       ↕
dapier_so101_hardware             future ros2_control SystemInterface
       ↕
/joint_states  ↔  ros2_control  ↔  /joint_trajectory
       ↑                                  ↑
       │                                  │
leader state ── dapier_so101_teleop ──────┘
       │
       ├── dapier_so101_data              future rosbag2/MCAP recorder
       └── dapier_so101_policy            future learned-policy bridge

All modules share dapier_so101_core:
joint order · units · calibration schema · position limits · velocity limits
~~~

## 왜 코어와 ROS 노드를 분리했는가

관절 변환과 제한 계산은 ROS 토픽이나 시리얼 포트가 없어도 검증할 수 있어야
한다. 그래서 dapier_so101_core는 rclcpp에 의존하지 않는 C++ 라이브러리로
만들었다. 이 계산을 하드웨어 드라이버, 텔레옵, 데이터 검증기가 함께 사용하면
모듈마다 관절 순서와 단위가 달라지는 문제를 줄일 수 있다.

dapier_so101_teleop은 ROS 메시지 수신, 시간 확인, enable 서비스, 명령 발행만
담당한다. 모터 register나 calibration EEPROM을 직접 만지지 않는다. 시리얼
포트의 소유자는 미래의 dapier_so101_hardware 하나로 제한할 계획이다.

자세한 결정 근거는 [ADR 0001](docs/adr/0001-own-ros2-runtime.md), 코드 읽는
순서는 [core 학습 노트](docs/modules/01-core.md)와
[safe teleop 학습 노트](docs/modules/02-safe-teleop.md)에 기록했다.
이번 구현 과정과 검증 경계는
[GitHub 블로그 초안](docs/blog/2026-08-04-own-so101-ros2-stack.md)에도 별도로
정리했다.

## 빌드와 테스트

이 저장소가 ~/DAPIER에 clone되어 있다는 기준이다.

~~~bash
mkdir -p ~/so101_ros2_ws/src
ln -s ~/DAPIER/so101_ros2 ~/so101_ros2_ws/src/dapier-so101-ros2

cd ~/so101_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to dapier_so101_teleop
source install/setup.bash

colcon test --packages-select dapier_so101_core dapier_so101_teleop
colcon test-result --verbose
~~~

2026-08-04 기준 결과:

- 두 패키지 빌드 성공
- core GTest 7개 통과
- leader/follower state가 없을 때 enable 거부 확인
- 합성 JointState가 정렬됐을 때 enable 및 JointTrajectory 발행 확인
- disable 서비스와 프로세스 정상 종료 확인

## 모의 실행

다음 launch는 시리얼 포트를 열지 않는다. 실제 하드웨어 드라이버와 연결하기
전 ROS 인터페이스만 학습하고 확인하는 용도다.

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/so101_ros2_ws/install/setup.bash

ros2 launch dapier_so101_teleop safe_teleop.launch.py
~~~

노드는 기본적으로 DISABLED 상태다. 리더와 팔로워의 신선한 JointState가 있고
두 자세 차이가 허용값 안에 들어온 뒤에만 아래 요청이 성공한다.

~~~bash
ros2 service call /dapier_so101/teleop/enable std_srvs/srv/SetBool "{data: true}"
~~~

끄기:

~~~bash
ros2 service call /dapier_so101/teleop/enable std_srvs/srv/SetBool "{data: false}"
~~~

## 안전 경계

- 시작 시 명령을 발행하지 않는다.
- enable 서비스가 명시적으로 성공해야만 명령을 발행한다.
- 리더와 팔로워 상태가 모두 필요하다.
- 상태가 0.25초 이상 끊기면 자동으로 disable한다.
- 시작 자세 차이가 기본 0.35 rad보다 크면 enable을 거부한다.
- 모든 명령은 관절 위치 제한과 초당 변화량 제한을 통과한다.
- 현재 disable은 새 명령 발행을 멈추는 동작이다. 물리적인 torque OFF와
  비상 정지는 하드웨어 계층에서 별도로 구현해야 한다.

## 참고 코드와 소유 범위

기존 ~/so101_ros2_ws/src/so101-ros-physical-ai는
legalaspro/so101-ros-physical-ai의 clone이며 참고 프로젝트다. 이 새 디렉터리는
그 저장소의 패키지 이름을 바꾼 복사본이 아니다. ROS 2 메시지 규약과 공식
ros2_control 인터페이스는 재사용하지만, 코어 계약과 안전 텔레옵은 DAPIER에서
별도로 작성하고 테스트한다.

Apache-2.0 코드나 외부 모델·mesh를 가져오는 단계에서는 원본과 라이선스를
파일 단위로 명시한다.

## 다음 구현 순서

1. STS3215 패킷을 파일 기반 fixture로 검사하는 버스 모듈
2. 실제 장비에서 motor ID 1~6 읽기 전용 진단
3. ros2_control SystemInterface의 configure/read 단계
4. 현재 위치로 command를 seed한 뒤 제한된 write 단계
5. hardware watchdog과 torque OFF 서비스
6. 실제 leader-follower 저속 검증
7. rosbag2/MCAP episode recorder
8. LeRobotDataset 변환 및 정책 bridge
