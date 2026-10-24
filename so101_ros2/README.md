# DAPIER SO-101 ROS 2 Stack

SO-101의 실행 계층과 TurtleBot3 지역화 경계를 직접 이해하고 구성하기 위한 ROS 2
Jazzy 프로젝트다. LeRobot의 원클릭 실행 명령을 로봇 런타임으로 사용하지 않고,
Visual SLAM 지역화, 관절 계약, 안전 제한, 텔레옵, 하드웨어, 데이터 기록의 경계를
단계별로 구현한다.

현재 이 디렉터리의 어떤 패키지도 LeRobot을 import하거나 실행하지 않는다.
LeRobot은 추후 데이터셋 변환과 정책 학습을 비교하는 오프라인 도구로만
연결할 예정이다.

## 현재 상태

| 모듈 | 상태 | 실제로 확인한 범위 |
|---|---|---|
| dapier_localization_core | contract-first 구현 | localization schema, sequence/TTL, frame·pose·covariance, tracking-state motion gate |
| dapier_so101_core | 구현·테스트 완료 | 관절 순서, YAML 검증, calibration 변환, 위치·속도 제한, research intent ingress |
| dapier_so101_teleop | 모의 통합 검증 완료 | 명시적 enable, state freshness, 시작 자세 차이, trajectory 발행 |
| dapier_localization_orbslam3 | 미구현 | 공식 ORB-SLAM3 adapter와 replay 검증 예정 |
| dapier_so101_hardware | 미구현 | 교체 장비에서 읽기 전용 통신부터 시작 예정 |
| dapier_so101_data | 미구현 | rosbag2/MCAP episode 기록 예정 |
| dapier_so101_policy | 미구현 | 학습이 끝난 정책을 ROS 명령으로 연결할 예정 |

중요: 실제 모터 통신, torque 제어, 실제 리더-팔로워 동작, 실제 카메라 Visual
SLAM은 아직 검증하지 않았다. 예제 calibration 파일은 의도적으로
`verified: false`이며 하드웨어에서 사용하면 안 된다.

## 구조

~~~text
RGB/RGB-D + CameraInfo + optional IMU + wheel odometry
       ↓
dapier_localization_orbslam3       future sensor adapter
       ↓
dapier_localization_core           pose · TF contract · tracking quality
       ↓
Python shoe perception / planner   no hardware authority
       ↓
versioned base/arm intent
       ↓
dapier_so101_core                  TTL · limits · interlock · watchdog
       ↓
dapier_so101_teleop / future base and arm bridges
       ↓
SO-101 motor bus + TurtleBot3 base

All control modules share dapier_so101_core:
joint order · units · calibration schema · position limits · velocity limits

All localization producers and consumers share dapier_localization_core:
frames · pose · covariance · tracking state · map/session/reset generation
~~~

Visual SLAM은 로봇이 지도에서 어디에 있는지 추정하고, Python RGB-D perception은
신발이 camera/base/map frame에서 어디에 있는지 추정한다. 두 경로는 calibrated TF로
결합하지만 같은 모듈로 합치지 않는다.

## 왜 코어와 ROS 노드를 분리했는가

관절 변환, 제한 계산, localization estimate 검증은 ROS 토픽이나 시리얼 포트가 없어도
검증할 수 있어야 한다. 그래서 `dapier_so101_core`와
`dapier_localization_core`는 rclcpp에 의존하지 않는 C++ 라이브러리로 만들었다.

`dapier_localization_core`는 ORB feature extraction이나 map optimization을 구현하지
않는다. 공식 ORB-SLAM3 adapter, deterministic fake, bag/MCAP replay producer가 같은
계약을 출력하게 한다. `tracking`, `recently_lost`, `lost`, `relocalized` 상태와 map/reset
generation을 소비자가 무시하지 못하게 하는 것이 이 코어의 책임이다.

`dapier_so101_teleop`은 ROS 메시지 수신, 시간 확인, enable 서비스, 명령 발행만
담당한다. 모터 register나 calibration EEPROM을 직접 만지지 않는다. 시리얼
포트의 소유자는 미래의 `dapier_so101_hardware` 하나로 제한할 계획이다.

자세한 결정 근거는 [ADR 0001](docs/adr/0001-own-ros2-runtime.md), 코드 읽는
순서는 [core 학습 노트](docs/modules/01-core.md)와
[safe teleop 학습 노트](docs/modules/02-safe-teleop.md)에 기록했다.
저장소 전체 계층 결정은 루트의 `docs/architecture/`를 본다.

## 빌드와 테스트

이 저장소가 `~/DAPIER`에 clone되어 있다는 기준이다.

~~~bash
mkdir -p ~/so101_ros2_ws/src
ln -s ~/DAPIER/so101_ros2 ~/so101_ros2_ws/src/dapier-so101-ros2

cd ~/so101_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-up-to dapier_localization_core dapier_so101_teleop
source install/setup.bash

colcon test --packages-select \
  dapier_localization_core \
  dapier_so101_core \
  dapier_so101_teleop
colcon test-result --verbose
~~~

장비가 없는 환경에서는 저장소 루트에서 다음 smoke를 실행한다.

~~~bash
scripts/verify-hardware-free
~~~

2026-08-04 기준 기존 ROS 결과:

- `dapier_so101_core`, `dapier_so101_teleop` 빌드 성공
- core GTest 7개 통과
- leader/follower state가 없을 때 enable 거부 확인
- 합성 JointState가 정렬됐을 때 enable 및 JointTrajectory 발행 확인
- disable 서비스와 프로세스 정상 종료 확인

새 `dapier_localization_core`의 ROS 2 `colcon build/test`는 사용자 로컬 Jazzy에서
별도 확인해야 한다. standalone C++17 contract smoke는 hardware-free 검증에 포함한다.

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
- localization이 initializing/recently-lost/lost이면 navigation과 manipulation을 hold한다.
- relocalization 또는 map/reset generation 변경 뒤에는 이전 목표와 action chunk를
  폐기하고 재계획한다.
- localization package는 `/cmd_vel`, trajectory, motor bus 또는 torque를 직접 쓰지 않는다.
- 현재 disable은 새 명령 발행을 멈추는 동작이다. 물리적인 torque OFF와
  비상 정지는 하드웨어 계층에서 별도로 구현해야 한다.

## 참고 코드와 소유 범위

기존 `~/so101_ros2_ws/src/so101-ros-physical-ai`는
`legalaspro/so101-ros-physical-ai`의 clone이며 참고 프로젝트다. 이 새 디렉터리는
그 저장소의 패키지 이름을 바꾼 복사본이 아니다. ROS 2 메시지 규약과 공식
ros2_control 인터페이스는 재사용하지만, 코어 계약과 안전 텔레옵은 DAPIER에서
별도로 작성하고 테스트한다.

ORB-SLAM3를 연결할 때도 source revision, license, vocabulary·configuration provenance를
고정한다. 논문의 Intel Core i7-7700 timing을 Raspberry Pi 성능으로 간주하지 않고
recorded input에서 별도 benchmark한다.

Apache-2.0 코드나 외부 모델·mesh를 가져오는 단계에서는 원본과 라이선스를
파일 단위로 명시한다.

## 다음 구현 순서

1. localization contract와 standalone C++ validator
2. deterministic fake 및 rosbag2/MCAP replay producer
3. low texture·blur·occlusion·frame drop·timestamp skew·lost/relocalized 회귀
4. 공식 ORB-SLAM3 adapter와 TF publisher
5. localization quality를 Nav2·base/arm safety gate에 연결
6. STS3215 패킷을 파일 기반 fixture로 검사하는 버스 모듈
7. 실제 장비에서 motor ID 1~6 읽기 전용 진단
8. ros2_control SystemInterface의 configure/read 단계
9. 현재 위치롔 command를 seed한 뒤 제한된 write 단계
10. hardware watchdog과 torque OFF 서비스
11. 실제 leader-follower 저속 검증
12. rosbag2/MCAP episode recorder
13. LeRobotDataset 변환 및 정책 bridge
