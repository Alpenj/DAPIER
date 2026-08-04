# DAPIER — 직접 조립하고 기록하며 배우는 로봇 프로젝트

이 저장소는 제가 로봇을 직접 조립하고, 터미널 로그와 실제 움직임을
확인하면서 배운 내용을 쌓아가는 작업 공간입니다. 아직 완성품을 소개하는
저장소가 아니라, 시뮬레이션·실물 제어·데이터 수집을 한 단계씩 연결하는
과정 자체를 남기고 있습니다.

현재 저장소에는 예전에 만든 Arduino 4축 로봇암 코드와, 앞으로 확장하려는
SO-101 기반 카드 딜러 실험이 함께 있습니다. 두 프로젝트를 한 번에
완성했다고 가정하지 않고, 각 폴더의 테스트와 실제 장비 확인 결과를
구분해서 기록합니다.

양팔 카지노 딜러라는 프로젝트 아이디어와, episode를 모아 policy를 만들고
한 팔에서 양팔로 확장하자는 작업 방향은 제가 정했습니다.

## 이 프로젝트에서 AI를 사용하는 방식

AI는 제가 정한 방향을 코드와 문서로 옮길 때 ROS 2 개념을 다시 설명받거나,
로그를 읽고, 명령어·테스트 코드의 반복 작업을 줄이는 보조 도구로 사용합니다.
배선, 전원, 모터 ID, 캘리브레이션, 실제 관절 방향과 안전 판정은 제가 직접
확인합니다. AI가 제안한 명령도 그대로 믿지 않고 한 줄씩 실행한 결과를
다음 작업의 기준으로 삼습니다.

따라서 체크되지 않은 항목은 아직 하지 않은 작업이고, 시뮬레이션에서
통과한 것이 실제 로봇에서도 통과했다는 뜻은 아닙니다.

## 어떤 패키지를 사용해야 하나

이 저장소는 용도에 따라 ROS 2 패키지를 분리합니다.

| 목적 | 패키지 | Arduino·시리얼 포함 |
|---|---|---|
| RViz·Gazebo 시뮬레이션만 학습 | `jdcobot100_sim` | 아니요 |
| 실제 Arduino 서보까지 제어 | `ros_arm` | 예 |
| 양팔 카지노 딜러 계약·플래너 개발 | `casino_dealer` | 아니요 |

시뮬레이션만 실행하려면 [`jdcobot100_sim`](jdcobot100_sim/README.md)을
사용합니다. 아래 내용은 실물 제어용 `ros_arm` 설명입니다.

양팔 카드 딜러의 관측·행동 계약과 블랙잭 딜 순서를 개발하려면
[`casino_dealer`](casino_dealer/README.md)를 사용합니다. 현재 버전은
실물 구동 전 단계이며, 외부 의존성 없이 JSON 딜 계획과 단위 테스트를
실행할 수 있습니다. SO-101 캘리브레이션부터 episode 수집·검수·정책
평가까지의 사람이 따라 하는 순서는
[`docs/SO101_CASINO_DEALER_RUNBOOK_KO.md`](docs/SO101_CASINO_DEALER_RUNBOOK_KO.md)에
정리되어 있습니다.

Arduino Uno에 연결된 네 개의 SG90/MG90 서보를 ROS 2 Jazzy의
`sensor_msgs/msg/JointState`와 USB 시리얼로 제어하는 학습 프로젝트입니다.

## 전체 데이터 흐름

```text
ros_arm_sequence_gui
        │ /joint_states (관절 이름 + rad)
        ▼
ros_arm_control
        │ A,base,shoulder,forearm,upper\n (servo degree)
        ▼
Arduino Uno
        │ PWM
        ▼
D3 / D5 / D6 / D9 서보
```

RViz와 실제 로봇은 같은 `/joint_states` 메시지를 사용하므로 슬라이더를
움직이면 화면의 모델과 실제 팔이 함께 움직입니다.

## 배선

| 관절 | Arduino 신호 핀 |
|---|---:|
| Base | D3 |
| Shoulder | D5 |
| Forearm | D6 |
| Upper arm | D9 |

서보 4개는 Arduino 5V 핀에서 직접 구동하지 않습니다. 별도 5V 전원을
사용하고 외부 전원의 GND와 Arduino GND를 반드시 공통으로 연결합니다.

## 좌표 변환

ROS/URDF의 관절 위치 단위는 radian이고 중심은 `0 rad`입니다. 일반적인
서보 명령 단위는 degree이고 중심은 `90°`입니다.

```text
servo_degree = 90 + degrees(urdf_radian)
```

예:

```text
URDF -0.1745 rad ≈ servo 80°
URDF  0.0000 rad = servo 90°
URDF +0.1745 rad ≈ servo 100°
```

브리지는 JointState 배열 순서를 가정하지 않고 다음 관절 이름으로 값을
찾습니다.

```text
dof_base
dof_shoulder
dof_elbow
dof_wrist_pitch
```

## 안전 제한

조립 직후 충돌을 피하기 위해 세 단계에서 동일하게 제한합니다.

```text
URDF/GUI: -30° ~ +30°
ROS bridge: servo 60° ~ 120°
Arduino: servo 60° ~ 120°
```

통신이 끊기면 Arduino는 현재 자세를 유지합니다. 서보를 갑자기 0°로
보내는 것보다 조립된 로봇암에 안전하기 때문입니다.

## Arduino 업로드

Arduino IDE에서 `arduino/ros_control/ros_control.ino`를 열어 Uno와
`/dev/ttyUSB0`을 선택한 뒤 업로드합니다.

CLI 예:

```bash
arduino --upload \
  --board arduino:avr:uno \
  --port /dev/ttyUSB0 \
  --pref upload.verify=false \
  arduino/ros_control/ros_control.ino
```

시리얼 프로토콜:

```text
PC → Arduino: A,90,80,100,90
Arduino → PC: OK,90,80,100,90
```

## ROS 2 빌드

```bash
cd ~/my_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros_arm
source install/setup.bash
```

## 실행

```bash
ros2 launch ros_arm model_rviz_launch.py
```

Arduino까지 함께 구동할 때만 다음 launch를 사용합니다.

```bash
ros2 launch ros_arm display_launch.py serial_port:=/dev/ttyUSB0
```

실행되는 노드:

- `robot_state_publisher`: URDF와 JointState를 TF로 변환
- `ros_arm_sequence_gui`: 관절 슬라이더, 시퀀스 저장·재생, `/joint_states` 발행
- `ros_arm_control`: radian을 서보 degree로 바꿔 시리얼 전송
- `rviz2`: 화면의 로봇 모델 표시

GUI의 `Randomize` 버튼은 누르지 않고 각 슬라이더를 천천히 움직여
관절 방향과 실제 안전 범위를 확인합니다.

## 서보 떨림 줄이기

브리지는 1°짜리 미세 명령 변화를 무시하는 `deadband_degrees=2`와
초당 최대 15회만 전송하는 `command_rate_hz=15.0`을 사용합니다.
이 설정은 GUI 값의 미세 변화와 과도한 시리얼 명령 때문에 목표점 주변을
왕복하는 현상을 줄입니다.

그래도 가만히 있을 때 계속 떠는 경우는 대부분 전원 또는 기계적
부하 문제입니다.

- 서보 4개는 5V 외부 전원을 사용하고 충분한 전류 용량을 확보합니다.
- 외부 전원 GND와 Arduino GND를 반드시 공통 연결합니다.
- 전원 레일 가까이에 470~1000µF 전해 콘덴서를 병렬 연결할 수 있습니다.
- 관절 끝에서 버티거나 링크가 비틀린 상태라면 범위를 더 좁힙니다.
- `Randomize`를 연속으로 누르면 네 관절 목표가 계속 바뀌므로 사용하지 않습니다.

SG90 계열의 내부 가변저항과 기어 유격 때문에 약간의 떨림은 남을 수
있습니다. 정지 시 PWM을 끄는 `detach()`는 떨림은 멈추지만 팔이 중력으로
떨어질 수 있어 이 프로젝트의 기본 동작에는 사용하지 않습니다.

## 자세 시퀀스 저장과 재생

launch를 실행하면 전용 시퀀스 GUI가 열립니다. GUI에 보이는 숫자는 실제
서보 각도인 60~120°이며, 내부에서
`radians(servo_degree - 90)`으로 변환해 `/joint_states`를 발행합니다.

1. 슬라이더로 한 관절씩 안전한 자세를 만듭니다.
2. 단계 이름과 이동 시간을 입력합니다.
3. `현재 자세 추가`를 누릅니다.
4. 필요한 자세를 순서대로 추가하고 표에서 확인합니다.
5. `재생`으로 실제 암과 RViz 움직임을 확인합니다.
6. `JSON 저장`으로 저장하고 다음 실행에서 다시 불러옵니다.

표의 셀은 더블클릭해 수정할 수 있습니다. 행을 더블클릭하면 해당 자세가
슬라이더와 실제 암에 적용됩니다. 순서 변경, 선택 삭제, 현재 자세로
덮어쓰기, 반복 재생도 지원합니다.

샘플 파일은 `sequences/fold_and_open.json`입니다.

```json
{
  "name": "center",
  "angles": [90, 90, 90, 90],
  "duration": 1.0
}
```

재생 중에는 시작 자세와 목표 자세 사이를 20ms 간격으로 선형 보간합니다.
일시정지는 현재 자세를 유지하고, 정지는 재생만 끝내며 서보를 위험한
0° 위치로 보내지 않습니다.

## 검증 명령

```bash
ros2 topic echo /joint_states
ros2 node list
```

브리지 로그의 정상 예:

```text
TX A,90,90,90,90
RX OK,90,90,90,90
```

## 1초 자동 관절 테스트 노드

GUI 대신 `auto_joint_publisher.py`를 사용하면 1초마다 한 관절씩 안전하게
매핑을 시험할 수 있습니다.

```text
전체 90°
→ Base 100° → Base 80° → Base 90°
→ Shoulder 100° → Shoulder 80° → Shoulder 90°
→ Forearm 100° → Forearm 80° → Forearm 90°
→ Upper 100° → Upper 80° → Upper 90°
→ 반복
```

실제 Arduino와 RViz를 자동으로 시험하는 명령:

```bash
ros2 launch ros_arm auto_demo_launch.py
```

기본 진폭은 중심에서 ±10°이고 주기는 1초입니다. 필요하면 launch
argument로 바꿀 수 있습니다.

```bash
ros2 launch ros_arm auto_demo_launch.py \
  period_seconds:=2.0 amplitude_degrees:=5.0
```

실물 모드에서는 노드가 `/joint_states`를 발행하므로 Arduino 브리지까지
함께 움직입니다. 암 주변을 비우고 전원을 확인한 뒤 실행해야 합니다.

## Gazebo 물리 시뮬레이션

Gazebo launch는 원본 `ros_arm.urdf`에서 혼합 모델을 동적으로 만듭니다.
`gazebo_description.py`가 실제 CAD STL visual과 관절 원점은 그대로
보존하고, `1e-09 kg` 수준의 inertial을 안정적인 학습용 값으로
교체합니다.

첫 번째 단순 형상 모델로 controller 연결을 확인한 뒤 실제 CAD visual로
교체했습니다. CAD 좌표에 추정 collision box를 적용하면 링크가 서로
밀어내며 속도가 과도하게 튀었기 때문에, 현재 기본 launch는 실제 외형의
모션 확인에 집중하도록 collision과 gravity를 비활성화했습니다. 정확한
충돌 시뮬레이션은 링크별 collision 치수를 별도로 측정한 뒤 추가해야
합니다.

필요 패키지:

```bash
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers
```

Gazebo 자동 시험:

```bash
ros2 launch ros_arm gazebo_auto_demo_launch.py
```

화면이 열리면 왼쪽 `Entity Tree`에서 `ros_arm`을 선택하고 `F`를 눌러
카메라를 로봇에 맞춥니다. 마우스 왼쪽 드래그는 회전, 가운데 드래그는
이동, 휠은 확대/축소입니다. 상단 재생/일시정지 버튼으로 물리 시계를
제어합니다. Gazebo 창을 닫으면 관련 ROS 노드도 함께 종료되므로 다시
보려면 위 launch 명령을 재실행합니다.

이 launch는 다음 순서로 동작합니다.

1. Gazebo Harmonic을 시작합니다.
2. `robot_description`의 로봇을 `ros_gz_sim create`로 생성합니다.
3. `gz_ros2_control`이 네 관절의 position interface를 등록합니다.
4. `joint_state_broadcaster`가 실제 시뮬레이션 각도를 `/joint_states`로
   발행합니다.
5. `arm_position_controller`가 네 관절 위치 명령을 받습니다.
6. 자동 노드가 목표를 1초마다 바꾸고 50Hz로 보간한 명령을 보냅니다.

Gazebo 모드에서는 자동 노드가 `/joint_states`를 직접 발행하지 않습니다.
그 토픽은 시뮬레이터가 측정한 결과를 전달하는 피드백이기 때문입니다.

```text
auto_joint_publisher
        │ /arm_position_controller/commands
        ▼
arm_position_controller
        ▼
gz_ros2_control → Gazebo physics
        │
        └→ joint_state_broadcaster → /joint_states
```

controller 확인:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic info /arm_position_controller/commands --verbose
```

정상 상태:

```text
joint_state_broadcaster active
arm_position_controller active
```

실물 launch와 Gazebo launch를 동시에 실행하면 토픽이 섞일 수 있으므로
처음 학습할 때는 한 번에 하나만 실행합니다. Gazebo launch에는 Arduino
브리지가 포함되지 않아 시뮬레이션 동작이 실제 암으로 전달되지 않습니다.

## 주요 파일

```text
ros_arm/
├── arduino/ros_control/ros_control.ino
├── launch/display_launch.py
├── launch/model_rviz_launch.py
├── launch/auto_demo_launch.py
├── launch/gazebo_auto_demo_launch.py
├── ros_arm/auto_joint_publisher.py
├── ros_arm/gazebo_description.py
├── ros_arm/ros_arm_bridge.py
├── config/gazebo_controllers.yaml
├── rviz/ros_arm.rviz
├── meshes -> ../onshape/jdcobot100/reference/assets
├── urdf/ros_arm.urdf
├── urdf/ros_arm_gazebo.urdf
├── package.xml
└── setup.py
```
