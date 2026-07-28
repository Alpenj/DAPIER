# ROS 2 + Arduino 4축 로봇암

Arduino Uno에 연결된 네 개의 SG90/MG90 서보를 ROS 2 Humble의
`sensor_msgs/msg/JointState`와 USB 시리얼로 제어하는 학습 프로젝트입니다.

## 전체 데이터 흐름

```text
joint_state_publisher_gui
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
base_shoulder
shoulder_arm1
arm1_arm2
arn2_end_arm
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
source /opt/ros/humble/setup.bash
colcon build --packages-select ros_arm
source install/setup.bash
```

## 실행

```bash
ros2 launch ros_arm display_launch.py serial_port:=/dev/ttyUSB0
```

실행되는 노드:

- `robot_state_publisher`: URDF와 JointState를 TF로 변환
- `joint_state_publisher_gui`: 관절 슬라이더와 `/joint_states` 발행
- `ros_arm_control`: radian을 서보 degree로 바꿔 시리얼 전송
- `rviz2`: 화면의 로봇 모델 표시

GUI의 `Randomize` 버튼은 누르지 않고 각 슬라이더를 천천히 움직여
관절 방향과 실제 안전 범위를 확인합니다.

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

## 주요 파일

```text
ros_arm/
├── arduino/ros_control/ros_control.ino
├── launch/display_launch.py
├── ros_arm/ros_arm_bridge.py
├── rviz/ros_arm.rviz
├── urdf/ros_arm.urdf
├── package.xml
└── setup.py
```
