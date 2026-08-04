# jdcobot100_sim

이 패키지는 제가 실제 부품을 연결하기 전에 RViz·Gazebo에서 관절 이름,
launch 순서, controller 연결을 확인하기 위해 사용하는 시뮬레이션 공간입니다.
여기서 보이는 움직임을 실제 서보가 움직였다고 해석하지 않습니다.

`jdcobot100`의 **시뮬레이션 전용** ROS 2 Jazzy 패키지입니다. RViz와
Gazebo Harmonic 실행에 필요한 파일만 포함합니다. Arduino 펌웨어, USB
시리얼 브리지, 실제 서보 제어 코드는 들어 있지 않습니다.

## ROS 2, RViz, Gazebo의 관계

- ROS 2: 노드, 토픽, TF, launch를 연결하는 로봇 소프트웨어 프레임워크
- RViz: ROS 2가 전달하는 URDF, TF, 센서 데이터를 보는 시각화 도구
- Gazebo: 중력, 관절, 충돌 등을 계산하는 물리 시뮬레이터
- MuJoCo: ROS 2와 독립적으로도 실행할 수 있는 별도의 물리 시뮬레이터

즉, ROS 2가 RViz인 것은 아닙니다. RViz와 Gazebo가 ROS 2의 데이터와
launch 기능을 이용합니다.

## 포함된 관절

```text
dof_base
dof_shoulder
dof_elbow
dof_wrist_pitch
```

## 설치

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 기준입니다.

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers
```

## 워크스페이스에 넣고 빌드

압축을 풀면 `jdcobot100_sim` 폴더가 나옵니다. 폴더 자체를 ROS 2
워크스페이스의 `src`에 넣습니다.

```bash
mkdir -p ~/jdcobot_ws/src
cp -r ~/Downloads/jdcobot100_sim ~/jdcobot_ws/src/
cd ~/jdcobot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select jdcobot100_sim
source install/setup.bash
```

새 터미널을 열 때마다 아래 두 줄을 다시 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/jdcobot_ws/install/setup.bash
```

## RViz 실행

```bash
ros2 launch jdcobot100_sim rviz.launch.py
```

`joint_state_publisher_gui`의 네 슬라이더를 움직이면 RViz 로봇 관절이
움직입니다. RViz는 물리를 계산하지 않고 URDF와 TF를 시각화합니다.

## Gazebo 실행

RViz를 종료한 뒤 새 터미널에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/jdcobot_ws/install/setup.bash
ros2 launch jdcobot100_sim gazebo.launch.py
```

Gazebo 기본 UI가 열리고 7초 뒤 네 관절이 차례로 ±10도 움직입니다.
별도 카메라·GUI 설정 파일은 적용하지 않았습니다. 마우스 왼쪽 드래그로
회전하고 가운데 드래그로 이동하며 휠로 확대·축소합니다.

정상 여부를 확인하는 명령은 다음과 같습니다.

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /clock --once
```

`joint_state_broadcaster`와 `arm_position_controller`가 `active`이면
컨트롤러가 정상 연결된 것입니다. 종료는 launch를 실행한 터미널에서
`Ctrl+C`를 누릅니다.

## MuJoCo 모델 위치

MuJoCo용 MJCF는 ROS 2 패키지와 형식이 달라 이 폴더에 중복해서 넣지
않았습니다. DAPIER 저장소의 다음 파일을 사용합니다.

```text
onshape/jdcobot100/reference/scene.xml
onshape/jdcobot100/reference/jdcobot100.xml
```

## 실제 로봇을 연결하려면

이 패키지에는 하드웨어 제어 기능이 없습니다. Arduino와 실제 서보를
연결하려면 별도의 `ros_arm` 패키지를 사용합니다. 저는 먼저 이 패키지로
URDF와 controller 흐름을 확인한 뒤에만 실물 작업으로 넘어갑니다.

AI는 launch 명령과 토픽 관계를 정리하는 데 도움을 줄 수 있지만, Gazebo의
정상 출력만으로 모터 배선이나 실제 안전 범위를 대신 판단하지 않습니다.
