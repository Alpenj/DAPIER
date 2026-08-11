# ros_dd_ws — 패키지 구조 및 가상 물리 환경 (ROS2 Jazzy)

`ros_dd_*` 커스텀 차동구동(differential-drive) 로봇 워크스페이스. `turtlebot3_ws`와
마찬가지로 이 PC는 **ROS2 Jazzy + gz-sim(Harmonic)**만 설치돼 있어, 원본 교재가
전제하는 Gazebo Classic 파이프라인(`gazebo_ros`, `gzserver`/`gzclient`,
`libgazebo_ros_*` 플러그인)은 존재하지 않는다. 아래 구조는 전부 gz-sim 기준으로
포팅·검증된 상태다.

## 1. 패키지 구조

```
ros_dd_ws/src/
├── ros_dd_description/   # URDF/xacro, robot_state_publisher 입력
├── ros_dd_gazebo/        # world, launch, ros_gz_bridge 설정, 커스텀 모델
├── ros_dd_cartographer/  # SLAM (cartographer_ros)
├── ros_dd_navigation/    # Nav2 bringup (AMCL/BT navigator/controller/planner)
├── ros_dd_teleop/        # 키보드 teleop (rclpy)
├── ros_dd_nav2/          # SimpleCommander 기반 goto_pose 스크립트
└── ros_dd_simulation/    # 메타패키지 (위 4개를 exec_depend로 묶음)
```

| 패키지 | build_type | 핵심 의존성 | 역할 |
|---|---|---|---|
| `ros_dd_description` | ament_cmake | `robot_state_publisher`, `joint_state_publisher`, `xacro` | URDF/xacro·(빈)meshes 설치, RViz용 로봇 모델 소스 |
| `ros_dd_gazebo` | ament_python | `ros_gz_sim`, `ros_gz_bridge`, `ros_dd_description` | world 파일, spawn/bridge 실행 launch, 커스텀 모델(`models/hexa`) |
| `ros_dd_cartographer` | ament_python | `cartographer_ros` | SLAM 지도 작성 |
| `ros_dd_navigation` | ament_python | `nav2_bringup`, `nav2_amcl`, `nav2_planner`, `nav2_controller`, `nav2_bt_navigator`, `slam_toolbox`, `cartographer_ros` | Nav2 자율주행 bringup |
| `ros_dd_teleop` | ament_python | `rclpy`, `geometry_msgs` | 키보드 teleop (`asdw_teleop`) |
| `ros_dd_nav2` | ament_python | — | `goto_pose.py` (SimpleCommander로 목표 지점 전송) |
| `ros_dd_simulation` | ament_cmake (metapackage) | 위 4개 exec_depend | 한 번에 설치하기 위한 묶음 패키지 |

`colcon build` 시 `ros_dd_simulation`에 대해 "메타패키지는 catkin에
buildtool_depend해야 한다"는 WARNING이 뜨는데, 이건 catkin 관례 기준 경고라
ROS2 ament_cmake 메타패키지에서는 흔히 나타나는 무해한 경고다(빌드 자체는
7개 패키지 전부 정상 통과).

## 2. 가상 물리 환경 (`ros_dd_gazebo/worlds/ros_dd.world`)

### gz-sim 시스템 플러그인이 필수

Gazebo Classic은 서버가 기본으로 물리/센서/스폰을 처리했지만, gz-sim은
world에 시스템 플러그인을 **명시적으로** 선언하지 않으면 아예 동작하지 않는다.
`turtlebot3_world.world`(이 PC에서 이미 검증된 예제)를 참고해 아래 5개를 포함:

- `gz-sim-physics-system` — 물리 엔진
- `gz-sim-user-commands-system` — GUI에서의 조작 명령 처리
- `gz-sim-scene-broadcaster-system` — 씬 상태 브로드캐스트
- `gz-sim-sensors-system` (`render_engine: ogre2`) — 렌더 기반 센서(라이다 등)
- `gz-sim-imu-system` — IMU

### 물리 엔진 설정

```
<physics type="ode">
  <real_time_update_rate>1000.0</real_time_update_rate>
  <max_step_size>0.001</max_step_size>          <!-- 1ms 스텝, 1000Hz -->
  <real_time_factor>1</real_time_factor>
  <ode><solver><type>quick</type><iters>150</iters> ...
</physics>
```

`quick` solver + `iters=150`은 turtlebot3 world와 동일 패턴(정확도보다 실시간성
우선). 접촉 관련 `cfm`/`erp`/`contact_surface_layer`도 그대로 이식.

### 지형·조명은 Fuel에서 로드

Gazebo Classic 시절 로컬 모델 DB에 있던 `ground_plane`/`sun`이 gz-sim에는
기본 제공되지 않아, `fuel.gazebosim.org`의 OpenRobotics 모델을 `<include><uri>`로
직접 가져온다(둘 다 `~/.gz/fuel/`에 캐시됨 — 최초 1회만 네트워크 필요).

### 커스텀 장애물 `model://hexa`

`(1.0, 2.0, 0)`에 `models/hexa`를 include. 내용은 실제로는 "육각형"이 아니라
원기둥 9개(3×3 배열, `<!-- Draw Circle -->`)와 `hexagon.dae`/`wall.dae` 메시를
사람 모양(머리·양손·양발·몸통)으로 배치한 장식용 조형물이다. **원래 SDF 안의
`<model name>`이 `ros_symbol`로 돼 있어 `gz model --list`에 `hexa`가 아니라
`ros_symbol`로 뜨는 라벨 불일치가 있었는데, 오늘 `hexa`로 맞춰 수정함.**
`model.config`의 `<description>`도 무관한 보일러플레이트("A simple blue
cube...")였던 걸 실제 내용으로 교체.

`ros_dd_gazebo/setup.py`가 `models/hexa/*.config`, `*.sdf`,
`models/hexa/meshes/*`를 각각 별도 `data_files` 항목으로 설치하는 점 주의
(mesh 폴더를 빠뜨리면 `model://hexa/meshes/*.dae` 참조가 install 트리에서
깨진다).

`models/robot/`은 world/launch 어디에서도 `include`되지 않는 미사용
자산이다(`model.sdf` 내부명은 `ros_dd_world_scaled_1_2_stabilized`). 예전
`model.config`가 ROBOTIS TurtleBot3 Burger 저작자 정보를 그대로 복붙해
사실과 다른 출처를 표기하고 있었는데, 오늘 이것도 정정함.

## 3. 로봇 물리 속성 (`ros_dd_description/urdf/ros_dd.xacro`)

- `base_footprint`(가상 원점) → `base_link`(본체, 0.5×0.3×0.15m box, mass 3.0kg)
- 좌/우 바퀴: `continuous` joint, `mu1=mu2=1.0`(구동 마찰 확보)
- 전/후 캐스터: 구형, `mu1=mu2=0.0`(자유 회전, 조향 저항 없음)
- `laser_link`: `base_link` 앞쪽 상단에 고정, gpu_lidar 센서 부착

### gz-sim 플러그인 (URDF 내 `<gazebo>` 블록)

| 플러그인 | 토픽 | 비고 |
|---|---|---|
| `gz::sim::systems::DiffDrive` | 구독 `cmd_vel`, 발행 `odom`+`/tf` | `wheel_separation`은 바퀴 중심 간 거리로 자동 계산 |
| `gz::sim::systems::JointStatePublisher` | `joint_states` | 좌/우 바퀴 joint만 발행 |
| IMU 센서 | `imu/data` | 가우시안 노이즈(σ=0.005) |
| gpu_lidar 센서 | `scan` | 360샘플, ±π rad, 0.05~8.0m, σ=0.01 |

Gazebo Classic의 `libgazebo_ros_diff_drive` 등 `<ros><remapping>` 블록은
gz-sim 플러그인에 없다 — 대신 전부 gz-transport로 발행되고,
`ros_dd_gazebo/params/ros_dd_bridge.yaml`의 `ros_gz_bridge parameter_bridge`가
ROS2 토픽으로 다리를 놓는다(`clock`/`joint_states`/`odom`/`tf`는
GZ→ROS, `cmd_vel`은 ROS→GZ).

## 4. 실행 흐름 (`ros_dd_gazebo/launch/ros_dd.launch.py`)

```
GZ_SIM_RESOURCE_PATH += ros_dd_gazebo/models   # model://hexa 탐색 경로 등록
  ↓
gz_sim.launch.py (-r -s, 서버)  +  gz_sim.launch.py (-g, GUI)   # 별도 프로세스
  ↓
robot_state_publisher (urdf → /robot_description, /tf)
  ↓
ros_gz_sim create -topic robot_description  (스폰)
  ↓
ros_gz_bridge parameter_bridge --config ros_dd_bridge.yaml
```

## 5. 오늘(2026-08-11) 정리한 것

- `ros_dd_navigation/package.xml`: 마크다운 코드펜스(` ``` `) 잔재 제거 —
  `<package>` 바로 아래 있던 비공백 텍스트라 `package_format3.xsd` 위반이었음
  (colcon build는 통과했지만 `rosdep`/`ament_lint` 계열 XML 검증에서는
  걸릴 수 있는 상태였음).
- `models/hexa/model.sdf`의 내부 `<model name>`을 `ros_symbol` → `hexa`로 정정,
  `model.config` 설명 실제 내용으로 교체.
- `models/robot/model.config`의 허위 저작자 정보(TurtleBot3
  Burger/ROBOTIS) 제거, 실제로는 미사용 자산임을 명시.
- 전체 워크스페이스 재빌드(7패키지) 및 헤드리스 gz-sim 실행으로
  `ground_plane`/`sun`/`hexa` 정상 로드, `gz model --list`에 `hexa`로
  올바르게 표시되는 것까지 확인.

## 참고

- [gz-sim(Harmonic) Migration Guide](https://gazebosim.org/api/sim/8/migrationsdf.html)
- [ros_gz_bridge README](https://github.com/gazebosim/ros_gz)
- `~/DAPIER/turtlebot3_ws/README.md` — 같은 Jazzy/gz-sim 포팅 과정을 먼저
  겪은 참고 워크스페이스 (Chapter 1)
