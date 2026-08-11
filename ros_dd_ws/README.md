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
- `models/hexa/meshes/{wall,hexagon}.dae`가 실제로 raw 정점 좌표
  수백 단위(wall.dae는 X 900, Y 1039 span)에 `<unit name="inch"
  meter="0.0254"/>`가 선언돼 있어서, `model.sdf`의 `<scale>0.25~0.8>`을
  곱해도 최종 크기가 5~10m대 — 6.5m×6m밖에 안 되는 이 테스트 월드
  전체를 뒤덮는 크기다. 로봇이 실제로는 거의 못 움직이고(스폰 근처에
  갇힘), 제자리에서 계속 회전하고, Nav2 플래너가 로봇 자기 위치조차
  거의 lethal(cost 99)로 보던 증상이 전부 이걸로 설명된다.
  **한때 `meter="0.01"`(cm)로 바꿔서 "사람 크기 조형물"처럼 줄였었는데,
  이건 검증 안 된 추측이었고 실제 교재 화면과 비교해보니 명백히
  다른 모양이라 원본(inch)으로 되돌림.** 이 `hexa` 모델 자체가 애초에
  교재의 SLAM/Nav2 실습(1.6.1절, `turtlebot3_gazebo
  turtlebot3_world.launch.py`)과는 무관한 별개 커스텀 오브젝트로
  보인다 — 교재의 해당 실습은 `turtlebot3_ws`(Chapter 1)에서 이미
  올바르게 재현했음. `hexa`가 정확히 어떤 모양이어야 하는지는
  미해결.

## 6. Nav2 자율주행 (Chapter 6)

> **주의**: 아래 SLAM 지도(`maps/ros_dd_map.*`)와 성공한 `NavigateToPose`
> 검증은 `hexa` 오브젝트를 임시로 cm 스케일(작게)로 고친 상태에서
> 진행한 것이다. `hexa`를 원본(inch, 거대한 크기)으로 되돌렸으므로
> 이 지도는 더 이상 실제 world 상태와 안 맞고, Nav2 파라미터 수정
> 내용(플러그인 네이밍·`enable_stamped_cmd_vel` 등)은 Jazzy 마이그레이션
> 자체는 여전히 유효하지만 재검증이 필요하다.

`ros_dd_navigation/config/nav2_params.yaml`은 훨씬 예전 Nav2(Foxy/Galactic
시절 turtlebot3 예제) 스타일로 작성돼 있어서, Jazzy의 `nav2_bringup`으로
그대로 돌리면 단계마다 다르게 깨진다. 헤드리스로 SLAM 지도 저장 →
`ros2 launch ros_dd_navigation navigation.launch.py`로 bringup → `ros2 action
send_goal /navigate_to_pose ...`까지 실제로 성공(`SUCCEEDED`, ground-truth
위치도 목표 근처 확인)시키기까지 만난 버그들:

| 증상 | 원인 | 수정 |
|---|---|---|
| `planner_server` 활성화 실패, "class ... does not exist" | `plugin: "nav2_navfn_planner/NavfnPlanner"` — `/` 구분자는 옛날 pluginlib 네이밍 | `"nav2_navfn_planner::NavfnPlanner"` (`::`) |
| `recoveries_server`(구 이름) 섹션이 통째로 죽은 설정 | Jazzy는 노드 이름 자체가 `behavior_server`, 플러그인도 `nav2_recoveries/Spin` → `nav2_behaviors::Spin`로 변경됨 | 섹션명·플러그인명 전부 교체 |
| `bt_navigator` 설정 중 세그폴트, 컨테이너 전체 다운 | 옛 `plugin_lib_names`(30개 BT 노드 나열)를 그대로 두면 Jazzy가 이미 자동 등록한 것과 중복 등록 시도 → `ID [ComputePathToPose] already registered` | `plugin_lib_names`/`default_bt_xml_filename` 삭제, `navigators`+`navigate_to_pose`/`navigate_through_poses` 플러그인 선언으로 교체 |
| `collision_monitor` 활성화 실패: `observation_sources` not initialized | Jazzy에서 새로 생긴 필수 lifecycle 노드인데 원본 파일엔 섹션 자체가 없었음 | `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml` 기본값 그대로 추가 |
| `docking_server` 활성화 실패, 이어서 `dock_plugins: []`로 하니 launch 자체가 `Expected 'value' to be one of [...] but got '()' of type 'tuple'`로 죽음 | 이 노드도 Jazzy 신규. 도킹 스테이션이 없어 빈 리스트를 주고 싶었지만 ROS2 파라미터 로더가 빈 배열의 타입을 못 정함 | 실제 도킹 하드웨어가 없어도 `simple_charging_dock` 플러그인을 그대로 로드(호출은 안 됨)하도록 기본값 유지 |
| `route_server`가 "Transform data too old" 스팸(map→odom, 시각 1786411995 vs 1618) | 이 노드도 Jazzy 신규, `use_sim_time` 미설정 → wall-clock(유닉스 epoch)으로 TF 조회 | `use_sim_time: true` 추가 (원래 이 워크스페이스는 route-graph 기능 자체를 안 씀) |
| 파일 전역에 흩어진 `use_sim_time: False` 17곳 | 실물 로봇 기준으로 작성된 원본이라 시뮬레이션 파라미터가 아예 없었음 | 전부 `True`로 일괄 치환 |
| Nav2가 목표를 받아도 로봇이 안 움직임(에러도 없음) | Jazzy Nav2는 `enable_stamped_cmd_vel` 기본값이 `False` → `controller_server`/`velocity_smoother`/`collision_monitor`/`docking_server`/`behavior_server` 전부 plain `Twist`를 발행하는데, 우리 `ros_gz_bridge`/DiffDrive 플러그인은 `TwistStamped`만 구독 — 타입이 달라 아무도 못 받음. `behavior_server`만 빼먹으면 `cmd_vel_nav` 토픽에 타입이 다른 퍼블리셔 2개가 붙으려다 `create_publisher() ... incompatible type`로 bringup이 죽는다 | 위 5개 노드 전부에 `enable_stamped_cmd_vel: true` |
| Nav2 자동 bringup이 `Failed to activate ... before timeout`으로 통째로 실패 | `turtlebot3_ws`에서도 겪은 것과 같은 초기 위치 타이밍 이슈 — `/initialpose`를 늦게 쏘면 `global_costmap`/`local_costmap`이 이미 포기한 뒤임 | launch 시작 후 5초 안에 `/initialpose` 발행. 놓치면 개별 노드 복구보다 Nav2 재시작이 빠름 |
| `/initialpose` 발행해도 AMCL이 계속 "cannot publish a pose" | 쿼터니언이 완벽히 정규화 안 됨(예: `z=0.93, w=0.36`, norm≈0.9995) — AMCL이 "malformed"로 거부 | identity(`w=1`) 또는 `math.sin/cos(yaw/2)`로 정확히 계산한 쿼터니언만 사용 |
| AMCL이 수렴 안 하고 `/amcl_pose`가 로봇이 정지해 있는데도 몇 미터씩 계속 튐 | **RViz2 인스턴스가 5개나 동시에 떠 있었음** — 이 세션 내내 재시작할 때마다 이전 RViz가 안 죽고 쌓여서(각 ~25% CPU) 시스템이 과부하 상태였고, AMCL이 스캔을 실시간으로 못 따라감 | `pkill`이 놓친 프로세스를 PID로 직접 `kill -9`, 살아있는 RViz를 1개로 정리 → 부하 낮추자 즉시 `/amcl_pose`가 실제 위치 근처(오차 ~0.02~0.04m)로 안정 수렴 |

**핵심 교훈**: 이 세션 내내 `pkill -f "패턴"`이 조용히 실패해서(이유
불명 — 아마 세션 하나에 gz-sim 서버 2개, robot_state_publisher 2개,
bridge 2개, rviz2 5개까지 쌓였었다) `/odom` 중복 발행으로 cartographer가
크래시하고, CPU 과부하로 AMCL이 발산하는 등 순수 설정 문제가 아닌
증상들이 한동안 진짜 버그처럼 보였다. Nav2/시뮬레이션을 재시작할 땐
`pkill` 실행 후 반드시 `ps aux`로 실제로 죽었는지 재확인하고, 의심되면
PID를 직접 뽑아 `kill -9`할 것.

## 7. TF Frame / REP 105 (Chapter 3)

`ros2 run tf2_tools view_frames`로 실제 구동 중(gz-sim + robot_state_publisher
+ cartographer)인 TF 트리를 떠보면:

```
map --(cartographer, ~201Hz)--> odom --(DiffDrive plugin, ~50Hz)--> base_footprint
  --(고정, robot_state_publisher)--> base_link
      ├─(고정)─→ front_caster_link
      ├─(고정)─→ rear_caster_link
      ├─(고정)─→ laser_link
      ├─(~20Hz)─→ left_wheel_link
      └─(~20Hz)─→ right_wheel_link
```

[REP 105](https://www.ros.org/reps/rep-0105.html)가 정의하는 표준 좌표계
체계가 그대로 나타난다.

### `map` → `odom` → `base_link`: 왜 두 단계로 나뉘나

REP 105는 로봇 위치 추정을 정확히 두 가지 성격으로 나눈다.

| 구간 | 발행 주체(이 워크스페이스) | 성격 |
|---|---|---|
| `map` → `odom` | `cartographer_node` (SLAM) | **불연속적**(discontinuous) 보정 가능. 루프 클로저·재정렬로 순간적으로 점프할 수 있다. "지도 기준 절대 위치"의 최신 추정치. |
| `odom` → `base_footprint` | gz-sim `DiffDrive` 플러그인(→`ros_gz_bridge`) | **연속적**(continuous). 절대 점프하지 않지만, 바퀴 적분(dead reckoning) 특성상 시간이 지나면 드리프트가 누적된다. |
| `base_footprint` → `base_link` → 나머지 | `robot_state_publisher`(URDF 고정 관절) | 로봇 강체에 고정. 시뮬레이션/실물 관계없이 절대 안 바뀜(바퀴 조인트만 예외 — 회전각이 계속 갱신됨). |

이 둘을 분리해두는 이유는 **컨슈머(costmap, 컨트롤러)가 필요에 따라 골라
쓸 수 있게** 하기 위함이다 — 로컬 장애물 회피처럼 "지금 이 순간의 상대
움직임"만 필요하면 튀지 않는 `odom` 프레임을 쓰고(`local_costmap`의
`global_frame: odom`이 바로 이 이유), 전역 경로 계획처럼 "지도 위 절대
위치"가 필요하면 `map` 프레임을 쓴다(`global_costmap`의
`global_frame: map`). 이번 세션에서 AMCL이 발산했을 때(6절 참고)
`/amcl_pose`(=`map`→`base_link`의 원천)만 몇 미터씩 튀고 `odom`→`base_link`
자체는 멀쩡했던 것도 이 분리 덕분에 진단이 가능했다.

### `base_footprint` vs `base_link`

REP 105는 `base_footprint`를 "로봇을 지면에 투영한 2D 프레임"으로 정의한다
(z=0, roll/pitch 없음). 이 워크스페이스에서는 `base_joint`(고정, xacro
`base_footprint`→`base_link`, `z=wheel_radius`만큼 오프셋)로 구현돼 있다.
2D 평면 내비게이션(Nav2 costmap의 `robot_base_frame`)은 `base_link`를
직접 쓰기도 하고 `base_footprint`를 쓰기도 하는데, 이 워크스페이스의
`nav2_params.yaml`은 costmap엔 `base_link`를, AMCL엔 `base_footprint`를
쓰도록 섞여 있다 — 로봇이 z축으로 기울어지지 않는(캐스터 바퀴형) 형태라
실질적 차이는 거의 없지만, 엄밀한 REP 105 준수를 원하면 전부
`base_footprint`로 통일하는 게 맞다.

### 정적(static) vs 동적(dynamic) 브로드캐스터

`view_frames` 출력에서 `rate: 10000.000`, `buffer_length: 0.000`으로 나오는
간선(`base_footprint`→`base_link`, `base_link`→캐스터/`laser_link`)은
`/tf_static`으로 **한 번만** 발행되는 고정 변환이다. 반대로 `odom`(~50Hz,
buffer 10s)·바퀴 조인트(~20Hz)·`map`→`odom`(~201Hz)은 `/tf`로 주기적으로
갱신되는 동적 변환이다. `robot_state_publisher`는 URDF의 `fixed` 조인트는
자동으로 `/tf_static`에, `continuous`/`revolute` 조인트는 `joint_states`
갱신마다 `/tf`에 발행하도록 구분해서 처리한다 — 이 구분을 사람이 직접
관리할 필요는 없고, URDF의 조인트 타입 선언만 정확하면 된다.

### 참고: 하루 전 캡처와 달라진 점

`frames_2026-08-10_16.35.3{2,8}.pdf`(어제 URDF만 띄운 상태로 캡처)는
`map`/`odom`도 없고 바퀴 조인트도 안 잡혀 있는데다, 캐스터 링크 이름이
`caster_link_front`/`caster_link_rear`로 지금(`front_caster_link`/
`rear_caster_link`)과 순서가 반대다 — 그 사이에 xacro를 손본 흔적.
오늘 캡처(`frames_2026-08-11_11.08.07.pdf`)가 현재 상태를 반영한 최신본.

## 8. 좌표계 관점에서 보는 Cartographer 연산 원리 (Chapter 4)

Cartographer는 **Local SLAM(front-end)**과 **Global SLAM(back-end)**
두 층으로 나뉜다. 실제로 로봇을 돌려서(전진 8s → 회전 4s → 전진 8s)
`/submap_list`와 `map`→`odom` TF를 관찰한 결과:

```
submap 0: pose (0, 0, 0)                    -- 궤적 시작점, map 프레임 원점
submap 1: pose (0.44, 0.60), yaw -12.9°
submap 2: pose (-1.53, -0.38), yaw -12.2°
submap 3: pose (-1.86, -0.57), yaw -12.3°

map -> odom: translation (3.221, 0.120, -0.235), yaw 119.8°
```

### Local SLAM (front-end) — submap 단위 스캔 매칭

`TRAJECTORY_BUILDER_2D`가 매 스캔을 **현재 활성 submap**에 정합
(scan matching)한다. `ros_dd_cartographer.lua`의
`use_online_correlative_scan_matching: true`가 이 단계 — 현재 위치
추정치 근처를 격자 탐색(correlative)하며 스캔과 submap이 가장 잘
겹치는 위치를 찾는다. 이동 거리가 쌓이면 현재 submap을 "완료"
처리하고 새 submap을 연다 — 위 데이터에서 submap 0→1→2→3으로
번호가 늘어난 게 이 과정이다. **이 단계의 좌표계는 `odom` 프레임
기준 상대 운동**(로컬)이며, `tracking_frame: laser_link` 설정대로
라이다 프레임의 움직임을 추적한다.

### Global SLAM (back-end) — pose graph 최적화

여러 submap이 쌓이면 `POSE_GRAPH`가 백그라운드에서 submap들 사이의
**제약조건(constraint)**을 계산한다 — 인접 submap과는 항상, 멀리
떨어진 submap끼리도 스캔이 겹칠 만큼 가까워지면(루프 클로저 후보)
추가로 계산한다. 이 제약조건들을 최소자승 최적화로 한꺼번에 풀어서
전체 submap 배치를 보정하는 게 pose graph 최적화다 — 위 데이터에서
각 submap의 `pose`가 전부 미세한 회전(-12°대)을 공유하는 게 이
전역 보정의 흔적이다. **이 단계의 좌표계는 `map` 프레임**(전역,
불연속 보정 가능) — 3절에서 정리한 REP 105의 `map`→`odom`이 바로
이 back-end 결과물이다.

### `map` → `odom`이 곧 "front-end 대비 back-end의 순수 보정량"

`odom`은 DiffDrive 플러그인의 순수 바퀴 적분값(드리프트 있음,
연속적)이고 `map`은 SLAM이 보정한 절대 추정치이므로, 그 사이의
`map`→`odom` 변환 자체가 "지금까지 얼마나 보정했는가"를 그대로
보여준다. 위 캡처에서 이 변환이 `(3.221, 0.120)` 이동 +
`119.8°` 회전인 것은 — 이 세션에서 로봇이 8초 직진 + 90°대
회전 + 8초 직진을 했으니 odom이 그 정도 각도로 드리프트했고
Cartographer가 그걸 보정하고 있다는 뜻이다(값 자체보다 **0이
아니라는 것, 그리고 회전 방향/크기가 실제 주행 궤적과 방향이 맞다는
것**이 이 단계에서 확인할 포인트).

### 실행 중 GUI로 확인하는 법

```bash
# submap 목록(위치/버전) 확인
ros2 topic echo /submap_list --once

# map->odom 보정량 직접 확인
ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true

# RViz의 ros_dd_cartographer.rviz 설정에는 Map, LaserScan, TF가
# 이미 포함돼 있어 submap 경계가 늘어나는 것과 라이다 스캔이
# 기존 지도에 정렬되는 것을 실시간으로 볼 수 있다.
```

## 9. Cartographer LUA 설정 심층 분석 (Chapter 5)

### 실행에 관여하는 노드는 2개, 역할이 다르다

| 노드 | 실행 파일 | 역할 |
|---|---|---|
| `cartographer_node` | `cartographer_ros/cartographer_node` | **실제 SLAM 연산**(8절의 local/global SLAM). `scan`/`odom`/`imu` 구독, `submap_list`·`map`→`odom` TF·trajectory 발행. `-configuration_directory`/`-configuration_basename` 인자로 이 LUA 파일을 읽는다. |
| `cartographer_occupancy_grid_node` | `cartographer_ros/cartographer_occupancy_grid_node` | SLAM 연산에는 관여하지 않는 **별도 변환기**. `cartographer_node`가 내부적으로 들고 있는 submap 데이터를 구독해 익숙한 `nav_msgs/OccupancyGrid`(`/map` 토픽)로 래스터화만 한다. `map_saver_cli`나 Nav2의 `map_server`가 소비하는 포맷이 바로 이거다. |

`ros_dd_cartograph.launch.py`가 이 둘을 같이 띄우는데, `occupancy_grid_node`
쪽 인자(`-resolution`, `-publish_period_sec`)는 **래스터 해상도/갱신
주기**일 뿐 SLAM 자체의 정밀도와는 무관하다 — SLAM 내부 submap
해상도는 `TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution`
(이 파일에서는 오버라이드 안 해서 기본값 0.05 그대로 사용)이 따로
결정한다.

### `options` 블록 — LUA가 SLAM에게 "이 로봇/환경은 이렇다"고 알려주는 부분

| 파라미터 | 값 | 의미 |
|---|---|---|
| `tracking_frame` | `laser_link` | Cartographer가 **추적하는 대상 프레임**. 라이다 프레임으로 잡으면 라이다 장착 오프셋(URDF의 `laser_joint`)을 자동으로 고려해 계산한다. |
| `published_frame` | `odom` | Cartographer가 TF로 **직접 발행하는 하위 프레임**. `base_link`가 아니라 `odom`으로 잡은 이유가 바로 아래 `provide_odom_frame: false`와 짝을 이룬다. |
| `provide_odom_frame` | `false` | gz-sim의 DiffDrive 플러그인이 이미 `odom`→`base_footprint`를 발행하고 있으니, Cartographer가 또 그 역할을 만들지 않겠다는 뜻. 이 설정 덕분에 최종적으로 `map`→`odom`(Cartographer)과 `odom`→`base_footprint`(DiffDrive)가 **겹치지 않고 이어진다** — 8절에서 관찰한 REP 105 3단 구조가 이 두 옵션의 조합으로 만들어진다. |
| `use_odometry` | `true` | `odom` 토픽(오도메트리)을 스캔 매칭의 초기 추정치로 같이 쓴다 — 순수 스캔 매칭보다 빠르고 안정적으로 수렴한다. |
| `use_imu_data`(TRAJECTORY_BUILDER_2D) | `false` | "터틀봇 원본과 동일" 주석대로 IMU는 안 씀 — 이 로봇의 IMU 노이즈(σ=0.005, `ros_dd.xacro`)가 오히려 방해될 수 있다는 판단으로 보인다. |
| `min_range` / `max_range` | `0.1` / `8.0` | 라이다 센서의 실제 스펙(`ros_dd.xacro`의 gpu_lidar range 0.05~8.0)에 맞춰서 범위 밖 데이터를 걸러낸다. `max_range`가 너무 크면(예: Nav2 AMCL 6절에서 겪은 `laser_max_range: 100` 실수처럼) 범위 밖 노이즈까지 스캔 매칭에 섞여 들어간다. |
| `use_online_correlative_scan_matching` | `true` | 8절의 front-end 스캔 매칭 방식 지정 — 현재 추정 위치 근처를 격자 탐색하는 방식. 계산량은 늘지만 초기 추정이 부정확해도 잘 버틴다(오도메트리 드리프트가 있는 저가 로봇에 유리). |
| `motion_filter.max_angle_radians` | `math.rad(0.1)`(약 5.7°) | 로봇이 이 각도 이상 회전해야 새 스캔을 "의미 있는 이동"으로 인정하고 처리한다 — 정지 중 노이즈로 인한 불필요한 재계산을 막는 필터. |
| `submap_publish_period_sec` / `pose_publish_period_sec` / `trajectory_publish_period_sec` | `0.3` / `0.005` / `0.03` | 각각 submap, pose, trajectory를 얼마나 자주 발행하는지 — pose가 가장 빠른(200Hz) 이유는 `map`→`odom` TF가 실시간성이 가장 중요하기 때문. |

### 교재 vs 실측 — `optimize_every_n_nodes`

Notion 학습 기록(2026-08-10)에서 이미 지적한 부분과 동일하게 확인됨:
이 LUA 파일은 `POSE_GRAPH.optimize_every_n_nodes = 0`을 **주석 처리**해
뒀다(60번째 줄). 즉 이 값을 오버라이드하지 않고 `pose_graph.lua`의
기본값을 그대로 쓴다는 뜻인데, 이 PC에 설치된 Cartographer(Jazzy용
바이너리)의 기본값을 직접 열어보면:

```
$ grep optimize_every_n_nodes /opt/ros/jazzy/share/cartographer/configuration_files/pose_graph.lua
optimize_every_n_nodes = 90,
```

**노드 90개마다 한 번씩 전역 최적화(8절의 back-end)를 돌린다**는
뜻 — 매 스캔마다 최적화하면 느리니 일정량 쌓아서 배치로 처리하는
설계. 주석 처리된 `= 0`은 "매번 최적화"를 의도했던 흔적일 수도,
단순히 실험 중 꺼둔 것일 수도 있다 — 문서·주석의 숫자를 그대로
믿지 말고 실제 설치된 바이너리의 기본값을 열어 확인하는 습관이
여기서도 유효했다.

## 10. 데이터 흐름 파이프라인 및 디버깅 (Chapter 7)

### 전체 데이터 흐름 (센서 → 액추에이터, 실측 토픽 그래프)

```
[gz-sim 물리엔진]
  gpu_lidar 센서 ──gz-transport──┐
  IMU 센서 ────────gz-transport──┤
  DiffDrive(조인트→odom) ─gz-tr──┤
                                  │  ros_gz_bridge (params/ros_dd_bridge.yaml)
                                  ▼
        /scan  /imu/data  /odom  /tf(odom→base_footprint)  /clock
                                  │
                                  ├──▶ cartographer_node ──▶ /submap_list, /tf(map→odom),
                                  │                          /trajectory_node_list,
                                  │                          /scan_matched_points2, /constraint_list
                                  │        │
                                  │        ▼
                                  │   cartographer_occupancy_grid_node ──▶ /map, /map_updates
                                  │
                                  ├──▶ (map 저장 후) Nav2: map_server → amcl ──▶ /tf(map→odom)
                                  │                  → planner_server/controller_server
                                  │                  → behavior_server/bt_navigator
                                  ▼
                              /cmd_vel (TwistStamped) ──ros_gz_bridge──▶ gz-transport
                                  │
                                  ▼
                          [gz-sim: DiffDrive 플러그인이 바퀴 조인트 속도로 변환]
```

`robot_state_publisher`는 이 흐름과 별도로 URDF 고정/회전 관절을
`/tf`, `/tf_static`, `/robot_description`으로 항상 발행 — 3절에서 다룬
`base_footprint`→`base_link`→센서 체인이 여기서 나온다.

### 디버깅 순서 — 오늘 실제로 쓴 방법

증상이 나오면 launch를 계속 재실행하기보다 아래 순서로 범위를 좁히는
쪽이 훨씬 빠르다(Notion 학습 기록 8절의 체크리스트를 이 워크스페이스
실전에서 그대로 검증함):

1. **노드가 살아 있는가** — `ps aux | grep <프로세스>`. 오늘 가장 많이
   당한 함정: `pkill -f`가 조용히 실패해서 gz-sim 서버 2개·bridge
   2개·rviz2 5개가 겹쳐 돈 적이 있다. `pkill` 뒤엔 반드시 `ps aux`로
   실제 사망을 재확인, 의심되면 PID 직접 `kill -9`.
2. **필요한 topic이 존재하는가** — `ros2 topic list`. 위 파이프라인
   표와 대조해서 빠진 게 있으면 그 앞 단계(브리지/노드)부터 의심.
3. **message type이 맞는가** — `ros2 topic info <topic> -v`.
   6절의 `/cmd_vel` 사고가 정확히 이거였다: `enable_stamped_cmd_vel`
   불일치로 발행측(Twist)과 구독측(TwistStamped) 타입이 달라
   "Publisher count: 2, Subscription count: 1"인데도 서로 안
   이어지고 있었다 — topic이 "존재"하는 것과 "실제로 연결돼 있는
   것"은 다르다.
4. **topic rate가 0이 아닌가** — `ros2 topic hz <topic>`.
5. **`header.frame_id`가 예상과 같은가** — `ros2 topic echo --once`.
6. **`use_sim_time`이 전 노드에서 일치하는가** — 6절의
   `route_server`가 이걸 놓쳐서 "Transform data too old"를 스팸했다
   (wall-clock epoch 시각 대 sim-time 시각을 비교하다 실패).
7. **TF tree가 연결되는가** — `ros2 run tf2_tools view_frames`,
   `ros2 run tf2_ros tf2_echo <target> <source> --ros-args -p
   use_sim_time:=true`(sim 사용 중이면 `use_sim_time` 빼먹으면
   엉뚱하게 "frame does not exist"로 보인다 — 실제로 겪음).
8. **같은 TF edge의 publisher가 둘 이상인가** —
   `ros2 topic info /odom -v`로 publisher 목록 확인. 오늘 세션에서
   좀비 bridge 2개가 `/odom`을 중복 발행해 cartographer가
   `PoseExtrapolator::AddOdometryData` CHECK 실패로 실제로 죽었다.
9. **파라미터 파일 경로·초기화 타이밍이 맞는가** — 6절의 Nav2 자동
   bringup처럼 launch 후 몇 초 안에 `/initialpose`를 못 쏘면
   `global_costmap`/`local_costmap`이 통째로 활성화를 포기한다.
10. **lifecycle 노드가 active인가** — `ros2 lifecycle get <node>`.

### 증상 → 원인 빠른 참조표 (전부 이 워크스페이스에서 실제로 겪은 것)

| 증상 | 실제 원인 |
|---|---|
| 로봇이 명령대로 안 움직이는데 에러도 없음 | `/cmd_vel` 발행측·구독측 메시지 타입 불일치(`Twist` vs `TwistStamped`) |
| 로봇이 명령 없이도 제자리에서 계속 회전 | (a) 마지막 비영(非零) 각속도 명령이 아직 도달 안 함, 또는 (b) 물리적으로 거대한 충돌체에 끼어있음(hexa 메시 스케일 이슈) |
| SLAM/Nav2 노드가 launch 직후 죽음 | 좀비 프로세스가 같은 토픽을 중복 발행(예: `/odom` 이중 publisher → cartographer CHECK 실패) |
| Nav2 bringup 전체가 "Failed to activate" | 초기 위치 타이밍 — costmap의 내부 타임아웃(~10초)을 놓침 |
| `/initialpose` 보내도 AMCL이 계속 거부 | 쿼터니언이 완벽히 정규화 안 됨("malformed"로 거부) |
| AMCL이 정지 상태에서도 위치가 몇 미터씩 튐 | CPU 과부하(주로 GUI/RViz 중복 실행)로 스캔 처리가 실시간을 못 따라감 |
| planner가 시작/목표 지점을 lethal로 봄 | 근처 충돌체 크기가 실제보다 훨씬 커서 costmap 전체가 오염됨 |
| 특정 노드만 계속 "Transform too old" | 그 노드에 `use_sim_time`이 빠져서 wall-clock으로 TF를 조회 중 |

## 참고

- [gz-sim(Harmonic) Migration Guide](https://gazebosim.org/api/sim/8/migrationsdf.html)
- [ros_gz_bridge README](https://github.com/gazebosim/ros_gz)
- [REP 105 -- Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- `~/DAPIER/turtlebot3_ws/README.md` — 같은 Jazzy/gz-sim 포팅 과정을 먼저
  겪은 참고 워크스페이스 (Chapter 1)
