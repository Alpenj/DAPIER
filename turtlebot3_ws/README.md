# turtlebot3_ws — SLAM·Nav2 (ROS2 Jazzy)

wikidocs의 TurtleBot3 SLAM·Nav2 실습 교재를 이 워크스페이스에 맞게 이식했다.
교재는 **ROS2 Humble + Gazebo Classic** 기준이라 여러 군데를 이번 환경
(**ROS2 Jazzy + gz-sim/Harmonic**)에 맞게 바꿔야 했고, 그 과정에서 겪은 버그와
해결 방법을 아래에 정리한다.

## 환경 차이

| 항목 | 교재(원본) | 이 워크스페이스 |
|---|---|---|
| OS | Ubuntu 22.04 | Ubuntu 24.04 |
| ROS2 | Humble | Jazzy |
| Gazebo | Gazebo Classic | gz-sim (Harmonic) |
| TurtleBot3 패키지 브랜치 | `-b humble` | `-b jazzy` |
| `/cmd_vel` 타입 | `geometry_msgs/Twist` | `geometry_msgs/TwistStamped` |
| `/particlecloud` (AMCL) | `geometry_msgs/PoseArray` | `nav2_msgs/msg/ParticleCloud` (토픽명도 `/particle_cloud`) |

## 디렉터리

```
turtlebot3_ws/
├── src/
│   ├── DynamixelSDK, turtlebot3, turtlebot3_msgs, turtlebot3_simulations   # ROBOTIS-GIT 업스트림 (gitignore, jazzy 브랜치)
│   └── nav2_goals_py/          # 우리가 작성한 ament_python 패키지
├── nav2_scripts/                # 단독 실행 버전 (교재 원형)
│   ├── go_to_pose.py
│   └── follow_waypoints.py
└── scripts/
    └── gentle_explorer.py       # SLAM 매핑용 자동 웨이포인트 탐색 스크립트
```

## 빌드

```bash
mkdir -p ~/DAPIER/turtlebot3_ws/src && cd ~/DAPIER/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/DAPIER/turtlebot3_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`~/.bashrc`에 워크스페이스 소싱과 `export TURTLEBOT3_MODEL=...` 등록.

## Part 1 — SLAM 지도 작성 (burger)

```bash
# 터미널 1
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 터미널 2
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# 터미널 3 — 키보드 teleop 대신 자동 주행
python3 scripts/gentle_explorer.py 320

# 터미널 4
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## Part 2 — Nav2 자율주행 (waffle_pi)

```bash
# 터미널 1
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 터미널 2 — Nav2 (AMCL 내장, Cartographer는 따로 안 띄움)
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

launch 직후 몇 초 안에 초기 위치를 쏴줘야 자동 bringup이 성공한다 (아래
트러블슈팅 참고):

```bash
for i in $(seq 1 15); do
  sub=$(ros2 topic info /initialpose 2>/dev/null | grep -c "Subscription count: [1-9]")
  [ "$sub" = "1" ] && ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" && break
  sleep 1
done
```

### 목표 지점 전송 3가지 방법

| 방법 | 명령 |
|---|---|
| CLI topic | `ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "..."` |
| CLI action | `ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose "..."` |
| Python (SimpleCommander) | `python3 nav2_scripts/go_to_pose.py` / `ros2 run nav2_goals_py go_to_pose` |

셋 다 실제로 목표까지 이동해서 `SUCCEEDED`를 받는 것까지 확인했다.

## 트러블슈팅 (실제로 겪은 것들)

**Cartographer 중복 실행 → 지도가 계속 널뛴다.**
같은 launch를 두 터미널에서 실행하면 `cartographer_node`가 두 개 뜨고, 둘 다
`map→odom` TF를 발행해서 지도가 흔들린다. `ros2 node list`는 중복을 바로 못
잡을 수 있으니 `ps aux | grep cartographer_node`로 실제 프로세스 개수를 확인한다.

**급회전/후진 위주 자동 주행이 지도를 오염시킨다 (고스팅).**
막힐 때 후진+제자리 급회전으로 탈출하는 로직을 특정 코너에서 반복하면
오도메트리가 어긋나고, 스캔 매칭으로 완전히 지워지지 않아 지도에 벽이 겹쳐
번진 흔적이 남는다. `gentle_explorer.py`는 속도 상한을 낮추고, 명령값에 rate
limiter를 걸고, 막히면 후진 없이 저속 제자리 회전만 하도록 만들어 이 문제를
피한다.

**정체 감지 로직의 무한 루프.** "최근 9초간 이동거리 0.15m 미만이면 정체"
조건을, 최소 샘플 개수(6개)로만 체크하면 감속 램프 때문에 6개가 실제로는
0.5~0.6초치밖에 안 돼서 정체→회전→정체가 무한 반복된다. 샘플 개수가 아니라
"가장 오래된 샘플이 몇 초 전인지"로 조건을 바꿔야 한다.

**`turtlebot3_navigation2`는 AMCL을 항상 포함한다.** launch 파일이
`nav2_bringup`의 `bringup_launch.py`를 그대로 쓰기 때문에 AMCL을 끄는
argument가 없다. Cartographer를 Nav2 단계에서도 같이 띄우면 `map→odom`을
AMCL과 Cartographer가 동시에 발행해서 SLAM 때와 같은 TF 경합이 재발한다.
Cartographer는 지도 작성 전용으로만 쓴다.

**Nav2 자동 bringup은 초기 위치 타이밍에 민감하다.** `global_costmap`/
`local_costmap`이 `map` 프레임을 약 10초 안에 못 받으면 bringup을 통째로
포기한다. `/initialpose`는 AMCL이 받아야 `map` 프레임이 발행되므로, launch
직후 곧바로 쏴줘야 한다. 타이밍을 놓쳐 실패하면 개별 노드를 하나씩
`ros2 lifecycle set /<node> activate`로 복구할 수도 있지만, 상태가 꼬이기
쉬워서 Nav2를 통째로 재시작하는 편이 더 빠르다.

**지도를 바꾸면 초기 위치 좌표도 같이 바뀐다.** Cartographer의 `map` 프레임
원점은 world 절대좌표가 아니라 "그 SLAM 세션이 시작된 지점"이다. 지도를
새로 만들면 원점도 달라지므로, 이전 지도에서 쓰던 `/initialpose` 좌표를
그대로 재사용하면 안 된다. `map.yaml`의 `origin`과 이미지 크기로 실제 범위를
확인하고 좌표를 다시 잡는다.

**`nav2_simple_commander`의 `lifecycleShutdown()`은 되돌릴 수 없다.**
스크립트 끝에서 호출하면 Nav2 lifecycle 노드들이 `finalized`(종단 상태)가
되어 `lifecycleStartup()`만 다시 불러서는 안 살아난다. 스크립트를 연달아
테스트하려면 그때마다 Nav2 프로세스를 통째로 재시작해야 한다.

**리다이렉트된 로그의 순서를 믿으면 안 된다.** `nohup ... > log 2>&1 &`로
실행하면 Python 자체 `print()`는 블록 버퍼링되어 늦게 flush되지만 `rclpy`
로거는 그보다 먼저 파일에 쓰인다. 로그에 찍힌 순서가 실제 시간 순서와 다르게
보일 수 있으므로, 타이밍이 의심스러우면 `PYTHONUNBUFFERED=1`을 걸고 다시
확인한다.

**`tf_transformations`가 이 환경의 numpy 2.0과 호환되지 않는다.**
(`np.maximum_sctype` 제거됨) 교재 스크립트에서 실제로는 안 쓰는 죽은
import였으므로 그냥 제거했다.

## RViz 디스플레이 항목 — 교재 부록과 실제 시스템의 차이

교재 부록 1·2는 Cartographer/Nav2 실행 중 RViz Displays 패널의 각 항목을
설명한다. TF 체인 구조와 AMCL/Inflation Layer 개념 설명은 정확하지만,
구체적인 노드/토픽 이름은 ROS1/Humble 기준과 섞여 있어 실제 시스템에서
확인한 값과 다른 부분이 있다.

| 교재 | 실제(ROS2 Jazzy + Nav2) |
|---|---|
| 라이다 프레임 `laser_link` | `base_scan` |
| `move_base` 노드 | 없음 — `bt_navigator` + `controller_server` + `planner_server` |
| `/move_base/NavfnROS/plan` | `/plan` |
| `/cmd_vel` → `Twist` | `TwistStamped` |
| `/particlecloud` → `PoseArray` | `/particle_cloud` → `nav2_msgs/msg/ParticleCloud` |

## 참고

- [TurtleBot3 Quick Start](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- [Nav2 Getting Started](https://docs.nav2.org/getting_started/index.html)
- [Nav2 Concepts](https://docs.nav2.org/concepts/)
