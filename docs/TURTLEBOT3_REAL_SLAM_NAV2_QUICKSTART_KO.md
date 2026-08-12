# TurtleBot3 Burger 실물 SLAM → 지도 저장 → Nav2 따라 하기

`record_id: DAPIER-2026-08-12-turtlebot3-real-slam-nav2`

이 문서는 이미 구성한 ASUS 노트북과 Jetson Nano TurtleBot3 Burger에서
터미널 명령을 그대로 따라 하며 실제 지도를 만들고 저장한 뒤, 같은 지도로
자율주행하는 순서다. 긴 설치·오류 조사 기록은
[`JETSON_NANO_TURTLEBOT3_RUNBOOK_KO.md`](JETSON_NANO_TURTLEBOT3_RUNBOOK_KO.md)에
따로 남긴다.

## 현재 실제 완료 상태

- [x] 노트북 ↔ Jetson Nano 실물 ROS 2 통신
- [x] OpenCR, 배터리, 좌우 모터, odom, TF, LDS-01 라이다 검증
- [x] TELEOP 실제 주행과 Cartographer 실물 지도 생성
- [x] 최종 지도 `~/maps/dapier_real_20260812.{yaml,pgm}` 저장·검증
- [x] 저장 지도에서 AMCL 초기 위치 수렴과 라이다-지도 정렬 확인
- [x] Nav2 6개 웨이포인트 실제 주행 후 `SUCCEEDED` 확인

최종 지도는 `887×206 cells`, `0.05m/cell`이며 장애물 3,225셀과 자유 공간
44,367셀이 들어 있다. 충전·냉각 후 2026-08-12에 AMCL 정렬(라이다 끝점 중
82.3%가 10cm 이내 지도 벽과 일치), 6개 웨이포인트의 19.52m 계획을
실행했다. 결과는 `status=4`, `error_code=0`, `missed=[]`였고 odometry 누적
이동거리는 21.69m였다. 자율주행 최고 선속도 0.18m/s와 각속도 1.0rad/s가
Nav2 → velocity smoother → collision monitor → 실물 `/cmd_vel` 전 구간에
실제로 전달됐다.

## 절대 지킬 안전 규칙

1. 로봇 주변 1m를 비우고 손이 닿는 거리에서 시작한다.
2. 빨간 모터 LED, OpenCR USER3 빨간 LED, 반복 비프음이 나타나면 즉시
   `s` 또는 Space를 누르고 전원을 끈다.
3. 모터가 뜨거우면 최소 20분 이상 식힌다.
4. 전원이 들어온 상태에서 Dynamixel 케이블을 뽑거나 끼우지 않는다.
5. `tb3-ready` 또는 `tb3-check`가 실패하면 우회해서 주행하지 않는다.

## 매번 시작할 때: 가장 짧은 절차

### 1. 로봇 켜기

1. 배터리를 연결한다.
2. TurtleBot3 전원을 켠다.
3. Jetson과 라이다가 올라오도록 약 30초 기다린다.
4. 모터/OpenCR만 껐다 켰고 Jetson은 계속 켜져 있었다면 새 터미널에서
   다음 명령을 한 번 실행한다.

```bash
tb3-restart
```

왜 실행하는가: OpenCR 전원만 재인가하면 Jetson의 기존 ROS 2 프로세스와
DDS endpoint가 남을 수 있다. 이 명령은 Jetson bringup을 재시작하고 이전
endpoint가 사라진 뒤 odom·TF·배터리·torque·라이다까지 정밀 검사한다.

전체 로봇을 함께 껐다 켰고 자동 bringup이 정상이라면 보통 생략한다.

### 2. 빠른 상태 확인

```bash
tb3-ready --mode slam
```

왜 `--mode slam`인가: SLAM은 바퀴뿐 아니라 라이다가 반드시 필요하므로
빠른 공통 검사에 `/scan` 검사를 추가한다. 정상일 때만 다음 명령을 실행한다.

정상 출력의 마지막 줄:

```text
OK: fast robot readiness gate passed
```

실패하면 원인을 억지로 건너뛰지 말고 다음 정밀 검사를 실행한다.

```bash
tb3-check
```

왜 따로 있는가: 평상시의 `tb3-ready`는 시작 지연을 줄인 빠른 검사이고,
`tb3-check`는 ROS daemon 초기화와 더 많은 연속 표본으로 중복 endpoint,
시간 역행, 순간 torque-off까지 찾는 장애 후 정밀 검사다.

## 새 지도 만들기: 터미널 3개

지도 이름 예시는 `my_room`이다. 같은 이름의 기존 지도는 덮어쓰지 않으므로
날짜나 장소를 붙여 새 이름을 사용한다.

### 터미널 1 — SLAM과 RViz

```bash
tb3-slam
```

왜 터미널 1을 계속 열어 두는가: Cartographer가 `/scan`과 wheel odometry로
지도를 계속 갱신하고 RViz에 `/map`과 TF를 제공하는 주 프로세스이기 때문이다.
이 프로세스를 먼저 종료하면 아직 저장하지 않은 지도를 잃는다.

`Starting real TurtleBot3 SLAM and RViz.`가 나오고 RViz에 회색 배경, 흰색
자유 공간, 검은 벽/장애물이 나타나는지 확인한다. 지도를 저장하기 전까지
이 터미널을 닫지 않는다.

### 터미널 2 — 키보드 조종

```bash
tb3-teleop
```

왜 전용 명령을 쓰는가: 노트북 Jazzy 기본 teleop은 `TwistStamped`를 쓰지만
Jetson Humble의 실물 노드는 plain `Twist`를 구독한다. 이 명령은 타입을
맞추고 지도 품질을 위해 최고 속도를 0.18m/s로 제한한다.

| 키 | 동작 |
|---|---|
| `w` | 전진 속도를 한 단계 올림 |
| `x` | 감속하고 계속 누르면 후진 |
| `a` / `d` | 주행하면서 부드럽게 좌회전 / 우회전 |
| `r` | 조향만 0으로 만들어 직진 복귀 |
| `s` 또는 Space | 즉시 정지 |
| `Ctrl+C` | 정지 명령 3회 전송 후 종료 |

처음에는 `w`를 2~3번만 누른다. 벽을 따라 완만하게 돌고, 이미 지나간
장소로 다시 돌아와 지도 윤곽이 겹치게 한다. 급회전, 빠른 후진, 라이다를
몸으로 가리는 행동은 지도 품질을 떨어뜨린다.

기본 mapping mode 최고 직선 속도는 `0.18m/s`다. 지도 작성이 끝난 뒤 넓은
바닥에서 주행만 할 때 공식 Burger 상한 `0.22m/s`를 쓰려면 다음과 같이
실행한다.

```bash
tb3-teleop --sport
```

왜 `--sport`를 분리했는가: Burger 공식 직선 상한 0.22m/s는 넓은 바닥에서
주행을 즐길 때만 쓰고, SLAM 중 급가속으로 odometry/scan 정합이 나빠지는
일을 막기 위해 기본 모드와 의도적으로 분리했다.

### 터미널 3 — 상태 확인과 저장

충분히 주행하고 출발 지점 근처로 다시 온 뒤 터미널 2에서 `s`를 누른다.
로봇이 완전히 정지하면 터미널 3에서 실행한다.

```bash
tb3-slam-check
tb3-map-save my_room
```

왜 두 명령을 순서대로 쓰는가: 첫 명령은 Cartographer·지도·TF가 현재
살아 있는지 확인하고, 둘째 명령은 그 상태에서 YAML/PGM을 새 이름으로
저장한 뒤 파일 크기·해상도·픽셀 구성을 재검증한다. 기존 이름은 덮어쓰지
않아 실수로 좋은 지도를 잃지 않는다.

성공하면 다음 두 파일이 생긴다.

```text
~/maps/my_room.yaml
~/maps/my_room.pgm
```

정상 출력 예:

```text
OK: valid map 887x206 @ 0.05m/cell
OK: map saved
```

반드시 저장 성공을 확인한 뒤 다음 순서로 종료한다.

1. 터미널 2의 TELEOP에서 `Ctrl+C`
2. 터미널 1의 SLAM에서 `Ctrl+C`

## 저장 지도에서 자율주행

현재 저장한 실제 지도를 사용하려면 다음 이름을 그대로 쓴다.

```bash
tb3-nav dapier_real_20260812
```

왜 지도 이름을 인자로 쓰는가: `~/maps/<이름>.yaml`을 절대경로로 검증해
가상 예제 지도가 아니라 방금 만든 실물 지도를 `map_server`에 넣기 위해서다.

새로 만든 `my_room` 지도를 사용한다면 다음처럼 바꾼다.

```bash
tb3-nav my_room
```

### 1. RViz에서 현재 위치 지정

1. 상단의 **2D Pose Estimate**를 누른다.
2. 지도에서 로봇이 실제로 서 있는 위치를 누른다.
3. 누른 채로 로봇 앞 방향으로 드래그한 뒤 놓는다.
4. 라이다 점과 지도 벽이 겹치는지 확인한다.

위치나 방향을 잘못 찍었다면 목표를 보내지 말고 다시 지정한다.

화면에서 드래그가 안 되는 것처럼 보이면 RViz 상단에서 파란색으로 선택된
도구를 확인한다. **Move Camera**가 선택된 상태에서는 지도만 움직이며 초기
자세가 입력되지 않는다.

1. **2D Pose Estimate** 글자 또는 아이콘을 한 번 누른다. 단축키는 `P`다.
2. 지도 위 실제 위치에서 **왼쪽 버튼을 누른 채** 로봇 앞 방향으로 마우스를
   옮긴다.
3. 화살표 방향이 실제 로봇 앞을 가리킬 때 버튼을 놓는다.
4. 잘못 찍었으면 같은 동작을 다시 하면 마지막 입력으로 갱신된다.

클릭만 하고 이동하지 않으면 위치는 들어가도 방향이 거의 0°로 입력될 수
있다. 화면의 분홍/빨강 라이다 점이 검은 지도 벽과 겹치지 않으면 Nav2 Goal을
누르지 않는다.

### 2. Nav2 전체 상태 확인

새 터미널에서 지도 이름을 똑같이 넣는다.

```bash
tb3-nav-check dapier_real_20260812
```

왜 RViz 초기 자세 뒤에 실행하는가: AMCL의 `map → odom`은 실제 시작 위치를
알려 준 뒤에만 생긴다. 이 명령은 부분 활성화된 Nav2가 있으면 안전하게
재활성화하고, 지도·TF·action server·Twist 타입·속도 제한과 실물 센서를
한꺼번에 확인하는 최종 주행 게이트다.

마지막 줄이 `OK: Nav2 is active`로 시작해야 한다. 이 검사는 다음을 모두
확인한다.

- 지정한 YAML/PGM 지도를 정확히 로드했는지
- AMCL의 `map → odom`과 로봇의 `odom → base_footprint`가 살아 있는지
- `/navigate_to_pose` action server가 정확히 하나인지
- `/cmd_vel`이 Humble 로봇과 맞는 plain `Twist`인지
- 자율주행 속도 제한 `0.18m/s`가 적용됐는지
- 배터리, torque, odom, TF, 라이다가 정상인지

### 3. 가까운 첫 목표 보내기

목표 결과를 정확히 남기기 위해 새 터미널에서 watcher를 먼저 실행한다.

```bash
tb3-nav-watch
```

`READY: click Nav2 Goal in RViz`가 나오면 다음 순서로 진행한다.

1. 처음에는 로봇에서 0.5~1m 떨어진, 장애물 없는 흰색 공간만 고른다.
2. RViz 상단의 **Nav2 Goal**을 누른다.
3. 목표 위치를 누르고 원하는 도착 방향으로 드래그한다.
4. 로봇 옆에서 주행을 지켜보고 이상하면 전원을 끈다.
5. watcher의 최종 결과가 `result=SUCCEEDED`인지 확인한다.

watcher는 실행 전에 남아 있던 옛 goal ID를 제외하고 새 RViz goal ID 하나만
추적한다. `CANCELED`, `ABORTED`, action server 없음, 5분 timeout은 모두
실패 종료 코드로 남는다.

첫 목표가 성공한 뒤에만 더 먼 목표를 시도한다. 이 프로젝트에서 완료로
인정하는 조건은 단순히 RViz가 열린 것이 아니라, 실제 로봇이 목표에 도착하고
`NavigateToPose` 결과가 `SUCCEEDED`인 것이다.

### 4. 여러 웨이포인트를 순서대로 방문하기

좌표는 RViz의 `map` 프레임 기준 `X Y 도착방향(도)`이다. 먼저
`--execute` 없이 **계획만** 검사한다.

```bash
tb3-waypoints \
  --pose 3.025 0.152 0 \
  --pose 6.025 0.152 0 \
  --pose 9.025 0.102 180 \
  --pose 6.025 0.152 180 \
  --pose 3.025 0.152 180 \
  --pose 0.100 0.030 -47.3
```

왜 먼저 계획만 하는가: 잘못 입력한 좌표, 회색 미탐색 영역, 검은 장애물,
끊긴 통로를 바퀴가 움직이기 전에 `ComputePathThroughPoses`로 거부하기
위해서다. `PLAN OK`의 waypoint 수, path pose 수, 전체 길이가 의도와
맞는지 확인한다. 위 좌표는 `dapier_real_20260812` 지도에서 실제 검증한
코스이므로 다른 지도에서는 그대로 쓰지 않는다.

계획이 맞으면 **같은 명령에 `--execute`만 추가**한다.

```bash
tb3-waypoints --execute \
  --pose 3.025 0.152 0 \
  --pose 6.025 0.152 0 \
  --pose 9.025 0.102 180 \
  --pose 6.025 0.152 180 \
  --pose 3.025 0.152 180 \
  --pose 0.100 0.030 -47.3
```

왜 `--execute`를 별도 표시하는가: 명령을 읽거나 계획을 검사하는 행위와
실물 바퀴를 움직이는 행위를 명확히 구분하기 위해서다. 실행 모드도 먼저
`tb3-ready --mode nav`를 자동 수행하고 전체 경로계획이 다시 성공한 뒤에만
`FollowWaypoints`를 보낸다. 최종 성공 조건은 다음 세 가지가 모두 맞는 것이다.

```text
TOUR RESULT: status=4 code=0 missed=[]
OK: every waypoint was visited
```

`status=4`만 보고 끝내지 않고 `code=0`과 `missed=[]`도 확인한다. 그래야
중간 웨이포인트를 건너뛰고 마지막 점만 도달한 경우를 성공으로 오인하지 않는다.

### 5. RViz 화면에서 여러 웨이포인트 찍기

터미널 좌표 대신 화면으로도 순서 목록을 만들 수 있다. Navigation 2 패널의
`Navigation`과 `Localization`이 둘 다 초록색 `active`인지 먼저 확인한다.
`Navigation: inactive`이고 **Startup** 버튼이 보이면 초기 자세를 지정한 뒤
`tb3-nav-check <지도이름>`을 실행한다. 그래도 inactive면 **Startup**을 한 번
누르고 다시 `tb3-nav-check`를 통과시킨다.

1. Navigation 2 패널의 **Waypoint / Nav Through Poses Mode**를 누른다.
   버튼 글자가 **Start Waypoint Following**으로 바뀌면 좌표 누적 모드다.
2. 상단 **Nav2 Goal**을 누른 뒤 첫 도착 위치에서 왼쪽 버튼을 누르고 도착
   방향으로 드래그해 놓는다.
3. 같은 **Nav2 Goal** 도구로 두 번째, 세 번째 위치도 차례대로 찍는다.
   화면에 웨이포인트 마커가 쌓이는지 확인한다.
4. 각 점에서 정지·방향 정렬하며 순서대로 방문하려면
   **Start Waypoint Following**을 누른다.
5. 점들을 하나의 연속 경로처럼 지나가려면 **Start Nav Through Poses**를
   누른다. 이번 실물 검증처럼 각 웨이포인트 방문 여부를 확인할 때는
   Waypoint Following을 사용한다.
6. 실행 전 목록을 버리려면 **Cancel Accumulation**, 실행 중 멈추려면
   **Cancel**을 누른다. `s` 키는 RViz 비상정지 키가 아니므로 실제 이상 시
   로봇 전원을 끈다.

왜 두 실행 버튼이 있는가: `FollowWaypoints`는 현재 waypoint index와 누락
목록을 결과로 주므로 “모든 점 방문”을 검증하기 좋다. `NavigateThroughPoses`는
전체 자세 목록을 하나의 navigation task로 다루므로 연속 경유에 가깝다.
화면 방식은 편하지만 경로 전체 길이를 미리 수치로 검증하지 않으므로, 먼
코스는 먼저 `tb3-waypoints` 계획 모드로 검사하는 편이 안전하다.

## 모터 보호 오류가 다시 생기면 이어 할 정확한 순서

이번 주행은 정상 완료했지만 좌측 모터 보호 오류 이력이 있으므로 재발하면
다음 순서로 복구한다.

1. 모터를 충분히 식힌다.
2. 전원을 끈 상태에서 좌측 바퀴가 프레임/타이어에 닿거나 유난히 뻑뻑하지
   않은지 확인한다.
3. 좌측 TTL 케이블 양쪽이 완전히 체결됐는지 확인한다.
4. 바퀴를 바닥에서 띄운 뒤 전원을 켠다.
5. 다음 명령을 실행한다.

```bash
tb3-restart
tb3-check
```

6. 빨간 LED가 없고 `torque=true`가 계속 유지되면 바닥에 내린다.
7. 저장 지도에서 Nav2를 시작한다.

```bash
tb3-nav dapier_real_20260812
```

8. **2D Pose Estimate** → `tb3-nav-check dapier_real_20260812` →
   `tb3-nav-watch` → 가까운 **Nav2 Goal** 순서로 진행한다.

냉간·무부하에서도 좌측 모터 LED와 torque-off가 재발하면 Nav2를 진행하지
않고 DYNAMIXEL Wizard 진단 단계로 전환한다.

## 오류가 나오면 이 표부터 보기

| 증상 | 의미 | 바로 할 일 |
|---|---|---|
| `tb3-ready` 실패 | 평상시 안전 조건 미충족 | `tb3-check` 실행 |
| Jetson unreachable | 전원·Wi-Fi·IP 문제 | Jetson 부팅과 같은 공유기 연결 확인 |
| `/cmd_vel` subscriber 0 | bringup 미실행 | `tb3-restart` |
| endpoint가 2개 이상 | 다른 ROS participant 또는 stale DDS | 관련 터미널 종료 후 `tb3-restart` |
| odom duplicate/regression | 다른 네트워크 데이터 혼입 또는 시간 문제 | domain 73 확인, `tb3-restart` |
| map → odom 없음 | SLAM/AMCL 미실행 또는 초기 위치 미지정 | SLAM 상태 또는 2D Pose Estimate 확인 |
| 배터리 11.1V 미만 + USER3/비프 | OpenCR 저전압 보호 | 전원 끄고 배터리 분리 충전 |
| 모터 자체 빨간 LED + torque false | XL430 보호 shutdown | 정지·전원 끄기·냉각·기계/케이블 확인 |
| `/scan` 없음 | 라이다/USB/컨테이너 장치 문제 | 라이다 회전과 `/dev/ttyUSB0` 확인 |
| 기존 지도 이름 오류 | 덮어쓰기 방지 | 새 지도 이름 사용 |

## 명령어 한 장 요약

```bash
# 평상시 빠른 확인
tb3-ready --mode slam

# 이상이 있었을 때 정밀 확인
tb3-check

# Jetson의 로봇 bringup 재시작 + 정밀 확인
tb3-restart

# 실제 지도 만들기
tb3-slam
tb3-teleop
tb3-slam-check
tb3-map-save my_room

# 저장 지도에서 자율주행
tb3-nav my_room
tb3-nav-check my_room
tb3-nav-watch

# 좌표 여러 개를 경로검사만 한 뒤 실제 순회
tb3-waypoints --pose X1 Y1 YAW1 --pose X2 Y2 YAW2
tb3-waypoints --execute --pose X1 Y1 YAW1 --pose X2 Y2 YAW2
```

## 새 PC 또는 새 clone에서 최초 1회

ROS 2 Jazzy와 이 저장소의 TurtleBot3 workspace가 이미 준비됐다는 전제다.

```bash
cd ~/DAPIER
./turtlebot3_ws/scripts/install_tb3_commands.sh

cd ~/DAPIER/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select dapier_turtlebot3_real
source install/setup.bash
```

새 터미널에서 명령을 못 찾으면 다음을 한 번 확인한다.

```bash
export PATH="$HOME/.local/bin:$PATH"
command -v tb3-ready tb3-slam tb3-teleop tb3-map-save tb3-nav
```

## 이번 실습에서 배운 내용

- SLAM은 라이다만 그리는 작업이 아니라 라이다, wheel odometry, TF, 시간
  순서가 모두 맞아야 유지된다.
- 같은 공유 네트워크의 다른 ROS 2 participant가 같은 topic 이름을 쓰면
  정상 메시지 사이에 다른 odom이 섞일 수 있다. 실물 전용 domain 73으로
  분리해 해결했다.
- Jazzy 기본 TELEOP의 `TwistStamped`와 Humble 로봇의 plain `Twist`는
  타입이 다르므로 실물 전용 TELEOP을 사용한다.
- 지도는 SLAM 터미널을 닫기 전에 YAML과 PGM 두 파일로 저장해야 한다.
- Nav2는 저장 지도를 여는 것만으로 위치를 알지 못한다. AMCL에 실제 시작
  위치와 방향을 알려야 한다.
- 30cm 목표에 기본 25cm goal tolerance를 쓰면 거의 움직이지 않고도 성공
  처리될 수 있었다. 실물 검증값 8cm/0.15rad로 줄이고, 위치 변화와 action
  결과를 함께 봐야 한다.
- 여러 점 자율주행은 계획 성공만으로 끝내지 않는다. `FollowWaypoints`의
  terminal status, error code, missed waypoint 목록이 모두 정상이어야 한다.
- 빠른 점검과 정밀 진단은 목적이 다르다. 평상시에는 동시에 최소 표본만
  확인하고, 장애 뒤에는 discovery 초기화와 긴 검사를 수행한다.
- 모터 torque-off를 우회해서 주행시키는 것은 해결이 아니다. 보호 원인을
  제거하고 정상 상태가 유지되는지 확인해야 한다.

## 공개 저장소에 포함하는 것과 제외하는 것

GitHub에는 실행 스크립트, 테스트, 실물 Nav2 패키지, 패치와 이 문서를
올린다. `~/maps`의 실제 공간 지도는 로컬 파일이며 건물 구조를 드러낼 수
있으므로 기본적으로 공개 저장소에 올리지 않는다. Notion에는 실제 실행
로그, 실패 원인, 지도 결과, 최종 Nav2 성공 여부를 비공개 학습 기록으로
남긴다.
