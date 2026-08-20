# 2ARM_ROBOT — 이동형 양팔 신발 정리 로봇

JDcobot200 양팔, TurtleBot3 Waffle Pi, Orbbec Astra 계열 RGB-D 카메라를 이용해 무작위로
놓인 신발 30켤레를 짝지어 정렬하거나 신발장에 넣는 DAPIER 팀 프로젝트다.

현재 구현은 Phase 0 데이터 계약에 실물 하드웨어 발견 결과를 연결하는 단계다. 양팔 12개
STS3215의 읽기 전용 telemetry, TurtleBot3 stationary baseline과 들린 바퀴 속도 응답은
확인했다. 실제 신발 episode, 카메라 depth stream, 팔 동작·토크 캘리브레이션 또는 ACT
성능을 검증했다는 의미는 아니다.

## 현재 구성

```text
2ARM_ROBOT/
├── src/
│   └── shoe_sorting_data/       # ROS 2 ament_python 패키지
├── docs/                         # 요구사항, 팀 결정, 조사 참고자료
├── scripts/
│   └── verify_ubuntu_ros2.sh     # 설치 없이 환경·테스트·빌드 검증
└── README.md
```

Phase 0에서 제공하는 기능:

- 좌·우 팔/그리퍼, base velocity, RGB/Depth timestamp episode 계약
- seed로 재현 가능한 합성 golden episode
- 합성 ROS 2 topic publisher와 approximate-time episode recorder
- timestamp gap, camera drop/skew, stream shape, joint jump, checksum 검사
- 조작 중 TurtleBot 측정/명령 속도 정지 interlock
- 검수 상태 및 calibration/config version quality gate
- SQLite 기반 train/validation, usable, success, shoe pair 질의
- one-shot 신발 임베딩 exemplar의 `match/abstain` 계약
- accepted episode 기반 typed skill exemplar 등록·호환 검색
- object/session/span 기반 exemplar 평가 leakage audit

실측 기반 전력·계산 보드 결정은 [전력·계산 보드 예산](docs/POWER_AND_COMPUTE_BUDGET.md),
URDF/MuJoCo/Gazebo 자산과 sim-to-real 순서는
[로봇 모델 자산 감사](docs/ROBOT_MODEL_ASSET_AUDIT.md)에 기록했다.
비식별 실측 원본과 요약은 [hardware evidence](docs/evidence/HARDWARE_EVIDENCE.md)에서 확인할 수 있다.

## Ubuntu ROS 2 교육 PC에서 시작

현재 게시 브랜치를 직접 받는 명령이다.

```bash
git clone --branch feat/2arm-robot-phase0 --single-branch \
  https://github.com/Alpenj/DAPIER.git
cd DAPIER/2ARM_ROBOT
bash scripts/verify_ubuntu_ros2.sh
set +u
source install/setup.bash
set -u
```

검증 스크립트는 ROS 2나 Python 패키지를 새로 설치하지 않는다. 현재 shell의
ROS 환경을 사용하고, 아직 source되지 않았다면 `/opt/ros/jazzy`와
`/opt/ros/humble`만 순서대로 확인한다. `build/`, `install/`, `log/`는 이
폴더 안에 생성되며 Git에는 올라가지 않는다.

필수 환경은 `python3`, `setuptools`, `ros2`, `colcon`이다. 하나라도 없으면
스크립트가 설치를 시도하지 않고 누락 항목을 출력한 뒤 종료한다.

## 실물 장비를 연결했을 때 가장 먼저 할 일

현재 저장소에는 읽기 전용 관절 snapshot과 바퀴 characterization은 있지만, 실제 카메라 frame,
rosbag 또는 신발을 집은 실물 episode는 없다. 따라서 아직 검증하지 않은 joint sign이나 카메라
topic 이름을 추측해 코드에 확정하지 않는다. 장비 driver를
실행한 뒤 아래 스크립트로 읽기 전용 snapshot부터 만든다.

```bash
cd ~/DAPIER/2ARM_ROBOT
bash scripts/capture_ros2_hardware_snapshot.sh \
  output/hardware_snapshots/first_connected
```

이 스크립트는 node/topic/type, endpoint QoS, `JointState`, `CameraInfo`, base
velocity/odometry의 첫 message를 저장한다. `Image`는 픽셀을 저장하지 않고
header만 수집한다. 어떤 motion command도 publish하지 않으며 출력 폴더가 비어
있지 않으면 덮어쓰지 않고 중단한다.

현재 장비 node가 하나도 실행되지 않았다면 exit 2와 `NO_CANDIDATE_TOPICS`를
반환한다. snapshot을 확인한 뒤에만 mock topic mapping을 실제 이름으로 교체한다.

수동 실행 시:

```bash
source /opt/ros/jazzy/setup.bash  # Humble 설치 PC는 humble로 변경
cd ~/DAPIER/2ARM_ROBOT

(cd src/shoe_sorting_data && python3 -m unittest discover -s test -v)
colcon build --symlink-install --packages-select shoe_sorting_data
set +u
source install/setup.bash
set -u
ros2 run shoe_sorting_data shoe_episode --help
```

## 합성 ROS 2 데이터를 episode로 녹화하기

실물 recorder와 RGB-D driver가 아직 완성되지 않았으므로 publisher가 양팔 state/action, base 측정/명령,
RGB/Depth metadata 등 8개 topic을 20 Hz로 만든다. recorder는 같은 시점의
topic을 묶어 기존 `samples.jsonl`과 `episode_manifest.json` 계약으로 저장한 뒤
quality validator를 실행한다.

아래 one-shot demo는 40 sample을 발행하고 녹화해 accepted 합성 episode 하나를
만든다. 기존 파일을 보호하기 위해 `--output` 폴더가 비어 있지 않으면 중단한다.

```bash
cd ~/DAPIER/2ARM_ROBOT
set +u
source install/setup.bash
set -u

ros2 run shoe_sorting_data shoe_mock_demo \
  --output output/mock_episodes/episode_000001 \
  --samples 40
```

publisher와 recorder를 별도 terminal에서 실행할 수도 있다.

```bash
# terminal 1
ros2 run shoe_sorting_data shoe_mock_publisher

# terminal 2: 합성 결과를 quality gate까지 accepted로 검사
ros2 run shoe_sorting_data shoe_mock_recorder \
  --output output/mock_episodes/episode_000002 \
  --samples 40 \
  --accept
```

중간에 recorder를 멈추거나 timeout이 발생하면 가능한 경우 `aborted` outcome과
failure reason을 manifest에 남기며 학습 usable 데이터로 승인하지 않는다.

## 합성 episode 20개 만들기

```bash
cd ~/DAPIER/2ARM_ROBOT
set +u
source install/setup.bash
set -u

ros2 run shoe_sorting_data shoe_episode generate \
  --root output/golden_episodes \
  --count 20 \
  --seed 100

ros2 run shoe_sorting_data shoe_episode validate \
  --manifest output/golden_episodes/episode_000001/episode_manifest.json

ros2 run shoe_sorting_data shoe_episode index \
  --root output/golden_episodes \
  --db output/episode_manifest.sqlite3

ros2 run shoe_sorting_data shoe_episode query \
  --db output/episode_manifest.sqlite3 \
  --usable true \
  --split validation
```

`output/`은 생성 결과용이며 Git에서 제외된다.

## 확정된 개발 방향

- ACT 기준선을 먼저 완성한다.
- DYNA-lite 데이터 계약과 quality gate를 사용한다.
- 4주차 이후 IDM/FDM/EMA 보조학습은 go/no-go ablation으로 판단한다.
- LLM/VLM은 신발 짝, 목표 슬롯, 스킬과 실패 복구를 결정한다.
- 관절 명령은 ACT 계열 정책과 별도 safety supervisor가 담당한다.
- 이동과 조작을 분리하고 Nav2 도킹 후 base 정지를 확인해야 조작을 허용한다.

GEN-1.5 조사에서는 짧은 physical prompt의 **형태만** 참고했다. 공개
checkpoint/API가 없으므로 GEN-1.5 자체를 실행하지 않으며, local 구현은
인식 exemplar와 검증된 skill metadata retrieval뿐이다. 상세 경계는
[`docs/GEN15_ADOPTION.md`](docs/GEN15_ADOPTION.md)에 기록했다.

상세 인수인계는 [`docs/PHASE0_HANDOFF.md`](docs/PHASE0_HANDOFF.md), 요구사항
원장은 [`docs/requirements-ledger.md`](docs/requirements-ledger.md)를 본다.

## 다음 코드 작업

실물 없이 계속 가능한 순서는 다음과 같다.

1. Phase 0 episode를 ACT/LeRobot 입력으로 변환하는 adapter
2. ACT용 train/validation split과 offline evaluator
3. 실제 신발 crop embedding/API adapter와 mock 서버

실물 확보 후에는 joint name/order/unit, gripper 차원, Astra Pro timestamp,
calibration version, base 정지 신호를 확인해 placeholder를 교체한다.
