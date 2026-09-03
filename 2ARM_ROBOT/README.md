# 2ARM_ROBOT — 이동형 양팔 신발 정리 로봇

SO-101 두 팔과 TurtleBot3 Waffle Pi를 결합해 박스가 있는 위치까지 이동하고, 오른팔로
박스를 열고 왼팔로 신발을 꺼낸 뒤 출발 위치로 운반·배치하는 DAPIER 팀 프로젝트다.

현재 장비 정본은 [`config/hardware_roles.json`](config/hardware_roles.json)이다. H201은
기둥 상단 작업공간 top-view RGB-D, Astra S는 TurtleBot3 전면 Visual SLAM RGB-D이며,
좌우 SO-101에는 각각 RGB wrist camera가 있다. Raspberry Pi 4는 base·경량 I/O·안전 감시를,
로컬 노트북은 perception·IL policy·LLM과 Visual SLAM 배치 비교를 담당한다. LLM은 Pi에서
실행하지 않는다.

MuJoCo 양팔·카메라·접촉 모델과 과거 ±3도 실측 로그 비교는 준비됐지만, 왼쪽 손가락 접촉은
CPU에 따라 양측 또는 한쪽 경계에 있고 양측일 때도 법선이 직교해 협지가 아니다. 따라서
박스-신발 전체 물리 성공은 아직 아니다. 과거 팔 로그도 사용자가 화면으로 확인한 commissioning은
아니므로 다음 실물 시험은 카메라를 먼저 띄운 뒤 별도 승인으로 진행한다.

## 현재 구성

```text
2ARM_ROBOT/
├── config/
│   └── hardware_roles.json      # 현재 하드웨어 역할 정본(비밀값 제외)
├── src/
│   └── shoe_sorting_data/       # 초기 JDcobot Phase 0 계약; 데이터 유틸만 선택 재사용
├── sim/
│   ├── mobile_dual_so101/       # 현재 Waffle Pi + SO-101 양팔 MuJoCo 모델
│   ├── jdcobot200_dual/         # legacy reference
│   ├── turtlebot3_waffle_pi/    # 공식 Waffle Pi URDF/mesh와 MuJoCo 변환
│   └── mobile_dual_arm/         # legacy JDcobot 조합 모델
├── docs/                         # 요구사항, 팀 결정, 조사 참고자료
├── scripts/
│   ├── dual_so101_smoke          # 승인형 양팔 저속 계측
│   ├── capture_usb_snapshot      # 승인형 read-only USB 전후 기록
│   └── run_astra_openni2_color  # 전면 Astra S 검증 runtime
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
[로봇 모델 자산 감사](docs/ROBOT_MODEL_ASSET_AUDIT.md)에 기록했다. WikiDocs 20199의
JDcobot200 전용 URDF 생성·MJCF 변환·그리퍼 자료는
[JDcobot200 URDF 가이드](docs/WIKIDOCS_20199_JDCOBOT200_URDF_GUIDE.md)에서 확인한다.
비식별 실측 원본과 요약은 [hardware evidence](docs/evidence/HARDWARE_EVIDENCE.md)에서 확인할 수 있다.

JDcobot200 원본 모델과 초기 ROS 2 패키지는 학습·회귀용 legacy 자료다. 현재 실물 명령 또는
하드웨어 역할의 근거로 사용하지 않는다. 현재 12차원 action, 좌우 namespace, camera role,
IK·접촉·충돌 검증은 [SO-101 이동형 양팔 모델](sim/mobile_dual_so101/README.md)을 기준으로 한다.

ROBOTIS 공식 Waffle Pi 자산과 MuJoCo 변환은
[Waffle Pi 기준 모델](sim/turtlebot3_waffle_pi/README.md)에 보존한다. 현재 조합 모델은
`sim/mobile_dual_so101`이며 wheel-level 명령은 내부 base adapter에만 두고 공개 이동 계약은
선속도·각속도와 docking goal을 사용한다.

## Ubuntu ROS 2 교육 PC에서 시작

현재 `main`을 받는 명령이다.

```bash
git clone https://github.com/Alpenj/DAPIER.git
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

현재 저장소에는 과거 읽기 전용 관절 snapshot, 제한된 양팔 ±3도 로그와 바퀴 characterization이
있다. 그러나 사용자가 화면으로 확인한 양팔 commissioning, 동시 4카메라 부하, 실제 신발
episode는 없다. 따라서 현재 장비 역할은 `config/hardware_roles.json`에서 읽고, driver topic,
캘리브레이션·안전 limit은 아래 읽기 전용 snapshot과 현장 측정 뒤 확정한다.

```bash
cd ~/DAPIER/2ARM_ROBOT
bash scripts/capture_ros2_hardware_snapshot.sh \
  output/hardware_snapshots/first_connected \
  --confirm VISIBLE_ROS2_SNAPSHOT_READONLY
```

이 스크립트는 node/topic/type, endpoint QoS, `JointState`, `CameraInfo`, base
velocity/odometry의 첫 message를 저장한다. `Image`는 픽셀을 저장하지 않고
header만 수집한다. 사용자가 현장에서 read-only graph 접근을 승인한 exact token이
없으면 ROS 2를 호출하지 않는다. 어떤 motion command도 publish하지 않으며 Git에서
제외된 `output/` 아래 새 폴더만 허용하고 기존 경로는 덮어쓰지 않는다.

현재 장비 node가 하나도 실행되지 않았다면 exit 2와 `NO_CANDIDATE_TOPICS`를
반환한다. snapshot을 확인한 뒤에만 mock topic mapping을 실제 이름으로 교체한다.

사용자가 현장에 있고 read-only 확인을 승인한 뒤, 카메라 실행 전과 양팔 시험 후 USB 상태를
각각 새 디렉터리에 기록한다. 결과는 Git에서 제외되는 `output/` 아래에만 생성된다.

```bash
2ARM_ROBOT/scripts/capture_usb_snapshot \
  2ARM_ROBOT/output/usb_snapshots/before \
  --confirm VISIBLE_USB_SNAPSHOT_READONLY

2ARM_ROBOT/scripts/capture_usb_snapshot \
  2ARM_ROBOT/output/usb_snapshots/after \
  --confirm VISIBLE_USB_SNAPSHOT_READONLY

diff -u 2ARM_ROBOT/output/usb_snapshots/{before,after}/lsusb-tree.txt
diff -u 2ARM_ROBOT/output/usb_snapshots/{before,after}/kernel-usb-events.txt
```

이 도구는 USB/V4L2 목록과 reset·disconnect·timeout 관련 kernel event만 읽고 장치 stream이나
serial port를 열지 않는다. 전체 snapshot 디렉터리는 그대로 Git에 올리지 않고 비식별 요약만
commissioning 근거로 정리한다.

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
- 이동의 학습 정책(IL)과 근접 파지·경로 보정(IK)을 결합하되 둘 다 safety gate를 우회하지 않는다.
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

## 다음 작업

1. MuJoCo에서 오른팔 뚜껑 접촉과 왼팔 양지 파지·friction-only lift gate 통과
2. H201 top-view와 Astra front-SLAM의 실제 depth·CameraInfo·extrinsic 검증
3. 좌우 wrist RGB를 포함한 4카메라 동시 FPS/drop/USB reset 측정
4. 사용자가 보는 화면과 E-stop을 준비한 뒤 양팔 ±3도 공개 실물 시험
5. 실측 STS3215 내부 profile·velocity·load/current로 MuJoCo actuator 보정
