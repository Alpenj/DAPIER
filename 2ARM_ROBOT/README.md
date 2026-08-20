# 2ARM_ROBOT — 이동형 양팔 신발 정리 로봇

JDcobot300 양팔, TurtleBot3 Waffle Pi, Orbbec Astra Pro를 이용해 무작위로
놓인 신발 30켤레를 짝지어 정렬하거나 신발장에 넣는 DAPIER 팀 프로젝트다.

현재 커밋은 **실물 연결 전 Phase 0**이다. 합성 episode의 데이터 계약,
quality gate, SQLite manifest와 자동 테스트까지 구현했으며 실제 로봇 동작,
카메라 캘리브레이션 또는 ACT 성능을 검증했다는 의미는 아니다.

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
- timestamp gap, camera drop/skew, stream shape, joint jump, checksum 검사
- 조작 중 TurtleBot 측정/명령 속도 정지 interlock
- 검수 상태 및 calibration/config version quality gate
- SQLite 기반 train/validation, usable, success, shoe pair 질의

## Ubuntu ROS 2 교육 PC에서 시작

현재 게시 브랜치를 직접 받는 명령이다.

```bash
git clone --branch feat/2arm-robot-phase0 --single-branch \
  https://github.com/Alpenj/DAPIER.git
cd DAPIER/2ARM_ROBOT
bash scripts/verify_ubuntu_ros2.sh
source install/setup.bash
```

검증 스크립트는 ROS 2나 Python 패키지를 새로 설치하지 않는다. 현재 shell의
ROS 환경을 사용하고, 아직 source되지 않았다면 `/opt/ros/jazzy`와
`/opt/ros/humble`만 순서대로 확인한다. `build/`, `install/`, `log/`는 이
폴더 안에 생성되며 Git에는 올라가지 않는다.

필수 환경은 `python3`, `setuptools`, `ros2`, `colcon`이다. 하나라도 없으면
스크립트가 설치를 시도하지 않고 누락 항목을 출력한 뒤 종료한다.

수동 실행 시:

```bash
source /opt/ros/jazzy/setup.bash  # Humble 설치 PC는 humble로 변경
cd ~/DAPIER/2ARM_ROBOT

(cd src/shoe_sorting_data && python3 -m unittest discover -s test -v)
colcon build --symlink-install --packages-select shoe_sorting_data
source install/setup.bash
shoe_episode --help
```

## 합성 episode 20개 만들기

```bash
cd ~/DAPIER/2ARM_ROBOT
source install/setup.bash

shoe_episode generate \
  --root output/golden_episodes \
  --count 20 \
  --seed 100

shoe_episode validate \
  --manifest output/golden_episodes/episode_000001/episode_manifest.json

shoe_episode index \
  --root output/golden_episodes \
  --db output/episode_manifest.sqlite3

shoe_episode query \
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

상세 인수인계는 [`docs/PHASE0_HANDOFF.md`](docs/PHASE0_HANDOFF.md), 요구사항
원장은 [`docs/requirements-ledger.md`](docs/requirements-ledger.md)를 본다.

## 다음 코드 작업

실물 없이 계속 가능한 순서는 다음과 같다.

1. 합성 ROS 2 topic을 받는 mock episode recorder
2. Phase 0 episode를 ACT/LeRobot 입력으로 변환하는 adapter
3. ACT용 train/validation split과 offline evaluator
4. 신발 pair embedding/API 응답 계약과 mock 서버

실물 확보 후에는 joint name/order/unit, gripper 차원, Astra Pro timestamp,
calibration version, base 정지 신호를 확인해 placeholder를 교체한다.
