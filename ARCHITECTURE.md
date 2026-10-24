# DAPIER architecture

DAPIER의 신규 로봇 코드는 언어가 아니라 실행 책임과 지연 특성에 따라 다음 세
계층으로 나눈다. 이 3계층은 `main`에 있는 설계 기준이다. 현재 sweepick 실물
실행이 실제로 거치는 경로는 아래 [sweepick 실물 실행 경로](#sweepick-실물-실행-경로-2026-10-검토-브랜치)에 따로 적었다.

| 계층 | 기준 경로 | 소유 책임 |
|---|---|---|
| Python 연구 계층 | [`2ARM_ROBOT/research/`](2ARM_ROBOT/research/README.md) | object perception, policy, planning, simulation, dataset, offline evaluation, versioned intent 생성 |
| C++ 인식·지역화 런타임 | [`so101_ros2/dapier_localization_core/`](so101_ros2/dapier_localization_core/README.md) | localization estimate 계약, tracking quality, map/reset generation, 향후 Visual SLAM·TF adapter |
| C++ 실시간 제어 계층 | [`so101_ros2/dapier_so101_core/`](so101_ros2/dapier_so101_core/) | receiver-local TTL, sequence, interlock, limit, watchdog, safe stop, hardware I/O |

Python과 C++ 제어 사이의 단일 계약은
[`contracts/research_realtime_control_v1.json`](contracts/research_realtime_control_v1.json)이다.
Visual SLAM·지역화 producer와 소비자 사이의 단일 계약은
[`contracts/localization_runtime_v1.json`](contracts/localization_runtime_v1.json)이다.

Python은 실제 장비 실행을 승인하지 않는다. localization runtime도 actuator 명령을
승인하거나 실행하지 않는다. C++ 제어 계층이 intent와 localization quality를 다시
검증한 뒤 로봇별 제한과 enable 상태를 적용한다.

현재 C++ 계층의 “실시간”은 제어 책임을 의미한다. PREEMPT_RT와 worst-case
latency를 측정하기 전에는 hard real-time을 검증했다고 주장하지 않는다. Visual
SLAM tracking, local mapping, loop/map merging과 full bundle adjustment는 variable
latency 작업이므로 motor command loop와 별도 process/package에서 실행한다.

## 비전 기반 sim-to-real 원칙

MuJoCo runtime policy와 planner는 물체의 `data.xpos`, body/geom ID 또는 segmentation
ID로 접근 좌표를 만들지 않는다. RGB detector 결과와 aligned metric depth,
camera intrinsics, 실제 장비에도 존재하는 camera-to-base 보정 transform으로 target을
계산한다. simulator truth는 reset, reward, offline label과 test-only 오차 측정에만
사용한다.

Visual SLAM은 로봇이 지도에서 어디에 있는지 추정한다. RGB-D object perception은
집을 물체가 camera/base/map frame에서 어디에 있는지 추정한다. 두 모듈은 분리하고
calibrated TF에서 결합한다. (초기 과제에서는 이 인식 대상이 신발이었다.)

## sweepick 실물 실행 경로 (2026-10, 검토 브랜치)

sweepick 상체의 실물 실행 소스는 [`pro/sweepick-review-20261015`](https://github.com/Alpenj/DAPIER/tree/pro/sweepick-review-20261015/2ARM_ROBOT/sweepick)
등 작업 브랜치에서 검수 중이며 아직 `main`에 병합하지 않았다. 실물 실행은 위
C++ 제어 계층을 거치지 않고, Python 실행기가 SO-101 서보 버스를 직접 소유한다.

```text
웹 콘솔 (START·승인·실행 기록 검수)
  -> 세션 (관측 -> 팔 선택 -> pregrasp -> 손목 영상 정렬 -> 하강 -> 집기 -> 들기
           -> 운반 -> 놓기 -> 복귀)
       관측: OS30A RGB-D 깊이 + camera->base 변환 -> 물체 윗면 위치
             손목 RGB -> 정렬 오차(px -> mm)
       계획: IK·충돌 검사(MuJoCo 모델) -> 관절 구간
  -> 실행기 (팔당 단일 owner, Goal 쓰기·ACK·readback, 진행 감시, 정지 조건)
  -> SO-101 서보 (Feetech, 위치 제어)
기록기: 손목 RGB 2 + OS30A RGB-D + 양팔 측정값·실제 Goal 명령 (학습 데이터 원본)
```

학습 정책(ACT)을 쓸 때는 정책이 카메라·관절 입력으로 관절 목표 묶음을 내고,
관절 한계·정지 조건 검사를 거쳐 실행된다. 2026-10-23 수집 경로에서는 ACT가
접근하다가 정체가 감지되면 교사 실행기가 이어받는다.

- 상체는 이동 베이스를 직접 명령하지 않는다. 필요할 때 정지 요청만 보내고, 임무
  성공 판정은 상위 임무 관리자가 한다(베이스·SLAM·도킹은 다른 팀 담당).
- 위 3계층(Python 연구 / C++ 지역화 / C++ 실시간 제어)으로 실물 실행을 옮기는 일은
  아직 하지 않았다. 이 절은 현재 실행 경로를 설명할 뿐 설계 기준을 바꾸지 않는다.

## ORB-SLAM3 source basis

ORB-SLAM3 is used as a system-architecture reference for visual, visual-inertial
and multi-map SLAM:

- Carlos Campos et al., *ORB-SLAM3: An Accurate Open-Source Library for Visual,
  Visual-Inertial and Multi-Map SLAM*, IEEE Transactions on Robotics,
  DOI: `10.1109/TRO.2021.3075644`.
- The paper separates Tracking, Local Mapping, Loop & Map Merging, Atlas,
  KeyFrame Database and Full BA responsibilities.
- It lists low-texture environments as a main failure case.
- Its timing measurements use an Intel Core i7-7700 with 32 GB RAM and do not
  establish Raspberry Pi performance.
- The paper supports RGB-D visual input and presents tightly integrated
  monocular-inertial and stereo-inertial systems. RGB-D support alone is not
  treated as proof of a ready-made tightly coupled RGB-D+IMU path for DAPIER's
  selected hardware.

## Detailed decisions

- [`ADR 0001 — Python 연구/C++ 실시간 제어 분리`](docs/architecture/0001-python-research-cpp-realtime-control.md)
- [`ADR 0002 — 센서 기반 비전 좌표`](docs/architecture/0002-vision-first-sim-to-real-perception.md)
- [`ADR 0003 — sim-to-real observation provenance`](docs/architecture/0003-sim-to-real-observation-provenance.md)
- [`ADR 0004 — C++ Visual SLAM localization runtime`](docs/architecture/0004-cpp-visual-slam-localization-runtime.md)

## 장비 없이 검증

```bash
scripts/verify-architecture-boundaries
python3 -m unittest discover -s 2ARM_ROBOT/research/test -v

c++ -std=c++17 -Wall -Wextra -Wpedantic \
  -Iso101_ros2/dapier_so101_core/include \
  so101_ros2/dapier_so101_core/src/realtime_control.cpp \
  so101_ros2/dapier_so101_core/test/realtime_control_contract_smoke.cpp \
  -o /tmp/dapier-realtime-contract-smoke
/tmp/dapier-realtime-contract-smoke

c++ -std=c++17 -Wall -Wextra -Wpedantic \
  -Iso101_ros2/dapier_localization_core/include \
  so101_ros2/dapier_localization_core/src/localization_runtime.cpp \
  so101_ros2/dapier_localization_core/test/localization_runtime_contract_smoke.cpp \
  -o /tmp/dapier-localization-contract-smoke
/tmp/dapier-localization-contract-smoke

scripts/verify-mujoco-headless --artifact-dir /tmp/dapier-mujoco-artifacts
```

TurtleBot3 Raspberry Pi 접속 설정은
[`2ARM_ROBOT/docs/TURTLEBOT3_REMOTE_ACCESS.md`](2ARM_ROBOT/docs/TURTLEBOT3_REMOTE_ACCESS.md)에
분리한다. 공개 저장소에는 계정명과 secret-free 템플릿만 두고 비밀번호는 넣지
않는다.
