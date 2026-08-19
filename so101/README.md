# SO-101 작업 허브

이 페이지는 SO-101 관련 코드와 실험 기록이 어디에 있는지 찾기 위한 인덱스다.
세부 실행 로그와 수치는 `records/`에 그대로 두고, 여기에는 지금 유효한 판단과
읽는 순서만 정리한다.

최종 목표는 한 팔 카드 pick/place를 안정화한 뒤 두 번째 팔을 추가하는 것이다.
현재 저장소에 있는 SO-101 결과는 아직 **단일 팔·단일 cube task**가 중심이며,
카지노 카드나 양팔 데이터로 해석하지 않는다.

## 현재 판단

저장소에 남아 있는 최신 실물 기록은 2026-08-13이다.

| 항목 | 현재 판단 |
|---|---|
| sim-first 기반 | `dapier_sim_first`의 G0 계약 검증과 G1 scripted pick-and-lift는 통과했다. 둘 다 실물 또는 learned policy 결과는 아니다. |
| 선택한 simulation policy | 기존 wrist-only v2 checkpoint를 유지한다. contact model을 바로잡은 뒤 unseen seed `2100..2119`에서 `11/20`이었고 80% release gate는 닫혀 있다. |
| corrected IK 데이터 | 초기 30 episode와 후속 100 episode 수집은 bilateral contact·1 mm penetration gate를 통과했다. 데이터 수집 성공과 student policy 성공은 별개다. |
| 재학습 결과 | new-from-base, v2 correction pass, 160-episode mixed rehearsal에서 만든 checkpoint는 모두 기준선을 넘지 못해 선택하지 않았다. |
| 실행·증거 도구 | action smoothing, human intervention, parallel rollout은 제어 안정화·교정 기록·실패 수집용이다. 정책 성능 향상이나 학습 완료를 뜻하지 않는다. |
| 실물 연결 | leader/follower USB serial board와 각 bus의 motor ID 1~6 응답은 확인했다. 첫 follower calibration은 range 기록이 즉시 끝나 실패했으며 teleoperation은 계속 금지 상태다. |
| 양팔 카지노 딜러 | planner와 one-card 기구학 baseline만 있다. 실제 카드, 흡착, 두 팔 충돌, 양팔 dataset은 아직 검증하지 않았다. |

이전 contact model에서 얻은 held-out `14/20` 두 세트는 현재 대표 성능으로 쓰지
않는다. 이후 성공 replay에서 최대 약 8.3 mm의 cube penetration이 측정됐고,
visible pad/cube contact와 1 mm gate를 추가한 뒤 같은 v2 정책을 다시 평가한
`11/20`을 현재 판단에 사용한다.

## 코드와 문서의 역할

| 위치 | 역할 | 주의할 점 |
|---|---|---|
| [`../dapier_sim_first`](../dapier_sim_first/README.md) | ROS 2와 실기체 없이 관절·단위·frame 계약과 G0/G1 Gate를 검증 | LeRobot runtime이나 실물 driver가 아니다. |
| [`ros2_ws`](ros2_ws/README.md) | DAPIER가 직접 소유하는 ROS 2 Jazzy core와 mock 안전 teleop | 실제 motor bus, torque control, policy bridge는 아직 없다. |
| [`integrations/lerobot_v0_6_so101_mujoco`](integrations/lerobot_v0_6_so101_mujoco/README.md) | LeRobot v0.6.0 checkout에 적용하는 MuJoCo overlay와 patch 보존본 | 독립 Python package가 아니며 upstream revision과 asset hash를 맞춰야 한다. |
| [`hardware_tools`](hardware_tools/README.md) | read-only inventory와 calibration·복구 도구 | `read_only`도 serial port를 연다. `writes_hardware`는 EEPROM을 바꿀 수 있다. |
| [`../casino_dealer`](../casino_dealer/README.md) | CardBench 계약, blackjack planner, episode manifest, one-card 기구학 baseline | 현재 SO-101 실물 실행 패키지가 아니다. |
| [`../docs/SO101_CASINO_DEALER_RUNBOOK_KO.md`](../docs/SO101_CASINO_DEALER_RUNBOOK_KO.md) | calibration부터 episode 검수·정책 평가까지 사람이 따라 하는 순서 | 체크되지 않은 단계는 완료로 보지 않는다. |
| [`../project-planning/2026-08-07-so101-sim-to-real-foundation.md`](../project-planning/2026-08-07-so101-sim-to-real-foundation.md) | Gate별 증거 수준과 중단 조건 | simulation metric을 physical threshold로 복사하지 않는다. |
| [`../project-planning/2026-08-11-robot-control-stack-concept-adoption.md`](../project-planning/2026-08-11-robot-control-stack-concept-adoption.md) | 동기식 step, post-action readback, offline digital-twin metric 설계 | 외부 runtime을 가져온 것이 아니라 필요한 개념만 독립 구현했다. |

`dapier_sim_first`와 `so101/ros2_ws`는 합치지 않는다. 앞쪽은 MuJoCo 기반의 순수
검증 경로이고, 뒤쪽은 ROS message와 안전 경계를 다루는 경로다.
`dapier_sim_first/digital_twin.py`도 두 runtime을 제어하는 계층이 아니라,
각 경로에서 내보낸 trace를 읽기 전용으로 비교하는 평가기다.

## 실험 기록을 읽는 순서

### 1. 환경과 관측·행동 계약

- [`통합 조사`](records/2026-08-07-so101-consolidation.md): 외부 checkout,
  DAPIER 정본, workspace symlink의 경계를 확인한 기록
- [`interactive sim`](records/2026-08-07-so101-interactive-sim-controls.md):
  viewer 입력 충돌을 피한 Shift chord와 scripted lift 검증
- [`wrist vision pick/place`](records/2026-08-07-so101-wrist-vision-pick-place.md):
  visible geometry, wrist RGB, cube 위치 추정과 pick/place 실험
- [`camera·IK·VLA routing`](records/2026-08-07-so101-camera-routing.md):
  top+wrist IK teacher와 wrist-only student의 입력 경계

이 단계의 핵심은 카메라와 controller route를 분리하고, privileged top image가
student dataset에 남지 않는지 확인한 것이다.

### 2. 첫 VLA 기준선과 실패 분석

- [`bounded VLA·casino baseline`](records/2026-08-10-so101-vla-casino-completion.md)
- [`VLA failure analysis`](records/2026-08-10-so101-vla-failure-analysis.md)

reset pose와 action horizon을 맞추면서 초기 20%보다 성능은 개선됐지만,
서로 다른 held-out 20-episode 세트에서 각각 70%에 머물렀다. 이 결과가 기존
v2 checkpoint를 선택하게 된 배경이지만 현재 release metric은 아니다.

### 3. 실행 안정화와 교정 evidence

- [`action smoothing과 RCS trace`](records/2026-08-11-so101-vla-action-smoothing.md):
  chunk-boundary jump를 줄였지만 같은-seed 성공 수는 바뀌지 않았다.
- [`human intervention`](records/2026-08-11-so101-vla-intervention.md):
  policy/human authority 전환과 source-labeled evidence를 만들었다. 출력은 아직
  native LeRobot dataset이 아니다.
- [`parallel rollout`](records/2026-08-11-so101-parallel-rollout.md):
  4 worker가 더 빠르지 않았으므로 성능 평가는 1 env로 유지하고, 병렬 경로는
  실패 trace 수집에만 사용한다.

이 세 작업은 하나의 “성능 향상”으로 합치지 않는다. smoothing은 command 품질,
intervention은 교정 기록, parallel rollout은 experience 수집 문제를 다룬다.

### 4. contact correction과 재학습

- [`corrected IK와 retraining`](records/2026-08-11-so101-corrected-ik-retraining.md)
- [`v2 + corrected IK mixed rehearsal`](records/2026-08-12-so101-v2-corrected-mixed-rehearsal.md)

기존 27% gripper target의 깊은 penetration을 renderer 문제로 넘기지 않고 contact
pair와 teacher grasp를 수정했다. corrected teacher 데이터는 수집 gate를 통과했지만,
새 student는 long-horizon phase와 bilateral contact 유지에 실패했다.

후속 mixed rehearsal은 기존 v2 60 episode와 corrected IK 100 episode를 합쳐
160 episode, 105,600 frame으로 다시 열리는 것까지 확인했다. 같은 unseen seed에서
기존 v2 baseline은 `5/10`, mixed checkpoint들은 최대 `3/10`이어서 어느 것도
승격하지 않았다. 현재 선택은 **기존 v2 policy + corrected contact model**이다.

### 5. 실물 연결과 calibration

- [`실물 연결과 이중 calibration 경로`](records/2026-08-13-so101-calibration-paths.md)
- [`hardware tools 사용법`](hardware_tools/README.md)

두 serial bus와 motor ID 응답은 확인했지만 첫 follower calibration은 완료되지
않았다. 안전 wrapper는 torque-off 확인, joint별 최소 span, 조기 Enter 실패 종료,
취소 처리와 마지막 torque-off를 강제한다. LeRobot를 사용하지 않는 직접 SDK
경로는 기본 선택이 아니라 inspect·감사·복구용이다.

## 로컬 작업 공간

현재 권장 LeRobot checkout 위치는
`$DAPIER_ROOT/.local-workspaces/so101/lerobot`이다. 오래된 기록에는
`$HOME/so101/lerobot` 경로가 남아 있으므로 명령을 그대로 복사하기 전에 현재
checkout 위치를 확인한다.

```text
$HOME/DAPIER/
├── dapier_sim_first/                 # sim-first 정본
├── so101/                            # 개인 SO-101 코드 허브
│   ├── ros2_ws/src/                  # 자체 ROS 2 패키지와 colcon workspace
│   ├── hardware_tools/               # 읽기 전용 진단 / 쓰기 도구
│   ├── integrations/                 # 외부 patch와 overlay
│   ├── docs/                         # 자체 ROS 2 설계 문서
│   └── records/                      # 날짜별 검증 기록
└── .local-workspaces/so101/lerobot/  # Git에서 제외한 upstream 실행 checkout
```

LeRobot upstream 전체, `.venv`, `build/install/log`, 원시 Dataset v3, 영상,
카메라 image, 실제 serial ID와 calibration JSON은 Git에 넣지 않는다. 저장소에는
재현에 필요한 revision, contract, metric, hash와 사람이 읽을 수 있는 기록만
남긴다.

## 다음 Gate

1. follower와 leader calibration을 각각 끝내고 다른 경로의 `inspect`로 교차 확인한다.
2. 두 JSON의 schema·motor ID·normalized pose를 비교하고 비슷한 시작 자세를 맞춘다.
3. emergency stop과 current limit를 준비한 뒤 한 관절씩 저속 방향을 확인한다.
4. wrist camera의 실제 extrinsic, image rotation과 FOV를 측정한다.
5. 한 장 pick/place를 사람이 조작해 영상·측정 state·action을 함께 기록한다.
6. 실물 episode를 검수한 뒤에만 replay와 짧은 policy rollout을 진행한다.
7. 한 팔 기준선이 안정된 뒤 두 번째 팔의 deck 고정 역할을 추가한다.

AI는 반복 코드, 테스트 초안, 실행 명령 정리와 로그 비교에 사용했다. 최종 상태는
커밋된 코드, 저장된 record와 직접 실행한 결과를 기준으로 판단하며, 새 기능이
추가됐다는 이유만으로 학습 성공이나 실물 준비 완료로 올리지 않는다.
