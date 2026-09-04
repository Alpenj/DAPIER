# 이동형 양팔 SO-101 박스·신발 미션 개발 및 연구 기록

- 기준일: 2026-09-02 (Asia/Seoul)
- 작업 브랜치: `pro/mujoco-shoe-mission-01`
- 검토 PR: [#40](https://github.com/Alpenj/DAPIER/pull/40) — Draft, merge 금지
- 선행 PR: [#39](https://github.com/Alpenj/DAPIER/pull/39) — merge 완료
- 일정 원본: 비공개 일정 기록

> **2026-09-04 후속 안전 상태:** 이 문서의 과거 실측은 현재 실행 허가가 아니다.
> 저장소 physical motion과 camera streaming은 connected-endpoint identity binding,
> 독립 device-side watchdog, 카메라 serial/cryptographic binding을 local safety integration에서
> 확인할 때까지 비활성 상태다.

## 1. 목표 미션

1. 시작 위치에서 신발 박스가 있는 B 위치까지 visual SLAM 자율주행한다.
2. B 위치에서 정지·안정화하고 카메라·촉각·모터 상태의 freshness를 확인한다.
3. 오른팔로 박스 날개/뚜껑을 열고 열린 상태를 유지한다.
4. 왼팔로 박스 내부 신발을 인식하고 집어 꺼낸다.
5. 신발을 운반 자세로 들고 시작 위치로 복귀한다.
6. 시작 위치에서 신발을 내려놓는다.

MuJoCo에서는 신발을 직육면체 proxy로 먼저 구현한다. 박스 실측치는 28.2 × 21 × 10.5 cm,
판지 두께는 1.5 mm다. 실제 신발 mesh·마찰·변형은 직육면체 접촉 파이프라인을 통과한 뒤 추가한다.

## 2. 현재 구현과 검증 상태

### 원격 반영 완료

- ROS2-free mission core와 명시적 orchestrator
- mobility, visual SLAM, object pose, dual-arm manipulation 경계
- workspace RGB-D + 전방 SLAM RGB-D + 좌/우 gripper RGB 카메라 입력 경계
- 좌/우 SO-101 gripper FSR 입력과 접촉·미끄럼 판단 경계
- edge command sequence/freshness/ack, health alert, world state
- Raspberry Pi가 아닌 workstation-side local LLM supervisor
- MuJoCo mobile dual-SO101 adapter와 scripted IK shoe mission
- 실측 박스 scene, 오른팔 lid opening, 왼팔 cuboid shoe extraction prototype

### 테스트 증거

- 박스 미션 단위 테스트: 6/6 통과
- grasp planner 단위 테스트: 5/5 통과
- 최신 후속 실행: MuJoCo 단위 테스트 213/213과 model smoke 통과, hardware_execution=false
- 수정된 물리 판정: 오른팔 날개 contact 2, 뚜껑 95.05도, 왼 고정측 contact 1,
  왼 이동측 contact 1이지만 contact normal이 opposing 조건을 만족하지 않았다. shoe weld 없음,
  뚜껑·오른 moving jaw 및 신발·왼 gripper 쌍의 허용치 초과 penetration도 감지돼 success=false
- MuJoCo viewer에서 사용자가 몸에서 먼 방향으로 열리는 뚜껑 배치를 확인

### 남은 release gate

1. 왼 고정측과 이동측이 각각 접촉한 뒤 weld 없이 마찰로 들어 올려야 한다.
2. RGB-D pose와 gripper RGB 보정을 실제 grasp target에 연결해야 한다.
3. 관절별 actuator saturation fraction이 높아 속도·gain·limit 조정이 필요하다.
4. 정상·no-contact·collision·stale-camera run의 export, replay, timeline 시각화가 필요하다.
5. 물리·센서·지연·모터·FSR parameter sweep과 반복 성공률이 아직 없다.
6. 실제 판지 변형, servo current/temperature, backlash, FSR 값을 사용한 sim-real 검증이 필요하다.
7. 1,862줄 box prototype은 helper 중복과 boolean event 조합을 줄이는 구조 검토가 필요하다.

이전 contact 2건은 양쪽 손가락이 아니라 같은 고정측 collision mesh의 접촉점 두 개였다.
접촉 직후 shoe weld를 켜던 구현을 제거했으며, 현재 P2는 완료가 아니라 진행 중이다.
PR #40은 Draft 상태를 유지한다. 정적 IK residual이나 단일측 contact를 동적 파지 성공으로
간주하지 않는다.

## 3. 모듈 경계

| 모듈 | 책임 | 제외하는 책임 |
|---|---|---|
| `mobility` | base twist/pose, differential-drive 변환, 좌·우 wheel telemetry, 정지 확인 | 바퀴별 독립 mission state machine |
| `visual_slam` | 위치 추정, map/route 상태, B·start waypoint | 저수준 arm motion |
| `camera_io` | front RGB-D, left/right gripper RGB, timestamp·calibration·drop telemetry | LLM 판단 |
| `object_pose` | 박스·뚜껑·신발 pose와 confidence | motor command |
| `manipulator` | 좌/우 arm command, joint/limit/current/temp 상태 | navigation |
| `tactile` | arm당 FSR 1개, contact·force trend·slip/regrasp signal | 정밀 6축 force 추정 |
| `health` | network·motor·camera·compute 상태와 경고 | LLM의 임의 안전 해제 |
| `transport` | sequence, deadline, freshness, ack, bounded command | task reasoning |
| `world_state` | 동기화된 작업 상태 snapshot | 장치 직접 제어 |
| `local_llm_supervisor` | workstation에서 예외 설명·제안·알림 | Raspberry Pi 추론, 즉시 motor authority |
| `orchestrator` | 단계 전이, timeout, recovery, abort | 센서 드라이버 구현 |

이동 모듈은 바퀴별로 쪼개지 않는다. 다만 좌우 회전과 slip/고장 진단을 위해 wheel-level 명령·속도·전류
telemetry는 mobility 내부 계약으로 유지한다.

## 4. ROS2가 맡던 기능과 대체 경계

ROS2 자체가 문제라는 결론이 아니다. 양산 core에서 framework coupling을 줄이고, 장치 adapter는 ROS2와
native transport를 모두 허용한다.

| ROS2 책임 | 현재/목표 대체 | 상태 |
|---|---|---|
| package/build/deploy | 명시적 Python package, 이후 CMake/C++ production executor, 동일 build command | 부분 구현 |
| message contract/pub-sub | typed dataclass/protocol과 bounded queue | 구현 중 |
| service/action | command envelope, sequence, deadline, ack, cancel/timeout | 구현 중 |
| lifecycle | explicit mission state + fault latch + rearm | 구현 중 |
| parameters | versioned config와 startup validation | 보강 필요 |
| TF | timestamp가 포함된 frame graph와 calibration snapshot | 미구현 |
| rosbag/logging | immutable JSON/episode record, event log, artifact hash | 부분 구현 |
| QoS/discovery | freshness·drop·deadline·retry·backpressure를 명시한 native transport | 부분 구현 |
| RViz/introspection | log-derived timeline, contact/collision/latency chart | 미구현 |
| hardware drivers | ROS2 adapter 또는 vendor/native adapter | 실기 연결 필요 |

ROS2를 제거했다고 주장하려면 TF, replay, visualization, discovery/QoS, vendor driver의 대체가 모두 있어야 한다.
현재 단계는 `ROS2-free core + 교체 가능한 adapter`이며 완전 대체가 아니다.

## 5. 이동 학습과 IK 조합 판정

강의에서 말한 `IR`은 연구 문헌의 단일 표준 용어가 아니므로 RL, IL 또는 IL+RL 중 무엇인지 확인해야 한다.
현재 최선의 기본안은 다음 hybrid 구조다.

1. 장거리 이동은 visual SLAM + classical global/local navigation을 baseline으로 둔다.
2. 학습은 local approach, base placement, recovery에 제한적으로 도입한다.
3. B 위치에서 base를 stop/settle한 뒤 fresh RGB-D/RGB/tactile snapshot을 취득한다.
4. grasp pose 후보를 만들고 collision-aware IK + motion generation으로 양팔 경로를 계산한다.
5. 오른팔은 뚜껑 open/hold, 왼팔은 shoe grasp/extract를 수행한다.
6. tactile contact·slip과 motor current를 사용해 파지 상태를 닫힌고리로 보정한다.

현재 IK는 MuJoCo 양팔 경로에서 실행·검증 중이고, IL은 ACT 데이터·checkpoint·safety dry-run 경계까지 구현됐다.
실제 RGB-D 관측을 받은 학습 policy의 제한된 목표/보정 proposal을 양팔 IK와 충돌 검사에 연결한 뒤에만
`IL + IK 통합 완료`로 판정한다.

IK는 도달 가능한 joint 해를 제공하지만 충돌 없는 시간 경로와 접촉 성공을 보장하지 않는다. 따라서
IK만 단독 사용하지 않고 collision checking, trajectory optimization, contact feedback을 묶는다.

### 최근 5개년 근거

| 자료 | 확인한 핵심 | DAPIER 결정 |
|---|---|---|
| [ReLMoGen, ICRA 2021](https://svl.stanford.edu/projects/relmogen/) | RL이 subgoal을 만들고 motion generator가 저수준 실행 | 학습/기하 계획 역할 분리 참고 |
| [Error-Aware Imitation Learning, 2022](https://proceedings.mlr.press/v164/wong22a.html) | multi-stage mobile manipulation에서 error detection/recovery가 중요 | 단계별 failure label과 recovery 즉시 반영 |
| [N²M², TRO 2023](https://mobile-rl.cs.uni-freiburg.de/) | learned base/torso/EE velocity와 IK arm joint를 결합 | local approach 학습 + IK 후보 |
| [AnyGrasp, TRO 2023](https://doi.org/10.1109/TRO.2023.3281153) | 다수 6/7-DoF grasp pose 후보를 생성 | grasp 후보 생성 연구 후보 |
| [cuRobo, 2023](https://research.nvidia.com/publication/2023-05_curobo-parallelized-collision-free-robot-motion-generation) | IK, collision check, geometric planning, trajectory optimization 결합 | IK 단독 금지 근거 |
| [Mobile ALOHA, CoRL 2024](https://mobile-aloha.github.io/) | whole-body imitation learning과 target demonstrations | 충분한 실기 demo가 생긴 뒤 IL 실험 |
| [OK-Robot, 2024](https://ok-robot.github.io/) | modular perception/navigation/grasp primitive도 실제 집에서 pose/hardware failure 발생 | 모듈형 baseline과 실기 gate 채택 |
| [HomeRobot OVMM, 2023](https://ovmm.github.io/) | open-vocabulary mobile manipulation의 실제 성능 격차 | sim 성공률을 실기 성능으로 외삽 금지 |

## 6. sim-to-real 위험과 최소 기록

| 위험 | 수집 항목 | gate/recovery |
|---|---|---|
| network delay/jitter/drop | send/receive monotonic, age, RTT, sequence gap, timeout | stale command hold, bounded retry, alert |
| motor 상태 | current/load/temp/voltage/error, commanded vs measured joint | 속도 저감, safe pose, fault latch |
| backlash/compliance | joint error, EE error, direction reversal | approach margin, slow final phase, recalibration |
| wheel slip/localization drift | wheel odom vs visual pose residual | relocalize, approach retry, stop/settle |
| camera mismatch | intrinsics/extrinsics hash, exposure, depth validity, frame drop | invalid snapshot reject/alert, recalibration |
| time sync | sensor timestamp, host receive time, sync delta | unsynced sample 제외 |
| tactile drift | baseline, filtered ADC, contact threshold, derivative | startup tare, hysteresis, regrasp |
| contact/friction mismatch | geom pair, normal force, slip, friction randomization seed | domain randomization, contact-specific retry |
| compute/power | CPU/GPU/RAM, loop period, undervoltage | Raspberry Pi는 capture/edge execution만 수행 |

모든 실패는 `mission_id`, `phase`, `module`, `timestamp`, `severity`, `code`, `measured`, `limit`, `recovery`,
`artifact_path`를 포함하는 event로 기록한다. 팀 공유 시 timeline, command age, motor health, camera FPS/drop,
contact pair/force, IK residual, collision pair를 같은 run ID로 시각화한다.

## 7. 데이터 전처리와 학습

### raw

- front RGB-D, left/right gripper RGB
- base pose/velocity와 left/right wheel telemetry
- left/right arm joint command·measured state·current/temp/error
- left/right FSR raw ADC와 calibration revision
- object/box/lid/shoe pose, contact/collision event
- command send/ack timestamp, network health, mission phase

### derived

- synchronized observation window와 validity mask
- RGB normalization, depth unit/invalid mask, crop/resize metadata
- proprioception/action normalization은 train split 통계만 사용
- phase label: navigate/open/hold/grasp/extract/transport/place/recovery
- failure label: stale/collision/no-contact/slip/overload/localization-loss
- sim randomization parameter와 real calibration snapshot

raw는 수정하지 않고 derived artifact에 source hash, transform version, calibration revision을 남긴다. 학습용 split은
episode 단위로 나누며 같은 run의 frame이 train/validation에 동시에 들어가지 않게 한다.

## 8. 원 일정에 맞춘 완충 계획

원본 최종일 2026-11-04는 유지한다. 각 단계는 원래 종료일보다 2~3영업일 먼저 내부 동결하고 남은 기간을
실기·통합 실패 대응에 사용한다.

| 구간 | 원 일정 | 내부 완료 목표 | 수행·gate | 확보 버퍼 |
|---|---|---|---|---|
| prototype | 09-01~09-11 | 09-10 | 09-02 right lid, 09-03 left contact, 09-04 full sequence/log, 09-07~09 robustness, 09-10 module gate | 09-11 1일 |
| tuning | 09-14~09-25 | 09-23 | friction/latency/motor/camera randomization, data preprocessing, IL/RL local approach ablation | 09-24~25 2일/휴일 |
| integration | 09-28~10-09 | 10-07 | SLAM→stop/settle→dual-arm→return E2E, sim-to-real bench, recovery drill | 10-08 + 10-09 휴일 |
| feedback | 10-12~10-23 | 10-20 | 팀 피드백 반영, 10-21 release candidate freeze, 반복 실기 | 10-22~23 2일 |
| final | 10-26~11-04 | 10-30 | 보고서·Notion·영상·재현 명령 동결, 11-02 dress rehearsal | 11-03 비상 보완, 11-04 최종 |

### 이번 주 상세

| 날짜 | 목표 | 완료 조건 |
|---|---|---|
| 09-02 | 오른팔 lid open/hold와 코드 공개 | lid angle gate, 금지 collision 0, PR #40 |
| 09-03 | 왼팔 shoe contact/grasp/extract | 동적 contact > 0, lift displacement, slip gate |
| 09-04 | box sequence와 데이터 export | 재현 seed 10회, event/plot artifact |
| 09-07 | 양팔 coordination과 recovery | lid hold 중 left extraction, timeout/retry |
| 09-08 | 전처리 pipeline | raw→derived provenance, no split leakage |
| 09-09 | local approach 학습 baseline | classical 대비 동일 지표 ablation |
| 09-10 | prototype 내부 동결 | 151/151 + E2E acceptance evidence |
| 09-11 | 완충일 | 실기 연결·회귀·문서 보완만 수행 |

## 9. 시뮬레이션 담당 실행안

시뮬레이션 담당자의 1차 책임은 `시뮬레이션이 실행된다`가 아니라 `시뮬레이션에서 검증한 동작·데이터·실패 조건을
실기 담당자가 재사용할 수 있다`까지다.

### 책임 범위

| 책임 | 시뮬레이션 담당자가 할 일 | 완료 산출물 |
|---|---|---|
| MuJoCo scene | 실측 박스, lid/wing hinge, cuboid shoe, dual SO-101, camera/tactile site 유지 | versioned scene config와 headless load test |
| 양팔 task | right open/hold + left approach/grasp/extract 동시 제약 | deterministic mission script와 state/event trace |
| 접촉 물리 | 허용/금지 geom pair, friction, contact force, slip, lift 판정 | contact assertion과 collision report |
| 경로 계획 | 정적 IK 후보를 collision-aware dynamic trajectory로 전환 | residual·clearance·duration 비교표 |
| fault injection | network, motor, camera, tactile, localization failure 주입 | seed별 pass/fail matrix와 recovery trace |
| 데이터 수집 | sim observation/action/event를 raw schema로 export | episode manifest와 source/calibration hash |
| 전처리 | time sync, mask, normalization, phase/failure label, split | 재현 명령과 no-leak test |
| 시각화 | timeline, IK residual, contact, collision, latency, motor health | run ID 기반 HTML/PNG/JSON artifact |
| sim-to-real | 실기 측정값으로 mass/friction/backlash/latency 범위를 갱신 | sim/real gap report와 parameter revision |

### 이번 prototype 구간의 구체 작업

#### 09-02 · 공개·계약 정리

1. PR #40을 Hermes 검토 대상으로 유지하고 merge하지 않는다.
2. 카메라 5개 계약과 home-pose clearance 실패의 원인이 요구 변경인지 모델 오류인지 구분한다.
3. 오른팔 lid open angle, lid hold duration, 허용/금지 collision pair를 로그로 남긴다.
4. 결과를 10분 브리핑용으로 `현상 → 수치 → 원인 가설 → 다음 실험` 4줄로 정리한다.

완료 조건: `149/151`을 숨기지 않고 실패 test name·측정값·재현 명령을 PR에 남긴다.

#### 09-03 · 왼팔 접촉·파지

1. 기존 정적 접촉 후보를 3~5개 선정한다.
2. 후보마다 approach waypoint, wrist orientation, gripper opening, closing timing을 기록한다.
3. lid를 오른팔이 hold한 상태에서 left wrist의 금지 충돌을 제거한다.
4. `contact > 0`, `shoe lift displacement`, `slip`, `gripper force trend`를 성공 판정에 넣는다.

완료 조건: 한 번 보이는 성공이 아니라 고정 seed 10회 중 8회 이상 성공하고, 실패 2회의 event trace가 남아야 한다.

#### 09-04 · 전체 sequence와 데이터 export

1. navigate-arrived fixture → stop/settle → open/hold → grasp/extract → transport pose 순서를 한 run으로 묶는다.
2. front RGB-D, 양 gripper RGB, base/arm/tactile/contact/event를 같은 timestamp 축으로 export한다.
3. 정상 run과 no-contact/collision/stale-camera failure run을 각각 만든다.
4. 팀원이 한 명령으로 headless replay와 plot 생성을 수행할 수 있게 한다.

완료 조건: 동일 seed 재실행 시 state transition과 failure code가 같아야 한다.

#### 09-07~09-09 · 강건성·전처리·학습 baseline

1. mass, friction, box pose, shoe pose, camera noise/drop, network delay, motor lag/backlash를 범위화한다.
2. one-factor-at-a-time sweep로 민감도를 먼저 찾고, 이후 조합 randomization을 수행한다.
3. raw를 수정하지 않고 derived dataset을 생성하며 episode split leakage를 검사한다.
4. 이동은 classical baseline을 먼저 측정하고, local approach에만 RL/IL 후보를 같은 지표로 비교한다.
5. manipulation은 collision-aware IK baseline을 유지하고 학습 policy가 motor safety authority를 갖지 않게 한다.

완료 조건: baseline과 학습 후보가 동일한 start/goal, seed, latency 조건에서 비교돼야 한다.

#### 09-10~09-11 · 내부 동결과 완충

1. `151/151`, headless E2E, seed sweep, dataset provenance를 release checklist로 묶는다.
2. 실기 담당자와 joint mapping, camera serial/calibration, FSR tare/threshold, motor limit를 대조한다.
3. 09-10에 prototype을 내부 동결하고 09-11은 새 기능이 아니라 회귀·실기 차이 수정에만 사용한다.

### 팀원 인수인계 계약

| 상대 | 시뮬레이션 담당자가 받을 것 | 시뮬레이션 담당자가 줄 것 |
|---|---|---|
| SLAM 담당자 | `arrived_B`, base pose/covariance, stationary flag, relocalization failure | manipulation-ready pose tolerance, stop/settle 시간, base placement 실패 범위 |
| 실기 담당자 | joint zero/range, motor current/temp/error, camera serial/intrinsics/extrinsics, FSR raw sample | 동일 이름의 sim schema, expected trajectory, 안전 limit 후보, 실기 비교 plot |
| 전체 팀 | 요구 변경, 성공 기준, 현장 실패 영상/로그 | PR, 재현 명령, run artifact, 10분 브리핑, Notion 결정 기록 |

### 검증 및 기록 원칙

- contact/lift assertion과 재현 로그를 기준으로 완료를 판정한다.
- IK residual뿐 아니라 충돌, 동적 궤적, 실제 접촉을 함께 검증한다.
- friction, backlash, latency는 실측값과 calibration revision으로 관리한다.
- 진행 중인 실패와 미검증 범위를 PR과 단계별 기록에 명시한다.
- sim/real raw data를 분리 보존하고 derived artifact가 원본 hash를 참조하게 한다.
- LLM은 로컬에서 고수준 skill만 제안하며 최종 motor 명령은 독립 safety gate를 통과시킨다.

## 10. 바로 다음 작업 순서

1. PR #40의 2개 suite failure를 계약/실측 기준으로 해결한다.
2. 왼팔 접촉 가능한 정적 후보를 동적 trajectory에 반영하고 lid–wrist 금지 충돌을 제거한다.
3. contact/lift/slip을 assertion으로 가진 headless E2E test를 추가한다.
4. box prototype의 중복 helper와 boolean state를 typed event/state transition으로 축소한다.
5. latency/motor/camera/tactile fault injection과 run-level visualization을 구현한다.
6. MuJoCo raw/derived dataset export와 split/provenance test를 만든다.
7. USB gripper RGB와 FSR 실기를 연결한 뒤 동일 schema로 sim/real 차이를 측정한다.
