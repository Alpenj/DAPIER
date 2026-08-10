# Robot Control Stack 핵심 개념의 DAPIER 채택 검토

`record_id: DAPIER-2026-08-11-rcs-concept-adoption`

## 결론

Robot Control Stack(RCS)을 DAPIER의 새 런타임으로 도입하지 않는다. 대신 RCS가 잘 드러내는 **동일한 sim/real 계약, 합성 가능한 환경 계층, 동기식 step 의미, digital-twin 비교, 좌표·단위 규약, 정책 프로세스 분리**를 DAPIER가 직접 소유하는 계약과 검증기로 채택한다.

이번 적용에서 실제로 추가한 것은 두 가지다.

1. G1 manifest가 `synchronous`, `post_action_readback`, `absolute_target` 실행 의미를 digest에 포함한다.
2. `dapier_sim_first.digital_twin`이 command/simulation/physical readback trace를 오프라인에서 비교해 joint별 MAE, RMSE, p95, endpoint error, command delay와 timestamp skew를 계산한다.

이 비교기는 serial port를 열거나 팔을 움직이지 않는다. 실측 threshold가 없으면 결과를 `PASS`가 아니라 `MEASURED`로 남긴다. 따라서 RCS의 digital-twin 발상을 가져오되 simulation 결과를 real 안전 인증으로 오해하지 않는 DAPIER의 기존 증거 경계를 유지한다.

## 조사 범위와 고정점

- 프로젝트 페이지: [Robot Control Stack — A Lean Ecosystem for Robot Learning at Scale](https://robotcontrolstack.github.io/)
- 공식 문서: [Robot Control Stack 0.7.2](https://robotcontrolstack.org/)
- 논문: [arXiv:2509.14932v2](https://arxiv.org/abs/2509.14932), 2026-03-10 개정, ICRA 2026 채택
- 코드: [RobotControlStack/robot-control-stack](https://github.com/RobotControlStack/robot-control-stack), 조사 checkout `f6606e2b9c2737cf1bd2bbab9e6d26a191785333`
- 정책 프로세스 분리: [RobotControlStack/vlagents](https://github.com/RobotControlStack/vlagents)
- DAPIER 기준: `main` `3b5266db...`, 2026-08-11 fetch 후 감사
- DAPIER Notion 근거: SO-101 자체 ROS 2 스택, SO-101 IK→VLA·카지노 최신 기록, 이미테이션 러닝 로드맵, VoLN-UAV·URF 검토

RCS 코드는 AGPL-3.0이고 `assets/` 일부는 개별 라이선스를 유지한다. 별도 VLAgents 저장소는 Apache-2.0이다. 이번 DAPIER 변경에는 RCS 소스나 asset을 복사하지 않았고 공개 문서와 코드에서 확인한 **설계 개념만 독립 구현**했다.

## RCS를 깊게 읽은 결과

### 1. sim과 real을 같은 step 계약으로 다룬다

RCS의 고수준 API는 Gymnasium의 `reset()`과 `step(action)`이다. base environment 아래에는 MuJoCo 또는 실제 robot interface가 있고, 위쪽 application은 같은 action/observation 계약을 사용한다. 기본 step은 synchronous라 target state에 도달한 뒤 observation을 돌려주며, teleoperation처럼 기다리지 않아야 하는 경우에는 asynchronous mode를 선택한다.

핵심은 “Gymnasium을 쓴다”가 아니라 **action이 언제 실행 완료로 간주되고 반환 observation이 어느 시점의 상태인지 계약에 적는 것**이다. DAPIER G1도 실제로는 action을 적용하고 MuJoCo substep을 진행한 뒤 measured readback을 기록하지만, 이전 manifest에는 이 의미가 명시되지 않았다.

### 2. 기능을 wrapper로 합성한다

RCS는 robot, gripper, camera, relative action, recorder를 wrapper로 쌓는다. 각 wrapper는 observation 변환 `f: S → S'` 또는 action 변환 `g: A' → A`를 담당한다. 이 방식은 camera나 gripper를 추가할 때 base robot driver를 다시 작성하지 않게 한다.

DAPIER가 채택할 핵심은 Gym wrapper class 자체가 아니다. 다음 책임을 서로 다른 변환/observer로 유지하는 것이다.

- backend adapter: MuJoCo, ROS 2, physical follower
- action transform: absolute/delta, degree/radian, clipping 거부
- observation transform: measured state, image key, timestamp alignment
- transition observer: dataset recorder, evaluator, safety audit
- application: virtual leader, scripted expert, policy rollout

현재 `SO101MujocoEnv`, ROS 2 backend, `Frame` validator와 episode writer가 이미 이 책임의 대부분을 가진다. 따라서 새 wrapper framework를 추가하기보다 기존 경계가 섞이지 않도록 문서와 contract test를 강화하는 편이 낫다.

### 3. 좌표와 단위를 빠짐없이 공개한다

RCS 문서는 다음을 한 페이지에 고정한다.

- robot frame: 오른손 좌표계, x 전방, y 좌측, z 상방
- Cartesian translation: meter
- joint/Euler angle: radian
- RCS quaternion: `xyzw`
- MuJoCo free-joint `qpos`: `wxyz`
- attachment frame와 추가 `tcp_offset` 분리
- gripper: 0 closed, 1 open

DAPIER의 SO-101 공개 action은 LeRobot과 맞춘 arm degree + gripper 0..100이고, ROS 2 경계에서만 radian으로 변환한다. 그러므로 RCS의 0..1 gripper 값을 DAPIER 정본으로 바꾸지 않는다. 대신 **각 backend adapter가 어떤 외부 규약을 DAPIER 계약으로 변환하는지** 명시하고 round-trip test로 고정한다.

### 4. dual-arm은 shared base frame으로 구성한다

RCS scene 구성은 world → root → shared base → per-robot base → attachment/TCP 순서를 사용한다. dual-arm 장면은 왼팔·오른팔을 하나의 shared base frame에 배치하고 전체 rig는 root transform으로 함께 이동한다.

이 개념은 CardBench에 직접 유용하다. 앞으로 G6 scene은 `left/right` joint namespace만 두는 데서 끝나지 않고 다음 transform을 manifest에 포함해야 한다.

- `table_to_world`
- `shared_base_to_table`
- `left_base_to_shared_base`
- `right_base_to_shared_base`
- 각 팔의 `attachment_to_tool`
- overhead/wrist camera extrinsic과 frame ID

다만 현재 CardBench는 결정론적 kinematic baseline이고 dual-arm MuJoCo dynamics가 없다. 이번 변경에서는 frame schema를 설계 권고로만 남기고 존재하지 않는 simulation 성공을 주장하지 않는다.

### 5. 기록은 read-only observer여야 한다

RCS 논문은 observation과 action을 30 Hz에서 시간 정렬해 Parquet으로 기록했다고 설명한다. 코드의 `StorageWrapper`는 transition을 비동기 batch shard에 먼저 쓰고 정상 종료 때 consolidate하는 crash-safe 전략을 사용한다.

DAPIER는 이미 measured state와 commanded action을 구분하고 immutable manifest/receipt, LeRobotDataset 재로딩 검증을 수행한다. 추가로 배울 부분은 recorder가 control 의미를 바꾸지 않는 read-only observer여야 한다는 점과 장시간 수집에서 append-only shard로 crash loss 범위를 제한하는 점이다. 이것은 G2 human demonstration을 구현할 때 P1 과제로 둔다.

### 6. policy 환경을 robot 환경과 분리한다

RCS 논문과 VLAgents는 robot/simulation runtime과 GPU policy runtime의 dependency 충돌을 별도 프로세스와 RPC로 격리한다. 같은 PC에서는 shared memory, 원격에서는 TCP를 사용할 수 있고 action chunk 중 몇 step을 실행할지 execution horizon을 제한한다.

DAPIER 설계에도 policy server, stale action, watchdog, action chunk expiry가 이미 계획돼 있다. RCS가 이 방향의 독립 근거를 보강한다. 그러나 현재 SO-101 wrist-only VLA는 local bounded evaluation 단계이고 real rollout gate를 통과하지 못했다. 따라서 이번에는 네트워크 runtime을 추가하지 않고 다음 구현의 요구사항만 확정한다.

- policy response에 schema/config/checkpoint digest 포함
- inference start/end, queue age, chunk horizon 기록
- timeout 또는 disconnect 시 last action 무한 반복 금지
- action chunk 일부 실행 후 새 observation으로 재계획
- robot process가 raw policy output을 직접 motor bus에 전달하지 않음

### 7. 논문 결과는 설계 근거이지 DAPIER 성능 약속이 아니다

논문은 SO-101 leader-follower 시연 120개를 수집했고 π0 real rollout 성공률을 0.62로 보고한다. 저자들은 낮은 DoF, gripper fixed-finger 쪽 충돌, link deflection과 motor backlash를 원인 후보로 제시한다. 이는 저가형 SO-101에서 data volume만 늘려도 문제가 해결된다고 가정하면 안 된다는 근거다.

FR3 scripted simulation에서는 3,000 episode 중 73%인 2,193개를 success filter로 남겼고, real+sim 혼합이 해당 실험의 real 성능을 개선했다. 그러나 simulation checkpoint 성능과 real 성능의 관계는 논문도 loose correlation 및 lower-bound 경향으로 표현한다. DAPIER는 이 결과를 그대로 목표치로 사용하지 않고, 동일 command/seed에서 sim/real gap을 먼저 측정한다.

## DAPIER 현재 상태와 중복 감사

| RCS 개념 | DAPIER 현재 상태 | 판정 |
|---|---|---|
| Gymnasium sim environment | `SO101MujocoEnv`, vector env, dual camera, deterministic reset 존재 | 이미 채택 |
| sim/real backend 교체 | ROS 2 bridge에 MuJoCo/follower backend 존재 | 이미 채택, protocol 명시 보완 후보 |
| measured observation / commanded action | G0/G1 Frame 및 LeRobot episode에서 구분 | 이미 채택 |
| exact unit/order/calibration | G0 contract, ROS radian adapter, calibration gate 존재 | 이미 채택 |
| synchronous/asynchronous 의미 | 코드 동작은 있으나 G1 manifest에 없음 | 이번 보완 |
| digital-twin 정량 비교 | 계획 문서만 있고 실행 가능한 metric module 없음 | 이번 구현 |
| recorder/replayer observer | G1 writer와 dataset 검증 존재, crash-safe shard/replayer는 부분 | G2 P1 |
| policy process isolation | 설계 문서에만 존재 | G3 이후 P1 |
| shared-base dual-arm scene | CardBench role split/kinematic baseline만 존재 | G6 P1 |
| sim checkpoint↔real correlation | wrist-only sim 70%, real 미실행 | hardware gate 이후 |

## 채택 우선순위

### P0 — 이번에 적용

1. **실행 의미가 포함된 manifest**
   - `step_mode=synchronous`
   - `observation_alignment=post_action_readback`
   - `action_reference=absolute_target`
   - `async_control=false`
   - control period를 input digest에 포함

2. **offline digital-twin metric**
   - exact joint order와 radian 단위
   - strictly increasing timestamp
   - 같은 sample count의 pre-synchronized trace만 입력
   - joint별 MAE/RMSE/p95/endpoint error
   - command 대비 sim/physical delay estimate와 delay gap
   - timestamp skew
   - threshold가 없으면 `MEASURED`; threshold가 명시된 경우에만 PASS/FAIL

3. **라이선스·증거 경계 문서화**
   - RCS source/asset 미복사
   - no-ROS 철학을 DAPIER ROS 2 대체 근거로 사용하지 않음
   - simulation metric을 hardware safety certification으로 승격하지 않음

### P1 — 다음 Gate에서 적용

1. G2 recorder를 append-only shard + clean consolidation 구조로 만들고 crash injection test를 추가한다.
2. transition observer interface로 recorder/evaluator/safety audit를 control backend와 분리한다.
3. G3 policy runtime을 별도 process로 분리하고 queue age, action expiry, execution horizon을 기록한다.
4. G6 dual-arm scene에 shared base와 per-arm/tool/camera transform manifest를 추가한다.
5. physical trace가 생기면 `digital_twin.py` 결과를 run receipt에 연결한다.

### 보류 또는 채택하지 않음

- RCS 전체 runtime, SO-101 extension 또는 asset의 직접 복사
- ROS 2 제거 또는 현재 `so101_ros2`의 대체
- DAPIER public gripper contract를 0..1로 변경
- 환경별 Python/OMPL/RealSense dependency를 DAPIER 기본 환경에 추가
- RCS의 `DigitalTwin` wrapper를 safety gate로 간주
- 논문의 90–120 Hz 처리량이나 VLA 성공률을 DAPIER 합격 기준으로 사용
- physical command를 simulation에서 먼저 실행했다는 이유만으로 안전하다고 판정

## 추가된 digital-twin 계약

`dapier_sim_first.digital_twin.JointTrace`는 다음을 요구한다.

| 필드 | 계약 |
|---|---|
| source | `command`, `simulation`, `physical` 중 정확히 하나 |
| contract_id | joint order·unit·adapter 의미를 고정한 공통 contract digest |
| source_revision | command plan, MuJoCo model, physical calibration의 source별 revision |
| joint_names | 비어 있지 않고 중복 없는 exact ordered tuple |
| timestamps_ns | nonnegative integer, strictly increasing |
| positions_rad | timestamp와 같은 길이, joint width exact, finite radian |
| sample alignment | upstream sync가 끝난 같은 sample index; evaluator가 임의 보간하지 않음 |

출력 schema는 `dapier.digital-twin.v1`이다. threshold는 실측 전 임의로 정하지 않는다. synthetic test에서는 알려진 1-step sim delay와 2-step physical delay를 넣어 1-step gap을 재현하고, joint order·NaN·timestamp·threshold 실패를 결함 주입한다.

이 첫 버전이 일부러 하지 않는 일도 명확하다.

- ROS bag/MCAP parsing
- timestamp interpolation
- camera extrinsic comparison
- friction/backlash parameter fitting
- hardware command
- safety certification

다음 단계에서는 observation sync가 만든 trace를 입력으로 연결하고, command trajectory와 physical/sim response에서 rise time, overshoot와 steady-state error를 추가한다.

## 검증 계획

```bash
python -m unittest dapier_sim_first.test.test_digital_twin -v
python -m unittest discover -s dapier_sim_first/test -v
python -m unittest discover -s casino_dealer/test -v
python -m compileall -q dapier_sim_first
ruff check dapier_sim_first
```

합격 조건은 다음과 같다.

- 기존 G0/G1 contract test가 모두 유지된다.
- 새 synthetic digital-twin test 6개가 통과한다.
- manifest 실행 의미가 input digest에 포함되고 exact mismatch가 거부된다.
- RCS code/asset이 DAPIER diff에 포함되지 않는다.
- Notion 기록이 이 문서와 GitHub commit/PR을 연결한다.

## 다음 실험에서 얻어야 할 증거

1. ROS 2 또는 CSV/MCAP에서 command/sim/physical trace를 같은 joint order와 radian으로 export한다.
2. 먼저 simulation 대 synthetic physical trace로 delay·offset 결함 주입을 반복한다.
3. 실제 장비는 read-only state와 승인된 저속 single-joint command가 준비된 뒤 별도 Gate에서만 수집한다.
4. 최소 세 번의 동일 command를 반복해 평균뿐 아니라 분산과 worst case를 기록한다.
5. 실측 분포를 본 뒤 threshold를 승인 manifest에 고정한다.
6. threshold 변경은 코드가 아니라 새 manifest revision으로 남긴다.

## 출처

### Robot Control Stack

- [프로젝트 페이지](https://robotcontrolstack.github.io/)
- [논문 HTML](https://arxiv.org/abs/2509.14932)
- [논문 PDF](https://arxiv.org/pdf/2509.14932)
- [공식 코드](https://github.com/RobotControlStack/robot-control-stack)
- [공식 문서 — Architecture](https://robotcontrolstack.org/user_guide/architecture.html)
- [공식 문서 — Gymnasium Interface](https://robotcontrolstack.org/user_guide/gym_interface.html)
- [공식 문서 — Conventions](https://robotcontrolstack.org/user_guide/conventions.html)
- [공식 문서 — Sim Scene Configuration](https://robotcontrolstack.org/user_guide/scene_configuration.html)
- [공식 문서 — SO101 Extension](https://robotcontrolstack.org/extensions/rcs_so101.html)
- [VLAgents](https://github.com/RobotControlStack/vlagents)

### DAPIER

- [SO-101 작업 허브](../so101/README.md)
- [SO-101 sim-first G0–G1](../dapier_sim_first/README.md)
- [SO-101 sim-to-real foundation](2026-08-07-so101-sim-to-real-foundation.md)
- [LeRobot/ROS 2 분해 설계](2026-08-06-dapier-lerobot-ros2-deconstruction-lab.md)
- [SO-101 VLA 실패 분석](../so101/records/2026-08-10-so101-vla-failure-analysis.md)
