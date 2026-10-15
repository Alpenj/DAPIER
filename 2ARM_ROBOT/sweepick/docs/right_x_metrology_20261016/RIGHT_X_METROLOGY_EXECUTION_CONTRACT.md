# RIGHT_X_METROLOGY_EXECUTION_CONTRACT — A R0 (2026-10-16)

상태: **후보 구현, 무장치 검증만 완료.** 실물 START, Motor/Torque write, 장치 open을 하지 않았다. 활성 Native·Console에 배포하지 않았다. `REAL_J_VALIDATED=false`, `Correction=null`.

기준: A 통합 `pro/sweepick-review-20261015`(d527494), B Handoff `37a9668`. 후보 소스는 작업 브랜치 `pro/sweepick-right-x-metrology-20261016`에 있다.

## 1. 현재 A 경로의 사실 (P0)

| 경로 | 코드가 실제로 하는 일 | X 측정 판단 |
|---|---|---|
| `current_view_candidate.propose_receiver_inner_pad_view()` | 목표 1: 같은 세션의 RIGHT_APPROACH 전 tool pose로 가는 일반 SE(3) 변환(`inv(start) @ clear`, 회전 포함). 목표 2: 카메라 기하로 정한 tool X축 회전(roll). 두 목표 뒤 관측 1회(`FINAL_TARGET_ONLY`) | **X-only 측정 아님.** 재분류하지 않았고 이 함수는 바꾸지 않았다 |
| `Owner.plan_observation_candidate()` | `CURRENT_TOOL` 상대 4×4를 현재 실측 FK 위에 곱해 원하는 tool pose를 만든다. 그 위치·접근축(열 0)·jaw축(열 2)을 A IK(`HandoverTeacher.plan_move`)에 준다. jaw 명령이 바뀌면 거부, tick 반올림, `sweepick_trajectory.make`, 보유 물체를 넣은 `segment_check`(모든 표본), native `validate`, 정상 역경로(`plan_contact_suffix(normal_only)`)까지 확인한다 | **그대로 재사용.** 회전 없는 X 변환을 넣었다 |
| `Owner.segment(purpose="OBSERVATION_VIEW")` | 기존 executor로 1회 실행. 적분 trim 없음(`settle_trim=()`). settled·resolved·Status 0·Goal 일치·정착 실측 자세 충돌 검사. 실행한 이동은 `contact_observation_history`에 남아 정상 역경로가 되돌린다 | **그대로 재사용** |
| `acquire_insertion_observation()`의 observation-only 분기 | 위 receiver 후보 하나를 직접 만들어 한 번만 쓴다 | 이 분기로는 X 측정을 할 수 없다. 별도 단계를 추가했다(아래) |

**Tool X와 `X_receive`의 관계**
- Tool X는 `right_cube_grasp` site 회전의 열 0이다. A IK가 접근축으로 받는 축이 이것이다.
- A 모델(후보 mapping, `verified=false`)에서 계획된 RIGHT_APPROACH 이동은 tool +X와 1.5° 차이다. 그 시점의 보유 물체 중심은 tool 좌표로 +X 45 mm, Y 8 mm 앞에 있다. 따라서 **모델 기준으로 +X는 보유 물체 쪽(수신 접근 방향)**이다.
- 실제 물리 방향은 확인하지 않았다. 운영자는 `x_basis_tool`과 그 근거(`x_basis_source`)를 승인 입력으로 직접 적는다. 기본값은 없다.

**5DOF IK 판단**
- 저장된 REAL 실측 2세션, 시작 자세 2종에서 시험 이동 ±5/±10/±20 mm 24건이 모두 계획됐다. IK·충돌 CLEAR·native 실행 가능·정상 역경로 성립.
- 바뀐 관절은 shoulder_lift·elbow·wrist_flex뿐이다. pan은 ±2 tick, wrist_roll은 0–1 tick이다.
  - 이유: SO101의 tool X축은 pan이 정하는 팔 평면 안에 있다. 그래서 그 축을 따라 평행이동해도 pan과 roll을 바꿀 필요가 없다.
- 이 두 자세에서는 5DOF 때문에 불가능한 경우가 없었다. 다른 시작 자세에서는 현장 실측으로 다시 계획해야 하며, 불가하면 `REFUSED_PLAN`으로 끝난다.
- 자세한 수치는 `RIGHT_X_METROLOGY_IK_COLLISION_REPORT.json`에 있다.

## 2. 실행 범위 `RIGHT_X_FINITE_METROLOGY`

**별도 승인 범위다.** 다음 경우에는 열리지 않는다.
- 기존 Full-chain 승인(`RIGHT_RECEIVE_NONCONTACT_OBSERVATION`) 또는 LEFT_START_TO_LIFT 승인으로는 열리지 않는다.
- 준비된 plan 파일에 `right_x_metrology`가 들어 있으면 launcher가 거부한다. 설정은 RIGHT_X 승인에서만 온다.
- 승인과 Manifest의 scope가 모두 `RIGHT_X_FINITE_METROLOGY`여야 한다. Manifest는 launcher, `sweepick_full_chain.py`, `sweepick_pick01.py`, `sweepick_move_a.py`, `right_x_metrology.py`, `current_view_candidate.py`를 묶어야 한다.

**세션 순서**
1. 기존 체인 그대로 진행한다. LEFT PICK → CLOSE → LIFT → CARRY, RIGHT CARRY → RIGHT_APPROACH, 각 단계의 기존 관측.
2. **T 자세.** 운영자가 둘 중 하나를 고른다.
   - `AFTER_RIGHT_PREGRASP`: RIGHT_APPROACH 직후 자세. 기존 receiver 관측 이동은 실행하지 않는다.
   - `AFTER_ADOPTED_VIEW`: 기존 receiver 관측 이동을 기존 방식대로 승인하고 실행한 뒤의 자세. 그 이동과 새 관측이 실제로 없었으면 `REFUSED_T_POSE`로 끝나고 측정은 하지 않는다.
3. **T 관측.** 새 실측과 세 카메라를 기록한다. 이동은 없다.
4. **F 이동.**
   1. 최신 실측을 다시 읽는다.
   2. 왼팔 보유와 bus를 확인한다.
   3. `x_basis_tool × fit_displacement_m`의 회전 없는 CURRENT_TOOL 변환을 기존 경로로 계획한다.
   4. 운영자가 그 계획 1건을 명시적으로 승인한다.
   5. `segment(OBSERVATION_VIEW)`로 1회 실행한다.
   6. 다시 실측하고 보유를 확인한다.
   7. F 관측을 기록한다.
5. **V 이동.** F의 실측 끝점에서 `verify_displacement_m`만큼 같은 방식으로 이동하고 관측한다. V는 fit에 넣지 않는다.
6. **기존 정상 역경로.** 실행된 X 이동을 역순으로 되돌린다(`OBSERVATION_BACK_*`). 이어서 RIGHT_BACK_TO_CARRY → RIGHT_RETURN_INITIAL(오른팔 Torque OFF) → LEFT_BACK_TO_LIFT를 진행한다.
7. **기존 내려놓기.** lower → open(책상 위에서 왼 그리퍼 열림) → retreat → home(왼팔 Torque OFF).

측정 구간의 조작은 RIGHT ARM5 비접촉 이동 F·V 두 번뿐이다. 다음은 측정에 끼워 넣지 않았다.
- 오른 그리퍼 명령, RIGHT INSERT, RIGHT CLOSE
- LEFT RELEASE·하중 이전
- 영상 기반 보정 명령
- servo 설정 write
- 자동 재시도·재접속·회수

`segment()`가 observation-only 범위에서 RIGHT_INSERT_REPLAN과 RIGHT_CLOSE를 거부하는 기존 장치는 그대로다.

## 3. 끝점 기록 (`RIGHT_X_METROLOGY.json`, 매 끝점마다 저장)

| 항목 | 기록 내용 |
|---|---|
| 실측 | 양팔 ticks·goal·torque·status, `bus_seq`·`t_bus_monotonic`·`cycle_start`, q12 |
| Opening | 양 그리퍼 `Present_Position`·`Goal_Position` raw tick. mm 간격은 UNKNOWN |
| 영상 | 기존 `decision_observations/<seq>_RIGHT_X_<role>/`에 Right·Left Wrist RGB, Top RGB·depth, 원래 `OBSERVATION.json`을 저장한다. 각 카메라의 `n`·receipt 시각은 카메라 자기 값이다 |
| 시각 | 카메라 receipt 시각과 bus read 시각을 따로 둔다. 같은 시각으로 맞추지 않는다 |
| 명령과 실제의 분리 | `command.commanded_tool_displacement_m`(승인 값), `command.command_fk_from_planning_actual`(반올림된 명령 tick의 FK), `actual_fk_from_previous`·`actual_fk_from_T`(실측 tick의 FK) |
| 혼입 | 각 변위를 승인 축 성분(`along_x_m`), 축 밖 성분(`off_axis_tool_m`·`off_axis_norm_m`), 회전(`rotation_deg`)으로 나눈다. 모두 후보 FK이며 물리 측정이 아니다 |
| Jacobian | `delta_u_px=null`, `J_uX_px_per_mm=null`, `REAL_J_VALIDATED=false`, `validated=false`, `Correction=null`. 목표 영상과 u 특징이 미확정이기 때문이다 |

## 4. 끝나는 방식

| 상황 | 결과 | 이후 |
|---|---|---|
| T·F·V 기록 완료 | `ENDPOINTS_RECORDED` | 기존 정상 역경로 |
| 계획 불가(IK·충돌·native·역경로) | `REFUSED_PLAN`, 그 이동은 쓰지 않음 | 기존 정상 역경로 |
| 운영자 미승인, 승인 함수 없음 | `DECLINED` | 기존 정상 역경로 |
| 새 관측 누락(세 카메라 중 하나라도) | `OBSERVATION_MISSING`, 다음 이동 없음 | 기존 정상 역경로 |
| 설정 오류, T 자세 불일치 | `REFUSED_CONFIG` / `REFUSED_T_POSE`, 읽기·이동 없음 | 기존 정상 역경로 |
| STOP, Servo/Bus fault, 왼팔 보유 미확인, 승인 대기 중 보유·bus 상실 | 예외 → `STOPPED_BY_FAULT` 기록 | 기존 Owner 중단 계약: 자동 복귀 없음, `on_stop`, 운영자 수동 회수 |
| 정상 역경로 계획 실패 | 기존 계약 그대로 `FAILED`, `returned=false` | 운영자 수동 회수 |

**이동 요청 파일.** 매 이동은 새 `OBSERVATION_MOVE_REQUEST.json`(scope `RIGHT_X_FINITE_METROLOGY_MOVE`)이 된다.
- 이전 요청에 대한 결정 파일(다른 digest)은 이번 결정으로 읽지 않는다. 계속 기다린다.
- 다른 scope로 된 결정은 승인이 아니다.
- START 사전 승인은 X 이동에 적용되지 않는다.

## 5. 바꾼 파일 (후보)

| 파일 | 변경 |
|---|---|
| `revision/right_x_metrology.py` (신규) | 순수 계약: 설정·승인 검증, 회전 없는 X 변환, 선언 write 목록 |
| `revision/sweepick_full_chain.py` | `run_right_x_metrology()`, `right_x_endpoint()`, `x_split()` 추가. observation-only 분기의 `AFTER_RIGHT_PREGRASP` 건너뛰기. `_approach`에서 측정 호출과 결과 기록 |
| `RUN_APPROVED_SESSION.py` | `RIGHT_X_FINITE_METROLOGY` scope, 승인·Manifest 검증, X 이동 승인 함수, 이전 결정 파일 무시 |

바꾸지 않은 것:
- STOP, Collision(`segment_check`), Calibration, IK(`plan_move`)
- move_a, gripper 로직
- `current_view_candidate.py`, Console

**새 안전 기준을 만들지 않았다.** 이동 거리·속도·허용오차 상한이 없다. 크기와 축은 운영자의 승인 입력이다.
