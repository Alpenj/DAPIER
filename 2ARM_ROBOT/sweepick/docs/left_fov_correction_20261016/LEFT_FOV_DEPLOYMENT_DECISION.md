# LEFT_FOV_DEPLOYMENT_DECISION — 왼팔 시야 유지 보정 배포 판단 (2026-10-16)

상태: **최소 구현과 무장치 회귀 완료.** 활성 Native와 라이브 Console에는 적용하지 않았다. 실물 START, 장치 Open, Motor/Torque write를 하지 않았다. 배포 여부는 사용자가 정한다.

## 1. 무엇이 바뀌나

**P0-1 독립 조건.** 집기 전 자세(`ready_to_descend`)에서 두 조건을 따로 평가한다.
- 기존 Gate 통과: 기존 판정 그대로
- 다음 하강 뒤 시야 유지: `fov_view_check`

두 조건이 갈리는 경우는 이렇다.
- 기존 Gate가 통과했는데 시야 위험(`FOV_RISK`)이면 `ALIGNED`를 반환하지 않는다. 같은 루프 안에서 기존 lateral 오차를 줄이는 보정을 한다.
- 보정 경로는 기존 1축 보정 그대로다: `correct()` → `plan_pick`(IK·경로 충돌) → stage `align`(궤적·충돌 인증·Servo 검사) → 다시 관측.
- 기존 함수를 단순히 다시 부르지 않는다. 다시 부르면 Gate 통과로 바로 `ALIGNED`가 되기 때문이다.

**보정의 성격.** 새 파지 목표를 만들지 않는다.
- 기존 `lateral_target_mm`까지의 오차를 줄인다. 즉 기존 파지 목표를 더 정확히 맞추는 방향이다.
- 파지 자세(tilt, jaw)는 그대로다.

**P0-3 유한한 동작.**
- **높이:** 집기 전 자세에서만 보정한다(`FOV_GATES = ("ready_to_descend",)`). 블록 위 정지 높이와 CLOSE 직전 깊이에서는 시야 보정으로 움직이지 않는다.
- **끝나는 조건(모두 기존 조건):**
  - 다음 관측에서 오차가 영상 분해능보다 줄지 않음
  - 영상이 이동과 반대로 움직임
  - `plan_pick` 또는 stage가 거부함
- **추가한 거부 조건(새 수치 없음):**
  - 시야 위험인데 이 관측의 offset이 확인되지 않음 → `NOT_ALIGNED`
  - 계산된 보정량이 정확히 0 → `NOT_ALIGNED`
- **결과.** 위 조건으로 끝나면 기존 복귀 계약(`NOT_ALIGNED_RETURNED_HOME`)을 따른다. Bus 장애나 STOP이면 기존 HOLDING 계약을 따른다.
- **바꾸지 않은 것.** 새 거리·토크·속도·허용오차·충돌 여유값은 없다. `ready_to_close` 조건도 완화하지 않았다.

**P0-2 예측 입력 분리.**
- 영상 위치는 원시 offset(`jaw midline + offset_mm`)으로 예측한다.
- 보정량은 `lateral_error_mm`(offset − `lateral_target_mm`)로 계산한다.
- 영상 경계 판정은 기존 `wrist_object`의 `touches_border` 정의를 그대로 쓴다.
- 기준 파일 `sweepick_left_fov_reference.json`은 최종 관측이 읽힌 저장 실물 기록 17건의 중앙값이다. 허용치가 아니다.

## 2. 근거와 한계

| 항목 | 결과 |
|---|---|
| Fake 시험 (P0-4 9개 경우) | 9개 통과. 기능을 끄면 그중 8개가 실패한다(남은 1개는 기존 Gate 실패 경로라 기능과 무관) |
| Native 회귀 | 공통 541개 ID 결과 변화 0건. 신규 55개 통과. Console-17 28개, pick02 17/8, Gripper 154/1이 활성 트리와 같다 |
| Console (배포 흉내) | 184개 통과(skip 2). Console-17·RIGHT_X 범위 모두 열림 |
| 저장 실물 재현 | 실패 `112020`은 첫 집기 전 관측에서 `FOV_RISK`. 오프라인 보정량 +6.44 mm를 기존 `plan_pick`이 받아들였고 pad 여유는 6.50 / 6.52 mm |
| 성공 기록 보존 | 최종 관측이 읽힌 17건(오늘 `121735` 포함) 중 16건은 지금과 같은 경로로 간다(그중 9건은 기존 Gate가 이미 재정렬). `right_receive_observation_02` 1건은 집기 전 높이에서 기존 재정렬을 한 번 더 한다 |

**한계**
- 실패는 1건뿐이다. 보정 성공률이 아니고, 새 실물 결과도 아니다.
- 실행기가 쓰는 기준은 17건 전부(자기 자신 포함)다. leave-one-out 결과는 `LEFT_FOV_REAL_REPLAY_COMPARISON.json`에 따로 있다.
- 하강 중 블록 위치가 움직이는 이유는 확인하지 못했다(원근 효과 가설).
- `112020` 보정 목표에 대한 stage 경로 충돌 인증은 장치 없이 돌리지 않았다. 실제 실행 때 stage가 다시 검사한다.

## 3. 배포 조건 (사용자 결정 필요)

이 Native 후보는 RIGHT_X 후보 위에 쌓여 있다(브랜치 기준 `483b2ef` → `8703027`). 그래서 배포하면 다음이 함께 바뀐다.
1. **Native.** 활성 폴더 `sweepick_261013_receive_alignment_restore`에 다음 파일을 덮어쓴다.
   - `RUN_APPROVED_SESSION.py`
   - `revision/sweepick_full_chain.py`, `right_x_metrology.py`, `sweepick_pick01.py`, `sweepick_episode.py`
   - `revision/left_fov_check.py`, `sweepick_left_fov_reference.json`
   - `PREPARED_SOURCE_MANIFEST.json`
   - 시험 파일
2. **Console.** 라이브 Console은 오늘 화면 변경판(`207e555`)이다. 여기에 RIGHT_X Console과 새 바인딩 목록, 두 시험 SHA 기록을 합쳐야 한다. 후보 Console(`sweepick_261016_left_fov_console`)이 그 합친 상태다.
3. **같은 시점.** Native와 Console을 같은 시점에 바꾼다. 한쪽만 바꾸면 LEFT_START_TO_LIFT나 RIGHT_X가 닫힌다.

RIGHT_X 없이 왼팔 시야 보정만 먼저 배포하려면, 활성 Native(`d527494` 계열) 위에 `pick01`·`episode`·새 파일 2개만 얹은 별도 후보를 만들어 다시 시험해야 한다. 이번에는 만들지 않았다.

**기존 PREPARED·FAULT 작업.** 배포 전 작업은 옛 SHA에 묶여 있다. 배포 후 START 검사에서 거부되는 것이 맞는 동작이다. 새 Manifest로 준비한다.

## 4. 판단

- 기능 구현과 무장치 근거로는 **배포 후보 조건을 갖췄다.** 다만 실물 효과는 아직 확인되지 않았다.
- 권장 순서:
  1. 사용자 승인
  2. 위 3절 조건으로 함께 배포
  3. 무장치 확인
  4. 현장 승인 뒤 실물 1회
- 그 실물 1회에서 확인할 것: 집기 전 높이에서 보정 전후 영상 오차와 예측 경계가 변하는지(SESSION.json의 `visual` 행 `fov`·`fov_before_this_look`), 그리고 최종 관측이 잘리지 않는지.
- LIVE_READY가 아니다. SHA가 같은 것은 물리 안전 승인이 아니다.
