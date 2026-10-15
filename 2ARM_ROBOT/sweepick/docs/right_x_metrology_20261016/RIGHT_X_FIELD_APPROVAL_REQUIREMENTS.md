# RIGHT_X_FIELD_APPROVAL_REQUIREMENTS — A R0 (2026-10-16)

**판정: 현장 실행 불가(승인 대기).** 이번 작업은 후보 구현과 무장치 검증까지다. 실물 START, Motor/Torque write, 장치 open, 활성 배포는 하지 않았다.

## 1. 실행 전에 먼저 해결해야 할 것

1. **사용자 승인:** 후보 코드를 활성 Native에 반영할지 결정한다. 반영할 때는 launcher, `sweepick_full_chain.py`, `right_x_metrology.py`를 함께 diff로 적용하고 SHA를 확인한다.
2. **Console 지원이 없다.** 지금 Console은 이 범위를 준비하거나 START할 수 없다. 필요한 작업은 다음과 같다.
   - 준비 단계에서 운영자 입력 5개를 받는다(아래 2절).
   - `DECLARED_WRITES`와 `EXCLUDED`를 화면에 그대로 보여 주고 확인을 받는다.
   - RIGHT_X 범위의 승인 파일과 Manifest를 만든다. Manifest scope는 `RIGHT_X_FINITE_METROLOGY`이고, `right_x_metrology.py`를 묶어야 한다.
   - 이동 요청 화면에 F/V 역할, 승인 변위, 계획 끝점을 보여 준다.

   이 작업은 별도 구현과 승인 대상이다. 지금 이동 요청은 기존 "제안 이동 승인/거절" 화면과 같은 요청·결정 파일 형식을 쓴다.
3. **현재 상태 확인:**
   - 직전 세션이 FAULT 또는 수동 복구 대기 상태가 아니어야 한다.
   - Owner와 포트 점유가 `{}`이어야 한다.
   - 손목 카메라 경로 확인을 통과해야 한다.
   - SHA가 일치해야 한다.

   SHA 일치와 `can_prepare=true`는 실물 안전 승인이 아니다.

## 2. 운영자가 승인 파일에 직접 정하는 값 (기본값 없음)

| 필드 | 의미 | 근거 |
|---|---|---|
| `t_pose` | `AFTER_RIGHT_PREGRASP`(RIGHT_APPROACH 직후, 기존 관측 이동 없음) 또는 `AFTER_ADOPTED_VIEW`(기존 관측 이동을 따로 승인한 뒤) | B 후보 005/037은 기존 관측 이동 뒤의 영상이다. 같은 자세에서 비교하려면 `AFTER_ADOPTED_VIEW`를 고른다. 그 이동은 기존과 같이 정착 미도달(+23~26 tick)일 수 있다 |
| `x_basis_tool` | `right_cube_grasp` tool 좌표계의 단위 방향 | A 모델에서는 tool +X가 계획된 접근 방향과 1.5° 차이이며 보유 물체 쪽을 향한다. 물리 방향은 미확인이다 |
| `x_basis_source` | 축 근거를 문장으로 기록 | 예: "A 모델 tool +X, 물리 방향 미확인" |
| `fit_displacement_m` | F의 부호 있는 변위(m) | 운영자가 정한다. 코드에 상한이 없다 |
| `verify_displacement_m` | V의 부호 있는 변위(m), F 끝점 기준 | 운영자가 정한다. fit에 넣지 않는다 |

`verify_from`은 `"F"`로 고정이다.

**부호를 정할 때 알아야 할 사실 (권고이며 gate가 아니다).**
- 모델에서 +X는 보유 물체 쪽이다. +X 이동은 모델상 물체와의 여유를 줄인다.
- 충돌 CLEAR는 거친 보유 자세(SEARCH_PRIOR)를 쓴 모델 판정일 뿐이다.
- 처음에는 −X(물체에서 멀어지는 쪽)로 F와 V를 하는 편이 비접촉을 유지하기 쉽다.
- 독립 부호 검증에는 같은 축에서 응답이 관측되면 충분하다. 반드시 반대 방향일 필요는 없다.

## 3. 승인 파일의 고정 필드

- `scope`와 `execution_scope`는 모두 `RIGHT_X_FINITE_METROLOGY`
- `session_out`, `prepared_manifest_sha256`
- `right_gripper_commands_permitted`, `insertion_permitted`, `close_permitted`, `left_release_permitted`는 모두 `false`
- `observation_view_preapproval`은 없어야 한다. 모든 이동은 운영자가 하나씩 결정한다
- `declared_motor_writes`는 아래 목록과 같아야 한다(`right_x_metrology.DECLARED_WRITES`)
  1. LEFT ARM+GRIPPER PICK: pregrasp, align, descend_a, descend (Goal_Position, Torque_Enable)
  2. LEFT GRIPPER CLOSE on the block (Goal_Position, Torque_Enable)
  3. LEFT ARM LIFT (Goal_Position)
  4. RIGHT ARM5 CARRY_RIGHT (Goal_Position, Torque_Enable)
  5. LEFT ARM CARRY_LEFT (Goal_Position)
  6. RIGHT ARM5 RIGHT_APPROACH (Goal_Position)
  7. RIGHT ARM5 기존 관측 이동: `AFTER_ADOPTED_VIEW`일 때만, 자기 운영자 결정으로 (Goal_Position)
  8. RIGHT ARM5 X 이동 F: 자기 운영자 결정으로 (Goal_Position)
  9. RIGHT ARM5 X 이동 V: 자기 운영자 결정으로 (Goal_Position)
  10. RIGHT ARM5 정상 역경로: 실행된 X·관측 이동의 역순, RIGHT_BACK_TO_CARRY (Goal_Position)
  11. RIGHT_RETURN_INITIAL, 오른팔 Torque OFF (Goal_Position, Torque_Enable)
  12. LEFT_BACK_TO_LIFT (Goal_Position)
  13. LEFT 기존 내려놓기: lower → open(책상 위에서 왼 그리퍼 열림) → retreat → home, 왼팔 Torque OFF (Goal_Position, Torque_Enable)
- `excluded`는 아래 목록과 같아야 한다(`right_x_metrology.EXCLUDED`)
  - RIGHT 그리퍼 명령, RIGHT INSERT, RIGHT CLOSE
  - LEFT RELEASE·하중 이전
  - 영상 기반 보정 명령, servo 설정·EEPROM write
  - 자동 재시도·재접속·회수

1–6과 10–13은 10/15 14:08 정상 세션이 실제로 수행한 write와 같은 단계다. 측정이 새로 더하는 것은 8, 9와 그 역경로뿐이다.

## 4. 현장 조건 (START 체크 시 운영자가 확인)

- 물체·팔 배치, 왼팔 지지, 케이블과 주변 공간
- 감독자가 있고 STOP을 준비했다
- 손목 카메라 장착과 USB 경로가 변하지 않았다
- 오른 그리퍼는 명령하지 않는다. 열림 상태는 raw tick으로만 기록된다

**F·V 승인 요청이 오면 매번 다음을 확인한다.**
- 요청의 `planned_from` 실측이 지금 화면의 자세인지
- 계획 끝점(target ticks), 충돌 판정 CLEAR, 역경로 성립 여부
- 운영자가 정한 변위·축이 맞는지

## 5. 성공으로 보지 않는 것

- `ENDPOINTS_RECORDED`는 끝점 기록이 끝났다는 뜻일 뿐이다. 목표 영상 확정이나 J 검증이 아니다.
- 실측 FK 변위는 후보 mapping FK다. 물리 X 변위로 승격하려면 별도 확인이 필요하다.
- 측정 뒤에도 `REAL_J_VALIDATED=false`, `Correction=null`이 유지된다. 그 이후의 J 계산과 검증은 B의 계약(FIELD_VERIFIED_TARGET_IMAGE, fit/independent sign)을 따른다.
