# RIGHT_X_CONSOLE_CONTRACT — A RIGHT_X_METROLOGY Console 연동 (2026-10-16)

상태: **후보 구현, 무장치 검증만 완료.** 실물 START, 장치 접근, Motor/Torque write를 하지 않았다. 활성 Native와 라이브 Console은 바꾸지 않았다.

기준: native 실행 후보 `0cb3ad4`(바꾸지 않음). Console 후보는 라이브 Console(통합 `d527494`과 같은 내용)에서 시작했다.

## 1. 지금 어디서 쓸 수 있나

- Console은 활성 native가 `RIGHT_X_TESTED_NATIVE_SHA256.json`의 12개 파일 SHA와 같을 때만 이 범위를 연다. 계약 표식만 있어서는 열리지 않는다.
- 지금 활성 native에는 이 계약이 없다. 그래서 이 Console을 배포해도 RIGHT_X는 `BACKEND_CHANGE_REQUIRED`로 닫혀 있다.
- RIGHT_X를 쓰려면 native 후보와 Console 후보를 함께 적용해야 한다. 적용 여부는 사용자가 정한다.
- **배포 시 주의.** native 후보를 활성으로 바꾸면 launcher와 `sweepick_full_chain.py`의 SHA가 바뀐다. 그러면 `CONSOLE17_TESTED_NATIVE_SHA256.json`과 맞지 않아 `LEFT_START_TO_LIFT`가 닫힌다. 후보는 Console-17 시험 28/28을 통과했으므로, 적용할 때 그 기록도 다시 만들어야 한다. 이번 작업에서는 그 기록을 바꾸지 않았다.

## 2. P0-1 전용 범위

**준비**
- 범위 선택 목록에 `RIGHT_X_FINITE_METROLOGY`를 추가했다.
- 준비 창에서 운영자 입력 5개를 받는다. 입력칸은 모두 비어 있고 기본값·권장값·상한이 없다.
  - `t_pose`
  - `x_basis_tool` 3성분(단위 벡터)
  - `x_basis_source`
  - F 변위(mm)
  - V 변위(mm)
- mm 입력은 m로 바꿔 보낸다. 서버가 native와 같은 형식 검사를 하고, launcher가 시작할 때 다시 검사한다.
- 준비하면 이 범위 전용 승인 파일과 Manifest가 생긴다.
  - 승인 파일: `scope = execution_scope = RIGHT_X_FINITE_METROLOGY`. `declared_motor_writes`와 `excluded`는 native `right_x_metrology.py`의 목록을 코드 실행 없이 읽어서 넣는다. 오른 그리퍼·삽입·닫기·왼손 해제 허용은 모두 `false`다.
  - Manifest: launcher, full_chain, pick01, move_a, `right_x_metrology.py`, `current_view_candidate.py`를 포함한 12개 파일의 SHA를 묶는다.
  - 기존 Full-chain 승인에서 복사하지 않는다.

**START**
- 준비된 작업 화면에 입력값, 전체 조작 13단계, 제외 7항목을 보여 준다.
- START를 누르려면 두 가지가 필요하다.
  - 기존 현장 승인 체크
  - "전체 조작 단계와 제외 범위 확인" 체크(`writes_acknowledged`)
- START 사전 승인 체크는 이 범위에서 숨기고 막는다. 서버에서도 거부한다.
- 서버는 START 전에 다음이 준비 시점과 그대로인지 다시 확인한다.
  - write 목록이 native 계약과 같은지
  - 입력값이 작업 기록과 같은지
  - Manifest의 SHA가 시험한 SHA와 같은지

**F/V 이동 요청 화면.** 요청 scope가 `RIGHT_X_FINITE_METROLOGY_MOVE`이면 다음을 보여 준다.
- 역할(F/V)
- 승인 변위(mm/m)와 축, 운영자가 적은 축 근거
- 계획 시작 실측: bus_seq, 오른팔 tick, 계획 시작이 요청 실측과 같은지
- 실제 목표 tick과 관절별 변화량
- 충돌검사 판정과 첫 접촉, native 실행 가능 여부, 목표 개수
- 정상 역경로 가능 여부
- 근거 구분과 `FIELD_PHYSICAL_CONFIRMATION_REQUIRED`

## 3. P0-2 입력 변위는 실물 허가가 아니다

| 항목 | 계획이 기대는 근거 | 현장 근거 |
|---|---|---|
| 방향 | 운영자 입력 문장, A 모델 tool 좌표(후보 mapping, 미검증) | 요청마다 운영자가 확인하고, 본 방향을 직접 적는다 |
| 시작 자세 | 요청 시점의 bus 실측 | 팔이 그 자세에 있음을 운영자가 확인 |
| 비접촉 공간 | A 모델 충돌검사(보유 물체는 SEARCH_PRIOR). 측정한 여유 아님 | 이 변위만큼의 빈 공간을 운영자가 확인 |

- 현장 확인 3개와 방향 메모가 모두 없으면 그 요청을 승인할 수 없다. 서버는 `FIELD_PHYSICAL_CONFIRMATION_REQUIRED`로 거부하고 결정 파일을 쓰지 않는다. 거절에는 현장 확인이 필요 없다.
- 계획이 다음 조건을 하나라도 못 갖추면 현장 확인이 있어도 승인할 수 없고 거절만 할 수 있다: 1개 목표, 충돌 CLEAR, native 실행 가능, 정상 역경로 성립, 시작 = 요청 실측, 역할 F/V.
- 현장 확인은 그 요청 digest에만 적용된다. 새 요청(F → V)이 오면 체크와 메모가 비워진다.
- 결정 파일에 남기는 것:
  - `field_physical_confirmation`: 확인 항목, 메모, `source = local_operator_field_observation`, 시각
  - `model_plan_basis`
  - `operator_displacement_is_physical_permission = false`
- ±5/10/20 mm 오프라인 계획값은 화면과 계약에 "권장값·안전 한계 아님"으로만 적었다.
- 새 거리·속도·허용오차 기준은 만들지 않았다.

## 4. P0-3 여러 번 승인할 때

후보 native의 `authorize_observation_plan`을 실제로 돌리고 fake owner를 붙여 시험했다(Console 단위 시험과 브라우저 시험).

| 상황 | 결과 |
|---|---|
| F와 V | 요청 digest가 서로 다르다(브라우저 시험 `d0392f15…` / `3af702c8…`) |
| V 대기 중 남아 있는 F 승인 파일 | V 결정으로 읽지 않는다. 계속 기다리고 Console은 V 요청을 보여 준다 |
| F digest로 다시 승인 | Console이 거부한다(지금 요청과 다름) |
| 다른 digest의 결정 | 무시한다. 계속 기다리고 요청 표시와 STOP 버튼은 그대로다 |
| digest는 같지만 session이나 scope가 다른 결정 | native가 미승인으로 받아 대기를 끝낸다. 이동하지 않고 기존 정상 역경로로 간다 |
| 대기 중 STOP | Console STOP(`.STOP` + C-c)이 `_STOP_SIGNAL`을 세우고, 다음 주기(0.1 s)에 RuntimeError로 대기가 끝난다. 이후는 기존 중단 계약이다 |
| 대기 중 보유·bus 상실 | 기존과 같이 예외가 난다 |

- **사람의 결정 대기에는 시간 제한이 없다.** 기존 계약과 같다. 무한 대기를 막는 장치는 세 가지다.
  - 대기 중 요청이 항상 화면에 보인다.
  - STOP을 받는다.
  - 주기마다 보유·bus를 다시 확인한다.
- 다른 session이나 다른 scope로 온 요청은 거절만 할 수 있다. FULL_CHAIN 작업에 RIGHT_X 요청이 와도 마찬가지다.
- RIGHT_X 작업 안에서 오는 기존 관측 이동 요청(`AFTER_ADOPTED_VIEW`)은 기존처럼 한 번 클릭으로 결정한다.

## 5. 기록

- `status()`는 RIGHT_X 작업에 `right_x_record`를 붙인다: `RIGHT_X_METROLOGY.json`의 status·why, 기록된 끝점, 이동별 결과.
- 실행 기록: RIGHT_X 세션은 정상으로 끝나도 `success_candidates`(학습용 성공 후보 데이터)에 넣지 않고 `partial`로 둔다. 측정 이동은 시연이 아니기 때문이다. `right_x_metrology` 요약 필드를 추가했다.
- RIGHT_X 작업 화면의 실행 범위 표시는 RIGHT_X 흐름 문구로 바꿨다. 다른 범위의 표시는 그대로다.
- RIGHT_X 종료 뒤에는 다음 세션을 자동으로 준비하지 않는다. 입력은 매번 운영자가 정한다.

## 6. 바꾸지 않은 것

- native 후보 파일. native를 실행하지 않았다.
- STOP, Collision, Calibration, IK, move_a, gripper 로직
- 기존 FULL_CHAIN과 LEFT_START_TO_LIFT의 준비·승인·START·STOP·관측 이동 결정 경로. 회귀 시험 결과가 같다.
- 실물 종료·수동 회수 계약. RIGHT_X는 FULL_CHAIN과 같은 종료 판정(양팔 RETURNED_RELEASED + 레지스터 0)과 FAULT·수동 복구 흐름을 쓴다.

## 7. 바꾼 파일 (Console 후보)

| 파일 | 변경 |
|---|---|
| `control.py` | RIGHT_X 범위와 backend 판정(표식 + 시험 SHA), 입력 검사, 준비·START 검사, 요청 요약, 현장 확인이 있어야 하는 결정, 범위 표시 |
| `app.py` | 준비 요청의 `right_x`, START의 `writes_acknowledged`, 결정의 `field_confirmation` 전달 |
| `records.py` | RIGHT_X 세션은 success_candidates에서 제외, `right_x_metrology` 요약 |
| `static/index.html`, `static/app.js`, `static/field.js`, `static/style.css` | 범위 선택, 입력 5개, write 목록 확인, F/V 요청 화면과 현장 확인 |
| `RIGHT_X_TESTED_NATIVE_SHA256.json` (신규) | 시험한 native 후보 12개 파일 SHA |
| `test_right_x_console.py` (신규) | 22개 시험 |

회귀 결과는 `RIGHT_X_CONSOLE_REGRESSION.json`에 있다.
