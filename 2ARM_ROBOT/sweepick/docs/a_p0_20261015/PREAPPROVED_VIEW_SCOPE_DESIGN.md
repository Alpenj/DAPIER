# START 사전 승인 — RIGHT 비접촉 관측 이동

기존 중간 승인(`authorize_observation_plan()`이 요청 파일을 쓰고 운영자 결정을 기다리는 절차)은 그대로 둔다. 그 앞에 **운영자가 START 화면에서 따로 선택하는 사전 승인 옵션**을 추가했다.

## 승인 기록 (콘솔 START 시 APPROVAL.json)

`observation_view_preapproval`:

| 필드 | 값 |
|---|---|
| `scope` | `RIGHT_RECEIVE_NONCONTACT_OBSERVATION_MOVE` |
| `candidate_id` | `CURRENT_RECEIVER_CLEAR_THEN_INNER_PAD_VIEW` |
| `candidate_count` | 1 |
| `purpose` | `NONCONTACT_INNER_PAD_VIEW` |
| `insertion_permitted`, `close_permitted` | false |
| `session_out` | 이 작업의 세션 |
| `prepared_manifest_sha256` | 이 작업 MANIFEST.json의 SHA |
| `operator_source` | `local_console_start_option` |

- FULL_CHAIN 범위에서만 선택할 수 있다. LEFT_START_TO_LIFT에서는 거부한다.
- 체크하지 않으면 필드가 없고, 동작은 기존과 같다.
- launcher는 실행 전 `validate_launch()`에서 이 기록을 세션·Manifest와 대조한다. 하나라도 다르면 장치를 열기 전에 중단한다. 범위를 넓히는 값(INSERT/CLOSE 허용, 후보 2개, 다른 scope)도 같다.

## 사용 조건 (launcher `preapproval_covers()`)

실제로 생성된 요청에 대해 아래를 **모두** 확인한다.

- 같은 세션, 같은 Manifest SHA
- 후보 종류가 `CURRENT_RECEIVER_CLEAR_THEN_INNER_PAD_VIEW`이고, 계획도 같은 후보
- 계획이 `ready`, phase `OBSERVATION_CANDIDATE`
- forward 단계가 1개 이상이고 모두 오른팔
- 모든 단계 인증이 `CLEAR`이고, native validation에 hard·provisional이 없음
- 첫 단계 시작 tick이 현재 실측 오른팔 tick과 같음. 현재 실측에서 새로 계획했다는 근거다
- 양팔 Status 0
- 이 Owner에서 아직 쓰지 않았음(1회 한정)
- 직전에 `owner.recovery_ready()`로 LEFT 보유와 Bus를 다시 확인함

모두 맞으면 다음과 같이 진행한다.

1. 같은 요청 파일을 기록한다.
2. `operator_source=START_PREAPPROVAL`인 결정 기록을 쓴다. 확인 항목도 함께 남긴다.
3. `observation_move_preapproval_applied`와 `observation_move_approval_consumed`를 남기고 진행한다.

콘솔에서는 요청 SHA와 결정 SHA가 같으므로 중간 팝업이 뜨지 않는다.

하나라도 다르면 `observation_move_preapproval_not_applicable`과 그 확인 항목을 남긴다. 그다음 **기존 운영자 결정 대기로 돌아간다.** 자동으로 움직이지 않는다. 다른 후보, 다른 Source, 충돌검사 미통과, 관측 불충분(plan not ready), Bus Fault, 지지 미확인이 모두 이 경로를 탄다. 지지 미확인이면 기존 대기 루프가 지금처럼 예외로 멈춘다.

## 바꾸지 않은 것

- IK, 충돌검사
- Servo·Bus Status 확인
- LEFT 보유 확인
- STOP: 사전 승인 경로에서도 `stop_checkpoint()`를 거친다
- 단일 Owner와 Port Lease
- Gripper Calibration·안전 Profile
- RIGHT INSERT/CLOSE 승인: observation-only 범위에서는 `Owner.segment`가 계속 거부한다
- LEFT RELEASE
- `sweepick_move_a.py`

사전 승인은 '누가 답하는가'만 바꾼다. Fake Bus에서 수동 모드와 사전 승인 모드의 write 목록이 같았다.

## 변경 파일

- native: `RUN_APPROVED_SESSION.py`
- console: `control.py`, `app.py`, `static/index.html`, `static/app.js`
- pick01은 이번 P0에서 바꾸지 않았다. d0aca04에서 시각 기록 변경이 있었다. 두 파일이 바뀌었으므로 `console/CONSOLE17_TESTED_NATIVE_SHA256.json`도 갱신했다
- 기존 콘솔 승인 Manifest 검증 계약(Console-17 SHA 결합)은 그대로 적용된다
