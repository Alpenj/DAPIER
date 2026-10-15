# Console-17 · LEFT_START_TO_LIFT Native 제한 실행 설계

실물 장치·카메라 접근, Motor/Torque write는 0회다. 아래 동작은 Fake Bus·Mock·브라우저 시험으로만 확인했다. 실제 쓰기 경로와 실물 LIFT 보유 종료는 검증하지 않았다.

## 실행 범위

```
START/현재 관측 → PREGRASP → ALIGN → DESCEND_A/DESCEND → CLOSE → ready_to_lift → LIFT → held_after_lift → LIMITED_SCOPE_TERMINAL
```

- 기존 `sweepick_pick01.session()`과 `stage()`, 기존 계획기와 실행기(`sweepick_move_a.run()/grip()`)를 그대로 쓴다. 새 IK·writer·reader는 없다.
- 경계는 `session(limit="LEFT_START_TO_LIFT")`이다. `held_after_lift` 판정과 snapshot 기록 직후, 기존 hand-over callback과 `lower/open/retreat/home` 진입 전에 `limited_lift_end()`로 반환한다. 이 반환 뒤에는 어떤 stage도 호출되지 않는다.
- `full_chain=False`만으로는 해결되지 않는다. 그 모드는 LIFT 뒤 put-back을 실행한다. `limit`을 주면 recorder mode도 `LEFT_START_TO_LIFT`가 된다. `full_chain=True`, hand-over callback, `record=False`와 함께 쓰면 `ValueError`가 난다.
- LIFT 이전의 기존 조건은 바꾸지 않았다.
  - 정렬·배치 확인 실패는 기존 계획대로 복귀한다.
  - `GRIP_NO_CONTACT_HOLDING`은 open/retreat/home으로 간다.
  - `GRIP_UNAVAILABLE_HOLDING`, fault, bus loss, STOP은 그 자리에서 끝난다.
  - `ready_to_lift`가 실패하면 open/retreat/home으로 간다.
  - 이것은 LIFT 이후의 자동 회수가 아니라 기존 LIFT 이전 실패 계약이다.
- `GRIP_UNAVAILABLE_HOLDING`은 접촉 성공이 아니다. 기존 `CLOSE_OK=("GRIP_CONTACT_HOLDING",)`가 그대로라서 LIFT로 가지 않는다.

## 종료 계약 (`LIMITED_SCOPE_TERMINAL.json`)

| terminal | 근거 |
|---|---|
| `LEFT_LIFT_HELD_CONFIRMED` | LIFT가 HOLDING으로 끝남 + 기존 `held_after_lift` 판정 ok |
| `LEFT_LIFT_HOLD_UNKNOWN` | LIFT는 끝났지만 `held_after_lift`가 ok 아님. 물체 낙하로 판정하지 않음 |
| `LEFT_CLOSE_UNAVAILABLE` | CLOSE 결과가 `GRIP_UNAVAILABLE_HOLDING` |
| `LEFT_LIFT_FAILED` | LIFT stage가 실행됐으나 허용 결과가 아님 |
| `OPERATOR_STOPPED` | STOP latch 또는 stage의 `operator_stop` gate |
| `BUS_OR_SERVO_FAULT` | stage가 `BUS_LOST`/`FAULT_*`/`EXECUTOR_EXCEPTION`이거나 gate가 `bus_lost`/`servo_fault`, 또는 예외 종료 |
| `ENDED_BEFORE_LIFT` / `NOT_STARTED` | LIFT 이전의 기존 종료 / 관측 단계에서 시작 안 함 |

내부 결과 문자열은 기존 규칙을 따른다. 예: `LEFT_LIFT_HELD_CONFIRMED_HOLDING_NEEDS_A_PERSON`. 분류값은 `limited_terminal`에 기록한다.

방식은 **보유 상태를 기록한 뒤 프로세스를 종료**하는 쪽으로 정했다. 비교와 근거는 [CONSOLE17_OWNER_TERMINATION.json](CONSOLE17_OWNER_TERMINATION.json)에 있다. 기존 baseline session도 `*_HOLDING_NEEDS_A_PERSON`을 이렇게 끝내며, Owner를 계속 붙잡아 두는 감독 대기에는 새 continuation 인터페이스가 필요하다.

`run_session()`은 recorder를 닫은 뒤 다음 내용을 기록한다.

- 마지막 stage의 실행기 종료 읽기(ticks/goal_register/torque/status/moving)
- 그 stage의 move.json 경로(ACK/readback)와 보유 판정 근거
- recorder release와 reader 종료 결과
- `normal_home=false`, `torque_off_confirmed=false`, `safe_shutdown=NOT_PERFORMED`, `motion_stop_confirmed=UNKNOWN`, `operator_required=true`, `next_run_allowed=false`

**프로세스 종료는 모터 정지나 Torque OFF의 증거가 아니다.** LIFT 이후 Torque write는 0회이며, 왼팔은 LIFT Goal로 Torque ON 상태다.

## Launcher·승인·Owner

- `RUN_APPROVED_SESSION.execution_scope(approval)`
  - 필드가 없으면 기존 `FULL_CHAIN_RECEIVE_CONTACT`다.
  - `LEFT_START_TO_LIFT`만 추가로 허용한다.
- `validate_limited_launch()`
  - 승인 `scope`가 `LEFT_START_TO_LIFT`이어야 한다. Full-chain 승인 `RIGHT_RECEIVE_NONCONTACT_OBSERVATION`은 거부한다.
  - 승인에 `right_arm_commands_permitted=false`, `automatic_return_after_lift=false`가 있어야 한다.
  - Manifest의 `scope`/`execution_scope`가 같아야 한다.
  - launcher, `sweepick_pick01.py`, `sweepick_move_a.py`가 Manifest에 결합돼 있어야 하고, 모든 파일 SHA가 일치해야 한다.
  - 출력 폴더와 STOP latch 검사는 기존과 같다.
- RIGHT RECEIVE 관측·접촉 계약(관측 plan, face geometry, optical build)은 이 범위의 선행조건이 아니며 준비하지 않는다.
- LEFT 관측·파지·충돌 검사는 session/stage 안에서 기존대로 수행한다.
- 실행은 `p1.run_session(out, execute=True, record=True, full_chain=False, limit=LEFT_START_TO_LIFT, episode_fn=episode)` 한 번이다.
  - 기존 단일 managed tmux owner, 배타 포트, Episode lease, STOP signal 처리, lifetime 기록을 그대로 쓴다.
  - Stage CLI를 여러 번 호출하지 않는다.
- Full-chain 호출 인자는 바꾸지 않았다. `test_main_full_chain_call_is_unchanged`로 확인했다.

## Console

- `control.limited_backend()`
  - 현재 native 파일 텍스트에 Console-17 계약이 있을 때만 `LEFT_START_TO_LIFT`를 `SUPPORTED_CONSOLE17_LIMITED_BACKEND`로 연다. import는 하지 않는다.
  - **현재 활성 native 폴더에는 아직 없으므로 실제 콘솔에서는 여전히 `BACKEND_CHANGE_REQUIRED`다.**
- 제한 범위 `prepare()`
  - Full-chain 승인 파일을 복사하지 않고, 이 범위 전용 승인을 만든다.
  - Manifest는 `scope=execution_scope=LEFT_START_TO_LIFT`이고, launcher와 실행기 소스를 명시적으로 결합한다.
- `start()`: 요청 범위, 준비된 작업 범위, 승인 범위, Manifest 범위가 모두 일치해야 기존 launcher를 실행한다.
- 종료 뒤 `status()`
  - held/unknown 보유 종료는 `LIMITED_HOLDING`, 그 밖의 종료는 `FAULT`다.
  - 둘 다 기존 `MANUAL_RECOVERY_CONFIRMATION` 전에는 새 준비가 막힌다.
  - `off_confirmed` 기준은 완화하지 않았다.
- `records.run_record()`
  - 제한 범위 기록은 `category='partial'`이며 `success`가 되지 않는다.
  - 보유 종료의 exit 3은 executor error로 세지 않는다. terminal 파일이 있을 때만 해당한다.

## 변경 범위

- native: `RUN_APPROVED_SESSION.py`, `revision/sweepick_pick01.py`
- console: `control.py`, `records.py`, `replay.py`, `app.py`, `static/app.js`, `static/field.js`, `static/review.js`, `static/index.html`
- 변경하지 않음: `sweepick_move_a.py`, `sweepick_grasp.py`, `sweepick_full_chain.py`, Jaw/progress profile, Calibration

파일별 SHA는 [SOURCE_SHA256.json](SOURCE_SHA256.json)에 있다.
