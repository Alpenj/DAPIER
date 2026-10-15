# Console-17 · FIELD_READY 판정

**판정: 소프트웨어 적용 완료 · 현장 START 전 조건 미충족 (FIELD_READY 아님).**

이번 작업에서 실제 START, Motor write, Torque write, 장치·카메라 open은 하지 않았다.

## 적용 상태 (2026-10-15)

| 항목 | 상태 |
|---|---|
| 활성 native 적용 | 완료 |
| 라이브 콘솔 적용 | 완료 |
| A Gripper 수정 SHA | `sweepick_move_a.py` `5dd5d760…` 그대로 |
| 콘솔 재시작 | 완료. 관리 Owner 없음, 팔 포트 점유 없음을 확인한 뒤 재시작했다 |
| `/api/control/scopes` | `LEFT_START_TO_LIFT` = `SUPPORTED_CONSOLE17_LIMITED_BACKEND`. `limited_source_check.all_match=true`(10/10 파일) |
| `can_prepare` | **false.** 이전 실행 `20261015_112218_f0d866`이 FAULT이고 수동 복구 확인 기록이 없다 |

- **활성 native 적용 방법**
  - 원본에서 시험한 작업 사본으로 가는 diff를 `patch`로 적용했다.
  - 결과 파일이 시험한 소스와 바이트 단위로 같은지 확인했다.
  - GitHub 공개본은 덮어쓰지 않았다.
  - 원본과 diff는 `/home/robot/sweepick_261015_console17_apply_backup/`에 보관했다.
- **SHA 기록**
  - 적용 전후 SHA: [CONSOLE17_APPLY_SOURCE_SHA256.json](CONSOLE17_APPLY_SOURCE_SHA256.json)
  - 시험한 소스 SHA: `console/CONSOLE17_TESTED_NATIVE_SHA256.json`

## 이번 P0 보완

1. **사후 충돌 진단 분리**
   - 제한 실행 종료 경로에서 `finish_collision_diagnostics()`를 실행하지 않는다. MuJoCo tube 진단 서브프로세스를 순차 실행하던 함수다.
   - 종료 기록 다음에 `COLLISION_DIAGNOSTICS_PENDING.json`(argv와 trajectory snapshot 경로)만 남긴다. 예외로 끝나도 남긴다.
   - 각 stage write 전의 명목 충돌검사(`--nominal-only`)와 판정은 그대로다.
   - 기존 Full-chain·baseline 경로는 종전대로 진단을 실행한다.
2. **오류 우선**
   - `LIMITED_SCOPE_TERMINAL.json`이 보유 성공이어도, 같은 세션에 기록된 servo/bus/executor 오류가 있으면 `control.status()`와 `records.run_record()` 모두 `FAULT`다.
   - 두 판정이 같다는 것을 같은 세션으로 시험했다.
3. **실물 준비 판단**
   - 문자열 표식만으로는 판단하지 않는다.
   - 활성 native 10개 파일 SHA가 시험한 소스 SHA와 모두 같아야 한다.
   - START 때 승인 Manifest가 그 SHA를 묶고 있어야 한다.
   - 시험 기록 파일이 없거나 하나라도 다르면 `BACKEND_CHANGE_REQUIRED`로 남는다.

## 회귀 (무장치)

[CONSOLE17_LIMITED_TERMINAL_REGRESSION.json](CONSOLE17_LIMITED_TERMINAL_REGRESSION.json)

| 묶음 | 결과 |
|---|---|
| 제한 종료 Native 시험 (활성 폴더에서 실행) | 28 PASS |
| Console-17 시험 (라이브 폴더) | 17 ok + 1 skip. skip은 "활성 native에 계약 없음" 확인으로, 적용 후에는 해당하지 않는다 |
| Console 전체 | 121 (120 ok, 1 skip) |
| A Gripper | 154 PASS |
| native 전체 | 471 PASS. 실패 38건과 수집 오류 1건은 수정 전과 목록이 같다. **신규 회귀 0** |
| Full-chain 단위 시험 사전 실패 28건 | 수정 전과 동일, 이번에 수리하지 않음 |

Console-16 시험 클래스는 "backend 없음"을 전제로 한다. 활성 native에 계약이 생긴 뒤에는 그 전제가 맞지 않아, 해당 클래스에서만 이를 test pin으로 고정했다. 검사 내용은 바꾸지 않았다.

## 남은 장애 요인 (START 전)

1. **이전 FAULT 미복구.** `20261015_112218_f0d866`(LEFT CLOSE 후 ID6 `0x20 Overload`, 이어서 ID6 handshake 누락)에 대한 수동 복구 확인이 없다. 운영자가 팔과 물체를 지지하고 양팔 서보 전원 차단·배치 복원을 확인해야 새 준비가 열린다.
2. **ID6 상태 미확인.** Overload 원인은 UNKNOWN이다. 이후 ID6가 응답하는지는 이번 작업에서 장치를 열지 않아 확인하지 않았다. 준비 단계의 기존 handshake가 실패하면 START 전에 멈춘다.
3. **실물 쓰기 경로 미검증.** LIFT 후 Torque ON 보유 상태로 끝나는 동작은 Fake로만 확인했다. 정상 종료 시 왼팔은 물체를 든 채 전원이 들어가 있으므로, 사람이 지지하는 위치에서만 시험한다.
4. **표시 차이 (기능 영향 없음).** 이 실패 실행의 콘솔 `block_reason`에는 최종 handshake 예외가 표시된다. 실행 기록과 Export의 first error는 더 이른 ID6 Overload(1792031028.225814)다. 판정(FAULT)은 둘이 같다.

## 다음 현장 시험 1회 (별도 현장 승인)

[CONSOLE17_FIELD_READINESS.md](CONSOLE17_FIELD_READINESS.md)의 최소 시험을 따른다. LEFT_START_TO_LIFT 1회, 오른팔 명령 없음, LIFT 후 write 0, `LIMITED_HOLDING` 표시와 다음 실행 차단을 확인한다.
