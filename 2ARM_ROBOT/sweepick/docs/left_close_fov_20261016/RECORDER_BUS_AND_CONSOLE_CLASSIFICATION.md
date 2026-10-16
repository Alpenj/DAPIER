# RECORDER_BUS_AND_CONSOLE_CLASSIFICATION — Recorder Bus 오류와 Console 분류 (2026-10-16)

대상: REAL Run `20261016_112020_4d7c2b`. 저장된 파일만 읽었고 장치는 열지 않았다.

## P0-4 Recorder Bus 오류

**기록된 사실**
- `EPISODE.json errors`에 `t_host_s 1792117282.674`(11:21:22.674) 한 건이 있다.
  - 내용: `right arm read: ConnectionError: Could not connect on port '/dev/dapier/right_arm' …`
- 블록 위 관측(01a, 손목 프레임 11:21:23.274) 동안 Recorder가 오른팔을 읽으려다 포트를 열지 못한 것이다.
  - 이때 오른팔은 어떤 stage도 쓰지 않았다(이번 run은 왼팔 PICK만 함).
- 기존 루프는 그 버스를 놓고 다음 주기에 다시 열었다. 오른팔 읽기는 오류 전 20회, 후 18회이고, 직전 성공 읽기는 11:21:21.920, 다음 성공 읽기는 11:21:22.917(오류 기록 0.24초 뒤)이다.
  - 이것은 Recorder의 기존 동작이다. 이번에 바꾸지 않았다.
- 왼팔 작업 판정(`ready_to_close`)과는 무관하다. 그 판정은 손목 영상의 경계 잘림으로 정해졌다.

**원인: `BUS_OPEN_CAUSE_UNCONFIRMED` 유지**
- lerobot은 `OSError`/`SerialException`을 `ConnectionError("…Could not connect…") from e`로 감싼다.
- Recorder `_arms()`는 `str(e)`만 남겼기 때문에 원래 오류(errno 등)가 이 기록에서는 사라졌다.
- `sweepick_move_a.bus_open_exception`(10/15 보강)은 실행기 stage의 bus open에만 쓰이고, Recorder 경로에는 적용되지 않았다.

**최소 패치 (작업 브랜치, 활성 미적용): `revision/sweepick_episode.py`**
- Recorder 오류 행에 `exception` 필드를 더한다. 기존 `bus_open_exception`을 그대로 쓰고, import가 안 될 때만 같은 형태의 type/message/errno 사슬을 직접 만든다.
- port는 추정하지 않고 `None`으로 둔다. 실제 포트는 메시지 안에 있다.
- 기존 `what` 문구, drop, 다음 open, 모든 Gate는 그대로다. 자동 재접속·재시도·Motor/Torque 변경은 추가하지 않았다.
- 시험 `revision/test_recorder_exception_chain.py` 2개 통과.
  - `ConnectionError ← OSError(EBUSY)` 사슬이 남는다.
  - 원인이 없는 오류는 한 단계짜리 사슬로 남는다.
  - 다른 팔 기록은 계속된다.

## P0-5 Console 분류 (작업 브랜치 Console 후보, 라이브 미적용)

| 항목 | 지금 라이브 표시 | 후보 표시 |
|---|---|---|
| 첫 작업 Gate | `EXECUTOR_ERROR · exit 3` | 멈춘 단계 **관측(직전: 하강)** · `ready_to_close=false` · "block is not fully readable…" (STATE.json의 마지막 판정이 false일 때만) |
| Native 종료 | outcome 문자열 안에만 있음 | `Native 종료: PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME · exit 3 (비성공 종료 분류, 실행기 충돌 아님)` |
| 단계 결과 | 없음 | `descend HOLDING → retreat_arm HOLDING → home_arm RETURNED_RELEASED` |
| Recorder 오류 | 따로 없음 | `Recorder 오류(작업 Gate와 별개) 11:21:22.674 … · 원인 미확정(BUS_OPEN_CAUSE_UNCONFIRMED)` |
| 왼팔 Torque | 구분 없음 | `최종 Torque 해제 확인 (home_arm RETURNED_RELEASED)` |
| 오른팔 Torque | 구분 없음 | `최종 해제 확인 없음 · 마지막 관측 11:21:32 Torque 0 (종료 확인 아님)` |

**바꾸지 않은 것**
- `state=FAULT`, `first_error`(`EXECUTOR_ERROR exit 3`), `safe_shutdown=UNKNOWN`, `normal_return_confirmed=false`
- FAULT 수동 복구 차단과 `off_confirmed`. 양팔 모두 RETURNED_RELEASED move가 있어야 하는 기존 판정이다.
- 오른팔의 마지막 Torque 0 관측은 종료 시점의 확인이 아니므로 `safe_shutdown`을 올리지 않는다.

**Recorder 오류의 원인 표시**
- `RECORDED`: 원인 사슬이 남아 있다(패치 후 기록).
- `BUS_OPEN_CAUSE_UNCONFIRMED`: 원인 없는 ConnectionError다.
- `MESSAGE_ONLY`: 메시지 자체가 원인이다(예: 모터 ID 1–6 무응답).

**시험**
- `test_failure_point.py` 10개 통과. 그중 이번 분리 시험 4개:
  - 재정렬된 이전 판정은 Gate로 쓰지 않는다.
  - exit 3과 Native 결과 분류를 구분한다.
  - Recorder 원인 상태 3종을 구분한다.
  - 마지막 Torque 관측은 최종 해제가 아니다.
- Console 전체 185개 중 기존 불안정 시험 `HttpReadOnly` 1개만 전체 실행에서 실패하고, 단독 실행에서는 통과한다.
