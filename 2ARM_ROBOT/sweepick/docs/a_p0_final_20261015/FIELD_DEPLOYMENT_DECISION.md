# FIELD_DEPLOYMENT_DECISION — A P0 최종 (2026-10-15)

**판정: 배포 보류.** 소프트웨어 무장치 검증은 통과했다. 그러나 현장 수동 복구가 확인되지 않아 활성 실행본·라이브 콘솔 적용과 장치 접근을 하지 않았다.

| 항목 | 상태 |
|---|---|
| 직전 세션 20261015_151937_81d00c | 콘솔 FAULT, `can_prepare=false`, 수동 복구 확인 없음 |
| 활성 native | 변경 없음(move_a `5dd5d760…`, launcher `59a33f1a…`, pick01 `815327ab…`) |
| 후보 native | move_a `030d37ee…`, launcher `9578a1ee…`, pick01 `2228d159…` |
| 라이브 콘솔 | 변경 없음. 후보 콘솔과 `CONSOLE17_TESTED_NATIVE_SHA256.json`은 후보 native SHA와 일치 |
| 과거 실행 Manifest·Source SHA | 수정하지 않음 |

## 이번 보완

1. **Bus open 예외 기록**
   - move_a의 두 open 지점(run, grip)에서 예외 사슬을 기록한다. 기록 항목은 종류, 전체 메시지, 원래 예외, errno, strerror, 포트, 단계다.
   - gate, REFUSED, exit, 쓰기 없음, STOP은 그대로다. 재접속·재시도·자동 회수는 없다.
   - 15:19 세션은 이전 코드로 기록됐으므로, 그 세션의 errno는 지금도 알 수 없다.
2. **사전 승인 경계**
   - 경유점 수가 후보 목표 수와 같아야 한다.
   - 단계 연속성, 각 단계의 인증 궤적 범위, 사용 직전 새 실측과 계획 시작값의 일치, 실제 START 승인과의 연결을 추가로 확인한다.
   - 하나라도 다르면 기존 수동 승인으로 돌아간다.
   - 절대 거리·각도 상한은 넣지 않았다. 검증된 근거가 없기 때문이다. 크기 상한이 필요하면 수동 승인을 쓴다.
3. **UI**
   - START 체크박스: 미선택이면 `preapprove_view=false`, 선택이면 `true`, LEFT_START_TO_LIFT에서는 비활성이며 `false`.
   - 브라우저 Smoke Test 3/3 통과.

## 회귀 (무장치)

| 묶음 | 결과 |
|---|---|
| 신규 Native | 51 PASS |
| A Gripper | 154 PASS |
| Console-17 Native | 28 PASS |
| Native 전체 | 522 PASS. 실패 38건과 수집 오류 1건이 활성 기준과 동일, 신규 0 |
| Full-chain 사전 실패 28건 | 동일, 수리하지 않음 |
| Console | 130 OK (2회) |
| Browser smoke | 3/3 |

두 세션을 오프라인으로 다시 재생한 결과는 다음과 같다.

| 세션 | 상태 | 첫 오류 | 정상 복귀 | 수용 미도달 | 진단 범위 완료 | 삽입 준비 | 인계 성공 |
|---|---|---|---|---|---|---|---|
| 14:08 | ENDED | 없음 | 확인 | 2건 | true | false | false |
| 15:19 | FAULT | BUS_LOST | false (`safe_shutdown=UNKNOWN`) | 2건 | false | false | false |

## 적용 순서 (현장 수동 복구 확인 뒤)

1. 15:19 세션의 실물 회수와 콘솔 수동 복구 확인
2. Owner와 포트 점유가 없는지 확인
3. 활성 native에 diff를 적용한다. 대상은 `RUN_APPROVED_SESSION.py`, `revision/sweepick_move_a.py`, `revision/sweepick_pick01.py`이고, 결과가 후보 SHA와 바이트 단위로 같은지 확인한다.
4. 라이브 콘솔에 diff를 적용한다. 대상은 `control.py`, `records.py`, `replay.py`, `app.py`, `static/app.js`, `static/index.html`, `CONSOLE17_TESTED_NATIVE_SHA256.json`이다. 그다음 재시작한다.
5. `/api/control/scopes`에서 `limited_source_check.all_match=true`인지 확인한다.

3과 4는 함께 적용한다. 콘솔만 먼저 적용하면 제한 실행이 SHA 불일치로 닫힌다. 의도된 동작이다.

## 남은 위험

- 15:19 Bus open 실패의 원인은 여전히 미확정이다. 다음 실패부터는 errno가 기록된다.
- 사전 승인은 Fake로만 검증했다. 실물에서 처음 쓸 때는 운영자가 감독해야 한다.
- 오른 elbow의 반복 정착 미도달(+23~26 tick)은 수용된 동작이다.
