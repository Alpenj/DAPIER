# FIELD_READINESS_DECISION — A P0 (2026-10-15)

**판정: 현장 실행 불가.** 소프트웨어 변경은 작업 브랜치에만 있다. 활성 실행본과 라이브 콘솔에는 적용하지 않았다.

| 항목 | 상태 |
|---|---|
| 직전 세션 20261015_151937_81d00c | FAULT. 오른 Return 계획 전에 Bus open 실패(BUS_LOST_CAUSE_UNCONFIRMED). 실패 시점 오른팔은 carry 자세 Torque ON 보유, 왼팔은 물체 보유. 실물 회수 확인 기록 없음 |
| 콘솔 | FAULT, `can_prepare=false`. 수동 복구 확인 전 새 준비·START 불가(읽기 전용 확인) |
| 장치 | 이번 작업에서 접근하지 않음. 15:34:59 이후 오른·왼 어댑터가 재등록됐다(커널 로그). 현재 장치 상태는 확인하지 않음 |
| 변경 적용 | 미적용. 적용하면 `RUN_APPROVED_SESSION.py`, `sweepick_pick01.py`, 콘솔 파일, `CONSOLE17_TESTED_NATIVE_SHA256.json`을 함께 diff로 적용해야 한다. 콘솔만 먼저 적용하면 제한 실행이 SHA 불일치로 닫힌다(의도된 동작) |
| A Gripper 수정 | `sweepick_move_a.py` 5dd5d760… 그대로 |

## 무장치 검증

| 묶음 | 결과 |
|---|---|
| 신규 Native(사전 승인·Fake Bus 비교·미도달 재현) | 28 PASS |
| 신규 Console | 8 PASS |
| Native 전체 | 506 PASS. 기존 실패 38건과 수집 오류 1건이 활성 기준과 동일, 신규 0 |
| A Gripper | 154 PASS |
| Console-17 Native | 28 PASS |
| Full-chain 단위 시험 사전 실패 28건 | 동일, 수리하지 않음 |
| Console 전체 | 129 OK (2회) |

브라우저 시험은 이번에 실행하지 않았다. 바뀐 화면 요소는 START 체크박스 1개다.

## 남은 장애 요인

1. 15:19 세션의 실물 회수와 수동 복구 확인
2. Bus open 실패 원인 미확정. OS 오류가 기록되지 않았고, 원인 확인용 기록 보강안은 move_a 변경이라 별도 승인이 필요하다
3. 오른팔 elbow의 반복 정착 미도달(+23~26 tick)은 수용된 동작이다. 정밀도 개선은 이번 범위 밖이다
4. 사전 승인 경로는 Fake로만 검증했다. 실물에서는 첫 사용 시 운영자 감독이 필요하다

## 다음 순서

1. 운영자 실물 회수와 수동 복구 확인
2. 활성본에 diff 적용, 콘솔 재시작, `/api/control/scopes` SHA 확인
3. 별도 현장 승인으로 1회 실행. 사전 승인은 선택 사항이다
