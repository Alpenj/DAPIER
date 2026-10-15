# Console-17 · 현장 준비 상태

| 구분 | 상태 |
|---|---|
| 오프라인(Fake/Mock/브라우저) 검증 | 통과. 아래 표 참고 |
| 실제 쓰기 경로·실물 LIFT 보유 종료 | **미검증** |
| 활성 실행본 적용 | **미적용.** 활성 native 폴더와 실행 중 콘솔(127.0.0.1:8765)은 바꾸지 않았다 |
| 대시보드 제한 실행 버튼 | 활성 native에 계약이 없으므로 **비활성 유지**(`BACKEND_CHANGE_REQUIRED`) |

LIVE_READY가 아니다.

## 무장치 검증 결과

| 묶음 | 결과 |
|---|---|
| A Gripper 기존 154건 (수정된 작업 사본) | 154 PASS |
| 기존 trim 시험 `KeyError: provisional` | 실패 유지. Fixture에 `other_arm_read`가 빠진 문제로 확인했다. 같은 검사에 reader를 넣으면 통과한다(신규 2건) |
| Console-17 Fake Native (session 13 + launcher 11 + trim 2) | 26 PASS |
| Console 단위·HTTP | 117 OK (기존 103 + 신규 14) |
| Console-16 브라우저 | 33/39. field 18/18, existing 15/15. A16 6건은 수정 전 코드에서도 같은 첫 단언에서 멈춘다 |
| Console-17 브라우저 | 6/6. 실제 control POST 0 |
| 전체 native 회귀 비교 | 작업 사본과 수정 전 사본의 실패 목록 38건이 동일하다. 기존 Fixture 문제이며 신규 실패는 0이다 |

Full-chain 단위 시험(`test_sweepick_full_chain.py`)은 receive pin을 지정하면 수정 전·후 모두 28건이 실패한다. 그래서 Full-chain 불변의 긍정 증거로 쓰지 않았다. 불변 근거는 다음 세 가지다.

- Full-chain launcher 호출 인자 시험
- `limit`이 없을 때 session 경로가 같다는 baseline 시험
- `limit` 분기에만 한정된 diff

## 적용 절차 (A owner가 수행)

1. 기존 실행이 없는지 확인한다. owner·포트 점유와 실행 중 세션이 없어야 한다.
2. 이 브랜치의 `native/RUN_APPROVED_SESSION.py`, `native/revision/sweepick_pick01.py` 변경을 활성 native 폴더의 원본에 diff로 적용한다. 공개본은 경로가 정리된 사본이라 덮어쓰지 않는다.
3. console 변경을 로컬 콘솔에 적용하고 재시작한다.
4. 콘솔 `/api/control/scopes`에서 `LEFT_START_TO_LIFT`가 `SUPPORTED_CONSOLE17_LIMITED_BACKEND`로 나오는지 확인한다.
5. 기존 승인이나 과거 관절 목표는 재사용하지 않는다. 새 준비를 하면 이 범위 전용 승인·Manifest가 생성된다.

## 가장 작은 현장 시험 1회 (별도 현장 승인 필요)

- **범위**: LEFT_START_TO_LIFT 1회. 왼팔 6축 Goal/Torque write가 포함된다. 오른팔 명령은 없고 CARRY도 없다.
- **사전 조건**
  - 작업대 배치를 확인한다.
  - fresh actual과 3-view 수신을 확인한다.
  - 왼팔 Torque OFF에서 시작한다.
  - 사람이 물체와 팔을 지지할 수 있는 위치에 있어야 한다.
  - ID6 `0x20` Overload 원인은 아직 UNKNOWN이다.
- **정상 종료**: `held_after_lift` 뒤 `LIMITED_SCOPE_TERMINAL.json`이 기록되고 owner가 종료된다.
  - 왼팔은 **Torque ON으로 물체를 든 채 남는다.**
  - 운영자가 물체와 팔을 지지한 뒤 기존 수동 복구 절차(전원 차단·배치 복원 확인)를 진행한다.
  - 정상 HOME/OFF는 수행하지 않는다.
- **확인할 것**
  - LIFT 이후 write 0을 move.json과 bus ring으로 확인한다.
  - terminal 기록의 마지막 actual/goal/torque가 실물과 맞는지 확인한다.
  - 콘솔이 `LIMITED_HOLDING`을 표시하고 다음 실행을 막는지 확인한다.
- **중단**: 기존 STOP. STOP 뒤 추가 stage는 없다.
