# RIGHT_X_INTEGRATION_HEAD — 통합 브랜치 반영 기록 (2026-10-16)

상태: **브랜치 통합만 했다.** 활성 Native와 라이브 Console에는 적용하지 않았다. 실물 START, 장치 접근, Motor/Torque write를 하지 않았다.

## 통합 전 확인

| 항목 | 값 |
|---|---|
| 통합 브랜치 | `pro/sweepick-review-20261015` |
| 기대한 HEAD | `d5274945923b257067b2ab12d112fd3c4498b473` |
| 실제 HEAD(통합 직전 `git fetch` 후) | 같음 |
| Fast-forward 가능 | 예: `d527494`가 아래 세 커밋의 조상 |
| `main` | 바꾸지 않음 |
| B 브랜치 `pro/sweepick-b20-metrology-bridge-20261016` (`8f80485`) | 건드리지 않음. 통합 브랜치의 조상도 자손도 아닌 별도 브랜치(분기점 `5295c6a`) |

## 통합한 커밋 (순서대로)

1. `0cb3ad4` — A RIGHT_X_FINITE_METROLOGY R0 Native 후보
2. `e9f8f1e` — A RIGHT_X_METROLOGY Console 후보
3. 이 문서가 들어 있는 커밋 — Console-17 호환성(P0)과 최종 회귀(P1)
   - `console/CONSOLE17_TESTED_NATIVE_SHA256.json`을 후보 Native에 다시 묶었다(launcher `46def840`, full_chain `19e7ca35`).
     후보 Native에서 Console-17 시험 28개가 통과한 뒤에 기록했다.
   - `console/test_console17.py`, `console/test_right_x_console.py` 시험 보정(동작 검사는 그대로).
   - `docs/right_x_final_20261016/` 산출물 4개.

통합 브랜치 HEAD는 3번 커밋이다. 그 SHA는 `git log -1 origin/pro/sweepick-review-20261015`로 확인한다(이 문서 안에 자기 커밋 SHA를 적을 수는 없다).

## 통합 후 세 범위

| 범위 | Console 판정 기록 | 활성 Native(현재) | 후보 Native를 활성 폴더에 덮어쓴 경우 |
|---|---|---|---|
| FULL_CHAIN_RECEIVE_CONTACT | 시험 SHA 문턱 없음(준비 시 Manifest) | 열림 | 열림 |
| LEFT_START_TO_LIFT | `CONSOLE17_TESTED_NATIVE_SHA256.json` | **닫힘** (launcher·full_chain SHA가 새 기록과 다름) | 열림 |
| RIGHT_X_FINITE_METROLOGY | `RIGHT_X_TESTED_NATIVE_SHA256.json` (12개, 바꾸지 않음) | 닫힘 (계약 없음) | 열림 |

- 통합 브랜치의 Console을 지금의 활성 Native와 같이 쓰면 LEFT_START_TO_LIFT가 닫힌다. 그래서 Console과 Native는 반드시 함께 배포해야 한다. 절차는 `RIGHT_X_ACTIVE_DEPLOYMENT_PLAN.md`에 있다.
- 라이브 Console은 이전 기록을 그대로 쓰고 있어서 지금도 LEFT_START_TO_LIFT가 열려 있다. 바뀐 것은 없다.
