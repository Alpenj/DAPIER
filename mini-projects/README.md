# 미니 프로젝트

SWEEPICK과 별개로 직접 설계·실험한 작은 프로젝트다. 각 README에서 교육 원본과 직접 구현·실험한 부분을 나눠 적는다.

| 프로젝트 | 한 일 | 폴더 |
|---|---|---|
| deepThinkCar 후보 평가·정지 조건 | 카메라·차선 상실 정지와 watchdog. 같은 분할에서 조향 MAE 14.93°→4.93°를 확인했지만 평가 범위가 좁아 실차 적용은 보류 | [deepThinkCar_mini](deepThinkCar_mini/README.md) · [PR](https://github.com/JD-edu/deepThinkCar_mini/pull/1) |
| CardBench | 양팔 카지노 딜러 아이디어를 위해 게임 순서와 관측·행동 형식을 먼저 코드로 고정한 소프트웨어 실험 | [casino_dealer](casino_dealer/README.md) · [런북](casino_dealer/SO101_CASINO_DEALER_RUNBOOK_KO.md) |
| SO-101 sim-first G0–G1 | 한 팔 SO-101의 SIM 환경 점검과 scripted pick-and-lift 기록 | [dapier_sim_first](dapier_sim_first/README.md) |

테스트와 실행 명령은 각 폴더의 README를 따른다. `dapier_sim_first`의 단위 테스트는 이 `mini-projects` 폴더에서 실행한다.
