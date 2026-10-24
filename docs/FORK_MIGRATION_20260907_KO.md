# 기존 fork 통합 기록

record_id: `DAPIER-2026-09-07-fork-migration`

나는 별도 GitHub fork 두 개를 DAPIER 아래로 합치고 원본 fork를 삭제하는
정리 작업으로, 각 `main`의 파일과 Git 이력을 `git subtree add`로 옮겼다.
하위 폴더는 submodule이 아니므로 DAPIER를 clone하면 함께 내려받는다.

| 하위 폴더 | 이전 fork | 교육 원본 | 보존한 main commit |
|---|---|---|---|
| `deepThinkCar_mini/` | `Alpenj/deepThinkCar_mini` | [JD-edu/deepThinkCar_mini](https://github.com/JD-edu/deepThinkCar_mini) | `e272ec836de823e761e5967c8a164afd61a7ea5a` |
| `so101_imitation_learning/` | `Alpenj/so101_imitation_learning` | [JD-edu/so101_imitation_learning](https://github.com/JD-edu/so101_imitation_learning) | `4d1cb6c77b5a6cd48bc493c1b18f79ebc7d517a8` |

deepThinkCar의 `agent/deepthinkcar-step2-5-safety`에는 main과 다른 작업이 있어,
DAPIER의 `archive/deepThinkCar_mini/agent/deepthinkcar-step2-5-safety` 브랜치에
원래 commit `13d58b78c73b9e8020132511d93ec389e91c451f`를 그대로 보존한다.
그 브랜치의 파일은 이전 저장소처럼 루트에 있으며, 이번 main 통합에는 적용하지 않았다.

이번 저장소 이전에는 기존 공개 fork가 추적하던 교육용 데이터·모델·예제 calibration도
그대로 포함한다. 새 개인 데이터나 로컬 미커밋 파일을 추가한 것은 아니다.
각 폴더의 기존 README는 교육 원본의 설명이며 내 실행 성공 기록으로 해석하지 않는다.
예제 calibration을 개인 장치에 적용하기 전에는 별도 검증이 필요하다.

## 내가 확인한 것

- 두 원본의 모든 branch를 별도 로컬 mirror에 백업하고 `git fsck --full`을 통과했다.
- deepThinkCar 680개, SO101 764개의 추적 파일을 옮겼다.
- 파일 내용·경로·mode를 포함하는 Git tree ID가 각각 원본 main과 일치했다.
- 별도 release와 열린 issue/PR은 없었다.
- 로컬의 기존 SO101 미커밋 작업과 환경은 그대로 보존했다.
- 코드 변경이 없는 이전이므로 SIM/MOCK/HW 실행 검증은 하지 않았다.

통합 직후 파일 동일성은 다음 명령으로 다시 확인할 수 있다.

```bash
test "$(git rev-parse HEAD:deepThinkCar_mini)" = 4ffc71b19dcf75ca02a2cdde4706c22f317d73a2
test "$(git rev-parse HEAD:so101_imitation_learning)" = acf7552329e1440fac44585296480582f05689e3
git merge-base --is-ancestor e272ec836de823e761e5967c8a164afd61a7ea5a HEAD
git merge-base --is-ancestor 4d1cb6c77b5a6cd48bc493c1b18f79ebc7d517a8 HEAD
```
