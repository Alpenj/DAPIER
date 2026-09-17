# 2ARM_ROBOT 엔지니어링 학습·포트폴리오 기록 기준

record_id: DAPIER-2026-09-16-engineering-learning-record

이 문서는 2ARM_ROBOT 작업을 단순한 작업 로그가 아니라 **재현 가능한 엔지니어링 근거와 학습 기록**으로 남기기 위한 기준이다. 코드가 많다는 사실보다 문제를 어떻게 발견하고, 어떤 증거로 원인을 좁히고, 어떤 안전 조건을 보존하면서 수정했는지를 남긴다.

## 1. 기록 위치별 역할

| 위치 | 남길 내용 | 목적 |
| --- | --- | --- |
| 소스코드 주석 | 비직관적인 invariant, 안전 이유, 수치 보정의 적용 조건 | 다음 구현자가 코드의 `왜`를 이해하게 함 |
| GitHub | 코드, fixture, test, commit/PR, 실행 명령, CI·artifact, 미검증 범위 | 실제 구현·재현 근거 |
| 비공개 Notion | 가설, 실패 과정, 화면 캡처, 수식·개념, 배운 점, 다음 질문 | 학습·면접 설명용 사고 과정 |

같은 긴 내용을 세 곳에 복사하지 않는다. GitHub는 증거, Notion은 설명, 소스 주석은 설계 의도를 맡는다. Notion URL·페이지 ID·개인 원문은 공개 GitHub에 넣지 않는다.

## 2. 의미 있는 변경의 최소 기록 구조

충돌·IK·제어·데이터·ACT·sim-to-real처럼 동작 의미가 바뀌는 작업은 가능하면 다음 흐름으로 남긴다.

1. **Problem** — 무엇이 실패했고 어느 phase에서 처음 드러났는가.
2. **Evidence** — 로그, viewer, 수치, fixture, 독립 계산 중 무엇으로 확인했는가.
3. **Decision** — 어떤 선택을 했고, 어떤 쉬운 우회는 왜 하지 않았는가.
4. **Validation** — positive/negative regression과 실제 실행을 어떻게 검증했는가.
5. **Result** — 어디까지 실제로 통과했고 어디부터는 미검증인가.
6. **Lesson / Next** — 일반화 가능한 개념과 다음 blocker는 무엇인가.

실패 시도도 원인을 좁히는 근거라면 삭제하지 않는다. 다만 포트폴리오용 요약에서는 위 구조로 압축한다.

## 3. 소스코드 주석 원칙

주석은 간단히 쓴다. 줄이 하는 일을 번역하지 말고 **왜 이 조건이 필요한지**를 1~3줄로 설명한다.

좋은 예:

```python
# Native narrowphase can return zero for separated geometry at isolated poses.
# Raise the distance only when an independent conservative certificate proves separation.
```

```python
# SIM-only near-support pair: keep contact/penetration checks, but do not apply
# the general workspace clearance until real assembly tolerance is measured.
```

피할 예:

```python
# Calculate distance.
distance = calculate_distance()
```

다음 경우에는 짧은 주석을 우선 남긴다.

- fail-closed 안전 조건이나 예외
- 일반 기준과 다른 scene-local 정책
- 수치 tolerance 또는 certificate의 적용 범위
- frame/unit/transmission 변환
- preview와 실제 physics 실행의 경계
- simulator truth와 sensor observation의 경계

장문의 디버깅 일지는 코드에 넣지 않고 Markdown/PR/Notion으로 보낸다.

## 4. 검증과 시각화

- SIM, MOCK, HW 증거를 분리한다. 한 계층의 PASS를 다른 계층의 성공으로 확대하지 않는다.
- 수정으로 통과시킬 positive case와 계속 실패해야 하는 negative case를 함께 둔다.
- 안전 로직은 threshold 완화나 pair 삭제보다 독립 증거와 regression을 우선한다.
- MuJoCo 동작, perception, 학습처럼 눈으로 확인할 가치가 있는 작업은 가능한 한 viewer·render·plot·진행 telemetry를 제공한다.
- preview용 qpos 배치와 `ctrl + mj_step()` 실제 physics를 화면과 보고서에서 구분한다.
- 장시간 rollout·수집·학습은 episode/seed/worker/phase 또는 loss/validation 진행률을 표시한다.

## 5. 완료 보고 체크리스트

의미 있는 작업을 끝낼 때 최소한 다음을 남긴다.

- 기준 Git SHA/branch와 dirty 여부
- 사용한 scene/model/dataset/checkpoint 또는 hardware profile revision
- 첫 실패 증상과 원인
- 변경한 파일과 핵심 설계 결정
- 실행한 검증 명령과 PASS/FAIL 수
- viewer/render/artifact 재현 방법이 있으면 그 명령
- 실제로 도달한 최종 phase와 미검증 범위
- 배운 개념 1~3개와 다음 작업

완료하지 않은 단계를 성공처럼 쓰지 않는다. `teacher SUCCESS`, `ACT task success`, `sim-to-real`, `hardware verified` 같은 표현은 각각 해당 근거가 있을 때만 사용한다.

## 6. 포트폴리오용 case study로 승격할 기준

다음 중 하나 이상이면 Notion에서 별도 case study로 정리할 가치가 있다.

- 단순 수정이 아니라 원인 가설을 실험으로 구분한 문제
- simulator/API 결과와 독립 물리·기하 증거가 충돌한 문제
- 안전성과 기능 사이의 정책 경계를 설계한 문제
- 실패 fixture와 negative regression으로 재발을 막은 문제
- Digital Twin → Teacher → ACT → Sim-to-Real 흐름의 다음 단계로 실제 진전을 만든 문제

포트폴리오에서는 시행착오 전체를 나열하지 말고 `Problem → Evidence → Decision → Validation → Result → Lesson`으로 설명한다.

## 7. 매 작업 턴 종료 시 GitHub·Notion 동기화

여기서 **작업 턴**은 코드나 문서에 검증된 변경이 생겼거나, 재현 가능한 새 증거 또는 다음 blocker가 확정된 하나의 실질적인 Codex 작업 주기를 뜻한다.

매 작업 턴이 끝날 때 다음 순서로 handoff한다.

1. 기존 학습 기록에 `Problem → Evidence → Decision → Validation → Result → Lesson / Next`를 추가한다.
2. 관련 회귀와 `git diff --check`를 실행하고, 실행하지 못한 검증은 이유와 함께 남긴다.
3. 현재 writer가 만든 검증된 변경만 named branch에 commit·normal push한다. 다른 writer의 dirty 변경을 reset/clean/revert하거나 임의로 함께 커밋하지 않는다.
4. `main`을 base로 PR을 열거나 기존 PR을 갱신한다. 해당 변경이 독립적으로 유효하고 회귀가 깨끗하면 전체 task가 아직 미완성이어도 **부분 진척을 명시한 채 main에 merge**한다. 실패 중이거나 안전 의미가 불명확한 코드, 반쯤 편집된 변경을 기록 의무 때문에 merge하지 않는다.
5. merge 결과가 나오면 PR과 main SHA, 실제 도달 phase, 핵심 증거, 제한사항, 다음 blocker를 비공개 Notion DAPIER 기록에도 반영한다.
6. Codex 환경에서 Notion 쓰기 권한이 없으면 업데이트했다고 주장하지 않는다. 대신 정확한 내용을 담은 `NOTION_SYNC_PAYLOAD`를 최종 보고에 포함해 ChatGPT 또는 사용자가 기존 Notion 기록에 반영할 수 있게 한다.

최종 보고에는 반드시 GitHub 상태를 `merged to main` / `PR open` / `not safe to merge` 중 하나로, Notion 상태를 `updated` / `sync payload produced` 중 하나로 명시한다.
