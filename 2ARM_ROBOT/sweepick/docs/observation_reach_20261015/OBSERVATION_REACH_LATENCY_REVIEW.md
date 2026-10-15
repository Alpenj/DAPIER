# RIGHT OBSERVATION 미도달 · Visual Servo 시각 기록 · B19 Shadow 인터페이스 (2026-10-15)

Fake Bus와 저장 기록만으로 검토했다. 실물 START, Motor/Torque write, 장치 접근은 하지 않았다. 활성 실행본과 라이브 콘솔은 바꾸지 않았다. 수치와 근거는 [OBSERVATION_REACH_RESULTS.json](OBSERVATION_REACH_RESULTS.json)에 있다.

## 1. 미도달의 정체

`20261015_140844_3cc05c`의 관측 이동 `observe_0_0`, `observe_0_1`(오른팔)에서 elbow만 목표보다 덜 갔다.

| 이동 | 시작 | 목표 | 정착 | 남은 tick | elbow load |
|---|---|---|---|---|---|
| observe_0_0 | 2741 | 2671 | 2697 | +26 | 116 |
| observe_0_1 | 2697 | 2666 | 2689 | +23 | 104 |

세 가지 판정을 구분하면 다음과 같다.

- **명령 완료**: 예. 모든 명령 단계가 전송되고 ACK를 받았다.
- **정착**: 예. 마지막 읽기들이 1 tick 안이었고 Moving=0, Velocity=0이었다.
- **목표 도달**: 아니오. 도달 허용오차는 14 tick이다.

Goal 레지스터는 관측 목표 그대로이고, Goal과 실측의 차이 26/23 tick은 following lead 이내였다. 그래서 Owner는 이 결과를 '정착·해석된 관측 자세'로 받았다. 실측 자세가 모델에서 CLEAR인지 확인한 뒤 그 자세에서 관측했다. 이후 삽입 준비가 되지 않은 이유는 파지 자세와 패드–물체 관계가 관측되지 않았기 때문이다. elbow 미도달 때문이 아니며, 실행은 정상 복귀로 끝났다. session_18에서도 같은 관측 이동의 elbow가 24 tick 짧게 끝났다. 반복되는 하중 자세의 침하로 보인다.

기존 Fake Bus의 `lag` 동작을 저장된 부족분에 맞춰 같은 결과를 재현했다(`test_obs_reach_20261015.py`, 3건). 이것은 결과 모델이지 서보 특성 측정이 아니다.

## 2. 관측 이동에 settle_trim이 없는 이유와 수정안 비교

- Owner는 `OBSERVATION_VIEW`에 `observation_view=True, settle_trim=()`을 넣는다.
- 이동 뒤에는 두 조건을 요구한다.
  - 적용된 Goal이 보정하지 않은 관측 목표(충돌검사를 받은 목표)와 같을 것
  - |Goal−실측| ≤ following lead
- 위 조건이 맞으면 측정된 정착 자세를 다시 충돌검사하고, 그 자세에서 관측한다.
- trim을 넣으면 Goal이 인증된 목표를 넘어가고, 물체를 든 상태에서 쓰기가 늘어난다. 그래서 이 용도에서는 뺀 것으로 읽힌다. 이 해석은 코드와 기록에서 추론한 것이다.

| 안 | 내용 | 제어 변경 | 판단 |
|---|---|---|---|
| A | 실행기 계약 유지. 콘솔에서 '수용된 정착 미도달'을 고장이 아닌 기록으로 분류 | 없음 | **채택** |
| B | 관측 이동에 기존 settle_trim 사용 | 있음 | 미채택. Fake에서 도달은 하지만 Goal이 관측 목표를 벗어나 Owner 계약과 인증 결합을 다시 정의해야 함 |
| C | 예상 침하만큼 목표를 미리 보정 | 있음 | 미채택. 근거 없는 보정량이 필요하고 목표·허용오차 변경에 해당 |

### 실제 결함: 콘솔 분류

콘솔 재생 코드는 모든 `TARGET_NOT_REACHED_HOLDING`을 고장으로 셌다. 그래서 이 실행의 첫 오류가 `TARGET_NOT_REACHED`가 되었다. 양팔이 RETURNED_RELEASED로 끝났는데도 `normal_return=false`, `safe_shutdown=UNKNOWN`으로 표시됐다.

수정 내용은 다음과 같다.

- Owner가 받아들이는 조건과 같을 때만 `OBSERVATION_VIEW_SETTLED_SHORT` 기록, 북마크, AI review 항목으로 남긴다. 조건은 명령 완료, 실측 정착, 해석 완료, Status 0이다.
- 해석되지 않은 관측이나 다른 stage의 미도달은 계속 고장이다.
- 같은 저장 실행을 사본에서 다시 만들었다. 수정 전에는 first error가 TARGET_NOT_REACHED였다. 수정 후에는 first error가 없고, `normal_return=true`, `RECORDED_BOTH_ARMS_OFF`이며, 미도달 2건이 별도로 기록된다.

## 3. 다음 경로는 최신 실측 기준인가

**그렇다. 수정이 필요 없다.**

- 저장 기록에서 observe_0_1의 시작은 observe_0_0의 실측 끝점(elbow 2697)이었다. 앞 이동의 목표 2671이 아니다.
- 실행기는 첫 쓰기 직전의 읽기로 궤적을 다시 만든다.
- Owner는 측정된 정착 자세를 충돌검사한다.
- Fake 재현에서도 04가 2697에서 시작했다.

## 4. Visual Servo 단계별 monotonic 시각 (기록만)

시계는 `time.monotonic`이다. Recorder의 `recv_monotonic_s`와 같은 시계다.

- `look()`: look 시작, top 프레임 완료, 관절 읽기, 손목 프레임·분석 완료, 관절 재읽기, look 완료를 `SCENE.json`의 `timing_monotonic_s`에 남긴다.
- `visual_align()`: 반복마다 look 호출·반환, 판정, 보정 계산, 계획 완료를 남긴다. 직전 align stage의 호출·반환 시각은 다음 행에 남긴다.

같은 Fake 세션 두 종류를 수정 전후 소스로 돌렸다. 버스 write 로그가 바이트 단위로 같았다(8973 / 8979 writes). 제어 속도와 허용오차는 바꾸지 않았다.

## 5. B19 Shadow-only 인터페이스

- A는 정렬 반복마다 `VSERVO_SHADOW_INPUT.jsonl`에 한 줄을 남긴다. 사용한 입력(look 파일, q12, 관절 읽기, 이미지 오차, A가 계산한 보정, 시각)이 들어가며 `executable=false`다.
- `sweepick_vservo_shadow.py`는 오프라인 모듈이다. 장치나 실행기를 import하지 않는다.
  - 기록을 B18 형식의 observation으로 바꾼다.
  - B 제안은 `executable=false`, `motor_write_authorized=false`이고 입력 look을 명시한 경우에만 `VSERVO_SHADOW_PROPOSALS.jsonl`에 저장한다.
- A 제어 경로는 이 파일을 읽지 않는다. B19는 자기 브랜치에서 계속 진행한다.

## 회귀

| 묶음 | 결과 |
|---|---|
| 신규 Native 시험 | 10 PASS |
| Native 전체 | 481 PASS. 실패 38건이 활성 기준 목록과 동일, 신규 0 |
| A Gripper | 154 PASS |
| Full-chain 단위 시험 사전 실패 28건 | 목록 동일, 수리하지 않음 |
| Console | 124 (123 ok, 1 skip) |

Console에서는 1회 실행에서 HTTP 읽기 시험 1건이 서버 기동 타이밍으로 실패했다. 단독 실행과 전체 재실행 2회에서는 통과했다.

## 적용 시 주의

`sweepick_pick01.py`가 바뀌므로, 활성본에 적용할 때 `console/CONSOLE17_TESTED_NATIVE_SHA256.json`도 같이 갱신해야 한다. 갱신하지 않으면 LEFT_START_TO_LIFT가 의도대로 `BACKEND_CHANGE_REQUIRED`로 닫힌다.
