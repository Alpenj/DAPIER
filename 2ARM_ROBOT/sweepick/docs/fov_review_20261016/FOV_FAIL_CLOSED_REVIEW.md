# LEFT FOV fail-closed 후보 검수

이 변경은 오프라인 후보다. `FOV_UNKNOWN`과 `OUT_OF_SUPPORT`는 사전 하강 `ALIGNED`를 열지 않으며 자동 보정도 만들지 않는다. 기존 `FOV_RISK`만, 완전한 합성 바인딩을 준 회귀시험에서 기존 `correct() → plan_pick() → stage("align")` 경로를 그대로 사용한다.

## 변경 계약

- 기준 JSON 누락·손상·구조 불완전, session-reference SHA 불일치, RAW frame binding 누락·형식 오류, camera/mount/resolution/object-width context 누락·불일치·계산 전제 위반, 계산된 view의 비유한 값, 미확인 jaw 기준, 수평 clip은 `FOV_UNKNOWN`이다. 현재 계산 전제는 left wrist, 320×240, 40 mm이며 두 context가 같은 잘못된 값이어도 통과시키지 않는다.
- 관측 offset이 저장 표본 범위 밖이면 다른 누락과 중복 집계하지 않고 `OUT_OF_SUPPORT`다. 보정 성공이나 `FOV_OK`로 외삽하지 않는다.
- B의 `MODEL_PRIOR ... INSIDE` 같은 필드는 A gate 입력이 아니다. 관측 수평 clip을 무효화하지 않는다.
- 새 gain, 거리, 허용오차, collision margin, current/load, dwell/count 제한을 추가하지 않았다. `correct()`, `plan_pick()`, `stage()` 본문도 바꾸지 않았다.

## 저장 REAL 18건 replay

원본 18개는 기준 파일의 source SHA와 모두 일치했다. 실제 라벨은 수평 clip 1, 비clip 17이다. 변경 전 원래 runtime gate 기준 confusion은 `TP 1 / FP 1 / TN 16 / FN 0 / UNKNOWN 0 / OOD 0`이다. 같은 결과를 support 범위까지 사후 반영한 배타 집계는 `TP 0 / FP 1 / TN 16 / FN 0 / UNKNOWN 0 / OOD 1`이다. 실제 clip 1건이 `FOV_RISK`였지만 저장 표본 support 밖이어서, 사후 표에서는 그 한 건을 TP와 OOD에 이중 계산하지 않고 OOD로 옮겼다. 변경 후 runtime gate와 support-aware 배타 집계는 모두 `TP 0 / FP 0 / TN 0 / FN 0 / UNKNOWN 17 / OOD 1`이다.

변경 후 UNKNOWN 17건은 퇴행을 숨긴 PASS가 아니다. 저장 기록에 현재 RAW-frame/mount/measured object-width context와 새 session-reference binding이 없어서 하강 승인을 만들 수 없다는 결과다. 자세한 익명 행은 `FOV_REPLAY_SUMMARY.json`에 있다.

## 검증 범위

- 관련 fake/replay 시험: 변경 전 16 PASS, 최종 27 PASS.
- HARD gate allowlist 정적 시험: 9 PASS.
- sparse worktree native baseline: 변경 전 341개 중 310 PASS / 26 FAIL / 5 ERROR, 변경 후 342개 중 311 PASS / 26 FAIL / 5 ERROR. 실패·오류 ID 변화는 없다. 누락된 fixture와 로컬 입력 때문에 생긴 기존 환경 실패를 수정하지 않았다.
- 기존 centering judge가 통과한 passed-gate fixture에서 FOV 자체가 유발한 align Goal write는 `FOV_RISK=26`, `FOV_UNKNOWN=0`, `OUT_OF_SUPPORT=0`이었다. 기존 judge가 not-ok이면서 correctable일 때의 lateral correction은 FOV와 독립적으로 유지되고, 다음 passed look에서 FOV를 다시 평가한다. 따라서 이 수치는 모든 기존 align 경로가 0이라는 뜻이 아니다. 실제 device/motor/torque/settings write는 0이다.
- IK 거부, collision certificate CONTACT, BUS_LOST, operator STOP은 하강으로 진행하지 않았고 기존 holding/return 의미를 유지했다.

## 남은 제한

현재 공개 reference에는 측정된 mount/object-width context가 없다. 따라서 이 후보는 활성 실행 또는 물리 신뢰성 PASS가 아니며 `executable=false`, `motor_write_authorized=false`다. SHA 필드 검사는 identity 문자열의 형식과 session binding만 확인하며 RAW 내용이나 clock freshness를 독립 인증하지 않는다. freshness는 기존 `look`/`_wrist_frame_once`의 새 3프레임, 전후 joint read, timing 경로를 그대로 사용한다. 새로운 물리값을 추정해 채우지 않았다. 실물 START, 카메라/serial open, 배포, Console 변경, 원본 Episode 변경은 수행하지 않았다.

최종 통합 담당 검수에서 큰 정수 JSON의 `OverflowError`도 UNKNOWN으로 닫고, 실제 임시 reference의 해당 손상 후 기존 귀환을 확인했다. 최종 소스에서 관련 27개와 allowlist 9개를 직접 실행해 36 PASS를 확인했으며 실패·오류·skip은 없다. 최종 sparse Native도 342건(311 PASS / 26 FAIL / 5 ERROR)으로 기존 실패·오류 ID가 동일하다. 원본 18건 pure replay와 A/B 순수 계약 대조 4건도 최종 SHA에 결합했다. 기존 26개 중간 회귀를 최종 27개 실행으로 오기하지 않았다. 독립 검토의 네 고정 결함과 큰 정수 경계는 모두 CLOSED다. 원본 RGB 내용·clock·물리 FOV 예측 정확도는 인증하지 않았다.

공개 전 기존 파일의 불필요한 도구 귀속 주석 한 곳을 정리했다. 최종 시험본과 공개본의 전체 Python AST가 동일함을 확인해 기능 검사는 재사용했다. 시험본 SHA와 공개본 SHA는 JSON에서 분리했다.
