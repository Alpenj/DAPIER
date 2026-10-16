# A/B 오프라인 검수와 선별 통합 제안 — 2026-10-16

통합 기준은 `483b2ef`, A FOV 기준은 `c15e434`, B 오버레이 기준은 `b270a4f`, C MOCK 기준은 `6d24284`다. 새 A 후보는 `1cd0444dfb8bdd1d2a70cd87c90bd4f895990498`, 새 B 후보는 `0042f8cdfdd9af64b234477b8e1a77e164e01820`다. 이 브랜치는 계약·SHA·검증 결과만 기록하며 Native·Console·B 실행 코드를 병합하지 않는다.

## 검수 결과

A는 FOV UNKNOWN/OOD를 pre-descent ALIGNED로 취급하지 않는다. 동일하지만 계산 전제에 맞지 않는 camera/resolution/object-width, 빈 identity, 비유한 계산, 큰 정수 JSON의 예외를 닫았다. FOV reference SHA를 `fov_sources`로 분리해 임시 reference 누락·손상·변경 후 기존 귀환을 유지했다. `correct()`·`plan_pick()`·`stage()`의 본문 SHA는 원본과 동일하다. 거리·gain·margin·current·dwell 등 새 actuator numeric HARD gate는 추가하지 않았다.

원본18 STATE의 SHA를 독립 재확인했다. 원래 runtime gate의 TP/FP/TN/FN/UNKNOWN/OOD는 `1/1/16/0/0/0`, 수정 후는 `0/0/0/0/17/1`이다. 원래 결과에 support 범위를 사후 적용한 표는 별도로 `0/1/16/0/0/1`이다. 현재17 UNKNOWN은 필요한 source-bound REAL context 부재이며 physical FOV PASS나 성능 향상으로 해석하지 않는다. FOV passed-gate fixture의 align Goal writes는 RISK26/UNKNOWN0/OOD0이다. fake 총 write에는 기존 pregrasp/귀환이 포함되며 각각9045/6005/6005다. 기존 centering gate가 실패하면 독립된 원래 correction 경로는 유지되고, 하강 전 passed look에서 FOV를 다시 검사한다.

B는 설정상의 image target과 MODEL_PRIOR closing direction을 구분했다. 8 RAW와2PNG를 로컬 실행자가 직접 확인했고 이전 원본 file/decoded SHA·camera/source binding과 특징값·PNG bytes를 보존했다. 왼팔의 current-confirmed X만 유효할 때 사용하며 수평 clipping은 null, 바닥 clipping은 기존 X의 조건부 허용이다. 오른팔 observed Pad X/Y, Y 목표, 상대 각도8건과 실물3D yaw는 UNKNOWN이다. B의 명목 top-envelope INSIDE는 지정 실패의 실제 X-clip을 놓쳤고 A FOV pass를 공급하지 않는다. A18 STATE와 B8 RGB는 별도 관측으로 보존하며 같은 frame/offset으로 섞지 않았다.

새 시험은 A FOV27+allowlist9 PASS, B 요청모듈71(실패/오류0, optional private-model fixture 클래스1 skip), B target/bridge38 PASS, 순수 A/B 대조4 PASS다. 동일 sparse Native는341(310P/26F/5E)→342(311P/26F/5E)이며 기존 실패/오류 ID 변화는 없다. 문서에 언급된 다른 full-tree 환경의 과거38F+1E와 혼동하지 않는다. 독립 검토에서 네 고정 결함과 큰 정수 경계는 CLOSED다. 최종 공개 전 주석의 불필요한 도구 귀속을 정리한 부분은 전체 Python AST 동일성을 확인했고 시험본/공개본 SHA를 분리했다.

## 선별 통합 제안

1. 통합 담당자는 원본 `483b2ef`와 두 새 후보 commit을 고정해 파일 단위 diff를 검토한다. A FOV 기준은 통합보다4커밋 앞서고 Console 변경이 포함된다. A 후보의 Native FOV 선행 변경·reference JSON·관련 tests가 현재 통합에 없는 의존성을 먼저 대조해야 한다. 전체 c15e434 또는 Native 파일을 덮어쓰지 않는다.
2. A의 이번7파일 patch를 FOV 선행 Native 변경과 함께 수동 검토한다. 기존 Console 이력, Owner·STOP·writer·collision 경로를 유지한다. `fov_sources`는 모터 stage source guard와 분리한다. 현재 REAL source/mount/object-width producer가 없으므로 실제 실행 준비는 BLOCKED다. 이 후보를 활성 실행에 적용하거나 Console tested-SHA를 바꿔 준비 완료로 표시하지 않는다.
3. B의 이번8파일 patch는 B의 기존 visual_servo 의존성과 함께 별도로 보존한다. B 원래 브랜치는 통합과14/9커밋 diverged하며, B 인덱스로 대체하면 최신 A/Console 항목을 잃는다. 순수 B 특징 추출을 A 후보 소스로 시험한4개 계약 대조만 수행했고 Native에서 B overlay를 import하거나 실시간 연결하지 않았다.
4. 실제 코드 통합이 필요해질 때 단일 writer가 SOURCE_INDEX를 파일별로 조정하고 양쪽 original/public SHA 이력을 보존한다. 새 혼합본의 관련 회귀·source/privacy 검사를 수행해야 한다. 현재 통합 검수 브랜치의 원래230개 index 항목은 그대로 보존하고 검수 산출물만 추가했다.
5. C와 ACT/SIM은 기존 역할을 유지한다. C는 MOCK_DEVICE_FREE와 PHYSICAL_FIELD를 구분하고 `base_footprint↔arm_base`의 실물 TF·라이브 토픽·실제 정지/재개가 미검증임을 유지한다. 새 C 작업·C2 라이브 Dry-Run·train/eval·장치 수집을 시작하지 않았다. SIM tick/load/force/dwell을 REAL 한계로 환산하지 않는다.

## 남은 조건과 승인 경계

현재 목표는 오프라인 검수 완료이며 배포 가능 판정이 아니다. source SHA 형식 검사는 RAW 내용·clock freshness를 독립 인증하지 않는다. 실제 `look`의 새3프레임·전후 joint read·timing 경로를 재사용했으며 임의 freshness threshold는 추가하지 않았다. RIGHT T→F→V의 물리 목표·동일 X 응답·독립 방향은 미충족이며 `FIELD_VERIFIED_TARGET_IMAGE=false`, `REAL_J_VALIDATED=false`, dimensions0, `Correction=null`이다. A ENDPOINTS_RECORDED는 T/J 검증과 구분한다.

실제 device/motor/torque/settings write0, START0, active deploy0, 원본 Episode 변경0이다. main merge·force push·기존 승인 재사용이 없으며 모든 B/보고 결과는 `executable=false`, `motor_write_authorized=false`다. RGB·Calibration·실측 ticks/device·원본 경로·private JUnit/로그는 공개하지 않는다.

[새 검증 결과](REVIEW_SPRINT_RESULT.json), [고정 HEAD·경로·index 대조](SOURCE_REF_AUDIT.json), [A 후보 검수](https://github.com/Alpenj/DAPIER/blob/1cd0444dfb8bdd1d2a70cd87c90bd4f895990498/2ARM_ROBOT/sweepick/docs/fov_review_20261016/FOV_FAIL_CLOSED_REVIEW.md), [B 계약 대조](https://github.com/Alpenj/DAPIER/blob/0042f8cdfdd9af64b234477b8e1a77e164e01820/2ARM_ROBOT/sweepick/visual_servo/OVERLAY_CONTRACT_DIFF.md)에 근거를 기록했다.
