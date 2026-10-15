# B20 기존 원본 회수 감사

record_id: `DAPIER-2026-10-15-existing-evidence-recovery`

기준은 B20 `4cd684f8fa013d956bd50e74c47acc055dbf6d30`이다. 기존 14개 decision observation 밖의 Episode 전체와 이전 원본을 조사했다. 새 수집·실물 이동·장치 open은 수행하지 않았다. 현재 물리 목표 기준과 독립 X 부호 검증은 여전히 미확보지만, 재사용할 영상·실측 관절·왼팔 보유 근거는 회수했다.

## 조사 범위와 실제 처리량

로컬 Console Run 6개를 목록화했다. 저장 Episode가 있는 5개와 별도 session_18의 오른손목 JPEG를 전부 읽어 원본 SHA와 기존 A red-component 특징을 기록했다. 나머지 Console Run 1개는 저장 Episode 영상이 없다. 우선 두 세션을 먼저 처리한 뒤 이전 세션으로 확장했다.

| 원본 묶음 | 오른손목 JPEG 전체 | 단일 red-component 후보 | 잘리지 않은 후보 |
|---|---:|---:|---:|
| 우선 세션 1 | 10,348 | 2,843 | 1,469 |
| 우선 세션 2 | 7,071 | 2,575 | 1,393 |
| 이전 Console 수신 세션 | 6,270 | 2,063 | 612 |
| Close 중 종료된 이전 세션 | 2,207 | 0 | 0 |
| 예외로 종료된 이전 세션 | 2,432 | 0 | 0 |
| session_18 | 11,613 | 2,670 | 1,601 |
| 합계 | **39,941** | **10,151** | **5,075** |

이는 기존 A의 색상/영역 prior로 분류한 영상량이다. 물리 object identity, 전체 물체 중심, Pad identity를 확인한 개수가 아니다. 전체를 알고리즘으로 분류했고, 추출한 대표 영상과 해당 top/left 문맥을 별도로 시각 검토했다.

105장의 Stage/decision 대표 영상을 원본 바이트 그대로 추출했다. 79장은 물체 영역 후보가 보이고, 59장은 해당 영역이 잘리지 않았다. PREGRASP/빈 jaw/RETURN 문맥도 남겼다. 모든 후보는 `UNVERIFIED_REFERENCE_CANDIDATE` 또는 물체가 없는 문맥 영상이다. `FIELD_VERIFIED_TARGET_IMAGE`는 0개다. 각 원본 파일명·전체 SHA·복사본·실측 상태와 관계는 비공개 원본 보고서에 있다.

우선 두 세션에서는 `frames.jsonl` 51,640행, `joints.jsonl` 382행, `events.jsonl` 322행, `MOTION_HISTORY.jsonl` 36행과 Stage `move.json` 67개를 검토했다. 모든 Stage의 실제 `present` 행을 읽고 recorder의 측정과 합쳤다. 명령·Goal·plan을 실측 관절로 대체하지 않았다. PREGRASP는 left-owned stage이므로 그 executor 행은 왼팔 실측으로 유지하고, 오른팔 상태에는 오른팔의 별도 실제 측정만 연결했다.

## 시간과 lease의 의미

기존 Episode `alignment()`을 그대로 재사용했다. 이는 executor 행의 host 시각보다 늦지 않은 최신 camera receipt와 age를 연결한다. 우선 세션의 관련 Stage는 earlier frame 부재가 없었고, receipt age 최대는 약 36 ms였다. 이 값은 exposure/q 동시성의 증거가 아니다.

Episode의 `recv_host_s`는 원래 `recv_monotonic_s`에 시작 시 한 번 측정한 wall offset을 더한 값이다. SDK exposure와 관절별 sampling 시각은 별도이며, 없는 값은 UNKNOWN이다. `joints.jsonl`이 arm lease 동안 비어 있더라도 Stage `move.json`의 actual `present`, read 시작 시각·소요 시간은 존재한다. 후보에 앞/뒤 실제 read와 source/row를 기록했으며 관절을 보간하거나 현재 상태로 재발행하지 않았다.

우선 세션 1/2의 right stream에는 각각 sequence 누락 53/15개, 최대 receipt gap 약 0.77/0.70초가 있다. 저장된 파일의 존재와 기록 전체의 연속성을 구분했다. 기존 Episode coverage `INCOMPLETE`와 원래 terminal outcome을 보존했다. 두 관찰 이동의 `TARGET_NOT_REACHED_HOLDING`, 추가 세션의 return 실패/수동 복구 요구도 성공으로 바꾸지 않았다.

## 왼팔 보유 의미 수정

기존 A `object_evidence()`는 **물체가 오른팔 Tool에 있는지 평가했는가/보였는가**를 나타낸다. `evidence_valid=true`는 평가 가능성이고, `object_at_tool=false`일 수도 있다. 이전 B 어댑터가 이를 왼팔 보유와 target association에 연결한 것은 잘못이었다.

공유 `left_hold_from_A_record()`로 어댑터와 B19 replay의 같은 오류를 한 번에 수정했다. 왼팔 보유 지원은 기존 LIFT의 `HELD_CONFIRMED`, 영상 동반 이동, lift 중 jaw 측정과 같은 packet의 `CURRENT_OBSERVATION`/left tool/top-frame/feedback identity가 함께 맞을 때만 표시한다. historical lift, torque on, tracked estimate, Search Prior만 있으면 현재 보유는 UNKNOWN이다.

우선 14개 중 7개, 전체 native observation 26개 중 13개에 현재 왼팔 보유를 뒷받침하는 근거가 있다. 나머지는 UNKNOWN이다. 과거 LIFT 확인은 4개 세션에 남아 있다. 이 보유 근거는 물리 6D pose나 right RGB와의 동일 target identity를 인증하지 않는다. cross-view target association은 UNKNOWN이며 모든 REAL correction은 null이다.

## 이전 기준·표식 자료

로컬 Sweepick 관련 루트 105개에서 이름과 관련 JSON 키를 검색했고, 관련 JSON 47개를 원본 대조했다. 중복 archive/fixture도 구분했다. session_18의 7개 native 관측 및 전체 Episode, 기존 right optical/hand-eye/ChArUco 자료, 이후 board-visible/transform audit, 이전 Pad annotation을 포함한다.

재사용 가능한 것은 실제 RGB·actual q/opening·camera route/mount 확인 기록, 기존 printed-board dimension provenance와 REAL board point 대응, LIFT/HELD 증거다. 원본별 경로·SHA·물리적 의미는 로컬 evidence ledger에 있다. board dimension은 기존 operator-confirmed prior이며 새 실측이 아니다. route/mount 확인도 optical/TCP 검증을 대체하지 않는다. 서로 다른 mounting/route epoch를 자동으로 같은 reference로 섞지 않았다.

기존 manual frame identification은 실제 inner Pad 번호·polygon·ordered edge를 null로 남겼다. positive Pad relation 중 확인된 것은 `SYNTHETIC` fixture 또는 모델 ROI/미식별 patch 관계이며 물리 Pad 정답이 아니다. `inner_pad_face_contract.py`는 CAD prior를 만든다. `sweepick_261009_offsite_relative_features.py`는 실제 표시된 patch/ordered edge를 처리할 수 있지만, 없는 Pad identity나 hidden face를 생성하지 않는다.

이후 board-visible 자료도 독립 base/world survey와 물리 mapping 검증이 없어 metric optical truth가 되지 않는다. 작은 PnP RMS를 TCP 정확도로 해석하지 않는다. 기존 ChArUco/hand-eye를 다시 수집하거나 보정치를 변경하지 않았다.

## X 응답과 검증

19개 Stage 전후 비교에서 raw 영상 `delta_u/delta_v`와 actual right ticks 기반 candidate FK의 병진·회전을 분리해 계산했다. 우선 두 세션의 `observe_0_1`은 약 51도 회전이 섞이고, 작은 관찰 이동도 깊이·회전·관점 변화가 있다. centroid 자체의 물리 feature identity와 같은 task reference도 미확인이다. 이 값을 국소 병진 Jacobian으로 fit하지 않았다. 독립 sign 검증은 0개, `REAL_J_VALIDATED=false`다.

기존 B20 전체 실행 시험 31개와 새 좌/우 의미 회귀 5개가 통과했다. 39,941장 분류·105장 원본 동일성·Stage arm 구분·26개 REAL 결과의 null/권한 false를 별도로 확인했다. B17/B18의 변경 없는 근거는 유지했다. 장치 open·motor command·executor·학습·MuJoCo step은 0회다.

재현 가능한 공개 단위검사:

```bash
cd 2ARM_ROBOT/sweepick
python -B -m unittest visual_servo.test_b19 visual_servo.test_b20 visual_servo.test_b20_recovery -v
python -B visual_servo/run_b20.py --inputs "$B20_PRIVATE_INPUTS" --output "$B20_NEW_OUTPUT"
python -B tests/check_public_snapshot.py
```

전체 원본 회수 스크립트·입력 SHA·frame index·extracted images·원본 상세 보고서는 로컬에만 보존한다. 공개 JSON은 익명 요약이다. [최소 현장 계약](B20_MINIMUM_FIELD_CAPTURE.md)과 [A Shadow 인터페이스](B20_A_SHADOW_INTERFACE.md)를 함께 읽는다. 원본 재검수는 여기서 종료하며 실물 수집이나 B21을 시작하지 않는다.
