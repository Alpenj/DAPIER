# FIELD_VERIFIED_TARGET_IMAGE 최소 데이터 계약

record_id: `DAPIER-2026-10-15-field-verified-target-image-contract`

2026-10-16 [실물 Pad 식별 갱신](PHYSICAL_PAD_IDENTIFICATION_UPDATE.md): 실제 Inner Pad의 물리 정의는 운영자가 확인했다. 이 사실은 **구조 식별 근거**이며, 아래 계약의 영상별 비접촉 목표 확인·current binding·mount 확인을 대신하지 않는다. 외부 사진 확인을 FIELD reference나 current evidence로 변환하지 않았다.

상태: **PROPOSED CONTRACT / FIELD_PHYSICAL_CONFIRMATION_REQUIRED**. 현재 네 후보는 모두 `UNVERIFIED_REFERENCE_CANDIDATE`다. 이 문서는 독립된 reference-image role을 정의하는 인계이며, 기존 어댑터에 새 role을 구현하거나 물리 검증을 수행한 결과가 아니다. 현재 `extract_right_features()`는 REAL reference에 `CURRENT_OBSERVED_PAD`와 current binding을 요구한다. 목표 영상을 그 role로 위장하거나 과거 binding을 current로 복사해서 이 검사를 통과시키지 않는다.

## 승격 조건과 상태

`UNVERIFIED_REFERENCE_CANDIDATE → FIELD_VERIFIED_TARGET_IMAGE` 승격에는 운영자가 확인한 실제 비접촉 task 관계, 물체/수신 손가락 identity, 확인 방법·근거·판정 출처, 불변 원본 image/actual feedback binding이 필요하다. 확인되지 않은 hidden Pad 경계·접촉 간격·파지 가능 영역을 생성하지 않는다. 실제 Pad polygon 전체가 없어도 확인된 목표 영상 방식으로 1D X를 정의할 수 있으나 물리 목표 관계의 확인은 생략하지 않는다.

목표 검증과 current 적합성, REAL J 검증, 실행 권한은 서로 별도 상태다. 목표 확인만으로 J나 current 물체 association을 확인한 것으로 처리하지 않는다. 실제 운영자 확인이 비어 있으면 `role=UNVERIFIED_REFERENCE_CANDIDATE`, `FIELD_PHYSICAL_CONFIRMATION_REQUIRED`를 유지한다.

## 두 Source Binding을 분리한다

| 항목 | Reference binding — 과거 기준 자체 | Current binding — 지금 평가하는 실제 관측 |
|---|---|---|
| identity | 자기 `run_id`, `observation_id`, camera/stream/frame identity, 원본 경로의 불변 record key | 자기 새 run/observation/frame identity. reference 값을 복사하지 않음 |
| image | 원본 file SHA, decoded array SHA·shape·dtype·색상/rectification mode | 현재 원본 SHA와 현재 decoded array SHA·shape·dtype/mode |
| actual | 기준 관측의 measured ticks/q12, mapping·좌우/순서/단위·feedback source SHA와 sequence/read interval | 현재 자기 measured ticks/q12, source SHA·sequence/read interval. Goal/command를 actual로 쓰지 않음 |
| 기존 binding 구조 | `observation_identity`, `feedback_sha256`, `camera_state_binding`를 **기준 관측**에서 산출 | 같은 필드들을 **현재 관측**에서 별도 산출 |
| time | 원래 receipt/SDK/exposure clock과 값·정지 read bracket. 없는 exposure는 null/UNKNOWN | 현재 관측의 자기 clock/read bracket. 과거 clock 숫자와 직접 비교하지 않음 |
| physical confirmation | 해당 영상의 비접촉 관계 확인, 대상/손가락 identity·확인 주체/방법/출처 | current 물체 identity/left hold/Opening/context 적합성. 과거 확인으로 현재 보유를 대신하지 않음 |

기존 `feedback_sha256`는 measured canonical q12 bytes의 SHA이고 원본 feedback 파일 SHA와 구분한다. `camera_state_binding`에는 image identity, file source SHA, decoded array SHA·shape·dtype를 남긴다. reference/current가 같은 실제 record인 경우를 제외하고 binding 동일성을 요구하지 않는다. 명시적인 comparison edge가 두 관측을 연결하며 두 binding 모두 불변 원본을 가리켜야 한다.

## 최소 필드

| 필드 | 계약 |
|---|---|
| `reference_id`, `source_kind`, `role` | 불변 reference identity, `REAL`, 물리 확인 후에만 `FIELD_VERIFIED_TARGET_IMAGE` |
| `reference_binding`, `current_binding` | 위의 두 독립 binding. 각각의 source record/image/actual feedback를 검증 가능해야 함 |
| `physical_confirmation` | `verified`, method, operator confirmation record source/hash, 확인한 비접촉 관계·물체/수신 손가락 identity·남은 가림/불확실성 |
| `feature_definition` | 관측 가능한 같은 물리 target의 동일 영상량. 예: whole visible red-component centroid의 u. 물리 전체 중심이라고 주장하지 않음 |
| `target_value_px` | 확인된 reference의 `u_target`; 영상 중앙·CAD projection·다른 시점의 값을 대신 넣지 않음. 첫 X에서 v/depth/rotation은 null |
| `comparison_context` | 물체 identity/형상/지지·left hold 근거, Right Opening의 실측 상태, camera/mounting epoch·mode, occlusion/특징 정의의 비교 가능성. 모르는 조건은 UNKNOWN |
| `X_receive_basis` | 실제 확인한 단위 방향·부호와 A model 표현, site·단위·물리 확인 출처. candidate FK와 measured physical displacement를 구분 |
| `fit_evidence`, `independent_sign_evidence` | 각 관측 자기 binding/source SHA, actual displacement·단위/scale 근거, command와의 구분, fit 제외 독립 관측 identity 및 부호 결과 |
| `jacobian_evidence` | 기존 `matrix_px_per_mm`, `feature_axes`, `motion_basis_model`, `source`, `context`, `feature_definitions`, `source_sha256`, `validated`와 연결. 새 target feature 정의에는 이전 observed-Pad J를 묵시적으로 재사용하지 않음 |

독립 비교 record는 `reference_id + reference_binding + current_binding + comparison_context + feature_definition`을 명시한다. 미래 producer가 기존 `context`로 전달할 때 이 reference/version·feature·axis/mount 관계가 보존되어야 한다. 이미지 차이를 계산할 수 있다는 사실만으로 물리 대응이 확인되는 것은 아니다.

## 비교와 보정의 판단

1. reference 자체의 물리 확인이 없으면 목표 미확정으로 남긴다.
2. current의 actual/카메라 freshness·left held target association·Opening/mount/물체/feature 조건이 미확정이면 비교 불가로 남긴다. 다른 시점의 evidence를 옮겨 채우지 않는다.
3. whole-component centroid가 잘리거나 가림 때문에 정의가 달라지면 그 특징을 제어에 쓰지 않는다. 별도 물리 edge identity 없이 부분 edge로 바꾸지 않는다.
4. 위 조건이 확인되면 `e_u = u_current - u_reference`를 1D 영상 오차로 표현할 수 있다. Y·깊이·회전은 관측되지 않은 채 null이며, 이 오차를 TCP mm 정확도로 해석하지 않는다.
5. REAL J가 fit·실제 축 scale·독립 sign/context 검증을 통과하기 전에는 **Correction은 계속 null**이다. `REAL_J_VALIDATED=false`와 `validated=false`를 유지한다. 검증 뒤의 비실행 제안도 A의 현재 IK·경로·collision·장치·STOP·Owner와 별도 승인을 대신하지 않는다.

이 계약의 미확정은 B의 evidence 상태다. 임의 travel cap·joint margin·max-step·current fraction·dwell/count threshold를 새 actuator HARD gate로 추가하지 않는다.

```json
{
  "status": "FIELD_PHYSICAL_CONFIRMATION_REQUIRED",
  "reference_role": "UNVERIFIED_REFERENCE_CANDIDATE",
  "FIELD_VERIFIED_TARGET_IMAGE": false,
  "REAL_J_VALIDATED": false,
  "Correction": null,
  "estimated_tool_correction": null,
  "executable": false,
  "motor_write_authorized": false
}
```

## 이번 제출의 검증 범위

네 후보의 12개 원본 view와 기존 추출본 SHA를 대조하고 PNG의 실제 패널 픽셀 보존과 레이아웃을 확인했다. A의 pure proposal schema 함수만 AST로 추출해 회전 없는 CURRENT_TOOL X 표현 수용, 미확인 frame과 imported q12 거부를 확인했다. 이는 IK/장치/물리 X 성공 검사가 아니다. A Native·B20 실행 Python을 수정하지 않아 기존 31개 실행 시험과 의미 회귀 5개의 PASS 근거를 재사용하며, 이 시험들을 새 실행 결과로 보고하지 않는다. 이전 전체 archive 재검수와 19쌍 분석도 재실행하지 않았다.

이번 작업은 정확히 세 Markdown과 한 현장 검토 PNG를 제출한다. PNG에는 실제 원본 RGB가 포함되어 공개 GitHub에 올리지 않으며 로컬 파일과 SHA로 인계한다. 공개 문서는 원본 영상·actual q/ticks·calibration·device identity·수신 timestamp·개인 경로를 포함하지 않는 기술 요약이다. 새 커밋은 이 문서 인계이며 이전 recovery commit을 새 결과로 제출하지 않는다.

현장 검토 PNG: `RIGHT_REFERENCE_OPERATOR_REVIEW.png` — SHA-256 `d78d0ab92558f99e62ed9fe405ba0410d34394e22ef9c4be4887d3b50d5b8a6f`. 원본 RGB 포함 로컬 전용 산출물이며 공개 저장소에는 포함하지 않는다.
