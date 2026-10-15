# Right reference 최종 현장 검토 후보

record_id: `DAPIER-2026-10-15-right-x-field-handoff`

상태: **FIELD_PHYSICAL_CONFIRMATION_REQUIRED**. `archive_candidate_005`, `037`을 우선 비교하고 `001`, `033`은 각 세션의 접근 시작 대조군으로 사용한다. 최종 shortlist를 확정했으며 물리 목표 영상은 아직 확정하지 않았다. 이번 작업은 이미 끝난 archive recovery 이후의 새 인계다. 전체 영상 재검수, LEFT HELD 의미 수정, 19쌍 X 응답 분석을 반복하지 않았다.

## 네 후보의 판단

| 후보 | 실제 관측 단계 | 물체 가시성 | 실제 Right Jaw / Pad 식별 | 비접촉 목표 판단 |
|---|---|---|---|---|
| 005 — 우선 A | 이전 세션 `03_observe_0_0_right` 마지막 실측 행 부근 | red component 4,687 px, frame 잘림 없음. 윗면·측면 일부가 보이며 검은 부재가 물체 앞을 가린다 | 검은 하드웨어 외형은 보이나 소유 팔·실제 jaw 번호·inner Pad 면/경계 대응은 미확정 | 가려진 접촉면의 간격을 영상으로 확정할 수 없다. **UNVERIFIED_REFERENCE_CANDIDATE** |
| 037 — 우선 B | 다음 세션 같은 단계 마지막 실측 행 부근 | red component 5,550 px, frame 잘림 없음. 005와 영상상 위치·가시 면적이 다르다 | 005와 마찬가지로 실제 inner Pad 경계나 번호를 확정할 수 없다 | 더 큰 가시 면적은 비접촉·파지 가능성의 증거가 아니다. **UNVERIFIED_REFERENCE_CANDIDATE** |
| 001 — 대조군 A | 이전 세션 `02_approach_right_approach_right` 첫 실측 행 부근 | red component 4,218 px, frame 잘림 없음 | 검은 부재와 물체의 중첩이 보이지만 물리 Right Pad 식별은 미확정 | 접근 시작의 외형 비교용. 목표 자세나 X 측정 pair로 채택하지 않음 |
| 033 — 대조군 B | 다음 세션 같은 단계 첫 실측 행 부근 | red component 4,071 px, frame 잘림 없음 | 001과 같은 식별 한계 | 접근 시작의 외형 비교용. 목표 자세나 X 측정 pair로 채택하지 않음 |

가시성·면적은 기존 RGB component 결과를 재사용했다. frame 잘림 없음은 전체 물체·전체 contact surface가 관측되었다는 뜻이 아니다. 왼손목의 전경 물체는 화면 경계에서 잘리고, Top은 어둡고 두 팔/물체가 겹쳐 보여 hidden contact를 해결하지 못한다. CAD projection을 이 영상의 물리 Pad로 해석하지 않았다.

005/037은 저장된 마지막 ARM5 실측값이 같아도 영상이 다르다. 물체 자세·occlusion·카메라 관계가 동일하다고 추정하지 않는다. 영상상 차이를 오른팔 pure X 응답으로 재분류하지 않는다. 관측 이동의 원래 `TARGET_NOT_REACHED_HOLDING` 결과도 목표 도달로 바꾸지 않는다.

## 현장 검토 이미지 읽기

`RIGHT_REFERENCE_OPERATOR_REVIEW.png`의 행 순서는 005 → 037 → 001 → 033, 열 순서는 Right Wrist → Top → Left Wrist다. 12개 패널 모두 실제 원본 영상 전체 프레임이며, wrist는 nearest 2배 확대, Top은 종횡비를 유지한 축소다. 밝기·대비 보정, crop, Pad 선, 물체 파지 가능 영역, 숨은 면 또는 CAD overlay는 없다. 초록 글씨는 **REAL RGB**, 하단 노란 글씨는 **MODEL/CAD는 표시하지 않음**을 뜻한다.

보조 영상은 오른손목 수신 시점보다 앞선 가장 가까운 기존 수신 프레임이다. 다음 차이는 host receipt 기준이며 동시 exposure를 뜻하지 않는다. Exposure timestamp와 exposure 시점의 정확한 q는 UNKNOWN이다.

| 후보 | Top 수신 선행 시간 | Left Wrist 수신 선행 시간 | 해당 프레임의 현재 left-hold witness |
|---|---:|---:|---|
| 005 | 7.41 ms | 23.97 ms | UNKNOWN |
| 037 | 31.40 ms | 12.19 ms | UNKNOWN |
| 001 | 27.39 ms | 20.06 ms | UNKNOWN |
| 033 | 30.80 ms | 12.18 ms | UNKNOWN |

과거 LEFT LIFT의 `HELD_CONFIRMED` 기록은 있으나 네 JPEG에 맞는 현재 native witness는 없다. 다른 시점의 보유 확인을 복사하지 않는다. Top/왼손목 영상은 운영자가 당시 보유·관계 판단을 돕는 실제 문맥이며 자동 identity/보유 확정이 아니다.

## 실제 관절과 Opening의 의미

네 후보의 q와 Opening 출처는 A Stage `move.json`의 `rows[row_index].present`다. Opening의 실측값은 gripper `Present_Position`의 raw tick으로 보고하며 Goal_Position이나 commanded를 대신 사용하지 않는다. 원본과 행은 로컬 상세 원장에 연결되어 있다. raw encoder 값에서 실제 Pad 간격 mm를 새로 계산하지 않았다. SIM gripper profile도 복사하지 않았다.

001/033은 접근 시작 부근의 영상으로 첫 Stage read보다 각각 약 17.01/7.40 ms 앞서 수신됐다. 앞선 right read는 약 31.63/32.36초 전 다른 Stage에 있어 exposure의 실제 자세를 보장하지 못한다. 005/037은 해당 Stage에서 영상 receipt 전후의 actual read가 있으나, 연속 시간 또는 exposure 동기화 증명으로 승격하지 않는다. 정확한 read interval·행·원본 SHA는 로컬 원장에 남겼다.

## 최종 선택 조건

005/037 중 하나를 고를 수 있는 조건은 운영자가 해당 실제 물체·수신 손가락·가려진 면을 포함한 비접촉 관계·지지 상태를 당시 물리 설정과 근거로 확인하고, 의도한 수신 목표라는 의미와 현 적용 context를 기록할 수 있는 경우다. 원하는 물리 관계를 이 12패널로 판별할 수 없으면 두 후보 모두 보류한다. 단순 외형 유사성·모델 CLEAR·영상 중앙·큰 component는 선택 근거가 아니다.

현재 결론은 **FIELD_PHYSICAL_CONFIRMATION_REQUIRED**다. 물리 목표/REAL Jacobian의 검증을 선언하지 않았으며 `REAL_J_VALIDATED=false`, `Correction=null`이다. 구체적인 최소 추가 관측과 실행 범위는 [측정 계획](RIGHT_X_FIELD_MEASUREMENT_PLAN.md), 두 Source Binding의 분리는 [목표 영상 계약](FIELD_VERIFIED_TARGET_IMAGE_CONTRACT.md)을 따른다.

현장 검토 PNG: `RIGHT_REFERENCE_OPERATOR_REVIEW.png` — SHA-256 `d78d0ab92558f99e62ed9fe405ba0410d34394e22ef9c4be4887d3b50d5b8a6f`. 원본 RGB 포함 로컬 전용 산출물이며 공개 저장소에는 포함하지 않는다.
