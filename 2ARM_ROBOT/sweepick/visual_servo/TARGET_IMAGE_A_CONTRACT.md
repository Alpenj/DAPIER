# Target Image Consumer — 최소 A 연동 계약

record_id: `DAPIER-2026-10-16-target-image-reference-consumer`

`visual_servo.target_image.TargetImageServoProposal(current_snapshot, reference, jacobian_evidence=None)`는 파일/byte 입력만 받는 순수 계산 함수다. 장치·A 실행기·학습·배포를 호출하지 않는다. 기존 `CURRENT_OBSERVED_PAD`는 기존 `RightRelativeServoProposal`로, 별도 `FIELD_VERIFIED_TARGET_IMAGE`는 이 함수로 전달한다. 두 role의 payload나 Binding을 서로 변환하지 않는다. 기존 Pad/Fast Path/LEFT HELD 구현은 변경하지 않았다.

## 입력 1 — 각 관측의 원본

`current_snapshot`과 `reference['snapshot']`은 각각 다음 필드를 가진다. reference에는 별도 `role=FIELD_VERIFIED_TARGET_IMAGE`, 불변 `reference_id`가 필요하다.

| 필드 | 요구 내용 |
|---|---|
| `observation_identity` | 해당 원본의 `run_id`, `observation_id`. 다른 관측의 이름을 복사하지 않음 |
| `record` | `bytes`, `sha256`: 원본 A `OBSERVATION.json`의 수정하지 않은 bytes와 출처에서 확인한 SHA |
| `image` | `bytes`, `sha256`, `name`: 그 record가 가리키는 원본 오른손목 encoded image. `name`은 `files.right_wrist.rgb`와 일치 |
| `evidence` | 다음 절의 실제 확인 기록을 원본 bytes/SHA로 전달. 없는 근거를 새 bool로 만들어 채우지 않음 |

consumer는 SHA를 실제 bytes에서 재계산하고 영상을 직접 decode한다. q12와 native q12, 양팔 actual ticks와 native Present_Position, usable/fresh/snapshot 여부, bus sequence/feedback stamp, camera reader/fresh 값을 대조한다. 시각·tick은 유한 scalar, sequence/frame 번호는 정수 식별자여야 하며 null끼리의 일치를 허용하지 않는다. 잘못된 중첩 JSON은 구조화된 근거 부족 결과로 반환한다. Goal/command로 actual을 대체하지 않는다. Opening은 actual right gripper Present_Position이다. calibrated q12와 raw feedback의 **원본 일치**를 확인하며, 이 검사로 physical FK/mapping 정확도를 새로 인증하지 않는다.

`snapshot_observation(snapshot)`은 자기 record/image에서 observation·binding·Opening을 재구성하는 helper다. binding에는 `observation_identity`, measured q12 byte의 `feedback_sha256`, image identity/file SHA/decoded array SHA·shape·dtype의 `camera_state_binding`, `record_sha256`, `image_sha256`가 들어간다. evidence 검증 후 그 evidence의 SHA도 최종 binding에 남는다. 기준과 현재의 binding은 각각 자기 bytes에서 계산한다.

## 입력 2 — 물리 확인과 현재 비교 조건

evidence 원본 JSON은 다음 의미를 담아야 한다. REAL은 실제 확인 출처가 있는 기록만 허용하며, TEST_FIXTURE는 `verification_origin=TEST_FIXTURE`로 구분한다.

| 필드 | 의미 |
|---|---|
| `source_kind`, `role`, `binding` | 원본과 같은 REAL/TEST_FIXTURE. 기준 role은 `FIELD_VERIFIED_TARGET_IMAGE`, 현재 role은 `CURRENT_TARGET_IMAGE_OBSERVATION`. 위에서 자기 관측으로 계산한 binding |
| `verification_origin`, `verification_source`, `verified` | REAL에서 `OPERATOR_PHYSICAL_REVIEW`, 실제 확인 기록의 불변 출처, `true`. 모델 추정·익명 주장·합성 확인은 REAL 물리 근거가 아님 |
| `noncontact_target_verified` | 기준 영상에서만 요구하는 실제 비접촉 task 목표 확인. 단순 모델 CLEAR나 영상 중앙으로 대체하지 않음 |
| `held_by_left`, `object_identity_verified`, `feature_correspondence_verified` | 그 시점의 실제 left hold와 물체/같은 물리 특징 대응. A의 오른팔 `object_evidence.evidence_valid`로 만들지 않음 |
| `mounting_verified`, `feature_unoccluded` | 해당 mounting 조건과 선택한 특징이 비교 가능한 실제 근거. 숨은 면·가림을 자동 해소하지 않음 |
| `context` | `object_id`, `physical_feature_id`, `mounting_condition`, `camera_mode`, `calibration_identity`, actual `right_gripper_present_tick` |

hash 일치는 bytes의 무결성 검사이며 물리 사실의 인증을 대신하지 않는다. 이 sidecar는 운영자가 확인한 실제 기록을 전달하는 경계다. A의 신뢰된 입력 담당자가 원본 출처와 확인 주체를 확인해야 하며 모델/정책의 출력을 이 확인 기록으로 받아서는 안 된다. 현재 REAL archive에는 이런 물리 목표 확인이 없어 승격하지 않았다. 새 operator 측정이나 재확인을 이번 구현에서 요청·실행하지 않았다.

두 context의 물체·특징·Opening·mount·mode·calibration 조건과 영상 shape/dtype를 대조한다. 양쪽 값이 같아도 UNKNOWN/MISSING/UNAVAILABLE/UNVERIFIED는 확인된 조건이 아니다. Opening 차이·가림·잘림·대상 불일치·UNKNOWN이면 비교 근거 부족으로 남긴다. 이는 image evidence 판정이며 새 actuator HARD gate나 이동 cap이 아니다. RGB만으로 모든 가림을 판별할 수 없으므로 `feature_unoccluded`의 실제 근거가 없으면 오류를 계산하지 않는다.

## 출력과 REAL Jacobian

목표와 현재가 각각 검증되고 비교 가능하면 `feature_error_px.u = u_current - u_reference`, v는 null이다. 특징은 기존 A fast extractor의 관측된 red component centroid다. 전체 물체 중심·Pad boundary·물리 파지 영역을 생성하지 않고 geometry graph도 호출하지 않는다. `CURRENT_OBSERVED_PAD`의 feature/J 정의를 목표 영상에 재사용하지 않는다.

J가 없거나 검증되지 않으면 영상 X 오차는 남길 수 있지만 `estimated_tool_correction=null`, `REAL_J_VALIDATED=false`다. TEST_FIXTURE도 실제 correction을 생성하지 않는다.

REAL correction이 필요한 향후 입력은 `jacobian_evidence={jacobian, fit_pairs, independent_pair}`다. 기존 J record는 `REAL_MEASURED`, `validated=true`, u의 1×1 행렬, 확인한 3×1 motion basis, 이번 comparison context와 `FEATURES` 정의를 사용한다. 각 pair는 `before`, `after` snapshot과 별도 `motion` 원본 bytes/SHA를 가진다.

motion 기록은 두 sample의 최종 binding, `source_kind`, `role=MEASURED_PHYSICAL_X_RESPONSE`, `axis_scale_verified=true`, `local_translation_comparable=true`, `command_substituted_for_actual=false`, `motion_basis_model`, 실제 `actual_displacement_mm`를 포함한다. candidate FK·command delta·회전/깊이/물체 변화가 섞인 이동을 physical X로 바꾸지 않는다.

consumer는 pair의 실제 영상 특징을 다시 계산하고 기존 `estimate_jacobian`으로 fit matrix를 재현한다. 제공 J는 그 full-precision 결과와 일치해야 한다. J의 `source_sha256`에는 `reference` evidence SHA, `fit_motion` SHA 목록, `independent_motion` SHA를 정확히 연결한다. 독립 after는 fit에서 쓰지 않은 record/observation identity와 **원본 `(run_id, stream, camera n)` capture identity**여야 하며 공유 baseline은 허용한다. 압축·stage·외부 이름 변경으로 같은 capture를 독립 관측으로 만들 수 없다. 실제 새 capture가 동일 화면을 재방문한 경우에는 pixel hash가 같다는 이유만으로 거부하지 않는다. 별도 관측의 actual X와 실제 영상 변화 부호가 맞아야 기존 보정 계산기로 전달한다. 분해능·noise 때문에 부호가 정해지지 않으면 검증되지 않는다. 새 물리 tolerance는 정하지 않았다.

모든 출력의 `executable=false`, `motor_write_authorized=false`는 유지한다. 저장 시점의 fresh 표시는 현재 실물 상태를 뜻하지 않는다. 향후 A는 자기 현재 actual·freshness·Owner·stage·IK·경로·collision·장치 한계·STOP·실행 승인을 그대로 담당한다. 이 소비자를 활성 A에 연결하거나 배포하지 않았다.

## 재현

기존 Python 가상환경에서 `python -B -m unittest visual_servo.test_target_image -v`로 합성 회귀를 실행한다. 실제 저장 음성 입력을 포함할 때는 다음 명령을 사용한다. input manifest의 `sessions`에는 기존 session 경로와 그에 대응하는 `a_source`를 지정한다. 원본·calibration·장치 identity는 로컬에 보존한다.

```bash
python -B visual_servo/run_target_image.py --inputs "$B20_PRIVATE_INPUTS" --output "$NEW_REGRESSION_OUTPUT"
python -B tests/check_public_snapshot.py
```

결과는 `TARGET_IMAGE_CONSUMER_REGRESSION.json`에 기록하며, 원본 SHA·상세 기록·시험 로그는 별도 로컬 실행 증거에만 보존한다. 실제 물리 목표와 REAL J를 확보한 결과로 해석하지 않는다. 결과 제출 후 사용자 승인을 기다린다.
