# B20 A 저장 관측용 Shadow 어댑터

```python
from visual_servo.a_shadow import proposal_from_A_record

proposal = proposal_from_A_record(
    document,                    # A의 원래 OBSERVATION.json 구조
    images_bgr,                  # top / left_wrist / right_wrist uint8 BGR 배열
    record_identity={"run_id": run_id, "observation_id": observation_id},
    record_sha256=record_sha256,
    image_sha256=image_sha256,    # 각 원본 encoded image의 SHA
    right_pad_reference=None,    # 물리 확인 및 현재 observation binding이 있을 때만 제공
    jacobian_evidence=None,      # 독립 검증된 REAL 근거가 있을 때만 제공
    a_source=frozen_source_dir,
    include_geometry=False,
)
```

호출자는 원본 JSON·영상 SHA를 확인하고 실제 arrays를 공급한다. 어댑터는 observation 파일을 다시 수집하지 않는다. 기존 pure A source를 읽고 계산하며, 장치·camera SDK·bus·ROS·executor·network를 열지 않는다. 함수는 입력 JSON이나 영상을 변경하지 않는다.

소비하는 기존 필드는 `source_kind`, `q12`, `actual_feedback.usable/ticks/t_bus_monotonic`, `observation.q12/present_ticks/object_evidence/held_object/left`, `camera_frames`, `files`다. actual q와 native q, actual ticks와 present ticks가 일치해야 한다. command/Goal을 actual state로 대체하지 않는다. 오른 gripper measured tick도 provenance로 남기되 raw `Goal=0`을 opening 목표로 재사용하지 않는다.

카메라 pixel bytes·파일 SHA·image identity·실측 q의 SHA를 기존 B18/B19 binding에 묶는다. host camera receipt와 packet receipt는 기록된 시계를 유지한다. 관절별 측정 시각과 native exposure가 없으면 null/unknown이다. `fresh=true`는 과거 기록의 생산 당시 판정이며 현재 장치 freshness가 아니다.

Native `object_evidence()`는 오른팔 Tool의 물체 존재 평가이며 `evidence_valid`는 평가 가능성을 뜻한다. 이를 left hold나 target identity로 사용하지 않는다. 공유 `left_hold_from_A_record()`는 기존 LIFT의 `HELD_CONFIRMED`/영상 동반 이동/jaw 측정과 현재 REAL left-tool/top-frame/feedback identity가 함께 맞을 때만 left hold를 지원한다. 과거 확인·torque·Search Prior·tracked estimate만 있으면 UNKNOWN이다. 이 결과도 right RGB의 같은 물체 identity나 물리 6D pose는 인증하지 않아 `target_association_verified=null`을 유지한다. recovery의 extraction 미실행을 성공으로 대체하지 않는다.

`right_pad_reference`는 현재 observation·camera·actual feedback에 바인딩된 `CURRENT_OBSERVED_PAD`의 물리 대응과 같은 feature definition이 필요하다. 별도 기준 영상의 binding을 복사해 이 role을 만들지 않는다. 목표 영상 방식은 [최소 현장 계약](B20_MINIMUM_FIELD_CAPTURE.md)의 조건부 `FIELD_VERIFIED_TARGET_IMAGE` 계약으로 분리했으며, 아직 자동 변환/승격 기능은 없다.

빠른 경로는 기존 red surface 함수로 centroid·잘림·상대 오차를 계산하고 `observed_face_graph`를 호출하지 않는다. `include_geometry=true`일 때만 graph를 별도 diagnostic으로 반환한다. graph 성공/실패는 현재 X 목표나 물리 Pad identity를 만들어 주지 않는다. source cache는 resolved path와 세 pure source의 SHA에 바인딩된다. 같은 경로의 파일이 바뀌어도 예전 함수를 재사용하지 않는다.

반환은 기존 `RightRelativeServoProposal`이며 `relative_feature_error_px`, `observed_control_dimensions`, `estimated_tool_correction`, `expected_image_error_change`, `reference_provenance`, `missing_evidence`, `feature_evidence`, `A_record_provenance`를 포함한다. 기준 또는 검증된 REAL J가 없으면 차원 0/correction null이다. 항상 `executable=false`, `motor_write_authorized=false`다.

양 세션 14개 저장 REAL 관측을 이 진입점으로 처리했다. A의 실제 실행 경로에는 연결하지 않았다. A는 기존 `RIGHT_APPROACH` 이후 관측을 전달해 별도 shadow 계산을 할 수 있지만, 이 결과는 기존 stage 전이·IK·collision·Owner·STOP·writer를 호출하거나 바꾸지 않는다. 실제 stage/terminal outcome은 원본 그대로 유지한다.

재현 명령은 [README_B20](README_B20.md)에 있다. 공개 JSON은 익명 기술 요약이며 원본 path·q/ticks·K/D/T·영상·장치 식별정보를 포함하지 않는다. 전체 근거와 source manifest는 로컬 산출물에 보존한다.
