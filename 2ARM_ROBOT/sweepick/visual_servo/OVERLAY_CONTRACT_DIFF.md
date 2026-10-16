# Dual Wrist Overlay 계약 대조 — 2026-10-16

기준은 `b270a4f`다. 이 변경은 각도 provenance와 오프라인 회귀만 엄밀화한다. Native·Console·Motor Writer·Calibration은 수정하지 않는다.

| 항목 | 이전 | 이번 후보 |
|---|---|---|
| 확인된 영상 edge vs 설정값 | `OBSERVED_EDGE_VS_EXISTING_IMAGE_REFERENCE` | `OBSERVED_EDGE_VS_CONFIGURED_IMAGE_REFERENCE`; `reference_source`에 `current_wrist_alignment.square_at_deg`와 configured target 의미 기록 |
| 확인된 영상 edge vs 모델 방향 | `OBSERVED_EDGE_VS_MODEL_PRIOR_DIRECTION` | 같은 role 유지, `model_projection.closing_line_uv` 출처 명시 |
| 실측 Gripper 방향/3D yaw | 미관측 | `observed_gripper_direction_deg=null`, `physical_3D_yaw=null`; 설정값이나 모델을 실측 방향으로 승격하지 않음 |
| 현재 8 REAL 상대 각도 | UNKNOWN | 모두 UNKNOWN 유지; 새로운 REAL 값 생성 없음 |
| 왼팔 X | 물체 최광폭 행 중심 − current-confirmed jaw midline | 정의 유지; 수평 clipping 시 null, 바닥 clipping은 기존 wrist_offset의 현재 확인/가독성 조건에서 X만 허용 |
| 물체 중심 | visible-component centroid | 부분 가시 영역의 영상 중심; 실제 2D 파지 중심/Y 목표 아님 |
| 하강 모델 | MODEL_PRIOR top envelope | 실제 실패 X-clip을 놓친 negative 결과 유지; A `FOV_OK`/ALIGNED와 연결 없음 |

8개 원본 RGB의 file/decoded SHA·camera/source/frame binding을 이전 R3와 대조해 동일함을 확인했다. 원본 8장과 새 비교 PNG 2장을 이번 로컬 실행자가 직접 확인했다. 외부 리뷰 문서 작성자의 미열람 상태와 구분하며, 이 육안 QA는 실제 Pad 좌표·가려진 접촉면·3D yaw 인증이 아니다. 원본, 별도 debug 출력, 공개 익명 요약을 분리했다. PNG는 이전 결과와 바이트가 동일하다.

이번 replay에서 요청된 4개 unittest 모듈 71개를 실행했고 실패/오류는 0이다. 선택적 `ActualAModelTests` 클래스 1건은 private frozen fixture 미지정으로 skip이다. 이전 69개 결과는 별도 prior 필드에 보존했다. 추가된 두 회귀는 configured target과 관측 방향의 혼동, MODEL INSIDE와 observed X-clip 충돌을 검사한다. 별도 target-image/metrology bridge 38개 검사도 실패/오류/skip 없이 새로 통과했다. 기존 가림/미확인 edge·윤곽 chord·다른 frame·overlay 재입력·X clipping·바닥 clipping 시험도 함께 실행했다.

검증 수치와 미확정 항목은 [이번 replay 요약](reports/OVERLAY_REPLAY_SUMMARY.json), 원본 불변·camera/source binding은 [Source SHA audit](reports/SOURCE_SHA_AUDIT.json)에 있다. 기존 [오버레이 검증 JSON](reports/DUAL_WRIST_OVERLAY_VALIDATION.json)은 당시 기록으로 보존했다. RIGHT T/F/V의 남은 조건은 [측정 계약](B20_RIGHT_X_MEASUREMENT_CONTRACT.md)을 따른다.

원본 RGB/Calibration/실측 ticks/device/절대 경로와 전체 replay 원장은 로컬에만 보관한다. 모든 결과는 `executable=false`, `motor_write_authorized=false`, `Correction=null`이다. 실제 구동·활성 적용·새 수집·광학 재추정·전체 archive 재분류를 수행하지 않았다.
