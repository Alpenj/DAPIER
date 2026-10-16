# B20 실제 Inner Pad 식별 갱신

record_id: `DAPIER-2026-10-16-physical-pad-identification`

**실물 Inner Pad의 의미는 운영자가 확인했다.** 오른팔 SO101의 두 gripper 손가락이 닫히면서 서로 마주 보는 안쪽 접촉면이다. 나는 제공된 외부 사진 3장과 기존 `archive_candidate_005/037`의 Right Wrist RGB 원본을 직접 대조했다. 이번에 해결된 것은 실물 부품의 의미와 구조적 구분이며, 손목 영상의 Pad 좌표나 비접촉 목표 자세를 검증한 결과는 아니다.

| 구분 | 이번에 확인한 내용 | 유지하는 한계 |
|---|---|---|
| **OPERATOR_CONFIRMED** | 실제 Inner Pad는 두 finger의 서로 마주 보는 안쪽 접촉면 | 물리 명칭 확인을 특정 프레임의 Pad polygon·jaw 번호·목표 자세 확인으로 복사하지 않는다. 사진 세 장은 닫힘 동작의 실측 시퀀스가 아니다. |
| **VISUALLY_OBSERVED** | 외부 사진에서 열린 두 finger, 검은 finger/프레임, 카메라 PCB와 경사 지지대를 구분할 수 있다. 일부 안쪽 면과 반복 홈·톱니 같은 외형이 보인다. | 표면의 정확한 평면·두께·재질·전체 윤곽·접촉 patch를 측정하지 않았다. 카메라 PCB/지지대는 Pad라는 명칭의 대상과 구분한다. |
| **VISUALLY_OBSERVED** | 사진 1은 근경·원경 gripper의 구조를, 사진 2는 가까운 열린 finger와 카메라 지지부를, 사진 3은 양팔과 두 손목 카메라의 배치를 보여 준다. | 사진의 근경/원경·화면 좌우를 robot side 또는 model jaw 1/2로 자동 매핑하지 않는다. 운영자가 지정한 오른팔 하드웨어 문맥과 프레임 안의 개별 부재 매핑은 별개다. |
| **UNVERIFIED** | Pad라는 물리 부품을 무엇이라 부르는지는 더 이상 공백이 아니다. | archive의 가려진 접촉면, Pad↔물체 간격, 접촉 여부, 원하는 비접촉 수신 관계와 그 `u_target`은 여전히 미검증이다. |

외부 촬영 카메라와 robot Right Wrist 카메라는 다른 관측계다. 외부 사진을 archive의 같은 시각·같은 q12·같은 mounting epoch로 간주하지 않았고, 사진 파일 시각을 exposure 시각으로 사용하지 않았다. 외부 사진을 기존 wrist calibration의 입력 영상으로 대체하거나 새 hand-eye/scale을 계산하지 않았다.

| 기존 후보 | 새 외부 사진과 비교해 설명 가능한 관계 | 여전히 확정할 수 없는 것 |
|---|---|---|
| `archive_candidate_005` | red block의 윗면·측면 일부가 보이며 검은 전경 하드웨어가 앞/아래쪽 영역을 가린다. 외부 사진은 finger·housing/프레임·카메라 지지대가 서로 다른 부재임을 보여 준다. | 해당 검은 silhouette의 소유 팔·특정 finger·Pad 1/2·브래킷 여부를 픽셀별로 확정할 수 없다. 가려진 면의 간격·접촉 상태는 UNKNOWN이다. |
| `archive_candidate_037` | 005와 유사한 가림 구조이며 영상상 물체 위치·가시 면적은 다르다. 실제 finger의 안쪽 면이라는 Pad 정의를 적용해 해석 범위를 좁혔다. | 가시 면적 증가나 외형 유사성은 Pad boundary·비접촉·파지 가능성의 증거가 아니다. 이 프레임도 `UNVERIFIED_REFERENCE_CANDIDATE`다. |

검은 물체 전체를 Inner Pad로 표시하거나, 모든 검은 전경을 카메라 브래킷으로 재분류하지 않는다. 005↔037을 새로운 X 응답 pair로 만들지 않았고, 기존 actual read와 `TARGET_NOT_REACHED_HOLDING` 결과도 변경하지 않았다. 기존 원본 RGB·Calibration·관절 기록을 재사용했으며 동일 촬영·방향 probe·정지 capture를 요구하지 않았다.

로컬 CAD/MJCF와의 대조는 다음 수준이다. 원본 파일·관련 mesh의 SHA는 사진 원장과 함께 로컬에 기록했다. 공개 요약에는 사진·Calibration 원본·장치 identity·개인 경로를 넣지 않는다.

| 기존 모델/계약 | 확인한 구현 의미 | 물리 확인과 구분 |
|---|---|---|
| `pgripper.py` / 기존 provenance | 두 jaw visual mesh와 반대 방향 coupling, distal collision hull, inner datum site를 만든다. 카메라 mount와 PCB/lens visual proxy는 별도 요소다. | 마주 보는 두 접촉면이라는 **의미는 운영자 정의와 부합**한다. 이 사실로 NORMA CAD mesh의 실물 치수·표면·SO101 mount·jaw 번호 대응을 인증하지 않는다. |
| 로컬 export MJCF `portable_model.xml` | `right_pgripper_jaw_1/2_visual`, `right_pgripper_pad_1/2`, 각 `*_inner` site, camera mount/PCB/lens 요소가 분리되어 있다. | visual jaw, collision hull, datum site는 서로 다른 모델 표현이다. 사진에서 새로운 3D 점·normal·gap을 생성하거나 site를 실측 좌표로 바꾸지 않는다. |
| `inner_pad_face_contract.py::inner_face_contract()` | 기존 CAD inner datum plane에 맞는 visual triangles·boundary·approach-side edge를 선택한다. float32 precision은 CAD 표현 오차용이다. | `role=MODEL_PRIOR`, `physical_mount_identity_verified=false`, `observed_boundary=false`를 그대로 유지한다. Pad의 물리적 정의가 확인돼도 해당 모델 surface의 실측 검증은 아니다. |
| `project_contract()` / camera provenance | projection과 facing은 MODEL_PRIOR이며, camera seat·optical centre/roll은 기존 추정이다. MJCF의 camera mount/PCB/lens는 visual 요소이며 별도 collision 형상을 검증한 상태가 아니다. | `visibility_is_observation=false`를 유지한다. 사진에서 지지대가 보이는 사실은 camera extrinsic·모델 image roll·metric scale·실제 collision CLEAR 검증과 다르다. 사진으로 가려진 collision 체적을 만들지 않고 기존 collision 검사도 변경하지 않는다. |

아직 비어 있는 정보는 **프레임별 실제 Pad 영상 대응**, **가려진 Pad↔물체 관계와 의도한 비접촉 목표**, **필요할 때 사용할 물리 X 응답·단위 근거**다. 전체 Pad polygon이나 완전한 6D 정합을 새로운 선행조건으로 추가하지 않는다. 기존 자료에서 판단 가능한 범위를 먼저 사용하며, 이번 갱신으로 새 촬영·수동 동작을 요청하지 않는다.

현재 상태는 다음과 같다. 이는 구조 식별 기록이며 기존 B current/reference evidence 문서가 아니다.

```json
{
  "physical_pad_definition_confirmed": true,
  "archive_pad_image_correspondence_verified": false,
  "physical_model_mount_verified": false,
  "FIELD_VERIFIED_TARGET_IMAGE": false,
  "REAL_J_VALIDATED": false,
  "Correction": null,
  "executable": false,
  "motor_write_authorized": false
}
```

사진 3장은 원본 bytes·SHA-256·저장 경로와 함께 로컬 전용으로 보존했다. 실제 코드·보정·하드웨어 설정은 바꾸지 않았다. 기존 74개 회귀 결과는 변경 없는 계산 코드의 이전 검증으로 보존하며 이번 사진 검토의 새 실물 검증 결과로 세지 않는다. 이번에는 사진 보존본/원본 SHA 일치, 후보 2장 출처, 기존 모델·계약·Calibration 무변경, 문서 상태와 공개 경계를 확인했다.
