# UPPER_BODY_INTERFACE_CONTRACT — 상체(양팔) ↔ 베이스 팀 인터페이스 계약 초안 (2026-10-16)

상태: **MOCK/OFFLINE 초안.** 베이스 팀과 아직 합의하지 않았다. 실제 장치, ROS 토픽, 베이스에는 연결하지 않았다. 구현은 `upper_body/sweepick_upper_body.py`(계약 계층)이고, 시험은 Mock 어댑터로만 했다.

## 1. 책임 경계

| 상체 팀(우리) | 베이스 팀 |
|---|---|
| 양팔·그리퍼·상체 카메라·인식, 팔 선택, PICK/HOLD/운반 자세/PLACE·RELEASE, 상체 상태 보고 | 바퀴, OpenCR, Nav2, RTAB-Map, 경로 계획, 도킹, 이동, 베이스 TF·지도 |
| `BASE_BLOCK_REQUEST`로 "다음 이동을 시작하지 말라"고 **요청**만 한다 | 베이스 모터·속도 명령 |

- **카메라 소유:** 전면 SLAM 카메라(`2bc5:0402`)는 TurtleBot3와 함께 **라즈베리 파이(베이스 팀)**에 연결한다(사용자 결정 2026-10-16). 상체는 이 카메라를 직접 열지 않는다. 상체가 쓰는 카메라는 노트북에 연결된 양 손목 RGB와 상단 RGBD(OS30A)다. SLAM 카메라 영상이나 위치가 필요하면 베이스 팀 메시지로 받으며, 그 메시지도 2절 스키마(시계·순번·출처)를 따라야 한다.
- 상체 코드는 베이스를 움직이지 않는다.
- 바퀴가 도는 동안 팔은 움직이지 않는다. 자동 재파지도 하지 않는다.
- 임무 성공은 **미션 관리자**가 선언한다. 베이스 팀의 2번 장소 도착 증거와 상체의 `ARM_DISPOSAL_CONFIRMED`를 결합해야 한다. 상체는 임무 완료를 선언하지 않는다(`mission_completed_by_upper_body=false`).

## 2. 공통 메시지 스키마 `sweepick.upper-body-msg.v1`

모든 메시지에 다음 필드가 있어야 한다. 하나라도 없으면 거부한다.
- `schema`, `kind`, `task_id`
- `sequence_id`(정수)
- `side`(`left`·`right`·`none`)
- `source_kind`(`REAL`·`SIM`·`MOCK`·`REPLAY`), `source_identity`
- `stamp`(유한한 수), `clock_id`
- `status`, `proof_ref`

**순서와 신선도.** 시간 창(초 단위)은 쓰지 않는다. 판단은 순서로 한다.
- 같은 `task_id`와 같은 `clock_id` 안에서 `sequence_id`가 증가해야 한다. 옛 번호나 같은 번호는 무시한다.
- `clock_id`가 바뀌면 비교할 수 없으므로 거부한다.
- 관측과 팔 결과는 그것이 근거로 삼는 베이스 정지 메시지보다 `stamp`가 **뒤**여야 하고 같은 `clock_id`여야 한다. 다른 시계는 신선하지 않은 것으로 본다.

정확한 ROS 토픽·서비스 이름과 타이밍 허용값은 **베이스 팀과 합의한 뒤 정한다.** 지금은 비워 둔다.

## 3. 받는 메시지

| kind | 추가 필수 증거 | 상체의 처리 |
|---|---|---|
| `BASE_PARKED_AT_PICKUP` | `stop_location_id`, `base_stopped_proof`, `controller_idle_proof` | `arrived=true` 한 비트만으로는 거부. 가장 최근 베이스 메시지가 이것이어야 PICK을 시작할 수 있다 |
| `BASE_PARKED_AT_DISPOSAL` | 위 3개 + `disposal_area_id`, `local_observation_permitted` | 1번 장소와 다른 `stop_location_id`여야 한다. 정지 뒤에 새로 찍은 투입 영역 관측이 필요하다 |
| `BASE_MOVING` | — | 팔 동작 전부 차단. 보유 확인(`hold_check`)만 한다 |
| `BASE_STOP_OR_FAULT` | `reason` | 팔 동작 전부 차단 |
| `OPERATOR_AUTHORIZATION` | `scope=UPPER_BODY_PICK_TRANSPORT_PLACE`, `native_manifest_sha256`, `supervised=true`, `recovery_state`(`NO_OPEN_FAULT` 또는 `OPERATOR_RECOVERY_CONFIRMED`), `robot_state_ref` | 이번 task와 이번 Native Manifest에 묶인다. 베이스 팀 신호로 대신할 수 없다. REAL 임무는 REAL 승인이 필요하다 |

## 4. 보내는 메시지

| kind | 보내는 조건 | 뜻이 아닌 것 |
|---|---|---|
| `ARM_NOT_READY(reason)` | 카메라·동기·파지 목표·도달·충돌·장착·FAULT 복구·승인 중 하나라도 불명확할 때 | — |
| `ARM_PICK_CONFIRMED(side, evidence)` | 선택한 팔이 이번 정지 **뒤**의 증거로 held=CONFIRMED이고 lifted일 때 | 임무 완료가 아니다. 과거 `HELD_CONFIRMED`를 복사하지 않는다 |
| `ARM_TRANSPORT_READY(side, evidence)` | 운반 자세 도달, 실제 readback, 그리퍼 보유 CONFIRMED, 팔·탑재체·차체 간섭 검토 PASS가 모두 있을 때 | 베이스 모터 명령이 아니다 |
| `ARM_DISPOSAL_CONFIRMED(side, evidence)` | 받침·여유 공간 확인 → 개방 → **물체 분리 관측**과 **투입 영역 안착 관측**이 따로 모두 있을 때. 그 뒤 안전 종료 | 개방 명령이나 ACK만으로는 보내지 않는다 |
| `ARM_STOPPED`, `ARM_FAULT`, `ARM_RECOVERY_REQUIRED` | STOP, 상실, 서보·버스 FAULT, 외부 owner 점유, readback 불일치, 이미 잡은 상태에서의 거부 | 보유·토크 상태는 `UNKNOWN`으로 남기고 추측하지 않는다 |
| `BASE_BLOCK_REQUEST` | 위 오류 메시지마다 함께 보낸다 | 요청일 뿐 베이스 명령이 아니다 |

## 5. 상체 상태기계

```
IDLE → PICK_AT_STOP → LIFT_HELD → SECURE_TRANSPORT_POSE → HOLD_WHILE_BASE_MOVES → PLACE_AT_STOP → RELEASE_CHECK → ARM_SAFE_END
            ↘ NOT_READY / STOPPED / FAULT / RECOVERY_REQUIRED (어느 단계에서든; 오류 쪽은 BASE_BLOCK_REQUEST 동반)
```

- **시작 전 확인.** 각 단계는 시작 직전에 베이스 상태, 승인, 관측 신선도를 다시 확인한다.
- **움직이기 직전 확인(time-of-use).** 어댑터는 실제로 움직이기 직전에 `still_allowed()`를 다시 부른다. 그 사이 베이스가 움직이기 시작했거나 STOP이 걸렸으면 쓰지 않는다.
- **이 순서는 행동 정의다.** 새 실물 동작 승인이 아니다.
- **기존 LEFT_START_TO_LIFT와의 관계.** 기존 범위는 LIFT 뒤 토크를 켠 채 보유하고 수동 복구로 끝난다. 이것을 `ARM_TRANSPORT_READY`로 바꾸지 않는다. 운반하려면 단일 Owner 유지, 재시작 시 보유 증거, 전원 상실 대응, 운반 중 readback, STOP·복구 계약이 따로 필요하다(`FIELD_READINESS_GATES.md`).

## 6. 팔 선택 (`SELECT_ARM`)

**판정 순서.** 팔마다 다음 순서로 판정한다. 처음으로 True가 아닌 항목이 그 팔의 결과를 정한다(False면 REFUSED, 근거가 없으면 UNKNOWN).
1. `verified_capability`
2. `mechanical_state`
3. `approach_ik`
4. `collision_and_view`
5. `grasp_room`
6. `transport_pose`
7. `place_pose`

**두 팔 비교.**
- 두 팔이 모두 적격일 때만 비교한다. 기준은 기존 계획기가 낸 접근 관절 이동량이다. 직선거리만으로 고르지 않는다.
- 이동량이 같으면 실물 근거가 있는 왼팔을 고른다.
- 비교 값이 없으면 고르지 않는다(`ARM_NOT_READY`).

**REAL 자격.** REAL 모드에서는 `eligible_for_REAL=true`가 필요하다. 오른팔은 실물 독립 PICK이 검증되기 전까지 false다.

## 7. 실제 어댑터(미작성)가 지켜야 할 것

REAL 어댑터는 기존 Native 경로를 재사용해야 한다.
- `sweepick_pick01` / `sweepick_move_a`의 Owner·stage·STOP·IK·충돌·Torque/FAULT
- 좌우 5축 + 그리퍼 mapping과 camera binding

추가 조건:
- 좌우 손목 카메라와 그리퍼 역할이 뒤바뀌지 않았는지 따로 검증한다.
- 어댑터는 결과를 사실대로 보고한다(`held`, `lifted`, `faults`, `readback_mismatch`, `owner_conflict`, `stopped`, `refused`, `moved`). 모르는 값은 `UNKNOWN`이다.
- 전압·전류·Status 이상은 기존 실행기가 판정한 FAULT를 그대로 전달한다. 여기서 새 문턱값을 만들지 않는다.
