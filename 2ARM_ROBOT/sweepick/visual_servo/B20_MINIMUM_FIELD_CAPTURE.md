# B20 기존 근거 이후의 최소 현장 계약

기존 원본에서 실제 영상·actual q/opening·LIFT/HELD와 일부 현재 왼팔 보유 근거를 회수했다. 같은 측정·카메라 identity probe·LIFT 확인·ChArUco 보정을 다시 요청하지 않는다. 현재 비어 있는 것은 물리적으로 확인된 **비접촉 task 목표**, 같은 target의 실제 X 응답, 독립적인 방향 검증이다. 현장 수집과 움직임은 실행하지 않았다.

## 먼저 기존 후보를 확인할 수 있는가

로컬 후보 보고서의 105개 추출 영상 중 59개는 red component가 잘리지 않는다. 각 후보에는 원본 SHA, 실제 Stage/관절 read, right opening의 앞/뒤 실측, camera receipt, top/left 문맥, 같은 native observation이 있을 경우의 left-hold witness가 연결되어 있다. 이는 검토할 **후보**이며 아직 비접촉 목표가 아니다.

운영자가 당시의 물리 설정과 보조 영상을 근거로 특정 후보가 원하는 수신 비접촉 관계였음을 확인할 수 있다면, 먼저 그 원본을 채택할 수 있는지 검토한다. 검은 jaw가 보이거나 nominal path가 clear였다는 이유만으로 확인을 대신하지 않는다. 뒤쪽 Pad/contact가 가려져 판단할 수 없으면 UNKNOWN을 유지한다. 과거 기준이 현재 물체·형상·opening·mounting epoch에 적용되는지도 별도로 확인한다.

기존 후보로 확정할 수 없을 때만, 운영자가 물리적으로 확인한 한 비접촉 기준 자세에서 right RGB와 actual feedback/opening을 함께 기록하는 한 번의 정지 관측이 필요하다. 이 단계가 `u_target`을 정의한다. 새 이동량 cap, 임의 dwell, reach/error tolerance를 제안하지 않는다.

## 조건부 FIELD_VERIFIED_TARGET_IMAGE 계약

목표 영상 방식은 물리 Pad를 직접 검출하지 않아도 첫 1D X 목표를 표현할 수 있다. 다만 이번 자료에서 그 물리 관계의 유효성을 확정하지 못했다. 아래는 확인 후 사용할 최소 계약이며, 현재 어댑터에 이 role을 구현하거나 correction을 생성한 것이 아니다.

| 필드 | 필요한 실제 근거 |
|---|---|
| `role` | `FIELD_VERIFIED_TARGET_IMAGE`; `CURRENT_OBSERVED_PAD`로 위장하지 않음 |
| `reference_id`, `source_kind` | 불변 reference identity와 `REAL`; 확인 방법·운영자 확인 기록의 출처 |
| 목표의 물리 의미 | 확인한 비접촉 수신 관계, 실제 `+X_receive` 방향, image feature/`u_target` 정의. 영상 중앙을 기본값으로 쓰지 않음 |
| 기준 영상과 상태 | 원본 image SHA·camera frame identity·actual q/feedback source SHA·실측 gripper opening·각 read interval |
| 적용 context | 같은 물체 identity/형상, 왼팔 보유와 물체 자세, camera mounting epoch·영상 mode·opening 조건. 적용 여부 UNKNOWN이면 사용하지 않음 |
| 시간 | 원래 host receipt/SDK clock과 실제 관절 read 구간. exposure 미기록이면 unknown; 정지 관측의 실제 feedback bracket을 보존하고 고정 dwell로 대체하지 않음 |
| binding | reference는 자기 관측의 identity/hash를 유지. current는 별도로 계산한 자기 binding을 유지. 비교 관계는 두 source를 명시적으로 연결하며 서로의 binding을 복사하지 않음 |

whole-object centroid를 가정하는 특징이 잘렸다면 사용하지 않는다. 부분 모서리를 목표로 쓰려면 그 모서리의 물리 identity와 별도 target definition이 필요하다. 현재 코드는 이를 새로 구현하지 않았다. Y·깊이·회전 값을 채우거나 완전한 2D/6D 검증을 첫 X 시험의 선행조건으로 만들지 않는다.

## X 식별에만 남은 관측

검증된 target을 얻은 뒤에는 같은 물체/기준/조건의 baseline과 실제 X 이동 후 영상·actual feedback을 연결해야 한다. 명령량과 실제 관절/FK 이동을 각각 남긴다. 기존 archive의 회전·깊이·물체 변화가 섞인 이동을 pure translation pair로 재분류하지 않는다.

fit에 사용하지 않은 후속 관측으로 실제 보정 방향과 영상 오차 변화의 부호를 확인해야 한다. 기존 joint 방향 probe나 left PICK 부호는 right RECEIVE의 독립 sign 검증을 대신하지 않는다. baseline을 공유할 수 있지만 같은 fit pair를 재사용한 검사를 독립 검증이라 부르지 않는다. 이 관측이 없으면 `REAL_J_VALIDATED=false`, correction null을 유지한다.

## 기존 도구와 승인 경계

정지 관측은 기존 `sweepick_read03_capture.py` 또는 Episode Recorder의 이미 있는 camera/feedback 기록을 재사용한다. arm lease 중 실제 관절은 기존 Stage `move.json`에서 읽는다. 새 수집 프로그램을 만들지 않는다. READ03도 camera/port를 여는 hardware access이므로 지금 실행하지 않는다. 기존 Owner·포트 점유와 함께 사용할 수 있는 경로를 A 담당 절차 안에서 선택해야 한다.

현재 보유·복구 상태는 과거 archive로 판단하지 않는다. 현장 복구/현재 상태 확인, 정확한 관측 또는 right-only 비접촉 동작 범위, 기존 STOP·IK·경로·충돌·장치 identity·fresh feedback·종료 절차에 대한 별도 승인이 있어야 한다. 과거 FAULT와 수동 복구 요구를 보존한다. B는 A Native/Console이나 활성 배포를 변경하지 않는다.

이 문서는 추가 실행 요청이나 승인이 아니다. 기존 원본 재검수 결과와 필요한 최소 근거만 제출하며, 자동 수집·실물 움직임·B21은 시작하지 않는다.
