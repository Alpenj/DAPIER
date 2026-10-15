# Right X 현장 측정 계획

record_id: `DAPIER-2026-10-15-right-x-field-measurement-plan`

상태: **FIELD_PHYSICAL_CONFIRMATION_REQUIRED**. 아래는 미실행 계획이다. 이번 작업에서는 장치를 열지 않았고 A Native·Console·servo setting·학습을 변경하지 않았다. 첫 1D X에 필요한 목표 관계와 방향 증거만 정리하며, 완전한 2D/6D 정합이나 새 calibration solve를 선행조건으로 만들지 않는다.

## 1. 기존 자료로 목표 영상을 확정하는 조건

005/037의 실제 Right RGB와 같은 수신 구간의 Top/왼손목 문맥, actual read 원장을 함께 검토한다. 운영자가 다음 사실을 확인 기록으로 남길 수 있으면 기존 원본을 목표 영상으로 채택할 수 있다.

1. 보이는 물체가 왼팔이 보유한 해당 물체이며, 수신 손가락/Pad의 물리 identity와 의도한 비접촉 수신 관계가 확인된다. 목표 영상 방식에서는 전체 Pad polygon을 만들 필요는 없지만, 가려진 면의 접촉 여부가 끝내 불명확하면 확인 완료로 처리하지 않는다.
2. 당시 영상의 상태가 원하는 task 목표이고, 실제 opening·물체 자세/지지·카메라 mounting epoch·영상 mode가 현재 적용 조건과 비교 가능하다. 모르는 관계를 같은 값으로 채우지 않는다.
3. 같은 물리 특징의 영상 정의와 `u_target`을 선택한 **그 영상**에서 정한다. 영상 중앙을 default target으로 사용하지 않는다. 현재 whole component centroid를 사용한다면 잘림과 occlusion 변화로 정의가 바뀌지 않는지 확인한다.
4. reference 원본 SHA와 자기 관측의 feedback/time binding, 운영자 확인의 출처가 남는다. 현재 프레임의 binding은 별도로 계산한다.

이 중 판단할 수 없는 항목이 있으면 기존 영상은 `UNVERIFIED_REFERENCE_CANDIDATE`로 유지한다. 001/033 대조군, 기존 LIFT, 방향 probe, hand-eye/ChArUco 검사를 반복해서 이 공백을 대신하지 않는다.

## 2. 확정할 수 없을 때 필요한 정지 목표 관측 1건

비어 있는 값은 **물리적으로 확인된 비접촉 수신 목표와 그 영상량**이다. 기존 영상은 가려진 Right Pad/contact 관계와 네 후보의 현재 보유 witness를 확정하지 못하므로 자동으로 정할 수 없다. 이 값이 첫 Right X image error의 기준을 막는다.

필요한 최소 작업은 운영자가 물리 관계를 확인한 한 정지 기준 자세에서 다음을 한 기록 단위로 확보하는 것이다. 그 자세로 이동하거나 수동 조정하는 행위는 이 문서로 승인되지 않는다.

| 확보 항목 | 남길 실제 값 |
|---|---|
| 영상 | Right RGB 원본, 같은 관측 구간 Top/Left Wrist, frame identity·원본 SHA·해상도·색상/rectification mode. 없는 view는 unavailable로 명시 |
| actual 관절 | 양팔 측정 ticks와 canonical q12, 좌우/순서/단위/mapping/calibration identity, actual read 구간·sequence. Goal/command는 별도 |
| Opening | right gripper Present_Position과 실제 관측 가능한 개폐 상태, 가능하면 실제 gap의 근거. mm 미측정은 UNKNOWN; 목표 register나 SIM profile로 채우지 않음 |
| 물체/보유 | 동일 물체 identity·형상·보이는 물리 특징·자세·지지/left hold 근거와 확인 주체. Torque ON만으로 확인하지 않음 |
| 물리 기준 | 어떤 수신 손가락/면이 기준인지, 비접촉 관계를 어떻게 직접 확인했는지, 원하는 task 목표라는 운영자 판정과 출처 |
| 시각/카메라 | receipt/SDK/exposure의 원래 clock, actual read interval, camera mounting epoch/stream 상태. Exposure 미기록이면 UNKNOWN; 정지 feedback bracket을 실제 자료로 남기고 고정 dwell 숫자로 대신하지 않음 |

기존 Episode Recorder가 이미 A owner에 속해 실행 중이면 그 reader와 actual Stage 기록을 사용한다. 별도 정지 수집이 필요할 때만 기존 `sweepick_read03_capture.py`를 A 담당 절차로 재사용한다. READ03도 카메라/포트를 여는 hardware access이며 활성 owner와 동시 실행하지 않는다. 새 수집 프로그램은 만들지 않는다.

## 3. A의 기존 비접촉 이동을 그대로 X 측정에 쓸 수 있는가

확인 범위는 고정한 A source snapshot의 함수와 호출 경로다. 관련 모듈을 import하거나 planner·executor·장치를 실행하지 않았다.

| 경로 | 실제 코드의 의미 | pure X 사용 판단 |
|---|---|---|
| `current_view_candidate.propose_receiver_inner_pad_view()` | 같은 session의 before-pregrasp clearance transform 다음에 카메라 geometry로 정한 tool roll을 수행. fresh 관측은 모든 target 이후 1회 | **그대로 사용할 수 없음**. clearance는 일반 SE(3), 다음은 회전이다. visibility proposal이지 X 식별 동작이 아님 |
| `Owner.plan_observation_candidate()` → A teacher `plan_move()` → `segment(... purpose="OBSERVATION_VIEW")` | `CURRENT_TOOL` 상대 4×4 transform을 A IK/유한 경로/기존 collision·native 검사로 변환. jaw 변경을 거부하고 actual을 기록 | **규약 재사용 가능, 해당 시작 자세의 실행 가능성은 미검증**. 회전 identity와 X만 있는 상대 transform의 schema 수용을 pure 함수 추출 검사로 확인 |
| `acquire_insertion_observation()`의 `observation_only` 분기 | 임의 외부 X proposal을 고르는 UI가 아니라 위 receiver clearance+roll producer를 직접 선택하며 `authorize_observation_plan` 확인 | 기존 관측 버튼/명령을 X 측정이라고 부를 수 없음. X-only proposal을 공급하는 명시적 A 소유 실행 범위가 별도로 필요 |
| 일반 `sweepick_move_a.run(... target=joint_ticks)` | 관절 목표와 native trajectory 실행 | 단일 관절 jog나 joint interpolation을 물리 Cartesian X라고 해석할 수 없음 |

회전 없는 nominal tool-relative X proposal은 `D = [[1,0,0,s],[0,1,0,0],[0,0,1,0],[0,0,0,1]]` 형태다. 여기서 `s`는 나중에 운영자가 승인할 실제 변위이며 이 문서는 숫자를 정하지 않는다. 검사에 사용한 대수 단위값은 실제 이동 제안이 아니다. `CURRENT_TOOL`의 X와 task의 `X_receive`가 일치한다는 실제 근거가 필요하다. 일치하지 않으면 확인된 단위 방향을 당시 tool frame으로 표현해 translation 벡터를 구성한다. image +u와 +X의 부호를 같다고 가정하지 않는다.

5-DOF arm의 특정 시작 자세에서 해당 방향·자세 유지가 가능한지는 A의 기존 IK/경로 결과와 실제 관측으로 판단해야 한다. nominal endpoint가 pure X라도 관절 보간 경로 전체가 straight X라는 보장은 없다. 경로상의 횡이동·회전은 collision/장면 변화를 확인하고, 정지 endpoint의 실제 비교 가능성이 확보된 경우에만 국소 응답을 계산한다. 후보 mapping/FK의 결과를 실제 physical X 검증으로 승격하지 않는다. 해당 자세에서 X-only endpoint를 만들 수 없거나 실제 회전/깊이/물체 변화와 분리할 수 없으면 혼합 응답으로 남기고 J에 쓰지 않는다.

## 4. 동일 기준의 실제 X 응답과 독립 부호 검증

기존 `FINAL_TARGET_ONLY`에서 T/F/V를 하나의 `targets` 목록에 넣으면 마지막 자세의 영상만 남는다. 따라서 baseline T 확보 뒤 F 동작과 fresh 관측, V 동작과 fresh 관측을 각각 연결해야 한다. 기존 observation-only의 단일 candidate 소비 상태를 임의로 초기화하거나 반복 버튼으로 재실행하지 않는다. 이 endpoint별 유한 측정 흐름을 공급할 진입점도 아래의 별도 A 담당 실행 범위에 포함한다.

정지 목표 `T`는 reference이면서 fit baseline으로 재사용할 수 있다. 같은 target과 context에서 `F`는 fit용, `V`는 fit에 넣지 않은 별도 검증용 endpoint로 둔다. baseline 공유는 가능하지만 같은 `T→F` 쌍을 다시 계산한 결과는 독립 검증이 아니다.

1. **축 정의:** 확인된 `+X_receive`의 물리 방향과 모델상 표현, site·단위·근거를 명시한다. command 방향, actual ticks 기반 FK, 실제 물리 방향 확인을 각각 기록한다. FK가 아직 candidate이면 mm gain도 provisional로 남긴다. 완전한 외부 6D pose solve 대신 이 국소 축을 확인할 실제 관측을 사용한다.
2. **Fit `T→F`:** left hold/물체/Opening/mount/특징 정의를 유지한 승인된 오른팔 비접촉 동작 뒤 actual q·Right RGB·보조 문맥을 확보한다. `delta_u = u_F - u_T`, `delta_X_actual`은 확인된 축으로의 실제 변위이며 command displacement와 분리한다. 실제 축 scale을 식별한 경우 `J_uX = delta_u / delta_X_actual` (px/mm). 단위·scale이 미확정이면 REAL metric Jacobian을 만들지 않는다.
3. **혼입 확인:** Y/Z·회전 변화와 물체 slip/Opening/occlusion 변화를 별도로 기록한다. 보이는 centroid 이동을 곧바로 actual X로 사용하지 않는다. 실제 측정 불확실성·task 근거로 분리 가능한지를 판단하며 새 고정 숫자 tolerance를 만들지 않는다. noise 때문에 방향을 구분할 수 없으면 `SIGN_UNRESOLVED`다.
4. **독립 검증 `F→V` 또는 `T→V`:** `V`는 fit에서 제외하고, 같은 축에 대해 별도로 승인한 실제 동작의 방향과 `delta_u` 부호를 관측한다. `sign(delta_u_observed)`와 `sign(J_uX * delta_X_actual)`가 일치하는지 확인한다. target 쪽 이동을 확인하는 경우 error 변화도 기록하고, overshoot/물체 변화와 분리해서 해석한다. 명령 부호나 모델 예측만 맞는 결과로 대체하지 않는다.
5. **채택:** reference/current/fit/독립 관측이 같은 context에 속하고 물리 X·단위·응답·부호가 확인된 경우에만 기존 REAL Jacobian 계약의 `REAL_MEASURED`, `validated=true` 근거를 구성한다. 전에는 `REAL_J_VALIDATED=false`, `Correction=null`이다. reference만 확인돼도 J가 검증되지 않으면 correction을 만들지 않는다.

수집량은 위 역할을 만족하는 최소 관측으로 정한다. 고정 이동 cap·임의 max-step·joint margin·current fraction·dwell/count HARD gate를 추가하지 않는다. 기록 품질 미확인은 B의 evidence 상태이며 새로운 actuator stop threshold가 아니다.

## 5. 필요한 별도 실행 범위와 승인 조건

이 인계는 X-only 실행을 구현하거나 승인하지 않는다. 기존 A 버튼의 clearance+roll을 조용히 바꾸지 않는다. 필요한 다음 범위는 **A owner가 확인된 현재 상태에서 지정 X-only proposal/endpoint를 기존 IK·collision·executor로 실행하고 각 endpoint를 관측하는 유한 right ARM5 비접촉 측정**이다. 기존 proposal 공급 지점의 최소 변경 또는 이미 승인된 A 실행 진입점이 필요하다면 A 담당자의 별도 작업으로 명시한다. B는 Native를 변경하지 않는다.

실행 전 A 담당자가 정확한 source/실행 진입점, 현재 fault·bus·Owner/port 상태, 실제 시작 상태, X frame/부호/변위·motion profile, 관절/장치 identity·EEPROM 범위, left support, right gripper를 쓰지 않는 범위, 전체 경로·collision, 정상 종료/복귀와 STOP/fault 때의 실제 인계 방식을 구체화해 운영자 승인을 받아야 한다. 자동 left motion·grip/release·torque/settings write·HOME/recovery는 범위에 포함하지 않는다. 기존 진입점이 추가 write를 수행한다면 그대로 채택하지 않고 그 write와 필요성을 따로 승인 범위에 올린다. 승인 설명 뒤 fresh actual을 확보하며 과거 q나 FAULT/BUS_LOST 기록으로 현재 상태를 대신하지 않는다.

현재 최종 상태: **FIELD_PHYSICAL_CONFIRMATION_REQUIRED**, `REAL_J_VALIDATED=false`, `Correction=null`, `executable=false`, `motor_write_authorized=false`. 현장 수집·이동·A 변경·새 학습·B21을 시작하지 않고 이 계획 제출로 종료한다.

## 확인한 A source snapshot

| 파일 | SHA-256 | 확인한 경계 |
|---|---|---|
| `sweepick_full_chain.py` | `d97aa60fca02a620d81ed9794e2b1fab24cabeb492940ff4e2379637bfd3db6e` | proposal schema → A IK/경로 검사 → observation-only authorization → actual 관측 |
| `sweepick_move_a.py` | `030d37ee20ce26396b7f40afee1e1817dae8f8a17a4e06af1f9959815af35a05` | joint target executor; CLI도 bus 접근을 포함하므로 호출하지 않음 |
| `current_view_candidate.py` | `9022f8120e16c05d7b251913f61ab083e08ca651a1a0bf8cbebc7627b3dee7db` | 현재 receiver proposal은 clearance + roll |
| `sweepick_read03_capture.py` | `c7c1dde7370dc6ea49a7c4210f11a0222c9bba230b9360138dcfdbe0a07b7349` | 기존 read-only 수집 진입점; 미실행 |
| `sweepick_episode.py` | `2da2b1f839538ae71b0254d991d7b7891c0fb3c6aced2e267fec26f1a8719ad9` | 기존 reader/lease/actual 기록·receipt alignment; 미실행 |

현재 로컬 source의 위 SHA에 한정한 정적 확인이다. A 담당자의 별도 변경·배포나 현재 실물 상태를 검사한 결과가 아니다. 향후 실행은 담당자가 실제 사용 source와 이 경계를 대조한다.

현장 검토 PNG: `RIGHT_REFERENCE_OPERATOR_REVIEW.png` — SHA-256 `d78d0ab92558f99e62ed9fe405ba0410d34394e22ef9c4be4887d3b50d5b8a6f`. 원본 RGB 포함 로컬 전용 산출물이며 공개 저장소에는 포함하지 않는다.
