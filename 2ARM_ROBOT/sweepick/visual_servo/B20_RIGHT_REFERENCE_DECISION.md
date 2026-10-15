# B20 오른팔 물리 기준 판정

record_id: `DAPIER-2026-10-15-right-reference-readiness`

B19 `3be52c8` 이후 저장 관측을 검토했다. 첫 REAL 비접촉 X 정렬에 사용할 목표 기준은 아직 확보하지 못했다. 구현·저장 영상 회귀·CPU 비교는 완료했으며, 실물 정렬은 실행하지 않았다.

| 방식 | 직접 확인한 근거 | 부족한 근거 | 판정 |
|---|---|---|---|
| A. 현재 RGB의 실제 Right Pad | 두 세션 14개 관측, red component·검은 jaw 후보와 CAD Pad projection | 영상의 어느 경계/면이 실제 inner Pad인지 확인한 대응, 현재 물체와 그 기준의 관계 | `PHYSICAL_PAD_REFERENCE_UNOBSERVED` |
| B. 확인된 비접촉 기준 자세의 목표 영상 | actual q·RGB·measured opening·receipt 기록, 기존 관찰 후보 자세 | 그 자세가 원하는 비접촉 task 관계라는 물리 확인, 같은 물체·형상·장착·개방량의 목표 특징 정의 | `PHYSICAL_REFERENCE_POSE_UNVERIFIED` |

검은 jaw가 보인다는 사실만으로 inner Pad의 면/경계를 식별하지 않았다. `000023` 관찰 후보는 red component가 보이지만 native relative alignment가 `MISSING`이고 `separately_observed_pads=[]`다. CAD nearest point, PnP assignment, 영상 중앙을 목표로 사용하지 않았다. B 방식은 A 방식의 Pad 경계를 반드시 요구하지 않지만, 비접촉 기준 자세의 물리적 의미를 확인해야 한다.

이전 세션은 `ENDED / success / normal_return_confirmed=true`이면서 task outcome은 `PARTIAL_OR_DIAGNOSTIC`이다. 추가 세션은 `FAULT / partial / normal_return_confirmed=false`이며 return 과정의 `BUS_LOST`를 보존했다. 두 세션의 관찰 이동 `TARGET_NOT_REACHED_HOLDING`도 정렬 성공으로 바꾸지 않았다. 정상 return, RGB 후보 가시성, 목표 정렬은 다른 결과다.

관찰 단계의 오른팔 arm 5축 torque는 기록상 on이고 gripper torque는 off다. 오른 gripper의 실제 opening은 기록되어 있지만 `Goal=0`을 opening 목표나 접촉 성공으로 해석하지 않았다. 왼팔의 powered hold context와 native object association 검증도 구분했다. 모든 실제 관측의 object evidence는 invalid 또는 recovery에서 미실행이다.

추가 세션의 candidate와 return 관측은 실제 오른팔 q가 같고, recovery에서는 held/relative extraction이 미실행이다. 이전 세션의 raw centroid 변화도 물리 Pad와 같은 물체 특징을 추적한 응답이 아니다. 사용할 REAL X pair와 독립 방향 검증은 0개다. `REAL_MEASURED` Jacobian을 생성하지 않았다.

## 현재 비어 있는 값과 최소 현장 근거

| 비어 있는 값 | 기존 자료로 결정할 수 없는 이유 | 막히는 실행 | 한 번의 최소 확보 방법 |
|---|---|---|---|
| 물리 task 목표와 `u_target` | Pad identity 및 확인된 기준 자세가 둘 다 없음 | X 오차의 목표·부호 정의 | 같은 물체를 왼팔로 보유하고, 실제 비접촉 목표 관계를 사람이 확인한 한 기준 자세에서 right RGB·actual q·실측 gripper opening을 함께 저장. 목표 특징을 물리 관계와 연결해 기록 |
| 실제 `+X_receive` 방향과 국소 영상 응답 | 기존 관찰 이동은 여러 관절/회전이 변하며 동일 reference가 없음 | 1D Jacobian 식별 | 같은 물체·기준·장착·개방량을 유지한 baseline/after 관측과 actual feedback/FK. 명령량과 실측 이동을 각각 보존 |
| 독립적인 보정 방향 검증 | 같은 fit pair의 부호는 독립 검증이 아님 | REAL J의 validated 승격 | fit에 쓰지 않은 후속 관측에서 실제 이동 방향과 영상 오차 변화 부호 확인. 필요하면 같은 baseline을 공유할 수 있음 |
| 시각/상태 결합 근거 | host receipt만 있고 exposure와 관절 측정 시각은 미확인 | 전후 비교의 신뢰성 | exposure–q 시계 매핑 또는 정지 상태에서 노출을 실제 feedback으로 감싼 기록. 기록되지 않은 시각은 null 유지 |

이것은 새 이동량·정지 dwell·오차 tolerance 요구가 아니다. 목표 기준 한 장과 X 식별 pair, fit과 독립된 방향 관측이 필요하다는 관측 계약이다. 기존 관절 방향 probe를 다시 요구하는 것이 아니며, 이전 left PICK의 응답 부호를 right RECEIVE로 옮기지 않는다.

실행 전에는 A 담당 경로가 현재 bus/feedback/fault 상태와 장치 identity, fresh actual pose, 기존 IK·경로·충돌 및 STOP 조건을 확인해야 한다. operator가 정확한 right-only 비접촉 경로·동작·중단/종료 절차를 승인해야 한다. 이 문서는 장치 접근이나 motion 승인이 아니다. 새 숫자 제한은 추가하지 않았다.

광학 holdout은 별도로 `FIELD_VALIDATION_REQUIRED`다. metric projection을 검증하려면 현재 장착에서 독립적으로 위치가 확인된 고정 board/world 기준과 서로 다른 정지 자세의 대응 기록이 필요하다. 반면 B 방식으로 물리 관계가 확인된 영상 목표를 얻으면, K/D/T를 TCP 정확도로 승격하지 않고도 첫 X pixel 정렬 계약을 구성할 수 있다. 완전한 2D/6D 검증은 선행조건이 아니다.

기존 자료로 가능한 B20 작업은 여기까지다. 위 실제 근거가 없는 상태에서 B21 개발이나 현장 측정을 시작하지 않는다.
