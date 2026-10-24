# 신발 정리 양팔로봇 A→B→C 전체 과정 회의안

## 제안 결론

A/B/C는 셋 중 하나를 고르는 대안이 아니라 **A→B→C를 순서대로 모두 경험하는
3단계 완주 로드맵**으로 사용한다. 사용자가 기획, 기구, 출력, simulation, perception,
데이터, 학습, IK, 제어, ROS, 실물 검증과 문서화 전 과정에 직접 발을 담그는 것이
프로젝트의 명시적 성공조건이다.

각 단계는 기능만 늘리는 방식이 아니다. 작은 범위라도 다음 lifecycle을 한 번씩 끝낸다.

```text
사용 시나리오 → 요구사항/지표 → CAD/출력 → digital twin → perception/data
→ IK·policy·control → safety gate → 통합 시험 → 실패 분석 → 문서/PR/demo
```

현재 중앙 지지대, Waffle 좌표, depth camera와 jitter 작업은 이 전체 제품 흐름을 지원하는
하위 트랙이다. 회의의 중심 질문은 구조물 하나가 아니라 다음 문장이다.

> 바닥의 신발을 인식하고 안전하게 집어, 양팔로 자세를 정돈한 뒤 지정 위치에
> 반복 가능하게 놓고, 실패하면 중단하거나 복구하는 로봇을 어떻게 단계적으로 만들 것인가?

## 공통 제품 범위

### 사용자 시나리오

1. 사용자가 신발을 작업영역에 놓는다.
2. 로봇이 depth/RGB로 신발과 좌우·짝·위치를 추정한다.
3. 접근 가능성과 충돌 없는 grasp 후보를 계산한다.
4. 한 팔 또는 양팔로 신발을 집고 자세를 정렬한다.
5. 지정된 정리 zone, 매트 또는 rack slot에 신발을 놓는다.
6. 성공 여부를 다시 관측하고 실패 시 retry, regrasp 또는 safe stop한다.
7. 이동이 필요한 단계에서는 팔을 transport pose로 접은 뒤 base가 이동한다.

### 공통 안전 원칙

- A/B의 manipulation 동안 mobile base는 기본적으로 고정한다.
- base 이동과 팔 작업은 동시에 시작하지 않는다.
- simulation/MOCK, 제한된 bench HW, 통합 HW 결과를 분리해 기록한다.
- 실물은 사람 승인, E-stop, 정확한 device profile, joint/velocity/effort limit,
  watchdog과 fail-safe stop을 갖춘 단계에서만 실행한다.
- 신발 성공률보다 사람·장치 안전 gate가 항상 우선한다.

## A — 신발 한 짝 end-to-end thin slice

목표는 **알려진 위치의 신발 한 짝을 집어 가까운 목표 zone으로 옮기는 전체 과정을
처음부터 끝까지 한 번 완주**하는 것이다.

| 분야 | A에서 직접 해볼 것 |
| --- | --- |
| 제품/UX | 시작 위치, 목표 zone, 성공/실패 정의와 demo script 작성 |
| 기구 | Waffle adapter, 중앙 STEP 지지대, shoulder/camera fit coupon |
| 출력 | K1 Max + Hyper PLA로 coupon 및 축소/분할 prototype 출력 |
| digital twin | 공식 Waffle mesh, SO-101, 신발 free body와 camera frame 검증 |
| perception | 고정된 신발 또는 MOCK pose로 3D pose contract 작성 |
| 조작 | 한 팔 DLS IK, 7차(septic) trajectory, grasp/transport/place state machine |
| 학습 | 소량 teleop/demo 데이터 형식과 replay를 경험하고 baseline과 비교 |
| 제어 | target/actual velocity·acceleration·jerk, contact와 torque 기록 |
| 통합 | simulation-only one-shoe scenario 반복 실행 |
| 문서 | 실패 영상/수치, BOM, 좌표, 테스트와 PR 기록 |

A 완료 gate:

1. 공식 좌표계에서 camera→base→arm→shoe transform이 닫힌다.
2. simulation에서 20회 중 16회 이상 pick-and-place 성공 또는 실패 원인이 분류된다.
3. actual jerk, torque saturation, forbidden contact와 지지다각형 gate를 보고한다.
4. Waffle/arm/camera coupon이 실제 치수와 맞고 dummy mass proof test를 통과한다.
5. 실물 motor 명령 없이도 전체 pipeline의 입력/출력과 로그를 재현할 수 있다.

A는 단순 목업이 아니라 모든 분야의 첫 번째 작은 완주다.

## B — 양팔 한 켤레 정리 MVP

목표는 **흐트러진 신발 한 켤레를 depth camera로 찾고, 양팔을 사용해 방향과 간격을
정렬하여 두 개의 지정 slot에 놓는 통합 MVP**다.

| 분야 | B에서 확장할 것 |
| --- | --- |
| 제품/UX | 한 켤레 정렬 기준, 좌/우 slot, 허용 오차와 사람 개입 규칙 |
| 기구 | 모듈형 중앙 지지대, 교체형 shoulder plate와 camera cradle, bolt/금속 보강 |
| 출력 | 파라메트릭 CAD/STEP/STL, fit coupon, BOM, revision 고정 G-code |
| perception | RGB-D segmentation, 6D/평면 pose, 좌우/짝 분류와 confidence |
| 데이터 | 실제 실패·성공이 포함된 teleop dataset, train/validation split |
| 학습 | imitation policy와 temporal ensemble, classical baseline A/B 비교 |
| 조작 | bimanual reachability, handoff/regrasp, 양팔 충돌과 동기화 |
| 제어 | command rate, backlash, latency, filtering과 jitter 계측/완화 |
| ROS | perception→planner/policy→controller 메시지 계약과 SIM domain 통합 |
| 검증 | shoe 종류·자세를 바꾼 반복 시험과 failure recovery |

B 완료 gate의 초기 목표값은 회의에서 확정한다.

- 신발 한 짝 grasp 성공률
- 한 켤레 최종 정리 성공률
- position/yaw/좌우 간격 오차
- cycle time과 사람 개입 횟수
- perception confidence와 pose error
- actual jerk·torque·contact·support margin
- regrasp/retry 성공률과 safe-stop 정확도

권장 운용은 base 정지 상태의 정리 작업이다. mobile base 이동은 C 전에 독립적으로
검증하며, B의 성공률을 높이기 위해 base와 arm을 동시에 움직이지 않는다.

## C — 이동·다품종·실패복구 통합

목표는 **여러 위치와 여러 켤레를 탐색하고, 이동 후 정리하며, 예상 밖 상황을 감지해
복구하거나 사람에게 넘기는 고급 연구 데모**다.

| 분야 | C에서 확장할 것 |
| --- | --- |
| task | 여러 켤레, 겹침, 뒤집힘, rack/현관 등 다양한 배치 |
| mobility | 탐색·접근·정렬, transport pose, base-arm 상호 배타 interlock |
| perception | tracking, occlusion 처리, 재관측과 scene memory |
| manipulation | long-horizon planning, regrasp, handoff와 recovery policy |
| 학습 | 다양한 신발 domain, active data collection, sim-to-real 비교 |
| 구조 | B 실측 결과에 따라 알루미늄 spine/판금 hybrid frame 검토 |
| 안전 | 이동 중 정지거리, 사람/장애물, 전원·E-stop·watchdog 통합 |
| 평가 | unseen shoes, clutter, 조명, 바닥 마찰과 장시간 반복 시험 |

C 완료는 한 번의 화려한 demo가 아니라 scenario matrix에서 success, intervention,
recovery와 safety 결과가 반복 가능하게 남는 것이다.

## A/B/C별 전 과정 경험 체크

| 과정 | A | B | C |
| --- | --- | --- | --- |
| 문제/사용자 시나리오 정의 | 한 짝 | 한 켤레 | 여러 켤레/장소 |
| CAD·3D 출력 | coupon/prototype | modular revision | hybrid 개선 |
| MuJoCo physics | 단일 pick/place | bimanual pair | mobile long-horizon |
| depth perception | MOCK/기본 pose | 실제 RGB-D pipeline | tracking/occlusion |
| 데이터 수집 | 소량 형식 검증 | 학습 dataset | active/domain 확대 |
| policy/IK | 각각 경험 | 비교·결합 | recovery/long-horizon |
| 제어/jitter | 계측 baseline | 완화 A/B test | 장시간 안정성 |
| ROS 통합 | SIM contract | 전체 SIM graph | mobility 포함 |
| 실물 검증 | coupon/dummy | 제한 bench→통합 | 반복 운용 |
| QA/문서/PR | 1회 완주 | release gate | scenario report |

## 전체 회의 의제

회의는 다음 순서로 진행하면 지지대 세부 설계에만 갇히지 않는다.

1. **제품 목표:** 정리 장소가 floor zone인지 rack인지, 신발 한 짝/한 켤레의 정의
2. **성공 지표:** 성공률, 정렬 오차, 시간, 개입, recovery와 안전 지표
3. **A demo:** 가장 작은 end-to-end 입력·출력과 20-trial test 정의
4. **B MVP:** 양팔이 반드시 필요한 동작과 한 팔로 처리할 동작 구분
5. **인지/데이터:** camera model, annotation, dataset 규모와 policy baseline
6. **기구/제작:** 중앙 지지대 높이·폭, shoulder/camera interface, K1 Max 출력과 보강
7. **제어/안전:** jitter, limits, watchdog, base-arm interlock, HW 승인 절차
8. **LLM 감독기:** 허용 skill, schema, 재계획 횟수, 사람 개입과 안전 veto 경계
9. **C 확장:** 이동과 여러 켤레를 넣을 시점 및 진입 gate
10. **역할/일정:** 각 산출물 담당, review 담당, 실측 날짜와 demo 날짜

회의에서 반드시 정할 결정:

- A의 신발/목표 zone/20회 시험 조건
- B의 최종 정리 형상과 양팔 사용 이유
- camera 정확 모델 확인 담당과 기한
- SO-101/Waffle 실측 담당과 기한
- upper socket 홀 중심 높이 394.051 mm·간격 254 mm, arm frame Z=387.686 mm와
  camera mast Z=550 mm의 전체 통과 폭 허용 범위
- A prototype에 금속 보강을 바로 넣을지, 변형 측정 후 넣을지
- 각 단계의 HW gate 승인자와 중단 기준

## 회의와 무관하게 계속 진행하는 병행 트랙

| 트랙 | 현재 상태 | 다음 산출물/게이트 |
| --- | --- | --- |
| Waffle 좌표·형상 | 공식 URDF/STL/STEP 좌표와 6-hole 후보 반영, 전체 49개 테스트 통과 | 실물 하부 nut 접근 확인 |
| support/camera | 상·하부 원본 STL과 10 mm overlap, 전용 mast camera Z=550 mm·27° | 실제 hole/optical datum 및 mast 출력 검증 |
| SO-101 interface | upper socket은 `base_so101_v2`만 대체하고 나머지 base 조립품 삽입 | 조립 순서·bolt 방향 및 fit coupon |
| physics/IK | physics-executed IK 구현, raw actual jerk gate는 FAIL | backlash/update latency 식별값 반영 |
| jitter | temporal ensemble 포함 후보 정리 중 | baseline/filter/ensemble 평가표와 test harness |
| shoe task | free shoe와 기본 task/test 존재 | A의 한 짝 state machine·20-trial metric |
| fabrication | K1 Max, Hyper PLA Black 1.75 mm/1 kg 확인 | nozzle·slicer/version 후 STL/G-code revision |
| LLM supervisor | provider-neutral typed skill schema와 fail-closed validator | deterministic task executor 연결 후 replay 평가 |
| safety | simulation-only, hardware 실행 없음 | dummy-load→정적 proof→승인된 제한 HW |

이 작업들은 회의 결정을 기다리며 멈추지 않는다. 다만 camera/arm interface의 최종 CAD와
printer-specific G-code는 정확한 실측과 현재 nozzle 확인 전에는 release하지 않는다.
