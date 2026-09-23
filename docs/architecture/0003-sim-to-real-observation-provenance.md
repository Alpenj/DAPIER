# ADR 0003: Sim-to-real observation provenance is mandatory

- Status: Accepted for staged implementation
- Date: 2026-09-02
- Scope: DAPIER mobile dual-arm research policy and dataset boundaries

## Context

PR #42 introduced an RGB-D path that reconstructs a shoe target without using MuJoCo
object poses. The existing state task and episode recorder still expose simulator truth
for evaluation and baseline experiments. A policy or dataset adapter could therefore
accidentally consume privileged fields even when RGB-D frames are also present.

Presence of camera data is not proof that policy input is sensor-derived. The producer,
source class, truth usage, and consumer must be checked explicitly.

## Decision

Every observation crossing a sim-to-real policy, training, or control-monitor boundary
must carry `dapier.observation-provenance.v1`.

Two source classes are defined:

1. `sensor_runtime`: only information available through physical sensors or calibrated
   transforms.
2. `simulator_privileged`: object/body pose, simulator IDs, reward or reset oracle.

`policy_runtime`, `training_episode`, and `control_monitor` accept only
`sensor_runtime`. `evaluation_oracle` and `reset_reward` may consume privileged state.
Missing provenance is rejected rather than inferred.

The existing simulation task and recorder remain available as legacy evaluation
baselines. They are not declared sim-to-real compatible. A separate dataset gate rejects
an episode unless both the manifest and every policy observation prove sensor-runtime
provenance.

## Consequences

### Positive

- RGB-D rendering can no longer be confused with RGB-D policy conditioning.
- Detection failure remains visible and cannot silently fall back to object truth.
- Legacy simulation results remain reproducible without being mislabeled as deployment
  evidence.
- ACT/LeRobot conversion has a concrete gate to enforce before training.

### Costs

- Producers must attach provenance and list sensor frames.
- Existing episode files fail the new sim-to-real gate until migrated.
- Policy integration needs a strict wrapper even when the underlying simulation executor
  remains unchanged.

## Verification

- pure-Python provenance and dataset-gate unit tests
- MuJoCo adapter tests for world-pose sanitization
- policy-query count remains zero when privileged observations are rejected
- sensor detection failure remains a sensor observation with no truth fallback
- `hardware_execution=false` remains mandatory

## Follow-up

1. Add a sensor-runtime episode producer.
2. Require the dataset gate in ACT/LeRobot conversion.
3. Connect the accepted policy observation to the C++ mock safety bridge.
4. Add ROS 2 camera/joint provenance and clock-domain validation.

## 2026-09-23: 손목 관측에서 제한 실행 후보로 연결

record_id: DAPIER-2026-09-23-wrist-observation-ingress

손목 보정 함수가 검사 도구에서만 호출되는 것을 확인하고,
`evaluate_single_shot_ik.py --wrist-observation-json FILE`을 기존 후보 생성기에 연결했다.
이 모드는 새 IK를 풀지 않고 실측 시작 상태에 영상 오차 보정을 적용한 뒤,
같은 FK·접근축·경로 검사로 후보를 평가한다. 반대팔과 그리퍼는 유지한다.

입력은 `dapier.wrist-observation.v1` JSON이다. `side="left"`,
`clock="host_monotonic_ns"`, `timestamp_ns`, `measured_state_sha256`,
`measured_q_model_rad`(왼팔 6축), `frame_source`(`path`, `sha256`),
`feature_center_uv`, `target_uv`, `confidence`를 명시한다.
UV는 [-1, 1] 정규화 좌표이며, 영상 취득과 관절 상태의 실제 시간 대응은 생산자가
확인해야 한다. 해시 일치만으로 노출 시점 동기화나 카메라 장착 방향을 검증했다고
판정하지 않는다. 현재 연결은 특징 관측 파일을 소비하며 카메라를 직접 열지 않는다.

특징 좌표 대신 기존 `PixelDetection` 결과를 사용할 때는 `detection`에
`label`, `detector`, `confidence`, `uses_privileged_labels`,
`mask_source`(`path`, `sha256`)를 넣는다. 마스크는 원본 RGB와 같은 크기의
boolean 또는 0/1 정수 NPY다. 원본 영상은 H×W×3 uint8 NPY 또는 이미지 파일이다.
이 경로는 마스크 중심을 [-1, 1] 영상 좌표로 변환하며 depth를 요구하지 않는다.
마스크 중심은 영상 특징이고 물리적인 파지 중심으로 자동 해석하지 않는다.
빈 마스크·크기 불일치·시뮬레이터 정답 라벨은 거부한다. 마스크와 명시적
`feature_center_uv`/`confidence` 입력을 동시에 제공할 수 없다.

후보에는 `goal_intent`와 손목 관측 원본이 함께 남는다.
`bounded_pregrasp_plan()`은 그 원본과 영상·마스크 해시를 다시 확인하고,
기존 `execute_bounded_pregrasp.py` 및 C++ `WRIST_ALIGN` 경로를 사용한다.
실측과 다른 시작값, 오래된/유실된 특징, 바뀐 영상은 거부한다.
위치 오차 0.5 mm, 수직축 2도, 기존 30 mm 경로 기준을 유지한다.

장치 없는 입력·어댑터 검사에서 이 연결을 확인했다. 위치만 맞고 축이 틀린
합성 목표가 거부되고, 손목 분기에서 IK가 호출되지 않는 것도 검사했다.
기존 native MOCK 전송·지연 피드백 검증은 재사용했다. 실제 손목 영상 취득,
장착 방향별 보정 효과, 실물 연속 피드백 루프와 집기 성공은 아직 검증하지 못했다.

### Metric 목표의 근거와 depth 취득 방식 분리

실행 계획 어댑터가 `depth_evidence`만 읽어 기하 기반 목표의 검증 결과를
전달할 수 없는 부분을 확인했다. 관측 JSON의 `metric_evidence`를 우선 읽고,
그 필드가 없을 때만 기존 `depth_evidence`를 사용하도록 수정했다.
`metric_target_verified`가 실제 boolean `true`일 때만 기존 native 검증값으로
전달한다. 기하 추정에는 방법·전제·보정 revision·불확실성을 원본 관측에 남긴다.
해당 관측 파일은 기존 source SHA 확인 대상이다.

`P2_PASS`는 관측 단계 판정이므로 이 실행 검증값으로 자동 변환하지 않는다.
현재 근거가 거부이면 예전 depth 승인으로 대체하지 않고, 잘못된 근거 형식은
거부한다. synthetic 입력으로 이 분기와 기존 depth 호환성을 검사했다:
`python -m unittest discover -s 2ARM_ROBOT/research/test -p test_real_sensor_ik_adapter.py`
(8개 통과). 실제 장치 접근 없이 검사했으며, 새 관측·실측 시작 상태·경로 검증과
실행 승인은 별도로 필요하다. 다음 작업은 관측 좌표와 충돌 장면의 책상 기준을
맞추는 것이다. direct depth 자체를 추가 촬영의 필수 사유로 삼지 않는다.

### 관측한 지지면을 경로 검사 장면에 연결

물체 중심만 관측 위치로 옮기고 책상은 nominal SIM 높이에 남아 있던 문제를
확인했다. 실제 입력에는 `scene_support`의 `frame="left_motor1_datum"`,
`normal_xyz=[0,0,1]`, `top_z_m`, `revision`을 기록한다. 기존 수평 책상과
world Z=0인 팔 베이스 조건에서 CAD datum offset으로 높이를 변환한다.
조건이 다른 장착에는 이 변환을 암묵적으로 적용하지 않는다.

`observed_task_env()`는 해당 높이로 책상 형상을 **컴파일하기 전에** 배치하고,
`bind_observed_block()`은 별도로 관측 물체 중심·방향을 배치한다. 상면 목표나
접근 TCP를 물체 중심으로 대체하지 않는다. 실제 입력에 지지면이 빠지면 IK 전에
거부하고, 기존 명시적 SIM 진단은 원래 장면을 유지한다.

실측 지지면을 사용하는 경로에는 nominal SIM의 structural near-support 예외를
적용하지 않는다. 그 페어는 삭제하지 않고 일반 clearance 검사에 남긴다.
native 피드백의 endpoint FK도 같은 원본 관측 SHA와 지지면으로 장면을 재구성한
뒤 compiled model SHA를 대조한다. 기본 SIM 모델 해시와 기존 예외 회귀는 유지했다.

장치 없는 검사에서 물체 바닥과 책상 높이의 일치, 변경된 관측의 endpoint 거부,
기본 모델 해시 보존, 기존 충돌 검사와 실측 장면의 일반 검사 페어 유지를 확인했다.
이 결과는 좌표·충돌 장면 연결 검사이며 현재 실물 시작 상태의 IK/경로 PASS나
물리적 관절 매핑 검증을 의미하지 않는다. 다음 실제 입력은 승인된 최신 관측과
양팔 readback이며, 보존 시각을 바꿔 fresh 입력으로 만들지 않는다.
