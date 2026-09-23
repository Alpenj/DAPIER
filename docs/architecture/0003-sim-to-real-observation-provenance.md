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

후보에는 `goal_intent`와 손목 관측 원본이 함께 남는다.
`bounded_pregrasp_plan()`은 그 원본과 영상 해시를 다시 확인하고,
기존 `execute_bounded_pregrasp.py` 및 C++ `WRIST_ALIGN` 경로를 사용한다.
실측과 다른 시작값, 오래된/유실된 특징, 바뀐 영상은 거부한다.
위치 오차 0.5 mm, 수직축 2도, 기존 30 mm 경로 기준을 유지한다.

장치 없는 입력·어댑터 검사에서 이 연결을 확인했다. 위치만 맞고 축이 틀린
합성 목표가 거부되고, 손목 분기에서 IK가 호출되지 않는 것도 검사했다.
기존 native MOCK 전송·지연 피드백 검증은 재사용했다. 실제 손목 영상 취득,
장착 방향별 보정 효과, 실물 연속 피드백 루프와 집기 성공은 아직 검증하지 못했다.
