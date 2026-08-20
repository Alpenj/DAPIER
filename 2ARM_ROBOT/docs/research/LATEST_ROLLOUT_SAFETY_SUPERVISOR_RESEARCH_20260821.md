# 최신 JDcobot ROS2 Rollout Adapter·독립 Safety Supervisor 조사 — 2ARM_ROBOT

확인일: 2026-08-21
대상: JDcobot 양팔(정확한 ROS2 driver/API는 현장 확인 전), TurtleBot3 Waffle Pi/XM430-W210-T, Astra Pro, ACT baseline, RTX 5050, 4인·6주·추가 예산 0원

## 이 단계를 이렇게 분리하는 이유

ACT는 observation에서 action chunk를 만들고 queue에서 하나씩 꺼내지만, 이는 servo의 joint limit·base motion·camera/feedback freshness·비상정지 상태를 보장하는 장치가 아니다. 실물 장비와 vendor ROS2 API가 아직 완전히 확정되지 않았으므로 Stage 5의 합격 기준은 "팔이 움직였다"가 아니라 **정책 proposal이 independent supervisor를 통과한 경우에만 mock transport로 executed action이 되는지, 그리고 모든 불확실성에서 fail-closed 되는지**다.

## 조사 범위와 장비 불확실성

- 공개된 1차 자료에서 이 프로젝트의 정확한 JDcobot model/firmware/ROS2 driver/control mode/command topic을 검증하지 못했다. 따라서 이 문서는 vendor register·topic·joint limit 값을 발명하지 않는다.
- TurtleBot3 Waffle Pi는 ROBOTIS 공식 ROS2 source에서 `cmd_vel`/`odom` 사용을 확인했다. 그러나 DAPIER 현장 topic type·QoS·odometry quality는 launch 후 측정해야 한다.
- XM430-W210-T의 공식 control table은 Goal Position, Velocity/Current/Position Limit가 존재함을 보여 주지만, 이는 **mobile-base motor의 device-side 2차 제한**이다. JDcobot arm safety profile을 대신하지 않는다.
- hardware E-stop 회로·power cut·human deadman 배선이 확인되지 않았으므로 software supervisor를 physical emergency stop의 대체물로 주장하지 않는다.

## 공식 실행 모델에서 확인한 사실

```text
fresh RGB-D + fresh joint feedback + base stationary feedback
                     │
ACT policy.reset()/select_action() ──proposal──► independent safety supervisor
                                                   │ allow only if every gate passes
                                                   ├── reject/hold/fault + event trace
                                                   └── executed action ─► JDcobot transport adapter
                                                                                │
                                                                   measured feedback / trace
```

- LeRobot ACT의 `reset()`은 temporal ensemble 또는 action queue를 비우며, `select_action()`은 queue가 비면 chunk의 앞 `n_action_steps`만 넣고 하나씩 반환한다. episode reset 후 stale chunk가 남지 않게 하는 것은 rollout adapter의 책임이다.
- LeRobot policy contract에서 `reset()`은 evaluation의 episode 시작에, `select_action()`은 매 control step에 호출된다. 정책 action은 supervisor가 아닌 proposal이다.
- ROS2 LifecycleNode는 `unconfigured → inactive → active → finalized` 등의 managed state/transition을 제공한다. supervisor는 explicit state에 따라 command를 막는 구조로 만들 수 있다.
- ROS2 SensorData QoS는 freshness 우선의 best-effort/small queue profile이다. QoS mismatch는 통신 자체를 막을 수 있으므로 sensor/command topic마다 driver가 제공하는 QoS를 먼저 discovery해야 한다.
- ROS2 deadline/liveliness는 DDS-level signal이지만, 실제 action freshness·feedback freshness는 application-level timestamp/monotonic watchdog으로 별도 판단해야 한다.

## Stage 5 최소 아키텍처 계약

| component | 소유 책임 | 절대 하면 안 되는 일 |
|---|---|---|
| `policy_bridge` | RGB-D/joint observation snapshot을 policy에 주고 proposal chunk의 action·source timestamp·checkpoint ID를 생성 | motor command topic을 직접 publish |
| `rollout_adapter` | episode/reset, policy queue, proposal sequence ID, observation/action staleness 전달 | limit를 임의 clamp한 뒤 성공처럼 publish |
| `safety_supervisor` | profile hash, freshness, base interlock, joint/rate/workspace/human/E-stop gate; allow/reject/hold/fault 결정 | policy import, policy inference, auto-rearm, sensor 결손을 정상으로 간주 |
| `hardware_transport` | supervisor가 승인한 12-DoF command만 vendor API/topic으로 변환하고 measured feedback 회수 | policy proposal 또는 unapproved command 수신 |
| `mock_transport` | hardware profile을 모사하고 proposal/executed trace를 남김 | default test에서 physical hardware를 열기 |
| `base_guard` | base odom/cmd feedback에서 stationary evidence를 supervisor에 제공 | arm policy가 `cmd_vel`을 제어하도록 허용 |

policy process가 죽거나 GPU inference가 멈춘 경우 supervisor/transport는 마지막 action을 계속 재발행하지 않고 hold/reject한다. supervisor process가 죽거나 health heartbeat가 stale이면 transport도 fail-closed 한다.

## 즉시 반영 — 실물 없이 unit/integration test 가능한 항목

### 1. explicit lifecycle + human arm gate

supervisor lifecycle을 아래처럼 고정한다.

| state | command behavior | active 전제 |
|---|---|---|
| `UNCONFIGURED` | 모든 actuation reject | no hardware profile loaded |
| `INACTIVE` | hold only; proposal 기록 가능 | profile/calibration/transport health 확인 전 또는 after reset |
| `ARMED` (application state, ROS Lifecycle `inactive` 위) | hold only; human approval token 대기 | physical E-stop released 확인을 **operator checklist**로 입력, base stationary evidence, fresh feedback |
| `ACTIVE` | passing proposal만 execute | per-episode human approval + all gates valid |
| `FAULT_LATCHED` | hold/reject only | E-stop, stale, limit, base motion, heartbeat, driver fault 중 하나 발생 |
| `FINALIZED` | transport close | explicit shutdown |

`FAULT_LATCHED`에서 `ACTIVE`로 자동 전이는 없다. operator가 원인을 확인하고 reset/cleanup/configure/approve 순서를 다시 밟아야 한다. ROS2 lifecycle의 unconfigured/inactive/active/finalized transition은 node 관리 수단이고, `ARMED`/fault latch 같은 system safety semantics은 application state로 trace한다.

### 2. fail-closed approval gates

action proposal은 아래 모두가 true일 때만 one step 실행된다. 하나라도 unknown/false면 **reject + hold**다.

| gate | verification input | fail-closed reason/event |
|---|---|---|
| profile identity | expected `hardware_profile_sha`, joint names/order, unit/control mode | `PROFILE_MISMATCH` |
| policy/episode identity | approved checkpoint SHA, episode ID, monotonically increasing proposal/chunk ID | `UNAPPROVED_POLICY` / `REPLAY_OR_OUT_OF_ORDER` |
| observation freshness | RGB-D header/received monotonic age, joint feedback age, source timestamp | `STALE_OBSERVATION` / `STALE_FEEDBACK` |
| action freshness | proposal created monotonic time, source observation ID, queue/action index | `STALE_ACTION` / `STALE_CHUNK` |
| base stationary | fresh `odom` twist below configurable linear/angular epsilon for K consecutive feedbacks **and** observed/nav `cmd_vel` zero window | `BASE_MOVING` / `BASE_STATE_UNKNOWN` |
| arm geometry | finite 12 values, named joint min/max, allowable rate from measured state and elapsed time, optional workspace check | `NONFINITE_ACTION` / `JOINT_LIMIT` / `RATE_LIMIT` / `WORKSPACE_LIMIT` |
| stop/health | E-stop input/health state, supervisor heartbeat, driver feedback/fault | `ESTOP_OR_HEALTH_FAULT` |
| human approval | current episode approval token, operator present/timeout not expired | `HUMAN_APPROVAL_MISSING` |

threshold values (`max_observation_age`, `max_action_age`, rate, base epsilon/K, joint limits)에는 plausible default를 넣지 않는다. hardware profile/safety YAML의 versioned fields로 두고, mock test는 fixture 값만 쓴다. 현장 characterization 뒤 human review로 설정한다.

### 3. ACT queue/reset/stale chunk rules

ACT `n_action_steps>1`은 policy가 한 번 추론한 chunk에서 앞 steps를 queue로 실행한다. rollout adapter는 queue action마다 아래 metadata를 부착한다.

`episode_id, rollout_id, checkpoint_sha, proposal_id, chunk_id, chunk_index, source_observation_id, source_observation_stamp, proposal_created_monotonic_ns, action_value, action_frame, policy_reset_generation`

다음 사건에서는 `policy.reset()`과 adapter local queue clear를 모두 수행하고, 이전 `policy_reset_generation` action을 무조건 reject한다.

1. episode start/end, human pause/takeover, policy checkpoint 교체
2. lifecycle deactivate/activate, fault latch, E-stop, base moving
3. observation/action/feedback stale, driver reconnect, calibration/profile change
4. rejected action 이후 config에서 정한 replan required event

`n_action_steps`를 늘려 inference call 수를 줄이는 실험은 Stage 4 offline gate·Stage 5 supervisor logs가 준비된 뒤에만 한다. first hardware rollout은 `n_action_steps=1`, temporal ensemble off를 conservative baseline으로 사용한다. 이는 performance tuning이 아닌 stale/chunk trace 단순화를 위한 선택이며, 나중에 제거 가능하다.

### 4. ROS2 QoS·timestamp·watchdog contract

| channel | initial rule | 이유 |
|---|---|---|
| RGB-D / joint feedback / odom | actual driver QoS discovery 후 compatible profile; sensor stream은 `SensorDataQoS`(best effort, small keep-last)가 후보 | 최신 sample이 retry된 오래된 sample보다 중요할 수 있고 QoS incompatibility는 delivery를 막는다. |
| approved arm command | driver-required QoS를 explicit config로 기록; reliable 여부를 추측하지 않음 | command topic type/QoS가 JDcobot driver에 따라 달라 아직 확인 전이다. |
| safety state/fault/approval | volatile state + durable audit log; late subscriber에 필요한 state는 explicit service/query로 재확인 | transient message 하나를 E-stop truth로 취급하지 않기 위해서다. |
| supervisor heartbeat | application-level monotonic deadline, missed count/last timestamp trace | DDS liveliness만으로 end-to-end control freshness를 판단할 수 없다. |

각 gate는 ROS header stamp와 process-local `received_monotonic_ns`를 모두 trace한다. 서로 다른 computer/clock에서 header stamp를 단순 비교하지 않는다. freshness watchdog의 primary clock은 same-process monotonic receive clock이고, sensor source header와 recorded timestamp skew는 diagnostics로 따로 남긴다.

### 5. proposal-versus-executed audit trace

한 policy action마다 immutable event를 남긴다. 최소 fields:

```text
event_id, monotonic_ns, episode_id, rollout_id, lifecycle_state,
policy_checkpoint_sha, proposal_id, chunk_id, chunk_index,
source_observation_id/stamp/age, feedback_id/age, base_stationary_evidence,
proposed_action[12], supervisor_decision, rejection_codes[],
executed_action[12]|null, transport_ack, measured_state_after|null,
profile_sha, calibration_id, human_approval_id, estop_state
```

`proposed_action`과 `executed_action`을 같은 field로 overwrite하지 않는다. supervisor가 reject/hold했을 때 executed action은 `null` 또는 explicitly named safe hold command여야 하며, clamp/transform이 있으면 transform name·before/after·reason을 모두 기록한다. 이는 offline evaluator의 `safety_or_intervention` taxonomy와 그대로 연결한다.

### 6. mock-first verification matrix

physical driver 없이 아래가 모두 deterministic test로 재현되어야 한다.

| test | expected supervisor decision |
|---|---|
| fresh approved 12-DoF proposal, stationary base, within limit | `ALLOW`, mock `executed_action == proposal` |
| one joint NaN/Inf, wrong shape/name/order/profile SHA | `REJECT`, no mock execution |
| position/rate/workspace boundary violation | `REJECT`, reason preserved |
| stale RGB-D, stale feedback, stale action/chunk, missed heartbeat | `FAULT_LATCHED` or `REJECT` per policy, no execution |
| `odom` moving / unknown, recent non-zero `cmd_vel` | `REJECT BASE_MOVING/UNKNOWN` |
| E-stop/human approval absent/expired | `FAULT_LATCHED`/`REJECT`, reset cannot auto-arm |
| episode reset/fault/reconnect after queued ACT action | old queue generation rejected; only a new observation/new proposal may execute |
| policy process failure/supervisor failure | transport hold; no last command replay |
| trace replay | proposal/decision/executed/action IDs remain one-to-one and monotonic |

mock transport is the default. real JDcobot transport must require an explicit hardware flag, explicit device identifier/profile match, and active human approval; CI and demo scripts must not set these by default.

## 실험 후보 — gate를 통과한 후에만

| 후보 | hypothesis | measuring gate |
|---|---|---|
| `n_action_steps` 1 vs >1 | longer action queue may reduce inference load but increase staleness/reject risk | same ACT checkpoint/task/safety profile; report policy query rate, action age, supervisor reject, success separately |
| temporal ensemble | per-step requery/ensemble may reduce jitter | required n_action_steps=1; compare measured joint velocity jitter, success, intervention; no benefit claim from offline loss alone |
| soft clamp vs reject | bounded slew may provide smoother recovery | default stays reject. adopt only after mock+teleop trials show transparent trace and no hidden limit violation |
| odom + wheel feedback stationary consensus | one feedback source may be noisy | calibration/characterization shows disagreement; otherwise simplest odom+cmd zero gate stays |
| ROS2 lifecycle node implementation | managed activation reduces startup ambiguity | test all transitions and failure callbacks; application fault latch remains independent |

## 참고만 — 최소 최신 관련 연구

| 자료 | 직접 관련성·embodiment 차이 | 이번 채택 위치 |
|---|---|---|
| PiL-World (2026) | action chunks의 policy-in-the-loop closed loop 평가를 다루지만 VLA/world-model infrastructure다. | chunk freshness가 offline metric과 real outcome 사이에 있다는 해석 참고만. RTX 5050/6주에 world model 도입은 보류. |
| SafeMIL (2025) | non-preferred trajectory로 safe imitation policy를 학습하는 연구지만, DAPIER의 runtime supervisor/hardware driver contract를 제공하지 않는다. | future data labeling 참고만. immediate safety gate 근거로 사용하지 않는다. |
| ROS2 QoS dependency-chain analysis (2025) | QoS dependency/verification을 다루나 platform/implementation 조건이 다르다. | QoS를 explicit config·test로 다룬다는 참고만. 바로 도입할 formal method는 아니다. |

임의 VLA/world-model 논문은 Stage 5의 fail-closed transport/supervisor 구현을 직접 개선하지 않으므로 포함하지 않았다.

## 보류

| 항목 | 이유 |
|---|---|
| supervisor가 policy process 안에서 실행 | GPU exception/queue bug가 safety gate까지 같이 중단될 수 있어 독립성이 사라진다. |
| software E-stop만으로 physical E-stop 대체 | electrical/driver fault에서 ROS process가 동작하지 않을 수 있다. 실제 E-stop wiring 확인 전 safety claim 금지. |
| vendor API/topic/register hard-code | JDcobot의 정확한 model/firmware/driver/control mode 정본이 현장 미확인이다. |
| base moving 중 양팔 autonomous manipulation | mobile inertia/pose drift/카메라 geometry가 action contract를 깨므로 docking 후 stationary interlock이 먼저다. |
| policy proposal을 silent clamp하여 execute | model error와 safety modification을 구분할 수 없고 offline/rollout 분석이 거짓이 된다. |
| stale action의 마지막 command 반복 | communication/model failure가 지속 motion으로 바뀔 수 있다. |
| multi-machine header clock 차이를 freshness의 유일 근거로 사용 | clock skew가 false allow/reject를 만든다. local monotonic watchdog을 병행한다. |

## 구현 순서와 학습 요약

1. **hardware-independent action/profile interface와 mock transport**를 만든다. 왜: JDcobot vendor API가 확정되지 않아도 12-DoF safety logic을 검증할 수 있기 때문이다.
2. **independent fail-closed supervisor와 lifecycle/fault latch**를 만든다. 왜: policy가 잘못되거나 멈춰도 motor command 경로가 독립적으로 닫혀야 한다.
3. **freshness/base-stationary/joint-rate/human approval gate**를 넣는다. 왜: RGB-D/teleop/SLAM/actuator의 서로 다른 지연과 base motion을 proposal과 분리해 차단하기 위해서다.
4. **ACT reset/queue generation + proposal/executed trace**를 연결한다. 왜: chunk action이 어느 observation·episode에서 왔는지, 무엇이 실제 실행됐는지 재현하기 위해서다.
5. **mock fault matrix를 통과한 뒤 vendor adapter characterization**을 한다. 왜: 실물에서는 transport mapping/limit/feedback만 새로 검증하고 safety semantics를 다시 발명하지 않기 위해서다.

## 근거 원문 및 확인일

모든 링크는 2026-08-21에 확인했다. 공식 ROS2/LeRobot/ROBOTIS 자료와 직접 관련 최신 연구만 사용했다.

1. ROS 2, [Lifecycle managed nodes 공식 문서](https://docs.ros.org/en/rolling/p/lifecycle/) 및 [lifecycle event API](https://docs.ros.org/en/ros2_packages/rolling/api/launch_ros/launch_ros.events.lifecycle.html) — state machine/transition.
2. ROS 2, [QoS 공식 문서](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html), [SensorDataQoS API](https://docs.ros.org/en/ros2_packages/jazzy/api/rclcpp/generated/classrclcpp_1_1SensorDataQoS.html), [deadline/liveliness design](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html) — compatible QoS, sensor freshness profile, DDS-level deadline/liveliness.
3. Hugging Face, [ACT model 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py), [policy protocol 공식 문서](https://github.com/huggingface/lerobot/blob/main/docs/source/bring_your_own_policies.mdx), [rollout strategy core](https://github.com/huggingface/lerobot/blob/main/src/lerobot/rollout/strategies/core.py), [policy tests](https://github.com/huggingface/lerobot/blob/main/tests/policies/test_policies.py) — reset/select_action/action queue, rollout reset/test conventions.
4. ROBOTIS, [TurtleBot3 ROS2 source](https://github.com/ROBOTIS-GIT/turtlebot3), [official absolute move example](https://github.com/ROBOTIS-GIT/turtlebot3/blob/main/turtlebot3_example/turtlebot3_example/turtlebot3_absolute_move/turtlebot3_absolute_move.py), [official teleop](https://github.com/ROBOTIS-GIT/turtlebot3/blob/main/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py) — `cmd_vel`/`odom`, Waffle velocity limits/zero-stop behavior in official source.
5. ROBOTIS, [XM430-W210-T control table](https://emanual.robotis.com/docs/kr/dxl/x/xm430-w210/) — hardware position/velocity/current limits; DAPIER arm limit의 근거가 아님.
6. Dong et al., 2026, [PiL-World](https://arxiv.org/abs/2606.05773), Lin et al., 2025, [SafeMIL](https://arxiv.org/abs/2511.08136), and 2025 [ROS2 QoS dependency-chain analysis](https://arxiv.org/abs/2509.03381) — 모두 reference-only; adoption 조건 미충족.

## 학습 메모

- **강의에서 확인**: ROS2 node가 publish할 수 있다는 것과 physical robot에 command를 허용해도 된다는 것은 다른 문제다.
- **외부 보강**: ACT action queue는 episode reset과 observation freshness가 명확하지 않으면 오래된 action을 재사용할 수 있다. ROS2 QoS는 delivery 조건이지 end-to-end safety decision이 아니다.
- **학습자 해석**: DAPIER의 시스템 역량은 "VLA가 양팔을 움직였다"보다, proposal·approval·execution·fault가 trace로 분리되고 hardware 없이도 모든 reject path가 재현된다는 데 있다.
- **다음 검증**: JDcobot vendor driver가 준비되면 device/model/feedback/control mode/limit을 hardware profile에 측정값으로 입력하고, same mock matrix를 hardware-in-the-loop(저속·무하중·human ready)로 한 항목씩 실행한다.
