# Native ACT proposal → independent supervisor → ROS2 dry-run 통합 조사

확인일: 2026-08-24

범위: DAPIER-native ACT(LeRobot runtime 의존 없음), JDcobot 양팔 12-DoF, Astra Pro, ROS2, 4인·6주·RTX 5050·추가 예산 0원

## 결론

Native ACT는 **action proposal만** 만들고, 독립 safety supervisor가 승인한 action만 mock ROS2 transport에 전달한다. 첫 baseline은 `n_action_steps=1`, temporal ensemble off다. 이는 성능 주장이 아니라 reset·staleness·proposal/executed trace를 한 action 단위로 검증하기 위한 최소 안전 계약이다.

이 문서는 ROS2 dry-run 및 mock trace 설계를 뒷받침한다. **JDcobot에 command를 publish하거나, 실제 로봇이 안전하게 동작했다는 성공 주장은 하지 않는다.** 정확한 JDcobot driver, control mode, topic type/QoS, joint limit, E-stop wiring은 현장 정본 확인 전이다.

## 채택 gate 적용

[`RESEARCH_ADOPTION_LEDGER.md`](../RESEARCH_ADOPTION_LEDGER.md)의 직접 관련성·1차 근거·embodiment 차이·6주/RTX 5050/무예산 검증·측정 가설·되돌릴 수 있음 기준을 적용했다. VLA, world model, 새 dependency는 이 control-path 계약을 직접 개선하지 않으므로 넣지 않았다.

## 공식 정본에서 확인한 사실

| 정본 | 확인한 사실 | DAPIER 결정 |
|---|---|---|
| Hugging Face current [ACTConfig](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/configuration_act.py)·[ACT model](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py) | `n_action_steps`는 chunk size 이하여야 한다. queue가 비면 chunk 앞 `n_action_steps`를 채우고 하나씩 반환한다. reset은 queue/temporal state를 비운다. temporal ensemble은 매 step re-query와 `n_action_steps=1` 조건을 쓴다. | **비교 정본만** 사용한다. Native runtime이 동일한 reset-generation 의미를 직접 소유한다. LeRobot import/queue는 채택하지 않는다. |
| Zhao et al., [ACT 원 논문](https://roboticsproceedings.org/rss19/p016.pdf)·[공식 코드](https://github.com/tonyzhaozh/act/blob/main/imitate_episodes.py) | action chunking은 미래 action sequence를 query하고, temporal aggregation은 매 control step에서 겹치는 예측을 합친다. | chunk는 proposal의 출처이며 motor 권한이 아니다. initial hardware baseline은 queue/aggregation tuning이 아니라 observation마다 one action replan이다. |
| Open Robotics, [managed lifecycle](https://design.ros2.org/articles/node_lifecycle.html) | managed node에는 구성·활성·비활성·종료 전이가 있으며 inactive state는 publish/service 제공을 허용하지 않는 안전한 관리 단계가 될 수 있다. | ROS lifecycle은 node orchestration에 사용한다. `ARMED`와 `FAULT_LATCHED`는 별도 application safety state로 기록하고 auto-rearm은 금지한다. |
| Open Robotics, [QoS settings](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)·[deadline/liveliness design](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html) | QoS compatibility가 delivery를 결정한다. deadline/liveliness는 DDS entity 상태 신호이며 end-to-end semantic freshness를 보장하지 않는다. | sensor/feedback/command actual QoS는 현장 discovery 뒤 기록한다. gate freshness는 same-process monotonic receive time과 application watchdog으로 별도 판정한다. |
| Open Robotics, [JointTrajectory message](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html) | command message는 joint names와 time-indexed points를 포함한다. 메시지 자체는 limit·approval·freshness를 검증하지 않는다. | driver가 이 message를 실제로 쓰는지 확인 전까지 dry-run trace의 표현만으로 사용한다. vendor topic/type를 hard-code하지 않는다. |

## 왜 `n_action_steps=1`이 첫 baseline인가

1. 매 control tick이 새 observation, checkpoint, reset generation에 직접 묶인다. queue action의 source observation age를 해석할 추가 branch가 없다.
2. reset/fault/checkpoint 교체 뒤 남아 있던 chunk가 다시 실행될 가능성을 최소화한다. `n_action_steps>1`은 reset-generation·chunk-index 시험을 통과한 뒤의 ablation이다.
3. 6주 안에는 inference frequency를 줄이는 최적화보다 proposal→decision→executed one-to-one audit가 더 측정 가능하다. RTX 5050에서 p50/p95 inference latency가 control deadline을 넘는다면 chunk size가 아니라 model/input resolution을 먼저 측정한다.

이는 one action만 생성한다는 뜻이 아니다. model은 H-step chunk를 계속 예측하되, baseline adapter는 index 0만 proposal로 내고 다음 tick에 다시 추론한다.

## integration contract

```text
fresh observation + current feedback
  → NativeActRuntime.predict_chunk(checkpoint_sha, reset_generation)
  → proposal(index=0, action[12], source observation identity)
  → IndependentSafetySupervisor.decide(proposal, feedback, base, approval)
  → ALLOW: mock ROS2 transport dry-run trace
     REJECT/HOLD/FAULT: executed_action=null, no command publish
```

### proposal record 필수 fields

`episode_id, rollout_id, checkpoint_sha, hardware_profile_sha, calibration_id, proposal_id, chunk_id, chunk_index, policy_reset_generation, source_observation_id, source_observation_stamp, source_received_monotonic_ns, proposal_created_monotonic_ns, action_frame, proposed_action[12]`

### decision/execution trace invariants

1. `proposal_id`는 rollout 안에서 단조 증가하며, 같은 ID의 decision은 정확히 하나다.
2. `ALLOW`이면 `approved_action`만 존재할 수 있다. dry-run은 실제 전송·ack가 없으므로 항상 `executed_action=null`이다. 실제 transport ack와 feedback이 확인된 이후에만 executed action을 기록하며, transform/clamp는 hidden rewrite가 아니라 named transform·before/after·reason을 별도 남긴다.
3. `REJECT`, `HOLD`, `FAULT_LATCHED`에서는 `executed_action=null`이며 transport publish/ack가 없어야 한다.
4. `checkpoint_sha`, `hardware_profile_sha`, `policy_reset_generation`, source observation identity가 승인 당시 값과 다르면 fail-closed 한다.
5. reset, lifecycle deactivate, E-stop, stale observation/action/feedback, base motion, driver reconnect, calibration/profile/checkpoint 변경은 **policy reset + local queue clear + generation increment**을 함께 한다. 이전 generation proposal은 reason `STALE_GENERATION`으로 reject한다.
6. same-process monotonic timestamp로 observation, feedback, proposal, heartbeat age를 계산한다. ROS header stamp는 provenance/diagnostic으로 보존하되 multi-machine freshness의 유일 근거로 쓰지 않는다.

## 즉시 반영 / 실험 / 보류

| 판정 | 항목 | 이유와 검증 |
|---|---|---|
| **즉시 반영** | Native ACT proposal에 checkpoint SHA·reset generation·source observation ID를 붙여 supervisor로만 전달 | interface가 현재 stale/replay 위험에 직접 답하고 mock test 가능하다. old generation/replay/shape mutation이 execute되지 않는지 확인한다. |
| **즉시 반영** | proposal와 executed action을 분리한 immutable dry-run trace | offline evaluator, fault analysis, 취업 포트폴리오 모두에 필요한 재현성이다. reject면 `executed_action=null`, allow면 one-to-one trace를 assert한다. |
| **즉시 반영** | lifecycle + independent application fault latch + monotonic watchdog | GPU/policy path가 fault여도 command permission을 닫는 reversible guard다. lifecycle transitions와 stale heartbeat tests로 확인한다. |
| **실험 후보** | `n_action_steps>1` queue | same checkpoint/task/safety profile에서 policy query rate, action age, reject rate, closed-loop success를 함께 비교한다. stale reject 증가 또는 success 개선 부재면 미채택이다. |
| **실험 후보** | temporal ensemble | current ACT rule처럼 per-step re-query가 필요하다. latency p95, velocity jitter, intervention, success 모두 개선될 때만 채택한다. |
| **참고만** | ROS2 `JointTrajectory` | current driver type가 확정되기 전에는 generic dry-run representation일 뿐이다. |
| **보류** | vendor command publisher·actual JDcobot limits·hardware success claim | JDcobot model/firmware/control API/E-stop/limit의 현장 검증이 없으며, 잘못된 assumption은 실물 위험이다. |
| **보류** | VLA/world model/new safety framework | current proposal-supervisor boundary보다 직접적인 6주 내 measurable benefit이 없고 RTX 5050/무예산 ablation 범위를 넘는다. |

## ROS2 dry-run 합격 기준

1. ROS2가 없어도 pure-Python mock transport에서 `ALLOW`만 action record를 받는다.
2. ROS2가 있는 교육장 PC에서는 lifecycle/QoS/topic discovery만 수행하고 physical command publisher는 default disabled다.
3. 12 action 값 finite, exact joint name/order/profile hash, fresh observation·feedback·heartbeat, base stationary, active human approval, approved checkpoint가 모두 true일 때만 `ALLOW`다.
4. trace replay에서 proposal→decision→execution identity가 보존되고, previous reset generation/duplicate/out-of-order proposal은 execute되지 않는다.
5. 모든 결과는 `hardware_execution=NOT_ATTEMPTED` 또는 `mock_only=true`를 명시한다. 실물 claim은 low-speed/no-load/human-ready HIL과 vendor profile characterization이 별도로 성공한 뒤에만 가능하다.

## DAPIER 장비·일정 한계

- JDcobot300 양팔 12-DoF, Astra Pro 한 대, TurtleBot3 docking 뒤 base stationary라는 현재 embodiment만 다룬다. ALOHA의 14-DoF/multi-camera 결과를 전이하지 않는다.
- RTX 5050은 policy inference p50/p95를 측정할 대상이지 deadline 충족을 가정할 근거가 아니다. GPU exception을 safety approval과 같은 process에서 처리하지 않는다.
- 4명·6주·추가 예산 0원의 우선순위는 mock fault matrix, driver topic/QoS/profile discovery, one controlled HIL path다. distributed safety framework나 새 model 학습은 보류한다.

## 다음 구현 체크

1. Native action queue의 `proposal()` output에 reset generation/checkpoint/observation metadata를 고정한다.
2. Stage 5 `SafetySupervisor` 입력으로 변환하되, direct vendor publisher를 추가하지 않는다.
3. mock ROS2 dry-run에서 allow/reject/fault/reset/replay trace를 검증한다.
4. 교육장 PC에서 discovery script로 JDcobot topic/type/QoS와 hardware profile evidence를 수집한다.

## 학습 메모

- ACT chunk는 미래 행동 제안이고 servo 권한이 아니다.
- QoS와 lifecycle은 통신·운영 상태를 돕지만, action freshness와 human approval을 대신하지 않는다.
- 완전 독립 runtime의 핵심은 LeRobot을 제거하는 데서 끝나지 않고, native checkpoint가 낸 proposal이 실제 executed command와 절대 섞이지 않게 trace하는 데 있다.
