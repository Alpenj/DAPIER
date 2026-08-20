# JDcobot rollout adapter · 독립 safety supervisor

확인일: 2026-08-21
단계: 5/5
현재 범위: policy proposal → 독립 supervisor → ROS2-shaped dry-run envelope와 fault trace
실물 명령: 구현·허용하지 않음

## 왜 policy와 supervisor를 분리하는가

ACT가 12D action chunk를 예측한다는 사실은 joint limit, base 정지, RGB-D/feedback freshness, E-stop, watchdog, human approval을 보장하지 않는다. policy/GPU process가 멈추거나 오래된 chunk를 내더라도 transport가 닫히도록 supervisor를 별도 모듈로 둔다.

```text
ACT proposal
    ↓
SafetySupervisor (policy import 없음)
    ├─ REJECT → FAULT_LATCHED → executed_action=null
    └─ PASS   → JDcobotRos2DryRunAdapter
                         └─ JointTrajectory-shaped envelope, published=false
```

## 연구 선정과 장비 불확실성

상세 조사: [`research/LATEST_ROLLOUT_SAFETY_SUPERVISOR_RESEARCH_20260821.md`](research/LATEST_ROLLOUT_SAFETY_SUPERVISOR_RESEARCH_20260821.md)

| 자료 | 확인 내용 | DAPIER 결정 |
|---|---|---|
| ROS2 Lifecycle 공식 문서 | node lifecycle과 application fault state를 분리할 근거 | 즉시 반영: explicit state/fault latch |
| ROS2 QoS·SensorDataQoS·deadline/liveliness | QoS 호환과 DDS signal이 end-to-end freshness를 대신하지 못함 | 현장 discovery + monotonic watchdog |
| LeRobot ACT model·policy/rollout 공식 코드·tests | `reset`, action queue, episode reset, stale chunk 위험 | 즉시 반영: reset generation·proposal metadata |
| ROBOTIS TurtleBot3 공식 source | `cmd_vel`/`odom` 경계 | base stationary + recent command zero gate |
| ROBOTIS XM430 control table | base motor device-side limit 존재 | 참고만: JDcobot arm limit 근거로 사용 금지 |
| PiL-World(2026), SafeMIL(2025), ROS2 QoS 분석(2025) | closed-loop chunk/safe imitation/QoS 연구 | reference-only; runtime supervisor 정본으로 미채택 |

이 프로젝트의 정확한 JDcobot firmware, ROS2 driver, controller topic/type/QoS, joint sign/zero/limit은 공개 정본과 현장 정보로 확인되지 않았다. 따라서 코드에 실제 topic·register·limit을 넣지 않았다. fixture limit은 `synthetic_fixture_only`, topic은 `unverified_pending_education_pc_controller_discovery`다.

## lifecycle과 fault latch

| state | command behavior |
|---|---|
| `UNCONFIGURED` | 전부 reject |
| `INACTIVE` | profile 확인·재arm 전 hold |
| `ARMED` | episode별 human approval identity 보유, 아직 hold |
| `ACTIVE` | 모든 gate를 통과한 proposal만 mock dispatch |
| `FAULT_LATCHED` | 전부 reject, policy reset generation 증가 |
| `FINALIZED` | 종료 |

fault 후 `ACTIVE` 자동 복귀는 없다. `reset_fault → configure/arm/activate` 순서를 다시 밟아야 한다. E-stop/stale/base motion/limit fault 이후 이전 ACT queue generation action은 거부한다.

## fail-closed gate

- hardware profile SHA와 approved checkpoint SHA
- episode/human approval identity
- proposal sequence monotonicity와 reset generation
- observation·feedback·proposal monotonic age
- camera/target freshness
- E-stop·watchdog health
- odom base linear/angular stationary tolerance
- recent base command zero
- 12D finite shape/order
- joint lower/upper와 measured state 대비 max delta per step
- silent clip 금지: limit/rate 초과는 reject

workspace/self-collision은 실제 kinematics·joint sign·link geometry가 검증되지 않아 아직 PASS gate에 넣을 수 없다. 실물 rollout 전 필수 live gate로 남긴다.

## ROS2 dry-run adapter

`JDcobotRos2DryRunAdapter`는 supervisor가 PASS한 12D action을 좌 6축·우 6축 `trajectory_msgs/msg/JointTrajectory` 형태의 중립 envelope로 나눈다. 하지만:

- `published=false`
- `executed_action=null`
- controller topic은 `null`
- ROS2·serial import 없음
- motor bus open/write 코드 없음

즉, mapping contract만 검증하며 실기체 adapter라고 과장하지 않는다.

## 실행

```bash
python -m shoe_sorting_data.cli rollout-safety-smoke \
  --output /tmp/dapier_stage5/rollout_safety_trace.json
```

## 2026-08-21 실제 검증 결과

artifact: `C:\Users\hjjeon\Documents\DAPIER\tmp\stage5-rollout-safety-20260821-v1\rollout_safety_trace.json`

| scenario | 결과 |
|---|---|
| fresh·stationary·within synthetic fixture limit | safety PASS, `SIMULATED_ONLY`, publish 없음 |
| left_arm_0 joint/rate limit | REJECT, `FAULT_LATCHED` |
| stale observation | REJECT, `FAULT_LATCHED` |
| base linear motion | REJECT, `FAULT_LATCHED` |
| E-stop unhealthy | REJECT, `FAULT_LATCHED` |
| watchdog unhealthy | REJECT, `FAULT_LATCHED` |

summary:

- scenarios 6
- safety pass 1 / reject 5
- ROS publish 0
- hardware dispatch authorization 0
- safe proposal action은 변경·clip하지 않음
- reject proposal의 approved/executed action은 null

## 실물 연결 전 반드시 필요한 것

1. JDcobot model/firmware/driver/controller interface 정본과 교육 PC ROS graph/QoS 확인
2. 좌·우 physical label, joint name/order/sign/zero, arm/gripper unit·control mode 확인
3. 저속·무하중·한 관절씩 per-joint hard/soft limit와 max rate 승인
4. physical E-stop/power cut/watchdog 경로 및 사람이 직접 확인한 reset 절차
5. fresh feedback와 controller ack 계약
6. base odom + recent `cmd_vel` zero window 실측
7. self/inter-arm collision·workspace geometry gate
8. 서명된 episode별 operator approval와 현장 감독

첫 실물 시험은 learned policy rollout이 아니다. read-only feedback → hold → 한 관절 저속 command → 양팔 분리 test → empty-workspace scripted test를 통과한 뒤에만 ACT proposal을 연결한다. 첫 ACT 시험은 trace 단순화를 위해 `n_action_steps=1`, temporal ensemble off를 후보 baseline으로 두며, 성능 최적값이라고 주장하지 않는다.
