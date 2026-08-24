# Stage 7 · Native ACT proposal와 독립 safety supervisor 통합

## 결과

DAPIER-native ACT가 만든 action chunk를 기존 Stage 5 `SafetySupervisor`와
`JDcobotRos2DryRunAdapter`에 연결했다.

```text
native checkpoint + raw RGB-D observation
  → ACT action chunk [H,12]
  → action[0] proposal (n_action_steps=1)
  → checkpoint/profile/reset generation/source observation identity
  → ACTIVE SafetySupervisor
     ├─ REJECT → FAULT_LATCHED, approved/executed action=null
     └─ PASS   → JointTrajectory-shaped dry-run envelope
                  published=false, executed_action=null
```

이 통합은 ROS2나 motor SDK를 import하지 않으며 실물 command 권한을 만들지 않는다.

## 왜 이 단계가 필요한가

Stage 6의 `infer`는 action chunk를 계산하지만, checkpoint 승인·base 정지·sensor/feedback
freshness·E-stop·watchdog·human approval·joint limit을 판정하지 않는다. 반대로 Stage 5
supervisor는 실제 ACT checkpoint가 만든 proposal을 받은 적이 없었다. 두 경계를 연결해야
“모델 출력”과 “안전 승인”, “실제 실행”을 한 trace에서 구분할 수 있다.

## 구현 순서와 이유

1. **checkpoint 파일 자체를 SHA-256으로 식별**한다. 경로 이름이 같아도 byte가 바뀐 모델은
   approved checkpoint로 인정하면 안 된다.
2. **chunk의 index 0만 proposal로 만든다.** 첫 baseline은 매 observation마다 다시 추론해
   queue action의 source age와 reset 원인을 단순하게 추적한다.
3. **source observation과 reset generation을 proposal에 기록**한다. episode/frame identity와
   이전 fault generation이 다른 action을 fail-closed할 수 있다.
4. **기존 supervisor를 그대로 호출**한다. limit/freshness/approval 검사를 connector에 복사하지
   않아 safety 정본이 두 군데로 갈라지지 않는다.
5. **PASS도 dry-run adapter까지만 전달**한다. `approved_action`은 존재할 수 있지만 publish/ack가
   없으므로 `executed_action`은 계속 `null`이다.

## proposal·trace 계약

- proposal: sequence, query/chunk/action index, episode/human approval, checkpoint/profile SHA,
  reset generation, source observation ID/frame/monotonic receive time, 12D action
- baseline: `action_index=0`, `n_action_steps=1`
- policy inference: `control_authorized=false`
- dry-run: `mock_only=true`, `hardware_execution=NOT_ATTEMPTED`
- PASS: action을 clip하거나 변경하지 않고 generic JointTrajectory-shaped envelope만 생성
- REJECT: supervisor가 `FAULT_LATCHED`로 전환하고 adapter는 dispatch하지 않음
- actual checkpoint SHA mismatch: `unapproved_policy`로 REJECT
- ROS2·serial·Dynamixel·LeRobot runtime import 없음

## 실행

교육 PC에서 build/setup 후 다음 명령을 실행한다.

```bash
cd ~/DAPIER/2ARM_ROBOT
colcon build --symlink-install --packages-select shoe_sorting_data
set +u
source install/setup.bash
set -u

ros2 run shoe_sorting_data shoe_dapier_act rollout-smoke \
  --output /tmp/dapier_native_act_rollout
```

smoke는 synthetic RGB-D 2 episode로 one-step ACT checkpoint를 만든 뒤, 실제 checkpoint
hash를 승인 config에 넣고 proposal→supervisor→dry-run adapter를 실행한다.

## 2026-08-24 검증 결과

- native rollout focused tests: 4/4 PASS
- base data/ROS package suite: 85 tests OK, optional ML 5 SKIP
- isolated CPU ML suite: 85 tests OK, environment-conditional 2 SKIP
- safe proposal: `decision=PASS`
- proposal action: inference chunk의 `action[0]`과 exact equality
- state/action: 12D
- checkpoint mismatch: `unapproved_policy`, `FAULT_LATCHED`
- source observation: `episode_000001:0`
- `control_authorized=false`
- `hardware_dispatch_authorized=false`
- `published=false`
- `executed_action=null`
- `hardware_execution=NOT_ATTEMPTED`

이 결과는 synthetic dry-run 계약 검증이며 JDcobot 동작 성공이나 신발 정리 성공 증거가 아니다.

## 연구 근거와 보완 판단

직접 관련된 ACT·ROS2 공식 정본만 확인했다. 상세 자료와 채택 판정은
[`research/LATEST_NATIVE_ACT_SUPERVISOR_INTEGRATION_RESEARCH_20260824.md`](research/LATEST_NATIVE_ACT_SUPERVISOR_INTEGRATION_RESEARCH_20260824.md)에 기록했다.

- **즉시 반영:** checkpoint/reset/source identity, one-to-one proposal/decision trace,
  monotonic freshness, `n_action_steps=1`, dry-run executed null
- **실험 후보:** `n_action_steps>1`, temporal ensemble. 같은 checkpoint/task/safety profile에서
  latency·action age·reject·success를 함께 측정할 때만 판단
- **참고만:** generic `JointTrajectory` envelope. 실제 JDcobot driver type은 현장 discovery 필요
- **보류:** vendor publisher, 실제 joint limit 가정, VLA/world model/new safety framework

## 다음 단계

교육 PC에서 JDcobot controller topic/type/QoS와 joint feedback을 read-only로 discovery하고
hardware profile evidence를 만든다. 그 전까지 현재 connector는 mock-only 상태를 유지한다.
