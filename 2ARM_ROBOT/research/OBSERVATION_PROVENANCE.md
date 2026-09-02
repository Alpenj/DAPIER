# Sim-to-real observation provenance

## 문제

MuJoCo의 `body.xpos`, free-joint `qpos`, geom ID와 segmentation ID는 실제 로봇이
직접 관측할 수 없는 privileged state다. 이런 값이 정책 또는 학습 episode에 한 번
섞이면 RGB-D pipeline이 있어도 정책은 시뮬레이터 정답에 의존할 수 있다.

따라서 “카메라를 렌더링했는가”와 “정책이 실제로 센서 관측만 소비했는가”를
별도 계약으로 검증한다.

## 출처 분류

| `source_kind` | 허용 내용 | 허용 소비자 |
|---|---|---|
| `sensor_runtime` | RGB/RGB-D, joint state, base twist, calibrated TF | policy, training episode, control monitor, evaluation |
| `simulator_privileged` | object body pose, geom/segmentation ID, reward oracle | reset, reward, evaluation oracle |

policy·training·control 경계에서는 provenance가 빠졌다고 내용을 추측하지 않는다.
명시적인 `observation_provenance`가 없으면 거부한다.

## 안전한 경로

```text
rendered/physical RGB-D + joint state
    ↓ sensor adapter
portable robot state + target estimate
    ↓ attach sensor_runtime provenance
SensorPolicyExecutor
    ↓ bounded research intent
C++ limit · interlock · watchdog · safe stop
```

MuJoCo sensor adapter는 `base_pose_map` 같은 simulator world pose를 정책 입력에서
제거하고, 좌·우 joint state·gripper·base twist와 RGB-D target만 남긴다.

## 실패 처리

- detection 실패: `target_available=false`, `target=null`, failure code 기록
- depth 결측: sensor failure 기록 또는 예외
- provenance 결측·모순: 즉시 거부
- privileged field 발견: 즉시 거부
- `control_authorized=true` 또는 `hardware_execution=true`: 즉시 거부
- sensor 실패 뒤 object truth fallback: 금지

정책은 target이 없을 때 hold·재관측·navigation 요청 중 하나를 선택할 수 있지만,
시뮬레이터 좌표를 대체값으로 받지 않는다.

## Dataset gate

sim-to-real 학습 후보 manifest는 다음을 명시해야 한다.

```json
{
  "provenance": {
    "sim_to_real_observation_compatible": true,
    "ground_truth_used_for_policy": false,
    "observation_provenance_schema_version": "dapier.observation-provenance.v1"
  }
}
```

각 sample에는 `policy_observation`이 있어야 하고 `training_episode` 검증을 통과해야
한다. 기존 `sim_episode.py` 출력에는 이 근거가 없으므로 자동으로 거부된다. 이는
기존 데이터를 삭제한다는 뜻이 아니라 simulation evaluation 자료로만 유지한다는
뜻이다.

## 재현 명령

```bash
cd ~/DAPIER
python3 -m unittest discover -s 2ARM_ROBOT/research/test -v
scripts/verify-mujoco-headless --artifact-dir /tmp/dapier-mujoco-artifacts
```

## 아직 하지 않은 것

- legacy recorder를 sensor-runtime episode producer로 전환
- ACT/LeRobot adapter에서 dataset gate 강제
- ROS 2 camera/joint message에 동일 provenance 연결
- C++ bridge와 실제 하드웨어 실행
- 실제 Astra intrinsics/extrinsics 측정
