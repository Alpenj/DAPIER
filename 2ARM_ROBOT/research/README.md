# DAPIER Python research layer

이 디렉터리는 인식, 정책 추론, 태스크 계획, 시뮬레이션, 데이터셋 생성과
오프라인 평가를 위한 **순수 Python 연구 계층**이다.

이 계층은 다음을 하지 않는다.

- serial 또는 motor SDK로 장치를 연다.
- `/dev/tty*` 또는 `/dev/serial/*`에 접근한다.
- torque, EEPROM/register 또는 actuator command를 직접 쓴다.
- 실제 로봇 실행을 승인한다.
- C++ 제어기의 watchdog, joint limit 또는 safe stop을 우회한다.

Python은 [`ControlIntent`](src/dapier_research/control_intent.py)를 만들 수 있지만,
그 intent는 제안일 뿐이다. `so101_ros2`의 C++ 계층이 receiver-local timestamp,
sequence, TTL, limit, interlock과 실행 허가를 다시 검사한다.

공유 제어 계약은 저장소 루트의
[`contracts/research_realtime_control_v1.json`](../../contracts/research_realtime_control_v1.json)이
단일 기준이다. 서로 다른 컴퓨터의 monotonic clock은 비교하지 않는다. Python이
남긴 `source_monotonic_ns`는 trace용이고, C++ 수신기가 자신의 monotonic clock으로
`ttl_ns`를 시작한다.

## RGB-D target geometry

[`vision_target.py`](src/dapier_research/vision_target.py)는 detector mask와 aligned
metric depth를 pinhole camera model로 역투영하고, calibrated 4x4 transform을 적용해
robot/planning frame의 visible-surface target과 불확실성을 계산한다.

이 경로의 원칙은 다음과 같다.

- runtime target은 RGB/RGB-D 센서 결과에서 계산한다.
- MuJoCo object pose와 segmentation ID는 runtime planner 입력으로 사용하지 않는다.
- simulator truth는 reset, reward, offline label과 test-only 오차 측정에만 사용한다.
- 깊이 결측, low confidence와 privileged detection은 fail-closed 처리한다.
- RGB-only 영상에서 검증되지 않은 절대 깊이를 만들어내지 않는다.

MuJoCo adapter와 실행 예시는
[`VISION_GUIDED_REACH.md`](../sim/mobile_dual_so101/VISION_GUIDED_REACH.md)를 본다.
설계 근거는
[`ADR 0002`](../../docs/architecture/0002-vision-first-sim-to-real-perception.md)에 있다.

## Observation provenance gate

[`observation_contract.py`](src/dapier_research/observation_contract.py)는 policy와
학습 episode가 어떤 관측을 소비했는지 명시적으로 구분한다.

- `sensor_runtime`: RGB/RGB-D, joint state, base twist처럼 실제 장비에도 존재하는 관측
- `simulator_privileged`: MuJoCo body pose, geom/segmentation ID처럼 시뮬레이터만 아는 값
- `policy_runtime`, `training_episode`, `control_monitor`는 `sensor_runtime`만 허용
- `evaluation_oracle`, `reset_reward`만 privileged observation을 허용
- provenance가 없거나 truth flag와 모순되면 추론하지 않고 거부
- sensor detection 실패를 MuJoCo object pose로 자동 대체하지 않음

[`sim_to_real_policy.py`](src/dapier_research/sim_to_real_policy.py)의
`SensorPolicyExecutor`는 기존 chunk executor 앞에서 provenance를 검사한다. 기존
`sim_policy.py`와 `sim_episode.py`는 simulation baseline으로 남아 있으며, 그 출력은
새 dataset gate를 통과하기 전에는 sim-to-real 학습 입력으로 사용할 수 없다.

계약과 학습용 예시는
[`OBSERVATION_PROVENANCE.md`](OBSERVATION_PROVENANCE.md)와
[`ADR 0003`](../../docs/architecture/0003-sim-to-real-observation-provenance.md)을 본다.

## 장비 없이 테스트

```bash
cd ~/DAPIER
python3 -m unittest discover -s 2ARM_ROBOT/research/test -v
scripts/verify-architecture-boundaries
```

MuJoCo 의존 테스트는 다음 검증에 포함된다.

```bash
scripts/verify-mujoco-headless --artifact-dir /tmp/dapier-mujoco-artifacts
```

기존 `2ARM_ROBOT/src/shoe_sorting_data`에는 실험 초기에 만든 ROS·하드웨어 인접
Python 도구가 섞여 있다. 이 디렉터리를 연구 계층으로 간주하지 않으며, 신규
정책·학습·평가 코드는 여기 `research/`에 추가한다. 기존 하드웨어 쓰기 경로는
C++ 계층으로 단계적으로 이동한다.
