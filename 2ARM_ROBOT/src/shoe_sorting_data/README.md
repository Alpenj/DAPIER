# Shoe Sorting Data Phase 0

JDcobot200 양팔, TurtleBot3 Waffle Pi, Orbbec Astra 계열 카메라용
DYNA-lite 데이터 기반입니다. 이 패키지는 ACT 학습 코드를 넣기 전에
episode의 관측·행동 순서와 품질 기준부터 고정합니다.

현재 구현 범위:

- 좌·우 팔, 좌·우 그리퍼, 베이스 속도, RGB/Depth timestamp 계약
- 관절 차원과 그리퍼 차원을 manifest에 명시(실기체 확인 전 변경 가능)
- 합성 golden episode 생성
- 합성 ROS 2 topic publisher와 approximate-time episode recorder
- timestamp gap, stream shape, camera health/skew/drop, joint jump, checksum 검사
- 조작 중 TurtleBot base의 명령 속도와 측정 속도 정지 interlock 검사
- 검수 대기 episode와 미확정 calibration/config를 학습 usable에서 제외
- SQLite manifest 생성과 train/validation, usable, success, shoe pair 질의
- one-shot perception exemplar의 유사도/margin 기반 match 또는 abstain
- accepted episode 기반 typed skill exemplar와 evaluation leakage audit
- lossless ROS2 RGB/Depth raw row payload와 SHA-256·timing·finalized 계약

SQLite manifest는 원본이 아닌 파생 snapshot이다. provenance column이 추가된
버전으로 갱신한 뒤에는 `shoe_episode index`를 다시 실행해 DB를 재생성한다.

현재 코드는 합성 데이터, ROS 2 topic 녹화 경로와 계약에 더해 읽기 전용 양팔 telemetry와
TurtleBot 바퀴 응답 분석을 검증합니다. 실제 팔 동작, 카메라 영상, 캘리브레이션, ACT 성공을
검증했다는 의미가 아닙니다.

## 바로 실행

패키지 폴더에서 외부 라이브러리 설치 없이 실행할 수 있습니다.

```powershell
cd C:\Users\hjjeon\Documents\DAPIER\repo\2ARM_ROBOT\src\shoe_sorting_data
python -m unittest discover -s test -v

python -m shoe_sorting_data.cli generate `
  --root .\output\golden_episodes `
  --count 20

python -m shoe_sorting_data.cli validate `
  --manifest .\output\golden_episodes\episode_000001\episode_manifest.json

python -m shoe_sorting_data.cli index `
  --root .\output\golden_episodes `
  --db .\output\episode_manifest.sqlite3

python -m shoe_sorting_data.cli query `
  --db .\output\episode_manifest.sqlite3 `
  --usable true `
  --split validation
```

## ACT/LeRobot interchange preflight

이 단계는 native LeRobot Dataset v3를 생성하지 않습니다. accepted quality gate를
통과한 원본 episode를 변경하지 않고 다음을 먼저 검증합니다.

- 좌팔 5 + 좌 gripper 1 + 우팔 5 + 우 gripper 1의 12차원 state/action 순서
- base velocity를 정책 출력에서 제외하고 stationary interlock으로 유지
- train split만 사용한 mean/std/min/max 통계
- object instance, session, recording span의 split 누수 차단
- 모든 입력·출력 파일의 SHA-256 conversion receipt
- RGB/Depth 픽셀 payload가 없을 때 native ACT 준비 완료로 표시하지 않는 blocker

```bash
ros2 run shoe_sorting_data shoe_episode act-export \
  --root output/golden_episodes \
  --output output/act_interchange_v001

ros2 run shoe_sorting_data shoe_episode act-verify \
  --root output/act_interchange_v001
```

출력 경로가 이미 존재하면 비어 있더라도 덮어쓰지 않습니다. 현재 mock camera는
metadata만 기록하므로 `act_numeric_contract_ready=true`,
`native_lerobot_ready=false`가 정상 결과입니다. 실제 RGB-D pixel writer와 native
LeRobot encoder를 구현한 뒤에만 image-conditioned ACT 학습 준비 완료로 승격합니다.

## Astra RGB-D raw payload fixture

`--camera-payload`는 작은 `rgb8`/`16UC1` 합성 frame을 ROS2 raw row 계약으로
저장합니다. 실제 Astra 검증을 대신하지 않으며, encoder와 quality gate의 입력
fixture로만 사용합니다.

```bash
ros2 run shoe_sorting_data shoe_episode generate \
  --root output/rgbd_fixture \
  --count 5 \
  --samples 4 \
  --camera-payload
```

raw frame은 `raw/workspace_rgb`와 `raw/workspace_depth` 아래에 저장되고 기존 파일을
덮어쓰지 않습니다. manifest v0.3의 `lifecycle.state=finalized`와
`integrity_verified=true`, payload byte count/SHA-256, stream header/receive timestamp,
sync delta가 모두 검증되어야 학습용 episode로 승인됩니다.

## Optional native LeRobot v3 encoder

기본 ROS2 package에는 LeRobot, Torch, Pillow, Datasets, PyArrow를 의존성으로
추가하지 않습니다. 다음 명령은 설치 없이 optional 환경 상태와 raw preflight를
확인합니다.

```bash
ros2 run shoe_sorting_data shoe_episode native-status

ros2 run shoe_sorting_data shoe_episode native-preflight \
  --root output/rgbd_fixture \
  --depth-unit mm
```

`native-export`는 optional stack이 모두 있을 때만 lazy import됩니다. 기존 raw와
output을 덮어쓰지 않고, `create→add_frame→save_episode→finalize` 결과와 source/output
hash를 `dapier_encoder_receipt.json`에 남깁니다. 실제 Astra depth unit을 확인하기
전에는 합성 fixture 외 데이터에 `--depth-unit mm`를 가정해서 사용하면 안 됩니다.

Stage 3의 native round-trip/ACT 입력 gate는 별도 ML 환경에서 실행합니다.

```bash
python -m shoe_sorting_data.cli generate \
  --root /tmp/dapier_stage3/raw \
  --count 2 --samples 3 --camera-payload \
  --camera-width 64 --camera-height 64

python -m shoe_sorting_data.cli native-export \
  --root /tmp/dapier_stage3/raw \
  --output /tmp/dapier_stage3/lerobot \
  --repo-id local/dapier-shoe-smoke \
  --depth-unit mm

python -m shoe_sorting_data.cli native-act-smoke \
  --root /tmp/dapier_stage3/lerobot \
  --repo-id local/dapier-shoe-smoke \
  --chunk-size 3
```

이 명령은 2 episode×3 frame fixture에서 official FPS delta, tail padding,
cross-episode no-leak, DataLoader shape와 ACT one-forward만 검사합니다. 실제 학습
성능을 주장하지 않으며, 1채널 depth는 native dataset에 보존하되 3채널 ResNet을
쓰는 ACT RGB baseline 입력에서는 제외합니다.

## Offline action-chunk evaluator

offline evaluator는 held-out validation/test prediction만 받고, `action_is_pad`가
False인 timestep의 horizon×joint/group error를 계산합니다. arm radian과 gripper
normalized position을 하나의 global MAE로 섞지 않습니다.

```bash
python -m shoe_sorting_data.cli offline-eval-fixture \
  --root /tmp/dapier_stage4/fixture \
  --padded-prediction 1000000

python -m shoe_sorting_data.cli offline-eval \
  --manifest /tmp/dapier_stage4/fixture/evaluation_manifest.json \
  --output /tmp/dapier_stage4/offline_evaluation_report.json
```

synthetic fixture의 결과는 padding/split/metric 계약 검증일 뿐 model 성능이
아닙니다. 실제 task success와 supervisor intervention은 Stage 5 real rollout에서
별도 측정합니다.

## JDcobot rollout safety dry-run

다음 명령은 motor나 ROS2 command topic을 열지 않습니다. policy proposal을
독립 supervisor의 lifecycle/freshness/base/joint/E-stop/watchdog gate에 통과시키고,
PASS action만 좌·우 `JointTrajectory` 형태의 dry-run envelope로 매핑합니다.

```bash
python -m shoe_sorting_data.cli rollout-safety-smoke \
  --output /tmp/dapier_stage5/rollout_safety_trace.json
```

fixture에는 실제 JDcobot topic/limit을 넣지 않았습니다. controller topic은 `null`,
limit source는 `synthetic_fixture_only`, `published=false`, `executed_action=null`입니다.
실물 연결은 joint sign/zero/limit, E-stop, watchdog, controller/QoS, human approval을
현장에서 검증한 뒤 별도 live transport로 추가해야 합니다.

Ubuntu에서는 상위 `2ARM_ROBOT` 폴더를 ROS 2 workspace로 사용합니다.

```bash
cd ~/DAPIER/2ARM_ROBOT
colcon build --symlink-install --packages-select shoe_sorting_data
set +u
source install/setup.bash
set -u
ros2 run shoe_sorting_data shoe_episode --help
```

## Mock ROS 2 recorder

다음 명령은 양팔 state/action, base 측정/명령, RGB/Depth metadata topic을 직접
발행하고 approximate-time recorder로 40 sample을 저장한다. 저장 직후 기존
quality validator 결과를 JSON으로 출력한다.

```bash
ros2 run shoe_sorting_data shoe_mock_demo \
  --output output/mock_episodes/episode_000001 \
  --samples 40
```

별도 process로 시험할 때는 `shoe_mock_publisher`와 `shoe_mock_recorder`를 각각
실행한다. recorder 기본 outcome은 사람 검수 전 상태인 `recorded`이며, 완전히
합성된 fixture를 quality gate까지 통과시킬 때만 `--accept`를 사용한다.

```bash
# terminal 1
ros2 run shoe_sorting_data shoe_mock_publisher

# terminal 2
ros2 run shoe_sorting_data shoe_mock_recorder \
  --output output/mock_episodes/episode_000002 \
  --samples 40 \
  --accept
```

출력 폴더가 비어 있지 않으면 기존 파일을 덮어쓰지 않고 실패한다. timeout이나
중단은 가능한 경우 `aborted`와 failure reason으로 기록된다.

## Perception·skill exemplar

아래 기능은 GEN-1.5 모델을 실행하는 것이 아니다. 신발 짝 후보와 검증된
skill metadata만 반환하며 `control_authorized=false`를 유지한다.

```bash
ros2 run shoe_sorting_data shoe_episode pair-add \
  --registry output/pair_registry.json \
  --exemplar-id shoe_a_left \
  --pair-id pair_a \
  --object-instance-id shoe_a_left_object \
  --embedding 1,0,0 \
  --session-id session_train_a \
  --background-id background_train

ros2 run shoe_sorting_data shoe_episode pair-match \
  --registry output/pair_registry.json \
  --embedding 0.99,0.01,0

ros2 run shoe_sorting_data shoe_episode skill-register \
  --manifest output/golden_episodes/episode_000001/episode_manifest.json \
  --output output/exemplars/grid_pick/skill_exemplar.json \
  --exemplar-id grid_pick_001 \
  --precondition base_stopped \
  --precondition camera_fresh \
  --postcondition shoe_in_target \
  --timeout-ms 15000 \
  --tag grid

ros2 run shoe_sorting_data shoe_episode skill-retrieve \
  --root output/exemplars \
  --manifest output/golden_episodes/episode_000002/episode_manifest.json \
  --skill-id pair_and_place \
  --tag grid

ros2 run shoe_sorting_data shoe_episode exemplar-audit \
  --exemplar-root output/exemplars \
  --evaluation-root output/golden_episodes/episode_000002
```

## 고의 오류 fixture

quality gate가 실제로 실패하는지 다음 fixture로 확인할 수 있습니다.

```powershell
python -m shoe_sorting_data.cli generate `
  --root .\output\bad_base_motion `
  --count 1 `
  --fault base_motion
```

지원 fault: `base_motion`, `camera_frame_gap`, `camera_skew`,
`checksum_mismatch`, `dimension_mismatch`, `duplicate_timestamp`, `joint_jump`,
`missing_camera`, `sample_gap`.

## 실물 recorder 연결 전 확정할 값

각 팔은 STS3215 모터 6개이며 현재 계약은 팔 관절 5개와 그리퍼 1개를 분리해
`arm_dof=5`, `gripper_dof=1`로 기록합니다. 다음 값을 저속 동작과 캘리브레이션으로
확인한 뒤 실제 manifest 생성기에 연결해야 합니다.

1. 좌·우 joint name, order, unit, sign
2. 그리퍼 명령·상태 차원과 단위
3. 실제 controller/software/calibration version
4. Astra Pro RGB/Depth frame timestamp와 허용 skew
5. Nav2 도킹 완료 후 base velocity가 0인지 판정하는 실제 신호

다음 구현 단계는 JDcobot ROS2 rollout adapter와 policy 밖의 독립 safety
supervisor입니다.
