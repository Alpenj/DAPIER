# SO-101 sim-first G0–G1 학습 기록

- `record_id`: `DAPIER-2026-08-07-so101-g0`, `DAPIER-2026-08-07-so101-g1-scripted-pick`
- 실행일: 2026-08-07
- 범위: G0 환경 smoke와 G1 scripted pick-and-lift 1 episode
- 구현 commit: G0 `00b211a6fc8f965a83337786582320e34629d4f1`, G1 `da5b84ed7959483ddaf1c8ce557806254ad86e02`

## 이번에 확인하려는 것

오늘 수업에서 내가 궁극적으로 만들고 싶은 양팔 카지노 딜러까지 바로
넘어가기 전에, 단일 SO-101 시뮬레이션의 가장 아래 계약부터 확인하고 있다.
이번 G0에서는 기존 MuJoCo 모델을 읽기 전용으로 불러와 여섯 관절의 이름과
순서, degree/radian 변환, gripper `0..100` 변환, 보정 파일 identity, 오래된
frame 거부 규칙을 검사한다.

G0를 통과한 뒤에는 scripted Virtual Leader, MuJoCo 접촉 기반 lift, front
image와 measured/action episode 기록까지 G1으로 확인했다. 사람이 조작한
demonstration, 정책 학습·평가, ROS 2 adapter, 시리얼 연결과 실물 제어는
아직 범위가 아니다. `dapier_sim_first.gate` CLI는 `init-g0`, `g0`,
`init-g1`, `g1`까지만 제공한다.

## 기존 작업과의 관계

작업을 시작하며 같은 이름처럼 보이는 폴더를 다시 확인했다.

| 위치 | 직접 확인한 역할 | 이번 G0에서 한 일 |
|---|---|---|
| `$HOME/so101` | LeRobot 0.6.0 checkout, 별도 venv, 커밋하지 않은 `so101_mujoco`, 진단 dataset | 코드를 수정하지 않고 MuJoCo venv와 기존 모델만 읽기 전용 입력으로 사용 |
| `$HOME/so101_ros2_ws` | `build/install/log`가 있는 colcon workspace | 빌드하거나 launch하지 않음 |
| `DAPIER/so101/ros2_ws` | DAPIER가 소유하는 ROS 2 core와 안전 teleop의 정식 소스 | 수정하지 않음 |
| `DAPIER-lerobot-ros2-lab` | 2026-08-06 연구 계획용 별도 Git worktree | 계획과 현재 환경 경계만 읽고 수정하지 않음 |

예전 `$HOME/so101_ros2_ws/src/dapier-so101-ros2` symlink 대신 현재는
`DAPIER/so101/ros2_ws` 자체를 colcon workspace로 사용한다.

## 환경 매트릭스 대조

설치나 다운로드 전에 아래 상태를 읽기 전용으로 확인했다.

| 항목 | 직접 확인한 값 | 계약 문서와의 대조 |
|---|---|---|
| OS | Ubuntu 24.04.4 LTS, amd64 | `Ubuntu 24.04` 후보 행과 일치 |
| ROS 2 | Jazzy, `/opt/ros/jazzy`, package 428개 | 후보 행과 일치. nexus/LeRobot/MuJoCo 전체 결합은 미검증 |
| system Python | 3.12.3, `/usr/bin/python3` | 후보 행 및 nexus의 Python 3.12+ 조건과 일치 |
| uv | 0.12.0 | 설치됨 |
| GPU | NVIDIA RTX 5050 Laptop GPU 8151 MiB, driver 595.84와 AMD Phoenix3 | 장치·driver만 확인. G0에서 render는 실행하지 않음 |
| system Python package | NumPy 1.26.4, pytest 7.4.4, torch 2.13.0 | MuJoCo, LeRobot, Gymnasium, so101-nexus는 설치되지 않음 |
| 기존 LeRobot venv | Python 3.12.3, MuJoCo 3.8.1, LeRobot 0.6.0, Gymnasium 1.3.0, torch 2.11.0+cu128 | 모델 smoke에는 사용 가능. 계약의 pinned nexus 0.5.1 조합으로 보지 않음 |

따라서 이 PC는 문서의 `Ubuntu 24.04 + ROS2 Jazzy + Python 3.12 후보` 축에는
맞는다. 하지만 `so101-nexus` 0.5.1이 설치된 것이 아니므로 전체 조합의
호환성을 확인했다고 쓰지 않는다. 별도 venv의 LeRobot 소스도 수정된 dirty
상태라 이번 G0 구현과 섞지 않았다.

## G0 구현

이번에 추가한 공개 경계는 다음뿐이다.

- `protocols.py`: `connect`, `disconnect`, `get_action` 구조와 exact frame schema
- `embodiment.py`: SO-101 여섯 채널과 body degree/radian, gripper 선형 변환
- `environment.py`: 개인정보와 hardware probe를 제외한 읽기 전용 환경 수집
- `gate.py`: 새 run manifest 생성, G0 검증, 재사용 불가능한 receipt 기록

frame은 embodiment/revision/calibration/channel order/unit가 모두 exact match해야
한다. `sequence_id`는 strictly increasing, monotonic timestamp는 nondecreasing이어야
하며 30 Hz의 두 control period보다 오래된 `age > 2T` frame은 거부한다.
범위를 넘는 action도 조용히 clip하지 않고 거부한다.

## 실제 검증

먼저 시스템 Python과 기존 LeRobot venv에서 같은 단위 테스트를 실행했다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s dapier_sim_first/test -v
PYTHONDONTWRITEBYTECODE=1 \
  "$HOME/so101/lerobot/.venv/bin/python" \
  -m unittest discover -s dapier_sim_first/test -v
```

두 interpreter에서 각각 `8/8`이 통과했다. 처음 실행에서는 endpoint의
degree/radian 부동소수점 오차가 약 `1e-16`만큼 범위를 벗어나는 실패가
발생했다. 실제 범위 초과 거부는 유지하고, `1e-12` 이하의 수학 round-off만
경계값으로 정규화한 뒤 다시 통과했다.

이번 실행은 저장소 밖의 새 경로만 사용했다.

```text
$HOME/dapier-runs/so101-foundation/20260806T233431Z-g0/
├── run-manifest.json
└── G0/
    ├── environment.json
    ├── contract.json
    └── receipt.json
```

직접 확인한 receipt는 `PASS`이며 결과는 다음과 같다.

| G0 metric | 결과 |
|---|---:|
| pinned revision manifest exact match | 5/5 |
| MuJoCo model load | 1/1 |
| joint/actuator channel과 order | 6/6 |
| unit conversion round-trip | 6/6 |
| calibration identity | 1/1 |
| valid frame acceptance | 2/2 |
| invalid frame deterministic rejection | 13/13 |
| schema/rejection-rule violation | 0 |

`pick_cube.xml`은 MuJoCo 3.8.1에서 로드됐고 named joint와 actuator 순서는 모두
`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`,
`gripper`였다. manifest hash는
`sha256:5f156f65375824c20cf3f59f9abcf49faa6379f3291666b07e1da4acc708a0ea`다.
manifest와 세 결과 JSON은 모두 mode `0444`로 생성됐다.

같은 명령을 같은 run에 다시 실행해 보니 exit code `2`와 함께
`RUN_ROOT contains an existing artifact or receipt; refusing reuse`로 중단됐다.
기존 PASS receipt를 새 실행 증거로 재사용하지 않는 조건도 확인했다.

revision `5/5`는 작업 계약에 적힌 다섯 SHA가 manifest와 exact match한다는
뜻이다. 다섯 upstream checkout이 이 PC에 모두 있다는 뜻은 아니다. 실제 G0
구현 commit은 manifest와 현재 repository가
`00b211a6fc8f965a83337786582320e34629d4f1`로 같은지도 별도로 검사했다.

## G0 당시 물건 집기 공백

G0와 물건 집기 성공은 다르다. 기존
`$HOME/so101/sim_dataset/so101_mujoco_joint_sweep`을 읽어 보니 LeRobot v3
형태로 5 episodes, 450 frames가 있고 6축 state/action과 front image가
기록돼 있다. 하지만 `next.success`는 `0/450`, episode 성공은 `0/5`였다.
이 dataset은 관절과 기록 파이프라인을 확인한 deterministic joint sweep이며
집기 demonstration이 아니다.

G1을 시작하며 원본 장면의 뒤쪽 큐브를 두 가지 joint-space trajectory로
접근해 보니 양쪽 jaw 접촉은 만들었지만 settled 높이 대비 약 7 mm 오른 뒤
빠졌다. 큐브를 팔 앞쪽의 reachable 위치로 옮겨도 원본 mesh collision만으로는
lift가 약 9 mm 안에서 끊겼다. 이 실패를 성공으로 표시하지 않고, task
geometry 문제와 controller 문제를 분리했다.

## G1에서 바꾼 task 조건

오늘 수업에서 SO-101의 작은 gripper가 50 mm 큐브를 평면 접촉으로 물 수
있도록 다음 조건을 `G1_TASK_CONFIG`와 manifest digest에 고정했다.

- 원본 50 mm, 50 g 큐브의 크기와 질량은 바꾸지 않았다.
- 큐브를 팔 앞쪽 작업공간의 `x=0.254531 m`, `y=-0.002931 m`에 두었다.
- 기존 green tray를 목표물이 아니라 floor top `z=0.044 m`인 지지대로 썼다.
- fixed/moving jaw에 half-size `20 × 20 × 2 mm`인 평면 finger pad를 하나씩
  추가했다. full thickness는 4 mm이고 sliding friction은 `2.0`이다.
- frame 0 이후 cube pose를 직접 바꾸거나 weld/equality로 붙이지 않았다.
  position actuator, 접촉, 마찰, 중력만으로 5000 MuJoCo substeps를 진행했다.
- evaluator는 controller와 분리해 settled 높이, 양쪽 pad 접촉, 지지대 접촉을
  frame trace에서 다시 계산한다.

따라서 이번 결과는 외부 checkout의 default `PickCube-v0` 성공이 아니라
`DAPIER-SO101-PaddedPickLift-v0` task 성공이다. 이 pad 조건이 실물에서도
같은 마찰을 낸다는 주장이나 sim-to-real 증거로 사용하지 않는다.

## G1 실제 검증

구현을 `da5b84ed7959483ddaf1c8ce557806254ad86e02`로 먼저 commit한 뒤,
저장소 밖의 새 run에서 seed 101, 30 Hz, 300 frames를 직접 실행했다.

```bash
export RUN_ROOT="$HOME/dapier-runs/so101-foundation/20260807T004658Z-g1"
MUJOCO_GL=egl HF_HUB_OFFLINE=1 \
  "$HOME/so101/lerobot/.venv/bin/python" -m dapier_sim_first.gate init-g1 \
  --run-root "$RUN_ROOT" --repo "$PWD" \
  --model "$HOME/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/pick_cube.xml" \
  --calibration "$HOME/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml" \
  --lerobot-root "$HOME/so101/lerobot"
MUJOCO_GL=egl HF_HUB_OFFLINE=1 \
  "$HOME/so101/lerobot/.venv/bin/python" -m dapier_sim_first.gate g1 \
  --manifest "$RUN_ROOT/run-manifest.json" --seed 101 --rate-hz 30 \
  --frames 300 --out "$RUN_ROOT/G1"
```

```text
$HOME/dapier-runs/so101-foundation/20260807T004658Z-g1/
├── run-manifest.json
└── G1/
    ├── task-model.xml
    ├── frame-trace.json
    ├── task-evaluation.json
    ├── provenance.json
    ├── preview.mp4
    ├── lerobot-v3/
    └── receipt.json
```

| G1 metric | 결과 |
|---|---:|
| episode | 1/1 |
| accepted frames | 300/300 |
| measured/action pairs | 300/300 |
| front image rows | 300/300 |
| LeRobot codebase schema | v3.0 |
| Parquet measured round-trip | 300/300 |
| Parquet action round-trip | 300/300 |
| schema/order/sequence/timestamp/stale/limit violation | 모두 0 |
| maximum lift from settled | 47.15 mm |
| minimum lift during final hold | 42.12 mm |
| bilateral pad contact during final hold | 30/30 frames |
| support contact during final hold | 0/30 frames |
| simulated time | 10.0 s / 5000 substeps |
| provenance | `source=scripted`, `human_demo=false` |

`preview.mp4`도 160×120, 30 Hz, 300 frames, 10.0초로 다시 열어 확인했다.
Parquet timestamp의 30 Hz grid 최대 오차는 약 `4.45e-7 s`였고 마지막
frame의 `next.success=true`, `next.done=true`도 확인했다. 같은 run을 다시
실행하면 기존 artifact 때문에 exit code `2`로 거부된다.

시스템 Python과 기존 LeRobot venv에서 전체 단위 테스트 `13/13`, Ruff
검사와 format check도 통과했다. LeRobot venv는 여전히 수정 중인 외부
checkout이며 G1은 그 소스를 고쳤다고 주장하지 않는다. public
`LeRobotDataset` writer와 exact file digest만 사용했다.

## 다음에 확인할 것

아직 확인하지 못한 부분은 pad 없는 기본 task의 안정적인 force-closure,
여러 초기 위치와 seed에서의 성공률, 사람이 조작한 G2 demonstration, 정책
학습·평가, 한 팔 카드 조작과 CardBench 양팔 확장이다. 이번 작업에서는 G2
이상, ROS 2 build/launch, serial probe, 물리 hardware movement를 진행하지
않았다.

## Robot Control Stack 개념 채택 — 2026-08-11

[Robot Control Stack 검토](../project-planning/2026-08-11-robot-control-stack-concept-adoption.md)를
현재 DAPIER 계약과 대조했다. RCS source와 asset은 AGPL-3.0 및 개별 asset
license 경계 때문에 복사하지 않았고, 전체 runtime도 의존성으로 추가하지 않았다.
대신 다음 개념을 DAPIER가 독립 구현했다.

- G1 manifest는 `synchronous`, `post_action_readback`, `absolute_target`,
  `async_control=false`를 input digest에 포함한다.
- [`digital_twin.py`](digital_twin.py)는 이미 동기화된 command/simulation/physical
  joint trace를 offline으로 비교한다.
- joint별 MAE, RMSE, p95, endpoint error, command delay gap과 timestamp skew를
  JSON 가능한 `dapier.digital-twin.v1` 결과로 만든다.
- 실측 threshold가 없으면 결과는 `MEASURED`이며 PASS 또는 sim-to-real 성공을
  주장하지 않는다.
- evaluator는 ROS 2, MuJoCo, LeRobot, serial 또는 hardware를 import하거나
  command하지 않는다.

새 contract test는 알려진 sim 1-step·physical 2-step 지연, threshold
PASS/FAIL, joint-order mismatch, timestamp skew, NaN과 nonmonotonic timestamp를
검사한다.

```bash
python -m unittest dapier_sim_first.test.test_digital_twin -v
python -m unittest discover -s dapier_sim_first/test -v
```

다음 physical Gate에서는 observation sync가 만든 trace만 이 평가기에 넣는다.
실제 팔의 threshold는 synthetic test 숫자를 복사하지 않고 반복 측정 후 별도
승인 manifest로 고정한다.
