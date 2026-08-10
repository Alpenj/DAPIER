# SO-101 카지노 딜러 실험 런북

이 문서는 제가 실제 장비를 연결할 때 같은 실수를 반복하지 않도록 적어둔
작업 순서입니다. 전원이 꺼진 상태에서 시작해 **캘리브레이션 → 저속
텔레옵 → episode 검수 → ACT 기준선 → 한 팔 카드 조작 → 양팔 확장** 순서로
진행합니다.

체크박스는 미리 채우지 않습니다. 실제로 명령을 실행하고 화면·로그·로봇
움직임을 확인한 뒤에만 체크합니다.

아래 명령은 소스·실행 환경·원시 증거를 섞지 않도록 세 경로를 구분한다.

```bash
export DAPIER_ROOT="${DAPIER_ROOT:-$HOME/DAPIER}"
export LEROBOT_ROOT="${LEROBOT_ROOT:-$HOME/so101/lerobot}"
export SO101_EVIDENCE_ROOT="${SO101_EVIDENCE_ROOT:-$HOME/dapier-runs/so101-hardware}"
```

`DAPIER_ROOT`에는 공개 가능한 코드와 문서만, `LEROBOT_ROOT`에는 upstream
checkout과 venv만, `SO101_EVIDENCE_ROOT`에는 calibration·JSONL·영상 같은
로컬 실행 증거만 둔다.

## 이 문서에서 AI가 도운 범위

카지노 딜러라는 아이디어, episode를 쌓아 policy를 만들자는 목표, 한 팔에서
양팔로 확장하는 순서는 제가 정했습니다. AI는 그 방향을 실행 가능한
명령어와 문서로 정리하고, ROS 2 로그와 오류 메시지를 읽고, 반복되는
검증 작업을 코드로 옮기는 데 도움을 줬습니다. 배선·전원·모터 움직임·
캘리브레이션 결과·episode 성공 여부는 제가 직접 확인합니다. 따라서 이
문서는 자동으로 완주된 결과 보고서가 아니라, 다음 실험에서 사람이 직접
이어갈 체크리스트입니다.

현재 목표는 실제 카지노 운영이 아니라, 고정된 테이블에서 안전하게
블랙잭 초기 딜을 시연하는 것이다. 현금·칩·사람 손을 작업 영역에 넣지
않는다.

## 0. 내가 먼저 구분할 역할

| 계층 | 담당 | 지금 사용할 것 |
|---|---|---|
| 게임 규칙 | 딜 순서, 플레이어 수, hole card | `casino_dealer` planner |
| 조작 policy | 카드 집기·놓기·뒤집기 | LeRobot ACT/behavior cloning |
| 저수준 제어 | 관절 feedback·gripper·속도 제한 | SO-101 + ROS 2/LeRobot |
| 데이터 | 영상·측정값·명령·성공 여부 | LeRobot 또는 rosbag2 MCAP |

게임 전체를 처음부터 policy 하나로 학습시키지 않는다. 먼저 `pick_card`와
`place_card`처럼 짧고 판정 가능한 skill을 학습시킨다.

## 1. 오늘: 하드웨어 없이 미리 끝내는 작업

### 1-1. 저장소와 Python 테스트

```bash
cd ~/DAPIER/casino_dealer
python3 -m unittest discover -s test -v
python3 -m casino_dealer.cli --players 3 --compact
python3 -m casino_dealer.episode_cli --help
```

`casino_dealer`를 설치하지 않은 상태에서도 위 명령을 실행할 수 있다.
실패하면 Python 경로가 패키지 디렉터리를 보고 있는지 확인한다.

### 1-2. 비구동 one-card 기구학 기준선

실제 장비 전에 왼팔 deck 안정화와 오른팔 one-card pick/place 순서를 bounded
Cartesian action으로 고정한다.

~~~bash
cd ~/DAPIER/casino_dealer
python3 -m casino_dealer.card_sim_cli \
  --episodes 100 \
  --seed 1000 \
  --output /tmp/casino_one_card_kinematic_receipt.json
~~~

2026-08-10 실행 결과는 `100/100`, 평균 `32.92` step, 최대 action delta
`0.02 m`였다. 이 baseline은 3D point, vacuum attachment, table clearance만
계산한다. dynamics/contact engine, 실제 카드 인식, vacuum, 카메라, serial과
모터를 사용하지 않았으므로 실제 양팔 성공으로 체크하지 않는다.

### 1-3. 첫 episode manifest 만들기

실제 데이터가 없어도 manifest 형식과 사람 검수 흐름을 미리 시험할 수 있다.

```bash
EP_ROOT="$HOME/.ros/so101_episodes/casino_one_card"
mkdir -p "$EP_ROOT/episode_000001"

cd ~/DAPIER/casino_dealer
python3 -m casino_dealer.episode_cli init \
  --path "$EP_ROOT/episode_000001/episode_manifest.json" \
  --task "Pick one card and place it on player_1." \
  --skill pick_and_place_card \
  --source lerobot \
  --fps 30 \
  --camera front \
  --calibration-ref so101_follower_main.json

python3 -m casino_dealer.episode_cli validate \
  --path "$EP_ROOT/episode_000001/episode_manifest.json"
```

episode를 촬영한 뒤에는 영상을 보고 다음처럼 판정한다.

```bash
cd ~/DAPIER/casino_dealer
python3 -m casino_dealer.episode_cli mark \
  --path "$EP_ROOT/episode_000001/episode_manifest.json" \
  --status accepted \
  --success true
```

카드 낙하·가림·통신 오류·사람 손 개입이 있으면 학습 데이터로 쓰지 않는다.

## 2. 다음 연결일: 전원 연결과 기본 확인

### 2-1. 전원 끄기 상태에서 조립 확인

- USB와 모터 전원을 모두 분리한다.
- Bus Servo Adapter 점퍼는 PC 제어용 `B`로 둔다.
- `D / V / G` 방향과 데이지체인 순서를 확인한다.
- 모터 라벨 전압과 전원공급장치 전압을 대조한다.
- 테이블 클램프와 비상 전원 차단 수단을 준비한다.
- 리더·팔로워를 동시에 연결하지 말고 한 팔씩 확인한다.

전원이 꺼진 상태에서만 케이블을 꽂고 뺀다.

### 2-2. PC 사전점검

```bash
bash "$DAPIER_ROOT/so101/hardware_tools/read_only/preflight.sh"
groups
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
```

`dialout`과 `video`가 현재 로그인 세션에 있어야 한다. 포트가 보이지
않으면 전원을 반복해서 넣지 말고 USB 케이블·어댑터·전원 LED부터 확인한다.

### 2-3. 포트 확정

```bash
cd "$LEROBOT_ROOT"
uv run lerobot-find-port
```

한 팔의 포트를 찾은 뒤 `FOLLOWER_PORT`, `LEADER_PORT`로 기록한다. 실제
경로는 컴퓨터마다 다르므로 문서에 임의의 `/dev/ttyACM0`를 영구값으로
적지 않는다. 가능하면 `/dev/serial/by-id/...`를 사용한다.

## 3. 모터 ID와 캘리브레이션

### 3-1. 새 모터의 ID/baudrate

새 모터 또는 ID가 불명확한 모터만, 프로그램 지시에 따라 한 번에 하나씩
연결한다.

```bash
cd "$LEROBOT_ROOT"
uv run lerobot-setup-motors \
  --robot.type=so101_follower \
  --robot.port=<FOLLOWER_PORT>

uv run lerobot-setup-motors \
  --teleop.type=so101_leader \
  --teleop.port=<LEADER_PORT>
```

이미 ID가 맞는 팔에는 이 단계를 반복하지 않는다.

### 3-2. 캘리브레이션 전 raw 값 기록

```bash
uv run python \
  "$DAPIER_ROOT/so101/hardware_tools/read_only/read_motor_positions.py" \
  --role follower --port <FOLLOWER_PORT> --id so101_follower_main \
  --samples 10 \
  --output "$SO101_EVIDENCE_ROOT/follower-before.jsonl"

uv run python \
  "$DAPIER_ROOT/so101/hardware_tools/read_only/read_motor_positions.py" \
  --role leader --port <LEADER_PORT> --id so101_leader_main \
  --samples 10 \
  --output "$SO101_EVIDENCE_ROOT/leader-before.jsonl"
```

### 3-3. 캘리브레이션

팔로워에는 torque-off interlock이 있는 로컬 보조 스크립트를 사용할 수
있다. 팔을 반드시 지지하고, 화면에 표시된 자세가 안전할 때만 Enter를
누른다.

```bash
uv run python \
  "$DAPIER_ROOT/so101/hardware_tools/writes_hardware/calibrate_follower_safe.py" \
  --port <FOLLOWER_PORT> \
  --id so101_follower_main
```

리더는 공식 LeRobot 절차를 사용한다.

```bash
uv run lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=<LEADER_PORT> \
  --teleop.id=so101_leader_main
```

캘리브레이션 중에는 팔을 받치고, 관절을 천천히 전체 안전 범위로 통과시킨다.
`wrist_roll`은 현재 LeRobot 기준 `0..4095`로 기록되는 축이다.

### 3-4. JSON 검증과 백업

```bash
uv run python \
  "$DAPIER_ROOT/so101/hardware_tools/read_only/validate_calibration.py"

mkdir -p "$SO101_EVIDENCE_ROOT/calibration-backup"
cp -a \
  ~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101_follower_main.json \
  ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/so101_leader_main.json \
  "$SO101_EVIDENCE_ROOT/calibration-backup/"
```

검증 전에는 텔레옵이나 ROS launch를 실행하지 않는다. 캘리브레이션 JSON과
개인 장비 경로는 GitHub에 올리지 않는다.

## 4. 저속 텔레옵 검증

```bash
uv run lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=<FOLLOWER_PORT> \
  --robot.id=so101_follower_main \
  --robot.max_relative_target=5 \
  --teleop.type=so101_leader \
  --teleop.port=<LEADER_PORT> \
  --teleop.id=so101_leader_main \
  --robot.cameras="{front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
  --display_data=true
```

처음에는 아무 물체도 두지 말고 한 관절씩 움직인다. 방향·소음·발열·케이블
장력·통신 오류가 정상인지 확인한다. 급격한 점프나 이상음이 있으면 즉시
전원을 끄고, 포트·ID·캘리브레이션을 다시 확인한다.

동일한 모터 버스에 ROS launch와 LeRobot 프로그램을 동시에 연결하지 않는다.
한 프로그램을 완전히 종료한 뒤 다른 프로그램을 시작한다.

## 5. 첫 카지노 episode 수집

처음 과제는 카드 한 장만 사용한다. 카드에는 개발 단계에서 AprilTag/ArUco
또는 고대비 마커를 붙여 성공 판정을 쉽게 만든다.

고정 조건:

- overhead/front 카메라 1대, 640×480, 30fps
- 카드 시작 위치와 목표 위치를 테이프로 표시
- 작업자 손이 화면과 로봇 작업 영역을 가리지 않음
- 동일한 테이블·조명·카메라 위치 유지
- `Pick one card and place it on player_1.` 한 문장만 사용

5개 시험 episode를 먼저 녹화하고, 영상·action 정렬·그리퍼 성공·저장 여부를
검수한다. 시험이 통과하면 같은 조건에서 50개를 수집한다. 모든 episode를
무조건 학습에 넣지 말고 manifest로 판정한다.

```bash
cd "$LEROBOT_ROOT"
uv run lerobot-record \
  --robot.type=so101_follower \
  --robot.port=<FOLLOWER_PORT> \
  --robot.id=so101_follower_main \
  --robot.max_relative_target=5 \
  --robot.cameras="{front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader \
  --teleop.port=<LEADER_PORT> \
  --teleop.id=so101_leader_main \
  --display_data=true \
  --dataset.repo_id=<HF_USER>/casino_one_card_test \
  --dataset.single_task="Pick one card and place it on player_1." \
  --dataset.fps=30 \
  --dataset.num_episodes=5 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=30 \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2
```

시험이 끝나면 각 episode에 대해 다음 상태 중 하나를 선택한다.

- `accepted`, `success=true`: 카드가 목표 영역에 놓이고 가림·충돌·통신 오류가 없음
- `rejected`, `success=false`: 카드 낙하, 가림, 급격한 관절 점프, 저장 오류 등

실패 episode는 지우지 말고 이유를 남긴 뒤 학습셋에는 제외한다.

```bash
cd ~/DAPIER/casino_dealer
python3 -m casino_dealer.episode_cli mark \
  --path "$HOME/.ros/so101_episodes/casino_one_card/episode_000001/episode_manifest.json" \
  --status rejected \
  --success false \
  --failure-reason "card dropped before reaching player_1"

python3 -m casino_dealer.episode_cli validate-tree \
  --root "$HOME/.ros/so101_episodes/casino_one_card"
```

## 6. replay → policy → rollout

데이터 품질을 검수한 뒤 episode 0 하나를 저속 replay한다. 사람과 물체를
치우고 전원 차단 수단을 손에 둔다.

```bash
cd "$LEROBOT_ROOT"
uv run lerobot-replay \
  --robot.type=so101_follower \
  --robot.port=<FOLLOWER_PORT> \
  --robot.id=so101_follower_main \
  --robot.max_relative_target=5 \
  --dataset.repo_id=<HF_USER>/casino_one_card \
  --dataset.episode=0
```

ACT 기준선은 기존 GPU 설정을 그대로 사용한다. CUDA OOM이 실제로 발생할
때만 batch를 8 → 4 → 2로 낮춘다.

```bash
uv run lerobot-train \
  --dataset.repo_id=<HF_USER>/casino_one_card \
  --policy.type=act \
  --policy.device=cuda \
  --policy.repo_id=<HF_USER>/act_casino_one_card \
  --output_dir=outputs/train/act_casino_one_card \
  --job_name=act_casino_one_card \
  --steps=100000 \
  --batch_size=8 \
  --wandb.enable=false
```

추론은 학습과 같은 조건에서 20초만 실행하고 5회 성공률과 실패 원인을
기록한다. 데이터에 없는 조명·카메라·카드 위치를 첫 평가에서 추가하지 않는다.

## 7. 양팔 카지노 딜러 확장

한 팔 카드 pick/place가 안정적으로 된 뒤에만 두 번째 팔을 추가한다.

1. 왼팔은 우선 덱 고정만 scripted trajectory로 수행한다.
2. 오른팔은 기존 one-card policy를 실행한다.
3. 두 팔의 measured state와 action을 같은 timestamp 기준으로 기록한다.
4. 실제 양팔 episode부터 `--arm-spec`을 두 번 사용한다.
5. 덱 고정·카드 집기·배치·release 사이의 충돌 영역을 표로 정의한다.
6. 마지막에 `casino_blackjack_plan --players 1`의 명령을 실행 adapter에 연결한다.

양팔 manifest 예:

```bash
python3 -m casino_dealer.episode_cli init \
  --path "$HOME/.ros/so101_episodes/casino_opening_deal/episode_000001/episode_manifest.json" \
  --task "Deal the blackjack opening hand." \
  --skill blackjack_opening_deal \
  --source rosbag2_mcap \
  --arm-spec left,so101_follower_left,so101_leader_left \
  --arm-spec right,so101_follower_right,so101_leader_right
```

현재 한 대의 SO-101만 있는 동안에는 양팔 데이터를 가장하지 않는다. 한 팔
episode를 먼저 쌓고, 두 번째 팔이 준비되면 새로운 dual-arm 데이터셋으로
분리한다.

## 완료 판정

- [x] `casino_dealer` 테스트 `20/20`과 3인 planner JSON 생성 성공
- [x] 비구동 one-card 기구학 baseline seed `1000..1099` `100/100`
- [ ] follower·leader 포트가 안정적으로 식별됨
- [ ] 두 팔 calibration JSON이 검증기를 통과함
- [ ] 저속 텔레옵에서 방향·발열·케이블·통신 이상 없음
- [ ] 카메라에 작업 영역과 그리퍼가 항상 보임
- [ ] 시험 episode 5개를 검수함
- [ ] 본 episode 50개 이상에서 manifest를 판정함
- [ ] replay episode 0이 안전하게 끝남
- [ ] ACT rollout 5회 결과와 실패 이유를 기록함
- [ ] 한 팔 성공 후에만 양팔 task로 확장함

## 실험 기록을 남기는 방법

각 작업일 끝에 아래 네 가지를 짧게 남깁니다.

1. 오늘 확인하려던 것
2. 실제로 실행한 명령
3. 화면이나 로그에서 확인한 결과
4. 다음에 다시 시작할 첫 명령

오류가 나면 성공한 것처럼 문장을 고치지 않고, 오류 원문과 전원을 끈
시점도 같이 남깁니다. 그래야 다음 사람이 아니라 미래의 내가 같은
문제에서 출발하지 않습니다.

## 즉시 중단 조건

- 모터가 갑자기 튀거나 굳음
- 전류음·기어 갈림·비정상 발열
- 카드나 케이블이 관절에 걸림
- 같은 포트를 두 프로세스가 사용함
- checksum/read timeout이 반복됨
- 사람이 작업 영역 안에 손을 넣어야 함

중단할 때는 먼저 전원을 끄고, 그 다음 프로그램을 종료한다. 빈
`joint_config_file:=` 인자를 launch 명령에 넣지 않는다. override가 필요
없으면 해당 인자를 아예 생략한다.
