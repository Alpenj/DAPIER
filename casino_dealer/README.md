# DAPIER Casino Dealer / CardBench

이 폴더는 제가 양팔 카지노 딜러를 만들어보기 위해 먼저 정리한
소프트웨어 실험 영역입니다. 아직 실제 양팔 로봇을 움직이는 패키지는
아니며, 게임 순서와 관측·행동 형식을 먼저 코드로 고정해보는 단계입니다.

양팔 카지노 딜러로 확장하자는 아이디어, episode를 쌓아 policy를 만들자는
목표, 한 팔부터 시작해 두 번째 팔을 붙이자는 순서는 제가 정했습니다.

처음부터 policy가 블랙잭 전체를 알아서 하게 만들려고 하지 않습니다.
현재는 사람이 직접 정한 딜 순서와 테스트 가능한 JSON을 기준으로 삼고,
나중에 카드 집기·놓기 같은 짧은 조작 skill에 episode와 policy를 붙일
예정입니다.

## 지금까지 직접 확인한 것

- CardBench v0 관측·행동 계약을 JSON으로 읽고 검증할 수 있음
- 왼쪽·오른쪽 팔의 상태와 목표를 분리해 표현함
- overhead 카메라와 선택적인 wrist 카메라를 표현함
- 1~7명 블랙잭 초기 딜 순서를 같은 입력에서 재현함
- 왼팔은 덱을 고정하고 오른팔은 카드를 옮기는 첫 역할 분할을 코드로 확인함
- 외부 장비 없이 unit test와 JSON 출력을 실행함
- 기록된 episode를 사람이 검수할 수 있도록 manifest CLI를 추가함
- 왼팔 deck 안정화와 오른팔 one-card pick/place를 분리한 3D 기구학
  baseline을 bounded action으로 실행하고 seed별 receipt를 남김

여기서 “확인”은 현재 코드와 테스트를 실행했다는 뜻입니다. 실제 카드,
카메라, 모터, 진공 장치는 아직 이 패키지에서 연결하지 않았습니다.

## Ubuntu CLI quick start

Python 3.10 or newer is sufficient for the planner and tests. On a new
education PC, configure GitHub authentication first if the DAPIER repository
is private, then run:

```bash
git clone https://github.com/Alpenj/DAPIER.git
cd DAPIER/casino_dealer
python3 -m unittest discover -s test -v
python3 -m casino_dealer.cli --players 3
```

To update an existing checkout later:

```bash
cd DAPIER
git pull --ff-only origin main
cd casino_dealer
python3 -m casino_dealer.cli --players 3 --compact
```

Generate a three-player opening deal:

```bash
cd casino_dealer
python -m casino_dealer.cli --players 3
```

Run the tests without ROS 2:

```bash
cd casino_dealer
python -m unittest discover -s test -v
```

Run the non-actuating one-card baseline:

~~~bash
cd casino_dealer
python -m casino_dealer.card_sim_cli \
  --episodes 100 \
  --seed 1000 \
  --output /tmp/casino_one_card_kinematic_receipt.json
~~~

이 명령은 두 Cartesian tool point, vacuum attachment, table clearance와
target radius만 계산합니다. 카메라·시리얼·모터를 열지 않으며 MuJoCo 같은
동역학 엔진도 사용하지 않습니다. 따라서 결과는 task-level 기구학
baseline이지 실제 카드 집기나 CardBench G6 physics 성공률이 아닙니다.

Build as a ROS 2 Jazzy package:

```bash
cd ~/jdcobot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select casino_dealer
source install/setup.bash
casino_blackjack_plan --players 3
```

## 이 계약을 먼저 정한 이유

The packaged contract is
`casino_dealer/contracts/cardbench_v0.json`. Its key rule is that
`observation.state.*.joint_position` means a measured joint position. A driver
must not label its last command as measured feedback. This distinction matters
for replay, imitation learning, and sim-to-real evaluation.

The v0 action vector contains ten scalar channels:

```text
left joint targets   4
right joint targets  4
left vacuum command  1
right vacuum command 1
```

The low-level action contract uses absolute joint targets. 지금은 이 형식이
실제 하드웨어와 맞는지 확인하는 중이며, 다른 policy나 adapter를 미리
지원한다고 주장하지 않습니다. Vacuum 값은 `float32` 0.0~1.0으로 정해두어
나중에 실제 장치의 명령으로 바꿀 수 있게 했습니다.

## 아직 직접 확인하지 않은 것

- 실제 카드 인식과 카드 방향 판정
- 충돌 영역과 양팔 동시 실행
- vacuum 또는 SO-101 gripper adapter
- episode를 자동으로 성공/실패 판정하는 센서
- teleoperation 기록과 학습 policy의 실물 rollout
- CardBench G6 physics/contact simulation

## 다음에 직접 해볼 순서

1. 한 대의 SO-101로 카드 한 장 pick/place를 안정화합니다.
2. episode를 영상·측정 관절값·명령과 함께 기록합니다.
3. 사람이 성공/실패 이유를 검수하고 ACT 기준선을 학습합니다.
4. replay와 짧은 rollout으로 데이터가 실제 조작에 도움이 되는지 확인합니다.
5. 두 번째 팔은 덱 고정부터 추가하고, 양팔 데이터를 별도 dataset으로 쌓습니다.

## AI 도움을 받은 부분

AI는 제가 정한 아이디어와 작업 순서를 코드·문서로 옮기는 과정에서
ROS/LeRobot 명령어 정리, 반복적인 테스트 코드 작성, 로그 해석을 도왔습니다.
카지노 딜러라는 방향과 무엇을 먼저 검증할지에 대한 판단은 제가 했습니다.
어떤 범위가 안전한지, 실제로 성공했는지, 다음에 무엇을 할지는 직접 장비와
터미널을 보며 결정합니다. 이 구분을 남겨두는 것이 이 저장소의 실험
기록에서 중요합니다.

## SO-101 preparation and episode collection

The planner is intentionally hardware-independent, but a policy needs real
episodes. The repository now includes a small sidecar manifest tool so a
person can record, review, and accept or reject each episode without editing
JSON by hand:

```bash
python3 -m unittest discover -s test -v
python3 -m casino_dealer.episode_cli --help
```

Create a manifest before collecting a single-arm card skill:

```bash
python3 -m casino_dealer.episode_cli init \
  --path "$HOME/.ros/so101_episodes/casino_one_card/episode_000001/episode_manifest.json" \
  --task "Pick one card and place it on player_1." \
  --skill pick_and_place_card \
  --source lerobot \
  --fps 30 \
  --camera front \
  --calibration-ref so101_follower_main.json
```

After recording, review the video and mark the result:

```bash
python3 -m casino_dealer.episode_cli mark \
  --path "$HOME/.ros/so101_episodes/casino_one_card/episode_000001/episode_manifest.json" \
  --status accepted \
  --success true

python3 -m casino_dealer.episode_cli validate-tree \
  --root "$HOME/.ros/so101_episodes/casino_one_card"
```

두 팔 episode를 실제로 만들 때는 calibrated arm마다 `--arm-spec`을 한 번씩
반복합니다:

```bash
python3 -m casino_dealer.episode_cli init \
  --path "$HOME/.ros/so101_episodes/casino_opening_deal/episode_000001/episode_manifest.json" \
  --task "Deal the blackjack opening hand." \
  --skill blackjack_opening_deal \
  --source rosbag2_mcap \
  --arm-spec left,so101_follower_left,so101_leader_left \
  --arm-spec right,so101_follower_right,so101_leader_right
```

The complete human-run sequence, including hardware safety, calibration,
teleoperation, recording, replay, and ACT evaluation, is in
[`docs/SO101_CASINO_DEALER_RUNBOOK_KO.md`](../docs/SO101_CASINO_DEALER_RUNBOOK_KO.md).
