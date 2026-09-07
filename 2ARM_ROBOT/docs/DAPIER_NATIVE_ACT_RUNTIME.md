# Stage 6 · LeRobot 완전 독립 DAPIER-native ACT runtime

> 현행 장비 역할은 [`../config/hardware_roles.json`](../config/hardware_roles.json)을 정본으로 한다: SO-101 양팔, 전면 Astra S, top-view HP-ASC-H201, 좌·우 wrist RGB.

## 결과

`shoe_sorting_data.dapier_native_act`가 finalized DAPIER episode를 직접 읽고 다음 경로를 소유한다.

```text
raw episode + RGB/Depth payload
  → episode 내부 H-step action window + padding mask
  → train-only state/action normalization
  → RGB-D CNN + CVAE + Transformer action decoder
  → masked L1 + KL 학습
  → DAPIER-native checkpoint
  → action chunk inference + stale queue reset
  → control_authorized=false proposal
```

이 경로는 `lerobot` package를 import하지 않는다. 기존 `lerobot_v3_encoder.py`와
`native_act_smoke.py`는 공식 구현과 비교하거나 Dataset v3를 교환할 때만 쓰는 optional
reference backend다.

## 왜 이 순서로 구현했나

1. **dataset window를 먼저 직접 소유**했다. 모델보다 먼저 episode 경계, 12차원 관절 순서,
   `[FFF, FFT, FTT]` tail mask를 고정해야 미래 action이 다음 episode로 새지 않는다.
2. **RGB-D decoding과 normalization을 dataset에 붙였다.** Astra raw byte를 검증한 후 RGB 3채널과
   normalized depth 1채널을 만들며, state/action 통계는 train split에서만 계산한다.
3. **작은 ACT core를 직접 구현**했다. 현재 state와 RGB-D를 조건으로 CVAE latent와 learned action
   query가 H-step action을 만들고 padded target은 loss에서 제외한다.
4. **checkpoint round-trip을 학습 gate로 만들었다.** 저장 직전과 다시 load한 모델의 deterministic
   inference tensor가 정확히 같지 않으면 학습 명령이 실패한다.
5. **inference를 제어와 분리**했다. CLI 출력은 `control_authorized=false`이며 action queue reset은 남은
   chunk를 버린다. 실물 명령은 기존 독립 safety supervisor 승인 뒤에만 별도 adapter가 전달한다.

이 순서는 “LeRobot 코드를 복사해 떼는 것”이 아니라 DAPIER의 데이터·하드웨어 계약에 맞는 작은
runtime을 직접 이해하고 소유하기 위한 것이다.

## 구현 범위

| 영역 | 현재 구현 | 검증 |
|---|---|---|
| dependency | NumPy·PyTorch만 optional, LeRobot import 없음 | AST import 검사 |
| dataset | accepted train episode 직접 탐색, state/action 12D flatten, episode-tail padding | 2 episode × 3 frame mask |
| vision | raw `rgb8` 계열 + `16UC1/32FC1` depth를 4채널 tensor로 변환 | payload checksum/shape gate 재사용 |
| model | 작은 CNN, CVAE posterior, Transformer decoder, H-step action head | CPU one optimizer step·finite inference |
| loss | valid scalar만 평균한 L1 + KL | padded timestep mask 사용 |
| checkpoint | config, normalization buffer, model/optimizer state, step | save→load prediction exact equality |
| inference | 파일 checkpoint + raw dataset item → physical-unit action chunk JSON | shape·finite·`control_authorized=false` |
| queue | 앞 `n_action_steps`만 실행 후보로 보관, reset 시 모두 폐기 | stale pop 실패 test |

## Ubuntu ROS2 교육 PC에서 실행

기존 ROS2 workspace를 먼저 build한다.

```bash
cd ~/DAPIER/2ARM_ROBOT
colcon build --symlink-install --packages-select shoe_sorting_data
set +u
source install/setup.bash
set -u
```

ML 환경에는 NumPy와 CUDA 버전에 맞는 PyTorch만 있으면 된다. LeRobot, Datasets, PyArrow는 native
경로에 필요 없다. 기존 교육 PC 환경을 먼저 확인한다.

```bash
ros2 run shoe_sorting_data shoe_dapier_act status
```

작은 독립 smoke는 synthetic RGB-D 2 episode를 만들고 one-step 학습, checkpoint reload,
inference, queue reset까지 실행한다.

```bash
ros2 run shoe_sorting_data shoe_dapier_act smoke \
  --output /tmp/dapier_native_act_smoke
```

실제 accepted train episode로 checkpoint를 만든다. 처음에는 메모리 여유를 위해 작은 batch/chunk로
시작하고 RTX 5050에서 측정 후 늘린다.

```bash
ros2 run shoe_sorting_data shoe_dapier_act train \
  --root output/accepted_episodes \
  --checkpoint output/checkpoints/native_act_step100.pt \
  --chunk-size 16 \
  --batch-size 8 \
  --max-steps 100 \
  --device cuda
```

RGB-only 대조군은 같은 명령에 `--rgb-only`를 붙인다. checkpoint에 depth 포함 여부와
`depth_scale`·`max_depth_m`가 함께 저장되므로 inference가 다른 전처리를 조용히 섞지 않는다.

한 frame에서 action chunk를 생성하되 로봇에는 보내지 않는다.

```bash
ros2 run shoe_sorting_data shoe_dapier_act infer \
  --root output/accepted_episodes \
  --checkpoint output/checkpoints/native_act_step100.pt \
  --item 0 \
  --device cuda
```

## 주장 경계와 다음 현장 gate

- 이것은 ACT의 action chunk·CVAE·masked loss를 직접 구현한 **DAPIER-native 최소 재구현**이다.
- original ALOHA/LeRobot 모델과 topology, preprocessing, weight key가 같지 않으므로 그 checkpoint와
  호환 또는 동등하다고 주장하지 않는다.
- synthetic smoke는 task success 증거가 아니다. 실제 신발 데이터의 held-out offline 평가와
  supervisor-guarded closed-loop 성공률이 별도로 필요하다.
- depth scale 기본값 `1000`은 mm→m 변환 knob다. 전면 Astra S와 top-view HP-ASC-H201 각각의
  실측 depth unit, CameraInfo, registration을 확인한 뒤 장치별로 고정한다.
- 첫 실물 rollout은 `n_action_steps=1`, base stationary, E-stop·joint limit·watchdog 승인으로 제한한다.
- Stage 7에서 inference proposal과 Stage 5 supervisor trace 연결을 완료했다. 실제 checkpoint SHA,
  reset generation, source observation identity를 검증하며 hardware publish는 계속 차단한다.
- 통합 smoke와 다음 현장 gate는 [`NATIVE_ACT_SUPERVISOR_INTEGRATION.md`](NATIVE_ACT_SUPERVISOR_INTEGRATION.md)에서 확인한다.

## 연구 근거

무관한 최신 VLA/world-model을 넣지 않고, 이 구현 결정에 직접 필요한 ACT 원 논문·공식 ACT 코드와
현재 LeRobot ACT 구현만 비교했다. 채택/실험/보류 판정과 원문 링크는
[`research/LATEST_DAPIER_NATIVE_ACT_RESEARCH_20260824.md`](research/LATEST_DAPIER_NATIVE_ACT_RESEARCH_20260824.md)에 기록했다.
