# Dataset v3 round-trip · ACT DataLoader smoke

확인일: 2026-08-21
단계: 3/5
범위: native LeRobot Dataset v3 writer 결과를 다시 열어 시간축·episode 경계·padding mask·ACT 입력 계약까지 검증

## 왜 이 순서로 진행하는가

파일이 생성됐다는 사실만으로 ACT 학습 입력이 맞다고 볼 수 없다. ACT는 현재 frame 하나가 아니라 `chunk_size` 길이의 미래 action window를 읽고, episode 끝을 넘는 위치는 `action_is_pad`로 loss에서 제외한다. 따라서 Stage 2 writer 다음에 바로 작은 round-trip을 두어 writer → reopen → temporal query → DataLoader → ACT loss 경로를 끝까지 확인한다. 이 gate를 통과한 뒤에야 Stage 4 evaluator가 같은 action chunk 계약 위에서 오차를 계산할 수 있다.

## 연구 선정 기준과 사용한 정본

이번 단계의 판정 근거는 아래 자료로 제한했다.

| 자료 | 선정 이유 | 실제 반영 | 분류 |
|---|---|---|---|
| LeRobot Dataset v3 공식 문서와 `LeRobotDataset` 구현 | writer lifecycle, reopen, delta window, episode padding의 현재 API 정본 | `create→add_frame→save_episode→finalize→reopen` 및 2×3 fixture 검사 | 즉시 반영 |
| LeRobot `datasets/factory.py`의 `resolve_delta_timestamps` | ACT action index를 dataset FPS 기반 seconds로 바꾸는 공식 경로 | 20 FPS에서 `[0.0, 0.05, 0.1]` exact assertion | 즉시 반영 |
| LeRobot `ACTConfig`와 `modeling_act.py` | `chunk_size`, `n_action_steps`, `action_is_pad`가 loss에 적용되는 현재 구현 | `chunk_size=3`, `n_action_steps=1`, one-forward finite loss 검사 | 즉시 반영 |
| LeRobot dataset/streaming 공식 tests | episode tail mask와 non-padded 값 검증 방식의 정본 | frame별 expected mask와 cross-episode no-leak 검사 | 즉시 반영 |
| Zhao et al., ACT 원 논문(2023) | action chunking·temporal ensemble의 원리 이해 | 구현 판정에는 사용하지 않고 배경 설명에만 사용 | 참고만 |
| Chen et al., Robo-DM(2025) | heterogeneous robot data 저장·decode trade-off 이해 | 현재 storage 병목·ACT padding 근거가 아니므로 코드 미반영 | 참고만/보류 |

최신 VLA·world model 논문은 Dataset v3 reader나 ACT padding 계약을 바꾸지 않으며 이번 실패 위험을 직접 줄이지 않으므로 검토 목록에 추가하지 않았다. 상세 근거와 원문 URL은 [`research/LATEST_ACT_DATALOADER_ROUNDTRIP_RESEARCH_20260821.md`](research/LATEST_ACT_DATALOADER_ROUNDTRIP_RESEARCH_20260821.md)에 기록했다.

## 구현 계약

`native-act-smoke`는 optional ML 환경에서만 LeRobot·Torch를 lazy import한다. 기본 ROS2 recorder 환경에는 이 패키지들을 추가하지 않는다.

검사 순서는 다음과 같다.

1. native v3 dataset을 새 `LeRobotDataset` instance로 reopen한다.
2. 정확히 2 episode×3 frame, 20 FPS, state/action 12D, RGB 3채널, depth 1채널인지 확인한다.
3. 공식 `resolve_delta_timestamps()`가 action window `[0, 1/fps, 2/fps]`를 만드는지 확인한다.
4. 두 episode에서 각각 `[F,F,F]`, `[F,F,T]`, `[F,T,T]` mask가 나오는지 확인한다.
5. episode 0의 padded action이 episode 1 첫 action을 가져오지 않는지 확인한다.
6. DataLoader batch shape가 action `(1,3,12)`, mask `(1,3)`인지 확인한다.
7. 공식 `ACTPolicy.forward()`가 `action_is_pad`를 포함한 batch로 finite loss를 계산하는지 확인한다.
8. 결과·runtime·LeRobot version/commit을 receipt에 저장하고 encoder receipt의 round-trip 상태를 PASS로 갱신한다.

## RGB와 Depth의 경계

native Dataset v3에는 RGB `(3,H,W)`와 depth `(1,H,W)`를 모두 보존하고 round-trip한다. 하지만 공식 ACT ResNet 입력은 3채널 image를 전제로 하므로 이번 ACT baseline forward에는 RGB만 넣었다. 1채널 depth를 3채널로 복제하거나 8-bit RGB로 위장하지 않았다.

Depth 활용은 다음 중 하나를 ACT RGB 기준선과 같은 split에서 ablation할 때만 추가한다.

- depth 전용 encoder 또는 late fusion
- 물리 단위를 유지한 3채널 derived representation
- RGB-only 대비 validation sequence error, task success, safety rejection 개선

## 실행

교육용 Ubuntu PC에서는 별도 ML 가상환경에서 실행한다. ROS2 overlay를 source하는 기본 terminal에 LeRobot을 강제 설치하지 않는다.

```bash
python -m shoe_sorting_data.cli generate \
  --root /tmp/dapier_stage3/raw \
  --count 2 \
  --samples 3 \
  --camera-payload \
  --camera-width 64 \
  --camera-height 64

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

`chunk_size=3`은 3-frame fixture에서 full/tail padding을 모두 검증하기 위한 smoke 전용 값이다. 실제 학습·rollout 추천값이 아니다.

## 2026-08-21 실제 검증 결과

격리 환경: `C:\Users\hjjeon\.cache\dapier\lerobot-v3-d451fe4-cpu`
검증 artifact: `C:\Users\hjjeon\Documents\DAPIER\tmp\stage3-native-roundtrip-20260821-v4`

| 항목 | 결과 |
|---|---|
| native writer/finalize/reopen | PASS |
| episode/frame | 2 / 6 |
| FPS·delta timestamps | 20 FPS / `[0.0, 0.05, 0.1]` |
| 양팔 state/action | float32 12D / 12D |
| RGB/depth | `(3,64,64)` / `(1,64,64)` |
| 두 episode tail masks | `[F,F,F]`, `[F,F,T]`, `[F,T,T]` 각각 반복 |
| cross-episode action leak | 없음 |
| DataLoader | action `(1,3,12)`, mask `(1,3)` |
| ACT one-forward | CPU, finite loss PASS |
| runtime | Python 3.12.6, Torch 2.7.1+cpu, CUDA 미사용 |
| LeRobot | 0.6.2, commit `d451fe4f1f1b00a812f95aa9534389b5e42ab155` |
| 기본 ROS2 환경 영향 | 없음 |

Windows의 TorchCodec는 full-shared FFmpeg DLL이 없어 공식 PyAV fallback 경고를 냈다. 이번 dataset은 `use_videos=False` image 저장이므로 결과에 영향을 주지 않았다. 시스템 FFmpeg를 추가 설치하지 않았고, 향후 video export를 채택할 때만 Ubuntu 격리 환경에서 codec round-trip을 별도 gate로 검증한다.

## PASS가 의미하지 않는 것

- ACT를 학습했다는 뜻이 아니다.
- 실제 신발 정리 성공률을 증명하지 않는다.
- `chunk_size=3`이 현장 최적값이라는 뜻이 아니다.
- depth를 ACT가 사용했다는 뜻이 아니다.
- synthetic 64×64 image가 Astra Pro 실데이터 품질을 대표하지 않는다.

다음 Stage 4에서는 padded timestep을 metric에서 제외하고, chunk size·execution step·action error를 같은 split에서 비교하는 offline evaluator를 만든다.
