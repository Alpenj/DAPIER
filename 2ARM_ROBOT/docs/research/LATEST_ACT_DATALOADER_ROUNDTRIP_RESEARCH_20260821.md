# 최신 ACT Dataloader·Round-trip 계약 조사 — 2ARM_ROBOT

확인일: 2026-08-21
대상: LeRobot Dataset v3 native export, JDcobot 양팔 12-DoF action/state, Astra Pro 1대, RTX 5050, 4인·6주·추가 예산 0원

## 이 단계를 이렇게 나누는 이유

Dataset v3 directory가 만들어졌다는 것은 파일 writer가 끝났다는 뜻일 뿐, ACT가 실제로 요청하는 action horizon·timestamp·padding mask가 맞다는 증거는 아니다. LeRobot의 train factory는 policy의 delta index를 dataset FPS로 나누어 `delta_timestamps`를 만들고, dataset reader는 episode 경계를 넘는 query를 padding mask와 함께 반환한다. 따라서 Stage 3의 목적은 2 episode×3 frame이라는 작은 fixture로 **writer → reopen → dataset window → PyTorch DataLoader → ACT input contract**를 끝까지 증명하는 것이다. 이는 training 성능 검증이 아니다.

## 공식 실행 흐름

```text
ACTConfig(chunk_size=H)
  └─ action_delta_indices = [0, 1, ..., H-1]
       └─ resolve_delta_timestamps: i / dataset.fps
            └─ LeRobotDataset(delta_timestamps={"action": [...]})
                 └─ __getitem__(t): action [H, 12] + action_is_pad [H]
                      └─ torch.utils.data.DataLoader
                           └─ batch action [B, H, 12], mask [B, H]
                                └─ ACT loss에서 padded timestep mask 적용
```

`ACTConfig.action_delta_indices`는 `list(range(chunk_size))`를 반환한다. LeRobot `resolve_delta_timestamps()`는 각 action index를 dataset FPS로 나눠 seconds 단위 window로 바꾼다. Dataset의 `tolerance_s`는 frame timestamp 간격과 delta timestamp가 `1/fps`의 배수인지 검사하는 허용오차다. 즉 action chunk 길이와 dataset FPS는 서로 독립된 임의 숫자가 아니라 같은 time grid 계약이다.

## 정본에서 확인한 주장과 적용 근거

| 주장 | 공식 1차 근거 | DAPIER 적용 |
|---|---|---|
| v3 write lifecycle은 `create → add_frame → save_episode → finalize`이다. `finalize`하지 않으면 writer footer/metadata가 불완전해 reload할 수 없다. | LeRobot v3 공식 문서·`LeRobotDataset` 구현 | native export 결과를 이 순서와 reopen 검증까지 통과해야 `published=true`로 표시한다. |
| `add_frame`은 user feature와 `task`를 요구하며 feature 누락·추가, dtype/shape 오류를 검사한다. | DatasetWriter 및 official tests | 12-DoF state/action, RGB-D feature의 preflight 실패를 원인별로 receipt에 남긴다. |
| `ACTConfig.action_delta_indices`는 `0..chunk_size-1`; `n_action_steps ≤ chunk_size`이고 temporal ensemble을 쓸 때 `n_action_steps=1`이어야 한다. | `configuration_act.py` | 2ep×3 smoke는 `chunk_size=3`, `n_action_steps=1`로 **계약만** 검증한다. 기본 `chunk_size=100`은 3-frame fixture에 맞지 않는다. |
| train dataset factory는 policy delta indices를 `i/fps`로 바꾼다. | `datasets/factory.py` | export receipt의 FPS와 smoke config의 FPS를 같은 값으로 고정하고, floating timestamp를 임의로 재샘플하지 않는다. |
| dataset은 delta window를 stack하고 episode 밖 timestep에 `<key>_is_pad` bool mask를 준다. streaming 공식 tests는 non-padded 값만 비교하며 padding은 last-valid broadcasting이라고 명시한다. | `streaming_dataset.py`, `test_streaming.py` | action tail 값이 반복되어도 target 정답으로 취급하지 않으며, `action_is_pad`가 loss로 전달되는지 확인한다. |
| ACT model은 reconstruction loss에 `~batch["action_is_pad"].unsqueeze(-1)`를 곱한다. | `policies/act/modeling_act.py` | smoke는 `action_is_pad` key, shape, tail True를 반드시 검사한다. mask 누락 시 ACT 학습 실행을 막는다. |
| DataLoader는 delta window가 확장된 tensor dict를 batch로 묶는다. | v3 공식 docs의 `torch.utils.data.DataLoader` 예 | batch=1 smoke에서 state/action/image key와 shape를 확인한다. RTX 5050에서 full train을 돌릴 이유는 없다. |

## 2 episode × 3 frame 최소 검사 설계

### fixture 고정값

- episodes: 2개, 각 3 frame, 같은 FPS(실제 export receipt의 `fps`)와 각 episode 내부에서 monotonic timestamp.
- action/state: `float32 (12,)`; joint order와 control mode는 Stage 1 hardware profile과 동일.
- task: 각 frame에 non-empty string.
- RGB: `uint8`, 하나의 fixed resolution. Depth가 준비되면 별도 key와 Stage 2 unit/metadata 검사도 추가한다.
- ACT smoke config: `chunk_size=3`, `n_action_steps=1`, `temporal_ensemble_coeff=None`, `batch_size=1`, CPU 또는 CUDA 한 batch read. 이 값은 작은 fixture의 **padding contract test**이며, 실제 rollout parameter 추천값이 아니다.

### 반드시 나와야 할 결과

`delta_timestamps={"action": [0/fps, 1/fps, 2/fps]}`일 때 episode frame index 0, 1, 2의 기대 mask는 아래다.

| current frame | action delta [0, 1, 2] | `action_is_pad` 기대 | 판정 |
|---:|---|---|---|
| 0 | [a0, a1, a2] | [False, False, False] | 정상 complete chunk |
| 1 | [a1, a2, padded] | [False, False, True] | tail padding 검출 |
| 2 | [a2, padded, padded] | [False, True, True] | tail padding 검출 |

padding payload가 last-valid value를 반복하는 구현이어도 mask가 True여야 한다. masked 영역의 action 수치가 특정 값(0 또는 last action)이라는 사실만으로 합격시키면 안 된다. episode 0→1 경계를 건너 action를 가져오지 않는 것도 검사한다.

### pass/fail 기준

**PASS**는 아래 전부가 만족할 때뿐이다.

1. 두 episode가 `save_episode`되고 전체 dataset이 `finalize` 후 새 `LeRobotDataset` instance에서 reopen된다.
2. `len`, episode count, task, float32 `(12,)` state/action, RGB shape가 raw export receipt와 일치한다.
3. `resolve_delta_timestamps(ACTConfig, metadata)`의 action window가 `[0, 1/fps, 2/fps]`이다.
4. direct dataset sample의 `action`은 `(3,12)`, `action_is_pad`는 `(3,)`이며 위 tail mask가 정확하다.
5. DataLoader batch=1의 `action`은 `(1,3,12)`, `action_is_pad`는 `(1,3)`이며 mask가 유지된다.
6. ACT policy/model input smoke에서 `action_is_pad`를 가진 batch를 받아 shape error 없이 one forward/loss path까지 간다. 실제 optimizer step·정책 성능 수치는 이 gate의 범위가 아니다.
7. output receipt가 LeRobot version/SHA, Python/Torch/CUDA, FPS, chunk config, exact mask assertion, PASS/SKIP/FAIL을 기록한다.

**FAIL**: wrong FPS multiple, missing/shape-mismatched mask, cross-episode leakage, unfinalized dataset, raw receipt mismatch, DataLoader collate 오류, ACT key/shape 오류.

**SKIP**: optional `lerobot[dataset]` 또는 PyTorch/codec dependency가 없는 기본 ROS2 environment. SKIP은 ROS recorder가 정상이라는 뜻일 뿐 native v3/ACT compatibility 성공이 아니다.

## 즉시 반영

| 항목 | 왜 지금 하는가 | 최소 구현 |
|---|---|---|
| `act_dataloader_smoke`를 native encoder integration test로 분리 | 2ep×3 fixture는 빠르고 CPU에서도 action horizon/padding 오류를 드러낸다. | temporary output에 dataset 작성·finalize·reopen·DataLoader·ACT one-forward, receipt 출력. |
| `chunk_size=3` fixture config 고정 | 3 frame만으로 full/tail mask 두 경우를 모두 만들 수 있다. | production config와 분리된 `smoke` profile; receipt에 not-for-training 표시. |
| exact `action_is_pad` assertion | tail action 값을 정답으로 학습시키는 사고를 막는다. | frame 0/1/2 expected mask 및 cross-episode no-leak test. |
| FPS/delta preflight | index-to-seconds 변환이 ACT data window의 정본이기 때문이다. | `i/fps` 계산값과 tolerance를 output receipt/test에서 비교. |
| ACT policy forward는 optional integration gate | dataset load 성공과 policy input success는 별개다. | basic environment에서는 skip, optional env에서는 batch=1/one forward만 수행. |

## 실험 후보

| 후보 | 근거와 목적 | 조건 |
|---|---|---|
| `chunk_size` 3/8/16 offline ablation | ACT chunk size는 action horizon을 직접 결정한다. 실제 task 길이·control rate에 맞는 값을 찾아야 한다. | real accepted episode가 충분히 모인 뒤 동일 split에서 validation sequence error, success, supervisor rejection rate를 비교. |
| `n_action_steps>1` rollout latency 실험 | config는 한 번 예측한 chunk에서 여러 step을 실행할 수 있게 한다. | safety supervisor가 먼저 구현되고, executed action index·staleness·abort 기준을 기록할 때만. |
| temporal ensemble | ACT config는 n_action_steps=1을 요구하며 queue 방식과 실행 특성이 다르다. | baseline action queue와 동일 robot/split에서 jitter·success·safety abort를 비교. |
| relative action processor | 공식 tests는 absolute↔relative round-trip과 chunk-level stats를 따로 확인한다. | JDcobot action convention이 absolute target으로 확정된 뒤, gripper 제외 여부와 train-only stats를 고정하고 round-trip을 통과할 때만. |

## 참고만 — 최신 연구/인프라 동향

| 자료 | 직접 관련성 | 이번 결정에서의 위치 |
|---|---|---|
| Chen et al., 2025 Robo-DM | vision/action 등 time-aligned heterogeneous robot data의 storage/decode trade-off를 다룬다. | raw→train format→readback audit 분리는 뒷받침하지만, EBML container 도입이나 ACT padding 규칙의 근거는 아니다. |
| Zhao et al., ACT 원 논문(2023) | action chunking과 temporal ensembling의 원 연구다. | LeRobot ACT implementation의 구조 이해에는 참고하지만, 이번 2025–2026 v3 API compatibility gate의 정본은 현재 LeRobot 코드/테스트다. |

위 논문들은 Stage 3 결정과 직접 연결되는 최소 범위만 참고했다. 최신 VLA/world model 논문은 Dataset v3 writer·ACT mask 계약을 바꾸지 않으므로 이번 조사에서 채택하지 않았다.

## 보류

| 보류 | 이유 |
|---|---|
| 2ep×3 fixture로 ACT training/성공률 주장 | smoke는 API·mask·tensor contract만 증명하며 학습량이 아니다. |
| 기본 ACT `chunk_size=100`을 현장 control 값으로 그대로 사용 | 6주 프로젝트의 actual FPS, action latency, safety supervision, task duration을 아직 측정하지 않았다. |
| padding tail을 삭제해 문제를 숨김 | episode 끝에서 실제 deployment/evaluation의 sequence 처리가 깨질 수 있다. mask와 evaluator에서 명시적으로 다룬다. |
| `action_is_pad` 없이 loss 실행 | action tail의 fabricated value가 learning target으로 섞일 수 있다. |
| torchvision/ACT full training을 ROS recording machine의 기본 검증으로 강제 | optional ML dependency failure가 카메라·teleop 기록을 막아서는 안 된다. |

## 구현 순서와 학습 요약

1. native encoder가 2ep×3 raw fixture를 v3로 export한다. **왜:** writer/finalize/reopen이라는 format의 완결성을 먼저 증명하기 위해서다.
2. ACTConfig의 `action_delta_indices`에서 `delta_timestamps`를 공식 factory로 계산한다. **왜:** index와 time grid가 어긋나면 다른 action horizon을 학습하기 때문이다.
3. frame 0/1/2 tail mask와 cross-episode no-leak를 검사한다. **왜:** padding의 수치가 아니라 mask가 loss 유효 구간을 정하기 때문이다.
4. DataLoader/ACT one-forward를 실행한다. **왜:** dataset reader와 policy processor/model 사이 key·rank·dtype mismatch를 training 전에 찾기 위해서다.
5. 이 gate 후 offline evaluator/action chunk policy를 설계한다. **왜:** 그때부터 error/success 비교가 실제 같은 temporal contract 위에서 가능하기 때문이다.

## 근거 원문과 확인일

모든 링크는 2026-08-21에 확인했다. Stage 3 결정에 직접 필요하지 않은 2024–2026 논문은 채택하지 않았다.

1. Hugging Face, [LeRobot Dataset v3 공식 문서](https://github.com/huggingface/lerobot/blob/main/docs/source/lerobot-dataset-v3.mdx) — `create/add_frame/save_episode/finalize`, temporal window, DataLoader 예시.
2. Hugging Face, [LeRobotDataset 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/lerobot_dataset.py) — delta timestamps, tolerance, `__getitem__` reader 구조.
3. Hugging Face, [dataset factory 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/factory.py) — policy delta indices를 `i/fps`로 변환하는 `resolve_delta_timestamps`.
4. Hugging Face, [ACTConfig 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/configuration_act.py) — `chunk_size`, `n_action_steps`, temporal ensembling validation, `action_delta_indices`.
5. Hugging Face, [ACT 모델 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py) — action queue, temporal ensemble, `action_is_pad`를 쓰는 reconstruction loss.
6. Hugging Face, [Dataset official tests](https://github.com/huggingface/lerobot/blob/main/tests/datasets/test_datasets.py) 및 [streaming dataset 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/streaming_dataset.py), [streaming tests](https://github.com/huggingface/lerobot/blob/main/tests/datasets/test_streaming.py) — shape/type/finalize readback, delta/padding 및 non-padded consistency test.
7. Chen et al., 2025, [Robo-DM 논문 원문](https://arxiv.org/abs/2505.15558) — time-aligned multimodal robot data storage의 최신 참고 근거.
8. Zhao et al., 2023, [ACT 원 논문](https://arxiv.org/abs/2304.13705) — action chunking/temporal ensembling의 원 연구; 이번 v3 API 결정의 정본은 아님.

## 학습 메모

- **강의에서 확인**: `DataLoader`는 단순 반복기가 아니라 policy가 요구하는 시간 window tensor와 batch 차원을 만드는 연결점이다.
- **외부 보강**: LeRobot에서는 policy config가 delta index를 선언하고 dataset factory가 FPS 기반 seconds window로 해석한다. padding은 경계를 넘는 action을 숨기는 것이 아니라 mask로 표시한다.
- **학습자 해석**: DAPIER의 첫 smoke test는 "ACT를 학습했다"가 아니라 "양팔 12-DoF demo가 ACT가 이해하는 시간축/마스크 계약을 통과했다"라는 정직한 포트폴리오 증거다.
- **다음 검증**: real accepted episode에서 production FPS와 desired chunk size를 정하기 전, Stage 4 evaluator가 padded target을 제외하고 action/state error와 success를 계산하는지 검증한다.
