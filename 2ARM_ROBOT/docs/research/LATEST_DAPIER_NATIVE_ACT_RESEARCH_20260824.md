# DAPIER-native ACT 런타임 조사 — LeRobot 의존성 없이 구현할 최소 계약

> 장비 주의 (2026-09-03): 본문의 JDcobot/Astra Pro 표기는 당시 분석 가정이다. 현행 실물 역할은 [`../../config/hardware_roles.json`](../../config/hardware_roles.json)을 따른다.

확인일: 2026-08-24

범위: JDcobot 양팔 12-DoF, Astra Pro RGB-D 원본 계약, ROS2 rollout/safety supervisor, 4인·6주, RTX 5050, 추가 예산 0원

채택 gate: [`RESEARCH_ADOPTION_LEDGER.md`](../RESEARCH_ADOPTION_LEDGER.md)의 6개 질문을 모두 적용했다.

## 결론

**가능하다.** DAPIER는 LeRobot package·dataset reader·processor·rollout을 import하지 않고도, ACT의 핵심인 **현재 observation → 미래 H-step action chunk**, **episode-tail padding mask**, **CVAE 학습 시 L1+KL**, **inference action queue 또는 temporal aggregation**을 자체 구현할 수 있다.

단, 이 결과는 **architecture-compatible minimal reimplementation**이다. 아래 조건을 모두 고정·검증하지 않는 한 LeRobot checkpoint 또는 원본 ALOHA ACT checkpoint와의 binary/weight/rollout 동등성은 주장하지 않는다.

- exact network topology와 parameter name/state-dict mapping
- image resize/channel order/ImageNet normalization, state/action normalization statistics
- action alignment, control frequency, chunk/queue/ensemble semantics
- model/PyTorch/CUDA version, random seed, optimizer/training schedule
- ALOHA 14-DoF·다중 camera·teleoperation 환경과 DAPIER JDcobot 12-DoF·Astra Pro·ROS2의 차이

따라서 native runtime의 초기 checkpoint format은 **DAPIER-native 전용**으로 정한다. 외부 checkpoint를 load하려면 별도 migration/equivalence suite가 통과할 때까지 `UNSUPPORTED`가 올바른 동작이다.

## 현재 결정과 직접 연결되는 정본 사실

| 자료 | 실제로 확인한 주장 | DAPIER와의 차이/한계 |
|---|---|---|
| Zhao et al., ACT 논문 | ACT는 action sequence의 generative model을 학습하고 action chunking·temporal ensembling을 제시한다. 공개 설명은 target joint position action, L1 reconstruction, VAE KL을 사용한다. | 논문은 ALOHA의 14 action dim과 real robot/task 조건이다. JDcobot 12-DoF, Astra 1 camera, 신발 정리 success를 예측하지 않는다. |
| 원본 `utils.py` | episode에서 random start timestep을 뽑고 현재 qpos/images와 이후 action sequence를 가져온 뒤 padding과 `is_pad`를 만든다. qpos/action mean/std를 dataset 전체 stack으로 계산하고 std lower bound를 둔다. | original의 real-data `start_ts-1` alignment hack은 ALOHA timestamp 조건에 의존한다. DAPIER는 Stage 1 header/monotonic action-anchor 계약으로 명시적으로 정렬하며 hack을 이식하지 않는다. |
| 원본 `policy.py` | training은 `num_queries`만큼 action/pad를 자르고, normalized image/qpos/action으로 CVAE를 학습한다. L1에 `~is_pad`를 곱하고 KL weight를 더한다. inference는 future action chunk를 반환한다. | 원본 masked L1 평균의 denominator는 padded 위치도 포함한다. native baseline은 current LeRobot처럼 valid scalar 수로 정규화해 padding 비율 변화에 지표가 흔들리지 않게 한다. 이는 checkpoint-equivalent 변경이다. |
| 원본 `imitate_episodes.py` | 기본은 chunk를 query frequency에 맞춰 하나씩 실행한다. temporal aggregation 선택 시 매 step query, 같은 timestep을 예측한 chunk들을 exponential weight로 합친다. | original loop은 ALOHA environment/constant/real-env code에 묶인다. DAPIER는 ROS2 safety supervisor가 proposal을 승인하기 전에는 실행하지 않는다. |
| 현재 LeRobot `ACTConfig`/`modeling_act.py` | `action_delta_indices = range(chunk_size)`, `n_action_steps ≤ chunk_size`; temporal ensemble은 `n_action_steps=1`을 요구한다. queue가 비면 새 chunk의 앞 `n_action_steps`를 넣고 하나씩 꺼낸다. training L1은 `action_is_pad` valid count로 정규화한다. | LeRobot의 config/dataset/processor/rollout package는 DAPIER native path의 runtime dependency가 아니다. 여기서는 API·mask·queue semantics 비교 정본으로만 사용한다. |
| 현재 LeRobot dataset/relative-action tests | episode tail pad, valid-only comparison, absolute↔relative action round-trip, chunk statistic을 테스트한다. | DAPIER는 LeRobot tests를 실행하지 않아도 같은 failure class를 native test로 재현한다. |

## native ACT의 최소 architecture contract

이 절은 “ACT라는 이름을 쓸 수 있는 최소 범위”다. state-only MLP나 single-step BC를 구현하고 ACT라고 부르지 않는다.

```text
Native episode reader (raw/v3-derived를 직접 읽음; LeRobot import 없음)
  └─ observation at t: workspace RGB[, depth optional] + qpos[12]
  └─ target: action[t : t+H, 12] + action_is_pad[H]
       └─ Native ACT training model
            image encoder + qpos encoder + action-query transformer decoder
            CVAE posterior during training / prior at inference
            outputs action_hat[B,H,12]
       └─ L1(valid only) + beta * KL
            └─ Native checkpoint + stats + config receipt
                 └─ Native runtime: reset / predict chunk / queue / proposal trace
                      └─ independent safety supervisor (separate Stage 5 component)
```

### required interfaces

| interface | 최소 필수 | non-negotiable invariant |
|---|---|---|
| `NativeActConfig` | `action_dim=12`, `chunk_size=H`, image feature shape/order, qpos/action normalization spec, VAE/KL options, queue/ensemble mode | config SHA와 checkpoint가 일치하지 않으면 load 거부 |
| `NativeActBatch` | `images`, `qpos[B,12]`, `actions[B,H,12]`, `action_is_pad[B,H]` | action dimension, dtype float32, mask bool, no cross-episode target |
| `NativeActPolicy.forward_train` | action prediction + finite loss dict | padded action value mutation이 loss/valid metric을 바꾸지 않음 |
| `predict_action_chunk` | `(B,H,12)` normalized 또는 clearly named physical-unit output | output unit/frame이 config receipt와 명시적으로 일치 |
| `reset` / `next_proposal` | episode/rollout/reset generation을 받아 queue를 clear | reset 전 chunk는 절대 re-execute하지 않음 |
| `NativeActCheckpoint` | model state, config, train-only stats, git SHA, dataset/split manifest SHA, tensor feature schema | external LeRobot/ALOHA checkpoint은 default unsupported |

Depth는 Stage 1 CameraInfo/unit/freshness gate가 통과한 후에만 optional visual feature로 추가한다. RGB-only baseline과 RGB-D는 feature schema와 checkpoint ID를 분리한다. depth가 없어도 stage 1 raw contract와 camera quality gate는 유지한다.

## 즉시 반영 — adoption gate를 모두 통과하는 항목

| 변경 | gate 통과 근거 | 검증/제거 방법 |
|---|---|---|
| DAPIER-native data/batch/checkpoint/runtime interface | 현재 dependency risk를 직접 낮추고, 12-DoF·native data에 맞춘다. 작은 fixture/CPU에서도 검증 가능하며 ACT baseline을 대체하지 않고 병렬 경로다. | no-LeRobot import test, schema/hash test. runtime module 삭제로 기존 raw recorder/safety path에 영향 없음. |
| H-step target + `action_is_pad`를 native dataset reader에서 생성 | 원본 ACT와 current LeRobot 모두 chunk/padding을 핵심으로 쓴다. Stage 3 `FFF/FFT/FTT` fixture로 측정 가능하다. | padding mutation invariance, cross-episode no-leak, horizon coverage tests. |
| train-only qpos/action normalization stats와 std floor | 원본 ACT가 qpos/action stats를 사용하며 small dataset에서 scale drift를 막는다. leakage gate와 바로 연결된다. | train/eval overlap tamper FAIL; zero-variance joint finite test; stats receipt hash. |
| valid-count normalized masked L1 + optional KL | current official LeRobot semantics은 valid target 개수로 L1을 정규화한다. DAPIER evaluator와 동일 mask를 공유할 수 있다. | `sum(abs_err*valid)/(valid_count*12)` exact test, no-valid failure/guard, finite KL test. 원본 loss scale과 다르므로 external checkpoint equivalence claim 금지. |
| reset generation·proposal trace를 가진 `n_action_steps=1` native rollout | Stage 5 stale queue/safety gate에 직접 필요하며, first real rollout에서 action provenance가 단순하다. RTX 5050/6주에서 latency를 계측할 수 있다. | reset→old proposal reject, action age/latency receipt; later queue mode 제거 가능. |

## 실험 후보 — baseline 완료 뒤 동일 조건 ablation

| 후보 | 가설 | 사전 측정과 go/no-go |
|---|---|---|
| native CVAE ACT vs deterministic action-chunk decoder | latent variable가 multimodal teleop variation에 도움이 될 수 있다. | 동일 split, image/action schema, train step/seed budget에서 held-out valid-only horizon MAE·real sorting success·intervention을 비교. improvement 없으면 deterministic을 유지. |
| `chunk_size` H=3/8/16 | action horizon이 action error, stale action, policy query rate를 바꾼다. | Stage 4 coverage/horizon metrics + Stage 5 reject/latency log; sample이 적어 coverage가 낮으면 결론 보류. |
| action queue (`n_action_steps>1`) | fewer inference calls may reduce compute pressure but stale proposal 위험을 높인다. | same checkpoint/task/safety profile; action age, reject rate, success를 모두 보고. Supervisor가 stale reject를 늘리면 미채택. |
| temporal aggregation | overlapping prediction 평균이 jitter를 줄일 수 있다. | current ACT rule처럼 per-step query only; joint velocity jitter, success, intervention 비교. RTX 5050 inference p95가 control deadline 안일 때만. |
| RGB-D feature addition | calibrated depth가 shelf/placement geometry에 기여할 수 있다. | Stage 1 depth raw/unit/calibration gate를 pass하고 RGB-only와 exact same group split으로 comparison. |

## 참고만

| 자료/개념 | 참고 이유 | 현재 미채택 이유 |
|---|---|---|
| ACT 논문의 80M 모델, ALOHA success/시간 수치 | action chunk/CVAE의 원리를 이해하는 데 유용 | hardware, camera, action dimension, task, GPU가 달라 RTX 5050·JDcobot 예상 성능 또는 training recipe로 쓸 수 없다. |
| original temporal aggregation `k=0.01` | overlapping chunk aggregation의 reference semantics | zero-filled action을 populated 판정에 쓰는 original detail은 valid zero command와 충돌할 수 있어 native implementation은 explicit validity mask를 쓴다. |
| current LeRobot architecture/config | API/validation semantics의 비교 정본 | LeRobot package/layout/processors/checkpoint format을 runtime dependency로 채택하지 않는다. |

## 보류

| 항목 | 보류 이유 |
|---|---|
| official LeRobot 또는 ALOHA ACT checkpoint load | exact topology/preprocess/state dict/action alignment equivalence가 없으며, false compatibility가 실물 command 위험을 만든다. |
| original ALOHA real-data `start_ts-1` alignment hack | DAPIER은 ROS header/monotonic/action-anchor offset을 기록하므로 implicit one-step shift를 넣을 근거가 없다. |
| delta joint action을 기본 action representation으로 변경 | ACT paper가 target joint position보다 degraded performance를 보고했고, JDcobot control mode/limits 실측이 완료되지 않았다. |
| temporal aggregation/queue를 first real rollout 기본값으로 사용 | stale proposal과 supervisor trace의 원인이 늘어난다. baseline은 `n_action_steps=1`, ensemble off다. |
| new VLA/world-model/auxiliary losses 추가 | 현재 policy/data/safety gate를 직접 개선하는 검증 가설이 없고 6주·RTX 5050 안에서 controlled ablation이 어렵다. |
| original/LeRobot checkpoint-equivalence를 포트폴리오 문구로 주장 | source·version·weights·normalization·rollout equivalence suite가 없는 상태다. |

## architecture-compatible minimal reimplementation과 checkpoint equivalence의 경계

| 수준 | 허용되는 주장 | 요구되는 증거 | 현재 상태 |
|---|---|---|---|
| **Native ACT contract-compatible** | “ACT의 action-chunk, padded-target masking, optional CVAE/temporal aggregation 원리를 DAPIER-native 12-DoF runtime으로 재구현했다.” | native fixture loss/mask/queue/reset tests, own checkpoint round-trip, held-out/rollout evaluation | 채택 가능 |
| **Functional comparison** | “같은 DAPIER split·task·safety profile에서 native variants를 ablation했다.” | fixed manifest, train-only stats, seeds/budget, offline + closed-loop + intervention report | baseline 뒤 실험 후보 |
| **LeRobot checkpoint compatible** | “LeRobot checkpoint을 native runtime에서 같은 output으로 실행한다.” | exact config/topology/weights/preprocess, fixed input output tolerance, tensor-by-tensor and rollout equivalence suite | 보류 |
| **Original ALOHA ACT reproduced** | “원본 ACT/ALOHA 결과를 재현했다.” | original robot/cameras/dataset/control rate/training recipe/eval protocol을 재현 | 범위 밖 |

## 최소 acceptance suite

1. **Dependency absence**: native module import graph에 `lerobot`가 없고, CI/runtime image에서 LeRobot 미설치 상태로 import/fixture train/eval 가능.
2. **12-DoF schema**: qpos/action/output가 정확히 `(12,)` 및 `(B,H,12)`이며 profile joint name/order hash mismatch가 fail-closed.
3. **Padding**: 2 episode×3 frame H=3에서 target mask `[FFF, FFT, FTT]`; padded action value를 바꿔도 loss/valid metric 불변; episode boundary leakage fail.
4. **Stats**: train-only dataset manifest로만 mean/std 생성; eval episode를 주입하면 leakage test fail; zero variance도 finite output.
5. **Training/inference**: CPU small batch one optimization step, finite L1/KL/gradient; native checkpoint save/load에서 fixed input output tolerance; no external checkpoint load.
6. **Chunk runtime**: `reset` 이후 old generation action reject; H=3/n_action_steps=1 proposal trace fields complete; policy inference p50/p95 and action age record.
7. **Safety integration**: `proposed_action`만 supervisor로 전달하며, native policy는 hardware transport import/publish하지 않음; reject trace가 executed action `null`을 남김.
8. **Evaluation**: fixed held-out group split에서 padding-excluded horizon×joint error/coverage; `closed_loop=NOT_MEASURED` unless actual supervisor-guarded rollout evidence exists.

## 구현 순서

1. **data adapter와 stats receipt**를 먼저 만든다. 이유: native model보다 12-DoF action order, split, padding의 정본이 먼저여야 한다.
2. **minimal native ACT training/inference core**를 구현한다. 이유: LeRobot import 없이 action-chunk/CVAE/masked-loss를 one batch로 증명한다.
3. **checkpoint/config compatibility guard와 native round-trip**을 만든다. 이유: 서로 다른 RGB-D feature/action frame checkpoint를 조용히 섞지 않기 위해서다.
4. **queue/reset/proposal trace**를 safety supervisor에 연결한다. 이유: policy output과 executed action을 분리해야 한다.
5. **작은 ablation 후 real rollout**으로 간다. 이유: offline loss만으로 shoe sorting success를 주장하지 않기 위해서다.

## 원문

모든 링크는 2026-08-24에 확인했다. 최신성만을 이유로 VLA/world-model 논문은 포함하지 않았으며, 아래 자료는 이 native ACT 결정에 직접 연결된다.

1. Zhao, Kumar, Levine, Finn, 2023, [Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware (ACT) 논문](https://arxiv.org/abs/2304.13705) 및 [RSS 원문](https://roboticsproceedings.org/rss19/p016.pdf) — action chunking, temporal ensembling, CVAE/L1/KL, target joint position action의 원 구조.
2. Tony Z. Zhao et al., [공식 ACT repository](https://github.com/tonyzhaozh/act), [dataset/stats 코드](https://github.com/tonyzhaozh/act/blob/main/utils.py), [policy 코드](https://github.com/tonyzhaozh/act/blob/main/policy.py), [train/eval/temporal aggregation 코드](https://github.com/tonyzhaozh/act/blob/main/imitate_episodes.py) — original data padding, normalization, loss, query/aggregation 실제 구현.
3. Hugging Face, [current ACTConfig](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/configuration_act.py), [current ACT model](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py), [relative-action tests](https://github.com/huggingface/lerobot/blob/main/tests/policies/test_relative_actions.py) — current comparison semantics: action indices, queue/reset, valid-count mask loss, action chunk test practices.

## 학습 메모

- **강의에서 확인**: ACT의 핵심은 특정 framework가 아니라, 현재 관측에서 미래 action sequence를 학습하고 episode-tail target을 mask하는 시간축 계약이다.
- **외부 보강**: 원본 ACT와 current LeRobot은 같은 개념을 공유하지만 loss normalization, runtime packaging, queue/aggregation implementation detail이 다를 수 있다. “ACT 기반”과 “checkpoint 호환”은 다른 주장이다.
- **학습자 해석**: DAPIER의 설득력 있는 결과는 external dependency를 많이 쓴 것이 아니라, native checkpoint가 어떤 12-DoF schema·stats·mask·safety profile에서 만들어졌는지 추적 가능하고 작게 검증된다는 데 있다.
- **다음 검증**: native implementation 완료 후 `LATEST_OFFLINE_EVALUATOR_CHUNK_RESEARCH_20260821.md`의 mask/split evaluator와 `LATEST_ROLLOUT_SAFETY_SUPERVISOR_RESEARCH_20260821.md`의 proposal/executed supervisor gate를 같은 receipt ID로 연결한다.
