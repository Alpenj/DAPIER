# 최신 Offline Evaluator·Action Chunk 검증 조사 — 2ARM_ROBOT

확인일: 2026-08-21
대상: JDcobot 양팔 12-DoF, Orbbec Astra Pro, ACT baseline, RTX 5050, 4인·6주·추가 예산 0원

## 이 단계를 이렇게 진행하는 이유

ACT의 validation L1이 낮아도 실제 로봇이 신발을 집고 짝을 맞춰 정리한다는 뜻은 아니다. 반대로 episode tail의 padded action까지 error 평균에 섞으면 모델이 내지 않은 가짜 target으로 지표가 좋아지거나 나빠질 수 있다. Stage 4 evaluator는 **(a) train에 쓰지 않은 accepted episode에서, (b) padding과 split/calibration 누수를 제외하고, (c) 12-DoF action chunk 오류와 data/safety 위험을 재현 가능하게 요약**하는 도구다. closed-loop success는 이후 독립 safety supervisor가 있는 real rollout에서 별도로 판정한다.

## 공식 LeRobot 실행 모델에서 확인한 사실

```text
ACTConfig(chunk_size=H)
  └─ action delta [0 ... H-1] / FPS 기반 dataset window
       └─ batch[action: B×H×12, action_is_pad: B×H]
            └─ ACT forward: valid mask로 padded timestep loss 제외
                 └─ offline evaluator: 같은 mask·held-out split로 per-horizon error 계산
                      └─ real rollout: policy.select_action → safety supervisor → observed success
```

- 공식 ACT config는 `action_delta_indices = [0, …, chunk_size-1]`을 쓰며, `n_action_steps`는 chunk size 이하이어야 한다. temporal ensembling이면 매 step 재질의가 필요하여 `n_action_steps=1`로 제한된다.
- 공식 dataset factory는 delta index를 `i / dataset.fps` timestamp로 해석한다. Dataset reader는 episode 경계 밖 query를 `<key>_is_pad` mask로 표시한다.
- 공식 ACT forward는 `~action_is_pad` mask를 action L1 loss에 적용한다. 따라서 evaluator도 같은 유효 timestep 정의를 써야 train validation loss와 비교가 가능하다.
- LeRobot simulator evaluator는 rollout의 reward, success, done을 episode별로 모으고 aggregate success/reward를 기록한다. 이는 offline prediction error와 다른 closed-loop metric이다.

## 즉시 반영 — 6주 안에 코드·데이터로 검증 가능한 계약

### 1. 평가 입력과 누수 차단

evaluator 입력은 `finalized && accepted` raw/v3 episode, fixed policy checkpoint, fixed evaluation manifest 세 가지다. evaluation manifest는 다음을 먼저 고정한다.

| 필드 | 필요한 이유 |
|---|---|
| `split_version`, `split_seed`, episode IDs | 동일 결과를 다시 계산하기 위해서 |
| `scene_id`, `object_set_id`, shoe-pair ID, `session_id`, `operator_id` | 같은 신발·배경·시연자의 거의 동일한 frame이 train/eval에 섞이는 leakage를 막기 위해서 |
| `calibration_id`, calibration SHA, camera/driver version | 카메라 위치·intrinsics가 달라진 episode를 숨기지 않고 drift를 분석하기 위해서 |
| `hardware_profile_sha`, action convention/control mode | joint order·unit·absolute/relative action이 다른 sample을 한 평균에 합치지 않기 위해서 |
| policy checkpoint SHA, train manifest SHA, code SHA | 평가 수치의 원인을 추적하기 위해서 |

split은 frame random split이 아니라 최소 **episode group split**으로 한다. 우선순위는 `object_set_id/shoe-pair ID → scene_id → session_id → calibration_id`다. 데이터가 너무 적어 group hold-out이 불가능하면 evaluator는 `generalization_claim=false`와 overlap 목록을 출력한다. train normalization stats는 train group만으로 만들고 evaluation episode 값이 stats 생성에 들어가면 FAIL이다.

### 2. padding 제외 action-chunk metric

각 sample의 `valid = ~action_is_pad`만 metric 분자·분모에 넣는다. action chunk를 `a_hat[b,h,j]`, target을 `a[b,h,j]`라 하면 다음을 모두 output JSON/CSV에 기록한다.

| metric | 정의 | 해석 |
|---|---|---|
| `valid_step_count[h]` | horizon h에서 valid mask가 False인 sample 수 | tail padding이 얼마나 지표를 제한했는지 |
| `mae_per_joint[h,j]` | valid sample의 `mean(abs(a_hat-a))` | 특정 joint/horizon의 평균 오차 |
| `rmse_per_joint[h,j]` | valid sample의 `sqrt(mean((a_hat-a)^2))` | 큰 오차에 민감한 위험 신호 |
| `max_abs_per_joint[h,j]` | valid sample의 max absolute error | 평균으로 숨은 급격한 command 감지 |
| `chunk_mae[h]` | 12 joint의 valid MAE 요약 | chunk 후반 degradation 관찰 |
| `coverage[h]` | `valid_step_count[h] / evaluated_samples` | horizon 간 metric 공정성 확인 |
| `masked_value_count` | mask True 원소 수 | padding 제외가 실제 적용됐는지 |

global single MAE만 보고하지 않는다. `H=3` smoke의 frame tail이나 실제 H가 큰 dataset에서는 후반 horizon의 coverage가 작다. 따라서 horizon별 지표와 coverage를 같이 기록하며, `coverage`가 사전 정한 최소값보다 작으면 그 horizon의 평균을 성능 결론에 쓰지 않는다.

### 3. 양팔 per-joint/group metric

JDcobot profile의 12 action names를 metadata에서 읽고 positional index 상수로 추정하지 않는다. 기본 그룹은 아래와 같이 report한다.

| 그룹 | 포함 | 별도 보고 이유 |
|---|---|---|
| `left_arm` | left 5 joint | 왼쪽 도달/충돌 경향 확인 |
| `left_gripper` | left gripper 1 | grasp open/close의 단위·saturation이 arm과 다름 |
| `right_arm` | right 5 joint | 오른쪽 도달/충돌 경향 확인 |
| `right_gripper` | right gripper 1 | pair alignment/grasp 실패와 직접 연결 가능 |
| `all_arm` | 양팔 10 joint | body posture/협응의 요약 |
| `all_gripper` | 2 gripper | grasp timing의 요약 |

physical-unit metric과 normalized metric을 섞지 않는다. joint position은 hardware profile의 rad/degree 등 source unit, gripper는 해당 command convention으로 원 단위를 표시한다. normalized MAE는 model comparison 보조값일 수 있지만 supervisor limit 판정에는 쓰지 않는다.

### 4. action chunk/padding 검증 케이스

최소 fixture(2 episode×3 frame, `H=3`)에서 아래를 automated test로 고정한다.

| frame | expected `action_is_pad` | evaluator가 세는 valid action timestep |
|---:|---|---:|
| 0 | `[False, False, False]` | 3 |
| 1 | `[False, False, True]` | 2 |
| 2 | `[False, True, True]` | 1 |

추가 failure tests:

1. padding 값만 크게 바꿔도 masked metric은 변하지 않아야 한다.
2. valid action 하나를 바꾸면 해당 joint/horizon MAE/RMSE가 변해야 한다.
3. episode 0 tail이 episode 1 action을 읽는 경우 FAIL이다.
4. `action_is_pad` 누락·rank mismatch·action shape mismatch는 FAIL이다.
5. `H`, FPS, delta timestamp가 policy checkpoint/config receipt와 다르면 FAIL이다.

### 5. failure taxonomy와 inspection packet

평가자는 sample/episode를 아래 중 하나 이상으로 분류하고, high-error top-K에 대해 RGB thumbnail path·depth availability·target/predicted action·mask·calibration ID를 묶은 inspection packet을 만든다. raw pixel이나 private operator 정보는 새로 복제하지 않고 existing artifact path/anonymous ID만 참조한다.

| 분류 | 정량 trigger 예시 | 다음 행동 |
|---|---|---|
| `data_contract` | hash/schema/timestamp/camera-info mismatch | export/recording 단계로 반송, 학습 제외 |
| `split_or_calibration_leakage` | train/eval group overlap 또는 calibration SHA overlap 정책 위반 | split 재생성, generalization 수치 무효 |
| `action_tail_padding` | mask 오류 또는 low coverage horizon | mask fix/shorter H 후보 검토 |
| `single_joint_or_gripper` | 특정 joint/group max/MAE 초과 | action convention, joint limit, gripper calibration 점검 |
| `bimanual_coordination` | 양팔 arm group error가 함께 상승 또는 placement phase 집중 | teleop demo/task phase/observation 품질 점검 |
| `perception_or_calibration` | same action family에서 scene/calibration 별 error 편차 | RGB-D registration, extrinsic, lighting/occlusion 확인 |
| `off_distribution` | unseen shoe/scene/session 표시와 high error 동시 발생 | 새로운 demo 수집 후보, 성능 일반화 주장 금지 |
| `safety_or_intervention` | offline bound violation 예측 또는 rollout supervisor intervention | rollout 중단·human review, policy 성능으로 합산하지 않음 |

threshold는 초기에는 hard-code된 성공 기준이 아니라 training split quantile/robot joint limit/안전 supervisor limit에서 파생하고, `threshold_version`에 기록한다. 이는 나중에 실패 수치에 맞춰 기준을 바꾸는 일을 막는다.

### 6. offline error와 closed-loop success를 분리

| 질문 | offline evaluator가 답할 수 있는가 | real rollout이 필요한가 |
|---|---|---|
| held-out expert action을 얼마나 근접 예측했나 | 예, padding 제외 MAE/RMSE/horizon/group metric | 아니오 |
| policy output이 train/eval contract·joint limit과 충돌하는가 | 일부 가능, predicted bound/rate precheck | 예, 실제 latency·state drift·contact 확인 |
| 신발 한 쌍을 집어 나란히 배치/신발장에 넣었나 | 아니오 | 예, 시도 단위 task success criterion과 영상 증거 필요 |
| intervention 없이 안전하게 수행했나 | 아니오 | 예, supervisor event/time/원인 필요 |

실제 평가에서는 task success를 시작 전 정의한다. 예: "동일 ID/시각적 짝인 두 신발이 지정된 slot 또는 나란한 target zone에 있고, 둘 다 zone 밖으로 다시 떨어지지 않으며, E-stop/hard-limit intervention 없이 종료". 이 기준은 API/LLM의 짝 판단 정확도와 manipulator 성공을 분리해 각각 기록한다.

### 7. safety/intervention 지표는 성공률의 부속값이 아니다

Stage 5 독립 supervisor 이전에는 real hardware autonomous rollout을 합격 판정에 쓰지 않는다. supervisor 이후 rollout log는 아래를 policy checkpoint·episode와 연결한다.

- `attempt_count`, `task_success_count`, `task_success_rate`와 exact binomial confidence interval(소표본이라 과장 금지)
- hard reject count/rate: joint limit, velocity/rate, workspace, stale observation/action, base moving, watchdog/E-stop reason별
- soft intervention count/rate: human pause/teleop takeover/replan request
- `time_to_intervention`, `time_to_abort`, recovery completion 여부
- near-limit margin min/p05 (hard violation 전의 위험 신호)
- executed action count와 policy query count, chunk index, observation/action staleness
- success 이후 재현성: shoe-pair/scene/session/calibration별 success table

감독기가 reject한 시도는 **success=0**과 별도 `safety_intervention=true`를 함께 기록한다. 단, reject를 policy failure인지 sensor/driver failure인지 taxonomy로 분리해 무조건 모델 탓으로 해석하지 않는다.

## 실험 후보 — gate를 통과할 때만

| 후보 | 가설 | 실행 gate와 결과 해석 |
|---|---|---|
| H=3/8/16 action horizon ablation | 길어진 chunk가 short-horizon error, staleness, intervention을 바꾼다. | 동일 train split/checkpoint budget, same safety config. coverage·horizon error·closed-loop success·intervention을 함께 비교. 하나만 좋다고 채택하지 않는다. |
| action queue vs temporal ensembling | 매 step 재질의/ensemble이 jitter와 task result를 바꿀 수 있다. | `n_action_steps=1` 조건, same scene/object groups, supervisor logs. temporal ensemble이 safety/latency를 악화하면 미채택. |
| calibration-stratified evaluator | calibration ID가 error variance의 큰 원인인지 찾는다. | 각 stratum에 충분한 held-out episode가 있을 때만; 표본이 너무 작으면 descriptive report로만 남긴다. |
| API/LLM pair judgement 분리 평가 | 새 신발의 pair-ID 추론이 manipulation error와 독립적으로 실패할 수 있다. | perception/pair label ground truth가 있을 때 top-1 pair accuracy와 physical sorting success를 별도 report. |

## 참고만 — 직접 관련 최신 연구

| 자료 | 직접 관련성·조건 차이 | 채택하지 않는 이유 |
|---|---|---|
| Dong et al., 2026, PiL-World | policy-in-the-loop evaluation에서 action chunk와 closed-loop observation을 다룬다. | VLA/world model scale과 학습·compute 요구가 DAPIER RTX 5050·6주·무예산 조건을 넘는다. offline evaluator의 "chunk와 closed-loop는 다르다"는 해석 참고로만 둔다. |
| Chen et al., 2025, Robo-DM | time-aligned heterogeneous robot data의 storage/decode 설계를 다룬다. | 평가 metric/ACT padding 계약을 제시하지 않는다. raw→train format provenance의 참고에 한정한다. |

위 외 최신 VLA/world model 논문은 Stage 4의 ACT baseline, mask metric, actual JDcobot 검증을 직접 개선하지 않으므로 포함하지 않았다.

## 보류

| 보류 | 이유 |
|---|---|
| offline MAE만으로 real success rate 추정/발표 | contact, perception drift, latency, mobile-base state, safety reject를 포함하지 못한다. |
| world-model evaluator 학습 | real 2ARM data·GPU·검증 시간이 부족하고 fixed ACT baseline과 비교가 흐려진다. |
| frame random split | near-duplicate image/action leakage로 수치가 부풀 수 있다. |
| padding 포함 global loss/MAE | episode tail fabricated values가 metric을 왜곡한다. |
| calibration revision을 섞은 단일 leaderboard | camera geometry 변화와 policy 일반화를 구분할 수 없다. |
| supervisor 없는 autonomous rollout | 모델/driver/camera 오류가 physical harm으로 이어질 수 있다. |

## 구현 순서와 학습 요약

1. **evaluation manifest/split을 먼저 고정한다.** 왜: 결과를 본 뒤 split을 바꾸는 누수를 막기 위해서다.
2. **공식 `action_is_pad`와 동일한 valid mask로 horizon·joint·group metric을 계산한다.** 왜: ACT가 실제 loss에서 제외한 tail을 evaluator가 다시 포함하면 비교가 무의미해진다.
3. **2ep×3 smoke mutation test를 통과시킨다.** 왜: real data 전에도 padding·cross-episode·shape 오류를 결정적으로 잡을 수 있기 때문이다.
4. **failure taxonomy/inspection packet을 출력한다.** 왜: 평균 하나가 아니라 다음 수집·보정·모델 수정 행동을 정하기 위해서다.
5. **Stage 5 supervisor가 생긴 뒤 offline/closed-loop/safety 지표를 함께 report한다.** 왜: 신발 정리의 실제 성공과 안전한 수행은 서로 다른 증거를 요구하기 때문이다.

## 근거 원문 및 확인일

모든 링크는 2026-08-21에 확인했다. 공식 LeRobot 코드/문서와 Stage 4 조건을 통과한 최소 최신 자료만 사용했다.

1. Hugging Face, [ACTConfig 공식 코드](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/configuration_act.py) — chunk size, n_action_steps, temporal ensemble 제약, delta index.
2. Hugging Face, [ACT 모델 공식 코드](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py) — action queue/ensemble과 `action_is_pad` valid-mask L1.
3. Hugging Face, [dataset factory 공식 코드](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/factory.py) — index/FPS/delta timestamp conversion.
4. Hugging Face, [streaming dataset 코드](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/streaming_dataset.py) 및 [streaming tests](https://github.com/huggingface/lerobot/blob/main/tests/datasets/test_streaming.py) — episode boundary padding, `<key>_is_pad`, last-valid padding과 non-padded consistency.
5. Hugging Face, [evaluation script](https://github.com/huggingface/lerobot/blob/main/src/lerobot/scripts/lerobot_eval.py), [training script](https://github.com/huggingface/lerobot/blob/main/src/lerobot/scripts/lerobot_train.py), [ACT official guide](https://github.com/huggingface/lerobot/blob/main/docs/source/act.mdx) — per-episode reward/success/done, aggregate success, actual rollout evaluation.
6. Dong et al., 2026, [PiL-World 논문 원문](https://arxiv.org/abs/2606.05773) — chunk-wise policy-in-the-loop evaluation의 직접 관련 참고. DAPIER에는 보류.
7. Chen et al., 2025, [Robo-DM 논문 원문](https://arxiv.org/abs/2505.15558) — time-aligned multimodal robot data의 참고. evaluator metric 근거로는 사용하지 않음.

## 학습 메모

- **강의에서 확인**: evaluation은 train loop의 마지막 출력이 아니라, train data·model output·real robot 결과를 같은 계약으로 비교하는 독립 단계다.
- **외부 보강**: ACT의 action chunk는 "여러 행동을 한번에 낸다"는 것 이상으로, horizon별 data coverage와 episode-tail mask를 필요로 한다.
- **학습자 해석**: DAPIER 포트폴리오에서 정직한 문장은 "held-out, padding-excluded 12-DoF action error와 failure breakdown을 먼저 공개하고, real sorting success와 intervention은 별도로 측정했다"이다.
- **다음 검증**: Stage 5 adapter/supervisor에서 action limit, stale observation, base-stationary, E-stop event를 evaluator taxonomy와 동일한 event schema로 기록한다.
