# DAPIER 관점: Generalist GEN-1.5의 양팔 신발 정리 적용 조사

> 확인일: 2026-08-20 (KST)
> 조사 대상: [Generalist AI — *GEN-1.5: Embodied Foundation Models are One-Shot Learners*](https://generalistai.com/blog/gen-1.5) 및 이 페이지가 직접 연결한 Generalist 1차 블로그·공식 매체 asset
> 작성 원칙: **[사실]**은 바로 뒤의 Generalist 공식 URL에 명시되거나 그 페이지가 로드하는 공개 asset에서 확인한 내용이다. **[추론/권고]**는 DAPIER의 하드웨어·기간·예산 제약에 적용해 도출한 판단이다. Generalist의 성능·일반화 주장은 논문 동료심사나 제3자 재현이 아니라 회사 발표 자체 보고임을 전제로 읽는다.

## 1. 결론부터

| 판단 | 결론 |
|---|---|
| GEN-1.5를 DAPIER에서 실행·재현 | **불가/보류.** 발표 페이지에는 weight, inference API, source code, checkpoint, model card, license, training recipe가 연결·공개되어 있지 않다. 파라미터 수, GPU/메모리, camera 수·해상도·FPS, action representation, robot embodiment, data 규모도 명시되지 않았다. [사실: GEN-1.5 페이지](https://generalistai.com/blog/gen-1.5) |
| 지금 채택할 것 | **짧은 재사용 스킬 시연을 data asset로 다루는 방식**, episode에 관측+행동을 함께 남기는 계약, held-out 물체·배치·손/팔 조건 평가, 높은 수준의 조합은 스킬 호출로 분리하는 구조다. 이는 GEN-1.5의 physical prompt 결과를 작은 시스템에 안전하게 축소한 **[추론/권고]**다. [근거: one-shot·composition 설명](https://generalistai.com/blog/gen-1.5#one-shot-in-context) |
| 6주 동안 미뤄야 할 것 | 30초 sensorimotor context를 받는 proprietary foundation model의 복제, 사람 손→JDcobot 직접 imitation, simulator prompt→실기 zero-shot, 1~10-step fine-tune 재현이다. 해당 기능은 대규모 비공개 pretraining의 결과이고 DAPIER에는 그 base model/학습 자원이 없다. [사실: GEN-1.5 공개 범위와 pretraining 설명](https://generalistai.com/blog/gen-1.5#scaling-pretraining) |
| 즉시 거절할 주장 | “ACT에 GEN-1.5를 붙이면 3~12초 데모로 새 신발을 정리한다”, “100 Hz라 JDcobot도 100 Hz 안전 제어가 된다”, “59%/83%가 신발 sorting 성능이다”는 모두 근거가 없다. 공식 평가는 10개의 **짧은 atomic manipulation** task이고 신발·이동 base·JDcobot·30켤레 continuous sorting benchmark가 아니다. [사실: 결과 정의](https://generalistai.com/blog/gen-1.5) |

**DAPIER의 우선순위.** [추론/권고] 6주 MVP는 `ACT(저수준 집기·이동·배치) → deterministic safety gate → 상위 인식/짝추론/스킬 state machine`을 완성하는 데 쓴다. GEN-1.5는 비교 대상 모델이 아니라, 이후 공개 artifact가 생겼을 때 검증할 **연구 가설과 evaluation protocol의 영감**으로만 기록한다. 프로젝트의 확정 제약은 [이동형 양팔 로봇 신발 정리 프로젝트 요구사항 원장](requirements-ledger.md)에 따른 JDcobot300 두 팔, TurtleBot3 Waffle Pi, Orbbec Astra Pro, RTX 5050 laptop, 추가 예산 0원, 4명, 핵심 6주다.

## 2. 원문 범위·메타데이터·공개 artifact 한계

### 2-1. 페이지 인벤토리

| 항목 | 확인 결과 |
|---|---|
| 문서 식별 | HTML title은 *Generalist - GEN-1.5: Embodied Foundation Models are One-Shot Learners*, canonical URL은 본 조사 대상 URL이다. author는 **Generalist Team**, 게시일은 **2026-08-19**, citation은 “Generalist AI Blog, Aug 2026”이다. [사실: 페이지](https://generalistai.com/blog/gen-1.5) |
| 메타데이터 | description/OG description은 “3~12초 데모의 in-context learning 또는 수분 데이터의 1~10 gradient step adaptation”을 표기한다. OG image는 1200×630 `generalist-gen1p5-og.jpg`, Twitter card는 `summary_large_image`다. [사실: 페이지 HTML](https://generalistai.com/blog/gen-1.5) |
| 본문 목차(8개 항목) | Introduction / Introducing GEN-1.5 / Scaling Pretraining for Robotics / One-Shot Learning In-Context / Few Gradient Step Adaptation / Physical Generalization / Looking Ahead / Citation. 단, DOM에서 `Introduction`은 H2가 아니라 abstract 문단의 `id="introduction"`이고, 실제 H2 8개에는 게시일 heading이 포함된다. H3 4개 중 하나는 Table of Contents이고 나머지 세 개가 composition, sim2real, human-to-robot 하위 절이다. [사실: 페이지](https://generalistai.com/blog/gen-1.5) |
| 정적·동적 도표 | Fig. 1~4, 16개 figcaption, per-task success chart, validation next-action-prediction-error line chart, fine-tuned checkpoint MDS map가 있다. 결과/학습 차트 수치는 페이지가 직접 불러오는 JS에 평문으로 들어 있다. [사실: results chart](https://generalistai.com/assets/pages/blog/gen-1.5/assets/results-chart.js), [training chart](https://generalistai.com/assets/pages/blog/gen-1.5/assets/training-plot.js), [checkpoint map](https://generalistai.com/assets/pages/blog/gen-1.5/assets/checkpoint-map.js) |
| 이미지 | 본문 `<img>`는 alt 없는 cover-frame PNG 1개(2,889,486 bytes)이며, OG image는 meta tag asset이다. cover는 WebGL/canvas animation script도 사용한다. [사실: cover image](https://generalistai.com/assets/pages/blog/gen-1.5/assets/images/generalist-gen1p5-cover-frame.png), [cover script](https://generalistai.com/assets/pages/blog/gen-1.5/assets/cover.js) |
| 영상 | 공식 YouTube embed 1개(`1cllCVK-9lo`, title “Introducing GEN-1.5, a one-shot learner”)와 first-party MP4 42개가 있다. 42개 원본을 모두 내려받아 검사한 합계는 **521.345초, 15,893 frame, 212.91 MiB**이고 codec은 모두 H.264다. 해상도 7종(640×358~2560×720), frame rate 4종(23.976/29.97/30/100 fps), audio stream 있음 36개·없음 6개다. 페이지 DOM의 video tag는 loop/muted로 재생된다. [사실: 페이지](https://generalistai.com/blog/gen-1.5), [공식 YouTube](https://www.youtube.com/watch?v=1cllCVK-9lo) |
| 동적/접근성 한계 | 도표는 JavaScript가 실행되어야 보이며, 영상은 별도 공식 transcript·caption track이 페이지에서 링크되지 않는다. 42개 MP4의 시작·중간·끝 **126개 대표 frame**을 montage로 육안 확인하고 전체 stream metadata를 검사했다. YouTube는 213초·640×360·24 fps metadata와 제목을 확인했으나 transcript는 제공되지 않았다. 모든 15,893 frame의 각 pixel 또는 모든 audio sample을 의미론적으로 전수 판독했다고 주장하지 않는다. |

### 2-2. 직접 연결된 first-party 기술 문맥과 공개성

GEN-1.5 본문이 직접 연결하는 Generalist 기술 글은 아래 네 개다. 각 글의 외부 참고문헌은 그 논문의 1차 출처이지 GEN-1.5의 code/weight는 아니다.

| 직접 링크 | 이번 조사에서 쓰는 의미 | 공개 artifact 판단 |
|---|---|---|
| [GEN-0 (2025-11-04)](https://generalistai.com/blog/gen-0) | physical interaction으로 학습하는 embodied foundation model이라는 이전 세대 문맥 | 페이지에는 자체 공개 checkpoint/code가 없다. |
| [GEN-1 (2026-04-02)](https://generalistai.com/blog/gen-1) | GEN-1.5가 말하는 “mastery”와 GEN-1의 99%+ task claim의 출처 | Generalist 자체 보고 blog이며 공개 모델 recipe가 아니다. |
| [Physical Commonsense (2026-01-29)](https://generalistai.com/blog/physical-commonsense) | unexpected variation에 대한 closed-loop physical intuition이라는 회사의 개념 설명 | code/weight가 아니다. |
| [The Robots Build Now, Too (2025-09-24)](https://generalistai.com/blog/the-robots-build-now-too) | one-shot assembly를 internal evaluation으로 소개한 선행 blog | 공개 benchmark/data/model 링크가 아니다. |

**재현성·license 결론.** [사실] GEN-1.5 페이지에는 paper PDF/arXiv, GitHub, Hugging Face, checkpoint, API spec, model/data card, license 링크가 없으며 partner 문의만 제공한다. [추론/권고] 따라서 코드에 `GEN-1.5` adapter를 만들거나 상용 사용 가능성을 전제하지 말고, 향후 계약/API/weight가 공식적으로 제공될 때에만 license, commercial terms, robot safety interface를 별도 검토한다. [근거: GEN-1.5 페이지](https://generalistai.com/blog/gen-1.5)

## 3. GEN-1.5가 실제로 공개한 모델·데이터·학습·평가 정보

### 3-1. 명시된 계약과 명시되지 않은 값

| 항목 | 공식 공개 내용 | DAPIER 해석 경계 |
|---|---|---|
| 모델 | “large multimodal model”, video와 other sensor/language/proprioceptive input을 처리한다. video memory window는 **30초**, output은 **100 Hz action trajectories**다. [사실: 모델 소개](https://generalistai.com/blog/gen-1.5) | architecture, parameter 수, tokenizer, action chunk/absolute-vs-delta, observation sampling, latency, RAM/VRAM은 **미공개**다. RTX 5050에서 실행 가능하다고 추론할 수 없다. |
| physical prompt | prompt는 sensor data+action trajectory의 sensorimotor sequence이며 30초 context 안에 넣는다. human은 handheld gripper pair, 또는 robot rollout으로 기록한다. one-shot은 **3~12초** single demo, gradient update 0회다. [사실: one-shot 절](https://generalistai.com/blog/gen-1.5#one-shot-in-context) | human hand 영상만으로 JDcobot joint action을 생성하는 공개 방법은 없다. 영상과 양팔 상태·행동의 time alignment가 필수라는 점만 유용한 설계 단서다. |
| training data | physical experience에서 ground-up pretrain했고 8개월 이상 continuous training, held-out validation next-action error가 3 training phase에서 하락했다고 한다. pretraining은 random continuous span이며 prompt처럼 discontinuous jump로 examples를 pack하지 않았다고 설명한다. [사실: scaling·one-shot 절](https://generalistai.com/blog/gen-1.5#scaling-pretraining) | 총 hours/scenes, source mix, embodiment, camera/sensor layout, label, filter, split, compute가 미공개다. “대규모”를 DAPIER 6주 데이터 양과 동등시하면 안 된다. |
| simulation | pretraining에는 rendered video/dynamics를 포함한 simulation data가 없다고 하며, simulated rollout을 prompt로 real robot behavior를 유도한 사례를 제시한다. [사실: sim2real 절](https://generalistai.com/blog/gen-1.5#sim2real) | simulator 종류, asset, physics, sim/real calibration, task별 trial 수가 없어 DAPIER의 zero-shot 성공 근거가 아니다. |
| few-step adaptation | 1~10 gradient step, 1~5분 데이터(약 10~50 demonstration)를 주장한다. 10 step/5분에는 pretraining과 유사 hyperparameter를 썼고, one-step/1분 held-out task success 66.5%를 보고하며 adaptation-specific tuning/sweep을 하지 않았다고 쓴다. 10 steps의 held-out weights change는 0.15% 미만이라고 한다. [사실: few-step 절](https://generalistai.com/blog/gen-1.5#few-gradient-steps) | optimizer, LR, batch, trainable layer, seed, task split이 없어 그대로 재현할 수 없다. |
| hardware/deployment | robot type, arm DoF, gripper, camera model/count/placement/resolution/FPS, control stack, safety enclosure, compute/GPU, end-to-end latency, memory/power는 **명시되지 않았다**. [사실: 페이지 전체 공개 범위](https://generalistai.com/blog/gen-1.5) | 100 Hz는 action output rate일 뿐 camera/robot command rate 또는 safe deployment latency가 아니다. |

### 3-2. 평가 정의와 수치

[사실] Figure 2는 **10개 diverse, short-horizon atomic manipulation task**의 task-success rate다. 10 gradient step은 task당 5분 데이터, in-context는 12초 demo (본문 일반 설명은 3~12초)다. 평균은 각각 **83% ±9%**와 **59% ±10%**(std. dev.)로 발표되었다. [결과 chart source](https://generalistai.com/assets/pages/blog/gen-1.5/assets/results-chart.js), [본문/figure caption](https://generalistai.com/blog/gen-1.5)

| task | 10 gradient steps / 5분 | in-context / 12초 | 주의 |
|---|---:|---:|---|
| Retrieve money from purse | 83.3% | 60.7% | 독립 trial 수·CI·성공 판정 규칙은 미공개 |
| Fold and crease paper | 69.3% | 50.0% | 위와 같음 |
| Twist lid off glass jar | 94.5% | 60.0% | 위와 같음 |
| Stack two small cups | 75.0% | 67.0% | 위와 같음 |
| Sweep trash with brush | 99.0% | 37.3% | 위와 같음 |
| Open book cover | 82.7% | 54.7% | 위와 같음 |
| Brush cube into bowl | 71.2% | 60.8% | 위와 같음 |
| Flip phone upside down | 81.0% | 78.0% | 위와 같음 |
| Unzip pencil pouch | 86.0% | 55.5% | 위와 같음 |
| Remove vacuum pad | 86.0% | 64.0% | 위와 같음 |

[사실] Figure 3 script는 held-out validation **next action prediction error**를 2025-12-15~2026-08-01 x-axis, Phase 1/2/3 표기로 그린다. 첫 source point는 21.427, 마지막은 11.862이고 UI는 이를 `×10⁻²` scale로 표시한다. loss 정의·unit·validation data size는 공개하지 않는다. [training chart source](https://generalistai.com/assets/pages/blog/gen-1.5/assets/training-plot.js) [추론/권고] 이 곡선을 DAPIER loss target으로 쓰지 말고, DAPIER episode-held-out rollout metric을 별도로 정의한다.

[사실] Figure 4는 10 task의 1/2/5/10 step checkpoint를 pairwise L2 weight distance의 classical MDS 2D embedding으로 그리고 radial-log transform/Procrustes alignment를 썼다고 설명한다. 이는 실제 action-space distance나 task similarity chart가 아니다. [checkpoint-map source](https://generalistai.com/assets/pages/blog/gen-1.5/assets/checkpoint-map.js)

### 3-3. capability claim의 정확한 범위

| claim | 공식 사례 | DAPIER에서의 올바른 해석 |
|---|---|---|
| one-shot | zipper, jar, wallet money 등에서 59% 평균이며 fine-tuned model보다 brittle하다고 명시한다. [사실](https://generalistai.com/blog/gen-1.5#one-shot-in-context) | [권고] local ACT도 1~2개 demonstration만으로 될 것이라 기대하지 말고, 빠른 데모를 **데이터 검수/초기화**에 사용한다. |
| composition | independently recorded unzip+retrieve-money 두 prompt를 이어 intermediate regrasp/reposition/recovery를 보였다고 한다. [사실](https://generalistai.com/blog/gen-1.5#compositional-generalization) | [권고] DAPIER는 learned implicit composition 대신 typed state machine으로 `approach → grasp → carry → place → verify`를 조합한다. |
| human→robot | 사람 손을 robot camera로 보며 시연하고 곧바로 robot hands가 재현하는 사례다. [사실](https://generalistai.com/blog/gen-1.5#human-to-robot) | [권고] 사람 영상은 goal/semantic reference로만 보관한다. JDcobot action training에는 calibrated robot demonstration을 우선한다. |
| physical generalization | brush로 배운 뒤 banana/dustpan, obstacle 제거, 양손 jar twist, 색/category sorting 등 사례를 보여 준다. 일부 표현은 “appears”, “sometimes”, “to the best of our knowledge”라는 저자 관찰이다. [사실](https://generalistai.com/blog/gen-1.5#physical-generalization) | [권고] 신규 신발 일반화는 pairing/recognition test와 grasp safety test를 분리한다. 영상 사례를 safety proof로 간주하지 않는다. |

## 4. 그림·차트·영상 매체 전수 인벤토리

모든 MP4 URL의 공통 prefix는 `https://generalistai.com/assets/pages/blog/gen-1.5/assets/videos/`이며, 아래 표는 페이지가 부여한 player title과 filename을 빠짐없이 기록한다. 한 줄의 `human → rollout`은 두 개의 독립 MP4다. **42 MP4 + YouTube 1개**라는 수는 DOM의 `video-player`와 `iframe` 수로 확인했다.

| 위치/캡션 | asset (title — filename) | 페이지가 주장하는 매체 의미 |
|---|---|---|
| Hero | GEN-1.5 hero video — YouTube `1cllCVK-9lo` | YouTube oEmbed title은 “Introducing GEN-1.5, a one-shot learner”; official Generalist channel embed다. [사실: embed](https://www.youtube.com/embed/1cllCVK-9lo) |
| Fig. 1 one-shot example | Marker Into Cup — `wired_hands_marker_cup_with_label_8x.mp4`; Pour Bolts — `wired_hands_pour_bolts_with_label_8x.mp4`; Zipper — `wired_hands_zipper_8x_with_label.mp4` | sensorimotor human demo prompt와 robot control의 비유. [사실: Fig. 1](https://generalistai.com/blog/gen-1.5) |
| Fig. 2 four prompt/rollout pairs | Physical Prompt/Model Rollout: Twist Lid — `top_down_human_jar.mp4` / `top_down_robot_jar.mp4`; Unzip — `top_down_human_unzip.mp4` / `top_down_robot_unzip.mp4`; Brush Cube — `top_down_human_brush.mp4` / `top_down_robot_brush.mp4`; Remove Vacuum Pad — `top_down_human_vacuum.mp4` / `top_down_robot_vacuum.mp4` | Figure 2의 10 task result 중 보이는 네 사례. |
| prompt UI | Physical Prompt Engineering Interface — `two_task_in_context.mp4` | drag-and-drop으로 prompt를 골라 unzip 후 retrieve-money를 연속 학습하는 live recording이라고 caption이 설명한다. |
| composition | Physical Prompt A — `compose_prompt_a.mp4`; Physical Prompt B — `compose_prompt_b.mp4`; Model Rollout of A+B — `compose_prompt.mp4` | unzip+retrieve-money independent examples의 composition 사례. |
| sim2real | Prompt From Simulation — `sim_to_real_sim.mp4`; Model Rollouts in Real World — `sim_to_real_real.mp4` | simulation-only prompt와 real rollout의 좌/우 비교. |
| human→robot | Human-to-Robot In-Context Learning — `human_to_robot_in_context_learning.mp4` | 본문 sentence와 title은 있으나 별도 figcaption·trial 수는 없다. |
| novel tool opening trio | Brushing a Block Into a Bowl (Expected) — `wide_brush_block_bowl_1.mp4`; Improvisation — `wide_brush_block_bowl_2.mp4`, `wide_brush_block_bowl_3.mp4` | brush demo 뒤 other tools로 새로운 contact sequence를 만든다는 section 도입 사례. |
| brush data/NN | Banana, Block, Bowl Improvisation — `banana_block_bowl_improv.mp4`; Brushing Multiple Blocks — `brush_many_blocks_bowl.mp4`; Ambidextrous Brushing — `ambidextrous_brush_block_bowl.mp4`; Human Demonstration — `human_brush_block_bowl.mp4`; Pretraining NN 1~4 — `brush_block_bowl_neighbor1.mp4`, `neighbor2.mp4`, `neighbor3.mp4`, `neighbor4.mp4` | nearest-neighbor language search over **1,891,392 scenes**라고 caption이 적은 reference set과 fine-tuning/rollout 비교. [사실](https://generalistai.com/blog/gen-1.5#physical-generalization) |
| obstacle | Handling Obstacles — `paper_cube_in_bowl.mp4`; Fine-Tuning Data — `cube_in_bowl_human.mp4`; Model Rollouts — `paper_cube_in_bowl_robot_view.mp4` | paper가 bowl을 덮은 상황을 처리한 1-step adaptation 사례. |
| obstruction | Removing Obstructions — `lego_stuck_robot.mp4`; Fine-Tuning Data — `lego_stuck_human.mp4` | fingertip에 낀 Lego를 반대 손으로 빼는 사례. |
| bimanual | Expected Model Rollout — `twist_lid_expected.mp4`; Improvised Model Rollout — `twist_lid_bimanual.mp4` | one-hand data에도 two-hand lid rotation이 나온다는 사례. |
| organize | Cube Sorting 1 — `cube_sorting_1.mp4`; Cube Sorting 2 — `cube_sorting_2.mp4` | single-block-to-bowl finetune model의 색/category sorting 관찰 사례. |
| unseen objects | Jar — `twist_lid_improv_jar.mp4`; Bottle — `twist_lid_improv_bottle.mp4`; To-Go Cup — `twist_lid_improv_togocup.mp4`; Container — `twist_lid_improv_container.mp4` | 10-step/5-minute jar-lid fine-tune 후 unseen container 일반화라는 사례. |

**매체 해석 한계.** [사실] caption은 위의 task/story를 말하지만, 영상별 camera pose, robot/hand, attempt count, edit 여부, real-time rate, failure clip 비율, audio transcript는 공개하지 않는다. [추론/권고] DAPIER 시연은 동일하게 “잘 된 clip”만 저장하지 말고 episode ID별 모든 attempt/failure/stop reason을 남겨야 한다.

## 5. DAPIER에 적용할 아키텍처 경계

### 5-1. 채택·보류·배제 매핑

| GEN-1.5에서 보이는 원리 | 결정 | DAPIER 구현 경계 |
|---|---|---|
| observation+action을 함께 담은 짧은 physical prompt | **지금 채택** | 로봇 demo를 `skill exemplar`로 보관하되 ACT training episode와 같은 timestamp/frame/calibration contract를 쓴다. Gen 모델 context에 넣어 실행하지는 않는다. |
| 복수 prompt의 implicit composition | **형태만 채택** | symbolic task graph/state machine이 named skill을 호출한다. precondition/postcondition/timeout을 명시한다. |
| 30초 rolling video context + 100 Hz output | **interface 참고만** | sensor/action ring buffer와 execution timestamp logging은 만든다. 30초/100 Hz를 요구사항으로 강제하지 않는다. 실측 control/vision latency로 결정한다. |
| one-shot human-to-robot | **보류** | 사람 시연은 신발 짝 labeling, goal 정의, UX video에 쓴다. action label은 calibrated robot teleop/kinesthetic/replay로 획득한다. |
| sim prompt zero-shot | **보류** | Gazebo/Isaac 등의 simulation은 scene layout/trajectory sanity check에 한정한다. zero-shot policy transfer 실험은 ACT baseline 성공 뒤 별도 research track이다. |
| 1~10 step model adaptation | **보류** | 공개 base model이 없으므로 experimental control은 “ACT를 1분/5분 incremental data로 재학습했을 때”이며 GEN claim 재현이라고 부르지 않는다. |
| emergent improvisation을 곧바로 actuator에 허용 | **배제** | collision, joint limit, base motion, gripper closing은 deterministic guard/stop authority가 모델보다 우선한다. |

### 5-2. 권장 실행 흐름

```text
Astra Pro RGB-D + LDS-02/odometry + joint/gripper state
                 │
                 ▼
  Perception: shoe instance / 3-D pose / left-right pairing confidence
                 │
                 ▼
  Task manager (typed state machine, human-confirm queue)
  select_pair → select_skill → dispatch / retry / quarantine
                 │
                 ▼
  ACT low-level skill: approach / pick / handoff(optional) / carry / place
                 │ proposed action
                 ▼
  Safety supervisor (workspace, joint/velocity, self-collision,
  base-stop, camera freshness, confidence, E-stop) ──reject/stop──► recovery
                 │ approved action
                 ▼
  JDcobot left/right controller + TurtleBot3 base
                 │
                 └────► timestamped episode logger + evaluation manifest
```

[추론/권고] GEN-1.5의 “behaviors can bridge/recover”는 상위 state machine이 **성공 조건을 재관측하고 다음 skill을 고르는** 설계로 번역한다. LLM/VLM은 pair candidate 설명·task ordering·recovery suggestion까지만 제공하며, raw joint/base action 또는 safety override 권한을 받지 않는다. 이는 원장의 “LLM이 실제 관절 행동을 직접 생성하지 않는다”는 기존 원칙과 일치한다.

### 5-3. 양팔 action/state/camera 계약 (초안)

[추론/권고] 아래는 JDcobot의 공개 DoF를 가정하지 않는 symbolic dimension 계약이다. 실제 `n_L/n_R`, gripper command range, command type(position/velocity/trajectory)은 vendor API와 calibration으로 확정한 뒤 versioning한다.

| group | 최소 필드 | 이유/검증 |
|---|---|---|
| common | `episode_id, t_mono_ns, schema_version, robot_config_version, calibration_version, task_id, skill_id, attempt_id` | 모든 sensor/action의 동일 clock와 재현 가능한 split을 만든다. |
| base state/action | `map_T_base`, odometry pose/twist, LDS health; `base_vx, base_wz` | base 이동 중 양팔 motion을 허용할지 state-machine precondition으로 통제한다. |
| left/right state | 각각 `q∈R^n, qdot∈R^n, ee_pose(base frame), gripper_position/effort, controller_status` | left/right ordering, frame, unit(rad/m/degree)을 schema에 고정한다. |
| proposed action | `u_L, u_R, u_base`, action mode, valid-until timestamp | action dim은 controller 명세로 versioned; **두 팔을 단순 concat한 tensor만으로 충돌 안전을 해결하지 않는다.** |
| RGB-D | `rgb`, `depth`, `camera_info/intrinsics`, `base_T_camera`, exposure/depth validity, frame timestamp | Astra Pro의 RGB-D는 recognition/3-D pose source; camera pose 변화·stale frame은 hard reject다. 원장은 Astra Pro를 이 역할로 확정했다. |
| semantic state | shoe instance ID, pair hypothesis IDs, left/right class if known, pose/covariance, occlusion, confidence, target slot/grid ID | pairing confidence가 threshold 미만/API failed면 unclassified tote로 보낸다는 원장 rule을 machine-readable하게 만든다. |
| outcome/safety | success predicate, failure code, contact/collision signal if available, guard decision, E-stop, human intervention | successful clips만 남기는 leakage를 막고 recovery metric을 계산한다. |

**prompt-like exemplar 규칙.** [추론/권고] `exemplar_id`는 3~12초처럼 짧아도 반드시 `camera calibration + robot embodiment + action schema + task goal + start/end condition`을 함께 참조한다. DAPIER는 single Astra/두 팔/platform이라는 외관 차이 때문에 사람이 찍은 임의 영상이나 simulation rollout을 local ACT input tensor에 그대로 섞지 않는다.

### 5-4. DAPIER에서 구현 가능한 ‘physical prompt’의 정직한 축소판

GEN-1.5처럼 원시 영상·센서·proprioception·language를 한 context에 넣어 새 동작을 바로 생성하는 것은 공개 checkpoint/API/architecture가 없어 DAPIER가 지금 재현할 수 없다. 대신 아래 세 수준을 구분한다.

1. **[지금 구현] one-shot perception exemplar:** 새 신발 한 켤레의 RGB-D crop/embedding과 색·형상·크기 feature를 등록해 같은 짝 후보를 찾는다. 이는 새 물체를 예시 하나로 찾는 것이지 **one-shot robot control이 아니다.**
2. **[지금 구현] typed skill exemplar retrieval:** 검증된 robot episode를 `start state + goal + embodiment + calibration + action schema + outcome`과 함께 저장한다. state machine은 유사 exemplar에서 `skill_id`와 parameter를 조회하고, ACT는 현재 관측에 맞는 저수준 action candidate를 만들며, deterministic safety guard가 실행을 승인/거절한다. 이 구조가 DAPIER에서 physical prompt 아이디어를 가장 안전하게 흡수하는 방식이다.
3. **[별도 연구] demo-conditioned ACT:** ACT 입력에 짧은 exemplar context를 실제로 추가하고 같은 controller/action schema 안에서 일반화를 검증할 수는 있다. 그러나 이를 구현·평가하기 전에는 GEN-1.5 재현이나 physical prompting이라고 부르지 않고, `demo-conditioned ACT experimental`로 명시한다.

[사실] GEN-1.5의 model memory는 최대 30초이고 physical prompt는 3~12초 한 번의 sensorimotor demonstration이다. 따라서 남은 context만 rolling observation에 사용할 수 있다. [추론/권고] DAPIER에서 여러 prompt를 조합하려면 길이뿐 아니라 camera/frame/time/action-schema alignment를 먼저 정의해야 하며, 단순 영상 이어붙이기는 허용하지 않는다.

## 6. 데이터 수집·안전·평가 설계

### 6-1. 0원 데이터 수집 우선순위

1. **[권고] 캘리브레이션/guard episode부터:** empty workspace에서 base, left-only, right-only, two-arm separation, E-stop, camera stale를 기록한다. 정책 training에는 제외하지만 safety regression set으로 유지한다.
2. **[권고] atomic skill library:** `approach-grasp-one-shoe`, `lift`, `carry`, `place-grid`, `place-open-bin`, `release/retreat`를 분리해 robot demo로 수집한다. full 30-pair trajectory를 먼저 수집하지 않는다.
3. **[권고] pairing/perception set:** 같은 pair의 좌/우, 유사 색/브랜드의 hard negative, occlusion, sole/upside-down, shadow, distance를 episode-level split으로 수집한다. train frame과 test frame을 섞지 않는다.
4. **[권고] composition/recovery set:** pick failure, grasp miss, object slip, target occupied, low confidence를 의도적으로 label한다. failure가 occur한 trial도 절대 삭제하지 않는다.
5. **[권고] held-out protocol:** (a) seen shoes/new pose, (b) seen pair/unseen lighting, (c) unseen shoe design/size/color, (d) overlap/occlusion을 episode와 physical object 기준으로 분리한다. 원장의 “학습에 없던 디자인·색상·크기” 평가는 (c)로 따로 보고한다.
6. **[권고] exemplar leakage audit:** prompt/retrieval exemplar와 평가 trial이 같은 물리 신발, 촬영 session, 연속 video span, background/fixture ID를 공유하는지 자동 검사한다. frame 단위 random split은 금지하고, object×session 기준으로 split manifest를 고정한다.

### 6-2. safety minimum bar

| gate | dispatch 전 | 실행 중 | 실패 시 |
|---|---|---|---|
| geometry | workspace/fixture/other-arm swept-volume, joint limit, target reachability 확인 | predicted action이 limit/arm separation을 깨면 next command 거절 | hold → retract only if verified-safe → operator check |
| perception | RGB-D timestamp, calibration version, shoe pose/covariance, pair confidence 확인 | stale frame/target lost/occlusion 증가 시 stop | `UNCLASSIFIED` bin 또는 human queue |
| base | base pose stable 및 arm motion 허용 zone 확인 | LDS obstacle/costmap or localization unhealthy면 arm/base stop | waypoint 재계획; no blind continuation |
| actuator | controller heartbeat, gripper state, E-stop healthy 확인 | watchdog/over-current/command timeout 감지 | hardware stop authority가 policy보다 우선 |

[추론/권고] “양손 improvisation” 사례는 DAPIER에서 dual-arm collision exemption이 아니다. 처음에는 **one arm active, other arm parked**를 default로 하고, bimanual handoff/assist는 separation monitor와 explicit coordination test를 통과한 뒤 P2로 연다.

### 6-3. ACT/VLA/GEN-1.5 비교의 공정한 정의

| 축 | ACT baseline | VLA extension | GEN-1.5에서 얻는 실험 가설 |
|---|---|---|---|
| 실제 실행물 | local robot-demo supervised low-level policy | 같은 action/safety contract 위에서 language/vision condition을 추가한 후보 | 공개 weight/API 없으므로 실행 비교군이 아님 |
| 적합한 질문 | 정해진 shoe/workspace에서 pick-place를 안정적으로 하는가 | 신규 shoe semantic instruction/pair explanation이 실패 recovery를 개선하는가 | 짧은 exemplar library가 task bootstrap·skill selection을 도울 수 있는가 |
| 비교 조건 | 동일 robot, camera, action budget, success predicate, held-out episode | ACT와 같은 data split/guard/attempt budget | 단지 연구 배경; 59/83%와 직접 수치 비교 금지 |
| 지표 | skill success, grasp/place pose error, safety reject, p50/p95 loop latency | 위 지표 + pairing F1/abstention calibration/recovery success | demo seconds, adaptation data/minutes, generalization slice를 기록하되 proprietary result와 동치 주장 금지 |

## 7. 6주 MVP 실험 계획과 중단 기준

### 7-1. 최종 범위를 먼저 줄인다

[추론/권고] 원장의 30켤레 최종 비전은 유지하되, 6주 acceptance scope는 **고정된 안전 workspace에서 seen 6켤레의 grid/open-bin 정리 + unseen 2켤레의 pair-confidence abstention/분류 평가**로 설정한다. 신발장 깊은 슬롯·혼잡한 이동·full 30-pair end-to-end는 안정화 4주로 넘긴다. 이는 자원 제약상 portfolio용 검증 가능한 MVP를 만드는 선택이며, 30켤레 목표의 축소 주장이 아니다.

| 주 | 산출물 | 실험/판정 |
|---|---|---|
| 1 | hardware inventory, TF/calibration, `v0` schema, E-stop/parking procedure | joint/base command dry-run과 camera timestamp log. robot DoF·power/CG·teleop 미결은 여기서 명시적으로 close한다. |
| 2 | safety supervisor + logger + atomic teleop/replay | one-arm pick/place safe rollout, all failure codes·intervention recorded. two-arm simultaneous motion은 아직 금지. |
| 3 | ACT `v0` for one shoe→grid and repeatable evaluator | seen object/pose episode-held-out success, action/vision p50/p95 latency, guard reject reason을 baseline으로 고정. |
| 4 | shoe detector/pair scorer + high-level state machine | pair confidence calibration, low-confidence quarantine, open-bin placement; paired task success는 perception·control failure를 분리해 report. |
| 5 | composition/recovery + limited VLA condition | `pick→carry→place→verify→retry` closed loop; ACT-only 대 동일 guard/data split의 language-conditioned candidate를 비교. `prompt-like exemplar` retrieval은 offline skill selection ablation만 한다. |
| 6 | blinded held-out evaluation, demo recording, report/data card | seen/unseen shoes·occlusion slices, safety events, human interventions, continuous attempts 전체를 공개 가능한 portfolio evidence로 묶는다. |

### 7-2. 숫자로 정하는 진행·중단 기준

아래 임계값은 GEN-1.5 결과가 아니라 6주 MVP의 **[추론/권고] 의사결정 기준**이다. 팀이 1주차 hardware envelope을 측정한 뒤 값 자체는 회의로 lock하고 이후 소급 완화하지 않는다.

| checkpoint | 다음 단계 진행 | 중단/축소 trigger |
|---|---|---|
| safety gate | 20회 무물체 dry-run에서 E-stop/heartbeat/limit/stale-frame injection이 기대한 stop state로 100% 전이 | 하나라도 policy command가 guard를 우회하거나 stop 후 restart state가 불명확하면 learning rollout 중단; hardware/state-machine fix 우선 |
| atomic ACT | episode-held-out seen shoe 20 trial에서 task success ≥70%, unsafe event 0, 모든 trial log | <70%면 VLA/dual-arm/long-horizon을 열지 않고 camera/calibration/grasp/data label root cause를 고친다 |
| pairing | hard negative 포함 held-out pair decision에서 threshold별 precision/recall과 abstain rate를 함께 보고 | confidence calibration이 없거나 low-confidence를 강제 pick하면 autonomous final placement 금지 |
| composition | 10 closed-loop pair episodes에서 verify/retry가 failed pick/occupied target을 safe terminal 또는 recoverable state로 100% 분류 | implicit learned composition/LLM free-form action으로 바꾸지 않는다 |
| VLA extension | ACT보다 same-split success 또는 recovery가 의미 있게 개선되고 p95 loop latency·unsafe count가 악화하지 않음 | 개선이 없으면 ACT+symbolic hierarchy를 MVP 최종안으로 하며 VLA는 future work |

## 8. DAPIER에 남는 연구 질문과 다음 확인 사항

1. **공개 접근성:** Generalist가 partner access 외에 versioned API, model card, license, robot input/output schema, latency/SLA를 공개하는지 확인 전에는 integration work를 시작하지 않는다. [사실: GEN-1.5 페이지](https://generalistai.com/blog/gen-1.5)
2. **JDcobot 실행 계약:** 양팔의 실제 DoF, reachable workspace, gripper hardware, command bandwidth/rate, self-collision model, emergency stop, payload/CG/power는 원장에서도 미결이다. 100 Hz 모델 output claim으로 이 값을 채우면 안 된다.
3. **camera/data rate:** Astra Pro의 실제 mounted view, RGB/depth time sync, exposure, depth dropout, robot frame extrinsics를 현장 측정해야 한다. GEN-1.5는 camera 수·resolution·sampling을 공개하지 않았다.
4. **teleoperation:** Generalist의 human prompt는 handheld gripper pair지만 DAPIER teleop device는 미정이다. 목표는 “사람 손 video imitation”이 아니라 robot action label의 정확성·안전성이다.
5. **benchmark:** 30-pair final vision의 success definition(짝 정확도, slot placement, continuous run, human intervention, damage/safety)을 1주차에 고정해야 ACT/VLA의 비교가 공정해진다.

## 9. 인용 방법

Generalist가 제시한 서지는 다음과 같다.
`Generalist Team, “GEN-1.5: Embodied Foundation Models are One-Shot Learners”, Generalist AI Blog, Aug 2026.`
[사실: citation section](https://generalistai.com/blog/gen-1.5#citation)

이 노트가 인용한 정량 그래프의 per-task 값·validation curve·checkpoint figure semantics는 별도 공개 논문 supplement가 아니라 동일 공식 페이지가 load하는 JavaScript asset에서 확인한 것이다: [results](https://generalistai.com/assets/pages/blog/gen-1.5/assets/results-chart.js), [training](https://generalistai.com/assets/pages/blog/gen-1.5/assets/training-plot.js), [checkpoint map](https://generalistai.com/assets/pages/blog/gen-1.5/assets/checkpoint-map.js).
