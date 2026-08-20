# DAPIER 관점: DYNA Dyna-2 인프라와 SLIM-0.5B의 이중팔 로봇 적용 조사

> 확인일: 2026-08-20 (KST)
> 조사 대상: [DYNA — *Training Dyna-2 at million-hour scale, repeatably*](https://www.dyna.co/research/dyna-2-infrastructure), [SLIM 프로젝트 페이지](https://kzz1031.github.io/slim-project-page/) 및 해당 페이지가 직접 연결한 저자 원문·공개 구현
> 작성 원칙: **[사실]**은 링크한 1차 출처의 명시 내용, **[추론/권고]**는 그 사실을 이중팔 프로젝트에 적용해 도출한 판단이다. DYNA의 성능·규모 수치는 회사의 자체 보고이며 독립 재현 결과가 아니다.

## 1. 결론부터

두 페이지는 경쟁하는 정책 두 개가 아니다.

| 판단 | 결론 |
|---|---|
| DYNA 인프라 | **대규모 로봇 데이터를 반복 가능하게 만드는 데이터·학습 운영 설계**다. MCAP, 시간 동기화, 버전 스탬프, manifest, local cache, 재시작은 이중팔 데이터 수집에도 직접 유용하다. [사실: DYNA](https://www.dyna.co/research/dyna-2-infrastructure) |
| SLIM | **행동-조건부 예측 latent를 먼저 만들고 flow matching으로 action chunk를 내는 0.47B 정책**이다. 현재 공개 recipe는 두 RGB view, 7D action, 7D state, action horizon 8의 LIBERO 기준이다. [사실: 논문 Appendix A.3–A.4](https://arxiv.org/html/2608.09771), [공개 config](https://github.com/kzz1031/SLIM/blob/main/configs/libero/stage2_policy_h8_40ep.yaml) |
| 이중팔에 대한 최선의 적용 | **DYNA의 data contract/curation 원칙을 먼저 도입하고, 그 위에서 SLIM을 14D 이상 행동·상태 schema로 확장해 작은 closed-loop baseline부터 검증**하는 순서가 타당하다. 이는 두 출처를 결합한 **추론/권고**다. |
| 바로 복제하면 안 되는 부분 | DYNA의 million-hour·PB·multi-cluster 구성은 현재 공개된 재현 artifact가 아니며, SLIM의 논문·공개 코드는 dual-arm 실험을 명시하지 않는다. 따라서 “DYNA 데이터로 SLIM을 그대로 학습” 또는 “7D checkpoint를 이중팔에 재사용”은 근거가 없다. [사실: DYNA 페이지의 공개 링크 목록](https://www.dyna.co/research/dyna-2-infrastructure), [SLIM 논문](https://arxiv.org/html/2608.09771), [SLIM README](https://github.com/kzz1031/SLIM/blob/main/README.md) |

우선순위는 다음과 같다.

1. **P0 — 수집 계약:** 좌·우 action/state, 모든 camera, time base, frame/calibration, gripper/contact, episode·schema·robot-software 버전을 한 episode에 함께 남긴다. 이는 DYNA가 설명한 multi-modal episode/버전 스탬프 원칙을 축소 적용한 **권고**다. [근거: DYNA](https://www.dyna.co/research/dyna-2-infrastructure)
2. **P0 — 안전한 baseline:** action chunk를 짧게 실행하고 매 chunk 재관측하는 SLIM식 closed-loop policy를 만들되, 충돌·joint limit·workspace·gripper interlock은 별도 deterministic guard가 승인해야 한다. 후반부 guard는 **권고**이며, 공개 SLIM 자료에는 그런 안전 계층이 기술되어 있지 않다. [근거: SLIM 방법](https://arxiv.org/html/2608.09771), [공개 구현](https://github.com/kzz1031/SLIM)
3. **P1 — 모델 확장:** `action_dim`, `state_dim`, action normalization, dataset adapter, evaluator를 이중팔 schema로 함께 바꾼다. config의 차원 필드는 가변이지만, canonical data/evaluation은 7D를 전제한다. [사실: config helper](https://github.com/kzz1031/SLIM/blob/main/slim/model/config.py), [data 문서](https://github.com/kzz1031/SLIM/blob/main/docs/data.md)
4. **P2 — scale-up:** 데이터가 병목이 된 뒤에만 MCAP topic-group, DAG, warehouse manifest, NVMe cache를 단계적으로 채택한다. PB/multi-cluster를 먼저 만드는 것은 과잉이다. 이는 DYNA가 보고한 병목 이동 순서를 프로젝트 규모에 맞춰 해석한 **권고**다. [근거: DYNA](https://www.dyna.co/research/dyna-2-infrastructure)

---

## 2. 조사 범위와 페이지·매체 인벤토리

### 2-1. DYNA 인프라 페이지

| 항목 | 확인 결과 |
|---|---|
| 페이지 메타데이터 | 제목은 *Training Dyna-2 at million-hour scale, repeatably*, 작성자는 Dyna Robotics, 표기일은 August 2026, 본문 목차는 Introduction / Scaling challenges / Looking forward / References다. [사실: 페이지](https://www.dyna.co/research/dyna-2-infrastructure) |
| 본문 구조 | `collection → landing bucket → ingestion → training-ready MCAP episodes → curation manifest → GPU training`의 4단계 data lifecycle을 설명하고 storage, ingestion, curation, delivery, optimizer, resilience를 세부 절로 둔다. [사실: Figure 1 및 본문](https://www.dyna.co/research/dyna-2-infrastructure#scaling-challenges) |
| 도표 | Figure 1–11이 있다: lifecycle, topic-group chunking, storage/read benchmark, ingestion DAG/throughput, manifest build/load, cache orchestration/read time, optimizer sharding/cost. 각 caption과 본문 수치를 읽었다. [사실: 페이지](https://www.dyna.co/research/dyna-2-infrastructure) |
| 동적 매체 | HTML은 desktop 및 mobile MP4를 `autoplay`, `loop`, `muted`, `playsinline`, `preload=none`으로 포함한다. 직접 stream metadata 확인 결과 desktop은 H.264 1280×720/30fps/29.70s, mobile은 H.264 750×422/24fps/29.75s이며 오디오 stream은 확인되지 않았다. desktop 영상을 2초 간격 15 frame contact sheet로 추가 확인했으며, collection→ingestion→curation→training lifecycle을 반복 애니메이션으로 보여주고 별도 자막·추가 수치는 보이지 않았다. [사실: desktop asset](https://www.dyna.co/assets/videos/dyna-2-infrastructure.mp4), [mobile asset](https://www.dyna.co/assets/videos/dyna-2-infrastructure-mobile.mp4) |
| 연결된 1차 기술 문서 | MCAP, Airflow, Kubernetes, Alluxio, Muon, FSDP, Slurm, Ansible 등의 공식 문서를 References로 직접 링크한다. DYNA의 구현 code/data/model weight는 이 페이지에서 직접 링크되지 않는다. [사실: References](https://www.dyna.co/research/dyna-2-infrastructure#references) |
| 관련 first-party 페이지 | nav의 [Dyna-2 research page](https://www.dyna.co/dyna-2)는 Dyna-2를 video와 action을 함께/따로 denoise하는 WAM이라고 설명하고, 39개 task의 두 stationary bimanual YAM platform을 held-out robot 평가로 사용했다고 적는다. 이는 인프라 페이지의 모델 문맥을 보완하지만 역시 회사 자체 보고다. [사실: Dyna-2](https://www.dyna.co/dyna-2) |

**매체 해석 한계.** 페이지의 DOM 텍스트·figure caption·수치·MP4 컨테이너/stream metadata는 확인했다. 다만 제공되는 인프라 영상에는 별도 transcript/caption track이 없고, 각 SVG/animation의 모든 프레임을 사람처럼 pixel 단위로 전수 판독했다고 주장하지 않는다. 이 노트의 figure 해석은 caption·접근 가능한 도식 텍스트와 수치에 한정한다.

### 2-2. SLIM 프로젝트 페이지와 직접 연결 artifact

| 항목 | 확인 결과 |
|---|---|
| 페이지 메타데이터 | description은 “compact latent interaction policy for language-guided robot manipulation”이고, 저자는 Jingkai Wang 등 10명으로 표기된다. [사실: 프로젝트 페이지](https://kzz1031.github.io/slim-project-page/) |
| 정적 그림 asset | `teaser_overview`, `mot_architecture`, `idm_fdm_objectives`, `overview_real_evaluation`, `realworld_average`, `ablation_lines`, `attention_heatmap` 및 logo SVG가 있다. 페이지 자체에는 `<video>` asset/link가 없다. [사실: 프로젝트 페이지 HTML](https://kzz1031.github.io/slim-project-page/) |
| 페이지의 결과 요약 | 0.47B parameter, LIBERO 97.5%, LIBERO-Plus zero-shot 77.45%, CALVIN ABC→D 4.556, real-world average progress 67.8, end-to-end policy latency 77.3ms, policy-server GPU memory 2.01GiB를 표기한다. [사실: 프로젝트 페이지](https://kzz1031.github.io/slim-project-page/) |
| 직접 연결 논문 | [arXiv:2608.09771v1 (2026-08-10)](https://arxiv.org/abs/2608.09771), HTML 원문에는 Figure 1–11, Table 1–6, Appendix A.1–A.6이 있다. [사실: 논문](https://arxiv.org/html/2608.09771) |
| 직접 연결 code | [kzz1031/SLIM](https://github.com/kzz1031/SLIM)은 공식 구현이라고 밝히며, training/evaluation/config/reproducibility artifact를 공개한다. repo 메타데이터의 license 표기는 `Other/NOASSERTION`이므로 재배포 전 [LICENSE/NOTICE](https://github.com/kzz1031/SLIM/blob/main/LICENSE)를 별도 검토해야 한다. [사실: repo](https://github.com/kzz1031/SLIM), [NOTICE](https://github.com/kzz1031/SLIM/blob/main/NOTICE) |

---

## 3. DYNA: 무엇을 주장하고, 이중팔 데이터에 무엇을 주는가

### 3-1. 시스템 요약

DYNA는 Dyna-2가 video와 action을 예측하므로 sample마다 “소수 decoded frame + action chunk 길이의 더 긴 proprioceptive state”를 읽어야 한다고 설명한다. 여러 camera/state/action stream의 sample rate가 달라, read pattern도 training hyperparameter가 된다는 주장이다. [사실: DYNA](https://www.dyna.co/research/dyna-2-infrastructure#episode-container-mcap-and-topic-group-chunking)

그에 대한 설계는 다음이다.

| 계층 | DYNA의 명시 설계·측정 | 이중팔 프로젝트 해석 |
|---|---|---|
| episode container | H5+per-frame JPEG에서 MCAP으로 옮기고 H.264 large-GOP와 topic-group chunking을 사용했다. camera끼리, proprio/action끼리 time-major로 묶어 sample당 topic별 read 대신 group별 read를 하게 한다. [사실: DYNA, Figure 2](https://www.dyna.co/research/dyna-2-infrastructure#episode-container-mcap-and-topic-group-chunking) | **[권고]** 최소 group을 `video`, `left+right proprio/action`, `force/contact`처럼 access pattern 기준으로 설계한다. 다만 작은 dataset에서는 먼저 표준 LeRobot/Parquet adapter로 correctness를 검증하고 MCAP 전환은 profiling 후 결정한다. |
| storage/read | 자체 teleop episode에서 JPEG baseline 대비 storage 약 68% 감소(80.3→25.1 MB/camera-minute), default chunk 대비 3.4× 적은 fetch(11.33→3.29), 2.9× 빠른 sample read(27.0→9.4ms)를 보고한다. [사실: DYNA, Figure 3](https://www.dyna.co/research/dyna-2-infrastructure#episode-container-mcap-and-topic-group-chunking) | **[권고]** 이 수치를 다른 camera/FPS/GOP/SSD에서 기대 성능으로 쓰지 말고, 동일한 sample window로 local benchmark한다. |
| ingestion | transform·quality check·feature enrichment을 Airflow DAG로 분해하고, step별 resource allocation, critical/non-critical 분리, durable state, schema/pipeline/robot software version stamp를 둔다. [사실: DYNA](https://www.dyna.co/research/dyna-2-infrastructure#ingestion-dag-decomposition-staggered-starts-and-bin-packed-batches) | **[권고]** 이중팔에는 time synchronization, missing camera, joint discontinuity, gripper/contact invalid, calibration mismatch를 hard/soft quality gate로 명시한다. episode version stamp는 재수집·relabel을 추적하는 P0다. |
| ingestion scale | staggered start와 byte-balanced bin packing으로 weekly throughput 14,000→440,000 episode-hours(31×)라고 보고한다. [사실: DYNA, Figure 5](https://www.dyna.co/research/dyna-2-infrastructure#ingestion-dag-decomposition-staggered-starts-and-bin-packed-batches) | **[권고]** 이중팔 초기에는 Airflow 자체가 아니라 idempotent transform/validation/manifest와 재실행 가능성을 먼저 확보한다. |
| curation | 43M episodes에서 file crawl이 약 48h였던 것을 production DB→warehouse CDC, SQL curation, columnar manifest로 바꾸고, 50M+ row table에서도 full curation이 seconds라고 설명한다. [사실: DYNA](https://www.dyna.co/research/dyna-2-infrastructure#curation-warehouse-queries-and-memory-mapped-tables) | **[권고]** episode table에는 robot ID, 좌·우 end-effector/gripper type, calibration version, task, outcome, camera health, operator/session, source split을 queryable column으로 둔다. |
| manifest load | one rank download + local disk + memory map + rank-wise zero-copy slice로 cold load 737.0→12.4s, per-node resident memory 2151→218GB를 보고한다. [사실: DYNA, Figure 7](https://www.dyna.co/research/dyna-2-infrastructure#curation-warehouse-queries-and-memory-mapped-tables) | **[권고]** small scale에도 immutable manifest와 seed/split 기록은 즉시 적용할 가치가 있지만, mmap/sharded loader는 실제 startup/RAM 병목이 있을 때 도입한다. |
| delivery | Alluxio 기반 node-local NVMe page cache를 warm하고, cloud single-reader 약 200MB/s 대비 cache 약 2GB/s/node, 1PB 1-pass를 57.9→5.8 days라고 보고한다. [사실: DYNA, Figure 8–9](https://www.dyna.co/research/dyna-2-infrastructure#delivery-cluster-local-cache-on-node-nvme) | **[권고]** local NVMe cache는 multi-node/remote object store 때의 P2다. 단일 workstation의 dual-arm 데이터에는 atomic local dataset cache와 checksum이 더 단순하다. |
| training topology | B200 NVLink node 내부에 optimizer state를 shard하고 node 사이에는 recompute를 허용하는 hybrid를 node count로 선택한다. 보고된 node count에서 fully sharded 대비 optimizer step mean이 약 3× 빠르고 broadcast가 7.6× 적다. [사실: DYNA, Figure 10–11](https://www.dyna.co/research/dyna-2-infrastructure#training-topology-aware-optimizer-sharding) | **[권고]** 단일 node SLIM reproduction에는 적용 대상이 아니다. multi-node training에서만 profiler로 inter-node collective가 병목임을 확인한 뒤 검토한다. |
| resilience | Slurm preflight(GPU inventory/error counter/kernel log/disk/container), checkpoint auto-restart, diagnostic attempt key, node recovery, Ansible cluster provisioning을 쓴다. [사실: DYNA](https://www.dyna.co/research/dyna-2-infrastructure#job-resilience-preflight-gating-and-auto-restart) | **[권고]** 수집 로봇에도 별도 적용한다: start 전에 camera/encoder/clock/calibration/storage health를 검사하고, 중단 episode는 immutable partial marker와 failure reason으로 남긴다. |

### 3-2. Dyna-2와 이중팔의 직접 관련성

DYNA의 같은 회사 모델 페이지는 human egocentric video 1M+ hours pretraining, hand-pose에서 유도한 wrist-pose/grasp pseudo-action, 그리고 39개 task/두 stationary bimanual YAM platform의 held-out robot evaluation을 주장한다. 또한 post-training의 14개 task 중 11개가 6-DOF YAM arm 두 개와 parallel-jaw gripper를 쓴다고 적는다. [사실: Dyna-2](https://www.dyna.co/dyna-2)

이는 “대규모 human video가 이중팔 robot representation에 전이될 수 있다”는 **가능성의 회사 측 증거**다. 그러나 Dyna-2 page가 공개한 corpus, pseudo-action extraction, YAM schema, checkpoint, benchmark split, robot-control adapter는 이 조사 범위에서 재현 artifact로 제공되지 않는다. 따라서 이중팔 프로젝트가 이를 baseline으로 채택할 수는 있어도, DYNA 수치의 재현 또는 human data만으로의 성공을 전제할 수는 없다. 첫 문장은 [사실: Dyna-2](https://www.dyna.co/dyna-2), 결론은 공개 artifact 범위에 대한 **추론**이다.

---

## 4. SLIM: 정책·학습·증거의 정확한 범위

### 4-1. 방법

SLIM(Self-supervised Latent Interaction Model)은 DINOv2 visual encoder가 낸 observation latent, current proprioception, action chunk, language를 **observation stream과 action stream의 Mixture-of-Transformers(MoT)**로 처리한다. language는 joint stream token이 아니라 stream별 cross-attention task condition으로 주입한다. 구현에서 hidden dimension은 768, MoT는 16 layer/12 head다. [사실: 논문 §3.2·Appendix A.3](https://arxiv.org/html/2608.09771), [Stage-1 config](https://github.com/kzz1031/SLIM/blob/main/configs/libero/stage1_idm0125_fdm1_h8.yaml)

| 단계 | 입력/목표 | 의미 |
|---|---|---|
| Stage 1 — IDM | current latent + **future** latent + state + language에서 masked/noised action chunk의 flow velocity를 예측한다. [사실: 논문 §3.3](https://arxiv.org/html/2608.09771) | “이 변화는 어떤 action이 설명하는가”를 학습한다. |
| Stage 1 — FDM | current latent + clean action + state + language에서 masked future latent를 예측하고, stop-gradient EMA visual target과 L1을 맞춘다. [사실: 논문 §3.3](https://arxiv.org/html/2608.09771) | action이 유도할 future visual representation을 학습한다. |
| Stage 2 — policy | future observation을 입력/명시 target으로 쓰지 않고, current observation/state/language로 flow-matching action chunk를 생성한다. [사실: 논문 §3.3](https://arxiv.org/html/2608.09771) | deploy 시 future video generation을 수행하지 않는 reactive policy다. |

**중요한 적용 경계.** SLIM의 Stage 1은 IDM과 FDM 모두 action chunk를 요구한다. 즉 action label 없는 일반 video를 그대로 넣는 “video-only SLIM Stage 1”은 공개 recipe가 아니다. Dyna-2의 human-video pseudo-action 접근을 재현하려면 별도 pose/action 추정·품질 검증 연구가 필요하다. 첫 문장은 [사실: SLIM 논문](https://arxiv.org/html/2608.09771), 두 번째는 [사실: Dyna-2](https://www.dyna.co/dyna-2)와 결합한 **추론**이다.

### 4-2. 공개 실험 결과와 읽는 법

| 영역 | 공개 수치 | 엄격한 해석 |
|---|---:|---|
| LIBERO | 1,950/2,000 = **97.50%**. [사실: canonical recipe](https://github.com/kzz1031/SLIM/blob/main/reproducibility/canonical_recipe.json) | simulation의 original four suite 결과다. |
| LIBERO-Plus | 7,768/10,030 = **77.45%**. 동일 checkpoint를 perturbation data로 학습하지 않고 zero-shot 평가했다. [사실: canonical recipe](https://github.com/kzz1031/SLIM/blob/main/reproducibility/canonical_recipe.json) | camera/robot/language/light/background/noise/layout 변화를 포함하지만 dual-arm 실제 하드웨어 결과는 아니다. [사실: Table 1](https://arxiv.org/html/2608.09771) |
| CALVIN ABC→D | five-instruction chain의 average length **4.556**. [사실: Table 2](https://arxiv.org/html/2608.09771) | held-out environment에서 long-horizon composition을 측정하지만 역시 7D action simulation protocol이다. [사실: data docs](https://github.com/kzz1031/SLIM/blob/main/docs/data.md) |
| real world | carrot→bowl, plate stack, toaster→plate, block stack, whiteboard wipe의 5 task에 task당 150 demo, 총 750 demo를 섞어 학습했다. 각 task-condition 10 trials, partial progress score다. [사실: 논문 §4.2](https://arxiv.org/html/2608.09771) | 프로젝트 페이지의 평균 progress **67.8**은 success rate와 동일 지표가 아니다. [사실: 프로젝트 페이지](https://kzz1031.github.io/slim-project-page/) |
| real-world OOD | SLIM은 nominal/distractor/lighting 평균 progress에서 두 비교 baseline보다 높다고 보고하고, background shift에서는 49로 π0.5의 54보다 낮다. [사실: 논문 §4.2](https://arxiv.org/html/2608.09771) | visual robustness가 모든 shift에서 우세하다는 뜻은 아니다. |
| ablation | EMA 사용 시 LIBERO-Plus 66.82→77.45, CALVIN 4.382→4.556이며, 최선 joint loss weight는 IDM:FDM=0.125:1로 보고한다. [사실: 논문 §4.3](https://arxiv.org/html/2608.09771) | 이 ratio/EMA를 dual-arm에서도 그대로 최적값으로 가정할 수는 없다. 이는 **추론**이다. |
| controlled inference | H100 80GB, PyTorch eager/BF16, model-only 측정에서 SLIM은 60.6ms/call, 4.26GiB incremental VRAM, 490.73 GFLOPs/action chunk다. [사실: Table 4·Appendix A.5](https://arxiv.org/html/2608.09771) | project page의 77.3ms·2.01GiB와 protocol이 다르므로 수치를 섞지 않는다. [사실: 프로젝트 페이지](https://kzz1031.github.io/slim-project-page/) |

### 4-3. 공개 구현이 실제로 고정한 것

canonical LIBERO recipe는 DINOv2-B/14와 frozen T5-small, **두 개 224×224 RGB view**, 7D action, 7D proprioception, 8-step horizon을 쓴다. Stage 1은 3 epoch/EMA 0.999/IDM:FDM 0.125:1, Stage 2는 40 epoch/flow matching이며 eight H100 80GB GPU와 global batch 128로 학습했다. [사실: reproducibility record](https://github.com/kzz1031/SLIM/blob/main/reproducibility/README.md), [data docs](https://github.com/kzz1031/SLIM/blob/main/docs/data.md)

`model.action_dim`, `model.state_dim`, `model.action_horizon`, `num_image_views`는 public config surface에 있으며 runtime config로 전달된다. 즉 차원 변경은 구조적으로 가능한 configuration point다. 그러나 canonical evaluator, normalization statistics, dataset keys, action chunk 및 release checkpoint는 7D를 전제한다. **“14D로 config만 바꾸면 재사용된다”는 결론은 성립하지 않는다.** [사실: config helper](https://github.com/kzz1031/SLIM/blob/main/slim/model/config.py), [defaults](https://github.com/kzz1031/SLIM/blob/main/slim/model/defaults.py), [data docs](https://github.com/kzz1031/SLIM/blob/main/docs/data.md)

또한 reproducibility record는 released training source commit과 public reproduction commit을 구분한다. 재현은 main branch가 아니라 기록된 canonical commit/config hash를 pin해야 한다. [사실: reproducibility README](https://github.com/kzz1031/SLIM/blob/main/reproducibility/README.md)

---

## 5. 주장–근거–이중팔 영향 표

| 구분 | 검증 가능한 주장 | 직접 근거 | 이중팔에 대한 영향 |
|---|---|---|---|
| 사실 | DYNA는 camera/proprio/action의 다른 sample rate를 동기 sample로 조립해야 한다고 본다. | [DYNA](https://www.dyna.co/research/dyna-2-infrastructure#episode-container-mcap-and-topic-group-chunking) | 좌·우 arm, wrist/overhead camera, gripper/contact clock을 같은 episode clock으로 연결해야 한다. **[추론]** |
| 사실 | DYNA는 per-episode schema/pipeline/robot software version을 stamp한다고 한다. | [DYNA](https://www.dyna.co/research/dyna-2-infrastructure#ingestion-dag-decomposition-staggered-starts-and-bin-packed-batches) | calibration/controller 변경으로 섞인 data를 query·exclude·reprocess할 수 있다. **[추론]** |
| 사실 | SLIM Stage 1은 action reconstruction과 future-latent prediction을 같이 쓴다. | [논문 §3.3](https://arxiv.org/html/2608.09771) | pairwise handoff/협응 transition에도 action-grounded representation을 줄 가능성이 있다. 단, 실제 검증은 없다. **[추론]** |
| 사실 | SLIM public recipe의 default action/state는 7/7D, image view는 둘이다. | [defaults](https://github.com/kzz1031/SLIM/blob/main/slim/model/defaults.py), [canonical config](https://github.com/kzz1031/SLIM/blob/main/configs/libero/stage2_policy_h8_40ep.yaml) | dual-arm action/state와 camera inventory에는 schema/adapter/eval 변경이 필요하다. **[추론]** |
| 사실 | SLIM real-world result는 5 task/750 demos/10 trials per condition의 progress-score protocol이다. | [논문 §4.2](https://arxiv.org/html/2608.09771) | 해당 결과만으로 bimanual coordination, collision avoidance, long dual-arm handoff의 성능을 주장할 수 없다. **[추론]** |
| 사실 | Dyna-2 page는 bimanual YAM data/evaluation을 언급하지만 public training artifact는 이 인프라 페이지에서 link하지 않는다. | [Dyna-2](https://www.dyna.co/dyna-2), [DYNA infrastructure references](https://www.dyna.co/research/dyna-2-infrastructure#references) | DYNA는 설계 reference이지 drop-in data/model dependency가 아니다. **[추론]** |

---

## 6. 비교: 어느 층을 가져와야 하는가

| 축 | DYNA infrastructure | SLIM | 이중팔 프로젝트 선택 |
|---|---|---|---|
| 주 문제 | million-hour training data lifecycle·throughput·resilience | compact language-conditioned closed-loop action policy | 둘 다 필요하되 **data contract → policy** 순서 |
| data assumptions | multi-modal episode, long proprio/action window, large corpus | labeled robot trajectories with current/future visual observation and action chunks | left/right state/action, camera, contact, calibration을 동기화한 labeled demo |
| scale | 1M+ hours, PB, 43M+ episodes, multi-cluster self-report | canonical 5,613 Stage-1 / 1,692 Stage-2 usable LIBERO episodes | small pilot에서 quality/coverage를 먼저 증명; scale infra는 metric-triggered |
| model | Dyna-2 proprietary WAM context | 공개 0.47B MoT + flow matching | SLIM을 baseline 후보로 fork/adapter; DYNA model 의존 금지 |
| evaluation | infra throughput/startup/utilization | LIBERO, LIBERO-Plus, CALVIN, 5 real tasks | bimanual success + partial progress + collision/limit/inter-arm clearance + OOD split |
| reproducibility | article-level technical narrative, implementation artifact 미연결 | commit/config/hash/eval protocol 공개 | SLIM 방식을 채택하되 pinned revision과 own manifest를 필수화 |

위 table의 DYNA·SLIM 열은 각각 [DYNA](https://www.dyna.co/research/dyna-2-infrastructure), [SLIM reproducibility record](https://github.com/kzz1031/SLIM/blob/main/reproducibility/canonical_recipe.json)에 근거한다. 마지막 열은 **추론/권고**다.

---

## 7. 이중팔 적용 로드맵

### Phase 0 — 먼저 고정할 episode contract (P0)

**[권고]** action 하나를 단순히 `left 7D + right 7D`로 붙이기 전에, 다음을 명시한다.

- `timestamp_ns`와 monotonic episode clock; camera/state/action interpolation 규칙
- `observation.images.{overhead,left_wrist,right_wrist}`와 각 intrinsics/extrinsics/calibration version
- `observation.state`: 좌·우 joint position/velocity, gripper, 가능하면 end-effector pose 및 contact/force validity
- `action`: 같은 frame에서의 좌·우 relative end-effector delta 또는 joint target, 좌·우 gripper command, command horizon 및 controller rate
- `frame_id`: base/table/object/tool frame 및 action의 reference frame
- `episode_manifest`: task/language, operator/session, robot/controller/firmware, camera health, success/failure/abort reason, source split, schema/pipeline version

이는 DYNA의 synchronized multi-modal sample·version stamp와 SLIM의 action/state normalization requirements를 결합한 **설계 권고**다. [근거: DYNA](https://www.dyna.co/research/dyna-2-infrastructure), [SLIM data docs](https://github.com/kzz1031/SLIM/blob/main/docs/data.md)

**수용 기준(권고):** recorder가 (a) missing/duplicate timestamps, (b) left/right action shape, (c) calibration mismatch, (d) controller stop/abort, (e) normalization-stat outlier를 episode ingestion에서 검출하고, split manifest를 재생성 가능하게 만든다.

### Phase 1 — SLIM-compatible bimanual pilot (P0)

1. **schema adapter:** public YAML의 `action_dim`/`state_dim`을 실제 vector 차원으로 바꾸고, dataset reader·action statistics·server request·evaluation을 같은 schema version에서 바꾼다. source config가 차원을 받는다는 것은 **사실**이지만, 모든 adapter 변경은 **권고**다. [근거: config helper](https://github.com/kzz1031/SLIM/blob/main/slim/model/config.py)
2. **관측:** initially overhead + two wrist view를 후보로 두되, image view 수/patch-token budget과 latency를 measurement로 결정한다. SLIM canonical two-view 성능이 third view의 효용을 입증하지는 않는다. [사실: SLIM docs](https://github.com/kzz1031/SLIM/blob/main/docs/data.md), 결론은 **추론**이다.
3. **학습:** Stage 1 IDM+FDM → Stage 2 policy라는 두 단계와 EMA ablation을 유지한 small-data comparison을 만든다. SLIM ablation은 Stage 1/EMA의 가치가 그 benchmark에 있음을 보이지만, 이중팔 최적값은 새로 측정해야 한다. [사실: 논문 §4.3](https://arxiv.org/html/2608.09771)
4. **실행:** action chunk 전체를 blind open-loop로 소모하지 않고, 짧은 receding horizon 뒤 re-observe한다. SLIM은 chunk policy이므로 이 방식은 자연스러운 **추론/권고**이며, chunk length/inference steps는 latency와 safety margin으로 sweep한다. [근거: SLIM Appendix A.5](https://arxiv.org/html/2608.09771)
5. **safety:** learned action 이전에 independent workspace, joint-limit, velocity/acceleration, inter-arm collision, gripper/contact interlock을 check하고 violation은 deterministic stop으로 기록한다. 이는 필수 **권고**이며, SLIM 공개 자료의 성능 수치가 safety certification은 아니다. [근거 범위: SLIM 논문](https://arxiv.org/html/2608.09771)

### Phase 2 — evaluation을 dual-arm 답게 만들기 (P1)

**[권고]** “task success” 하나로 끝내지 말고 다음을 함께 보고한다.

| 평가 묶음 | 최소 지표 |
|---|---|
| nominal | task success, progress milestone, completion time, retry/replan count |
| coordination | 양손 동시/순차 phase 정확도, handoff success, object drop, inter-arm minimum clearance |
| robustness | camera pose, lighting, background, distractor, object pose, latency/network jitter, gripper variation의 held-out split |
| safety | joint/workspace/collision guard activation, emergency stop, contact limit exceed, aborted episode reason |
| data quality | timestamp skew, missing stream rate, calibration-version mix, failed quality gate 비율 |

SLIM의 LIBERO-Plus와 real-world protocol은 visual OOD와 partial progress를 분리해 측정하는 출발점이고, DYNA는 dataset quality/version query를 강화하는 출발점이다. 그러나 inter-arm clearance·safety 지표는 두 페이지가 제공한 결과가 아니라 이중팔에 필요한 **추론/권고**다. [근거: SLIM](https://arxiv.org/html/2608.09771), [DYNA](https://www.dyna.co/research/dyna-2-infrastructure)

### Phase 3 — DYNA식 scale trigger (P2)

다음 중 하나가 측정되기 전에는 warehouse/Alluxio/multi-node optimizer를 만들지 않는 것을 권한다.

- epoch start/manifest build가 연구자 iteration의 지배적 대기 시간이다.
- training GPU가 input I/O로 자주 idle이고 local profiling이 storage round-trip을 원인으로 보인다.
- re-label/reprocess가 잦아 episode provenance query가 수작업으로는 불가능하다.
- node 수 증가 뒤 optimizer collective가 step time의 지배 항목이다.

이는 DYNA의 “규모마다 병목이 storage→ingestion→manifest→training으로 이동한다”는 설명을 operational gate로 바꾼 **추론/권고**다. [근거: DYNA](https://www.dyna.co/research/dyna-2-infrastructure)

---

## 8. DAPIER 현재 장비·6주 일정에 대한 구체 결정

현재 요구사항 원장의 확정 조건은 JDcobot300 양팔, TurtleBot3 Waffle Pi, Orbbec Astra Pro 1대, RTX 5050 Laptop GPU, 추가 예산 0원, 4명·6주 MVP다. 이 조건에서는 다음처럼 적용 범위를 줄이는 것이 타당하다. 하드웨어·일정 정보는 [DAPIER 요구사항 원장](../project-planning/mobile_dual_arm_shoe_sorting_ledger.md), 기술 판단은 앞 절의 DYNA/SLIM 근거를 결합한 **권고**다.

### 8-1. 채택할 최소 아키텍처

```text
TurtleBot3 Nav2/SLAM ──> 작업 위치 정렬 후 base 정지
                                 │
Astra Pro RGB-D ──> 신발 검출·짝 추론·3D 목표 ──> skill/language condition
                                 │
좌·우 joint/gripper state ──> ACT baseline / 축소 SLIM 실험 ──> action chunk
                                 │
                                 └──> deterministic safety supervisor
                                      ├─ joint/workspace limit
                                      ├─ inter-arm collision/clearance
                                      ├─ stale observation/action expiry
                                      └─ stop/abort + failure receipt
```

- **이동과 조작을 첫 MVP에서 분리한다.** Nav2가 작업 위치에 도달하고 base가 정지한 뒤에만 양팔 policy를 활성화한다. 이동 base와 두 팔을 한 action chunk로 처음부터 공동 학습하면 action dimension, 안전 검증, 데이터 요구량이 동시에 증가한다.
- **신발 짝 추론과 관절 제어를 분리한다.** VLM/시각 임베딩은 `pair_id`, 좌우, 목표 슬롯, 신뢰도와 허용 skill만 만들고, joint action은 ACT 또는 SLIM 계열 policy가 생성한다. LLM/VLM 출력은 safety supervisor를 우회하지 않는다.
- **Astra Pro 한 대를 정본 workspace view로 쓴다.** SLIM의 공개 결과는 workspace+wrist 두 RGB view이므로, 같은 Astra 영상을 단순 복제해 “SLIM 정본 재현”이라고 주장하지 않는다. depth는 3D 목표·바닥/신발장 평면·충돌 여유를 계산하는 deterministic perception 보조로 쓰고, SLIM 입력에 넣으려면 별도 RGB-D encoder 실험으로 표시한다.
- **양팔 action 차원은 실기체 introspection 뒤 고정한다.** JDcobot300이 팔당 6관절이고 별도 gripper command가 있다면 예시는 `left 6 + left gripper 1 + right 6 + right gripper 1 = 14D`지만, 실제 joint name/order·unit·gripper 장착 여부가 확인되기 전에는 14D를 정본으로 확정하지 않는다.

### 8-2. DYNA에서 즉시 가져올 것과 보류할 것

| 시점 | 채택 | 구현 수준 |
|---|---|---|
| 즉시 | 동기 episode contract, quality gate, schema/pipeline/calibration/software version, immutable manifest, 실패 사유, 재실행 가능 단계 | raw 수집은 ROS 2 `rosbag2` MCAP 후보, 학습용은 LeRobot dataset으로 변환; episode마다 JSON receipt와 checksum |
| 즉시 | curation query | 별도 warehouse 대신 local Parquet + DuckDB/Polars로 `task`, `shoe_id`, `success`, `camera_health`, `calibration_version`, `operator`, `split`을 질의 |
| 측정 후 | video compression/chunk tuning | loader p95와 GPU idle이 실제 병목일 때만 H.264/GOP·MCAP read benchmark 수행 |
| 보류 | Airflow, Kubernetes, Alluxio, Slurm, Ansible, multi-node optimizer sharding | 단일 노트북·6주·무예산 범위에는 운영 비용이 이득보다 크다 |

### 8-3. SLIM 적용 수준

**권장 1순위는 ACT 기준선 유지 + SLIM의 Stage-1 아이디어를 축소 ablation으로 추가**하는 것이다.

1. ACT baseline을 동일 dataset/split/evaluator로 먼저 닫는다.
2. 같은 observation encoder에 `현재 latent + 미래 latent → action chunk` IDM과 `현재 latent + action chunk → 미래 latent` FDM 보조 loss를 추가한 작은 모델을 만든다.
3. `ACT`, `ACT+IDM`, `ACT+FDM`, `ACT+IDM+FDM+EMA`를 같은 seed로 비교한다.
4. 개선이 확인된 뒤에만 공개 SLIM MoT 전체 구조 또는 flow-matching policy로 확장한다.

그 이유는 SLIM 정본 학습이 8×H100 80GB를 사용했고, 공개 inference 수치도 H100 기준이기 때문이다. 논문의 4.26GiB incremental VRAM은 model loading, language encoding, simulator/RPC를 제외한 값이므로 RTX 5050 8GB에서 전체 server가 동작한다는 보장이 아니다. 공개 checkpoint는 7D single-arm 계약이어서 양팔 12–14D output head에 그대로 사용할 수도 없다. 따라서 먼저 BF16 batch-1 checkpoint smoke와 p50/p95 latency·peak VRAM·OOM 여부만 격리 측정하고, full fine-tuning은 성공을 전제하지 않는다. [근거: SLIM Appendix A.4–A.5](https://arxiv.org/html/2608.09771)

### 8-4. 6주 실행 순서와 중단 기준

| 주차 | 산출물 | 다음 단계 진입 조건 |
|---|---|---|
| 1주 | joint/camera/time/calibration contract, base-stop interlock, episode recorder, quality checker | 좌·우 state/action order·unit·timestamp mismatch 0건인 replay 20회 |
| 2주 | 신발 1개 집기·운반·놓기 skill, raw→LeRobot 변환, DuckDB manifest | 성공 episode와 실패/abort episode가 같은 schema로 재로딩됨 |
| 3주 | 단일팔 ACT baseline과 독립 evaluator | held-out 초기 자세에서 성공률·지연·실패 taxonomy 보고 가능 |
| 4주 | 양팔 role-split baseline과 ACT+IDM/FDM/EMA offline ablation | validation loss뿐 아니라 held-out closed-loop progress가 ACT보다 개선될 때만 확대 |
| 5주 | 조명·배경·distractor·신규 신발·camera pose·latency jitter OOD 평가 | collision/limit violation 0, 모든 실패가 receipt에 분류됨 |
| 6주 | 바닥 격자 end-to-end 시연, ACT 대 축소-SLIM 비교표, 재현 manifest | 5회 이상 연속 시연과 seed별 원시 결과 공개 가능 |

**중단 기준:** (a) single-arm baseline이 안정화되지 않음, (b) 양팔 최소 clearance guard가 검증되지 않음, (c) GPU OOM 또는 policy p95가 제어 주기 예산을 넘음, (d) dataset timestamp/calibration 오류가 남아 있음 중 하나면 full SLIM·신발장·이동 중 조작으로 확장하지 않는다.

### 8-5. DAPIER용 핵심 평가표

| 계층 | 최소 지표 |
|---|---|
| 데이터 | episode 수·유효 frame 수, camera/state/action timestamp skew p95/max, dropped/stale frame, calibration/schema 혼합률, ingestion reject 사유 |
| 정책 | 집기·운반·짝맞춤·배치 성공률, 0/0.5/1 progress, 미학습 신발 성능, action p50/p95 latency, VRAM, retry/replan 횟수 |
| 양팔 | 동시/순차 phase 정확도, object drop, 양팔 minimum clearance, 역할 교대·handoff 성공률(도입 시) |
| 시스템 | time-to-first-batch, loader throughput, GPU utilization, checkpoint resume 성공, attempt별 failure receipt |
| 안전 | joint/workspace/inter-arm guard activation, action expiry, E-stop/abort, 사람 개입 횟수 |

이 평가표에서 DYNA는 데이터·시스템 지표의 근거, SLIM은 OOD·progress·latency ablation의 근거를 제공한다. 신발 짝추론 정확도와 Nav2 도킹 오차는 두 자료가 다루지 않으므로 DAPIER가 별도 지표로 추가해야 한다.

---

## 9. 제한·미해결 사항

1. **DYNA 재현성:** 이 페이지는 매우 구체적인 내부 benchmark와 시스템 수치를 제시하지만 Dyna-2 training data, MCAP writer 설정, DAG/warehouse/cache code, exact hardware topology를 공개 artifact로 링크하지 않는다. 따라서 수치는 design evidence이지 external reproduction evidence가 아니다. [사실: DYNA 페이지/References](https://www.dyna.co/research/dyna-2-infrastructure#references)
2. **SLIM의 embodiment gap:** 검토한 SLIM paper/README/config에는 `dual-arm` 문자열이 없고 real-world section은 five tasks/demos/progress protocol을 기술하지만 bimanual architecture benchmark를 보고하지 않는다. dual-arm transfer는 미검증이다. [사실: 논문](https://arxiv.org/html/2608.09771), [README](https://github.com/kzz1031/SLIM/blob/main/README.md)
3. **행동 표현 선택:** 14D joint target, dual end-effector delta, hybrid pose+gripper, force/impedance action 중 무엇이 더 안정적인지는 두 자료에서 답하지 않는다. 이는 robot controller·calibration·task contact에 따라 별도 ablation할 미해결 사항이다.
4. **human-video 활용:** DYNA는 hand-pose 기반 pseudo-action을 주장하지만 pipeline/data는 공개되지 않았고, SLIM Stage 1은 action-labeled trajectory를 전제한다. 둘을 연결하려면 label uncertainty·quality filtering·embodiment mapping을 새로 검증해야 한다. [사실: Dyna-2](https://www.dyna.co/dyna-2), [SLIM §3.3](https://arxiv.org/html/2608.09771)
5. **latency 수치 비교:** SLIM page의 end-to-end 77.3ms/2.01GiB와 paper의 controlled H100 model-only 60.6ms/4.26GiB는 같은 protocol이 아니다. hardware/control loop에 대한 SLA를 이 숫자들에서 직접 역산하면 안 된다. [사실: 프로젝트 페이지](https://kzz1031.github.io/slim-project-page/), [논문 Appendix A.5](https://arxiv.org/html/2608.09771)
6. **media 접근 한계:** DYNA MP4의 stream format/duration, 2초 간격 15 frame contact sheet와 page figure captions는 확인했으나 891개 전체 frame을 pixel-by-pixel 의미 분류하지는 않았다. SLIM project page는 video가 없고 정적 image assets 7개를 원본 해상도로 확인했다.

---

## 10. 1차 출처 목록

1. [DYNA, Training Dyna-2 at million-hour scale, repeatably](https://www.dyna.co/research/dyna-2-infrastructure) — 본 조사에서의 인프라 주장, Figure 1–11, References의 원문.
2. [DYNA, Dyna-2: A 1-Million-Hour Scaling Law for World-Action Models](https://www.dyna.co/dyna-2) — WAM 문맥과 DYNA가 주장하는 bimanual evaluation의 범위.
3. [Wang et al., SLIM-0.5B arXiv HTML, v1](https://arxiv.org/html/2608.09771) — 방법, table, ablation, real-world protocol, inference protocol.
4. [SLIM project page](https://kzz1031.github.io/slim-project-page/) — page-level figure/media inventory와 표시된 headline metrics.
5. [SLIM official repository](https://github.com/kzz1031/SLIM) — 공개 구현의 범위.
6. [SLIM reproducibility record](https://github.com/kzz1031/SLIM/blob/main/reproducibility/README.md) 및 [machine-readable canonical recipe](https://github.com/kzz1031/SLIM/blob/main/reproducibility/canonical_recipe.json) — pinned revisions, data, hardware, evaluation counts.
7. [SLIM data/model asset docs](https://github.com/kzz1031/SLIM/blob/main/docs/data.md), [public config helper](https://github.com/kzz1031/SLIM/blob/main/slim/model/config.py), [canonical Stage-2 config](https://github.com/kzz1031/SLIM/blob/main/configs/libero/stage2_policy_h8_40ep.yaml) — concrete dimensions and data assumptions.
