# 최신 연구·공식 정본 → 2ARM_ROBOT 반영 원장

> 목적: 각 단계에서 무엇을 읽었고, 어떤 주장만 채택했으며, 코드·테스트에 어떻게 반영했는지 추적한다.
> 원칙: 논문 성능 수치를 그대로 기대효과로 주장하지 않는다. 장비·데이터·평가 조건이 다른 내용은 실험 후보 또는 보류로 분리한다.

## 연구 채택 gate

아래 질문을 모두 통과하지 못한 자료는 production 코드나 baseline에 넣지 않는다.

1. **현재 결정과 직접 관련되는가?** 지금 단계의 실패 위험, interface, 평가 질문 중 하나에 답해야 한다.
2. **1차 근거인가?** 논문 원문, 공식 코드, 공식 문서, 표준만 채택 근거로 쓴다. 블로그·요약 영상은 탐색용일 뿐이다.
3. **조건 차이를 설명했는가?** robot embodiment, camera view, action representation, dataset 규모, 평가 환경 차이를 적는다.
4. **우리 자원에서 검증 가능한가?** 6주, 4명, RTX 5050, 추가 예산 0원 안에서 작은 재현 또는 ablation이 가능해야 한다.
5. **측정 가능한 가설인가?** ACT baseline과 같은 split·task·예산에서 success, error, latency, intervention 같은 사전 지표가 있어야 한다.
6. **되돌릴 수 있는가?** 실패해도 ACT baseline, raw dataset, safety gate를 깨지 않고 제거할 수 있어야 한다.

판정:

- `즉시 반영`: 위 gate를 통과하고 모델 성능과 무관하게 데이터 신뢰도·안전·재현성을 높이는 변경
- `실험 후보`: baseline 완료 후 동일 조건 ablation으로만 판단할 변경
- `참고만`: 개념 이해에는 유용하지만 현재 설계 결정에 직접 쓰지 않는 자료
- `보류`: 자원·장비·재현 조건이 맞지 않거나 위험 대비 이득을 측정할 수 없는 변경

금지:

- 논문이 최신/SOTA라는 이유만으로 dependency나 모델을 추가
- 다른 embodiment의 성공률을 DAPIER 예상 성능으로 사용
- 8×H100 같은 학습 recipe를 RTX 5050 실행 계획으로 표현
- baseline·평가 지표 없이 여러 보조목표를 동시에 도입
- 코드 공개·dataset schema·재현 조건을 확인하지 않은 구조를 필수 사양으로 채택

## 기록 필드

| 필드 | 의미 |
|---|---|
| 자료 | 논문·공식 문서·공식 코드 이름 |
| 저자/기관·연도 | 정본 소유자와 공개 연도 |
| 확인 내용 | 원문에서 실제 확인한 구조·API·운영 지침 |
| DAPIER 결정 | 즉시 반영 / 실험 후보 / 보류 |
| 코드·테스트 증거 | 반영 결과를 검증할 로컬 artifact |
| 확인일 | URL과 내용을 확인한 날짜 |

## Stage 1 · Astra RGB/Depth payload 계약

상세 조사: [`research/LATEST_RGBD_DATA_CONTRACT_RESEARCH_20260821.md`](research/LATEST_RGBD_DATA_CONTRACT_RESEARCH_20260821.md)

| 자료 | 저자/기관·연도 | 원문 | 확인 내용 | DAPIER 결정 | 코드·테스트 증거 | 확인일 |
|---|---|---|---|---|---|---|
| LeRobot Dataset v3 공식 문서 | Hugging Face, 2025–2026 | [원문](https://github.com/huggingface/lerobot/blob/main/docs/source/lerobot-dataset-v3.mdx) | numeric/visual stream, metadata, stats, episode boundary와 finalize 책임 분리 | **즉시 반영**: raw/derived 분리와 finalized gate | `camera_payload.py`, `contract.py`, `test_camera_payload.py` | 2026-08-21 |
| LeRobot depth video encoding 공식 문서 | Hugging Face, 2025–2026 | [원문](https://github.com/huggingface/lerobot/blob/main/docs/source/video_encoding_parameters.mdx) | depth unit과 quantizer metadata 없이 codec 저장 시 물리량 복원 불가 | **즉시 반영**: 16UC1/32FC1 raw 보존. **실험 후보**: native depth video | `ASTRA_RGBD_PAYLOAD_CONTRACT.md` | 2026-08-21 |
| LeRobot dataset metadata 코드 | Hugging Face, 2026 | [원문](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/dataset_metadata.py) | depth feature의 `is_depth_map`, `depth_unit` metadata 처리 | **실물 연결 시 반영**: depth unit 검증 후 native feature metadata 생성 | Stage 2 encoder 예정 | 2026-08-21 |
| ROS2 message_filters API | Open Robotics, 2026 문서 | [원문](https://docs.ros.org/en/ros2_packages/rolling/api/message_filters/message_filters.html) | ApproximateTime은 message header timestamp를 기준으로 매칭하며 arrival-time 대체는 위험 | **즉시 반영**: header와 receive monotonic을 분리하고 sync Δt 저장 | `recorder.py`, `quality.py`, sync tamper test | 2026-08-21 |
| DROID | Khazatsky et al., 2024 | [논문](https://arxiv.org/abs/2403.12945), [공식 프로젝트](https://droid-dataset.github.io/) | real-robot dataset이 camera calibration/provenance를 별도 제공하고 revision을 관리 | **실물 연결 시 반영**: immutable calibration snapshot과 camera serial | Stage 1 현장 gate, Stage 5 예정 | 2026-08-21 |
| DROID trajectory schema/policy learning | DROID 공식 코드, 2024–2025 | [schema](https://github.com/droid-dataset/droid/blob/main/droid/postprocessing/schema.py), [policy README](https://github.com/droid-dataset/droid_policy_learning) | camera serial/extrinsics metadata와 target-domain demonstration co-training | **즉시 반영**: provenance 유지. **실험 후보**: target demo 보강 | `contract.py`, split leakage gate | 2026-08-21 |
| Orbbec Astra ROS2 driver | Orbbec, 공식 저장소 | [원문](https://github.com/orbbec/ros2_astra_camera/blob/master/README.MD) | CameraInfo, calibration file, depth registration, color-depth sync/extrinsic launch option | **실물 연결 시 반영**: launch parameter와 CameraInfo snapshot | `ASTRA_RGBD_PAYLOAD_CONTRACT.md`의 미완료 gate | 2026-08-21 |
| Astra Pro CameraInfo NaN 공개 사례 | OrbbecSDK_ROS2 issue #134 | [원문](https://github.com/orbbec/OrbbecSDK_ROS2/issues/134) | Astra Pro 계열에서 invalid CameraInfo가 발생할 수 있는 현장 위험 | **실물 연결 시 즉시 차단**: K/D/R/P finite·dimension 검사 | CameraInfo 실물 test 예정 | 2026-08-21 |
| Robots Pre-train Robots / MCR | Jiang et al., 2024 | [논문](https://arxiv.org/abs/2410.22325) | vision과 proprioception/action을 함께 쓰는 representation 학습 가능성 | **실험 후보**: ACT baseline 뒤 보조목표. 성능 보장 주장 금지 | 주 4 이후 go/no-go ablation | 2026-08-21 |

## Stage 2 · native LeRobot Dataset v3 encoder

상세 조사: [`research/LATEST_LEROBOT_V3_ENCODER_RESEARCH_20260821.md`](research/LATEST_LEROBOT_V3_ENCODER_RESEARCH_20260821.md)

| 자료 | 저자/기관·연도 | 원문 | 확인 내용 | DAPIER 결정 | 코드·테스트 증거 | 확인일 |
|---|---|---|---|---|---|---|
| LeRobot Dataset v3 공식 문서 | Hugging Face, 2025–2026 | [원문](https://github.com/huggingface/lerobot/blob/main/docs/source/lerobot-dataset-v3.mdx) | v3 layout과 create/add/save/finalize lifecycle | **즉시 반영** | `lerobot_v3_encoder.py`, native preflight tests | 2026-08-21 |
| LeRobotDataset/Writer 공식 코드 | Hugging Face, 2026 | [Dataset](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/lerobot_dataset.py), [Writer](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/dataset_writer.py) | writer가 timestamp/frame index 생성, add_frame feature 검증, finalize 필수 | **즉시 반영** | native frame schema와 writer 호출 순서 | 2026-08-21 |
| LeRobot Dataset 공식 tests | Hugging Face, 2026 | [원문](https://github.com/huggingface/lerobot/blob/main/tests/datasets/test_datasets.py) | missing/extra/type/shape와 create→reopen 검증 관행 | **즉시 반영**: Stage 3 release gate | `test_lerobot_v3_encoder.py`, Stage 3 예정 | 2026-08-21 |
| LeRobot depth encoding/config | Hugging Face, 2026 | [문서](https://github.com/huggingface/lerobot/blob/main/docs/source/video_encoding_parameters.mdx), [코드](https://github.com/huggingface/lerobot/blob/main/src/lerobot/configs/video.py) | depth marker/unit과 12-bit quantizer metadata 필요 | **즉시 반영**: explicit depth unit. **실험 후보**: depth video | `native-preflight --depth-unit` | 2026-08-21 |
| LeRobot installation/extras | Hugging Face, 2026 | [원문](https://github.com/huggingface/lerobot/blob/main/docs/source/installation.mdx) | 기능별 extras와 optional package 경계 | **즉시 반영**: lazy import, base dependency 유지 | `native-status`, dependency failure test | 2026-08-21 |
| Robo-DM | Chen et al., 2025 | [논문](https://arxiv.org/abs/2505.15558), [코드](https://github.com/BerkeleyAutomation/robodm) | heterogeneous stream alignment와 decode/storage 병목 개선 | **참고만/보류**: 현재 측정된 병목 없음 | ACT baseline 후 storage 병목 발생 시만 재검토 | 2026-08-21 |

## Stage 3 · Dataset v3 round-trip와 ACT dataloader

상세 조사: [`research/LATEST_ACT_DATALOADER_ROUNDTRIP_RESEARCH_20260821.md`](research/LATEST_ACT_DATALOADER_ROUNDTRIP_RESEARCH_20260821.md)
구현·검증: [`ACT_DATASET_ROUNDTRIP_SMOKE.md`](ACT_DATASET_ROUNDTRIP_SMOKE.md)

| 자료 | 저자/기관·연도 | 원문 | 확인 내용 | DAPIER 결정 | 코드·테스트 증거 | 확인일 |
|---|---|---|---|---|---|---|
| LeRobot Dataset v3 공식 문서·Dataset 구현 | Hugging Face, 2025–2026 | [문서](https://github.com/huggingface/lerobot/blob/main/docs/source/lerobot-dataset-v3.mdx), [코드](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/lerobot_dataset.py) | finalize/reopen, delta window와 episode padding | **즉시 반영** | `native_act_smoke.py`, 2ep×3frame receipt | 2026-08-21 |
| LeRobot dataset factory | Hugging Face, 2026 | [원문](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/factory.py) | policy delta index를 `i/fps` seconds로 변환 | **즉시 반영** | 20 FPS → `[0,0.05,0.1]` exact assertion | 2026-08-21 |
| LeRobot ACTConfig·ACT 모델 | Hugging Face, 2026 | [config](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/configuration_act.py), [model](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py) | chunk size, n_action_steps, `action_is_pad` loss mask | **즉시 반영** | DataLoader mask shape와 ACT finite-loss forward | 2026-08-21 |
| LeRobot dataset/streaming 공식 tests | Hugging Face, 2026 | [dataset tests](https://github.com/huggingface/lerobot/blob/main/tests/datasets/test_datasets.py), [streaming tests](https://github.com/huggingface/lerobot/blob/main/tests/datasets/test_streaming.py) | tail padding mask와 non-padded consistency 검사 | **즉시 반영** | `[FFF, FFT, FTT]` × 2, cross-episode no-leak | 2026-08-21 |
| ACT 원 논문 | Zhao et al., 2023 | [원문](https://arxiv.org/abs/2304.13705) | action chunking·temporal ensemble의 원 구조 | **참고만**: 현재 API 판정은 공식 LeRobot 코드 사용 | Stage 4 chunk ablation 배경 | 2026-08-21 |
| Robo-DM | Chen et al., 2025 | [원문](https://arxiv.org/abs/2505.15558) | multimodal robot data storage/decode trade-off | **참고만/보류**: 측정된 병목 없음 | container·streaming 미도입 | 2026-08-21 |

## Stage 4 · offline evaluator와 action chunk/padding

상세 조사: [`research/LATEST_OFFLINE_EVALUATOR_CHUNK_RESEARCH_20260821.md`](research/LATEST_OFFLINE_EVALUATOR_CHUNK_RESEARCH_20260821.md)
구현·검증: [`OFFLINE_EVALUATOR_ACTION_CHUNK.md`](OFFLINE_EVALUATOR_ACTION_CHUNK.md)

| 자료 | 저자/기관·연도 | 원문 | 확인 내용 | DAPIER 결정 | 코드·테스트 증거 | 확인일 |
|---|---|---|---|---|---|---|
| LeRobot ACTConfig·ACT model | Hugging Face, 2026 | [config](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/configuration_act.py), [model](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py) | chunk horizon, `action_is_pad` valid-mask loss | **즉시 반영** | `offline_evaluator.py`, padding mutation tests | 2026-08-21 |
| LeRobot dataset factory·streaming | Hugging Face, 2026 | [factory](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/factory.py), [tests](https://github.com/huggingface/lerobot/blob/main/tests/datasets/test_streaming.py) | FPS delta, episode boundary, mask/non-padded consistency | **즉시 반영** | horizon coverage, cross-episode target-ID failure test | 2026-08-21 |
| LeRobot train/eval scripts | Hugging Face, 2026 | [train](https://github.com/huggingface/lerobot/blob/main/src/lerobot/scripts/lerobot_train.py), [eval](https://github.com/huggingface/lerobot/blob/main/src/lerobot/scripts/lerobot_eval.py) | offline eval loss와 rollout success/reward는 별도 증거 | **즉시 반영** | report의 `closed_loop=NOT_MEASURED` | 2026-08-21 |
| PiL-World | Dong et al., 2026 | [원문](https://arxiv.org/abs/2606.05773) | action chunk와 policy-in-the-loop 평가의 최신 연구 | **참고만/보류**: compute·모델 조건 불일치 | world-model evaluator 미도입 | 2026-08-21 |
| Robo-DM | Chen et al., 2025 | [원문](https://arxiv.org/abs/2505.15558) | multimodal data provenance/storage 참고 | **참고만**: ACT metric 근거 아님 | input/records SHA provenance만 유지 | 2026-08-21 |

## Stage 5 · JDcobot rollout와 독립 safety supervisor

상세 조사: [`research/LATEST_ROLLOUT_SAFETY_SUPERVISOR_RESEARCH_20260821.md`](research/LATEST_ROLLOUT_SAFETY_SUPERVISOR_RESEARCH_20260821.md)
구현·검증: [`JDCOBOT_ROLLOUT_SAFETY_SUPERVISOR.md`](JDCOBOT_ROLLOUT_SAFETY_SUPERVISOR.md)

| 자료 | 저자/기관·연도 | 원문 | 확인 내용 | DAPIER 결정 | 코드·테스트 증거 | 확인일 |
|---|---|---|---|---|---|---|
| ROS2 Lifecycle | Open Robotics, 공식 | [원문](https://docs.ros.org/en/rolling/p/lifecycle/) | managed node state와 transition | **즉시 반영**: application `FAULT_LATCHED` 병행 | `SafetySupervisor` lifecycle tests | 2026-08-21 |
| ROS2 QoS·SensorDataQoS·deadline/liveliness | Open Robotics, 공식 | [QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html), [design](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html) | driver QoS discovery 필요, DDS signal과 end-to-end freshness 차이 | **현장 반영**: compatible QoS + monotonic watchdog | observation/feedback/proposal age gates | 2026-08-21 |
| LeRobot ACT·rollout 공식 코드/tests | Hugging Face, 2026 | [ACT](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py), [rollout](https://github.com/huggingface/lerobot/blob/main/src/lerobot/rollout/strategies/core.py) | reset/action queue/episode 경계 | **즉시 반영**: reset generation·stale queue reject | fault latch/rearm tests | 2026-08-21 |
| TurtleBot3 ROS2 공식 source | ROBOTIS, 공식 | [원문](https://github.com/ROBOTIS-GIT/turtlebot3) | `cmd_vel`/`odom` 이동 경계 | **즉시 반영**: odom+recent command stationary gate | base motion mutation | 2026-08-21 |
| XM430-W210-T control table | ROBOTIS, 공식 | [원문](https://emanual.robotis.com/docs/kr/dxl/x/xm430-w210/) | device-side position/velocity/current limit | **참고만**: mobile base motor이며 arm limit 근거 아님 | JDcobot limit 미가정 | 2026-08-21 |
| PiL-World·SafeMIL·ROS2 QoS analysis | 2025–2026 | [PiL-World](https://arxiv.org/abs/2606.05773), [SafeMIL](https://arxiv.org/abs/2511.08136), [QoS](https://arxiv.org/abs/2509.03381) | chunk closed-loop/safe imitation/QoS 연구 | **reference-only/보류** | 새 model/formal framework 미도입 | 2026-08-21 |

## Stage 6 · LeRobot 완전 독립 DAPIER-native ACT runtime

상세 조사: [`research/LATEST_DAPIER_NATIVE_ACT_RESEARCH_20260824.md`](research/LATEST_DAPIER_NATIVE_ACT_RESEARCH_20260824.md)
구현·검증: [`DAPIER_NATIVE_ACT_RUNTIME.md`](DAPIER_NATIVE_ACT_RUNTIME.md)

| 자료 | 저자/기관·연도 | 원문 | 확인 내용 | DAPIER 결정 | 코드·테스트 증거 | 확인일 |
|---|---|---|---|---|---|---|
| ACT 원 논문 | Zhao et al., 2023 | [논문](https://arxiv.org/abs/2304.13705), [RSS 원문](https://roboticsproceedings.org/rss19/p016.pdf) | action chunk, CVAE, L1+KL, temporal ensemble의 원 구조 | **즉시 반영**: chunk/CVAE/masked loss 원리. **실험 후보**: temporal ensemble | `dapier_native_act.py`, independent smoke | 2026-08-24 |
| ACT 공식 repository | Zhao et al., 2023– | [코드](https://github.com/tonyzhaozh/act), [dataset](https://github.com/tonyzhaozh/act/blob/main/utils.py), [policy](https://github.com/tonyzhaozh/act/blob/main/policy.py) | future action padding, qpos/action normalization, CVAE loss, rollout query 방식 | **즉시 반영**: train-only stats와 episode-tail mask. **보류**: ALOHA `start_ts-1` alignment hack | dataset/mask/checkpoint tests | 2026-08-24 |
| LeRobot ACT 공식 코드 | Hugging Face, 2026 | [config](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/configuration_act.py), [model](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py) | valid-count masked L1, chunk queue/reset, `n_action_steps` constraint | **비교 정본만**: runtime dependency 제거. **즉시 반영**: valid-only loss·stale reset | AST no-LeRobot import, queue reset test | 2026-08-24 |
| LeRobot relative-action tests | Hugging Face, 2026 | [tests](https://github.com/huggingface/lerobot/blob/main/tests/policies/test_relative_actions.py) | episode-tail valid mask와 action representation round-trip 검사 관행 | **참고만**: native failure class에 맞는 test만 재작성 | `[FFF,FFT,FTT]` × 2 | 2026-08-24 |

Stage 6은 “LeRobot/ALOHA checkpoint 호환”을 주장하지 않는다. 외부 checkpoint load, delta-action 기본화,
temporal ensemble, VLA/world model은 동일 split·예산의 측정 가능한 go/no-go 실험 전까지 보류한다.

## Stage 7 · Native ACT proposal와 독립 supervisor 통합

상세 조사: [`research/LATEST_NATIVE_ACT_SUPERVISOR_INTEGRATION_RESEARCH_20260824.md`](research/LATEST_NATIVE_ACT_SUPERVISOR_INTEGRATION_RESEARCH_20260824.md)
구현·검증: [`NATIVE_ACT_SUPERVISOR_INTEGRATION.md`](NATIVE_ACT_SUPERVISOR_INTEGRATION.md)

| 자료 | 저자/기관·연도 | 원문 | 확인 내용 | DAPIER 결정 | 코드·테스트 증거 | 확인일 |
|---|---|---|---|---|---|---|
| ACT 원 논문·공식 rollout | Zhao et al., 2023 | [논문](https://roboticsproceedings.org/rss19/p016.pdf), [코드](https://github.com/tonyzhaozh/act/blob/main/imitate_episodes.py) | chunk prediction과 temporal aggregation은 motor safety 권한을 제공하지 않음 | **즉시 반영**: chunk를 proposal 출처로만 취급. **실험 후보**: aggregation | `native_act_rollout.py`, action[0] exact test | 2026-08-24 |
| LeRobot ACTConfig·model | Hugging Face, 2026 | [config](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/configuration_act.py), [model](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py) | queue/reset과 `n_action_steps` 의미 | **비교 정본만**: LeRobot import 없이 `n_action_steps=1` native baseline | reset/checkpoint/source identity trace | 2026-08-24 |
| ROS2 lifecycle·QoS/deadline/liveliness | Open Robotics, 공식 | [lifecycle](https://design.ros2.org/articles/node_lifecycle.html), [QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html), [design](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html) | lifecycle/QoS가 application freshness·human approval을 대신하지 않음 | **즉시 반영**: fault latch + monotonic age. **현장 확인**: driver QoS | 기존 `SafetySupervisor`, integration tests | 2026-08-24 |
| JointTrajectory message | Open Robotics, 공식 | [message](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html) | names와 time-indexed points 표현이며 limit/approval 검증 기능은 없음 | **참고만**: generic dry-run envelope, vendor type 미가정 | `published=false`, `executed_action=null` | 2026-08-24 |

`n_action_steps>1`, temporal ensemble, vendor command publisher, 실제 joint limit 가정과 VLA/world model은
같은 task/checkpoint/safety 조건의 측정 또는 현장 정본이 생길 때까지 보류한다.

## Stage 8 · 이동형 양팔 SO-101 박스 열기·신발 추출

상세 조사·일정: [`research/LATEST_MOBILE_DUAL_SO101_BOX_SHOE_MISSION_RESEARCH_20260902.md`](research/LATEST_MOBILE_DUAL_SO101_BOX_SHOE_MISSION_RESEARCH_20260902.md)

| 자료 | 저자/기관·연도 | 원문 | 확인 내용 | DAPIER 결정 | 코드·테스트 증거 | 확인일 |
|---|---|---|---|---|---|---|
| N²M² | Freiburg, 2023 | [프로젝트](https://mobile-rl.cs.uni-freiburg.de/) | learned mobile motion과 IK arm solution 결합 | **실험 후보**: local approach/base placement 학습. 장거리 SLAM 대체는 보류 | PR #40 mobility/manipulator contract | 2026-09-02 |
| ReLMoGen | Stanford, 2021 | [프로젝트](https://svl.stanford.edu/projects/relmogen/) | RL subgoal과 low-level motion generator 분리 | **참고 반영**: learned proposal과 deterministic execution 분리 | orchestrator/transport boundary | 2026-09-02 |
| Error-Aware IL | Wong et al., 2022 | [PMLR](https://proceedings.mlr.press/v164/wong22a.html) | multi-stage failure detection/recovery | **즉시 반영**: phase/failure label과 recovery event | mission state, event log 계획 | 2026-09-02 |
| AnyGrasp | Fang et al., 2023 | [DOI](https://doi.org/10.1109/TRO.2023.3281153) | dense 6/7-DoF grasp candidate 생성 | **실험 후보**: cuboid baseline 뒤 grasp candidate 비교 | `grasp_planner.py` | 2026-09-02 |
| cuRobo | NVIDIA, 2023 | [원문](https://research.nvidia.com/publication/2023-05_curobo-parallelized-collision-free-robot-motion-generation) | IK, collision check, planner, trajectory optimization 결합 | **즉시 판정**: IK 단독으로 완료 주장 금지 | collision/contact release gate | 2026-09-02 |
| Mobile ALOHA | Fu et al., 2024 | [프로젝트](https://mobile-aloha.github.io/) | whole-body imitation learning과 target demonstrations | **실험 후보**: 충분한 실기 demonstration 뒤 IL | dataset preprocessing 계획 | 2026-09-02 |
| OK-Robot | Columbia, 2024 | [프로젝트](https://ok-robot.github.io/) | modular navigation/grasp의 실제 pose/hardware failure | **즉시 반영**: 모듈형 baseline과 실기 gate | 149/151 suite + contact failure 공개 | 2026-09-02 |
| HomeRobot OVMM | Meta/Georgia Tech, 2023 | [프로젝트](https://ovmm.github.io/) | 실제 mobile manipulation 성능 격차 | **즉시 반영**: sim 성공률을 실기 성공률로 외삽 금지 | sim-to-real matrix | 2026-09-02 |

현재 판정은 `visual SLAM/classical navigation baseline + 제한적 local approach 학습 + collision-aware IK/motion generation + tactile closed loop`다.
`IR`의 의미가 RL/IL/IL+RL 중 무엇인지는 강사 확인 전까지 확정하지 않는다.
