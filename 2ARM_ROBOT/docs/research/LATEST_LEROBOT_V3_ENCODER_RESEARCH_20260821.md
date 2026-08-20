# 최신 LeRobot Dataset v3 Native Encoder 조사 — 2ARM_ROBOT

확인일: 2026-08-21
대상: JDcobot 양팔·Orbbec Astra Pro·ROS2 recorder·RTX 5050·4인 6주·추가 예산 0원

## 이 단계를 지금 조사하는 이유

Stage 1의 raw RGB-D 계약은 ROS2 현장 기록을 신뢰할 수 있게 만든다. Stage 2는 그 정본을 **선택적으로** LeRobot Dataset v3로 내보내 ACT dataloader가 읽게 만드는 단계다. 변환 라이브러리를 recorder의 필수 의존성으로 넣으면 카메라 기록 자체가 PyTorch·PyArrow·FFmpeg·codec 버전 충돌에 묶인다. 따라서 공식 API·테스트가 요구하는 최소 계약과 optional dependency 경계를 먼저 정한다.

## 공식 LeRobot v3에서 확인한 실행 구조

```text
DAPIER raw finalized episode
  └─ 사전검증 (hash / schema / RGB-D unit / timestamp / calibration)
       └─ optional `lerobot[dataset]` 환경에서만 import
            └─ LeRobotDataset.create(features, fps, root, use_videos, ...)
                 └─ add_frame(frame) × N
                 └─ save_episode(parallel_encoding=...)
                 └─ finalize()
                      └─ v3 data/ Parquet + videos/ + meta/info.json·stats.json·episodes/
                           └─ 새 LeRobotDataset(root=...) + DataLoader smoke read
```

LeRobot v3의 writer API는 각 frame에 user-defined feature와 `task`를 요구하고, `timestamp`·`frame_index`는 writer가 계산한다. `add_frame()`은 image를 임시 저장하고, `save_episode()`가 Parquet/video와 episode metadata를 기록하며, 마지막 `finalize()`가 writer/footer를 닫는다. 따라서 DAPIER의 original timestamp·frame index는 frame key로 덮어쓰지 않고 별도 provenance feature/episode manifest로 보존해야 한다.

## 정본 사실과 DAPIER 해석

| 항목 | 공식 자료에서 확인한 사실 | DAPIER 설계 판단 |
|---|---|---|
| v3 파일 구조 | `meta/info.json`은 schema/FPS/path template, `stats.json`은 normalization stats, `meta/episodes`는 경계/offset, `data`는 Parquet, `videos`는 camera별 MP4 shard를 가진다. | raw episode를 삭제·대체하지 않는 export target으로 쓴다. `encoder_receipt.json`에 raw manifest SHA와 generated v3 manifest SHA를 연결한다. |
| writer lifecycle | `LeRobotDataset.create()` → `add_frame()` → `save_episode()` → `finalize()` 순서다. `finalize()` 미호출 시 Parquet footer/metadata가 불완전해 dataset load가 실패할 수 있다. | exporter는 임시 output directory에서만 쓰고, `finalize` 후 reopen/read 검증까지 성공해야 destination에 publish한다. |
| frame schema 검사 | 공식 tests는 feature 누락/추가, dtype, shape error를 `add_frame()`에서 검출한다. image는 HWC 또는 CHW를 받지만 expected shape/range가 어긋나면 실패한다. | 12-DoF 양팔 state/action, RGB `uint8`, depth 원본 dtype/shape를 exporter 전 preflight에서도 검사한다. 실패를 LeRobot stack trace로만 남기지 않는다. |
| depth metadata | v3 metadata는 feature `info`의 `is_depth_map`, `depth_unit`을 읽고 depth stats를 unit에 맞춰 처리한다. depth video는 `DepthEncoderConfig`가 정하며 `depth_min/max/shift/use_log`를 metadata에 보존한다. | depth raw unit은 Stage 1 실측값을 사용한다. Astra Pro 값이 확정 전이면 Dataset v3 depth export를 거부한다. feature name·unit·quantizer 기록 없이 generic image로 가장하지 않는다. |
| depth codec | raw `uint16 mm`/`float32 m`은 8-bit codec으로 보존할 수 없어서 공식 depth pipeline은 12-bit quantization, `gray12le`, HEVC Main 12를 사용한다. 기본은 lossless option이며 range 밖 값은 clip될 수 있다. | 6주 baseline은 raw `16UC1` archive를 정본으로 하고, v3 depth video는 **small round-trip을 통과한 optional derived artifact**로만 만든다. working range를 측정 전 default 0.01–10m에 묶지 않는다. |
| optional dependency | LeRobot은 `dataset`, `training`, `hardware`, `evaluation` 등 extras로 기능을 나누며 dataset 생성에는 `datasets`, `pyarrow`, AV/torchcodec 계열이 필요하다. 공식 policy factory도 lazy import로 optional dependency를 유지한다. | `shoe_sorting_data` 기본 설치에 LeRobot을 넣지 않는다. `act-export-native` 실행 순간에만 import하고, 미설치면 설치 명령·필요 extra·원본 보존 상태를 설명하는 오류를 낸다. |
| 공식 테스트 관행 | `tests/datasets/test_datasets.py`는 생성→frame 추가→episode 저장→finalize→`dataset[0]` shape를 확인하고, import가 없는 rollout test는 `pytest.importorskip("datasets")`로 skip한다. | 단위 테스트는 LeRobot 없는 CI에서도 계속 통과해야 한다. integration smoke는 extra 유무를 명시적으로 `SKIP` 또는 `PASS`로 보고해 "미실행"을 성공으로 표시하지 않는다. |
| 최근 데이터 인프라 보강 | Robo-DM(2025)은 vision/language/action 이종 stream을 시간 정렬한 self-contained storage와 decoder/cache 비용을 다룬다. | 대규모/클라우드 container 이식은 보류하되, raw 정본→portable train format→readback audit의 2-tier 구조는 적용한다. |

## 즉시 반영 — 현재 코드에 넣을 원칙

### 1. native encoder는 별도 optional adapter로 둔다

다음 조건이 모두 만족될 때만 `import lerobot` 한다.

1. 입력 episode가 Stage 1의 `finalized && accepted` 상태다.
2. raw manifest, calibration snapshot, RGB/Depth payload hash가 preflight에서 일치한다.
3. `lerobot[dataset]`와 exporter가 검증한 **정확한 LeRobot git SHA 또는 package version**이 `encoder_receipt`에 기록된다.
4. target output root가 비어 있거나 job ID가 붙은 새 directory다. 기존 dataset을 in-place로 수정하지 않는다.

기본 ROS2 recorder/CLI import path에는 `lerobot`, `pyarrow`, video decoder를 import하지 않는다. `importlib.util.find_spec` 또는 try/except는 실행 가능 여부를 판정하는 데만 쓰고, 설치된 module의 version을 receipt에 적는다.

### 2. 2ARM feature mapping을 좁고 명시적으로 고정한다

최초 native dataset schema는 task-conditioned ACT에 필요한 아래만 포함한다.

| v3 key | dtype / shape | DAPIER source | 주의 |
|---|---|---|---|
| `observation.state` | `float32`, `(12,)` | left 5+gripper, right 5+gripper position | joint order·단위를 hardware profile hash로 연결 |
| `action` | `float32`, `(12,)` | same control convention의 target/teleop action | position/velocity/delta를 섞지 않고 `control_mode` provenance에 기록 |
| `observation.images.workspace_rgb` | `video` 또는 `image`, `(3,H,W)` | raw RGB | actual encoding·color ordering을 preflight에서 변환/기록 |
| `observation.images.workspace_depth` | `video` 또는 `image`, `(1,H,W)` + depth info | raw depth | `is_depth_map=true`, actual `depth_unit`와 invalid convention 필수 |
| `task` | string | shoe pairing/placement instruction | task registry의 stable ID와 원본 manifest 연결 |

mobile base odometry, SLAM, LLM judgment는 처음 v3 ACT baseline의 입력 feature가 아니다. 하지만 episode manifest에는 계속 남겨 이후 evaluator·VLA의 provenance로 쓴다.

### 3. raw-to-v3 receipt와 atomic publish를 강제한다

`encoder_receipt.json`에 입력 episode IDs, raw manifest SHA-256, calibration IDs, code git SHA, Python/LeRobot version, schema, fps, video/depth encoder config, created timestamp, preflight 결과, output file hash와 round-trip 결과를 기록한다.

작업 중 쓰는 `*.partial` directory는 finalize/readback 실패 시 남겨 진단 가능하게 하고 `published=false`로 표시한다. 유효 output은 `published=true` receipt를 쓴 뒤 최종 path로 rename한다. 원본 raw 디렉터리는 이 작업이 읽기 전용으로 취급한다.

### 4. 최소 round-trip을 release gate로 만든다

small dataset(최소 2 episode × 3 frames)은 다음을 확인한다.

1. `create` 후 required feature + `task`만 가진 frame을 추가할 수 있다.
2. 각 episode의 `save_episode`, 전체 `finalize` 후 새 `LeRobotDataset` instance로 reopen한다.
3. episode 수, frame 수, `observation.state`/`action` shape·dtype, task, RGB shape를 원본 manifest와 비교한다.
4. depth가 있으면 readback value/physical unit, `is_depth_map`, `depth_unit`, quantizer config가 expectation과 일치하는지 비교한다. clip count도 receipt에 남긴다.
5. `torch.utils.data.DataLoader`가 최소 batch 1을 읽고 ACT dataset adapter가 요구하는 key/tensor shape를 반환한다.

현장 데이터가 없으면 synthetic `uint8 RGB`와 synthetic `uint16 depth mm`를 테스트 fixture로 써 API 계약을 검증하되, Astra Pro native depth round-trip을 통과했다고 주장하지 않는다.

### 5. optional integration은 별도 환경에서 고정한다

RTX 5050 노트북의 ROS2 교육 PC에는 기존 ROS2 workspace와 별도의 Python venv/uv environment를 둔다. version matrix(ROS distro, Python, torch/CUDA, LeRobot SHA, `lerobot[dataset]`, ffmpeg)를 `native_encoder_environment.md` 또는 receipt에 기록한다. dataset export와 ACT smoke가 동시에 가능한 작은 fixture로 먼저 확인하고, full training은 그 뒤에 실행한다.

## 실험 후보 — baseline 통과 뒤 수행

| 후보 | 검토 근거 | 실행 조건과 측정 |
|---|---|---|
| Depth video vs lossless image export | 공식 v3 depth pipeline은 storage/streaming을 위한 12-bit video를 제공한다. | 동일 raw episodes로 lossless image와 depth video의 file size, encode/decode 시간, invalid/clipped pixels, physical-unit error를 비교. 정책 성능이 아닌 data fidelity를 먼저 통과시킨다. |
| `use_videos=true` streaming encoding | v3는 많은 episode를 shard로 합치고 streaming encode를 지원한다. | 5~10 episode fixture에서 CPU/RAM/소요시간을 계측하고 ROS recording process와 분리된 offline export에서만 사용한다. |
| Robo-DM/다른 container의 export | 2025 Robo-DM은 large-scale retrieval/압축 병목을 개선한다. | 6주 baseline·포트폴리오가 끝난 뒤, 실제 storage/decode 병목이 측정된 경우에만 비교한다. LeRobot v3 interoperability를 깨지 않는 derived export여야 한다. |
| v3 metadata에 학습 전용 split/eval tag 추가 | v3 metadata/episode boundary는 searchable relational records를 지원한다. | scene/object/session split이 고정된 뒤 train-only stats와 no-leak evaluator 결과를 separate manifest로 기록한다. |

## 보류 — 이번 범위에서는 하지 않는 것

| 항목 | 보류 이유 |
|---|---|
| LeRobot main branch를 ROS2 base environment에 무조건 설치 | 공식 문서도 v3의 release 경계와 extras를 구분한다. CUDA/codec/PyArrow 충돌이 실제 recorder를 중단시킬 위험이 있다. |
| Dataset v3 output을 유일한 원본으로 삼기 | video quantization/codec/version 변화로 인한 복원 차이를 추적하기 어렵고, raw payload provenance가 사라진다. |
| 첫 단계부터 multi-camera/wrist video shard 병렬 encode | Astra Pro 1대와 RGB-D calibration/time gate를 먼저 증명해야 하며, 더 많은 camera는 encode/debug 변수를 늘린다. |
| 6주 안에 Robo-DM·RLDS·Lance를 모두 구현 | format conversion 숫자만 늘고 ACT baseline과 real rollout 검증이 늦어진다. |
| encoder test가 skip된 것을 PASS로 표기 | optional dependency가 없는 기본 CI에서 skip은 정상이나, native v3 compatibility의 증거는 아니다. |

## 구현 순서와 학습 포인트

1. **Stage 1 raw contract를 완성**한다. 이유: encoder가 바꿀 수 없는 사실(픽셀/unit/time/calibration)을 확정해야 한다.
2. **preflight + lazy import adapter를 만든다.** 이유: LeRobot 설치 여부가 recorder 안정성과 분리되어야 한다.
3. **small v3 round-trip**을 실행한다. 이유: `save_episode` 성공만으로 metadata/footer/depth decode가 정상이라는 뜻은 아니기 때문이다.
4. **ACT dataloader smoke**를 실행한다. 이유: dataset reader와 policy input contract는 별개이며 key/shape/stats 문제가 이 지점에서 드러난다.
5. **offline evaluator와 action chunk/padding**으로 넘어간다. 이유: native 파일 형식 검증 뒤에야 policy sequence의 성능/안전 검증에 의미가 생긴다.

## 근거 원문 및 확인일

모든 링크는 2026-08-21 확인했으며, 블로그/2차 해설 대신 공식 문서·코드·테스트와 논문 원문만 설계 근거로 사용했다.

1. Hugging Face, [LeRobot Dataset v3 공식 문서](https://github.com/huggingface/lerobot/blob/main/docs/source/lerobot-dataset-v3.mdx) — v3 layout, writer lifecycle, `finalize`, migration.
2. Hugging Face, [LeRobotDataset 공식 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/lerobot_dataset.py) — `add_frame`, `save_episode`, `clear_episode_buffer`, `finalize` API와 writer 조건.
3. Hugging Face, [DatasetWriter 공식 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/dataset_writer.py) — frame feature 검증, temporary image/video 처리, episode save 경계.
4. Hugging Face, [Dataset tests](https://github.com/huggingface/lerobot/blob/main/tests/datasets/test_datasets.py) — missing/extra/type/shape/image validation 및 create→save→finalize→readback 테스트 관행.
5. Hugging Face, [depth video encoding 공식 문서](https://github.com/huggingface/lerobot/blob/main/docs/source/video_encoding_parameters.mdx) 및 [video config 코드](https://github.com/huggingface/lerobot/blob/main/src/lerobot/configs/video.py) — `is_depth_map`, `depth_unit`, 12-bit depth codec/quantizer/defaults.
6. Hugging Face, [installation 공식 문서](https://github.com/huggingface/lerobot/blob/main/docs/source/installation.mdx), [package extras 코드](https://github.com/huggingface/lerobot/blob/main/src/lerobot/__init__.py), [AGENTS.md](https://github.com/huggingface/lerobot/blob/main/AGENTS.md) — feature extras와 lazy import/`require_package` 관행.
7. Chen et al., 2025, [Robo-DM 논문 원문](https://arxiv.org/abs/2505.15558) 및 [공개 코드](https://github.com/BerkeleyAutomation/robodm) — heterogeneous robot stream 저장·alignment·decode 비용을 다루는 최신 데이터 인프라 비교 근거.

## 학습 메모

- **강의에서 확인**: ACT 학습은 결국 observation/action tensor 계약을 요구한다. Dataset format은 그 tensor를 반복 가능하게 재생하는 수단이다.
- **외부 보강**: LeRobot v3는 "한 episode = 한 파일"보다 schema·metadata·shard를 정본으로 다루며 depth를 별도 물리량으로 표시한다.
- **학습자 해석**: DAPIER에서는 native encoder가 robot driver가 아니라 interchange adapter다. 설치 실패는 recorder 실패가 아니며, round-trip 실패는 raw data를 손상시키지 않는다.
- **다음 확인**: native optional environment에서 2 episode×3 frame fixture를 `create→add_frame→save_episode→finalize→reopen→DataLoader`로 실행하고, receipt에 `PASS/SKIP/FAIL` 중 하나를 정확히 기록한다.
