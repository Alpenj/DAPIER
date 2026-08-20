# 최신 RGB-D 데이터 계약 조사 — 2ARM_ROBOT

확인일: 2026-08-21
적용 대상: JDcobot 양팔, TurtleBot3 Waffle Pi, Orbbec Astra Pro 1대, RTX 5050 노트북, 4인·6주·추가 예산 0원

## 이 조사를 먼저 하는 이유

1단계의 목표는 카메라 영상을 "나중에 학습에 쓸 수 있는 증거"로 남기는 것이다. RGB-D 정책 성능보다 앞서는 조건은 프레임이 실제 픽셀값, 같은 시각의 로봇 상태·행동, 해당 카메라의 보정값과 연결되어 다시 읽힐 수 있는가이다. 이 문서는 공개된 1차 자료(논문 원문, 공식 프로젝트, 공식 코드·문서)만 사용해 그 최소 계약을 정한다.

## 조사 범위와 해석 원칙

- **원본 보존 우선**: ROS 메시지의 `encoding`, 행렬 크기, 바이트 수를 바꾸지 않은 원본 payload를 먼저 저장한다. 학습용 압축·변환본은 파생 산출물이다.
- **동기화는 측정값으로 기록**: "동기화됨"이라는 불리언 대신 각 입력의 header timestamp, 수신 시각, 매칭 차이(Δt), 설정 tolerance를 남긴다.
- **보정은 에피소드 외부 상수가 아니다**: intrinsics·distortion·depth↔color extrinsics·robot base↔camera transform 및 그 파일 hash/version을 episode manifest에 스냅샷한다.
- **장비 한계를 설계에 반영**: Astra Pro 실장 드라이버·CameraInfo 유효성은 아직 현장 검증 전이다. 따라서 어떤 해상도, depth 단위, depth registration도 코드 상수로 가정하지 않는다.

## 최신 정본에서 확인한 설계

| 주제 | 1차 근거에서 확인한 사실 | DAPIER에 주는 의미 |
|---|---|---|
| 멀티모달 저장 | LeRobot Dataset v3는 저차원 시계열을 Parquet, 카메라를 별도 visual stream, schema·FPS·stats·episode 경계를 metadata로 분리한다. 여러 episode가 하나의 파일에 있어도 metadata offset으로 복원한다. | RGB/Depth 파일과 action/state JSON을 임시 이름으로만 묶지 말고 episode manifest를 정본으로 둔다. |
| depth 표현 | LeRobot은 `is_depth_map`과 기록 당시 `depth_unit`을 feature metadata에 저장한다. 비디오로 만들 때 원본 `uint16 mm` 또는 `float32 m`를 8-bit codec에 넣지 않고 12-bit quantization과 quantizer 설정을 저장해 물리 단위로 복원한다. | Stage 1은 lossless `16UC1` 원본과 명시 unit을 보존한다. MP4 depth는 Dataset v3 변환 단계의 선택적 파생본이며, 원본을 대체하지 않는다. |
| timestamp 동기화 | LeRobotDataset API는 `tolerance_s`를 timestamp synchronization tolerance로 받는다. ROS 2 `ApproximateTimeSynchronizer`도 header timestamp를 기준으로 매칭하며 headerless/arrival-time 동기화는 지연이 예측 불가하므로 피하라고 명시한다. | RGB, depth, left/right joint state, teleop action마다 header stamp 및 pair Δt를 저장하고, tolerance 초과 frame은 학습 승인에서 제외한다. |
| calibration/provenance | DROID는 multi-camera intrinsics/extrinsics를 제공하며, 2025-04에 36k episodes의 개선 calibration을 별도 배포했다. DROID 코드의 trajectory schema도 카메라 serial과 extrinsics를 metadata로 추출한다. | calibration 수정이 과거 episode를 조용히 바꾸면 재현 불가다. calibration snapshot hash와 camera serial/driver/firmware를 manifest에 고정한다. |
| 완결성 | LeRobot v3의 `finalize()`는 buffered episode metadata와 Parquet footer를 flush/close하며, 하지 않으면 파일이 불완전하여 load할 수 없다고 명시한다. | recorder는 기록 중 `recording`, 검증 성공 뒤 `accepted`, 종료/해시 검증 뒤 `finalized`의 상태 전이를 갖고, 비정상 종료 episode는 학습 후보에서 제외한다. |
| 데이터 품질의 정책 효과 | DROID는 target setting에 소수 target-domain demonstrations를 co-training에 넣는 방식을 권고한다. 2024 MCR은 robot proprioception/action을 활용한 visual pretraining의 이점을 보고한다. | 6주·30켤레 문제에서는 대규모 데이터 흉내보다 **동일 카메라·동일 신발장·실제 양팔 action**의 고품질 demo와 state/action 연결이 우선이다. 표현학습 보조목표는 baseline 뒤 실험한다. |

## Stage 1 데이터 계약 — 즉시 반영

아래 항목은 추가 구매 없이 recorder·manifest·quality gate에 바로 넣는다. 값이 장비에서 아직 확정되지 않은 항목은 **실측 후 채움**으로 남기며, 임의 default를 기록값으로 쓰지 않는다.

### 1. episode 디렉터리와 원본/파생 분리

```text
episode-000123/
  manifest.json                 # schema/version, 상태, 파일 목록과 SHA-256
  calibration/
    color_camera_info.yaml      # 수집 시점 snapshot
    depth_camera_info.yaml
    transforms.yaml             # base↔camera 및 depth↔color
    provenance.json             # camera serial, driver/firmware, ROS distro, git SHA
  raw/
    rgb/000000.png              # 원본 RGB payload의 lossless 저장본
    depth/000000.png            # 원본 16UC1의 lossless 저장본
    timeline.jsonl              # frame-level timestamp·Δt·state/action 참조
  derived/                      # resize, registered depth, MP4 등 재생성 가능 산출물
```

`manifest.json`에는 `schema_version`, `episode_id`, `recording_started_at`, `recording_ended_at`, `fps_target`, `sync_tolerance_ns`, `calibration_snapshot_sha256`, source git SHA, 모든 파일의 SHA-256와 `finalization_state`를 둔다. 원본 파일을 overwrite하지 않고 파생본에는 입력 hash·변환 코드 SHA·파라미터를 기록한다.

### 2. frame-level 필수 필드

각 accepted frame은 최소 아래 필드를 가진다.

| 범주 | 필수 필드 |
|---|---|
| 식별 | `episode_id`, `frame_index`, `task_id`, `operator_id`(비식별 ID), `scene_id`, `object_set_id` |
| RGB | `rgb_path`, `rgb_header_stamp_ns`, `rgb_received_monotonic_ns`, `rgb_encoding`, `rgb_width`, `rgb_height`, `rgb_sha256` |
| Depth | `depth_path`, `depth_header_stamp_ns`, `depth_received_monotonic_ns`, `depth_encoding`, `depth_width`, `depth_height`, `depth_unit`, `invalid_depth_value`, `depth_sha256` |
| 동기화 | `anchor_stamp_ns`, `rgb_depth_delta_ns`, `left_state_delta_ns`, `right_state_delta_ns`, `action_delta_ns`, `sync_tolerance_ns`, `sync_status` |
| 조작 | `left_joint_position`, `right_joint_position`, `left_gripper`, `right_gripper`, `action`, `action_frame`, `control_mode` |
| 품질 | `camera_info_valid`, `calibration_id`, `calibration_snapshot_sha256`, `dropped_topic_counts`, `quality_gate_result`, `rejection_reason` |

**anchor는 robot action 또는 control tick timestamp**로 정하고, camera message 도착 시각은 디버깅용 보조값으로만 쓴다. RGB-depth 등록이 꺼진 경우 `registered_to_color=false`와 각기 다른 intrinsics를 분명히 기록한다.

### 3. Astra Pro depth 안전 규칙

- ROS message의 실제 `encoding`을 manifest에 기록한다. 기대하는 `16UC1`이라고 가정해도 `32FC1`, `Y11` 등 다른 형식이면 저장·검증이 멈추고 원인을 표시한다.
- `depth_unit`은 device/driver가 보고하고 물체 거리 실측으로 확인한 뒤 `mm` 또는 `m`으로 확정한다. `16UC1` 값에 임의로 `0.001`을 곱한 값을 원본으로 저장하지 않는다.
- 0, NaN, saturation 등 invalid convention과 프레임별 invalid ratio를 기록한다. 신발의 검은 고무·반사 재질은 depth hole을 만들 수 있으므로 RGB fallback을 가능하게 남긴다.
- `CameraInfo.K`, `D`, distortion model, image size가 finite이고 dimension과 맞는지 quality gate에서 검사한다. Astra Pro 계열 ROS2 driver에서 CameraInfo intrinsics가 NaN인 사례가 공개되어 있으므로, 이 검사는 실제 배포 차단 조건이다.

### 4. calibration/provenance 최소 기준

수집 시작 전과 calibration 변경 뒤 아래를 캡처한다.

- `/camera/color/camera_info`, `/camera/depth/camera_info` 원문과 topic/frame ID
- camera serial, firmware, USB mode, ROS2 distribution, driver package/version, launch 파라미터
- `depth_registration`, `color_depth_synchronization` 같은 driver 설정
- `T_base_camera`, `T_depth_color`의 방향·단위·좌표계 표기 및 유효성 검증 결과
- 로봇 model/firmware, joint order, gripper convention, control mode, recorder source git SHA

같은 physical camera가 이동하거나 launch parameter가 바뀌면 새 `calibration_id`를 발급한다. 보정값 자체를 나중에 교체하는 것이 아니라 새 artifact를 생성하고, 기존 episode는 기존 snapshot을 계속 참조한다.

### 5. finalize와 integrity gate

`accepted`가 되려면 다음이 모두 참이어야 한다.

1. 필수 토픽 모두 header timestamp가 있고 최대 Δt가 `sync_tolerance_ns` 이하이다.
2. RGB/Depth payload byte size·encoding·shape가 manifest와 일치한다.
3. camera_info가 finite하고 dimensions가 해당 이미지와 일치한다.
4. calibration snapshot과 raw 파일의 SHA-256가 manifest와 일치한다.
5. joint/action 길이·joint order·단위가 hardware profile과 일치한다.

실패 episode는 삭제하지 않고 `rejected`와 구체적 사유를 기록한다. 학습 exporter는 `finalized && accepted`만 읽는다. 이 규칙은 데이터가 적은 6주 프로젝트에서 잘못된 demo가 training에 섞이는 비용을 낮춘다.

## 최신 연구를 반영한 보완 항목

### 즉시 반영 — 비용 0, 일정 영향 낮음

| 보완 | 이유 | 완료 판정 |
|---|---|---|
| 원본 RGB/Depth와 파생 LeRobot v3 artifact 분리 | LeRobot depth 비디오도 quantizer metadata 없이는 물리량 복원이 불가능하다. | raw hash가 변환 전후 동일하며 derived manifest가 source hash를 가리킨다. |
| Δt 분포 및 dropped-topic counts 기록 | Approximate sync는 편리하지만 정확도 보장은 아니다. quality gate가 실제 지연을 노출한다. | episode summary에 p50/p95/max Δt와 topic별 drop 수가 있다. |
| calibration snapshot immutability | DROID가 뒤늦게 calibration 개선본을 배포한 사실은 과거 데이터와 보정 revision을 연결해야 함을 보여 준다. | episode가 `calibration_id + sha256`를 참조하고 overwrite가 거부된다. |
| `CameraInfo` finite/dimension 검사 | Astra Pro 계열의 invalid intrinsics 공개 사례가 있으므로 point cloud/3D 추론 전 차단해야 한다. | NaN 또는 size mismatch demo가 `rejected`로 끝난다. |
| scene/object/operator/session split key 저장 | target-domain co-training과 공정한 offline evaluation을 위해 같은 신발·같은 배경의 frame leakage를 막는다. | exporter가 split key 누수를 검출한다. |

### 실험 후보 — baseline 뒤에만 실행

| 후보 | 기대 효과 | go/no-go 조건 |
|---|---|---|
| native LeRobot Dataset v3 depth video | multi-camera Dataset v3/ACT dataloader와 연결하고 저장 효율을 높인다. | Stage 1 raw round-trip·unit/dequantization 검증 후, optional dependency 환경에서 small dataset round-trip 성공. |
| RGB+Depth vs RGB-only ablation | 신발 위치·선반 깊이에서 depth가 실제로 이득인지 검증한다. | 동일 episode split, 동일 ACT backbone·학습 예산으로 success 및 grasp/placement error를 비교. |
| proprioception/action 기반 representation 보조목표 | 작은 target-domain data에서 visual feature를 보강할 가능성. | ACT baseline이 재현되고 validation success 또는 error가 사전 정의된 기준 이상 개선될 때만 유지. |
| calibration 재검증 도구(손-눈/3D target) | 카메라 이동 후 base-frame placement의 오차를 정량화한다. | 보정 전후 reprojection/3D placement error를 기록하고 rollout 안전 gate와 연결. |

### 보류 — 6주/무예산 범위를 넘거나 정본이 부족함

| 보류 항목 | 사유 |
|---|---|
| depth를 RGB 8-bit MP4로 강제 저장 | 값·단위가 소실되어 3D/깊이 정책 검증의 근거가 사라진다. |
| 다중 카메라·wrist camera 즉시 추가 | 현재 Astra Pro 1대도 driver/calibration 검증이 선행되어야 하며 장비·통합 부담이 크다. |
| SLAM map을 imitation dataset의 필수 입력으로 결합 | mobile base가 docking 후 정지하는 프로젝트 구조에서 ACT manipulation baseline을 지연시킨다. map은 navigation lane의 별도 artifact로 둔다. |
| 외부 대규모 데이터로 2ARM policy를 곧바로 pretrain | embodiment·action space·camera geometry가 달라 6주 안에 비교 가능한 통제가 어렵다. DROID의 권고도 target-domain demo co-training이다. |

## 구현 순서에 대한 연결

1. **Astra Pro RGB/Depth payload 저장 계약**: 여기의 raw, time, calibration, integrity 조건을 code/test로 고정한다. 이유는 이후 encoder/evaluator가 신뢰할 입력을 만들기 위해서다.
2. **optional native Dataset v3 encoder**: raw contract가 확정된 뒤에만 변환한다. 이유는 LeRobot 버전·codec 의존성이 ROS recorder를 깨지 않게 하기 위해서다.
3. **small v3 round-trip + ACT dataloader smoke**: 파일이 있다는 것과 ACT가 정확한 shape/unit/timestamp로 읽는 것은 다르므로 작은 데이터로 먼저 증명한다.
4. **offline evaluator + action chunk/padding**: training loss만으로 closed-loop 안전성을 대신할 수 없으므로, episode/segment 수준의 error·padding·split을 검증한다.
5. **JDcobot rollout + 독립 safety supervisor**: policy는 후보 action만 내고, range/rate/E-stop/정지 상태 검증은 별도 supervisor가 담당한다.

## 근거 원문 및 확인일

모든 링크는 2026-08-21에 확인했다. 수치·성능 주장은 이 문서의 설계 결정에 쓰지 않고, 각 원문의 공개 구조·API·운영 지침만 인용했다.

1. Hugging Face, [LeRobot Dataset v3.0 공식 문서](https://github.com/huggingface/lerobot/blob/main/docs/source/lerobot-dataset-v3.mdx) — Parquet/video/metadata 분리, `finalize()`, v3 directory와 metadata 구조.
2. Hugging Face, [LeRobot depth video encoding 공식 문서](https://github.com/huggingface/lerobot/blob/main/docs/source/video_encoding_parameters.mdx) — depth unit, 12-bit quantization, metadata와 read-time unit.
3. Hugging Face, [LeRobot dataset metadata 공식 코드](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/dataset_metadata.py) — `is_depth_map`, `depth_unit`, depth stats rescaling 구현.
4. ROS 2, [message_filters API](https://docs.ros.org/en/ros2_packages/rolling/api/message_filters/message_filters.html) — header timestamp 기반 ApproximateTime 및 headerless/arrival-time 주의.
5. Khazatsky et al., 2024, [DROID 논문 원문](https://arxiv.org/abs/2403.12945) 및 [공식 프로젝트](https://droid-dataset.github.io/) — 대규모 real-robot dataset, multi-camera calibration 공개와 2025 calibration update.
6. DROID 공식 코드, [trajectory schema](https://github.com/droid-dataset/droid/blob/main/droid/postprocessing/schema.py) 및 [policy-learning README](https://github.com/droid-dataset/droid_policy_learning) — camera serial/extrinsics metadata, target-domain demonstration co-training 안내.
7. Orbbec 공식 ROS2 Astra driver, [ros2_astra_camera README](https://github.com/orbbec/ros2_astra_camera/blob/master/README.MD) — CameraInfo, calibration file, depth registration/color-depth sync/extrinsic publish parameters.
8. Orbbec 공식 ROS2 driver issue, [Astra Pro CameraInfo NaN 사례](https://github.com/orbbec/OrbbecSDK_ROS2/issues/134) — `CameraInfo` 유효성 gate가 필요한 실제 공개 결함 사례. 이 항목은 일반 성능 근거가 아니라 Astra Pro 현장 사전점검 위험 근거다.
9. Jiang et al., 2024, [Robots Pre-train Robots / MCR 논문 원문](https://arxiv.org/abs/2410.22325) — vision, proprioception, action을 함께 다루는 representation 보조학습의 실험 근거. DAPIER 적용은 실험 후보일 뿐 성능 보장은 아니다.

## 이번 단계의 학습 메모

- **강의에서 확인**: ROS2 message header와 `CameraInfo`는 카메라 프레임을 단순 이미지가 아닌 시간·기하 정보가 있는 센서 관측으로 다룰 수 있게 한다.
- **외부 보강**: LeRobot v3와 DROID는 데이터 파일 자체보다 schema, stats, camera metadata, episode boundary를 같이 남기는 방식을 정본으로 사용한다.
- **학습자 해석**: 2ARM_ROBOT의 차별점은 "RGB-D를 쓴다"가 아니라, 각 grasp/placement 행동이 어떤 unit·보정·timestamp의 관측에 근거했는지 재검증할 수 있다는 데 있다.
- **다음 검증**: Astra Pro를 연결한 상태에서 60초 샘플을 기록하고 `encoding`, `depth_unit`, `CameraInfo` finite 검사, p95 Δt, raw hash round-trip을 측정한다. 이 결과 전에는 native Dataset v3/ACT camera policy를 성공했다고 주장하지 않는다.
