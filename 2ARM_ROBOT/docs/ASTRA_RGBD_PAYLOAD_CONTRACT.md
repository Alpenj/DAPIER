# 전면 Astra + 작업공간 H201 RGB/Depth 픽셀 payload 저장 계약

> 계약 버전: `dapier.ros2-image-payload.v0.1`
> episode 버전: `dapier.shoe-episode.v0.3`
> 상태: 4카메라 역할 확정, MuJoCo 4카메라 lossless episode 완료, 실물 multi-camera recorder 미완료

## 왜 이 단계가 먼저인가

LeRobot Dataset이나 ACT가 실행되더라도 원본 RGB/Depth의 encoding, byte layout, timestamp, 단위가 유실되면 결과를 재검증할 수 없다. 그래서 모델 의존성을 설치하기 전에 ROS 2 `sensor_msgs/Image` payload를 손실 없이 보존하는 계약을 먼저 고정한다.

## 저장 원칙

1. ROS image row bytes를 변환하지 않고 `.raw`로 저장한다.
2. RGB와 depth는 서로 다른 encoding 계약으로 검사한다.
3. raw 파일은 절대 overwrite하지 않는다.
4. 파생 LeRobot artifact는 별도 output에 만들고 raw SHA-256을 참조한다.
5. `accepted + finalized + integrity_verified` episode만 exporter가 읽는다.

카메라 역할은 다음처럼 고정한다.

| 역할 | 장치 | 소비자 |
|---|---|---|
| `front_rgbd` | Orbbec Astra S | TurtleBot3 Visual SLAM·map localization |
| `workspace_rgbd` | HP-ASC-H201 / eYs3D R77 | 박스·신발 top-view 인식과 재관측 |
| `left_gripper_rgb` | 왼쪽 SO-101 RGB | 왼손 접근·파지 |
| `right_gripper_rgb` | 오른쪽 SO-101 RGB | 오른손 뚜껑 접근·접촉 |

기존 `shoe_sorting_data` Phase 0 실물 writer 예시는 호환성을 위해 `workspace_rgb/depth` 두 stream을
유지한다. MuJoCo `sim_episode.py`는 네 카메라 역할을 같은 episode clock으로 저장한다. 네 실물
카메라를 같은 clock으로 기록하는 adapter는 아직 완료되지 않았다.

```text
episode_000001/
├── episode_manifest.json
├── samples.jsonl
└── raw/
    ├── front_rgb/frame_000000.raw
    ├── front_depth/frame_000000.raw
    ├── workspace_rgb/frame_000000.raw
    ├── workspace_depth/frame_000000.raw
    ├── left_gripper_rgb/frame_000000.raw
    └── right_gripper_rgb/frame_000000.raw
```

## frame payload metadata

```json
{
  "contract_version": "dapier.ros2-image-payload.v0.1",
  "storage": "ros2_raw_rows",
  "stream": "workspace_depth",
  "path": "raw/workspace_depth/frame_000000.raw",
  "width": 640,
  "height": 480,
  "encoding": "16UC1",
  "is_bigendian": 0,
  "step": 1280,
  "byte_count": 614400,
  "sha256": "..."
}
```

## 허용 encoding

| stream | 허용 encoding | 현재 의미 |
|---|---|---|
| `front_rgb`, `left_gripper_rgb`, `right_gripper_rgb` | `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, `8UC1` | 전면·손목 색상 payload |
| `front_depth` | `mono16`, `16UC1`, `16SC1`, `32FC1` | 전면 SLAM depth payload |
| `workspace_rgb` | `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, `8UC1` | 색상·채널 순서를 metadata로 보존 |
| `workspace_depth` | `mono16`, `16UC1`, `16SC1`, `32FC1` | 원본 정수/실수 depth 값을 변환 없이 보존 |

`16UC1=mm`라고 코드에서 가정하지 않는다. 실제 depth unit은 각 카메라 driver 설정과 거리 실측 후 calibration/provenance snapshot에서 확정한다.

## timing 계약

required payload sample은 다음을 함께 저장한다.

- control tick 기준 `anchor_timestamp_ns`
- 현재 12개 stream(관절/명령/베이스 6 + 카메라 6)의 header timestamp
- 현재 12개 stream의 receive monotonic timestamp
- 한 sample에서 `max(header)-min(header)`인 `sync_delta_ns`
- manifest의 sync tolerance

arrival time은 header timestamp를 대체하지 않고 지연·drop 진단에만 사용한다.

## lifecycle 계약

v0.3 manifest는 다음이 아니면 유효하지 않다.

```json
{
  "lifecycle": {
    "state": "finalized",
    "integrity_verified": true
  }
}
```

기록 중이거나 crash로 끝난 디렉터리는 학습 입력으로 승격하지 않는다.

## quality gate

- payload 필수 모드에서 누락 차단
- stream별 잘못된 encoding 차단
- `step >= width * bytes_per_pixel`
- `byte_count == step * height`
- raw file SHA-256 일치
- episode 밖으로 나가는 상대 경로 차단
- 기존 raw frame overwrite 차단
- sync delta 재계산 결과와 metadata 일치
- header/receive timestamp map 완전성

## 실행 방법

```bash
ros2 run shoe_sorting_data shoe_episode generate \
  --root output/rgbd_fixture \
  --count 5 \
  --samples 4 \
  --camera-payload

ros2 run shoe_sorting_data shoe_episode validate \
  --manifest output/rgbd_fixture/episode_000001/episode_manifest.json
```

## 최신 연구 보강 결과

상세 근거는 [`research/LATEST_RGBD_DATA_CONTRACT_RESEARCH_20260821.md`](research/LATEST_RGBD_DATA_CONTRACT_RESEARCH_20260821.md)에 있다.

### 즉시 반영

- raw와 derived artifact 분리
- frame별 header/receive timestamp와 sync Δt
- 파일별 SHA-256와 finalized gate
- scene/object/operator/session provenance 기반 split 누수 방지 유지

### 실물 연결 시 반영

- CameraInfo K/D/R/P finite·dimension 검사
- color/depth intrinsics와 base↔camera, depth↔color transform snapshot
- camera serial, firmware, driver, ROS distro, launch parameter, git SHA
- depth unit, invalid value, invalid ratio 실측

### baseline 뒤 실험 후보

- native Dataset v3 depth video
- RGB-only와 RGB+Depth ACT ablation
- proprioception/action 보조 표현학습

### 보류

- depth를 8-bit RGB MP4로 저장
- 실측 bandwidth 없이 4카메라를 최고 해상도·FPS로 고정
- ACT manipulation 입력에 SLAM map을 즉시 결합
- embodiment가 다른 대규모 외부 데이터로 곧바로 pretrain

## 현재 완료/미완료 경계

완료:

- 합성 RGB8/16UC1 raw round-trip
- 누락·변조·경로 탈출·unit/geometry·overwrite 차단
- Windows/Ubuntu pure Python 검증
- MuJoCo 전면/workspace RGB-D와 좌우 wrist RGB의 synchronized raw episode 저장

미완료:

- Astra S와 H201 각각의 실제 encoding/resolution/depth unit 확인
- 실제 `CameraInfo`와 calibration snapshot
- 장시간 기록의 p50/p95 sync delta와 dropped topic count
- `front_rgbd`, `workspace_rgbd`, 좌우 `gripper_rgb`를 같은 monotonic clock으로 저장하는 실물 adapter
- native LeRobot Dataset v3 변환
