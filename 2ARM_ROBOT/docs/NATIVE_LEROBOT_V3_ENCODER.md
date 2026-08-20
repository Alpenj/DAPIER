# Optional Native LeRobot Dataset v3 Encoder

> encoder contract: `dapier.lerobot-v3-encoder.v0.1`
> upstream source reference: `huggingface/lerobot@d451fe4f1f1b00a812f95aa9534389b5e42ab155`
> 상태: lazy import·preflight·native writer 구현 완료, 실제 native round-trip은 Stage 3 gate

## 왜 optional adapter인가

ROS2 recorder의 책임은 실제 robot/camera 데이터를 잃지 않고 기록하는 것이다. LeRobot, Torch, Pillow, Datasets, PyArrow, codec은 derived dataset을 만드는 offline dependency다. 둘을 같은 필수 환경에 넣으면 native encoder 설치 실패가 실제 데이터 수집 실패로 번진다.

따라서 `shoe_sorting_data` 기본 import는 optional package를 불러오지 않는다. `native-export`를 호출한 순간에만 native stack을 검사하고 import한다.

## 명령 경계

```bash
# 설치 없이 가능
ros2 run shoe_sorting_data shoe_episode native-status

# 설치 없이 가능: raw contract와 native schema만 검증
ros2 run shoe_sorting_data shoe_episode native-preflight \
  --root output/rgbd_episodes \
  --depth-unit mm

# optional LeRobot environment에서만 가능
ros2 run shoe_sorting_data shoe_episode native-export \
  --root output/rgbd_episodes \
  --output output/lerobot_v3_v001 \
  --repo-id local/dapier-shoe-v001 \
  --depth-unit mm
```

`depth-unit`은 `mm` 또는 `m`을 명시해야 한다. Astra 실제 unit을 확인하기 전에 native depth export를 실행하면 안 된다. 합성 16UC1 fixture의 `mm`는 API smoke를 위한 테스트 조건이지 Astra 실측 결과가 아니다.

## 최초 native schema

| key | dtype/shape | source |
|---|---|---|
| `observation.state` | float32 `(12,)` | 좌팔 5 + 좌 gripper + 우팔 5 + 우 gripper |
| `action` | float32 `(12,)` | 동일 joint order의 target/teleop action |
| `observation.images.workspace_rgb` | image `(H,W,3)` | Stage 1 lossless RGB raw |
| `observation.images.workspace_depth` | image `(H,W,1)` | depth raw + `is_depth_map=true` + explicit unit |
| `task` | string | manifest language instruction |

base odometry, SLAM map, LLM 판단은 첫 ACT baseline feature에 넣지 않는다. 원본 manifest provenance에는 보존한다.

## 처리 흐름

```text
finalized raw episode
  -> quality/hash/payload/unit/fps preflight (no optional import)
  -> optional stack lazy import
  -> partial native output
  -> LeRobotDataset.create
  -> add_frame × N
  -> save_episode × episodes
  -> finalize
  -> raw hashes unchanged 확인
  -> output hashes + encoder receipt
  -> final path publish
```

실패한 partial directory는 진단용 failure receipt를 남기고 raw source는 변경하지 않는다. 기존 output path도 덮어쓰지 않는다.

## 현재 PC dependency 결과

| module | 상태 |
|---|---|
| NumPy | 설치됨 (`2.5.1`) |
| LeRobot | 미설치 |
| Pillow | 미설치 |
| Torch | 미설치 |
| Datasets | 미설치 |
| PyArrow | 미설치 |

이 상태에서도 base recorder, quality, preflight를 포함한 62개 테스트가 Windows와 WSL Ubuntu에서 통과했다. 이는 native 호환성 PASS가 아니라 **optional 격리 PASS**다.

## 최신 연구/정본 보강

상세 조사: [`research/LATEST_LEROBOT_V3_ENCODER_RESEARCH_20260821.md`](research/LATEST_LEROBOT_V3_ENCODER_RESEARCH_20260821.md)
전체 채택 원장: [`RESEARCH_ADOPTION_LEDGER.md`](RESEARCH_ADOPTION_LEDGER.md)

즉시 반영:

- 공식 writer lifecycle `create→add_frame→save_episode→finalize`
- native frame에서 timestamp/frame_index를 직접 넣지 않고 writer 생성값 사용
- raw source hash와 derived output hash를 receipt로 연결
- optional dependency가 없으면 명시적 `available=false`
- first schema를 12DoF+RGB+Depth+task로 제한

실험 후보:

- depth image와 12-bit depth video의 fidelity/size/latency 비교
- native streaming encoder는 5~10 episode 계측 뒤 go/no-go

보류:

- ROS2 base environment에 LeRobot main 강제 설치
- v3 output을 유일한 원본으로 사용
- Robo-DM/RLDS/Lance 동시 구현
- optional integration test SKIP을 native compatibility PASS로 표시

## Stage 3 release gate

최소 2 episodes × 3 frames에서 다음이 모두 성공해야 native encoder 완료로 승격한다.

1. create/add/save/finalize
2. 새 LeRobotDataset instance로 reopen
3. episode/frame 수와 key·shape·dtype 확인
4. depth `is_depth_map`, unit, pixel value round-trip
5. Torch DataLoader batch
6. ACT delta timestamp/action chunk key 생성 확인
