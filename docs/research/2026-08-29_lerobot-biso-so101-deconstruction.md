# LeRobot BiSOFollower 해체분석과 DAPIER 양팔 SO-101 개인화

## 1. 결론

LeRobot의 `BiSOFollower`는 두 SO-101을 하나의 객체처럼 보이게 하는 얇은 조합기다. 핵심은 왼팔·오른팔 객체 생성, 이름 prefix, 순차 관측, 순차 명령 전달이다. 공유 좌표계, 두 팔 충돌, 명령 원자성, 카메라 timestamp 동기화, TurtleBot3 정지 조건, LLM 권한 제한은 제공하지 않는다.

DAPIER는 전체 LeRobot runtime을 복사하지 않는다. 정책·Dataset kernel은 upstream을 추적하고, 아래 항목만 직접 소유한다.

- left→right 고정 12채널 계약
- 팔별 calibration identity
- 전면 manipulation RGB-D와 상단 navigation RGB-D의 독립 timestamp/frame 계약
- TurtleBot3 `base_locked` 조건
- 명령 전 전체 검증과 양팔 충돌 gate
- LLM의 typed skill/arm selection 권한과 raw joint command 금지

이번 변경은 실제 motor bus나 ROS topic을 열지 않는다. 교육용 랩탑의 미동기화 코드와 충돌하지 않도록 hardware-independent contract와 분석 문서만 추가했다.

## 2. 고정한 1차 출처

| 출처 | 고정값 | 사용 범위 |
|---|---|---|
| [LeRobot repository](https://github.com/huggingface/lerobot/tree/4aaff99be4a1d81568c08c8f0296b41b40c99ec4) | `4aaff99be4a1d81568c08c8f0296b41b40c99ec4` | 2026-08-29 checkout |
| [`bi_so_follower.py`](https://github.com/huggingface/lerobot/blob/4aaff99be4a1d81568c08c8f0296b41b40c99ec4/src/lerobot/robots/bi_so_follower/bi_so_follower.py) | 152 lines | 양팔 구성·관측·명령 routing |
| [`config_bi_so_follower.py`](https://github.com/huggingface/lerobot/blob/4aaff99be4a1d81568c08c8f0296b41b40c99ec4/src/lerobot/robots/bi_so_follower/config_bi_so_follower.py) | 36 lines | 두 팔·공유 카메라 설정 |
| [`bimanual.py`](https://github.com/huggingface/lerobot/blob/4aaff99be4a1d81568c08c8f0296b41b40c99ec4/src/lerobot/utils/bimanual.py) | 63 lines | lifecycle 위임 |
| [`so_follower.py`](https://github.com/huggingface/lerobot/blob/4aaff99be4a1d81568c08c8f0296b41b40c99ec4/src/lerobot/robots/so_follower/so_follower.py) | 242 lines | 단일 팔 bus·camera·calibration·action |
| [SO-101 documentation](https://github.com/huggingface/lerobot/blob/4aaff99be4a1d81568c08c8f0296b41b40c99ec4/docs/source/so101.mdx) | 같은 commit | 조립·포트·calibration 실행 문맥 |

빈 줄과 Apache-2.0 license header는 실행 의미가 없으므로 아래 표에서 묶었다. 그 외 runtime line은 전부 범위에 포함했다.

## 3. `bi_so_follower.py` 152줄 해체분석

| line | upstream 동작 | DAPIER 판단 |
|---:|---|---|
| 1 | Python executable shebang | runtime 의미 없음 |
| 2–15 | copyright·Apache-2.0 header | 코드를 복사할 경우 보존 필요. 이번 구현은 구조를 독립 작성했다. |
| 17–18 | logging, cached property import | feature schema cache에만 사용 |
| 20 | `RobotAction`, `RobotObservation` type alias | dict 의미를 강하게 검증하지 않는다. |
| 21 | `BimanualMixin` import | lifecycle을 left/right로 위임한다. |
| 22 | connection-state decorator | 객체 연결 여부만 확인한다. safety authorization이 아니다. |
| 24–26 | Robot, SOFollower, config import | 양팔 로봇은 두 단일 팔의 composition이다. |
| 28 | module logger | 해당 파일에서는 실질적 log 호출이 없다. |
| 31 | `BimanualMixin, Robot` 상속 순서 | mixin method가 MRO에서 Robot보다 앞선다. |
| 32–34 | class docstring | 기체 설명뿐, shared-base 안전 계약은 없다. |
| 36–37 | config class와 registry name | CLI/config factory 식별자다. DAPIER public contract로 복사하지 않는다. |
| 39–41 | base init과 config 저장 | 아직 포트·camera는 열지 않는다. |
| 43–45 | top-level camera key 집합 | 공유 카메라 이름을 unprefixed로 유지한다. |
| 46–52 | 공유·팔별 camera 이름 충돌 검사 | 좋은 경계라 채택한다. DAPIER의 두 공유 카메라는 `manipulation_rgbd`, `navigation_rgbd`로 고정한다. |
| 53 | 공유 camera를 왼팔 config에 합침 | lifecycle 편의를 위한 임의 소유다. DAPIER는 camera를 어느 팔에도 소유시키지 않는다. |
| 55–67 | 왼팔 config 재구성 | id에 `_left`를 붙이고 port/PID/calibration 경로를 복사한다. |
| 69–81 | 오른팔 config 재구성 | `_right` id와 독립 port를 쓴다. 공유 camera는 넣지 않는다. |
| 83–84 | 단일 팔 객체 두 개 생성 | DAPIER도 두 bus/calibration을 독립 장치로 취급한다. |
| 86–87 | 두 camera dict 병합 | key collision 이후 단순 병합이다. capture timestamp 정렬은 없다. |
| 89–97 | motor feature prefix | `left_` 6개 뒤 `right_` 6개 순서를 만든다. 이번 DAPIER 12채널 계약의 직접 비교 지점이다. |
| 99–106 | camera feature prefix | 공유 camera는 그대로, per-arm camera만 side prefix를 붙인다. |
| 108–110 | observation feature cache | motor+camera schema를 한 dict로 노출한다. |
| 112–114 | action feature cache | motor 12개만 action으로 노출한다. camera는 action이 아니다. |
| 116–118 | motor setup 순차 실행 | 왼팔 setup이 끝나야 오른팔을 시작한다. hardware 초기화용이며 동시성은 필요 없다. |
| 120 | 연결 여부 decorator | 두 팔이 연결됐는지만 보며 base stop·E-stop·workspace는 보지 않는다. |
| 121–132 | 관측 수집 | 왼팔과 공유 camera를 먼저 읽고 오른팔을 나중에 읽는다. 하나의 동시 snapshot이 아니다. |
| 134 | 연결 여부 decorator | action 전 safety gate가 아니라 connection gate다. |
| 135–139 | 왼팔 action 추출 | `left_`가 아닌 key는 조용히 무시한다. 필수 6개 completeness 검사도 없다. |
| 140–143 | 오른팔 action 추출 | 동일한 silent-ignore 문제가 있다. |
| 145 | 왼팔 command 전송 | 실제 bus write가 먼저 발생한다. |
| 146 | 오른팔 command 전송 | 실패하면 왼팔만 이미 움직인 partial dispatch가 된다. 원자적 양팔 command가 아니다. |
| 148–150 | 실제 전송값에 prefix 복구 | 팔별 clipping 결과를 다시 합친다. |
| 152 | merged sent action 반환 | measured state가 아니라 command echo다. 성공 판정에 사용하면 안 된다. |

## 4. `config_bi_so_follower.py` 36줄 해체분석

| line | upstream 동작 | DAPIER 판단 |
|---:|---|---|
| 1–15 | shebang·Apache-2.0 | 실행 의미 없음 |
| 17 | dataclass import | 단순 구성값 보관 |
| 19 | camera config import | LeRobot camera backend에 결합 |
| 21–22 | base robot·single arm config import | config composition |
| 25–27 | `bi_so_follower` 등록과 dataclass | factory용 이름 |
| 28 | docstring | 안전·frame 의미 없음 |
| 30–31 | left/right config 필수 | 독립 port/config 원칙은 채택 |
| 33–36 | 공유 camera dict | side에 종속되지 않는 camera를 지원한다. capture clock·extrinsic·용도는 명시하지 않는다. |

DAPIER의 공유 camera 두 대는 이름만 공유 camera로 두지 않고 각각 아래 metadata를 요구한다.

| camera | frame | 역할 | 필수 metadata |
|---|---|---|---|
| `manipulation_rgbd` | `manipulation_camera_link` | 물체 mask, depth, 3D pose, grasp 검증 | RGB/depth source stamp, intrinsics, `base_link` extrinsic, depth validity |
| `navigation_rgbd` | `navigation_camera_link` | RGB-D odometry, semantic map, 먼 물체 보조 탐색 | RGB/depth source stamp, intrinsics, `base_link` extrinsic, localization covariance |

## 5. `BimanualMixin` 63줄 해체분석

| line | upstream 동작 | 위험/채택 여부 |
|---:|---|---|
| 1–15 | license header | 실행 의미 없음 |
| 17–19 | Any와 decorators | concrete arm type을 강제하지 않는다. |
| 22–34 | mixin 계약·MRO 설명 | lifecycle composition만 담당한다는 분리는 좋다. |
| 36–37 | left/right arm placeholder | runtime type guarantee 없음 |
| 39–41 | 두 팔 모두 연결됐을 때만 true | 채택 |
| 43–45 | 두 팔 모두 calibrated일 때만 true | 파일 identity·검증 시각은 별도로 남겨야 한다. |
| 47–50 | left connect 후 right connect | right 실패 시 left cleanup이 보장되지 않는다. DAPIER hardware adapter는 rollback/torque-off receipt를 요구한다. |
| 52–54 | left calibration 후 right calibration | 반드시 팔별 승인·파일·hash를 분리한다. 두 팔 일괄 interactive calibration은 금지한다. |
| 56–58 | left configure 후 right configure | 한 팔만 configure된 partial state를 기록해야 한다. |
| 60–63 | left disconnect 후 right disconnect | left 예외 시 right가 남을 수 있다. hardware layer는 `finally` cleanup을 소유한다. |

## 6. 단일 `SOFollower`에서 양팔 프로젝트에 영향을 주는 줄

| line | 사실 | DAPIER 적용 |
|---:|---|---|
| 46–63 | motor ID 1–6, body degree 또는 normalized, gripper 0–100, Feetech bus와 camera 생성 | 각 팔은 독립 USB bus지만 같은 ordered joint contract를 쓴다. |
| 65–85 | motor/camera observation·action feature 정의 | depth는 `<camera>_depth`; DAPIER ROS bridge에서는 RGB/depth stamp를 분리 보존한다. |
| 87–109 | bus+camera connect, 필요 시 calibration, configure | 자동 interactive calibration을 production launch에서 호출하지 않는다. |
| 111–157 | calibration file 사용 또는 torque-off 후 range 기록 | left/right calibration hash를 episode manifest에 함께 기록한다. |
| 159–171 | position mode, PID, gripper torque/current 보호값 write | 현장 승인 없는 register write를 DAPIER core로 복사하지 않는다. |
| 173–177 | motor별 setup | motor ID 설정은 장치별 유지보수 절차다. |
| 179–202 | arm read 후 camera 순차 read | timestamp 없는 dict를 동시 observation으로 간주하지 않는다. |
| 204–230 | `.pos` key만 추출, optional relative clip, sync write, command echo 반환 | DAPIER는 exact 12 key validation→양팔 safety→dispatch 순서를 강제하고 command echo와 measured observation을 분리한다. |
| 232–238 | bus disconnect 후 camera disconnect | 실패 경로 cleanup evidence가 필요하다. |

## 7. DAPIER 개인화 계약

### 7-1. 12채널 action

순서는 고정한다.

```text
left_shoulder_pan
left_shoulder_lift
left_elbow_flex
left_wrist_flex
left_wrist_roll
left_gripper
right_shoulder_pan
right_shoulder_lift
right_elbow_flex
right_wrist_flex
right_wrist_roll
right_gripper
```

각 팔은 자신의 `sha256:<calibration-json>` identity를 가진다. 한 팔의 calibration을 다른 팔에 재사용하지 않는다. frame의 단일 `calibration_id` 필드에는 left→right 두 hash의 ordered composite hash를 쓰고, 원래 두 hash도 manifest에 보존한다. body action은 degree, gripper는 `0..100`, simulator/ROS core 경계는 radian이라는 기존 DAPIER 의미를 유지한다.

### 7-2. navigation과 manipulation 분리

```text
navigation_rgbd + LDS-02 + odometry
        → localization / Nav2 / pre-grasp base pose
        → base stop + lock confirmation
manipulation_rgbd
        → object pose / grasp candidates / verification
        → arm allocation / safety / SO-101 action
```

조작 action vector에는 TurtleBot3 velocity를 넣지 않는다. 양팔 manipulation episode 동안 `base_locked=true`와 measured zero velocity가 precondition이다.

### 7-3. LLM 권한

허용 출력:

```json
{"skill":"PICK_RIGHT","object_id":"shoe_17","grasp_candidate_id":2}
```

금지 출력:

- raw joint positions
- motor current/PID/torque register
- calibration write
- collision/safety override
- low-confidence 강제 pick

### 7-4. dispatch 순서

```text
exact 12 keys
→ finite/range/calibration/frame/timestamp 확인
→ base_locked 확인
→ left/right IK·reachability
→ swept-volume arm-arm/robot/environment collision
→ 두 팔 command 모두 준비
→ guarded dispatch
→ post-command measured readback
```

LeRobot처럼 왼팔을 먼저 write한 뒤 오른팔을 검증하지 않는다. 실제 serial bus 두 개에 완전한 물리적 원자성은 없으므로, 사전검증을 전부 끝내고 partial dispatch를 fault event로 기록한다.

## 8. 이번에 구현한 코드

| 파일 | 변경 |
|---|---|
| `dapier_sim_first/embodiment.py` | 기존 single-arm `EmbodimentSpec` 두 개를 합성하는 `BimanualEmbodimentSpec` 추가 |
| `dapier_sim_first/__init__.py` | bimanual contract public export |
| `dapier_sim_first/test/test_bimanual.py` | 12채널 순서, 팔별 calibration, round-trip, incomplete action rejection, 기존 frame validator 호환 5개 검사 |
| `dapier_sim_first/README.md` | single 6채널과 dual 12채널 책임 명시 |

이 코드는 hardware driver가 아니다. LeRobot `BiSOFollower.send_action()`을 복사하지 않고, 실제 write 전에 사용할 수 있는 unit/order/calibration contract만 소유한다.

## 9. 학교 랩탑 동기화 뒤 연결할 항목

1. 랩탑의 `git status`, branch, commit, untracked 파일을 먼저 보존한다.
2. 두 SO-101 port·calibration 파일은 Git에 넣지 않고 hash와 logical ID만 기록한다.
3. 현재 학교 코드의 action key/order/unit을 이 12채널 계약과 대조한다.
4. 전면·상단 RGB-D topic, encoding, depth scale, frame, source stamp를 기록한다.
5. 실제 hardware adapter는 12채널 command 전체 validation 후에만 두 bus에 전달한다.
6. 단일 팔 저속 검증→양팔 교대→분리 workspace 동시 실행 순으로 Gate를 연다.
7. collaborative bimanual grasp는 별도 swept-volume/self-collision 검증 전까지 금지한다.

## 10. 평가에서 보여 줄 학습 증거

- upstream 152줄을 그대로 복사하지 않고 각 책임과 빠진 안전 조건을 설명했다.
- 정책·Dataset kernel과 hardware runtime을 분리했다.
- 두 팔 이름·순서·단위·calibration identity를 실행 가능한 테스트로 고정했다.
- 공유 RGB-D 두 대를 임의로 왼팔에 소유시키지 않고 독립 sensor contract로 분리했다.
- LLM을 관절 제어기가 아니라 typed task/skill selector로 제한했다.
- 아직 동기화되지 않은 실물 코드를 완료된 것으로 주장하지 않았다.
