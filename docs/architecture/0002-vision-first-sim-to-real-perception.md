# ADR 0002 — 센서 기반 비전 좌표를 사용하는 sim-to-real 인식 경로

- 상태: Accepted
- 적용일: 2026-09-02
- 범위: MuJoCo 신발 접근, 실제 RGB-D/RGB 인식, Python 연구 계층과 C++ 제어 경계

## 배경

MuJoCo는 물체의 정확한 world pose와 geom/body ID를 즉시 제공한다. 이 값으로 IK
목표를 만들면 시뮬레이터에서는 쉽게 동작하지만 실제 로봇에는 같은 정보가 없다.
정책이나 planner가 이 privileged state에 의존하면 시뮬레이션 성능이 좋아 보여도
실제 카메라 오차, 가림, 깊이 결측, 보정 오차와 detector 실패를 학습하지 못한다.

실제 TurtleBot3 + 양팔 SO-101은 전방 RGB-D 카메라와 그리퍼 RGB 카메라가 관찰한
영상으로 물체를 찾아야 한다. 따라서 시뮬레이션도 가능한 한 같은 관측 계약을
사용해야 한다.

## 결정

### 1. runtime target은 영상에서만 생성한다

정책 추론과 접근 경로 계획에 들어가는 물체 위치는 다음 입력으로 계산한다.

```text
RGB image
  -> detector mask / bbox / keypoint
aligned metric depth
  -> optical-frame point cloud or robust visible-surface centroid
camera intrinsics + calibrated camera-to-robot transform
  -> base/manipulation frame target + covariance + confidence
```

현재 기준 optical frame은 ROS 관례를 사용한다.

- `+X`: 영상 오른쪽
- `+Y`: 영상 아래
- `+Z`: 카메라 전방

MuJoCo 카메라는 local `-Z` 방향을 보기 때문에 adapter가 축 변환을 명시적으로
적용한다. 물체의 body pose는 이 변환에 사용하지 않는다. 카메라와 robot/base frame
사이의 transform만 사용하며, 이는 실제 장비의 TF 또는 외부 보정값에 대응한다.

### 2. privileged simulator state의 허용 범위를 제한한다

MuJoCo object pose, geom ID와 segmentation ID는 다음 용도에만 허용한다.

- 초기 scene 배치와 domain randomization
- reward, 성공 판정과 안전한 테스트 assertion
- sensor estimate의 test-only 오차 측정
- detector 학습용 offline label 생성

다음 용도에는 금지한다.

- runtime policy observation의 물체 좌표
- IK 또는 grasp target 생성
- detector의 runtime mask 생성
- confidence gate 우회

`vision_guided_reach.py`에는 object body 이름이나 ground-truth observation import를
두지 않는다. 회귀 테스트가 이 금지 경계를 검사한다.

### 3. Python 연구 계층이 인식과 계획을 소유한다

`2ARM_ROBOT/research/src/dapier_research/vision_target.py`는 MuJoCo와 하드웨어에
독립적인 RGB-D 투영 수식과 불확실성 계약을 소유한다. 입력은 detector mask,
aligned metric depth, intrinsics와 4x4 보정 transform이다.

`2ARM_ROBOT/sim/mobile_dual_so101/vision_guided_reach.py`는 MuJoCo 전용 adapter다.
렌더링한 RGB-D, 카메라 pose와 base frame transform을 research 계층에 전달하고,
센서로 추정한 target만 IK에 전달한다.

Python 결과는 여전히 제안이다. IK와 collision 검사를 통과하면 versioned
`ControlIntent`를 만들 수 있지만 실제 하드웨어 실행을 승인하지 않는다.

### 4. C++ 계층이 실제 명령의 최종 권한을 가진다

C++ 실시간 제어 계층은 Python이 보낸 joint intent를 다음 순서로 다시 검사한다.

```text
schema -> sequence -> receiver-local TTL -> enable/interlock
       -> joint/base limits -> rate limits -> watchdog -> hardware I/O
```

비전 confidence와 IK 성공은 C++ 안전 검사를 대체하지 않는다. Python이 만든 intent에
`hardware_authorized`, device path, motor ID 또는 register write를 넣지 않는다.

### 5. RGB-only 카메라에서 가짜 metric 좌표를 만들지 않는다

aligned depth가 없는 RGB 카메라는 한 장의 영상만으로 일반 물체의 절대 거리와 3D
위치를 결정할 수 없다. RGB-only 경로는 다음 중 하나가 검증될 때만 metric target을
만든다.

- 알려진 3D keypoint/크기와 PnP
- calibrated multi-view 또는 stereo
- base/arm motion을 이용한 triangulation
- 검증된 monocular depth와 별도 uncertainty gate
- image-based visual servoing처럼 metric 위치를 요구하지 않는 제어

현재 PR은 전방 RGB-D 기반 coarse approach를 구현한다. 그리퍼 RGB 카메라의
fine alignment와 최종 grasp는 후속 단계이며, 완료된 것으로 주장하지 않는다.

## 현재 구현

- 범용 `PinholeIntrinsics`, `PixelDetection`, `VisionTargetEstimate`
- mask 내부 유효 깊이 비율 검사
- median/MAD 기반 depth outlier 제거
- visible-surface 3D 좌표와 covariance/confidence 계산
- low-confidence 및 privileged-label fail-closed 처리
- MuJoCo camera frame에서 ROS optical frame으로 명시적 축 변환
- sensor-derived target -> IK -> collision check -> research intent 제안
- 정책 관측에서 object ground truth를 제거한 `vision-policy-observation.v1`

현재 `SimBlueShoeDetector`는 파란색 primitive 신발을 위한 deterministic RGB fixture다.
MuJoCo geom/body ID를 사용하지 않지만, 실제 환경의 일반화 detector는 아니다.
물리 배포 전에는 학습된 detector/keypoint model과 실제 RGB-D calibration으로
교체해야 한다.

## 검증 계약

장비 없이 다음을 확인한다.

- 순수 NumPy RGB-D projection과 transform 수식
- depth 결측과 outlier 처리
- simulator segmentation 기반 detection 거부
- MuJoCo render의 blue target을 sensor path로 재구성
- MuJoCo camera/ROS optical 축 변환
- policy observation에 object truth가 없는지
- sensor target이 IK 입력으로 전달되는지
- intent가 hardware authorization을 포함하지 않는지

실제 카메라 calibration, detector 정확도, grasp 성공률, occlusion robustness와
hard real-time 성능은 이 ADR 또는 시뮬레이션 테스트로 검증되지 않는다.
