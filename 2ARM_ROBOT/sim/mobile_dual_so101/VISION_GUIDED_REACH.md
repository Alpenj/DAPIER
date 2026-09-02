# MuJoCo RGB-D 기반 신발 접근 좌표

`vision_guided_reach.py`는 MuJoCo object body pose를 IK target으로 읽지 않는다.
다음 경로로 접근 좌표를 만든다.

```text
front_depth_camera RGB
  -> SimBlueShoeDetector mask
aligned depth
  -> ROS optical-frame visible-surface points
camera intrinsics + camera-to-base transform
  -> base-frame target + confidence/covariance
approach offset
  -> MuJoCo world adapter -> IK -> collision check -> ControlIntent proposal
```

`SimBlueShoeDetector`는 현재 파란색 primitive 신발을 찾기 위한 deterministic
simulation fixture다. geom/body ID나 segmentation image를 사용하지 않지만 실제
신발 detector는 아니다. 실제 카메라에서는 학습된 detection/keypoint model과
측정된 intrinsics/extrinsics로 교체한다.

## 실행

먼저 저장소의 pinned SO-101 asset을 준비하고 MuJoCo 의존성을 설치한다. 그다음:

```bash
cd ~/DAPIER
export PYTHONPATH="$PWD/2ARM_ROBOT/research/src"
python3 2ARM_ROBOT/sim/mobile_dual_so101/vision_guided_reach.py \
  --side left \
  --width 160 \
  --height 120
```

출력에서 확인할 값:

- `estimate.source = rgb_detection_plus_aligned_metric_depth`
- `estimate.ground_truth_used = false`
- `ground_truth_used_for_target = false`
- `control_intent`에 `hardware_authorized` 또는 device 정보가 없음
- `hardware_execution = false`

기본 floor shoe가 현재 arm reach 밖이면 IK가 거부될 수 있다. 이는 target 좌표를
simulator truth로 대체할 이유가 아니다. base navigation/docking으로 reach envelope에
들어온 뒤 같은 sensor path로 다시 추정해야 한다.

## 테스트

```bash
python3 -m unittest discover -s 2ARM_ROBOT/research/test -v
scripts/verify-mujoco-headless --artifact-dir /tmp/dapier-mujoco-artifacts
```

MuJoCo 테스트에서 target body pose는 sensor reconstruction error를 측정하는 test-only
oracle로만 사용한다. planner source에는 object body 이름, ground-truth observation과
segmentation 기반 runtime target 경로를 두지 않는다.

## 아직 포함하지 않는 것

- 일반 신발 detector 학습과 정확도 평가
- 그리퍼 RGB 카메라의 PnP/multi-view 또는 image-based visual servoing
- 최종 접촉·파지·미끄럼 검증
- 실제 Astra intrinsics/extrinsics calibration
- C++ hardware dispatch와 실물 로봇 실행

따라서 이 단계는 **sensor-derived coarse approach/IK 제안**이며 실제 grasp 성공을
의미하지 않는다.
