# MuJoCo RGB-D 기반 신발 접근 좌표

`vision_guided_reach.py`는 MuJoCo object body pose를 IK target으로 읽지 않는다.
다음 경로로 접근 좌표를 만든다.

```text
workspace_depth_camera RGB
  -> image detector mask
aligned metric depth
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

## DAPIER 장면 프로필

기존 tower camera의 27° 설정은 장거리 SLAM 시야와 구조 검증을 위해 만들어졌고,
기본 바닥 신발의 영상 기반 접근을 검증하기에는 근거리 하단 시야가 부족했다.
`dapier_vision_demo.py`는 기본 신발을 실제 RGB-D frustum 안에 넣기 위한 **35° 하향
simulation profile**을 별도로 사용한다.

이 각도는 시뮬레이션에서 검증된 provisional 값이다. 실제 작업영역 H201 장착 각도와 optical
origin을 측정하기 전에는 실물 calibration 값으로 간주하지 않는다. 프로필은 모델
생성 중에만 적용되고 기존 27° 상수를 변경하거나 다른 테스트에 누출하지 않는다.

## 사람이 먼저 확인할 artifact 생성

```bash
cd ~/DAPIER
export PYTHONPATH="$PWD/2ARM_ROBOT/research/src"

python3 2ARM_ROBOT/sim/mobile_dual_so101/dapier_vision_demo.py \
  --artifact-dir /tmp/dapier-vision \
  --artifact-only
```

생성 파일:

```text
/tmp/dapier-vision/vision_rgb.png
/tmp/dapier-vision/vision_mask.png
/tmp/dapier-vision/vision_target_report.json
```

확인 기준:

- `vision_rgb.png`에서 바닥 신발이 실제 카메라 시야에 보인다.
- `vision_mask.png`의 흰 영역이 신발 upper를 따라간다.
- 보고서의 `estimate.source`는 `rgb_detection_plus_aligned_metric_depth`다.
- `estimate.ground_truth_used`, `ground_truth_used_for_target`,
  `control_authorized`, `hardware_execution`은 모두 `false`다.

## IK 제안까지 실행

```bash
python3 2ARM_ROBOT/sim/mobile_dual_so101/dapier_vision_demo.py \
  --side left \
  --width 320 \
  --height 240
```

기본 floor shoe는 현재 tower-mounted arm reach 밖에 있을 수 있어 IK 또는 collision
단계에서 거부될 수 있다. 이는 정상적인 fail-closed 결과다. target 좌표를 simulator
truth로 대체하지 말고, base navigation/docking으로 reach envelope 안에 들어온 뒤
같은 sensor path로 다시 추정해야 한다.

## 전체 테스트와 CI artifact

```bash
python3 -m unittest discover -s 2ARM_ROBOT/research/test -v
scripts/verify-mujoco-headless --artifact-dir /tmp/dapier-mujoco-artifacts
```

`verify-mujoco-headless`는 순수 Python RGB-D 수식 테스트, MuJoCo adapter 테스트,
기존 simulation 테스트를 실행하고 다음 review artifact도 함께 만든다.

```text
/tmp/dapier-mujoco-artifacts/vision/vision_rgb.png
/tmp/dapier-mujoco-artifacts/vision/vision_mask.png
/tmp/dapier-mujoco-artifacts/vision/vision_target_report.json
```

MuJoCo 테스트에서 target body pose는 sensor reconstruction error를 측정하는 test-only
oracle로만 사용한다. planner source에는 object body 이름, ground-truth observation과
segmentation 기반 runtime target 경로를 두지 않는다.

## 아직 포함하지 않는 것

- 일반 신발 detector 학습과 정확도 평가
- 그리퍼 RGB 카메라의 PnP/multi-view 또는 image-based visual servoing
- base docking 이후 실제 reachable pose와 연속 재계획
- 최종 접촉·파지·미끄럼 검증
- 실제 작업영역 H201 intrinsics/extrinsics calibration
- C++ hardware dispatch와 실물 로봇 실행

따라서 이 단계는 **sensor-derived coarse target과 안전한 IK 제안 경로**이며 실제
바닥 신발 grasp 성공을 의미하지 않는다.
