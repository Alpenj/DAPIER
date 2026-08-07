# SO-101 손목 RGB pick-and-place 실습 기록

`record_id: DAPIER-2026-08-07-so101-wrist-vision-pick-place`

## 오늘 확인하려던 것

오늘 수업에서 만든 SO-101 MuJoCo 장면에 eye-in-hand 카메라를 직접 붙이고,
카메라 RGB 영상으로 파란 cube를 찾은 뒤 집어서 녹색 tray로 옮기는 흐름을
확인하고 있다. 이번 실습은 simulation만 대상으로 하며 실제 SO-101이나 ROS 2
hardware backend는 열지 않았다.

## 검은 판처럼 보이던 형상

노란 SO-101 CAD mesh만으로는 손가락과 cube의 접촉이 불안정했다. 그래서 양쪽
손가락 안쪽에 box contact geom을 추가했는데, 처음에는 이 전체를 검은색으로
표시해 고무 pad가 아니라 큰 판처럼 보였다.

직접 렌더를 비교한 뒤 다음처럼 분리했다.

- 물리에 쓰는 contact envelope: `55 x 40 x 4 mm`, 투명, collision 활성
- 화면에 보이는 rubber lining: `36 x 13 x 4.2 mm`, 검은색, collision 비활성

즉 검은 부분은 손가락 안쪽의 단순화한 고무 라이닝이다. 실제 SO-101 pad 형상을
정밀하게 복제한 것은 아니며, 보이지 않는 큰 envelope가 transfer 중 cube를
받치는 한계는 남아 있다.

## 카메라와 RGB 위치 추정

카메라를 그리퍼 정중앙에 두면 fixed finger가 cube를 가렸다. 그래서 그리퍼
local frame 기준 `(0.05, -0.07, 0.04) m`에 위·측면 offset을 두고 작업점을
내려다보게 했다. 카메라 body id가 gripper body id와 같은지 확인해 팔을 따라
움직이는 것도 검증했다. 화면에서는 노란 지지대 끝의 검은 box가 카메라
housing이고, 작은 청록색 원판이 lens다. 이 mount와 housing은 collision을 만들지
않는다.

cube 위치 추정에는 다음 정보만 사용한다.

1. wrist RGB에서 파란색 mask의 bounding box 중심
2. 현재 robot state로 정해지는 wrist camera pose와 `fovy`
3. work surface와 cube 크기로 미리 정한 cube top plane 높이

simulator의 cube body pose, depth, segmentation id, contact 정보는 planner 입력에
사용하지 않는다. RGB pixel ray를 알려진 top plane과 교차해 world XY를 구한 뒤,
그 위치에 맞춘 joint trajectory를 만든다.

## viewer에서 실행하는 방법

```bash
cd "$HOME/so101/lerobot"
.venv/bin/python examples/so101_mujoco/teleoperate.py \
  --input keyboard --seed 101 --cube-randomization 0.025
```

- `Shift+V`: episode를 원자적으로 reset하고 30 frame 동안 안정화한 뒤 wrist
  RGB로 cube를 찾아 pick-and-place를 실행한다.
- `Shift+C`: external view와 wrist camera view를 바꾼다.
- `Shift+P`: reset 뒤 기존 scripted pick-and-lift만 실행한다.
- 자동 동작이 끝나면 physics를 pause해 결과를 확인할 수 있다.

`Shift+V`를 누르면 viewer도 wrist camera로 전환한다. 외부에서 전체 팔과 tray를
다시 보려면 `Shift+C`를 누른다.

## 직접 실행해 본 결과

| 검증 | 결과 |
|---|---:|
| SO-101 MuJoCo test | `22/22 PASS` |
| randomized seed `0..29` RGB pick-and-place | `30/30 PASS` |
| wrist RGB XY 오차 | 평균 `2.715 mm`, 최대 `6.459 mm` |
| 최종 tray 중심 XY 오차 | 평균 `34.324 mm`, 최대 `45.496 mm` |
| seed 101, 640x480 wrist RGB XY 오차 | `2.050 mm` |
| camera parent | `wrist cam body == gripper body` |
| upstream asset SHA-256 | `14/14 OK` |
| clean v0.6.0 + patch + overlay 재구성 test | `22/22 PASS` |

30개 seed 검증에서는 cube XY를 각각 `+-25 mm` 범위에서 무작위로 놓았다. 성공
판정은 tray 중심과 cube 중심의 XY 거리가 `55 mm`보다 작고 cube 높이가 tray
범위 안에 있는 조건이다. Gymnasium의 비대칭·비정규화 action space 권고 warning
1건은 그대로 남아 있다.

## 아직 확인하지 못한 부분

현재 방식은 한 장의 RGB로 위치를 추정한 뒤 정해진 trajectory를 실행한다. 이동
중 재검출하는 visual servo, 조명 변화에 강한 detector, 다른 색과 크기의 물체,
camera noise와 calibration drift는 아직 확인하지 않았다. 보이지 않는 contact
envelope도 실제 finger pad 물성으로 바꿔야 한다.

이번 `30/30`은 이 MuJoCo 장면과 `+-25 mm` 범위의 결과다. learned policy 성공이나
sim-to-real 성공으로 기록하지 않는다. 실제 하드웨어 제어도 진행하지 않았다.
