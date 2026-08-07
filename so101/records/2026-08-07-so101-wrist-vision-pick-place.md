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

처음에는 보이지 않는 큰 collision envelope와 작은 검은 visual을 분리했지만,
직접 접촉 위치를 추적해 보니 화면과 물리가 다르게 보이는 원인이 됐다. 이번에는
그 구조를 제거하고 한 손가락당 box 하나만 남겼다.

- fixed pad: `pos=(-0.0109, -0.0002221, -0.097517) m`
- moving pad: `pos=(-0.0093, -0.0750583, 0.0188972) m`
- 두 pad 크기: `24 x 16 x 6 mm`
- 검은 visual과 collision: 같은 geom

접촉면은 STL에서 측정한 양쪽 fingertip 평면에 맞췄다. 검은 부분은 실제 제품
pad를 정밀 복제한 것이 아니라 평평한 rubber contact proxy이지만, 적어도 보이는
검은 면과 MuJoCo가 접촉시키는 면은 같다.

또 직접 확인해 보니 upstream의 fingertip collision mesh는 오목한 CAD 그대로가
아니라 convex hull로 계산됐다. 그 결과 눈으로는 빈 공간인데 cube contact가
발생했다. `gripper`와 `moving_jaw_so101_v1` body의 collision용 mesh 사본만 끄고,
보이는 CAD mesh와 위의 검은 pad collision을 남겼다.

회색 시작 tray의 높은 벽은 들어 올린 cube 모서리에 걸렸다. 벽 collision만 끄지
않고, 화면과 물리 양쪽에서 같은 `12 mm` 높이의 낮은 rim으로 바꿨다. floor 위로
올라오는 실제 rim 높이는 약 `6 mm`다.

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
  --input keyboard --seed 101
```

- `Shift+V`: episode를 원자적으로 reset하고 30 frame 동안 안정화한 뒤 wrist
  RGB로 cube를 찾아 pick-and-place를 실행한다.
- `Shift+N`: 동작 없이 다음 random scene으로 reset한다.
- `Shift+C`: external view와 wrist camera view를 바꾼다.
- `Shift+P`: reset 뒤 기존 scripted pick-and-lift만 실행한다.
- 자동 동작이 끝나면 physics를 pause해 결과를 확인할 수 있다.

`Shift+V`를 누르면 viewer도 wrist camera로 전환한다. 외부에서 전체 팔과 tray를
다시 보려면 `Shift+C`를 누른다.

cube randomization 기본값은 `+-25 mm`다. `--seed 101`로 시작하면 초기 scene은
101이고, 첫 `Shift+V` 또는 `Shift+N`은 102, 다음 reset은 103을 쓴다. 모든 reset이
같은 sequence를 공유하므로 첫 `Shift+V`가 초기 cube 위치를 다시 만드는 현상은
없다. 고정 위치가 필요할 때만 `--cube-randomization 0`을 준다.

## 직접 실행해 본 결과

| 검증 | 결과 |
|---|---:|
| SO-101 MuJoCo test | `24/24 PASS` |
| randomized seed `101..130` RGB pick-and-place | `30/30 PASS` |
| wrist RGB XY 오차 | 평균 `3.08 mm`, 최대 `6.55 mm` |
| 최종 tray 중심 축별 오차 | 최대 `< 50 mm` |
| live GUI, 초기 seed 101 뒤 첫 `Shift+V` | seed `102`, `PASS`, cube `(0.2185, 0.1528, 0.0388) m` |
| camera parent | `wrist cam body == gripper body` |
| upstream asset SHA-256 | `14/14 OK` |
| clean v0.6.0 + patch + overlay 재구성 test | `24/24 PASS` |

30개 seed 검증에서는 cube XY를 각각 `+-25 mm` 범위에서 무작위로 놓았다. 녹색
tray가 원형이 아니라 사각형이므로 성공 판정도 중심 거리 원이 아니라 실제 usable
square를 따른다. cube 중심의 X/Y 축별 오차가 각각 `50 mm`보다 작고 높이가 tray
범위 안이면 성공이다. Gymnasium의 비대칭·비정규화 action space 권고 warning
1건은 그대로 남아 있다.

## 아직 확인하지 못한 부분

현재 방식은 한 장의 RGB로 위치를 추정한 뒤 정해진 trajectory를 실행한다. 이동
중 재검출하는 visual servo, 조명 변화에 강한 detector, 다른 색과 크기의 물체,
camera noise와 calibration drift는 아직 확인하지 않았다. 현재 검은 pad도 실제
제조 형상과 재료 시험값이 아니라 CAD 접촉 평면에 맞춘 단순 box proxy다.

이번 `30/30`은 이 MuJoCo 장면과 `+-25 mm` 범위의 결과다. learned policy 성공이나
sim-to-real 성공으로 기록하지 않는다. 실제 하드웨어 제어도 진행하지 않았다.
