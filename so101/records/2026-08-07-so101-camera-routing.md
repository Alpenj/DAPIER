# SO-101 카메라 기준과 IK→VLA 분리 실습 기록

`record_id: DAPIER-2026-08-07-so101-camera-routing`

## 오늘 확인하려는 것

오늘 수업에서 두 가지를 이어서 확인하고 있다.

1. gripper 위 카메라가 실제 SO-101 mount와 같은 rigid motion을 하도록 sim과
   ROS 2 frame 기준을 맞춘다.
2. top view가 있는 데이터 수집에서는 IK를 teacher로 쓰고, top view가 없는
   wrist-only 실행에서는 VLA만 쓰도록 경로를 분리한다.

이번 작업은 simulation과 URDF/TF 정적 검증만 대상으로 한다. 실제 모터를
연결하거나 명령하지 않는다.

## 기존 파일을 직접 읽어 본 결과

`$HOME/so101_ros2_ws/src/so101-ros-physical-ai`에는
`so101_description/urdf/so101_cameras.xacro`가 이미 있었다. 그러나 wrist
camera 기본값 `(0, 0, -0.02) m`, `(-1.5708, 0, -1.5708) rad`는 실측값이
아니라 사용자가 맞추는 예시였다. 더 중요한 점은 parent가 움직이는 jaw여서
그리퍼 개폐에 따라 camera frame도 돌아간다는 것이다.

TheRobotStudio의 공식 [SO-ARM100 저장소](https://github.com/TheRobotStudio/SO-ARM100)도
모든 SO-101에 공통인 카메라 extrinsic 하나를 제공하지 않는다. 32×32 UVC
module, hex-nut mount, webcam, RealSense D405/D435 등 서로 다른 optional mount가
있다. 그래서 이번에는 integrated 32×32 UVC module mount를 구체적인 기준으로
선택하고 다른 mount에도 맞는 값이라고 일반화하지 않는다.

현재 PC의 USB video 장치는 노트북 내장 ASUS FHD camera뿐이고 실제 SO-101
wrist camera는 없다. 따라서 CAD에서 알 수 있는 장착면과 실제 lens의 optical
center를 구분한다.

## CAD 장착면에서 만든 카메라 계약

기준 STL은 다음 파일이다.

- source revision: `7629d2ad9853d10fb903093a33ef6114099d97e5`
- path: `Optional/Wrist_Cam_Mount_32x32_UVC_Module/stl/` 아래 SO-101 STL
- SHA-256: `b4345ccf23f1f2ed3f4885c205cac5afbed6ddd1b183617c4801751e3bafb7b4`

STL의 35 mm camera-board mounting face 중심과 법선을 공식 SO-101 MJCF의
wrist-roll mesh pose로 변환했다. 같은 물리 pose를 MuJoCo와 ROS 좌표 표현에
각각 기록한다.

| 항목 | 값 |
|---|---|
| rigid parent | MuJoCo `gripper`, ROS `gripper_link` |
| local position | `(0.0025, -0.072057361, 0.004150235) m` |
| MuJoCo camera X axis | `(1, 0, 0)` |
| MuJoCo camera Y axis | `(0, 0.906307787, 0.422618262)` |
| optical look direction | `(0, 0.422618262, -0.906307787)` |
| ROS `camera_link` rpy | `(0, 1.134464014, 1.570796327) rad` |
| profile id | `therobotstudio_integrated_32x32_mount_surface_v1` |

MuJoCo profile은
`so101/integrations/lerobot_v0_6_so101_mujoco/overlay/.../camera_profiles.json`에
두고 source revision, hash와 검증 상태를 같이 저장한다. wrist camera가 없는
원본 robot MJCF는 그대로 두고 environment load 시 `gripper` body에 camera를
생성한다. ROS 2 변경은 별도
`integrations/so101_ros_physical_ai_camera/so101-camera-profile.patch`로 보존한다.

현재 `vertical_fov_degrees=75`는 sim-only 값이다. 실제 lens offset, 실제 board
방향, image rotation, intrinsics와 FOV를 측정하지 못했으므로 profile의
`physical_alignment`는 `false`다. 지금 맞춘 것은 **mount가 gripper와 같이
움직이는 rigid transform**이며 실제 pixel view가 이미 같다는 뜻은 아니다.

## top+wrist IK teacher 경로

기본 policy camera set을 `top,wrist`로 바꿨다. viewer의 `Shift+C`는
external → top → wrist를 순환한다. `Shift+V`를 누르면 다음 순서로 실행한다.

1. 새로운 random seed로 reset한다.
2. 팔을 top camera가 cube를 가리지 않는 관찰 자세로 옮긴다.
3. top RGB의 blue mask와 camera calibration, 알려진 cube top plane만으로
   world XY를 계산한다.
4. 그 XY로 IK pick-and-place trajectory를 만든다.
5. expert recording이면 top RGB, wrist RGB, measured state와 commanded action을
   같은 frame에 저장한다.

planner 입력에는 cube body pose, depth, segmentation id를 넣지 않는다. recorder는
`Shift+V` automation 중인 frame만 받으므로 수동 조작이 IK demonstration에
섞이지 않는다. dataset에는 `meta/dapier_control_route.json`을 추가해 teacher가
top RGB IK이고 student가 wrist-only VLA라는 관계를 기록한다.

## wrist-only VLA student 경로

camera set이 `wrist` 하나뿐이면 route는 VLA다. 이 경로는 IK로 fallback하지
않고 `--input policy --policy-path ...`가 없으면 즉시 거부한다. checkpoint가
있을 때는 LeRobot 표준 `lerobot_eval`에 wrist image만 전달한다.

IK teacher dataset에서 student dataset을 만드는 명령은
`lerobot_edit_dataset`으로 `observation.images.top`만 제거한다. 그 다음
`lerobot_train --policy.type=smolvla`로 wrist image, state, language task와
action을 학습하고, wrist-only evaluator로 되돌려 실행한다. 이 세 명령은 코드
builder와 test로 고정했다.

아직 실제 expert dataset을 수집하지 않았고 SmolVLA checkpoint도 학습하지
않았다. parser probe에서는 dataset/checkpoint argument가 정상 해석된 뒤 일부러
지정한 없는 경로를 읽는 단계에서 멈추는 것까지만 확인했다. 따라서 VLA 성공률은
아직 결과로 적지 않는다.

## 직접 실행해 본 결과

| 검증 | 결과 |
|---|---:|
| Ruff check / format check | `PASS`, 13 files |
| SO-101 MuJoCo test | `31/31 PASS` |
| top RGB IK, random seed `0..9` | `10/10 PASS` |
| top RGB cube XY 오차 | 평균 `0.670 mm`, 최대 `0.939 mm` |
| upstream SO-101 MJCF/STL hash | `14/14 OK` |
| clean LeRobot v0.6.0 + patch + overlay | `31/31 PASS` |
| ROS Xacro → URDF / `check_urdf` | `PASS` |
| ROS optical direction 수치 오차 | `4.69e-10` |
| ROS launch Python `compileall` | `PASS` |
| `colcon build` | `2 packages finished` |

MuJoCo test에는 camera profile provenance, wrist camera parent, 카메라가 gripper와
함께 움직이는지, top/wrist render, fail-closed route, dataset sidecar, dataset
derivation/train/eval command와 다섯 random pick-and-place seed가 포함된다. 별도
10-seed batch도 다시 실행했다.

ROS 검증에서는 Xacro를 펼쳐 `wrist_camera_link`가 `gripper_link`의 child인지와
standard optical-frame joint까지 합친 축을 검사했다. `so101_description`과
`so101_bringup`만 빌드했고 launch나 hardware driver는 실행하지 않았다.

## 아직 확인하지 못한 부분과 다음에 할 것

- 실제로 사용할 32×32 camera module의 정확한 모델과 mount 조립 방향을 정한다.
- 정지한 실제 arm에서 calibration target을 촬영해 lens optical center,
  `gripper_link -> camera_optical_frame`, intrinsics, distortion와 image rotation을
  측정한다.
- 그 측정값으로 JSON과 ROS Xacro를 함께 갱신한 뒤 sim/real 같은 pose에서
  reprojection 오차를 비교한다.
- top+wrist IK successful episode를 새 외부 dataset에 수집하고 top feature만
  제거한 wrist-only dataset의 feature/schema를 다시 검사한다.
- SmolVLA를 실제로 학습해 고정 seed에서 success rate를 보고한다.

지금 결과는 CAD mount 기준 sim controller 성공이다. physical camera alignment,
learned VLA, sim-to-real 또는 실제 SO-101 제어 성공으로 승격하지 않는다.
