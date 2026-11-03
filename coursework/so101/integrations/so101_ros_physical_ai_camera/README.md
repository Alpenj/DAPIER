# SO-101 ROS 2 손목 카메라 프레임 patch

`record_id: DAPIER-2026-08-07-so101-ros-camera-frame`

오늘 수업에서 `$HOME/so101_ros2_ws/src/so101-ros-physical-ai`의 카메라
Xacro를 직접 확인해 보니 wrist camera frame은 이미 있었지만 기본 위치는
`(0, 0, -0.02) m`인 사용자 조정용 예시값이었다. parent도
`moving_jaw_so101_v1_link`라서 그리퍼를 여닫으면 실제 고정 카메라와 다르게
frame이 움직였다.

이 디렉터리에는 upstream 전체를 복사하지 않고, 아래 기준 checkout에 적용할
작은 patch만 보존한다.

- upstream: `https://github.com/legalaspro/so101-ros-physical-ai.git`
- base revision: `58318c905a2c61289fa907de85cb8473322fbe68`
- upstream license: Apache-2.0 (`LICENSE.apache-2.0.txt`)
- patch: `so101-camera-profile.patch`

## 적용한 카메라 계약

공식 SO-101 저장소에는 하나의 보편적인 wrist camera URDF extrinsic이 없고,
서로 다른 카메라용 mount CAD가 여러 개 있다. 이번에는 TheRobotStudio의
`Wrist_Cam_Mount_32x32_UVC_Module_SO101.stl`을 기준 형상으로 선택했다.

- SO-ARM100 revision: `7629d2ad9853d10fb903093a33ef6114099d97e5`
- STL SHA-256:
  `b4345ccf23f1f2ed3f4885c205cac5afbed6ddd1b183617c4801751e3bafb7b4`
- ROS parent: `gripper_link`
- `camera_link` xyz: `0.0025 -0.072057361 0.004150235` m
- `camera_link` rpy: `0.0 1.134464014 1.570796327` rad
- standard optical-frame joint까지 합친 local look direction:
  `(0, 0.422618262, -0.906307787)`

이 값은 공식 wrist-roll mesh pose와 32×32 mm camera-board mounting face의
중심·법선을 MuJoCo `gripper` frame으로 변환해 얻고, 같은 rigid transform을
ROS `gripper_link` frame 표현으로 바꾼 것이다. MuJoCo 쪽 정본은 LeRobot
integration overlay의 `assets/camera_profiles.json`이다.

## 아직 실제 카메라와 같다고 말할 수 없는 부분

이번 값은 **카메라 보드 장착면** 기준이다. 현재 PC에는 실제 SO-101 wrist
camera가 연결되어 있지 않아 lens optical-center offset, 보드가 mount에 들어가는
방향, 영상 회전, 해상도별 intrinsics와 실제 FOV를 측정하지 못했다. 따라서
`physical_alignment_verified=false`이며, 지금 sim 데이터는 CAD mount가 움직이는
방식만 맞춘 상태다.

실제 모듈이 정해지면 정지한 팔에서 checkerboard/AprilTag로
`gripper_link -> camera_optical_frame`을 측정하고, 같은 측정값으로 ROS Xacro와
MuJoCo JSON을 함께 갱신해야 한다. 그 검증 전에는 이 데이터를 실제 카메라와
pixel-level로 동일하다고 사용하지 않는다.

## 깨끗한 checkout에 적용

```bash
export ROS_CAMERA_PATCH_ROOT=/path/to/DAPIER/so101/integrations/so101_ros_physical_ai_camera
export SO101_ROS_ROOT=/path/to/so101-ros-physical-ai

git -C "$SO101_ROS_ROOT" checkout --detach \
  58318c905a2c61289fa907de85cb8473322fbe68
git -C "$SO101_ROS_ROOT" apply --check \
  --unidiff-zero \
  "$ROS_CAMERA_PATCH_ROOT/so101-camera-profile.patch"
git -C "$SO101_ROS_ROOT" apply \
  --unidiff-zero \
  "$ROS_CAMERA_PATCH_ROOT/so101-camera-profile.patch"
```

## 오늘 직접 검증한 범위

ROS 2 Jazzy 환경에서 wrist camera를 켠 Xacro를 URDF로 펼치고
`check_urdf`로 tree를 읽었다. `wrist_camera_link`가 `gripper_link`의 child인지,
optical frame의 local X/Y/Z 축과 위 look direction이 MuJoCo profile과 일치하는지
수치로 확인했다. 이어서 다음 두 package만 빌드했다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_description so101_bringup --symlink-install
```

결과는 `2 packages finished`였고 launch Python도 `compileall`을 통과했다. 이
검증에서는 ROS launch, serial 접속, controller 활성화, 모터 명령을 실행하지
않았다.
