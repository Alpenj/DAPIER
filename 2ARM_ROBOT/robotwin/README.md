# RoboTwin Dual SO-101

이 폴더는 내가 RoboTwin 2.0을 Dual SO-101에 적용하며 확인한 코드와 기록의 정본이다.
`~/RoboTwin`은 공식 upstream 실행 환경이고, 재현할 변경은 이 폴더의 installer·overlay·patch에만 둔다.

## 현재 확인한 범위

2026-09-05에 `dapier_handover_block` 한 episode를 SAPIEN 물리 접촉으로 끝까지 실행했다.
왼팔이 지름 30 mm, 길이 160 mm인 원기둥을 집어 들어 오른팔에 전달하고 왼팔이 빠진 뒤에도
오른쪽 두 jaw가 물체를 서로 반대 방향으로 누르는 것을 성공 조건으로 삼았다. equality constraint나
물체 강제 부착은 사용하지 않았다.

이 결과는 **SIM 성공**이다. 실제 모터, 카메라, serial, ROS graph에는 접근하지 않았고 실물
sim-to-real 성공을 뜻하지 않는다.

## main 통합 시 지원 범위 — 2026-09-07

이번에 다시 실행한 범위는 CPU 계약 검사와 synthetic ACT/checkpoint/mock-bus 테스트다.
planner는 이제 **시작점을 포함해 한 점이라도 constraint 검사가 실패하면 해당 경로를 거부**한다.
이전의 초기 contact 구간 허용 예외를 제거했으므로, 위 9월 5일 handover 결과를 수정된 planner의
성공 근거로 재사용하지 않는다. 새 SAPIEN/CuRobo rollout은 아직 확인하지 못했다.

RoboTwin의 joint/action은 `rad + gripper 0..1`, `action[t] = measured state[t+1]` 계약이다.
아래 NPZ는 SIM/offline 변환 결과이며, 실제 LeRobot record/train 경로에 자동으로 넣는 importer는
연결하지 않았다. 실제 IL 단위·action 의미와 대조하기 전에는 두 dataset을 합치지 않는다.

`install_runtime.py`는 사용자가 지정한 외부 RoboTwin·URDF·mesh에 수동 적용하는 SIM 도구다.
CI는 이 installer나 실제 장치를 실행하지 않는다. 외부 runtime pin, dirty 변경, asset 출처와
재배포 권리는 아직 검증하지 못했으며, 해당 환경을 main의 재현 보장 범위에 포함하지 않는다.
실물 dispatch는 이번 통합의 실행 승인 대상이 아니다.

## 실물과 맞춘 계약

- 팔: 동일한 SO-101 follower 2대, 각 5 arm joints + gripper
- action/state 순서: `left 5 rad, left gripper 0..1, right 5 rad, right gripper 0..1`
- arm base: `(-64, +80, 109.5) mm`, `(-64, -80, 109.5) mm`
- top RGB-D: HP-ASC-H201/R77, optical center `(-64, 0, 420) mm`
- wrist RGB: 좌·우 320×240
- H201 depth: 640×460, canonical 변환에서는 `uint16 mm`
- 실행 역할: 노트북이 RoboTwin·학습·인식을 맡고 Raspberry Pi 4는 이 실행 대상이 아니다.

배치 수치는 기존 [`hardware_roles.json`](../config/hardware_roles.json)과
[`mobile_dual_so101/README.md`](../sim/mobile_dual_so101/README.md)를 재사용했다. wrist camera
extrinsic과 FOV는 CAD 기반 provisional 값이며 실측 calibration이 아니다.

## 구현한 handover 순서

1. 왼팔 pre-grasp → 원기둥 grasp
2. 왼팔 80 mm lift
3. 오른팔 handover 대기 위치 이동
4. 왼팔이 중앙 전달 위치로 30/40/25 mm 이동
5. 오른팔이 여섯 구간으로 접근해 원기둥을 0.2 aperture로 잡음
6. 왼팔을 `0.30 → 0.40 → 0.50 → 0.65 → 0.80 → 1.0`으로 점진 개방
7. 왼팔이 `-30/-40/+20 mm`로 빠진 뒤 home 복귀
8. 오른쪽 두 jaw 접촉, 반대 접촉 법선, 양팔 충돌 0, 물체 높이로 성공 판정

SO-101은 5축이라 6-DoF pose를 그대로 강제하지 않았다. CuRobo로 위치와 접근축을 만족하는
후보를 구하고 jaw 방향, 관절 한계, self/world collision을 검사한 뒤 quintic joint path를 만든다.
그래프 fallback이 현재 상태에서 떨어진 점으로 시작할 때는 같은 제한을 적용한 연결 구간을 앞에 붙인다.

## 데이터에서 발견해 수정한 문제

공식 RoboTwin 기본 renderer는 CUDA ray tracing과 OIDN을 강제했다. 현재 SAPIEN 3.0.0b1
환경에서는 `OIDN Error: invalid handle`과 RGB 과노출·노이즈가 발생했다. DAPIER task에만
SAPIEN `default` raster shader를 사용하도록 patch했고, 좌·상단·우 카메라의 실제 저장 frame을
육안 확인했다.

공식 저장기는 servo drive target을 `state`로 기록하고 다음 target을 `action`으로 옮긴다. 이 방식은
이번 SO-101 경로에서 한 frame 0.25246 rad 불연속을 만들었다. DAPIER task는 SAPIEN의 실제 qpos와
실제 gripper aperture를 기록하고 `action=다음 시점에 도달한 관절 상태`로 저장한다. 수정 후 15 Hz
팔 관절 최대 변화는 0.033236 rad로, 설정한 0.5 rad/s 한계 안이다.

## 재현

```bash
cd ~/DAPIER-vision-relative-manipulation-02
unset LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=~/RoboTwin/venv/bin:/usr/local/cuda-12.8/bin:$PATH

python 2ARM_ROBOT/robotwin/install_runtime.py \
  --robotwin ~/RoboTwin \
  --source-urdf ~/so101_ros2_ws/src/so101-ros-physical-ai/so101_description/urdf/legacy/so101_new_calib.urdf \
  --mesh-source ~/so101_ros2_ws/src/so101-ros-physical-ai/so101_description/meshes

cd ~/RoboTwin
bash collect_data.sh dapier_handover_block dapier_so101_smoke 0
```

생성 HDF5는 그대로 실물 dataset이라고 부르지 않는다. 다음 명령이 12축 순서, gripper 범위,
세 카메라 크기, frame 수, H201 depth를 검사한 뒤 canonical NPZ를 원자적으로 만든다.

```bash
cd ~/DAPIER-vision-relative-manipulation-02
~/RoboTwin/venv/bin/python 2ARM_ROBOT/robotwin/convert_episode.py \
  ~/RoboTwin/data/dapier_so101_smoke/dapier_handover_block/dapier_dual_so101/data/episode_0000000.hdf5 \
  /tmp/dapier-robotwin-handover-0000000.npz
```

출력 shape는 `state/action (T,12)`, 좌·우 RGB `(T,3,240,320)`, H201 depth
`(T,1,460,640) uint16 mm`다. 상세한 실행 결과와 실패 기록은
[`HANDOVER_VALIDATION_20260905.md`](HANDOVER_VALIDATION_20260905.md)에 남겼다.

## 다음 gate

RoboTwin synthetic episode를 ACT에 넣기 전 여러 seed의 성공률, 물체·카메라·조명·마찰 범위를
실측 기반으로 확장해야 한다. 그 뒤 실제 LeRobot episode와 같은 schema로 합치고, 실물에서는
`policy shadow → calibration/limit/watchdog 검사 → 저속 rollout` 순서로 확인한다. 실제 두 팔이
반복 성공하기 전에는 sim-to-real 완료로 기록하지 않는다.
