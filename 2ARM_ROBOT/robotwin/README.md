# RoboTwin Dual SO-101

이 폴더가 DAPIER의 RoboTwin 관련 코드와 검증 기록의 정본이다. `~/RoboTwin`은
공식 upstream 실행 환경으로만 사용하며 그 checkout의 변경은 DAPIER 결과물로
간주하지 않는다.

## 첫 깊은 개발 과제

`handover_block`을 Dual SO-101으로 생성하고 같은 정책 입력·출력을 실물 양팔까지
전달한다. 박스 모델링 없이도 양팔 reachability,
5축 IK, 양팔 충돌, 물리 접촉, gripper, 12차원 state/action, 세 카메라 observation을
한 번에 검증할 수 있기 때문이다. 이 과제가 통과한 뒤 `place_dual_shoes`로 확장한다.

## 실제 하드웨어 계약

- 팔: 동일한 SO-101 follower 2대, 각각 5 arm joints + gripper
- action: `[left 5 rad, left gripper 0..1, right 5 rad, right gripper 0..1]`
- arm base: `(x, y, z)=(-64, +80, 109.5) mm`, `(-64, -80, 109.5) mm`
- top RGB-D: HP-ASC-H201/R77, optical center `(-64, 0, 420) mm`, 아래 43도
- wrist RGB: 좌/우 320×240, 학습 observation
- H201 depth: 640×460 `uint16 mm`, IK·3D target용
- 실행 역할: laptop이 RoboTwin/학습/인식을 맡고 Raspberry Pi 4는 대상이 아니다.
- 실물 calibration 값과 controller serial은 Git에 넣지 않는다.

수치는 기존
[`hardware_roles.json`](../config/hardware_roles.json)과
[`mobile_dual_so101/README.md`](../sim/mobile_dual_so101/README.md)의 승인 배치를
재사용한다. 시뮬레이터 결과가 실물 성공을 뜻하지 않으며, 실제 follower별 calibration,
관절 범위, 지연, 발열, 전류, 촉각, 카메라 extrinsic은 rollout gate에서 다시 검사한다.

## 생성과 검증

```bash
cd ~/DAPIER/2ARM_ROBOT/robotwin
python3 test_contract.py

~/RoboTwin/venv/bin/python build_dual_so101.py \
  --source-urdf /path/to/official/so101_new_calib.urdf \
  --mesh-source /path/to/official/meshes \
  --output ~/RoboTwin/assets/embodiments/dapier-dual-so101
```

두 번째 명령은 RoboTwin venv의 SAPIEN으로 생성 URDF를 실제 load하고, active joint
12개와 필수 camera/base link를 확인한다. CuRobo planning과 demonstration 생성은 별도
완료 조건이며 이 검사를 통과했다는 이유로 완료 처리하지 않는다.

## 단계 게이트

1. **Asset:** 공식 SO-101 URDF/mesh provenance, SAPIEN load, 12축 계약
2. **Planner:** 각 팔 CuRobo IK·collision sphere·reachability
3. **Task:** `handover_block` 1 episode 물리 성공 및 HDF5/영상 저장
4. **Dataset:** 실제 12축·좌/우 RGB·H201 depth 스키마로 변환/재로딩
5. **Transfer:** 실제 calibration mapping, limit/watchdog, 저속 shadow→rollout

## Sim-to-real 완료 조건

MuJoCo와 RoboTwin은 서로 다른 좌표계·동역학을 쓰더라도 아래 경계에서는 같은 값을
내보내야 한다.

- state/action 관절 순서와 단위: 좌 5 rad + 좌 gripper 0..1 + 우 5 rad + 우 gripper 0..1
- 관측 역할: left wrist RGB, H201 depth, right wrist RGB와 각 timestamp
- 시뮬레이터 정답 pose는 학습·실물 입력에 사용하지 않음
- sim action은 follower별 실제 calibration으로 ticks에 변환하기 전에 관절 limit,
  변화량, collision, freshness, watchdog 검사를 통과해야 함
- domain randomization에는 카메라 extrinsic·조명·마찰·질량·backlash·지연·frame drop의
  실측 범위를 사용하며 임의 범위를 성공 근거로 쓰지 않음

완료 판정은 `sim dataset 재로딩 → 실제 observation으로 policy shadow 실행 → 명령 없이
오차 측정 → 제한된 저속 rollout` 순서다. 실제 두 팔이 같은 task를 반복 성공하기 전에는
sim-to-real 완료라고 기록하지 않는다.

현재 완료는 1단계의 generator와 SAPIEN load smoke뿐이다. Planner 이후는 실제 실행
결과를 commit과 함께 갱신한다.

`overlay/`는 공식 checkout에 복사되는 최소 파일만 보관한다. ALOHA용 원본
`handover_block`의 740 mm 테이블과 큰 작업 범위를 그대로 쓰지 않고, DAPIER task는
바닥 기준 surface, 40×40×120 mm 시험 물체, 로봇 앞 40~80 mm 좌우 범위로 줄였다.
CuRobo는 한 팔을 계획할 때 중앙 몸체·H201 mast·반대 팔 collision sphere를 함께
읽고 반대 팔은 compact home에 lock한다. sphere 근사는 실제 planning을 통과한 뒤에도
mesh collision과 MuJoCo contact로 교차검증해야 한다.

```bash
~/RoboTwin/venv/bin/python install_runtime.py \
  --robotwin ~/RoboTwin \
  --source-urdf /path/to/official/so101_new_calib.urdf \
  --mesh-source /path/to/official/meshes
```
