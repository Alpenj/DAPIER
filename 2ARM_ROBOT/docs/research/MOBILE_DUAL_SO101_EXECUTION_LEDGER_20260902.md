# 이동형 양팔 SO-101 박스·신발 미션 실행 원장

- `record_id`: `DAPIER-2026-09-02-mobile-dual-shoe-execution`
- 시작일: 2026-09-02 (Asia/Seoul)
- 후속 브랜치: `pro/mujoco-shoe-followup-01`
- 고정 기준: PR [#40](https://github.com/Alpenj/DAPIER/pull/40) head `fecc0c7`
- 실행 범위: MuJoCo-only, offline data, hardware-free test

## 내가 끝내려는 결과

나는 visual SLAM 도착 신호부터 오른팔 박스 열기·유지, 왼팔 신발 proxy 파지·꺼내기,
운반·복귀·내려놓기까지의 양팔 미션을 재현 가능한 simulation과 데이터 흐름으로 완성한다.
일정의 날짜를 기다리지 않고 각 단계의 완료 조건을 통과하면 바로 다음 단계로 넘어간다.

## 완료 기준

1. 전체 hardware-free/MuJoCo test가 통과한다.
2. 오른팔은 뚜껑을 열어 유지하고, 왼팔은 동적 contact를 만든 뒤 신발 proxy를 박스 밖으로 꺼낸다.
3. 허용 contact와 금지 collision을 geom pair로 구분하고 금지 collision을 0으로 유지한다.
4. 같은 seed에서 state transition과 failure code를 재현한다.
5. 정상·no-contact·collision·stale-camera run을 export하고 같은 명령으로 replay·plot을 만든다.
6. mass, friction, pose, camera, network, motor, FSR 변화에서 성공률과 recovery를 비교한다.
7. SLAM 도착·정지 계약부터 복귀·내려놓기까지 headless E2E를 검증한다.
8. 실기 측정값을 받으면 simulation parameter와 gap report를 갱신한다.
9. 최종 수치와 그림을 원본 run ID까지 역추적할 수 있게 보관한다.

## 작업 권한과 경계

- PR #40의 브랜치와 head는 Hermes 검토가 끝날 때까지 변경하지 않는다.
- 후속 코드는 이 worktree와 브랜치에서만 수정한다.
- simulation entrypoint는 ROS, serial, motor SDK를 import하거나 장치를 열지 않는다.
- 실제 motor, torque, EEPROM, publisher, read-only hardware snapshot은 별도 승인 없이 실행하지 않는다.
- 테스트 성공을 실기 성공으로 표현하지 않는다.
- GitHub에는 직접 실행한 명령과 결과만 남긴다.
- 눈으로 확인해야 하는 MuJoCo 단계는 headless 선행 검증 후 사용자에게 관찰 항목을 먼저 알리고 viewer를 연다.
- viewer에서 양팔 자세·접촉·충돌·trajectory를 함께 확인하기 전에는 시각 검증 완료로 기록하지 않는다.

## 실행 그래프

Topology: pipeline

Integration owner와 final verifier는 이 기록을 작성하는 내가 맡는다. 앞 단계의 계약이 바뀌면 영향받는
뒤 단계만 다시 검증한다.

| ID | 목표 | 선행 조건 | 출력 | 완료 검사 | 상태 |
|---|---|---|---|---|---|
| G0 | PR #40 고정과 후속 worktree 격리 | 없음 | 별도 branch/worktree, 이 원장 | PR head 불변, clean baseline | 완료 |
| P1 | camera contract와 home clearance를 정렬 | G0 | 원인·수정·회귀 test | 전체 suite 통과 | 완료 |
| P2 | 왼팔 동적 contact·파지·꺼내기와 금지 충돌 제거 | P1 | trajectory, contact/collision assertion | seed acceptance와 collision 0 | 예정 |
| P3 | 양팔 전체 sequence, export, replay, 시각화 | P2 | 정상·실패 run artifact | 동일 seed 재현 | 예정 |
| T1 | 물리·센서·지연·모터·FSR tuning | P3 | sweep matrix와 parameter revision | baseline 대비 강건성 표 | 예정 |
| I1 | SLAM 도착·복귀를 포함한 E2E integration | T1 | 통합 state trace와 recovery | 반복 E2E acceptance | 예정 |
| F1 | 실측 기반 sim-real feedback와 RC 회귀 | I1 | gap report와 RC evidence | 알려진 blocker 명시 | 예정 |
| R1 | 재현 문서·영상·발표 artifact 동결 | F1 | 최종 evidence bundle | 새 환경 재현·리허설 | 예정 |

## 기준점에서 직접 확인된 상태

- PR #40의 원격 head는 `fecc0c7`이고 두 GitHub check가 통과했다.
- 기존 로컬 기록은 box mission 5/5, grasp planner 4/4, 전체 149/151이다.
- 남은 두 항목은 camera 수 기대값 3과 모델 5의 불일치, home-pose camera clearance
  0.1075 m와 gate 0.13 m의 불일치다.
- 물리 데모에서는 오른팔 뚜껑 열림을 확인했지만 왼 gripper–shoe 동적 contact는 0이었고,
  lid–left wrist 비의도 충돌이 남아 있다.
- 이 값들은 후속 브랜치에서 다시 실행하기 전까지 기준점 기록이며 새 검증 결과가 아니다.

## 단계별 학습 기록 형식

각 단계를 마칠 때 아래 항목을 이 파일에 이어 쓴다.

1. 오늘 확인하려던 가설
2. 변경한 코드와 이유
3. 실행 환경과 명령
4. 통과·실패 수치와 artifact 경로
5. 실패 원인과 수정 과정
6. simulation에서 확인한 범위
7. 아직 실기에서 확인하지 못한 범위
8. 다음 단계의 첫 검증

## G0 · 검토 브랜치 격리

### 직접 한 일

PR #40의 head `fecc0c7`에서 `pro/mujoco-shoe-followup-01` 브랜치와
`DAPIER-mujoco-shoe-followup-01` worktree를 만들었다. PR #40 worktree의 미커밋 문서와
스크린샷은 옮기거나 수정하지 않았다.

### 확인한 결과

- 후속 worktree HEAD: `fecc0c7`
- PR #40 원격 head: `fecc0c7`
- PR #40 상태: Draft/Open, merge state clean
- GitHub checks: `hardware-free-tests`, `mujoco-headless-tests` 성공

### 다음에 확인할 것

P1에서 전체 suite를 같은 SO-101 MJCF 입력으로 다시 실행하고, 두 실패가 실제 요구 변경인지
잘못된 고정값인지 코드·모델 근거로 분리한다.

## P1 · camera contract와 home clearance 정렬

### 확인하려던 가설

MuJoCo 모델의 카메라 5개는 실제 센서 요구가 늘어난 결과가 아니라 원본 wrist camera와 후속
provisional gripper camera가 중복된 결과라고 예상했다. camera clearance 0.13 m 실패는
실제 SO-101 base assembly를 복원한 뒤 남은 이전 geometry 회귀값인지 확인했다.

### 직접 확인한 원인

모델 camera 이름을 출력해 보니 다음 5개가 있었다.

- `front_depth_camera`
- `left_wrist_cam`, `right_wrist_cam`
- `left_gripper_camera`, `right_gripper_camera`

고정 SO-101 MJCF의 `wrist_cam`은 실제 InnoMaker 130도 카메라 기준으로 보정돼 있었다.
mission adapter는 이 카메라를 두고 65도 provisional camera를 양쪽에 하나씩 더 만들었다.
따라서 실물 입력 계약인 전방 RGB-D 1개와 양쪽 gripper RGB 2개보다 MuJoCo object가 두 개 많았다.

camera clearance의 최단 pair도 출력했다. `depth_camera_collision`과 복원된 왼쪽
`waveshare_mounting_plate_so101_v2` 사이가 0.1075428816 m였다. 0.13 m assertion은
Waveshare plate를 제거했던 이전 모델에서 추가됐고, 이후 `1a32622`에서 실제 base assembly를
복원할 때 갱신되지 않았다.

### 변경한 코드와 이유

- 원본 `left/right_wrist_cam`을 `left/right_gripper_camera` 역할로 이름을 바꿔 재사용했다.
- 65도 provisional camera와 실제 optical frame처럼 보이던 임시 site를 제거했다.
- camera payload의 `optical_frame`은 실제 MuJoCo camera object 이름을 사용한다.
- 테스트는 camera object 3개, 보정 FOV 70.5도, 이전 wrist 이름 부재를 확인한다.
- 복원된 mounting plate를 포함한 camera 회귀 여유를 0.10 m로 명시했다.
  전체 protected path의 operational gate 0.03 m는 변경하지 않았다.

### 실행 검증

집중 검증:

```bash
python -m unittest -v \
  test_mujoco_mission_adapters.MuJoCoMissionAdapterTest.test_mobile_model_is_separate_and_has_three_cameras \
  test_mujoco_mission_adapters.MuJoCoMissionAdapterTest.test_three_camera_payloads_are_rendered_and_synchronized \
  test_collision_guard.CollisionGuardTest.test_step_support_home_protects_base_column_and_bare_camera
```

- 결과: 3/3 통과
- RGB-D와 양쪽 RGB frame payload 렌더·동기화 포함

전체 검증:

```bash
python -m unittest discover -s 2ARM_ROBOT/sim/mobile_dual_so101/test -v
```

- 결과: 151/151 통과, 17.878 s
- `hardware_execution=false`

### 배운 점

센서 role 수와 simulator camera object 수를 같다고 가정하면 중복 장착을 놓칠 수 있다. 외부 MJCF를
합성할 때는 source camera의 보정 provenance를 먼저 확인하고 새 camera를 추가해야 한다. geometry
회귀 기준도 mesh를 제거하거나 복원한 commit과 함께 갱신해야 하며, collision gate를 단순히 낮추는
방식과 의도된 설계 envelope를 갱신하는 방식을 구분해야 한다.

### 아직 확인하지 못한 것

- 실제 양쪽 USB RGB camera의 serial, resolution, FOV, intrinsics/extrinsics
- 실제 Waveshare plate와 depth camera 사이 거리
- MuJoCo camera 영상과 실기 영상의 reprojection 차이

### 다음에 확인할 것

P2에서 오른팔이 뚜껑을 유지하는 동안 왼팔의 approach·contact·grasp·extract를 동적 물리로
검증한다. headless 사전 검증 뒤 사용자에게 관찰 항목을 알리고 MuJoCo viewer를 연다.
