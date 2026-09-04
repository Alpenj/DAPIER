# H201 depth에서 오른팔 pre-grasp까지 연결한 기록

`record_id`: `DAPIER-2026-09-04-depth-box-pregrasp`

## 내가 만들려는 동작

Manipulation 단계는 다음 순서로 고정한다. 이 앞뒤에는 Visual SLAM 이동이 들어가고,
팔을 움직일 때 모바일 베이스는 정지한다.

1. top-view H201 depth로 박스를 본다.
2. 카메라 픽셀과 깊이를 로봇 base 기준 좌표로 바꾼다.
3. 오른팔을 박스 앞으로 보내고 오른손목 RGB로 오차를 보정한다.
4. 오른팔로 박스를 연다.
5. 왼팔을 박스 안으로 보내고 왼손목 RGB로 오차를 보정한다.
6. 왼팔로 신발을 들어 올린다.

## 오늘 직접 연결한 범위

`vision_box_pregrasp.py`는 MuJoCo H201 카메라에서 640×460 metric depth를 렌더하고,
실물 calibration과 같은 형태의 intrinsics와 `base ← optical` transform으로 점군을
복원한다. runtime 계산에는 박스 body pose와 segmentation ID를 넣지 않는다.

뚜껑의 앞날개와 좌우 날개 때문에 점군 PCA만 쓰면 기본 장면에서도 방향이 약 16°
틀어졌다. depth 점군을 0~180°로 1°씩 돌려 최소면적 직사각형을 찾는 단순한 방법으로
바꾸니 기본 장면의 박스 XY 중심 오차가 약 1.6 mm가 됐다. 이 오차는 검출 이후
테스트에서만 MuJoCo 정답과 비교한 값이다.

검출한 박스 pose와 실측 박스 치수로 로봇 쪽 `front tuck flap`의 중심선을 찾고,
그중 오른팔이 잡을 지점을 박스 중심선에서 오른쪽 90 mm로 정했다. 처음에는 옆쪽
`dust flap`을 목표로 잘못 해석했고, MuJoCo 창에서 박스 본체 위로 팔이 가는 것을 보고
앞날개 기준으로 수정했다.

```text
앞날개 grasp 지점 전방 5 cm / 상단 9.5 cm
→ 전방 4 cm / 상단 7.5 cm
→ 전방 3 cm / 상단 5.5 cm
→ 전방 2 cm / 상단 4.5 cm
→ 집게를 연 채 전방 4 mm / 상단 5 mm 접촉점
→ 집게 닫기
```

각 목표마다 이전 관절 자세에서 오른팔 5축 DLS IK를 다시 풀고, 양팔·TurtleBot3
본체·카메라 지지대·바닥뿐 아니라 박스 collision geom과의 거리도 검사한다. 마지막
목표만 한 번 계산해 보내지 않는 이유는 이후 각 구간 사이에 오른손목 RGB 재관측을
끼워 넣기 위해서다. 앞날개는 시각용 판이 아니라 충돌 geom으로 바꿨으며, 마지막
접촉 구간에서만 목표 날개와의 접촉을 허용하고 나머지 박스 벽 충돌은 계속 막는다.

## 실행 결과

```text
기본 박스 중심 추정: (418.5, -0.5) mm
기본 MuJoCo 정답:    (420.0,  0.0) mm
기본 XY 중심 오차:   약 1.6 mm
앞날개 오른팔 grasp:  (286.0, -90.5, 105.7) mm
박스 이동/yaw 시험:  27/27 (x/y ±30 mm, yaw ±5°)
오른팔 IK 구간:       4/4 수렴
최대 IK residual:     0.474 mm
박스 포함 경로 검사: 4/4 통과, pre-grasp 요구 clearance 5 mm
앞날개 접촉 IK:       최대 residual 0.446 mm, 모든 조합 접촉 1건 이상
손목 RGB 지연 이동:   박스 +10 mm 이동 시 초기 오차 10 px 초과
손목 RGB 1회 보정:    보정 후 앞날개 edge 오차 3 px 미만
실물 명령:            없음
```

손목 RGB 보정은 판지와 바닥의 색 경계에서 앞날개 모서리 row를 검출한다. 현재 무늬 없는
판지가 근접 화면을 거의 채우므로 손목 RGB 하나만으로 좌우·깊이를 모두 복원하지 않는다.
top-view H201이 3D XY를 계속 담당하고 손목 RGB는 마지막 전후 오차만 줄인다. SIM에서는
5 mm probe로 pixel/m 민감도를 계산했지만, 실물에서는 이 probe를 그대로 실행하지 않고
사전 실측한 camera/arm Jacobian과 최대 12 mm correction limit를 사용해야 한다.

재현 명령은 다음과 같다. `DAPIER_SO101_MJCF`에는 로컬 SO-101 MuJoCo asset을 넣는다.

```bash
DAPIER_SO101_MJCF=/path/to/so101_new_calib.xml \
python -m unittest \
  2ARM_ROBOT/sim/mobile_dual_so101/test/test_collision_guard.py \
  2ARM_ROBOT/sim/mobile_dual_so101/test/test_compact_mobile_dual_so101.py \
  2ARM_ROBOT/sim/mobile_dual_so101/test/test_vision_box_pregrasp.py -v

python 2ARM_ROBOT/sim/mobile_dual_so101/vision_box_pregrasp.py \
  --model-path /path/to/so101_new_calib.xml --view

python 2ARM_ROBOT/sim/mobile_dual_so101/vision_box_pregrasp.py \
  --model-path /path/to/so101_new_calib.xml --teleop

python 2ARM_ROBOT/sim/mobile_dual_so101/vision_box_pregrasp.py \
  --model-path /path/to/so101_new_calib.xml --teleop \
  --record-output output/teleop_attempts/right_pregrasp_001
```

teleop 키는 좌/우 화살표로 팔 선택, `1~6`으로 관절 선택, 위/아래 화살표로 이동,
`O/C`로 그리퍼 열기/닫기, `H`로 home, `Space`로 정지/재개한다. SIM 전용이며
박스 collision geometry도 검사한다.

기록 모드에서는 실제로 적용한 control target을 20 Hz로 모은 뒤 같은 compact 모델에
재생하면서 전면/상단 RGB-D와 좌우 손목 RGB, 양팔 state/action을 기존 DAPIER episode
계약으로 저장한다. 종료 직전 3개 sample에서 오른팔 gripper frame이 visual target
15 mm 안에 계속 있어야만 `accepted`가 된다. 그 외 시도는 `recorded`이고 학습 입력으로
승격하지 않는다.

작은 SIM 검증에서는 65 frame accepted episode를 만들고 기존 DAPIER-native ACT dataset이
state/action 12D, RGB-D 4 channel, action chunk를 읽는 것을 확인했다. CPU one-step 학습과
checkpoint reload도 통과했다. 이는 저장→학습 배선 smoke이지 일반화된 자율 동작의 성공률
증거는 아니다. 여러 위치·yaw·조명에서 accepted 시범을 모은 뒤 다음 기존 명령으로 학습한다.

```bash
PYTHONPATH=2ARM_ROBOT/src/shoe_sorting_data \
python -m shoe_sorting_data.dapier_native_act train \
  --root output/accepted_episodes \
  --checkpoint output/checkpoints/right_pregrasp_act.pt \
  --chunk-size 16 --batch-size 8 --max-steps 100 --device cuda
```

checkpoint 출력 경계도 SO-101 기준으로 연결했다. 오른팔 단계에서는 policy가 낸 왼쪽
6축 값을 실행하지 않고 현재 측정값으로 고정한 뒤 기존 freshness·base 정지·joint limit·
E-stop·watchdog 검사를 통과시킨다. 오른쪽 arm radian은 LeRobot bus degree로, normalized
gripper는 0~100으로 바꾼다. fake bus E2E에서 checkpoint 추론부터 오른팔 6모터
`Goal_Position` 1회 write까지 통과했으며, 좌측 bus write는 없다. 사용한 checkpoint는
one-step wiring smoke이므로 실물 task 실행 승인의 근거는 아니다.

카메라 확인은 아래 명령으로 `좌 손목 | H201 | 우 손목` 한 창에 표시한다. 미리보기는
320×240/10fps이고 학습 원본 저장 해상도와 분리한다.

```bash
2ARM_ROBOT/scripts/show_camera_dashboard
```

## 아직 확인하지 못한 부분

- 실제 H201의 intrinsics, depth scale과 invalid code
- 실측 `base ← H201 optical` 외부 파라미터
- 실물 오른손목 RGB의 앞날개 특징과 사전 실측 image Jacobian
- 프레임 age, 두 카메라 timestamp skew와 frame drop
- depth hole·가림·조명·박스 이동에 대한 실물 허용치
- 오른팔 접촉 후 뚜껑 열림을 현재 visual 경로와 결합하는 작업
- 왼팔 접근과 신발 인출

다음에는 실제 H201 depth 한 프레임을 같은 함수에 넣을 수 있는 저장 경계를 먼저
연결하고, 현재 SIM 손목 RGB 보정을 실측 Jacobian 기반 경계로 교체한다. 실제 calibration이 없으면
MuJoCo transform을 실물 값처럼 복사하지 않고 실행을 막는다.
