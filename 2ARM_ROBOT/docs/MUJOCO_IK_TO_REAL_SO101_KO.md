# MuJoCo IK 결과를 실물 SO-101 동작으로 변환하는 과정

이 문서는 compact Dual SO-101 MuJoCo 모델에서 계산한 오른팔 박스 접근 동작을
실물 오른팔에 전달하는 첫 commissioning 경로를 기록한다. 현재 단계는 카메라로
실제 박스 pose를 다시 계산하는 production 경로가 아니라, 시뮬레이션에서 검토한
관절 변화량과 실물의 방향 대응을 눈으로 확인하는 시험이다.

## 1. MuJoCo에서 목표점 정의

현재 박스 중심은 로봇 기준 `(420, 0) mm`이고, 오른팔 pre-grasp 목표는 박스보다
135 mm 앞, 오른팔 쪽으로 90 mm, 바닥에서 200 mm 높이인 다음 점이다.

```text
p_target = (0.285, -0.090, 0.200) m
```

이 값은 현재 배치 검증용 MuJoCo 좌표다. 실제 작업에서는 상단 RGB-D가 구한
박스 pose를 `base_link`와 오른팔 base frame으로 변환해 목표점을 다시 만든다.
MuJoCo의 정답 좌표를 실물 perception 값처럼 사용하지 않는다.

## 2. IK가 관절각을 구하는 방법

`solve_bimanual_position_ik()`는 damped least-squares 방식으로 오른쪽 gripper
site와 목표점 사이 오차를 반복해서 줄인다.

```text
e = p_target - p_gripper(q)
delta_q = J^T (J J^T + lambda^2 I)^-1 e
q_next = clamp(q + clamp(delta_q, -0.05, 0.05), joint_limits)
```

- `q`: 현재 5개 팔 관절각
- `J`: MuJoCo가 계산한 gripper 위치 Jacobian
- `lambda = 0.02`: 특이점 부근에서 역행렬이 불안정해지는 것을 줄이는 damping
- 1회 관절 변화량: 최대 `0.05 rad`
- 위치 수렴 기준: `0.5 mm`

현재 결과는 100회 제한 안에서 수렴했고 위치 잔차는 약 `0.061 mm`다. 경로는
MuJoCo collision guard로 TurtleBot3 몸체, 중앙 지지대, RGB-D, 반대쪽 팔, 박스
collision과 관절 한계를 검사한다. 현재 시각 검토 단계의 최소 clearance는 5 mm다.

## 3. MuJoCo home과 IK goal의 차이 추출

실물 calibration 영점과 MuJoCo joint 영점은 같다고 가정할 수 없다. 따라서
MuJoCo의 절대 goal 각도를 실물 servo에 그대로 쓰지 않고, MuJoCo에서 실제로
변한 양만 추출한다.

```text
delta_q_sim = q_goal_sim - q_home_sim
```

| 오른팔 관절 | MuJoCo home | MuJoCo goal | 상대 변화량 |
|---|---:|---:|---:|
| shoulder_pan | 0.264° | 3.273° | +3.010° |
| shoulder_lift | -90.059° | -3.445° | +86.614° |
| elbow_flex | 89.083° | 35.814° | -53.270° |
| wrist_flex | 0.950° | -5.332° | -6.282° |
| wrist_roll | -86.022° | -89.401° | -3.379° |
| gripper | -9.998° | -9.998° | 0° |

## 4. 실물 calibration 좌표로 변환

실행 직전에 오른팔의 calibrated degree를 읽고 다음처럼 상대량을 더한다.

```text
q_goal_real = q_start_real + delta_q_sim
```

LeRobot `DEGREES` 모드가 servo raw tick으로 바꿀 때 사용하는 관계는 다음과
같다. `mid`는 해당 관절 calibration의 `range_min`과 `range_max` 중간값이다.

```text
mid = (range_min + range_max) / 2
raw_goal = int(q_goal_real * 4095 / 360 + mid)
```

명령 전에 계산된 raw goal이 각 관절의 calibration 범위를 넘지 않는지 다시
검사한다. 이 방식은 실물의 현재 자세를 기준으로 같은 관절 방향과 이동량을
재현하지만, 링크 장착 방향 자체가 MuJoCo와 다르면 Cartesian 위치까지 같아지는
것은 아니다. 그래서 첫 실행은 방향 대응을 확인하는 commissioning이다.

## 5. 한 번에 goal을 쓰지 않고 8초 궤적으로 나누기

가장 큰 변화는 shoulder lift의 86.614°다. goal을 한 번에 쓰지 않고 20 Hz,
8초 동안 septic time scaling으로 160개 목표를 만든다.

```text
u = t / T
s(u) = 35u^4 - 84u^5 + 70u^6 - 20u^7
q(t) = q_start + s(u) * (q_goal - q_start)
```

이 식은 시작과 끝에서 속도, 가속도, jerk가 0이 되도록 만든다. 현재 target
프로파일의 이론상 최대값은 약 `23.68 deg/s`, `10.17 deg/s²`, `8.88 deg/s³`이며
실행 gate는 각각 `25`, `12`, `10`을 넘으면 시작 전에 거부한다.

## 6. 실물 실행 전 gate

실행기는 다음 조건을 모두 확인한 뒤에만 오른팔 1~5번 모터의 torque를 켠다.

1. interactive TTY와 현장 작업자 플래그
2. exact confirmation string
3. owner-only trusted profile과 calibration SHA-256
4. `/dev/dapier/right_arm`의 USB controller identity 재확인
5. motor ID 1~6과 STS3215 control table
6. 전체 오른팔 torque off, position mode, stationary, status 0
7. 온도 50°C 이하, 전압 raw 110~130, load 절댓값 500 이하, current 300 이하
8. 계산된 목표가 실물 calibration 범위 안인지 확인

왼팔에는 goal이나 torque 명령을 쓰지 않는다.

## 7. 이동 중 피드백과 중단

목표는 50 ms마다 갱신한다. 250 ms마다 position, velocity, load, current,
temperature, voltage, status, torque를 다시 읽는다. 다음 중 하나가 발생하면
예외 경로로 들어가 오른팔 1~5번 torque를 끈다.

- 관절 추종 오차 18° 초과
- 온도, 전압, load, current 제한 초과
- servo status 오류 또는 torque 상실
- 9초 leg deadline 초과
- `Ctrl-C` 또는 종료 신호
- USB read/write 예외

오른팔은 pre-grasp에 도달해 2초간 유지한 뒤 같은 septic 궤적으로 시작 자세에
복귀한다. 마지막에는 torque-off를 read-back으로 확인하며, 모든 trace는 Git에
올리지 않는 owner-only JSON log에 기록한다.

## 8. production 단계에서 바뀌는 부분

최종 동작은 이 고정 좌표 시험을 반복하는 구조가 아니다.

```text
상단 RGB-D 박스 pose
→ camera optical frame에서 3D 목표 생성
→ hand-eye/extrinsic으로 base/right-arm frame 변환
→ IK와 충돌 검사
→ pre-grasp까지 짧은 action chunk 실행
→ 오른팔 손목 RGB로 재관측
→ 오차가 크면 정지·재계획
→ 촉각/모터 전류로 실제 접촉 확인
```

즉 IK는 기하학적으로 안전한 접근 경로를 만들고, IL은 잡을 날개와 grasp 자세 및
동작 순서를 정하며, wrist visual servo가 sim-to-real 위치 오차를 닫는다. LLM은
이 실시간 루프에 들어가지 않고 고수준 작업 지시만 담당한다.

## 현재 검증 상태

- MuJoCo IK 수렴과 5 mm collision guard: SIM PASS
- 고정 joint delta, septic profile, calibration range, exact confirmation: MOCK PASS
- 실물 오른팔 pre-grasp: exact confirmation 후 현장 실행 대기
