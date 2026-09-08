# Vision-relative IK 수학 계약

이 문서는 중앙 HP-ASC-H201 RGB-D와 좌·우 SO-101을 같은 작업 평면에 두는
구조에서, 박스 위치가 바뀌어도 오른팔이 뚜껑 날개 앞까지 접근하는 계산을
정리한다. 실물 runtime의 관측 원점은 MuJoCo world가 아니라 실제 camera optical
frame이다. 고정된 MuJoCo 좌표나 관절각을 저장해 재생하지 않는다.

## 1. 좌표계

- `W`: 실측·로그·양팔 협업에 선택적으로 사용하는 `workcell_base`
- `C`: 상단 RGB-D optical frame
- `R`: 오른팔 base frame
- `O`: 검출한 박스 또는 뚜껑 날개 frame
- `E`: 오른팔 gripper tool frame

RGB-D 픽셀 $(u,v)$의 깊이를 $Z=D(u,v)$, 카메라 내부 파라미터를
$(f_x,f_y,c_x,c_y)$라고 하면 물체의 metric 3D 점은 실제 카메라 frame에서
다음처럼 복원한다.

$$
{}^{C}p_O=
\begin{bmatrix}
(u-c_x)Z/f_x \\
(v-c_y)Z/f_y \\
Z
\end{bmatrix}
$$

박스의 여러 depth point와 RGB 경계·keypoint를 맞춰 중심, 방향과 날개 법선을
구성하면 물체 pose `${}^{C}T_O`가 된다. 오른팔 IK에 필요한 값은 실물에서 보정한
camera-to-arm 변환 `${}^{R}T_C`를 적용한 다음 값이다.

$$
{}^{R}T_O = {}^{R}T_C {}^{C}T_O
$$

양팔이 공유하는 물리 기준 frame을 운영할 때만 다음과 같이 풀어 쓴다.

$$
{}^{R}T_O = ({}^{W}T_R)^{-1} {}^{W}T_C {}^{C}T_O
$$

카메라와 팔을 같은 책상에 놓았다는 사실만으로 `${}^{R}T_C`가 결정되지는 않는다.
카메라 높이·기울기와 오른팔 base 위치를 실제 장비에서 측정하거나 기준 마커로
보정해야 한다. 이 값은 MuJoCo world 좌표에서 복사하지 않는다.

MuJoCo에서는 같은 카메라 모델로 RGB·depth를 렌더링한 뒤 위 역투영부터 동일하게
수행한다. `data.site_xpos`, 물체 body pose 같은 simulator 정답 좌표는 디버그
비교에만 사용하며 perception·policy·실물 명령 입력으로 전달하지 않는다.

## 2. 물체 기준 목표점

날개 중심 위치를 $p_O$, 로봇이 접근할 단위 법선 벡터를 $n_O$라고 한다.
pre-grasp 거리 $d_p$와 최종 접근 거리 $d_a$를 사용해 목표를 만든다.

$$
p_{pre} = p_O - d_p n_O, \qquad
p_{approach} = p_O - d_a n_O
$$

첫 기준값은 $d_p=0.10\sim0.15\,m$이고 $d_a$는 접촉하지 않는 안전 거리로 둔다.
두 값은 실측 결과로 조정하며 소스에 고정된 절대 박스 좌표로 사용하지 않는다.

## 3. SO-101의 IK 목표

그리퍼 개폐를 제외한 SO-101 팔은 5축이다.

$$
q = [q_{pan},q_{lift},q_{elbow},q_{wrist\_flex},q_{wrist\_roll}]^T
$$

따라서 위치 3축과 회전 3축을 항상 동시에 강제하는 완전한 6-DoF pose IK는
과구속될 수 있다. 이 프로젝트에서는 다음을 푼다.

- gripper 위치 XYZ: 필수
- gripper 접근축과 날개 법선 정렬: 필수
- 접근축 주위 roll: 자유 또는 약한 선호값

현재 위치 오차와 접근축 오차는 다음과 같다.

$$
e_p = p^* - p(q), \qquad
e_a = a(q) \times a^*
$$

여기서 $a(q)$는 현재 gripper 접근축, $a^*$는 목표 접근축이다. 가중 오차는
다음처럼 구성한다.

$$
e = \begin{bmatrix}e_p \\ w_a e_a\end{bmatrix}, \qquad
J = \begin{bmatrix}J_p \\ w_a J_r\end{bmatrix}
$$

## 4. Damped Least Squares

현재 `physics_ik.py`는 MuJoCo `mj_jacSite()`로 위치·회전 Jacobian을 구하고
Damped Least Squares(DLS)로 관절 갱신량을 계산한다.

$$
\Delta q = J^T(JJ^T + \lambda^2 I)^{-1}e
$$

$\lambda$는 특이점 부근의 과도한 관절 이동을 줄인다. 계산한 갱신량은 매 반복에서
제한하고 관절 범위 안으로 자른다.

$$
q_{k+1}=\operatorname{clip}
\left(q_k+\operatorname{clip}(\Delta q,-\Delta q_{max},\Delta q_{max}),
q_{min},q_{max}\right)
$$

현재 시뮬레이션 기본값은 다음과 같다.

| 항목 | 값 | 의미 |
|---|---:|---|
| damping $\lambda$ | 0.02 | 특이점 완화 |
| 최대 반복 | 100 | 무한 반복 방지 |
| 반복당 최대 관절 변화 | 0.05 rad | 큰 수치 점프 방지 |
| 위치 허용 오차 | 0.0005 m | 시뮬레이션 수렴 기준 |
| 접근축 허용 오차 | 2 deg | 선택한 tool-axis 기준 |

이 값은 실물 정밀도를 증명하지 않는다. 카메라·외부 보정·관절 영점 오차를 측정한
뒤 실물 허용 오차를 따로 정한다.

## 5. 경로와 동역학 제한

IK 해는 끝점일 뿐 충돌 없는 이동을 보장하지 않는다. 현재 코드는 시작점과 목표점
사이에 7차 보간을 사용한다.

$$
s=t/T, \qquad
h(s)=35s^4-84s^5+70s^6-20s^7
$$

$$
q(t)=q_0+h(s)(q_1-q_0)
$$

경로 전체에서 관절 한계, 좌·우 팔, 카메라 지지대, 테이블과 박스 충돌을 검사하고
속도·가속도·jerk·추종 오차 제한도 확인한다. 끝점 IK residual만 작다고 성공으로
판정하지 않는다.

## 6. 손목 RGB 폐루프 보정

상단 RGB-D는 실제 optical frame에서 coarse 3D 목표를 만들고 오른팔 손목 RGB는
접근 중 영상 오차를 줄인다. 목표 특징의 영상 좌표를 $u^*$, 현재 좌표를 $u$라고
하면

$$
e_u=u-u^*, \qquad v_C=-kL^+e_u
$$

$L$은 image Jacobian이고 $v_{C_w}$는 손목 카메라 기준의 작은 보정 속도다. 손목
카메라는 팔과 같이 움직이므로 base-to-camera 변환도 현재 관절값의 함수다.

$$
{}^{R}T_{C_w}(q) = {}^{R}T_E(q) {}^{E}T_{C_w}
$$

이 보정을 tool/base frame으로 변환한 뒤 다시 관절 공간으로 바꾼다.

$$
\dot q = J^\#\,\operatorname{Ad}_{{}^{R}T_{C_w}(q)}v_{C_w}
$$

실행 시 한 번에 최종 목표로 가지 않고 1~2 cm 구간마다 재관측한다. 손목 RGB의
단일 픽셀만으로 metric 깊이를 새로 단정하지 않고, 상단 RGB-D 깊이와 마지막 유효
3D pose를 함께 쓴다.

## 7. 실행 허용 조건

다음 조건을 모두 만족할 때만 다음 구간을 제안한다.

$$
\begin{aligned}
&\text{observation age} \le TTL \\
&\|e_p\| \le \epsilon_p,\quad \|e_a\| \le \epsilon_a \\
&q_{min}\le q\le q_{max} \\
&d_{collision}(q(s)) > d_{safe}\quad \forall s\in[0,1]
\end{aligned}
$$

물체가 사라짐, timestamp 지연, IK 미수렴, 특이점, 충돌 여유 부족, 관절 추종 오차,
과전류 또는 촉각 과압이 발생하면 정지 후 재관측·재계획한다.

## 8. IL·IK·LLM 책임

- IL: 잡을 날개, 접근 방향, grasp primitive와 단계 순서를 제안한다.
- IK: 현재 관절값과 관측된 물체 pose로 도달 가능한 관절 목표를 계산한다.
- collision/safety: IK 해와 경로의 실행 가능 여부를 독립적으로 판정한다.
- tactile/current: 실제 접촉과 과부하를 판정한다.
- LLM: 고수준 작업 지시만 담당하고 servo loop와 모터 명령을 직접 제어하지 않는다.

## 9. 현재 구현과 다음 단계

2026-09-04에 `vision_box_pregrasp.py`로 MuJoCo H201의 렌더 depth를 optical frame에서
역투영하고, camera-to-base transform을 적용해 박스의 base-frame pose를 계산하는
경로를 연결했다. 물체 body pose나 segmentation ID는 계획 입력으로 사용하지 않는다.
뚜껑 날개가 단순 PCA의 방향을 틀어지게 하므로 depth 점군에 최소면적 직사각형을
맞춘다. 알려진 박스 본체와 날개 치수 중 관측 footprint에 가까운 형상을 선택해
박스 중심을 복원한다.

기본 장면에서 depth로 계산한 박스 중심은 `(0.4185, -0.0005) m`였고, 테스트에서만
조회한 MuJoCo 정답 `(0.4200, 0.0000) m`과의 XY 오차는 약 `1.6 mm`였다. 박스를
XY로 이동하고 yaw를 ±5° 바꾼 세 장면 모두 중심 오차 `8 mm` 이내를 통과했다.
오른팔은 로봇 쪽 앞날개의 오른팔 grasp 지점을 기준으로 전방
`5 → 4 → 3 → 2 cm`, 상단 `9.5 → 7.5 → 5.5 → 4.5 cm`의 네 구간을 순서대로 다시 IK 계산하며,
각 구간에서 양팔·본체·카메라 지지대·바닥·박스 collision을 검사한다. 기본 장면의
네 IK 뒤에는 집게를 연 채 앞날개 전방 `4 mm`, 상단 `5 mm`까지 내려가 닫는 접촉
구간이 있다. 앞날개 collision은 활성화하고 이 마지막 구간에서만 목표 날개 접촉을
허용한다. 640×460 depth로 박스 x/y를 각각 ±30 mm, yaw를 ±5° 바꾼 27조합이 모두
통과했으며, 최대 pre-grasp IK residual은 `0.474 mm`, 최대 접촉 IK residual은
`0.446 mm`, 각 조합의 앞날개 접촉은 최소 1건이었다. 이는 SIM 검증이며 실물 정밀도
증거가 아니다.

계획 후 박스가 base X 방향으로 10 mm 이동한 실패 주입에서는 오른손목 RGB의 판지-바닥
모서리 row가 10 px 넘게 달라졌다. SIM에서 5 mm probe로 구한 pixel/m 민감도와 최대
12 mm 보정 제한을 적용하고 다시 IK·충돌 검사를 수행하니 잔차가 3 px 미만으로 줄었다.
이는 전후 1축 보정만 검증한 결과다. 무늬 없는 판지가 근접 화면을 채우므로 좌우 위치는
H201 3D 추정을 유지하며, 실물에서는 probe 대신 사전 실측한 image Jacobian을 사용한다.

아직 실제 H201 intrinsics와 `base ← H201 optical` 외부 파라미터, 실물 depth의
invalid code/scale, 오른손목 RGB 특징 검출과 1~2 cm 폐루프 보정은 완료되지 않았다.
박스 열기, 왼팔 접근과 신발 인출도 이 pose 계약 위에 순서대로 연결해야 한다.

다음 검증 순서는 다음과 같다.

1. 실제 H201 depth 저장과 intrinsics/extrinsics calibration profile 연결
2. 오른손목 RGB 오차 주입과 1~2 cm 구간별 재관측
3. depth hole·가림·지연·pose noise·모터 오차 sweep
4. 현재 검증한 오른팔 날개 접촉 경로와 박스 열림을 결합
5. 왼손목 RGB 보정, 왼팔 접근과 신발 인출
6. 작업 전후 Visual SLAM 이동과 manipulation mode interlock

별도의 bounded commissioning 명령은 exact confirmation과 interactive TTY에서만
열리지만, 위 depth 기반 계획은 아직 실물 command 경로에 연결하지 않았다. 위 수식과
시뮬레이션 통과는 실물 visual-servo 실행 승인이 아니다.

## 10. 2026-09-08: 카메라로 보고 집기까지 직접 따라가기

record_id: DAPIER-2026-09-08-rgbd-ik-control-learning

나는 4cm·약 20g 블록 실험을 보면서, 화면 속 움직임을 어떤 계산으로 바꾸는지
단계별로 확인하고 있다. 아래의 **현재 실행**과 **다음 구현안**을 구분한다.
상단 장치는 대화에서 HS201로 부르며 저장소 정본 이름은 HP-ASC-H201이다.

### 10.1 RGB와 depth를 같이 쓰는 이유 — 다음 구현안

같은 상단 RGB-D 카메라의 RGB는 **무엇을 잡을지**, depth는 **그 물체가 어디에 있는지**
판단하는 데 쓴다. 색을 찾기 위해 손목 카메라나 새 카메라가 반드시 필요한 것은 아니다.

```text
지시: 빨간 블록을 잡아
  → 상단 RGB에서 빨간 물체 영역 선택
  → 그 영역과 정렬된 depth에서 유효한 거리 점 수집
  → 3D 중심·크기·접근 방향 추정
  → 접근 목표점 → IK → 경로 검사·시간 배분
  → 제한된 관절 목표 → 실행 → 재관측·들림 확인
```

- 첫 색 검출은 설치된 OpenCV의 RGB→HSV, `inRange`와 연결된 영역 검사로 시작할 수 있다.
  빨강은 Hue 경계 양쪽 범위를 합쳐야 하며 조명·채도 임계값은 실측 조정한다.
- RGB와 depth는 한 장치에서도 서로 다른 픽셀 좌표일 수 있다. SDK가 정렬한 모드인지
  확인하고, 아니면 depth 점을 RGB optical frame으로 변환·투영해 색 영역과 연결한다.
  단순히 두 영상을 같은 크기로 resize하는 것은 기하학적 정렬이 아니다.
- 마스크 경계의 배경 혼합과 depth hole을 제외하고 여러 점을 사용한다. 깊이 중앙값과
  크기·높이 검사, 후보 수·유효 점 수·촬영 시각을 함께 확인한다. 후보가 모호하면 재관측한다.
- 현재 실물 수집 경로에서 확인한 것은 상단 metric depth 저장이다. 상단 RGB 동시 수집,
  RGB-depth 정렬과 시간 동기 검증이 이번 설명만으로 구현 완료된 것은 아니다.
- depth 컬러맵의 빨강은 거리 표시일 뿐 물체의 실제 빨간색이 아니다.

이 결합은 이번에 정리한 다음 구현안이다. 현재 cube 실험의 검출기는 RGB 색 마스크를
사용하지 않는다. [OpenCV 색 영역 검출](https://docs.opencv.org/4.13.0/da/d97/tutorial_threshold_inRange.html)을
재사용하고 새 인식 모델은 필요한 물체 범위가 이를 넘어설 때 판단한다.

### 10.2 픽셀을 로봇이 이해하는 미터 좌표로 바꾸기 — 현재 실행

현재 `vision_box_pregrasp.py`의 `estimate_box_pose_from_depth()`를 재사용한다.
로컬 cube 실험은 640×460 가상 depth에서 각 픽셀의 optical 좌표를 다음처럼 계산한다.

```text
Z = depth_m[v, u]
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
p_workcell = R_workcell_from_camera * [X, Y, Z] + t_workcell_from_camera
```

`fx, fy`는 픽셀 단위 초점거리, `cx, cy`는 영상 주점이다. 위 식은 보정/정류된 영상의
optical Z 깊이에 적용한다. 실제 데이터는 depth 단위·왜곡·intrinsics를 먼저 확인한다.
팔별 로컬 모델을 쓰면 다시 `p_arm = inverse(T_workcell_from_arm) * p_workcell`로 바꾼다.
현재 양팔 MuJoCo 모델은 두 팔과 카메라가 같은 world에 있으므로 world 목표를 직접 푼다.
[카메라 투영·보정의 공식 정의](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html).

현재 cube 검출은 작업영역 X 8~35cm, Y -10~10cm와 윗면 높이 35~45mm에 들어오는 점을
선택한다. 181개 평면 방향 후보의 최소면적 직사각형을 맞추고 알려진 4cm 크기와 비교한다.
색을 구분하거나 임의 물체를 알아보는 범용 검출기는 아니다. confidence는 크기 오차 기반
휴리스틱이며 성공 확률이 아니다. 정육면체의 yaw는 90도 대칭이고 현 cube 동작은 검출 yaw를
그립 방향에 연결하지 않았다.

`vision-pick-20g-gentle/report.json`에서 윗면 중심은 약 `(199.74, 0, 40.05)mm`,
1560개 점, 관측 가로·세로는 약 39.26·39.20mm였다. 수평 책상 위에 바로 선 4cm cube라는
가정으로 윗면에서 20mm 내려 물체 중심을 구한다. 이 가정을 신발·기울어진 물체에 재사용하지 않는다.
카메라 보정은 현재 가상 모델에서 얻으며 실물 외부 보정값이 아니다.

### 10.3 물체 중심에서 관절각으로 — 현재 IK

현재 로컬 `vision_tabletop_pick.py`는 검출 중심을 손가락 사이의 기하학적 기준점
`left_cube_grasp`의 목표로 전달한다. 이 site는 좌표 표시점이며 물체를 붙이는 제약이 아니다.
`target_at()` → `solve_bimanual_position_ik()` 순서로 호출한다.

1. 직전 명령 관절값으로 순기구학을 계산해 현재 기준점 위치를 구한다.
2. 목표 위치와의 차이, 손가락 접근축과 아래 방향 `[0, 0, -1]`의 차이를 계산한다.
3. `mj_jacSite()`의 Jacobian으로 작은 관절 변화가 손끝에 미치는 영향을 얻는다.
4. 위 4절의 DLS 식으로 관절 변화량을 계산하고 반복당 0.05rad 및 관절 범위로 제한한다.
5. 위치 0.5mm·접근축 2도 이내면 수렴으로 판정한다. cube 호출은 최대 300회이고 실패하면 이동하지 않는다.

SO-101은 팔 5축+집게이므로 위치 3개와 접근축 방향 2개를 목표로 삼으며 임의의 6D 자세를
항상 만족한다고 하지 않는다. 현재 cube에서는 왼팔 5축을 풀고 오른팔과 집게 값은 유지한다.
계산용 별도 상태의 `qpos`를 바꾸는 IK 탐색과 실제 물리 실행 상태를 강제 이동시키는 것은 다르다.
실물에서는 직전 명령값만 믿지 않고 timestamp가 있는 측정 관절 상태에서 다시 계획해야 한다.

### 10.4 어디로 갈지와 얼마나 빨리 갈지 — 현재 경로와 한계

현재 cube의 목표 순서는 관측 중심 기준이다. 절대 블록 좌표를 명령에 복사하지 않는다.

```text
옆 6cm·위 6cm 접근 → 다시 관측 → X로 3mm 비킨 채 위 6cm 정렬
→ 비킨 위치의 위 15mm까지 하강(최소 2.5초) → 마지막 15mm 하강(최소 2초)
→ 큐브 중심 높이에서 옆으로 3mm 접근(최소 2초) → 집게 닫기·접촉 확인
→ 위로 5cm(최소 3초) → 3초 유지
```

`move()`는 `plan_septic_joint_trajectory()`로 시작·목표 관절각 사이를 7차 곡선으로 연결한다.
`q(t)=q0+h(t/T)*(q1-q0)`이고 h는 위 5절에 있다. 이상적인 연속 명령의 시작·끝에서 속도,
가속도·jerk가 0이다. 기본 명령 제한은 0.5rad/s, 1.5rad/s², 8rad/s³이며 각 제한에 필요한
시간 중 가장 긴 것을 택한다. `minimum_duration_s`는 고정 종료시간이 아니라 하한이다.

**관절 보간은 손끝의 직선 이동도, 무충돌도 보장하지 않는다.** 위의 “15mm”는 IK 기준점의
오프셋이지 모든 손가락 표면의 clearance가 아니다. 현재 충돌 모델은 모터 외함은 보존하지만
손가락 몸통 전체를 표현하지 않아 완전한 경로 안전을 주장하지 않는다. 7차 명령을 15Hz로
sample-and-hold하므로 실제 관절 운동이 연속 다항식과 똑같이 매끄럽다고도 보장하지 않는다.

현재 3mm 우회는 이 방향의 4cm cube에 대한 수정이다. 다른 방향의 물체에는 검증된
물체/grasp 좌표계로 바꿔야 한다. 다음에는 손가락 전체 형상의 swept clearance를 확인하고, 필요하면
Cartesian 중간점을 순서대로 IK 계산한다. 물체가 가려지거나 움직이면 새 관측을 기다리고
목표를 갱신해야 한다. 현 cube는 접근 전 두 번 관측한 뒤 마지막 좌표를 쓰므로 지속 visual servo가 아니다.

### 10.5 모터 목표값을 적용하기 — SIM과 HW를 나누기

현재 `tick()`은 12축 유한값·범위를 확인하고 `data.ctrl[:] = target`을 쓴 뒤
`mj_step()` 34회를 실행한다. 목표 갱신 15Hz, 물리 timestep 1/510초다. 초기화 뒤 실제 실행
`qpos`를 덮어쓰지 않는다. MuJoCo position actuator가 목표와 현재 관절각 차이로 힘을 만들고,
중력·마찰·접촉에 따라 실제 위치가 결정된다. 따라서 목표각 도달과 블록 들림을 따로 검사한다.
[MuJoCo position actuator](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position).

단위 경계는 다음과 같으며 왼팔 6축→오른팔 6축 순서를 유지한다.

| 경계 | 팔 5축씩 | 집게 1축씩 |
| --- | --- | --- |
| 저장된 follower action / 현재 ACT 후처리 출력 | degree | 0~100 |
| 공통 정책 계약 | rad | 0~1 |
| 현재 MuJoCo `ctrl` / IK 출력 | rad | 모델 집게 관절 rad |
| 실제 모터 버스 | 보정된 encoder tick | 보정된 encoder tick |

재생 경계는 `model_rad = radians(recorded_deg * sign + zero_offset_deg)`다.
집게는 정규화한 개폐값을 모델 관절 범위에 선형 대응한다. **0.5rad는 50% 열림이 아니다.**
현재 sign=+1, offset=0은 실물 검증 전 후보이며 저장된 leader→follower 보정을 두 번 적용하지 않는다.

실물 목표 파이프라인은 의도와 구현 상태를 나눈다. 설계는 유효 관측→관절 목표→범위·속도·충돌·
시간 검사→보정 변환→장치 명령→실측 피드백이다. 현재 cube Python 스크립트에는 HW 송신 경로가 없다.
기존 C++ 코어의 범위·변화율 제한과 tick 변환은 재사용 후보지만 독립 실행·watchdog까지 완성된 것은 아니다.
C++ 예제는 집게도 rad이며 설치된 LeRobot은 집게 0~100을 쓰므로 그대로 연결하지 않는다.
두 코드의 팔 tick 변환도 동일하다고 가정하지 않는다. homing offset은 장치 설정과 좌표 변환을
분리해 한 번만 적용해야 한다. 실물 연결은 보정·identity·추종 오차·통신 상실 정지와 현장 승인 후에만 한다.

### 10.6 task와 물체를 바꾸는 법 — 다음 구현안

“빨간 블록”, “파란 블록”, “이 신발”처럼 목표를 바꾸는 구조는 가능하다. 다만
**인식 가능한 물체와 잡기 가능한 물체는 별도 범위**다. 현재 ACT에는 언어 지시 입력이 없다.

- 같은 4cm 블록의 색 변경: 색 선택 조건과 검출 목표를 바꾸고 기존 접근·집기를 재평가한다.
- 블록 크기·방향 변경: 인식 치수, grasp 기준점, 집게 벌림과 접근 경로를 함께 바꾼다.
- 신발 등 형상·재질 변경: 특징/분할 검출과 잡을 지점·방향·파지 방식이 필요하다.
  등록한 잡기 동작을 선택하거나 별도 demonstration·정책 검증을 거친다.
- 모르는 물체 또는 후보가 여러 개: 임의 물체를 실행 대상으로 고르지 않고 지정·재관측을 요구한다.

처음에는 지원 물체 목록과 검증된 grasp를 연결하는 작은 task 규칙이면 된다. 자유 문장 해석이나
새 인식 모델은 별도 확장이다. 고수준 지시가 모터 제한·충돌 검사·실물 승인 경계를 우회하지 않는다.
ACT가 관절 action을 출력하면 IK를 다시 적용하지 않고 실행 검사를 거친다.

### 10.7 이번 실험 결과와 직접 바꿔볼 위치

- 과거 `vision-pick-20g-gentle`은 집기·3초 유지에 통과했지만 접근 때 최대 21.4N이었다.
  같은 trace를 재생해 관절 위치가 일치하는지 확인하고 substep 접촉 위치를 조사했다.
  고정 손가락 아래 모서리가 큐브 윗면을 치는 것이 원인이었다. 닫는 속도만 줄여서는 해결되지 않았다.
- 3mm 옆으로 비켜 내려오는 경로로 바꾼 후 접근 단계 패드 힘은 0N이 됐다.
  접촉이 순간적으로 생긴 것과 들어 올릴 만큼 유지되는 것을 구분해, 양쪽 가상 힘이 각각
  1N 이상으로 5개 15Hz 주기 동안 유지돼야 lift를 허용한다. 이후에도 1N을 계속 제어하는 것은 아니다.
- 현재 `tabletop-contact-final`의 MuJoCo 3.8.1 실행은 바닥 집기·3초 유지 통과,
  최종 중심 67.10mm(약 47.1mm 상승), 전체 최대 패드 힘 2.145N, 최대 겹침 0.0518mm, warning 0이다.
- 실제로 집은 상태를 복제해 1,500 step 더 유지하면 높이 변화 -0.646mm·양면 접촉 1,500회다.
  같은 상태에서 패드 접촉을 끄면 -47.108mm·접촉 0회로 떨어진다. 처음부터 접촉을 끄면
  집기 조건에 실패하고 lift를 명령하지 않는다. 물체 pose 강제 이동·부착은 없다.
- 이전 floating fixture는 초기 6.468mm 관통이 확인돼 유효한 grasp 근거에서 제외했다.
  이제 잘못된 초기 배치를 검출하는 negative 검사로만 쓴다. ACT·양팔 전달·실물 성공과 구분한다.

접촉 모델도 함께 기록한다. 이전 pyramidal의 추가 유지에서는 2.30mm 미끄러져 2mm 검사에
실패했다. 현재는 elliptic/Newton/tolerance 1e-10, impratio 1, NoSlip 0을 사용한다.
엔진 3.3.7에서의 별도 통과와 달리 3.8.1에서 0.5N 기준은 이동 중 놓쳤고,
1N 기준·작은 닫기 증분으로 재검증했다. 설정 선택이 실제 마찰·물성 보정을 대신하지 않는다.
[MuJoCo 접촉 계산 설명](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-settings).
들어 올릴 때 순간 속도 약 0.199m/s와 미세한 미끄러짐은 남아 있어 모든 덜컹거림이 해결됐다는 뜻은 아니다.

로컬 실험 파일 `sim/mobile_dual_so101/vision_tabletop_pick.py`에서 `observe()`는 인식,
`target_at()`은 IK, `move()`는 시간 경로, `tick()`은 물리 실행이다. 이동시간은
`minimum_duration_s`, 집게 닫는 증분은 현재 15Hz당 0.00025rad, 초기 벌림은 0.50rad에서 조정한다.
`--entry-clearance`는 옆으로 비키는 거리(m), `--grasp-force`는 접촉 확인 기준(N)이다.
이 값들은 실물 안전값이 아니며 가상 힘 센서도 실물에 그대로 사용할 수 없다.
`tabletop_replay.json`의 크기·질량은 물체 설정이고 물체 목표 위치를 추정하는 인식과는 분리한다.

코드를 바꿔도 열린 viewer가 자동 갱신되지는 않는다. 새 output 경로로 재실행하고
화면의 단계명과 `report.json`의 실패 단계·접촉·들림 결과를 함께 본다.
이번 변경에 cube/ACT 실행 코드, 접촉 회귀 검사와 설명을 함께 포함한다.
단위·관측·명령 제한 self-test, 바닥 집기/접촉 제거, 실제 CPU 2-worker × 30-transition 연결을
검증했으며 원본·개인 calibration은 공개하지 않는다. 보정된 작업 성공률 평가는 여전히 미완료다.
