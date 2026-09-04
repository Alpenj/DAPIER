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

현재 구현된 것은 MuJoCo 모델 Jacobian을 사용하는 양팔 DLS IK, 관절 범위 제한,
경로 충돌 검사, 7차 보간과 동역학 측정이다. 아직 IK 목표 생성은 실제 RGB-D
역투영 경로와 연결되지 않았다. 새 workcell의 실측 외부 파라미터, RGB-D 박스
pose, 손목 RGB visual servo와 실물 command 연결은 완료되지 않았다.

다음 검증 순서는 다음과 같다.

1. workcell 좌표계와 카메라·양팔 외부 파라미터 계약
2. 사진 배치의 MuJoCo 모델과 camera frame 시각화
3. 이동·회전된 박스의 synthetic RGB-D pose 입력
4. 오른팔 pre-grasp·approach IK와 전체 경로 충돌 검사
5. 손목 RGB 오차 주입과 구간별 재계획
6. 조명·가림·지연·pose noise·모터 오차 sweep
7. 별도 안전 통합 완료 후 사용자 입회 read-only 및 제한적 실물 검증

현재 저장소의 nonzero 실물 motion은 안전 검토에 따라 차단되어 있다. 위 수식의
시뮬레이션 통과는 실물 실행 승인이 아니다.
