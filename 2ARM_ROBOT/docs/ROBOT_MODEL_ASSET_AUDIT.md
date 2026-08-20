# DAPIER 로봇 모델 자산 감사와 sim-to-real 계획

기준일: 2026-08-20

## 결론

TurtleBot3 Waffle Pi의 공식 URDF와 메시는 재사용할 수 있다. JDcobot200 강사 저장소의 URDF,
MJCF, STL은 구조 참고에는 유용하지만 저장소에 명시적인 라이선스가 확인되지 않았으므로 DAPIER에
복사하지 않는다. DAPIER 팔 모델은 실측 치수와 장치별 보정값으로 별도 작성한다.

## 확인된 자산

### TurtleBot3 Waffle Pi

ROBOTIS 공식 `turtlebot3` 저장소의 Jazzy 브랜치에는 다음 자산이 있다.

- `turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf`
- `turtlebot3_description/meshes/bases/waffle_pi_base.stl`
- 바퀴 STL과 LiDAR 메시
- Astra 센서 DAE와 텍스처

공식 저장소는 Apache-2.0이다
([ROBOTIS TurtleBot3 저장소](https://github.com/ROBOTIS-GIT/turtlebot3/tree/jazzy)). 따라서 라이선스와
고지를 유지하며 의존하거나 재사용할 수 있다. 다만 현재 Ubuntu 노트북의 ROS 2 Jazzy 설치에는
`turtlebot3_description` 패키지가 발견되지 않았다. 패키지는 임의로 설치하지 않았으며, 시뮬레이션
빌드 전에 누락 의존성으로 처리한다.

공식 Astra 메시는 카메라 외형 배치의 시작점일 뿐이다. 실제 연결 장치의 라벨은 AADJA1300GX이고
USB에서 Orbbec Astra 계열로 확인했지만, 동일 외형·광학 중심인지는 확인하지 않았다. 실제 장치
치수와 렌즈 원점을 측정해 `camera_link` 변환을 정해야 한다.

### JDcobot200 강사 참고 저장소

[JD-edu/jdcobot200_imitation_learning](https://github.com/JD-edu/jdcobot200_imitation_learning)에는
URDF, MJCF, MuJoCo scene, STL/part 자산과 sim-to-real 예제가 있다. 2026-08-20 GitHub API 확인에서
저장소 라이선스 endpoint는 404였고 최상위 LICENSE/COPYING 파일도 발견되지 않았다.

따라서 다음 원칙을 적용한다.

- 모터 ID 순서와 필요한 기능을 이해하는 참고 자료로만 사용한다.
- 소스 코드, URDF 수치, MJCF, 메시를 DAPIER로 복사하지 않는다.
- 저작권자가 라이선스를 명시하거나 사용 허가를 주면 그 범위와 고지를 기록한 뒤 재평가한다.

## 실측으로 확인한 팔 구성

- 팔 두 대에서 각각 STS3215 ID 1–6이 1Mbps로 응답했다.
- 현재 의미 모델은 ID 1–5를 5개 팔 관절, ID 6을 그리퍼로 분리한다. 총 모터 수는 팔당 6개,
  양팔 12개다.
- 이 역할 순서는 참고 코드 구조와 일치하지만 실제 저속 동작으로 아직 검증하지 않았다.
- 한 팔은 모든 offset이 `+85`이고 position limit이 `0..4095`라 초기값 가능성이 높다.
- 다른 팔은 관절별 offset과 일부 position limit이 다르게 저장되어 있다. 더 구체적으로 보정된 흔적일
  수는 있지만, 장착 자세와 충돌을 확인하기 전에는 안전한 정답으로 간주하지 않는다.
- offset은 엔코더 영점, position limit은 기구 안전 범위, 전류/load는 동적 부하에 관한 값이다.
  관절 부하가 다르다는 이유만으로 offset이나 position limit이 자동 결정되지는 않는다.

## DAPIER 전용 모델 구조

팔 모델을 만들 때 하나의 공통 기구 Xacro와 장치별 calibration 파일을 분리한다.

```text
base_footprint
└─ turtlebot3_waffle_pi
   └─ dapier_mount
      ├─ arm_a_mount ─ arm_a_joint_1 ... arm_a_joint_5 ─ arm_a_gripper
      ├─ arm_b_mount ─ arm_b_joint_1 ... arm_b_joint_5 ─ arm_b_gripper
      └─ camera_mount ─ camera_link ─ camera_optical_frame
```

공통 Xacro에는 링크 길이, 관절축, collision, visual, 질량과 관성, nominal joint limit을 둔다.
장치별 YAML에는 다음을 둔다.

- `arm_a`/`arm_b`의 실제 좌우 의미와 고정 장착 변환
- 모터 ID, 엔코더 영점, 회전 부호, radian 변환
- 충돌로 검증한 soft/hard position limit
- 속도·가속도·전류 제한과 검증 상태
- calibration 버전, 측정 일시, 근거 파일 해시

USB serial 문자열은 배포 모델에 직접 박지 않고 로컬 하드웨어 프로필에서 alias로 해석한다.

## Gazebo와 MuJoCo 역할

1. URDF/Xacro를 시각화해 TF 트리, 축 방향, 영점과 양팔 대칭을 확인한다.
2. primitive collision부터 시작해 self-collision과 TurtleBot 상판 간섭을 확인한다.
3. 질량·무게중심·관성을 실측한 뒤 Gazebo/GZ에서 이동 베이스와 양팔의 전복 경향을 검증한다.
4. 동일 URDF를 MuJoCo 변환의 입력으로 사용하되 actuator gain, damping, friction, contact는 별도
   MJCF overlay에서 식별한다.
5. 실기체 저속 step 응답과 시뮬레이터 응답을 비교해 관절별 offset, 부호, 지연, 속도/전류 제한을
   맞춘다.
6. sim-to-real 정책 앞에 실기체 hard limit, self-collision, base stationary interlock, watchdog,
   E-stop을 둔다.

URDF만 있다고 sim-to-real이 완성되는 것은 아니다. 정확한 장착 변환, joint sign/zero, 질량·관성,
마찰, 지연, 센서 intrinsics/extrinsics와 제어 주기까지 같은 의미로 맞아야 한다.

## 다음 실측 순서

1. 팔 A/B에 물리 라벨을 붙이고 각 ID를 한 번에 하나씩 아주 작은 각도로 움직여 역할과 부호를 확인한다.
2. 홈 자세에서 엔코더 원시값을 기록하고, 기구 충돌 직전이 아닌 여유 있는 soft limit을 설정한다.
3. 링크 길이, 팔 베이스 장착 좌표, 카메라 장착 좌표를 측정한다.
4. 링크/브래킷/카메라/배터리 질량과 무게중심을 측정한다.
5. RGB-D ROS 드라이버를 준비한 뒤 intrinsics, depth scale, RGB-depth 정합과 base extrinsics를 보정한다.
6. 그 후 DAPIER 전용 Xacro와 MuJoCo overlay를 생성한다.
