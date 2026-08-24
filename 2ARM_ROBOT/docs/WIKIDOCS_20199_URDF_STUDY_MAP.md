# WikiDocs 20199 URDF 학습 지도 — DAPIER 양팔 로봇 적용

기준일: 2026-08-24

대상: [jdCobot200 5축 로봇암 사용 매뉴얼](https://wikidocs.net/book/20199) 중 본문이 확인된 URDF 핵심 페이지 4개

## 이 문서의 범위와 사용 원칙

책의 URDF 관련 장을 순서대로 요약하고, 현재 DAPIER의 양팔 로봇 모델 작업에 연결한다. 책의 문장과 코드를 대량 복제하지 않았으며, 아래의 **책 내용**은 원문 요약이고 **외부 보강**은 ROS/URDF 공식 프로젝트에서 별도 확인한 내용이다.

책은 JDcobot200, DAPIER의 대상은 JDcobot300 양팔이다. 따라서 책의 링크 길이, 질량, 조인트 제한, actuator 수치, 메시를 정답으로 복사하지 않는다. 특히 기존 자산 감사에서 강사 저장소 자산의 라이선스가 명시되지 않았다고 확인했으므로, 구조와 검증 순서만 참고한다.

## 관련 페이지 지도 (책 목차 순서)

|순서|책 페이지|직접성|책 내용 요약|DAPIER에서의 다음 행동|
|---:|---|---|---|---|
|1|[3.1 Onshape를 이용해서 로봇 URDF 만들기](https://wikidocs.net/384466)|직접|Onshape Assembly/Mate를 URDF 링크·조인트로 변환하고, mate 축·질량·조인트 limit·mesh 경로를 검증해야 함을 설명한다.|DAPIER 전용 CAD/실측 모델에서 Onshape export를 출발점으로 쓰되, export 뒤 수동 검증을 필수화한다.|
|2|[3.2 URDF를 MUJOCO에 올리기](https://wikidocs.net/384670)|직접|URDF→MJCF 변환 뒤 actuator 누락, gain 과다, 비현실적 질량/관성 때문에 제어가 불안정해질 수 있음을 다룬다.|URDF의 기구 의미와 MJCF의 제어 의미를 혼동하지 않는다. ACT의 12차원 명령은 실제 driver/safety 승인 뒤에만 actuator로 연결한다.|
|3|[3.3 로봇 그리퍼를 MUJOCO에서 사용하기](https://wikidocs.net/385219)|직접|메시·관절 계층·그리퍼 양손가락 연동·actuator 벡터 검증을 다룬다.|양팔은 팔당 5관절+그리퍼(총 12 DoF)의 **명시적 이름/순서 계약**을 먼저 고정하고, 그리퍼 기구 연동은 실제 구조 확인 후 모델링한다.|
|4|[3장 jdCobot200 로봇암 URDF 생성, MUJOCO에 띄우기](https://wikidocs.net/366978)|장 표지|URDF 생성과 MuJoCo 활용을 묶는 제목 페이지이며 본문은 작성 중이다.|위 3개 실습 페이지의 공통 범위를 가리키는 목차 표지로만 기록한다.|

`3.4 jdcobot200 FK 프로그래밍`([385222](https://wikidocs.net/385222))은 목차상 인접하지만 공개 본문이 작성 중이므로 핵심 페이지 목록에서 제외했다. 관련성이 낮은 1장 사양/DH/워크스페이스와 2장 조립·기초 제어도 이번 노트 범위에서 제외했다. 다만 실측 링크 길이·원점·모터 부호를 확보할 때 다시 참조할 수 있다.

## 핵심 URDF 개념

- `robot`: 하나의 로봇 모델 루트다.
- `link`: 강체 한 개다. `visual`은 보기용, `collision`은 충돌용, `inertial`은 질량·무게중심·관성이다.
- `joint`: parent link와 child link의 상대 변환과 운동 자유도를 정의한다. 양팔에는 주로 `revolute`/`fixed`, 바퀴에는 `continuous`가 맞는다.
- `origin xyz/rpy`: 자식 프레임을 부모 프레임 기준으로 어디에 어떻게 놓는지다. 축 부호가 틀리면 제어·FK·카메라 extrinsic이 모두 틀어진다.
- `axis`: 회전/병진 조인트의 운동축이다. CAD mate connector를 export한 뒤 반드시 RViz에서 확인한다.
- `limit`: `revolute`/`prismatic`의 범위·속도·effort다. URDF limit은 문서화된 모델 limit이며 실기 hard limit을 대체하지 않는다.
- `inertial`: 0 또는 placeholder 질량은 동역학 시뮬레이션을 무의미하게 만든다. 실측/제조사 근거가 생기기 전에는 제어 파라미터를 튜닝하지 않는다.

**외부 보강(공식):** `urdfdom`은 URDF XML을 로봇 모델 자료구조로 파싱하는 공식 라이브러리이며, 현재 명세는 `link`, `joint`, 시각/충돌 형상, 관절 종류와 pose 등을 다룬다. 공식 `urdf_tutorial`도 시각 모델 → 가동 조인트 → 물리/충돌 → Xacro 순서를 제시한다. [urdfdom](https://github.com/ros/urdfdom) · [ROS URDF tutorial](https://github.com/ros/urdf_tutorial/tree/ros2)

## DAPIER 기존 자산에 연결

|자산|관찰된 상태|이 노트에서의 활용|주의|
|---|---|---|---|
|`jdcobot100_sim/urdf/jdcobot100.urdf`|XML 파싱 성공: 5 links, 4 joints. Onshape 자동 생성 표기와 `1e-09` 관성이 있다.|Onshape export 형태·mesh URI·계층 확인용 참고본.|JDcobot100이며 placeholder 관성이라 JDcobot300 제어/동역학 정답이 아니다.|
|`onshape/jdcobot100/reference/jdcobot100.urdf`|XML 파싱 성공: 5 links, 4 joints. reference의 mesh URI는 `package://assets/...`다.|reference export와 패키지형 mesh 경로를 비교하는 예제.|이 파일도 JDcobot100 참고본이다.|
|`ros_dd_ws/src/ros_dd_description/urdf/`|`ros_dd.xacro`와 생성된 `ros_dd.urdf`가 있다. 후자는 7 links, 6 joints로 XML 파싱 성공.|Xacro 원본을 수정하고 생성 URDF를 산출물로 취급하는 패턴, base/바퀴의 `fixed`·`continuous` joint 예제.|생성된 `ros_dd.urdf`는 직접 수정하지 않는다.|
|`2ARM_ROBOT/docs/ROBOT_MODEL_ASSET_AUDIT.md`|DAPIER 전용 양팔 Xacro와 장치별 calibration 분리를 이미 결정했다.|이 문서의 실측→Xacro→primitive collision→sim→hardware safety 순서를 구현 체크리스트로 삼는다.|JD-edu URDF/MJCF/메시 수치의 복사는 라이선스 확인 전 금지다.|

## 최소 URDF 예제 (이 문서에서 새로 작성)

아래는 문법과 좌표 관계만 배우기 위한 1관절 예제다. 실제 JDcobot300 수치·한계·관성을 의미하지 않는다.

```xml
<?xml version="1.0"?>
<robot name="learning_arm">
  <link name="base_link"/>
  <link name="arm_link">
    <visual>
      <origin xyz="0 0 0.10" rpy="0 0 0"/>
      <geometry><box size="0.04 0.04 0.20"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.10" rpy="0 0 0"/>
      <geometry><box size="0.04 0.04 0.20"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.10" rpy="0 0 0"/>
      <mass value="0.10"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="1.0" velocity="0.5"/>
  </joint>
</robot>
```

## 안전한 실습 순서

1. **읽기 전용**으로 두 JDcobot100 URDF와 `ros_dd.urdf`를 XML parse한다. 파싱 성공은 XML/URDF 구조의 첫 관문일 뿐, mesh·관성·실기체 일치 검증은 아니다.
2. DAPIER 전용 `dapier_dual_arm.xacro`는 새로 만들 때만 수정한다. `base_footprint → base → arm_a/arm_b/camera` tree와 12개 명령 이름을 표로 먼저 합의한다.
3. 링크 길이, 각 mount transform, 모터 ID/부호/zero, soft/hard limit, 질량/COM은 실측 근거와 calibration version을 함께 기록한다.
4. 생성 URDF에는 `check_urdf`를 실행한다. 공식 `urdfdom` 소스에 이 검사 도구가 포함되어 있으나 설치 유무는 환경마다 다르므로 `command -v check_urdf`가 성공할 때만 실행한다.

   ```bash
   check_urdf path/to/dapier_dual_arm.urdf
   ```

5. tree 이미지 도구가 설치돼 있으면 `urdf_to_graphiz`를 사용해 parent/child와 끊긴 link를 점검한다. urdfdom 공식 소스에 `urdf_to_graphviz.cpp`가 존재하지만, 배포 명령의 철자·출력 위치는 ROS 배포판에 따라 확인한 뒤 실행한다.

   ```bash
   command -v urdf_to_graphiz && urdf_to_graphiz path/to/dapier_dual_arm.urdf
   ```

6. RViz는 `robot_state_publisher`가 `robot_description`과 `/joint_states`로 TF를 계산하는 흐름에서만 연다. 공식 tutorial 패키지는 RViz에서 시각 모델, 가동 joint, 충돌/관성, Xacro를 단계적으로 검증한다. 현 저장소의 기존 launch를 우선 사용하고, 새 publisher나 모터 명령은 만들지 않는다.
7. RViz에서 기준 프레임, 양팔 축/영점, camera optical frame, mesh 누락을 확인한다. 다음 단계는 primitive collision이며, 실제 JDcobot 명령 publish는 **아직 금지**다.

## 확인 질문

1. `visual`과 `collision`을 같은 고해상도 mesh로 두면 어떤 검증·성능 문제가 생길 수 있는가?
2. `joint axis`의 부호가 실제 서보의 증가 방향과 반대일 때, URDF만 고치기 전에 함께 확인해야 할 calibration 항목은 무엇인가?
3. `check_urdf` 통과가 실제 로봇 안전을 보장하지 않는 이유는 무엇인가?
4. URDF 관성값과 MJCF actuator gain을 같은 파일에서 관리하지 않는 이유는 무엇인가?
5. 양팔 ACT action vector의 index와 URDF joint name을 어떻게 versioned contract로 고정할 것인가?

## 완료 기준

- [ ] 관련 책 3장과 3.1~3.3을 읽고, 책의 주장과 공식 ROS 보강을 구분했다.
- [ ] 세 로컬 URDF XML parse가 성공했고, JDcobot100 자산을 참고용으로만 표시했다.
- [ ] DAPIER 전용 트리와 12 DoF 이름/순서 초안을 팀이 승인했다.
- [ ] 실측 전에는 model joint limit을 hardware limit로 사용하지 않기로 확인했다.
- [ ] `check_urdf`와 선택적 graph/RViz 검증을 실제 설치된 도구에서 성공시킨 뒤 결과를 기록했다.
- [ ] 이후에도 safety supervisor 승인 없이 JDcobot command publisher를 만들거나 실행하지 않는다.

## 출처 (2026-08-24 확인)

### 책 원문

- [책 목차: jdCobot200 5축 로봇암 사용 매뉴얼](https://wikidocs.net/book/20199)
- [3장 jdCobot200 로봇암 URDF 생성, MUJOCO에 띄우기](https://wikidocs.net/366978)
- [3.1 Onshape를 이용해서 로봇 URDF 만들기](https://wikidocs.net/384466)
- [3.2 URDF를 MUJOCO에 올리기](https://wikidocs.net/384670)
- [3.3 로봇 그리퍼를 MUJOCO에서 사용하기](https://wikidocs.net/385219)
- [3.4 jdcobot200 FK 프로그래밍](https://wikidocs.net/385222)

### 외부 보강 (공식/1차 자료)

- [ROS urdfdom: URDF parser와 지원 형식](https://github.com/ros/urdfdom)
- [urdfdom source: `check_urdf.cpp`, `urdf_to_graphviz.cpp`](https://github.com/ros/urdfdom/tree/master/urdf_parser/src)
- [ROS `urdf_tutorial` ROS 2 브랜치](https://github.com/ros/urdf_tutorial/tree/ros2)
- [ROS `robot_state_publisher` 프로젝트](https://github.com/ros/robot_state_publisher)

### 저장소 내부 근거

- `jdcobot100_sim/urdf/jdcobot100.urdf`
- `onshape/jdcobot100/reference/jdcobot100.urdf`
- `ros_dd_ws/src/ros_dd_description/urdf/ros_dd.xacro`
- `ros_dd_ws/src/ros_dd_description/urdf/ros_dd.urdf`
- `2ARM_ROBOT/docs/ROBOT_MODEL_ASSET_AUDIT.md`
