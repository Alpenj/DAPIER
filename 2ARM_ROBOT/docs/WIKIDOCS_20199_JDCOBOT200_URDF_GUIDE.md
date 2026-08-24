# WikiDocs 20199 기반 JDcobot200 URDF·MuJoCo 실습 가이드

확인일: 2026-08-24

대상: WikiDocs 「jdCobot200 5축 로봇암 사용 매뉴얼」과 이 책이 직접 연결한 `JD-edu/jdcobot200_imitation_learning` 공개 저장소

## 목적과 경계

이 문서는 JDcobot200의 URDF 생성·MuJoCo 변환·그리퍼 연동 흐름을 **학습용 참조**로 정리한다. DAPIER 실물은 JDcobot300 양팔이므로, 200의 링크 길이·질량·관성·관절 제한·서보 게인·메시를 정답으로 복사하지 않는다. 특히 이 문서에는 외부 URDF, MJCF, STL, 코드의 전문을 포함하지 않으며 원문 링크와 검증 절차만 제공한다.

## 200 전용 페이지·소스 지도

| 확인 대상 | 제공 내용 | DAPIER에서 쓰는 방식 |
|---|---|---|
| [책 목차](https://wikidocs.net/book/20199) | 3장이 URDF 생성, MuJoCo, 그리퍼, FK 순서임을 확인한다. | 학습 순서의 기준으로만 사용한다. |
| [3장 표지](https://wikidocs.net/366978) | “jdCobot200 로봇암 URDF 생성, MUJOCO에 띄우기” 범위를 가리킨다. 본문은 작성 중이다. | 하위 실습의 묶음 이름으로만 기록한다. |
| [3.1 Onshape → URDF](https://wikidocs.net/384466) | Onshape assembly/mate, `onshape-to-robot`, 생성 로그, PyBullet 시각화, 변환 후 MJCF 연결을 설명한다. | CAD 원본에서 자체 300 URDF를 만드는 절차와 점검 항목을 배운다. |
| [3.2 URDF → MuJoCo](https://wikidocs.net/384670) | 변환 뒤 actuator 누락, 제어 차원, 게인·질량·관성으로 인한 불안정, calibration default를 다룬다. | 300용 MJCF에 actuator를 명시하고 낮은 값부터 검증하는 원칙을 적용한다. |
| [3.3 그리퍼 MuJoCo](https://wikidocs.net/385219) | 메시 자산, scene/model 분리, 5회전 관절+그리퍼 제어 벡터, 양 손가락 연동을 설명한다. | 300 그리퍼의 실제 기구학을 재측정한 뒤 한 actuator와 기구 제약을 설계한다. |
| [3.4 FK](https://wikidocs.net/385222) | 2026-08-24 확인 시 본문이 “작성중”이다. | FK 수치·코드를 채택하지 않는다. |
| [JD-edu 공개 저장소](https://github.com/JD-edu/jdcobot200_imitation_learning) | 200 하드웨어 제어부터 URDF, MJCF, 그리퍼, FK/IK/시연 데이터까지의 예제 폴더를 제공한다. | 폴더 구조와 검증 흐름만 참조한다. |
| [URDF 생성 폴더](https://github.com/JD-edu/jdcobot200_imitation_learning/tree/main/2_jdcobot200_URDF_gen) | `assets/`, `config.json`, `pybullet_display.py`, `robot.urdf`가 있다. | 공개 자산을 복사하지 않고 자체 CAD export의 산출물 비교에만 사용한다. |
| [URDF→MJCF 폴더](https://github.com/JD-edu/jdcobot200_imitation_learning/tree/main/3_URDF_to_MJCF) | `assets/`, URDF/MJCF 파일, 변환 스크립트가 있다. | 자체 300 모델에 같은 “변환→로드→확인” 순서만 적용한다. |
| [그리퍼 MuJoCo 폴더](https://github.com/JD-edu/jdcobot200_imitation_learning/tree/main/5_gripper_MUJOCO_move) | 200용 그리퍼 모델·환경·구동 예제를 제공한다. | 양 손가락 반대방향 연동이라는 개념만 참고한다. |
| [ROS urdfdom](https://github.com/ros/urdfdom) | ROS의 URDF core data structure와 XML parser를 제공한다. | `check_urdf`로 URDF 문법·트리 오류를 먼저 잡되, 실제 기구 정확성 검사는 별도로 수행한다. |

## 안전한 참조·다운로드 원칙

1. 외부 저장소는 브라우저로 원본을 열어 구조와 설명을 확인한다. 라이선스 확인 없이 파일을 DAPIER에 복사·재배포하지 않는다.
2. Onshape 키는 Git, Notion, 채팅, `config.json`, shell history에 기록하지 않는다. 필요한 경우 개인 계정에서 최소 읽기 권한의 키를 발급하고 환경변수로만 주입한다.
3. 노출 의심 키는 즉시 Onshape에서 폐기하고 재발급한다. WikiDocs도 키는 placeholder로만 제시한다.
4. `onshape-to-robot` 실행은 개인 소유 또는 사용 승인을 받은 300 CAD 문서에서만 한다. 타인의 문서 URL, workspace ID, element ID도 공개 자산으로 취급하지 않는다.
5. 다운로드한 파일은 별도 임시 작업공간에서만 검사한다. DAPIER에는 직접 측정·재작성한 300 자산과 출처 기록만 넣는다.

### 라이선스 상태

2026-08-24에 `JD-edu/jdcobot200_imitation_learning`의 공개 루트 목록과 `LICENSE` 경로를 확인했다. 저장소 루트 목록에 라이선스 파일이 표시되지 않았고 `LICENSE` URL도 404였다. 따라서 **라이선스가 확인되지 않았다**. 공개 저장소라는 사실만으로 URDF/MJCF/STL/코드의 복사·수정·배포 권한이 생기지 않는다. 재사용이 필요하면 저작권자에게 서면 허가를 받고, 허가 범위와 원본 commit을 별도 기록한다.

## URDF → MJCF → 그리퍼 작업 흐름

### 1. 300 CAD에서 URDF 생성

- 최상위 assembly의 링크와 mate를 정리하고, 회전축이 CAD 좌표계와 일치하는지 확인한다.
- 움직이는 관절 이름·축·부모/자식 링크·limit을 먼저 표로 확정한다. ROS URDF는 로봇의 geometry와 organization을 XML로 표현하며, `JointState`와 `robot_state_publisher`는 그 joint 이름으로 TF를 계산한다. [ROS 2 URDF 문서](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html)
- CAD 파트의 material/mass가 빠졌다면 변환을 멈춘다. 3.1은 이것이 0에 가까운 질량과 시뮬레이터 붕괴를 만들 수 있다고 경고한다.
- `onshape-to-robot` 결과의 링크 수, joint 수, 축 방향, mesh URI를 화면에서 검사한다. 생성 성공은 물리 정확성의 증거가 아니다.

### 2. URDF 검증과 ROS 시각화

- XML well-formed 검사 뒤 `check_urdf <300.urdf>`를 실행한다. 공식 urdfdom은 URDF용 core data structure와 XML parser를 제공하므로, 이 검사는 **파싱·링크 트리 검증**이다. 관성, 축 부호, mesh 단위, 실물 일치는 검증하지 않는다. [urdfdom](https://github.com/ros/urdfdom)
- ROS 환경에서는 `robot_state_publisher`에 300 URDF를 로드한다.
- `joint_state_publisher` 또는 실제 feedback의 `JointState` 이름과 URDF joint 이름을 정확히 일치시킨다.
- RViz에서 각 관절을 하나씩 작은 범위로 바꾸며 링크 방향·원점·충돌 형상을 확인한다. ROS 공식 튜토리얼도 URDF, JointState, `robot_state_publisher`로 TF/RViz 검증을 구성한다. [ROS 2 예제](https://docs.ros.org/en/ros2_documentation/rolling/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-cpp.html)

### 3. MJCF 변환과 안정화

- MuJoCo는 URDF를 로드할 수 있으나, 변환 결과에 actuator가 충분히 생긴다고 가정하지 않는다. 3.2처럼 `model.nu`, `data.ctrl`과 actuator 이름·순서를 먼저 확인한다.
- 각 300 관절 actuator에 300의 **실측 limit와 보수적인 force/position 범위**를 명시한다. 200 문서의 숫자는 사용하지 않는다.
- 초기 자세와 목표 자세 차이를 작게 유지하고, 낮은 stiffness부터 one-joint simulation으로 게인을 올린다. 링크 질량·관성·damping·armature는 실측 또는 시스템 식별 전까지 임시값임을 모델 메타데이터에 표시한다.
- MuJoCo 공식 문서는 default class로 공통 속성을 한 곳에서 관리할 수 있음을 설명한다. 따라서 calibration 값은 300 MJCF의 별도 default class에 두고 production 값을 덮어쓰지 않는다. [MuJoCo modeling 문서](https://mujoco.readthedocs.io/en/stable/modeling.html)

### 4. 그리퍼와 scene 분리

- `scene.xml`(바닥·카메라·물체)과 `jdcobot300.xml`(로봇)을 include로 분리한다. 3.3의 모델/환경 분리 원칙을 따른다.
- 실제 300 그리퍼가 양 손가락 대칭 기구면, 하나의 제어 입력과 기구학적 반대방향 연동을 사용한다. 그러나 equality 계수, 축, opening limit는 반드시 실측한다.
- 충돌 mesh는 시각 mesh와 분리하거나 단순화하고, 핀치·자기충돌·작업대 충돌을 장면별로 확인한다.

## 그대로 재사용할 것과 300에서 재측정할 것

| 구분 | 항목 |
|---|---|
| 재사용 가능 | CAD→URDF→시각화→MJCF→actuator 검증의 순서, scene/model 분리, actuator 존재 검사, 저게인부터 시작, 그리퍼 연동을 별도 검증하는 방식 |
| 반드시 재측정 | 링크 길이/원점/축, joint 이름·부호·gear ratio, hard/soft limit, 질량·질량중심·관성, collision 형상, servo ID/통신 규약, torque·속도·전류 제한, 그리퍼 stroke와 손가락 연동식 |
| 채택 금지 | JDcobot200 URDF·MJCF·STL·코드 전문, 예제의 수치 게인·force range·관절 limit를 300의 정답으로 사용하는 것 |

## 직접 실습 체크리스트

- [ ] 300 CAD와 실물을 대조해 base부터 end-effector까지 링크/조인트 표를 작성했다.
- [ ] 각 축의 양(+) 방향과 zero pose를 사진·측정값으로 기록했다.
- [ ] 300 URDF가 XML 검사와 ROS `robot_state_publisher` 로드에 성공했다.
- [ ] RViz에서 한 번에 한 관절만 작은 범위로 움직여 TF/mesh/axis를 확인했다.
- [ ] 300 MJCF가 로드되고 actuator 수·이름·control index가 설계표와 일치한다.
- [ ] 모든 joint와 그리퍼의 ctrlrange/forcerange는 300 실측값 또는 명시된 임시 보수값이다.
- [ ] gravity on에서 NaN/Inf/huge QACC 없이 정지 자세를 유지한다.
- [ ] 그리퍼 opening/closing, 물체 미접촉, 자기충돌, 작업대 충돌을 별도 시험했다.
- [ ] 외부 200 자산을 저장소에 복사하지 않았고, 필요 자산의 허가 여부를 기록했다.

## 완료 판정 질문

1. URDF의 모든 movable joint가 실제 300 관절 하나와 일대일 대응하는가?
2. 실제 encoder feedback의 이름·순서·부호가 URDF와 MJCF, safety supervisor의 canonical order와 모두 일치하는가?
3. MJCF의 actuator 수가 제어 벡터 차원과 일치하고, `data.ctrl`에 빈 채널이 없는가?
4. gravity와 작은 목표 변화에서 수치 폭주, 관통, 비현실적 흔들림이 없는가?
5. 그리퍼의 두 손가락이 실제 기구와 동일한 방향·stroke·limit로 동작하는가?
6. 각 수치가 200 예제를 복사한 값이 아니라 300 실측값 또는 명시된 임시값인가?
7. 실물 command는 아직 발행하지 않았는가? 이 문서의 완료는 모델 검증 완료이며, 실물 구동 권한이 아니다.

모든 질문에 증거 파일(측정표, RViz 캡처, MuJoCo load log, actuator 표)로 답할 수 있을 때 URDF/MJCF 학습 단계를 완료한다.

## 확인한 원문 링크

- [WikiDocs 책](https://wikidocs.net/book/20199), [3장](https://wikidocs.net/366978), [3.1](https://wikidocs.net/384466), [3.2](https://wikidocs.net/384670), [3.3](https://wikidocs.net/385219), [3.4](https://wikidocs.net/385222)
- [JD-edu/jdcobot200_imitation_learning](https://github.com/JD-edu/jdcobot200_imitation_learning), [URDF 생성 디렉터리](https://github.com/JD-edu/jdcobot200_imitation_learning/tree/main/2_jdcobot200_URDF_gen), [MJCF 변환 디렉터리](https://github.com/JD-edu/jdcobot200_imitation_learning/tree/main/3_URDF_to_MJCF), [그리퍼 디렉터리](https://github.com/JD-edu/jdcobot200_imitation_learning/tree/main/5_gripper_MUJOCO_move)
- [ROS 2 URDF](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html), [ROS 2 robot_state_publisher 예제](https://docs.ros.org/en/ros2_documentation/rolling/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-cpp.html), [MuJoCo Modeling](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [ROS urdfdom](https://github.com/ros/urdfdom), [check_urdf 소스](https://github.com/ros/urdfdom/blob/master/urdf_parser/src/check_urdf.cpp)
