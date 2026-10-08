# sweepick | 이동형 양팔 로봇의 조작·학습·Sim-to-Real

DAPIER 교육에서 진행하는 팀 프로젝트의 공식 제품명은 **sweepick**이다. SO-101 양팔과 TurtleBot3 Waffle Pi를 연결해 물체를 인식하고 조작·운반하는 시스템을 개발한다. 장기적으로는 물체 정리와 청소를 이어 수행하는 가정용 서비스 로봇을 목표로 한다. 초기 박스·신발 과제는 개발 이력으로 보존하되, 프로젝트의 현재 범위를 신발 정리로만 한정하지 않는다.

**전형주 주 담당: 모방학습 · 시뮬레이션 · Sim-to-Real.** Visual SLAM의 설계·구현·코드 작업에도 협업 참여했다. 이동 모듈의 주 담당은 팀원 `shouttt1320`이다.

[전체 포트폴리오](../README.md) · [sweepick 제품 소스](sweepick/README.md) · [실험과 문제 해결 기록](docs/RESEARCH_SUMMARY_KO.md) · [이동·도킹 협업 코드](https://github.com/shouttt1320/Dapier_project_visaul_slam)

## 현재 제품 코드와 실행 경계

최신 native 소스는 [`sweepick/src/sweepick`](sweepick/src/sweepick/)에서 기능별로 찾을 수 있다. 기존 추적 파일에는 CI·ROS package/resource·문서를 포함해 `2ARM_ROBOT` 경로 참조가 약 160곳 있어 이번 정리에서는 상위 경로를 유지했다. 기존 ROS 2 package인 [`src/shoe_sorting_data`](src/shoe_sorting_data/), [`research`](research/), [`sim`](sim/)과 [`robotwin`](robotwin/)도 호환성과 연구 이력을 위해 제자리에 둔다.

현재 통합 진입점의 모듈 명령은 다음과 같다.

```bash
PYTHONPATH=2ARM_ROBOT/sweepick/src \
  python -m sweepick.integration.sweepick_manipulation_session --help
```

이 모듈의 실행 경로는 device writer를 통해 실제 `Goal_Position` write까지 이어질 수 있다. 위 명령은 parser 도움말만 확인하며, 실제 실행은 별도의 현장 승인과 장치별 필수 입력이 필요하다.

| 기능 | 대표 경로 | 역할과 경계 |
|---|---|---|
| 관측 | [`perception`](sweepick/src/sweepick/perception/) | OS30A·손목 카메라와 물체 영역 관측. 센서 관측과 SIM truth를 구분한다. |
| 조작 | [`sweepick_bimanual_handover.py`](sweepick/src/sweepick/manipulation/sweepick_bimanual_handover.py) | `FullEpisodeHandoff`를 감싸는 후단 연결부다. 물리 제어기를 새로 구현하지 않고 공통 실행기를 호출한다. |
| 공통 제어 | [`sweepick_trajectory_executor.py`](sweepick/src/sweepick/control/sweepick_trajectory_executor.py) | 양팔이 공유하는 `run`·`grip` 실행 경로다. 좌우별 실행기 사본을 두지 않는다. |
| 기록 | [`sweepick_episode_recorder.py`](sweepick/src/sweepick/recording/sweepick_episode_recorder.py) | 관측·명령·실제 feedback을 한 세션 기록으로 묶는다. 제어기 역할은 하지 않는다. |
| 통합 | [`sweepick_manipulation_session.py`](sweepick/src/sweepick/integration/sweepick_manipulation_session.py) | 관측부터 후단 조작과 기록까지 연결하는 현재 entrypoint다. |
| 학습 | [`learning/README.md`](sweepick/src/sweepick/learning/README.md) | 새 구현 없이 기존 학습·추론·SIM·legacy ACT 위치와 근거를 안내한다. |

목표 흐름은 실제 관측 → 왼팔 집기 → 오른팔 수신과 하중 이전 → 왼손 해제 → 오른팔 배치 → STOW → 작업 영역 재관측이다. 이동·도킹은 외부 팀 모듈과의 통합 경계이며, 이 저장소의 양팔 조작 성공으로 대신 계산하지 않는다.

이번 구조 정리는 native 소스를 공개 경로에 포함하고 역할을 찾기 쉽게 만든 변경이다. 구조 정리 자체는 REAL 승격이나 새 실물 검증이 아니다. run14는 센서 계획 기반 무보조 왼팔 집기·들기와 hold 관측 기록이며 자동 `placed_back` 완료는 확인되지 않았다. 전체 양팔 성공이나 ACT 성공 결과도 아직 제출되지 않았다. 오른쪽 REAL jaw profile, 독립 support 근거와 held-object pose는 계속 `MISSING`이며, 후보·SIM·합성 기록으로 채우지 않는다.

## 풀고 있는 문제

로봇이 물체에 닿았다는 사실만으로 들어 올릴 수 있는 것은 아니었다. 파지 이후의 압축량, 계획한 손끝 위치와 실행 후 위치의 차이, 시뮬레이션 접촉 설정에 따른 미끄러짐을 나누어 조사했다. 카메라가 본 위치를 관절 명령으로 옮기는 과정에서는 좌표계, 영점, 단위와 관측 시각도 함께 확인했다.

학습 쪽에서는 시연을 저장하는 것에서 끝내지 않고, 측정 상태와 전송 명령의 의미, 에피소드 경계, 전처리와 행동 묶음의 실행을 연결하는 데 집중한다. 모델의 학습 실행, 보류 데이터 예측과 폐루프 조작 성공은 별도로 평가한다.

## 구성과 코드 위치

| 영역 | 다루는 내용 | 코드·문서 |
|---|---|---|
| 양팔 모델 | SO-101 양팔, 기본형·PGripper 구성, 이동형·책상형 장면 | [MuJoCo 모델](sim/mobile_dual_so101/README.md), [기준 배치](sim/mobile_dual_so101/INTEGRATION_SCENES.md) |
| 데이터 | 관절 순서·단위, 영상과 시각, 에피소드 품질과 출처 | [shoe_sorting_data](src/shoe_sorting_data/) |
| 정책 | 행동 묶음, CVAE 학습·체크포인트·추론을 다루는 작은 native ACT 구현 | [dapier_native_act.py](src/shoe_sorting_data/shoe_sorting_data/dapier_native_act.py) |
| 조작 검증 | 접촉·하중 지지·끝점 오차·미끄러짐을 분리한 비교 | [연구 기록과 고정 소스](docs/RESEARCH_SUMMARY_KO.md) |
| 이동·도킹 | RTAB-Map RGB-D 매핑·localization, Nav2와 듀얼 ArUco 도킹 | [팀 이동 모듈](https://github.com/shouttt1320/Dapier_project_visaul_slam) |
| 장치 실행 | 보정·명령 제한·피드백·정지 조건 | [공통 하드웨어 안전 절차](../docs/HARDWARE_SAFETY_KO.md) |

native ACT 코드와 LeRobot을 재사용한 다른 학습 실험은 같은 구현으로 취급하지 않는다. native 구현은 공식 ACT 체크포인트 호환을 주장하지 않는다.

### 장비 역할과 과거 설정

최근 조작 연구에서는 SO-101 양팔·PGripper, 작업공간용 OS30A RGB-D와 양손목 RGB를 다룬다. 이동 모듈의 Astra S는 Visual SLAM·도킹용 센서다. 두 RGB-D 센서의 역할을 섞지 않는다.

`main`의 [hardware_roles.json](config/hardware_roles.json)은 `effective_date=2026-09-03`이며 작업공간 카메라를 H201로 기록한 **당시 구성**이다. 일부 모델 README와 실습 절차도 그 배치를 설명한다. 이후 OS30A·PGripper 연구의 설명을 읽었다고 이 설정 파일이나 모든 실행기가 갱신된 것으로 해석하지 않는다. 실제 실행에는 해당 장치와 소스 revision의 설정·보정이 필요하다. 이 문서 갱신에서는 설정이나 런타임을 변경하지 않았다.

## 공개 구현과 실험 결과의 구분

| 근거 | 확인한 범위 | 별도로 남은 것 |
|---|---|---|
| `main`과 병합된 [PR #64](https://github.com/Alpenj/DAPIER/pull/64)·[PR #65](https://github.com/Alpenj/DAPIER/pull/65) | 접촉·중력 wrench 분석, 그리퍼 단독 파지와 솔버 민감도 진단·회귀 기록 | 기본 실행기의 전체 조작 성공과 실물 성공 |
| 2026-09-21 [후보 커밋 f6b61db](https://github.com/Alpenj/DAPIER/commit/f6b61dbc81273f6d775d96f7950461fac9113cbb), [PR #68](https://github.com/Alpenj/DAPIER/pull/68) | normal HOME→접근→CLOSE→LIFT→HOLD의 단일 연속 SIM. 18,203 step / 36.406 s, HOLD 3.000 s | 여러 초기조건의 신뢰도, ACT 정책 성공, 실물 성공. PR은 닫힘·미병합 |
| [PR #69](https://github.com/Alpenj/DAPIER/pull/69) | 센서·손목 후보와 제한된 native 실행 경로, 지연·정지 MOCK 피드백 연결 기록 | 센서 기반 실물 조작 검증. PR은 닫힘·미병합 |
| 2026-09-30 [후보 커밋 3720da1](https://github.com/Alpenj/DAPIER/commit/3720da1ee678793480035cbfa3db96a546c73671), [PR #70](https://github.com/Alpenj/DAPIER/pull/70) | 저장 관절값의 변환 비교와 팔꿈치 기준 측정 준비 | 실물 영점 채택·동적 검증. PR은 닫힘·미병합이며 독립 실행에 필요한 비공개 입력은 미포함 |
| [이동 모듈 소스](https://github.com/shouttt1320/Dapier_project_visaul_slam/tree/3942b81a0d3131aeed6d97fad83c2a5b389ec18a) | RTAB-Map localization·Nav2·정밀 도킹을 구성한 코드 | 현재 장비에서의 재현, 양팔 통합 과제와 실제 전기적 충전 성공 |

PR 상태 확인일은 2026-10-08이다. 연구 PR을 닫은 것과 `main`에 병합한 것은 다르다. 위 후보 결과는 고정 커밋의 연구 이력으로 남기며, 저장소를 새로 받은 사람이 기본 실행만으로 같은 결과를 얻는다고 안내하지 않는다.

## 다음 검증

관측한 물체 위치, 측정 관절 상태와 실제 장비의 좌표 대응을 연결하고 제한된 접근·집기·들기·유지를 평가한다. 학습 정책은 같은 데이터 분할·전처리·성공 기준에서 기준 제어기와 비교한다. 이동과 조작의 결합에서는 도착·정지, 조작 결과와 후속 이동 조건을 확인한다.

정리·청소는 확장 목표다. 개별 모듈의 코드, 한 조건의 SIM 성공이나 장치 연결 기록을 전체 가정용 서비스 과제 완료로 쓰지 않는다.

## 실행 안내와 과거 실습

현재 코드의 사용법은 해당 하위 모듈 README와 [안전 절차](../docs/HARDWARE_SAFETY_KO.md)를 먼저 확인한다. 개인 보정, 장치 식별자와 원시 영상·데이터는 공개하지 않는다. 문서의 예시 명령은 실물 실행 승인이 아니다.

초기 ROS 2 환경 점검, 합성 토픽 녹화, golden episode 생성과 장치 snapshot 절차는 [갱신 전 README의 고정본](https://github.com/Alpenj/DAPIER/blob/9b1138dbc462d293cbde62961d4d3534a71b2061/2ARM_ROBOT/README.md)에 보존했다. 그 문서의 신발·H201 구성과 실행 제한은 당시 소스의 안내다. 과거 실행 절차를 최신 장비 설정으로 대신하지 않는다.

외부 라이브러리와 자산은 각 폴더의 출처·라이선스를 따른다. 팀 이동 모듈의 전체 구현을 개인 단독 성과로 계산하지 않는다.

구조·문서 정리: 2026-10-09. 활성 런타임·물리 파라미터·보정값은 변경하지 않았다. 새 소스 package의 경로·호환 import·설치 의존성과 무장치 검사는 제품 README에 구분했다.
