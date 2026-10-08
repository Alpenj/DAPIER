# TJJ | 이동형 양팔 로봇의 조작·학습·Sim-to-Real

DAPIER 교육에서 진행하는 팀 프로젝트다. SO-101 양팔과 TurtleBot3 Waffle Pi를 연결해 물체를 인식하고 조작·운반하는 시스템을 개발한다. 장기적으로는 물체 정리와 청소를 이어 수행하는 가정용 서비스 로봇을 목표로 한다. 초기 박스·신발 과제는 개발 이력으로 보존하되, 프로젝트의 현재 범위를 신발 정리로만 한정하지 않는다.

**전형주 주 담당: 모방학습 · 시뮬레이션 · Sim-to-Real.** Visual SLAM의 설계·구현·코드 작업에도 협업 참여했다. 이동 모듈의 주 담당은 팀원 `shouttt1320`이다.

[전체 포트폴리오](../README.md) · [실험과 문제 해결 기록](docs/RESEARCH_SUMMARY_KO.md) · [이동·도킹 협업 코드](https://github.com/shouttt1320/Dapier_project_visaul_slam)

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

문서 갱신: 2026-10-08. 런타임·물리 파라미터·보정값·테스트·의존성은 변경하지 않았다.
