# DAPIER | 전형주 · Physical AI와 로봇 소프트웨어

DAPIER 국비교육에서 배운 내용을 코드와 실험으로 확인하고, 개인 기여를 정리한 포트폴리오 저장소다. 모방학습, 물리 시뮬레이션, Sim-to-Real을 중심으로 로봇의 관측·데이터·제어·평가를 연결하고 있다.

대표 팀 프로젝트는 **sweepick — 이동형 양팔 로봇**이다. SO-101 양팔의 물체 조작과 TurtleBot3의 이동을 연결하고, 물체 정리에서 가정용 서비스 작업으로 확장하는 것을 목표로 한다. 초기 신발 정리 과제는 개발 이력으로 남겼으며, 현재 프로젝트를 그 과제 하나로 한정하지 않는다.

[포트폴리오·이력서](https://julianjeonresume.netlify.app/) · [sweepick 양팔 프로젝트](2ARM_ROBOT/README.md) · [제품 소스](2ARM_ROBOT/sweepick/README.md) · [문제 해결과 실험 근거](2ARM_ROBOT/docs/RESEARCH_SUMMARY_KO.md) · [학습 아카이브](https://github.com/Alpenj/physical-ai-lab)

## 먼저 볼 내용

| 확인할 내용 | 코드와 문서 |
|---|---|
| 프로젝트 목표, 담당 범위, 구현과 검증 상태 | [sweepick / 2ARM_ROBOT](2ARM_ROBOT/README.md) |
| 파지 실패·끝점 오차·미끄러짐을 조사한 과정 | [양팔 조작 연구 기록](2ARM_ROBOT/docs/RESEARCH_SUMMARY_KO.md) |
| 양팔 모델, 접촉과 경로 검사 | [MuJoCo 모델](2ARM_ROBOT/sim/mobile_dual_so101/README.md) |
| 에피소드·영상·행동 묶음 처리와 정책 코드 | [데이터·학습 패키지](2ARM_ROBOT/src/shoe_sorting_data/) |
| Visual SLAM·Nav2·정밀 도킹 협업 | [팀 이동 모듈](https://github.com/shouttt1320/Dapier_project_visaul_slam) |

## 내가 맡은 부분

**주 담당은 모방학습·시뮬레이션·Sim-to-Real이다.** 시연 데이터의 의미와 시간 정렬, 정책 입력과 출력, 물리 모델, 접촉·경로 검사와 평가 기준을 다룬다. 문제가 생기면 조건을 고정한 비교를 통해 데이터, 계획, 제어와 시뮬레이션의 영향을 나누어 확인한다.

Visual SLAM 모듈의 설계·구현·코드 작업에도 협업 참여했다. 이동 모듈의 주 담당은 팀원 `shouttt1320`이며, 해당 모듈과 내 양팔 조작 작업의 기여를 구분한다. 교육 예제·외부 라이브러리·팀 작업을 모두 독자 개발 성과로 계산하지 않는다.

## 확인한 결과와 적용 범위

- **공개 구현:** 양팔 MuJoCo 모델, 에피소드 품질 검사, ACT 계열 학습 코드와 접촉 분석·회귀 검사를 읽을 수 있다. 파일이 있다는 사실과 현재 장비에서의 실행 성공은 구분한다.
- **보존한 실험:** 2026-09-21 후보 커밋에서 normal HOME부터 접근·집기·들기·3초 유지까지 한 조건의 연속 SIM 결과를 기록했다. [PR #68](https://github.com/Alpenj/DAPIER/pull/68)의 결과이며, 해당 PR은 닫혔지만 병합되지 않았다. 현재 `main`의 기본 실행기 성능으로 표시하지 않는다.
- **이동 모듈:** 협업 저장소에는 RTAB-Map RGB-D 매핑·localization, Nav2, 듀얼 ArUco 도킹과 GUI 코드가 있다. 양팔까지 연결한 실물 통합 성공률이나 전기적 충전 성공을 뜻하지 않는다.
- **진행 과제:** 실물 좌표·영점·관측 정합과 제한된 실행을 연결하고, 학습 정책과 이동·조작 통합을 별도로 평가한다. 실물 전체 과제의 완료를 주장하지 않는다.

수치, 비교 조건과 소스 revision은 [연구 기록](2ARM_ROBOT/docs/RESEARCH_SUMMARY_KO.md)에 모았다. 닫힌 연구 PR의 결과를 `main`에 병합된 기능과 섞지 않는다.

## 교육·실습 코드

| 폴더 | 내용 |
|---|---|
| [so101](so101/README.md) · [so101_ros2](so101_ros2/README.md) | SO-101 실습, LeRobot 연동, ROS 2 제어 |
| [so101_imitation_learning](so101_imitation_learning/) | PyTorch·모방학습 교육 코드 |
| [deepThinkCar_mini](deepThinkCar_mini/) | 자율주행 자동차 실습 |
| [turtlebot3_ws](turtlebot3_ws/README.md) · [ros_dd_ws](ros_dd_ws/README.md) | 이동로봇 시뮬레이션, SLAM·내비게이션 학습 |
| [jdcobot100_sim](jdcobot100_sim/README.md) · [ros_arm](ros_arm/README.md) | 로봇암 시뮬레이션과 Arduino 제어 |
| [dapier_sim_first](dapier_sim_first/README.md) · [casino_dealer](casino_dealer/README.md) | 한 팔 조작과 카드 딜러 실험 |

실행 환경은 각 폴더의 README를 확인한다. 오래된 실습의 장비 구성·파라미터는 해당 날짜의 조건이며, 현재 장비에 그대로 적용하지 않는다. 실물 접근·제어는 [하드웨어 안전 절차](docs/HARDWARE_SAFETY_KO.md)를 따른다.

외부 코드·자산의 출처와 라이선스는 해당 폴더에 보존했다. 별도 학습 포크를 이 저장소로 옮긴 내역은 [통합 기록](docs/FORK_MIGRATION_20260907_KO.md)에서 확인할 수 있다.

문서 갱신: 2026-10-08. 공개 소스와 보존된 실험 기록을 연결한 소개이며, 이번 문서 갱신을 새로운 학습·실물 시험 결과로 세지 않는다.
