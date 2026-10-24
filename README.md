# DAPIER | 전형주 · Physical AI와 로봇 소프트웨어

DAPIER 국비교육에서 배운 내용을 코드와 실험으로 확인하고, 개인 기여를 정리한 포트폴리오 저장소다. 모방학습, 물리 시뮬레이션, Sim-to-Real을 중심으로 로봇의 관측·데이터·제어·평가를 연결하고 있다.

대표 팀 프로젝트는 **sweepick — 이동형 양팔 청소 로봇**이다(팀 프로젝트명 TJJ). 바닥의 물체를 보고 접근해 집고, 지정한 곳에 놓은 뒤 그 자리를 청소하고 충전대로 돌아가는 저비용 모바일 매니퓰레이터를 목표로 한다. 2026-10-16부터 내 팀의 범위는 **상체(SO-101 양팔·그리퍼·상부 카메라)의 집기·운반·놓기**이고, 이동 베이스·Visual SLAM·도킹은 다른 팀이 맡는다. 초기 신발 정리 과제는 개발 이력으로만 남겼다.

[포트폴리오·이력서](https://julianjeonresume.netlify.app/) · [sweepick 양팔 프로젝트](2ARM_ROBOT/README.md) · [sweepick 실행 소스(검토 브랜치)](https://github.com/Alpenj/DAPIER/tree/pro/sweepick-review-20261015/2ARM_ROBOT/sweepick) · [문제 해결과 실험 근거](2ARM_ROBOT/docs/RESEARCH_SUMMARY_KO.md) · [학습 아카이브](https://github.com/Alpenj/physical-ai-lab)

## 먼저 볼 내용

| 확인할 내용 | 코드와 문서 |
|---|---|
| 프로젝트 목표, 담당 범위, 구현과 검증 상태 | [sweepick / 2ARM_ROBOT](2ARM_ROBOT/README.md) |
| 실물 집기 실행기·콘솔·저장 기록 검수 코드 | [2ARM_ROBOT/sweepick (검토 브랜치, main 미병합)](https://github.com/Alpenj/DAPIER/tree/pro/sweepick-review-20261015/2ARM_ROBOT/sweepick) |
| 파지 실패·끝점 오차·미끄러짐을 조사한 과정 | [양팔 조작 연구 기록](2ARM_ROBOT/docs/RESEARCH_SUMMARY_KO.md) |
| 양팔 모델, 접촉과 경로 검사 | [MuJoCo 모델](2ARM_ROBOT/sim/mobile_dual_so101/README.md) |
| Visual SLAM·Nav2·정밀 도킹 협업 | [팀 이동 모듈](https://github.com/shouttt1320/Dapier_project_visaul_slam) |

## 내가 맡은 부분

**주 담당은 모방학습·시뮬레이션·Sim-to-Real이다.** 시연 데이터의 의미와 시간 정렬, 정책 입력과 출력, 물리 모델, 접촉·경로 검사와 평가 기준을 다룬다. 문제가 생기면 조건을 고정한 비교를 통해 데이터, 계획, 제어와 시뮬레이션의 영향을 나누어 확인한다.

Visual SLAM 모듈의 설계·구현·코드 작업에도 협업 참여했다. 이동 모듈의 주 담당은 팀원 `shouttt1320`이며, 해당 모듈과 내 양팔 조작 작업의 기여를 구분한다. 교육 예제·외부 라이브러리·팀 작업을 모두 독자 개발 성과로 계산하지 않는다.

## 확인한 결과와 적용 범위

| 항목 | 상태 | 내용 |
|---|---|---|
| 실물 집기 (규칙 기반 교사) | 실물 확인 | 2026-10-08 왼팔이 사람 도움 없이 블록을 집고·들고·내려놓은 첫 1회. OS30A 깊이로 블록 위치를 잡고 IK로 접근한 뒤 손목 카메라로 정렬했다. |
| 자동 교사 시연 수집 | 실물 확인 | 2026-10-18~23 자동 교사(관측 → 손목 영상 정렬 → 하강 → 집기 → 들기 → 운반 → 놓기 → 복귀)를 약 170회 실행해, 3-view(손목 RGB 2 + OS30A RGB-D)와 양팔 측정값·실제 서보 명령을 함께 기록했다. 교사 판정 성공은 빨강 40 mm 큐브 약 50회, 검정 30 mm 큐브 20회, 검정 막대 22회다. |
| ACT 접근 + 교사 마무리 | 실물 1회 | 2026-10-23 학습 정책(ACT)이 블록 위까지 접근하고, 정체를 감지하면 교사 실행기가 이어받아 집기~운반~놓기~복귀까지 마쳤다. ACT 단독 집기 성공은 아직 없다. |
| ACT 재학습 | 진행 중 | 위 실물 시연 원본에서 학습 데이터를 다시 만들어(교사 정지 구간 제외, 날짜 단위 held-out) 헤드 점군 + 양 손목 RGB 입력 ACT를 학습하고 있다. 결과는 아직 없다. |
| 이동·도킹 | 다른 팀 담당 | 협업 저장소의 RTAB-Map RGB-D 매핑·localization, Nav2, 듀얼 ArUco 도킹 코드. 상체와 연결한 실물 통합 성공은 아직 없다. |

- 상태는 `실물 확인 / 실물 1회 / SIM 확인 / 진행 중`으로 나눠 적었다. 한 번의 성공을 반복 성공률로 쓰지 않는다.
- sweepick 실행 소스는 `pro/sweepick-*` 작업 브랜치에서 검수하고 있으며 아직 `main`에 병합하지 않았다. `main`의 기존 코드(신발 정리 데이터 패키지, native ACT 등)는 이전 단계의 구현이다.
- 2026-09-21 SIM 연속 조작 결과([PR #68](https://github.com/Alpenj/DAPIER/pull/68), 닫힘·미병합) 등 이전 수치와 비교 조건은 [연구 기록](2ARM_ROBOT/docs/RESEARCH_SUMMARY_KO.md)에 모았다.

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

문서 갱신: 2026-10-24. 프로젝트 소개를 sweepick 기준으로 바꾸고 10/08~10/23 실물 진행을 요약했다. 코드·설정·실험 결과 파일은 이 갱신에서 바꾸지 않았다.
