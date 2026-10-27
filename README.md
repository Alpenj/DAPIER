# DAPIER | 전형주 · Physical AI와 로봇 소프트웨어

DAPIER에서 배운 내용을 로봇 연동, 시뮬레이션과 데이터 검증으로 확인하는 포트폴리오 저장소다. 관측을 학습 데이터로 만들고, 학습 정책의 출력이 실제 로봇 명령으로 이어지는 과정을 중심으로 정리했다.

대표 팀 프로젝트 **SWEEPICK**은 청소 경로의 작은 물체를 옮겨 공간을 확보하는 저비용 양팔 모바일 매니퓰레이터다. 물체 이동·청소·저전력 복귀·도킹·충전 흐름은 시뮬레이션에서 구현했고, 실물 기능을 통합하고 있다. 초기 신발 정리 과제는 개발 이력으로 보존한다.

**전형주 주 담당: 모방학습·시뮬레이션·Sim-to-Real·데이터 검증. Visual SLAM도 직접 설계·구현했다.** 프로젝트 전체 실험 결과는 담당 경험과 연결해 설명하되, 외부 ACT·RoboTwin 구현과 팀 공동 자료의 저작 범위는 구분한다.

[포트폴리오·이력서](https://julianjeonresume.netlify.app/) · [SWEEPICK 프로젝트](2ARM_ROBOT/README.md) · [CUSTOM ACT V1 실험 기록](2ARM_ROBOT/docs/CUSTOM_ACT_V1_KO.md) · [공개 그림·영상](https://julianjeonresume.netlify.app/research/custom-act-v1/) · [이전 조작 연구](2ARM_ROBOT/docs/RESEARCH_SUMMARY_KO.md)

## CUSTOM ACT V1: 데이터에서 실행까지

학습 손실만으로 로봇의 작업 완료를 판단하기 어려웠다. 시연을 어떤 기준으로 나누었는지, 정책이 어떤 관측을 받는지, 예측한 행동 청크가 실제 명령으로 어떻게 이어지는지를 함께 기록했다.

| 단계 | 수행 내용 | 확인한 결과 |
|---|---|---|
| 데이터 수집 | 원본 113개 궤적의 동작을 유지하며 손목 RGB의 물체색·배경·테이블 텍스처·조명을 5개 외형으로 구성. 관측·상태·명령·시각 정합 확인 | 565개 뷰. 궤적 train/val/test=59/29/25, 뷰=295/145/125. 같은 궤적의 외형은 동일 분할 유지 |
| 학습 | 양손목 RGB·head PCD·관절 상태를 입력으로 사용하는 ACT 학습 | 200 epoch 완료. best validation은 epoch103, prior normalized L1=0.01553 |
| 추론 | 별도로 고정한 epoch30 체크포인트, temporal ensemble ON(k=0.01)으로 SIM 폐루프 평가 | 143개 위치 각 1회 중 132회 완료(92.3%). Test 위치는 27/31(87.1%) |
| 분석 | 예측 청크의 target 시점·가중치·관절 명령·실측값·phase·영상 연결 | 실패 11회 중 grasp/lift 7회, stow 4회. 청크 집계 결과를 저장 CSV에서 재구성해 확인 |

폐루프 완료 기준은 **30 mm 이상 Lift → Stow → 물리 2초 Hold**다. 외형은 랜덤화하고 물체 크기(40 mm)·yaw·물성·카메라 보정은 고정했다. 위 92.3%는 이 SIM 조작 평가의 관측값이다. 200 epoch 최종 모델의 폐루프 결과, 실물 성공률이나 전체 청소 시나리오 성공률과 합산하지 않는다.

## 실물 기록과 후속 연동

기존 실물 트랙은 신규 SIM 평가와 별도로 유지한다. 2026-10-18~26 Native 교사 콘솔은 집기·이동·놓기·복귀 120/198회를 성공으로 판정했다. 실물 데이터 기반 ACT TJJ v2는 학습46/검증7 run의 관절 예측오차 3.30°(epoch690)를 기록했고, 정책 정체 후 교사 인계로 교정 시연16개를 확보했다. 예측오차는 작업 완료율과 다른 지표다.

10/27 후속 연구는 ACT30과 실물 실행기의 관측·12축 절대행동 계약, 앙상블·STOP·feedback·파지 증거의 연결을 확인하는 단계다. 새 30초 정지 shadow는 모터 명령 없이 읽기 경로를 확인한 기록이며, 전체 180초 검증과 새 실물 파지 완료를 뜻하지 않는다. [최신 연구 노트](https://www.notion.so/4065788b5ed1810aaec9c710b0d9e288)(접근 권한 필요)에 조건과 후속 기록을 연결했다.

## 코드와 연구 이력

| 확인할 내용 | 코드와 문서 |
|---|---|
| 수집·학습·추론·분석의 조건과 근거 | [CUSTOM ACT V1](2ARM_ROBOT/docs/CUSTOM_ACT_V1_KO.md) |
| 실물 실행기·콘솔·저장 기록 검수 | [sweepick 검토 브랜치](https://github.com/Alpenj/DAPIER/tree/pro/sweepick-review-20261015/2ARM_ROBOT/sweepick) |
| 양팔 모델·접촉·경로 검사 | [MuJoCo 모델](2ARM_ROBOT/sim/mobile_dual_so101/README.md) · [이전 연구](2ARM_ROBOT/docs/RESEARCH_SUMMARY_KO.md) |
| Visual SLAM·Nav2·도킹 모듈 | [이동 모듈 협업 저장소](https://github.com/shouttt1320/Dapier_project_visaul_slam) |

상체 실행기는 10/16부터 이동 베이스를 직접 구동하지 않고 정지 요청과 조작 결과를 전달하는 역할로 나눴다. 이 실행 구조와 본인의 Visual SLAM 설계·구현 경험은 별개다. 공동 저장소와 교육 예제·외부 라이브러리의 기여 이력은 보존한다.

연구 실행기와 후속 결과가 모두 공개 `main`에 병합된 것은 아니다. 검토 브랜치 및 고정 후보의 결과는 해당 revision에서 읽는다. 이번 갱신은 문서와 공개 요약 근거를 정리한 것으로, 런타임·보정값·장치 설정을 변경하지 않았다.

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

문서 갱신: 2026-10-27. 지정된 CUSTOM ACT V1 내장 보고서·CSV·학습 로그와 기존 Notion·실물 기록을 대조했다.
