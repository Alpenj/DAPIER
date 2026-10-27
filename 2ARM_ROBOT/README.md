# SWEEPICK | 이동형 양팔 청소 로봇의 조작·학습·Sim-to-Real

청소 경로의 작은 물체를 인식해 옮기고, 확보된 공간을 청소하는 저비용 모바일 매니퓰레이터다. DAPIER 팀 프로젝트에서 SO-101 양팔·카메라·이동 베이스와 학습 기반 제어를 연결했다. 물체 이동·청소·저전력 복귀·도킹·충전은 시뮬레이션 구현을 완료했고, 실물 기능을 통합하고 있다.

**전형주 주 담당: 모방학습·시뮬레이션·Sim-to-Real·데이터 검증. Visual SLAM도 직접 설계·구현했다.** 프로젝트 전체 실험 결과는 담당 경험과 연결해 설명하되, 외부 ACT·RoboTwin 구현과 팀 공동 자료의 저작 범위는 구분한다.

[포트폴리오·이력서](https://julianjeonresume.netlify.app/) · [CUSTOM ACT V1 상세](docs/CUSTOM_ACT_V1_KO.md) · [그림·영상](https://julianjeonresume.netlify.app/research/custom-act-v1/) · [이전 조작 연구](docs/RESEARCH_SUMMARY_KO.md) · [실물 검토 브랜치](https://github.com/Alpenj/DAPIER/tree/pro/sweepick-review-20261015/2ARM_ROBOT/sweepick)

## CUSTOM ACT V1: 데이터에서 실행까지

학습 손실만으로 로봇의 작업 완료를 판단하기 어려웠다. 시연을 어떤 기준으로 나누었는지, 정책이 어떤 관측을 받는지, 예측한 행동 청크가 실제 명령으로 어떻게 이어지는지를 함께 기록했다.

| 단계 | 수행 내용 | 확인한 결과 |
|---|---|---|
| 데이터 수집 | 원본 113개 궤적의 동작을 유지하며 손목 RGB의 물체색·배경·테이블 텍스처·조명을 5개 외형으로 구성. 관측·상태·명령·시각 정합 확인 | 565개 뷰. 궤적 train/val/test=59/29/25, 뷰=295/145/125. 같은 궤적의 외형은 동일 분할 유지 |
| 학습 | 양손목 RGB·head PCD·관절 상태를 입력으로 사용하는 ACT 학습 | 200 epoch 완료. best validation은 epoch103, prior normalized L1=0.01553 |
| 추론 | 별도로 고정한 epoch30 체크포인트, temporal ensemble ON(k=0.01)으로 SIM 폐루프 평가 | 143개 위치 각 1회 중 132회 완료(92.3%). Test 위치는 27/31(87.1%) |
| 분석 | 예측 청크의 target 시점·가중치·관절 명령·실측값·phase·영상 연결 | 실패 11회 중 grasp/lift 7회, stow 4회. 청크 집계 결과를 저장 CSV에서 재구성해 확인 |

폐루프 완료 기준은 **30 mm 이상 Lift → Stow → 물리 2초 Hold**다. 외형은 랜덤화하고 물체 크기(40 mm)·yaw·물성·카메라 보정은 고정했다. 위 92.3%는 이 SIM 조작 평가의 관측값이다. 200 epoch 최종 모델의 폐루프 결과, 실물 성공률이나 전체 청소 시나리오 성공률과 합산하지 않는다.

## 실물 데이터와 실행기 연동

| 트랙 | 확인된 결과 | 지표의 의미 |
|---|---|---|
| Native 교사 콘솔, 10/18~26 | 120/198회 집기·이동·놓기·복귀 성공 판정 | 규칙 기반 교사 실행 기록 |
| 실물 데이터 ACT TJJ v2 | train46/val7 run, 관절 예측오차3.30°·epoch690 | 보류 데이터 예측 평가 |
| DAgger, 10/23~25 | 정체 후 교사 인계 교정 시연16개 | 정책이 방문한 상태의 교정 데이터 |
| 10/27 후속 연동 | ACT30·실물 실행기의 관측/절대행동·앙상블·STOP·feedback·파지 증거 경로 정합 확인 | 저장 기록·코드·정지 shadow 단계 |

10/08의 첫 무인 왼팔 집기, 10/18~23의 약170회 자동 교사 실행, 10/23의 ACT 접근 뒤 교사 마무리 기록은 위 후속 실험에 이르는 개발 이력이다. 10/26까지의 TJJ 실물 기록에서 ACT 단독 집기 완료는 관찰되지 않았다. 신규 CUSTOM ACT V1 SIM 폐루프 완료와 이 실물 기록을 혼동하지 않는다.

10/27의 새 IR2 정지 shadow는 양팔 각300 prefix를 30초 동안 확인했고 위반0·모터쓰기0이었다. 기록상 full180s=false/all_pass=false이므로 전체 검증 통과로 표시하지 않는다. 최신 후속 노트3개는 [연구 노트 입구](https://www.notion.so/4065788b5ed1810aaec9c710b0d9e288)(접근 권한 필요)에서 연결한다.

10/16 이후 상체 실행기는 이동 베이스를 직접 움직이지 않고 정지 요청을 전달한다. 모듈을 나눈 실행 구조는 본인의 Visual SLAM 직접 설계·구현 경험을 축소하는 의미가 아니다. 이동·도킹 공동 코드의 출처와 변경 이력도 유지한다.

## 구성과 코드 위치

| 영역 | 다루는 내용 | 코드·문서 |
|---|---|---|
| 실물 집기 실행기 | 관측 → 손목 영상 정렬 → 하강 → 집기 → 들기 → 운반 → 놓기 → 복귀, 단일 장치 소유·정지 조건·웹 콘솔·저장 기록 검수 | [2ARM_ROBOT/sweepick (검토 브랜치, main 미병합)](https://github.com/Alpenj/DAPIER/tree/pro/sweepick-review-20261015/2ARM_ROBOT/sweepick) |
| 양팔 모델 | SO-101 양팔, 기본형·PGripper 구성, 이동형·책상형 장면 | [MuJoCo 모델](sim/mobile_dual_so101/README.md), [기준 배치](sim/mobile_dual_so101/INTEGRATION_SCENES.md) |
| 데이터 (이전 단계) | 관절 순서·단위, 영상과 시각, 에피소드 품질과 출처 | [shoe_sorting_data](src/shoe_sorting_data/) |
| 정책 (이전 단계) | 행동 묶음, CVAE 학습·체크포인트·추론을 다루는 작은 native ACT 구현 | [dapier_native_act.py](src/shoe_sorting_data/shoe_sorting_data/dapier_native_act.py) |
| 조작 검증 | 접촉·하중 지지·끝점 오차·미끄러짐을 분리한 비교 | [연구 기록과 고정 소스](docs/RESEARCH_SUMMARY_KO.md) |
| 이동·도킹 | RTAB-Map RGB-D 매핑·localization, Nav2와 듀얼 ArUco 도킹 | [팀 이동 모듈](https://github.com/shouttt1320/Dapier_project_visaul_slam) |
| 장치 실행 | 보정·명령 제한·피드백·정지 조건 | [공통 하드웨어 안전 절차](../docs/HARDWARE_SAFETY_KO.md) |

native ACT 코드와 LeRobot을 재사용한 다른 학습 실험은 같은 구현으로 취급하지 않는다. native 구현은 공식 ACT 체크포인트 호환을 주장하지 않는다.

### 장비 역할과 과거 설정

최근 조작 연구에서는 SO-101 양팔·PGripper, 작업공간용 OS30A RGB-D와 양손목 RGB를 다룬다. 이동 베이스 전면 중앙의 Astra RGB-D는 Visual SLAM용 별도 센서다. 두 RGB-D 센서의 역할을 섞지 않는다.

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

## 실행 안내와 과거 실습

> **`main` 코드의 실물 실행 상태 (2026-09-04 안전 고지 유지):** 이 저장소 `main`의 2ARM_ROBOT 실행기에서 physical motion과 camera streaming은 local safety integration(connected-endpoint identity binding, 독립 device-side watchdog, 카메라 serial/cryptographic binding 확인)이 끝날 때까지 비활성화돼 있다. sweepick 실물 실행은 검토 브랜치의 별도 실행기(단일 owner·정지 조건·현장 승인)로 하며, 그 코드는 아직 `main`에 병합하지 않았다.

현재 코드의 사용법은 해당 하위 모듈 README와 [안전 절차](../docs/HARDWARE_SAFETY_KO.md)를 먼저 확인한다. 개인 보정, 장치 식별자와 원시 영상·데이터는 공개하지 않는다. 문서의 예시 명령은 실물 실행 승인이 아니다.

초기 ROS 2 환경 점검, 합성 토픽 녹화, golden episode 생성과 장치 snapshot 절차는 [갱신 전 README의 고정본](https://github.com/Alpenj/DAPIER/blob/9b1138dbc462d293cbde62961d4d3534a71b2061/2ARM_ROBOT/README.md)에 보존했다. 그 문서의 신발·H201 구성과 실행 제한은 당시 소스의 안내다. 과거 실행 절차를 최신 장비 설정으로 대신하지 않는다.

외부 라이브러리와 자산은 각 폴더의 출처·라이선스를 따른다. Visual SLAM의 직접 설계·구현 경험과 공동 저장소의 외부·팀 기여 이력을 함께 보존한다.

문서 갱신: 2026-10-27. 새 CUSTOM ACT V1의 수집·학습·SIM 평가·분석을 추가하고 기존 실물·코드·후보 PR 기록은 보존했다. 런타임·물리 파라미터·보정값·테스트·의존성은 변경하지 않았다.
