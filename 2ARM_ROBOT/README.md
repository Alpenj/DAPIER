# sweepick (TJJ) | 이동형 양팔 청소 로봇의 조작·학습·Sim-to-Real

DAPIER 교육에서 진행하는 팀 프로젝트다. 바닥의 물체를 보고(OS30A RGB-D) 접근해 집고, 지정한 곳에 놓은 뒤 그 자리를 청소하고 충전대로 돌아가는 저비용 모바일 매니퓰레이터 **sweepick**을 만든다. 2026-10-16부터 이 팀의 범위는 **상체(SO-101 양팔·그리퍼·손목 카메라·작업공간 카메라)의 팔 선택·집기·운반·놓기**이고, 이동 베이스·Visual SLAM·도킹은 다른 팀이 맡는다. 상체는 베이스를 직접 움직이지 않고 정지 요청만 보낸다. 초기 박스·신발 과제는 개발 이력으로만 남겼다.

**전형주 주 담당: 모방학습 · 시뮬레이션 · Sim-to-Real.** Visual SLAM의 설계·구현·코드 작업에도 협업 참여했다. 이동 모듈의 주 담당은 팀원 `shouttt1320`이다.

[전체 포트폴리오](../README.md) · [sweepick 실행 소스(검토 브랜치)](https://github.com/Alpenj/DAPIER/tree/pro/sweepick-review-20261015/2ARM_ROBOT/sweepick) · [실험과 문제 해결 기록](docs/RESEARCH_SUMMARY_KO.md) · [이동·도킹 협업 코드](https://github.com/shouttt1320/Dapier_project_visaul_slam)

## 풀고 있는 문제

로봇이 물체에 닿았다는 사실만으로 들어 올릴 수 있는 것은 아니었다. 파지 이후의 압축량, 계획한 손끝 위치와 실행 후 위치의 차이, 시뮬레이션 접촉 설정에 따른 미끄러짐을 나누어 조사했다. 카메라가 본 위치를 관절 명령으로 옮기는 과정에서는 좌표계, 영점, 단위와 관측 시각도 함께 확인했다.

학습 쪽에서는 시연을 저장하는 것에서 끝내지 않고, 측정 상태와 전송 명령의 의미, 에피소드 경계, 전처리와 행동 묶음의 실행을 연결하는 데 집중한다. 모델의 학습 실행, 보류 데이터 예측과 폐루프 조작 성공은 별도로 평가한다.

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

## 10/08 이후 실물 진행

| 날짜 | 내용 | 상태 |
|---|---|---|
| 10/08 | 왼팔이 사람 도움 없이 블록을 집고·들고·내려놓은 첫 1회 (OS30A 깊이 → IK 접근 → 손목 영상 정렬) | 실물 1회 |
| 10/16 | 범위를 상체로 한정(베이스·SLAM·도킹은 다른 팀), 양팔 상체 상태기·팔 선택 | 코드 확인 |
| 10/18~23 | 자동 교사 약 170회 실행, 3-view(손목 RGB 2 + OS30A RGB-D)·양팔 측정값·실제 서보 명령 기록. 교사 판정 성공: 빨강 40 mm 큐브 약 50, 검정 30 mm 큐브 20, 검정 막대 22 | 실물 확인 |
| 10/23 | ACT가 블록 위까지 접근 → 정체 감지 → 교사 실행기가 집기~운반~놓기~복귀 마무리 | 실물 1회 |
| 10/23~ | 실물 시연 원본에서 학습 데이터를 다시 만들어 헤드 점군 + 양 손목 RGB 입력 ACT 재학습 | 진행 중 |

ACT 단독으로 집기를 끝낸 실물 결과는 아직 없다. 지금 관찰된 주된 실패는 ACT가 블록 위에서 내려가지 못하고 멈추는 것이다.

## 다음 검증

ACT가 블록 위에서 멈추지 않고 정렬·하강까지 이어 가는지를 실시간 shadow와 제한된 실물 실행으로 확인한다. 같은 데이터 분할·전처리에서 교사 실행기와 비교하고, ACT가 실제로 머무는 상태에서 교사가 교정한 기록(DAgger)을 학습에 더한다. 이동 베이스와의 결합에서는 도착·정지, 조작 결과와 후속 이동 조건을 확인한다.

청소·도킹까지 이어지는 전체 시나리오는 목표다. 개별 모듈의 코드, 한 번의 실물 성공이나 SIM 결과를 전체 과제 완료로 쓰지 않는다.

## 실행 안내와 과거 실습

현재 코드의 사용법은 해당 하위 모듈 README와 [안전 절차](../docs/HARDWARE_SAFETY_KO.md)를 먼저 확인한다. 개인 보정, 장치 식별자와 원시 영상·데이터는 공개하지 않는다. 문서의 예시 명령은 실물 실행 승인이 아니다.

초기 ROS 2 환경 점검, 합성 토픽 녹화, golden episode 생성과 장치 snapshot 절차는 [갱신 전 README의 고정본](https://github.com/Alpenj/DAPIER/blob/9b1138dbc462d293cbde62961d4d3534a71b2061/2ARM_ROBOT/README.md)에 보존했다. 그 문서의 신발·H201 구성과 실행 제한은 당시 소스의 안내다. 과거 실행 절차를 최신 장비 설정으로 대신하지 않는다.

외부 라이브러리와 자산은 각 폴더의 출처·라이선스를 따른다. 팀 이동 모듈의 전체 구현을 개인 단독 성과로 계산하지 않는다.

문서 갱신: 2026-10-24(프로젝트 소개를 sweepick 기준으로 바꾸고 10/08 이후 실물 진행 추가). 런타임·물리 파라미터·보정값·테스트·의존성은 변경하지 않았다.
