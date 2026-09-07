# DAPIER | Physical AI 프로젝트·실험 기록

**로봇의 관측·행동·데이터·안전·평가를 연결하는 수업 및 개인·팀 프로젝트 작업 저장소입니다.**

[전형주 포트폴리오](https://julianjeonresume.netlify.app/) · [검토 시작 안내](docs/portfolio/README.md) · [전체 폴더 지도](docs/portfolio/REPOSITORY_MAP.md) · [결과·검증 범위](docs/portfolio/EVIDENCE.md)

> **접근과 기준 버전:** 2026-09-05 확인 기준 이 저장소는 비공개입니다. 외부 검토자는 별도 접근 권한 없이 코드를 볼 수 없습니다. 이 문서는 `main` 기준선과 진행 중인 개발 PR을 구분합니다. 문서 정리로 개발 PR을 병합하거나 실물 실행을 승인하지 않습니다.

## 처음 보는 분을 위한 읽는 순서

| 확인하려는 내용 | 시작 위치 |
|---|---|
| 무엇을 만들고 있는가 | 아래 프로젝트 표 → [검토 안내](docs/portfolio/README.md) |
| 어떤 코드를 봐야 하는가 | [전체 폴더 지도](docs/portfolio/REPOSITORY_MAP.md) → 각 프로젝트 README |
| 무엇을 실제로 확인했는가 | [결과·근거·미검증 범위](docs/portfolio/EVIDENCE.md) |
| 로봇 없이 확인할 수 있는가 | 아래 무장비 확인 명령과 각 프로젝트의 환경 안내 |
| 최근 개발과 기존 결과가 왜 다른가 | [진행 중인 PR #51](https://github.com/Alpenj/DAPIER/pull/51)과 `main`의 기준일을 따로 확인 |

## 주요 프로젝트

| 프로젝트 | 해결하려는 문제·구현 범위 | 코드·문서 |
|---|---|---|
| 이동형 양팔 신발 정리 | 이동·조작을 분리하고 관측, 12차원 양팔 데이터, 품질 검사와 안전 계층을 연결하는 팀 프로젝트 | [`2ARM_ROBOT/`](2ARM_ROBOT/README.md) |
| SO-101 sim-first | 관절·단위·좌표계 계약과 접촉 조건을 검사하고 pick-and-lift 정책을 평가 | [`so101/`](so101/README.md), [`dapier_sim_first/`](dapier_sim_first/README.md) |
| ROS 2 안전 제어 구조 | 관절 계약, calibration 변환, 제한 계산과 합성 JointState 기반 safe teleop | [`so101_ros2/`](so101_ros2/README.md) |
| 이동로봇 SLAM·Nav2 | TurtleBot3 및 커스텀 차동구동 로봇의 시뮬레이션·통신·지도·경로 실습 | [`turtlebot3_ws/`](turtlebot3_ws/README.md), [`ros_dd_ws/`](ros_dd_ws/README.md) |
| 카드 딜러 기초 | blackjack planner, episode manifest, one-card 기구학 baseline | [`casino_dealer/`](casino_dealer/README.md) |
| 4축 로봇암·CAD | RViz/Gazebo 모델과 Arduino Uno 서보 제어를 별도로 학습 | [`jdcobot100_sim/`](jdcobot100_sim/README.md), [`ros_arm/`](ros_arm/README.md), [`onshape/jdcobot100/`](onshape/jdcobot100/) |

카드 딜러는 기존 SO-101 실습의 목표이며, 이동형 양팔 신발 정리와 동일한 완료 과제가 아닙니다. JDcobot200/Astra 기반 초기 모델, SO-101/H201을 사용하는 후속 개발, 4축 Arduino 실습의 장치 구성을 서로 섞지 않습니다.

## 상태를 읽는 기준

| 기준 | 확인 범위 | 아직 뜻하지 않는 것 |
|---|---|---|
| `main` 기준선 `c8e1eb0` | 2026-09-01 PR #39 병합 상태. 루트의 기존 실물 요약은 2026-08-20 기록 기준 | 9월 개발 브랜치의 모든 작업이 기본 브랜치에 반영됐다는 의미가 아님 |
| SO-101 corrected-contact 평가 | 기존 v2 정책의 unseen seed `2100..2119` 결과 `11/20`. 80% release gate 미달 | 실물 성공률·release 통과가 아님 |
| PR #51, `3ff180a` | Draft 개발 보고: RoboTwin/SAPIEN에서 양팔 접촉 전달 SIM seed 3 성공, canonical episode 변환 등 | multi-seed 강건성·전체 ACT 학습·held-out 성능·실물 신발 꺼내기 완료가 아님 |

PR #51은 확인 시점에 `main`이 아니라 `pro/mujoco-shoe-followup-01`을 대상으로 한 stacked Draft PR입니다. 변경되는 PR의 상태와 head를 다시 확인합니다. 현재 문서 정리에서 실험을 재실행하지 않았으며 수치는 기존 기록과 PR 보고의 인용입니다.

### 이전 결과를 숨기지 않는 이유

기존 접촉 모델의 held-out `14/20` 결과는 finger pad가 물체 내부로 최대 약 8.3 mm 침투한 문제가 발견돼 현재 성능으로 사용하지 않습니다. 접촉 조건을 수정한 뒤의 `11/20`과 구분합니다. 새 정책이 더 나빴던 기록, calibration 실패, 지도 불일치도 삭제하지 않습니다. [상세 결과와 제외 이유](docs/portfolio/EVIDENCE.md)에 정리했습니다.

## 무장비 확인

저장소 전체를 하나의 Python 환경 또는 colcon workspace로 한 번에 빌드하지 않습니다. 각 프로젝트 README의 환경을 사용합니다. 아래는 기존 README에서 안내하던 무장비 진입점입니다.

```bash
git clone https://github.com/Alpenj/DAPIER.git
cd DAPIER

# 각각 별도 subshell에서 실행하므로 이후 상대 경로가 유지됩니다.
(cd casino_dealer && python3 -m unittest discover -s test -v)
(cd casino_dealer && python3 -m casino_dealer.cli --players 3)
python3 -m unittest discover -s dapier_sim_first/test -v
```

비공개 저장소 접근 권한이 필요합니다. 이번 문서 정리에서 위 테스트를 새로 실행하지 않았습니다. `dapier_sim_first`의 단위 테스트와 G1 전체 재현은 다르며, 후자에는 문서에 지정된 MuJoCo 모델·calibration·별도 환경이 필요합니다. ROS 2 실습의 기본 기준은 Ubuntu 24.04/ROS 2 Jazzy이며 다른 버전은 각 문서의 범위를 확인합니다.

## 실물 작업은 별도 절차

[SO-101 hardware tools](so101/hardware_tools/README.md), [카드 딜러 runbook](docs/SO101_CASINO_DEALER_RUNBOOK_KO.md), [2ARM 하드웨어 기록](2ARM_ROBOT/docs/evidence/HARDWARE_EVIDENCE.md)을 해당 브랜치·장치·날짜와 함께 봅니다.

기존 `main`의 8월 기록에서는 양쪽 motor ID 1~6 응답을 확인했지만 follower calibration이 `MIN=POS=MAX=2047`로 끝나 저장이 거부됐습니다. offset은 기록됐으므로 당시에는 teleoperation을 금지했습니다. 후속 장비에서 calibration을 수행한 기록이 있더라도 다른 장치와 오래된 프로필에 그대로 적용할 수 없습니다.

실물 실행 전에는 장치별 calibration, motor ID와 관절 단위·방향, current limit, E-stop, 카메라 내·외부 파라미터, 저속 단일 관절 확인과 정지 조건을 별도로 확인합니다. SIM·MOCK·CI 통과만으로 실물 motion command를 허용하지 않습니다.

## 개발·기록 원칙

- command와 measured state, SIM·MOCK·HW 결과를 분리합니다. seed·revision·contract hash·camera profile·action horizon·판정 기준을 함께 남깁니다.
- 원시 데이터·영상, 장치 serial·개인 calibration, `.venv`, `build/install/log`는 공개 결과 문서에 추가하지 않습니다. 기존 Git 이력 전체의 민감정보 감사가 끝났다는 의미는 아닙니다.
- 외부 코드·모델은 overlay·patch·provenance·license 경계를 유지합니다. 팀 프로젝트 전체 구현을 개인 단독 기여로 소개하지 않습니다.

AI는 반복 코드·테스트 초안, 명령 정리와 로그 비교에 보조적으로 사용했습니다. 기여는 실제 변경과 검증 기록으로 확인합니다. [GitHub 협업 안내](docs/CHATGPT_PRO_CODEX_COWORK_KO.md)의 branch/worktree·remote SHA 확인, 검토와 병합 절차를 유지하며 Codex-managed worktree와 custom `scripts/cowork start`를 중첩하지 않습니다.

이 문서는 탐색 경로를 정리한 것이며 소스·안전 설정·의존성·저장소 공개 범위를 변경하지 않았습니다. [정리 전 README](https://github.com/Alpenj/DAPIER/blob/c8e1eb0dc6345c6e38f1169e3e1e1b43a297e116/README.md)와 기존 실험 문서는 이력으로 보존합니다.
