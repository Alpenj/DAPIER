# 전체 저장소 구조

[루트](../../README.md) · [검토 안내](README.md)

기준: `main` commit `c8e1eb0dc6345c6e38f1169e3e1e1b43a297e116`. 아래는 정리 전 루트의 22개 항목을 빠짐없이 분류한 지도입니다. 재귀적으로 존재하는 모든 파일을 코드 리뷰하거나 실행한 목록은 아닙니다. 기존 경로는 이동·삭제·개명하지 않습니다.

## 프로젝트·실습

| 기존 경로 | 역할 | 입구 |
|---|---|---|
| `2ARM_ROBOT/` | 이동형 양팔 신발 정리, 데이터 계약·품질 검사·시뮬레이션·실측 기록 | [README](../../2ARM_ROBOT/README.md) |
| `so101/` | SO-101 전체 인덱스, integration·실험·hardware tools | [README](../../so101/README.md) |
| `dapier_sim_first/` | 무장비 G0/G1 및 offline digital-twin evaluator | [README](../../dapier_sim_first/README.md) |
| `so101_ros2/` | ROS 2 Jazzy core와 mock safe teleop | [README](../../so101_ros2/README.md) |
| `casino_dealer/` | CardBench 계약·planner·manifest·one-card baseline | [README](../../casino_dealer/README.md) |
| `turtlebot3_ws/` | TurtleBot3 SLAM·Nav2 | [README](../../turtlebot3_ws/README.md) |
| `ros_dd_ws/` | 커스텀 차동구동 로봇 workspace | [README](../../ros_dd_ws/README.md) |
| `jdcobot100_sim/` | 4축 로봇 RViz·Gazebo·ros2_control 학습 | [README](../../jdcobot100_sim/README.md) |
| `ros_arm/` | Arduino Uno 4축 서보·serial·GUI | [README](../../ros_arm/README.md) |
| `onshape/` | CAD·mesh·MuJoCo reference 자산 | [jdcobot100](../../onshape/jdcobot100/) |

## 공통 문서·검증·환경

| 기존 경로 | 역할 |
|---|---|
| `docs/` | runbook·협업·설계 문서; 이번 `portfolio/` 안내의 상위 폴더 |
| `project-planning/` | 설계 결정·Gate·중단 조건 |
| `scripts/` | 작업·검증 보조 도구; 이름만 보고 모두 무장비로 간주하지 않음 |
| `test/` | 루트 보조 도구의 테스트; 프로젝트별 테스트 폴더와 별도 |
| `.github/` | workflow·PR template·Dependabot 설정 |
| `requirements-cowork.txt` | cowork 검사 의존성 |
| `requirements-mujoco.in` | MuJoCo 환경 입력 의존성 |
| `requirements-mujoco.txt` | MuJoCo 환경 의존성 목록 |
| `.gitignore` | 생성물 등의 추적 제외 규칙; 이미 추적된 파일·이력 삭제 기능은 아님 |
| `.gitattributes` | Git 파일 속성 |
| `AGENTS.md` | 개발 에이전트용 작업 규칙; 채용 검토의 시작 문서가 아님 |
| `README.md` | 프로젝트 개요와 검토 동선 |

## SO-101 세부 동선

1. [SO-101 허브](../../so101/README.md)에서 현재 선택한 baseline을 읽습니다.
2. [sim-to-real foundation](../../project-planning/2026-08-07-so101-sim-to-real-foundation.md)에서 Gate와 중단 조건을 확인합니다.
3. [sim-first](../../dapier_sim_first/README.md)에서 G0/G1과 평가 범위를 확인합니다.
4. [LeRobot overlay](../../so101/integrations/lerobot_v0_6_so101_mujoco/README.md)에서 구현·외부 코드 경계를 봅니다.
5. 실물은 [runbook](../SO101_CASINO_DEALER_RUNBOOK_KO.md)과 [hardware tools](../../so101/hardware_tools/README.md)를 장치별로 확인합니다.

## 폴더를 옮기지 않은 이유

이 저장소는 독립적인 ROS 2 workspace, Python import, 상대 mesh·dataset 경로와 실행 스크립트를 포함합니다. 보기 좋게 폴더를 일괄 이동하면 실행·빌드·수업 자료와의 연결을 바꿀 수 있습니다. 현재 단계에서는 탐색 문서를 정리하고 실행 경로를 유지합니다.

후속 물리적 재구성이 필요하면 이동 후보와 참조 목록을 먼저 만들고, 코드·launch·package 설정·문서 링크를 함께 수정하는 별도 PR에서 테스트합니다. raw dataset·출력물·모델은 크기만 보고 삭제하지 않고 출처·재현 필요성·권리와 이력 처리 방식을 별도로 판단합니다.
