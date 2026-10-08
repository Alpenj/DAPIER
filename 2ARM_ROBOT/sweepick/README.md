# sweepick 양팔 관측·조작 소스

현재 현장 통합 소스를 기능별로 옮긴 Python package다. `2ARM_ROBOT`의 ROS package, SIM·연구·외부 자산 경계를 유지한다. 최상위 폴더를 `sweepick/`으로 바꾸는 작업은 CI·ROS·모델 상대경로를 함께 다뤄야 하므로 이번 PR에서 보류한다.

목표 흐름은 관측 → 왼 집기와 들기 → carry → 오른 접근·삽입·닫기 → 하중 이전 확인 → 왼 해제·retreat → 오른 PLACE → 양팔 STOW → 영역 재관측이다. 하나의 로컬 세션이 양팔 포트를 소유하고 기존 유한 trajectory `run`·`grip`을 호출한다. `sweepick_command_application.Applier`는 기존 제안·STOP 계약이며, 별도 per-cycle writer를 동시에 켜는 경로를 추가하지 않았다.

## 기능과 실제 코드

| 폴더 | 대표 파일·입력 | 출력과 실행 권한 |
|---|---|---|
| [perception](src/sweepick/perception/README.md) | OS30A·손목 frame, 원래 stamp/sequence, 분리된 command/actual feedback | 관측·유효성·후보 좌표. capture는 장치 read가 가능하다. |
| [manipulation](src/sweepick/manipulation/README.md) | held pose/불확실성, scene/TF, 기존 RECEIVE teacher | carry·오른 접근/삽입·release/place/stow 호출. 실제 제어 callback과 wrapper를 구분한다. |
| [control](src/sweepick/control/README.md) | mapping·servo 상태·trajectory·장치별 profile | 공유 `run`·`grip`, command application, collision preview. 승인된 실행에서는 motor/torque write에 도달한다. |
| [recording](src/sweepick/recording/README.md) | 3-view·양팔 measured feedback·applied event | 한 session ID의 episode와 기록 품질. recorder가 성공·support를 만들어내지 않는다. |
| [learning](src/sweepick/learning/README.md) | 실제 기존 ACT 학습/추론 코드 위치 | 기존 package·외부 runtime을 재사용한다. 학습기·checkpoint를 복제하지 않았다. |
| [integration](src/sweepick/integration/README.md) | HANDOFF_SNAPSHOT·실행 범위·최종 관측 | 세션·소유권·상태·후단 호출. MISSING을 그대로 전달한다. |

전체 실제 파일 tree, 이관표, revision 대조와 시험은 [구조 변경 보고서](docs/sweepick_261009_repository_review.md), [old→new CSV](docs/sweepick_261009_repository_path_map.csv), [source manifest](docs/sweepick_261009_repository_source_manifest.json)에 있다. 호환 모듈은 `src/`에 있으며 새 package의 **동일 module object**를 가리킨다. 기존 함수 이름·schema·단위·관절 순서·profile 숫자는 유지한다.

## 실행 진입점과 환경

Python 3.12의 기존 프로젝트 환경으로 검증했다. 이 package의 import 의존성은 numpy와 기존 프로젝트에 고정된 MuJoCo 3.3.7이다. 경로별로 기존 torch·OpenCV·LeRobot·ROS/SDK와 현장 runtime이 추가로 필요하다. 시스템 Python이나 전역 환경을 바꾸지 않는다. 별도 환경에서 아래처럼 설치하고 도움말을 확인할 수 있다.

```bash
python -m pip install --no-deps ./2ARM_ROBOT/sweepick
python -m sweepick.integration.sweepick_manipulation_session --help
python -m sweepick.control.sweepick_trajectory_executor --help
python -m sweepick.integration.sweepick_handoff_snapshot_adapter --help
```

| 진입점 | 실제 용도 | 이번 확인 범위 |
|---|---|---|
| `sweepick.integration.sweepick_manipulation_session` | 기존 `observe/look/stage/correct/session`, `--record`, `--full-chain`, `--full-chain-scope {full,observe}` | parser/help, import와 소스 계약. 세션 실행하지 않음. |
| `sweepick.control.sweepick_trajectory_executor` | 기존 `plan/out/return`, `run/grip`. `plan`도 실제 feedback을 읽을 수 있다. | parser/help와 무장치 단위 시험. 실제 writer 실행하지 않음. |
| `sweepick.integration.sweepick_handoff_snapshot_adapter` | 제공된 JSON snapshot 검증 | parser/help와 import/resource 경로. 실제 live hold로 승격하지 않음. |
| `sweepick.integration.sweepick_observation_session` | replay 또는 live read-only/ACT shadow | 지원 runtime에서 import 확인만. 카메라·bus·정책 forward 실행하지 않음. |
| `sweepick.manipulation.sweepick_bimanual_handover.build` | 최신 로컬 `scope="full" / "observe"` API에 실제 owner callback 연결 | 함수·class AST 보존과 고정 RECEIVE import 확인. 전체 과제 실행하지 않음. |

`FullEpisodeHandoff`는 단계 호출을 맡는다. 실제 로컬 제어 결합은 `sweepick_bimanual_handover`의 `Owner`, `Release`, `Downstream`, `Area`가 제공하며 executor의 `run/grip`을 재사용한다. wrapper 존재만으로 실물 controller·support/profile가 준비됐다고 판정하지 않는다.

## 입력·자산과 검증 범위

공개 branch에는 이번 고정 native 소스, 호환 진입점, 기존 시험/fixture, 공개 가능한 5개 기존 profile/근거 JSON이 포함된다. 왼 Jaw와 motion/progress profile은 원래 측정·PROVISIONAL 범위와 SHA를 그대로 유지한다. 오른 REAL profile·support·right wrist reference·확인된 PLACE 정보는 생성하지 않았다. 현장 private 입력 위치와 `SWEEPICK_RECEIVE_DIR`의 명시적 외부 고정본 override 계약은 유지한다.

현장 `tjj-runtime/v1`의 teacher·IK·collision·ACT source는 GitHub main의 SIM과 revision/API가 다르다. 외부 runtime boundary와 manifest를 유지하며, main의 비슷한 파일로 교체하지 않는다. 기존 `tabletop_replay.json`과 `integration_desk_source.json`은 branch에 실제 포함되어 있고 현장 source의 같은 자산과 SHA가 일치한다. SO-101 MJCF/mesh·SDK binary·149 MB compiled scene·checkpoint·raw episode는 이번 업로드에 포함하지 않는다. 외부 모델의 기존 출처·license와 `DAPIER_SO101_MJCF` 획득 경계를 따른다.

run14는 센서 계획 기반 무보조 왼 집기·들기 관측의 기준선이다. 자동 `placed_back`은 미확인이며 전체 양팔 또는 ACT 실물 성공 자료가 아니다. 저장 SIM 성공 두 사례와 완료된 RELEASE owner/STOP 반례는 보존하며 다시 실행하지 않았다. 이번 구조 시험은 합성/무장치 연결 검증이고 `REAL_RECEIVE_READY`·실물 성공으로 기록하지 않는다.

work6는 학습·대용량 자료와 독립 RECEIVE 지원 작업을 보존한다. 로컬 통합 담당자가 센서·추론·실행의 활성 파일과 장치를 관리한다. 팀 이동·도킹 코드는 [외부 저장소](https://github.com/shouttt1320/Dapier_project_visaul_slam)에 있고 복사하지 않았다.

## 병합과 현장 반영

이 PR은 draft이며 main 병합·현장 적용을 수행하지 않는다. 활성 소스가 다시 바뀌면 이관표의 old SHA와 최신 함수/API를 먼저 대조한다. 서버의 옛 executor나 이전 patch를 최신 로컬 파일 위에 덮어쓰지 않는다. 기존 기록의 source SHA는 유지하고, 이동한 코드의 새 SHA는 이관표에 별도로 기록한다. 이전 `STATE.json`을 새 source SHA로 고쳐 재사용하지 않는다. 실행 중인 세션에서 경로나 소스를 교체하지 않는다.

운영 파일은 `sweepick_<기능>_<역할>.py`, 실험·기록은 사건일 기준 `sweepick_YYMMDD_<기능>_<용도>`를 쓴다. 표준 metadata 파일명과 기존 package identity는 유지한다. 좌우 공통 executor·servo·Jaw를 복제하지 않고, 없는 controller나 새 gate를 폴더 예시에 맞춰 만들지 않는다. 이 기준은 구조·명명 안내이며 실물 통합의 선행조건이 아니다.
