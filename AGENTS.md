# DAPIER Physical AI 학습 기록 원칙

이 저장소는 DAPIER Physical AI 교육을 수강하며 직접 배우고 실습한 과정을
기록하는 개인 학습 저장소다. 교육은 2026년 11월까지 진행 중이며, 완성된
프로젝트나 학습 교재로 설명하지 않는다.

Notion은 강의 녹음, 전사, 실패, 미완성 생각을 보관하는 비공개 원본이다.
DAPIER에는 직접 실행하고 확인한 코드, Markdown, 실습 결과만 공개한다.
녹음 원본, Notion URL, 페이지 ID, 개인정보, 비밀값, 실행하지 않은 성공 결과는
공개하거나 커밋하지 않는다.

글은 실제 사람이 학습하는 1인칭 현재진행형으로 작성한다. "오늘 수업에서",
"직접 실행해 보니", "아직 확인하지 못한 부분", "다음에 확인할 것"을 중심으로
쓴다. 홍보성 표현이나 AI가 모든 것을 해결한 것처럼 보이는 문장은 사용하지
않는다. 실행 조건, 명령어, 오류, 결과, 미해결 문제를 구체적으로 기록한다.

작업 전 기존 폴더 구조와 변경사항을 확인하고, 실제 검증 후 작은 단위로
커밋한다. `record_id`는 `DAPIER-YYYY-MM-DD-short-topic` 형식을 사용한다.

## 양팔 프로젝트 공통 개발 기준 — Codex·Hermes

SO-101 양팔의 계획·구현·리뷰·실행 안내 전에
[`2ARM_ROBOT/docs/DEVELOPMENT_CONTRACT.md`](2ARM_ROBOT/docs/DEVELOPMENT_CONTRACT.md)를
읽는다. IL+IK+rule의 단계별 분담, Python/C++/C 책임, ROS2-free 실행 경계,
양팔·카메라·sim-to-real 계약을 변경안에 적용한다. LeRobot/RoboTwin 실행 래퍼를
완성된 개인화 런타임으로 안내하지 않는다. 로컬 원문 위치는
`~/.config/dapier/dual_so101_project_sources.local.md`에 있으며 공개하지 않는다.
원문·과거 문서와 현재 사용자 결정이 충돌하면 차이를 기록하고 현재 결정을 따른다.

## GitHub 및 Codex 에이전트 규칙

- Codex cloud와 `scripts/cowork`를 사용하는 Mode A에서는 요청받은
  `pro/<task>` 브랜치에서만 작업하고 같은 브랜치에 커밋하고 normal push한다.
- ChatGPT 데스크톱 Codex-managed Worktree를 사용하는 Mode B에서는 detached
  HEAD가 정상 시작 상태다. 앱의 **Create branch here** 또는 **Hand off**가 branch
  전환을 담당하며, 이 모드에 `pro/<task>` checkout을 강제하거나
  `scripts/cowork start`를 함께 사용하지 않는다.
- 다른 작업자의 변경을 reset, checkout, revert, force-push로 없애지 않는다.
- 한 브랜치에는 writer 한 명만 둔다. 병렬 시도는 writer별 브랜치로 분리하고,
  non-fast-forward가 발생하면 merge나 rebase로 해결하지 말고 중단해 보고한다.
- SIM, MOCK, HW 근거를 명확히 구분한다. 시뮬레이션 통과를 실물 성공으로 쓰지
  않는다.
- `.env`, API key, token, serial ID, 개인 calibration 파일, 원시 dataset과 녹음은
  커밋하지 않는다.
- 관련된 가장 작은 테스트를 실행하고, 실행하지 못한 검증과 이유를 최종 요약에
  남긴다.
- 변경 목적, 검증 명령, 결과를 커밋 메시지 또는 PR 본문에 남긴다.

## 모든 에이전트의 실물 하드웨어 안전 경계

이 절의 조건은 세션의 시작 디렉터리와 무관하게 항상 적용한다. 하위 경로의
`AGENTS.md`는 Codex가 해당 경로에서 시작한 세션과 경로별 code review에 추가
맥락을 제공하지만, 실물 안전에 필수적인 조건을 하위 파일에만 두지 않는다.

- 로컬·원격·데스크톱·자동화 환경의 모든 에이전트는 사람의 명시적 승인 없이
  실물 장치를 열거나 움직이거나 설정을 바꾸지 않는다.
- 기본 허용 범위는 source/문서 읽기, 정적 분석, lint, unit test, mock과 실물
  장치에 연결되지 않는 simulation이다.
- 기본 금지 범위는 `/dev/tty*`, `/dev/serial/by-id/*` open, serial packet 전송,
  torque enable/disable, EEPROM/register write, motor jog, actuator command,
  실물 ROS graph로의 `/cmd_vel`·trajectory·JointState publish와 `sudo`를 이용한
  장치 권한 변경이다. endpoint가 SIM인지 HW인지 불명확하면 HW로 취급한다.
- 실물 명령은 사람이 현장에 있고 E-stop을 준비한 상태에서 장치 profile과 정확한
  명령, 안전 조건과 exact confirmation string을 지정하고, 실행 직전에 같은
  대화에서 승인했을 때만 interactive TTY에서 실행할 수 있다.
  `DAPIER_ALLOW_HARDWARE=1` 같은 환경변수 하나만으로는 승인하지 않는다.
- 승인 후에도 예상 role과 device profile을 먼저 대조한다. motor ID, model,
  voltage, temperature, load, torque state 또는 연결 endpoint가 예상과 다르면
  명령을 보내지 말고 중단해 보고한다.
- 움직임 명령은 명시적인 joint/velocity/effort 범위, 짧은 timeout 또는 watchdog,
  통신 상실 시 fail-safe stop을 갖춰야 한다. 무제한 명령이나 안전 정지 경로가
  검증되지 않은 명령은 승인 후에도 실행하지 않는다.
- 자동 ROS 테스트는 명시적인 SIM/MOCK namespace 또는 격리된 ROS domain에서만
  실행한다. graph나 domain이 불명확하면 HW로 취급한다.
- firmware upload와 read-only hardware snapshot도 장치에 접속하면 실물 접근이다.
  승인과 device profile 확인 없이 실행하지 않는다.
- CI, cloud, background task와 비대화형 실행에서는 실물 명령을 항상 거부한다.
- CI에 self-hosted runner를 추가하거나 attached hardware를 자동 탐색하게 하지
  않는다.
- 장치를 읽기만 하는 inventory도 serial port를 열면 실물 접근이다. 승인 전에는
  parser와 fixture만 검증한다.

## Code Review Rules

### Hardware writes

- Flag any serial write, torque change, EEPROM/register mutation, motor jog,
  `/cmd_vel`, trajectory, JointState, or actuator command path that can reach
  physical hardware without an explicit human gate and a safe default.
- Read-only parsing and simulation must stay separate from device-opening or
  write-capable tools. Tests must use mocks and must not auto-discover real devices.
