# 실물 하드웨어 안전 절차

이 문서는 실물 접근·제어 전에 적용할 공통 조건이다. 문서를 읽거나 환경변수를
설정한 것만으로 장치 접근이 승인되지는 않는다. 기존 코드의 보호 조건과
캘리브레이션을 보존하며, SIM·MOCK 결과를 실물 동작이나 task 성공으로 확대하지 않는다.

## 보정과 수동 작업

SO-101 새 작업과 재개 작업에서는 로컬
`2ARM_ROBOT/recording/config/CALIBRATION_CURRENT.md`를 먼저 확인한다.
공통 보정 파일은 `~/.config/dapier/lerobot-calibration`에 보관하며,
2026-09-15에 네 장치의 값과 readback을 확인했다. 개인 보정 원본은 공개하지 않는다.

- 현장 수동 텔레옵은 사용자가 종료한다. 임의의 60초 종료·±30도 시험 제한을
  다시 추가하지 않으며, 보정된 관절 범위는 목표 제한으로 처리한다.
- 소프트웨어 토크/전류 제한값을 모터에 덮어쓰지 않는다. 기존 모터 보호 설정을 보존한다.
- 이미테이션 러닝의 리더–팔로워 대응은 고정 기준이다. 시작 자세를 매번 새 기준으로
  삼는 상대 텔레옵으로 변경하지 않는다.
- 과거 episode·측정 보정과 시작 오프셋은 원본을 보존한다. 현재 MuJoCo 좌표나
  보정값으로 자동 치환하지 않는다.

## 접근 승인과 장치 확인

- 기본 실행 범위는 소스·문서 읽기, 정적 분석, lint, unit test, mock과 실물 장치에
  연결되지 않는 simulation이다.
- `/dev/tty*`, `/dev/serial/by-id/*` open, serial packet 전송, firmware upload,
  read-only hardware snapshot과 원격 실물 장비 진단도 장치 접근으로 취급한다.
  endpoint나 ROS graph가 SIM인지 HW인지 불명확하면 HW로 취급한다.
- 실제 명령은 사람이 현장에 있고 E-stop 또는 즉시 사용 가능한 전원 차단을 준비한
  상태에서 승인한다. 장치 profile, 정확한 명령, 범위, 안전 조건과 exact confirmation
  string을 정하고 실행 직전에 같은 대화에서 명시적으로 확인한다.
  `DAPIER_ALLOW_HARDWARE=1` 같은 환경변수 하나만으로는 승인하지 않는다.
- 하드웨어 실행은 interactive TTY에서 수행한다. 먼저 예상 role과 device profile을
  대조하고 motor ID, model, voltage, temperature, load, torque state 또는 연결
  endpoint가 다르면 명령을 보내지 않고 중단한다.
- torque enable/disable, EEPROM/register write, homing offset, operating mode,
  position limit, calibration 변경과 복원은 승인된 구체적 범위를 벗어나 수행하지 않는다.
- 발견한 포트·serial ID·보정 파일을 승인으로 간주하지 않는다. 개인 장치 identity,
  보정 원본, 비밀번호·token·private key는 Git이나 공개 로그에 넣지 않는다.
- `sudo`로 장치 권한을 변경하거나 `/dev/ttyUSB*`를 자동 선택하지 않는다.

## 움직임과 정지 조건

- joint/velocity/effort 범위, 저속 동작, 제한된 정책·궤적 시험의 실행 시간과 종료
  조건을 명시한다. 관절 제한, 단위, 좌표계, freshness와 기존 오류 중단 조건을 보존한다.
- 현장 수동 teleop·에피소드 녹화에는 독립 자동 정지 장치나 통신 단절 자동 정지
  검증을 일괄 선행조건으로 요구하지 않는다. 장치 확인, 명시적 승인, 현장 입회,
  전원 차단, 저속과 동작 범위 제한은 유지한다.
- 2026-09-11 결정에 따라 사람이 전체 동작을 감독하고 즉시 전원을 차단할 수 있는
  제한된 실물 정책·궤적 시험에도 독립 자동 정지 검증을 일괄 선행조건으로 요구하지
  않는다. 실행 직전 승인, 장치·보정 확인과 기존 제한은 유지한다.
- 위 예외는 무인·자율 정책 실행에 적용하지 않는다. 무인 실행에는 통신 상실 시
  자동 정지 경로와 그 검증이 필요하며 수동 녹화 승인을 무인 실행으로 확대하지 않는다.
- 수동 전원 차단이나 호스트 보조 코드를 독립 자동 정지 검증으로 기록하지 않는다.
  enable service도 E-stop이나 torque-off의 보장이 아니다.

## 모듈과 자동 검증의 경계

- Python 연구 계층과 localization 계층은 장치 실행 권한을 갖지 않는다. 실제 제어는
  C++ 제어 계층의 보정·한계·watchdog·safe-stop 검사를 거쳐 수행한다.
- `ros_arm_bridge` 실행, Arduino/USB 연결, firmware upload와 물리 joint 명령은
  동일한 승인을 따른다. Arduino serial open은 보드를 reset하고 servo 명령을
  전달할 수 있으므로 단순한 파일 읽기로 취급하지 않는다.
- SO-101 `read_only/`와 `writes_hardware/` 도구는 inspect 모드라도 bus에
  접속하면 승인된 실물 접근이다.
- `/cmd_vel`, trajectory, JointState, torque 요청과 actuator command는 실물에
  도달할 수 있는 한 위 조건을 따른다. simulation publisher와 자동 ROS 테스트는
  명시적인 SIM/MOCK namespace 또는 격리된 ROS domain에서만 실행한다.
- 테스트는 bus·serial·publisher를 mock으로 대체하고 host ROS graph나 연결된
  장치로 fallback하지 않는다. CI에는 self-hosted runner나 장치 자동 탐색을 추가하지 않는다.
- CI·cloud·background task와 비대화형 실행에서는 실물 명령을 거부한다.
