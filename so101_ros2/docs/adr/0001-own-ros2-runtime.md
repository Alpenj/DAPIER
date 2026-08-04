# ADR 0001: SO-101 실행 계층을 자체 ROS 2 모듈로 분리한다

- Date: 2026-08-04
- Status: Accepted for software architecture
- Domain: Architecture
- Impact: High
- Hardware validation: Pending
- Notion tags: SO-101, ROS 2, ros2_control, Architecture

## 배경

LeRobot은 calibration, teleoperation, recording, training을 빠르게 시작하기에
유용하다. 그러나 한 명령으로 전체 흐름을 실행하면 관절 순서와 단위, 시리얼
포트 소유권, 안전 정지 조건, 데이터 계약이 어느 코드에서 결정되는지 직접
확인하기 어렵다.

이번 프로젝트의 목표는 이미테이션 러닝 결과만 얻는 것이 아니라 로봇 상태가
어떤 경계를 지나 명령과 episode로 바뀌는지 단계별로 학습하고 기록하는 것이다.
따라서 로봇 런타임을 LeRobot CLI에 묶지 않는 구조가 필요했다.

## 결정

SO-101의 실시간 실행 계층은 직접 만든 ROS 2 패키지로 구성한다.

- 공통 관절 계약과 calibration 수학은 dapier_so101_core가 소유한다.
- 실제 시리얼 포트는 미래의 dapier_so101_hardware 하나만 소유한다.
- 리더-팔로워 변환과 enable 조건은 dapier_so101_teleop이 소유한다.
- episode는 ROS 토픽을 rosbag2/MCAP으로 먼저 기록한다.
- LeRobot은 오프라인 dataset 변환, 정책 학습, 비교 기준으로 연결한다.
- 학습 정책은 ROS 토픽 계약을 통해 교체 가능하게 만든다.

## 선택 이유

### 계산과 입출력을 분리한다

관절 순서 정렬, raw 값 변환, 위치 clamp, 속도 제한은 ROS daemon이나 실제
모터 없이도 단위 테스트할 수 있다. 이를 순수 C++ 코어로 분리하면 hardware,
teleop, dataset validator가 같은 규칙을 공유할 수 있다.

### 시리얼 포트 소유자를 하나로 제한한다

LeRobot motor process와 ros2_control driver가 같은 포트를 동시에 열면 패킷이
섞이거나 응답을 서로 가져갈 수 있다. 하드웨어 프로세스를 하나로 고정하고
나머지 기능은 ROS 토픽을 사용하게 만든다.

### 기본 상태를 fail closed로 둔다

노드가 실행됐다는 사실만으로 로봇이 움직여서는 안 된다. leader/follower
상태가 모두 신선하고, 시작 자세가 충분히 가깝고, 운영자가 enable을 요청한
경우에만 명령을 발행한다.

### 데이터 수집과 학습 프레임워크를 분리한다

원본 episode를 ROS 시간과 토픽 그대로 보존하면 정책 종류나 데이터셋 버전이
변해도 다시 변환할 수 있다. LeRobot 포맷은 원본 기록이 아니라 파생 산출물로
취급한다.

## 고려한 선택지

### 선택지 A: LeRobot CLI만 사용

장점은 빠른 baseline과 커뮤니티 호환성이다. 단점은 런타임 내부 경계를 직접
학습하고 수정하기 어렵다는 점이다. 비교 baseline으로는 유지하지만 주 실행
경로로 사용하지 않는다.

### 선택지 B: LeRobot motor class를 ROS 노드 안에 직접 삽입

구현은 빠르지만 LeRobot lifecycle과 ros2_control lifecycle이 섞이고 포트
소유권이 불명확해질 수 있다. 채택하지 않았다.

### 선택지 C: ROS 2 런타임을 직접 만들고 LeRobot은 오프라인으로 연결

초기 구현량은 늘어나지만 모듈 경계, 테스트, 안전 조건, 데이터 원본을 직접
소유할 수 있다. 이 선택지를 채택했다.

### 선택지 D: 데이터셋과 학습 프레임워크까지 전부 새로 구현

학습 목적에는 도움이 될 수 있지만 지금은 로봇 시스템 경계 학습보다 범위가
커진다. ROS 런타임이 안정된 뒤 필요성을 다시 판단한다.

## 결과와 부담

긍정적 결과:

- LeRobot 없이도 teleop runtime을 실행할 수 있다.
- 각 모듈을 실제 장비 없이 테스트할 수 있다.
- 실제로 검증한 범위와 아직 계획인 범위를 문서에서 구분하기 쉽다.
- 추후 양팔, 시뮬레이터, 다른 정책 backend로 확장할 수 있다.

부담과 위험:

- Feetech 프로토콜 오류 처리와 하드웨어 lifecycle을 직접 책임져야 한다.
- calibration 수학과 URDF 관절 방향을 실제 장비에서 교차 검증해야 한다.
- 소프트웨어 제한만으로 물리적인 비상 정지를 대신할 수 없다.
- 기존 구현보다 초기 개발 속도가 느리다.

## 구현 상태

2026-08-04에 확인한 것:

- dapier_so101_core의 계약, 변환, 제한 구현
- GTest 7개 통과
- dapier_so101_teleop 빌드
- 상태 없음 → enable 거부
- 정렬된 합성 상태 → enable 성공과 trajectory 발행
- 명시적 disable과 정상 종료

아직 확인하지 않은 것:

- 실제 STS3215 패킷 통신
- 실제 calibration 값과 ROS radian의 일치
- torque enable/disable
- 실제 팔의 leader-follower 움직임
- 통신 장애 시 물리적 정지
- episode 기록과 정책 rollout

## 다음 검토 조건

장비 교체 후 읽기 전용 motor scan이 안정적으로 반복 성공하면 이 ADR을 다시
검토한다. 실제 하드웨어 결과가 현재 joint contract와 다르면 기록을 남기고
schema version을 올린다.
