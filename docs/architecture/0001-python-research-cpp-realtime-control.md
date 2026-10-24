# ADR 0001 — Python 연구 계층과 C++ 실시간 제어 계층 분리

- 상태: Accepted
- 적용일: 2026-09-02
- 범위: DAPIER 이동형 양팔, SO-101, TurtleBot3 제어 경로

## 배경

DAPIER에는 Python 기반 시뮬레이션, 데이터 기록, 정책 실험과 ROS 2/C++ 기반
관절 계약·텔레옵이 함께 존재한다. 초기 실습 과정에서 일부 Python 파일이 실제
장치 또는 command topic에 가까운 책임까지 맡았기 때문에, 연구 코드가 그대로
하드웨어 실행 권한을 갖게 될 위험이 있다.

정책의 빠른 반복과 로봇의 안전한 주기 제어는 요구 특성이 다르다. 연구 코드는
재현성, 데이터 탐색과 모델 교체가 중요하다. 제어 코드는 bounded latency,
watchdog, limit, command arbitration과 fail-closed 상태가 중요하다.

## 결정

### 1. Python은 연구 책임만 소유한다

신규 Python 연구 코드는 `2ARM_ROBOT/research/`에 둔다.

소유 책임:

- perception, mapping과 object pose 추정
- ACT/VLA 등 policy inference와 task planning
- MuJoCo simulation과 synthetic data
- dataset 변환, 품질 분석과 offline evaluation
- versioned `ControlIntent` 생성

금지 책임:

- serial/motor SDK 또는 `/dev/tty*` 직접 접근
- torque, EEPROM/register, actuator 또는 wheel command 직접 쓰기
- 실제 하드웨어 실행 승인
- C++ watchdog, limit, interlock 또는 safe-stop 우회

Python은 실행 명령이 아니라 **intent**를 제안한다. 기존
`2ARM_ROBOT/src/shoe_sorting_data`의 하드웨어 인접 Python 도구는 과도기 코드로
분류한다. 신규 연구 코드를 그 경로에 더하지 않고, 하드웨어 쓰기 책임은 단계별로
C++로 이동한다.

### 2. C++는 실시간 제어 책임을 소유한다

C++ 제어 코드는 `so101_ros2/`에 둔다.

소유 책임:

- intent ingress와 receiver-local timestamp
- schema, sequence와 TTL 검증
- command arbitration과 enable 상태
- joint/base position, velocity와 acceleration limit
- stale state, tracking error와 communication watchdog
- safe-stop, torque-off 요청과 hardware I/O
- measured state와 fault telemetry 발행

여기서 “실시간”은 책임과 구현 언어의 경계를 뜻한다. PREEMPT_RT, thread priority,
memory locking, DDS 설정과 worst-case latency를 측정하기 전에는 hard real-time을
검증했다고 주장하지 않는다.

### 3. 경계는 단일 버전 계약으로 고정한다

언어 중립 기준은
`contracts/research_realtime_control_v1.json`이다. Python은 이 JSON을 읽고,
C++ 상수는 `scripts/generate-control-contract-header`로 생성한다. CI는 generated
header가 계약과 다르면 실패한다.

intent에는 hardware authorization, device path 또는 motor register를 넣지 않는다.
서로 다른 호스트의 monotonic clock은 비교할 수 없으므로
`source_monotonic_ns`는 trace에만 사용한다. C++ 수신기가 자신의 monotonic
clock으로 bounded `ttl_ns`를 시작한다.

### 4. 이동과 조작 intent를 섞지 않는다

한 intent는 `hold`, `arm_joint_position`, `base_twist` 중 하나다. 팔과 베이스
명령을 한 payload에 동시에 넣으면 C++ ingress가 거부한다. 상위 task planner가
이동 완료와 base settled 상태를 확인한 뒤 별도 조작 intent를 만든다.

### 5. TurtleBot3 접속 비밀값은 로컬에만 둔다

공개 템플릿에는 `TB3_USER=user`와 SSH-key 방식만 둔다. 현재 비밀번호는 저장소,
PR, CI 또는 자동화 인자에 남기지 않는다. 실제 값은 Git에서 제외한
`2ARM_ROBOT/config/local/turtlebot3.env`와 로컬 SSH agent/keychain에서 관리한다.

## 데이터 흐름

```text
Python research
  perception / ACT / planner / simulation
                    |
                    | versioned ControlIntent
                    v
C++ real-time ingress
  schema -> sequence -> local TTL -> interlock -> limits -> watchdog
                    |
                    | validated setpoint
                    v
ros2_control / hardware driver
                    |
                    v
measured state / fault telemetry -> Python recorder and evaluator
```

## 자동 검증

`scripts/verify-architecture-boundaries`는 다음을 장비 없이 검사한다.

- Python 연구 패키지의 hardware/ROS 저수준 import와 device path 금지
- C++ 실시간 core의 Python embedding 또는 subprocess 실행 금지
- JSON 계약과 generated C++ header 동기화
- TurtleBot3 공개 env 템플릿의 계정명, SSH-key 방식과 secret key 금지
- Python intent 단위 테스트와 독립 C++ contract smoke test

## 결과와 한계

장점:

- 정책 실험 실패가 motor write 경로로 직접 이어지지 않는다.
- C++에서 limit와 watchdog을 한곳에 유지할 수 있다.
- Python/C++ 사이의 단위와 TTL 의미가 버전으로 고정된다.
- Raspberry Pi의 실제 접속 정보와 연구 코드를 분리한다.

비용:

- Python과 C++ 사이 adapter가 필요하다.
- 계약 변경 시 양쪽 테스트와 generated header를 함께 갱신해야 한다.
- 기존 Python 하드웨어 도구는 즉시 삭제하지 않고 단계적으로 이관해야 한다.

이 ADR만으로 실제 장비 안전이나 hard real-time이 검증되는 것은 아니다. 실제
TurtleBot3/SO-101 검증은 사람 승인, E-stop, 낮은 속도와 별도 hardware evidence가
필요하다.
