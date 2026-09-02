# DAPIER architecture

DAPIER의 신규 로봇 코드는 다음 두 실행 책임으로 나눈다.

| 계층 | 기준 경로 | 소유 책임 |
|---|---|---|
| Python 연구 계층 | [`2ARM_ROBOT/research/`](2ARM_ROBOT/research/README.md) | perception, policy, planning, simulation, dataset, offline evaluation, versioned intent 생성 |
| C++ 실시간 제어 계층 | [`so101_ros2/`](so101_ros2/README.md) | receiver-local TTL, sequence, interlock, limit, watchdog, safe stop, hardware I/O |

두 계층의 단일 계약은
[`contracts/research_realtime_control_v1.json`](contracts/research_realtime_control_v1.json)이다.
Python은 실제 장비 실행을 승인하지 않으며, C++ 계층이 intent를 다시 검증한 뒤
로봇별 제한과 enable 상태를 적용한다.

현재 C++ 계층의 “실시간”은 제어 책임을 의미한다. PREEMPT_RT와 worst-case
latency를 측정하기 전에는 hard real-time을 검증했다고 주장하지 않는다.

상세 결정과 과도기 코드 처리 원칙은
[`ADR 0001`](docs/architecture/0001-python-research-cpp-realtime-control.md)을 본다.

## 장비 없이 검증

```bash
scripts/verify-architecture-boundaries
python3 -m unittest discover -s 2ARM_ROBOT/research/test -v

c++ -std=c++17 -Wall -Wextra -Wpedantic \
  -Iso101_ros2/dapier_so101_core/include \
  so101_ros2/dapier_so101_core/src/realtime_control.cpp \
  so101_ros2/dapier_so101_core/test/realtime_control_contract_smoke.cpp \
  -o /tmp/dapier-realtime-contract-smoke
/tmp/dapier-realtime-contract-smoke
```

TurtleBot3 Raspberry Pi 접속 설정은
[`2ARM_ROBOT/docs/TURTLEBOT3_REMOTE_ACCESS.md`](2ARM_ROBOT/docs/TURTLEBOT3_REMOTE_ACCESS.md)에
분리한다. 공개 저장소에는 계정명과 secret-free 템플릿만 두고 비밀번호는 넣지
않는다.
