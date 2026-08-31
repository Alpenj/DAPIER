# DAPIER C++ task executor contract

나는 Python policy가 실물 joint 값이나 hardware-enable 권한을 직접 보내지 않도록,
C++ 쪽에 manipulation·carry-ready·navigation·place 상태 전환 계약을 먼저 추가한다.

현재는 **SIM-only proposal layer**다. 이 라이브러리는 ROS publisher, serial port,
motor command, torque command를 만들지 않는다. `TaskDecision`의
`command_dispatch_allowed`와 `hardware_execution`은 항상 `false`다.

## Python/C++ 경계

```text
Python (ACT / RGB-D / LLM planner)
  -> skill + fresh observation sequence + world-state facts
C++ TaskExecutor
  -> phase/freshness/base-settled/grasp gate 검사
  -> simulation proposal only
Future ROS executor
  -> TF / collision / watchdog / human gate 확인 후 별도 구현
```

## 현재 gate

- arm skill은 base가 정지했을 때만 제안한다.
- lift는 grasp가 검증된 뒤에만 carry-ready로 전환한다.
- navigate는 verified grasp와 settled base를 요구한다.
- place는 navigation 뒤 base가 다시 정지한 경우에만 제안한다.
- stale sequence, hardware authority claim, recovery budget 초과는 거부한다.
- stop은 항상 수용하지만 어떤 command도 dispatch하지 않고 executor를 latch한다.

실물 SO-101·TurtleBot3 실행은 별도 ROS 2 executor, 장치 profile 검증, watchdog,
E-stop, 현장 사람의 명시 승인까지 추가된 뒤에만 검토한다.
