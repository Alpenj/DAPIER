# C++ safety controller and mock command sink

## 목적

Python 정책·태스크 계획과 Visual SLAM이 만든 제안을 실제 ROS 또는 모터 계층에
전달하기 전에, 하드웨어와 무관한 C++에서 최종 실행 조건을 검사한다. 이 단계는
실물 명령을 보내지 않으며 `RecordingMockCommandSink`에 승인·hold·safe-stop 결과만
기록한다.

## 입력과 출력

```text
ResearchControlIntent
+ receiver-local monotonic time
+ measured joint/base state
+ localization motion decision
+ task phase/interlock facts
        ↓
SafetyController
        ↓
SafeCommand
        ↓
RecordingMockCommandSink
```

## 현재 강제되는 조건

- versioned schema, monotonic sequence, receiver-local TTL
- 명시적 operator enable
- E-stop health 확인
- measured-state freshness
- localization `Proceed`
- collision-clear interlock
- 관절 hard position limit
- 관절 maximum requested velocity
- command horizon 동안 허용되는 최대 위치 변화량
- arm motion 중 base settled 상태
- base transport 중 carry envelope와 verified grasp
- base linear/angular speed limit
- command watchdog와 measured-state watchdog
- safe stop latch와 disabled 상태에서의 명시적 reset

## safe stop 의미

현재 core는 물리 장치를 열지 않는다. safe stop 결과는 현재 측정 자세를 포함한
명시적 stop/hold 요청이며, 실제 hardware sink는 다음을 추가로 구현해야 한다.

- controller별 hold 또는 zero-velocity 명령
- 통신 오류 시 torque-off 또는 장치 제조사가 보장하는 정지 동작
- E-stop 회로 상태 read-back
- 실제 명령 적용 여부와 measured-state read-back
- 장치 identity, 전압, 온도와 fault 상태 검사

따라서 mock test 통과를 실제 비상 정지 검증으로 기록하지 않는다.

## 장비 없이 검증

```bash
cd ~/DAPIER
scripts/verify-hardware-free
```

ROS 2 Jazzy 설치 환경에서는 다음 package build/test도 수행한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to dapier_safety_core
colcon test --packages-select \
  dapier_localization_core dapier_so101_core dapier_safety_core
colcon test-result --verbose
```

## 실물 연결 전에 남아 있는 값

- 양팔 실제 joint limit와 verified calibration
- SO-101 controller의 hold/torque-off semantics
- TurtleBot3 velocity controller의 watchdog/zero-command semantics
- E-stop 회로와 복구 절차
- 실제 measured-state latency와 packet-loss 한계
- 저속·무부하 commissioning 속도
