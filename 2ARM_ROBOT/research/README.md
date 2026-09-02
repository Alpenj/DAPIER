# DAPIER Python research layer

이 디렉터리는 인식, 정책 추론, 태스크 계획, 시뮬레이션, 데이터셋 생성과
오프라인 평가를 위한 **순수 Python 연구 계층**이다.

이 계층은 다음을 하지 않는다.

- serial 또는 motor SDK로 장치를 연다.
- `/dev/tty*` 또는 `/dev/serial/*`에 접근한다.
- torque, EEPROM/register 또는 actuator command를 직접 쓴다.
- 실제 로봇 실행을 승인한다.
- C++ 제어기의 watchdog, joint limit 또는 safe stop을 우회한다.

Python은 [`ControlIntent`](src/dapier_research/control_intent.py)를 만들 수 있지만,
그 intent는 제안일 뿐이다. `so101_ros2`의 C++ 계층이 receiver-local timestamp,
sequence, TTL, limit, interlock과 실행 허가를 다시 검사한다.

공유 계약은 저장소 루트의
[`contracts/research_realtime_control_v1.json`](../../contracts/research_realtime_control_v1.json)이
단일 기준이다. 서로 다른 컴퓨터의 monotonic clock은 비교하지 않는다. Python이
남긴 `source_monotonic_ns`는 trace용이고, C++ 수신기가 자신의 monotonic clock으로
`ttl_ns`를 시작한다.

## 장비 없이 테스트

```bash
cd ~/DAPIER
python3 -m unittest discover -s 2ARM_ROBOT/research/test -v
scripts/verify-architecture-boundaries
```

기존 `2ARM_ROBOT/src/shoe_sorting_data`에는 실험 초기에 만든 ROS·하드웨어 인접
Python 도구가 섞여 있다. 이 디렉터리를 연구 계층으로 간주하지 않으며, 신규
정책·학습·평가 코드는 여기 `research/`에 추가한다. 기존 하드웨어 쓰기 경로는
C++ 계층으로 단계적으로 이동한다.
