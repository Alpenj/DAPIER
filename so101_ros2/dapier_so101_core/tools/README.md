# Bounded pregrasp / wrist executor

record_id: DAPIER-2026-09-23-native-pregrasp-feedback

현재 장치 없는 연결 검증까지 완료했다. 실물 구동·접촉·들림·HOLD 수용은 미검증이다.

호출은 `execute_bounded_pregrasp.py` → `bounded_pregrasp_plan()` → 이 디렉터리의
`bounded_pregrasp_main.cpp` → `execute_pregrasp()` → transport의 `send()/read()`다.
손목 영상 오차는 기존 `wrist_correction_intent()`의 제안으로 들어오며, FK/축/경로를
검사한 후보와 관절 목표가 일치해야 같은 진입점에 연결된다. Python은 모터 SDK를 호출하지 않는다.

실제 transport는 로컬 Feetech native `SerialPort/CommunicationProtocol`을 재사용한다.
ROS plugin의 고정 midpoint·설정 변경·deactivate torque-off는 호출하지 않는다.
속도 register 46, Goal_Position 42, Torque_Enable=1 register 40만 쓰는 경로다.
calibration/EEPROM/보호 register 쓰기와 정상 종료의 자동 torque-off는 없다.
외부 SDK가 C++20을 요구하므로 이 standalone build도 C++20을 사용한다.

장치 없는 확인(저장소 루트):

```bash
bash so101_ros2/dapier_so101_core/tools/build_bounded_pregrasp /tmp/bounded_pregrasp_native
python so101/hardware_tools/motion/check_native_pregrasp.py /tmp/bounded_pregrasp_native /tmp/native-pregrasp-evidence-new
```

위 Python은 프로젝트 가상환경에서 실행한다. 이미 있는 증거 디렉터리를 덮어쓰지 않는다.
실제 launcher/C++ loop를 통과하고 transport만 지연 plant로 바꾼다. 정지 plant는 timeout,
손목 제안은 `WRIST_ALIGN_REACHED_HOLDING`으로 끝난다. `task_success`는 항상 false다.
`bounded_pregrasp_smoke.cpp`는 stale/cancel/부분 arming/양자화 경계도 검사한다.

명목 quintic 이동 시간과 실제 정착 시간은 다르다. 저장된 후보를 지연 MOCK으로
실행했을 때 공통 진행률 제한으로 경로 이탈은 해소됐지만, 기존 `명목 시간 + 1초`
예산에서는 종점에 도달하지 못했다. 후보 launcher의 `--maximum-duration-s`로
최대 60초 안에서 실행 예산을 명시할 수 있다. 기본값은 그대로이며 이미 작성된
plan의 시간은 이 옵션으로 바꿀 수 없다. 명시한 시간은 승인할 plan에 저장되고,
타임아웃 시 자동 연장·재시도하지 않는다. MOCK에서 정한 시간은 실물 응답 근거가 아니다.
같은 저장 후보의 별도 40초 MOCK 실행은 기록 구간 약 24.07초에 관절 종점에 도달했다
(최대 관절 잔차 0.000424 rad). 원래 16.97초 실패는 그대로 보존했다.
이 결과는 `PREGRASP_REACHED_HOLDING`이며 Cartesian endpoint·실물 task 성공은 아니다.
옵션 전달·잘못된 예산·stale 입력 거부는 `test_real_sensor_ik_adapter.py`와
`test_pregrasp_refusal.py`에서 장치 없이 검사한다.

실물 실행에는 별도 현장 승인, 실제 source/binary/profile/plan 고정, 최신 target/readback,
물리적으로 확인한 매핑, 위치 0.5 mm·수직축 2° 및 tracking envelope 전체의 clearance
30 mm 증거가 필요하다. 후보의 단일 경로 clearance를 envelope 증거로 승격하지 않는다.
토크가 꺼진 PREGRASP와 토크를 유지하는 WRIST_ALIGN 진입을 구분하고 실제 register를 확인한다.

현재 남은 연결 조건: 실제 모드에 맞는 RGB/depth 정합과 metric target, 현재 양팔 상태가
모델 범위 안에 있는 경로, tracking envelope 증거, 실물 feedback의 Cartesian endpoint 검사,
실제 wrist frame/관절 시각 연결, 접촉·들림·3.000초 HOLD·지지 종료 검증.
외부 SDK 한 호출 내부의 재시도/OS drain은 즉시 취소를 보장하지 않는다. 호출 사이의 취소·시간
검사와 별도 프로세스 종료 제한은 독립 하드웨어 watchdog 또는 graceful stop의 증명이 아니다.
