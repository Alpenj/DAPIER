# JDcobot200 단일 관절 micro-jog

## 이번 단계의 범위

오늘 수업에서 확인한 두 팔은 팔당 STS3215 모터 6개가 1 Mbps로 응답했다. 기존 코드는
읽기 전용 probe까지만 있었기 때문에, 실제 동작의 첫 단계로 한 팔·한 관절·한 번의 작은
이동만 허용하는 `shoe_arm_jog`를 추가했다.

이 도구는 EEPROM, ID, homing offset, position limit을 쓰지 않는다. 쓰기 가능한 범위는
STS3215 RAM 40~49번이며 다음 두 패킷만 만든다.

- 단일 transaction: torque enable, acceleration, goal position, goal time, goal speed,
  runtime torque limit
- 종료 transaction: torque disable

## fail-closed 조건

- `/dev/serial/by-id` 고유 경로와 사용자가 확인한 serial이 일치해야 한다.
- 안전상 JDcobot controller가 정확히 한 대만 연결돼 있어야 한다.
- ID 1~6이 모두 model 777, position mode, torque off, stationary, error 0이어야 한다.
- 시작 위치와 목표 위치가 저장된 limit에서 64 tick 이상 떨어져 있어야 한다.
- 기본 이동은 motor 1, `+8 tick`(약 0.7도), speed 30, acceleration 10,
  runtime torque limit 1000(100%), 최대 1초다.
- 전압 11.0~13.0 V, 온도 45°C 이하, 전류 650 mA 이하, load 35% 이하를 강제한다.
- 성공·실패·SIGINT·SIGTERM 경로에서 ID 1~6에 torque off를 두 번 보내고 다시 읽는다.

OS 강제 종료, 전원 계통 고장, USB adapter 고장은 소프트웨어 `finally`만으로 막을 수 없다.
물리 전원 차단 수단을 확보하고 팔을 잡지 않은 상태에서만 실행한다.

## 실행

실제 serial은 공개 저장소나 결과 파일에 기록하지 않고 실행 시에만 전달한다.

```bash
ros2 run shoe_sorting_data shoe_arm_jog preflight \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_SERIAL-if00 \
  --expected-serial SERIAL --motor-id 1 --delta-ticks 8

ros2 run shoe_sorting_data shoe_arm_jog jog \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_SERIAL-if00 \
  --expected-serial SERIAL --motor-id 1 --delta-ticks 8 \
  --confirm JOG_SINGLE_ARM_MICRO
```

출력에는 실제 serial 대신 SHA-256 identity만 남는다. 이 micro-jog가 성공해도 전체 관절
limit, payload 능력, 반복 동작, 물체 파지, 양팔 동기 제어가 검증된 것은 아니다.

## 양팔 공통 전체 가동범위 프로파일

직접 motor ID 1이 베이스 yaw인 것을 확인한 뒤 +40도 명령에서 실제 39.6도 이동,
peak current 13 mA, peak load 8.8%, 최대 온도 43°C와 종료 후 전체 torque off를 확인했다.
같은 speed 120, acceleration 40, runtime torque limit 1000(100%)을
`joint-full-range`와 MuJoCo hardware teleop의 기본값으로 쓴다.
양팔은 같은 모델이므로 어느 팔이든 `--port`로 컨트롤러를 선택하고, 그 팔의 motor ID
1~6에 동일한 프로파일을 적용한다. 두 컨트롤러가 동시에 USB에 연결돼 있어도 선택한
by-id 경로와 `--expected-serial`이 일치하면 된다.

`joint-full-range`에는 별도의 상대 이동량 상한이나 끝단 margin을 두지 않는다. 요청 각도를
tick으로 바꾼 목표가 모터에 저장된 최소~최대 position limit 전체 범위 안에 있으면
실행한다. 전압 11.0~13.0 V, 온도 45°C, 전류 650 mA, load 35%, 목표 초과 8 tick과
종료 시 전체 torque off 차단은 그대로 유지한다.

```bash
ros2 run shoe_sorting_data shoe_arm_jog joint-full-range \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_SERIAL-if00 \
  --expected-serial SERIAL --motor-id 1 --delta-degrees 40 \
  --confirm JOG_SINGLE_ARM_MICRO
```

다른 관절은 `--motor-id 2`부터 `6`까지 선택한다. 반대 방향이나 더 큰 이동은
`--delta-degrees -40`, `--delta-degrees 90`처럼 지정한다.
직전 실측 온도가 43°C였으므로 연속 실행하지 않고 충분히 식힌 다음 상태를 확인한다.
