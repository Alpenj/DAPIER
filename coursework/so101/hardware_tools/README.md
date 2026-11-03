# SO-101 하드웨어 보조 도구

이 디렉터리는 예전에 `$HOME/so101/scripts`에서 사용하던 작은 도구를 역할별로
옮긴 곳이다. 파일을 옮기는 과정에서 개인 홈 경로와 특정 USB serial ID 기본값을
제거했다. 실제 포트는 실행할 때 명시해야 한다.

## 경계

| 분류 | 파일 | 장비에 미치는 영향 |
|---|---|---|
| `read_only` | `preflight.sh` | OS, 장치 목록, GPU, LeRobot import와 calibration 파일 목록만 확인 |
| `read_only` | `validate_calibration.py` | JSON 파일만 읽어 schema와 모터 ID를 검사 |
| `read_only` | `read_motor_positions.py` | serial bus를 열고 `Present_Position`을 읽지만 목표값은 쓰지 않음 |
| `read_only` | `watch_follower_calibration.py` | torque가 꺼졌는지 확인한 뒤 위치만 읽음 |
| `writes_hardware` | `calibrate_follower_safe.py` | LeRobot API를 사용하되 torque-off와 최소 range span을 강제하고 기존 calibration을 교체할 수 있음 |
| `writes_hardware` | `calibrate_sts3215_direct.py` | LeRobot를 import하지 않고 `scservo_sdk`로 STS3215 EEPROM을 직접 검사·교정·복원 |

`read_only`라는 이름은 모터 목표를 보내지 않는다는 뜻이다. serial 장치를 열기는
하므로 다른 ROS 2/LeRobot 프로세스가 같은 포트를 쓰는 동안 실행하지 않는다.
`writes_hardware`는 별도 승인과 현장 안전 준비 없이 실행하지 않는다.

외부 `so101-ros-physical-ai` stack을 시작하던 launcher는 그 코드와의 결합을
드러내기 위해 [`integrations/feetech_ros2_driver/launcher`](../integrations/feetech_ros2_driver/launcher)에
따로 보관했다. 기본 backend는 `mock`으로 바꾸었고, `real`은 포트를 명시해야만
진입한다.

## 실행 경로

```bash
export DAPIER_ROOT="${DAPIER_ROOT:-$HOME/DAPIER}"
export LEROBOT_ROOT="${LEROBOT_ROOT:-$DAPIER_ROOT/.local-workspaces/so101/lerobot}"

cd "$LEROBOT_ROOT"
uv run python \
  "$DAPIER_ROOT/so101/hardware_tools/read_only/validate_calibration.py" \
  --help
```

실제 calibration 파일, 포트, Hugging Face 사용자명과 token은
`DEFAULTS.env.example`을 복사한 Git 비추적 파일이나 shell 환경에서만 지정한다.
오늘 통합 작업에서는 syntax와 `--help`, 기존 follower/leader calibration JSON
validator, read-only preflight를 확인했다. preflight는 장치 목록만 열람했고 모터
port 연결이나 제어는 실행하지 않았다.

## 캘리브레이션 경로 선택

두 경로는 **대안**이다. 같은 serial port에 동시에 실행하지 않고, 한 경로를
완전히 종료한 뒤 다른 경로의 `inspect`로 결과를 교차 검증한다.

### A. LeRobot 사용 — 기본 권장

Follower는 로컬 안전 래퍼를 사용한다. 이 래퍼는 LeRobot의 공식
`SO101Follower.calibrate()`를 호출하지만 다음 interlock을 추가한다.

- 6축 `Torque_Enable=0`을 확인한 뒤에만 진행
- shoulder pan/lift/wrist flex 1000 tick, elbow 800 tick, gripper 500 tick
  미만이면 저장 거부
- 진행 상태를 80열 이내의 `P/L/E/F/G` 한 줄에서 갱신하고 Enter를 누르면
  즉시 저장 또는 실패 종료
- `q`/Ctrl+C에서 즉시 안전 취소
- 성공·실패와 관계없이 종료 전에 torque를 다시 끔

```bash
cd "$LEROBOT_ROOT"
source .venv/bin/activate
cd "$DAPIER_ROOT/so101/hardware_tools/writes_hardware"
python calibrate_follower_safe.py --port "$FOLLOWER_PORT" --id so101_follower_main
```

Leader는 공식 CLI를 사용한다. 기존 calibration이 있으면 프롬프트에서 `c`를
입력해야 새 범위를 기록한다.

```bash
cd "$LEROBOT_ROOT"
./.venv/bin/lerobot-calibrate --teleop.type=so101_leader --teleop.port="$LEADER_PORT" --teleop.id=so101_leader_main
```

### B. LeRobot 미사용 — 감사·복구용

`calibrate_sts3215_direct.py`는 소스 수준에서 `import lerobot`을 금지하고
FEETECH SDK만 사용한다. 직접 다루는 STS3215 주소는 position limit
`9/11`, homing offset `31`, operating mode `33`, torque `40`, lock
`55`, present position `56`이다.

기본 모드는 읽기 전용 `inspect`다.

```bash
cd "$LEROBOT_ROOT"
./.venv/bin/python "$DAPIER_ROOT/so101/hardware_tools/writes_hardware/calibrate_sts3215_direct.py" --mode inspect --role follower --port "$FOLLOWER_PORT"
```

EEPROM을 쓰는 `calibrate`와 `restore`는 정확한 확인 문자열 없이는
거부한다. `calibrate`는 수정 전 레지스터를 먼저 별도 JSON으로 백업하고,
완료 후 다시 읽은 값이 요청값과 정확히 같아야 최종 JSON과 receipt를 저장한다.

```bash
./.venv/bin/python "$DAPIER_ROOT/so101/hardware_tools/writes_hardware/calibrate_sts3215_direct.py" --mode calibrate --role follower --port "$FOLLOWER_PORT" --output "$FOLLOWER_CALIBRATION_JSON" --confirm WRITE_STS3215_CALIBRATION
```

```bash
./.venv/bin/python "$DAPIER_ROOT/so101/hardware_tools/writes_hardware/calibrate_sts3215_direct.py" --mode restore --role follower --port "$FOLLOWER_PORT" --input "$FOLLOWER_CALIBRATION_BACKUP" --confirm WRITE_STS3215_CALIBRATION
```

직접 경로는 공식 CLI보다 책임 범위가 넓으므로 기본 선택이 아니다. LeRobot
설치 문제를 분리하거나, EEPROM/JSON 계약을 독립적으로 재현하거나, 저장 전
레지스터 백업을 복원할 때만 사용한다.

## 범위 기록 화면

중앙 자세 프롬프트에서 Enter는 한 번만 누른다. 이후 표가 나타나면
`wrist_roll`을 제외한 다섯 축을 하나씩 안전한 전체 범위로 움직인다.
한 줄 `SPAN` 값이 변하고 모든 축이 `OK`가 된 뒤 Enter를 한 번 눌러
저장한다. 미달 상태에서 Enter를 누르면 무한히 계속하지 않고 저장 없이
종료되며 부족한 축을 표시한다. `q` 또는 Ctrl+C도 저장 없이 종료한다.
기계적 끝단을 힘으로 밀어 span 기준을 맞추지 않는다.
