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
| `writes_hardware` | `calibrate_follower_safe.py` | torque를 끄고 calibration 절차를 시작하며 기존 calibration을 교체할 수 있음 |

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
export LEROBOT_ROOT="${LEROBOT_ROOT:-$HOME/so101/lerobot}"

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
