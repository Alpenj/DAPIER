# ros_dd 4륜 MuJoCo 실습

기존 `ros_dd` 차체 치수(0.5 × 0.3 × 0.15 m), 바퀴 반지름 0.075 m,
좌우 바퀴 간격 0.35 m를 사용해 네 바퀴 skid-steer 차량으로 구성했다.
ROS 2나 Gazebo 없이 MuJoCo 물리와 키 입력만으로 바로 실행할 수 있다.

## 실행

DAPIER 수업용 Python 3.12 환경이 설치된 PC:

```bash
cd ~/DAPIER/ros_dd_ws/mujoco
~/DAPIER/so101_imitation_learning/.venv/bin/python ros_dd_mujoco.py
```

새 PC 또는 압축파일을 받은 사람:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python ros_dd_mujoco.py
```

## 조작

| 입력 | 동작 |
|---|---|
| `W` / `S` | 전진 / 후진 |
| `A` / `D` | 좌회전 / 우회전 |
| 마우스 휠 | 최대 주행 속도 ±0.2 m/s |
| `Ctrl` + 마우스 휠 | 카메라 확대 / 축소 |
| 마우스 드래그 | 카메라 회전 / 이동 |
| `Space` | 즉시 정지 |
| `R` | 초기 위치로 리셋 |
| `Esc` | 종료 |

## 화면 없이 물리 검증

```bash
python ros_dd_mujoco.py --headless --command forward --seconds 2
python ros_dd_mujoco.py --headless --command left --seconds 2
```

`finite=True`이고 전진 시 x 위치, 회전 시 yaw가 변하면 모델·actuator·물리
step이 연결된 것이다. 이 모델은 수업용 단순화 모델이며 실제 모터 토크나
타이어 변형을 식별한 실차 digital twin은 아니다.
